// 在隔离的 Chromium 临时配置中加载实际构建的 Vue 页面。所有请求在本页由模拟接收端处理；
// CSP 同时禁止网络连接，因此测试不会访问开发代理、当前驾驶舱端口或任何机载接口。
import assert from 'node:assert/strict'
import { execFileSync } from 'node:child_process'
import { mkdtemp, readFile, rm, writeFile } from 'node:fs/promises'
import { tmpdir } from 'node:os'
import { join } from 'node:path'
import { fileURLToPath, pathToFileURL } from 'node:url'
import { build } from 'vite'

function installReceiver() {
  const receiver = { mode: 'manual_control', source: 'mock', online: true, network: true, armed: false,
    session: 0, moves: [], heartbeats: [], velocityCalls: 0, stops: 0, ttl: 3000, stateReplies: 0, telemetryReplies: 0, resets: [],
    repeatedActions: false, action: null, actionTtl: 900, actionStarts: [], actionReleases: [], holdRejections: 0,
    sessionRequests: [] }
  window.receiver = receiver
  const actionProtocol = () => receiver.repeatedActions || receiver.sessionReusable
  const advanceAction = () => {
    const action = receiver.action
    const now = performance.now()
    if (!action) return receiver.sessionReusable && receiver.standstillPending ? 'standstill_pending' : 'ready'
    if (receiver.sessionReusable && receiver.keyboardReleased && action.stopAt === null) action.stopAt = now + 200
    if (action.stopAt === null && now >= action.deadline) action.stopAt = now + 200
    if (action.stopAt === null) return 'active'
    if (now < action.stopAt) return 'stopping'
    if (!action.released) return 'release_required'
    receiver.action = null
    return 'ready'
  }
  window.fetch = async (path, options = {}) => {
    const reply = (body, status = 200) => new Response(JSON.stringify(body), { status })
    if (path === '/api/control/velocity') receiver.velocityCalls++
    if (!receiver.network) throw new TypeError('模拟浏览器断网')
    if (path === '/api/state') {
      if (receiver.stateDelayMs) await new Promise(resolve => setTimeout(resolve, receiver.stateDelayMs))
      // 故意固定控制能力为可用，单独检验前端对独立守护报告的门控。
      if (receiver.holdState) await new Promise(resolve => { receiver.releaseState = resolve })
      receiver.stateReplies++
      const actionState = actionProtocol() && receiver.armed ? advanceAction() : null
      return reply({
        deployment: receiver.mode === 'simulator' ? 'simulator' : 'station', mode: receiver.mode,
        pc2_connected: receiver.online, robot: { online: receiver.online, ...receiver.robotDetails }, map: null, pose: null,
        independent_telemetry: receiver.independentTelemetry,
        remote_stop: receiver.remoteStop,
        stop_reset: receiver.stopReset,
        control: { available: !receiver.oneClick || receiver.armed,
          activation_available: receiver.oneClick === true && !receiver.armed && !receiver.armStarting,
          armed: receiver.online && receiver.armed, source: receiver.online ? receiver.source : null,
          limits: { linear_mps: receiver.mode === 'manual_control' ? 0.20 : 0.05,
            angular_radps: receiver.mode === 'manual_control' ? Math.PI / 18 : 0.1,
            ...(receiver.mode === 'manual_control' ? { forward_mps: 0.20 } : {}) },
          ...(receiver.reusableHold ? { reusable_hold: true } : {}),
          ...(actionState ? { action_state: actionState, action_id: receiver.action?.id } : {}) },
      })
    }
    if (path === '/api/telemetry') {
      if (receiver.telemetryDelayMs) await new Promise(resolve => setTimeout(resolve, receiver.telemetryDelayMs))
      if (receiver.holdTelemetry) await new Promise(resolve => { receiver.releaseTelemetry = resolve })
      receiver.telemetryReplies++
      return reply({ deployment: 'station', pc2_connected: receiver.online,
        robot: { online: receiver.online, ...receiver.robotDetails }, remote_stop: receiver.remoteStop })
    }
    if (!receiver.online) return reply({ ok: false, reason: 'pc2_unavailable' }, 503)
    const command = JSON.parse(options.body ?? '{}')
    if (path === '/api/control/stop-reset') {
      receiver.resets.push({ method: options.method, command })
      const rejection = await new Promise(resolve => { receiver.replyReset = resolve })
      return rejection ? reply({ ok: false, reason: rejection }, 409) : reply({ ok: true })
    }
    if (path === '/api/control/session') {
      receiver.sessionRequests.push(command)
      if (receiver.oneClick) {
        receiver.armStarting = true
        await new Promise(resolve => setTimeout(resolve, 420))
        receiver.armStarting = false
      }
      receiver.armed = true
      receiver.hasMoved = false
      receiver.sequence = 0
      receiver.action = null
      receiver.sessionReusable = receiver.reusableHold && command.reusable_hold === true
      const policy = receiver.continuousHold ? { continuous_hold: true,
        ...(receiver.sessionReusable ? { reusable_hold: true } : {}), ...receiver.continuousReceiptFields }
        : receiver.repeatedActions ? { repeated_actions: true, action_limit_ms: 30000 } : { expires_in_ms: receiver.ttl }
      return reply({ ok: true, session_id: String(++receiver.session), ...policy })
    }
    if (!receiver.armed || command.session_id !== String(receiver.session)) return reply({ ok: false, reason: 'invalid_session' }, 409)
    if (path === '/api/control/action/start') {
      if (receiver.holdActionStart) {
        receiver.holdActionStart = false
        await new Promise(resolve => { receiver.resumeActionStart = resolve })
      }
      if (command.sequence <= receiver.sequence) return reply({ ok: false, reason: 'stale_sequence' }, 409)
      const actionState = advanceAction()
      if (actionState === 'standstill_pending') return reply({ ok: false, reason: 'standstill_required' }, 409)
      if (actionState !== 'ready') return reply({ ok: false, reason: 'action_not_ready' }, 409)
      if (receiver.keyboardReleased) {
        receiver.holdRejections++
        return reply({ ok: false, reason: 'operator_hold_released' }, 409)
      }
      receiver.sequence = command.sequence
      const duration = receiver.sessionReusable ? Infinity : receiver.actionTtl
      receiver.action = { id: command.sequence, deadline: performance.now() + duration, released: false, stopAt: null }
      receiver.actionStarts.push(command)
      return reply({ ok: true, action_id: command.sequence,
        ...(receiver.sessionReusable ? {} : { expires_in_ms: duration }) })
    }
    if (path === '/api/control/action/release') {
      if (command.sequence <= receiver.sequence) return reply({ ok: false, reason: 'stale_sequence' }, 409)
      receiver.sequence = command.sequence
      if (receiver.action?.id === command.action_id) {
        receiver.action.released = true
        receiver.action.stopAt ??= performance.now() + 200
      }
      receiver.actionReleases.push(command)
      return reply({ ok: true })
    }
    if (path === '/api/control/velocity') {
      if (command.sequence <= receiver.sequence) return reply({ ok: false, reason: 'stale_sequence' }, 409)
      const moving = command.linear_mps !== 0 || command.angular_radps !== 0
      if (moving && receiver.sessionReusable && advanceAction() !== 'active') {
        return reply({ ok: false, reason: 'action_expired' }, 409)
      }
      if (moving && receiver.keyboardReleased) {
        receiver.holdRejections++
        const delay = receiver.nextHoldRejectDelayMs
        receiver.nextHoldRejectDelayMs = 0
        if (delay) await new Promise(resolve => setTimeout(resolve, delay))
        receiver.holdRejectReplyAt = performance.now()
        return reply({ ok: false, reason: 'operator_hold_released' }, 409)
      }
      receiver.sequence = command.sequence
      if (!moving) {
        if ((!actionProtocol() && receiver.hasMoved) || receiver.mode !== 'manual_control') return reply({ ok: false }, 409)
        receiver.heartbeats.push({ ...command, at: performance.now() })
        // 模拟零帧已到 PC2，但它的失败响应直到第一条非零指令成功之后才到浏览器。
        if (receiver.delayNextHeartbeat) {
          receiver.delayNextHeartbeat = false
          await new Promise(resolve => { receiver.finishHeartbeat = resolve })
          return reply({ ok: false, reason: 'stale_sequence' }, 409)
        }
      } else {
        if (actionProtocol() && (advanceAction() !== 'active' || command.action_id !== receiver.action?.id)) {
          return reply({ ok: false, reason: 'action_expired' }, 409)
        }
        receiver.hasMoved = true
        receiver.lastMoveAt = performance.now()
        receiver.moves.push(command)
      }
    } else if (path === '/api/control/stop') { receiver.stops++; receiver.armed = false }
    else return reply({ ok: false }, 404)
    return reply({ ok: true })
  }
}

async function exerciseControls() {
  const r = window.receiver
  const sleep = ms => new Promise(resolve => setTimeout(resolve, ms))
  const until = async (check, reason) => {
    const deadline = performance.now() + 2400
    while (!check()) {
      if (performance.now() >= deadline) throw new Error(reason)
      await sleep(20)
    }
  }
  const expect = (ok, reason) => { if (!ok) throw new Error(reason) }
  const arm = () => document.querySelector('.arm-button')
  const ready = () => arm() && !arm().disabled
  const key = (type, name, repeat = false) => window.dispatchEvent(new KeyboardEvent(type, { key: name, repeat, cancelable: true }))
  const acquire = async (name = '遥控') => {
    await until(ready, `${name}未能重新启用`)
    arm().click()
    await until(() => document.querySelector('.control-state.ready'), `${name}会话未获准`)
  }
  const start = async name => {
    const n = r.moves.length
    await acquire()
    key('keydown', name)
    await until(() => r.moves.length > n, '按下方向键未发送速度')
  }
  const quiet = async () => {
    const n = r.velocityCalls
    await sleep(250)
    expect(r.velocityCalls === n, '结束后仍尝试发送保活或运动请求')
  }
  const result = document.getElementById('test-result')
  try {
    await until(() => document.querySelector('.inspection-actions a[href="#/maintenance"]'), '驾驶舱遥控入口缺失')
    document.querySelector('.inspection-actions a[href="#/maintenance"]').click()
    await until(ready, '模拟底盘不可用')
    expect(document.querySelector('.mode-chip').textContent.includes('PC2 模拟底盘'), '模拟底盘来源未标注')
    expect(!document.querySelector('.remote-stop-readings'), '把隔离模拟底盘展示为实机守护')
    await acquire()
    const remaining = () => document.querySelector('.control-remaining strong')
    expect(arm().disabled && arm().textContent.includes('控制已启用'), '已取得会话时仍显示初始启用按钮')
    const initialRemaining = Number(remaining()?.textContent)
    expect(initialRemaining > 0 && initialRemaining <= 3, '倒计时没有使用网关返回的会话期限')
    await until(() => r.heartbeats.length >= 3, '等待首次操作时没有持续发送保活')
    await until(() => Number(remaining()?.textContent) < initialRemaining, '剩余时间没有递减')
    expect(r.moves.length === 0, '等待首次操作时已有运动输出')
    const gaps = r.heartbeats.slice(1).map((item, i) => item.at - r.heartbeats[i].at)
    expect(gaps.every(ms => ms >= 80 && ms <= 200), '空闲保活间隔偏离 100 ms')
    r.delayNextHeartbeat = true
    await until(() => r.finishHeartbeat, '未捕获待处理的零帧')
    const lastIdle = r.heartbeats.at(-1)
    key('keydown', 'w')
    await until(() => r.moves.length === 1, '按键未接替在途保活')
    expect(r.moves[0].sequence === lastIdle.sequence + 1, '保活和运动序号不连续')
    r.finishHeartbeat()
    await until(() => r.moves.length >= 3, '迟到的空闲保活失败中止了运动')
    expect(r.heartbeats.at(-1) === lastIdle && !r.stops, '运动期间仍有空闲保活或错误停车')
    key('keyup', 'w')
    await until(() => !r.armed, '松键未结束首次动作会话')
    expect(!remaining(), '会话结束后仍显示剩余时间')
    await quiet()

    r.ttl = 500
    await acquire()
    await until(() => !r.armed && !remaining(), '限时会话到期后未撤掉倒计时与控制')
    r.ttl = 3000
    await acquire()
    window.dispatchEvent(new Event('blur'))
    await until(() => !r.armed, '首次按键前失焦未结束会话')
    await quiet()
    await acquire()
    document.querySelector('.stop-button').click()
    await until(() => !r.armed, '首次按键前手动停止未结束会话')
    await quiet()
    await acquire()
    location.hash = '/cockpit'
    await until(() => !r.armed, '首次按键前离页未结束会话')
    await quiet()
    location.hash = '/maintenance'
    await acquire()
    // 浏览器可能没有收到 offline 通知；保活失败或状态请求失败也必须撤销本地意图。
    const idleSession = r.session
    r.network = false
    await until(() => document.querySelector('.control-state.stop_unconfirmed'), '等待首次按键时断网未触发停车')
    await quiet()
    r.armed = false
    r.network = true
    await until(ready, '等待首次按键时断网后未能核实撤权')
    await quiet()
    expect(r.session === idleSession, '恢复连接后重新发起了空闲保活会话')
    expect(document.querySelector('.control-limits > div:last-child strong').textContent.trim() === '10.0 °/s',
      '转向限速没有换算成度/秒显示')
    const turn = Math.PI / 18
    for (const [name, linear, angular] of [
      ['w', .20, 0], ['ArrowUp', .20, 0], ['s', -.20, 0], ['ArrowDown', -.20, 0],
      ['a', 0, turn], ['ArrowLeft', 0, turn], ['d', 0, -turn], ['ArrowRight', 0, -turn],
    ]) {
      await start(name)
      const command = r.moves.at(-1)
      expect(command.linear_mps === linear && command.angular_radps === angular, '机身方向或限速错误')
      const previous = r.stops
      key('keyup', name)
      await until(() => r.stops === previous + 1, '松键未停车')
      await quiet()
    }
    // 合成指针不会获得系统级捕获，先验证捕获失败不会发令，再模拟浏览器把释放事件送回原按钮。
    await until(ready, '方向按钮不可用')
    arm().click()
    await until(() => document.querySelector('.control-state.ready'), '方向按钮会话未获准')
    const button = document.querySelector('.direction-key.forward')
    const pointer = type => button.dispatchEvent(new PointerEvent(type, { button: 0, pointerId: 91, bubbles: true }))
    const movesBeforePointer = r.moves.length
    pointer('pointerdown')
    expect(r.moves.length === movesBeforePointer, '指针捕获失败后仍然发令')
    button.setPointerCapture = id => expect(id === 91, '指针编号不匹配')
    pointer('pointerdown')
    await until(() => r.moves.length > movesBeforePointer, '方向按钮未发送速度')
    expect(r.moves.at(-1).linear_mps === .20, '前进按钮未使用 PC2 上报的前向限速')
    const beforeRelease = r.stops
    pointer('pointerup')
    await until(() => r.stops === beforeRelease + 1, '松开方向按钮未停车')
    await start('ArrowUp')
    const beforeBlur = r.stops
    window.dispatchEvent(new Event('blur'))
    await until(() => r.stops === beforeBlur + 1, '页面失焦未停车')
    key('keydown', 'ArrowUp', true)
    await quiet()
    await start('s')
    const beforeLeave = r.stops
    location.hash = '/cockpit'
    await until(() => r.stops === beforeLeave + 1, '离开遥控页未停车')
    location.hash = '/maintenance'
    // 浏览器离线时 stop 无法送达；站端先恢复且 PC2 仍掉线时，不能误报停车已确认。
    r.ttl = 500
    await start('d')
    r.network = false
    window.dispatchEvent(new Event('offline'))
    await until(() => document.querySelector('.control-state.stop_unconfirmed'), '断网后没有保留停止待确认')
    const session = r.session
    await quiet()
    r.online = false
    r.network = true
    // 模拟接收端超时撤权。真正的 PC2 端计时与底盘停车由网关集成测试验证。
    r.armed = false
    await sleep(1700)
    expect(arm().disabled && document.querySelector('.control-state.stop_unconfirmed'), '把断开的 PC2 误作已确认')
    r.online = true
    await until(ready, 'PC2 重新可达后未核实旧会话结束')
    await quiet()
    expect(r.session === session, '重连自动建立了控制会话')
    r.source = null
    await until(() => !ready(), '底盘来源未知仍能发令')
    r.mode = 'robot_readonly'
    r.source = 'unitree'
    const guard = { source: 'pc2_systemd', guard: 'running', latched: true, age_ms: 20, max_age_ms: 2500 }
    r.remoteStop = guard
    const text = selector => document.querySelector(selector)?.textContent.trim()
    await until(() => text('.control-note') === '停车已锁存', '锁存提示被只读原因掩盖')
    expect(!ready(), '锁存的只读实机仍可启用')
    location.hash = '/cockpit'
    await until(() => text('.inspection-heading .latch-reading') === '停车已锁存', '驾驶舱未显示锁存')
    expect(text('.inspection-heading .guard-reading b') === '运行中', '驾驶舱没有独立显示守护状态')
    const reset = () => document.querySelector('.ic-reset-button')
    expect(reset()?.disabled, '缺 PC2 复位能力时不应开放入口')
    r.stopReset = { available: true, reason: 'stop_reset_ready' }
    await until(() => reset() && !reset().disabled, '新鲜锁存且 PC2 允许复位时未显示入口')
    reset().click()
    const resetDialog = document.querySelector('.ic-reset-dialog')
    const confirm = resetDialog.querySelector('.ic-action')
    expect(resetDialog.open && confirm.disabled, '打开弹框时应先进行现场核对')
    resetDialog.querySelector('input').click()
    await until(() => !confirm.disabled, '完成现场核对后确认按钮未启用')
    r.remoteStop = { ...guard, age_ms: 2500 }
    await until(() => confirm.disabled && !resetDialog.querySelector('input').checked, '守护过期后未撤销现场确认')
    r.remoteStop = guard
    await until(() => !reset().disabled, '新鲜锁存数据到达后未恢复复位入口')
    expect(confirm.disabled, '守护数据恢复后使用了旧确认')
    resetDialog.querySelector('.ic-secondary').click()
    expect(!resetDialog.open && !r.resets.length, '取消现场确认时不应发出请求')
    reset().click()
    expect(confirm.disabled, '第二次打开弹框时现场确认未清空')
    const outputBeforeReset = [r.session, r.velocityCalls, r.stops].join(',')
    resetDialog.querySelector('input').click()
    await until(() => !confirm.disabled, '现场确认后仍不能提交')
    confirm.click()
    confirm.click()
    await until(() => r.replyReset && reset().disabled, '复位请求未进入等待状态')
    expect(r.resets.length === 1 && r.resets[0].method === 'POST'
      && JSON.stringify(r.resets[0].command) === '{"confirmed":true}', '确认仅应发送一次固定格式复位请求')
    r.replyReset()
    await until(() => text('.ic-reset-notice') === '复位已受理，等待 PC2 确认', '复位回执到达后缺少等待遥测状态')
    expect(text('.inspection-heading .latch-reading') === '停车已锁存' && reset().disabled, '复位回执覆盖了有效锁存')
    r.remoteStop = { ...guard, latched: false }
    await until(() => text('.ic-reset-notice') === '停车锁存已复位', '新鲜未锁存遥测到达后未确认复位')
    expect(reset().disabled && [r.session, r.velocityCalls, r.stops].join(',') === outputBeforeReset, '复位时修改了控制输出')
    r.remoteStop = guard
    await until(() => !reset().disabled, '夹具未恢复停车锁存')
    reset().click()
    resetDialog.querySelector('input').click()
    await until(() => !confirm.disabled, '再次现场确认后仍不能提交')
    confirm.click()
    await until(() => r.resets.length === 2, '第二次确认未发出复位请求')
    r.replyReset('standstill_required')
    await until(() => text('.ic-reset-notice').includes('尚未确认停稳'), 'PC2 拒绝原因未显示')
    expect(text('.inspection-heading .latch-reading') === '停车已锁存', '拒绝复位后锁存被覆盖')
    // 稳定响应的样本年龄和请求耗时都计入有效期，观察多轮刷新之间是否出现空读数。
    const sampleAge = { age_ms: 70, max_age_ms: 500 }
    r.robotDetails = {
      velocity: { x_mps: .03, y_mps: .04, ...sampleAge },
      batteries: { primary: { soc_percent: 68, ...sampleAge }, secondary: { soc_percent: 67, ...sampleAge } },
    }
    r.stateDelayMs = 140
    const readings = () => [text('.ic-kpis > .ic-kpi:first-of-type .ic-reading'),
      ...Array.from(document.querySelectorAll('.ic-kpis .ic-pack .ic-reading'), item => item.textContent.trim())]
    const visible = () => readings().join(',') === '0.050m/s,68%,67%'
    await until(visible, '模拟速度和双电池读数未显示')
    const stableUntil = r.stateReplies + 5
    let flickered = false
    const observer = new MutationObserver(() => { flickered ||= !visible() })
    observer.observe(document.querySelector('.ic-kpis'), { subtree: true, childList: true, characterData: true })
    try {
      await until(() => r.stateReplies >= stableUntil, '稳定遥测没有完成多轮刷新')
      expect(!flickered && visible(), '稳定响应期间速度或双电池读数变空')
    } finally {
      observer.disconnect()
    }
    r.holdState = true
    await until(() => r.releaseState, '未截获挂起的遥测请求')
    await until(() => readings().join(',') === '—m/s,—%,—%', '新请求挂起后仍显示过期遥测')
    expect(text('.inspection-heading .latch-reading') === '停车已锁存', '遥测过期影响了独立的锁存状态')
    r.holdState = false
    r.stateDelayMs = 0
    r.releaseState()
    delete r.releaseState
    await until(visible, '遥测恢复后未重新显示速度和双电池')

    // 独立直读启用后，让合并状态持续增加控制等待时间。新鲜读数不应再跟着控制请求变空。
    r.independentTelemetry = true
    await until(() => r.telemetryReplies > 0 && visible(), '驾驶舱未使用独立只读入口')
    r.stateDelayMs = 560
    const controlBaseline = r.stateReplies
    flickered = false
    observer.observe(document.querySelector('.ic-kpis'), { subtree: true, childList: true, characterData: true })
    try {
      await until(() => r.stateReplies >= controlBaseline + 2, '控制通道未完成两次慢响应')
      expect(!flickered && visible(), '慢控制响应让独立只读遥测闪烁')
      expect(text('.latch-reading') === '停车已锁存', '慢控制响应影响了独立守护读数')
    } finally {
      observer.disconnect()
    }
    r.stateDelayMs = 0
    r.holdTelemetry = true
    await until(() => r.releaseTelemetry, '未截获挂起的独立遥测请求')
    await until(() => text('.ic-summary-heading small') === '部分为上次读数', '短时延迟时未标记上次读数')
    expect(visible(), '短时延迟时直接清空了上次读数')
    await until(() => readings().join(',') === '—m/s,—%,—%', '独立遥测停止后仍显示过期读数')
    r.holdTelemetry = false
    r.releaseTelemetry()
    await until(visible, '独立遥测恢复后未重新显示')

    // 机身偏航角差分有自己的观测时效，原厂角速度为零时也独立显示。
    r.robotDetails = { ...r.robotDetails, angular_radps: null,
      velocity: { ...r.robotDetails.velocity, yaw_radps: 0 },
      attitude: { roll_rad: 0, pitch_rad: 0, yaw_rad: Math.PI / 6, yaw_rate_radps: Math.PI / 18,
        age_ms: 5, max_age_ms: 500 },
    }
    const summaryRate = () => text('.ic-kpis .ic-yaw-rate b')
    await until(() => summaryRate() === '+10.0 °/s', '驾驶舱未显示 IMU 左转转速')
    // 静止观测跨过旧版 0.05°/s 的显示舍入点，微小正负变化都应为约零。
    for (const degrees of [0.084, -0.062, 0.042]) {
      const next = r.telemetryReplies + 1
      r.robotDetails.attitude.yaw_rate_radps = degrees * Math.PI / 180
      await until(() => r.telemetryReplies >= next && summaryRate() === '≈0.0 °/s', 'IMU 静止显示仍在正负跳变')
    }
    r.robotDetails.attitude.yaw_rate_radps = Math.PI / 18
    await until(() => summaryRate() === '+10.0 °/s', '实际转向被静止显示规则掩盖')
    // 短暂跨过 250 毫秒时效时，转速和标签都应稳定，摘要不受转速时效影响。
    r.telemetryDelayMs = 145
    r.robotDetails.attitude.age_ms = 70
    const imuReplies = r.telemetryReplies + 4
    const summaryRateLabel = () => text('.ic-kpis .ic-yaw-rate')
    let rateFlickered = false
    const imuObserver = new MutationObserver(() => {
      rateFlickered ||= summaryRateLabel() !== 'IMU 转速 +10.0 °/s'
        || text('.ic-summary-heading small') !== '按接口显示'
    })
    imuObserver.observe(document.querySelector('.ic-kpis'), { subtree: true, childList: true, characterData: true })
    try {
      await until(() => r.telemetryReplies >= imuReplies, '延时遥测未完成多轮刷新')
      expect(!rateFlickered && summaryRateLabel() === 'IMU 转速 +10.0 °/s', '短时延迟让 IMU 转速或标签闪烁')
    } finally {
      imuObserver.disconnect()
    }
    r.telemetryDelayMs = 0
    r.robotDetails.attitude.age_ms = 5
    const missingFrame = r.telemetryReplies + 1
    delete r.robotDetails.attitude.yaw_rate_radps
    await until(() => r.telemetryReplies >= missingFrame, '未收到缺测的 IMU 帧')
    expect(summaryRateLabel() === 'IMU 转速 +10.0 °/s', '单帧缺测让 IMU 转速或标签变化')
    r.robotDetails.attitude.yaw_rate_radps = Math.PI / 18
    document.querySelector('.ic-kpis > .ic-kpi:first-of-type').click()
    const detailRate = () => text('.ic-dialog .ic-yaw-rate')
    const detailRateLabel = () => document.querySelector('.ic-dialog .ic-yaw-rate')?.previousElementSibling?.textContent
    await until(() => detailRate() === '+10.0 °/s', '本体详情未单独显示 IMU 转速')
    expect(detailRateLabel() === '转速估计（IMU 偏航角差分）', '本体详情转速标签异常')
    expect(document.querySelector('.ic-dialog[open]').textContent.includes('原厂偏航角速度'), '原厂速度来源未区分')
    r.holdTelemetry = true
    await until(() => r.releaseTelemetry, '未截获独立遥测请求以检查 IMU 转速过期')
    await until(() => summaryRate() === '— °/s', 'IMU 转速过期后仍沿用旧读数')
    expect(summaryRateLabel() === 'IMU 转速 — °/s' && detailRateLabel() === '转速估计（IMU 偏航角差分）',
    'IMU 转速过期后标签改变')
    expect(detailRate() === '— °/s' && text('.ic-dialog .ic-instrument:last-child .ic-axis-grid > div:last-child dd') === '30.00',
      'IMU 转速过期时未与仍有效的姿态角区分')
    r.robotDetails.attitude.yaw_rate_radps = -Math.PI / 18
    r.holdTelemetry = false
    r.releaseTelemetry()
    await until(() => summaryRate() === '-10.0 °/s', '驾驶舱未保留 IMU 右转符号')
    document.querySelector('.ic-dialog[open] button[aria-label="关闭详情"]').click()

    r.robotDetails.online = false
    await until(() => summaryRate() === '— °/s', '本体离线后仍显示 IMU 旧读数')
    expect(text('.inspection-heading .latch-reading') === '停车已锁存', '本体离线误改了在线 PC2 的锁存')
    delete r.robotDetails.online
    await until(() => summaryRate() === '-10.0 °/s', '本体恢复后未更新 IMU 转速')
    r.online = false
    await until(() => text('.inspection-heading .latch-reading') === '停车锁存未知', 'PC2 断线后仍显示旧锁存')
    expect(summaryRate() === '— °/s', 'PC2 断线后仍显示 IMU 上次转速')
    expect(text('.inspection-heading .guard-reading b') === '未知', '掉线后继续显示旧守护状态')
    expect(reset().disabled, 'PC2 断线后仍可复位锁存')
    r.online = true
    r.mode = 'manual_control'
    location.hash = '/maintenance'
    await until(() => text('.control-note') === '停车已锁存', '实机来源没有显示锁存')
    await until(() => text('.velocity-cards .imu-yaw-rate strong') === '-10.0', '维护页未显示 IMU 转速')
    expect(text('.velocity-cards .imu-yaw-rate span') === 'IMU 转速估计', '维护页转速标签异常')
    expect(Array.from(document.querySelectorAll('.status-details > div')).some(row =>
      row.querySelector('dt').textContent === '原厂偏航角速度' && row.querySelector('dd').textContent === '0.000 rad/s'),
    '维护页未同时保留原厂偏航角速度')
    delete r.robotDetails.attitude.yaw_rate_radps
    await until(() => text('.velocity-cards .imu-yaw-rate strong') === '—', '旧接口缺转速时未显示未知')
    expect(text('.velocity-cards .imu-yaw-rate span') === 'IMU 转速估计', '维护页转速缺测后标签改变')
    const sessionsBefore = r.session
    arm().click()
    key('keydown', 'w')
    await quiet()
    expect(!ready() && r.session === sessionsBefore, '矛盾的控制许可覆盖了守护锁存')
    r.remoteStop = undefined
    await until(() => text('.guard-reading b') === '未知', '旧版遥测缺字段时未标未知')
    expect(!ready(), '缺守护数据时可启用实机控制')
    r.remoteStop = { ...guard, source: 'mock', latched: false }
    await sleep(300)
    expect(!ready() && text('.latch-reading') === '停车锁存未知', '错误来源可证明解除锁存')
    r.remoteStop = { ...guard, guard: 'stopped', latched: null }
    await until(() => text('.guard-reading b') === '未运行', '未区分守护停止和状态未知')
    expect(!ready(), '守护停止仍可启用')
    r.remoteStop = { ...guard, latched: false, age_ms: 2100 }
    await until(ready, '新鲜运行/未锁存的实机夹具未恢复可用')
    r.holdState = true
    await until(() => r.releaseState, '未截获挂起的状态请求')
    await until(() => text('.guard-reading b') === '未知', '尚未收到下一帧时过期显示仍未撤下')
    expect(!ready() && text('.latch-reading') === '停车锁存未知', '过期的未锁存报告放行')
    r.holdState = false
    r.remoteStop = { ...guard, latched: false }
    r.releaseState()
    await acquire()
    r.remoteStop = guard
    await until(() => !r.armed && text('.control-note') === '停车已锁存', '持有会话时锁存未撤控')
    await quiet()

    // 仅隔离夹具把指定测试按下作为可信输入。模拟 PC2 的键盘证据晚于浏览器首包。
    r.remoteStop = { ...guard, latched: false }
    r.fixtureTrustedKeyboard = true
    r.keyboardReleased = true
    r.nextHoldRejectDelayMs = 160
    const beforeKeyboardMoves = r.moves.length
    const beforeKeyboardStops = r.stops
    await acquire('实体键盘首包')
    const keyDownAt = performance.now()
    key('keydown', 'ArrowUp')
    await until(() => r.holdRejections === 1, '未模拟首包先到的按键证据竞争')
    r.keyboardReleased = false
    await until(() => r.moves.length === beforeKeyboardMoves + 1, '延迟拒绝回执后，首包没有完成重试')
    expect(r.holdRejectReplyAt - keyDownAt >= 150 && r.lastMoveAt - r.holdRejectReplyAt < 75,
      '起步重试仍受 100 毫秒正常发包间隔限制')
    expect(r.moves.at(-1).linear_mps === .20 && r.stops === beforeKeyboardStops, '首包重试改变了方向或撤销了会话')
    await until(() => text('.event-list').includes('PC2 已确认首条速度请求'), '起步成功未记录首条回执')
    expect(text('.event-list').includes('可信键盘事件') && text('.event-list').includes('35 ms 后重试'), '起步日志缺少输入来源或重试记录')
    // 故意省略网页松键。第一条运动回执之后，独立松键拒绝要停车，绝不能再次续发。
    r.keyboardReleased = true
    await until(() => !r.armed, '运动后独立键盘松键没有撤权')
    r.keyboardReleased = false
    await quiet()
    expect(r.holdRejections === 2 && r.moves.length === beforeKeyboardMoves + 1, '已运动的旧按键被重新发送')
    key('keyup', 'ArrowUp')

    r.keyboardReleased = true
    await acquire('等待首包时松键')
    key('keydown', 'ArrowUp')
    await until(() => r.holdRejections === 3, '未出现等待键盘证据的首包')
    key('keyup', 'ArrowUp')
    r.keyboardReleased = false
    await until(() => !r.armed, '等待初次证据时松键未撤销')
    await quiet()
    expect(r.moves.length === beforeKeyboardMoves + 1, '网页松键后首包重试仍发出了非零速度')

    r.repeatedActions = true
    r.keyboardReleased = true
    const beforeRepeatedReply = r.stateReplies
    await until(() => r.stateReplies > beforeRepeatedReply, '重复动作模式未刷新')
    await acquire('实体键盘重复动作')
    await until(() => !document.querySelector('.direction-key.forward').disabled, '重复动作未进入就绪')
    key('keydown', 'ArrowUp')
    await until(() => r.holdRejections === 4, '未模拟重复动作的首条起步被拒绝')
    r.keyboardReleased = false
    await until(() => r.moves.length === beforeKeyboardMoves + 2, '重复动作未等到键盘证据后起步')
    window.dispatchEvent(new Event('blur'))
    await until(() => !r.armed, '重复动作完成验证后失焦没有停车')
    const holdRejectionsAfterRepeat = r.holdRejections
    r.actionStarts.length = 0
    r.repeatedActions = false
    r.fixtureTrustedKeyboard = false
    r.keyboardReleased = true
    await acquire('非可信键盘')
    key('keydown', 'ArrowUp')
    await until(() => !r.armed, '非可信键盘事件的首包拒绝没有停车')
    await quiet()
    expect(r.holdRejections === holdRejectionsAfterRepeat + 1, '非可信键盘事件获得了重试')
    expect(text('.event-list').includes('非可信键盘事件'), '起步日志未区分非可信按键')
    r.keyboardReleased = false

    r.mode = 'simulator'
    await until(() => text('.mode-chip') === '模拟器', '原模拟器模式未刷新')
    await acquire('原模拟器')
    await quiet()
    key('keydown', 'ArrowLeft')
    await until(() => r.hasMoved, '原模拟器没有收到运动请求')
    key('keyup', 'ArrowLeft')
    await until(() => !r.armed, '原模拟器松键后未结束会话')

    r.mode = 'manual_control'
    r.source = 'mock'
    r.repeatedActions = true
    await until(() => text('.mode-chip') === 'PC2 模拟底盘', '连续点动的模拟底盘模式未刷新')
    await acquire('PC2 模拟底盘连续点动')
    const repeatedSession = r.session
    const directionButton = direction => document.querySelector(`.direction-key.${direction}`)
    const actionReady = () => document.querySelector('.control-state.ready') && !directionButton('forward').disabled
    await until(actionReady, '新会话未获得 PC2 动作就绪状态')
    expect(!remaining(), '可连续点动的会话显示了总时长倒计时')

    key('keydown', 'w')
    await until(() => r.moves.at(-1)?.action_id === r.action?.id && r.action?.id, '首段动作未发出带动作标识的速度')
    expect(r.actionStarts.at(-1).direction === 'forward', '前进方向未发送')
    expect(Number(remaining()?.textContent) > 0, '动作没有独立倒计时')
    const firstId = r.action.id
    key('keyup', 'w')
    await until(() => r.actionReleases.at(-1)?.action_id === firstId, '松键未释放动作')
    expect(r.armed && r.session === repeatedSession, '正常松键导致整场会话撤销')
    key('keydown', 's')
    expect(r.actionStarts.length === 1, '停稳确认之前接受了下一段')
    await until(actionReady, 'PC2 停稳后未恢复方向键')

    r.actionTtl = 500
    key('keydown', 's')
    await until(() => r.actionStarts.length === 2 && r.moves.at(-1)?.action_id === r.action?.id, '同一会话不能按下后退')
    expect(r.actionStarts.at(-1).direction === 'backward' && r.moves.at(-1).linear_mps === -.20, '后退动作有误')
    await until(() => document.querySelector('.control-state.release_required'), '动作到期没有要求松键')
    const stoppedMoveCount = r.moves.length
    const holdCount = r.heartbeats.length
    key('keydown', 's', true)
    key('keydown', 'ArrowRight')
    await until(() => r.heartbeats.length > holdCount, '动作到期后准备会话没有保活')
    expect(r.moves.length === stoppedMoveCount && r.actionStarts.length === 2 && r.armed, '按键一直按住时到期后重新起步')
    key('keyup', 's')
    await until(actionReady, '到期动作松键后无法再次操作')

    // 发出的起步请求可能晚于释放抵达 PC2；释放使用预先保留的动作号。
    r.holdActionStart = true
    key('keydown', 'a')
    await until(() => r.resumeActionStart, '未截获待确认的起步请求')
    key('keyup', 'a')
    await until(() => r.actionReleases.length === 3, '起步回执前松键没有发送释放')
    r.resumeActionStart()
    await until(actionReady, '提前松键后未恢复就绪')
    expect(r.moves.length === stoppedMoveCount && r.actionStarts.length === 2, '迟到起步造成非零速度')

    key('keydown', 'ArrowRight')
    await until(() => r.actionStarts.length === 3 && r.moves.at(-1)?.action_id === r.action?.id, '新的按下没有开始第三段')
    window.dispatchEvent(new Event('blur'))
    await until(() => !r.armed, '可连续点动会话失焦后未撤销')
    await quiet()
    expect(r.session === repeatedSession, '重复动作额外申请了会话')

    // 持续按压必须由 PC2 明确授权，页面只接纳可信实体键和普通速度序号。
    r.repeatedActions = false
    r.continuousHold = true
    r.continuousReceiptFields = { expires_in_ms: 30000 }
    await until(ready, '无法验证冲突的持续按压回执')
    const beforeConflictingStop = r.stops
    const beforeConflictingOutput = r.velocityCalls
    arm().click()
    await until(() => r.stops === beforeConflictingStop + 1 && ready(), '未交回冲突的持续按压会话')
    expect(r.velocityCalls === beforeConflictingOutput, '冲突回执允许发送速度或心跳')
    r.continuousReceiptFields = undefined
    await acquire('持续按压')
    expect(!remaining() && text('.control-note').includes('按住实体方向键'), '持续按压没有进入实体键模式')
    expect(directionButton('forward').disabled, '持续按压仍开放屏幕方向按钮')
    const continuousMoves = r.moves.length
    const continuousHeartbeats = r.heartbeats.length
    pointer('pointerdown')
    key('keydown', 'ArrowUp')
    await until(() => r.heartbeats.length >= continuousHeartbeats + 2, '持续按压未保持就绪心跳')
    expect(r.moves.length === continuousMoves, '持续按压接受了指针或合成按键')
    r.fixtureTrustedKeyboard = true
    key('keydown', 'ArrowUp')
    await until(() => r.moves.length >= continuousMoves + 2, '持续按压未连续发送实体键对应的速度')
    expect(r.moves.at(-1).linear_mps === .20 && r.moves.at(-1).action_id === undefined
      && !remaining(), '持续按压意外套用了限时动作协议')
    key('keyup', 'ArrowUp')
    await until(() => !r.armed, '持续按压松键未结束会话')
    await quiet()

    // 新 PC2 允许在同一启用会话中反复按下、松开。每次新按下仍有独立的动作号。
    r.reusableHold = true
    r.oneClick = true
    r.source = 'unitree'
    r.remoteStop = { source: 'pc2_systemd', guard: 'running', latched: true, age_ms: 0, max_age_ms: 2500 }
    await until(() => arm().disabled && text('.control-note').includes('停车已锁存'), '可重复按键模式绕过停车锁存')
    r.remoteStop.latched = false
    await acquire('同会话连续遥控')
    const reusableSession = r.session
    const preReuseStop = r.stops
    const stateAfterAcquire = r.stateReplies
    await until(() => r.stateReplies > stateAfterAcquire, '同会话连续遥控未确认已启用的 PC2 动作状态')
    await sleep(20)
    expect(r.sessionRequests.at(-1).reusable_hold === true, '页面未显式协商同会话连续遥控')
    expect(text('.stop-button').includes('关闭控制') && text('.control-note').includes('松键停车'),
      '未说明同会话松键停车或未提供关闭控制')
    // 待命时停稳判据暂未满足。保留会话并明确等待，但不能补发这期间的按键。
    const waitingMoves = r.moves.length
    const waitingStarts = r.actionStarts.length
    const waitingHeartbeats = r.heartbeats.length
    r.standstillPending = true
    await until(() => text('.control-state').includes('已启用，等待停稳'), '待命失去停稳判据后未显示等待')
    key('keydown', 'ArrowUp')
    await until(() => r.heartbeats.length >= waitingHeartbeats + 2, '等待停稳期间未维持零速心跳')
    r.standstillPending = false
    await until(() => document.querySelector('.control-state.ready'), '新鲜停稳反馈未恢复待命')
    key('keydown', 'ArrowUp', true)
    key('keyup', 'ArrowUp')
    expect(r.actionStarts.length === waitingStarts && r.moves.length === waitingMoves,
      '等待停稳期间的方向键在恢复停稳后补发了动作')
    expect(r.armed && r.stops === preReuseStop && r.session === reusableSession,
      '临时等待停稳撤销了控制会话')

    // 页面看到停稳与 PC2 收到动作请求之间，机身可能重新进入停稳确认。
    r.holdActionStart = true
    key('keydown', 'ArrowUp')
    await until(() => r.resumeActionStart, '未发出需暂缓的动作请求')
    r.standstillPending = true
    r.resumeActionStart()
    r.resumeActionStart = undefined
    await until(() => text('.control-state').includes('已启用，等待停稳'), '起步明确被停稳判据拒绝后未进入等待')
    r.standstillPending = false
    await until(() => document.querySelector('.control-state.ready'), '起步拒绝后未用新鲜反馈恢复待命')
    key('keydown', 'ArrowUp', true)
    key('keyup', 'ArrowUp')
    expect(r.armed && r.stops === preReuseStop && r.session === reusableSession
      && r.actionStarts.length === waitingStarts && r.moves.length === waitingMoves,
      '停稳拒绝导致撤权或重新发送了上一个按键')
    for (const [name, direction, linear, angular] of [
      ['ArrowUp', 'forward', .20, 0], ['s', 'backward', -.20, 0],
      ['ArrowLeft', 'left', 0, turn], ['d', 'right', 0, -turn],
    ]) {
      const startsBefore = r.actionStarts.length
      key('keydown', name)
      await until(() => r.actionStarts.length > startsBefore && r.moves.at(-1)?.action_id === r.action?.id,
        `同会话没有收到新的方向动作 ${name}`)
      const { id } = r.action
      const move = r.moves.at(-1)
      expect(!remaining() && r.actionStarts.at(-1).direction === direction
        && move.linear_mps === linear && move.angular_radps === angular, '新动作的方向、速度或期限错误')
      key('keyup', name)
      await until(() => r.actionReleases.at(-1)?.action_id === id, '松键未发送本段释放')
      const stoppedMoves = r.moves.length
      key('keydown', 'ArrowDown')
      await until(() => document.querySelector('.control-state.ready'), '停稳后未恢复已启用状态')
      expect(r.armed && r.session === reusableSession && r.stops === preReuseStop, '松键撤掉了本场会话')
      expect(r.actionStarts.length === startsBefore + 1 && r.moves.length === stoppedMoves,
        '停稳时提前按下的方向键被排队发送')
    }
    // 独立键盘先观测到松开，但网页漏收：暂停零帧，避免给没有松键确认的会话续命。
    key('keydown', 'ArrowUp')
    await until(() => r.action && r.moves.at(-1)?.action_id === r.action.id, '无法开始漏收松键验证')
    const unchangedHeartbeats = r.heartbeats.length
    r.keyboardReleased = true
    await until(() => document.querySelector('.control-state.release_required'), '独立松键后页面未停止发送旧方向')
    key('keydown', 'ArrowUp', true)
    await sleep(200)
    expect(r.heartbeats.length === unchangedHeartbeats, '未收到浏览器松键时仍发送零帧保活')
    key('keyup', 'ArrowUp')
    await until(() => document.querySelector('.control-state.ready'), '浏览器收到松键后未恢复已启用')
    r.keyboardReleased = false
    document.querySelector('.stop-button').click()
    await until(() => !r.armed && ready(), '关闭控制未结束同会话连续遥控')
    await quiet()
    expect(r.session === reusableSession, '连续操作额外启用了控制会话')
    // 探测到单击启用能力只开放按钮。PC2 完成准备前不允许发送方向请求。
    r.reusableHold = false
    r.oneClick = true
    r.source = 'unitree'
    r.remoteStop = { source: 'pc2_systemd', guard: 'running', latched: true, age_ms: 0, max_age_ms: 2500 }
    await until(() => arm().disabled && text('.control-note').includes('停车已锁存'), '单击启用绕过停车锁存')
    r.remoteStop = { ...r.remoteStop, latched: false }
    const oneClickPrior = r.session
    const oneClickMoves = r.moves.length
    const oneClickHeartbeats = r.heartbeats.length
    await until(() => !arm().disabled, '未显示单击启用按钮')
    expect(r.session === oneClickPrior, '只读状态查询触发了启用')
    arm().click()
    await until(() => r.armStarting, '单击未请求 PC2 启用')
    key('keydown', 'ArrowDown')
    key('keyup', 'ArrowDown')
    await until(() => r.heartbeats.length > oneClickHeartbeats, '启用后未开始零速保活')
    expect(r.armed && r.moves.length === oneClickMoves, '未确认会话时出现运动')
    await until(() => arm().textContent.includes('控制已启用'), '未确认单击启用后的真实会话')
    key('keydown', 'ArrowDown')
    await until(() => r.moves.length > oneClickMoves && r.moves.at(-1).linear_mps < 0, '单击启用后无法后退')
    key('keyup', 'ArrowDown')
    await until(() => !r.armed && !arm().disabled, '松键后未恢复可重新启用状态')
    expect(r.session === oneClickPrior + 1, '松键后自动重新申请了会话')
    result.textContent = 'PASS'
  } catch (error) { result.textContent = String(error) }
}

const root = fileURLToPath(new URL('..', import.meta.url))
const temp = await mkdtemp(join(tmpdir(), 'a2-manual-ui-'))
try {
  const output = join(temp, 'dist')
  await build({ root, logLevel: 'error', build: { outDir: output }, plugins: [{
    name: 'isolated-keyboard-fixture',
    enforce: 'pre',
    transform(source, id) {
      if (!id.endsWith('/MaintenanceConsole.vue')) return null
      const original = "event.isTrusted ? 'trusted_key' : 'untrusted_key'"
      assert(source.includes(original), '生产入口须保留可信键盘事件检查')
      return source.replace(original, "(event.isTrusted || window.receiver.fixtureTrustedKeyboard === true) ? 'trusted_key' : 'untrusted_key'")
    },
  }] })
  const index = await readFile(join(output, 'index.html'), 'utf8')
  const assetPath = index.match(/src="(\/assets\/[^"\s]+\.js)"/)[1]
  const compiled = await readFile(join(output, assetPath), 'utf8')
  const html = `<!doctype html><meta charset="utf-8"><meta http-equiv="Content-Security-Policy" content="connect-src 'none'">
    <div id="app"></div><pre id="test-result">pending</pre><script>(${installReceiver})();</script>
    <script type="module">${compiled.replaceAll('</script', '<\\/script')}</script>
    <script>window.addEventListener('load', ${exerciseControls});</script>`
  const fixture = join(temp, 'manual.html')
  await writeFile(fixture, html)
  const dom = execFileSync(process.env.CHROME_BIN || 'google-chrome', ['--headless', '--disable-gpu',
    '--disable-background-networking', '--disable-component-update', '--disable-extensions', '--disable-sync',
    '--no-first-run', `--user-data-dir=${join(temp, 'profile')}`, '--virtual-time-budget=32000', '--dump-dom',
    pathToFileURL(fixture).href], { encoding: 'utf8', stdio: ['ignore', 'pipe', 'pipe'], timeout: 45000 })
  assert.equal(dom.match(/<pre id="test-result">([^<]*)<\/pre>/)?.[1], 'PASS')
  console.log('PASS: Vue 手动遥控、连续点动、持续按压、驾驶舱守护与锁存复位、遥测刷新与过期、断线和超时撤权、隔离模拟底盘与原模拟器。')
} finally { await rm(temp, { recursive: true, force: true }) }
