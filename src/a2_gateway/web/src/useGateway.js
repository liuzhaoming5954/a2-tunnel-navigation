import { computed, onBeforeUnmount, onMounted, ref, shallowRef, watch } from 'vue'

// 为 500 ms 的机身读数留出两跳传输的抖动余量。轮询保持串行，慢请求期间不并发。
const POLL_INTERVAL_MS = 150
const COCKPIT_CONTROL_STATUS_INTERVAL_MS = 500
// 运输抖动跨过有效期时，只在驾驶舱短暂保留并标明上次读数。
const COCKPIT_READING_GRACE_MS = 200
// PC2 的 IMU 差分转速比姿态角有效期更短。前端轮询停顿时也单独撤下转速。
const MAX_IMU_YAW_RATE_AGE_MS = 250
const VELOCITY_INTERVAL_MS = 100
// 只给首次起步补偿键盘采样时差。PC2 原有 400 毫秒指令看门狗继续独立判定。
const KEYBOARD_START_RETRY_INTERVAL_MS = 35
const KEYBOARD_START_RETRY_MS = 280
// PC2 的只读守护报告最多有效 2.5 秒，旧版本响应或异常长的期限不能延长放行。
const MAX_REMOTE_STOP_AGE_MS = 2500
const UNKNOWN_REMOTE_STOP = Object.freeze({ guard: 'unknown', latched: null })
const signedRotation = new Intl.NumberFormat('zh-CN', {
  signDisplay: 'exceptZero', useGrouping: false, minimumFractionDigits: 1, maximumFractionDigits: 1,
})

export function formatImuYawRate(rateRadps) {
  if (!Number.isFinite(rateRadps)) return '—'
  const degreesPerSecond = rateRadps * 180 / Math.PI
  // 显示精度为 0.1°/s。静止附近的噪声显示为约零，原始估算仍保存在遥测中。
  return Math.abs(degreesPerSecond) < 0.1 ? '≈0.0' : signedRotation.format(degreesPerSecond)
}

export const DIRECTIONS = {
  forward: { label: '前进', key: 'W', linear: 1, angular: 0 },
  back: { label: '后退', key: 'S', linear: -1, angular: 0 },
  left: { label: '左转', key: 'A', linear: 0, angular: 1 },
  right: { label: '右转', key: 'D', linear: 0, angular: -1 },
}

const ERROR_MESSAGES = {
  control_unavailable: '当前控制不可用',
  control_disabled: '当前控制已禁用',
  invalid_session: '控制会话已失效',
  session_expired: '控制会话已过期',
  lease_expired: '控制会话已到期',
  deadman_expired: '连续控制指令超时',
  stale_sequence: '控制序号已过期',
  session_active: '已有控制会话',
  session_busy: '已有其他控制会话',
  readonly: '当前为只读模式',
  robot_readonly: '当前实机为只读模式',
  receiver_stop_unverified: '缺少底盘接收端失联停车凭证',
  motion_arbiter_unavailable: '运动仲裁服务不可用',
  motion_arbiter_rejected: '运动仲裁服务拒绝请求',
  pc2_unavailable: 'PC2 未连接',
  manual_control_disabled: 'PC2 手动遥控未启用',
  manual_link_offline: '驾驶舱至 PC2 的控制隧道未连接',
  manual_robot_mismatch: '遥控端与遥测端的机器人标识不一致',
  manual_invalid_response: 'PC2 遥控接口响应无效',
  manual_auth_failed: '驾驶舱至 PC2 的控制通道鉴权失败',
  manual_receiver_source_mismatch: 'PC2 实机遥控入口接到了模拟底盘',
  stop_latched: '停车已锁存',
  stop_guard_not_ready: '机载停车守护尚未放行',
  stop_guard_unknown: '停车守护状态未知',
  stop_latch_unknown: '停车锁存状态未知',
  stop_unconfirmed: '机载接收端正在确认停稳',
  local_authorization_required: '等待机载维护授权',
  feedback_stale: '机身状态反馈已过期',
  guard_stale: '方向通行状态已过期',
  standstill_required: '等待机身停稳',
  control_owned: '原厂控制器占用控制权',
  control_unknown: '等待新鲜的原厂手柄回中反馈',
  authorization_changed: '机载维护授权已改变',
  operator_hold_neutral_required: '松开实体方向键后再启用控制',
  operator_hold_stale: '独立键盘信号已失效',
  operator_hold_released: '独立键盘未确认按住',
  action_expired: '本段动作已到期',
  action_not_ready: '机身尚未就绪',
  invalid_action: '动作已失效',
  action_required: '等待新的方向按下',
  direction_not_authorized: '本次许可不包含该方向',
  obstacle: '检测到障碍，行进已受阻',
  map_unavailable: '暂无占据栅格',
}

export function describeError(reason) {
  if (!reason) return '原因未知'
  return ERROR_MESSAGES[reason] ?? reason
}

export async function request(path, { body, keepalive = false, timeout = 1800, controller = new AbortController() } = {}) {
  const timer = setTimeout(() => controller.abort(), timeout)
  try {
    const response = await fetch(path, {
      method: body === undefined ? 'GET' : 'POST',
      headers: body === undefined ? { Accept: 'application/json' } : {
        Accept: 'application/json',
        'Content-Type': 'application/json',
      },
      body: body === undefined ? undefined : JSON.stringify(body),
      cache: 'no-store',
      signal: controller.signal,
      keepalive,
    })
    const data = await response.json().catch(() => null)
    if (!response.ok || !data || data.ok === false || (body !== undefined && data.ok !== true)) {
      const reason = data?.reason ?? `HTTP ${response.status}`
      throw Object.assign(new Error(describeError(reason)), { reason, status: response.status })
    }
    return data
  } catch (error) {
    if (controller.signal.aborted) throw new Error('请求超时或已取消')
    if (error instanceof TypeError) throw new Error('网关连接中断')
    throw error
  } finally {
    clearTimeout(timer)
  }
}

export function useGateway(cockpitVisible) {
  const state = shallowRef(null)
  const telemetry = shallowRef(null)
  const telemetryConnection = ref('connecting')
  const map = shallowRef(null)
  const connection = ref('connecting')
  const mapStatus = ref('idle')
  const phase = ref('idle')
  const activeDirection = ref(null)
  const events = ref([])
  const fault = ref('')
  const ownSession = shallowRef(null)
  const pendingStop = shallowRef(null)
  const actionExpiresAt = ref(null)
  let eventId = 0
  let pollTimer
  let cancelDetailExpiry
  let telemetryController
  let velocityTimer
  let leaseTimer
  let pollController
  let mapController
  let velocityController
  let actionController
  let activeAction
  let keyboardStartRetryUntil = null
  let mapRequestId
  let mapRetryAt = 0
  let stopInFlight = false
  let armInFlight = false
  let nextStopRetryAt = 0
  let lastPollStartedAt = -Infinity
  let intent = 0
  let sequence = 0
  let disposed = false

  const limits = computed(() => {
    const source = state.value?.control?.limits
    // 旧 PC2 或独立模拟器只上报双向线速度。
    const forward = source?.forward_mps ?? source?.linear_mps
    return {
      linear: Number.isFinite(source?.linear_mps) && source.linear_mps > 0 ? source.linear_mps : 0,
      forward: Number.isFinite(forward) && forward > 0 ? forward : 0,
      angular: Number.isFinite(source?.angular_radps) && source.angular_radps > 0 ? source.angular_radps : 0,
    }
  })
  // 只有来自 PC2 遥控后端的快照才代表远端会话。站端在链路断开时也能
  // 返回 HTTP 200，此时的 armed=false 不能作为 PC2 已撤权的证据。
  const controlSource = computed(() => {
    const snapshot = state.value
    if (snapshot?.mode === 'manual_control') {
      if (snapshot.deployment === 'station' && snapshot.pc2_connected === true
          && ['mock', 'unitree'].includes(snapshot.control?.source)) return snapshot.control.source
      return null
    }
    return ['simulator', 'robot'].includes(snapshot?.mode) ? snapshot.mode : null
  })
  // 控制许可与停车守护是两个独立的现场信号。模拟接收器沿用自己的锁存，
  // 实机站端即使收到 available=true，也须有这次连接的新鲜守护报告。
  const remoteStop = computed(() => {
    const snapshot = state.value
    if (!snapshot || (snapshot.deployment !== 'station' && snapshot.mode !== 'robot')) return null
    if (snapshot.mode === 'manual_control' && snapshot.control?.source === 'mock') return null
    if (connection.value !== 'connected' || snapshot.pc2_connected !== true) return UNKNOWN_REMOTE_STOP
    return snapshot.remote_stop ?? UNKNOWN_REMOTE_STOP
  })
  const independentTelemetry = computed(() => cockpitVisible?.value === true
    && state.value?.deployment === 'station' && state.value.independent_telemetry === true)
  const cockpitState = computed(() => {
    if (!independentTelemetry.value) return state.value
    const { robot, pc2_connected, pc2_reason } = telemetry.value ?? {}
    return { ...state.value, robot, pc2_connected, pc2_reason,
      stop_reset: connection.value === 'connected' ? state.value.stop_reset : null }
  })
  const cockpitConnection = computed(() => independentTelemetry.value ? telemetryConnection.value : connection.value)
  const cockpitRemoteStop = computed(() => {
    if (!independentTelemetry.value || !remoteStop.value) return remoteStop.value
    if (telemetryConnection.value !== 'connected' || telemetry.value?.pc2_connected !== true) return UNKNOWN_REMOTE_STOP
    return telemetry.value.remote_stop ?? UNKNOWN_REMOTE_STOP
  })
  const remoteStopBlock = computed(() => {
    const report = remoteStop.value
    if (!report) return null
    if (report.latched === true) return 'stop_latched'
    if (report.guard === 'stopped') return 'stop_guard_not_ready'
    if (report.guard !== 'running') return 'stop_guard_unknown'
    return report.latched === false ? null : 'stop_latch_unknown'
  })
  const available = computed(() => (
    connection.value === 'connected'
    && state.value?.robot?.online === true
    && Boolean(controlSource.value)
    && !remoteStopBlock.value
    && state.value?.control?.available === true
    && Boolean(limits.value.linear || limits.value.angular)
  ))
  const canArm = computed(() => available.value && state.value.control.armed === false
    && !ownSession.value && !pendingStop.value && !['arming', 'stopping'].includes(phase.value))
  const canStop = computed(() => Boolean(ownSession.value || pendingStop.value || phase.value === 'arming' || phase.value === 'stopping'))
  const hasControlSession = computed(() => ownSession.value !== null)
  const repeatedActions = computed(() => ownSession.value?.repeatedActions === true)
  const continuousHold = computed(() => ownSession.value?.continuousHold === true)
  // 供维护页显示剩余时间，与现有会话超时使用同一个本机单调时钟截止点。
  const controlExpiresAt = computed(() => ownSession.value?.expiresAt ?? null)

  function log(level, message) {
    events.value = [{ id: ++eventId, time: Date.now(), level, message }, ...events.value].slice(0, 30)
  }

  function clearEvents() {
    events.value = []
  }

  function canDrive(direction) {
    const requested = DIRECTIONS[direction]
    const control = state.value?.control
    const ready = repeatedActions.value
      ? (phase.value === 'ready' && control?.armed === true && control.action_state === 'ready' && !activeAction)
        || (activeDirection.value === direction && ['action_starting', 'moving', 'release_required'].includes(phase.value))
      : ['ready', 'moving'].includes(phase.value)
    return Boolean(
      requested
      && available.value
      && ownSession.value?.source === controlSource.value
      && ready
      && ((requested.linear && linearLimit(direction)) || (requested.angular && limits.value.angular))
    )
  }

  function linearLimit(direction) {
    return direction === 'forward' ? limits.value.forward : limits.value.linear
  }

  async function loadMap(metadata) {
    if (!metadata) {
      mapController?.abort()
      mapController = undefined
      mapRequestId = undefined
      map.value = null
      mapStatus.value = 'idle'
      return
    }
    if (metadata.id === map.value?.id || metadata.id === mapRequestId || performance.now() < mapRetryAt) return
    mapController?.abort()
    map.value = null
    const controller = new AbortController()
    mapController = controller
    mapRequestId = metadata.id
    mapStatus.value = 'loading'
    try {
      const next = await request('/api/map', { controller, timeout: 3500 })
      if (disposed || controller.signal.aborted || next.id !== state.value?.map?.id) return
      if (!Number.isInteger(next.width) || next.width < 1 || !Number.isInteger(next.height) || next.height < 1
          || !Number.isFinite(next.resolution_m) || next.resolution_m <= 0
          || ![next.origin_x_m, next.origin_y_m, next.origin_yaw_rad].every(Number.isFinite)
          || !Array.isArray(next.cells) || next.cells.length !== next.width * next.height
          || next.cells.some(cell => !Number.isInteger(cell) || cell < -1 || cell > 100)) {
        throw new Error('地图栅格与尺寸不一致')
      }
      map.value = next
      mapStatus.value = 'ready'
      log('info', '占据栅格已加载')
    } catch (error) {
      if (mapController !== controller || disposed) return
      mapStatus.value = 'error'
      mapRetryAt = performance.now() + 1800
      log('warn', `地图加载失败：${error.message}`)
    } finally {
      if (mapController === controller) mapRequestId = undefined
    }
  }

  // 在浏览器的单调时钟上给每组观测单独计时。后续轮询卡住时，
  // 仍须按期撤下旧读数；本机与 PC2 的墙钟差不参与判定。
  function publishTimedSnapshot(next, startedAt, apply, readingGrace = 0, previous = null) {
    const robot = next.robot
    const attitudeSample = robot.attitude
    const imuConnected = robot.online === true && (next.deployment !== 'station' || next.pc2_connected === true)
    const rateAge = attitudeSample?.age_ms
    const rateLimit = Math.min(attitudeSample?.max_age_ms, MAX_IMU_YAW_RATE_AGE_MS)
    const freshRate = imuConnected && Number.isFinite(attitudeSample?.yaw_rate_radps)
      && Number.isFinite(rateAge) && Number.isFinite(attitudeSample.max_age_ms) && rateAge >= 0 && rateAge < rateLimit
      ? { rateRadps: attitudeSample.yaw_rate_radps, expiresAt: startedAt + rateLimit - rateAge } : null
    // 短暂缺测只可沿用同一在线来源的上一帧，原始截止时间固定，不会随空帧延长。
    const previousRate = imuConnected && previous?.robot && previous.deployment === next.deployment
      && previous.robot.id === robot.id && previous.robot.source === robot.source ? previous.robot.imuYaw : null
    const imuRate = freshRate ?? previousRate
    let expiryTimer
    function publish(expired) {
      const now = performance.now()
      const elapsed = now - startedAt
      let earliestExpiry = Infinity
      function fresh(sample, grace = 0, maxAge = Infinity) {
        if (!sample || !Number.isFinite(sample.age_ms) || sample.age_ms < 0
            || !Number.isFinite(sample.max_age_ms)) return null
        const remaining = Math.min(sample.max_age_ms, maxAge) - sample.age_ms - elapsed
        if (remaining > 0) {
          earliestExpiry = Math.min(earliestExpiry, remaining)
          return sample
        }
        if (remaining + grace <= 0) return null
        earliestExpiry = Math.min(earliestExpiry, remaining + grace)
        return { ...sample, previous_reading: true }
      }
      const report = next.remote_stop
      const validGuard = report?.source === 'pc2_systemd' && ['running', 'stopped', 'unknown'].includes(report.guard)
        && [null, true, false].includes(report.latched) && !(report.latched === false && report.guard !== 'running')
        && Number.isInteger(report.age_ms) && Number.isInteger(report.max_age_ms)
        && report.max_age_ms > 0 && report.max_age_ms <= MAX_REMOTE_STOP_AGE_MS
      const current = { ...next, remote_stop: validGuard ? fresh(report) : null }
      if (robot.batteries || robot.velocity || robot.attitude) {
        let attitude = fresh(robot.attitude, readingGrace)
        if (Number.isFinite(attitude?.yaw_rate_radps) && !fresh(attitude, 0, MAX_IMU_YAW_RATE_AGE_MS)) {
          attitude = { ...attitude, yaw_rate_radps: null }
        }
        current.robot = { ...robot,
          batteries: { primary: fresh(robot.batteries?.primary, readingGrace),
            secondary: fresh(robot.batteries?.secondary, readingGrace) },
          velocity: fresh(robot.velocity, readingGrace), attitude,
        }
      }
      // 转速标签保持固定。为短时传输抖动预留 200 毫秒，超过显示期限时清空读数。
      const rateRemaining = imuRate ? imuRate.expiresAt + COCKPIT_READING_GRACE_MS - now : -Infinity
      const imuYaw = rateRemaining > 0 ? imuRate : null
      if (imuYaw) earliestExpiry = Math.min(earliestExpiry, rateRemaining)
      current.robot = { ...current.robot, imuYaw }
      apply(current, expired)
      if (Number.isFinite(earliestExpiry)) {
        expiryTimer = setTimeout(() => publish(true), Math.ceil(earliestExpiry))
      }
    }
    publish(false)
    return () => clearTimeout(expiryTimer)
  }

  // 驾驶舱的读数直连站端只读入口，控制状态请求即使挂起也不会占用遥测轮询。
  watch(independentTelemetry, (enabled, _, onCleanup) => {
    if (!enabled) return
    let stopped = false
    let nextPoll
    let cancelExpiry
    telemetry.value = null
    telemetryConnection.value = 'connecting'
    onCleanup(() => {
      stopped = true
      clearTimeout(nextPoll)
      cancelExpiry?.()
      telemetryController?.abort()
    })
    async function pollTelemetry() {
      const startedAt = performance.now()
      const controller = new AbortController()
      telemetryController = controller
      try {
        const next = await request('/api/telemetry', { controller })
        if (stopped) return
        if (next.deployment !== 'station' || !next.robot || typeof next.pc2_connected !== 'boolean') {
          throw new Error('网关返回了不完整的只读遥测')
        }
        cancelExpiry?.()
        cancelExpiry = publishTimedSnapshot(next, startedAt, snapshot => {
          telemetry.value = snapshot
      }, COCKPIT_READING_GRACE_MS, telemetryConnection.value === 'connected' ? telemetry.value : null)
        telemetryConnection.value = 'connected'
      } catch {
        if (!stopped) telemetryConnection.value = 'disconnected'
      } finally {
        if (!stopped) nextPoll = setTimeout(pollTelemetry, Math.max(0, POLL_INTERVAL_MS - (performance.now() - startedAt)))
      }
    }
    void pollTelemetry()
  })

  async function poll() {
    if (disposed) return
    const startedAt = performance.now()
    const controller = new AbortController()
    pollController = controller
    try {
      const next = await request('/api/state', { controller })
      if (disposed) return
      if (!next.robot || !next.control || !['simulator', 'robot', 'robot_readonly', 'manual_control'].includes(next.mode)) {
        throw new Error('网关返回了不完整的状态')
      }
      const wasConnected = connection.value === 'connected'
      const wasOnline = state.value?.robot?.online === true
      cancelDetailExpiry?.()
      cancelDetailExpiry = publishTimedSnapshot(next, startedAt, (snapshot, expired) => {
        state.value = snapshot
        // 请求卡住时也要撤销正在启用或已经持有的实机会话，停止请求不受锁存拦截。
        if (expired && (ownSession.value || phase.value === 'arming') && remoteStopBlock.value) {
          void stop(describeError(remoteStopBlock.value))
        }
      }, 0, wasConnected ? state.value : null)
      lastPollStartedAt = startedAt
      connection.value = 'connected'
      if (!wasConnected) log('ok', '操作网关已连接')
      if (next.robot.online && (!wasConnected || !wasOnline)) log('ok', '机器人遥测在线')
      if (!next.robot.online && wasOnline) log('warn', '机器人遥测离线')
      if (((ownSession.value || phase.value === 'arming') && !available.value)
          || (ownSession.value && ownSession.value.source !== controlSource.value)) {
        void stop('机器人不可控，解除控制')
      } else if (ownSession.value && startedAt >= ownSession.value.confirmedAt) {
        if (next.control.armed === false) {
          void stop(describeError(next.control.reason))
        } else if (ownSession.value.repeatedActions) {
          updateActionState(next.control, startedAt)
        } else if (next.control.reason === 'obstacle' && activeDirection.value) {
          void stop('检测到障碍，停止并解除控制')
        }
      }
      if (pendingStop.value && performance.now() >= nextStopRetryAt) void sendStop()
      void loadMap(next.map)
    } catch (error) {
      if (disposed) return
      if (connection.value !== 'disconnected') log('warn', `网关离线：${error.message}`)
      connection.value = 'disconnected'
      if (ownSession.value || phase.value === 'arming') void stop('网关离线，解除控制')
    } finally {
      // 以请求发出时刻为轮询起点。保持同一时刻最多一个请求，避免把响应耗时
      // 叠加到每一轮间隔，导致两份正常遥测之间短暂显示为过期。
      const interval = independentTelemetry.value && !ownSession.value && !pendingStop.value && phase.value !== 'arming'
        ? COCKPIT_CONTROL_STATUS_INTERVAL_MS : POLL_INTERVAL_MS
      if (!disposed) pollTimer = setTimeout(poll, Math.max(0, interval - (performance.now() - startedAt)))
    }
  }

  async function arm() {
    if (!canArm.value || armInFlight || document.hidden) return
    const current = ++intent
    const source = controlSource.value
    const startedAt = performance.now()
    phase.value = 'arming'
    fault.value = ''
    armInFlight = true
    try {
      const result = await request('/api/control/session', { body: {} })
      if (typeof result.session_id !== 'string' || !result.session_id) throw new Error('网关未返回有效会话')
      const lifetime = Number.isFinite(result.expires_in_ms) ? result.expires_in_ms : 0
      const repeats = result.repeated_actions === true
        && state.value?.mode === 'manual_control' && Number.isSafeInteger(result.action_limit_ms)
        && result.action_limit_ms > 0 && result.action_limit_ms <= 30000
      // 持续按压由 PC2 显式授予，无总倒计时。混入其他许可字段时立即交回会话。
      const hasContinuousMode = Object.hasOwn(result, 'continuous_hold')
      const continuous = result.continuous_hold === true && state.value?.mode === 'manual_control'
        && Object.keys(result).length === 3 && Object.hasOwn(result, 'ok') && Object.hasOwn(result, 'session_id')
      const session = { id: result.session_id, source, expiresAt: repeats || continuous ? null : startedAt + lifetime,
        confirmedAt: performance.now(), repeatedActions: repeats, actionLimitMs: result.action_limit_ms,
        continuousHold: continuous }
      const modeValid = hasContinuousMode ? continuous
        : repeats || (result.repeated_actions !== true && session.expiresAt > performance.now())
      if (disposed || intent !== current || !available.value || source !== controlSource.value
          || document.hidden || !modeValid) {
        pendingStop.value = { ...session, stoppedAt: performance.now() }
        await sendStop()
        return
      }
      ownSession.value = session
      sequence = 0
      if (session.expiresAt !== null) {
        leaseTimer = setTimeout(() => void stop('控制会话到期'), session.expiresAt - performance.now())
      }
      phase.value = 'ready'
      log('ok', '控制会话已启用')
      // PC2 从授予会话起就监测链路。方向尚未按下时只发零帧，PC2 不向底盘发运动输出。
      if (state.value.mode === 'manual_control') void sendVelocity(session, null)
    } catch (error) {
      if (intent === current && !disposed) {
        phase.value = 'idle'
        fault.value = error.message
        log('warn', `启用控制失败：${error.message}`)
      }
    } finally {
      armInFlight = false
      if (phase.value === 'stopping' && !pendingStop.value) phase.value = 'idle'
    }
  }

  async function sendStop() {
    const session = pendingStop.value
    if (!session || stopInFlight) return
    stopInFlight = true
    phase.value = 'stopping'
    nextStopRetryAt = performance.now() + 1400
    try {
      await request('/api/control/stop', { body: { session_id: session.id }, keepalive: true })
      if (pendingStop.value !== session) return
      pendingStop.value = null
      phase.value = 'idle'
      fault.value = ''
      if (!disposed) log('ok', '停止请求已确认，控制已解除')
    } catch (error) {
      if (pendingStop.value !== session) return
      if ((error.reason === 'invalid_session' || (session.expiresAt !== null && performance.now() >= session.expiresAt))
          && lastPollStartedAt >= session.stoppedAt && session.source === controlSource.value
          && available.value && state.value.control.armed === false) {
        pendingStop.value = null
        phase.value = 'idle'
        fault.value = ''
        if (!disposed) log('info', '网关已确认旧控制会话结束')
        return
      }
      phase.value = 'stop_unconfirmed'
      if (fault.value !== error.message && !disposed) log('warn', `停止请求待确认：${error.message}`)
      fault.value = error.message
    } finally {
      stopInFlight = false
    }
  }

  async function stop(reason = '手动停止') {
    if (!ownSession.value && !pendingStop.value && !armInFlight) return
    intent += 1
    clearTimeout(velocityTimer)
    clearTimeout(leaseTimer)
    velocityController?.abort()
    actionController?.abort()
    activeAction = null
    keyboardStartRetryUntil = null
    actionExpiresAt.value = null
    activeDirection.value = null
    if (ownSession.value) {
      // 仅接受停车意图之后发起的新轮询。延迟到达的旧快照可能仍显示未启用。
      pendingStop.value = { ...ownSession.value, stoppedAt: performance.now() }
      ownSession.value = null
      log('info', reason)
    }
    phase.value = 'stopping'
    await sendStop()
  }

  function retryKeyboardStart(error, requestSequence, requestStartedAt, sameIntent, retry, fail) {
    const deadline = keyboardStartRetryUntil
    if (error.status !== 409 || error.reason !== 'operator_hold_released' || deadline === null
        || state.value?.mode !== 'manual_control' || (controlSource.value !== 'unitree' && !continuousHold.value)) return false
    const now = performance.now()
    const canRetry = !document.hidden && now + KEYBOARD_START_RETRY_INTERVAL_MS < deadline
    const elapsed = Math.round(now - (deadline - KEYBOARD_START_RETRY_MS))
    const roundTrip = Math.round(now - requestStartedAt)
    log('info', `PC2 回执为键盘松开（序号 ${requestSequence}，按下后 ${elapsed} ms，往返 ${roundTrip} ms）${canRetry ? `，${KEYBOARD_START_RETRY_INTERVAL_MS} ms 后重试` : '，停止重试'}`)
    if (!canRetry) return false
    // 浏览器按下可能先于独立采样到达 PC2。快速重试期间，每个包仍由 PC2 核对同方向实体键。
    velocityTimer = setTimeout(() => {
      if (!sameIntent()) return
      if (performance.now() < deadline && !document.hidden) retry()
      else fail()
    }, KEYBOARD_START_RETRY_INTERVAL_MS)
    return true
  }

  async function sendVelocity(session, direction, action = null) {
    const current = intent
    const idle = direction === null
    const sameIntent = () => !disposed && ownSession.value === session && intent === current
      && (activeDirection.value === direction || (idle && session.repeatedActions))
    if (!sameIntent()) return
    const permitted = idle
      ? state.value?.mode === 'manual_control' && available.value && session.source === controlSource.value
        && (session.repeatedActions ? ['ready', 'action_stopping', 'release_required'].includes(phase.value) : phase.value === 'ready')
      : canDrive(direction) && (!session.repeatedActions || (phase.value === 'moving' && activeAction === action))
    if (!permitted || document.hidden || (session.expiresAt !== null && performance.now() >= session.expiresAt)) {
      void stop('控制条件失效')
      return
    }
    const requested = DIRECTIONS[direction]
    const controller = new AbortController()
    velocityController = controller
    const requestSequence = ++sequence
    const requestStartedAt = performance.now()
    try {
      await request('/api/control/velocity', {
        body: {
          session_id: session.id,
          sequence: requestSequence,
          linear_mps: idle ? 0 : requested.linear * linearLimit(direction),
          angular_radps: idle ? 0 : requested.angular * limits.value.angular,
          ...(action ? { action_id: action.id } : {}),
        },
        controller,
        timeout: 650,
      })
      if (sameIntent()) {
        if (!idle && keyboardStartRetryUntil !== null) {
          const elapsed = Math.round(performance.now() - (keyboardStartRetryUntil - KEYBOARD_START_RETRY_MS))
          log('ok', `PC2 已确认首条速度请求（序号 ${requestSequence}，按下后 ${elapsed} ms）`)
          keyboardStartRetryUntil = null
        }
        velocityTimer = setTimeout(() => void sendVelocity(session, direction, action), VELOCITY_INTERVAL_MS)
      }
    } catch (error) {
      if (sameIntent()) {
        if (action && error.reason === 'action_expired') {
          awaitPhysicalRelease(session, action)
          return
        }
        const fail = () => {
          const elapsed = Math.round(performance.now() - requestStartedAt)
          log('warn', `遥控请求失败：${error.message}（序号 ${requestSequence}，${elapsed} ms）`)
          void stop('遥控请求中断，解除控制')
        }
        if (idle || !retryKeyboardStart(error, requestSequence, requestStartedAt, sameIntent,
          () => void sendVelocity(session, direction, action), fail)) fail()
      }
    } finally {
      if (velocityController === controller) velocityController = undefined
    }
  }

  function awaitPhysicalRelease(session, action) {
    if (activeAction !== action || action.released || phase.value === 'release_required') return
    intent += 1
    clearTimeout(velocityTimer)
    velocityController?.abort()
    actionExpiresAt.value = null
    keyboardStartRetryUntil = null
    // 保留实体按键或指针状态，到期后按住不动也不能自动开始下一段。
    phase.value = 'release_required'
    log('info', '本段动作结束，等待松键')
    void sendVelocity(session, null)
  }

  function updateActionState(control, startedAt) {
    if (!['ready', 'active', 'stopping', 'release_required'].includes(control.action_state)) {
      void stop('网关未确认动作状态，解除控制')
      return
    }
    const action = activeAction
    if (!action || startedAt < action.startedAt) return
    if (action.released) {
      // 停车回执只确认收到释放。必须用回执之后发出的轮询确认 PC2 已停稳。
      if (action.releaseConfirmedAt !== null && startedAt >= action.releaseConfirmedAt
          && control.action_state === 'ready' && control.action_id == null) {
        activeAction = null
        phase.value = 'ready'
        log('ok', '停稳已确认，可以重新按键')
      }
    } else if (control.action_id === action.id && ['stopping', 'release_required'].includes(control.action_state)) {
      awaitPhysicalRelease(ownSession.value, action)
    }
  }

  async function startAction(session, action) {
    const current = intent
    const sameIntent = () => !disposed && intent === current && ownSession.value === session && activeAction === action
      && activeDirection.value === action.direction && phase.value === 'action_starting'
    if (!sameIntent()) return
    if (!canDrive(action.direction) || document.hidden) {
      void stop('控制条件失效')
      return
    }
    const controller = new AbortController()
    actionController = controller
    const requestStartedAt = performance.now()
    try {
      // 起步与保活共用序号。等回执再发更高序号，避免空闲保活抢先抵达造成误拒。
      const result = await request('/api/control/action/start', { body: {
        session_id: session.id, sequence: action.id,
        direction: action.direction === 'back' ? 'backward' : action.direction,
      }, controller, timeout: 650 })
      if (!sameIntent()) return
      if (result.action_id !== action.id || !Number.isSafeInteger(result.expires_in_ms)
          || result.expires_in_ms <= 0 || result.expires_in_ms > session.actionLimitMs
          || !available.value || document.hidden) {
        throw new Error('网关未确认本段动作')
      }
      actionExpiresAt.value = action.startedAt + result.expires_in_ms
      phase.value = 'moving'
      log('info', `开始${DIRECTIONS[action.direction].label}`)
      void sendVelocity(session, action.direction, action)
    } catch (error) {
      if (sameIntent()) {
        const fail = () => {
          const elapsed = Math.round(performance.now() - requestStartedAt)
          log('warn', `动作申请失败：${error.message}（序号 ${action.id}，${elapsed} ms）`)
          void stop('动作申请未确认，解除控制')
        }
        if (!retryKeyboardStart(error, action.id, requestStartedAt, sameIntent,
          () => void startAction(session, action), fail)) fail()
      }
    } finally {
      if (actionController === controller) actionController = undefined
    }
  }

  async function releaseAction(session, action) {
    const current = ++intent
    clearTimeout(velocityTimer)
    velocityController?.abort()
    actionController?.abort()
    action.released = true
    keyboardStartRetryUntil = null
    activeDirection.value = null
    actionExpiresAt.value = null
    phase.value = 'action_stopping'
    const controller = new AbortController()
    actionController = controller
    try {
      // 即使起步回执仍在路上，也用原起步序号释放，让 PC2 拦住迟到的起步和速度包。
      await request('/api/control/action/release', { body: {
        session_id: session.id, sequence: ++sequence, action_id: action.id,
      }, controller, keepalive: true, timeout: 650 })
      if (intent !== current || ownSession.value !== session || activeAction !== action) return
      action.releaseConfirmedAt = performance.now()
      log('info', '已松键，等待停稳确认')
      void sendVelocity(session, null)
    } catch (error) {
      if (intent === current && ownSession.value === session) {
        log('warn', `动作释放失败：${error.message}`)
        void stop('动作释放未确认，解除控制')
      }
    } finally {
      if (actionController === controller) actionController = undefined
    }
  }

  function beginDirection(direction, input = 'pointer') {
    if (!canDrive(direction) || activeDirection.value || document.hidden
        || (continuousHold.value && input !== 'trusted_key')) return false
    // 第一条非零指令和刚刚发出的零帧可能交错到达。两者共用递增序号；
    // 取消空闲定时器并切换意图，使迟到的零帧回执不会续发保活或打断当前运动。
    intent += 1
    clearTimeout(velocityTimer)
    velocityController?.abort()
    activeDirection.value = direction
    const startedAt = performance.now()
    const realManual = state.value?.mode === 'manual_control' && controlSource.value === 'unitree'
    keyboardStartRetryUntil = input === 'trusted_key' && (realManual || continuousHold.value)
      ? startedAt + KEYBOARD_START_RETRY_MS : null
    if (realManual) {
      const inputLabel = input === 'trusted_key' ? '可信键盘事件' : input === 'untrusted_key' ? '非可信键盘事件' : '屏幕方向键'
      log('info', `网页收到${DIRECTIONS[direction].label}输入（${inputLabel}）`)
    }
    if (repeatedActions.value) {
      activeAction = { id: ++sequence, direction, startedAt, released: false, releaseConfirmedAt: null }
      phase.value = 'action_starting'
      void startAction(ownSession.value, activeAction)
      return true
    }
    phase.value = 'moving'
    log('info', `开始${DIRECTIONS[direction].label}`)
    void sendVelocity(ownSession.value, direction)
    return true
  }

  function endDirection(direction) {
    if (activeDirection.value !== direction) return
    if (repeatedActions.value && activeAction) {
      void releaseAction(ownSession.value, activeAction)
    } else {
      void stop('松开方向键，停止并解除控制')
    }
  }

  function blur() {
    void stop('操作窗口失焦，解除控制')
  }

  function visibility() {
    if (document.hidden) void stop('操作页面离开前台，解除控制')
  }

  function offline() {
    connection.value = 'disconnected'
    pollController?.abort()
    telemetryConnection.value = 'disconnected'
    telemetryController?.abort()
    void stop('浏览器断网，解除控制')
  }

  function pageHide() {
    void stop('关闭操作页面，解除控制')
  }

  onMounted(() => {
    void poll()
    window.addEventListener('blur', blur)
    window.addEventListener('offline', offline)
    window.addEventListener('pagehide', pageHide)
    document.addEventListener('visibilitychange', visibility)
  })

  onBeforeUnmount(() => {
    disposed = true
    pageHide()
    clearTimeout(pollTimer)
    cancelDetailExpiry?.()
    clearTimeout(velocityTimer)
    clearTimeout(leaseTimer)
    pollController?.abort()
    mapController?.abort()
    window.removeEventListener('blur', blur)
    window.removeEventListener('offline', offline)
    window.removeEventListener('pagehide', pageHide)
    document.removeEventListener('visibilitychange', visibility)
  })

  return { state, map, connection, mapStatus, phase, activeDirection, events, fault,
    cockpitState, cockpitConnection, cockpitRemoteStop,
    remoteStop, remoteStopBlock, limits, available, canArm, canStop, canDrive, controlExpiresAt,
    hasControlSession, repeatedActions, continuousHold, actionExpiresAt,
    arm, stop, beginDirection, endDirection, clearEvents }
}
