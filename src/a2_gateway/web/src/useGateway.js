import { computed, onBeforeUnmount, onMounted, ref, shallowRef } from 'vue'

const POLL_INTERVAL_MS = 200
const VELOCITY_INTERVAL_MS = 100

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
  receiver_stop_unverified: '底盘接收端失联停车待验证',
  motion_arbiter_unavailable: '运动仲裁服务不可用',
  motion_arbiter_rejected: '运动仲裁服务拒绝请求',
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

export function useGateway() {
  const state = shallowRef(null)
  const map = shallowRef(null)
  const connection = ref('connecting')
  const mapStatus = ref('idle')
  const phase = ref('idle')
  const activeDirection = ref(null)
  const events = ref([])
  const fault = ref('')
  const ownSession = shallowRef(null)
  const pendingStop = shallowRef(null)
  let eventId = 0
  let pollTimer
  let velocityTimer
  let leaseTimer
  let pollController
  let mapController
  let velocityController
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
    return {
      linear: Number.isFinite(source?.linear_mps) && source.linear_mps > 0 ? source.linear_mps : 0,
      angular: Number.isFinite(source?.angular_radps) && source.angular_radps > 0 ? source.angular_radps : 0,
    }
  })
  const available = computed(() => (
    connection.value === 'connected'
    && state.value?.robot?.online === true
    && state.value?.mode !== 'robot_readonly'
    && state.value?.control?.available === true
    && Boolean(limits.value.linear || limits.value.angular)
  ))
  const canArm = computed(() => available.value && !ownSession.value && !pendingStop.value && !['arming', 'stopping'].includes(phase.value))
  const canStop = computed(() => Boolean(ownSession.value || pendingStop.value || phase.value === 'arming' || phase.value === 'stopping'))

  function log(level, message) {
    events.value = [{ id: ++eventId, time: Date.now(), level, message }, ...events.value].slice(0, 30)
  }

  function clearEvents() {
    events.value = []
  }

  function canDrive(direction) {
    const requested = DIRECTIONS[direction]
    return Boolean(
      requested
      && available.value
      && ownSession.value
      && ['ready', 'moving'].includes(phase.value)
      && ((requested.linear && limits.value.linear) || (requested.angular && limits.value.angular))
    )
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

  async function poll() {
    if (disposed) return
    const startedAt = performance.now()
    const controller = new AbortController()
    pollController = controller
    try {
      const next = await request('/api/state', { controller })
      if (disposed) return
      if (!next.robot || !next.control || !['simulator', 'robot', 'robot_readonly'].includes(next.mode)) {
        throw new Error('网关返回了不完整的状态')
      }
      const wasConnected = connection.value === 'connected'
      const wasOnline = state.value?.robot?.online === true
      state.value = next
      lastPollStartedAt = startedAt
      connection.value = 'connected'
      if (!wasConnected) log('ok', '操作网关已连接')
      if (next.robot.online && (!wasConnected || !wasOnline)) log('ok', '机器人遥测在线')
      if (!next.robot.online && wasOnline) log('warn', '机器人遥测离线')
      if ((ownSession.value || phase.value === 'arming') && !available.value) {
        void stop('机器人不可控，解除控制')
      } else if (ownSession.value && startedAt >= ownSession.value.confirmedAt) {
        if (next.control.armed === false) void stop(describeError(next.control.reason))
        if (next.control.reason === 'obstacle' && activeDirection.value) {
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
      if (!disposed) pollTimer = setTimeout(poll, POLL_INTERVAL_MS)
    }
  }

  async function arm() {
    if (!canArm.value || armInFlight || document.hidden) return
    const current = ++intent
    const startedAt = performance.now()
    phase.value = 'arming'
    fault.value = ''
    armInFlight = true
    try {
      const result = await request('/api/control/session', { body: {} })
      if (typeof result.session_id !== 'string' || !result.session_id) throw new Error('网关未返回有效会话')
      const lifetime = Number.isFinite(result.expires_in_ms) ? result.expires_in_ms : 0
      const session = { id: result.session_id, expiresAt: startedAt + lifetime, confirmedAt: performance.now() }
      if (disposed || intent !== current || !available.value || document.hidden || session.expiresAt <= performance.now()) {
        pendingStop.value = session
        await sendStop()
        return
      }
      ownSession.value = session
      sequence = 0
      leaseTimer = setTimeout(() => void stop('控制会话到期'), session.expiresAt - performance.now())
      phase.value = 'ready'
      log('ok', '控制会话已启用')
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
      if ((error.reason === 'invalid_session' || performance.now() >= session.expiresAt)
          && lastPollStartedAt >= session.confirmedAt
          && state.value?.control?.armed === false && connection.value === 'connected') {
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
    activeDirection.value = null
    if (ownSession.value) {
      pendingStop.value = ownSession.value
      ownSession.value = null
      log('info', reason)
    }
    phase.value = 'stopping'
    await sendStop()
  }

  async function sendVelocity(session, direction) {
    if (disposed || ownSession.value !== session || activeDirection.value !== direction) return
    if (!canDrive(direction) || performance.now() >= session.expiresAt) {
      void stop('控制条件失效')
      return
    }
    const requested = DIRECTIONS[direction]
    const controller = new AbortController()
    velocityController = controller
    try {
      await request('/api/control/velocity', {
        body: {
          session_id: session.id,
          sequence: ++sequence,
          linear_mps: requested.linear * limits.value.linear,
          angular_radps: requested.angular * limits.value.angular,
        },
        controller,
        timeout: 650,
      })
      if (ownSession.value === session && activeDirection.value === direction) {
        velocityTimer = setTimeout(() => void sendVelocity(session, direction), VELOCITY_INTERVAL_MS)
      }
    } catch (error) {
      if (ownSession.value === session) {
        log('warn', `运动请求失败：${error.message}`)
        void stop('运动请求中断，解除控制')
      }
    } finally {
      if (velocityController === controller) velocityController = undefined
    }
  }

  function beginDirection(direction) {
    if (!canDrive(direction) || activeDirection.value || document.hidden) return false
    activeDirection.value = direction
    phase.value = 'moving'
    log('info', `开始${DIRECTIONS[direction].label}`)
    void sendVelocity(ownSession.value, direction)
    return true
  }

  function endDirection(direction) {
    if (activeDirection.value === direction) void stop('松开方向键，停止并解除控制')
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
    limits, available, canArm, canStop, canDrive, arm, stop, beginDirection, endDirection, clearEvents }
}
