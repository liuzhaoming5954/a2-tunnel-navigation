<script setup>
import { computed, onBeforeUnmount, onMounted, watch } from 'vue'
import Icon from './Icon.vue'
import MapView from './MapView.vue'
import { DIRECTIONS, describeError } from './useGateway.js'

// 网关由外壳持有，退出维护页后的停止重试仍由同一会话完成。
const props = defineProps({ gateway: { type: Object, required: true } })
const {
  state, map, connection, mapStatus, phase, activeDirection, events, fault,
  limits, available, canArm, canStop, canDrive, arm, stop, beginDirection, endDirection, clearEvents,
} = props.gateway

const directionIcons = { forward: 'up', back: 'down', left: 'left', right: 'right' }
const directionOrder = ['forward', 'left', 'back', 'right']
const keyDirections = { w: 'forward', s: 'back', a: 'left', d: 'right' }
const eventLabels = { ok: '成功', info: '操作', warn: '告警' }
const phaseLabels = {
  idle: '待启用',
  arming: '启用中',
  ready: '已启用',
  moving: '指令发送中',
  stopping: '停止确认中',
  stop_unconfirmed: '停止待确认',
}
const robotModes = {
  idle: '待命',
  standing: '站立',
  stopped: '停止',
  moving: '运动',
  walking: '行走',
  turning: '转向',
  sitting: '坐姿',
  lying: '卧姿',
  unknown: '未知',
  readonly: '只读',
  read_only: '只读',
  disconnected: '离线',
  simulated: '模拟',
  simulator: '模拟',
}
const clock = new Intl.DateTimeFormat('zh-CN', {
  hour: '2-digit', minute: '2-digit', second: '2-digit', hourCycle: 'h23',
})

const isConnected = computed(() => connection.value === 'connected')
const robot = computed(() => state.value?.robot)
const robotOnline = computed(() => isConnected.value && robot.value?.online === true)
const battery = computed(() => Number.isFinite(robot.value?.battery_percent) && robotOnline.value
  ? Math.max(0, Math.min(100, robot.value.battery_percent))
  : null)
const currentPose = computed(() => {
  const pose = state.value?.pose
  return isConnected.value && pose?.map_id === state.value?.map?.id
    && [pose?.x_m, pose?.y_m, pose?.yaw_rad].every(Number.isFinite)
    ? pose
    : null
})
const modeLabel = computed(() => ({
  simulator: '模拟器',
  robot: '实机',
  robot_readonly: '实机只读',
})[state.value?.mode] ?? '等待数据')
// 机器人离线时不沿用缓存的运动模式，未收录的机型模式按网关原值显示。
const robotMode = computed(() => robotOnline.value && robot.value.mode
  ? (robotModes[robot.value.mode] ?? robot.value.mode)
  : '—')
// 位姿通过当前地图校验后，再将网关的弧度换成维护面板使用的角度。
const heading = computed(() => currentPose.value ? currentPose.value.yaw_rad * 180 / Math.PI : null)

function age(stamp, sampleAge) {
  if (!isConnected.value) return '—'
  const now = state.value?.server_time_ms
  const delay = Number.isFinite(sampleAge) && sampleAge >= 0
    ? sampleAge
    : Number.isFinite(now) && Number.isFinite(stamp) ? now - stamp : null
  if (delay === null) return '—'
  if (delay < 0) return '时钟未对齐'
  return delay < 1000 ? `${Math.round(delay)} ms` : `${(delay / 1000).toFixed(1)} s`
}

function fixed(value, digits = 2) {
  return Number.isFinite(value) ? value.toFixed(digits) : '—'
}

function formatClock(value) {
  return Number.isFinite(value) ? clock.format(value) : '—'
}

const unavailableLabel = computed(() => {
  if (!isConnected.value) return '网关在线后可查看控制状态'
  if (!robotOnline.value) return '机器人遥测离线'
  if (state.value?.mode === 'robot_readonly') return '当前实机为只读模式'
  return state.value?.control?.reason ? describeError(state.value.control.reason) : '当前控制不可用'
})

let pointerControl
let keyControl

function pressPointer(direction, event) {
  if (event.button !== 0 || !beginDirection(direction)) return
  event.currentTarget.setPointerCapture(event.pointerId)
  pointerControl = { id: event.pointerId, direction }
}

function releasePointer(event) {
  if (event.pointerId !== pointerControl?.id) return
  const { direction } = pointerControl
  pointerControl = undefined
  endDirection(direction)
}

function keyDown(event) {
  // 编辑框或弹出导航获得焦点时，不把文字输入解释为行进意图。
  if (event.target?.closest?.('input, textarea, select, [contenteditable="true"], .portal-sidebar, .portal-bar')) return
  const key = event.key.toLowerCase()
  if ((key === ' ' || key === 'escape') && canStop.value) {
    event.preventDefault()
    if (!event.repeat) void stop('操作员触发停止')
    return
  }
  const direction = keyDirections[key]
  if (!direction || event.altKey || event.ctrlKey || event.metaKey || event.repeat || !canDrive(direction)) return
  event.preventDefault()
  if (beginDirection(direction)) keyControl = { key, direction }
}

function keyUp(event) {
  if (event.key.toLowerCase() !== keyControl?.key) return
  event.preventDefault()
  const { direction } = keyControl
  keyControl = undefined
  endDirection(direction)
}

watch(activeDirection, direction => {
  if (!direction) {
    keyControl = undefined
    pointerControl = undefined
  }
})

onMounted(() => {
  window.addEventListener('keydown', keyDown)
  window.addEventListener('keyup', keyUp)
})

onBeforeUnmount(() => {
  window.removeEventListener('keydown', keyDown)
  window.removeEventListener('keyup', keyUp)
})
</script>

<template>
  <section class="shell maintenance-shell">
    <section class="overview" aria-labelledby="overview-heading">
      <div>
        <h1 id="overview-heading">维护遥控</h1>
        <p class="robot-identity">
          <span>{{ robot?.model || 'Unitree A2' }}</span>
          <template v-if="robot?.id"><i></i><span class="mono">{{ robot.id }}</span></template>
        </p>
      </div>
      <span class="mode-chip" :class="state?.mode">
        <Icon name="radar" :size="16" /> {{ modeLabel }}
      </span>
    </section>

    <div class="dashboard">
      <section class="panel manual-panel" aria-labelledby="manual-heading">
        <header class="panel-header">
          <div class="panel-title">
            <span class="panel-icon"><Icon name="gamepad" /></span>
            <h2 id="manual-heading">手动控制</h2>
          </div>
        </header>
        <div class="manual-body">
          <div class="control-state" :class="phase" role="status">
            <i class="state-dot"></i>
            <span>{{ available || canStop ? phaseLabels[phase] : '控制不可用' }}</span>
          </div>
          <button type="button" class="arm-button" :disabled="!canArm" @click="arm">
            <Icon name="power" :size="17" /> {{ phase === 'arming' ? '启用中' : '启用控制' }}
          </button>
          <div class="control-divider"></div>

          <div class="control-label"><span>行进方向</span><Icon name="keyboard" :size="16" /></div>
          <div class="direction-pad" aria-label="按住发送行进指令">
            <button
              v-for="direction in directionOrder" :key="direction" type="button" class="direction-key"
              :class="[direction, { active: activeDirection === direction }]"
              :aria-label="DIRECTIONS[direction].label" :aria-pressed="activeDirection === direction"
              :disabled="!canDrive(direction)"
              @pointerdown.prevent="pressPointer(direction, $event)"
              @pointerup="releasePointer" @pointercancel="releasePointer" @lostpointercapture="releasePointer"
            >
              <kbd>{{ DIRECTIONS[direction].key }}</kbd>
              <Icon :name="directionIcons[direction]" :size="20" />
              <span>{{ DIRECTIONS[direction].label }}</span>
            </button>
          </div>
          <button type="button" class="stop-button" :disabled="!canStop" @click="stop('操作员触发停止')">
            <Icon name="stop" :size="18" /><span>停止并解除</span><kbd>Space</kbd>
          </button>

          <p v-if="fault" class="control-note warning" role="alert"><Icon name="info" :size="15" />{{ fault }}</p>
          <p v-else-if="!available" class="control-note"><Icon name="info" :size="15" />{{ unavailableLabel }}</p>
          <p v-else class="control-note"><Icon name="shield" :size="15" />按住行进，松开后停止并解除控制。</p>
          <div class="control-limits">
            <span>控制限速</span>
            <strong>{{ available ? fixed(limits.linear) : '—' }} <small>m/s</small></strong>
            <strong>{{ available ? fixed(limits.angular) : '—' }} <small>rad/s</small></strong>
          </div>
        </div>
      </section>

      <MapView :map="map" :pose="currentPose" :status="mapStatus" :connection="connection" />

      <aside class="telemetry-stack" aria-label="机器人遥测">
        <section class="panel status-panel" aria-labelledby="status-heading">
          <header class="panel-header">
            <div class="panel-title">
              <span class="panel-icon"><Icon name="pulse" /></span>
              <h2 id="status-heading">机器人状态</h2>
            </div>
            <span class="status-chip" :class="robotOnline ? 'good' : 'muted'"><i class="state-dot"></i>{{ robotOnline ? '在线' : '离线' }}</span>
          </header>
          <div class="status-body">
            <div class="battery-heading">
              <span><Icon name="battery" :size="17" /> 电池电量</span>
              <strong>{{ battery === null ? '—' : Math.round(battery) }}<small>%</small></strong>
            </div>
            <div class="battery-bar" role="progressbar" aria-label="电池电量" aria-valuemin="0" aria-valuemax="100" :aria-valuenow="battery ?? undefined">
              <span :style="{ width: (battery ?? 0) + '%' }" :class="{ low: battery !== null && battery <= 20 }"></span>
            </div>
            <dl class="status-details">
              <div><dt>运动模式</dt><dd>{{ robotMode }}</dd></div>
              <div><dt>遥测延迟</dt><dd class="mono">{{ age(robot?.telemetry_stamp_ms, robot?.telemetry_age_ms) }}</dd></div>
            </dl>
            <div class="velocity-cards">
              <div><span>线速度</span><strong>{{ fixed(robotOnline ? robot?.linear_mps : null) }}</strong><small>m/s</small></div>
              <div><span>角速度</span><strong>{{ fixed(robotOnline ? robot?.angular_radps : null) }}</strong><small>rad/s</small></div>
            </div>
          </div>
        </section>

        <section class="panel pose-panel" aria-labelledby="pose-heading">
          <header class="panel-header">
            <div class="panel-title">
              <span class="panel-icon"><Icon name="compass" /></span>
              <h2 id="pose-heading">实时位姿</h2>
            </div>
            <span class="status-chip" :class="currentPose ? 'good' : 'muted'"><i class="state-dot"></i>{{ currentPose ? '有效' : '无数据' }}</span>
          </header>
          <div class="pose-body">
            <dl class="pose-values">
              <div><dt><span>X</span>位置</dt><dd>{{ fixed(currentPose?.x_m, 3) }} <small>m</small></dd></div>
              <div><dt><span>Y</span>位置</dt><dd>{{ fixed(currentPose?.y_m, 3) }} <small>m</small></dd></div>
              <div><dt><span class="theta">θ</span>朝向</dt><dd>{{ fixed(heading, 1) }} <small>°</small></dd></div>
            </dl>
            <div class="pose-caption">
              <span>坐标系 <b class="mono">{{ state?.map?.frame_id || '—' }}</b></span>
              <span>{{ age(currentPose?.stamp_ms, currentPose?.age_ms) }}</span>
            </div>
          </div>
        </section>
      </aside>
    </div>

    <section class="panel log-panel" aria-labelledby="log-heading">
      <header class="panel-header">
        <div class="panel-title">
          <span class="panel-icon"><Icon name="list" /></span>
          <h2 id="log-heading">事件记录</h2>
        </div>
        <button type="button" class="quiet-button" :disabled="!events.length" @click="clearEvents"><Icon name="clear" :size="15" />清空</button>
      </header>
      <div class="event-list" role="log" aria-label="网关与操作事件">
        <div v-if="!events.length" class="event-empty">暂无事件</div>
        <div v-for="event in events" :key="event.id" class="event-row">
          <time class="mono">{{ formatClock(event.time) }}</time>
          <span class="event-level" :class="event.level">{{ eventLabels[event.level] }}</span>
          <span>{{ event.message }}</span>
        </div>
      </div>
    </section>
  </section>
</template>
