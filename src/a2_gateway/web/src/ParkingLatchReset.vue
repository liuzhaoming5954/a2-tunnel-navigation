<script setup>
import { computed, onBeforeUnmount, ref, watch } from 'vue'
import { request } from './useGateway.js'

const props = defineProps({
  connected: { type: Boolean, required: true },
  report: { type: Object, required: true },
  availability: { type: Object, default: null },
})

const dialog = ref(null)
const acknowledged = ref(false)
const phase = ref('idle')
const notice = ref('')
let statusTimer
let disposed = false

const busy = computed(() => phase.value !== 'idle')
const blockedReason = computed(() => {
  if (!props.connected) return 'PC2 未连接'
  if (props.report.guard !== 'running') return '停车守护状态不可用'
  if (props.report.latched === false) return '停车未锁存'
  if (props.report.latched !== true) return '停车锁存未知'
  if (props.availability?.available !== true) return resetReason(props.availability?.reason)
  return ''
})
const canReset = computed(() => !busy.value && !blockedReason.value)
const latchCleared = computed(() => props.connected && props.report.guard === 'running' && props.report.latched === false)

function resetReason(reason) {
  return ({
    pc2_unavailable: 'PC2 未连接',
    stop_guard_not_ready: '停车守护尚未就绪',
    stop_guard_not_running: '停车守护尚未就绪',
    stop_guard_stale: '停车守护数据已过期',
    stop_guard_unknown: '停车守护状态未知',
    stop_latch_unknown: '停车锁存未知',
    stop_not_latched: '停车未锁存',
    stop_unconfirmed: '停车守护尚未确认停稳',
    stop_reset_busy: '另一条复位请求正在处理',
    stop_sender_active: '发令服务尚未退出',
    stop_handset_active: '原厂手柄仍有按键或摇杆输入',
    stop_handset_changed: '复位期间原厂手柄状态发生变化',
    stop_handset_unavailable: '无法确认原厂手柄已回中',
    stop_not_stationary: '机身尚未确认停稳',
    stop_reset_rolled_back: '机载复位未完成，停车锁存已恢复',
    stop_reset_recovery_pending: '机载停车守护正在恢复，请核对当前停车状态',
    standstill_required: '机身尚未确认停稳',
    robot_offline: 'A2 状态离线',
    manual_auth_failed: 'PC2 控制隧道鉴权失败',
    manual_invalid_response: 'PC2 控制隧道返回的状态无效',
    manual_link_offline: 'PC2 控制隧道未连接',
    manual_robot_mismatch: '控制端与遥测端的机器人标识不一致',
    session_active: '已有控制会话',
    session_busy: '已有其他控制会话',
  })[reason] ?? 'PC2 复位服务当前不可用'
}

function openConfirmation() {
  if (!canReset.value) return
  acknowledged.value = false
  dialog.value?.showModal()
}

function applyObservedReset() {
  clearTimeout(statusTimer)
  phase.value = 'idle'
  notice.value = '停车锁存已复位'
}

// POST 受理与读到机载新状态是两个观测。按独立的新鲜遥测判断复位结果。
function waitForStatus(message) {
  if (latchCleared.value) {
    applyObservedReset()
    return
  }
  phase.value = 'awaiting'
  notice.value = message
  statusTimer = setTimeout(() => {
    phase.value = 'idle'
    notice.value = '复位结果未确认，请核对当前停车状态'
  }, 6000)
}

watch(latchCleared, cleared => {
  if (cleared && phase.value === 'awaiting') applyObservedReset()
  if (!cleared && notice.value === '停车锁存已复位') notice.value = ''
})
// 确认窗口打开后如就绪条件变化，须重新核实再确认。
watch(canReset, enabled => {
  if (!enabled) acknowledged.value = false
})

async function submitReset() {
  if (!canReset.value || !acknowledged.value) return
  phase.value = 'sending'
  notice.value = '正在请求复位'
  dialog.value?.close()
  try {
    // 站端转发复位可等待 24 秒，页端为站端的前置核对以及两跳传输预留时间。
    await request('/api/control/stop-reset', { body: { confirmed: true }, timeout: 30000 })
    if (!disposed) waitForStatus('复位已受理，等待 PC2 确认')
  } catch (error) {
    if (disposed) return
    // 传输中断或服务端错误时请求可能已经到达，继续看遥测，页面不自动重发。
    if (!error.reason || error.status >= 500) {
      waitForStatus('复位响应未确认，等待 PC2 状态')
      return
    }
    phase.value = 'idle'
    notice.value = '复位失败：' + resetReason(error.reason)
  }
}

onBeforeUnmount(() => {
  disposed = true
  clearTimeout(statusTimer)
})
</script>

<template>
  <div class="ic-parking-reset">
    <button class="ic-reset-button" type="button" aria-haspopup="dialog" :disabled="!canReset" :title="blockedReason" @click="openConfirmation">
      复位停车锁存
    </button>
    <span class="ic-reset-notice" role="status">{{ notice || (report.latched === true ? blockedReason : '') }}</span>
    <dialog ref="dialog" class="ic-dialog ic-reset-dialog" aria-labelledby="ic-reset-title" aria-describedby="ic-reset-conditions">
      <article>
        <header><h2 id="ic-reset-title">复位停车锁存</h2></header>
        <ul id="ic-reset-conditions">
          <li>A2 已完全停稳</li>
          <li>已排除触发停车的原因</li>
          <li>原厂手柄 A 键已松开，摇杆已回中</li>
        </ul>
        <label><input v-model="acknowledged" type="checkbox" :disabled="!canReset"> 我已在现场核实上述条件</label>
        <p v-if="blockedReason" role="status">{{ blockedReason }}</p>
        <footer>
          <button type="button" class="ic-secondary" autofocus @click="dialog.close()">取消</button>
          <button type="button" class="ic-action" :disabled="!canReset || !acknowledged" @click="submitReset">确认复位</button>
        </footer>
      </article>
    </dialog>
  </div>
</template>

<style scoped>
.ic-parking-reset { display: flex; flex-wrap: wrap; align-items: center; gap: 4px 8px; margin-top: 7px; }
.ic-reset-button { min-height: 26px; padding: 3px 8px; border: 1px solid var(--ic-line); background: var(--ic-panel); color: var(--ic-focus); font-size: 12px; white-space: nowrap; }
.ic-reset-button:enabled:hover { border-color: var(--ic-focus); }
.ic-reset-button:disabled, .ic-reset-dialog button:disabled { color: var(--ic-muted); background: var(--ic-surface); border-color: var(--ic-line); cursor: default; }
.ic-reset-notice { font-size: 12px; color: var(--ic-muted); }
.ic-reset-dialog ul { list-style: disc; padding-left: 20px; font-size: 13px; line-height: 1.8; }
.ic-reset-dialog label { margin: 8px 0; }
</style>
