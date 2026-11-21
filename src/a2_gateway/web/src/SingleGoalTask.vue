<script setup>
import { computed, onBeforeUnmount, onMounted, reactive, ref } from 'vue'
import { describeError, request } from './useGateway.js'

const props = defineProps({ connection: { type: String, required: true } })
const endpoint = '/api/navigation/goal'
const phases = {
  idle: '等待目标', sending: '等待 Nav2 接收', active: '正在导航', canceling: '等待取消结果',
  succeeded: 'Nav2 已到达目标', canceled: 'Nav2 已取消目标', failed: '导航失败',
  uncertain: '执行结果待核实',
}
const terminal = new Set(['idle', 'succeeded', 'canceled', 'failed'])
const key = 'a2.station.single-goal.pending.v1'
const hashPattern = /^[0-9a-f]{64}$/
const messages = {
  pc2_unavailable: 'PC2 连接中断', pc2_invalid_response: 'PC2 返回了无效的导航状态',
  map_mismatch: '地图版本不一致，请重新核对目标', goal_busy: '另一个目标正在执行',
  goal_outside_map: '目标超出当前导航地图', goal_cell_not_free: '目标落在障碍物或未知区域',
  goal_id_conflict: '目标编号与先前提交的内容冲突', nav2_unavailable: 'Nav2 导航服务不可用',
  goal_not_current: '此编号已不是当前目标', invalid_goal: '目标格式或数值无效',
  goal_controller_unavailable: '机载目标控制器不可用', nav2_rejected: 'Nav2 拒绝了目标',
  nav2_aborted: 'Nav2 中止了导航', restart_outcome_unknown: '进程重启后，先前目标的结果待核实',
  nav2_response_timeout: '等待 Nav2 回执超时', cancel_outcome_unknown: '取消结果待核实',
  platform_offline: '机身状态离线或已过期', navigation_grid_unavailable: '等待 PC2 接收导航栅格',
  navigation_grid_mismatch: '运行中的导航栅格与配置地图不一致', body_pose_unavailable: '缺少同图、有效的机身定位或 TF',
  odin1_waiting: '等待 ODIN1 重定位位姿', odin1_stale: 'ODIN1 位姿已过期',
  odin1_invalid: 'ODIN1 位姿记录无效', odin1_map_mismatch: 'ODIN1 重定位地图版本不一致',
  navigation_readiness_unknown: 'PC2 无法确认导航就绪条件',
}
function explain(error) {
  return messages[error.reason] ?? error.message
}

// 仅保存结果尚未确定的下发请求。刷新或掉线时，用原 ID 和原参数核对，绝不自动补发。
function previousRequest() {
  try {
    const item = JSON.parse(sessionStorage.getItem(key))
    const body = item?.body
    if (typeof body?.goal_id === 'string' && hashPattern.test(body.map_sha256) &&
        body.frame_id === 'map' && [body.x_m, body.y_m, body.yaw_rad].every(Number.isFinite) &&
        (item.previousId === null || typeof item.previousId === 'string')) return item
  } catch { /* 禁用浏览器存储时仍可通过 PC2 的当前状态恢复界面。 */ }
  return null
}
const pending = ref(previousRequest())
function remember(ticket) {
  pending.value = ticket
  try {
    if (ticket) sessionStorage.setItem(key, JSON.stringify(ticket))
    else sessionStorage.removeItem(key)
  } catch { /* 当前页面仍在内存中持有原请求。 */ }
}

const form = reactive({ x: '', y: '', heading: '' })
const goal = ref(null)
const readiness = ref(null)
const busy = ref(false)
const readError = ref('')
const actionError = ref('')
const online = computed(() => props.connection === 'connected' && goal.value && !readError.value)
// 当前目标若已被换成第三个 ID，GET 无法证明旧请求的终态。页面只提示核查，不重发旧目标。
const replaced = computed(() => pending.value && goal.value &&
  goal.value.goal_id !== pending.value.body.goal_id && goal.value.goal_id !== pending.value.previousId)
const canPrepare = computed(() => online.value && !busy.value && !replaced.value && terminal.has(goal.value.phase))
const canSubmit = computed(() => canPrepare.value && (pending.value || readiness.value?.ready !== false))
const canCancel = computed(() => !busy.value && goal.value?.goal_id &&
  ['sending','active','uncertain'].includes(goal.value.phase))
const coordinates = computed(() => {
  if (!goal.value?.goal_id) return ''
  const { x_m, y_m, yaw_rad } = goal.value
  return `(${x_m.toFixed(3)}, ${y_m.toFixed(3)}) m · ${(yaw_rad * 180 / Math.PI).toFixed(1)}°`
})
function clearForm() {
  Object.assign(form, { x: '', y: '', heading: '' })
}

let alive = true
let version = 0
let timer
function update(response, fromStatus = false) {
  const next = response?.goal
  if (response?.ok !== true || !next || !Object.hasOwn(phases, next.phase) ||
      next.frame_id !== 'map' || !hashPattern.test(next.map_sha256) ||
      (next.phase === 'idle' ? next.goal_id !== null : typeof next.goal_id !== 'string' ||
        ![next.x_m, next.y_m, next.yaw_rad].every(Number.isFinite))) {
    throw new Error('PC2 导航状态格式异常')
  }
  // 如果操作员填写坐标后换了地图，应在新地图重新填写，防止沿用旧参考系。
  if (goal.value && next.map_sha256 !== goal.value.map_sha256) clearForm()
  // 旧接口可能没有诊断字段。POST 的业务回执也可不带该字段，此时沿用最近一次查询的诊断。
  if (Object.hasOwn(response, 'readiness')) {
    const value = response.readiness
    readiness.value = typeof value?.ready === 'boolean' ? value : { ready: false, reason: 'navigation_readiness_unknown' }
  } else if (fromStatus) {
    readiness.value = null
  }
  goal.value = next
  readError.value = ''
  if (pending.value && next.goal_id === pending.value.body.goal_id) {
    remember(null)
    clearForm()
    actionError.value = ''
  }
}
async function poll() {
  clearTimeout(timer)
  if (!alive || busy.value) return
  const current = ++version
  try {
    const response = await request(endpoint)
    if (alive && current === version) update(response, true)
  } catch (error) {
    if (alive && current === version) readError.value = explain(error)
  } finally {
    if (alive && current === version) timer = setTimeout(poll, 500)
  }
}

// getRandomValues 在本机回环和局域网页面均可用，按 RFC 4122 生成 v4 UUID。
function newGoalId() {
  const bytes = crypto.getRandomValues(new Uint8Array(16))
  bytes[6] = (bytes[6] & 15) | 64
  bytes[8] = (bytes[8] & 63) | 128
  const hex = Array.from(bytes, value => value.toString(16).padStart(2, '0')).join('')
  return [hex.slice(0,8), hex.slice(8,12), hex.slice(12,16), hex.slice(16,20), hex.slice(20)].join('-')
}
async function command(cancel = false) {
  if (busy.value || (cancel ? !canCancel.value : !canSubmit.value)) return
  const observed = goal.value
  let body = pending.value?.body
  if (!cancel && !body) {
    if (![form.x, form.y, form.heading].every(Number.isFinite) || Math.abs(form.heading) > 180) {
      actionError.value = '请填写 X、Y 和 −180° 至 180° 的目标朝向。'
      return
    }
    body = { goal_id: newGoalId(), frame_id: 'map', map_sha256: observed.map_sha256,
      x_m: form.x, y_m: form.y, yaw_rad: form.heading * Math.PI / 180 }
  }
  busy.value = true
  actionError.value = ''
  ++version
  clearTimeout(timer)
  let posted = false
  try {
    if (cancel) {
      // 取消始终指向点击时显示的 ID。即便状态查询暂时失败，也让机载端核对该取消请求。
      posted = true
      const response = await request(endpoint + '/cancel', { body: { goal_id: observed.goal_id } })
      if (!alive) return
      if (response.goal?.goal_id !== observed.goal_id) throw new Error('PC2 的取消回执编号不一致')
      update(response)
      return
    }
    // 下发前再读一次服务端状态，处理其他页面已完成或更换目标的情况。
    const fresh = await request(endpoint)
    if (!alive) return
    update(fresh, true)
    const latest = goal.value
    if (latest.goal_id === body.goal_id) return
    // 两次轮询之间也可能丢失定位：只允许核对原 ID，新目标须使用本次读取的就绪结果。
    if (!pending.value && readiness.value?.ready === false) {
      throw new Error(messages[readiness.value.reason] ?? describeError(readiness.value.reason))
    }
    if (replaced.value || !terminal.has(latest.phase) || observed.goal_id !== latest.goal_id) {
      throw new Error('PC2 的当前目标已变化，请核对状态。')
    }
    if (body.map_sha256 !== latest.map_sha256) {
      if (pending.value) remember(null)
      clearForm()
      throw new Error('地图已更换，请按当前地图重新填写目标。')
    }
    remember({ body, previousId: latest.goal_id })
    posted = true
    const response = await request(endpoint, { body })
    if (!alive) return
    if (response.goal?.goal_id !== body.goal_id) throw new Error('PC2 的目标回执编号不一致')
    update(response)
  } catch (error) {
    if (!alive) return
    // 明确的客户请求错误允许更正参数。超时或连接中断时，原请求及其 ID 必须保留。
    if (!cancel && posted && error.status >= 400 && error.status < 500 && error.status !== 408) {
      remember(null)
    }
    actionError.value = explain(error)
  } finally {
    busy.value = false
    if (alive) void poll()
  }
}
onMounted(poll)
onBeforeUnmount(() => { alive = false; ++version; clearTimeout(timer) })
</script>

<template>
  <section class="portal-reserved single-goal">
    <header class="portal-page-heading"><h1>PC2 单点导航</h1></header>
    <div class="goal-layout">
      <form class="goal-card" @submit.prevent="command()">
        <h2>地图目标</h2>
        <p v-if="goal" class="map-version">map · SHA-256 <code>{{ goal.map_sha256 }}</code></p>
        <fieldset :disabled="!!pending || !canPrepare">
          <label>目标 X（米）<input v-model.number="form.x" type="number" step="any" required /></label>
          <label>目标 Y（米）<input v-model.number="form.y" type="number" step="any" required /></label>
          <label>目标朝向（度）<input v-model.number="form.heading" type="number" step="any" min="-180" max="180" required /></label>
        </fieldset>
        <p v-if="pending">等待核对请求 {{ pending.body.goal_id }}<br />
          目标 ({{ pending.body.x_m }}, {{ pending.body.y_m }}) m · {{ (pending.body.yaw_rad * 180 / Math.PI).toFixed(1) }}°</p>
        <p v-if="replaced">当前目标已更换，旧请求结果需核对 PC2 记录。</p>
        <button v-if="replaced" type="button" @click="remember(null); clearForm(); actionError = ''">已核对，清除本页待核对记录</button>
        <button class="primary" type="submit" :disabled="!canSubmit">
          {{ pending ? '重试原目标' : '下发目标' }}
        </button>
        <p v-if="actionError" role="alert" class="error">{{ actionError }}</p>
      </form>
      <section class="goal-card">
        <h2>导航反馈</h2>
        <p class="phase" role="status">{{ online ? phases[goal.phase] : '等待 PC2 导航状态' }}</p>
        <p v-if="online && readiness" role="status" :class="{ error: !readiness.ready }">
          {{ readiness.ready ? 'PC2 导航已就绪' : 'PC2 导航未就绪' }}
          <span v-if="!readiness.ready"> · {{ messages[readiness.reason] ?? describeError(readiness.reason) }}</span>
        </p>
        <p v-if="readError" class="error">{{ readError }}</p>
        <dl v-if="goal?.goal_id">
          <dt>目标编号</dt><dd><code>{{ goal.goal_id }}</code></dd>
          <dt>目标位姿</dt><dd>{{ coordinates }}</dd>
          <template v-if="online && goal.phase === 'active' && Number.isFinite(goal.distance_remaining_m)">
            <dt>剩余距离</dt><dd>{{ goal.distance_remaining_m.toFixed(2) }} m</dd>
          </template>
          <template v-if="goal.reason"><dt>状态说明</dt><dd>{{ messages[goal.reason] ?? describeError(goal.reason) }}</dd></template>
        </dl>
        <p v-if="goal && ['succeeded','canceled'].includes(goal.phase)">机身实际停稳状态待独立反馈确认。</p>
        <button v-if="goal?.goal_id" type="button" :disabled="!canCancel" @click="command(true)">取消目标</button>
      </section>
    </div>
  </section>
</template>

<style scoped>
.goal-layout { display:grid; grid-template-columns:minmax(0,1.3fr) minmax(240px,1fr); gap:16px }
.goal-card { min-width:0; padding:clamp(16px,2vw,26px); border:1px solid var(--portal-line); border-radius:10px; background:#fff }
.goal-card h2 { margin:0 0 16px; font-size:17px }
.goal-card fieldset { padding:0; margin:16px 0; border:0; display:grid; grid-template-columns:repeat(3,minmax(0,1fr)); gap:12px }
.goal-card label, .map-version, .goal-card dt { color:var(--portal-muted); font-size:13px }
.goal-card code { overflow-wrap:anywhere }
.goal-card input { margin-top:5px; padding:8px; width:100%; border:1px solid var(--portal-line); border-radius:5px; font:inherit }
.goal-card button { border:1px solid var(--portal-line); border-radius:5px; padding:8px 15px; background:#fff; color:var(--ink); font-weight:600 }
.goal-card .primary { color:white; background:var(--action) }
.goal-card button:disabled { opacity:.55 }
.goal-card p { margin:10px 0 }
.goal-card dl { margin:12px 0; display:grid; grid-template-columns:auto 1fr; gap:8px 14px }
.goal-card dd { margin:0 }
.phase { color:var(--action); font-weight:650 }
.error { color:#923626 }
@media(max-width:900px){ .goal-layout { grid-template-columns:1fr } }
@media(max-width:480px){ .goal-card fieldset { grid-template-columns:1fr } }
</style>
