<script setup>
import { computed, onBeforeUnmount, onMounted, ref } from 'vue'
import Icon from './Icon.vue'
import { describeError, request } from './useGateway.js'

const props = defineProps({ state: { type: Object, required: true }, connection: { type: String, required: true } })
const fields = [
  { key: 'x_m', label: 'X 坐标（米）' },
  { key: 'y_m', label: 'Y 坐标（米）' },
  { key: 'dwell_s', label: '停留时间（秒）' },
]
const defaults = [
  { sequence: 1, x_m: -4.15, y_m: -1.1, dwell_s: 2 },
  { sequence: 2, x_m: -3.45, y_m: -1.1, dwell_s: 4 },
]
const phases = { idle: '等待下发', moving: '前往巡检点', settling: '已到达，等待停稳', dwelling: '停留中',
  waiting: '等待下发点 2', completed: '巡检完成', failed: '巡检中止' }
const eventLabels = { accepted: '已接收', departed: '出发', arrived: '已到达', stopped: '已停稳 · 开始停留',
  dwell_finished: '停留结束', completed: '巡检完成', failed: '巡检中止' }
const reasons = { invalid_patrol_point: '请检查点位坐标和停留时间', invalid_patrol_target: '目标点超出可通行区域',
  patrol_out_of_order: '请先下发点 1', patrol_point_conflict: '该点已按另一组参数接收，请核对回执',
  patrol_busy: '已有巡检任务执行中', patrol_finished: '该任务已结束',
  simulator_closed: '模拟器已关闭', session_busy: '维护遥控占用中，请先结束遥控' }

const storageKey = 'a2.simulator.pending-patrol.v1'
function loadPending() {
  // 只缓存本页尚未收齐回执的下发意图。执行状态始终取自模拟器。
  try {
    const saved = JSON.parse(sessionStorage.getItem(storageKey))
    if (typeof saved?.mission_id === 'string' && saved.points?.length === 2 &&
      saved.points.every((p, i) => p?.sequence === i + 1 && fields.every(f => Number.isFinite(p[f.key])))) return saved
  } catch { /* 浏览器可禁用会话存储，仍可从模拟器查询已接收的点。 */ }
  return null
}
const pending = ref(loadPending())
const draft = ref((pending.value?.points ?? defaults).map(point => ({ ...point })))
const patrol = ref(null)
const submitting = ref(false)
const readError = ref('')
const submitError = ref('')
const executing = computed(() => patrol.value && !['idle', 'completed', 'failed'].includes(patrol.value.phase))
const accepted = computed(() => executing.value ? patrol.value.points.length : 0)
const activeTarget = computed(() => patrol.value?.points.find(p => p.sequence === patrol.value.active_sequence))
const live = computed(() => props.connection === 'connected' && !readError.value)
const distance = computed(() => {
  const pose = props.state.pose
  const target = activeTarget.value
  if (!live.value || !target || !pose) return null
  return Math.hypot(pose.x_m - target.x_m, pose.y_m - target.y_m).toFixed(2)
})
const finishedCount = computed(() => patrol.value?.events.filter(e => e.event === 'dwell_finished').length ?? 0)
const buttonLabel = computed(() => submitting.value ? '正在下发…' : accepted.value === 1 ? '继续下发点 2'
  : accepted.value === 2 ? '两点已接收' : pending.value ? '重新下发' : '下发两点巡检')
let alive = true
let timer
let readVersion = 0

function savePending(value) {
  pending.value = value
  try {
    if (value) sessionStorage.setItem(storageKey, JSON.stringify(value))
    else sessionStorage.removeItem(storageKey)
  } catch { /* 容量或隐私设置不影响当次会话中的请求重试。 */ }
}

function applySnapshot(result) {
  if (!Object.hasOwn(phases, result?.phase) || !Array.isArray(result.points) || !Array.isArray(result.events)) {
    throw new Error('模拟器返回了无效任务状态')
  }
  // 已受理的点以服务器值为准。任务结束后允许编辑下一轮参数，不让后台轮询覆盖输入。
  const newTask = !patrol.value || patrol.value.mission_id !== result.mission_id
  const active = !['completed', 'failed', 'idle'].includes(result.phase)
  if (active || (newTask && (!pending.value || result.mission_id === pending.value.mission_id))) {
    for (const point of result.points) {
      if (point.sequence === 1 || point.sequence === 2) draft.value[point.sequence - 1] = { ...point }
    }
  }
  patrol.value = result
  if (result.mission_id === pending.value?.mission_id &&
      (result.points.length === 2 || ['completed', 'failed'].includes(result.phase))) savePending(null)
}

async function poll() {
  clearTimeout(timer)
  if (!alive || submitting.value) return
  const version = ++readVersion
  try {
    const result = await request('/api/patrol')
    if (!alive || version !== readVersion) return
    applySnapshot(result)
    readError.value = ''
  } catch (error) {
    if (alive && version === readVersion) readError.value = error.message
  } finally {
    if (alive && version === readVersion) timer = setTimeout(poll, 250)
  }
}

async function submit() {
  if (!alive || !live.value || submitting.value || accepted.value === 2) return
  if (!draft.value.every(p => fields.every(f => Number.isFinite(p[f.key])) && p.dwell_s >= 0)) {
    submitError.value = '请填写两个点的有效坐标及停留秒数。'
    return
  }
  submitting.value = true
  submitError.value = ''
  ++readVersion
  clearTimeout(timer)
  const points = draft.value.map(p => ({ ...p }))
  const expectedMission = (executing.value && patrol.value.mission_id) || pending.value?.mission_id
  try {
    // 先查询已有回执再发送，涵盖 POST 已成功但响应丢失或网页在两次请求间刷新的情况。
    const known = await request('/api/patrol')
    if (!alive) return
    applySnapshot(known)
    // 同一模拟器可在多个标签页查看。重试期间若原任务已收尾，不把旧点击解释成新任务。
    if (expectedMission === known.mission_id && ['failed', 'completed'].includes(known.phase)) return
    if (executing.value && expectedMission !== known.mission_id) {
      throw new Error('模拟器已有其他巡检任务，请核对当前任务后操作。')
    }
    if (executing.value && known.points.length === 2) return
    const missionId = expectedMission ||
      'sim-' + Array.from(crypto.getRandomValues(new Uint32Array(3)), n => n.toString(16)).join('-')
    savePending({ mission_id: missionId, points })
    for (const point of points) {
      if (!alive || props.connection !== 'connected') throw new Error('网关连接中断')
      if (known.mission_id === missionId && known.points.some(p => p.sequence === point.sequence)) continue
      // 每次只提交一个点，必须取得前一点的明确受理回执，才提交下一点。
      const response = await request('/api/patrol/point', { body: { mission_id: missionId, ...point } })
      if (!alive) return
      if (response.patrol?.mission_id !== missionId ||
          !response.patrol.points?.some(p => p.sequence === point.sequence)) throw new Error('未收到对应点位的回执')
      applySnapshot(response.patrol)
      if (response.patrol.phase === 'failed') return
    }
  } catch (error) {
    if (alive) submitError.value = reasons[error.reason] ?? error.message
  } finally {
    submitting.value = false
    if (alive) void poll()
  }
}

onMounted(poll)
onBeforeUnmount(() => { alive = false; ++readVersion; clearTimeout(timer) })
</script>

<template>
  <section class="patrol-page portal-reserved">
    <header class="portal-page-heading"><h1>两点模拟巡检</h1></header>
    <div class="patrol-layout">
      <form class="patrol-card" @submit.prevent="submit">
        <h2><Icon name="compass" />巡检点位</h2>
        <fieldset v-for="point in draft" :key="point.sequence"
          :disabled="submitting || accepted >= point.sequence">
          <legend>点 {{ point.sequence }}</legend>
          <div class="patrol-fields">
            <label v-for="field in fields" :key="field.key">{{ field.label }}
              <input type="number" step="any" required :min="field.key === 'dwell_s' ? 0 : undefined"
                :aria-label="'点 ' + point.sequence + ' ' + field.label" v-model.number="point[field.key]" />
            </label>
          </div>
        </fieldset>
        <button class="patrol-submit" type="submit" :disabled="!live || !patrol || submitting || accepted === 2">
          <Icon name="next" :size="16" />{{ buttonLabel }}
        </button>
        <p v-if="submitError" class="patrol-error" role="alert">{{ submitError }}</p>
      </form>

      <section class="patrol-card patrol-progress">
        <h2><Icon name="pulse" />执行进度</h2>
        <p class="patrol-current" role="status">{{ !live ? '等待网关恢复' : (phases[patrol?.phase] ?? '读取任务中') }}</p>
        <p v-if="readError" class="patrol-error">{{ readError }}</p>
        <p v-if="patrol?.phase === 'failed'" class="patrol-error">{{ reasons[patrol.reason] ?? describeError(patrol.reason) }}</p>
        <dl>
          <div><dt>已完成</dt><dd>{{ finishedCount }} / 2 点</dd></div>
          <div v-if="activeTarget"><dt>当前目标</dt><dd>点 {{ activeTarget.sequence }} · ({{ activeTarget.x_m }}, {{ activeTarget.y_m }}) m</dd></div>
          <div><dt>模拟机身 X / Y</dt><dd>{{ live && state.pose ? state.pose.x_m.toFixed(2) + ' / ' + state.pose.y_m.toFixed(2) + ' m' : '—' }}</dd></div>
          <div v-if="distance !== null"><dt>距当前目标</dt><dd>{{ distance }} m</dd></div>
          <div v-if="live && patrol?.phase === 'dwelling'"><dt>剩余停留</dt><dd>{{ patrol.remaining_s.toFixed(1) }} s</dd></div>
        </dl>
      </section>
    </div>

    <section class="patrol-card patrol-records">
      <h2><Icon name="list" />执行记录</h2>
      <table v-if="patrol?.events.length">
        <thead><tr><th>任务用时（秒）</th><th>巡检点</th><th>事件</th></tr></thead>
        <tbody><tr v-for="(event, index) in patrol.events" :key="index">
          <td>{{ event.time_s.toFixed(2) }}</td><td>点 {{ event.sequence }}</td><td>{{ eventLabels[event.event] ?? event.event }}</td>
        </tr></tbody>
      </table>
      <p v-else>尚无执行记录。</p>
    </section>
  </section>
</template>

<style scoped>
.patrol-page { max-width: 1260px; }
.patrol-layout { display: grid; grid-template-columns: minmax(0, 1.3fr) minmax(230px, .85fr); gap: 16px; }
.patrol-card { min-width: 0; padding: clamp(16px, 2vw, 26px); border: 1px solid var(--portal-line); border-radius: 10px; background: #fff; }
.patrol-card h2 { display: flex; align-items: center; gap: 9px; margin-bottom: 18px; font-size: 17px; }
.patrol-card h2 svg { color: var(--action); }
.patrol-card fieldset { margin: 0 0 16px; padding: 0; border: 0; min-width: 0; }
.patrol-card legend { margin-bottom: 6px; padding: 0; font-weight: 600; }
.patrol-fields { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 12px; }
.patrol-fields label { color: var(--portal-muted); font-size: 12px; }
.patrol-fields input { display: block; width: 100%; margin-top: 5px; padding: 8px; border: 1px solid #aaaebf; border-radius: 4px; color: var(--ink); font: 15px var(--mono); }
.patrol-fields input:focus { outline: 2px solid var(--action); outline-offset: 1px; }
.patrol-submit { display: inline-flex; align-items: center; gap: 8px; padding: 9px 15px; border: 0; border-radius: 5px; color: white; background: var(--action); font-weight: 600; }
.patrol-submit:disabled { background: #666a7c; }
.patrol-current { color: var(--action); font-size: 19px; font-weight: 600; }
.patrol-progress dl { margin-top: 12px; }
.patrol-progress dl div { display: flex; align-items: baseline; justify-content: space-between; gap: 10px; padding: 8px 0; border-bottom: 1px solid var(--portal-line); }
.patrol-progress dt, .patrol-records p { color: var(--portal-muted); }
.patrol-progress dd, .patrol-records td:first-child { font-variant-numeric: tabular-nums; }
.patrol-records { margin-top: 16px; }
.patrol-records table { width: 100%; border-collapse: collapse; text-align: left; }
.patrol-records th, .patrol-records td { padding: 7px 4px; border-bottom: 1px solid var(--portal-line); }
.patrol-records th { color: var(--portal-muted); font-weight: 500; }
.patrol-error { margin-top: 10px; color: #923626; }
@media (max-width: 940px) { .patrol-layout { grid-template-columns: 1fr; } }
@media (max-width: 500px) { .patrol-fields { gap: 6px; } .patrol-card { padding: 14px; } }
</style>
