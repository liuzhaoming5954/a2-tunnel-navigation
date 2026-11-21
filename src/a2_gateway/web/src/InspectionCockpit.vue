<script setup>
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import Icon from './Icon.vue'
import InspectionCockpitHeader from './InspectionCockpitHeader.vue'
import MapView from './MapView.vue'
import OdinPointCloud from './OdinPointCloud.vue'
import TunnelScene from './TunnelScene.vue'
import { sampleAlarms, scenePoints, shiftTasks } from './cockpitData.js'

const props = defineProps({
  state: { type: Object, default: null },
  map: { type: Object, default: null },
  mapStatus: { type: String, default: 'idle' },
  connection: { type: String, default: 'connecting' },
})

// 标题采用操作员本机日期。跨日保持更新，避免遥测轮询重复格式化日期。
const today = ref(new Date())
const dateParts = computed(() => {
  const date = today.value
  const parts = [date.getFullYear(), date.getMonth() + 1, date.getDate()]
    .map(value => String(value).padStart(2, '0'))
  return {
    iso: parts.join('-'),
    label: parts.join('.'),
    weekday: date.toLocaleDateString('zh-CN', { weekday: 'long' }),
  }
})
let dateTimer
onMounted(() => { dateTimer = setInterval(() => { today.value = new Date() }, 60000) })
onBeforeUnmount(() => clearInterval(dateTimer))

const dialog = ref(null)
const mapSection = ref(null)
const scene = ref(null)
const detail = ref(null)
// 点云是现场采集的静态地图。遥测模式切换或断线重连时保留操作员选定的视图。
const mapView = ref('odin')
const selectedPoint = ref('')
// 上位机首次进入只展示已接入数据；演示入口可默认显示标注来源的业务样例。
// 本机评审链接可显式打开右栏样例；日常工作站入口仍默认采用接口数据。
const previewExamples = new URLSearchParams(window.location.search).get('examples') === '1'
const chosenPolicy = ref(null)
const sourcePolicy = computed({
  get: () => chosenPolicy.value ?? ((previewExamples || knownMode.value === 'simulator') ? 'demo' : 'gateway'),
  set: policy => { chosenPolicy.value = policy },
})
const knownMode = ref('')
const reviewedIds = ref([])

const connected = computed(() => props.connection === 'connected')
const station = computed(() => props.state?.deployment === 'station')
const online = computed(() => connected.value && props.state?.robot?.online === true)
const robot = computed(() => props.state?.robot || {})
// 网关负责底盘遥测有效性；掉线时不把上一帧电量当作实时数据。
const battery = computed(() => {
  const value = robot.value.battery_percent
  return online.value && Number.isFinite(value) && value >= 0 && value <= 100 ? Math.round(value) : null
})
const source = computed(() => station.value ? 'PC2 只读接口' : ({
  simulator: '网关模拟', robot: '实机网关', robot_readonly: '实机只读',
})[knownMode.value] || '等待网关')
const gridSource = computed(() => station.value ? '上位机本地栅格' : source.value + ' · 二维地图')
const robotStatus = computed(() => {
  if (!connected.value) return props.connection === 'connecting' ? '连接中' : '网关离线'
  if (station.value && props.state.pc2_reason === 'pc2_invalid_response') return 'PC2 接口异常'
  if (station.value && props.state.pc2_connected === false) return 'PC2 未连接'
  if (station.value && props.state.pc2_connected !== true) return 'PC2 状态未知'
  return online.value ? '在线' : '遥测离线'
})
const modeName = computed(() => {
  if (!online.value || !robot.value.mode) return '未知'
  return ({ standing: '站立', walking: '行走', stopped: '停止', idle: '待命' })[robot.value.mode] || String(robot.value.mode)
})
const sampleBusiness = computed(() => sourcePolicy.value === 'demo')
// 源遥测时间与 HTTP 响应时间属于不同观测，页面仅采用遥测自身的时间字段。
const telemetryTime = computed(() => {
  const stamp = robot.value.telemetry_stamp_ms
  if (!Number.isFinite(stamp) || stamp <= 0 || stamp > 8.64e15) return '未知'
  return new Date(stamp).toLocaleString('zh-CN', { hour12: false })
})
const businessSource = computed(() => sampleBusiness.value ? '本地模拟' : '未接入')
// 运行摘要只读网关。右栏选择样例时，不能把演示任务或告警数填入运行摘要。
const summaryRobot = computed(() => {
  if (!online.value) return '等待遥测'
  return modeName.value === '未知' ? '在线' : modeName.value
})
// 视图只切换当前地图组件；网关连接由外层工作区维持。
const mapViews = [
  { id: 'odin', name: 'ODIN1 点云' },
  { id: 'scene', name: '示意场景' },
  { id: 'grid', name: '二维栅格' },
]
const activeAlarms = computed(() => sampleBusiness.value ? sampleAlarms : [])
const taskRows = computed(() => sampleBusiness.value ? shiftTasks : [])
const currentPose = computed(() => {
  const pose = props.state?.pose
  return connected.value && pose?.map_id && pose.map_id === props.map?.id && pose.map_id === props.state?.map?.id
    && [pose.x_m, pose.y_m, pose.yaw_rad].every(Number.isFinite) ? pose : null
})

// 网关返回 mode 之后再判断来源。离线保留最后识别的模式，使实机断线后仍可区分上次接入的实际来源。
watch(() => props.state?.mode, mode => {
  if (!mode || mode === knownMode.value) return
  knownMode.value = mode
  chosenPolicy.value = null
  selectedPoint.value = ''
  if (dialog.value?.open) dialog.value.close()
}, { immediate: true })

// 原生对话框处理焦点、Escape 和键盘操作；所有入口均为只读或仅限本次页面的演示操作。
async function openDetail(type, item = null) {
  detail.value = { type, item }
  await nextTick()
  if (dialog.value && !dialog.value.open) dialog.value.showModal()
}
function closeDetail() {
  dialog.value?.close()
}
function openPoint(point) {
  if (!Object.hasOwn(scenePoints, point)) return
  selectedPoint.value = point
  openDetail('point', point)
}
async function locatePoint(point) {
  if (!Object.hasOwn(scenePoints, point)) return
  selectedPoint.value = point
  mapView.value = 'scene'
  closeDetail()
  await nextTick()
  scene.value?.showInspectionPoint()
  mapSection.value?.scrollIntoView({ block: 'nearest' })
}
function markReviewed(id) {
  if (!reviewedIds.value.includes(id)) reviewedIds.value = [...reviewedIds.value, id]
}
const detailTitle = computed(() => {
  const entry = detail.value
  if (!entry) return ''
  if (entry.type === 'point') return scenePoints[entry.item]?.name
  if (entry.type === 'task' || entry.type === 'alarm') return entry.item.title
  return ({ sources: '数据来源', robot: '机器人本体状态', dock: '电量与充电仓', tasks: '当班任务', alarms: '活动告警', liveTask: '当前任务', liveAlarms: '活动告警' })[entry.type]
})
</script>

<template>
  <div class="ic-root">
    <InspectionCockpitHeader :date-parts="dateParts" @sources="openDetail('sources')" />

    <div class="ic-workbench">
      <section ref="mapSection" class="ic-map" aria-label="巡检地图">
        <header class="ic-map-head">
          <div class="ic-map-title"><Icon name="layers" :size="19" /><h2>巡检地图</h2></div>
          <div class="ic-switch" role="group" aria-label="地图视图">
            <button v-for="view in mapViews" :key="view.id" type="button" :aria-pressed="mapView === view.id" @click="mapView = view.id">
              {{ view.name }}
            </button>
          </div>
        </header>
        <OdinPointCloud v-if="mapView === 'odin'" />
        <div v-else-if="mapView === 'scene'" class="ic-scene">
          <TunnelScene ref="scene" :selected-point="selectedPoint" @select-point="openPoint" />
        </div>
        <div v-else class="ic-grid-map"><MapView :map="map" :pose="currentPose" :status="mapStatus" :connection="connection" /></div>
        <div v-if="mapView !== 'odin'" class="ic-map-caption">
          <span>{{ mapView === 'scene' ? '本地合成场景' : gridSource }}</span>
          <span v-if="mapView === 'scene'">{{ selectedPoint ? selectedPoint + ' · ' + scenePoints[selectedPoint].name : '3 个巡检点位' }}</span>
          <span v-else>{{ connected ? (currentPose ? '定位有效' : '无有效位姿') : (map ? '缓存栅格 · 网关离线' : '网关未连接') }}</span>
        </div>
      </section>

      <aside class="ic-side" aria-label="任务与告警">
        <section class="ic-shift" aria-labelledby="ic-timeline-title">
          <header class="ic-side-head">
            <h2 id="ic-timeline-title"><button type="button" title="查看当班任务明细" @click="openDetail('tasks')">任务时间线</button></h2>
            <span class="ic-origin">{{ businessSource }}</span>
          </header>
          <ol v-if="taskRows.length" class="ic-timeline">
            <li v-for="task in taskRows" :key="task.id" :class="task.kind">
              <button type="button" class="ic-task-row" @click="openDetail('task', task)">
                <span class="ic-task-top"><time>{{ task.time }}</time><span>{{ task.status }}</span></span>
                <strong>{{ task.title }}</strong>
                <span class="ic-task-location">
                  <span class="ic-code">{{ task.id }}</span>
                  <span>{{ task.point }} · {{ scenePoints[task.point].name }}</span>
                </span>
                <span v-if="task.kind === 'active'" class="ic-progress">
                  <progress :value="task.finished" :max="task.total" aria-label="点位完成进度"></progress>
                  {{ task.finished }} / {{ task.total }} 点位
                </span>
              </button>
            </li>
          </ol>
          <div v-else class="ic-empty">
            <p>巡检任务接口尚未接入。</p>
            <button type="button" @click="sourcePolicy = 'demo'">查看本地样例 <Icon name="next" :size="14" /></button>
          </div>
        </section>
        <section class="ic-alerts" aria-labelledby="ic-alerts-title">
          <header class="ic-side-head">
            <h2 id="ic-alerts-title"><button type="button" title="查看全部告警" @click="openDetail('alarms')">告警动态</button></h2>
            <span class="ic-origin">{{ businessSource }}</span>
          </header>
          <ul v-if="activeAlarms.length" class="ic-alarm-list">
            <li v-for="alarm in activeAlarms" :key="alarm.id">
              <button type="button" class="ic-alarm-row" :class="alarm.tone" @click="openDetail('alarm', alarm)">
                <Icon name="warning" :size="13" />
                <span class="ic-alarm-content">
                  <span><span class="ic-severity">{{ alarm.severity }}</span><strong>{{ alarm.title }}</strong></span>
                  <small><time>{{ alarm.time }}</time> · {{ alarm.point }} {{ scenePoints[alarm.point].name }}<span v-if="reviewedIds.includes(alarm.id)"> · 已阅</span></small>
                </span>
              </button>
            </li>
          </ul>
          <div v-else class="ic-empty"><p>告警接口尚未接入，数量未知。</p></div>
        </section>
      </aside>
    </div>

    <section class="ic-kpis" aria-labelledby="ic-summary-title">
      <div class="ic-summary-heading">
        <h2 id="ic-summary-title">运行摘要</h2>
        <small>{{ knownMode === 'simulator' ? '网关模拟数据' : '按接口显示' }}</small>
      </div>
      <button type="button" class="ic-kpi" @click="openDetail('robot')">
        <span>机器人本体</span><strong>{{ summaryRobot }}</strong>
      </button>
      <button type="button" class="ic-kpi" @click="openDetail('dock')">
        <span>电池电量</span><strong class="ic-reading">{{ battery ?? '—' }}<small>%</small></strong>
      </button>
      <button type="button" class="ic-kpi" @click="openDetail('liveTask')">
        <span>当前任务</span><strong>未接入</strong>
      </button>
      <button type="button" class="ic-kpi" @click="openDetail('liveAlarms')">
        <span>活动告警</span><strong class="ic-reading">—<small>条</small></strong>
      </button>
    </section>

    <dialog ref="dialog" class="ic-dialog" aria-labelledby="ic-dialog-title" @close="detail=null" @click="($event.target===dialog) && closeDetail()">
      <article v-if="detail"><header><h2 id="ic-dialog-title">{{ detailTitle }}</h2><button type="button" aria-label="关闭详情" @click="closeDetail()"><Icon name="close" :size="17" /></button></header>
        <template v-if="detail.type==='sources'">
          <dl><div><dt>本体与电量</dt><dd>{{ source }} · {{ robotStatus }}</dd></div><div><dt>二维栅格</dt><dd>{{ gridSource }}</dd></div><div><dt>ODIN1 点云</dt><dd>本地实测 PCD 抽样，保留原始坐标和颜色</dd></div><div><dt>ODIN1 传感器位置</dt><dd>PC2 同图重定位，箭头表示传感器地图航向</dd></div><div><dt>示意场景与点位</dt><dd>本地合成场景，与实机坐标独立</dd></div><div><dt>任务、充电仓、告警</dt><dd>{{ sampleBusiness ? '本地合成样例' : '尚无业务接口' }}</dd></div></dl>
          <fieldset><legend>业务数据展示</legend>
            <label v-for="option in [{id:'demo',text:'显示本地模拟业务'},{id:'gateway',text:'仅显示已接入数据'}]" :key="option.id"><input v-model="sourcePolicy" type="radio" name="ic-source" :value="option.id"> {{ option.text }}</label>
          </fieldset><p>此设置只影响任务、充电仓和告警的演示样例。</p>
        </template>
        <template v-else-if="detail.type==='robot' || detail.type==='dock'">
          <dl><div><dt>遥测来源</dt><dd>{{ source }} · {{ robotStatus }}</dd></div><div><dt>机器人标识</dt><dd>{{ robot.id || '未知' }}</dd></div>
            <div><dt>遥测观测时间</dt><dd>{{ telemetryTime }}</dd></div>
            <template v-if="detail.type==='robot'"><div><dt>本体故障</dt><dd>未接入</dd></div><div><dt>运动模式</dt><dd>{{ modeName }}</dd></div><div><dt>线速度</dt><dd>{{ online && Number.isFinite(robot.linear_mps) ? robot.linear_mps.toFixed(2) + ' m/s' : '未知' }}</dd></div></template>
            <template v-else><div><dt>电池电量</dt><dd>{{ battery === null ? '未知' : battery+'%' }}</dd></div><div><dt>充电仓连接</dt><dd>{{ sampleBusiness ? '在线 · 本地模拟' : '未接入' }}</dd></div><div><dt>仓体故障</dt><dd>{{ sampleBusiness ? '无故障 · 本地模拟' : '未接入' }}</dd></div><div><dt>充电仓占用</dt><dd>{{ sampleBusiness ? '空闲 · 本地模拟' : '未接入' }}</dd></div><div><dt>充电状态</dt><dd>{{ sampleBusiness ? '未充电 · 本地模拟' : '未接入' }}</dd></div></template>
          </dl><button v-if="detail.type==='dock' && sampleBusiness" class="ic-action" type="button" @click="locatePoint('P03')">在场景中查看回仓交接点 →</button>
        </template>
        <template v-else-if="detail.type==='liveTask' || detail.type==='liveAlarms'">
          <p>{{ detail.type === 'liveTask' ? '巡检任务接口尚未接入，当前任务未知。' : '告警接口尚未接入，活动告警数量未知。' }}</p>
        </template>
        <template v-else-if="detail.type==='tasks' || detail.type==='alarms'">
          <p>数据来源 · {{ businessSource }}</p>
          <button v-for="item in (detail.type==='tasks'?taskRows:activeAlarms)" :key="item.id" type="button" class="ic-detail-row" @click="openDetail(detail.type==='tasks'?'task':'alarm',item)"><span>{{ item.time }}　{{ item.title }}</span><b>→</b></button>
          <p v-if="!sampleBusiness">业务接口尚未接入。</p>
        </template>
        <template v-else-if="detail.type==='point'">
          <p>本地合成场景</p><dl><div><dt>点位编号</dt><dd>{{ detail.item }}</dd></div><div><dt>设施编号</dt><dd>{{ scenePoints[detail.item].asset }}</dd></div><div><dt>巡检内容</dt><dd>{{ scenePoints[detail.item].purpose }}</dd></div></dl>
        </template>
        <template v-else>
          <p>本地模拟 · {{ detail.item.id }}</p><dl><div><dt>时间</dt><dd>{{ detail.item.time }}</dd></div><div><dt>关联点位</dt><dd>{{ detail.item.point }} · {{ scenePoints[detail.item.point].name }}</dd></div>
            <div><dt>状态</dt><dd>{{ reviewedIds.includes(detail.item.id) ? '已阅' : detail.item.status }}</dd></div>
            <div v-if="detail.type==='task'"><dt>完成点位</dt><dd>{{ detail.item.finished }} / {{ detail.item.total }}</dd></div>
            <div v-else><dt>来源</dt><dd>{{ detail.item.source }}</dd></div></dl><p v-if="detail.item.detail">{{ detail.item.detail }}</p>
          <footer><button type="button" class="ic-action" @click="locatePoint(detail.item.point)">在场景中查看 →</button>
            <button v-if="detail.type==='alarm' && !reviewedIds.includes(detail.item.id)" type="button" class="ic-secondary" @click="markReviewed(detail.item.id)">标记已阅（本次演示）</button></footer>
        </template>
      </article>
    </dialog>
  </div>
</template>

<style src="./cockpit.css"></style>
