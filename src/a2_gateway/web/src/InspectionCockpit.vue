<script setup>
import { computed, nextTick, ref, watch } from 'vue'
import Icon from './Icon.vue'
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

const dialog = ref(null)
const mapSection = ref(null)
const scene = ref(null)
const detail = ref(null)
// 点云是现场采集的静态地图。遥测模式切换或断线重连时保留操作员选定的视图。
const mapView = ref('odin')
const selectedPoint = ref('')
const sourcePolicy = ref('demo')
const knownMode = ref('')
const reviewedIds = ref([])

const connected = computed(() => props.connection === 'connected')
const online = computed(() => connected.value && props.state?.robot?.online === true)
const robot = computed(() => props.state?.robot || {})
// 网关负责底盘遥测有效性；掉线时不把上一帧电量当作实时数据。
const battery = computed(() => {
  const value = robot.value.battery_percent
  return online.value && Number.isFinite(value) && value >= 0 && value <= 100 ? Math.round(value) : null
})
const source = computed(() => ({
  simulator: '网关模拟', robot: '实机网关', robot_readonly: '实机只读',
})[knownMode.value] || '等待网关')
const robotStatus = computed(() => {
  if (!connected.value) return props.connection === 'connecting' ? '连接中' : '网关离线'
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
const activeTask = shiftTasks.find(task => task.kind === 'active')
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
  return ({ sources: '数据来源', robot: '机器人本体状态', dock: '电量与充电仓', tasks: '当班任务', alarms: '活动告警' })[entry.type]
})
</script>

<template>
  <div class="ic-root">
    <div class="ic-heading">
      <div><h1>巡检驾驶舱</h1><p>机器人遥测、巡检任务与告警</p></div>
      <button type="button" class="ic-source-button" @click="openDetail('sources')">
        <Icon name="stack" :size="16" />数据来源<i></i><strong>{{ source }}</strong><span aria-hidden="true">⌄</span>
      </button>
    </div>

    <section ref="mapSection" class="ic-panel ic-map" aria-label="巡检态势地图">
      <header class="ic-section-head"><div class="ic-section-title"><Icon name="stack" /> <h2>巡检态势地图</h2></div>
        <div class="ic-switch" role="group" aria-label="地图视图">
          <button type="button" :aria-pressed="mapView==='odin'" @click="mapView='odin'">ODIN1 点云</button>
          <button type="button" :aria-pressed="mapView==='scene'" @click="mapView='scene'">示意场景</button>
          <button type="button" :aria-pressed="mapView==='grid'" @click="mapView='grid'">二维栅格</button>
        </div>
      </header>
      <OdinPointCloud v-if="mapView==='odin'" />
      <div v-else-if="mapView==='scene'" class="ic-scene"><TunnelScene ref="scene" :selected-point="selectedPoint" @select-point="openPoint" /></div>
      <div v-else class="ic-grid-map"><MapView :map="map" :pose="currentPose" :status="mapStatus" :connection="connection" /></div>
      <div v-if="mapView!=='odin'" class="ic-map-caption"><span class="ic-map-source"><i class="ic-dot" :class="mapView==='scene' || connected ? 'green' : 'amber'"></i>{{ mapView==='scene' ? '本地合成场景' : source + ' · 二维地图' }}</span>
        <span v-if="mapView==='scene'">{{ selectedPoint ? selectedPoint + ' · ' + scenePoints[selectedPoint].name : '3 个巡检点位' }}</span>
        <span v-else>{{ connected ? (currentPose ? '定位有效' : '无有效位姿') : (map ? '缓存栅格 · 网关离线' : '网关未连接') }}</span>
      </div>
    </section>

    <div class="ic-kpis">
      <button type="button" class="ic-kpi" @click="openDetail('robot')">
        <span class="ic-kpi-top"><span class="ic-kicon"><Icon name="pulse" :size="17" /></span>机器人本体<span class="ic-tag">{{ source }}</span></span>
        <span class="ic-kvalue">{{ robot.model || (knownMode ? 'A2' : '待连接') }}<small class="ic-pill" :class="{ green: online }"><i></i>{{ robotStatus }}</small></span>
        <span class="ic-kfoot">运动模式 <b>{{ modeName }}</b></span>
      </button>
      <button type="button" class="ic-kpi" @click="openDetail('dock')">
        <span class="ic-kpi-top"><span class="ic-kicon"><Icon name="battery" :size="17" /></span>电量与充电仓<span class="ic-tag">{{ source }}</span></span>
        <span class="ic-kvalue ic-num">{{ battery ?? '—' }}<small>%</small><span class="ic-battery" aria-hidden="true"><i v-for="n in 7" :key="n" :class="{ filled: battery !== null && n <= Math.ceil(battery / 100 * 7) }"></i></span></span>
        <span class="ic-kfoot"><template v-if="sampleBusiness">仓空闲 · 未充电 <em class="ic-tag">本地模拟</em></template><template v-else>仓占用／充电 <b>未接入</b></template></span>
      </button>
      <button type="button" class="ic-kpi" @click="sampleBusiness ? openDetail('task', activeTask) : openDetail('tasks')">
        <span class="ic-kpi-top"><span class="ic-kicon blue"><Icon name="trail" :size="17" /></span>当前任务<span class="ic-tag">{{ businessSource }}</span></span>
        <span class="ic-kvalue" :class="{ 'ic-task-name': sampleBusiness }">{{ sampleBusiness ? activeTask.title : '—' }}</span>
        <span class="ic-kfoot"><template v-if="sampleBusiness"><i class="ic-dot green"></i>进行中 <span class="ic-divider">/</span> 已完成 1 / 3 个点位</template><template v-else>等待任务接口</template></span>
      </button>
      <button type="button" class="ic-kpi" @click="openDetail('alarms')">
        <span class="ic-kpi-top"><span class="ic-kicon amber"><Icon name="shield" :size="17" /></span>活动告警<span class="ic-tag">{{ businessSource }}</span></span>
        <span class="ic-kvalue ic-num">{{ sampleBusiness ? activeAlarms.length : '—' }}<small>条</small></span>
        <span class="ic-kfoot"><template v-if="sampleBusiness"><i class="ic-dot red"></i>告警 1 <span class="ic-divider">/</span><i class="ic-dot amber"></i>提醒 1</template><template v-else>等待告警接口</template></span>
      </button>
    </div>

    <div class="ic-lower">
      <section class="ic-panel ic-shift">
        <header class="ic-section-head"><div class="ic-section-title"><Icon name="clock" /><h2>当班任务</h2></div><span class="ic-tag">{{ businessSource }}</span></header>
        <div v-if="taskRows.length" class="ic-task-list">
          <button v-for="task in taskRows" :key="task.id" type="button" class="ic-task-row" :class="task.kind" @click="openDetail('task',task)">
            <span class="ic-timeline-dot"></span>
            <span class="ic-task-content"><span class="ic-task-meta"><b>{{ task.time }}</b><span class="ic-pill" :class="{green:task.kind==='active'}">{{ task.status }}</span></span>
              <strong>{{ task.title }}</strong><small>{{ task.id }} <span>·</span> {{ task.point }} {{ scenePoints[task.point].name }}</small>
              <span v-if="task.kind==='active'" class="ic-task-progress"><i></i><small>1 / 3 点位</small></span>
            </span>
          </button>
        </div>
        <p v-else class="ic-empty">当班任务接口尚未接入。</p>
        <button class="ic-panel-link" type="button" @click="openDetail('tasks')">查看班次明细 <span aria-hidden="true">→</span></button>
      </section>

      <section class="ic-panel ic-alerts" aria-label="告警列表">
        <header class="ic-section-head"><div class="ic-section-title"><Icon name="radar" /><h2>告警动态</h2><span class="ic-tag">{{ businessSource }}</span></div>
          <button class="ic-text-link" type="button" @click="openDetail('alarms')">全部告警 <span aria-hidden="true">→</span></button>
        </header>
        <div v-if="activeAlarms.length" class="ic-table-wrap"><table><thead><tr><th>发生时间</th><th>级别</th><th>告警内容</th><th>关联点位</th><th>处理状态</th><th><span class="ic-sr">详情</span></th></tr></thead>
          <tbody><tr v-for="alarm in activeAlarms" :key="alarm.id"><td class="ic-mono">{{ alarm.time }}</td><td><span class="ic-alert-level" :class="alarm.tone"><i></i>{{ alarm.severity }}</span></td><td><button class="ic-table-title" type="button" @click="openDetail('alarm',alarm)">{{ alarm.title }}</button></td><td>{{ alarm.point }} · {{ scenePoints[alarm.point].name }}</td><td>{{ reviewedIds.includes(alarm.id) ? '已阅' : alarm.status }}</td><td><button type="button" class="ic-text-link" @click="openDetail('alarm',alarm)">详情</button></td></tr></tbody>
        </table></div>
        <p v-else class="ic-empty">告警接口尚未接入，当前数量未知。</p>
      </section>
    </div>

    <dialog ref="dialog" class="ic-dialog" aria-labelledby="ic-dialog-title" @close="detail=null" @click="($event.target===dialog) && closeDetail()">
      <article v-if="detail"><header><h2 id="ic-dialog-title">{{ detailTitle }}</h2><button type="button" aria-label="关闭详情" @click="closeDetail()">×</button></header>
        <template v-if="detail.type==='sources'">
          <dl><div><dt>本体、电量与二维图</dt><dd>{{ source }} · {{ robotStatus }}</dd></div><div><dt>ODIN1 点云</dt><dd>本地实测 PCD 抽样，保留原始坐标和颜色</dd></div><div><dt>ODIN1 传感器位置</dt><dd>PC2 同图重定位，箭头表示传感器地图航向</dd></div><div><dt>示意场景与点位</dt><dd>本地合成场景，与实机坐标独立</dd></div><div><dt>任务、充电仓、告警</dt><dd>{{ sampleBusiness ? '本地合成样例' : '尚无业务接口' }}</dd></div></dl>
          <fieldset><legend>业务数据展示</legend>
            <label v-for="option in [{id:'demo',text:'显示本地模拟业务'},{id:'gateway',text:'仅显示已接入数据'}]" :key="option.id"><input v-model="sourcePolicy" type="radio" name="ic-source" :value="option.id"> {{ option.text }}</label>
          </fieldset><p>本体与二维图始终读取网关；本地业务样例仅用于演示。</p>
        </template>
        <template v-else-if="detail.type==='robot' || detail.type==='dock'">
          <dl><div><dt>遥测来源</dt><dd>{{ source }} · {{ robotStatus }}</dd></div><div><dt>机器人标识</dt><dd>{{ robot.id || '未知' }}</dd></div>
            <div><dt>遥测观测时间</dt><dd>{{ telemetryTime }}</dd></div>
            <template v-if="detail.type==='robot'"><div><dt>本体故障</dt><dd>未接入</dd></div><div><dt>运动模式</dt><dd>{{ modeName }}</dd></div><div><dt>线速度</dt><dd>{{ online && Number.isFinite(robot.linear_mps) ? robot.linear_mps.toFixed(2) + ' m/s' : '未知' }}</dd></div></template>
            <template v-else><div><dt>电池电量</dt><dd>{{ battery === null ? '未知' : battery+'%' }}</dd></div><div><dt>充电仓连接</dt><dd>{{ sampleBusiness ? '在线 · 本地模拟' : '未接入' }}</dd></div><div><dt>仓体故障</dt><dd>{{ sampleBusiness ? '无故障 · 本地模拟' : '未接入' }}</dd></div><div><dt>充电仓占用</dt><dd>{{ sampleBusiness ? '空闲 · 本地模拟' : '未接入' }}</dd></div><div><dt>充电状态</dt><dd>{{ sampleBusiness ? '未充电 · 本地模拟' : '未接入' }}</dd></div></template>
          </dl><button v-if="detail.type==='dock' && sampleBusiness" class="ic-action" type="button" @click="locatePoint('P03')">在场景中查看回仓交接点 →</button>
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
