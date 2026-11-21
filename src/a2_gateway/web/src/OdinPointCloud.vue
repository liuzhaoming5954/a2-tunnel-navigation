<script setup>
import { computed, onBeforeUnmount, onMounted, ref, shallowRef } from 'vue'
import Icon from './Icon.vue'
import { createOdinView } from './odinPointCloud.js'
import { subscribeOdinSensor } from './odinSensorFeed.js'

const canvas = ref(null)
const metadata = shallowRef(null)
const status = ref('loading')
const message = ref('')
const number = new Intl.NumberFormat('zh-CN')
const pointers = new Map()
const poseStatus = ref('waiting')
const sensorPose = shallowRef(null)
const marker = shallowRef(null)
const reference = shallowRef(null)
const poseLabels = {
  live: 'PC2 实时重定位', waiting: '等待 PC2 重定位', stale: '重定位数据已过期',
  invalid: '重定位数据无效', map_mismatch: '重定位地图与点云不匹配', disconnected: '定位服务未连接',
}
let activeRequest, view, stopSensor

// 缺少有效重定位时仍保留读数栏的单位与位置，避免将上一次坐标误认为当前观测。
const readings = computed(() => {
  const pose = sensorPose.value
  return [
    { label: 'X', value: pose?.x_m.toFixed(3), unit: 'm' },
    { label: 'Y', value: pose?.y_m.toFixed(3), unit: 'm' },
    { label: 'Z', value: pose?.z_m.toFixed(3), unit: 'm' },
    { label: '航向', value: pose ? (pose.yaw_rad * 180 / Math.PI).toFixed(1) : null, unit: '°' },
  ]
})

// 状态、坐标读数和地图标记在同一次更新中替换。离线和过期均清空上次坐标。
function updateSensor(next, pose) {
  poseStatus.value = next
  sensorPose.value = pose
  view?.setSensorPose(pose)
}

function requireFile(response) {
  if (response.status === 404) throw new Error('网关未提供 ODIN1 点云展示文件')
  if (response.status === 409) throw new Error('地图文件已更新，请重新加载')
  if (!response.ok) throw new Error('地图服务暂时不可用（HTTP ' + response.status + '）')
  return response
}
function validMetadata(map) {
  return typeof map?.sourceName === 'string' && map.sourceName.length > 0
    && Number.isSafeInteger(map.pointCount) && map.pointCount > 0
    && Number.isSafeInteger(map.sourcePoints) && map.sourcePoints >= map.pointCount
    && map.strideBytes === 16 && map.encoding === 'xyz-f32le-rgba-u8'
    && /^[0-9a-f]{64}$/.test(map.sampleSha256)
    && [0, 1, 2].every(i => Number.isFinite(map.viewBounds?.min?.[i])
      && Number.isFinite(map.viewBounds?.max?.[i]) && map.viewBounds.min[i] <= map.viewBounds.max[i])
}

async function load() {
  activeRequest?.abort()
  stopSensor?.()
  stopSensor = null
  view?.destroy()
  view = null
  metadata.value = null
  reference.value = null
  message.value = ''
  status.value = 'loading'
  const request = new AbortController()
  activeRequest = request
  const timeout = setTimeout(() => request.abort(), 30000)
  try {
    // 只访问网关发布的轻量样本。携带元数据中的版本，避免切换地图时配错来源。
    const options = { signal: request.signal, cache: 'no-store' }
    const description = await fetch('/api/maps/odin1/metadata', options)
    const map = await requireFile(description).json()
    if (!validMetadata(map)) throw new Error('网关返回的点云描述不完整或格式不兼容')
    const response = await fetch('/api/maps/odin1/points?version=' + map.sampleSha256, options)
    const bytes = await requireFile(response).arrayBuffer()
    if (request !== activeRequest) return
    view = createOdinView(canvas.value, map, bytes, position => { marker.value = position },
      frameReference => { reference.value = frameReference })
    metadata.value = map
    status.value = 'ready'
    stopSensor = subscribeOdinSensor(map.sampleSha256, updateSensor)
  } catch (error) {
    // 页面切换或用户再次加载会撤销旧请求，旧响应不能覆盖新页面的加载状态。
    if (request !== activeRequest) return
    if (request.signal.aborted) message.value = '获取点云超时，请检查网关连接'
    else if (error instanceof TypeError) message.value = '无法连接点云服务，请检查网关连接'
    else if (error instanceof SyntaxError) message.value = '网关未返回有效的地图描述'
    else message.value = error.message
    status.value = 'error'
  } finally {
    clearTimeout(timeout)
    if (activeRequest === request) activeRequest = null
  }
}

// 保留预览的鼠标拖动、右键平移和滚轮锚定缩放。双指捏合同时跟随两指中点平移。
function pointerDown(event) {
  if (!view || event.button > 2) return
  event.currentTarget.setPointerCapture(event.pointerId)
  pointers.set(event.pointerId, [event.clientX, event.clientY])
}
function pointerMove(event) {
  const previous = pointers.get(event.pointerId)
  if (!view || !previous) return
  const x = event.clientX, y = event.clientY
  const dx = x - previous[0], dy = y - previous[1]
  if (pointers.size === 2) {
    const [, other] = [...pointers].find(([id]) => id !== event.pointerId)
    const before = Math.hypot(previous[0] - other[0], previous[1] - other[1])
    const after = Math.hypot(x - other[0], y - other[1])
    const rect = canvas.value.getBoundingClientRect()
    if (before > 2 && after > 2) {
      view.zoom(after / before, (previous[0] + other[0] - rect.width) / 2 - rect.left,
        (previous[1] + other[1] - rect.height) / 2 - rect.top)
    }
    view.pan(dx / 2, dy / 2)
  } else if (event.shiftKey || (event.buttons & 6)) view.pan(dx, dy)
  else view.rotate(dx, dy)
  pointers.set(event.pointerId, [x, y])
}
function release(event) {
  pointers.delete(event.pointerId)
}
function wheel(event) {
  if (!view) return
  const rect = canvas.value.getBoundingClientRect()
  const unit = event.deltaMode === 1 ? 16 : event.deltaMode === 2 ? rect.height : 1
  const delta = Math.max(-480, Math.min(480, event.deltaY * unit))
  view.zoom(Math.exp(-delta * .0018), event.clientX - rect.left - rect.width / 2,
    event.clientY - rect.top - rect.height / 2)
}
// 保持与驾驶舱已有三维场景相同的键盘操作，便于无法使用拖拽的操作员切换视角。
function keyDown(event) {
  if (!view) return
  const offsets = { ArrowLeft: [-16, 0], ArrowRight: [16, 0], ArrowUp: [0, 12], ArrowDown: [0, -12] }
  if (event.key === 'Home') view.reset()
  else if (offsets[event.key]) view.rotate(...offsets[event.key])
  else return
  event.preventDefault()
}
onMounted(load)
onBeforeUnmount(() => {
  const request = activeRequest
  activeRequest = null
  request?.abort()
  stopSensor?.()
  view?.destroy()
  pointers.clear()
})
</script>

<template>
  <section class="odin-map" aria-label="ODIN1 点云地图" :aria-busy="status === 'loading'">
    <div class="odin-viewport">
      <canvas ref="canvas" v-show="status === 'ready'" tabindex="0"
        aria-label="ODIN1 实测点云和传感器位置。拖动或按方向键旋转，滚轮缩放，右键平移，Home 键恢复默认视角。"
        title="拖动旋转 · 滚轮缩放 · 右键平移"
        @pointerdown="pointerDown" @pointermove="pointerMove" @pointerup="release"
        @pointercancel="release" @lostpointercapture="release" @contextmenu.prevent
        @wheel.prevent="wheel" @keydown="keyDown"></canvas>
      <div v-if="sensorPose && marker?.inView" class="odin-marker" aria-hidden="true"
        :style="{ left: marker.x + 'px', top: marker.y + 'px' }">
        <svg viewBox="-24 -24 48 48" width="48" height="48">
          <g v-if="marker.headingDeg !== null" :transform="'rotate(' + marker.headingDeg + ')'">
            <path d="M4 0 H19 M12 -5 L19 0 L12 5" />
          </g>
          <circle r="4.5" />
        </svg>
        <span>ODIN1 传感器</span>
      </div>
      <template v-if="status === 'ready'">
        <!-- 画幅角标固定在视口边缘，点云和位姿仍使用画布本身的坐标系。 -->
        <i v-for="corner in ['top-left', 'top-right', 'bottom-left', 'bottom-right']"
          :key="corner" class="odin-corner" :class="corner" aria-hidden="true"></i>
        <div class="odin-source">
          <span class="odin-file"><i aria-hidden="true"></i>实测点云 <b>{{ metadata.sourceName }}</b></span>
          <span class="odin-counts" title="抽样展示点数 / 实测原始点数">显示 <b>{{ number.format(metadata.pointCount) }}</b> / <b>{{ number.format(metadata.sourcePoints) }}</b> 点</span>
        </div>
        <div v-if="reference" class="odin-reference" aria-hidden="true">
          <svg viewBox="0 0 70 60" width="70" height="60">
            <g v-for="axis in reference.axes" :key="axis.label">
              <line x1="36" y1="37" :x2="36 + 21 * axis.x" :y2="37 + 21 * axis.y" :stroke="axis.color" stroke-width="1.4" />
              <text :x="36 + 28 * axis.x" :y="40 + 28 * axis.y" :fill="axis.color" text-anchor="middle">{{ axis.label }}</text>
            </g>
            <circle cx="36" cy="37" r="2" fill="#b8c0c8" />
          </svg>
          <span class="odin-scale" :style="{ width: reference.scale.width + 'px' }">{{ reference.scale.meters }} m<i></i></span>
        </div>
        <div class="odin-tools" role="group" aria-label="点云视角">
          <div class="odin-tool-group">
            <button type="button" @click="view.top()">
              <svg class="odin-view-icon" viewBox="0 0 24 24" aria-hidden="true"><path d="M2 12s4-7 10-7 10 7 10 7-4 7-10 7S2 12 2 12Z" /><circle cx="12" cy="12" r="3" /></svg>俯视
            </button>
            <button type="button" title="适配全部抽样点" @click="view.fitAll()"><Icon name="fit" :size="13" />全图</button>
            <button type="button" title="恢复默认视角（Home）" aria-label="复位到默认视角" @click="view.reset()">
              <svg class="odin-view-icon" viewBox="0 0 24 24" aria-hidden="true"><path d="M3 11a9 9 0 1 1 2 7M3 4v7h7" /></svg>复位
            </button>
          </div>
          <div class="odin-tool-group odin-zoom">
            <button type="button" title="缩小" aria-label="缩小" @click="view.zoom(1 / 1.4)"><Icon name="minus" :size="14" /></button>
            <button type="button" title="放大" aria-label="放大" @click="view.zoom(1.4)"><Icon name="plus" :size="14" /></button>
          </div>
        </div>
      </template>
      <div v-else class="odin-state" role="status">
        <div class="odin-state-icon"><Icon :name="status === 'error' ? 'emptyMap' : 'layers'" :size="26" /></div>
        <strong>{{ status === 'error' ? '点云地图暂不可用' : '正在加载 ODIN1 实测点云' }}</strong>
        <span v-if="status === 'error'">{{ message }}</span>
        <button v-if="status === 'error'" type="button" @click="load">重新加载</button>
      </div>
    </div>
    <div v-if="status === 'ready'" class="odin-pose" aria-label="传感器地图坐标与航向">
      <div class="odin-pose-heading">
        <strong>ODIN1 传感器位置</strong>
        <div class="odin-pose-status">
          <span role="status" class="odin-signal" :data-status="poseStatus"><i></i>{{ poseLabels[poseStatus] }}</span>
          <button v-if="sensorPose && marker && !marker.inView" type="button" @click="view.centerSensor()">居中查看</button>
        </div>
      </div>
      <dl v-for="reading in readings" :key="reading.label" class="odin-coord" :title="reading.label === '航向' ? '地图 XY 平面，+X 朝 +Y 为正' : undefined">
        <dt>{{ reading.label }}</dt><dd>{{ reading.value ?? '—' }}<small>{{ reading.unit }}</small></dd>
      </dl>
    </div>
  </section>
</template>

<style scoped>
/* 地图占用父面板分配的可用高度。用本组件的宽度控制图例排版，适应侧栏收起和小屏。 */
.odin-map {
  --odin-line: #50565b;
  --odin-muted: #b6c2c7;
  --odin-bright: #e0e4df;
  --odin-accent: #90b7cb;
  --odin-error: #ffa37c;
  --odin-mono: "Liberation Mono", "Noto Sans Mono CJK SC", monospace;
  min-width: 0;
  color: var(--odin-bright);
  background: #282c30;
  color-scheme: dark;
  container: odin-map / inline-size;
}
.odin-viewport {
  position: relative;
  height: var(--ic-map-height, 50vh);
  overflow: hidden;
  background: radial-gradient(ellipse at 43% 35%, #1b2328 0%, #171b1e 59%, #131518 100%);
}
.odin-viewport canvas {
  position: absolute;
  inset: 0;
  width: 100%;
  height: 100%;
  display: block;
  touch-action: none;
  cursor: grab;
}
.odin-viewport canvas:active { cursor: grabbing; }
.odin-viewport canvas:focus-visible { outline: 2px solid var(--odin-accent); outline-offset: -3px; }

/* 角标叠在透明图层上，不接收鼠标事件，因此拖拽可以从画布任意位置开始。 */
.odin-corner { position: absolute; width: 10px; height: 10px; color: #929c9f; pointer-events: none; }
.odin-corner.top-left { left: 6px; top: 6px; border-left: 1px solid; border-top: 1px solid; }
.odin-corner.top-right { right: 6px; top: 6px; border-right: 1px solid; border-top: 1px solid; }
.odin-corner.bottom-left { left: 6px; bottom: 6px; border-left: 1px solid; border-bottom: 1px solid; }
.odin-corner.bottom-right { right: 6px; bottom: 6px; border-right: 1px solid; border-bottom: 1px solid; }

/* 文件来源、图尺与坐标轴覆盖于画布之上，均不截获地图的拖动和滚轮事件。 */
.odin-source {
  position: absolute;
  top: 13px;
  left: 16px;
  right: 16px;
  display: flex;
  align-items: baseline;
  justify-content: space-between;
  gap: 6px 16px;
  pointer-events: none;
  color: var(--odin-muted);
  font-size: 11px;
  line-height: 1.5;
  text-shadow: 0 1px 2px #101315;
}
.odin-file { display: flex; align-items: baseline; flex-wrap: wrap; gap: 0 8px; min-width: 0; }
.odin-file i { width: 6px; height: 6px; align-self: center; background: #3b7a9e; }
.odin-file b { overflow-wrap: anywhere; color: #bec7c9; font: 11px/1.5 var(--odin-mono); }
.odin-counts { flex: none; white-space: nowrap; }
.odin-counts b { color: var(--odin-bright); font: 600 11px var(--odin-mono); }
.odin-reference {
  position: absolute;
  bottom: 11px;
  left: 11px;
  display: flex;
  align-items: flex-end;
  gap: 9px;
  color: #c5cece;
  font: 11px var(--odin-mono);
  pointer-events: none;
}
.odin-reference svg text { font-size: 10px; }
.odin-scale { display: grid; gap: 4px; text-align: center; margin-bottom: 3px; white-space: nowrap; }
.odin-scale i { height: 5px; border: 1px solid #a7b2b8; border-top: 0; }

/* 切换视角与缩放分组；键盘方向键、Home 及双指缩放使用同一个相机。 */
.odin-tools { position: absolute; right: 14px; bottom: 12px; display: flex; gap: 6px; }
.odin-tool-group {
  display: flex;
  align-items: center;
  overflow: hidden;
  background: #24282cf5;
  border: 1px solid #5c6163;
}
.odin-map button {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  gap: 6px;
  min-height: 29px;
  padding: 0 9px;
  border: 0;
  border-radius: 0;
  background: transparent;
  color: var(--odin-bright);
  cursor: pointer;
  font: inherit;
  font-size: 11px;
  line-height: 1.4;
}
.odin-view-icon { width: 13px; height: 13px; fill: none; stroke: currentColor; stroke-width: 1.7; }
.odin-tool-group button + button { border-left: 1px solid var(--odin-line); }
.odin-zoom button { width: 30px; padding: 0; }
.odin-map button:is(:hover, :focus-visible) { background: #41484c; }
.odin-map button:focus-visible { outline: 2px solid var(--odin-accent); outline-offset: -2px; }
.odin-state {
  position: absolute;
  inset: 0;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 7px;
  padding: 20px;
  color: var(--odin-muted);
  text-align: center;
  font-size: 12px;
}
.odin-state-icon { color: var(--odin-accent); margin-bottom: 3px; }
.odin-state:has(> button) .odin-state-icon { color: var(--odin-error); }
.odin-state strong { font-size: 16px; font-weight: 600; color: var(--odin-bright); }
.odin-state button, .odin-pose-heading button {
  border: 1px solid #676f74;
}
.odin-state button { margin-top: 7px; padding-inline: 14px; }

.odin-pose {
  display: grid;
  grid-template-columns: minmax(150px, 1.6fr) repeat(4, minmax(0, 1fr));
  min-height: 62px;
  align-items: center;
  border-top: 1px solid #4e5558;
}
.odin-pose-heading { display: grid; align-content: center; gap: 3px; padding: 5px 12px 5px 18px; }
.odin-pose-heading strong { font-size: 14px; line-height: 1.4; font-weight: 660; }
.odin-pose-status { display: flex; align-items: center; flex-wrap: wrap; gap: 3px 8px; }
.odin-pose-heading button { min-height: 21px; padding: 0 5px; font-size: 10px; }
.odin-signal { display: inline-flex; align-items: center; gap: 7px; color: #aeb7b9; font-size: 11px; line-height: 1.4; }
.odin-signal i { width: 6px; height: 6px; flex: none; border: 1px solid currentColor; }
.odin-signal[data-status="live"] { color: var(--odin-accent); }
.odin-signal[data-status="live"] i { background: currentColor; }
.odin-signal:is([data-status="invalid"], [data-status="map_mismatch"]) { color: var(--odin-error); }
.odin-coord {
  display: grid;
  align-content: center;
  min-height: 33px;
  margin: 0;
  padding-left: clamp(8px, 2cqi, 18px);
  gap: 4px;
  border-left: 1px solid var(--odin-line);
  font-family: var(--odin-mono);
}
.odin-coord dt { color: #b8c0c8; font-size: 11px; }
.odin-coord dd { margin: 0; font-size: 19px; font-weight: 600; line-height: 1; font-variant-numeric: tabular-nums; }
.odin-coord small { margin-left: 5px; font-size: 11px; font-weight: 400; color: #a7b5bc; }

/* 标记锚点与投影坐标重合。方向线和来源标签独立于底图颜色，保持高对比度。 */
.odin-marker { position: absolute; width: 0; height: 0; pointer-events: none; }
.odin-marker svg { position: absolute; left: -24px; top: -24px; overflow: visible; filter: drop-shadow(0 1px 2px #121417); }
.odin-marker path { fill: none; stroke: var(--odin-accent); stroke-width: 3; stroke-linecap: round; stroke-linejoin: round; }
.odin-marker circle { fill: #3b7a9e; stroke: var(--odin-bright); stroke-width: 2.5; }
.odin-marker > span {
  position: absolute;
  left: 8px;
  top: 12px;
  padding: 2px 6px;
  border: 1px solid #748e9c;
  white-space: nowrap;
  color: var(--odin-bright);
  background: #24282cf2;
  font-size: 11px;
}

@container odin-map (max-width: 640px) {
  .odin-source { left: 12px; right: 12px; top: 10px; flex-wrap: wrap; }
  .odin-pose { grid-template-columns: repeat(4, minmax(0, 1fr)); padding-bottom: 10px; }
  .odin-pose-heading { grid-column: 1 / -1; display: flex; flex-wrap: wrap; align-items: center; gap: 8px; padding: 9px 12px; }
  .odin-coord { padding-left: 12px; }
  .odin-coord:first-of-type { border-left: 0; }
  .odin-coord dd { font-size: 14px; overflow-wrap: anywhere; }
}
@container odin-map (max-width: 470px) {
  .odin-source { flex-direction: column; gap: 2px; }
  /* 双指设备上的视角键至少高 44 像素，图尺单独抬高一行以避开按钮边框。 */
  .odin-reference { left: 7px; bottom: 64px; }
  .odin-tools { right: 9px; bottom: 9px; }
}
@media (pointer: coarse) {
  .odin-tool-group button { min-height: 44px; min-width: 44px; }
}
</style>
