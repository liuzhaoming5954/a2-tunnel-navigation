<script setup>
import { onBeforeUnmount, onMounted, ref, shallowRef } from 'vue'
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
const poseLabels = {
  live: 'PC2 实时重定位', waiting: '等待 PC2 重定位', stale: '重定位数据已过期',
  invalid: '重定位数据无效', map_mismatch: '重定位地图与点云不匹配', disconnected: '定位服务未连接',
}
let activeRequest, view, stopSensor

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
    view = createOdinView(canvas.value, map, bytes, position => { marker.value = position })
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
        aria-label="ODIN1 实测点云和传感器位置。拖动或按方向键旋转，滚轮缩放，Home 键恢复默认视角。"
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
        <p class="odin-gestures">拖动旋转 <i>·</i> 滚轮缩放 <i>·</i> 右键平移</p>
        <div class="odin-tools" role="group" aria-label="点云视角">
          <div class="odin-tool-group">
            <button type="button" @click="view.top()">俯视</button>
            <button type="button" title="适配全部抽样点" @click="view.fitAll()">全图</button>
            <button type="button" @click="view.reset()">默认视角</button>
          </div>
          <div class="odin-tool-group odin-zoom">
            <button type="button" title="缩小" aria-label="缩小" @click="view.zoom(1 / 1.4)"><Icon name="minus" :size="16" /></button>
            <button type="button" title="放大" aria-label="放大" @click="view.zoom(1.4)"><Icon name="plus" :size="16" /></button>
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
    <div v-if="metadata" class="odin-source">
      <span class="odin-file"><Icon name="layers" :size="15" /><span>实测点云</span> <b>{{ metadata.sourceName }}</b></span>
      <span class="odin-counts">展示 <b>{{ number.format(metadata.pointCount) }}</b> 点 <i>/</i> 原始 <b>{{ number.format(metadata.sourcePoints) }}</b> 点</span>
    </div>
    <div v-if="status === 'ready'" class="odin-pose">
      <div class="odin-pose-heading">
        <Icon name="center" :size="15" /><strong>ODIN1 传感器位置</strong>
        <span role="status" class="odin-signal" :data-status="poseStatus"><i></i>{{ poseLabels[poseStatus] }}</span>
        <button v-if="sensorPose && marker && !marker.inView" type="button" @click="view.centerSensor()">居中查看</button>
      </div>
      <div v-if="sensorPose" class="odin-coords" aria-label="传感器地图坐标与航向">
        <span><b>X</b> {{ sensorPose.x_m.toFixed(3) }} m</span><span><b>Y</b> {{ sensorPose.y_m.toFixed(3) }} m</span><span><b>Z</b> {{ sensorPose.z_m.toFixed(3) }} m</span>
        <span title="地图 XY 平面，+X 朝 +Y 为正"><b>航向</b> {{ (sensorPose.yaw_rad * 180 / Math.PI).toFixed(1) }}°</span>
      </div>
    </div>
  </section>
</template>

<style scoped>
/* 绘图区与来源栏使用同一组深色；尺寸继续跟随驾驶舱分配给地图的可用高度。 */
.odin-map {
  --odin-base: #151b30;
  --odin-strip: #202941;
  --odin-line: #38435e;
  --odin-muted: #bfcae0;
  --odin-bright: #f4f4f9;
  --odin-accent: #b9c6ff;
  --odin-error: #ffb9b3;
  display: flex;
  flex-direction: column;
  min-width: 0;
  min-height: 0;
  background: var(--odin-base);
  color: var(--odin-bright);
  color-scheme: dark;
}
.odin-viewport { position: relative; height: var(--ic-map-height); overflow: hidden; }
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

/* 视角与倍率分组放在画布边缘。提示不拦截鼠标，拖拽仍能到达底层画布。 */
.odin-tools { position: absolute; bottom: 14px; right: 14px; display: flex; gap: 6px; }
.odin-tool-group {
  display: flex;
  align-items: center;
  padding: 2px;
  background: #202941f2;
  border: 1px solid #667293;
  border-radius: 7px;
  box-shadow: 0 2px 5px #0b102259;
}
.odin-map button {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  min-height: 32px;
  padding: 4px 9px;
  border: 0;
  border-radius: 4px;
  background: transparent;
  color: var(--odin-bright);
  font: inherit;
  font-size: 12px;
  line-height: 1.4;
}
.odin-zoom button { width: 32px; padding: 0; }
.odin-map button:is(:hover, :focus-visible) { background: #354367; color: #e2e7ff; }
.odin-map button:focus-visible { outline: 2px solid var(--odin-accent); outline-offset: -2px; }
.odin-viewport .odin-gestures {
  position: absolute;
  bottom: 20px;
  left: 16px;
  margin: 0;
  padding: 2px 5px;
  border-radius: 3px;
  pointer-events: none;
  color: var(--odin-muted);
  background: #151b30e8;
  font-size: 11px;
}
.odin-gestures i, .odin-counts i { font-style: normal; color: #96a2bb; margin-inline: 3px; }

/* 仅在未绘制实测点时显示网格底纹，接入数据后保留原始 PCD 颜色与几何。 */
.odin-state {
  position: absolute;
  inset: 0;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  gap: 7px;
  padding: 20px;
  background-image: radial-gradient(circle, #2c344d 1px, transparent 1px);
  background-size: 22px 22px;
  color: var(--odin-muted);
  text-align: center;
  font-size: 12px;
}
.odin-state-icon {
  display: grid;
  place-items: center;
  width: 52px;
  height: 52px;
  margin-bottom: 6px;
  border: 1px solid #4c587a;
  border-radius: 12px;
  background: #252e49;
  color: var(--odin-accent);
}
/* 加载失败时，错误图标与位姿无效提示使用同一警示色。 */
.odin-state:has(> button) .odin-state-icon { color: var(--odin-error); }
.odin-state strong { font-size: 16px; font-weight: 600; color: var(--odin-bright); }
.odin-state button, .odin-pose-heading button {
  padding-inline: 13px;
  border: 1px solid #7381a5;
  background: #2c375a;
  color: #d8e0ff;
}
.odin-state button { margin-top: 7px; }

/* 地图文件与动态位姿各占一行；遇到长文件名或窄屏时换行显示完整读数。 */
.odin-source, .odin-pose {
  display: flex;
  justify-content: space-between;
  align-items: center;
  flex-wrap: wrap;
  gap: 5px 16px;
  padding: 8px 16px;
  border-top: 1px solid var(--odin-line);
  background: var(--odin-strip);
  color: var(--odin-muted);
  font-size: 12px;
  line-height: 1.5;
}
.odin-file, .odin-pose-heading, .odin-coords, .odin-signal {
  display: inline-flex;
  align-items: center;
  flex-wrap: wrap;
  gap: 4px 8px;
}
.odin-file { min-width: 0; }
.odin-file > svg, .odin-pose-heading > svg { flex: none; color: var(--odin-accent); }
.odin-file > b, .odin-counts > b, .odin-coords {
  font-family: var(--ic-mono, monospace);
  font-weight: 500;
  font-variant-numeric: tabular-nums;
  color: var(--odin-bright);
}
.odin-file > b { overflow-wrap: anywhere; }
.odin-pose { background: #252e46; }
.odin-pose-heading > strong { font-weight: 600; color: var(--odin-bright); }
.odin-signal { gap: 6px; color: #ebc98d; }
.odin-signal i { width: 6px; height: 6px; flex: none; border-radius: 50%; background: currentColor; }
.odin-signal[data-status="live"] { color: var(--odin-accent); }
.odin-signal:is([data-status="invalid"], [data-status="map_mismatch"]) { color: var(--odin-error); }
.odin-pose-heading button { min-height: 26px; padding: 1px 8px; }
.odin-coords { gap: 4px 14px; }
.odin-coords span { white-space: nowrap; }
.odin-coords b { font: inherit; color: var(--odin-muted); }

/* 标记锚点与投影坐标重合。方向线和来源标签独立于底图颜色，保持高对比度。 */
.odin-marker { position: absolute; width: 0; height: 0; pointer-events: none; }
.odin-marker svg { position: absolute; left: -24px; top: -24px; overflow: visible; filter: drop-shadow(0 1px 2px #0a1025); }
.odin-marker path { fill: none; stroke: var(--odin-accent); stroke-width: 3; stroke-linecap: round; stroke-linejoin: round; }
.odin-marker circle { fill: #515fb1; stroke: #f4f5ff; stroke-width: 2.5; }
.odin-marker > span {
  position: absolute;
  left: 8px;
  top: 12px;
  padding: 2px 6px;
  border: 1px solid #7281ae;
  border-radius: 4px;
  white-space: nowrap;
  color: #e0e6ff;
  background: #242e50f2;
  font-size: 11px;
}

@container cockpit (max-width: 620px) {
  .odin-viewport .odin-gestures { display: none; }
  .odin-tools { right: 8px; bottom: 8px; }
  .odin-source, .odin-pose { padding-inline: 12px; }
}
@media (pointer: coarse) {
  .odin-viewport .odin-gestures { display: none; }
  .odin-tool-group button { min-height: 42px; min-width: 42px; }
}
</style>
