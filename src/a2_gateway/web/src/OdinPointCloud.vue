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
      <div v-if="status === 'ready'" class="odin-tools" role="group" aria-label="点云视角">
        <button type="button" @click="view.top()">俯视</button>
        <button type="button" title="适配全部抽样点" @click="view.fitAll()">全图</button>
        <button type="button" @click="view.reset()">默认视角</button>
        <button type="button" title="缩小" aria-label="缩小" @click="view.zoom(1 / 1.4)"><Icon name="minus" :size="15" /></button>
        <button type="button" title="放大" aria-label="放大" @click="view.zoom(1.4)"><Icon name="plus" :size="15" /></button>
      </div>
      <div v-else class="odin-state" role="status">
        <Icon :name="status === 'error' ? 'emptyMap' : 'layers'" :size="24" />
        <span>{{ status === 'error' ? message : '正在加载 ODIN1 实测点云' }}</span>
        <button v-if="status === 'error'" type="button" @click="load">重新加载</button>
      </div>
    </div>
    <div v-if="metadata" class="odin-source">
      <span>实测点云 <b>{{ metadata.sourceName }}</b></span>
      <span>展示 {{ number.format(metadata.pointCount) }} 点 <i> / </i> 原始 {{ number.format(metadata.sourcePoints) }} 点</span>
    </div>
    <div v-if="status === 'ready'" class="odin-pose">
      <div class="odin-pose-heading">
        <strong>ODIN1 传感器位置</strong>
        <span role="status" :class="{live: poseStatus === 'live'}"><i></i>{{ poseLabels[poseStatus] }}</span>
        <button v-if="sensorPose && marker && !marker.inView" type="button" @click="view.centerSensor()">居中查看</button>
      </div>
      <div v-if="sensorPose" class="odin-coords" aria-label="传感器地图坐标与航向">
        <span>X {{ sensorPose.x_m.toFixed(3) }} m</span><span>Y {{ sensorPose.y_m.toFixed(3) }} m</span><span>Z {{ sensorPose.z_m.toFixed(3) }} m</span>
        <span title="地图 XY 平面，+X 朝 +Y 为正">航向 {{ (sensorPose.yaw_rad * 180 / Math.PI).toFixed(1) }}°</span>
      </div>
    </div>
  </section>
</template>

<style scoped>
.odin-map { display: flex; flex-direction: column; min-width: 0; min-height: 0; background: #0e1921; color: #e8f3ee; }
.odin-viewport { position: relative; height: var(--ic-map-height); overflow: hidden; }
.odin-viewport canvas { position: absolute; inset: 0; width: 100%; height: 100%; display: block; touch-action: none; cursor: grab; }
.odin-viewport canvas:active { cursor: grabbing; }
.odin-viewport canvas:focus-visible { outline: 2px solid #8cdab4; outline-offset: -3px; }
.odin-tools { position: absolute; right: 10px; top: 10px; display: flex; gap: 1px; padding: 2px; background: #14262ded; border: 1px solid #5c7273; border-radius: 7px; }
.odin-map button { display: inline-flex; align-items: center; justify-content: center; min-width: 30px; min-height: 28px; padding: 4px 8px; border: 0; border-radius: 4px; background: transparent; color: #e8f3ee; font: inherit; font-size: 12px; }
.odin-map button:hover, .odin-map button:focus-visible { color: #dcfff0; background: #354e51; }
.odin-state { position: absolute; inset: 0; display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 10px; padding: 12px; color: #ccdadb; font-size: 13px; text-align: center; }
.odin-state button { border: 1px solid #657978; padding-inline: 14px; }
.odin-source { display: flex; flex-wrap: wrap; justify-content: space-between; gap: 3px 12px; padding: 7px 12px; color: #b6cbca; background: #17272c; font-size: 11px; line-height: 1.5; overflow-wrap: anywhere; }
.odin-source b { font-weight: 500; color: #e9f2ee; }
.odin-source i { margin: 0 2px; font-style: normal; color: #77918e; }
.odin-marker { position: absolute; width: 0; height: 0; pointer-events: none; }
.odin-marker svg { position: absolute; left: -24px; top: -24px; overflow: visible; filter: drop-shadow(0 1px 2px #082527); }
.odin-marker path { fill: none; stroke: #6ffff0; stroke-width: 3; stroke-linecap: round; stroke-linejoin: round; }
.odin-marker circle { fill: #0d9187; stroke: #e7fff8; stroke-width: 2.5; }
.odin-marker > span { position: absolute; left: 8px; top: 12px; white-space: nowrap; padding: 2px 5px; border-radius: 4px; color: #b4fff6; background: #102d32e8; font-size: 11px; }
.odin-pose { display: flex; justify-content: space-between; flex-wrap: wrap; gap: 4px 10px; padding: 8px 12px; background: #f6faf7; color: #425f53; font-size: 11px; }
.odin-pose-heading, .odin-coords { display: flex; align-items: center; flex-wrap: wrap; gap: 4px 9px; }
.odin-pose-heading > span { display: inline-flex; align-items: center; gap: 4px; }
.odin-pose-heading i { width: 6px; height: 6px; border-radius: 50%; background: #a2864c; }
.odin-pose-heading .live i { background: #008e7b; }
.odin-pose-heading button { color: #126550; border: 1px solid #96b3a9; min-height: 22px; padding: 1px 6px; }
.odin-coords { font-variant-numeric: tabular-nums; }
</style>
