<script setup>
import { computed, onBeforeUnmount, onMounted, reactive, ref, watch } from 'vue'
import Icon from './Icon.vue'

const props = defineProps({
  map: { type: Object, default: null },
  pose: { type: Object, default: null },
  status: { type: String, default: 'idle' },
  connection: { type: String, default: 'connecting' },
})

const viewport = ref(null)
const canvas = ref(null)
const gridEnabled = ref(true)
const trailEnabled = ref(true)
const dragging = ref(false)
const view = reactive({ width: 0, height: 0, scale: 1, fitScale: 1, centerX: 0, centerY: 0 })
const trail = []
let raster = null
let frame
let observer
let drag
let ratio = 1

const validPose = computed(() => {
  const pose = props.pose
  return props.map && pose?.map_id === props.map.id
    && [pose.x_m, pose.y_m, pose.yaw_rad].every(Number.isFinite)
    ? pose
    : null
})
const emptyLabel = computed(() => ({
  loading: '正在加载占据栅格',
  error: '地图加载失败，正在重试',
})[props.status] ?? (props.connection === 'disconnected' ? '等待网关重连' : '等待地图数据'))
const scaleDistance = computed(() => {
  const meters = 75 / view.scale
  const unit = 10 ** Math.floor(Math.log10(meters))
  return (meters >= 5 * unit ? 5 : meters >= 2 * unit ? 2 : 1) * unit
})

function worldToScreen(x, y) {
  return [view.width / 2 + (x - view.centerX) * view.scale, view.height / 2 - (y - view.centerY) * view.scale]
}

function mapCorners(map) {
  const angle = Number.isFinite(map.origin_yaw_rad) ? map.origin_yaw_rad : 0
  const x = map.origin_x_m
  const y = map.origin_y_m
  const width = map.width * map.resolution_m
  const height = map.height * map.resolution_m
  return [[0, 0], [width, 0], [width, height], [0, height]].map(([dx, dy]) => [
    x + dx * Math.cos(angle) - dy * Math.sin(angle),
    y + dx * Math.sin(angle) + dy * Math.cos(angle),
  ])
}

function fitMap() {
  if (!props.map || !view.width || !view.height) return
  const corners = mapCorners(props.map)
  const x = corners.map(point => point[0])
  const y = corners.map(point => point[1])
  const minX = Math.min(...x)
  const maxX = Math.max(...x)
  const minY = Math.min(...y)
  const maxY = Math.max(...y)
  view.centerX = (minX + maxX) / 2
  view.centerY = (minY + maxY) / 2
  view.fitScale = Math.min(Math.max(1, view.width - 76) / (maxX - minX), Math.max(1, view.height - 76) / (maxY - minY))
  view.scale = view.fitScale
  queueDraw()
}

function resize() {
  if (!viewport.value || !canvas.value) return
  const oldWidth = view.width
  const oldHeight = view.height
  view.width = viewport.value.clientWidth
  view.height = viewport.value.clientHeight
  ratio = Math.min(window.devicePixelRatio || 1, 2)
  canvas.value.width = Math.round(view.width * ratio)
  canvas.value.height = Math.round(view.height * ratio)
  if (oldWidth !== view.width || oldHeight !== view.height) fitMap()
  queueDraw()
}

function makeRaster(map) {
  const image = document.createElement('canvas')
  image.width = map.width
  image.height = map.height
  const context = image.getContext('2d')
  const pixels = context.createImageData(map.width, map.height)
  for (let y = 0; y < map.height; y++) {
    for (let x = 0; x < map.width; x++) {
      const occupancy = map.cells[(map.height - 1 - y) * map.width + x]
      const offset = 4 * (y * map.width + x)
      const level = Math.max(0, Math.min(100, occupancy)) / 100
      const color = occupancy < 0 ? [26, 42, 59] : [
        Math.round(34 + 104 * level),
        Math.round(63 + 103 * level),
        Math.round(72 + 109 * level),
      ]
      pixels.data.set([...color, 255], offset)
    }
  }
  context.putImageData(pixels, 0, 0)
  return image
}

function drawWorldGrid(context) {
  if (!gridEnabled.value) return
  const step = scaleDistance.value
  const gridPixels = step * view.scale
  if (!Number.isFinite(gridPixels) || gridPixels < 1) return
  const [originX, originY] = worldToScreen(0, 0)
  const x0 = ((originX % gridPixels) + gridPixels) % gridPixels
  const y0 = ((originY % gridPixels) + gridPixels) % gridPixels
  context.lineWidth = 1
  context.strokeStyle = 'rgba(139, 171, 188, .14)'
  context.beginPath()
  for (let x = x0; x < view.width; x += gridPixels) {
    context.moveTo(x, 0)
    context.lineTo(x, view.height)
  }
  for (let y = y0; y < view.height; y += gridPixels) {
    context.moveTo(0, y)
    context.lineTo(view.width, y)
  }
  context.stroke()
}

function drawRaster(context) {
  if (!raster || !props.map) return
  const map = props.map
  const [x, y] = worldToScreen(map.origin_x_m, map.origin_y_m)
  const pixelScale = map.resolution_m * view.scale
  context.save()
  context.translate(x, y)
  context.rotate(-(map.origin_yaw_rad || 0))
  context.imageSmoothingEnabled = false
  context.drawImage(raster, 0, -map.height * pixelScale, map.width * pixelScale, map.height * pixelScale)
  context.strokeStyle = '#52677b'
  context.lineWidth = 1
  context.strokeRect(0, -map.height * pixelScale, map.width * pixelScale, map.height * pixelScale)
  context.restore()
}

function drawTrail(context) {
  if (!trailEnabled.value || trail.length < 2) return
  context.beginPath()
  context.moveTo(...worldToScreen(trail[0].x_m, trail[0].y_m))
  for (const pose of trail.slice(1)) context.lineTo(...worldToScreen(pose.x_m, pose.y_m))
  context.strokeStyle = 'rgba(87, 221, 191, .62)'
  context.lineWidth = 2.3
  context.lineCap = 'round'
  context.lineJoin = 'round'
  context.stroke()
}

function drawPose(context) {
  const pose = validPose.value
  if (!pose) return
  const [x, y] = worldToScreen(pose.x_m, pose.y_m)
  context.save()
  context.translate(x, y)
  context.beginPath()
  context.arc(0, 0, 24, 0, Math.PI * 2)
  context.fillStyle = 'rgba(89, 224, 194, .10)'
  context.fill()
  context.beginPath()
  context.arc(0, 0, 17, 0, Math.PI * 2)
  context.setLineDash([3, 4])
  context.lineWidth = 1
  context.strokeStyle = 'rgba(103, 244, 211, .55)'
  context.stroke()
  context.setLineDash([])
  context.rotate(-pose.yaw_rad)
  context.beginPath()
  context.moveTo(13, 0)
  context.lineTo(-8, -7.5)
  context.lineTo(-5, 0)
  context.lineTo(-8, 7.5)
  context.closePath()
  context.fillStyle = '#65f3d2'
  context.fill()
  context.strokeStyle = '#092e32'
  context.lineWidth = 1.5
  context.stroke()
  context.restore()
}

function draw() {
  frame = undefined
  const context = canvas.value?.getContext('2d')
  if (!context) return
  context.setTransform(ratio, 0, 0, ratio, 0, 0)
  context.fillStyle = '#0b1726'
  context.fillRect(0, 0, view.width, view.height)
  drawRaster(context)
  drawWorldGrid(context)
  drawTrail(context)
  drawPose(context)
}

function queueDraw() {
  if (frame === undefined) frame = requestAnimationFrame(draw)
}

function zoomAt(factor, x = view.width / 2, y = view.height / 2) {
  if (!props.map) return
  const previous = view.scale
  const next = Math.max(view.fitScale * 0.45, Math.min(view.fitScale * 16, previous * factor))
  const scaleDelta = 1 / previous - 1 / next
  view.centerX += (x - view.width / 2) * scaleDelta
  view.centerY -= (y - view.height / 2) * scaleDelta
  view.scale = next
  queueDraw()
}

function wheel(event) {
  if (!props.map) return
  const bounds = viewport.value.getBoundingClientRect()
  zoomAt(Math.exp(-event.deltaY * 0.0014), event.clientX - bounds.left, event.clientY - bounds.top)
}

function centerPose() {
  if (!validPose.value) return
  view.centerX = validPose.value.x_m
  view.centerY = validPose.value.y_m
  queueDraw()
}

function dragStart(event) {
  if (!props.map || event.button !== 0) return
  drag = { id: event.pointerId, x: event.clientX, y: event.clientY, centerX: view.centerX, centerY: view.centerY }
  dragging.value = true
  canvas.value.setPointerCapture(event.pointerId)
}

function dragMove(event) {
  if (drag?.id !== event.pointerId) return
  view.centerX = drag.centerX - (event.clientX - drag.x) / view.scale
  view.centerY = drag.centerY + (event.clientY - drag.y) / view.scale
  queueDraw()
}

function dragStop(event) {
  if (drag?.id !== event.pointerId) return
  dragging.value = false
  drag = undefined
}

watch(() => props.map, map => {
  raster = map ? makeRaster(map) : null
  trail.length = 0
  if (validPose.value) trail.push(validPose.value)
  fitMap()
  queueDraw()
})

watch(validPose, pose => {
  if (pose) {
    const previous = trail.at(-1)
    if (!previous || Math.hypot(pose.x_m - previous.x_m, pose.y_m - previous.y_m) > 0.012) {
      trail.push(pose)
      if (trail.length > 240) trail.shift()
    }
  }
  queueDraw()
})

watch([gridEnabled, trailEnabled], queueDraw)

onMounted(() => {
  if (props.map) raster = makeRaster(props.map)
  observer = new ResizeObserver(resize)
  observer.observe(viewport.value)
  resize()
})

onBeforeUnmount(() => {
  observer?.disconnect()
  if (frame !== undefined) cancelAnimationFrame(frame)
})
</script>

<template>
  <section class="panel map-panel" aria-labelledby="map-heading">
    <header class="panel-header map-header">
      <div class="panel-title">
        <span class="panel-icon"><Icon name="stack" /></span>
        <div class="title-group">
          <h2 id="map-heading">地图与定位</h2>
          <p v-if="map" class="subline map-id" :title="map.id">{{ map.id }}</p>
        </div>
      </div>
      <div class="map-layer-controls" aria-label="地图图层">
        <button type="button" class="layer-button" :class="{ selected: gridEnabled }" :aria-pressed="gridEnabled" @click="gridEnabled = !gridEnabled">
          <Icon name="grid" :size="14" /> 网格
        </button>
        <button type="button" class="layer-button" :class="{ selected: trailEnabled }" :aria-pressed="trailEnabled" @click="trailEnabled = !trailEnabled">
          <Icon name="trail" :size="14" /> 轨迹
        </button>
      </div>
    </header>
    <div ref="viewport" class="map-viewport" :class="{ dragging, 'has-map': map }">
      <canvas ref="canvas" role="img" aria-label="占据栅格、实时机器人位置和历史轨迹"
              @wheel.prevent="wheel" @pointerdown="dragStart" @pointermove="dragMove"
              @pointerup="dragStop" @pointercancel="dragStop" @lostpointercapture="dragStop" />
      <div v-if="!map" class="map-empty" role="status">
        <Icon name="emptyMap" :size="28" />
        <span>{{ emptyLabel }}</span>
      </div>
      <div v-if="map" class="map-axis" aria-label="地图坐标轴">
        <svg width="48" height="49" viewBox="0 0 48 49" fill="none" aria-hidden="true">
          <path d="M12 33V12m-4 4 4-4 4 4m-4 17h21m-4-4 4 4-4 4" stroke="currentColor" stroke-width="1.2" stroke-linecap="round" stroke-linejoin="round"/>
          <text x="9" y="9" fill="currentColor">Y</text><text x="35" y="37" fill="currentColor">X</text>
        </svg>
      </div>
      <div v-if="map" class="map-tools" aria-label="地图视图工具">
        <button type="button" aria-label="放大地图" title="放大地图" @click="zoomAt(1.28)"><Icon name="plus" :size="17" /></button>
        <button type="button" aria-label="缩小地图" title="缩小地图" @click="zoomAt(1 / 1.28)"><Icon name="minus" :size="17" /></button>
        <button type="button" aria-label="适应地图" title="适应地图" @click="fitMap"><Icon name="fit" :size="17" /></button>
        <button type="button" aria-label="定位到机器人" title="定位到机器人" :disabled="!validPose" @click="centerPose"><Icon name="center" :size="17" /></button>
      </div>
      <div v-if="map" class="map-scale">
        <div class="scale-line" :style="{ width: `${scaleDistance * view.scale}px` }"></div>
        <span>{{ scaleDistance.toPrecision(2).replace(/\.0$/, '') }} m</span>
      </div>
      <span v-if="map" class="map-view-hint">拖动平移 · 滚轮缩放</span>
    </div>
    <footer class="map-footer">
      <div class="map-legend" aria-label="地图图例">
        <span><i class="legend-swatch legend-free"></i>空闲</span>
        <span><i class="legend-swatch legend-occupied"></i>占据</span>
        <span><i class="legend-swatch legend-unknown"></i>未知</span>
        <span v-if="validPose"><i class="legend-swatch legend-robot"></i>机器人</span>
      </div>
      <div v-if="map" class="map-dimensions">{{ map.width }} × {{ map.height }} <span>·</span> {{ map.resolution_m }} m/格</div>
    </footer>
  </section>
</template>
