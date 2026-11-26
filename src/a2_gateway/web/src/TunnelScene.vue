<script setup>
import { computed, onBeforeUnmount, onMounted, reactive, ref, watch } from 'vue'
import { dockPosition, inspectionPoints, robotPosition, tunnelPrimitives } from './tunnelGeometry.js'
import { scenePoints } from './cockpitData.js'

const props = defineProps({ selectedPoint: { type: String, default: '' } })
const emit = defineEmits(['select-point'])
const initialView = { yaw: -.40, pitch: .48, zoom: 1 }
const view = reactive({ ...initialView, width: 640, height: 300 })
const viewport = ref(null), canvas = ref(null)
const dragging = ref(false), route = ref(true), scatter = ref(false)
let observer, frame, gesture

// 直接正交投影合成模型。本组件不接收 ROS 位姿，切换二维地图时也不写入任何定位数据。
const projection = computed(() => {
  const cx = Math.cos(view.yaw), sx = Math.sin(view.yaw)
  const cp = Math.cos(view.pitch), sp = Math.sin(view.pitch)
  const raw = ([x,y,z]) => [(x*cx - y*sx), (x*sx + y*cx)*sp - z*cp, (x*sx + y*cx)*cp + z*sp]
  // 以当前包围盒自适应缩放，保证宽屏和窄屏默认都能看到完整管廊。
  const hull = []
  for (const x of [-12.3,12.3]) for (const y of [-2.85,2.85]) for (const z of [-.55,3.3]) hull.push(raw([x,y,z]))
  const left = Math.min(...hull.map(p=>p[0])), right = Math.max(...hull.map(p=>p[0]))
  const top = Math.min(...hull.map(p=>p[1])), bottom = Math.max(...hull.map(p=>p[1]))
  const fit = Math.min((view.width-40)/(right-left), (view.height-70)/(bottom-top))
  const scale = Math.max(1, fit*view.zoom)
  const ox = view.width*.5 - (left+right)*.5*scale
  const oy = view.height*.52 - (top+bottom)*.5*scale
  return { scale, project(point) {
    const [x,y,depth] = raw(point)
    return [ox+x*scale, oy+y*scale, depth]
  } }
})

function place(point) {
  const [x,y] = projection.value.project(point)
  return { left: x+'px', top: y+'px' }
}
const markers = computed(() => inspectionPoints.map(p => ({ ...p, style: place(p.position) })))
const dogTag = computed(() => place([robotPosition[0],robotPosition[1],1.25]))
const dockTag = computed(() => place(dockPosition))
const axisLines = computed(() => {
  // 提供随视角改变的小坐标轴，便于在拖动后确认管廊方向。
  const {project,scale} = projection.value, center = project([0,0,0])
  return [[1,0,0,'X','#6bb0c0'],[0,1,0,'Y','#70baab'],[0,0,1,'Z','#839eb9']].map(([x,y,z,name,color]) => {
    const p = project([x,y,z])
    return { x:(p[0]-center[0])/scale, y:(p[1]-center[1])/scale, name, color }
  })
})

function draw() {
  frame = 0
  const element = canvas.value
  if (!element) return
  const ctx = element.getContext('2d')
  if (!ctx) return
  const {width,height} = view
  const dpr = Math.min(window.devicePixelRatio || 1, 2)
  // 只有像素尺寸真正变化才重建画布，拖动及开关图层均复用缓存。
  const cw = Math.round(width*dpr), ch = Math.round(height*dpr)
  if (element.width !== cw || element.height !== ch) { element.width = cw; element.height = ch }
  ctx.setTransform(dpr,0,0,dpr,0,0)
  ctx.clearRect(0,0,width,height)
  const { project, scale } = projection.value
  function path(points) {
    ctx.beginPath()
    points.forEach((p,i) => { const [x,y] = project(p); i ? ctx.lineTo(x,y) : ctx.moveTo(x,y) })
  }

  // 网格向边界渐淡；压在地台下的阴影使剖面轮廓与环境清楚分开。
  ctx.save()
  const grid = ctx.createRadialGradient(width*.48,height*.55,0,width*.48,height*.55,width*.73)
  grid.addColorStop(0,'rgba(99,154,173,.24)'); grid.addColorStop(.72,'rgba(78,122,148,.13)'); grid.addColorStop(1,'rgba(70,112,135,0)')
  ctx.strokeStyle = grid; ctx.lineWidth = .65
  for (let i=-38;i<=38;i+=1.4) {
    path([[i,-34,-.68],[i,34,-.68]]); ctx.stroke()
    path([[-42,i,-.68],[42,i,-.68]]); ctx.stroke()
  }
  path([[-12.3,-2.75,-.66],[12.5,-2.75,-.66],[12.5,3.03,-.66],[-12.3,3.03,-.66]])
  ctx.closePath(); ctx.shadowColor='#020a15'; ctx.shadowBlur=14; ctx.shadowOffsetY=5; ctx.fillStyle='#081523'; ctx.fill()
  ctx.restore()

  // 采用从远到近的绘制顺序；管道和地板均切成短节，旋转时维持遮挡关系。
  const ordered = tunnelPrimitives.filter(p => p.group !== 'route' || route.value)
    .map(p => ({ p, depth: project(p.center)[2] })).sort((a,b) => a.p.layer-b.p.layer || a.depth-b.depth)
  for (const {p} of ordered) {
    path(p.points)
    if (p.width) {
      ctx.lineWidth=p.width; ctx.strokeStyle=p.color; ctx.stroke()
    } else {
      ctx.closePath(); ctx.fillStyle=p.color; ctx.fill()
    }
  }
  if (scatter.value) {
    // 确定性生成的采样点仅作图层示例；标注为合成，离线及重绘均得到相同点集。
    ctx.fillStyle='rgba(124,227,223,.57)'
    for (let i=0;i<1420;i++) {
      const x=((i*179)%1423)/1423*24-12, v=((i*479)%1433)/1433
      const onWall=i%3===0
      const [sx,sy]=project([x,onWall?-2.47:(v*5.12-2.56),onWall?v*2.96:.052])
      ctx.fillRect(sx,sy,Math.max(.8,scale*.043),Math.max(.8,scale*.043))
    }
  }
}
function redraw() { if (!frame) frame = requestAnimationFrame(draw) }
function resize() {
  const box=viewport.value?.getBoundingClientRect()
  if (box?.width && box.height) { view.width=box.width; view.height=box.height; redraw() }
}
function clamp(value, min, max) { return Math.min(max,Math.max(min,value)) }
function zoomBy(factor) { view.zoom=clamp(view.zoom*factor,.7,2.25) }
function resetView() { Object.assign(view,initialView) }
// 明细页的“在场景中查看”需要恢复点位图层与完整视角，即使再次定位同一个点。
function showInspectionPoint() {
  route.value = true
  resetView()
}
defineExpose({ showInspectionPoint })
function rotate(event) {
  if (event.button!==0 || !event.isPrimary) return
  event.currentTarget.setPointerCapture(event.pointerId)
  gesture={id:event.pointerId,x:event.clientX,y:event.clientY,yaw:view.yaw,pitch:view.pitch}
  dragging.value=true
}
function move(event) {
  if (gesture?.id!==event.pointerId) return
  view.yaw=gesture.yaw+(event.clientX-gesture.x)*.007
  view.pitch=clamp(gesture.pitch+(event.clientY-gesture.y)*.005,.18,1.13)
}
function release(event) {
  if (gesture?.id===event.pointerId) { gesture=null; dragging.value=false }
}
function wheel(event) { if (event.deltaY !== 0) zoomBy(event.deltaY<0?1.085:1/1.085) }
function key(event) {
  const rotateBy = { ArrowLeft:[-.12,0], ArrowRight:[.12,0], ArrowUp:[0,.08], ArrowDown:[0,-.08] }[event.key]
  if (rotateBy) { event.preventDefault(); view.yaw+=rotateBy[0]; view.pitch=clamp(view.pitch+rotateBy[1],.18,1.13) }
  if (event.key==='Home') { event.preventDefault(); resetView() }
}
onMounted(() => { observer=new ResizeObserver(resize); observer.observe(viewport.value); resize() })
watch([projection,route,scatter],redraw)
onBeforeUnmount(() => { observer?.disconnect(); cancelAnimationFrame(frame); gesture=null })
</script>

<template>
  <section ref="viewport" class="tunnel-scene" aria-label="地下管廊三维合成示意">
    <canvas ref="canvas" class="scene-canvas" :class="{dragging}" tabindex="0"
      aria-label="合成三维剖面。拖动或按方向键旋转，滚轮缩放，Home 键复位。点位可用下方按钮选择。"
      @pointerdown="rotate" @pointermove="move" @pointerup="release" @pointercancel="release" @lostpointercapture="release"
      @wheel.prevent="wheel" @keydown="key" />
    <div class="scene-origin"><i></i>合成场景</div>
    <div class="scene-layers" aria-label="场景图层">
      <button type="button" :aria-pressed="route" :class="{active:route}" @click="route=!route"><span></span>路线</button>
      <button type="button" :aria-pressed="scatter" :class="{active:scatter}" @click="scatter=!scatter"><span></span>点云</button>
    </div>
    <template v-if="route">
      <button v-for="m in markers" :key="m.id" class="scene-point" type="button" :class="{selected:props.selectedPoint===m.id}"
        :style="m.style" :aria-label="m.id+' '+scenePoints[m.id].name" :title="m.id+' '+scenePoints[m.id].name"
        :aria-pressed="props.selectedPoint===m.id" @click="emit('select-point',m.id)"><i></i><b>{{m.id}}</b></button>
    </template>
    <span class="model-label robot" :style="dogTag">A2</span>
    <span class="model-label dock" :style="dockTag">充电仓</span>
    <div class="scene-axes" aria-hidden="true"><svg width="44" height="42" viewBox="0 0 44 42">
      <g v-for="axis in axisLines" :key="axis.name" :stroke="axis.color"><line x1="21" y1="25" :x2="21+13*axis.x" :y2="25+13*axis.y"/>
        <text :x="21+19*axis.x" :y="27+19*axis.y" :fill="axis.color" stroke="none">{{axis.name}}</text></g>
      <circle cx="21" cy="25" r="1.6" fill="#9fb7be"/>
    </svg></div>
    <span class="scene-hint">拖动旋转 · 滚轮缩放</span>
    <div class="scene-tools" aria-label="视图控制">
      <button type="button" title="放大" aria-label="放大" :disabled="view.zoom>=2.25" @click="zoomBy(1.22)">＋</button>
      <button type="button" title="缩小" aria-label="缩小" :disabled="view.zoom<=.7" @click="zoomBy(1/1.22)">−</button>
      <button type="button" title="复位视图" aria-label="复位视图" @click="resetView"><svg viewBox="0 0 18 18" aria-hidden="true"><path d="M3.5 7.4a5.6 5.6 0 1 1 .4 4.3M3.3 3.8v4.1h4.1"/></svg></button>
    </div>
  </section>
</template>

<style scoped>
.tunnel-scene{width:100%;height:100%;min-height:260px;position:relative;overflow:hidden;isolation:isolate;color:#ccd4eb;background:radial-gradient(ellipse at 48% 48%,#2b3555 0%,#1e2841 51%,#151b30 87%);font-family:inherit}
.scene-canvas{position:absolute;inset:0;width:100%;height:100%;cursor:grab;touch-action:none;outline-offset:-2px}
.scene-canvas.dragging{cursor:grabbing}
.scene-origin,.scene-hint,.model-label,.scene-axes{pointer-events:none;position:absolute}
.scene-origin{top:13px;left:13px;display:flex;align-items:center;gap:6px;font-size:10px;letter-spacing:.04em;color:#c2cbe0}
.scene-origin i{width:7px;height:7px;border:1px solid #8999c7;transform:rotate(45deg)}
.scene-layers,.scene-tools{display:flex;align-items:center;gap:0;position:absolute;border:1px solid #596482;border-radius:5px;background:#202941ed;overflow:hidden}
.scene-layers{right:12px;top:10px}
.scene-layers button,.scene-tools button{appearance:none;border:none;border-radius:0;background:transparent;color:#bec8e2;font:inherit;display:flex;align-items:center;justify-content:center;cursor:pointer}
.scene-layers button{height:24px;padding:0 8px;font-size:10px;gap:5px}
.scene-layers button+button,.scene-tools button+button{border-left:1px solid #4b5571}
.scene-layers button span{width:5px;height:5px;border:1px solid #8b98b5;border-radius:50%}
.scene-layers button.active{color:var(--accent-strong);background:#3a4877}
.scene-layers button.active span{border-color:var(--accent);background:var(--accent)}
.scene-axes{bottom:4px;left:7px}
.scene-axes text{font:7px monospace;text-anchor:middle}
.scene-hint{bottom:10px;left:54px;font-size:10px;color:#afbad6}
.scene-tools{bottom:8px;right:11px}
.scene-tools button{width:26px;height:24px;font:17px/1 monospace}
.scene-tools button:disabled{opacity:.4;cursor:default}
.scene-tools button svg{width:14px;height:14px;fill:none;stroke:currentColor;stroke-width:1.5;stroke-linecap:round;stroke-linejoin:round}
.scene-layers button:hover,.scene-tools button:not(:disabled):hover{color:#f1f3ff;background:#465581}
.scene-point{position:absolute;display:flex;align-items:center;justify-content:center;transform:translate(-50%,-50%);width:23px;height:24px;cursor:pointer;background:none;border:0;overflow:visible;z-index:2}
.scene-point i{width:8px;height:8px;border:2px solid #9bf3d6;border-radius:50%;background:#287664;box-shadow:0 0 0 3px #164d46bb}
.scene-point b{position:absolute;left:50%;top:calc(100% + 1px);transform:translateX(-50%);padding:2px 4px;color:#dbe2ff;font:9px/1.05 var(--mono,monospace);font-weight:550;white-space:nowrap;border:1px solid #627196;background:#1c2744ec;border-radius:3px}
.scene-point b::before{content:'';position:absolute;width:1px;height:6px;bottom:100%;left:50%;background:#8799c4}
.scene-point:hover i,.scene-point.selected i{background:#d4dcff;border-color:#f2f4ff;box-shadow:0 0 0 4px #9eaef27a}
.scene-point.selected b{background:#c4d1ff;color:#202840;border-color:#cdd7ff}
.model-label{font:9px var(--mono,monospace);color:#e1e6f7;transform:translate(-50%,calc(-100% - 10px));border-bottom:1px solid #94a1c4;background:#202941ed;padding:2px 4px;white-space:nowrap}
.model-label::after{content:'';position:absolute;top:100%;left:50%;height:9px;border-left:1px solid #a1afcf}
.model-label.dock{font-family:inherit;font-size:9px;color:#d8e0f3}
@media(max-width:540px){.tunnel-scene{min-height:240px}.scene-hint{display:none}}
</style>
