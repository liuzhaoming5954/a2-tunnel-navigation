// 复用离线预览的正交投影与点深度比较。实测点和 ODIN1 传感器共用同一地图相机。
const HOME_ELEVATION = 0.60
const VERTICAL = Math.PI / 2
const RECORD_BYTES = 16
const littleEndian = new Uint8Array(new Uint32Array([1]).buffer)[0] === 1

// ImageData 的通道顺序固定为 RGBA；按本机端序打包后可以一次写入一个像素。
function pixel(red, green, blue) {
  return littleEndian
    ? (red | (green << 8) | (blue << 16) | 0xff000000) >>> 0
    : ((red << 24) | (green << 16) | (blue << 8) | 255) >>> 0
}
// 图尺按正交相机的水平屏幕轴标定，采用测绘常用的 1、2、5 刻度；放大后自动换单位长度。
function scaleBar(pixelsPerMeter) {
  const target = 78 / pixelsPerMeter
  const decade = 10 ** Math.floor(Math.log10(target))
  const meters = [1, 2, 5, 10].map(value => value * decade)
    .reduce((best, value) => Math.abs(value - target) < Math.abs(best - target) ? value : best)
  return { meters: Number(meters.toPrecision(3)), width: meters * pixelsPerMeter }
}

// 把视口四角反投影到地图水平面，只绘制可见范围的米制网格。正侧视时投影退化成直线，省略网格。
function drawGroundGrid(context, dpr, transform, step) {
  const { width, height } = context.canvas
  const inverse = transform.inverse()
  const xs = [], ys = []
  for (const px of [0, width]) {
    for (const py of [0, height]) {
      const corner = inverse.transformPoint({ x: px, y: py })
      xs.push(corner.x)
      ys.push(corner.y)
    }
  }
  const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys)
  const xStart = Math.ceil(minX / step), xEnd = Math.floor(maxX / step)
  const yStart = Math.ceil(minY / step), yEnd = Math.floor(maxY / step)
  // 当坐标量级已超出整数格号的精度，跳过辅助层，避免格号累加失效而卡住实测点显示。
  if (![xStart, xEnd, yStart, yEnd].every(Number.isSafeInteger)) return
  const line = (x1, y1, x2, y2) => {
    const start = transform.transformPoint({ x: x1, y: y1 })
    const end = transform.transformPoint({ x: x2, y: y2 })
    context.moveTo(start.x, start.y)
    context.lineTo(end.x, end.y)
  }
  for (const major of [true, false]) {
    context.beginPath()
    for (let i = xStart; i <= xEnd; i++) {
      if ((i % 4 === 0) === major) line(i * step, minY, i * step, maxY)
    }
    for (let i = yStart; i <= yEnd; i++) {
      if ((i % 4 === 0) === major) line(minX, i * step, maxX, i * step)
    }
    context.strokeStyle = major ? 'rgba(153,179,205,.18)' : 'rgba(153,179,205,.065)'
    context.lineWidth = (major ? .82 : .65) * dpr
    context.stroke()
  }
}

export function createOdinView(canvas, metadata, buffer, showMarker, showReference = () => {}) {
  const count = metadata.pointCount
  if (!Number.isSafeInteger(count) || count < 1 || buffer.byteLength !== count * RECORD_BYTES) {
    throw new Error('点云文件长度与地图描述不一致')
  }
  const context = canvas.getContext('2d')
  if (!context) throw new Error('浏览器未能创建点云画布')

  // 前十二字节为小端 XYZ，后四字节为 RGB 和填充。原始点和颜色仅在加载时解码一次。
  const data = new DataView(buffer)
  const xyz = new Float32Array(count * 3)
  const colors = new Uint32Array(count)
  let meanX = 0, meanY = 0, meanZ = 0
  let xx = 0, xy = 0, yy = 0
  for (let index = 0; index < count; index++) {
    const offset = RECORD_BYTES * index
    const x = data.getFloat32(offset, true)
    const y = data.getFloat32(offset + 4, true)
    const z = data.getFloat32(offset + 8, true)
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      throw new Error('点云中存在无效坐标')
    }
    xyz[3 * index] = x
    xyz[3 * index + 1] = y
    xyz[3 * index + 2] = z
    colors[index] = pixel(data.getUint8(offset + 12), data.getUint8(offset + 13), data.getUint8(offset + 14))

    // 在线累积 XY 协方差，从实际扫描点推算默认视角，使主体尽量沿屏幕长边展开。
    const dx = x - meanX, dy = y - meanY
    meanX += dx / (index + 1)
    meanY += dy / (index + 1)
    meanZ += (z - meanZ) / (index + 1)
    xx += dx * (x - meanX)
    xy += dx * (y - meanY)
    yy += dy * (y - meanY)
  }
  // 视线在主轴方向基础上略偏转，便于观察走廊两侧的深度差。
  const homeYaw = Math.atan2(2 * xy, xx - yy) / 2 - .077
  let yaw, elevation, pivotX, pivotY, pivotZ, unitsWide, unitsHigh
  let zoom = 1, panX = 0, panY = 0, frame = 0
  let image, rgba, depth
  let pose = null, camera = null

  // 根据最近一次绘图的相机计算传感器原点和地图 XY 航向的屏幕投影。
  // 只有视角变化需要重绘点云，定位更新只移动独立的标记。
  function projectSensor() {
    if (!camera || !pose) {
      showMarker(null)
      return
    }
    const x = pose.x_m - camera.originX, y = pose.y_m - camera.originY, z = pose.z_m - camera.originZ
    const screenX = camera.pixelX(x, y) / camera.dpr
    const screenY = camera.pixelY(x, y, z) / camera.dpr
    // 用同一投影变换两点，二者的屏幕差向量给出箭头方向。
    // 当水平轴与视线重合时，屏幕方向没有定义，此视角只画原点并保留航向读数。
    const forwardX = x + Math.cos(pose.yaw_rad), forwardY = y + Math.sin(pose.yaw_rad)
    const dx = camera.pixelX(forwardX, forwardY) / camera.dpr - screenX
    const dy = camera.pixelY(forwardX, forwardY, z) / camera.dpr - screenY
    showMarker({
      x: screenX, y: screenY,
      inView: screenX >= 0 && screenX < canvas.clientWidth && screenY >= 0 && screenY < canvas.clientHeight,
      headingDeg: Math.hypot(dx, dy) > 1e-6 ? Math.atan2(dy, dx) * 180 / Math.PI : null,
    })
  }
  function schedule() {
    if (!frame) frame = requestAnimationFrame(draw)
  }
  function fit(angle, bounds = null) {
    yaw = homeYaw
    elevation = angle
    const cy = Math.cos(yaw), sy = Math.sin(yaw), se = Math.sin(angle), ce = Math.cos(angle)
    let left = Infinity, right = -Infinity, bottom = Infinity, top = -Infinity
    for (let i = 0; i < xyz.length; i += 3) {
      const x = xyz[i], y = xyz[i + 1], z = xyz[i + 2]
      // 分位边界只参与相机构图。绘制时仍提交全部样本，全图模式也会纳入外围点。
      if (bounds && (x < bounds.min[0] || x > bounds.max[0] || y < bounds.min[1] ||
        y > bounds.max[1] || z < bounds.min[2] || z > bounds.max[2])) continue
      const u = cy * (x - meanX) + sy * (y - meanY)
      const v = se * (-sy * (x - meanX) + cy * (y - meanY)) + ce * (z - meanZ)
      left = Math.min(left, u)
      right = Math.max(right, u)
      bottom = Math.min(bottom, v)
      top = Math.max(top, v)
    }
    if (left === Infinity) return fit(angle)
    const u = (left + right) / 2, v = (top + bottom) / 2
    pivotX = meanX + u * cy - v * sy * se
    pivotY = meanY + u * sy + v * cy * se
    pivotZ = meanZ + v * ce
    unitsWide = Math.max(1e-6, right - left)
    unitsHigh = Math.max(1e-6, top - bottom)
    zoom = 1
    panX = panY = 0
    schedule()
  }

  function draw() {
    frame = 0
    const cssWidth = canvas.clientWidth, cssHeight = canvas.clientHeight
    if (!cssWidth || !cssHeight) return
    // 限制高分屏缓冲总量，只有面板尺寸变化时才重新分配颜色和深度数组。
    const dpr = Math.min(window.devicePixelRatio || 1, 2, Math.sqrt(2500000 / (cssWidth * cssHeight)))
    const width = Math.max(1, Math.round(cssWidth * dpr))
    const height = Math.max(1, Math.round(cssHeight * dpr))
    if (!image || width !== image.width || height !== image.height) {
      canvas.width = width
      canvas.height = height
      image = context.createImageData(width, height)
      rgba = new Uint32Array(image.data.buffer)
      depth = new Float32Array(width * height)
    }
    rgba.fill(0)
    depth.fill(-Infinity)
    const scale = .90 * Math.min(width / unitsWide, height / unitsHigh) * zoom
    const centerX = width / 2 + panX * dpr, centerY = height / 2 + panY * dpr
    const cy = Math.cos(yaw), sy = Math.sin(yaw), ce = Math.cos(elevation), se = Math.sin(elevation)
    const pixelX = (x, y) => centerX + scale * (cy * x + sy * y)
    const pixelY = (x, y, z) => centerY - scale * (se * (-sy * x + cy * y) + ce * z)
    camera = { pixelX, pixelY, dpr, originX: pivotX, originY: pivotY, originZ: pivotZ }
    const size = Math.max(1, Math.round(1.45 * dpr))
    for (let index = 0; index < count; index++) {
      const i = index * 3
      const x = xyz[i] - pivotX, y = xyz[i + 1] - pivotY, z = xyz[i + 2] - pivotZ
      const transverse = -sy * x + cy * y
      const px = Math.floor(pixelX(x, y))
      const py = Math.floor(pixelY(x, y, z))
      if (px < 0 || py < 0 || px >= width || py >= height) continue
      const near = -ce * transverse + se * z
      const endX = Math.min(width, px + size), endY = Math.min(height, py + size)
      for (let row = py; row < endY; row++) {
        for (let col = px; col < endX; col++) {
          const p = row * width + col
          if (near > depth[p]) {
            depth[p] = near
            rgba[p] = colors[index]
          }
        }
      }
    }
    context.putImageData(image, 0, 0)
    // 透明像素露出底层地格，采集点保留原始 RGB。采用反向合成，不需要在每次拖动时读回整张画布。
    context.save()
    context.globalCompositeOperation = 'destination-over'
    if (Math.abs(se) >= .015) {
      const grid = new DOMMatrix([scale * cy, scale * se * sy, scale * sy, -scale * se * cy,
        pixelX(-pivotX, -pivotY), pixelY(-pivotX, -pivotY, metadata.viewBounds.min[2] - pivotZ)])
      // 随缩放成倍切换地格间距，主线每四格一次；网格锚在地图原点，不会随平移滑动。
      const step = 2.5 * 2 ** Math.ceil(Math.log2(18 * dpr / (2.5 * scale * Math.abs(se))))
      drawGroundGrid(context, dpr, grid, step)
    }
    context.restore()
    // 坐标三轴取相机旋转矩阵在屏幕上的分量，与点云及位姿箭头同步旋转。
    showReference({
      scale: scaleBar(scale / dpr),
      axes: [
        { label: 'X', x: cy, y: se * sy, color: '#cf9690' },
        { label: 'Y', x: sy, y: -se * cy, color: '#9bbda7' },
        { label: 'Z', x: 0, y: -ce, color: '#a5bee8' },
      ],
    })
    projectSensor()
  }

  // 以指针位置为缩放锚点，避免放大细部时目标滑离光标。平移使用 CSS 像素。
  function changeZoom(factor, anchorX = 0, anchorY = 0) {
    const next = Math.max(.15, Math.min(100, zoom * factor))
    panX = anchorX - (anchorX - panX) * next / zoom
    panY = anchorY - (anchorY - panY) * next / zoom
    zoom = next
    schedule()
  }
  function rotate(dx, dy) {
    yaw -= dx * .007
    elevation = Math.max(-VERTICAL, Math.min(VERTICAL, elevation + dy * .007))
    schedule()
  }
  function pan(dx, dy) {
    panX += dx
    panY += dy
    schedule()
  }

  fit(HOME_ELEVATION, metadata.viewBounds)
  const observer = new ResizeObserver(schedule)
  observer.observe(canvas)
  return {
    reset: () => fit(HOME_ELEVATION, metadata.viewBounds),
    top: () => fit(VERTICAL, metadata.viewBounds),
    fitAll: () => fit(HOME_ELEVATION),
    zoom: changeZoom, rotate, pan,
    setSensorPose(value) {
      pose = value
      projectSensor()
    },
    centerSensor() {
      if (!pose) return
      pivotX = pose.x_m
      pivotY = pose.y_m
      pivotZ = pose.z_m
      panX = panY = 0
      schedule()
    },
    destroy() {
      observer.disconnect()
      cancelAnimationFrame(frame)
    },
  }
}
