// 本模型使用独立的演示坐标，X 沿管廊、Y 横跨通道、Z 向上。
// 顶点只描述本地合成场景，无坐标变换可以将其接入现场地图。
export const inspectionPoints = [
  { id: 'P01', position: [-6.5, .52, .12] },
  { id: 'P02', position: [1.4, -.24, .12] },
  { id: 'P03', position: [8.2, .48, .12] },
]
export const robotPosition = [-2.7, .55, .79]
export const dockPosition = [10.75, -.78, 2.1]

const primitives = []
// 地面、贴地线和立体设备分层，可避免大片地板的中心深度挡住贴地路线。
let layer = 1
function tone(color, amount) {
  const rgb = parseInt(color.slice(1), 16)
  const channel = shift => Math.min(255, Math.max(0, Math.round(((rgb >> shift) & 255) * amount)))
  return 'rgb(' + [channel(16), channel(8), channel(0)].join(',') + ')'
}
function face(points, color, group = '') {
  const center = points.reduce((sum, point) => sum.map((v, i) => v + point[i] / points.length), [0, 0, 0])
  primitives.push({ points, center, color, group, layer })
}
function line(points, color, width = 1, group = '') {
  // 细线按自身深度参加排序，因此远侧支架不会盖住前景管道。
  const center = points[0].map((value, i) => (value + points.at(-1)[i]) * .5)
  primitives.push({ points, center, color, group, width, layer })
}

function box(x, y, z, length, breadth, height, color) {
  const x1 = x + length, y1 = y + breadth, z1 = z + height
  const a = [x, y, z], b = [x1, y, z], c = [x1, y1, z], d = [x, y1, z]
  const e = [x, y, z1], f = [x1, y, z1], g = [x1, y1, z1], h = [x, y1, z1]
  face([a,b,f,e], tone(color,.69))
  face([c,d,h,g], tone(color,.83))
  face([d,a,e,h], tone(color,.98))
  face([b,c,g,f], tone(color,.67))
  face([e,f,g,h], tone(color,1.21))
}
function pipeAlongX(start, end, y, z, radius, color, closeEnds = false) {
  const count = 16
  const ring = (x, k) => [x, y + radius * Math.cos(k), z + radius * Math.sin(k)]
  for (let n = 0; n < count; n++) {
    const t = n * 2 * Math.PI / count, next = (n + 1) * 2 * Math.PI / count
    const mid = (t + next) * .5
    // 圆管外壁保留圆周方向的高光和暗部，切口另加法兰及深色内腔。
    const light = .77 + .28 * Math.sin(mid) + .13 * Math.cos(mid) + .2 * Math.max(0, Math.sin(mid + .35)) ** 12
    face([ring(start,t),ring(end,t),ring(end,next),ring(start,next)], tone(color,light))
  }
  if (closeEnds) {
    for (const x of [start, end]) {
      const points = Array.from({ length: count }, (_, n) => ring(x, n * 2 * Math.PI / count))
      face(points, tone(color,.60))
    }
  }
}
function ringFace(x, y, z, radius, color) {
  face(Array.from({ length: 16 }, (_, n) => [x, y + radius * Math.cos(n*Math.PI/8), z + radius * Math.sin(n*Math.PI/8)]), color)
}
function rod(a, b, radius, color) {
  // 关节连杆使用封闭侧面的细方柱，近远两对腿可以随视角分辨。
  const dx = b[0] - a[0], dz = b[2] - a[2], len = Math.hypot(dx,dz) || 1
  const around = p => [[-1,-1],[1,-1],[1,1],[-1,1]].map(([s,t]) => [p[0]+s*radius*dz/len,p[1]+t*radius,p[2]-s*radius*dx/len])
  const left = around(a), right = around(b)
  for (let i = 0; i < 4; i++) face([left[i],right[i],right[(i+1)%4],left[(i+1)%4]], tone(color,[.8,.98,1.16,.72][i]))
}

// 坑壁保留背面和两端的剖切轮廓；步道前缘降低，露出巡检路线。
for (let i = 0; i < 12; i++) {
  const x = -12 + i * 2
  layer = 0
  box(x,-2.74,-.49,2,5.48,.46,'#416077')
  box(x+.016,-2.71,-.03,1.968,5.38,.07, i % 2 ? '#395568' : '#3b586b')
  layer = 1
  box(x,-2.75,0,2,.25,2.95,'#304d65')
  box(x,-2.77,2.95,2,.30,.12,'#5d778c')
  box(x,2.52,.04,2,.21,.2,'#577185')
  line([[x+.02,-2.489,.08],[x+.02,-2.489,2.9]],'#688294',.68)
  line([[x+.03,2.51,.07],[x+1.96,2.51,.07]], '#89a2ae',.65)
  // 地面板缝、检修沟和近侧排水格栅提供可辨识的尺寸参照。
  layer = .3
  for (let j = 0; j < 6; j++) {
    const gx = x + j/3
    line([[gx,1.62,.07],[gx,2.02,.07]],'#172e41',1.15)
  }
  for (const y of [1.61,2.03,-.92]) line([[x,y,.07],[x+2,y,.07]],'#73909b',.58)
}

layer = 1

// 后墙三根主管采用蓝钢、青灰和铜色；靠近观测侧的低位管和支架保持开放视线。
const pipes = [
  { y:-1.87,z:1.98,r:.35,color:'#7298af' }, { y:-1.64,z:1.20,r:.245,color:'#a86f4c' },
  { y:-1.65,z:.55,r:.17,color:'#527b88' }, { y:2.12,z:.64,r:.17,color:'#648da1' },
]
for (const p of pipes) {
  for (let i=0;i<24;i++) pipeAlongX(-12+i,-11+i,p.y,p.z,p.r,p.color)
  for (let x=-11.98;x<12.08;x+=3) {
    pipeAlongX(x-.058,x+.058,p.y,p.z,p.r+.048, '#a2acac',true)
    for (const offset of [-.078,.07]) pipeAlongX(x+offset, x+offset+.018,p.y,p.z,p.r+.053,'#4d6875')
  }
  // 剖切端可看见厚管壁和暗色内腔，避免端面像实心圆钢。
  for (const end of [-12.09,12.1]) {
    ringFace(end,p.y,p.z,p.r+.042,'#92a4aa')
    ringFace(end + Math.sign(end)*.003,p.y,p.z,p.r*.80,'#142b3b')
  }
}
for (let x=-11.8;x<12;x+=3.92) {
  // 支架、悬臂与灯带的重复节距，体现走廊的纵向延伸。
  box(x,-2.49,.08,.13,.15,2.9,'#728b9a')
  box(x,-2.45,1.56,.16,.91,.11,'#627885')
  box(x,-2.45,.84,.16,1.02,.09,'#5d7886')
  box(x,1.99,.09,.14,.25,.38,'#6a8190')
  box(x,-2.46,2.82,.17,1.17,.13,'#667e91')
  box(x+.25,-2.46,2.57,1.11,.08,.09,'#b0d7d8')
}
// 两端保留顶梁截面。梁身采用灰蓝，标识条采用偏暖的安全色。
for (const x of [-11.8,11.71]) {
  box(x,2.37,.06,.20,.19,2.89,'#658095')
  box(x,-2.55,2.99,.25,5.12,.20,'#678095')
  for (let j=0;j<4;j++) box(x+.005,2.553,.45+j*.28,.18,.015,.12,'#b99654')
}

// 路面标线沿各段绘成有宽度的面片，可作为独立图层开关。
layer = .5
const route = [[-10.1,.52],[-6.5,.52],[-3.4,.52],[-1.8,-.24],[1.4,-.24],[4.5,-.24],[6,.48],[8.2,.48],[10.7,.48]]
for (let n=1;n<route.length;n++) {
  const [x0,y0]=route[n-1], [x1,y1]=route[n], len=Math.hypot(x1-x0,y1-y0)
  const count=Math.ceil(len/.65)
  for (let j=0;j<count;j++) {
    const t=j/count,u=(j+1)/count
    for (const [half,z,color] of [[.14,.083,'#397a75'],[.066,.091,'#5af2c3']]) {
      const px=-(y1-y0)/len*half, py=(x1-x0)/len*half
      const ax=x0+t*(x1-x0), ay=y0+t*(y1-y0), bx=x0+u*(x1-x0), by=y0+u*(y1-y0)
      face([[ax+px,ay+py,z],[bx+px,by+py,z],[bx-px,by-py,z],[ax-px,ay-py,z]],color,'route')
    }
  }
}

layer = 1

// 仓体位于剖开段的端部。侧板、门框、顶部检修条与开口指示灯可区分仓和机器人。
function dockBox(x, y, z, length, breadth, height, color) {
  box(11.48 - x - length, y, z, length, breadth, height, color)
}
dockBox(0,-1.29,.07,1.58,1.56,.10,'#8397a0')
dockBox(.02,-1.27,.16,.15,1.5,1.63,'#7a8b96')
dockBox(.05,-1.28,.16,1.52,.12,1.63,'#8498a3')
dockBox(.05,.11,.16,1.52,.12,1.63,'#7e939b')
dockBox(0,-1.30,1.79,1.67,1.60,.15,'#8c9da5')
dockBox(1.57,-1.26,.24,.08,.12,1.40,'#a3bbc1')
dockBox(1.57,.10,.24,.08,.12,1.40,'#a3bbc1')
dockBox(1.60,-1.1,1.68,.06,1.12,.08,'#5af2c3')
for (let i=0;i<5;i++) dockBox(.25+i*.20,-.98,1.944,.07,.87,.016,'#5e7785')

// 入口阀组用环形阀轮表示，监测点在背墙安装带状态灯的采集盒。
for (const vx of [-6.8,-5.93]) {
  box(vx-.07,-1.91,2.26,.14,.14,.32,'#a7b3af')
  const rim = Array.from({length:25}, (_,i) => [vx+.29*Math.cos(i*Math.PI/12),-1.84+.29*Math.sin(i*Math.PI/12),2.59])
  line(rim,'#d99873',1.65)
  line([[vx-.28,-1.84,2.59],[vx+.28,-1.84,2.59]],'#b77c5a',1)
}
box(1.16,-2.43,2.12,.52,.24,.54,'#b3c0b2')
box(1.24,-2.17,2.39,.34,.02,.13,'#547c79')
box(1.26,-2.148,2.418,.08,.012,.045,'#65e1ad')

// 四足模型以 A2 的示意比例表达机身、传感头和四根分段腿，不作为实机结构模型或标定数据。
const [rx, ry] = robotPosition
box(rx-.64,ry-.30,.63,1.27,.61,.32,'#bdc7bb')
box(rx-.48,ry-.27,.94,.89,.54,.115,'#d9e1d2')
box(rx+.43,ry-.24,.87,.29,.49,.26,'#a3b5b8')
box(rx+.706,ry-.22,.94,.036,.43,.13,'#273d48')
box(rx+.74,ry-.16,.986,.025,.11,.047,'#66edd0')
for (const s of [-1,1]) for (const d of [-1,1]) {
  const vx=rx+.44*d, vy=ry+s*.32
  const knee=[vx+.21*d,vy+s*.08,.38], toe=[vx-.14*d,vy+s*.13,.12]
  rod([vx,vy,.75],knee,.071,'#c2cec6')
  rod(knee,toe,.046,'#758998')
  box(toe[0]-.10,toe[1]-.055,.065,.21,.11,.07,'#233744')
}

export const tunnelPrimitives = primitives
