// 以下为本地合成夹具。点位编号仅在三维示意图内部有效，
// 不含任何实机坐标、传感器阈值或可下发的动作。
export const scenePoints = {
  P01: { name: '入口阀组', purpose: '核对入口阀组标识和仪表外观', asset: 'V-01' },
  P02: { name: '气体监测点', purpose: '记录气体通道读数和采集状态', asset: 'G-02' },
  P03: { name: '回仓交接点', purpose: '核对返回位置和交接记录', asset: 'D-01' },
}

export const shiftTasks = [
  { id: 'MIS-031', title: '入口设备点检', time: '08:30', end: '08:52', status: '已完成', kind: 'done', point: 'P01', finished: 3, total: 3 },
  { id: 'MIS-032', title: '管廊环境巡检', time: '10:00', end: '执行中', status: '进行中', kind: 'active', point: 'P02', finished: 1, total: 3 },
  { id: 'MIS-033', title: '返程与仓位复检', time: '14:30', end: '待执行', status: '待执行', kind: 'queued', point: 'P03', finished: 0, total: 3 },
]

export const sampleAlarms = [
  { id: 'ALM-018', time: '10:12:36', severity: '告警', tone: 'red', title: '气体读数超出示例阈值', source: '气体监测', point: 'P02', detail: '本地样例中，监测点出现一次高于示例规则的读数。', status: '待确认' },
  { id: 'ALM-017', time: '10:06:18', severity: '提醒', tone: 'amber', title: '点位照片待人工复核', source: '巡检载荷', point: 'P01', detail: '初拍照片已归档，待复核人员确认仪表读数是否清晰。', status: '待确认' },
]
