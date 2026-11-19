// 哈希路由由浏览器维护历史，机载 Python 静态服务无需配置路径回退。
// 入口只声明本期框架涉及的业务。其余模块显示接入状态，不挂接任何写操作。
export const groups = [
  { title: '巡检作业', icon: 'clipboard', pages: [
    ['tasks', '任务管理', '任务计划与执行记录', '按任务查看执行状态、点位进度和采集结果。驾驶舱可预览当班任务的示例详情。'],
    ['routes', '路线规划', '路线库与地图版本', '在同一地图版本上管理往返路线和巡检点序。'],
    ['points', '巡检点位', '点位与预设动作', '维护设备点位、采集方向、载荷动作与识别配置。'],
  ] },
  { title: '现场资源', icon: 'layers', pages: [
    ['video', '视频管理', '监控点与录像', '按管廊区段组织监控点位、实况与历史录像。'],
    ['assets', '设备台账', '设备档案', '关联设施、巡检点位及历次巡检成果。'],
    ['dock', '回仓记录', '机器人与充电仓', '按机器人和充电仓核对入仓、充电及故障记录。'],
  ] },
  { title: '数据与分析', icon: 'analysis', pages: [
    ['photos', '巡检照片', '点位照片与复核', '按点位查看原图、补拍记录和人工复核意见。'],
    ['gas', '气体数据', '气体通道与样本', '按时间和通道追溯采样值、有效性及采集位置。'],
    ['datasets', '数据集管理', '数据集与标注', '管理样本、标签字典和标注版本。'],
    ['algorithms', '算法管理', '算法版本与点位绑定', '管理算法版本、适用点位和识别结果。'],
  ] },
  { title: '处置与报告', icon: 'chart', pages: [
    ['alerts', '告警中心', '历史告警与处置', '查询告警的发生、恢复、证据和阅处记录。驾驶舱可预览示例告警。'],
    ['reports', '巡检报表', '任务与周期报表', '按日、周、年汇总巡检成果，并追溯到单次执行。'],
  ] },
  { title: '系统管理', icon: 'settings', pages: [
    ['integration', '开放接口', '第三方平台接入', '管理应用授权、接口说明和门户接入。'],
    ['administration', '账号与配置', '人员、权限与审批', '管理人员账号、角色、审批流程和系统参数。'],
  ] },
]

const pages = new Map([
  ['cockpit', { title: '巡检驾驶舱', section: '工作台', icon: 'cockpit' }],
  ['maintenance', { title: '维护遥控', section: '维护工具', icon: 'gamepad' }],
])
for (const group of groups) {
  for (const [id, title, subject, description] of group.pages) {
    pages.set(id, { title, subject, description, section: group.title, icon: group.icon })
  }
}

export function readPage() {
  const id = window.location.hash.replace(/^#\/?/, '')
  return id || 'cockpit'
}

export function pageInfo(id) {
  return pages.get(id) ?? { title: '页面不存在', section: '工作台', icon: 'compass' }
}
