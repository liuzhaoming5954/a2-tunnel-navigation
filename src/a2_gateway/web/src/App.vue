<script setup>
import { computed, nextTick, onBeforeUnmount, onMounted, ref, watch } from 'vue'
import Icon from './Icon.vue'
import InspectionCockpit from './InspectionCockpit.vue'
import MaintenanceConsole from './MaintenanceConsole.vue'
import PatrolTask from './PatrolTask.vue'
import SingleGoalTask from './SingleGoalTask.vue'
import { groups, pageInfo, readPage } from './navigation.js'
import { useGateway } from './useGateway.js'
import './portal.css'

// 只创建一个网关客户端，驾驶舱接收只读快照。维护页卸载时，网关仍可确认停止请求。
const gateway = useGateway()
const { state, map, mapStatus, connection, phase } = gateway
const current = ref(readPage())
const info = computed(() => pageInfo(current.value))
const openGroups = ref(new Set(['巡检作业', '处置与报告', info.value.section]))
const mobileOpen = ref(false)
const navToggle = ref(null)
const navHome = ref(null)
const clock = ref(new Date())
const smallScreen = window.matchMedia('(max-width: 760px)')
let clockTimer
const dateFormat = new Intl.DateTimeFormat('zh-CN', {year:'numeric',month:'2-digit',day:'2-digit', weekday:'short'})
const timeFormat = new Intl.DateTimeFormat('zh-CN', {hour:'2-digit',minute:'2-digit',hour12:false})
const gatewayLabel = computed(() => {
  if (connection.value !== 'connected') return connection.value === 'connecting' ? '网关连接中' : '网关离线'
  if (state.value?.deployment === 'station') {
    if (state.value.pc2_connected) return '上位机在线 · PC2 已连接'
    return state.value.pc2_reason === 'pc2_invalid_response' ? '上位机在线 · PC2 接口异常' : '上位机在线 · PC2 未连接'
  }
  return { simulator:'模拟网关在线', robot_readonly:'实机网关 · 只读', robot:'实机网关在线' }[state.value?.mode] ?? '来源未知'
})

function toggleGroup(title) {
  const open = new Set(openGroups.value)
  if (open.has(title)) open.delete(title)
  else open.add(title)
  openGroups.value = open
}

function closeNav(restoreFocus = false) {
  mobileOpen.value = false
  if (restoreFocus) nextTick(() => navToggle.value?.focus())
}

function handleMenu() {
  mobileOpen.value = !mobileOpen.value
  // 移动端抽屉覆盖维护操作区。先撤销本页控制意图，避免遮挡期间继续占用会话。
  if (mobileOpen.value) {
    if (current.value === 'maintenance') void gateway.stop('打开导航，解除维护控制')
    nextTick(() => navHome.value?.focus())
  }
}

function onHashChange() {
  const next = readPage()
  if (next === current.value) return
  if (current.value === 'maintenance') void gateway.stop('离开维护遥控页面，解除控制')
  current.value = next
  closeNav()
  openGroups.value = new Set([...openGroups.value, info.value.section])
  window.scrollTo(0, 0)
}

function resized(event) {
  if (!event.matches) closeNav()
}

function menuKey(event) {
  if (!mobileOpen.value) return
  if (event.key === 'Escape') { closeNav(true); return }
  if (event.key !== 'Tab') return
  // 导航展开时，键盘焦点循环保持在可见的抽屉条目中。
  const items = [...document.querySelectorAll('.portal-sidebar a, .portal-sidebar button')].filter(el => el.offsetWidth)
  const edge = event.shiftKey ? items[0] : items.at(-1)
  if (edge && document.activeElement === edge) {
    event.preventDefault()
    const nextItem = event.shiftKey ? items.at(-1) : items[0]
    nextItem.focus()
  }
}

watch(info, item => { document.title = item.title + ' · A2 巡检平台' }, { immediate: true })
onMounted(() => {
  clockTimer = setInterval(() => { clock.value = new Date() }, 10000)
  smallScreen.addEventListener('change', resized)
  window.addEventListener('hashchange', onHashChange)
  window.addEventListener('keydown', menuKey)
})
onBeforeUnmount(() => {
  clearInterval(clockTimer)
  smallScreen.removeEventListener('change', resized)
  window.removeEventListener('hashchange', onHashChange)
  window.removeEventListener('keydown', menuKey)
})
</script>

<template>
  <div class="portal">
    <a class="portal-skip" href="#workspace-content" @click.prevent="$refs.workArea.focus()">跳至页面内容</a>
    <button v-if="mobileOpen" class="portal-scrim" aria-label="关闭导航遮罩" tabindex="-1" @click="closeNav(true)" />
    <aside id="workspace-nav" class="portal-sidebar" :class="{ 'is-open': mobileOpen }" aria-label="工作区">
      <a href="#/cockpit" class="portal-brand" @click="closeNav()">
        <span class="portal-mark" aria-hidden="true"><Icon name="tunnel" :size="24" /></span>
        <span class="portal-brand-name"><strong>A2 巡检平台</strong><small>管廊机器人工作台</small></span>
      </a>
      <button type="button" class="portal-nav-close" aria-label="关闭导航" @click="closeNav(true)"><Icon name="close" /></button>
      <nav aria-label="主导航" class="portal-nav-list">
        <div class="portal-nav-label">工作空间</div>
        <a ref="navHome" href="#/cockpit" class="portal-main-link" :aria-current="current === 'cockpit' ? 'page' : undefined" @click="closeNav()">
          <Icon name="cockpit" />巡检驾驶舱
        </a>
        <template v-for="(group, index) in groups" :key="group.title">
          <div v-if="index === 4" class="portal-nav-label portal-nav-rule">平台与维护</div>
          <button type="button" class="portal-group" :aria-expanded="openGroups.has(group.title)" :aria-controls="'nav-group-'+index" @click="toggleGroup(group.title)">
            <Icon :name="group.icon" /><span>{{ group.title }}</span>
            <Icon name="chevron" class="portal-chevron" :class="{expanded: openGroups.has(group.title)}" :size="15" />
          </button>
          <div v-show="openGroups.has(group.title)" :id="'nav-group-'+index" class="portal-subnav">
            <a v-for="[id, title] in group.pages" :key="id" :href="'#/'+id" :aria-current="current === id ? 'page' : undefined" @click="closeNav()">{{ title }}</a>
          </div>
        </template>
        <a href="#/maintenance" class="portal-main-link" :aria-current="current === 'maintenance' ? 'page' : undefined" @click="closeNav()">
          <Icon name="gamepad" />维护遥控
        </a>
      </nav>
      <div class="portal-foot"><span class="portal-foot-icon"><Icon name="terminal" :size="17" /></span><span><strong>本地工作区</strong><small>Unitree A2</small></span></div>
    </aside>
    <div class="portal-work" :inert="mobileOpen">
      <header class="portal-bar">
        <button ref="navToggle" type="button" class="portal-mobile-toggle" aria-label="打开导航" aria-controls="workspace-nav" :aria-expanded="mobileOpen" @click="handleMenu"><Icon name="menu" /></button>
        <div class="portal-crumb"><span>{{ info.section }}</span><Icon name="chevron" :size="12"/><strong>{{ info.title }}</strong></div>
        <time class="portal-time" :datetime="clock.toISOString()"><span>{{ dateFormat.format(clock) }}</span><b>{{ timeFormat.format(clock) }}</b></time>
        <span class="portal-link-state" role="status" :class="{online: connection === 'connected' && state?.pc2_connected !== false}"><i aria-hidden="true"></i>{{ gatewayLabel }}</span>
      </header>
      <main id="workspace-content" ref="workArea" class="portal-content" tabindex="-1">
        <aside v-if="current !== 'maintenance' && ['stopping','stop_unconfirmed'].includes(phase)" role="status" class="portal-stop-status">
          {{ phase === 'stopping' ? '正在确认维护控制解除' : '维护控制停止待确认' }} <a href="#/maintenance">查看维护状态</a>
        </aside>
        <InspectionCockpit v-if="current === 'cockpit'" :state="state" :map="map" :connection="connection" :map-status="mapStatus" />
        <MaintenanceConsole v-else-if="current === 'maintenance'" :gateway="gateway" />
        <PatrolTask v-else-if="current === 'tasks' && state?.deployment === 'simulator'"
          :state="state" :connection="connection" />
        <SingleGoalTask v-else-if="current === 'tasks' && state?.deployment === 'station' && state.single_goal_enabled === true"
          :connection="connection" />
        <section v-else class="portal-reserved">
          <header class="portal-page-heading"><h1>{{ info.title }}</h1><span v-if="info.subject" class="portal-planned">模块待接入</span></header>
          <div class="portal-empty">
            <div class="portal-empty-icon"><Icon :name="info.icon" :size="26" /></div>
            <h2>{{ info.subject || '未找到此页面' }}</h2>
            <p>{{ info.description || '请从主导航选择页面。' }}</p>
            <a class="portal-back" href="#/cockpit">返回巡检驾驶舱 <Icon name="next" :size="15"/></a>
          </div>
        </section>
      </main>
    </div>
  </div>
</template>
