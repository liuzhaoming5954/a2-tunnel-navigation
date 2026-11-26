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
const current = ref(readPage())
const inCockpit = computed(() => current.value === 'cockpit')
const gateway = useGateway(inCockpit)
const { state, map, mapStatus, connection, phase, cockpitState, cockpitConnection, cockpitRemoteStop } = gateway
const info = computed(() => pageInfo(current.value))
// 系统管理菜单保留维护遥控入口，各模块继续使用原有哈希地址。
const menuGroups = groups.map(group => group.title === '系统管理'
  ? { ...group, pages: [...group.pages, ['maintenance', '维护遥控']] }
  : group)
const activeGroup = computed(() => menuGroups.find(group => group.pages.some(([id]) => id === current.value))?.title)
const openGroup = ref(null)
const mobileOpen = ref(false)
const navToggle = ref(null)
const navHome = ref(null)
const navList = ref(null)
const workArea = ref(null)
const compact = window.matchMedia('(max-width: 860px)')

// 轮询成功只说明当前网关可访问。PC2 的灯单独取上位机本次请求的结果，掉线时不沿用上次的在线指示。
const gatewayStatus = computed(() => {
  const snapshot = inCockpit.value ? cockpitState.value : state.value
  const link = inCockpit.value ? cockpitConnection.value : connection.value
  if (link !== 'connected') {
    if (snapshot?.deployment === 'station') return [
      { text: '上位机未连接', tone: 'offline' }, { text: 'PC2 状态未知', tone: 'unknown' },
    ]
    return [{ text: link === 'connecting' ? '网关连接中' : '网关离线', tone: 'unknown' }]
  }
  if (snapshot.deployment === 'station') {
    let remote = { text: 'PC2 状态未知', tone: 'unknown' }
    if (snapshot.pc2_connected === true && snapshot.control?.source === 'mock') {
      remote = { text: 'PC2 模拟底盘已连接', tone: 'simulation' }
    } else if (snapshot.pc2_connected === true) remote = { text: 'PC2 已连接', tone: 'online' }
    else if (snapshot.pc2_reason === 'pc2_invalid_response') remote = { text: 'PC2 接口异常', tone: 'offline' }
    else if (snapshot.pc2_connected === false) remote = { text: 'PC2 未连接', tone: 'offline' }
    return [{ text: '上位机在线', tone: 'online' }, remote]
  }
  if (snapshot.mode === 'simulator') return [{ text: '模拟网关在线', tone: 'simulation' }]
  if (snapshot.mode === 'robot_readonly') return [{ text: '实机网关 · 只读', tone: 'online' }]
  if (snapshot.mode === 'robot') return [{ text: '实机网关在线', tone: 'online' }]
  return [{ text: '网关来源未知', tone: 'unknown' }]
})

function closeNav(restoreFocus = false) {
  openGroup.value = null
  mobileOpen.value = false
  if (restoreFocus) nextTick(() => navToggle.value?.focus())
}

function handleMenu() {
  if (mobileOpen.value) {
    closeNav(true)
    return
  }
  // 抽屉覆盖维护操作区，先解除当前控制，避免菜单展开期间仍占用会话。
  if (current.value === 'maintenance') void gateway.stop('打开导航，解除维护控制')
  mobileOpen.value = true
  openGroup.value = activeGroup.value
  nextTick(() => navHome.value?.focus())
}

function navigate() {
  closeNav()
  nextTick(() => workArea.value?.focus({ preventScroll: true }))
}

function onHashChange() {
  const next = readPage()
  if (next === current.value) return
  if (current.value === 'maintenance') void gateway.stop('离开维护遥控页面，解除控制')
  current.value = next
  closeNav()
  window.scrollTo(0, 0)
}

function onLayoutChange() {
  closeNav()
}

function onOutsideNav(event) {
  if (!mobileOpen.value && !navList.value?.contains(event.target)) openGroup.value = null
}

function menuKey(event) {
  if (event.key === 'Escape') {
    if (mobileOpen.value) closeNav(true)
    else if (openGroup.value) {
      // 收起桌面子菜单后将焦点还给开启它的按钮，方便键盘连续切换模块。
      navList.value?.querySelector('button[aria-expanded="true"]')?.focus()
      openGroup.value = null
    }
    return
  }
  if (!mobileOpen.value || event.key !== 'Tab') return
  const items = [...navList.value.querySelectorAll('a, button')].filter(item => item.offsetWidth)
  if (document.activeElement === (event.shiftKey ? items[0] : items.at(-1))) {
    event.preventDefault()
    const target = event.shiftKey ? items.at(-1) : items[0]
    target.focus()
  }
}

watch(info, item => { document.title = item.title + ' · A2 管廊巡检工作站' }, { immediate: true })
onMounted(() => {
  compact.addEventListener('change', onLayoutChange)
  document.addEventListener('pointerdown', onOutsideNav)
  document.addEventListener('focusin', onOutsideNav)
  window.addEventListener('hashchange', onHashChange)
  window.addEventListener('keydown', menuKey)
})
onBeforeUnmount(() => {
  compact.removeEventListener('change', onLayoutChange)
  document.removeEventListener('pointerdown', onOutsideNav)
  document.removeEventListener('focusin', onOutsideNav)
  window.removeEventListener('hashchange', onHashChange)
  window.removeEventListener('keydown', menuKey)
})
</script>

<template>
  <div class="portal" :class="{ 'is-cockpit': current === 'cockpit' }">
    <a class="portal-skip" href="#workspace-content" @click.prevent="workArea.focus()">跳至页面内容</a>
    <header class="portal-bar">
      <button ref="navToggle" type="button" class="portal-mobile-toggle" aria-label="打开导航" aria-controls="workspace-nav" :aria-expanded="mobileOpen" @click="handleMenu"><Icon name="menu" /></button>
      <a href="#/cockpit" class="portal-brand" @click="navigate">
        <span class="portal-mark"><Icon name="tunnel" :size="22" /></span>
        <strong>A2 管廊巡检工作站</strong>
      </a>
      <button v-if="mobileOpen" class="portal-scrim" aria-label="关闭导航遮罩" tabindex="-1" @click="closeNav(true)" />
      <nav id="workspace-nav" ref="navList" aria-label="主导航" class="portal-nav-list" :class="{ 'is-open': mobileOpen }">
        <button type="button" class="portal-nav-close" aria-label="关闭导航" @click="closeNav(true)"><Icon name="close" /></button>
        <a ref="navHome" href="#/cockpit" class="portal-nav-tab" :aria-current="current === 'cockpit' ? 'page' : undefined" @click="navigate">驾驶舱</a>
        <div v-for="(group, index) in menuGroups" :key="group.title" class="portal-nav-group">
          <button type="button" class="portal-nav-tab" :class="{ 'is-current': activeGroup === group.title }"
            :aria-expanded="openGroup === group.title" :aria-controls="'nav-group-'+index"
            @click="openGroup = openGroup === group.title ? null : group.title">
            {{ group.title }}<Icon class="portal-chevron" name="chevron" :size="13" />
          </button>
          <div v-show="openGroup === group.title" :id="'nav-group-'+index" class="portal-subnav">
            <a v-for="[id, title] in group.pages" :key="id" :href="'#/'+id" :aria-current="current === id ? 'page' : undefined" @click="navigate">{{ title }}</a>
          </div>
        </div>
      </nav>
      <div class="portal-link-state" role="status" aria-live="polite" aria-atomic="true">
        <span v-for="indicator in gatewayStatus" :key="indicator.text" class="portal-indicator" :class="'is-'+indicator.tone">
          <i aria-hidden="true"></i>{{ indicator.text }}
        </span>
      </div>
    </header>
    <div class="portal-work" :inert="mobileOpen">
      <main id="workspace-content" ref="workArea" class="portal-content" tabindex="-1">
        <aside v-if="current !== 'maintenance' && ['stopping','stop_unconfirmed'].includes(phase)" role="status" class="portal-stop-status">
          {{ phase === 'stopping' ? '正在确认维护控制解除' : '维护控制停止待确认' }} <a href="#/maintenance">查看维护状态</a>
        </aside>
        <InspectionCockpit v-if="inCockpit" :state="cockpitState" :map="map" :connection="cockpitConnection" :map-status="mapStatus" :remote-stop="cockpitRemoteStop" />
        <MaintenanceConsole v-else-if="current === 'maintenance'" :gateway="gateway" />
        <PatrolTask v-else-if="current === 'tasks' && state?.deployment === 'simulator'"
          :state="state" :connection="connection" />
        <!-- 只读站也展示目标表单。部署权限改变时重建组件，上一轮轮询或发令的回执不会更新新页面。 -->
        <SingleGoalTask v-else-if="current === 'tasks' && state?.deployment === 'station'"
          :key="state.single_goal_enabled === true ? 'enabled' : 'readonly'"
          :enabled="state.single_goal_enabled === true" :connection="connection" />
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
