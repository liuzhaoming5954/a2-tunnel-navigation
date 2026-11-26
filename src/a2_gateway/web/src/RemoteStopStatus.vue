<script setup>
import { computed } from 'vue'

// 上位机轮询层已经按浏览器单调时钟撤下过期样本；本组件只解释确实读到的两项状态。
const props = defineProps({ report: { type: Object, required: true } })
const guardLabel = computed(() => ({ running: '运行中', stopped: '未运行' })[props.report.guard] ?? '未知')
const latchLabel = computed(() => {
  if (props.report.latched === true) return '停车已锁存'
  if (props.report.latched === false) return '停车未锁存'
  return '停车锁存未知'
})
</script>

<template>
  <div class="remote-stop-readings" role="status" aria-label="PC2 停车守护与锁存">
    <span class="guard-reading">PC2 停车守护 <b>{{ guardLabel }}</b></span>
    <span class="latch-reading" :class="{ latched: report.latched === true }">{{ latchLabel }}</span>
  </div>
</template>

<style scoped>
.remote-stop-readings {
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  gap: 3px 10px;
  margin-top: 7px;
  color: var(--ic-muted, var(--muted));
  font-size: 12px;
  line-height: 1.45;
}
.guard-reading, .latch-reading { white-space: nowrap; }
.guard-reading b { margin-left: 2px; color: var(--ic-ink, var(--text)); font-weight: 600; }
.latch-reading { padding-left: 9px; border-left: 1px solid currentColor; }
.latch-reading.latched { color: var(--ic-focus, var(--amber)); font-weight: 650; }
@media (max-width: 480px) {
  .remote-stop-readings { min-width: 0; gap: 2px 6px; }
  .latch-reading { padding-left: 0; border-left: 0; }
}
</style>
