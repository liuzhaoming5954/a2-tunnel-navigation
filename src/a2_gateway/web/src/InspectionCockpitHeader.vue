<script setup>
import Icon from './Icon.vue'
// 版画随脚本内嵌，现有上位机离线包可直接显示，无需额外静态文件。
import plantMural from './nuclear-plant.svg?inline'

defineProps({
  dateParts: { type: Object, required: true },
})
defineEmits(['sources'])
</script>

<template>
  <header class="inspection-hero">
    <div class="inspection-badge" aria-hidden="true"><b>A2</b></div>
    <div class="inspection-heading">
      <span class="inspection-caption" lang="ru">ПУЛЬТ УПРАВЛЕНИЯ</span>
      <h1>巡检驾驶舱</h1>
      <slot />
    </div>
    <!-- 版画只作装饰；进入手动遥控页后仍需由操作员主动启用会话。 -->
    <img class="inspection-mural" :src="plantMural" width="720" height="118" alt="">
    <div class="inspection-actions">
      <time :datetime="dateParts.iso">{{ dateParts.label }}<span>{{ dateParts.weekday }}</span></time>
      <a class="ic-source-button" href="#/maintenance"><Icon name="gamepad" :size="14" />手动遥控</a>
      <button class="ic-source-button" type="button" aria-haspopup="dialog" @click="$emit('sources')">
        <Icon name="settings" :size="14" />数据来源
      </button>
    </div>
  </header>
</template>

<style scoped>
.inspection-hero {
  position: relative;
  isolation: isolate;
  display: flex;
  align-items: center;
  min-width: 0;
  min-height: 144px;
  overflow: hidden;
  color: var(--ic-ink, #e0e2dc);
}
.inspection-badge {
  position: relative;
  display: grid;
  place-items: center;
  flex: none;
  width: 110px;
  height: 85px;
  margin-right: 22px;
  padding-right: 5px;
  clip-path: polygon(0 0, 100% 0, 100% 77%, 82% 100%, 0 100%);
  transform: skew(-3deg);
  background: var(--ic-action, #e04f16);
  color: #141517;
}
.inspection-badge b {
  padding-bottom: 9px;
  font: 82px/.93 Impact, "Liberation Sans Narrow", sans-serif;
  letter-spacing: -.045em;
  transform: scaleX(1.1) skew(-4deg);
}
.inspection-badge::after {
  content: "";
  position: absolute;
  inset: auto 25px 9px 12px;
  height: 3px;
  border-block: 1px solid #121417;
}
.inspection-heading { min-width: 0; padding-bottom: 5px; }
.inspection-caption {
  display: flex;
  align-items: center;
  gap: 10px;
  color: #a4b3b9;
  font: 600 10px/1.1 "Liberation Mono", monospace;
  letter-spacing: .18em;
}
.inspection-caption::before { content: ""; width: 26px; height: 2px; background: var(--ic-action, #e04f16); }
.inspection-heading h1 {
  margin: 6px 0 0;
  color: #e1e1d9;
  font: 900 48px/1.24 "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
  letter-spacing: .035em;
  white-space: nowrap;
}
/* 宽屏给日期和按钮预留独立区域，版画沿地图上边缘收底。 */
.inspection-mural {
  position: absolute;
  z-index: -1;
  right: clamp(145px, 13vw, 184px);
  bottom: 0;
  width: min(754px, calc(100% - 445px));
  height: auto;
  pointer-events: none;
}
.inspection-actions {
  display: grid;
  justify-items: end;
  gap: 10px;
  margin-left: auto;
}
.inspection-actions time {
  color: #a6afb1;
  font: 12px/1.45 "Liberation Mono", monospace;
  letter-spacing: .04em;
}
.inspection-actions time span {
  margin-left: 9px;
  font: 11px/1.45 "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
}
.inspection-actions .ic-source-button {
  display: flex;
  align-items: center;
  gap: 9px;
  padding: 7px 11px;
  border: 1px solid #5c6265;
  border-radius: 0;
  background: #25292c;
  color: var(--ic-ink, #e0e2dc);
  font: 550 12px/1.45 "Noto Sans CJK SC", "Microsoft YaHei", sans-serif;
  white-space: nowrap;
  text-decoration: none;
}
.inspection-actions .ic-source-button:hover { border-color: #b8c0c8; background: #353a40; }
.inspection-actions .ic-source-button:focus-visible { outline: 2px solid #e04f16; outline-offset: 2px; }
@media (max-width: 1000px) {
  .inspection-hero { min-height: 118px; }
  .inspection-badge { width: 81px; height: 66px; margin-right: 15px; }
  .inspection-badge b { font-size: 64px; }
  .inspection-heading h1 { font-size: 36px; }
}
@media (max-width: 650px) {
  /* 手机端把版画放在标题下方的空白带，让按钮和标题各自占有完整的点击或阅读区域。 */
  .inspection-hero { min-height: 112px; padding: 9px 0 40px; }
  .inspection-mural { right: 0; width: min(320px, 88%); max-height: 43px; object-fit: contain; object-position: right bottom; }
  .inspection-badge { width: 48px; height: 43px; margin-right: 10px; padding-right: 2px; }
  .inspection-badge b { font-size: 39px; padding-bottom: 4px; }
  .inspection-badge::after { inset: auto 12px 4px 6px; }
  .inspection-heading { padding-bottom: 0; }
  .inspection-caption { font-size: 8px; gap: 5px; letter-spacing: .06em; }
  .inspection-caption::before { width: 12px; }
  .inspection-heading h1 { font-size: clamp(24px, 5vw, 30px); margin-top: 4px; letter-spacing: .01em; }
  .inspection-actions time { display: none; }
  .inspection-actions .ic-source-button { min-height: 40px; padding: 5px 8px; gap: 5px; }
}
</style>
