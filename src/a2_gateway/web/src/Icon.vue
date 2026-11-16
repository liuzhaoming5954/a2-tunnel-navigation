<script setup>
defineProps({ name: { type: String, required: true }, size: { type: Number, default: 18 } })

const paths = {
  logo: ['m5 18 7-12 7 12', 'M8 13h8M12 6V3M5 18H2m17 0h3'],
  radar: ['M20.5 13a8.5 8.5 0 1 1-9.5-9.5', 'M16 13a4 4 0 1 1-5-5', 'm13 11 7-7', 'M18 4h2v2'],
  gamepad: ['M7 7h10a4 4 0 0 1 3.8 2.8l1 4.4a3 3 0 0 1-4.8 3.1L15 15H9l-2 2.3a3 3 0 0 1-4.8-3.1l1-4.4A4 4 0 0 1 7 7Z', 'M7 10v4m-2-2h4m6 0h.01M18 10h.01'],
  pulse: ['M3 12h4l2.5-6 4.5 12 2.5-6H21'],
  compass: ['M12 22a10 10 0 1 0 0-20 10 10 0 0 0 0 20Z', 'm15.5 8.5-2 5-5 2 2-5 5-2Z'],
  stack: ['m12 3 9 5-9 5-9-5 9-5Zm-9 9 9 5 9-5M3 16l9 5 9-5'],
  power: ['M12 3v8m-5-6a8 8 0 1 0 10 0'],
  up: ['M12 19V5m-6 6 6-6 6 6'],
  down: ['M12 5v14m-6-6 6 6 6-6'],
  left: ['M15 5H9a4 4 0 0 0-4 4v9m-4-4 4 4 4-4'],
  right: ['M9 5h6a4 4 0 0 1 4 4v9m-4-4 4 4 4-4'],
  stop: ['M7 3h10l4 4v10l-4 4H7l-4-4V7l4-4Z', 'M9 9h6v6H9z'],
  keyboard: ['M3 6h18v12H3zM6 9h.01M9 9h.01M12 9h.01M15 9h.01M18 9h.01M6 12h.01M9 12h.01M12 12h.01M15 12h.01M18 12h.01M8 15h8'],
  plus: ['M12 5v14M5 12h14'],
  minus: ['M5 12h14'],
  fit: ['M8 4H4v4m12-4h4v4M4 16v4h4m12-4v4h-4', 'M9 9h6v6H9z'],
  center: ['M12 7V3m0 18v-4m5-5h4M3 12h4', 'M12 17a5 5 0 1 0 0-10 5 5 0 0 0 0 10Z'],
  grid: ['M4 4h16v16H4zM4 12h16M12 4v16'],
  trail: ['M7 17h5a3 3 0 0 0 0-6h-2a3 3 0 0 1 0-6h5', 'M18 5h.01M4 17h.01'],
  battery: ['M3 7h16v10H3zM21 10v4M6 10h7v4H6z'],
  speed: ['M4.9 19a9 9 0 1 1 14.2 0Z', 'm12 14 3.5-4.5M6.5 15h.01M17.5 15h.01M12 8h.01'],
  clock: ['M12 22a10 10 0 1 0 0-20 10 10 0 0 0 0 20Z', 'M12 6v6l4 2'],
  list: ['M8 6h12M8 12h12M8 18h12M4 6h.01M4 12h.01M4 18h.01'],
  clear: ['M4 7h16m-13 0 1 13h8l1-13M9 7V4h6v3m-4 4v5m2-5v5'],
  info: ['M12 22a10 10 0 1 0 0-20 10 10 0 0 0 0 20Z', 'M12 11v5m0-8h.01'],
  shield: ['M12 22s7-3.5 7-10V5l-7-3-7 3v7c0 6.5 7 10 7 10Z', 'm9 12 2 2 4-4'],
  emptyMap: ['m3 6 6-3 6 3 6-3v15l-6 3-6-3-6 3V6Zm6-3v15m6-12v15'],
}
</script>

<template>
  <svg :width="size" :height="size" viewBox="0 0 24 24" fill="none" stroke="currentColor"
       stroke-width="1.7" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
    <path v-for="path in paths[name]" :key="path" :d="path" />
  </svg>
</template>
