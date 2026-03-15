<script setup>
import { computed } from 'vue'
import { useRobotSSE } from '../composables/useRobotSSE.js'

const { globalRatio, connected } = useRobotSSE()

const props = defineProps({
  size: { type: Number, default: 145 }
})

const step = 5
const min = 0
const max = 100

const displayValue = computed(() => connected.value ? Math.round(globalRatio.value) : 0)

// Arc calculations (same as CircularGauge)
const radius = computed(() => (props.size - 12) / 2)
const circumference = computed(() => 2 * Math.PI * radius.value)
const progress = computed(() => {
  const percent = Math.min(Math.max((displayValue.value - min) / (max - min), 0), 1)
  return percent * 0.7
})
const strokeDasharray = computed(() => `${progress.value * circumference.value} ${circumference.value}`)
const bgStrokeDasharray = computed(() => `${0.7 * circumference.value} ${circumference.value}`)

async function sendRatio(value) {
  try {
    await fetch('/api/lua', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ script: `robot.set_velocity_scaling(${value / 100})` })
    })
  } catch (e) {
    console.error('Failed to set global ratio:', e)
  }
}

function decrease() {
  const next = Math.max(min, globalRatio.value - step)
  sendRatio(next)
}

function increase() {
  const next = Math.min(max, globalRatio.value + step)
  sendRatio(next)
}
</script>

<template>
  <div class="speed-gauge" :style="{ width: size + 'px', height: size + 'px' }">
    <svg :width="size" :height="size" class="gauge-svg">
      <defs>
        <filter id="glow-speed" x="-50%" y="-50%" width="200%" height="200%">
          <feGaussianBlur stdDeviation="3" result="coloredBlur"/>
          <feMerge>
            <feMergeNode in="coloredBlur"/>
            <feMergeNode in="SourceGraphic"/>
          </feMerge>
        </filter>
      </defs>
      <!-- Background arc -->
      <circle
        :cx="size / 2" :cy="size / 2" :r="radius"
        fill="none" stroke="rgba(59, 130, 246, 0.15)" :stroke-width="6"
        stroke-linecap="round" :stroke-dasharray="bgStrokeDasharray"
        :transform="`rotate(135 ${size/2} ${size/2})`"
      />
      <!-- Progress arc -->
      <circle
        :cx="size / 2" :cy="size / 2" :r="radius"
        fill="none" stroke="#3b82f6" :stroke-width="6"
        stroke-linecap="round" :stroke-dasharray="strokeDasharray"
        :transform="`rotate(135 ${size/2} ${size/2})`"
        class="progress-arc" filter="url(#glow-speed)"
      />
    </svg>
    <div class="gauge-content">
      <span class="gauge-label">Speed</span>
      <div class="gauge-controls">
        <button class="ctrl-btn" @click="decrease">&minus;</button>
        <span class="gauge-value">{{ displayValue }}</span>
        <button class="ctrl-btn" @click="increase">+</button>
      </div>
      <span class="gauge-unit">%</span>
    </div>
  </div>
</template>

<style scoped>
.speed-gauge {
  position: relative;
  display: flex;
  align-items: center;
  justify-content: center;
}

.gauge-svg {
  position: absolute;
  top: 0;
  left: 0;
}

.progress-arc {
  transition: stroke-dasharray 0.5s ease;
}

.gauge-content {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  z-index: 1;
  background: rgba(8, 12, 21, 0.9);
  border-radius: 50%;
  width: 76%;
  height: 76%;
  border: 1px solid rgba(59, 130, 246, 0.25);
}

.gauge-label {
  font-size: 10px;
  color: var(--text-secondary);
  text-transform: uppercase;
  letter-spacing: 1px;
  margin-bottom: 2px;
}

.gauge-controls {
  display: flex;
  align-items: center;
  gap: 6px;
}

.gauge-value {
  font-family: 'Oxanium', sans-serif;
  font-size: 28px;
  font-weight: 500;
  color: var(--text-primary);
  line-height: 1;
  min-width: 36px;
  text-align: center;
}

.gauge-unit {
  font-size: 12px;
  color: var(--text-secondary);
  margin-top: 2px;
}

.ctrl-btn {
  width: 22px;
  height: 22px;
  border-radius: 50%;
  border: 1px solid rgba(59, 130, 246, 0.4);
  background: rgba(59, 130, 246, 0.1);
  color: var(--accent-blue, #3b82f6);
  font-size: 16px;
  font-weight: 700;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 0;
  line-height: 1;
  transition: all 0.15s;
}

.ctrl-btn:hover {
  background: rgba(59, 130, 246, 0.25);
  border-color: rgba(59, 130, 246, 0.7);
}

.ctrl-btn:active {
  background: rgba(59, 130, 246, 0.4);
}
</style>
