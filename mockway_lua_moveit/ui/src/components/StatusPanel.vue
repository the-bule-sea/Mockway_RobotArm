<script setup>
import { computed } from 'vue'
import { useRobotSSE } from '../composables/useRobotSSE.js'

const { connected, errorId, errorMessage } = useRobotSSE()

async function clearError() {
  try {
    await fetch('/api/lua', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ script: 'CleanError()' })
    })
  } catch (e) {
    console.error('Failed to clear error:', e)
  }
}

const communicationItem = computed(() => ({
  title: 'Communication',
  subtitle: connected.value ? 'Connected' : 'Disconnected',
  status: connected.value ? 'check' : 'warning'
}))
</script>

<template>
  <aside class="left-panel">
    <div class="status-item">
      <div :class="['status-icon', communicationItem.status]">
        <svg v-if="connected" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
          <polyline points="20 6 9 17 4 12"></polyline>
        </svg>
        <svg v-else viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
          <line x1="18" y1="6" x2="6" y2="18"></line>
          <line x1="6" y1="6" x2="18" y2="18"></line>
        </svg>
      </div>
      <div class="status-text">
        <span class="status-title">{{ communicationItem.title }}</span>
        <span class="status-subtitle">{{ communicationItem.subtitle }}</span>
      </div>
    </div>

    <!-- Error Panel -->
    <div v-if="errorId !== 0" class="error-panel">
      <div class="error-title">ERROR</div>
      <div class="error-code">Error Code: {{ errorId }}</div>
      <div class="error-message">{{ errorMessage }}</div>
      <button class="clear-error-btn" @click="clearError">Clear Error</button>
    </div>
  </aside>
</template>

<style scoped>
.left-panel {
  display: flex;
  flex-direction: column;
  justify-content: flex-start;
  gap: 0;
  padding-top: 12px;
  padding-bottom: 12px;
}

.status-item {
  display: flex;
  align-items: center;
  gap: 14px;
  padding: 16px 0;
}

.status-icon {
  width: 28px;
  height: 28px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
}

.status-icon.check {
  background: transparent;
  border: 2px solid var(--accent-green);
  color: var(--accent-green);
}

.status-icon.warning {
  background: transparent;
  border: 2px solid var(--accent-orange);
  color: var(--accent-orange);
}

.status-icon svg {
  width: 14px;
  height: 14px;
}

.status-text {
  display: flex;
  flex-direction: column;
  gap: 2px;
}

.status-title {
  font-size: 13px;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.5px;
  color: var(--accent-cyan);
  line-height: 1.3;
}

.status-subtitle {
  font-size: 13px;
  color: var(--text-secondary);
  line-height: 1.2;
}

.error-panel {
  background: rgba(255, 50, 50, 0.08);
  border: 1px solid var(--accent-red, #ef4444);
  border-radius: 8px;
  padding: 15px;
  margin-top: 16px;
  text-align: center;
}

.error-title {
  font-size: 11px;
  color: var(--accent-red, #ef4444);
  text-transform: uppercase;
  letter-spacing: 2px;
  margin-bottom: 10px;
  font-weight: 700;
}

.error-code {
  font-family: 'Oxanium', sans-serif;
  font-size: 16px;
  font-weight: 500;
  color: var(--text-primary);
  margin-bottom: 6px;
}

.error-message {
  font-size: 13px;
  color: var(--text-secondary);
  margin-bottom: 12px;
  line-height: 1.4;
}

.clear-error-btn {
  padding: 6px 18px;
  font-size: 11px;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 1px;
  color: #fff;
  background: var(--accent-red, #ef4444);
  border: none;
  border-radius: 4px;
  cursor: pointer;
  transition: opacity 0.2s;
}

.clear-error-btn:hover {
  opacity: 0.8;
}

</style>
