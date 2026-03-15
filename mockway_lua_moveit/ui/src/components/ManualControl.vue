<script setup>
import { ref, computed, onMounted, onBeforeUnmount } from 'vue'
import { useRobotSSE } from '../composables/useRobotSSE'

const emit = defineEmits(['close'])

const { jointAngles, endPose, connected } = useRobotSSE()

const mode = ref('jog')
const space = ref('joint')
const inchDistance = ref(1)
const isMoving = ref(false)
const lastError = ref('')

const inchPresets = [0.1, 0.5, 1, 5, 10]

const JOG_JOINT_VEL = 20    // deg/s
const JOG_LIN_VEL   = 50    // mm/s
const JOG_ROT_VEL   = 20    // deg/s

let jogTimer = null

const jointLabels = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
const cartLabels = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']

const labels = computed(() => space.value === 'joint' ? jointLabels : cartLabels)
const values = computed(() => space.value === 'joint' ? jointAngles.value : endPose.value)
const unit = computed(() => space.value === 'joint' ? '\u00B0' : (inchDistance.value ? 'mm/\u00B0' : ''))

async function sendLua(script) {
  try {
    const res = await fetch('/api/lua', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ script })
    })
    const data = await res.json()
    if (!data.success) {
      lastError.value = data.error || data.message
    } else {
      lastError.value = ''
    }
  } catch (err) {
    lastError.value = 'Connection error: ' + err.message
  }
}

function setMode(m) {
  mode.value = m
  if (m === 'jog') {
    const servoMode = space.value === 'joint' ? 'joint_jog' : 'twist'
    sendLua(`robot.switch_servo_mode("${servoMode}")`)
  }
}

function setSpace(s) {
  space.value = s
  if (mode.value === 'jog') {
    const servoMode = s === 'joint' ? 'joint_jog' : 'twist'
    sendLua(`robot.switch_servo_mode("${servoMode}")`)
  }
}

function handlePress(index, direction) {
  if (isMoving.value) return
  isMoving.value = true
  lastError.value = ''

  const idx = index + 1
  const dir = direction

  if (mode.value === 'jog') {
    const sendJogCmd = () => {
      if (space.value === 'joint') {
        sendLua(`robot.servo_joint(${idx}, ${dir * JOG_JOINT_VEL})`)
      } else {
        const vx  = idx === 1 ? dir * JOG_LIN_VEL : 0
        const vy  = idx === 2 ? dir * JOG_LIN_VEL : 0
        const vz  = idx === 3 ? dir * JOG_LIN_VEL : 0
        const rx  = idx === 4 ? dir * JOG_ROT_VEL : 0
        const ry  = idx === 5 ? dir * JOG_ROT_VEL : 0
        const rz  = idx === 6 ? dir * JOG_ROT_VEL : 0
        sendLua(`robot.servo_cartesian(${vx}, ${vy}, ${vz}, ${rx}, ${ry}, ${rz})`)
      }
    }
    sendJogCmd()
    jogTimer = setInterval(sendJogCmd, 50)
  } else {
    const dist = inchDistance.value
    if (space.value === 'joint') {
      sendLua(`local j = robot.get_joint_positions(); j[${idx}] = j[${idx}] + ${dir * dist}; robot.move_to_joints(j)`)
    } else {
      const dx  = idx === 1 ? dir * dist : 0
      const dy  = idx === 2 ? dir * dist : 0
      const dz  = idx === 3 ? dir * dist : 0
      const drx = idx === 4 ? dir * dist : 0
      const dry = idx === 5 ? dir * dist : 0
      const drz = idx === 6 ? dir * dist : 0
      sendLua(`robot.move_linear_relative(${dx}, ${dy}, ${dz}, ${drx}, ${dry}, ${drz})`)
    }
  }
}

function handleRelease() {
  if (!isMoving.value) return
  isMoving.value = false
  if (jogTimer) {
    clearInterval(jogTimer)
    jogTimer = null
  }
  sendLua('robot.servo_stop()')
}

function formatValue(val) {
  if (val === undefined || val === null) return '---'
  return Number(val).toFixed(2)
}

onMounted(() => {
  sendLua('robot.switch_servo_mode("joint_jog")')
})

onBeforeUnmount(() => {
  if (jogTimer) {
    clearInterval(jogTimer)
    jogTimer = null
  }
  if (isMoving.value) {
    isMoving.value = false
    sendLua('robot.servo_stop()')
  }
})
</script>

<template>
  <Teleport to="body">
    <div class="modal-overlay" @click.self="emit('close')">
      <div class="modal-container">
        <!-- Title Bar -->
        <div class="modal-titlebar">
          <div class="titlebar-left">
            <svg viewBox="0 0 24 24" class="titlebar-icon">
              <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-1 17.93c-3.95-.49-7-3.85-7-7.93 0-.62.08-1.21.21-1.79L9 15v1c0 1.1.9 2 2 2v1.93zm6.9-2.54c-.26-.81-1-1.39-1.9-1.39h-1v-3c0-.55-.45-1-1-1H8v-2h2c.55 0 1-.45 1-1V7h2c1.1 0 2-.9 2-2v-.41c2.93 1.19 5 4.06 5 7.41 0 2.08-.8 3.97-2.1 5.39z"/>
            </svg>
            <span>Manual Control</span>
          </div>
          <button class="btn-close" @click="emit('close')">
            <svg viewBox="0 0 24 24"><path d="M19 6.41L17.59 5 12 10.59 6.41 5 5 6.41 10.59 12 5 17.59 6.41 19 12 13.41 17.59 19 19 17.59 13.41 12z"/></svg>
          </button>
        </div>

        <!-- Control Bar -->
        <div class="control-bar">
          <div class="toggle-group">
            <button :class="['toggle-btn', { active: mode === 'jog' }]" @click="setMode('jog')">Jog</button>
            <button :class="['toggle-btn', { active: mode === 'inch' }]" @click="setMode('inch')">Inch</button>
          </div>
          <div class="toggle-group">
            <button :class="['toggle-btn', { active: space === 'joint' }]" @click="setSpace('joint')">Joint</button>
            <button :class="['toggle-btn', { active: space === 'cartesian' }]" @click="setSpace('cartesian')">Cartesian</button>
          </div>
          <div v-if="mode === 'inch'" class="distance-group">
            <span class="distance-label">Dist:</span>
            <button
              v-for="d in inchPresets"
              :key="d"
              :class="['dist-btn', { active: inchDistance === d }]"
              @click="inchDistance = d"
            >{{ d }}</button>
          </div>
        </div>

        <!-- Axis Grid -->
        <div class="axis-grid">
          <div v-for="(label, i) in labels" :key="label" class="axis-row">
            <button
              class="jog-btn jog-minus"
              @mousedown.prevent="handlePress(i, -1)"
              @mouseup.prevent="handleRelease()"
              @mouseleave="handleRelease()"
              @touchstart.prevent="handlePress(i, -1)"
              @touchend.prevent="handleRelease()"
              @touchcancel="handleRelease()"
            >&minus;</button>
            <div class="axis-info">
              <span class="axis-label">{{ label }}</span>
              <span class="axis-value">{{ formatValue(values[i]) }}</span>
            </div>
            <button
              class="jog-btn jog-plus"
              @mousedown.prevent="handlePress(i, 1)"
              @mouseup.prevent="handleRelease()"
              @mouseleave="handleRelease()"
              @touchstart.prevent="handlePress(i, 1)"
              @touchend.prevent="handleRelease()"
              @touchcancel="handleRelease()"
            >+</button>
          </div>
        </div>

        <!-- Status Bar -->
        <div class="status-bar">
          <div :class="['status-dot', { connected: connected, error: !connected }]"></div>
          <span v-if="lastError" class="status-error">{{ lastError }}</span>
          <span v-else class="status-ready">{{ connected ? 'Ready' : 'Disconnected' }}</span>
        </div>
      </div>
    </div>
  </Teleport>
</template>

<style scoped>
.modal-overlay {
  position: fixed;
  inset: 0;
  z-index: 1000;
  display: flex;
  align-items: center;
  justify-content: center;
  background: rgba(0, 0, 0, 0.2);
  backdrop-filter: blur(2px);
  animation: fadeIn 0.2s ease;
}

@keyframes fadeIn {
  from { opacity: 0; }
  to { opacity: 1; }
}

.modal-container {
  display: flex;
  flex-direction: column;
  width: 90vw;
  max-width: 700px;
  background: rgba(10, 14, 26, 0.6);
  border: 1px solid rgba(59, 130, 246, 0.35);
  border-radius: 12px;
  box-shadow:
    0 0 40px rgba(59, 130, 246, 0.12),
    0 24px 80px rgba(0, 0, 0, 0.5);
  overflow: hidden;
  animation: slideUp 0.25s ease;
}

@keyframes slideUp {
  from { opacity: 0; transform: translateY(20px); }
  to { opacity: 1; transform: translateY(0); }
}

/* Title Bar */
.modal-titlebar {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 18px;
  background: rgba(15, 23, 42, 0.5);
  border-bottom: 1px solid rgba(59, 130, 246, 0.25);
  flex-shrink: 0;
}

.titlebar-left {
  display: flex;
  align-items: center;
  gap: 10px;
  font-family: 'Oxanium', sans-serif;
  font-size: 13px;
  font-weight: 500;
  text-transform: uppercase;
  letter-spacing: 2px;
  color: var(--accent-blue);
}

.titlebar-icon {
  width: 18px;
  height: 18px;
  fill: var(--accent-blue);
}

.btn-close {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 30px;
  height: 30px;
  background: rgba(255, 255, 255, 0.05);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 6px;
  cursor: pointer;
  transition: all 0.2s;
}

.btn-close svg {
  width: 16px;
  height: 16px;
  fill: var(--text-secondary);
  transition: fill 0.2s;
}

.btn-close:hover {
  background: rgba(239, 68, 68, 0.15);
  border-color: var(--accent-red);
}

.btn-close:hover svg {
  fill: var(--accent-red);
}

/* Control Bar */
.control-bar {
  display: flex;
  align-items: center;
  gap: 16px;
  padding: 12px 18px;
  background: rgba(15, 23, 42, 0.3);
  border-bottom: 1px solid rgba(59, 130, 246, 0.15);
  flex-wrap: wrap;
}

.toggle-group {
  display: flex;
  border: 1px solid rgba(59, 130, 246, 0.3);
  border-radius: 6px;
  overflow: hidden;
}

.toggle-btn {
  padding: 5px 14px;
  background: transparent;
  border: none;
  font-family: 'Rajdhani', sans-serif;
  font-size: 12px;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 1px;
  color: var(--text-secondary);
  cursor: pointer;
  transition: all 0.2s;
}

.toggle-btn + .toggle-btn {
  border-left: 1px solid rgba(59, 130, 246, 0.3);
}

.toggle-btn.active {
  background: rgba(59, 130, 246, 0.2);
  color: var(--accent-blue);
}

.toggle-btn:hover:not(.active) {
  background: rgba(59, 130, 246, 0.08);
  color: var(--text-primary);
}

.distance-group {
  display: flex;
  align-items: center;
  gap: 6px;
}

.distance-label {
  font-family: 'Rajdhani', sans-serif;
  font-size: 11px;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 1px;
  color: var(--text-secondary);
}

.dist-btn {
  padding: 3px 10px;
  background: rgba(255, 255, 255, 0.04);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 4px;
  font-family: 'Oxanium', sans-serif;
  font-size: 11px;
  color: var(--text-secondary);
  cursor: pointer;
  transition: all 0.2s;
}

.dist-btn.active {
  background: rgba(59, 130, 246, 0.18);
  border-color: var(--accent-blue);
  color: var(--accent-blue);
}

.dist-btn:hover:not(.active) {
  background: rgba(255, 255, 255, 0.08);
  color: var(--text-primary);
}

.speed-bar {
  border-bottom: none;
}

/* Axis Grid */
.axis-grid {
  display: flex;
  flex-direction: column;
  gap: 6px;
  padding: 16px 18px;
}

.axis-row {
  display: flex;
  align-items: center;
  gap: 10px;
}

.jog-btn {
  width: 52px;
  height: 44px;
  border-radius: 6px;
  border: 1px solid;
  font-size: 22px;
  font-weight: 700;
  cursor: pointer;
  transition: all 0.15s;
  touch-action: none;
  user-select: none;
  -webkit-user-select: none;
  display: flex;
  align-items: center;
  justify-content: center;
}

.jog-minus {
  background: rgba(239, 68, 68, 0.08);
  border-color: rgba(239, 68, 68, 0.35);
  color: var(--accent-red);
}

.jog-minus:active {
  background: rgba(239, 68, 68, 0.25);
  box-shadow: 0 0 14px rgba(239, 68, 68, 0.3);
}

.jog-minus:hover {
  background: rgba(239, 68, 68, 0.15);
}

.jog-plus {
  background: rgba(34, 197, 94, 0.08);
  border-color: rgba(34, 197, 94, 0.35);
  color: var(--accent-green);
}

.jog-plus:active {
  background: rgba(34, 197, 94, 0.25);
  box-shadow: 0 0 14px rgba(34, 197, 94, 0.3);
}

.jog-plus:hover {
  background: rgba(34, 197, 94, 0.15);
}

.axis-info {
  flex: 1;
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0 12px;
  height: 44px;
  background: rgba(15, 23, 42, 0.35);
  border: 1px solid rgba(59, 130, 246, 0.15);
  border-radius: 6px;
}

.axis-label {
  font-family: 'Oxanium', sans-serif;
  font-size: 14px;
  font-weight: 600;
  letter-spacing: 1px;
  color: var(--accent-cyan);
}

.axis-value {
  font-family: 'Oxanium', sans-serif;
  font-size: 14px;
  font-weight: 400;
  color: var(--text-primary);
  letter-spacing: 0.5px;
}

/* Status Bar */
.status-bar {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 10px 18px;
  background: rgba(15, 23, 42, 0.3);
  border-top: 1px solid rgba(59, 130, 246, 0.15);
}

.status-dot {
  width: 7px;
  height: 7px;
  border-radius: 50%;
  background: var(--text-secondary);
  flex-shrink: 0;
}

.status-dot.connected {
  background: var(--accent-green);
  box-shadow: 0 0 6px var(--accent-green);
}

.status-dot.error {
  background: var(--accent-red);
  box-shadow: 0 0 6px var(--accent-red);
}

.status-ready {
  font-family: 'Rajdhani', sans-serif;
  font-size: 11px;
  color: var(--text-secondary);
  letter-spacing: 0.5px;
}

.status-error {
  font-family: 'Rajdhani', sans-serif;
  font-size: 11px;
  color: var(--accent-red);
  letter-spacing: 0.5px;
}
</style>
