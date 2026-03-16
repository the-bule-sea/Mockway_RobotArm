const r = (v) => Math.round(v * 10000) / 10000

// Shadow inputs for 6 joint values (rad)
function jointShadows(joints) {
  const labels = ['J1','J2','J3','J4','J5','J6']
  const inputs = {}
  labels.forEach((l, i) => {
    inputs[l] = { shadow: { type: 'math_number', fields: { NUM: r(joints[i] || 0) } } }
  })
  return inputs
}

// Shadow inputs for pose: X/Y/Z/Roll/Pitch/Yaw
function poseShadows(pose) {
  const labels = ['X','Y','Z','Roll','Pitch','Yaw']
  const inputs = {}
  labels.forEach((l, i) => {
    inputs[l] = { shadow: { type: 'math_number', fields: { NUM: r(pose[i] || 0) } } }
  })
  return inputs
}

// Zero shadows for 6-velocity servo_joints
const ZERO_VEL = (() => {
  const inputs = {}
  ;['V1','V2','V3','V4','V5','V6'].forEach(l => {
    inputs[l] = { shadow: { type: 'math_number', fields: { NUM: 0 } } }
  })
  return inputs
})()

// Zero shadows for servo_cartesian
const ZERO_CART = (() => {
  const inputs = {}
  ;['VX','VY','VZ','RX','RY','RZ'].forEach(l => {
    inputs[l] = { shadow: { type: 'math_number', fields: { NUM: 0 } } }
  })
  return inputs
})()

// Zero shadows for move_linear_relative
const ZERO_REL = (() => {
  const inputs = {}
  ;['DX','DY','DZ','DRX','DRY','DRZ'].forEach(l => {
    inputs[l] = { shadow: { type: 'math_number', fields: { NUM: 0 } } }
  })
  return inputs
})()

/**
 * Create toolbox config with current robot values as defaults.
 * @param {number[]} joints - current joint positions (rad)
 * @param {number[]} pose   - current end-effector pose [x,y,z,roll,pitch,yaw] (m, rad)
 */
export function createToolbox(joints, pose) {
  return {
    kind: 'categoryToolbox',
    contents: [

      // ==================== Servo ====================
      {
        kind: 'category',
        name: 'Servo',
        colour: '30',
        contents: [
          { kind: 'block', type: 'robot_switch_servo_mode' },
          { kind: 'block', type: 'robot_servo_joint', inputs: {
            VEL: { shadow: { type: 'math_number', fields: { NUM: 0.3 } } }
          }},
          { kind: 'block', type: 'robot_servo_joints', inputs: { ...ZERO_VEL } },
          { kind: 'block', type: 'robot_servo_cartesian', inputs: { ...ZERO_CART } },
          { kind: 'block', type: 'robot_servo_stop' }
        ]
      },

      // ==================== Motion ====================
      {
        kind: 'category',
        name: 'Motion',
        colour: '140',
        contents: [
          { kind: 'label', text: 'PTP (Point-to-Point)' },
          { kind: 'block', type: 'robot_move_to_named' },
          { kind: 'block', type: 'robot_move_to_joints', inputs: jointShadows(joints) },
          { kind: 'block', type: 'robot_move_to_pose_rpy', inputs: poseShadows(pose) },
          { kind: 'label', text: 'Linear' },
          { kind: 'block', type: 'robot_move_linear_rpy', inputs: poseShadows(pose) },
          { kind: 'block', type: 'robot_move_linear_relative', inputs: { ...ZERO_REL } },
          { kind: 'block', type: 'robot_move_linear_relative_tool', inputs: { ...ZERO_REL } }
        ]
      },

      // ==================== Parameters ====================
      {
        kind: 'category',
        name: 'Parameters',
        colour: '270',
        contents: [
          { kind: 'block', type: 'robot_set_velocity_scaling', inputs: {
            FACTOR: { shadow: { type: 'math_number', fields: { NUM: 0.3 } } }
          }},
          { kind: 'block', type: 'robot_set_acceleration_scaling', inputs: {
            FACTOR: { shadow: { type: 'math_number', fields: { NUM: 0.1 } } }
          }},
          { kind: 'block', type: 'robot_set_planning_time', inputs: {
            SECONDS: { shadow: { type: 'math_number', fields: { NUM: 5 } } }
          }},
          { kind: 'block', type: 'robot_set_planner' }
        ]
      },

      // ==================== Status ====================
      {
        kind: 'category',
        name: 'Status',
        colour: '210',
        contents: [
          { kind: 'block', type: 'robot_get_joint_positions' },
          { kind: 'block', type: 'robot_get_current_pose' },
          { kind: 'block', type: 'robot_get_current_rpy' }
        ]
      },

      // ==================== Tools ====================
      {
        kind: 'category',
        name: 'Tools',
        colour: '45',
        contents: [
          { kind: 'block', type: 'robot_sleep', inputs: {
            SECONDS: { shadow: { type: 'math_number', fields: { NUM: 1 } } }
          }},
          { kind: 'block', type: 'robot_log', inputs: {
            MSG: { shadow: { type: 'text', fields: { TEXT: 'message' } } }
          }},
          { kind: 'block', type: 'robot_log_warn', inputs: {
            MSG: { shadow: { type: 'text', fields: { TEXT: 'warning' } } }
          }},
          { kind: 'block', type: 'robot_log_error', inputs: {
            MSG: { shadow: { type: 'text', fields: { TEXT: 'error' } } }
          }},
          { kind: 'block', type: 'robot_ok' },
          { kind: 'block', type: 'robot_deg2rad', inputs: {
            DEG: { shadow: { type: 'math_number', fields: { NUM: 90 } } }
          }},
          { kind: 'block', type: 'robot_rad2deg', inputs: {
            RAD: { shadow: { type: 'math_number', fields: { NUM: 1.5708 } } }
          }},
          { kind: 'block', type: 'robot_print', inputs: {
            TEXT: { shadow: { type: 'text', fields: { TEXT: 'Hello' } } }
          }}
        ]
      },

      // ==================== Data ====================
      {
        kind: 'category',
        name: 'Data',
        colour: '230',
        contents: [
          { kind: 'block', type: 'robot_table_index' },
          { kind: 'sep', gap: '16' },
          { kind: 'block', type: 'math_number', fields: { NUM: 0 } },
          { kind: 'block', type: 'text', fields: { TEXT: '' } }
        ]
      },

      { kind: 'sep' },

      // ==================== Logic ====================
      {
        kind: 'category',
        name: 'Logic',
        colour: '210',
        contents: [
          { kind: 'block', type: 'controls_if' },
          { kind: 'block', type: 'logic_compare' },
          { kind: 'block', type: 'logic_boolean' },
          { kind: 'block', type: 'logic_negate' }
        ]
      },

      // ==================== Loops ====================
      {
        kind: 'category',
        name: 'Loops',
        colour: '120',
        contents: [
          { kind: 'block', type: 'controls_repeat_ext', inputs: {
            TIMES: { shadow: { type: 'math_number', fields: { NUM: 10 } } }
          }},
          { kind: 'block', type: 'controls_whileUntil' }
        ]
      },

      // ==================== Math ====================
      {
        kind: 'category',
        name: 'Math',
        colour: '230',
        contents: [
          { kind: 'block', type: 'math_number', fields: { NUM: 0 } },
          { kind: 'block', type: 'math_arithmetic', inputs: {
            A: { shadow: { type: 'math_number', fields: { NUM: 1 } } },
            B: { shadow: { type: 'math_number', fields: { NUM: 1 } } }
          }}
        ]
      },

      // ==================== Variables ====================
      {
        kind: 'category',
        name: 'Variables',
        colour: '330',
        custom: 'VARIABLE'
      }
    ]
  }
}
