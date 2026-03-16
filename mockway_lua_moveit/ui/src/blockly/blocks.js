import * as Blockly from 'blockly/core'

// ======================== Colors ========================
const SERVO_HUE  = 30    // orange
const MOTION_HUE = 140   // green
const PARAM_HUE  = 270   // purple
const STATUS_HUE = 210   // blue
const TOOL_HUE   = 45    // yellow
const DATA_HUE   = 230   // grey-blue

// ======================== Servo Blocks ========================

Blockly.Blocks['robot_switch_servo_mode'] = {
  init() {
    this.appendDummyInput()
      .appendField('SwitchServoMode')
      .appendField(new Blockly.FieldDropdown([
        ['joint_jog', 'joint_jog'],
        ['twist',     'twist']
      ]), 'MODE')
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(SERVO_HUE)
    this.setTooltip('Switch servo command type: joint_jog or twist')
  }
}

Blockly.Blocks['robot_servo_joint'] = {
  init() {
    this.appendDummyInput()
      .appendField('ServoJoint  J')
      .appendField(new Blockly.FieldDropdown([
        ['1','1'],['2','2'],['3','3'],['4','4'],['5','5'],['6','6']
      ]), 'INDEX')
    this.appendValueInput('VEL').setCheck('Number').appendField('vel')
    this.setInputsInline(true)
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(SERVO_HUE)
    this.setTooltip('Single joint velocity jog (rad/s). Non-blocking.')
  }
}

Blockly.Blocks['robot_servo_joints'] = {
  init() {
    this.appendValueInput('V1').setCheck('Number').appendField('ServoJoints  V1')
    ;['V2','V3','V4','V5','V6'].forEach(l => {
      this.appendValueInput(l).setCheck('Number').appendField(l)
    })
    this.setInputsInline(true)
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(SERVO_HUE)
    this.setTooltip('6-axis simultaneous velocity jog {v1..v6} (rad/s). Non-blocking.')
  }
}

Blockly.Blocks['robot_servo_cartesian'] = {
  init() {
    this.appendValueInput('VX').setCheck('Number').appendField('ServoCartesian  Vx')
    ;['VY','VZ','RX','RY','RZ'].forEach(l => {
      this.appendValueInput(l).setCheck('Number').appendField(l)
    })
    this.setInputsInline(true)
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(SERVO_HUE)
    this.setTooltip('Cartesian velocity jog: Vx/Vy/Vz (m/s), Rx/Ry/Rz (rad/s). Non-blocking.')
  }
}

Blockly.Blocks['robot_servo_stop'] = {
  init() {
    this.appendDummyInput().appendField('ServoStop()')
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(SERVO_HUE)
    this.setTooltip('Stop servo motion by publishing zero velocity')
  }
}

// ======================== PTP Motion Blocks ========================

Blockly.Blocks['robot_move_to_named'] = {
  init() {
    this.appendDummyInput()
      .appendField('MoveToNamed')
      .appendField(new Blockly.FieldTextInput('home'), 'NAME')
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(MOTION_HUE)
    this.setTooltip('PTP motion to SRDF named state (blocking)')
  }
}

Blockly.Blocks['robot_move_to_joints'] = {
  init() {
    this.appendValueInput('J1').setCheck('Number').appendField('MoveToJoints  J1')
    ;['J2','J3','J4','J5','J6'].forEach(l => {
      this.appendValueInput(l).setCheck('Number').appendField(l)
    })
    this.setInputsInline(true)
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(MOTION_HUE)
    this.setTooltip('PTP motion to joint positions (rad, blocking)')
  }
}

Blockly.Blocks['robot_move_to_pose_rpy'] = {
  init() {
    this.appendValueInput('X').setCheck('Number').appendField('MoveToPoseRPY  X')
    ;['Y','Z','Roll','Pitch','Yaw'].forEach(l => {
      this.appendValueInput(l).setCheck('Number').appendField(l)
    })
    this.setInputsInline(true)
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(MOTION_HUE)
    this.setTooltip('PTP to Cartesian pose: X/Y/Z (m), Roll/Pitch/Yaw (rad, blocking)')
  }
}

// ======================== Linear Motion Blocks ========================

Blockly.Blocks['robot_move_linear_rpy'] = {
  init() {
    this.appendValueInput('X').setCheck('Number').appendField('MoveLinearRPY  X')
    ;['Y','Z','Roll','Pitch','Yaw'].forEach(l => {
      this.appendValueInput(l).setCheck('Number').appendField(l)
    })
    this.setInputsInline(true)
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(MOTION_HUE)
    this.setTooltip('Linear motion to Cartesian pose: X/Y/Z (m), Roll/Pitch/Yaw (rad, blocking)')
  }
}

Blockly.Blocks['robot_move_linear_relative'] = {
  init() {
    this.appendValueInput('DX').setCheck('Number').appendField('MoveLinearRel  dX')
    ;['DY','DZ','DRX','DRY','DRZ'].forEach(l => {
      this.appendValueInput(l).setCheck('Number').appendField(l)
    })
    this.setInputsInline(true)
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(MOTION_HUE)
    this.setTooltip('Relative linear motion in base frame: dX/dY/dZ (mm), dRx/dRy/dRz (deg, blocking)')
  }
}

Blockly.Blocks['robot_move_linear_relative_tool'] = {
  init() {
    this.appendValueInput('DX').setCheck('Number').appendField('MoveLinearRelTool  dX')
    ;['DY','DZ','DRX','DRY','DRZ'].forEach(l => {
      this.appendValueInput(l).setCheck('Number').appendField(l)
    })
    this.setInputsInline(true)
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(MOTION_HUE)
    this.setTooltip('Relative linear motion in tool frame: dX/dY/dZ (mm), dRx/dRy/dRz (deg, blocking). Delta is expressed in the end-effector coordinate system.')
  }
}

// ======================== Parameter Blocks ========================

Blockly.Blocks['robot_set_velocity_scaling'] = {
  init() {
    this.appendValueInput('FACTOR').setCheck('Number').appendField('SetVelocityScaling')
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(PARAM_HUE)
    this.setTooltip('Set max velocity scaling factor [0.01, 1.0]')
  }
}

Blockly.Blocks['robot_set_acceleration_scaling'] = {
  init() {
    this.appendValueInput('FACTOR').setCheck('Number').appendField('SetAccelerationScaling')
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(PARAM_HUE)
    this.setTooltip('Set max acceleration scaling factor [0.01, 1.0]')
  }
}

Blockly.Blocks['robot_set_planning_time'] = {
  init() {
    this.appendValueInput('SECONDS').setCheck('Number').appendField('SetPlanningTime')
    this.appendDummyInput().appendField('s')
    this.setInputsInline(true)
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(PARAM_HUE)
    this.setTooltip('Set planning timeout (seconds)')
  }
}

Blockly.Blocks['robot_set_planner'] = {
  init() {
    this.appendDummyInput()
      .appendField('SetPlanner')
      .appendField(new Blockly.FieldDropdown([
        ['RRTConnect', 'RRTConnect'],
        ['RRT',        'RRT'],
        ['PRM',        'PRM'],
        ['LIN',        'LIN'],
        ['CIRC',       'CIRC'],
        ['PTP',        'PTP']
      ]), 'PLANNER')
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(PARAM_HUE)
    this.setTooltip('Switch motion planner (e.g. RRTConnect, LIN for Pilz linear)')
  }
}

// ======================== Status Blocks ========================

Blockly.Blocks['robot_get_joint_positions'] = {
  init() {
    this.appendDummyInput().appendField('GetJointPositions()')
    this.setOutput(true, null)
    this.setColour(STATUS_HUE)
    this.setTooltip('Get current joint positions as table {j1..j6} (rad)')
  }
}

Blockly.Blocks['robot_get_current_pose'] = {
  init() {
    this.appendDummyInput().appendField('GetCurrentPose()')
    this.setOutput(true, null)
    this.setColour(STATUS_HUE)
    this.setTooltip('Get current end-effector pose (quaternion) as table {x,y,z,qx,qy,qz,qw}')
  }
}

Blockly.Blocks['robot_get_current_rpy'] = {
  init() {
    this.appendDummyInput().appendField('GetCurrentRPY()')
    this.setOutput(true, null)
    this.setColour(STATUS_HUE)
    this.setTooltip('Get current end-effector pose (RPY) as table {roll, pitch, yaw} (rad)')
  }
}

// ======================== Tool Blocks ========================

Blockly.Blocks['robot_sleep'] = {
  init() {
    this.appendValueInput('SECONDS').setCheck('Number').appendField('Sleep')
    this.appendDummyInput().appendField('s')
    this.setInputsInline(true)
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(TOOL_HUE)
    this.setTooltip('Pause execution (seconds)')
  }
}

Blockly.Blocks['robot_log'] = {
  init() {
    this.appendValueInput('MSG').setCheck(null).appendField('log')
    this.setInputsInline(true)
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(TOOL_HUE)
    this.setTooltip('Output ROS INFO log message')
  }
}

Blockly.Blocks['robot_log_warn'] = {
  init() {
    this.appendValueInput('MSG').setCheck(null).appendField('log_warn')
    this.setInputsInline(true)
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(TOOL_HUE)
    this.setTooltip('Output ROS WARN log message')
  }
}

Blockly.Blocks['robot_log_error'] = {
  init() {
    this.appendValueInput('MSG').setCheck(null).appendField('log_error')
    this.setInputsInline(true)
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(TOOL_HUE)
    this.setTooltip('Output ROS ERROR log message')
  }
}

Blockly.Blocks['robot_ok'] = {
  init() {
    this.appendDummyInput().appendField('robot.ok()')
    this.setOutput(true, 'Boolean')
    this.setColour(TOOL_HUE)
    this.setTooltip('Returns true while ROS node is running — use as while-loop condition')
  }
}

Blockly.Blocks['robot_deg2rad'] = {
  init() {
    this.appendValueInput('DEG').setCheck('Number').appendField('deg2rad')
    this.setInputsInline(true)
    this.setOutput(true, 'Number')
    this.setColour(TOOL_HUE)
    this.setTooltip('Convert degrees to radians')
  }
}

Blockly.Blocks['robot_rad2deg'] = {
  init() {
    this.appendValueInput('RAD').setCheck('Number').appendField('rad2deg')
    this.setInputsInline(true)
    this.setOutput(true, 'Number')
    this.setColour(TOOL_HUE)
    this.setTooltip('Convert radians to degrees')
  }
}

Blockly.Blocks['robot_print'] = {
  init() {
    this.appendValueInput('TEXT').setCheck(null).appendField('print')
    this.setInputsInline(true)
    this.setPreviousStatement(true, null)
    this.setNextStatement(true, null)
    this.setColour(TOOL_HUE)
    this.setTooltip('Print value to stdout')
  }
}

// ======================== Data Blocks ========================

// Table index access  table[index]
Blockly.Blocks['robot_table_index'] = {
  init() {
    this.appendValueInput('TABLE').setCheck(null).appendField('')
    this.appendValueInput('INDEX').setCheck('Number').appendField('[')
    this.appendDummyInput().appendField(']')
    this.setInputsInline(true)
    this.setOutput(true, null)
    this.setColour(DATA_HUE)
    this.setTooltip('Access table element by index (1-based)')
  }
}
