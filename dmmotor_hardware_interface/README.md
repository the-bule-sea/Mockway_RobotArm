# DM Motor Hardware Interface for ROS2 Control

ROS2 Control hardware interface plugin for DM motors (DM-J4310-2EC and DM4340) supporting multiple CAN interfaces.

## TODO

**fix:**

与move_group同时启动时，会出现 servo_node 等待 robot state update 的问题。

一直打印提示：`[servo_node]: Waiting to receive robot state update. `

参考：

[generic_system.hpp](https://github.com/ros-controls/ros2_control/blob/jazzy/hardware_interface/include/mock_components/generic_system.hpp)

[generic_system.cpp](https://github.com/ros-controls/ros2_control/blob/jazzy/hardware_interface/src/mock_components/generic_system.cpp)

[franka_hardware](https://github.com/frankarobotics/franka_ros2/blob/jazzy/franka_hardware/include/franka_hardware/franka_hardware_interface.hpp)

## Features

- Support for DM-J4310-2EC and DM4340 motors
- MIT control mode for smooth position/velocity control
- **Multiple CAN interface support:**
  - SocketCAN (Linux native CAN interface)
  - USB-CAN adapter (WitMotion USB-CAN and compatible devices)
- Real-time motor state feedback (position, velocity, torque, temperature)
- Configurable per-joint motor parameters

## Motor Configuration

### Joint Assignment
- Joint 1 (J1): DM-J4310-2EC
- Joint 2 (J2): DM4340
- Joint 3 (J3): DM4340
- Joint 4 (J4): DM-J4310-2EC
- Joint 5 (J5): DM-J4310-2EC
- Joint 6 (J6): DM-J4310-2EC

### Motor Parameters

**DM-J4310-2EC:**
- Max Position: ±12.5 rad
- Max Velocity: 30.0 rad/s
- Max Torque: 10.0 Nm

**DM4340:**
- Max Position: ±12.5 rad
- Max Velocity: 8.0 rad/s
- Max Torque: 28.0 Nm

## Prerequisites

Choose one of the following CAN interface options:

### Option 1: SocketCAN (Linux Native CAN)

#### 1.1 Install SocketCAN utilities

```bash
sudo apt-get update
sudo apt-get install can-utils
```

#### 1.2 Configure CAN Interface

Create a script to configure the CAN interface (e.g., `/etc/systemd/system/can-setup.sh`):

```bash
#!/bin/bash
# Setup CAN interface
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
```

Make it executable:
```bash
chmod +x /etc/systemd/system/can-setup.sh
```

To automatically setup CAN on boot, create a systemd service (`/etc/systemd/system/can-setup.service`):

```ini
[Unit]
Description=Setup CAN interface
After=network.target

[Service]
Type=oneshot
ExecStart=/etc/systemd/system/can-setup.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Enable the service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable can-setup.service
sudo systemctl start can-setup.service
```

#### 1.3 Verify CAN Interface

```bash
# Check if CAN interface is up
ip link show can0

# Monitor CAN traffic
candump can0

# Send test frame
cansend can0 123#DEADBEEF
```

### Option 2: USB-CAN Adapter (WitMotion and Compatible)

#### 2.1 Connect USB-CAN Adapter

Connect your USB-CAN adapter to the computer. The adapter should appear as a serial device.

#### 2.2 Find Serial Port

```bash
# List available serial ports
ls /dev/ttyUSB* /dev/ttyACM*

# Or use dmesg to see the device
dmesg | grep tty
```

Common ports:
- `/dev/ttyUSB0` - Most USB-serial adapters
- `/dev/ttyACM0` - Some USB-CDC adapters
- `COM3`, `COM4`, etc. - Windows (not officially supported yet)

#### 2.3 Set Serial Port Permissions

Add your user to the `dialout` group to access serial ports:

```bash
sudo usermod -a -G dialout $USER
```

Log out and log back in for changes to take effect.

Alternatively, create a udev rule (`/etc/udev/rules.d/99-usb-can.rules`):

```
# WitMotion USB-CAN adapter
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout"
```

Reload udev rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

#### 2.4 Test USB-CAN Connection

You can test the USB-CAN adapter using the Python driver:

```bash
cd ~/ws/mockway_robotics/tools/motor_gui
python3 dm_motor_driver.py
```

Or use a serial terminal:
```bash
# Install minicom if not already installed
sudo apt-get install minicom

# Connect to the adapter (adjust port and baudrate)
minicom -D /dev/ttyUSB0 -b 921600

# Send AT command to test
AT+AT
# Should receive: OK
```

## Building

```bash
cd ~/ws/mockway_robotics
colcon build --packages-select dmmotor_hardware_interface
source install/setup.bash
```

## Configuration

The hardware interface is configured in the URDF/xacro file (`moveit_mockway_config/config/mockway_description.ros2_control.xacro`).

### Hardware Parameters

#### For SocketCAN

```xml
<hardware>
    <plugin>dmmotor_hardware_interface/DMMototHardwareInterface</plugin>
    <param name="can_interface_type">socketcan</param>  <!-- Optional, default is socketcan -->
    <param name="can_port">can0</param>
    <param name="can_baudrate">1000000</param>
    <param name="position_kp">40.0</param>
    <param name="position_kd">1.0</param>
</hardware>
```

#### For USB-CAN Adapter

```xml
<hardware>
    <plugin>dmmotor_hardware_interface/DMMototHardwareInterface</plugin>
    <param name="can_interface_type">usb_can</param>
    <param name="can_port">/dev/ttyUSB0</param>           <!-- Serial port -->
    <param name="serial_baudrate">921600</param>          <!-- Serial baudrate, default: 921600 -->
    <param name="can_baudrate">1000000</param>            <!-- CAN baudrate -->
    <param name="position_kp">40.0</param>
    <param name="position_kd">1.0</param>
</hardware>
```

**Parameter Reference:**
- `can_interface_type`: Interface type - `socketcan` or `usb_can` (default: `socketcan`)
- `can_port`: For SocketCAN: interface name (e.g., `can0`). For USB-CAN: serial port (e.g., `/dev/ttyUSB0`)
- `can_baudrate`: CAN bus baudrate in bps (typically 1000000 for 1 Mbps)
- `serial_baudrate`: (USB-CAN only) Serial port baudrate (default: 921600)
- `position_kp`: Position control proportional gain [0-500]
- `position_kd`: Position control derivative gain [0-5]

### Joint Parameters

For each joint, specify:
- `motor_id`: CAN ID of the motor (1-127)
- `master_id`: CAN ID for receiving feedback (typically 0)
- `motor_type`: Either `DM_J4310_2EC` or `DM4340`

Example:
```xml
<joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <param name="motor_id">1</param>
    <param name="master_id">0</param>
    <param name="motor_type">DM_J4310_2EC</param>
</joint>
```

## Usage

### Starting the Hardware Interface

The hardware interface is automatically loaded by ros2_control when you launch MoveIt:

```bash
ros2 launch moveit_mockway_config demo.launch.py
```

### Motor Control Modes

The hardware interface uses **MIT control mode** for position control:

```
torque = kp * (p_cmd - p_actual) + kd * (v_cmd - v_actual)
```

Control parameters (`position_kp` and `position_kd`) can be tuned in the xacro file.

### Monitoring Motor Status

```bash
# Check joint states
ros2 topic echo /joint_states

# Monitor controller status
ros2 control list_controllers

# View hardware component status
ros2 control list_hardware_components
```

## Troubleshooting

### CAN Interface Issues

#### SocketCAN Issues

**Problem:** `Failed to open CAN interface can0`

**Solutions:**
1. Check if CAN interface exists: `ip link show can0`
2. Bring up the interface: `sudo ip link set up can0`
3. Verify bitrate: `ip -details link show can0`
4. Check CAN hardware connection

#### USB-CAN Issues

**Problem:** `Failed to open serial port /dev/ttyUSB0`

**Solutions:**
1. Check if device exists: `ls -l /dev/ttyUSB0`
2. Verify USB connection: `dmesg | tail`
3. Check permissions: `sudo usermod -a -G dialout $USER` (log out/in after)
4. Try different USB ports
5. Check USB cable quality

**Problem:** `Warning: Failed to enter AT mode`

**Solutions:**
1. Verify serial baudrate is correct (typically 921600)
2. Try power-cycling the USB-CAN adapter
3. Check if adapter is in correct mode
4. Some adapters may work without AT mode confirmation - check if CAN communication works

**Problem:** `Permission denied when accessing serial port`

**Solutions:**
1. Add user to dialout group: `sudo usermod -a -G dialout $USER`
2. Log out and log back in
3. Or use udev rules (see USB-CAN setup section)
4. Temporary fix: `sudo chmod 666 /dev/ttyUSB0`

**Problem:** `No feedback from motors`

**Solutions:**
1. Check CAN wiring and termination resistors (120Ω at both ends)
2. Verify motor power supply
3. Monitor CAN bus: `candump can0`
4. Check motor IDs match configuration
5. Ensure motors are properly enabled

**Problem:** `Motors not responding to commands`

**Solutions:**
1. Verify motors are enabled (check error codes)
2. Check motor ID configuration
3. Ensure CAN baudrate matches (1 Mbps)
4. Verify MIT control parameters are reasonable

### Error Codes

Motor error codes are reported in feedback:
- `0x0`: Disabled
- `0x1`: Enabled (normal operation)
- `0x8`: Over-voltage
- `0x9`: Under-voltage
- `0xA`: Over-current
- `0xB`: MOS over-temperature
- `0xC`: Coil over-temperature
- `0xD`: Communication loss
- `0xE`: Overload

## Safety Notes

1. **Emergency Stop**: Always have an emergency stop mechanism
2. **Workspace Limits**: Ensure joint limits are properly configured
3. **Temperature Monitoring**: Monitor motor temperatures during operation
4. **Power Supply**: Use appropriate power supply (check motor specifications)
5. **Initial Testing**: Test with low velocities and accelerations initially

## Development and Debugging

### Enable Debug Logging

```bash
ros2 launch moveit_mockway_config demo.launch.py --ros-args --log-level dmmotor_hardware_interface:=debug
```

### Test Individual Motors

Use the Python driver for testing:

```bash
cd ~/ws/mockway_robotics/tools/motor_gui
python3 dm_motor_driver.py
```

## References

- [ROS2 Control Documentation](https://control.ros.org/)
- [SocketCAN Documentation](https://www.kernel.org/doc/Documentation/networking/can.txt)
- DM Motor Manual (refer to manufacturer documentation)

## License

MIT License - Copyright (c) 2025 Mockway Robotics
