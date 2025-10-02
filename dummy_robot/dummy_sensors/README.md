## **What Is This?**

This demo creates and runs two ROS 2 **nodes** individually, namely `dummy_laser` and `dummy_joint_states`.

**dummy_laser** publishes `sensor_msgs/msg/LaserScan` on a ROS 2 publisher topic, `/scan`.

**dummy_joint_states** publishes `sensor_msgs/msg/JointState` on a ROS 2 publisher topic, `/joint_states`.

Both **dummy_laser** and **dummy_joint_states** are dependencies of **dummy_robot_bringup**, and are usually expected to be run from the launch file there.
Please refer to [dummy_robot_bringup](https://github.com/ros2/demos/tree/rolling/dummy_robot/dummy_robot_bringup).

## **Build**

```bash
colcon build --packages-select dummy_sensors
```

## **Run**

### **1 - dummy_laser**

```bash
ros2 run dummy_sensors dummy_laser
```

```bash
# Terminal Output
[INFO] [1658564972.776089023] [dummy_laser]: angle inc:	0.004363
[INFO] [1658564972.776138078] [dummy_laser]: scan size:	1081
[INFO] [1658564972.776148752] [dummy_laser]: scan time increment: 	0.000000
```

### **2 - dummy_joint_states**

```bash
ros2 run dummy_sensors dummy_joint_states
```


## **Verify**

A similar terminal output should be seen after running commands described in the **Run** section above:

### **1 - dummy_laser**

```bash
# Open new terminal.
ros2 topic echo /scan
```

```bash
# Terminal Output
---
header:
  stamp:
    sec: 1658569107
    nanosec: 51491730
  frame_id: single_rrbot_hokuyo_link
angle_min: -2.356194496154785
angle_max: 2.356194496154785
angle_increment: 0.004363323096185923
time_increment: 2.7777778086601757e-05
scan_time: 0.03999999910593033
range_min: 0.0
range_max: 10.0
ranges:
- 0.39602306485176086
- 0.39602306485176086
- 0.39602306485176086
- 0.39602306485176086
- 0.39602306485176086
- 0.39602306485176086
- 0.39602306485176086
- '...'
intensities: []
---
```

### **2 - dummy_joint_states**

```bash
# Open new terminal.
ros2 topic echo /joint_states
```

```bash
# Terminal Output
---
header:
  stamp:
    sec: 1658569569
    nanosec: 414505214
  frame_id: ''
name:
- single_rrbot_joint1
- single_rrbot_joint2
position:
- -0.033653174530243454
- -0.033653174530243454
velocity: []
effort: []
---
header:
  stamp:
    sec: 1658569569
    nanosec: 434418040
  frame_id: ''
name:
- single_rrbot_joint1
- single_rrbot_joint2
position:
- -0.13326191698696957
- -0.13326191698696957
velocity: []
effort: []
---
header:
  stamp:
    sec: 1658569569
    nanosec: 454443902
  frame_id: ''
name:
- single_rrbot_joint1
- single_rrbot_joint2
position:
- -0.2315391504196944
- -0.2315391504196944
velocity: []
effort: []
---
header:
  stamp:
    sec: 1658569569
    nanosec: 474417706
  frame_id: ''
name:
- single_rrbot_joint1
- single_rrbot_joint2
position:
- -0.3275029211980919
- -0.3275029211980919
velocity: []
effort: []
---
header:
  stamp:
    sec: 1658569569
    nanosec: 494442418
  frame_id: ''
name:
- single_rrbot_joint1
- single_rrbot_joint2
position:
- -0.4201943910459491
- -0.4201943910459491
velocity: []
effort: []
---
```


## **References**

- `sensor_msgs/msg/LaserScan` Message Format: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/LaserScan.msg
- `sensor_msgs/msg/JointState` Message Format: https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/JointState.msg
