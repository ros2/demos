## **What Is This?**

This demo creates and runs a ROS2 **node**, `dummy_map_server`, that publishes `nav_msgs::msg::OccupancyGrid` on a ROS2 publisher topic, `/map`.

**dummy_map_server** is also a dependency of **dummy_robot_bringup**.
 Please refer to [dummy_robot_bringup](https://github.com/ros2/demos/tree/rolling/dummy_robot/dummy_robot_bringup).

## **What Is An Occupancy Grid?**

An occupancy grid is essentially a **2-dimensional** map with each cell containing an integer to indicate the presence, absence or general state of a space in a certain physical room a robot is in. With this set of information, it better informs how a robot can best navigate through obstacles.

Eg.

`-1` represents **unknown** spaces.
`0` represents **unoccupied** spaces.
`1` represents **occupied** spaces.

:warning: Note that the integer within a cell is application-specific and can go beyond `-1` and `1`.

![](img/occupancy_grid.png)

The bolded integer indicates the position of a robot with lidar.

## **Build**

```bash
cd $HOME
mkdir demo_ws/src
cd ~/demo_ws/src
git clone https://github.com/ros2/demos.git
cd ~/demo_ws
source /opt/ros/<DISTRO_NAME>/setup.bash
colcon build --packages-select dummy_map_server
```

## **Run**

```bash
cd $HOME
cd ~/demo/src
source install/setup.bash
ros2 run dummy_map_server dummy_map_server
```

## **Verify**

A similar terminal output should be seen after running `ros2 run dummy_map_server` and the following commands:

```bash
# Open new terminal.
source /opt/ros/<DISTRO_NAME>/setup.bash
ros2 topic echo /map
```


```bash
# Terminal Output
header:
  stamp:
    sec: 1658559453
    nanosec: 308405010
  frame_id: world
info:
  map_load_time:
    sec: 0
    nanosec: 0
  resolution: 0.10000000149011612
  width: 100
  height: 100
  origin:
    position:
      x: -5.0
      y: -5.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
data:
- -1
- -1
- -1
- -1
- -1
- -1
- -1
- -1
- -1
- '...'
---

```

## **References**

- `nav_msgs::msg::OccupancyGrid` Message Format: https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg 
