## **What Is This?**

This is a simple demo robot with all components from publishing **joint states** to publishing **fake laser data** to visualizing the robot model on a map in `RViz2`.

## **Build**

```bash
colcon build --package-select dummy_map_server dummy_sensors dummy_robot_bringup
```

## **Run**

```bash
ros2 launch dummy_robot_bringup dummy_robot_bringup_launch.py
```

## **Verify**

A similar terminal output should be seen:

```bash
[INFO] [launch]: All log files can be found below /home/$USER/.ros/log/2022-07-22-17-02-33-629155-comname-39641
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [dummy_map_server-1]: process started with pid [39643]
[INFO] [robot_state_publisher-2]: process started with pid [39645]
[INFO] [dummy_joint_states-3]: process started with pid [39647]
[INFO] [dummy_laser-4]: process started with pid [39649]
[dummy_laser-4] [INFO] [1658480554.018685559] [dummy_laser]: angle inc:	0.004363
[dummy_laser-4] [INFO] [1658480554.018738346] [dummy_laser]: scan size:	1081
[dummy_laser-4] [INFO] [1658480554.018747091] [dummy_laser]: scan time increment: 0.000028
[robot_state_publisher-2] Link single_rrbot_link1 had 1 children
[robot_state_publisher-2] Link single_rrbot_link2 had 1 children
[robot_state_publisher-2] Link single_rrbot_link3 had 2 children
[robot_state_publisher-2] Link single_rrbot_camera_link had 0 children
[robot_state_publisher-2] Link single_rrbot_hokuyo_link had 0 children
[robot_state_publisher-2] [INFO] [1658480554.019739425] [robot_state_publisher]: got segment single_rrbot_camera_link
[robot_state_publisher-2] [INFO] [1658480554.019783457] [robot_state_publisher]: got segment single_rrbot_hokuyo_link
[robot_state_publisher-2] [INFO] [1658480554.019791344] [robot_state_publisher]: got segment single_rrbot_link1
[robot_state_publisher-2] [INFO] [1658480554.019796905] [robot_state_publisher]: got segment single_rrbot_link2
[robot_state_publisher-2] [INFO] [1658480554.019801861] [robot_state_publisher]: got segment single_rrbot_link3
[robot_state_publisher-2] [INFO] [1658480554.019806522] [robot_state_publisher]: got segment world
```

The robot should be displayed similarly in `RViz2`:

![](img/robot_in_rviz.gif)

## **FAQ**

`Q`: I ran `ros2 launch dummy_robot_bringup dummy_robot_bringup_launch.py` and `rviz2`. However, nothing is displayed in `RViz2` window.

`A`: This issue could be related to the **ROS 2 Daemon**. It serves the same role as a **ROS1 Master** but within **ROS 2**. Restarting the daemon, as follows, should resolve this issue.

`Reference`: https://github.com/ros2/ros2cli/issues/582#issuecomment-775997721

```bash
ros2 daemon stop; ros2 daemon start
# Verify
# Open new terminal
ros2 launch dummy_robot_bringup dummy_robot_bringup_launch.py
# Open new terminal
ros2 node list
```

```bash
# You should see the following terminal output:
/dummy_joint_states
/dummy_laser
/dummy_map_server
/robot_state_publisher
```

## **References**

- Original Rolling Demo Tutorial: https://docs.ros.org/en/rolling/Tutorials/Demos/dummy-robot-demo.html
- What is ROS 2 Daemon: https://answers.ros.org/question/327348/what-is-ros2-daemon/
