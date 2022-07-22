## **What Is This?**

This demo provides a simple utility to **connect to a workstation default camera device** and **display** it in an **OpenCV** window like below:

![](img/result.png)

## **Build**

```bash
cd $HOME
mkdir -p demo_ws/src
cd ~/demo_ws/src
git clone https://github.com/ros2/demos.git
cd ~/demo_ws
source /opt/ros/<DISTRO_NAME>/setup.bash
# source /opt/ros/rolling/setup.bash
colcon build --package-select image_tools
```

## **Run**

In `image_tools` ROS2 package, 2 executable are provided, namely `cam2image` and `showimage` with different functions.

## **1 - cam2image**
Running this executable connects to your workstation's default camera device's video stream and publishes on ROS2 publisher, `/image` and `/flipimage`.

```bash
# Open new terminal
cd ~/demo_ws
source install/setup.bash
ros2 run image_tools cam2image
```

## **2 - showimage**
Running this executable creates ROS2 node, `showimage`, which subscribes to the `sensor_msgs::msg::Image` topic, `/image` and display the image messages in an **OpenCV** window.  

```bash
# Open new terminal
cd ~/demo_ws
source install/setup.bash
# Run showimage ROS2 node to display the cam2image sensor_msg::msg::Image messages.
ros2 run image_tools showimage
```
