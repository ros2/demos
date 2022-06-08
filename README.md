# demos_custom

Clearbot

In order to run multiple cam2image and lock the camera device on specific USB port of the Jetson, image_tools_custom is created.

## 1. Lock camera
Default cameras are two Arducams
Please replace "/media/clearbot/eeffe203-9376-4abb-9428-2bb29c2e299e1" with the path of your 16GB emmc

```bash
	sudo cp /home/clearbot/ws/src/demos_custom/image_tools/etc/udev/rules.d/80-uvccam.rules /etc/udev/rules.d/
	sudo cp /home/clearbot/ws/src/demos_custom/image_tools/etc/udev/rules.d/80-uvccam.rules /media/clearbot/eeffe203-9376-4abb-9428-2bb29c2e299e1/etc/udev/rules.d
	sudo udevadm control --reload-rules
	sudo service udev restart
	sudo udevadm trigger
```

Check the symlink of the cameras
```bash
	ls -l /dev/video*
```
You will see /dev/videox and /dev/videoy which are linking to the video device
## 2. Run cam2image_custom_0 and cam2image_custom_2
Open a new terminal
```bash
	ros2 run image_tools_custom cam2image_custom_0
```

Open another new terminal
```bash
	ros2 run image_tools_custom cam2image_custom_2
```

