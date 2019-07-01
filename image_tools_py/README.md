## Image Tools Demo in Python

This is a demonstration of the Quality of Service (QoS) features of ROS 2 using Python.
There are two programs implemented here: cam2image_py, and showimage_py.

### CAM2IMAGE_PY
This is a Python program that will take data from an attached camera, and publish the
data to a topic called "image", with the type sensor_msgs/msg/Image.  If a camera
isn't available, this program can also generate a default image and smoothly "move"
it across the screen, simulating motion.  The usage output from the program looks like
this:

```
usage: cam2image_py [-h] [-b] [-d DEPTH] [-f FREQUENCY] [-k {0,1}] [-r {0,1}]
                    [-s {0,1}] [-t TOPIC] [-x WIDTH] [-y HEIGHT]

optional arguments:
  -h, --help            show this help message and exit
  -b, --burger          Produce images of burgers rather than connecting to a
                        camera (default: False)
  -d DEPTH, --depth DEPTH
                        Queue depth (default: 10)
  -f FREQUENCY, --frequency FREQUENCY
                        Publish frequency in Hz (default: 30)
  -k {0,1}, --keep {0,1}
                        History QoS setting, 0 - keep last sample, 1 - keep
                        all the samples (default: 1)
  -r {0,1}, --reliability {0,1}
                        Reliability QoS setting, 0 - best effort, 1 - reliable
                        (default: 1)
  -t TOPIC, --topic TOPIC
                        Topic to publish on (default: image)

  -s {0,1}, --show {0,1}
                        Show the camera stream (default: 0)
  -x WIDTH, --width WIDTH
                        Image width (default: 320)
  -y HEIGHT, --height HEIGHT
                        Image height (default: 240)
```

The -d, -k, and -r parameters control various aspects of the QoS implementation, and
are the most interesting to play with when testing out QoS.

Note that this program also subscribes to a topic called "flip_image" of type
std_msgs/msg/Bool.  If flip_image is set to False, the data coming out of the camera
is sent as usual.  If flip_image is set to True, the data coming out of the camera is
flipped around the Y axis.

If the -s parameter is set to 1, then this program opens up a (local) window to show
the images that are being published.  However, these images are *not* coming in through
the ROS 2 pub/sub model, so this window cannot show off the QoS parameters (it is mostly
useful for debugging).  See SHOWIMAGE_PY below for a program that can show QoS over the
pub/sub model.

### SHOWIMAGE_PY
This is a Python program that subscribes to the "image" topic, waiting for data.  As
new data comes in, this program accepts the data and can optionally render it to
the screen.  The usage output from the program looks like this:

usage: showimage_py [-h] [-d QUEUE_DEPTH] [-k {0,1}] [-r {0,1}] [-s {0,1}]
                    [-t TOPIC]

optional arguments:
  -h, --help            show this help message and exit
  -d QUEUE_DEPTH, --depth QUEUE_DEPTH
                        Queue depth (default: 10)
  -k {0,1}, --keep {0,1}
                        History QoS setting, 0 - keep last sample, 1 - keep
                        all the samples (default: 1)
  -r {0,1}, --reliability {0,1}
                        Reliability QoS setting, 0 - best effort, 1 - reliable
                        (default: 1)
  -s {0,1}, --show {0,1}
                        Show the camera stream (default: 1)
  -t TOPIC, --topic TOPIC
                        use topic TOPIC instead of the default (default: image)

The -d, -k, and -r parameters control various aspects of the QoS implementation, and
are the most interesting to play with when testing out QoS.

If the -s parameter is set to 1, then this program opens up a window to show the images
that have been received over the ROS 2 pub/sub model.  This program should be used
in conjunction with cam2image_py to demonstrate the ROS 2 QoS capabilities over lossy/slow
links.

### EXAMPLE USAGE
To use the above programs, you would run them something like the following:

# In the first terminal, run the data publisher.  This will connect to the 1st camera
# available, and print out "Publishing image #" for each image it publishes.
```
$ ros2 run image_tools_py cam2image_py
```

# If you don't have a local camera, you can use the -b parameter to generate data on
# the fly rather than get data from a camera:
```
$ ros2 run image_tools_py cam2image_py -b
```

# In a second terminal, run the data subscriber.  This will subscribe to the "image"
# topic and render any frames it receives.
```
$ ros2 run image_tools_py showimage_py
```
