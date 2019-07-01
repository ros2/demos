# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# System imports
import argparse
import sys

# Local imports
import burger_py

# OpenCV imports
import cv2

# ROS2 imports
import rclpy
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
import sensor_msgs.msg
import std_msgs.msg


# Convert an OpenCV matrix encoding type to a string format recognized by
# sensor_msgs::Image
def mat2encoding(frame):
    encoding = ''

    encodings = {1: 'mono', 3: 'bgr', 4: 'rgba'}
    for channels, prefix in encodings.items():
        if frame.shape[2] == channels:
            encoding += prefix
            break
    else:
        raise ValueError('Unsupported frame shape %d' % (frame.shape[2]))

    types = {'uint8': '8', 'int16': '16'}
    for dtype, num in types.items():
        if frame.dtype == dtype:
            encoding += num
            break
    else:
        raise ValueError('Unsupported frame type ' + frame.dtype)

    return encoding


# Convert an OpenCV matrix to a ROS Image message.
def convert_frame_to_message(frame, frame_id, msg):
    msg.height = frame.shape[0]
    msg.width = frame.shape[1]
    msg.encoding = mat2encoding(frame)
    msg.data = frame.data.tobytes()
    msg.step = int(len(msg.data) / msg.height)
    msg.header.frame_id = str(frame_id)


def main(args=None):
    # Pass command-line arguments to rclpy.
    rclpy.init(args=args)

    # Parse the command-line options.
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-b', '--burger', dest='burger_mode', action='store_true', default=False,
        help='Produce images of burgers rather than connecting to a camera')
    parser.add_argument(
        '-d', '--depth', dest='depth', action='store', default=qos_profile_system_default.depth, type=int,
        help='Queue depth')
    parser.add_argument(
        '-f', '--frequency', dest='frequency', action='store', default=30, type=int,
        help='Publish frequency in Hz')
    parser.add_argument(
        '-k', '--keep', dest='history_policy', action='store', default=qos_profile_system_default.history,
        type=int, choices=[0, 1],
        help='History QoS setting, 0 - keep last sample, 1 - keep all the samples')
    parser.add_argument(
        '-r', '--reliability', dest='reliability_policy', action='store',
        default=qos_profile_system_default.reliability, type=int, choices=[0, 1],
        help='Reliability QoS setting, 0 - best effort, 1 - reliable')
    parser.add_argument(
        '-s', '--show', dest='show_camera', action='store', default=0, type=int, choices=[0, 1],
        help='Show the camera stream')
    parser.add_argument(
        '-t', '--topic', dest='topic', action='store', default='image', type=str,
        help='Topic to publish on')
    parser.add_argument(
        '-x', '--width', dest='width', action='store', default=320, type=int,
        help='Image width')
    parser.add_argument(
        '-y', '--height', dest='height', action='store', default=240, type=int,
        help='Image height')
    args = parser.parse_args()

    # Initialize a ROS 2 node to publish images read from the OpenCV interface to
    # the camera.
    node = rclpy.create_node('cam2imagepy')

    # Set the parameters of the quality of service profile.  Initialize as the
    # default profile and set the QoS parameters specified on the command line.
    custom_camera_qos_profile = qos_profile_system_default

    # Depth represents how many messages to store in history when the history policy is KEEP_LAST
    custom_camera_qos_profile.depth = args.depth

    # The reliability policy can be reliable, meaning that the underlying transport layer will try
    # ensure that every message gets received in order, or best effort, meaning that the transport
    # makes no guarantees about the order or reliability of delivery.
    custom_camera_qos_profile.reliability = args.reliability_policy

    # The history policy determines how messages are saved until the message is taken by the reader
    # KEEP_ALL saves all messages until they are taken.
    # KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth"
    # parameter.
    custom_camera_qos_profile.history = args.history_policy

    node.get_logger().info("Publishing data on topic '%s'" % (args.topic))

    # Create the image publisher with our custom QoS profile.
    pub = node.create_publisher(
        sensor_msgs.msg.Image, args.topic,
        qos_profile=custom_camera_qos_profile)

    is_flipped = False

    msg = sensor_msgs.msg.Image()
    msg.is_bigendian = False

    def flip_image_cb(msg):
        nonlocal is_flipped

        is_flipped = msg.data

        output = 'on' if is_flipped else 'off'

        node.get_logger().info('Set flip mode to: ' + output)

    custom_flip_qos_profile = qos_profile_sensor_data
    custom_flip_qos_profile.depth = 10
    node.create_subscription(
        std_msgs.msg.Bool, 'flip_image', flip_image_cb, qos_profile=custom_flip_qos_profile)

    if args.burger_mode:
        burger_cap = burger_py.Burger()
    else:
        # Initialize OpenCV video capture stream.  Always open device 0.
        cam_cap = cv2.VideoCapture(0)

        # Set the width and height based on command-line arguments.
        cam_cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cam_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        if not cam_cap.isOpened():
            node.get_logger().fatal('Could not open video stream')
            sys.exit(1)

    # Our main event loop will spin until the user presses CTRL-C to exit.
    frame_number = 1
    while rclpy.ok():
        # Get the frame from the video capture.
        frame = None
        if args.burger_mode:
            frame = burger_cap.render_burger(args.width, args.height)
        else:
            ret, frame = cam_cap.read()

        # Check if the frame was grabbed correctly.
        if frame is not None:
            # Convert to a ROS image
            if is_flipped:
                # Flip the frame if needed
                flipped_frame = cv2.flip(frame, 1)
                convert_frame_to_message(flipped_frame, frame_number, msg)
            else:
                convert_frame_to_message(frame, frame_number, msg)

            if args.show_camera == 1:
                # Show the image in a window called 'cam2image_py', if requested.
                cv2.imshow('cam2image_py', frame)
                # Draw the image to the screen and wait 1 millisecond
                cv2.waitKey(1)

            # Publish the image message and increment the frame_id.
            node.get_logger().info('Publishing image #%d' % (frame_number))
            pub.publish(msg)
            frame_number += 1

        # Do some work in rclpy and wait for more to come in.
        rclpy.spin_once(node, timeout_sec=1.0 / float(args.frequency))


if __name__ == '__main__':
    main()
