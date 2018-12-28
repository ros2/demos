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

# OpenCV imports
import cv2

# Numpy imports
import numpy

# ROS2 imports
import rclpy
from rclpy.qos import qos_profile_default
import sensor_msgs.msg


def encoding2mat(encoding):
    encodings = {'mono': 1, 'bgr': 3, 'rgba': 4}
    for prefix, channels in encodings.items():
        if encoding.startswith(prefix):
            break
    else:
        raise ValueError('Unsupported encoding ' + encoding)

    types = {'8': 'uint8', '16': 'int16'}
    for prefix, dtype in types.items():
        if encoding[channels:] == prefix:
            break
    else:
        raise ValueError('Unsupported encoding ' + encoding)

    return dtype, channels


def main(args=None):
    # Pass command-line arguments to rclpy.
    rclpy.init(args=args)

    # Parse the command-line options.
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '-d', '--depth', dest='depth', action='store', default=qos_profile_default.depth, type=int,
        help='Queue depth')
    parser.add_argument(
        '-k', '--keep', dest='history_policy', action='store', default=qos_profile_default.history,
        type=int, choices=[0, 1],
        help='History QoS setting, 0 - keep last sample, 1 - keep all the samples')
    parser.add_argument(
        '-r', '--reliability', dest='reliability_policy', action='store',
        default=qos_profile_default.reliability, type=int, choices=[0, 1],
        help='Reliability QoS setting, 0 - best effort, 1 - reliable')
    parser.add_argument(
        '-s', '--show', dest='show_camera', action='store', default=1, type=int, choices=[0, 1],
        help='Show the camera stream')
    args = parser.parse_args()

    if args.show_camera == 1:
        cv2.namedWindow('showimage_py')

    # Initialize a ROS 2 node to subscribe to images read from the OpenCV interface to
    # the camera.
    node = rclpy.create_node('showimagepy')

    custom_qos_profile = qos_profile_default

    # Depth represents how many messages to store in history when the history policy is KEEP_LAST.
    custom_qos_profile.depth = args.depth

    # The reliability policy can be reliable, meaning that the underlying transport layer will try
    # ensure that every message gets received in order, or best effort, meaning that the transport
    # makes no guarantees about the order or reliability of delivery.
    custom_qos_profile.reliability = args.reliability_policy

    # The history policy determines how messages are saved until the message is taken by the reader
    # KEEP_ALL saves all messages until they are taken.
    # KEEP_LAST enforces a limit on the number of messages that are saved, specified by the "depth"
    # parameter.
    custom_qos_profile.history = args.history_policy

    def image_cb(msg):
        node.get_logger().info('Received image #' + msg.header.frame_id)

        if args.show_camera:
            dtype, n_channels = encoding2mat(msg.encoding)
            dtype = numpy.dtype(dtype)
            dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
            if n_channels == 1:
                frame = numpy.ndarray(
                    shape=(msg.height, msg.width),
                    dtype=dtype,
                    buffer=bytes(msg.data))
            else:
                frame = numpy.ndarray(
                    shape=(msg.height, msg.width, n_channels),
                    dtype=dtype,
                    buffer=bytes(msg.data))

            cv2.imshow('showimage_py', frame)
            # Draw the image to the screen and wait 1 millisecond
            cv2.waitKey(1)

    node.create_subscription(
        sensor_msgs.msg.Image, 'image', image_cb, qos_profile=custom_qos_profile)

    while rclpy.ok():
        rclpy.spin_once(node)


if __name__ == '__main__':
    main()
