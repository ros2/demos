# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import argparse
import sys
from time import sleep

import rospy

from std_msgs.msg import Header  # Note: this must come from a ROS 1 path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'data_name', nargs='?', default='ros1',
        help='Name of the data (must comply with ROS topic rules)')

    parser.add_argument(
        '--period', type=float, default=0.5, action='store',
        help='Time in seconds between messages')

    parser.add_argument(
        '--end-after', type=int, action='store',
        help='Script will exit after publishing this many messages')

    args = parser.parse_args()

    topic_name = '%s_data' % args.data_name
    data_pub = rospy.Publisher(topic_name, Header, queue_size=10)
    rospy.init_node('%s_pub' % topic_name, anonymous=True)

    msg = Header()
    cycle_count = 0

    def publish_msg(val):
        msg.frame_id = str(val)
        data_pub.publish(msg)
        rospy.loginfo('Publishing: "{0}"'.format(val))
        sys.stdout.flush()  # this is to get the output to show immediately when using Launch

    while not rospy.is_shutdown():
        if args.end_after is not None and cycle_count >= args.end_after:
            publish_msg(-1)
            exit(0)

        publish_msg(cycle_count)
        cycle_count += 1

        try:
            sleep(args.period)
        except KeyboardInterrupt:
            publish_msg(-1)
            raise


if __name__ == '__main__':
    main()
