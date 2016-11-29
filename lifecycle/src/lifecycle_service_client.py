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

import rclpy

from lifecycle_msgs.srv import GetState, ChangeState

def main(args=None):
    rclpy.init(args)

    node = rclpy.create_node('lc_client_py')
    cli = node.create_client(GetState, 'lifecycle_manager__get_state')

    while rclpy.ok():
        req = GetState.Request()
        req.node_name = 'lc_talker'
        cli.call(req)
        cli.wait_for_future()
        print('lc_talker is in state %d' % cli.response.state)

if __name__ == '__main__':
    main();
