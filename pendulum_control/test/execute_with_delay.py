# Copyright 2015 Open Source Robotics Foundation, Inc.
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
import time

from ros2run.api import run_executable


def main():
    parser = argparse.ArgumentParser(description='Delay and execute an executable.')
    parser.add_argument('delay', metavar='T', type=float, help='Start delay in ms')
    parser.add_argument('executable', metavar='exec', type=str, nargs='+',
                        help='Executable to execute, with a variable number of arguments.')
    args = parser.parse_args()

    delay_time = args.delay * 0.001
    time.sleep(delay_time)
    # use ros2run api to handle KeyboardInterrupt
    return run_executable(path=args.executable[0], argv=args.executable[1:])


if __name__ == '__main__':
    sys.exit(main())
