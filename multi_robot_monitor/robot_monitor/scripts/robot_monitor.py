#!/usr/bin/env python3

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
import re
import sys
from threading import Lock, Thread
import time

import rclpy
from rclpy.qos import qos_profile_default, qos_profile_sensor_data, QoSReliabilityPolicy

import matplotlib.pyplot as plt

from std_msgs.msg import Int64


time_between_statuses = 0.6  # time in seconds between expected status publications
stale_time = 3.0*time_between_statuses  # time in seconds after which a status is considered stale
time_between_display_updates = 0.9


class MonitoredRobot:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.status = 'Offline'
        self.received_values = []
        self.expected_value = None
        self.time_of_last_status = None
        self.initial_value = None
        self.status_changed = True

    def increment_expected_value(self):
        if self.expected_value:
            self.expected_value += 1

    def robot_status_callback(self, msg):
        received_value = msg.data
        status = 'Alive'
        if self.expected_value is None:
            self.expected_value = received_value
            self.initial_value = received_value
        if received_value == -1:
            status = 'Offline'
            self.expected_value = None
        else:
            self.received_values.append(received_value)
        self.time_of_last_status = time.time()  # TODO: time stamp of msg
        self.status_changed = status != self.status
        self.status = status

    def check_status(self, current_time=time.time()):
        if self.status != 'Offline':
            elapsed_time = current_time - self.time_of_last_status 
            if elapsed_time > stale_time*1.1:
                self.status = 'Stale'
                self.status_changed = True

        status_changed = self.status_changed
        self.status_chagned = False
        return status_changed

    def current_reception_rate(self):
        n = 10
        if self.status != 'Offline':
            last_received = self.received_values[int(-n*1.5):]
            # How many of the expected values have been received?
            expected_values = range(
                max(self.initial_value, self.expected_value - n + 1),
                self.expected_value + 1)
            count = len(set(expected_values) & set(last_received))
            rate = count / len(expected_values)
            return rate


class RobotMonitor:
    def __init__(self):
        self.status_changed = False
        self.status_topic_pattern = re.compile("(\w*)_status_?(\w*)")
        self.monitored_robots = {}

    def add_monitored_robot(self, robot_id, node, qos_profile):
        # Create a subscription to the robot's status topic
        robot = MonitoredRobot(robot_id)
        topic_name = self.make_topic_name(robot_id, qos_profile)
        print('Subscribing to topic: %s' % topic_name)
        sub = node.create_subscription(
            Int64,
            topic_name,
            robot.robot_status_callback,
            qos_profile)

        allowed_latency = 0.3
        # TODO get rid of this awful hack
        time.sleep(allowed_latency)
        timer = node.create_timer(time_between_statuses, robot.increment_expected_value)
        self.monitored_robots[robot_id] = robot

    def make_topic_name(self, robot_id, qos_profile):
        best_effort = qos_profile.reliability is QoSReliabilityPolicy.RMW_QOS_POLICY_BEST_EFFORT
        topic_name = '{0}_status{1}'.format(
            robot_id, '_best_effort' if best_effort else '')
        return topic_name

    def get_topic_info(self, topic_name):
        match = re.search(self.status_topic_pattern, topic_name)
        if match and match.groups():
            topic_info = {'reliability': 'reliable'}
            topic_info['name'] = match.groups()[0]
            reliability = match.groups()[1]
            if reliability == 'best_effort': 
                topic_info['reliability'] = 'best_effort'
            return topic_info

    def update_robot_statuses(self):
        any_status_changed = False
        current_time = time.time()
        for robot_id, robot in self.monitored_robots.items():
            status_changed = robot.check_status(current_time)
            any_status_changed |= status_changed
        return any_status_changed

    def output_status(self):
        print('---------------')
        for robot_id, robot in self.monitored_robots.items():
            print('%s: %s' % (robot_id, robot.status))
        print('---------------')

    def check_status(self):
        status_changed = self.update_robot_statuses()
        return status_changed


class RobotMonitorDashboard:

    def __init__(self, robot_monitor):
        self.robot_monitor = robot_monitor
        self.monitored_robots = []
        self.colors = "bgrcmykw"
        self.robot_count = 0
        self.reception_rate_plots = {}
        self.reception_rates_over_time = {}
        self.x_range = 150

        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.axis([0, self.x_range, 0, 1.1])

    def add_monitored_robot(self, robot_id):
        line, = self.ax.plot([], [],
            '-', c=self.colors[self.robot_count % len(self.colors)], label=robot_id)
        self.ax.legend()
        self.reception_rate_plots[robot_id] = line
        self.robot_count += 1
        self.reception_rates_over_time[robot_id] = []
        self.monitored_robots.append(robot_id)

    def update_dashboard(self):
        status_changed = self.robot_monitor.check_status()

        for robot_id, robot in self.robot_monitor.monitored_robots.items():
            if robot_id not in self.monitored_robots:
                self.add_monitored_robot(robot_id)
            print(self.reception_rates_over_time)
            reception_rate_over_time = self.reception_rates_over_time[robot_id]
            reception_rate_over_time.append(robot.current_reception_rate())
            y_data = reception_rate_over_time[-self.x_range:]
            x_data = range(0, len(y_data))
            line = self.reception_rate_plots[robot_id]
            line.set_xdata(x_data)
            line.set_ydata(y_data)
            try:
               self.fig.canvas.draw()
            except:
                print('except interrupt')
                raise


robot_monitor = RobotMonitor()
robot_monitor_dashboard = RobotMonitorDashboard(robot_monitor)
monitor_lock = Lock()

class RCLPYThread(Thread):
    def __init__(self):
        Thread.__init__(self)

    def run(self):
        rclpy.init()
        self.node = rclpy.create_node('robot_monitor')
        rclpy_ok = True
        while rclpy.ok():
            # Check if there is a new robot online
            # TODO: use graph events
            topic_names_and_types = self.node.get_topic_names_and_types()
            for topic_name in topic_names_and_types.topic_names:
                topic_info = robot_monitor.get_topic_info(topic_name)
                robot_name = topic_info['name']

                is_new_robot = robot_name and robot_name not in robot_monitor.monitored_robots
                if is_new_robot:
                    # Register new robot with the monitor
                    qos_profile = qos_profile_default
                    if topic_info['reliability'] == 'best_effort':
                        qos_profile = qos_profile_sensor_data
                    monitor_lock.acquire()
                    robot_monitor.add_monitored_robot(robot_name, self.node, qos_profile)
                    monitor_lock.release()

            if robot_monitor.monitored_robots:
                rclpy.spin_once(self.node)

    def stop(self):
        monitor_lock.release()
        self.node.destroy_node()
        rclpy.shutdown()
        return


def main(argv=sys.argv[1:]):

    try:
        thread = RCLPYThread()
        thread.start()

        last_time = time.time()
        while(1):
            now = time.time()
            if now - last_time > time_between_display_updates:
                monitor_lock.acquire()
                robot_monitor_dashboard.update_dashboard()
                monitor_lock.release()
        thread.join()

    except KeyboardInterrupt:
        thread.stop()
        thread.join()


if __name__ == '__main__':
    main()
