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

from std_msgs.msg import Float32, Int64

DISPLAY_DASHBOARD = False

time_between_msgs = 0.3  # time in seconds between expected messages
stale_time = 3.0*time_between_msgs  # time in seconds after which a message is considered stale
time_between_rate_calculations = 0.5  # time in seconds between analysis of reception rates
allowed_latency = 1.5  # allowed time in seconds between expected and received messages


class MonitoredRobot:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.status = 'Offline'
        self.received_values = []
        self.reception_rate_over_time = []
        self.expected_value = None
        self.time_of_last_status = None
        self.initial_value = None
        self.status_changed = False
        self.timer_for_incrementing_expected_value = None

    def increment_expected_value(self):
        monitored_robots_lock.acquire()
        if self.expected_value is not None:
            self.expected_value += 1
        monitored_robots_lock.release()

    def timer_for_allowed_latency_callback(self):
        self.timer_for_allowed_latency.cancel()
        self.timer_for_incrementing_expected_value.reset()

    def robot_status_callback(self, msg):
        # print('%s: %s' % (self.robot_id, str(msg.data)))
        received_value = msg.data
        status = 'Alive'
        monitored_robots_lock.acquire()
        if self.expected_value is None:
            self.expected_value = received_value
            self.initial_value = received_value
            self.timer_for_allowed_latency.reset()
        if received_value == -1:
            status = 'Offline'
            self.timer_for_incrementing_expected_value.cancel()
            self.expected_value = None
        else:
            self.received_values.append(received_value)
        self.time_of_last_status = time.time()  # TODO: time stamp of msg
        status_changed = status != self.status
        self.status_changed |= status_changed  # don't clear the flag before check_status
        self.status = status
        monitored_robots_lock.release()

    def check_status(self, current_time=time.time()):
        if self.status != 'Offline':
            elapsed_time = current_time - self.time_of_last_status 
            if elapsed_time > stale_time*1.1:
                self.status_changed = self.status != 'Stale'
                self.status = 'Stale'

        status_changed = self.status_changed
        self.status_changed = False
        return status_changed

    def current_reception_rate(self):
        n = 5
        if self.status != 'Offline':
            expected_values = range(
                max(self.initial_value, self.expected_value - n + 1),
                self.expected_value + 1)
            # How many of the expected values have been received?
            count = len(set(expected_values) & set(self.received_values))
            rate = count / len(expected_values)
            return rate


class RobotMonitor:
    def __init__(self):
        self.status_changed = False
        self.status_topic_pattern = re.compile("(\w*)_status_?(\w*)")
        self.monitored_robots = {}
        self.publishers = {}
        self.reception_rate_topic_name = 'reception_rate'

    def add_monitored_robot(self, robot_id, node, qos_profile):
        # Create a subscription to the robot's status topic
        robot = MonitoredRobot(robot_id)
        topic_name = self.make_topic_name(robot_id, qos_profile)
        robot.topic_name = topic_name
        print('Subscribing to topic: %s' % topic_name)
        sub = node.create_subscription(
            Int64,
            topic_name,
            robot.robot_status_callback,
            qos_profile)

        # Create a timer for maintaining the expected value received from the robot
        timer_for_allowed_latency = node.create_timer(
            allowed_latency, robot.timer_for_allowed_latency_callback)
        timer_for_allowed_latency.cancel()
        timer = node.create_timer(time_between_msgs, robot.increment_expected_value)
        timer.cancel()
        monitored_robots_lock.acquire()
        robot.timer_for_incrementing_expected_value = timer
        robot.timer_for_allowed_latency = timer_for_allowed_latency
        self.monitored_robots[robot_id] = robot
        monitored_robots_lock.release()

        # Create a publisher for the reception rate of the robot
        self.publishers[robot_id] = node.create_publisher(
            Float32, self.reception_rate_topic_name + '/' + topic_name)

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
        monitored_robots_lock.acquire()
        for robot_id, robot in self.monitored_robots.items():
            status_changed = robot.check_status(current_time)
            any_status_changed |= status_changed
        monitored_robots_lock.release()
        return any_status_changed

    def output_status(self):
        print('---------------')
        monitored_robots_lock.acquire()
        for robot_id, robot in self.monitored_robots.items():
            print('%s: %s' % (robot_id, robot.status))
        monitored_robots_lock.release()
        print('---------------')

    def check_status(self):
        status_changed = self.update_robot_statuses()
        if status_changed:
            self.output_status()
        return status_changed

    def calculate_reception_rates(self):
        status_changed = self.check_status()
        monitored_robots_lock.acquire()
        for robot_id, robot in self.monitored_robots.items():
            rate = robot.current_reception_rate()
            robot.reception_rate_over_time.append(rate)
            if rate:
                rateMsg = Float32()
                rateMsg.data = rate
                self.publishers[robot_id].publish(rateMsg)
        monitored_robots_lock.release()


class RobotMonitorDashboard:

    def __init__(self, robot_monitor):
        self.robot_monitor = robot_monitor
        self.monitored_robots = []
        self.colors = "bgrcmykw"
        self.robot_count = 0
        self.reception_rate_plots = {}
        self.x_range = 150

        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        self.ax.axis([0, self.x_range, 0, 1.1])

    def add_monitored_robot(self, robot_id):
        y_data = [None] * self.x_range
        x_data = range(0, len(y_data))
        line, = self.ax.plot(x_data, y_data,
            '-', c=self.colors[self.robot_count % len(self.colors)], label=robot_id)
        self.ax.legend(loc='lower center')
        self.reception_rate_plots[robot_id] = line
        self.robot_count += 1
        self.monitored_robots.append(robot_id)

    def update_dashboard(self):
        status_changed = self.robot_monitor.check_status()
        self.robot_monitor.calculate_reception_rates()
        monitored_robots_lock.acquire()
        for robot_id, robot in self.robot_monitor.monitored_robots.items():
            robot_id = robot.topic_name
            if robot_id not in self.monitored_robots:
                self.add_monitored_robot(robot_id)
            y_data = robot.reception_rate_over_time[-self.x_range:]
            # Pad data so fits graph properly
            y_data = [None] * (self.x_range - len(y_data)) + y_data
            line = self.reception_rate_plots[robot_id]
            line.set_ydata(y_data)
        monitored_robots_lock.release()

        try:
            self.fig.canvas.draw()
        except:
            print('except interrupt')
            raise

rclpy.init()
robot_monitor = RobotMonitor()
monitored_robots_lock = Lock()

class RCLPYThread(Thread):
    def __init__(self):
        Thread.__init__(self)

    def run(self):
        self.node = rclpy.create_node('robot_monitor')
        rclpy_ok = True
        while rclpy.ok():
            # Check if there is a new robot online
            # TODO: use graph events
            topic_names_and_types = self.node.get_topic_names_and_types()
            for topic_name in topic_names_and_types.topic_names:
                topic_info = robot_monitor.get_topic_info(topic_name)
                if topic_info is None:
                    continue
                robot_name = topic_info['name']

                is_new_robot = robot_name and robot_name not in robot_monitor.monitored_robots
                if is_new_robot:
                    # Register new robot with the monitor
                    qos_profile = qos_profile_default
                    if topic_info['reliability'] == 'best_effort':
                        qos_profile = qos_profile_sensor_data
                    robot_monitor.add_monitored_robot(robot_name, self.node, qos_profile)

            if robot_monitor.monitored_robots:
                start_spin_time = time.time()
                rclpy.spin_once(self.node, 0.05)

    def stop(self):
        self.node.destroy_node()
        rclpy.shutdown()
        return


def main(argv=sys.argv[1:]):
    if DISPLAY_DASHBOARD:
        robot_monitor_dashboard = RobotMonitorDashboard(robot_monitor)

    try:
        thread = RCLPYThread()
        thread.start()

        last_time = time.time()

        while(1):
            now = time.time()
            if now - last_time > time_between_rate_calculations:
                last_time = now
                if DISPLAY_DASHBOARD:
                    robot_monitor_dashboard.update_dashboard()
                else:
                    robot_monitor.check_status()
                    robot_monitor.calculate_reception_rates()
        thread.join()

    except KeyboardInterrupt:
        thread.stop()
        thread.join()


if __name__ == '__main__':
    main()
