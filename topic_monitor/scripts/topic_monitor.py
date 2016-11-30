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
time_between_rate_calculations = 1  # time in seconds between analysis of reception rates
allowed_latency = 10  # allowed time in seconds between expected and received messages


class MonitoredTopic:
    def __init__(self, topic_id):
        self.topic_id = topic_id
        self.status = 'Offline'
        self.received_values = []
        self.reception_rate_over_time = []
        self.expected_value = None
        self.time_of_last_data = None
        self.initial_value = None
        self.status_changed = False
        self.timer_for_incrementing_expected_value = None

    def increment_expected_value(self):
        monitored_topics_lock.acquire()
        if self.expected_value is not None:
            self.expected_value += 1
        monitored_topics_lock.release()

    def timer_for_allowed_latency_callback(self):
        self.timer_for_allowed_latency.cancel()
        self.timer_for_incrementing_expected_value.reset()

    def topic_data_callback(self, msg):
        print('%s: %s' % (self.topic_id, str(msg.data)))
        received_value = msg.data
        status = 'Alive'
        monitored_topics_lock.acquire()
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
        self.time_of_last_data = time.time()  # TODO: time stamp of msg
        status_changed = status != self.status
        self.status_changed |= status_changed  # don't clear the flag before check_status
        self.status = status
        monitored_topics_lock.release()

    def check_status(self, current_time=time.time()):
        if self.status != 'Offline':
            elapsed_time = current_time - self.time_of_last_data 
            if elapsed_time > stale_time*1.1:
                self.status_changed = self.status != 'Stale'
                self.status = 'Stale'

        status_changed = self.status_changed
        self.status_changed = False
        return status_changed

    def current_reception_rate(self):
        rate = 0.0
        if self.status != 'Offline':
            expected_values = range(
                max(self.initial_value, self.expected_value - WINDOW_SIZE + 1),
                self.expected_value + 1)
            # How many of the expected values have been received?
            count = len(set(expected_values) & set(self.received_values))
            rate = count / len(expected_values)
        return rate



class TopicMonitor:
    def __init__(self):
        self.status_changed = False
        self.data_topic_pattern = re.compile("(\w*)_data_?(\w*)")
        self.monitored_topics = {}
        self.publishers = {}
        self.reception_rate_topic_name = 'reception_rate'

    def add_monitored_topic(self, topic_id, node, qos_profile):
        # Create a subscription to the topic
        monitored_topic = MonitoredTopic(topic_id)
        topic_name = self.make_topic_name(topic_id, qos_profile)
        monitored_topic.topic_name = topic_name
        print('Subscribing to topic: %s' % topic_name)
        sub = node.create_subscription(
            Int64,
            topic_name,
            monitored_topic.topic_data_callback,
            qos_profile)

        # Create a timer for maintaining the expected value received from the topic
        timer_for_allowed_latency = node.create_timer(
            allowed_latency, monitored_topic.timer_for_allowed_latency_callback)
        timer_for_allowed_latency.cancel()
        timer = node.create_timer(time_between_msgs, monitored_topic.increment_expected_value)
        timer.cancel()
        monitored_topics_lock.acquire()
        monitored_topic.timer_for_incrementing_expected_value = timer
        monitored_topic.timer_for_allowed_latency = timer_for_allowed_latency
        self.monitored_topics[topic_id] = monitored_topic
        monitored_topics_lock.release()

        # Create a publisher for the reception rate of the topic
        self.publishers[topic_id] = node.create_publisher(
            Float32, self.reception_rate_topic_name + '/' + topic_name)

    def make_topic_name(self, topic_id, qos_profile):
        best_effort = qos_profile.reliability is QoSReliabilityPolicy.RMW_QOS_POLICY_BEST_EFFORT
        topic_name = '{0}_data{1}'.format(
            topic_id, '_best_effort' if best_effort else '')
        return topic_name

    def get_topic_info(self, topic_name):
        match = re.search(self.data_topic_pattern, topic_name)
        if match and match.groups():
            topic_info = {'reliability': 'reliable'}
            topic_info['name'] = match.groups()[0]
            reliability = match.groups()[1]
            if reliability == 'best_effort': 
                topic_info['reliability'] = 'best_effort'
            return topic_info

    def update_topic_statuses(self):
        any_status_changed = False
        current_time = time.time()
        monitored_topics_lock.acquire()
        for topic_id, monitored_topic in self.monitored_topics.items():
            status_changed = monitored_topic.check_status(current_time)
            any_status_changed |= status_changed
        monitored_topics_lock.release()
        return any_status_changed

    def output_status(self):
        print('---------------')
        monitored_topics_lock.acquire()
        for topic_id, monitored_topic in self.monitored_topics.items():
            print('%s: %s' % (topic_id, monitored_topic.status))
        monitored_topics_lock.release()
        print('---------------')

    def check_status(self):
        status_changed = self.update_topic_statuses()
        if status_changed:
            self.output_status()
        return status_changed

    def calculate_reception_rates(self):
        status_changed = self.check_status()
        monitored_topics_lock.acquire()
        for topic_id, monitored_topic in self.monitored_topics.items():
            rate = monitored_topic.current_reception_rate()
            monitored_topic.reception_rate_over_time.append(rate)
            rateMsg = Float32()
            rateMsg.data = rate
            #TODO add lock here
            try:
                self.publishers[topic_id].publish(rateMsg)
            except KeyError:
                pass
        monitored_topics_lock.release()


class TopicMonitorDashboard:

    def __init__(self, topic_monitor):
        self.topic_monitor = topic_monitor
        self.monitored_topics = []
        self.colors = 'bgrcmykw'
        self.markers = 'o>sp*hDx+'
        self.topic_count = 0
        self.reception_rate_plots = {}
        self.x_range = 30
        self.x_data = [x * time_between_rate_calculations for x in range(0, self.x_range)]

        self.make_plot()

    def make_plot(self):
        plt.ion()
        self.fig = plt.figure()
        plt.title('Reception rate over time')
        plt.xlabel('Time (s)')
        plt.ylabel('Reception rate (last %i msgs)' % WINDOW_SIZE)
        self.ax = self.fig.add_subplot(111)
        self.ax.axis([0, self.x_range * time_between_rate_calculations, 0, 1.1])

        # Shrink axis' height to make room for legend
        shrink_amnt = 0.2
        box = self.ax.get_position()
        self.ax.set_position([box.x0, box.y0 + box.height * shrink_amnt,
            box.width, box.height * (1 - shrink_amnt)])


    def add_monitored_topic(self, topic_id):
        # Make first instance of the line so that we only have to update it later
        y_data = [None] * self.x_range
        line, = self.ax.plot(self.x_data, y_data,
            '-', color=self.colors[self.topic_count % len(self.colors)],
            marker=self.markers[self.topic_count % len(self.markers)], label=topic_id)
        self.reception_rate_plots[topic_id] = line
        self.ax.plot(self.x_data, [None] * self.x_range)

        # Update the plot x-axis labels
        if self.topic_count == 0:
            labels = ['t - ' + t.get_text() for t in reversed(self.ax.xaxis.get_ticklabels())]
            self.ax.xaxis.set_ticklabels(labels)

        # Update the plot legend to include the new line
        self.ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1),
        fancybox=True, shadow=True, ncol=2)

        self.topic_count += 1
        self.monitored_topics.append(topic_id)

    def update_dashboard(self):
        monitored_topics_lock.acquire()
        for topic_id, monitored_topic in self.topic_monitor.monitored_topics.items():
            topic_id = monitored_topic.topic_name
            if topic_id not in self.monitored_topics:
                self.add_monitored_topic(topic_id)
            y_data = monitored_topic.reception_rate_over_time[-self.x_range:]
            # Pad data so it's the right size
            y_data = [None] * (self.x_range - len(y_data)) + y_data
            line = self.reception_rate_plots[topic_id]
            line.set_ydata(y_data)
        monitored_topics_lock.release()

        self.fig.canvas.draw()

topic_monitor = TopicMonitor()
monitored_topics_lock = Lock()
WINDOW_SIZE = 20

class RCLPYThread(Thread):
    def __init__(self):
        rclpy.init()
        Thread.__init__(self)

    def run(self):
        self.node = rclpy.create_node('topic_monitor')
        while rclpy.ok():
            # Check if there is a new topic online
            # TODO: use graph events
            topic_names_and_types = self.node.get_topic_names_and_types()
            for topic_name in topic_names_and_types.topic_names:
                topic_info = topic_monitor.get_topic_info(topic_name)
                if topic_info is None:
                    continue
                topic_name = topic_info['name']

                is_new_topic = topic_name and topic_name not in topic_monitor.monitored_topics
                if is_new_topic:
                    # Register new topic with the monitor
                    qos_profile = qos_profile_default
                    if topic_info['reliability'] == 'best_effort':
                        qos_profile = qos_profile_sensor_data
                    topic_monitor.add_monitored_topic(topic_name, self.node, qos_profile)

            if topic_monitor.monitored_topics:
                start_spin_time = time.time()
                rclpy.spin_once(self.node, 0.05)

    def stop(self):
        self.node.destroy_node()
        rclpy.shutdown()
        return


def main():
    global WINDOW_SIZE

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--display',
        dest='display_dashboard',
        action='store_true',
        default=False,
        help='Display the reception rate of topics')

    parser.add_argument(
        '-n',
        type=int,
        nargs='?',
        dest='window_size',
        action='store',
        help='Number of messages in reception rate calculation')
    args = parser.parse_args()

    if args.window_size:
        WINDOW_SIZE = args.window_size

    if args.display_dashboard:
        topic_monitor_dashboard = TopicMonitorDashboard(topic_monitor)

    try:
        thread = RCLPYThread()
        thread.start()

        last_time = time.time()

        while(1):
            now = time.time()
            if now - last_time > time_between_rate_calculations:
                last_time = now
                topic_monitor.check_status()
                topic_monitor.calculate_reception_rates()
                if args.display_dashboard:
                    topic_monitor_dashboard.update_dashboard()
        thread.join()

    except KeyboardInterrupt:
        thread.stop()
        thread.join()


if __name__ == '__main__':
    main()
