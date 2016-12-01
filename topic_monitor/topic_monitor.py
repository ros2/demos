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
import importlib
import re
from threading import Lock, Thread
import time

import rclpy
from rclpy.qos import qos_profile_default, qos_profile_sensor_data

from std_msgs.msg import Float32


time_between_msgs = 0.3               # time in seconds between expected messages
stale_time = 3.0 * time_between_msgs  # time in seconds after which a message is considered stale
time_between_rate_calculations = 1    # time in seconds between analysis of reception rates
allowed_latency = 10                  # allowed time in seconds between expected and received msgs


class MonitoredTopic:
    """Monitor for the reception rate and status of a single topic."""

    def __init__(self, topic_id, lock=None):
        self.topic_id = topic_id
        self.status = 'Offline'
        self.received_values = []
        self.reception_rate_over_time = []
        self.expected_value = None
        self.time_of_last_data = None
        self.initial_value = None
        self.status_changed = False
        self.timer_for_incrementing_expected_value = None
        self.lock = lock or Lock()

    def increment_expected_value(self):
        with self.lock:
            if self.expected_value is not None:
                self.expected_value += 1

    def timer_for_allowed_latency_callback(self):
        self.timer_for_allowed_latency.cancel()
        self.timer_for_incrementing_expected_value.reset()

    def topic_data_callback(self, msg):
        print('%s: %s' % (self.topic_id, str(msg.data)))
        received_value = int(msg.data)
        status = 'Alive'
        with self.lock:
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

    def check_status(self, current_time=time.time()):
        if self.status != 'Offline':
            elapsed_time = current_time - self.time_of_last_data
            if elapsed_time > stale_time * 1.1:
                self.status_changed = self.status != 'Stale'
                self.status = 'Stale'

        status_changed = self.status_changed
        self.status_changed = False
        return status_changed

    def current_reception_rate(self, window_size):
        rate = 0.0
        if self.status != 'Offline':
            expected_values = range(
                max(self.initial_value, self.expected_value - window_size + 1),
                self.expected_value + 1)
            # How many of the expected values have been received?
            count = len(set(expected_values) & set(self.received_values))
            rate = count / len(expected_values)
        return rate


class TopicMonitor:
    """Monitor of a set of topics that match a specified topic name pattern."""

    def __init__(self, window_size):
        self.data_topic_pattern = re.compile("((?P<data_name>\w*)_data_?(?P<reliability>\w*))")
        self.status_changed = False
        self.monitored_topics = {}
        self.publishers = {}
        self.reception_rate_topic_name = 'reception_rate'
        self.window_size = window_size
        self.monitored_topics_lock = Lock()

    def add_monitored_topic(self, topic_type, topic_name, node, qos_profile):
        # Create a subscription to the topic
        monitored_topic = MonitoredTopic(topic_name, lock=self.monitored_topics_lock)
        monitored_topic.topic_name = topic_name
        print('Subscribing to topic: %s' % topic_name)
        sub = node.create_subscription(
            topic_type,
            topic_name,
            monitored_topic.topic_data_callback,
            qos_profile)

        # Create a timer for maintaining the expected value received on the topic
        timer_for_allowed_latency = node.create_timer(
            allowed_latency, monitored_topic.timer_for_allowed_latency_callback)
        timer_for_allowed_latency.cancel()
        timer = node.create_timer(time_between_msgs, monitored_topic.increment_expected_value)
        timer.cancel()
        with self.monitored_topics_lock:
            monitored_topic.timer_for_incrementing_expected_value = timer
            monitored_topic.timer_for_allowed_latency = timer_for_allowed_latency
            self.monitored_topics[topic_name] = monitored_topic

            # Create a publisher for the reception rate of the topic
            self.publishers[topic_name] = node.create_publisher(
                Float32, self.reception_rate_topic_name + '/' + topic_name)

    def get_topic_info(self, topic_name):
        """Infer topic info (e.g. QoS reliability) from a topic name."""
        match = re.search(self.data_topic_pattern, topic_name)
        if match and match.groups():
            if match.groups()[0] != topic_name:
                # Only part of the topic name matches
                return None

            topic_info = {'reliability': 'reliable'}
            topic_info['topic_name'] = topic_name
            topic_info['data_name'] = match.group('data_name')

            reliability = match.group('reliability')
            if reliability == 'best_effort':
                topic_info['reliability'] = 'best_effort'
            return topic_info

    def is_supported_msg_type(self, message_type):
        supported_data_types = [int, float]
        if not hasattr(message_type, 'data'):
            return False
        dummy_msg = message_type()
        return type(dummy_msg.data) in supported_data_types

    def update_topic_statuses(self):
        any_status_changed = False
        current_time = time.time()
        with self.monitored_topics_lock:
            for topic_id, monitored_topic in self.monitored_topics.items():
                status_changed = monitored_topic.check_status(current_time)
                any_status_changed |= status_changed
        return any_status_changed

    def output_status(self):
        print('---------------')
        with self.monitored_topics_lock:
            for topic_id, monitored_topic in self.monitored_topics.items():
                print('%s: %s' % (topic_id, monitored_topic.status))
        print('---------------')

    def check_status(self):
        status_changed = self.update_topic_statuses()
        if status_changed:
            self.output_status()
        return status_changed

    def calculate_reception_rates(self):
        with self.monitored_topics_lock:
            for topic_id, monitored_topic in self.monitored_topics.items():
                rate = monitored_topic.current_reception_rate(self.window_size)
                monitored_topic.reception_rate_over_time.append(rate)
                rateMsg = Float32()
                rateMsg.data = rate
                self.publishers[topic_id].publish(rateMsg)

    def get_window_size(self):
        return self.window_size


class TopicMonitorDisplay:
    """Display of the monitored topic reception rates."""

    def __init__(self, topic_monitor):
        self.topic_monitor = topic_monitor
        self.monitored_topics = []
        self.colors = 'bgrcmykw'
        self.markers = 'o>sp*hDx+'
        self.topic_count = 0
        self.reception_rate_plots = {}
        self.x_range = 30
        self.x_data = [x * time_between_rate_calculations for x in range(self.x_range)]

        self.make_plot()

    def make_plot(self):
        self.fig = plt.figure()
        plt.title('Reception rate over time')
        plt.xlabel('Time (s)')
        plt.ylabel('Reception rate (last %i msgs)' % self.topic_monitor.get_window_size())
        self.ax = self.fig.add_subplot(111)
        self.ax.axis([0, self.x_range * time_between_rate_calculations, 0, 1.1])

        # Shrink axis' height to make room for legend
        shrink_amnt = 0.2
        box = self.ax.get_position()
        self.ax.set_position(
            [box.x0, box.y0 + box.height * shrink_amnt, box.width, box.height * (1 - shrink_amnt)])

    def add_monitored_topic(self, topic_id):
        # Make first instance of the line so that we only have to update it later
        y_data = [None] * self.x_range
        line, = self.ax.plot(
            self.x_data, y_data, '-', color=self.colors[self.topic_count % len(self.colors)],
            marker=self.markers[self.topic_count % len(self.markers)], label=topic_id)
        self.reception_rate_plots[topic_id] = line
        self.ax.plot(self.x_data, [None] * self.x_range)

        # Update the plot x-axis labels
        if self.topic_count == 0:
            labels = ['t - ' + t.get_text() for t in reversed(self.ax.xaxis.get_ticklabels())]
            self.ax.xaxis.set_ticklabels(labels)

        # Update the plot legend to include the new line
        self.ax.legend(
            loc='upper center', bbox_to_anchor=(0.5, -0.1), fancybox=True, shadow=True, ncol=2)

        self.topic_count += 1
        self.monitored_topics.append(topic_id)

    def update_display(self):
        with self.topic_monitor.monitored_topics_lock:
            for topic_id, monitored_topic in self.topic_monitor.monitored_topics.items():
                topic_id = monitored_topic.topic_name
                if topic_id not in self.monitored_topics:
                    self.add_monitored_topic(topic_id)
                y_data = monitored_topic.reception_rate_over_time[-self.x_range:]
                # Pad data so it's the right size
                y_data = [None] * (self.x_range - len(y_data)) + y_data
                line = self.reception_rate_plots[topic_id]
                line.set_ydata(y_data)

        self.fig.canvas.draw()
        plt.pause(0.0001)
        plt.show(block=False)


class DataReceivingThread(Thread):
    def __init__(self, topic_monitor):
        super(DataReceivingThread, self).__init__()
        self.topic_monitor = topic_monitor

    def run(self):
        self.node = rclpy.create_node('topic_monitor')
        run_topic_listening(self.node, self.topic_monitor)

    def stop(self):
        self.node.destroy_node()
        rclpy.shutdown()


def get_msg_module_from_name(message_type_name):
    """Try to import the module associated with a message name."""
    separator_idx = message_type_name.find('/')
    message_package = message_type_name[:separator_idx]
    message_name = message_type_name[separator_idx + 1:]
    module = importlib.import_module(message_package + '.msg')
    msg_module = getattr(module, message_name)
    return msg_module


def run_topic_listening(node, topic_monitor):
    """Subscribe to relevant topics and manage the data received from susbcriptions."""
    while rclpy.ok():
        # Check if there is a new topic online
        # TODO: use graph events
        topic_names_and_types = node.get_topic_names_and_types()

        it = zip(topic_names_and_types.topic_names, topic_names_and_types.type_names)
        for topic_name, type_name in it:
            # Infer the appropriate QoS profile from the topic name
            topic_info = topic_monitor.get_topic_info(topic_name)
            if topic_info is None:
                # The topic is not for being monitored
                continue

            # Check that the message type of the topic is supported
            try:
                topic_type = get_msg_module_from_name(type_name)
            except ImportError:
                continue
            if not topic_monitor.is_supported_msg_type(topic_type):
                continue

            is_new_topic = topic_name and topic_name not in topic_monitor.monitored_topics
            if is_new_topic:
                # Register new topic with the monitor
                qos_profile = qos_profile_default
                if topic_info['reliability'] == 'best_effort':
                    qos_profile = qos_profile_sensor_data
                topic_monitor.add_monitored_topic(topic_type, topic_name, node, qos_profile)

        if topic_monitor.monitored_topics:
            rclpy.spin_once(node, timeout_sec=0.05)


def process_received_data(topic_monitor, show_display=False):
    """Process data that has been received from topic subscriptions."""
    if show_display:
        topic_monitor_display = TopicMonitorDisplay(topic_monitor)

    last_time = time.time()
    while(True):
        now = time.time()
        if now - last_time > time_between_rate_calculations:
            last_time = now
            topic_monitor.check_status()
            topic_monitor.calculate_reception_rates()
            if show_display:
                topic_monitor_display.update_display()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--display',
        dest='show_display',
        action='store_true',
        default=False,
        help='Display the reception rate of topics')

    parser.add_argument(
        '-n',
        type=int,
        nargs='?',
        dest='window_size',
        default=20,
        action='store',
        help='Number of messages in reception rate calculation')

    args = parser.parse_args()
    if args.show_display:
        global plt
        import matplotlib.pyplot as plt

    rclpy.init()
    topic_monitor = TopicMonitor(args.window_size)

    try:
        # Run two infinite loops simultaneously: one for receiving data (subscribing to topics and
        # handling callbacks), and another for processing the received data (calculating the
        # reception rate and publishing/displaying it).

        # Since the display needs to happen in the main thread, we run the "data processing" loop
        # in the main thread and run the "data receiving" loop in a secondary thread.

        # Start the "data receiving" loop in a new thread
        data_receiving_thread = DataReceivingThread(topic_monitor)
        data_receiving_thread.start()

        # Start the "data processing" loop in the main thread
        process_received_data(topic_monitor, args.show_display)

    except KeyboardInterrupt:
        data_receiving_thread.stop()

    finally:
        # Block this thread until the other thread terminates
        data_receiving_thread.join()


if __name__ == '__main__':
    main()
