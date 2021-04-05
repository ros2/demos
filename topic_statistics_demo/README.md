# Topic Statistics Demo

The demo application in this package demonstrates [Topic Statistics](https://docs.ros.org/en/rolling/Concepts/About-Topic-Statistics.html) feature in ROS 2.
The application creates ROS 2 nodes to publish messages to topics, subscribes to the statistics topic and displays the statistics data received.

The demo application in this package `display_topic_statistics` creates the following ROS 2 nodes:
1. Talker and Listener nodes to generate message traffic
2. Statistics listener node to display generated statistics

The application requires an argument `message_type` - the type of message chatter to generate.
Possible values are `string` and `imu`.

The application also accepts the following optional arguments to configure the Listener node's subscription:
1. `--publish-topic`: Topic to which topic statistics are published. Default topic is `/statistics`.
2. `--publish-period`: Publish period for publication of statistics. Default value is 5s.

Once the application starts, the talker node will publish messages on a topic that the listener node has subscribed to.
The listener's subscription will generate topic statistics upon receiving messages.
Statistics are published to the statistics topic at a pre-determined frequency.
The statistics listener node listens to these statistics and prints them for the user to see.
