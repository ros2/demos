# Logging and logger configuration

In the future functionality akin to ROS 1's [`rosout`](http://wiki.ros.org/rosout) concept will be available so that logging output from processes can be recorded to the console and files, and made accessible for consumers located on remote machines.
Additionally, configuration of loggers will be possible from files at startup and at runtime via by remote procedural calls.

The logging functionality currently supported is:
- console output.
- local programmatic configuration of logger levels.


## Demo

In this demo, different types of log calls are shown and the severity level of different loggers is configured externally.

Start the demo with:
```
ros2 run logging_demo logging_demo_main
```

Over time you will see output from various log calls with different properties.
To start with you will only see output from log calls with severity `INFO` and above.
Note that the first message will only be logged once as that is a property of the log call used for that message.

### Logger level configuration: locally

After 10 iterations the severity threshold of the logger will be set to `DEBUG`, which will cause additional messages to be logged.

Some of these debug messages cause additional functions/expressions to be evaluated, which were previously skipped as `DEBUG` log calls were not enabled.
See [the source code]() of the demo for further explanation of the calls used, and see the [`rclcpp` logging documentation]() for a full list of supported logging calls.

### Logger level configuration: externally

In the future there will be a generalized approach to external configuration of loggers.
In the meantime, this demo provides a service that can be called externally to request configuration of logger levels for known names of loggers in the process.

The demo previously started is already running this service.
To set the severity threshold of the demo's logger back to `INFO`, call the service with:

```
ros2 service call /config_logger logging_demo/ConfigLogger "{logger_name: 'logger_usage_demo', severity_threshold: INFO}"
```

This service call will work on any logger that is running in the process provided that you know its name.
This includes the loggers in the ROS 2 core, such as `rcl` (the common client library package).

The server that responds to the logger configuration requests has been developed as a component so that it may easily be added to an existing composition-based system.
If you are using [manual composition](https://github.com/ros2/ros2/wiki/Composition#compile-time-composition-using-ros-services-2), you only need to
To enable debug logging for `rcl`, call:

```
ros2 service call /config_logger logging_demo/ConfigLogger "{logger_name: 'rcl', severity_threshold: DEBUG}"
```


# Configuration

By default, console output will be formatted to include the message severity, logger name, and the message.
Information such as the file name, function name and line number of the log call are also available.
Custom console output format can be configured with the `RCUTILS_CONSOLE_OUTPUT_FORMAT` environment variable: see the [`rcutils` documentation for details]().
