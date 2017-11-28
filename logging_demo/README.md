# Logging and logger configuration

See [the logging page]() for details on available functionality.

In this demo, different types of log calls are shown and the severity level of different loggers is configured locally and externally.

Start the demo with:
```
ros2 run logging_demo logging_demo_main
```

Over time you will see output from various log calls with different properties.
To start with you will only see output from log calls with severity `INFO` and above (`WARN`, `ERROR`, `FATAL`).
Note that the first message will only be logged once as that is a property of the log call used for that message.

### Logger level configuration: locally

After 10 iterations the severity threshold of the logger will be set to `DEBUG`, which will cause additional messages to be logged.

Some of these debug messages cause additional functions/expressions to be evaluated, which were previously skipped as `DEBUG` log calls were not enabled.
See [the source code]() of the demo for further explanation of the calls used, and see the [`rclcpp` logging documentation]() for a full list of supported logging calls.

### Logger level configuration: externally

In the future there will be a generalized approach to external configuration of loggers at runtime (similar to how [`rqt_logger_level`](http://wiki.ros.org/rqt_logger_level) in ROS 1 allows logger configuration via remote procedural calls).
**This concept is not yet officially supported in ROS 2.** In the meantime, this demo provides an **example** service that can be called externally to request configuration of logger levels for known names of loggers in the process.

The demo previously started is already running this example service.
To set the severity threshold of the demo's logger back to `INFO`, call the service with:

```
ros2 service call /config_logger logging_demo/ConfigLogger "{logger_name: 'logger_usage_demo', severity_threshold: INFO}"
```

This service call will work on any logger that is running in the process provided that you know its name.
This includes the loggers in the ROS 2 core, such as `rcl` (the common client library package).
To enable debug logging for `rcl`, call:

```
ros2 service call /config_logger logging_demo/ConfigLogger "{logger_name: 'rcl', severity_threshold: DEBUG}"
```

You should see debug output from `rcl` start to show.

#### Using the logger config component

The server that responds to the logger configuration requests has been developed as a component so that it may be added to an existing composition-based system.
For example, if you are using [a container to run your nodes](https://github.com/ros2/ros2/wiki/Composition#using-components), to be able to configure your loggers you only need to request that it additionally load the `logging_demo::LoggerConfig` component into the container.

As an example, if you want to debug the `composition::Talker` demo, you can start the talker as normal with:

Shell 1:
```
ros2 run composition api_composition
```
Shell 2:
```
ros2 run composition api_composition_cli composition composition::Talker
```

And then when you want to enable debug logging, load the `LoggerConfig` component with:

Shell 2
```
ros2 run composition api_composition_cli logging_demo logging_demo::LoggerConfig
```

And finally, configure all unset loggers to the debug severity by addressing the empty-named logger. Note that loggers that have been specifically configured to use a particular severity will not be affected by this call.

Shell 2:
```
ros2 service call /config_logger logging_demo/ConfigLogger "{logger_name: '', severity_threshold: DEBUG}"
```
You should see debug output from any previously unset loggers in the process start to appear, including from the ROS 2 core.
