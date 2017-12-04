# Logging and logger configuration

The logging functionality currently supported is:
- Client libraries (`rclcpp` and `rclpy`) using a common logging library to provide log calls with a variety of filters on hierarchical loggers.
- Console output (file output and functionality akin to [`rosout`](http://wiki.ros.org/rosout) for remote consumption of messages is forthcoming).
- Programmatic configuration of logger levels (config files and runtime configuration is forthcoming).

## Logging usage

<!-- the `rclcpp` documentation won't include logging until we make a new doc dump -->
In C++:
- See the [logging demo]() for example usage.
- See the [`rclcpp` documentation]() for an extensive list of functionality.

In Python:
- See the [`rclpy` tests](https://github.com/ros2/rclpy/blob/master/rclpy/test/test_logging.py) for example usage.

## Logger configuration

Logger configuration is still under development.
For now, the severity of individual loggers can be configured programmatically with, e.g.:

In C++:
```
rcutils_logging_set_logger_severity_threshold("logger_name", RCUTILS_LOG_SEVERITY_DEBUG);
```

In Python:
```
rclpy.logging.set_logger_severity_threshold('logger_name', rclpy.logging.LoggingSeverity.DEBUG)
```

The [logging demo]() provides an example of manually exposing a service so that loggers can be configured externally; in the future we expect runtime configuration capabilities of loggers to be exposed automatically.

## Console output configuration

<!-- the rcutils docs won't mention this env var until we make a new doc dump -->
By default, console output will be formatted to include the message severity, logger name, and the message.
Information such as the file name, function name and line number of the log call are also available.
Custom console output format can be configured with the `RCUTILS_CONSOLE_OUTPUT_FORMAT` environment variable: see the [`rcutils` documentation for details]().
As `rclpy` and `rclcpp` both use `rcutils` for logging, this will effect all Python and C++ nodes.
