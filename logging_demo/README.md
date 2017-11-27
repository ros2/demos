# Logging and logger configuration

In the future functionality akin to ROS 1's [`rosout`](http://wiki.ros.org/rosout) concept will be available so that logging output from multiple processes will be aggregated and made accessible for consumers located on remote machines.
Currently local console output is supported.

## Usage

### C++

In `rclcpp` logging is performed with macros.

See the [`rclcpp` logging documentation]().

### Python

See the [`rclpy` logging documentation]().

### Configuration

By default, console output will be formatted to include the message severity, logger name, and the message.
Information such as the file name, function name and line number of the log call are also available.
Custom console output format can be configured with the `RCUTILS_CONSOLE_OUTPUT_FORMAT` environment variable: see the [`rcutils` documentation for details]().
