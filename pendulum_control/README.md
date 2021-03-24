## Build instructions

This pendulum control demo is available in the default ROS 2 install.
Follow the instructions to build ROS 2 from source: https://docs.ros.org/en/rolling/Installation/Linux-Development-Setup.html

## Running the demo

```
. install/setup.bash
. install/pendulum_control/bin/pendulum_demo
```

A few command line arguments related to real-time performance profiling are provided by rttest.
See https://github.com/ros2/rttest/blob/master/README.md for more information.

## Running with real-time performance

The demo will print out its performance statistics continuously and at the end of the program.

Example final output:
```
rttest statistics:
  - Minor pagefaults: 0
  - Major pagefaults: 0
  Latency (time after deadline was missed):
    - Min: 2414 ns
    - Max: 45949 ns
    - Mean: 8712.12 ns
    - Standard deviation: 1438.74
```

Ideally you want to see 0 minor or major pagefaults and an average latency of less than 30,000 nanosceonds
(3% of the 1 millisecond update period).

If you see pagefaults, you may not have permission to lock memory via `mlockall`.
You need to adjust the system limits for memory locking.

Add to `/etc/security/limits.conf`:
```
<user>    -   memlock   <limit in kB>
```

A limit of `-1` is unlimited.
If you choose this, you may need to accompany it with `ulimit -l unlimited` after editing the file.

After saving the file, log out and log back in.

If you see a high mean latency in the results, you may need to adjust the maximum priority for processes.

Add to `/etc/security/limits.conf`:
```
<user>    -   rtprio   <maximum priority>
```

The range of the priority is 0-99.
However, do NOT set the limit to 99 because then your processes could interfere with important system processes that run at the top priority (e.g. watchdog).
This demo will attempt to run the control loop at priority 98.

With these settings you can get decent average performance even if you don't have the RT_PREEMPT kernel installed, but you will likely see an unacceptably large maximum latency and periodic latency spikes.

## Dynamic allocation

The demo will also print to the console whenever `malloc` is called, along with debug symbols from the backtrace for that stack.

If you search the output of the demo, you will see that `malloc` is only called during the initialization phase of the program.

This is consistent with the requirements of real-time programming (to prevent non-determinstic blocking in the allocator).

However, without memory locking, you may still see some pagefaults due to reading memory that was allocated but not read into cache.
