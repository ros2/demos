## Build instructions

If you haven't already, clone [rttest](https://github.com/jacquelinekay/rttest) into your ament workspace.

Build:

```
./src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install --only pendulum_control
```

Run:

```
. install/setup.bash
./build/pendulum_control/pendulum_demo[__rmw_opensplice/__rmw_connext]
```
The demo should work with both middleware implementations.

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
You can run the program as sudo, or you can adjust the system limits for memory locking.

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

With these settings you can get decent performance even if you don't have the RT_PREEMPT kernel installed, but you will likely see an unacceptably large maximum latency and periodic latency spikes.
