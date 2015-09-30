#!/usr/bin/env bash

pendulum_logger__rmw_opensplice_cpp &
logger_pid=$!

# run pendulum demo indefinitely
pendulum_demo__rmw_opensplice_cpp -i 0 &
#pendulum_demo__rmw_opensplice_cpp -i 0
demo_pid=$!

# Initialize system stress
stress -c 2 -i 2 --timeout 18000000000 &
stress_pid=$!
# Publish new command messages every 3 minutes
# Run for 5 hours (5*60/3 = 100 iterations)
# run for 1 hr: 60/3 = 20 iterations

for i in {1..100}; do
  sleep 3m
  pendulum_teleop__rmw_opensplice_cpp $i
done

# Kill backgrounded processes on exit
kill -9 $logger_pid $demo_pid $stress_pid
#kill -9 $logger_pid
