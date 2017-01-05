#!/usr/bin/env bash

pendulum_logger &
logger_pid=$!

# run pendulum demo indefinitely
pendulum_demo -i 0 &
demo_pid=$!

# Initialize system stress
stress -c 2 -i 2 --timeout 18000000000 &
stress_pid=$!
for i in {1..100}; do
  sleep 3m
  pendulum_teleop $i
done

# Kill backgrounded processes on exit
kill -9 $logger_pid $demo_pid $stress_pid
