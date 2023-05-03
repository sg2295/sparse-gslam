#!/bin/bash
DUR=$1  # The duration this script runs for
INT=$2  # The interval at which we record the memory usage for each process

# Get the process IDs for each process
PID_RUNNER=$(pgrep -f "thin_runner")
PID_RL=$(pgrep -f "roslaunch")
PID_RM=$(pgrep -f "rosmaster")
PID_TF=$(pgrep -f "static_transfor")

# Holds the maximum memory usage
MAX_RUNNER=0
MAX_RL=0
MAX_RM=0
MAX_TF=0

# Check memory usage for each process for the specified duration & interval
END=$(($(date +%s) + DUR))
while [ $(date +%s) -lt $END ]; do
  MEM_RUNNER=$(ps -p "$PID_RUNNER" -o rss --no-headers)
  MEM_RL=$(ps -p "$PID_RL" -o rss --no-headers)
  MEM_RM=$(ps -p "$PID_RM" -o rss --no-headers)
  MEM_TF=$(ps -p "$PID_TF" -o rss --no-headers)
  # Update max memory usage for each process
  if [ "$MEM_RUNNER" -gt "$MAX_RUNNER" ]; then
    MAX_RUNNER=$MEM_RUNNER
  fi
  if [ "$MEM_RL" -gt "$MAX_RL" ]; then
    MAX_RL=$MEM_RL
  fi
  if [ "$MEM_RM" -gt "$MAX_RM" ]; then
    MAX_RM=$MEM_RM
  fi
  if [ "$MEM_TF" -gt "$MAX_TF" ]; then
    MAX_TF=$MEM_TF
  fi
  sleep $INT
done

# Convert maximum memory usage to megabytes & print to screen
MAX_RUNNER_MB=$(echo "scale=2; $MAX_RUNNER / 1024" | bc)
MAX_RL_MB=$(echo "scale=2; $MAX_RL / 1024" | bc)
MAX_RM_MB=$(echo "scale=2; $MAX_RM / 1024" | bc)
MAX_TF_MB=$(echo "scale=2; $MAX_TF / 1024" | bc)

echo "thin_runner=$MAX_RUNNER_MB MB"
echo "ros_launch=$MAX_RL_MB MB"
echo "ros_master=$MAX_RM_MB MB"
echo "static_transform=$MAX_TF_MB MB"
