#!/bin/bash

# Duration in seconds for which the script will monitor memory usage
DURATION="$1"
# The interval in seconds at which the script will check memory usage
INTERVAL="$2"

# Initialize the maximum memory usage variable
MAX_USAGE=0

# Calculate the end time
END_TIME=$(($(date +%s) + DURATION))

while [ $(date +%s) -lt $END_TIME ]; do
  # Get the current used memory for all processes
  CURRENT_USAGE=$(ps -eo rss | awk '{sum+=$1} END {print sum/1024}')

  # Update the maximum memory usage if the current usage is higher
  if [ "$(echo "$CURRENT_USAGE > $MAX_USAGE" | bc)" -eq 1 ]; then
    MAX_USAGE=$CURRENT_USAGE
  fi

  # Sleep for the defined interval
  sleep $INTERVAL
done

# Print the maximum memory usage in MB
echo "Maximum memory usage: $MAX_USAGE MB"
