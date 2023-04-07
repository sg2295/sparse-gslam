PID_LOG=$(pgrep -f "log_runner")
PID_RL=$(pgrep -f "roslaunch")
PID_RM=$(pgrep -f "rosmaster")
PID_TF=$(pgrep -f "static_transfor")


DUR=$1  # Duration
INT=$2  # Interval

MAX_LOG=0
MAX_RL=0
MAX_RM=0
MAX_TF=0

END=$(($(date +%s) + DUR))

while [ $(date +%s) -lt $END ]; do
  MEM_LOG=$(ps -p "$PID_LOG" -o rss --no-headers)
  MEM_RL=$(ps -p "$PID_RL" -o rss --no-headers)
  MEM_RM=$(ps -p "$PID_RM" -o rss --no-headers)
  MEM_TF=$(ps -p "$PID_TF" -o rss --no-headers)

  if [ "$MEM_LOG" -gt "$MAX_LOG" ]; then
    MAX_LOG=$MEM_LOG
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


MAX_LOG_MB=$(echo "scale=2; $MAX_LOG / 1024" | bc)
MAX_RL_MB=$(echo "scale=2; $MAX_RL / 1024" | bc)
MAX_RM_MB=$(echo "scale=2; $MAX_RM / 1024" | bc)
MAX_TF_MB=$(echo "scale=2; $MAX_TF / 1024" | bc)

echo "Log_runner=$MAX_LOG_MB MB"
echo "Ros_launch=$MAX_RL_MB MB"
echo "Ros_master=$MAX_RM_MB MB"
echo "Static_transform=$MAX_TF_MB MB"


