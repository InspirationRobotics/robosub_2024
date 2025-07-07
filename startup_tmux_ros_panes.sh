#!/bin/bash

SESSION_NAME="ros_pane_setup"

# Start a new tmux session with one window
tmux new-session -d -s $SESSION_NAME -n "main"

source /opt/ros/noetic/setup.bash
# Split horizontally to create Pane 1 (top-right)
tmux split-window -h -t $SESSION_NAME:0.0
tmux split-window -v -t $SESSION_NAME:0.0
tmux split-window -v -t $SESSION_NAME:0.2

tmux send-keys -t $SESSION:0.0 'echo "Hello from Pane 0"' C-m
tmux send-keys -t $SESSION:0.1 'echo "Hello from Pane 1"' C-m
tmux send-keys -t $SESSION:0.1 'roscore' C-m
tmux send-keys -t $SESSION:0.2 'roslaunch mavros px4.launch' C-m
tmux send-keys -t $SESSION:0.3 'python3 -m auv.device.pix_standalone' C-m
# Optional: Select the first pane (idle) on attach
tmux select-pane -t $SESSION_NAME:0.0

# Attach to session
tmux attach-session -t $SESSION_NAME
