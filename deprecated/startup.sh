#!/bin/bash

echo "Going to /home/m4version2/m4v2-code"

cd /home/m4version2/m4v2-code

echo "Creating two tmux sessions (interface, control)"

# Start two detached (-d) tmux sessions called interface and control
tmux new-session -d -s interface
tmux new-session -d -s control

echo "Starting interface.sh"

# start interfaces.launch
tmux send-keys -t interface './interface.sh' Enter

sleep 5

echo "Starting control.launch on control"

# start m4_control.launch
tmux send-keys -t control 'cd ./m4_home/m4_ws' Enter
tmux send-keys -t control './launch.sh' Enter

sleep 5

echo "Robot Operational !"
