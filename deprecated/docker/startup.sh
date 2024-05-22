#!/bin/bash

# echo "Sourcing startup scripts"
# source /home/m4version2/.profile
# source /home/m4version2/.bashrc

echo "Going to /home/m4version2/m4v2-code"

cd /home/m4version2/m4v2-code

echo "Creating two tmux sessions (interface, control)"

# Start two detached (-d) tmux sessions called interface and control
tmux new-session -d -s interface
tmux new-session -d -s control

echo "Starting docker on control"

# Start docker on control
tmux send-keys -t control './run_docker.sh' Enter

sleep 10

echo "Starting interface.sh"

# start interfaces.launch
tmux send-keys -t interface './interface.sh' Enter

sleep 7

echo "Starting control.launch on control"

# start m4_control.launch
tmux send-keys -t control 'cd m4_ws' Enter
tmux send-keys -t control 'source install/setup.bash' Enter
tmux send-keys -t control 'ros2 launch m4_base control.launch.py' Enter

sleep 7

echo "Robot Operational !"