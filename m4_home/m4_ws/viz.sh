echo "Creating a tmux session rosbridge"

# Start detached (-d) tmux sessions called rosbridge
tmux new-session -d -s rosbridge

echo "Starting rosbridge"

tmux send-keys -t rosbridge 'source /opt/ros/foxy/setup.bash' Enter
tmux send-keys -t rosbridge 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml' Enter