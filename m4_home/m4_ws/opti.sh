echo "Creating two tmux sessions (vrpn_mocap, angle_broadcaster)"

# Start two detached (-d) tmux sessions called vrpn_mocap and angle_broadcaster
tmux new-session -d -s vrpn_mocap
tmux new-session -d -s angle_broadcaster

echo "Starting vrpn_client"

tmux send-keys -t vrpn_mocap 'source /opt/ros/foxy/setup.bash' Enter
tmux send-keys -t vrpn_mocap 'ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.33' Enter

echo "Starting angle_broadcaster"

tmux send-keys -t angle_broadcaster 'source install/setup.bash' Enter
tmux send-keys -t angle_broadcaster 'ros2 run m4_base angle_broadcaster' Enter
