source /opt/ros/foxy/setup.bash
source ./install/setup.bash
cd ./src/roboclaw_reset
sudo ./reset_roboclaw.sh
ros2 launch morphing_lander control.launch.py
