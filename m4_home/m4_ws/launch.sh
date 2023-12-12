source ./install/setup.bash
cd ./src/roboclaw_reset
sudo ./reset_roboclaw.sh
ros2 launch m4_base control.launch.py
