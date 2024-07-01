# Create a new tmux session named "robot"
tmux new-session -d -s robot

# Split the window into 4 panes in square formation
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v

# Select pane 3
tmux select-pane -t 3

# cd to m4v2-code/
tmux send-keys -t robot:0.3 'cd ..' C-m

# run interfaces
tmux send-keys -t robot:0.3 './interface.sh' C-m

# Select pane 0
tmux select-pane -t 0

# launch tilt controller
tmux send-keys -t robot:0.0 './launch_tilt.sh' C-m

# Select pane 2
tmux select-pane -t 2

# launch mpc controller
tmux send-keys -t robot:0.2 './launch_mpc.sh' C-m

# Select pane 1
tmux select-pane -t 1

# record.sh
tmux send-keys -t robot:0.1 './record.sh' C-m


