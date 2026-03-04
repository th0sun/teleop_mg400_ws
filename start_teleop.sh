#!/bin/bash

# ==========================================
# MG400 Teleop Manager (Tmux)
# Matrix approach: [Robot] x [Frontend]
# ==========================================

SESSION_NAME="mg400_teleop"

# Option: Cleanup
if [ "$1" = "clean" ]; then
    echo "🧹 Cleaning up..."
    tmux kill-session -t $SESSION_NAME 2>/dev/null
    cd tools/mock_robot/docker && docker compose down
    echo "✅ Done."
    exit 0
fi

clear
echo "=========================================="
echo " 🤖 MG400 Teleop Manager "
echo "=========================================="
echo "[ STEP 1: Select Robot Backend ]"
echo "1. Real MG400 Robot"
echo "2. Mock Robot (Docker)"
echo "------------------------------------------"
read -p "Select Robot (1-2): " ROBOT_CHOICE

if [[ ! "$ROBOT_CHOICE" =~ ^[1-2]$ ]]; then
    echo "❌ Invalid selection. Exiting."
    exit 1
fi

echo ""
echo "[ STEP 2: Select Frontend ]"
echo "1. Python 3-D Simulator (PyQt5)"
echo "2. Unity App (Real VR/AR headset via ros_tcp_endpoint)"
echo "3. CLI Mock Frontend (Text-based debug)"
echo "4. None (Headless ROS 2 Node only)"
echo "------------------------------------------"
read -p "Select Frontend (1-4): " FRONTEND_CHOICE

if [[ ! "$FRONTEND_CHOICE" =~ ^[1-4]$ ]]; then
    echo "❌ Invalid selection. Exiting."
    exit 1
fi

# Ensure no old session is running
tmux kill-session -t $SESSION_NAME 2>/dev/null

# ==========================================
# 1. Setup Robot Backend
# ==========================================
if [ "$ROBOT_CHOICE" = "1" ]; then
    echo "🌍 Connecting to Real Robot..."
    export ROBOT_IP="192.168.1.6" # Change this if your real robot IP is different
elif [ "$ROBOT_CHOICE" = "2" ]; then
    echo "🐳 Starting Docker Mock..."
    cd tools/mock_robot/docker
    docker compose up -d
    cd ../../../
    
    # Give it a second to start, then inspect IP
    sleep 2
    MOCK_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' dobot 2>/dev/null)
    
    if [ -z "$MOCK_IP" ]; then
        echo "❌ Could not find Mock Robot IP. Ensure Docker is running."
        exit 1
    fi
    export ROBOT_IP="$MOCK_IP"
    echo "✅ Mock Robot running at IP: $ROBOT_IP"
fi

echo "🚀 Starting ROS 2 systems in Tmux..."

# Create new tmux session in detached mode
tmux new-session -d -s $SESSION_NAME

# Pane 0: Teleop Node (ROS 2 Core)
tmux send-keys -t $SESSION_NAME:0.0 "source install/setup.bash" C-m
tmux send-keys -t $SESSION_NAME:0.0 "export ROBOT_IP=$ROBOT_IP" C-m
tmux send-keys -t $SESSION_NAME:0.0 "ros2 run teleop_logic teleop_node" C-m

# ==========================================
# 2. Setup Frontend
# ==========================================

# If not 'None', split the window
if [ "$FRONTEND_CHOICE" != "4" ]; then
    tmux split-window -h -t $SESSION_NAME:0
    tmux send-keys -t $SESSION_NAME:0.1 "source install/setup.bash" C-m
fi

if [ "$FRONTEND_CHOICE" = "1" ]; then
    # Python 3-D Simulator
    tmux send-keys -t $SESSION_NAME:0.1 "cd tools/mg400_simulator" C-m
    tmux send-keys -t $SESSION_NAME:0.1 "chmod +x run.sh && ./run.sh" C-m
    
elif [ "$FRONTEND_CHOICE" = "2" ]; then
    # Unity TCP Endpoint
    tmux send-keys -t $SESSION_NAME:0.1 "ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000" C-m

elif [ "$FRONTEND_CHOICE" = "3" ]; then
    # CLI Mock Frontend
    tmux send-keys -t $SESSION_NAME:0.1 "ros2 run teleop_logic mock_frontend" C-m
fi

# Attach to the session so the user can see logs
tmux attach-session -t $SESSION_NAME

