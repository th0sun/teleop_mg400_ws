#!/bin/bash

# ==========================================
# MG400 Teleop Manager (Tmux)
# Matrix approach: [Robot] x [Frontend]
# ==========================================

SESSION_NAME="mg400_teleop"
DEFAULT_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

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
echo "ROS_DOMAIN_ID (DDS domain): $DEFAULT_DOMAIN_ID"
echo "(set ROS_DOMAIN_ID before running to override)"
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

# Ensure no old session is running and lock domain ID
tmux kill-session -t $SESSION_NAME 2>/dev/null
export ROS_DOMAIN_ID=$DEFAULT_DOMAIN_ID

# ==========================================
# 1. Setup Robot Backend
# ==========================================
if [ "$ROBOT_CHOICE" = "1" ]; then
    echo "🌍 Connecting to Real Robot..."
    export ROBOT_IP="192.168.1.6" # Change this if your real robot IP is different
elif [ "$ROBOT_CHOICE" = "2" ]; then
    echo "🐳 Starting Docker Mock..."
    pushd tools/mock_robot/docker >/dev/null
    docker compose up -d
    MOCK_CONTAINER=$(docker compose ps -q dobot)
    popd >/dev/null

    if [ -z "$MOCK_CONTAINER" ]; then
        echo "❌ Could not find mock robot container (dobot)."
        exit 1
    fi

    # Give it a second to start, then inspect IP
    sleep 2
    MOCK_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$MOCK_CONTAINER" 2>/dev/null)
    
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
tmux send-keys -t $SESSION_NAME:0.0 "export ROS_DOMAIN_ID=$DEFAULT_DOMAIN_ID && source install/setup.bash && export ROBOT_IP=$ROBOT_IP && ros2 run teleop_logic teleop_node; exec bash" C-m

# ==========================================
# 2. Setup Frontend
# ==========================================

# If not 'None', split the window
if [ "$FRONTEND_CHOICE" != "4" ]; then
    tmux split-window -h -t $SESSION_NAME:0
    
    if [ "$FRONTEND_CHOICE" = "1" ]; then
        # Python 3-D Simulator
        tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=$DEFAULT_DOMAIN_ID && source install/setup.bash && cd tools/mg400_simulator && chmod +x run.sh && ./run.sh; exec bash" C-m
    elif [ "$FRONTEND_CHOICE" = "2" ]; then
        # Unity TCP Endpoint
        tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=$DEFAULT_DOMAIN_ID && source install/setup.bash && ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000; exec bash" C-m
    elif [ "$FRONTEND_CHOICE" = "3" ]; then
        # CLI Mock Frontend
        tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=$DEFAULT_DOMAIN_ID && source install/setup.bash && ros2 run teleop_logic mock_frontend; exec bash" C-m
    fi
fi

# ==========================================
# 3. Tmux Settings & Layout
# ==========================================
tmux set -g mouse on
tmux set -g pane-border-status top
tmux set -g pane-border-format " #{pane_index}: #{pane_title} "

tmux select-pane -t $SESSION_NAME:0.0 -T "🤖 Teleop Node"
if [ "$FRONTEND_CHOICE" != "4" ]; then
    tmux select-pane -t $SESSION_NAME:0.1 -T "🖥️ Frontend"
fi

tmux set -g status-right " 💡 Ctrl+C=stop | ↑Enter=restart | Shift+drag=copy | Ctrl+D=close pane | kill-server=exit all "
tmux set -g status-right-length 90
tmux set -g status-style "bg=#1a1a2e fg=#aaaaaa"

# Focus main pane
tmux select-pane -t $SESSION_NAME:0.0
tmux attach-session -t $SESSION_NAME

# ==========================================
# 4. Cleanup after detach/exit
# ==========================================
if [ "$ROBOT_CHOICE" = "2" ]; then
    echo ""
    read -p "� Stop Docker Mock? (y/N): " STOP_DOCKER
    if [[ "$STOP_DOCKER" =~ ^[Yy]$ ]]; then
        cd tools/mock_robot && docker compose down
        echo "🐳 Docker stopped."
    fi
fi
