#!/bin/bash

# ==========================================
# MG400 Teleop Manager (Tmux)
# Matrix approach: [Robot] x [Frontend]
# Supports macOS (Simulator-only) & Linux (Full ROS 2)
# ==========================================

SESSION_NAME="mg400_teleop"
DEFAULT_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
OS_TYPE=$(uname -s)

# Option: Cleanup
if [ "$1" = "clean" ]; then
    echo "🧹 Cleaning up..."
    tmux kill-session -t $SESSION_NAME 2>/dev/null
    if [ "$OS_TYPE" = "Linux" ]; then
        cd tools/mock_robot/docker && docker compose down 2>/dev/null
    fi
    echo "✅ Done."
    exit 0
fi

clear
echo "=========================================="
echo " 🤖 MG400 Teleop Manager "
echo "=========================================="
echo "OS detected: $OS_TYPE"

# ==========================================
# macOS Mode: Simulator Only (Unity Mock)
# ==========================================
if [ "$OS_TYPE" = "Darwin" ]; then
    echo ""
    echo "⚠️  macOS detected — ROS 2 is not available locally."
    echo "The Python 3-D Simulator will connect to your Ubuntu"
    echo "machine via Unity Mock TCP (port 10000)."
    echo ""
    echo "Make sure the Ubuntu machine is running:"
    echo "  ./start_teleop.sh   (select Frontend: Simulator or Unity)"
    echo "------------------------------------------"
    read -p "Launch Simulator now? (Y/n): " LAUNCH_SIM

    if [[ "$LAUNCH_SIM" =~ ^[Nn]$ ]]; then
        echo "Exiting."
        exit 0
    fi

    echo "🚀 Launching MG400 Simulator (Unity Mock mode)..."
    cd tools/mg400_simulator
    chmod +x run.sh
    ./run.sh
    exit 0
fi

# ==========================================
# Linux Mode: Full ROS 2 Stack
# ==========================================
echo "ROS_DOMAIN_ID (DDS domain): $DEFAULT_DOMAIN_ID"
echo "(set ROS_DOMAIN_ID before running to override)"
echo ""
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
echo "1. Python 3-D Simulator (Unity Mock via ros_tcp_endpoint)"
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

# Pane 0: Teleop Node (ROS 2 Core) — always runs
tmux send-keys -t $SESSION_NAME:0.0 "export ROS_DOMAIN_ID=$DEFAULT_DOMAIN_ID && source install/setup.bash && export ROBOT_IP=$ROBOT_IP && ros2 run teleop_logic teleop_node; exec bash" C-m

# ==========================================
# 2. Setup Frontend
# ==========================================

if [ "$FRONTEND_CHOICE" = "1" ]; then
    # ── Python 3-D Simulator (needs ros_tcp_endpoint + local simulator) ──
    # Pane 1: ros_tcp_endpoint (bridge for Unity Mock protocol)
    tmux split-window -h -t $SESSION_NAME:0
    tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=$DEFAULT_DOMAIN_ID && source install/setup.bash && ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000; exec bash" C-m

    # Pane 2: Simulator GUI
    tmux split-window -v -t $SESSION_NAME:0.1
    tmux send-keys -t $SESSION_NAME:0.2 "cd tools/mg400_simulator && chmod +x run.sh && ./run.sh; exec bash" C-m

    tmux select-pane -t $SESSION_NAME:0.0 -T "🤖 Teleop Node"
    tmux select-pane -t $SESSION_NAME:0.1 -T "🌐 TCP Endpoint (10000)"
    tmux select-pane -t $SESSION_NAME:0.2 -T "🖥️ Simulator"

elif [ "$FRONTEND_CHOICE" = "2" ]; then
    # ── Unity App (needs ros_tcp_endpoint only, Unity runs on separate device) ──
    tmux split-window -h -t $SESSION_NAME:0
    tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=$DEFAULT_DOMAIN_ID && source install/setup.bash && ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000; exec bash" C-m

    tmux select-pane -t $SESSION_NAME:0.0 -T "🤖 Teleop Node"
    tmux select-pane -t $SESSION_NAME:0.1 -T "🌐 TCP Endpoint (10000)"

    echo ""
    echo "📱 Unity App mode: ros_tcp_endpoint is listening on port 10000."
    echo "   Connect your Unity VR/AR headset or macOS Simulator to this machine's IP."

elif [ "$FRONTEND_CHOICE" = "3" ]; then
    # ── CLI Mock Frontend (pure ROS 2, no TCP endpoint needed) ──
    tmux split-window -h -t $SESSION_NAME:0
    tmux send-keys -t $SESSION_NAME:0.1 "export ROS_DOMAIN_ID=$DEFAULT_DOMAIN_ID && source install/setup.bash && ros2 run teleop_logic mock_frontend; exec bash" C-m

    tmux select-pane -t $SESSION_NAME:0.0 -T "🤖 Teleop Node"
    tmux select-pane -t $SESSION_NAME:0.1 -T "📟 CLI Frontend"

elif [ "$FRONTEND_CHOICE" = "4" ]; then
    # ── Headless (no frontend) ──
    tmux select-pane -t $SESSION_NAME:0.0 -T "🤖 Teleop Node (Headless)"
fi

# ==========================================
# 3. Optional: RViz Visualization
# ==========================================
echo ""
echo "[ STEP 3: RViz2 Robot Visualizer ]"
read -p "Launch RViz2? (y/N): " RVIZ_CHOICE

if [[ "$RVIZ_CHOICE" =~ ^[Yy]$ ]]; then
    # display.launch.py includes:
    #   - rsp.launch.py → robot_state_publisher (TF + /robot_description from URDF)
    #   - joint_state_publisher_gui  (for manual testing without live /joint_states)
    #   - rviz.launch.py → rviz2 with display.rviz config
    # NOTE: When teleop_node is running, /joint_states is published by FeedbackHandler.
    # joint_state_publisher_gui will be overridden by live data automatically.
    tmux split-window -v -t $SESSION_NAME:0.0
    RVIZ_PANE=$(tmux display-message -p -t $SESSION_NAME "#{pane_index}")
    tmux send-keys -t $SESSION_NAME:0.$RVIZ_PANE \
        "export ROS_DOMAIN_ID=$DEFAULT_DOMAIN_ID && source install/setup.bash && ros2 launch mg400_bringup display.launch.py; exec bash" C-m
    tmux select-pane -t $SESSION_NAME:0.$RVIZ_PANE -T "📡 RViz2 + RSP"
    echo "✅ RViz2 + robot_state_publisher launching..."
fi

# ==========================================
# 4. Optional: Monitor GUI (Tkinter + Graphs)
# ==========================================
echo ""
echo "[ STEP 4: Monitor GUI (Joint Tracking) ]"
read -p "Launch Monitor GUI? (y/N): " MONITOR_CHOICE

if [[ "$MONITOR_CHOICE" =~ ^[Yy]$ ]]; then
    tmux split-window -v -t $SESSION_NAME:0.0
    MON_PANE=$(tmux display-message -p -t $SESSION_NAME "#{pane_index}")
    tmux send-keys -t $SESSION_NAME:0.$MON_PANE \
        "export ROS_DOMAIN_ID=$DEFAULT_DOMAIN_ID && source install/setup.bash && ros2 run teleop_logic monitor_gui; exec bash" C-m
    tmux select-pane -t $SESSION_NAME:0.$MON_PANE -T "📊 Monitor GUI"
    echo "✅ Monitor GUI launching..."
fi

# ==========================================
# 5. Tmux Settings & Layout
# ==========================================
tmux set -g mouse on
tmux set -g pane-border-status top
tmux set -g pane-border-format " #{pane_index}: #{pane_title} "

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
    read -p "🐳 Stop Docker Mock? (y/N): " STOP_DOCKER
    if [[ "$STOP_DOCKER" =~ ^[Yy]$ ]]; then
        cd tools/mock_robot/docker && docker compose down
        echo "🐳 Docker stopped."
    fi
fi
