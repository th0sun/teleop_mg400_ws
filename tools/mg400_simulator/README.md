# MG400 3-D Simulator

Unity-like 3-D robot simulator for the MG400 arm, built in Python with PyQt5 + OpenGL.

## Features

| Feature | Details |
|---|---|
| 3-D viewport | OpenGL with URDF `.dae` mesh rendering, orbit / pan / zoom |
| 1:1 EE drag | Click the orange sphere → drag to move end-effector in camera plane |
| Axis drag | Click Red/Green/Blue arrows to precisely move EE along X/Y/Z world axes |
| Joint sliders | J1–J4 sliders with live FK preview |
| Teach & Repeat | Record waypoints, preview poses, and play them back sequentially |
| Robot commands | Enable / Disable / Clear Error / E-Stop |
| I/O | Suction & light toggle |
| ROS 2 bridge | Optional – publishes to `/unity/joint_cmd`, subscribes to `/joint_states_deg` |
| Ghost pose | Last-sent pose shown dimmed in viewport |

## Viewport controls

| Input | Action |
|---|---|
| Left-drag | Orbit camera |
| Middle-drag / Alt+Left | Pan camera |
| Scroll wheel | Zoom |
| Drag orange sphere | Move end-effector (1:1 depth-locked drag) |
| Drag Gizmo arrow | Move end-effector along specific X/Y/Z axis |
| `F` | Focus camera on EE |
| `R` | Reset camera |

## Installation

```bash
cd tools/mg400_simulator
pip install -r requirements.txt
```

On Ubuntu you may also need:
```bash
sudo apt install python3-pyqt5 python3-opengl
```

## Running

### Standalone (no ROS required)
```bash
cd tools/mg400_simulator
python3 main.py
# Or use the convenience script: ./run.sh
```

### With ROS 2 bridge (live robot / mock)

If you are running the simulator on the same Linux machine as ROS 2:
```bash
# Terminal 1 – start the mock robot
cd tools/mock_robot && docker compose up -d

# Terminal 2 – start teleop stack
source ~/teleop_mg400_ws/install/setup.bash
export ROBOT_IP=<mock_container_ip>
ros2 run teleop_logic teleop_node

# Terminal 3 – launch simulator
source ~/teleop_mg400_ws/install/setup.bash
python3 tools/mg400_simulator/main.py
# → enable "Connect to ROS 2" in the control panel
```

### Running Simulator remotely (e.g., Mac/Windows client ↔ Ubuntu ROS 2 host)

You can run the rich 3-D simulator natively on your personal machine (Mac/Windows) while the ROS 2 core and robot logic run on a separate Ubuntu machine or VM. As long as they are on the same local network, they can communicate via `FastRTPS`/`CycloneDDS` Discovery.

**1. On the Ubuntu ROS 2 Host:**
Ensure the machine is connected to the same local network as your client.
```bash
# Export a specific Domain ID to isolate traffic and ensure discovery
export ROS_DOMAIN_ID=0
source ~/teleop_mg400_ws/install/setup.bash

# Run your teleop node (and mock robot if needed)
ros2 run teleop_logic teleop_node
```

### Running Simulator as a Unity Mock (Mac/Windows)

Since `rclpy` cannot be easily installed on macOS or Windows, the simulator features a built-in **Unity TCP Mock**. This allows the Python simulator to masquerade as the Unity VR Client and communicate directly with the `ros-tcp-endpoint` (Port 10000) on your Ubuntu machine without needing a native ROS 2 distribution installed locally.

**On the Ubuntu Machine (Robot Controller):**
1. Start the complete teleop stack which includes `teleop_node` and `ros-tcp-endpoint`:
   ```bash
   ros2 launch teleop_logic teleop_all.launch.py ros_ip:=0.0.0.0
   ```
   *(Note down the IP address of this Ubuntu machine, e.g., `192.168.1.6`)*

**On your Mac/Windows Machine (Simulator):**
1. Start the simulator normally:
   ```bash
   ./run.sh
   # or
   source .venv/bin/activate
   python3 main.py
   ```
2. In the right control panel under **"ROS 2 Bridge / Unity Mock"**: 
   - Enter your Ubuntu machine's IP in the **Host** field.
   - Leave the **Port** as `10000`.
   - Check **"Connect"**.
3. You can now control the robot remotely exactly as the Unity VR headset does!

## Architecture

```
main.py
  └── MainWindow (ui/main_window.py)
        ├── RobotViewport (ui/robot_viewport.py)   ← PyOpenGL 3-D scene
        │     • orbit / pan / zoom camera
        │     • accurate URDF mesh loading via trimesh
        │     • Unity-style gizmo drag (axis & sphere) using ray-plane IK
        │     • emits joints_changed signal
        ├── ControlPanel (ui/control_panel.py)      ← right sidebar
        │     • joint sliders ↔ viewport (bidirectional)
        │     • speed / mode / I/O controls
        ├── TeachPanel (ui/teach_panel.py)          ← right sidebar (tab)
        │     • waypoint recording and playback logic
        └── ROSBridge (core/ros_bridge.py)          ← optional rclpy node
              • pub /unity/joint_cmd
              • sub /joint_states_deg
              • pub dashboard commands
```

## Kinematics

Uses the exact same constants as `teleop_logic/utils/kinematics.py`:

| Link | Vector (mm) |
|---|---|
| LINK1 (shoulder offset) | [43, 0, 0] |
| LINK2 (upper arm) | [0, 0, 175] |
| LINK3 (forearm) | [175, 0, 0] |
| LINK4 (wrist) | [66, 0, -57] |

Analytical IK closed-form solution used for real-time drag.
