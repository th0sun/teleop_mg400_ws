# MG400 VR Teleop V2 — Combined Engine Workspace

## สถาปัตยกรรม

```
Unity (Frontend/VR)
    │
    │  ROS-TCP-Connector (port 10000)
    ▼
┌─────────────────────────────────────────────┐
│  teleop_mg400_ws                            │
│                                             │
│  ┌──────────────┐   ┌───────────────────┐   │
│  │ teleop_logic │──▶│   mg400_engine    │   │
│  │ (Python)     │   │   (C++ driver)    │   │
│  │              │   │                   │   │
│  │ - TeleopCtrl │   │ - mg400_interface │   │
│  │ - M11/M14/M15│   │ - mg400_plugin    │   │
│  │ - Analytics  │   │ - mg400_node      │   │
│  │ - ClockSync  │   │ - mg400_msgs      │   │
│  └──────────────┘   └───────────────────┘   │
│         │                     │             │
│  ┌──────────────┐   ┌───────────────────┐   │
│  │ unity_bridge │   │ mg400_description │   │
│  │ (ROS-TCP-EP) │   │ (URDF/meshes)     │   │
│  └──────────────┘   └───────────────────┘   │
└─────────────────────────────────────────────┘
    │
    ▼
  MG400 Robot (TCP/IP: 29999/30003/30004)
```

## โครงสร้างโฟลเดอร์

```
teleop_mg400_ws/
├── src/
│   ├── teleop_logic/          # [Python] สมองของระบบ — teleop node, logic, analytics
│   ├── mg400_engine/          # [C++] เครื่องยนต์จาก HarvestX
│   │   ├── mg400_interface/   #   TCP driver (dashboard/motion/feedback)
│   │   ├── mg400_plugin/      #   ROS2 service/action wrappers
│   │   ├── mg400_plugin_base/ #   Base class สำหรับ plugin
│   │   ├── mg400_node/        #   Main C++ node
│   │   ├── mg400_msgs/        #   Custom msg/srv/action
│   │   └── mg400_common/      #   Shared C++ utilities
│   ├── mg400_description/     # URDF + meshes
│   ├── mg400_bringup/         # Launch files
│   └── unity_bridge/          # ROS-TCP-Endpoint
├── config/
│   └── ros_interface.json     # Topic/service spec ทั้งหมด (Unity ↔ ROS)
├── tools/
│   ├── mock_frontend.py       # จำลอง Unity frontend สำหรับเทส
│   ├── mock_robot/            # MG400 Mock server (Docker)
│   └── dobot_sdk/             # Dobot Python SDK อ้างอิง
├── docs/
│   ├── alarmController.json   # Alarm code reference
│   ├── alarmServo.json
│   └── dobot_commands.json    # Dobot TCP/IP command reference
└── README.md
```

## วิธี Build

```bash
# 1. ไปที่ workspace
cd ~/teleop_mg400_ws

# 2. Build ทั้งหมด
colcon build --symlink-install

# 3. Source
source install/setup.bash
```

## วิธีรัน

### รันทั้งระบบ (Unity bridge + Teleop node)
```bash
ros2 launch teleop_logic teleop_all.launch.py
```

### รันแยกทีละตัว
```bash
# Unity Bridge
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000

# Teleop Node
ros2 run teleop_logic teleop_node

# Monitor GUI (fallback, ใช้เมื่อ Unity ไม่พร้อม)
ros2 run teleop_logic monitor_gui
```

### เทสโดยไม่ต้องเปิด Unity
```bash
# Mock Frontend (จำลอง Unity)
ros2 run teleop_logic mock_frontend

# หรือรัน standalone
python3 tools/mock_frontend.py
```

### Mock Robot (จำลอง MG400)
```bash
cd tools/mock_robot && docker-compose up
```

## Interface Unity ↔ ROS

ดูรายละเอียด topic/service ทั้งหมดที่ `config/ros_interface.json`

### Unity ส่งมา (Publisher)
| Topic | Type | คำอธิบาย |
|---|---|---|
| `/unity/joint_cmd` | sensor_msgs/JointState | คำสั่ง joint 4 แกน (rad) |
| `/vr/suction_cmd` | std_msgs/Bool | เปิด/ปิดหัวดูด |
| `/mg400/light_cmd` | std_msgs/Int32MultiArray | สั่งไฟ [port, state] |
| `/teleop/unity_pong` | std_msgs/String | Heartbeat echo |
| `/unity/enable_robot` | std_msgs/Bool | Enable/Disable (placeholder) |
| `/unity/clear_error` | std_msgs/Empty | Clear Error (placeholder) |
| `/unity/speed_factor` | std_msgs/Int32 | Speed 0-100% (placeholder) |
| `/unity/control_mode` | std_msgs/String | jointmovj/movj/movl (placeholder) |
| `/unity/logic_mode` | std_msgs/String | default/m11/m14/m15/m8_raw (placeholder) |
| `/unity/tool_select` | std_msgs/Int32 | Tool index 0-9 (placeholder) |
| `/unity/teach_mode` | std_msgs/Bool | Teach mode (placeholder) |
| `/unity/emergency_stop` | std_msgs/Bool | E-Stop (placeholder) |

### ROS ส่งกลับ (Subscriber)
| Topic | Type | คำอธิบาย |
|---|---|---|
| `/joint_states` | sensor_msgs/JointState | สถานะ joint จริง (125Hz) |
| `/mg400/robot_mode` | std_msgs/Int32 | Robot mode (1-11) |
| `/mg400/error_status` | std_msgs/Int32 | Error code |
| `/mg400/do_status` | std_msgs/Int64 | DO bitmask |
| `/teleop/sent_command` | sensor_msgs/JointState | คำสั่งที่ส่งจริง |
| `/mg400/tool_vector_actual` | std_msgs/Float64MultiArray | XYZ จริง |
| `/teleop/ros_ping` | std_msgs/Int64 | Heartbeat ping |
| `/teleop/debug` | std_msgs/String | Debug messages |
| `/teleop/latency_report` | std_msgs/String | Latency JSON (placeholder) |
| `/mg400/connection_status` | std_msgs/Bool | สถานะเชื่อมต่อ (placeholder) |
| `/teleop/active_mode` | std_msgs/String | โหมดปัจจุบัน (placeholder) |

## ที่มาของแต่ละส่วน

| Component | มาจาก | หมายเหตุ |
|---|---|---|
| `teleop_logic/` | project_teleop (mg400_controller) | ปรับ import paths เป็น teleop_logic.* |
| `mg400_engine/` | HarvestX_MG400_ROS2 | ตัด Qt GUI ออก, เก็บ core engine |
| `mg400_description/` | HarvestX_MG400_ROS2 | URDF + meshes |
| `mg400_bringup/` | HarvestX_MG400_ROS2 | ตัด rviz config ออก |
| `unity_bridge/` | project_teleop (ROS-TCP-Endpoint) | ไม่แก้ไข |
| `config/ros_interface.json` | project_teleop (ปรับปรุงใหม่) | เพิ่ม placeholder topics |
| `tools/mock_frontend.py` | สร้างใหม่ | จำลอง Unity ทุก topic |
| `tools/mock_robot/` | project_teleop (MG400_Mock) | Docker mock server |
| `tools/dobot_sdk/` | Dobot_TCP_IP_Python_V4 | SDK อ้างอิง |

## สิ่งที่ตัดออก (ไม่ได้นำมา)

| Package | เหตุผล |
|---|---|
| `mg400_joy` | ใช้ Unity VR controller แทน joystick |
| `mg400_rviz_plugin` | Unity เป็น frontend แทน RViz |
| `mg400_node/Qt GUI` | Unity + mock_frontend แทน |
| `mg400_bringup/rviz/` | ไม่ใช้ RViz เป็นหลัก |
| `mg400_simulator` | ใช้ MG400_Mock แทน |
| `HarvestX .github/, doc/, media/` | CI/doc ไม่จำเป็น |
