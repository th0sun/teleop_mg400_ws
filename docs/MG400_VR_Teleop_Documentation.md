# MG400 VR Teleop V2 — คู่มือโปรเจคฉบับสมบูรณ์

**เวอร์ชัน:** 2.0.0 | **วันที่:** มีนาคม 2026 | **ผู้พัฒนา:** thesun  
**Framework:** ROS 2 (Python + C++) | **หุ่นยนต์:** Dobot MG400

---

# สารบัญ

1. [ภาพรวมโปรเจค](#1-ภาพรวมโปรเจค)
2. [สถาปัตยกรรมระบบ](#2-สถาปัตยกรรมระบบ)
3. [โครงสร้างโฟลเดอร์](#3-โครงสร้างโฟลเดอร์)
4. [Package: teleop_logic](#4-package-teleop_logic)
5. [Package: mg400_engine (C++)](#5-package-mg400_engine)
6. [Package: mg400_description](#6-package-mg400_description)
7. [Package: mg400_bringup](#7-package-mg400_bringup)
8. [Package: unity_bridge](#8-package-unity_bridge)
9. [Tools](#9-tools)
10. [ROS Interface](#10-ros-interface)
11. [วิธี Build และรัน](#11-วิธี-build-และรัน)

---

# 1. ภาพรวมโปรเจค

ระบบ **VR Teleoperation** สำหรับหุ่นยนต์ **Dobot MG400** ควบคุมผ่าน **Meta Quest 3 VR Headset** โดยใช้ **Unity** เป็น Frontend และ **ROS 2** เป็น Backend

## หลักการทำงาน

1. ผู้ใช้สวม VR Headset ขยับมือควบคุม
2. Unity แปลงตำแหน่ง VR Controller เป็นมุม Joint
3. ข้อมูลส่งผ่าน ROS-TCP-Connector ไปยัง ROS 2
4. `teleop_logic` รับคำสั่ง ตรวจสอบ ทำนาย และส่งไปหุ่นยนต์
5. MG400 รับคำสั่งผ่าน TCP/IP (Port 29999/30003/30004)
6. Feedback กลับมาแสดงผลบน Unity และ Monitor GUI

## เทคโนโลยีหลัก

| เทคโนโลยี | การใช้งาน |
|---|---|
| ROS 2 | Middleware |
| Python 3 | teleop_logic (สมองของระบบ) |
| C++ | mg400_engine (TCP Driver) |
| Unity + C# | VR Frontend |
| Kalman Filter | ทำนายตำแหน่งล่วงหน้า |
| Tkinter + Matplotlib | Monitor GUI |

---

# 2. สถาปัตยกรรมระบบ

```
Unity (Quest 3 VR)
    │  ROS-TCP-Connector (port 10000)
    ▼
┌──────────────────────────────────────────┐
│  teleop_mg400_ws                          │
│  ┌──────────────┐  ┌──────────────────┐  │
│  │ teleop_logic  │─▶│  mg400_engine    │  │
│  │ (Python)      │  │  (C++ driver)    │  │
│  └──────────────┘  └──────────────────┘  │
│  ┌──────────────┐  ┌──────────────────┐  │
│  │ unity_bridge  │  │ mg400_description│  │
│  └──────────────┘  └──────────────────┘  │
└──────────────────────────────────────────┘
    │
    ▼  MG400 Robot (TCP/IP: 29999/30003/30004)
```

## พอร์ตสื่อสารกับ MG400

| พอร์ต | ชื่อ | หน้าที่ |
|---|---|---|
| 29999 | Dashboard | Enable, Disable, ClearError, SpeedFactor |
| 30003 | Motion | JointMovJ, MovJ, MovL |
| 30004 | Feedback | Real-time 1440 bytes @ 125Hz |

---

# 3. โครงสร้างโฟลเดอร์

```
teleop_mg400_ws/
├── src/
│   ├── teleop_logic/          # [Python] สมองของระบบ
│   │   ├── teleop_logic/
│   │   │   ├── teleop_node.py      # Main Node (825 lines)
│   │   │   ├── monitor_gui.py      # GUI (861 lines)
│   │   │   ├── mock_frontend.py    # จำลอง Unity
│   │   │   ├── config/             # การตั้งค่า
│   │   │   ├── core/               # Connection, Sender, Feedback
│   │   │   ├── logic/              # Controller, Validator, Planner
│   │   │   ├── utils/              # Kinematics, Clock, Haptic
│   │   │   ├── structs/            # Binary packet struct
│   │   │   └── trajectory/         # Record & playback
│   │   └── launch/
│   ├── mg400_engine/          # [C++] TCP Driver (HarvestX)
│   ├── mg400_description/     # URDF + meshes
│   ├── mg400_bringup/         # Launch files
│   └── unity_bridge/          # ROS-TCP-Endpoint
├── config/ros_interface.json
├── tools/                     # Mock tools
└── docs/                      # Alarm references
```

---

# 4. Package: teleop_logic

## 4.1 teleop_node.py — Main Entry Point (825 lines)

### Function: `select_control_mode()`
เมนูเลือกโหมด: Control Mode (jointmovj/movj/movl) และ Logic Mode (default/m11/m14/m15/m8_raw)

### Class: `TeleopNode(Node)`

**Constructor `__init__`** — ลำดับ Initialize:
1. `RobotConnection` → เชื่อมต่อ TCP 3 channels
2. `JointValidator` → ตรวจสอบขอบเขต Joint
3. `MotionPlanner` → สร้างคำสั่ง TCP
4. `TeleopController` → "The Brain" ตัดสินใจ
5. `TargetPredictor` → Kalman Filter
6. `ExperimentalStrategy` (ถ้าเลือก mode ทดลอง)
7. `LatencyAnalyzer`, `ClockCalibrator`
8. Publishers/Subscribers ทั้งหมด
9. `FeedbackHandler`, `CommandSender`
10. `ErrorHandler`, `CollisionHaptic`, `TrajectoryRecorder`
11. `SafetyMonitor`, `InteractiveCommandHandler`
12. Timers: Control Loop 50Hz, Safety 1Hz, Haptic 20Hz

**`_log_worker_loop()`** — Background thread สำหรับ Disk I/O (CSV, Latency, Perf logs)

**`_publish_heartbeat()`** — 1Hz ping ไป Unity เพื่อวัด RTT

**`_unity_pong_callback(msg)`** — รับ pong จาก Unity คำนวณ RTT และ Clock Offset

**`_unity_callback(msg)`** — **Main Input Handler** รับคำสั่งจาก Unity:
1. Anti-NaN Protection
2. Validate & Clamp joints
3. Clock Calibration (Triple-Lock)
4. Kalman Prediction (Dynamic Horizon: ลด horizon เมื่อใกล้เป้า)
5. Publish predicted target, Unity XYZ
6. Log to CSV (Async)
7. อัพเดท `latest_target` (ไม่ส่งคำสั่งตรงนี้)

**`_suction_callback(msg)`** — เปิด/ปิดหัวดูด (Smart Suction: รอถึงเป้าก่อนดูด)

**`_handle_suction_cmd(state)`** — Sequence: Vacuum ON/OFF → Blow ON → Auto-off timer

**`_light_callback(msg)`** — สั่ง Digital Output ไฟ

**`_control_loop()`** — **Main Loop 50Hz** กลยุทธ์ "Proximity + Stuck Detection":
1. Update velocity
2. Smart Suction trigger check
3. Publish Tool Vectors, DO Status, Robot Mode, Error
4. Query Tool Index ทุก ~5s
5. Auto-Recovery: ClearError() ถ้า Mode=9
6. Motion Tracking (LatencyAnalyzer)
7. **ตัดสินใจส่งคำสั่ง:**
   - Path A: Experimental Mode → `ExperimentalStrategy.process()`
   - Path B: Default → `controller.should_send_command()` → `sender.send()`

**`shutdown()`** — ปิดทุกอย่าง: stop threads, disconnect, close files

**`execute_motion_command(q_target)`** — Synchronized blocking motion

**`_publish_haptic_feedback()`** — 20Hz haptic สำหรับ Quest 3

**`check_safety_status()`** — 1Hz delegate ไปยัง SafetyMonitor

**`main()`** — Entry point: select_control_mode → init → spin

---

## 4.2 config/ — การตั้งค่า

### robot_config.py (50 lines)

| ตัวแปร | ค่า | คำอธิบาย |
|---|---|---|
| `JOINT_LIMITS` | Dict | J1:±160°, J2:-25~85°, J3:-25~105°, J4:±360° |
| `ELBOW_ANGLE_LIMIT` | (-60, 60) | J3-J2 relative angle |
| `CONTROL_MODE` | "jointmovj" | โหมดเริ่มต้น |
| `SPATIAL_THRESHOLD` | 0.0005 rad | กรองเคลื่อนที่เล็กๆ |
| `LOGIC_MODE` | "default" | Logic เริ่มต้น |
| `RAW_HZ` | 10 | ความถี่ M8_RawData |
| `ENABLE_GET_ERROR` | True | เปิด GetError API |

### motion_config.py (72 lines)

**Speed:** MAX_SPEED=300°/s, ACC=100, CP=100

**Control Parameters:**
- `PROXIMITY_THRESHOLD` = 0.08 rad — ระยะใกล้เป้า
- `STUCK_VELOCITY_THRESHOLD` = 0.005 rad/s — ถือว่านิ่ง
- `STUCK_TIME_THRESHOLD` = 0.15s — ต้องนิ่งนานเท่านี้
- `DYNAMIC_PROXIMITY_BASE_RAD` = 0.02 rad — ระยะพื้นฐาน
- `DYNAMIC_PROXIMITY_LOOKAHEAD_SEC` = 0.25s — Lookahead

**Hardware:** VACUUM_DO=16, BLOW_DO=15, GREEN=3, YELLOW=4, RED=5

**Topics:** UNITY="/unity/joint_cmd", RVIZ="/joint_states" ฯลฯ

### network_config.py (25 lines)

ROBOT_IP จาก ENV หรือ "192.168.1.6", Dashboard=29999, Cmd=30003, Feedback=30004, Timeout=2s

---

## 4.3 core/ — โมดูลหลัก

### robot_connection.py — RobotConnection (219 lines)

จัดการ Socket 3 channels + Thread-safe dash_lock

| Method | คำอธิบาย |
|---|---|
| `connect()` | เชื่อมต่อ 3 sockets |
| `enable_robot()` | ClearError → Enable → SpeedFactor(100) |
| `_reconnect_port(name)` | Auto-reconnect |
| `send_dashboard_cmd(cmd)` | Non-blocking dashboard + auto-reconnect |
| `send_and_wait(cmd, timeout)` | Blocking dashboard + response |
| `send_motion_cmd(cmd)` | Non-blocking motion port |
| `disconnect()` | ปิดทั้งหมด |

### command_sender.py — CommandSender (142 lines)

| Method | คำอธิบาย |
|---|---|
| `_queue_worker()` | Background thread สำหรับ dashboard commands |
| `send(cmd)` | Non-blocking motion (port 30003) |
| `set_digital_output(port, status)` | Async DO via queue |
| `send_command_with_sync(cmd, timeout)` | Blocking: ส่ง → parse ID → รอเริ่ม → รอเสร็จ |
| `_parse_command_id(response)` | แกะ ID จาก `"{123}"` |

### feedback_handler.py — FeedbackHandler (227 lines)

รับ Binary 1440-byte packets จาก port 30004

**`_run()`** — Flush strategy: อ่านทุก packet ใช้แค่ล่าสุด

**`_process_packet(data)`** — Parse binary:

| Offset | ข้อมูล | Type |
|---|---|---|
| 48 | TestValue | uint64 (verify: 0x0123456789ABCDEF) |
| 432 | QActual (6 joints) | float64×6 |
| 24 | RobotMode | uint64 |
| 8/16 | DI/DO Status | uint64 |
| 64 | SpeedScaling | float64 |
| 1029 | ErrorStatus | byte |
| 1038 | CollisionState | byte |
| 1112 | CurrentCommandId | uint64 |
| 624 | ToolVectorActual | float64×6 |
| 768 | ToolVectorTarget | float64×6 |

- Zero-Lock Prevention: reject blank packets
- Sanity Check: reject jumps > 0.5 rad
- คำนวณ Passive Joints → Publish 9-joint JointState
- Forward Kinematics → Flange XYZ

**Getters:** `get_current_position()`, `get_robot_mode()`, `get_command_id()`, `get_tool_vector()`, `get_flange_actual()`, `get_error_status()`, `get_do_status()`

---

## 4.4 logic/ — ลอจิกการควบคุม

### teleop_controller.py — TeleopController "The Brain" (164 lines)

| Method | คำอธิบาย |
|---|---|
| `update_robot_state(q, now)` | คำนวณ velocity (EMA alpha=0.3) |
| `check_stuck_condition(vel, dist, now)` | ตรวจ: vel<0.005 + dist>0.08 + time>0.15s |
| `should_send_command(target, q_current)` | **Core Decision:** |
| | **Strategy A:** Dynamic Proximity = base + vel×lookahead |
| | **Strategy B:** Stuck Detection safety recovery |
| `format_command_string(target, q_current)` | Validate → Clamp → format TCP cmd |

### joint_validator.py — JointValidator (120 lines)

| Method | คำอธิบาย |
|---|---|
| `validate_and_clamp(q_target)` | Clamp joints + Elbow constraint (J3-J2) + NaN check |
| `is_within_limits(q_rad)` | Check only (no clamp) |
| `get_limits(joint_index)` | Get min/max for joint |

### motion_planner.py — MotionPlanner (134 lines)

| Method | คำอธิบาย |
|---|---|
| `should_skip_motion(target, current)` | กรอง < SPATIAL_THRESHOLD |
| `calculate_speed(target, current)` | Adaptive speed ตามระยะ |
| `format_command(q_rad, speed)` | สร้าง `JointMovJ(...)` / `MovJ(...)` / `MovL(...)` |
| `plan_motion(target, current)` | Single point plan |
| `plan_batch_motion(target, current, steps=3)` | Micro-interpolation (semicolon joined) |

### safety_monitor.py — SafetyMonitor (134 lines)

**`check_and_publish(feedback)`** — 1Hz check:
1. Motor temps: warn >55°C, critical >65°C
2. Collision detection
3. Error check (basic feedback + advanced GetError API)
4. Publish JSON to `/mg400/safety_status`
5. Log diagnostics CSV

### target_predictor.py — TargetPredictor (138 lines)

**Kalman Filter** ชดเชย robot inertia ด้วยการทำนายล่วงหน้า 80ms

State: `[position, velocity]` × 4 joints

**`update_and_predict(raw_q, timestamp, q_actual)`:**
1. Dynamic dt จาก timestamp จริง
2. Predict step: x̂=F·x, P=F·P·F'+Q
3. Update step: Kalman Gain → correct state
4. Safety: ถ้า vel<1°/s → return raw (มือหยุด)
5. Safety: ถ้า error>17° → return raw (jump)
6. Dampened Horizon: ลด prediction เมื่อใกล้เป้า
7. Return: `position + velocity × horizon`

### experimental_logic.py — ExperimentalStrategy (591 lines)

4 โหมดทดลอง (TEMPORARY for A/B testing):

**Helpers:** `_VelocityTracker` (EMA velocity), `_CurvatureTracker` (gated curvature)

**`_RawLogic` (M8):** ส่งดิบตาม RAW_HZ ไม่มี logic

**`_M11Logic` (M11_Stable):** Velocity-clamped integrator (250°/s) + adaptive 10/25Hz

**`_M14Logic` (M14_Smooth):** Feedforward 80ms + curvature gate + adaptive rate (300°/s)

**`_M15Logic` (M15_Sharp):** Strict velocity clamp (260°/s) + tight curvature gate, no feedforward

**`ExperimentalStrategy`:** Public wrapper — `process()` → logic.process() → format TCP cmd

---

## 4.5 utils/ — ยูทิลิตี้

### kinematics.py — KinematicsCalculator (128 lines)

| Method | คำอธิบาย |
|---|---|
| `calculate_passive_joints(active_4)` | 4 active → 9 joints (parallel link) |
| `validate_sanity(current, new, thr=0.5)` | ตรวจจับ jump ผิดปกติ |
| `forward_kinematics(joints_deg)` | FK: LINK1+RotY(LINK2,j2)+RotY(LINK3,j3)+LINK4 → RotZ(pos,j1) |

Link Constants: L1=[43,0,0], L2=[0,0,175], L3=[175,0,0], L4=[66,0,-57] mm

### clock_calibrator.py — ClockCalibrator (147 lines)

Sync Unity↔ROS clocks: Sliding Min-Window + Linear Regression drift estimation

| Method | คำอธิบาย |
|---|---|
| `calibrate(t1_unity, t2_ros)` | Jump detect → drift correct → min-window → regression |
| `_estimate_drift()` | polyfit(x,y,1) → clamp ±5ms/s |
| `get_current_offset()` | Total offset + drift |

### collision_haptic.py — CollisionHaptic (117 lines)

| Method | คำอธิบาย |
|---|---|
| `update_and_publish(collision_state)` | Detect transition → calc intensity → publish JSON |
| `get_haptic_intensity(state)` | Linear decay: 1.0→0.0 ใน 0.3s |

### error_decoder.py — RobotErrorDecoder (78 lines)

| Method | คำอธิบาย |
|---|---|
| `_load_alarm_files()` | โหลด alarmController.json + alarmServo.json |
| `decode_error(error_id)` | Return (description, cause, solution) |

### error_handler.py — ErrorHandler (185 lines)

Advanced error handler ใช้ GetError() API

| Method | คำอธิบาย |
|---|---|
| `check_errors()` | เรียก GetErrorID() → parse → decode |
| `format_error_message(error)` | Format สำหรับแสดงผล |
| `log_errors(errors)` | Log ด้วย severity emoji |
| `get_error_history(count)` | ดึงประวัติ error |
| `get_highest_severity(errors)` | หา severity สูงสุด (0-3) |

### interactive_cmd.py — InteractiveCommandHandler (103 lines)

รับคำสั่งจาก Keyboard แบบ Real-time (Thread แยก)

| คำสั่ง | Dashboard Command |
|---|---|
| `e` | EnableRobot() |
| `d` | DisableRobot() |
| `c` | ClearError() |
| `r` | ResetRobot() |
| `s` | EmergencyStop() |
| `p` | Pause() |
| `u` | Continue() |
| `hz <num>` | ปรับ RAW_HZ สำหรับ M8 mode |
| `q` | Quit |

### latency_analyzer.py — LatencyAnalyzer (210 lines)

Track timestamps T1-T5 และวิเคราะห์ latency

**Timeline:** T1(Unity send) → T2(ROS recv) → T3(Cmd send) → T4(Motion start) → T5(Target reached)

| Method | คำอธิบาย |
|---|---|
| `start_tracking(t1,t2,t3,target)` | เริ่ม track คำสั่งใหม่ |
| `mark_motion_start(now)` | บันทึก T4 |
| `mark_target_reached(now)` | บันทึก T5 |
| `analyze_arrival(q_current, vel)` | คำนวณ: network_ms, decision_ms, response_ms, motion_ms, e2e_ms + validation |
| `format_sent_report(...)` | สร้าง CLI report สำหรับ SEND event |

**Validation Criteria:** final_error<0.005rad, max_joint<0.008rad, velocity<0.003rad/s

### teleop_logger.py — TeleopLogger (124 lines)

CSV file logging สำหรับ 2 ไฟล์:
1. `teleop_latency_*.csv` — Latency breakdown (arrivals)
2. `teleop_struct_*.csv` — Performance metrics (sends)

| Method | คำอธิบาย |
|---|---|
| `log_latency_breakdown(...)` | บันทึก T1-T5 + metrics |
| `log_performance_metrics(...)` | บันทึก decision/send events |

---

## 4.6 structs/dobot_structs.py (90 lines)

NumPy dtype สำหรับ 1440-byte feedback packet ของ Dobot — ทุก field ตาม official SDK

Fields สำคัญ: DigitalInputs, DigitalOutputs, RobotMode, QActual, ToolVectorActual, ToolVectorTarget, MotorTemperatures, ErrorStatus, CollisionState, CurrentCommandId

---

## 4.7 trajectory/trajectory_recorder.py (270 lines)

### Class: TrajectoryRecorder

Teach-and-repeat: บันทึกเส้นทาง VR → เล่นซ้ำ

| Method | คำอธิบาย |
|---|---|
| `start_recording()` | เริ่มบันทึก |
| `record_waypoint()` | บันทึก waypoint (เรียกที่ ~10Hz) |
| `stop_recording()` | หยุดบันทึก return trajectory |
| `save_trajectory(filename)` | บันทึก JSON |
| `load_trajectory(filename)` | โหลด JSON |
| `replay_trajectory(speed_scale, loop)` | เล่นซ้ำ ปรับความเร็วได้ |
| `get_trajectory_info()` | ข้อมูล trajectory |
| `clear_trajectory()` | ลบ trajectory |

---

## 4.8 monitor_gui.py (861 lines)

### Class: SessionLogger
Auto-logging 20Hz ทั้ง joints_tracking.csv + xyz_tracking.csv

### Class: ExecutionMonitor
Track IDLE→MOVING→ARRIVED, วัดเวลา execution

### Class: JointMonitorNode(Node)
ROS Node สำหรับ subscribe ทุก topic + publish suction/light commands

### Class: MonitorGUI
Tkinter GUI:
- **Joint Table:** Target vs Actual vs Diff (color-coded)
- **Cartesian Monitor:** Unity FK, Flange, TCP, Tool Offset
- **Control Panel:** Suction toggle, Light toggles (Green/Yellow/Red)
- **Execution Metrics:** IDLE/MOVING/ARRIVED + timer + stats
- **Status Bar:** Robot Mode, Error decode, DO hex
- **Real-Time Graphs:** 4 subplots (J1-J4) แสดง Unity Raw, Predicted, Cmd Sent, Actual
- **Session Logger:** Auto-start CSV logging

---

## 4.9 mock_frontend.py (326 lines)

จำลอง Unity frontend สำหรับทดสอบ

**Publishers:** joint_cmd, suction, light, pong, enable, clear_error, speed, control_mode, logic_mode, tool, teach, estop

**Subscribers:** joint_states, robot_mode, error, do_status, sent_command, ping, debug, tool_actual, connection, active_mode, latency

**คำสั่ง CLI:** `j <j1-j4>`, `suction on/off`, `light <port> on/off`, `enable`, `disable`, `clear`, `speed <0-100>`, `mode <mode>`, `logic <mode>`, `status`, `ping`, `quit`

---

## 4.10 Launch File: teleop_all.launch.py (39 lines)

เปิดทั้ง Unity Bridge + Teleop Node พร้อมกัน

Arguments: `ros_ip` (default 0.0.0.0), `ros_port` (default 10000)

---

# 5. Package: mg400_engine (C++)

เครื่องยนต์จาก **HarvestX MG400 ROS2** — ตัด Qt GUI ออก เก็บ core engine

## 5.1 mg400_interface — TCP Driver

C++ library สำหรับสื่อสาร TCP กับ MG400 ทั้ง 3 ports

**โครงสร้าง:**
- `include/` — Headers (12 files): tcp_interface, dashboard_commander, motion_commander, feedback_listener
- `src/` — Implementation (10 files)
- `example/` — ตัวอย่างการใช้งาน
- `test/` — Unit tests
- `resources/` — Resource files

**หน้าที่:** จัดการ TCP connection, ส่ง/รับคำสั่ง, parse feedback packets ในระดับ C++

## 5.2 mg400_plugin — ROS2 Service Wrappers

Plugin system ที่ wrap Dashboard API และ Motion API เป็น ROS2 Services

**โครงสร้าง:**
- `include/` — 33 header files (1 per API command)
- `src/` — 32 source files
- `dashboard_api_plugins.xml` — Plugin descriptors (Dashboard commands)
- `motion_api_plugins.xml` — Plugin descriptors (Motion commands)

**Dashboard Plugins:** EnableRobot, DisableRobot, ClearError, ResetRobot, SpeedFactor, SetDO, GetDO, GetErrorID, EmergencyStop ฯลฯ

**Motion Plugins:** JointMovJ, MovJ, MovL, ServoJ, SetTool, SetUser ฯลฯ

## 5.3 mg400_plugin_base — Plugin Base Class

Abstract base class สำหรับ Plugin system

- `include/` — 2 headers: plugin_base.hpp + visibility.hpp

## 5.4 mg400_node — Main C++ Node

ROS2 Node หลักสำหรับ C++ engine

- `src/mg400_node.cpp` — สร้าง node, โหลด plugins, จัดการ lifecycle

## 5.5 mg400_msgs — Custom Messages

| ประเภท | จำนวน | ตัวอย่าง |
|---|---|---|
| msg/ | 20 files | JointState, RobotMode, ErrorStatus |
| srv/ | 26 files | EnableRobot, MovJ, SetDO, GetErrorID |
| action/ | 6 files | JointMovJ, MovJ, MovL |

## 5.6 mg400_common — Shared Utilities

C++ utility functions ที่ใช้ร่วมกันทั้ง engine

---

# 6. Package: mg400_description

URDF Model + Meshes ของ MG400

- `urdf/` — 3 files (URDF model ที่อธิบาย kinematic chain 9 joints)
- `meshes/` — 12 STL files (3D models ของแต่ละ link)

---

# 7. Package: mg400_bringup

Launch files สำหรับเปิด C++ engine node

- `launch/` — 6 launch files
- `test/` — 3 test files

---

# 8. Package: unity_bridge

**ROS-TCP-Endpoint** — สะพานเชื่อม Unity ↔ ROS2

- จาก Unity Technologies (ไม่แก้ไข)
- `ros_tcp_endpoint/` — 12 source files
- รับ/ส่งข้อมูลผ่าน TCP port 10000

---

# 9. Tools

## mock_frontend.py (standalone)
สำเนาเดียวกับ `teleop_logic/mock_frontend.py` สำหรับรันแบบ standalone

## mock_robot/ (Docker)
MG400 Mock Server จำลองหุ่นยนต์จริง:
- จำลอง 3 ports (29999, 30003, 30004)
- ส่ง feedback packets 1440 bytes
- รองรับ Dashboard commands ทั้งหมด
- รัน: `cd tools/mock_robot && docker-compose up`

## dobot_sdk/
Dobot Python SDK อ้างอิง (Dobot_TCP_IP_Python_V4)

---

# 10. ROS Interface

## Unity → ROS (Publishers)

| Topic | Type | คำอธิบาย |
|---|---|---|
| `/unity/joint_cmd` | JointState | คำสั่ง joint 4 แกน (rad) |
| `/vr/suction_cmd` | Bool | เปิด/ปิดหัวดูด |
| `/mg400/light_cmd` | Int32MultiArray | สั่งไฟ [port, state] |
| `/teleop/unity_pong` | String | Heartbeat echo |
| `/unity/enable_robot` | Bool | Enable/Disable |
| `/unity/clear_error` | Empty | Clear Error |
| `/unity/speed_factor` | Int32 | Speed 0-100% |
| `/unity/control_mode` | String | jointmovj/movj/movl |
| `/unity/logic_mode` | String | default/m11/m14/m15/m8_raw |
| `/unity/tool_select` | Int32 | Tool index 0-9 |
| `/unity/teach_mode` | Bool | Teach mode |
| `/unity/emergency_stop` | Bool | E-Stop |

## ROS → Unity (Subscribers)

| Topic | Type | คำอธิบาย |
|---|---|---|
| `/joint_states` | JointState | สถานะ joint จริง (125Hz) |
| `/mg400/robot_mode` | Int32 | Robot mode (1-11) |
| `/mg400/error_status` | Int32 | Error code |
| `/mg400/do_status` | Int64 | DO bitmask |
| `/teleop/sent_command` | JointState | คำสั่งที่ส่งจริง |
| `/mg400/tool_vector_actual` | Float64MultiArray | XYZ จริง |
| `/teleop/ros_ping` | Int64 | Heartbeat ping |
| `/teleop/debug` | String | Debug messages |
| `/mg400/haptic_feedback` | String | Haptic JSON |
| `/teleop/predicted_target` | JointState | Kalman predicted |
| `/teleop/unity_xyz` | Float64MultiArray | FK of Unity input |

---

# 11. วิธี Build และรัน

## Build
```bash
cd ~/teleop_mg400_ws
colcon build --symlink-install
source install/setup.bash
```

## รันทั้งระบบ
```bash
ros2 launch teleop_logic teleop_all.launch.py
```

## รันแยก
```bash
# Unity Bridge
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0 -p ROS_TCP_PORT:=10000

# Teleop Node
ros2 run teleop_logic teleop_node

# Monitor GUI
ros2 run teleop_logic monitor_gui
```

## ทดสอบ
```bash
# Mock Frontend (แทน Unity)
ros2 run teleop_logic mock_frontend

# Mock Robot (แทน MG400)
cd tools/mock_robot && docker-compose up
```

---

# 12. Data Flow Diagram

```
VR Controller (Quest 3)
    │
    ▼
Unity (C#) ─── joint angles (rad) ───▶ /unity/joint_cmd
    │                                        │
    │                                        ▼
    │                              ┌─────────────────────┐
    │                              │   _unity_callback()  │
    │                              │                      │
    │                              │ 1. validate_and_clamp│
    │                              │ 2. clock_calibrate   │
    │                              │ 3. kalman_predict    │
    │                              │ 4. store latest_target│
    │                              └──────────┬──────────┘
    │                                         │
    │                              ┌──────────▼──────────┐
    │                              │  _control_loop()     │
    │                              │  (50Hz Timer)        │
    │                              │                      │
    │                              │ 1. update_velocity   │
    │                              │ 2. should_send_cmd?  │
    │                              │    - Dynamic Prox    │
    │                              │    - Stuck Detection  │
    │                              │ 3. format_command    │
    │                              │ 4. send to robot     │
    │                              └──────────┬──────────┘
    │                                         │
    │                                         ▼
    │                              MG400 (Port 30003)
    │                                         │
    │                              ┌──────────▼──────────┐
    │                              │ FeedbackHandler      │
    │                              │ (Port 30004, 125Hz)  │
    │                              │                      │
    │                              │ Parse 1440-byte      │
    │                              │ Publish /joint_states │
    │                              └──────────┬──────────┘
    │                                         │
    ◄─────────────────────────────────────────┘
         /joint_states (feedback to Unity)
```

---

*สิ้นสุดเอกสาร — MG400 VR Teleop V2 Documentation*
