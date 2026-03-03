#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock Frontend Node — จำลอง Unity frontend สำหรับเทสระบบ
ใช้แทน Unity เมื่อต้องการเทส topic/service โดยไม่ต้องเปิด Unity

วิธีใช้:
    ros2 run teleop_logic mock_frontend
    หรือ python3 tools/mock_frontend.py

คำสั่ง:
    j <j1> <j2> <j3> <j4>  — ส่ง joint command (degrees)
    suction on/off          — เปิด/ปิดหัวดูด
    light <port> on/off     — สั่งไฟ (port 3=green, 4=yellow, 5=red)
    enable / disable        — Enable/Disable robot
    clear                   — Clear Error
    speed <0-100>           — Speed Factor
    mode <jointmovj/movj/movl> — Control Mode
    logic <default/m11/m14/m15/m8_raw> — Logic Mode
    tool <0-9>              — Tool Select
    teach on/off            — Teach Mode
    estop                   — Emergency Stop
    status                  — แสดงสถานะล่าสุด
    ping                    — ดู RTT
    quit                    — ออก
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import (
    Bool, Int32, Int32MultiArray, Int64, Float64MultiArray, String, Empty
)
import numpy as np
import threading
import time
import sys


class MockFrontend(Node):
    def __init__(self):
        super().__init__('mock_frontend')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # === Publishers (Unity → ROS) ===
        self.pub_joint_cmd = self.create_publisher(JointState, '/unity/joint_cmd', qos)
        self.pub_suction = self.create_publisher(Bool, '/vr/suction_cmd', 10)
        self.pub_light = self.create_publisher(Int32MultiArray, '/mg400/light_cmd', 10)
        self.pub_pong = self.create_publisher(String, '/teleop/unity_pong', 10)
        self.pub_enable = self.create_publisher(Bool, '/unity/enable_robot', 10)
        self.pub_clear_error = self.create_publisher(Empty, '/unity/clear_error', 10)
        self.pub_speed = self.create_publisher(Int32, '/unity/speed_factor', 10)
        self.pub_control_mode = self.create_publisher(String, '/unity/control_mode', 10)
        self.pub_logic_mode = self.create_publisher(String, '/unity/logic_mode', 10)
        self.pub_tool = self.create_publisher(Int32, '/unity/tool_select', 10)
        self.pub_teach = self.create_publisher(Bool, '/unity/teach_mode', 10)
        self.pub_estop = self.create_publisher(Bool, '/unity/emergency_stop', 10)

        # === Subscribers (ROS → Unity) ===
        self.sub_joint_states = self.create_subscription(
            JointState, '/joint_states', self._cb_joint_states, 10)
        self.sub_robot_mode = self.create_subscription(
            Int32, '/mg400/robot_mode', self._cb_robot_mode, 10)
        self.sub_error = self.create_subscription(
            Int32, '/mg400/error_status', self._cb_error, 10)
        self.sub_do = self.create_subscription(
            Int64, '/mg400/do_status', self._cb_do, 10)
        self.sub_sent = self.create_subscription(
            JointState, '/teleop/sent_command', self._cb_sent, 10)
        self.sub_ping = self.create_subscription(
            Int64, '/teleop/ros_ping', self._cb_ping, 10)
        self.sub_debug = self.create_subscription(
            String, '/teleop/debug', self._cb_debug, 10)
        self.sub_tool_actual = self.create_subscription(
            Float64MultiArray, '/mg400/tool_vector_actual', self._cb_tool_actual, 10)
        self.sub_connection = self.create_subscription(
            Bool, '/mg400/connection_status', self._cb_connection, 10)
        self.sub_active_mode = self.create_subscription(
            String, '/teleop/active_mode', self._cb_active_mode, 10)
        self.sub_latency = self.create_subscription(
            String, '/teleop/latency_report', self._cb_latency, 10)

        # State
        self.latest_joints_deg = [0.0] * 4
        self.robot_mode = 0
        self.error_status = 0
        self.do_status = 0
        self.last_sent_deg = [0.0] * 4
        self.last_ping_ms = 0
        self.ping_sent_time = 0.0
        self.rtt_ms = 0.0
        self.tool_xyz = [0.0] * 6
        self.connected = False
        self.active_mode = "unknown"
        self.latency_report = ""

        self.get_logger().info("Mock Frontend พร้อมใช้งาน พิมพ์ 'help' เพื่อดูคำสั่ง")

    # === Callbacks ===
    def _cb_joint_states(self, msg):
        if len(msg.position) >= 9:
            self.latest_joints_deg = [
                np.degrees(msg.position[0]),
                np.degrees(msg.position[1]),
                np.degrees(msg.position[3]),
                np.degrees(msg.position[8])
            ]

    def _cb_robot_mode(self, msg):
        self.robot_mode = msg.data

    def _cb_error(self, msg):
        self.error_status = msg.data

    def _cb_do(self, msg):
        self.do_status = msg.data

    def _cb_sent(self, msg):
        if len(msg.position) >= 4:
            self.last_sent_deg = [np.degrees(p) for p in msg.position[:4]]

    def _cb_ping(self, msg):
        self.last_ping_ms = msg.data
        # Echo pong
        pong = String()
        pong.data = str(msg.data)
        self.pub_pong.publish(pong)
        self.rtt_ms = (time.time() * 1000) - msg.data

    def _cb_debug(self, msg):
        pass  # Silent by default

    def _cb_tool_actual(self, msg):
        if len(msg.data) >= 6:
            self.tool_xyz = list(msg.data)

    def _cb_connection(self, msg):
        self.connected = msg.data

    def _cb_active_mode(self, msg):
        self.active_mode = msg.data

    def _cb_latency(self, msg):
        self.latency_report = msg.data

    # === Commands ===
    def send_joint(self, j1, j2, j3, j4):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4']
        msg.position = [np.radians(j1), np.radians(j2), np.radians(j3), np.radians(j4)]
        self.pub_joint_cmd.publish(msg)
        self.get_logger().info(f"Sent joint: [{j1:.1f}, {j2:.1f}, {j3:.1f}, {j4:.1f}] deg")

    def send_suction(self, on: bool):
        msg = Bool()
        msg.data = on
        self.pub_suction.publish(msg)
        self.get_logger().info(f"Suction: {'ON' if on else 'OFF'}")

    def send_light(self, port: int, on: bool):
        msg = Int32MultiArray()
        msg.data = [port, 1 if on else 0]
        self.pub_light.publish(msg)
        self.get_logger().info(f"Light port {port}: {'ON' if on else 'OFF'}")

    def send_enable(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.pub_enable.publish(msg)
        self.get_logger().info(f"Robot: {'ENABLE' if enable else 'DISABLE'}")

    def send_clear_error(self):
        self.pub_clear_error.publish(Empty())
        self.get_logger().info("Clear Error sent")

    def send_speed(self, factor: int):
        msg = Int32()
        msg.data = max(0, min(100, factor))
        self.pub_speed.publish(msg)
        self.get_logger().info(f"Speed Factor: {msg.data}%")

    def send_control_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.pub_control_mode.publish(msg)
        self.get_logger().info(f"Control Mode: {mode}")

    def send_logic_mode(self, mode: str):
        msg = String()
        msg.data = mode
        self.pub_logic_mode.publish(msg)
        self.get_logger().info(f"Logic Mode: {mode}")

    def send_tool(self, index: int):
        msg = Int32()
        msg.data = index
        self.pub_tool.publish(msg)
        self.get_logger().info(f"Tool Select: {index}")

    def send_teach(self, on: bool):
        msg = Bool()
        msg.data = on
        self.pub_teach.publish(msg)
        self.get_logger().info(f"Teach Mode: {'ON' if on else 'OFF'}")

    def send_estop(self):
        msg = Bool()
        msg.data = True
        self.pub_estop.publish(msg)
        self.get_logger().info("EMERGENCY STOP!")

    def print_status(self):
        mode_names = {
            1: "INIT", 2: "BRAKE_OPEN", 3: "POWER_OFF", 4: "DISABLED",
            5: "ENABLED", 6: "DRAG", 7: "RUNNING", 8: "RECORDING",
            9: "ERROR", 10: "PAUSE", 11: "JOG"
        }
        print("\n" + "=" * 50)
        print(f"  Robot Mode   : {self.robot_mode} ({mode_names.get(self.robot_mode, '?')})")
        print(f"  Error Status : {self.error_status}")
        print(f"  Connected    : {self.connected}")
        print(f"  Active Mode  : {self.active_mode}")
        print(f"  DO Status    : {bin(self.do_status)}")
        print(f"  RTT          : {self.rtt_ms:.0f} ms")
        print(f"  Actual Joints: [{', '.join(f'{j:.2f}' for j in self.latest_joints_deg)}] deg")
        print(f"  Last Sent    : [{', '.join(f'{j:.2f}' for j in self.last_sent_deg)}] deg")
        print(f"  Tool XYZ     : [{', '.join(f'{v:.1f}' for v in self.tool_xyz[:3])}] mm")
        print("=" * 50 + "\n")


def input_loop(node: MockFrontend):
    HELP = """
คำสั่ง:
  j <j1> <j2> <j3> <j4>   ส่ง joint (degrees)
  suction on/off           เปิด/ปิดหัวดูด
  light <port> on/off      สั่งไฟ (3=green, 4=yellow, 5=red)
  enable / disable         Enable/Disable robot
  clear                    Clear Error
  speed <0-100>            Speed Factor
  mode <jointmovj/movj/movl>  Control Mode
  logic <default/m11/m14/m15/m8_raw>  Logic Mode
  tool <0-9>               Tool Select
  teach on/off             Teach Mode
  estop                    Emergency Stop
  status                   แสดงสถานะล่าสุด
  help                     แสดงคำสั่งทั้งหมด
  quit                     ออก
"""
    print(HELP)

    while rclpy.ok():
        try:
            line = input("mock> ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        try:
            if cmd == 'j' and len(parts) == 5:
                node.send_joint(float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]))
            elif cmd == 'suction':
                node.send_suction(parts[1].lower() == 'on')
            elif cmd == 'light' and len(parts) == 3:
                node.send_light(int(parts[1]), parts[2].lower() == 'on')
            elif cmd == 'enable':
                node.send_enable(True)
            elif cmd == 'disable':
                node.send_enable(False)
            elif cmd == 'clear':
                node.send_clear_error()
            elif cmd == 'speed' and len(parts) == 2:
                node.send_speed(int(parts[1]))
            elif cmd == 'mode' and len(parts) == 2:
                node.send_control_mode(parts[1])
            elif cmd == 'logic' and len(parts) == 2:
                node.send_logic_mode(parts[1])
            elif cmd == 'tool' and len(parts) == 2:
                node.send_tool(int(parts[1]))
            elif cmd == 'teach':
                node.send_teach(parts[1].lower() == 'on')
            elif cmd == 'estop':
                node.send_estop()
            elif cmd == 'status':
                node.print_status()
            elif cmd == 'ping':
                print(f"RTT: {node.rtt_ms:.0f} ms")
            elif cmd == 'help':
                print(HELP)
            elif cmd in ('quit', 'exit', 'q'):
                break
            else:
                print(f"Unknown command: {line}  (พิมพ์ 'help' เพื่อดูคำสั่ง)")
        except Exception as e:
            print(f"Error: {e}")

    rclpy.shutdown()


def main():
    rclpy.init()
    node = MockFrontend()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    input_loop(node)

    node.destroy_node()


if __name__ == '__main__':
    main()
