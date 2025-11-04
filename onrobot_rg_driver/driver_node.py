#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

# Try import from installed package; fall back to local for dev runs
try:
    from onrobot_rg_driver.onrobot import RG
except ImportError:
    import os, sys
    sys.path.append(os.path.dirname(__file__))
    from onrobot import RG

from onrobot_rg_driver.msg import RGStatus
from onrobot_rg_driver.srv import (
    Open, Close, MoveToWidth, Stop, SetForce, SetWidth, GetStatus
)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class OnRobotRGNode(Node):
    def __init__(self):
        super().__init__('rg')

        # ---------- Parameters ----------
        self.declare_parameter('ip_address', '192.168.1.1')
        self.declare_parameter('port', 502)
        self.declare_parameter('gripper', 'rg6')         # 'rg2' or 'rg6'
        self.declare_parameter('status_rate_hz', 20.0)   # pub rate for /status

        ip = self.get_parameter('ip_address').get_parameter_value().string_value
        port = int(self.get_parameter('port').get_parameter_value().integer_value)
        gripper = self.get_parameter('gripper').get_parameter_value().string_value
        self.status_rate_hz = float(self.get_parameter('status_rate_hz').value)

        # ---------- Driver ----------
        try:
            self.rg = RG(gripper=gripper, ip=ip, port=port)
        except Exception as e:
            self.get_logger().fatal(f'Failed to init RG driver: {e}')
            raise

        # Serialize Modbus ops (reads/writes)
        self.lock = threading.Lock()

        # ---------- Publisher ----------
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability  = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_status = self.create_publisher(RGStatus, 'status', qos)

        # ---------- Services ----------
        self.create_service(Open,        'open',          self.handle_open)
        self.create_service(Close,       'close',         self.handle_close)
        self.create_service(MoveToWidth, 'move_to_width', self.handle_move)
        self.create_service(Stop,        'stop',          self.handle_stop)
        self.create_service(SetForce,    'set_force',     self.handle_set_force)
        self.create_service(SetWidth,    'set_width',     self.handle_set_width)
        self.create_service(GetStatus,   'get_status',    self.handle_get_status)

        # ---------- Status timer ----------
        period = 1.0 / max(1e-3, self.status_rate_hz)
        self.timer = self.create_timer(period, self.publish_status)

        self.get_logger().info(
            f'OnRobot RG node up: model={gripper} ip={ip}:{port} rate={self.status_rate_hz}Hz'
        )

    # ================= Status helpers =================
    def _safe_read_widths(self):
        """Return (width_mm, width_with_offset_mm, fingertip_offset_mm). Never throws."""
        try:
            with self.lock:
                w    = float(self.rg.get_width())
                woff = float(self.rg.get_width_with_offset())
                tip  = float(self.rg.get_fingertip_offset())
            return w, woff, tip
        except Exception as e:
            self.get_logger().warn(f'Status read (widths) failed: {e}')
            return 0.0, 0.0, 0.0

    def _safe_read_flags(self):
        """
        Return 7 flags as ints:
          [busy, grip_detected, s1_pushed, s1_trigged, s2_pushed, s2_trigged, safety_error]
        Never throws.
        """
        try:
            with self.lock:
                flags = self.rg.get_status()
            if not flags or len(flags) < 7:
                return [0]*7
            return [1 if int(v) else 0 for v in flags[:7]]
        except Exception as e:
            self.get_logger().warn(f'Status read (flags) failed: {e}')
            return [0]*7

    def _read_status_msg(self):
        msg = RGStatus()
        msg.stamp = self.get_clock().now().to_msg()

        w, woff, tip = self._safe_read_widths()
        msg.current_width_mm = w
        msg.current_width_with_offset_mm = woff
        msg.fingertip_offset_mm = tip

        f = self._safe_read_flags()
        msg.is_moving       = bool(f[0])
        msg.object_detected = bool(f[1])
        msg.s1_pushed       = bool(f[2])
        msg.s1_trigged      = bool(f[3])
        msg.s2_pushed       = bool(f[4])
        msg.s2_trigged      = bool(f[5])
        msg.safety_error    = bool(f[6])
        msg.ok = not (msg.s1_trigged or msg.s2_trigged or msg.safety_error)
        return msg

    def publish_status(self):
        self.pub_status.publish(self._read_status_msg())

    # ================= Services =================
    def handle_open(self, req, res):
        force = int(req.force_n_10th) if req.force_n_10th > 0 else 400
        try:
            with self.lock:
                self.rg.open_gripper(force_val=force)
            res.accepted = True
            res.message = 'opening'
        except Exception as e:
            res.accepted = False
            res.message = str(e)
        return res

    def handle_close(self, req, res):
        force = int(req.force_n_10th) if req.force_n_10th > 0 else 400
        try:
            with self.lock:
                self.rg.close_gripper(force_val=force)
            res.accepted = True
            res.message = 'closing'
        except Exception as e:
            res.accepted = False
            res.message = str(e)
        return res

    def handle_move(self, req, res):
        width_10th_mm = int(max(0.0, req.width_mm * 10.0))
        force = int(req.force_n_10th) if req.force_n_10th > 0 else 400
        try:
            with self.lock:
                self.rg.move_gripper(width_val=width_10th_mm, force_val=force)
            res.accepted = True
            res.message = 'moving'
        except Exception as e:
            res.accepted = False
            res.message = str(e)
        return res

    def handle_stop(self, req, res):
        try:
            with self.lock:
                # 0x0008 = stop (per onrobot.py docstring)
                self.rg.set_control_mode(0x0008)
            res.accepted = True
            res.message = 'stopped'
        except Exception as e:
            res.accepted = False
            res.message = str(e)
        return res

    def handle_set_force(self, req, res):
        max_force = getattr(self.rg, 'max_force', 1200)
        val = int(clamp(req.force_n_10th, 0, max_force))
        try:
            with self.lock:
                self.rg.set_target_force(val)
            res.success = True
            res.message = 'force set'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def handle_set_width(self, req, res):
        max_width_10th = getattr(self.rg, 'max_width', 1600)
        val = int(clamp(req.width_mm * 10.0, 0, max_width_10th))
        try:
            with self.lock:
                self.rg.set_target_width(val)
            res.success = True
            res.message = 'width set'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def handle_get_status(self, req, res):
        res.status = self._read_status_msg()
        return res


def main():
    rclpy.init()
    node = OnRobotRGNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
