#!/usr/bin/env python3
# ==============================================================================
# File: cartesian_jogger.py
# Purpose: Provides a Tkinter-based Graphical User Interface (GUI). 
#          FIXED: Removed manual coordinate offsets that conflicted with the 
#          proper TF mathematical tree, resulting in precise picking.
# ==============================================================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState, CameraInfo
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data
import tkinter as tk
import threading
import math
import os
import time
from .config import SETTINGS

class VisualJogger(Node):
    def __init__(self):
        super().__init__('visual_jogger')
        self.cartesian_publisher = self.create_publisher(Pose, '/target_position', 10)
        
        self.arm_publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_publisher = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        self.create_subscription(TFMessage, '/world/default/pose/info', self.gt_callback, 10)
        
        self.cam_frame_id = 'camera_math_frame'
        self.info_sub = self.create_subscription(CameraInfo, '/right_cam/camera_info', self.info_callback, qos_profile_sensor_data)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.cfg = SETTINGS
        self.timer = self.create_timer(0.2, self.timer_callback)
        
        self.step_cm = self.cfg["robot"]["step_cm"]
        self.step_deg = self.cfg["robot"]["step_deg"]
        
        self.real_x_cm = -40.0
        self.real_y_cm = 0.0
        self.real_z_cm = 40.0
        self.real_roll_deg = 0.0
        self.real_pitch_deg = 0.0
        self.real_yaw_deg = 0.0
        
        self.cmd_x = -40.0
        self.cmd_y = 0.0
        self.cmd_z = 40.0
        self.cmd_roll = -180.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0
        
        self.gazebo_red = "Gazebo Red:      Waiting for data..."
        self.gazebo_green = "Gazebo Green:    Waiting for data..."
        
        self.mode = "joint"
        self.updating_sliders = False
        self.attached_color = None
        
        self.angle_text_var = None
        self.real_loc_var = None
        self.attach_status_var = None
        
        self.live_red_var = None
        self.live_green_var = None
        self.cap_red_var = None
        self.cap_green_var = None
        
        self.debug_cam_to_apple = None
        self.debug_base_to_tcp = None
        self.debug_base_to_cam = None
        self.debug_world_to_tcp = None
        self.debug_world_to_cam = None
        self.debug_world_to_apple = None
        self.debug_base_to_apple = None
        self.debug_real_apple = None
        
        self.live_red_target = None
        self.live_green_target = None
        self.captured_red_target = None
        self.captured_green_target = None
        
        self.sliders = []
        self.entry_vars = []
        self.gripper_slider = None
        self.latest_arm_angles = [0.0] * 6
        
        self.arm_joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        self.gripper_joint_names = [
            'gripper_to_left_finger', 'gripper_to_right_finger'
        ]

    def info_callback(self, msg):
        pass

    def gt_callback(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == 'red_apple':
                x = t.transform.translation.x * 100.0
                y = t.transform.translation.y * 100.0
                z = t.transform.translation.z * 100.0
                self.gazebo_red = f"{'Gazebo Red:':<16} X: {x:6.1f}   Y: {y:6.1f}   Z: {z:6.1f}"
            elif t.child_frame_id == 'green_apple':
                x = t.transform.translation.x * 100.0
                y = t.transform.translation.y * 100.0
                z = t.transform.translation.z * 100.0
                self.gazebo_green = f"{'Gazebo Green:':<16} X: {x:6.1f}   Y: {y:6.1f}   Z: {z:6.1f}"

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return roll, pitch, yaw

    def joint_callback(self, msg):
        if self.angle_text_var is None or len(self.sliders) < 6 or self.gripper_slider is None:
            return
        try:
            angles = []
            for i in range(6):
                index = msg.name.index(self.arm_joint_names[i])
                angles.append(msg.position[index])
            self.latest_arm_angles = angles
            self.angle_text_var.set(
                f"Base: {angles[0]:6.2f} Shoulder: {angles[1]:6.2f} Elbow: {angles[2]:6.2f}\n"
                f"Wrist 1: {angles[3]:6.2f} Wrist 2: {angles[4]:6.2f} Wrist 3: {angles[5]:6.2f}"
            )
            if self.mode == "cartesian":
                self.updating_sliders = True
                for i in range(6):
                    degree_val = math.degrees(angles[i])
                    self.sliders[i].set(degree_val)
                    if len(self.entry_vars) == 6:
                        self.entry_vars[i].set(f"{degree_val:.1f}")
                
                self.updating_sliders = False
        except ValueError:
            pass

    def get_tf_string(self, prefix, target, source):
        try:
            t = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            x = t.transform.translation.x * 100.0
            y = t.transform.translation.y * 100.0
            z = t.transform.translation.z * 100.0
            return f"{prefix:<16} X: {x:6.1f}   Y: {y:6.1f}   Z: {z:6.1f}"
        except:
            return f"{prefix:<16} Searching network..."

    def timer_callback(self):
        if self.real_loc_var is not None:
            try:
                t = self.tf_buffer.lookup_transform('world', 'tcp_link', rclpy.time.Time())
                self.real_x_cm = t.transform.translation.x * 100.0
                self.real_y_cm = t.transform.translation.y * 100.0
                self.real_z_cm = t.transform.translation.z * 100.0
                q = t.transform.rotation
                r, p, y = self.quaternion_to_euler(q.x, q.y, q.z, q.w)
                self.real_roll_deg = math.degrees(r)
                self.real_pitch_deg = math.degrees(p)
                self.real_yaw_deg = math.degrees(y)
                self.real_loc_var.set(
                    f"TCP X: {self.real_x_cm:6.1f} Y: {self.real_y_cm:6.1f} Z: {self.real_z_cm:6.1f} cm\n"
                    f"Roll: {self.real_roll_deg:6.1f} Pitch: {self.real_pitch_deg:6.1f} Yaw: {self.real_yaw_deg:6.1f}"
                )
            except:
                pass

        if self.debug_world_to_tcp is not None:
            self.debug_base_to_tcp.set(self.get_tf_string("Base to TCP:", 'base_link', 'tcp_link'))
            self.debug_world_to_tcp.set(self.get_tf_string("World to TCP:", 'world', 'tcp_link'))
            
            self.debug_base_to_cam.set(self.get_tf_string("Base to Cam:", 'base_link', self.cam_frame_id))
            self.debug_world_to_cam.set(self.get_tf_string("World to Cam:", 'world', self.cam_frame_id))
            
            cam_red = self.get_tf_string("Cam to Red:", self.cam_frame_id, 'apple_red')
            cam_green = self.get_tf_string("Cam to Green:", self.cam_frame_id, 'apple_green')
            self.debug_cam_to_apple.set(f"{cam_red}\n{cam_green}")
            
            world_red = self.get_tf_string("World to Red:", 'world', 'apple_red')
            world_green = self.get_tf_string("World to Green:", 'world', 'apple_green')
            self.debug_world_to_apple.set(f"{world_red}\n{world_green}")
            
            base_red = self.get_tf_string("Base to Red:", 'base_link', 'apple_red')
            base_green = self.get_tf_string("Base to Green:", 'base_link', 'apple_green')
            self.debug_base_to_apple.set(f"{base_red}\n{base_green}")
            
            self.debug_real_apple.set(f"{self.gazebo_red}\n{self.gazebo_green}")

        if self.live_red_var is not None and self.live_green_var is not None:
            for color in ['red', 'green']:
                try:
                    tw = self.tf_buffer.lookup_transform('world', f'apple_{color}', rclpy.time.Time())
                    wx = tw.transform.translation.x * 100.0
                    wy = tw.transform.translation.y * 100.0
                    wz = tw.transform.translation.z * 100.0

                    if color == 'red':
                        self.live_red_var.set(f"Live Red\nX: {wx:6.1f}  Y: {wy:6.1f}  Z: {wz:6.1f}")
                        self.live_red_target = [wx, wy, wz]
                    else:
                        self.live_green_var.set(f"Live Green\nX: {wx:6.1f}  Y: {wy:6.1f}  Z: {wz:6.1f}")
                        self.live_green_target = [wx, wy, wz]
                except:
                    pass

    def capture_positions(self):
        if self.live_red_target:
            self.captured_red_target = list(self.live_red_target)
            self.cap_red_var.set(f"Captured Red\nX: {self.captured_red_target[0]:6.1f}  Y: {self.captured_red_target[1]:6.1f}  Z: {self.captured_red_target[2]:6.1f}")
        if self.live_green_target:
            self.captured_green_target = list(self.live_green_target)
            self.cap_green_var.set(f"Captured Green\nX: {self.captured_green_target[0]:6.1f}  Y: {self.captured_green_target[1]:6.1f}  Z: {self.captured_green_target[2]:6.1f}")

    def move_cartesian(self, axis, direction):
        if self.mode == "joint":
            self.cmd_x = self.real_x_cm
            self.cmd_y = self.real_y_cm
            self.cmd_z = self.real_z_cm
            self.cmd_roll = self.real_roll_deg
            self.cmd_pitch = self.real_pitch_deg
            self.cmd_yaw = self.real_yaw_deg
            
        self.mode = "cartesian"

        if axis == 'x': self.cmd_x += self.step_cm * direction
        elif axis == 'y': self.cmd_y += self.step_cm * direction
        elif axis == 'z':
            self.cmd_z += self.step_cm * direction
            if self.cmd_z < 42.0:
                self.cmd_z = 42.0
        elif axis == 'roll': self.cmd_roll += self.step_deg * direction
        elif axis == 'pitch': self.cmd_pitch += self.step_deg * direction
        elif axis == 'yaw': self.cmd_yaw += self.step_deg * direction
            
        msg = Pose()
        msg.position.x = self.cmd_x / 100.0
        msg.position.y = self.cmd_y / 100.0
        msg.position.z = self.cmd_z / 100.0
        qx, qy, qz, qw = self.euler_to_quaternion(
            math.radians(self.cmd_roll), math.radians(self.cmd_pitch), math.radians(self.cmd_yaw))
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        self.cartesian_publisher.publish(msg)

    def execute_apple_action(self, color, action):
        target = self.captured_red_target if color == 'red' else self.captured_green_target
        if target is None: return

        self.mode = "cartesian"
        self.cmd_x = target[0]
        # The TF tree properly tracks camera to base to TCP perfectly in world space.
        # Adding manual offsets here corrupts the mathematical target positioning.
        self.cmd_y = target[1]
        
        if action == 'hover':
            self.cmd_z = self.cfg["robot"]["hover_z_cm"]
        elif action == 'pick':
            self.cmd_z = self.cfg["robot"]["pick_z_cm"]
            
        self.cmd_roll = -180.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0

        msg = Pose()
        msg.position.x = self.cmd_x / 100.0
        msg.position.y = self.cmd_y / 100.0
        msg.position.z = self.cmd_z / 100.0
        qx, qy, qz, qw = self.euler_to_quaternion(
            math.radians(self.cmd_roll), math.radians(self.cmd_pitch), math.radians(self.cmd_yaw))
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        self.cartesian_publisher.publish(msg)

    def execute_center_action(self, action):
        self.mode = "cartesian"
        self.cmd_x = self.cfg["robot"]["center_x_cm"]
        self.cmd_y = self.cfg["robot"]["center_y_cm"]
        
        if action == 'hover':
            self.cmd_z = self.cfg["robot"]["hover_z_cm"]
        elif action == 'pick':
            self.cmd_z = self.cfg["robot"]["pick_z_cm"]
            
        self.cmd_roll = -180.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0

        msg = Pose()
        msg.position.x = self.cmd_x / 100.0
        msg.position.y = self.cmd_y / 100.0
        msg.position.z = self.cmd_z / 100.0
        qx, qy, qz, qw = self.euler_to_quaternion(
            math.radians(self.cmd_roll), math.radians(self.cmd_pitch), math.radians(self.cmd_yaw))
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        self.cartesian_publisher.publish(msg)

    def execute_close(self, color):
        target_mm = self.cfg["robot"]["gripper_close_mm"]
        self.updating_sliders = True
        self.gripper_slider.set(target_mm)
        self.updating_sliders = False
        self.send_gripper_gap(target_mm)
        self.attached_color = color
        
        if self.attach_status_var:
            self.attach_status_var.set(f"Attaching {color} apple with virtual joint...")
            if hasattr(self, 'window'):
                self.window.update()
            
        def _attach_thread():
            ret = os.system(f'ign topic -t /attach_{color} -m ignition.msgs.Empty -p " "')
            if self.attach_status_var:
                if ret == 0:
                    self.window.after(0, lambda: self.attach_status_var.set(f"Attached {color} apple successfully!"))
                else:
                    self.window.after(0, lambda: self.attach_status_var.set(f"Error: Failed to attach {color} apple."))
                    
        threading.Thread(target=_attach_thread).start()

    def execute_open(self):
        target_mm = self.cfg["robot"]["gripper_open_mm"]
        self.updating_sliders = True
        self.gripper_slider.set(target_mm)
        self.updating_sliders = False
        self.send_gripper_gap(target_mm)
        
        if self.attach_status_var:
            self.attach_status_var.set("Releasing object from virtual joint...")
            if hasattr(self, 'window'):
                self.window.update()
            
        if self.attached_color:
            color = self.attached_color
            self.attached_color = None
            def _detach_thread():
                ret = os.system(f'ign topic -t /detach_{color} -m ignition.msgs.Empty -p " "')
                if self.attach_status_var:
                    if ret == 0:
                        self.window.after(0, lambda: self.attach_status_var.set(f"Detached {color} apple successfully!"))
                    else:
                        self.window.after(0, lambda: self.attach_status_var.set(f"Error: Failed to detach {color} apple."))
            threading.Thread(target=_detach_thread).start()

    def reset_gazebo_world(self):
        self.execute_open()
        if hasattr(self, 'window'):
            self.window.after(1500, self._perform_reset)
        else:
            threading.Timer(1.5, self._perform_reset).start()

    def _perform_reset(self):
        def _reset_thread():
            self.window.after(0, lambda: self.attach_status_var.set("Gazebo Sim: Detaching and halting momentum..."))
            
            # Detach both concurrently in background using strictly Ignition commands
            os.system('ign topic -t /detach_red -m ignition.msgs.Empty -p " " > /dev/null 2>&1 &')
            os.system('ign topic -t /detach_green -m ignition.msgs.Empty -p " " > /dev/null 2>&1 &')
            
            rx, ry, rz = self.cfg["simulation"]["apple_red_start_x"], self.cfg["simulation"]["apple_red_start_y"], self.cfg["simulation"]["reset_z"]
            gx, gy = self.cfg["simulation"]["apple_green_start_x"], self.cfg["simulation"]["apple_green_start_y"]
            
            cmd_red = f'ign service -s /world/default/set_pose --timeout 2000 --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --req \'name: "red_apple", position: {{x: {rx}, y: {ry}, z: {rz}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}\''
            cmd_green = f'ign service -s /world/default/set_pose --timeout 2000 --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --req \'name: "green_apple", position: {{x: {gx}, y: {gy}, z: {rz}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}\''
            
            # The "Physics Clamping" Trick:
            # set_pose only teleports objects; it inherently preserves their velocity buffers.
            # By rapidly clamping the objects to their exact start coordinates for ~0.75 seconds, 
            # we force Gazebo's DART physics solver to register zero displacement.
            # This triggers the engine's "auto-sleep" state, successfully hard-resetting all velocity.
            for _ in range(15):
                os.system(f'({cmd_red} > /dev/null 2>&1) & ({cmd_green} > /dev/null 2>&1) & wait')
                time.sleep(0.05)
            
            self.window.after(0, self._update_reset_ui)
            
        threading.Thread(target=_reset_thread).start()
        
    def _update_reset_ui(self):
        self.live_red_target = None
        self.live_green_target = None
        self.captured_red_target = None
        self.captured_green_target = None
        self.cap_red_var.set("Captured Red\nWaiting...")
        self.cap_green_var.set("Captured Green\nWaiting...")
        if self.attach_status_var:
            self.attach_status_var.set("Gazebo Sim: Apples Reset to Initial Positions.")

    def run_automation_sequence(self):
        def sequence_thread():
            self.attach_status_var.set("Automation: Iniciando secuencia...")
            time.sleep(0.5)

            steps = [
                ("Search state", self.go_search_state, "move"),
                ("Capture targets", self.capture_positions, "instant"),
                ("Hover red pos", lambda: self.execute_apple_action('red', 'hover'), "move"),
                ("Pick red target pos", lambda: self.execute_apple_action('red', 'pick'), "move"),
                ("Close red", lambda: self.execute_close('red'), "gripper"),
                ("Hover red pos", lambda: self.execute_apple_action('red', 'hover'), "move"),
                ("Hover center pos", lambda: self.execute_center_action('hover'), "move"),
                ("Place center pos", lambda: self.execute_center_action('pick'), "move"),
                ("Open red", self.execute_open, "gripper"),
                ("Hover center pos", lambda: self.execute_center_action('hover'), "move"),
                ("Hover green pos", lambda: self.execute_apple_action('green', 'hover'), "move"),
                ("Pick green pos", lambda: self.execute_apple_action('green', 'pick'), "move"),
                ("Close green", lambda: self.execute_close('green'), "gripper"),
                ("Hover green pos", lambda: self.execute_apple_action('green', 'hover'), "move"),
                ("Hover red pos", lambda: self.execute_apple_action('red', 'hover'), "move"),
                ("Pick red pos", lambda: self.execute_apple_action('red', 'pick'), "move"),
                ("Open green", self.execute_open, "gripper"),
                ("Hover red pos", lambda: self.execute_apple_action('red', 'hover'), "move"),
                ("Hover center pos", lambda: self.execute_center_action('hover'), "move"),
                ("Pick center pos", lambda: self.execute_center_action('pick'), "move"),
                ("Close red", lambda: self.execute_close('red'), "gripper"),
                ("Hover center pos", lambda: self.execute_center_action('hover'), "move"),
                ("Hover green pos", lambda: self.execute_apple_action('green', 'hover'), "move"),
                ("Pick green pos", lambda: self.execute_apple_action('green', 'pick'), "move"),
                ("Open red", self.execute_open, "gripper"),
                ("Hover green pos", lambda: self.execute_apple_action('green', 'hover'), "move"),
                ("Search state", self.go_search_state, "move")
            ]

            for name, action, step_type in steps:
                self.window.after(0, lambda n=name: self.attach_status_var.set(f"Automation: {n}"))
                action()
                
                if name == "Capture targets":
                    if self.captured_red_target is None or self.captured_green_target is None:
                        self.window.after(0, lambda: self.attach_status_var.set("Automation Error: Targets not found!"))
                        return
                    time.sleep(0.5)
                elif step_type == "move":
                    success = self.wait_for_movement(timeout=10.0)
                    if not success:
                        self.window.after(0, lambda n=name: self.attach_status_var.set(f"Automation Warn: Timeout on {n}"))
                elif step_type == "gripper":
                    time.sleep(1.0) 

            self.window.after(0, lambda: self.attach_status_var.set("Automation: Sequence Complete!"))

        threading.Thread(target=sequence_thread).start()

    def wait_for_movement(self, timeout=10.0):
        start_time = time.time()
        time.sleep(0.5) 
        
        tol_cm = self.cfg["robot"]["tolerance_cm"]
        tol_rad = self.cfg["robot"]["tolerance_rad"]
        
        while time.time() - start_time < timeout:
            if self.mode == "cartesian":
                dist = math.sqrt((self.real_x_cm - self.cmd_x)**2 + 
                                 (self.real_y_cm - self.cmd_y)**2 + 
                                 (self.real_z_cm - self.cmd_z)**2)
                if dist < tol_cm: 
                    time.sleep(0.2) 
                    return True
            elif self.mode == "joint":
                if hasattr(self, 'target_angles_rad') and len(self.latest_arm_angles) == 6:
                    diffs = [abs(self.latest_arm_angles[i] - self.target_angles_rad[i]) for i in range(6)]
                    if max(diffs) < tol_rad: 
                        time.sleep(0.2)
                        return True
            time.sleep(0.1)
        return False

    def send_arm_angles(self, angles_rad):
        self.target_angles_rad = angles_rad
        
        max_diff_rad = 0.1
        if hasattr(self, 'latest_arm_angles') and len(self.latest_arm_angles) == 6:
            for i in range(6):
                diff = abs(angles_rad[i] - self.latest_arm_angles[i])
                if diff > max_diff_rad:
                    max_diff_rad = diff
        
        calc_time = max(1.0, max_diff_rad / 1.0)
        sec = int(calc_time)
        nanosec = int((calc_time - sec) * 1e9)

        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = angles_rad
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = nanosec
        
        traj_msg.points.append(point)
        self.arm_publisher.publish(traj_msg)

    def send_gripper_gap(self, gap_mm):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.gripper_joint_names
        point = JointTrajectoryPoint()
        finger_pos = (100.0 - float(gap_mm)) / 2000.0
        finger_pos = max(0.0, min(0.05, finger_pos))
        point.positions = [finger_pos, finger_pos]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500000000
        traj_msg.points.append(point)
        self.gripper_publisher.publish(traj_msg)

        if hasattr(self, 'latest_arm_angles') and len(self.latest_arm_angles) == 6:
            self.send_arm_angles(self.latest_arm_angles)

    def slider_moved(self, *args):
        if self.updating_sliders or self.mode != "joint": return
        angles_rad = []
        for i in range(6):
            val = self.sliders[i].get()
            if len(self.entry_vars) == 6:
                self.entry_vars[i].set(f"{val:.1f}")
            angles_rad.append(math.radians(val))
        self.send_arm_angles(angles_rad)

    def entry_changed(self, event, idx):
        self.mode = "joint"
        self.updating_sliders = True
        try:
            val = float(self.entry_vars[idx].get())
            self.sliders[idx].set(val)
            angles_rad = []
            for i in range(6):
                angles_rad.append(math.radians(self.sliders[i].get()))
            self.send_arm_angles(angles_rad)
        except ValueError:
            pass
        self.updating_sliders = False

    def gripper_moved(self, *args):
        if self.updating_sliders: return
        self.send_gripper_gap(self.gripper_slider.get())

    def set_mode_joint(self, event):
        self.mode = "joint"

    def go_home(self):
        self.mode = "joint"
        self.updating_sliders = True
        home_angles_deg = [0.0, -90.0, -90.0, -90.0, 90.0, 0.0]
        home_angles_rad = [0.0, -1.5708, -1.5708, -1.5708, 1.5708, 0.0]
        for i in range(6):
            self.sliders[i].set(home_angles_deg[i])
            if len(self.entry_vars) == 6:
                self.entry_vars[i].set(f"{home_angles_deg[i]:.1f}")
        self.updating_sliders = False
        self.send_arm_angles(home_angles_rad)

    def go_search_state(self):
        self.mode = "joint"
        self.updating_sliders = True
        search_angles_deg = [0.0, -90.0, -90.0, -90.0, 90.0, 0.0]
        search_angles_rad = [0.0, -1.5708, -1.5708, -1.5708, 1.5708, 0.0]
        for i in range(6):
            self.sliders[i].set(search_angles_deg[i])
            if len(self.entry_vars) == 6:
                self.entry_vars[i].set(f"{search_angles_deg[i]:.1f}")
        self.updating_sliders = False
        self.send_arm_angles(search_angles_rad)


def run_interface(ros_node):
    cfg = ros_node.cfg
    window = tk.Tk()
    ros_node.window = window
    window.title(cfg["ui"]["window_title"])
    window.geometry(cfg["ui"]["window_geometry"])

    main_frame = tk.Frame(window)
    main_frame.pack(fill="both", expand=True, padx=cfg["ui"]["pad_x"], pady=cfg["ui"]["pad_y"])

    control_frame = tk.Frame(main_frame)
    control_frame.grid(row=0, column=0, sticky="n")

    tk.Label(control_frame, text="Cartesian (cm)", font=cfg["ui"]["font_h2"]).grid(row=0, column=0, columnspan=3, pady=cfg["ui"]["btn_pad_y"])
    tk.Label(control_frame, text="Manual Joint Control", font=cfg["ui"]["font_h2"]).grid(row=0, column=3, columnspan=3, pady=cfg["ui"]["btn_pad_y"])

    def click_cartesian(axis, direction): ros_node.move_cartesian(axis, direction)

    bw = cfg["ui"]["btn_width_sm"]
    px = cfg["ui"]["btn_pad_x"]
    py = cfg["ui"]["btn_pad_y"]
    
    tk.Button(control_frame, text="X +", command=lambda: click_cartesian('x', 1), height=1, width=bw).grid(row=1, column=2, padx=px)
    tk.Button(control_frame, text="X -", command=lambda: click_cartesian('x', -1), height=1, width=bw).grid(row=1, column=0, padx=px)
    tk.Button(control_frame, text="Y +", command=lambda: click_cartesian('y', 1), height=1, width=bw).grid(row=2, column=2, pady=py)
    tk.Button(control_frame, text="Y -", command=lambda: click_cartesian('y', -1), height=1, width=bw).grid(row=2, column=0, pady=py)
    tk.Button(control_frame, text="Z +", command=lambda: click_cartesian('z', 1), height=1, width=bw).grid(row=3, column=2, pady=py)
    tk.Button(control_frame, text="Z -", command=lambda: click_cartesian('z', -1), height=1, width=bw).grid(row=3, column=0, pady=py)
    tk.Button(control_frame, text="Roll +", command=lambda: click_cartesian('roll', 1), height=1, width=bw).grid(row=4, column=2, pady=py)
    tk.Button(control_frame, text="Roll -", command=lambda: click_cartesian('roll', -1), height=1, width=bw).grid(row=4, column=0, pady=py)
    tk.Button(control_frame, text="Pitch +", command=lambda: click_cartesian('pitch', 1), height=1, width=bw).grid(row=5, column=2, pady=py)
    tk.Button(control_frame, text="Pitch -", command=lambda: click_cartesian('pitch', -1), height=1, width=bw).grid(row=5, column=0, pady=py)
    tk.Button(control_frame, text="Yaw +", command=lambda: click_cartesian('yaw', 1), height=1, width=bw).grid(row=6, column=2, pady=py)
    tk.Button(control_frame, text="Yaw -", command=lambda: click_cartesian('yaw', -1), height=1, width=bw).grid(row=6, column=0, pady=py)

    slider_frame = tk.Frame(control_frame)
    slider_frame.grid(row=1, column=3, rowspan=6, columnspan=3, padx=cfg["ui"]["pad_x"])

    labels = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]
    for i in range(6):
        tk.Label(slider_frame, text=labels[i]).grid(row=i, column=0, sticky="e")
        s = tk.Scale(slider_frame, from_=-180, to=180, orient="horizontal", length=150, showvalue=0)
        s.bind("<Button-1>", ros_node.set_mode_joint)
        s.bind("<ButtonRelease-1>", ros_node.slider_moved)
        s.grid(row=i, column=1)
        ros_node.sliders.append(s)
        var = tk.StringVar(value="0.0")
        ros_node.entry_vars.append(var)
        e = tk.Entry(slider_frame, textvariable=var, width=5)
        e.bind("<Return>", lambda event, idx=i: ros_node.entry_changed(event, idx))
        e.bind("<FocusOut>", lambda event, idx=i: ros_node.entry_changed(event, idx))
        e.bind("<Button-1>", ros_node.set_mode_joint)
        e.grid(row=i, column=2, padx=5)

    btn_frame = tk.Frame(slider_frame)
    btn_frame.grid(row=6, column=0, columnspan=3, pady=10)
    tk.Button(btn_frame, text="GO HOME", command=ros_node.go_home, bg=cfg["ui"]["colors"]["home_btn"]).pack(side="left", padx=5)
    tk.Button(btn_frame, text="SEARCH STATE", command=ros_node.go_search_state, bg=cfg["ui"]["colors"]["search_btn"]).pack(side="left", padx=5)

    tk.Label(control_frame, text="Gripper Gap (mm)", font=cfg["ui"]["font_mono"]).grid(row=7, column=3, columnspan=3)
    gripper_slider = tk.Scale(control_frame, from_=0, to=100, orient="horizontal", length=150)
    gripper_slider.bind("<Button-1>", ros_node.set_mode_joint)
    gripper_slider.bind("<ButtonRelease-1>", ros_node.gripper_moved)
    gripper_slider.set(100)
    gripper_slider.grid(row=8, column=3, columnspan=3)
    ros_node.gripper_slider = gripper_slider

    status_frame = tk.Frame(control_frame)
    status_frame.grid(row=9, column=0, columnspan=6, pady=20)

    real_text = tk.StringVar()
    real_text.set("Listening to real position...")
    ros_node.real_loc_var = real_text
    angle_text = tk.StringVar()
    angle_text.set("Waiting for joint angles...")
    ros_node.angle_text_var = angle_text

    tk.Label(status_frame, text="Real Position (World to TCP):", font=cfg["ui"]["font_h3"]).pack()
    tk.Label(status_frame, textvariable=real_text, font=cfg["ui"]["font_mono_bold"], fg=cfg["ui"]["colors"]["status_warn"]).pack()
    tk.Label(status_frame, textvariable=angle_text, font=cfg["ui"]["font_mono"], fg=cfg["ui"]["colors"]["status_green"]).pack()

    action_frame = tk.Frame(main_frame)
    action_frame.grid(row=0, column=1, padx=cfg["ui"]["pad_x"], sticky="n")

    tk.Label(action_frame, text="Vision Targets & Actions", font=cfg["ui"]["font_h1"]).pack(pady=2)

    top_btn_frame = tk.Frame(action_frame)
    top_btn_frame.pack(pady=2)
    tk.Button(top_btn_frame, text="RESET APPLES", command=ros_node.reset_gazebo_world, bg=cfg["ui"]["colors"]["reset_btn"], font=cfg["ui"]["font_h3"], width=15, height=1).pack(side="left", padx=2)
    tk.Button(top_btn_frame, text="CAPTURE TARGETS", command=ros_node.capture_positions, bg=cfg["ui"]["colors"]["capture_btn"], fg=cfg["ui"]["colors"]["capture_text"], font=cfg["ui"]["font_h3"], width=18, height=1).pack(side="left", padx=2)
    tk.Button(top_btn_frame, text="RUN AUTO", command=ros_node.run_automation_sequence, bg=cfg["ui"]["colors"]["auto_btn"], font=cfg["ui"]["font_h3"], width=12, height=1).pack(side="left", padx=2)

    ros_node.attach_status_var = tk.StringVar(value="Status: Waiting for user action...")
    tk.Label(action_frame, textvariable=ros_node.attach_status_var, font=cfg["ui"]["font_h3"], fg=cfg["ui"]["colors"]["status_ok"]).pack(pady=2)
    
    live_frame = tk.Frame(action_frame)
    live_frame.pack(pady=2)
    ros_node.live_red_var = tk.StringVar(value="Live Red\nWaiting...")
    tk.Label(live_frame, textvariable=ros_node.live_red_var, font=cfg["ui"]["font_mono"], fg=cfg["ui"]["colors"]["status_warn"]).pack(side="left", padx=20)
    ros_node.live_green_var = tk.StringVar(value="Live Green\nWaiting...")
    tk.Label(live_frame, textvariable=ros_node.live_green_var, font=cfg["ui"]["font_mono"], fg=cfg["ui"]["colors"]["status_green"]).pack(side="left", padx=20)

    cap_frame = tk.Frame(action_frame)
    cap_frame.pack(pady=2)
    
    bw_lg = cfg["ui"]["btn_width_lg"]
    bw_md = cfg["ui"]["btn_width_md"]
    
    red_col = tk.Frame(cap_frame)
    red_col.pack(side="left", padx=5)
    ros_node.cap_red_var = tk.StringVar(value="Captured Red\nWaiting...")
    tk.Label(red_col, textvariable=ros_node.cap_red_var, font=cfg["ui"]["font_mono"], fg="darkred", width=25).pack(pady=2)
    tk.Button(red_col, text="Hover Red (+10cm)", command=lambda: ros_node.execute_apple_action('red', 'hover'), bg=cfg["ui"]["colors"]["red_hover"], width=bw_lg).pack(pady=2)
    tk.Button(red_col, text="Pick Red Target", command=lambda: ros_node.execute_apple_action('red', 'pick'), bg=cfg["ui"]["colors"]["red_pick"], width=bw_lg).pack(pady=2)
    
    red_btn_frame = tk.Frame(red_col)
    red_btn_frame.pack(pady=2)
    tk.Button(red_btn_frame, text="Close", command=lambda: ros_node.execute_close('red'), bg=cfg["ui"]["colors"]["red_close"], width=bw_md).pack(side="left", padx=1)
    tk.Button(red_btn_frame, text="Open", command=ros_node.execute_open, bg=cfg["ui"]["colors"]["red_open"], width=bw_md).pack(side="left", padx=1)

    green_col = tk.Frame(cap_frame)
    green_col.pack(side="left", padx=5)
    ros_node.cap_green_var = tk.StringVar(value="Captured Green\nWaiting...")
    tk.Label(green_col, textvariable=ros_node.cap_green_var, font=cfg["ui"]["font_mono"], fg="darkgreen", width=25).pack(pady=2)
    tk.Button(green_col, text="Hover Green (+10cm)", command=lambda: ros_node.execute_apple_action('green', 'hover'), bg=cfg["ui"]["colors"]["green_hover"], width=bw_lg).pack(pady=2)
    tk.Button(green_col, text="Pick Green Target", command=lambda: ros_node.execute_apple_action('green', 'pick'), bg=cfg["ui"]["colors"]["green_pick"], width=bw_lg).pack(pady=2)
    
    green_btn_frame = tk.Frame(green_col)
    green_btn_frame.pack(pady=2)
    tk.Button(green_btn_frame, text="Close", command=lambda: ros_node.execute_close('green'), bg=cfg["ui"]["colors"]["green_close"], width=bw_md).pack(side="left", padx=1)
    tk.Button(green_btn_frame, text="Open", command=ros_node.execute_open, bg=cfg["ui"]["colors"]["green_open"], width=bw_md).pack(side="left", padx=1)

    center_col = tk.Frame(cap_frame)
    center_col.pack(side="left", padx=5)
    tk.Label(center_col, text=f"Center Swap Area\nX: {cfg['robot']['center_x_cm']}  Y: {cfg['robot']['center_y_cm']}", font=cfg["ui"]["font_mono"], fg=cfg["ui"]["colors"]["status_ok"], width=25).pack(pady=2)
    tk.Button(center_col, text="Hover Center (+10cm)", command=lambda: ros_node.execute_center_action('hover'), bg=cfg["ui"]["colors"]["center_hover"], width=bw_lg).pack(pady=2)
    tk.Button(center_col, text="Place/Pick Center", command=lambda: ros_node.execute_center_action('pick'), bg=cfg["ui"]["colors"]["center_pick"], width=bw_lg).pack(pady=2)
    
    center_btn_frame = tk.Frame(center_col)
    center_btn_frame.pack(pady=2)
    tk.Button(center_btn_frame, text="Release Apple", command=ros_node.execute_open, bg=cfg["ui"]["colors"]["center_open"], width=15).pack(side="left", padx=1)

    debug_frame = tk.Frame(main_frame, highlightbackground="black", highlightthickness=1)
    debug_frame.grid(row=0, column=2, sticky="n")

    tk.Label(debug_frame, text="Transformation Matrices", font=cfg["ui"]["font_h2"]).pack(pady=5)
    ros_node.debug_cam_to_apple = tk.StringVar(value="Cam to Apple: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_cam_to_apple, font=cfg["ui"]["font_mono"], justify="left").pack(pady=2, anchor="w")
    ros_node.debug_base_to_tcp = tk.StringVar(value="Base to TCP: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_base_to_tcp, font=cfg["ui"]["font_mono"], justify="left").pack(pady=2, anchor="w")
    ros_node.debug_base_to_cam = tk.StringVar(value="Base to Cam: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_base_to_cam, font=cfg["ui"]["font_mono"], justify="left").pack(pady=2, anchor="w")
    ros_node.debug_world_to_tcp = tk.StringVar(value="World to TCP: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_world_to_tcp, font=cfg["ui"]["font_mono"], justify="left").pack(pady=2, anchor="w")
    ros_node.debug_world_to_cam = tk.StringVar(value="World to Cam: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_world_to_cam, font=cfg["ui"]["font_mono"], justify="left").pack(pady=2, anchor="w")
    ros_node.debug_base_to_apple = tk.StringVar(value="Base to Apple: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_base_to_apple, font=cfg["ui"]["font_mono"], justify="left").pack(pady=2, anchor="w")
    ros_node.debug_world_to_apple = tk.StringVar(value="World to Apple: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_world_to_apple, font=cfg["ui"]["font_mono"], justify="left").pack(pady=2, anchor="w")

    tk.Label(debug_frame, text="Ground Truth Comparisons", font=cfg["ui"]["font_h2"]).pack(pady=(15,5))
    ros_node.debug_real_apple = tk.StringVar(value="Gazebo Ground Truth: Waiting...")
    tk.Label(debug_frame, textvariable=ros_node.debug_real_apple, font=cfg["ui"]["font_mono"], fg=cfg["ui"]["colors"]["status_ok"], justify="left").pack(pady=2, anchor="w")

    def _init_detach():
        os.system('ign topic -t /detach_red -m ignition.msgs.Empty -p " " &')
        os.system('ign topic -t /detach_green -m ignition.msgs.Empty -p " " &')
    threading.Thread(target=_init_detach).start()

    ros_node.go_home()
    ros_node.reset_gazebo_world()

    def ros_spin():
        rclpy.spin_once(ros_node, timeout_sec=0.01)
        window.after(20, ros_spin)
        
    window.after(20, ros_spin)
    window.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = VisualJogger()
    run_interface(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()