# 文件名: robot_controller.py (已修正)
# -*- coding: utf-8 -*-
import os
import pickle
import time
import math
import random
import threading
import subprocess
import json
import traceback
# from HersheyFonts import HersheyFonts
# use lerobot SO100 follower interface (same as used in myrobot.py)
try:
    from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
except Exception:
    SO100Follower = None
    SO100FollowerConfig = None

class RobotController:
    def __init__(self, port1="/dev/ttyACM0", port2=None):
        # 1. 定义所有物理参数和状态变量
        self.SPEED_MOVE = 80
        self.NEUTRAL_POSE = [250, 0, 80, 0, 0, 0]
        self.FAST_SPEED = 125
        self.MEDIUM_SPEED = 70
        self.SLOW_SPEED = 30
        self.BREATHE_IN_SPEED = 15
        self.BREATHE_OUT_SPEED = 40
        self.current_pos = self.NEUTRAL_POSE[:]
        
        self.animation_thread = None
        self.stop_animation_flag = threading.Event()
        self.stop_task_flag = threading.Event()
        
        # writing/pen related functionality removed

    # writing/pen related functionality removed
        
        # 2. 硬件连接和初始化
        print("🔧 初始化双臂机械臂 (lerobot)...")
        # robots will be stored as {'arm1': <robot>, 'arm2': <robot>} when ports provided
        self.robots = {'arm1': None, 'arm2': None}

        # helper to create+connect a SO100Follower
        def _create_robot(port_path):
            if SO100Follower is None:
                raise RuntimeError("lerobot package not available")
            cfg = SO100FollowerConfig(port=port_path)
            r = SO100Follower(cfg)
            r.connect()
            try:
                if hasattr(r, 'calibrate'):
                    r.calibrate()
            except Exception:
                pass
            return r

        # attempt to create arm1
        try:
            self.robots['arm1'] = _create_robot(port1)
            print(f"✅ arm1 连接成功: {port1}")
        except Exception as e:
            print(f"⚠️ arm1 连接失败 ({port1}): {e}")
            self.robots['arm1'] = None

        # attempt to create arm2 if provided
        if port2:
            try:
                self.robots['arm2'] = _create_robot(port2)
                print(f"✅ arm2 连接成功: {port2}")
            except Exception as e:
                print(f"⚠️ arm2 连接失败 ({port2}): {e}")
                self.robots['arm2'] = None

        # set a default convenience attribute for single-arm compatibility
        # prefer arm1, fall back to arm2 if only one is connected
        self.ua = self.robots.get('arm1') or self.robots.get('arm2')

        # timer used to restore default face after a face_<name> call with timeout
        self._reset_timer = None

        # dynamically create face_<name> methods so voice_assistant can call them on robot_controller
        try:
            self._register_face_methods()
        except Exception as e:
            print(f"⚠️ 无法在 RobotController 上注册 face_ 方法: {e}")

        # (no external writing resources required)
    
    def _move_to(self, target_pos, speed, arm=None):
        # Adapted for lerobot: convert a target cartesian pose [x,y,z,rx,ry,rz]
        # to joint goals using inverse kinematics and send via send_action.
        # If arm is None, use the currently active arm (set by callers like robot_write),
        # otherwise default to 'arm1'.
        if arm is None:
            arm = getattr(self, '_active_arm', None) or 'arm1'
        robot = self.robots.get(arm)
        if robot is None:
            robot = self.ua
        if robot is None:
            return
        # We only use x,y for planar inverse kinematics from myrobot.py
        try:
            x = target_pos[0] / 1000.0 if abs(target_pos[0]) > 10 else target_pos[0]
            y = target_pos[1] / 1000.0 if abs(target_pos[1]) > 10 else target_pos[1]
            # Use the inverse kinematics from myrobot.py logic (units: meters)
            # fall back to simple mapping if inverse_kinematics not present
            try:
                # inline IK from myrobot.py
                def inverse_kinematics(x, y, l1=0.1159, l2=0.1350):
                    theta1_offset = math.atan2(0.028, 0.11257)
                    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
                    r = math.sqrt(x**2 + y**2)
                    r_max = l1 + l2
                    if r > r_max:
                        scale_factor = r_max / r
                        x *= scale_factor
                        y *= scale_factor
                        r = r_max
                    r_min = abs(l1 - l2)
                    if r < r_min and r > 0:
                        scale_factor = r_min / r
                        x *= scale_factor
                        y *= scale_factor
                        r = r_min
                    cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)
                    theta2 = math.pi - math.acos(max(-1.0, min(1.0, cos_theta2)))
                    beta = math.atan2(y, x)
                    gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
                    theta1 = beta + gamma
                    joint2 = theta1 + theta1_offset
                    joint3 = theta2 + theta2_offset
                    joint2 = max(-0.1, min(3.45, joint2))
                    joint3 = max(-0.2, min(math.pi, joint3))
                    joint2_deg = math.degrees(joint2)
                    joint3_deg = math.degrees(joint3)
                    joint2_deg = 90 - joint2_deg
                    joint3_deg = joint3_deg - 90
                    return joint2_deg, joint3_deg
                # ensure meters: many parts of the file use mm; try to detect
                # If values seem large (>10), assume mm and convert to meters for IK
                x_m = x / 1000.0 if abs(x) > 10 else x
                y_m = y / 1000.0 if abs(y) > 10 else y
                j2_deg, j3_deg = inverse_kinematics(x_m, y_m)
                # Construct action dict using joint names expected in myrobot
                action = {
                    'shoulder_lift.pos': j2_deg,
                    'elbow_flex.pos': j3_deg
                }
                # Optionally set shoulder_pan from target_pos[1] or current
                # Use current_pos[1] delta to set shoulder_pan as small adjustment
                if len(target_pos) >= 6:
                    pan_guess = target_pos[5] if target_pos[5] is not None else 0.0
                    # set shoulder_pan only if bus supports it
                    action['shoulder_pan.pos'] = pan_guess
                robot.send_action(action)
                self.current_pos = target_pos
            except Exception:
                # fallback: try set_coords if available
                if hasattr(robot, 'set_coords'):
                    try:
                        robot.set_coords(target_pos, speed)
                        self.current_pos = target_pos
                    except Exception:
                        pass
        except Exception:
            return

    def move_zero(self):
        print("  -> 回到原点")
        self._move_to(self.NEUTRAL_POSE, self.MEDIUM_SPEED)
    
    def back_zero(self):
        self.move_zero()

    def _thinking_loop(self):
        print(" -> 动画开始: 思考 (缓慢摇头)")
        sway_y_amp = 20
        rotation_z_amp = 10
        pose_left = [self.NEUTRAL_POSE[0], self.NEUTRAL_POSE[1] + sway_y_amp, self.NEUTRAL_POSE[2], 0, 0, -rotation_z_amp]
        pose_right = [self.NEUTRAL_POSE[0], self.NEUTRAL_POSE[1] - sway_y_amp, self.NEUTRAL_POSE[2], 0, 0, rotation_z_amp]
        while not self.stop_animation_flag.is_set():
            self._move_to(pose_left, self.SLOW_SPEED)
            if self.stop_animation_flag.wait(timeout=1.0): break
            self._move_to(pose_right, self.SLOW_SPEED)
            if self.stop_animation_flag.wait(timeout=1.0): break
        print(" -> 动画结束: 思考")

    def _talking_loop(self):
        print(" -> 动画开始: 说话 (节奏点头)")
        while not self.stop_animation_flag.is_set():
            # --- MODIFICATION START ---
            # 1. Nodding amplitude and speed adjusted as per user request.
            nod_x_amp = random.uniform(2, 5)     # Amplitude reduced for subtlety
            nod_z_amp = random.uniform(-5, -10)  # Amplitude reduced for subtlety
            speed = 20                           # Speed slowed down to 20
            # --- MODIFICATION END ---
            
            pose_nod_down = [self.NEUTRAL_POSE[0] + nod_x_amp, self.NEUTRAL_POSE[1], self.NEUTRAL_POSE[2] + nod_z_amp, 0, 0, 0]
            
            self._move_to(pose_nod_down, speed)
            if self.stop_animation_flag.wait(timeout=0.2): break
            self._move_to(self.NEUTRAL_POSE, speed)
            if self.stop_animation_flag.wait(timeout=0.2): break
        print(" -> 动画结束: 说话")

    def _start_animation(self, loop_function):
        if self.animation_thread and self.animation_thread.is_alive():
            self.stop_animation(return_to_neutral=False)
        self.stop_animation_flag.clear()
        self.animation_thread = threading.Thread(target=loop_function, daemon=True)
        self.animation_thread.start()

    def start_thinking_animation(self):
        self._start_animation(self._thinking_loop)

    def start_talking_animation(self):
        self._start_animation(self._talking_loop)

    def stop_animation(self, return_to_neutral=False):
        if self.animation_thread and self.animation_thread.is_alive():
            self.stop_animation_flag.set()
            self.animation_thread.join(timeout=1.5)
        self.stop_animation_flag.clear()
        if return_to_neutral:
             self._move_to(self.NEUTRAL_POSE, self.MEDIUM_SPEED)
        
    def stop_current_task(self):
        print("▶️ 发出任务中断信号...")
        self.stop_task_flag.set()
        self.stop_animation(return_to_neutral=False)

    def move_forward(self, distance, arm='arm1'):
        target_pos = self.current_pos[:]; target_pos[0] += float(distance)
        print("  -> 前进", distance, "当前坐标:", target_pos)
        self._move_to(target_pos, self.SPEED_MOVE, arm=arm)

    def move_back(self, distance, arm='arm1'):
        target_pos = self.current_pos[:]; target_pos[0] -= float(distance)
        print("  -> 后退", distance, "当前坐标:", target_pos)
        self._move_to(target_pos, self.SPEED_MOVE, arm=arm)

    def move_left(self, distance, arm='arm1'):
        target_pos = self.current_pos[:]; target_pos[1] -= float(distance)
        print("  -> 左移", distance, "当前坐标:", target_pos)
        self._move_to(target_pos, self.SPEED_MOVE, arm=arm)

    def move_right(self, distance, arm='arm1'):
        target_pos = self.current_pos[:]; target_pos[1] += float(distance)
        print("  -> 右移", distance, "当前坐标:", target_pos)
        self._move_to(target_pos, self.SPEED_MOVE, arm=arm)

    def move_up(self, distance, arm='arm1'):
        target_pos = self.current_pos[:]; target_pos[2] += float(distance)
        print("  -> 上移", distance, "当前坐标:", target_pos)
        self._move_to(target_pos, self.SPEED_MOVE, arm=arm)

    def move_down(self, distance, arm='arm1'):
        target_pos = self.current_pos[:]; target_pos[2] -= float(distance)
        print("  -> 下移", distance, "当前坐标:", target_pos)
        self._move_to(target_pos, self.SPEED_MOVE, arm=arm)


    def move_to_random_vqa_point(self):
        print("  -> (空闲) 去一个新的地方看看...")
        random_x = random.uniform(100, 300)
        random_y = random.uniform(-150, 150)
        random_z = random.uniform(-20, 100)
        random_pose = [random_x, random_y, random_z, 0, 0, 0]
        self._move_to(random_pose, self.MEDIUM_SPEED)
        time.sleep(1.5)

    def perform_subtle_look(self):
        print("  -> (空闲) 执行动作: 谨慎扫视")
        look_amplitude_y = random.uniform(-120, 120)
        random_rotation_z = random.uniform(8, 15)
        look_pos_left = [self.NEUTRAL_POSE[0], self.NEUTRAL_POSE[1] + look_amplitude_y, self.NEUTRAL_POSE[2] - 5, 0, 0, random_rotation_z]
        look_pos_right = [self.NEUTRAL_POSE[0], self.NEUTRAL_POSE[1] - look_amplitude_y, self.NEUTRAL_POSE[2] - 5, 0, 0, -random_rotation_z]
        self._move_to(look_pos_left, self.MEDIUM_SPEED)
        time.sleep(0.8)
        self._move_to(look_pos_right, self.MEDIUM_SPEED)
        time.sleep(0.9)
        self._move_to(self.NEUTRAL_POSE, self.MEDIUM_SPEED)

    def perform_head_tilt(self):
        print("  -> (空闲) 执行动作: 好奇歪头")
        direction = random.choice([-1, 1])
        tilt_y_amp = random.uniform(-100, 100) * direction
        tilt_z_amp = random.uniform(-35, 30)
        random_rotation_z = random.uniform(10, 20) * direction
        tilt_pose = [self.NEUTRAL_POSE[0], self.NEUTRAL_POSE[1] + tilt_y_amp, self.NEUTRAL_POSE[2] + tilt_z_amp, 0, 0, random_rotation_z]
        self._move_to(tilt_pose, self.MEDIUM_SPEED)
        time.sleep(1.2)
        self._move_to(self.NEUTRAL_POSE, self.MEDIUM_SPEED)

    def perform_gentle_breath(self):
        print("  -> (空闲) 执行动作: 温和呼吸")
        random_y_offset = random.uniform(-150, 150)
        temp_neutral_pose = self.NEUTRAL_POSE.copy()
        temp_neutral_pose[1] += random_y_offset
        self._move_to(temp_neutral_pose, self.MEDIUM_SPEED)
        time.sleep(1.0)
        breath_rounds = random.randint(2, 4)
        for i in range(breath_rounds):
            x_amp = random.uniform(5, 15)
            z_amp = random.uniform(8, 20)
            inhale_pose = [temp_neutral_pose[0] + x_amp, temp_neutral_pose[1], temp_neutral_pose[2] - z_amp, 0, 0, 0]
            self._move_to(inhale_pose, self.BREATHE_IN_SPEED)
            time.sleep(0.8)
            self._move_to(temp_neutral_pose, self.BREATHE_OUT_SPEED)
            time.sleep(0.4)
        self._move_to(self.NEUTRAL_POSE, self.MEDIUM_SPEED)

    def perform_quick_peck(self):
        print("  -> (空闲) 执行动作: 快速啄击")
        peck_x = self.NEUTRAL_POSE[0] + random.uniform(25, 35)
        peck_y = self.NEUTRAL_POSE[1] + random.uniform(-10, 10)
        peck_z = self.NEUTRAL_POSE[2] - random.uniform(75, 85)
        peck_pose = [peck_x, peck_y, peck_z, 0, 0, 0]
        self._move_to(peck_pose, self.FAST_SPEED)
        time.sleep(0.15)
        self._move_to(self.NEUTRAL_POSE, self.FAST_SPEED)

    def perform_wide_sway(self):
        print("  -> (空闲) 执行动作: 开阔摇摆")
        sway_y_amp = random.uniform(70, 80)
        sway_z_amp = random.uniform(20, 30)
        random_rotation_z = random.uniform(5, 10)
        sway_left_pose = [self.NEUTRAL_POSE[0], self.NEUTRAL_POSE[1] + sway_y_amp, self.NEUTRAL_POSE[2] - sway_z_amp, 0, 0, random_rotation_z]
        sway_right_pose = [self.NEUTRAL_POSE[0], self.NEUTRAL_POSE[1] - sway_y_amp, self.NEUTRAL_POSE[2] - sway_z_amp, 0, 0, -random_rotation_z]
        self._move_to(sway_left_pose, self.MEDIUM_SPEED)
        time.sleep(0.5)
        self._move_to(sway_right_pose, self.MEDIUM_SPEED)
        time.sleep(0.5)
        self._move_to(self.NEUTRAL_POSE, self.MEDIUM_SPEED)
        
    def get_idle_action_list(self):
        return [
            self.perform_subtle_look,
            self.perform_head_tilt,
            self.perform_gentle_breath,
            self.perform_quick_peck,
            self.perform_wide_sway
        ]

    def _does_text_fit(self, text, char_height_mm, spacing_ratio):
        area_width = self.WRITE_AREA['y_max'] - self.WRITE_AREA['y_min']
        area_height = self.WRITE_AREA['x_max'] - self.WRITE_AREA['x_min']
        line_height = char_height_mm * spacing_ratio
        if line_height > area_height: return False
        cursor_x_rel = area_height
        cursor_y_rel = 0
        for char in text:
            is_full_width_punct = char in self.PUNCTUATION_MAP
            is_chinese = self.PUNCTUATION_MAP.get(char, char) in self.chinese_data
            char_width = char_height_mm if (is_chinese or is_full_width_punct) else char_height_mm / 2.0
            if char == ' ': 
                cursor_y_rel += char_width
                continue
            if cursor_y_rel + char_width > area_width:
                cursor_y_rel = 0
                cursor_x_rel -= line_height
                if cursor_x_rel - line_height < 0: return False
            cursor_y_rel += char_width * spacing_ratio
        return True
        
    # writing-related helpers fully removed

    def replay_from_file(self, filename, speed_factor=1.0, kp=0.5, control_freq=50):
        """
        Load a recorded actions JSON file and replay it on the connected arms.

        Args:
            filename: path to JSON file produced by DualArmActionRecorder.save_to_file
            speed_factor: replay speed multiplier
            kp: proportional gain for replay P-control
            control_freq: control frequency used during replay
        Returns:
            True on success, False on failure
        """
        try:
            # lazy import DualArmActionReplayer from robot_recorder_reproducer to avoid circular imports
            from robot_recorder_reproducer import DualArmActionReplayer
        except Exception:
            print("无法导入 DualArmActionReplayer (请确保 robot_recorder_reproducer.py 在同一目录且可导入)")
            return False

        if not any(self.robots.values()):
            print("❌ 无已连接的机械臂，无法复现动作")
            return False

        try:
            with open(filename, 'r') as f:
                data = json.load(f)

            actions = data.get('actions')
            if not actions:
                print(f"文件 {filename} 中没有动作数据")
                return False

            replayer = DualArmActionReplayer(self.robots, kp=kp, control_freq=control_freq)
            replayer.replay_actions(actions, speed_factor=speed_factor)
            return True
        except Exception as e:
            print(f"从文件复现失败: {e}")
            traceback.print_exc()
            return False
    
        
    def wave_hand(self):
        print("  -> 挥挥手")
        if not any(self.robots.values()):
            print("❌ 无已连接的机械臂，无法挥手")
            return
        if self.robots.get('arm1'):
            arm = 'arm1'
        elif self.robots.get('arm2'):
            arm = 'arm2'
        else:
            print("❌ 无已连接的机械臂，无法挥手")
            return
        # call replay_from_file with the filename only
        self.replay_from_file("/home/neethu/lerobot-0.3.3/mytest/dual_arm_actions_huishou.json")

    # ------------------------------------------------------------------
    # ADB helpers and dynamic face_* methods (so voice_assistant can call robot_controller.face_xxx)
    # ------------------------------------------------------------------
    def _adb_broadcast(self, action: str, extras: dict = None):
        extras = extras or {}

        def _run():
            cmd = ['adb', 'shell', 'am', 'broadcast', '-a', action]
            for k, v in extras.items():
                cmd.extend(['--es', k, str(v)])
            try:
                proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=5)
                if proc.returncode != 0:
                    stderr = proc.stderr.decode('utf-8', errors='ignore')
                    print(f"❌ adb broadcast failed: {stderr}")
                else:
                    out = proc.stdout.decode('utf-8', errors='ignore')
                    print(f"adb broadcast sent: action={action} extras={extras} -> {out.strip()}")
            except FileNotFoundError:
                print("❌ adb not found: ensure platform-tools are installed and adb is in PATH")
            except subprocess.TimeoutExpired:
                print("❌ adb broadcast timed out")
            except Exception as e:
                print(f"❌ adb broadcast error: {e}")

        t = threading.Thread(target=_run, daemon=True)
        t.start()

    def _set_emotion(self, name: str, duration_seconds: int = 10):
        if not name:
            return
        self._adb_broadcast('com.neethu.robotface.SET_EMOTION', extras={'emotion': name})

        # cancel previous timer
        try:
            if self._reset_timer and self._reset_timer.is_alive():
                self._reset_timer.cancel()
        except Exception:
            pass

        if duration_seconds and duration_seconds > 0:
            def _reset_later():
                self._adb_broadcast('com.neethu.robotface.RESET')

            self._reset_timer = threading.Timer(float(duration_seconds), _reset_later)
            self._reset_timer.daemon = True
            self._reset_timer.start()

    def _register_face_methods(self):
        base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'phone', 'app', 'src', 'main', 'res', 'raw'))
        if not os.path.isdir(base_dir):
            base_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'phone', 'app', 'src', 'main', 'res', 'raw'))
        if not os.path.isdir(base_dir):
            print(f"⚠️ RobotController: 未找到 raw 目录，跳过 face_ 方法注册：{base_dir}")
            return

        allowed_ext = {'.mp4', '.mov', '.m4v', '.webm', '.3gp'}
        for fname in os.listdir(base_dir):
            name, ext = os.path.splitext(fname)
            if ext.lower() in allowed_ext:
                method_name = f'face_{name}'
                if hasattr(self, method_name):
                    continue

                def _make_method(n):
                    def _method(duration_seconds: int = 10):
                        print(f"🖼️ RobotController: 切换表情到 '{n}'，持续 {duration_seconds}s")
                        self._set_emotion(n, duration_seconds)
                    return _method

                setattr(self, method_name, _make_method(name))
                print(f"✅ RobotController 注册方法: {method_name}() 对应文件: {fname}")