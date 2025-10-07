#!/usr/bin/env python3
"""
LeRobot 双臂机械臂动作录制和复现系统 - 终端输入版本
支持同时录制和复现两个机械臂的动作
"""

import time
import logging
import traceback
import json
import math
from datetime import datetime
import os
import threading

# 设置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 关节校准系数
JOINT_CALIBRATION = [
    ['shoulder_pan', 6.0, 1.0],
    ['shoulder_lift', 2.0, 0.97],
    ['elbow_flex', 0.0, 1.05],
    ['wrist_flex', 0.0, 0.94],
    ['wrist_roll', 0.0, 0.5],
    ['gripper', 0.0, 1.0],
]

class DualArmActionRecorder:
    """双臂动作录制器"""
    
    def __init__(self, robots):
        self.robots = robots  # {'arm1': robot1, 'arm2': robot2}
        self.recorded_actions = []
        self.is_recording = False
        self.start_time = None
        self.recording_freq = 10  # 录制频率 10Hz
        self.recording_thread = None
        
    def start_recording(self):
        """开始录制"""
        self.recorded_actions = []
        self.is_recording = True
        self.start_time = time.time()
        print("开始录制双臂机械臂动作...")
        print("请手动移动两个机械臂到不同位置")
        print("录制中...按回车键停止录制")
        
        # 启动录制线程
        self.recording_thread = threading.Thread(target=self._recording_loop)
        self.recording_thread.daemon = True
        self.recording_thread.start()
    
    def stop_recording(self):
        """停止录制"""
        self.is_recording = False
        if self.recording_thread:
            self.recording_thread.join()
        duration = time.time() - self.start_time
        print(f"\n录制完成! 共录制 {len(self.recorded_actions)} 个动作点，时长: {duration:.2f}秒")
    
    def _recording_loop(self):
        """录制循环"""
        last_record_time = 0
        record_interval = 1.0 / self.recording_freq
        
        while self.is_recording:
            current_time = time.time()
            if current_time - last_record_time >= record_interval:
                # 获取两个机械臂的当前位置
                arm1_positions = self._get_current_joint_positions('arm1')
                arm2_positions = self._get_current_joint_positions('arm2')
                timestamp = current_time - self.start_time
                
                action_point = {
                    'timestamp': timestamp,
                    'arm1': arm1_positions,
                    'arm2': arm2_positions,
                    'time_since_start': timestamp
                }
                self.recorded_actions.append(action_point)
                
                # 显示当前录制状态
                print(f"已录制 {len(self.recorded_actions)} 个点，当前时间: {timestamp:.1f}s", end='\r')
                
                last_record_time = current_time
            
            time.sleep(0.01)  # 短暂休眠避免过高CPU占用
    
    def _get_current_joint_positions(self, arm_name):
        """获取指定机械臂的当前关节位置"""
        robot = self.robots[arm_name]
        current_obs = robot.get_observation()
        joint_positions = {}
        
        for key, value in current_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                # 应用校准系数
                calibrated_value = apply_joint_calibration(motor_name, value)
                joint_positions[motor_name] = calibrated_value
        
        return joint_positions
    
    def save_to_file(self, filename=None):
        """保存录制的动作到文件"""
        if not self.recorded_actions:
            print("没有录制的动作可保存")
            return None
        
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"dual_arm_actions_{timestamp}.json"
        
        data = {
            'metadata': {
                'recorded_at': datetime.now().isoformat(),
                'total_points': len(self.recorded_actions),
                'total_duration': self.recorded_actions[-1]['timestamp'] if self.recorded_actions else 0,
                'recording_freq': self.recording_freq,
                'arm1_joint_names': list(self.recorded_actions[0]['arm1'].keys()) if self.recorded_actions else [],
                'arm2_joint_names': list(self.recorded_actions[0]['arm2'].keys()) if self.recorded_actions else []
            },
            'actions': self.recorded_actions
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"双臂动作已保存到文件: {filename}")
        return filename
    
    def load_from_file(self, filename):
        """从文件加载录制的动作"""
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            self.recorded_actions = data['actions']
            print(f"从文件 {filename} 加载了 {len(self.recorded_actions)} 个动作点")
            return True
        except Exception as e:
            print(f"加载文件失败: {e}")
            return False

class DualArmActionReplayer:
    """双臂动作复现器"""
    
    def __init__(self, robots, kp=0.5, control_freq=50):
        self.robots = robots
        self.kp = kp
        self.control_freq = control_freq
        self.control_period = 1.0 / control_freq
    
    def replay_actions(self, recorded_actions, speed_factor=1.0):
        """
        复现录制的双臂动作
        
        Args:
            recorded_actions: 录制的动作列表
            speed_factor: 速度因子 (1.0=正常速度, 2.0=2倍速, 0.5=半速)
        """
        if not recorded_actions:
            print("没有动作可复现")
            return
        
        print(f"开始复现 {len(recorded_actions)} 个动作点，速度因子: {speed_factor}")
        print("按 Ctrl+C 可中断复现")
        
        try:
            start_time = time.time()
            total_duration = recorded_actions[-1]['timestamp'] / speed_factor
            
            # 创建时间索引的动作映射
            action_timeline = []
            for action in recorded_actions:
                action_timeline.append({
                    'time': action['timestamp'] / speed_factor,
                    'arm1_targets': action['arm1'],
                    'arm2_targets': action['arm2']
                })
            
            current_action_index = 0
            last_print_time = time.time()
            
            while current_action_index < len(action_timeline):
                current_time = time.time() - start_time
                
                # 找到当前时间对应的目标动作
                while (current_action_index < len(action_timeline) and 
                       action_timeline[current_action_index]['time'] <= current_time):
                    current_action_index += 1
                
                if current_action_index == 0:
                    # 还没有到第一个动作点
                    arm1_targets = action_timeline[0]['arm1_targets']
                    arm2_targets = action_timeline[0]['arm2_targets']
                elif current_action_index >= len(action_timeline):
                    # 所有动作点都已执行，保持最后一个位置
                    arm1_targets = action_timeline[-1]['arm1_targets']
                    arm2_targets = action_timeline[-1]['arm2_targets']
                else:
                    # 在两个动作点之间进行插值
                    prev_action = action_timeline[current_action_index - 1]
                    next_action = action_timeline[current_action_index]
                    
                    time_ratio = ((current_time - prev_action['time']) / 
                                 (next_action['time'] - prev_action['time']))
                    time_ratio = max(0, min(1, time_ratio))  # 限制在0-1之间
                    
                    # 线性插值计算目标位置
                    arm1_targets = {}
                    arm2_targets = {}
                    
                    for joint_name in prev_action['arm1_targets'].keys():
                        prev_pos = prev_action['arm1_targets'][joint_name]
                        next_pos = next_action['arm1_targets'][joint_name]
                        arm1_targets[joint_name] = prev_pos + (next_pos - prev_pos) * time_ratio
                    
                    for joint_name in prev_action['arm2_targets'].keys():
                        prev_pos = prev_action['arm2_targets'][joint_name]
                        next_pos = next_action['arm2_targets'][joint_name]
                        arm2_targets[joint_name] = prev_pos + (next_pos - prev_pos) * time_ratio
                
                # 执行P控制移动到目标位置
                self._move_to_positions('arm1', arm1_targets)
                self._move_to_positions('arm2', arm2_targets)
                
                # 显示进度
                if time.time() - last_print_time > 0.5:  # 每0.5秒显示一次进度
                    progress = (current_time / total_duration) * 100
                    print(f"复现进度: {progress:.1f}% ({current_time:.1f}s / {total_duration:.1f}s)", end='\r')
                    last_print_time = time.time()
                
                time.sleep(self.control_period)
            
            print("\n双臂动作复现完成!")
            
        except KeyboardInterrupt:
            print("\n复现被用户中断")
    
    def _move_to_positions(self, arm_name, target_positions):
        """使用P控制移动到目标位置"""
        robot = self.robots[arm_name]
        
        # 获取当前机器人状态
        current_obs = robot.get_observation()
        current_positions = {}
        for key, value in current_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                current_positions[motor_name] = value
        
        # P控制计算
        robot_action = {}
        for joint_name, target_pos in target_positions.items():
            if joint_name in current_positions:
                current_pos = current_positions[joint_name]
                error = target_pos - current_pos
                
                # P控制: output = Kp * error
                control_output = self.kp * error
                
                # 转换控制输出为位置命令
                new_position = current_pos + control_output
                robot_action[f"{joint_name}.pos"] = new_position
        
        # 发送动作到机器人
        if robot_action:
            robot.send_action(robot_action)

def apply_joint_calibration(joint_name, raw_position):
    """应用关节校准系数"""
    for joint_cal in JOINT_CALIBRATION:
        if joint_cal[0] == joint_name:
            offset = joint_cal[1]
            scale = joint_cal[2]
            calibrated_position = (raw_position - offset) * scale
            return calibrated_position
    return raw_position


def replay_from_file(filename, robots, speed_factor=1.0, kp=0.5, control_freq=50):
    """
    Load a recorded actions JSON file and replay it on the provided robots.

    Args:
        filename: path to the recorded JSON file (as produced by DualArmActionRecorder.save_to_file)
        robots: dict {'arm1': robot1, 'arm2': robot2}
        speed_factor: replay speed multiplier (1.0 = normal)
        kp: proportional gain used by replayer
        control_freq: control frequency for replayer
    Returns:
        True on success, False otherwise
    """
    try:
        with open(filename, 'r') as f:
            data = json.load(f)

        actions = data.get('actions')
        if not actions:
            print(f"文件 {filename} 中没有动作数据")
            return False

        replayer = DualArmActionReplayer(robots, kp=kp, control_freq=control_freq)
        replayer.replay_actions(actions, speed_factor=speed_factor)
        return True
    except Exception as e:
        print(f"从文件复现失败: {e}")
        traceback.print_exc()
        return False


def move_to_zero_position(robots, duration=3.0, kp=0.5):
    """移动到零位置"""
    print("使用P控制缓慢移动到零位置...")
    
    # 获取当前状态
    current_obs = {}
    for arm_name, robot in robots.items():
        current_obs[arm_name] = robot.get_observation()
    
    # 零位置目标
    zero_positions = {
        'shoulder_pan': 0.0,
        'shoulder_lift': 0.0,
        'elbow_flex': 0.0,
        'wrist_flex': 0.0,
        'wrist_roll': 0.0,
        'gripper': 0.0
    }
    
    control_freq = 50
    total_steps = int(duration * control_freq)
    step_time = 1.0 / control_freq
    
    for step in range(total_steps):
        # 获取当前状态
        current_obs = {}
        for arm_name, robot in robots.items():
            current_obs[arm_name] = robot.get_observation()
        
        # 对每个机械臂执行P控制
        for arm_name, robot in robots.items():
            robot_action = {}
            for joint_name, target_pos in zero_positions.items():
                if f"{joint_name}.pos" in current_obs[arm_name]:
                    current_pos = current_obs[arm_name][f"{joint_name}.pos"]
                    calibrated_pos = apply_joint_calibration(joint_name, current_pos)
                    error = target_pos - calibrated_pos
                    control_output = kp * error
                    new_position = calibrated_pos + control_output
                    robot_action[f"{joint_name}.pos"] = new_position
            
            if robot_action:
                robot.send_action(robot_action)
        
        if step % (control_freq // 2) == 0:
            progress = (step / total_steps) * 100
            print(f"移动到零位置进度: {progress:.1f}%")
        
        time.sleep(step_time)
    
    print("双臂机器人已移动到零位置")

def get_current_joint_positions(robots):
    """获取当前关节位置并显示"""
    print("\n当前关节位置:")
    
    for arm_name, robot in robots.items():
        current_obs = robot.get_observation()
        print(f"{arm_name}:")
        
        for key, value in current_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                calibrated_value = apply_joint_calibration(motor_name, value)
                print(f"  {motor_name}: {calibrated_value:.1f}°")
        
        print()

def manual_position_control(robots):
    """双臂手动位置控制模式"""
    print("\n双臂手动位置控制模式")
    print("输入格式: 机械臂名称 关节名称 角度")
    print("例如: arm1 shoulder_pan 30")
    print("输入 'show' 显示当前位置")
    print("输入 'back' 返回主菜单")
    
    # 初始化目标位置为当前位置
    target_positions = {
        'arm1': {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
        },
        'arm2': {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
        }
    }
    
    # 获取初始位置
    for arm_name, robot in robots.items():
        current_obs = robot.get_observation()
        for key, value in current_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                calibrated_value = apply_joint_calibration(motor_name, value)
                target_positions[arm_name][motor_name] = calibrated_value
    
    control_freq = 50
    control_period = 1.0 / control_freq
    
    try:
        while True:
            # 对每个机械臂执行P控制
            for arm_name, robot in robots.items():
                current_obs = robot.get_observation()
                current_positions = {}
                for key, value in current_obs.items():
                    if key.endswith('.pos'):
                        motor_name = key.removesuffix('.pos')
                        current_positions[motor_name] = value
                
                robot_action = {}
                for joint_name, target_pos in target_positions[arm_name].items():
                    if joint_name in current_positions:
                        current_pos = current_positions[joint_name]
                        error = target_pos - current_pos
                        control_output = 0.5 * error
                        new_position = current_pos + control_output
                        robot_action[f"{joint_name}.pos"] = new_position
                
                if robot_action:
                    robot.send_action(robot_action)
            
            # 检查用户输入
            user_input = input_non_blocking()
            if user_input:
                user_input = user_input.strip().lower()
                
                if user_input == 'back':
                    break
                elif user_input == 'show':
                    get_current_joint_positions(robots)
                else:
                    parts = user_input.split()
                    if len(parts) == 3:
                        arm_name = parts[0]
                        joint_name = parts[1]
                        try:
                            angle = float(parts[2])
                            if arm_name in target_positions and joint_name in target_positions[arm_name]:
                                target_positions[arm_name][joint_name] = angle
                                print(f"设置 {arm_name} {joint_name} 目标位置: {angle}°")
                            else:
                                print(f"无效的机械臂名称或关节名称: {arm_name} {joint_name}")
                                print("可用机械臂: arm1, arm2")
                                print("可用关节: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper")
                        except ValueError:
                            print("请输入有效的角度数值")
                    else:
                        print("请输入 '机械臂名称 关节名称 角度' 或 'show' 或 'back'")
            
            time.sleep(control_period)
            
    except KeyboardInterrupt:
        print("\n退出手动控制模式")

def input_non_blocking():
    """非阻塞输入检测"""
    import sys
    import select
    
    if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.readline().strip()
    return None

def inverse_kinematics(x, y, l1=0.1159, l2=0.1350):
    """逆运动学计算"""
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
    theta2 = math.pi - math.acos(cos_theta2)
    
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

def main():
    """主函数"""
    print("LeRobot 双臂机械臂动作录制和复现系统 - 终端输入版本")
    print("="*60)
    
    try:
        # 导入必要模块
        from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
        
        # 配置双臂机器人
        print("配置双臂机器人...")
        
        # 获取端口
        arm1_port = input("请输入第一个机械臂USB端口 (默认 /dev/ttyACM0): ").strip()
        if not arm1_port:
            arm1_port = "/dev/ttyACM0"
        
        arm2_port = input("请输入第二个机械臂USB端口 (默认 /dev/ttyACM1): ").strip()
        if not arm2_port:
            arm2_port = "/dev/ttyACM1"
        
        print(f"第一个机械臂端口: {arm1_port}")
        print(f"第二个机械臂端口: {arm2_port}")
        
        # 创建机器人实例
        arm1_config = SO100FollowerConfig(port=arm1_port)
        arm2_config = SO100FollowerConfig(port=arm2_port)
        
        arm1_robot = SO100Follower(arm1_config)
        arm2_robot = SO100Follower(arm2_config)
        
        robots = {
            'arm1': arm1_robot,
            'arm2': arm2_robot
        }
        
        # 初始化录制器和复现器
        recorder = DualArmActionRecorder(robots)
        replayer = DualArmActionReplayer(robots)
        
        # 连接设备
        print("连接第一个机械臂...")
        arm1_robot.connect()
        print("连接第二个机械臂...")
        arm2_robot.connect()
        print("双臂设备连接成功!")
        
        # 询问是否重新校准
        for arm_name, robot in robots.items():
            while True:
                calibrate_choice = input(f"是否重新校准{arm_name}? (y/n): ").strip().lower()
                if calibrate_choice in ['y', 'yes']:
                    print(f"开始重新校准{arm_name}...")
                    robot.calibrate()
                    print(f"{arm_name}校准完成!")
                    break
                elif calibrate_choice in ['n', 'no']:
                    print(f"使用{arm_name}之前的校准文件")
                    break
                else:
                    print("请输入 y 或 n")
        
        # 移动到零位置
        move_to_zero_position(robots)
        
        # 主菜单
        while True:
            print("\n" + "="*50)
            print("请选择模式:")
            print("1. 手动移动录制模式 (双臂同时)")
            print("2. 手动位置控制模式 (双臂)")
            print("3. 复现已录制的动作 (双臂)")
            print("4. 显示当前关节位置 (双臂)")
            print("5. 移动到零位置 (双臂)")
            print("6. 退出")
            
            choice = input("请输入选择 (1-6): ").strip()
            
            if choice == '1':
                # 手动移动录制模式
                print("\n双臂手动移动录制模式")
                print("请确保两个机械臂扭矩已禁用，可以手动移动")
                
                try:
                    # 禁用两个机械臂的扭矩
                    for arm_name, robot in robots.items():
                        robot.bus.disable_torque()
                        print(f"{arm_name}机械臂扭矩已禁用")
                    
                    input("按回车键开始录制...")
                    recorder.start_recording()
                    
                    # 等待用户按回车停止录制
                    input()
                    recorder.stop_recording()
                    
                    # 重新使能扭矩
                    for arm_name, robot in robots.items():
                        robot.bus.enable_torque()
                        print(f"{arm_name}机械臂扭矩已重新使能")
                    
                    # 询问是否保存
                    save_choice = input("是否保存录制的动作? (y/n): ").strip().lower()
                    if save_choice in ['y', 'yes']:
                        filename = input("输入文件名 (回车使用默认名称): ").strip()
                        if not filename:
                            recorder.save_to_file()
                        else:
                            if not filename.endswith('.json'):
                                filename += '.json'
                            recorder.save_to_file(filename)
                
                except Exception as e:
                    print(f"录制过程中出现错误: {e}")
                    # 确保扭矩被重新使能
                    for arm_name, robot in robots.items():
                        try:
                            robot.bus.enable_torque()
                        except:
                            pass
            
            elif choice == '2':
                # 手动位置控制模式
                manual_position_control(robots)
            
            elif choice == '3':
                # 复现模式
                # 列出可用的动作文件
                json_files = [f for f in os.listdir('.') if f.endswith('.json') and f.startswith('dual_arm_actions_')]
                if not json_files:
                    print("没有找到录制的双臂动作文件")
                    continue
                
                print("\n可用的双臂动作文件:")
                for i, filename in enumerate(json_files):
                    print(f"{i+1}. {filename}")
                
                file_choice = input("请选择文件编号或输入文件名: ").strip()
                try:
                    if file_choice.isdigit():
                        file_index = int(file_choice) - 1
                        if 0 <= file_index < len(json_files):
                            filename = json_files[file_index]
                        else:
                            print("无效的选择")
                            continue
                    else:
                        filename = file_choice
                        if not filename.endswith('.json'):
                            filename += '.json'
                    
                    # 加载动作文件
                    if recorder.load_from_file(filename):
                        # 询问复现速度
                        speed_input = input("请输入复现速度因子 (默认 1.0): ").strip()
                        speed_factor = float(speed_input) if speed_input else 1.0
                        
                        # 开始复现
                        replayer.replay_actions(recorder.recorded_actions, speed_factor)
                
                except Exception as e:
                    print(f"复现失败: {e}")
                    traceback.print_exc()
            
            elif choice == '4':
                # 显示当前关节位置
                get_current_joint_positions(robots)
            
            elif choice == '5':
                # 移动到零位置
                move_to_zero_position(robots)
            
            elif choice == '6':
                print("退出程序")
                break
            
            else:
                print("无效的选择，请重新输入")
        
        # 断开连接
        for arm_name, robot in robots.items():
            print(f"断开{arm_name}连接...")
            robot.disconnect()
        print("程序结束")
        
    except Exception as e:
        print(f"程序执行失败: {e}")
        traceback.print_exc()
        print("请检查:")
        print("1. 机器人是否正确连接")
        print("2. USB端口是否正确")
        print("3. 是否有足够的权限访问USB设备")
        print("4. 机器人是否正确配置")

if __name__ == "__main__":
    main()
