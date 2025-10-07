# 文件名: main.py (已修正并增强)
import os
import sys
import config
from robot_controller import RobotController
from voice_assistant import DuplexVoiceAssistant
from camera_controller import CameraController

def main():
    """
    程序主函数：组装并启动应用
    """
    os.environ['DASHSCOPE_LOGGING_LEVEL'] = 'WARNING'

    # --- 第1步：硬件初始化 ---
    print("--- 正在初始化硬件 ---")
    robot_controller = RobotController(port1=config.ROBOT_LEFT_ARM_PORT, port2=config.ROBOT_RIGHT_ARM_PORT)
    camera_controller = CameraController(camera_index=2)

    # --- 第2步：硬件连接检查 ---
    if not robot_controller.ua:
        print("\n‼️ 致命错误：机械臂未能成功初始化，程序无法继续。")
        print(f"   请检查：")
        print(f"   1. config.py 中的 ROBOT_LEFT_ARM_PORT ('{config.ROBOT_LEFT_ARM_PORT}') 是否正确。")
        print(f"   2. config.py 中的 ROBOT_RIGHT_ARM_PORT ('{config.ROBOT_RIGHT_ARM_PORT}') 是否正确。")
        print("   3. 机械臂设备是否已连接并通电。")
        camera_controller.cleanup()
        sys.exit(1)
    print("----------------------")

    # --- 第3步：软件与服务初始化 ---
    assistant = DuplexVoiceAssistant(
        robot_controller=robot_controller,
        camera_controller=camera_controller
    )

    # --- 第4步：启动应用 ---
    print("\n🎤 桌面机器人启动...")
    print("=" * 50)
    camera_controller.start_preview()
    
    # 启动主循环
    assistant.run()

    # --- 第5步：程序结束后的清理 ---
    print("\n--- 主程序即将退出，执行最终清理 ---")
    camera_controller.cleanup()
    robot_controller.back_zero()
    print("👋 程序已安全关闭。")


if __name__ == "__main__":
    main()