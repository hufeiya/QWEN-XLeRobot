# 文件名: config.py

# 阿里云百炼 API 配置
# 替换成你自己的
APP_ID = ""
API_KEY = ""
WORKSPACE_ID = ""
WEBSOCKET_URL = "wss://dashscope.aliyuncs.com/api-ws/v1/inference"
MODEL_NAME = "multimodal-dialog"

# 语音配置
VOICE_NAME = "longxiaochun_v2"
SAMPLE_RATE = 16000
SAMPLE_WIDTH = 2
CHANNELS = 1
AUDIO_FRAME_SIZE = 640

# 机械臂配置
ROBOT_LEFT_ARM_PORT = "/dev/ttyACM0"
ROBOT_RIGHT_ARM_PORT = "/dev/ttyACM1"
ROBOT_ARM_BAUDRATE = 115200
