# 文件名: camera_controller.py
# 【更新】增加了实时预览窗口和更健壮的初始化。

import os
import base64
import sys
import threading
import time
import subprocess
from io import BytesIO
from PIL import Image
try:
    from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
    from lerobot.cameras.opencv.camera_opencv import OpenCVCamera
    from lerobot.cameras.configs import ColorMode, Cv2Rotation
    _HAS_LEROBOT_CAM = True
except Exception:
    _HAS_LEROBOT_CAM = False
try:
    import numpy as _np
    _HAS_NUMPY = True
except Exception:
    _HAS_NUMPY = False

class CameraController:
    """
    摄像头控制器，增加了实时预览功能。
    """
    def __init__(self, camera_index=0):
        print("📷 初始化摄像头...")
        self.mode = None
        self.device_path = f"/dev/video{camera_index}"

        # Try to use lerobot OpenCVCamera first (preferred)
        if _HAS_LEROBOT_CAM:
            try:
                cfg = OpenCVCameraConfig(
                    index_or_path=int(camera_index),
                    fps=15,
                    width=640,
                    height=480,
                    color_mode=ColorMode.RGB,
                    rotation=Cv2Rotation.NO_ROTATION
                )
                self.camera = OpenCVCamera(cfg)
                self.camera.connect()
                # warm-up read
                try:
                    _ = self.camera.async_read(timeout_ms=200)
                except Exception:
                    pass
                self.mode = 'opencv'
                print("✅ 使用 lerobot OpenCVCamera 初始化成功。")
                return
            except Exception as e:
                print(f"⚠️ 无法使用 OpenCVCamera: {e}")

        # Fallback: require ffmpeg and on-demand capture via subprocess
        print("ℹ️ 采用 ffmpeg 作为回退拍照方案（按需）。")
        if not os.path.exists(self.device_path):
            print(f"❌ 致命错误：找不到设备 {self.device_path}。")
            sys.exit(1)
        self.mode = 'ffmpeg'

    def _read_worker(self):
        # legacy continuous read worker removed; we capture on demand in capture_image_base64
        return

    def _preview_worker(self):
        # preview functionality removed for headless on-demand mode
        return

    def start_preview(self):
        """启动实时预览窗口"""
    print("⚠️ start_preview 已禁用（按需拍照模式）")

    def stop_preview(self):
        """停止实时预览"""
    print("⚠️ stop_preview 已禁用（按需拍照模式）")

    def capture_image_base64(self):
        """
        从摄像头捕获一帧图像，并将其编码为JPEG格式的Base64字符串。
        """
        if self.mode == 'opencv' and _HAS_LEROBOT_CAM:
            try:
                frame = self.camera.async_read(timeout_ms=1000)
                if frame is None:
                    print("❌ 拍照失败：OpenCVCamera 未返回帧")
                    return None
                if not _HAS_NUMPY:
                    print("❌ 无法处理帧：缺少 numpy，请安装 numpy 或使用 ffmpeg 回退。")
                    return None
                # frame is expected as numpy array (H,W,3) in RGB
                try:
                    img = Image.fromarray(frame)
                except Exception:
                    # try convert to uint8 array
                    arr = _np.array(frame, dtype=_np.uint8)
                    img = Image.fromarray(arr)

                buf = BytesIO()
                img.save(buf, format='JPEG')
                image_base64 = base64.b64encode(buf.getvalue()).decode('utf-8')
                print("📸 成功捕获并编码图像（OpenCVCamera）。")
                return image_base64
            except Exception as e:
                print(f"❌ 使用 OpenCVCamera 拍照失败: {e}")
                # fall through to ffmpeg fallback if available

        # Fallback to ffmpeg on-demand capture
        cmd = [
            'ffmpeg', '-hide_banner', '-loglevel', 'error',
            '-f', 'v4l2',
            '-video_size', '640x480',
            '-i', self.device_path,
            '-frames:v', '1',
            '-f', 'image2pipe', '-vcodec', 'mjpeg', '-'
        ]
        try:
            proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=6)
            if proc.returncode != 0 or not proc.stdout:
                stderr = proc.stderr.decode('utf-8', errors='ignore')
                print(f"❌ 拍照失败：ffmpeg 错误: {stderr}")
                return None
            jpeg = proc.stdout
        except FileNotFoundError:
            print("❌ 拍照失败：找不到 ffmpeg。")
            return None
        except subprocess.TimeoutExpired:
            print("❌ 拍照失败：ffmpeg 超时")
            return None
        except Exception as e:
            print(f"❌ 拍照失败：{e}")
            return None

        try:
            image = Image.open(BytesIO(jpeg))
            buf = BytesIO()
            image.save(buf, format='JPEG')
            image_bytes = buf.getvalue()
            image_base64 = base64.b64encode(image_bytes).decode('utf-8')
            print("📸 成功捕获并编码图像（ffmpeg 回退）。")
            return image_base64
        except Exception as e:
            print(f"❌ 拍照失败：无法处理 JPEG 帧：{e}")
            return None

    def cleanup(self):
        """
        释放摄像头资源。
        """
        print("🧹 正在关闭摄像头...")
        self.stop_preview()
        if getattr(self, 'proc', None):
            try:
                self.proc.terminate()
            except Exception:
                pass
            try:
                self.proc.wait(timeout=1)
            except Exception:
                pass
        print("✅ 摄像头资源已释放。")