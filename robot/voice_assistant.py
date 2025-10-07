# 文件名: voice_assistant.py (已修正并解决写字打断和自我对话问题)
import sys
import os
import time
import threading
import queue
import json
import inspect
import re
import random

try:
    import keyboard as _keyboard
except Exception:
    _keyboard = None
try:
    import pyaudio
except ImportError:
    print("❌ PyAudio 库未安装，请先执行：pip install PyAudio")
    sys.exit(1)

from dashscope import MultiModalConversation
from dashscope.multimodal.dialog_state import DialogState
from dashscope.multimodal.multimodal_dialog import MultiModalDialog, MultiModalCallback
from dashscope.multimodal.multimodal_request_params import (
    Upstream, Downstream, ClientInfo, RequestParameters, Device,
    RequestToRespondParameters
)
import config

class AudioPlayer:
    def __init__(self, callback_handler):
        self.pyaudio_instance = pyaudio.PyAudio()
        self.stream = None
        self.audio_queue = queue.Queue()
        self.is_playing = True
        # event callback called when playback finishes (queue drains)
        self.on_finished = None
        # internal flag: set to True when audio has been queued/playing
        self._was_playing = False
        # timestamp of last enqueued audio (seconds since epoch)
        self._last_enqueue_time = 0.0
        # lock to protect _was_playing and _last_enqueue_time
        self._play_lock = threading.Lock()
        # idle threshold (seconds) to wait before firing on_finished
        self._idle_finish_seconds = 1.0
        self.play_thread = threading.Thread(target=self._play_worker, daemon=True)
        self._init_stream()
        self.play_thread.start()

    def _init_stream(self):
        if self.stream:
            try: self.stream.stop_stream(); self.stream.close()
            except: pass
        self.stream = self.pyaudio_instance.open(
            format=self.pyaudio_instance.get_format_from_width(config.SAMPLE_WIDTH),
            channels=config.CHANNELS, rate=config.SAMPLE_RATE, output=True, frames_per_buffer=1024
        )

    def _play_worker(self):
        while self.is_playing:
            try:
                data = self.audio_queue.get(timeout=0.1)
                if data and self.stream: self.stream.write(data)
            except queue.Empty: continue
            except Exception as e: print(f"❌ 播放音频错误：{e}"); self._init_stream()
            # if queue becomes empty after having played audio, wait for idle period before firing on_finished
            try:
                with self._play_lock:
                    was_playing = self._was_playing
                    last_time = self._last_enqueue_time
                if was_playing and self.audio_queue.empty():
                    # check idle time
                    if time.time() - last_time >= self._idle_finish_seconds:
                        print("🎤 音频播放队列已清空且空闲阈值满足，触发 finished 回调。")
                        with self._play_lock:
                            self._was_playing = False
                        if callable(self.on_finished):
                            try:
                                self.on_finished()
                            except Exception:
                                pass
            except Exception:
                pass

    def add_audio(self, data):
        self.audio_queue.put(data)
        # mark that audio is pending/playing and update last enqueue time
        with self._play_lock:
            self._was_playing = True
            self._last_enqueue_time = time.time()

    def clear_queue(self):
        with self.audio_queue.mutex: self.audio_queue.queue.clear()

    def is_busy(self):
        return not self.audio_queue.empty()

    def stop(self):
        self.is_playing = False
        if self.play_thread: self.play_thread.join(timeout=1)
        if self.stream:
            try: self.stream.stop_stream(); self.stream.close()
            except: pass
        self.pyaudio_instance.terminate()
        # notify finished when explicitly stopped
        try:
            if callable(self.on_finished):
                self.on_finished()
        except Exception:
            pass


# TerminalKeyListener: fallback for headless SSH terminals when 'keyboard' is not usable
class TerminalKeyListener:
    def __init__(self):
        import termios, tty, select
        self.termios = termios
        self.tty = tty
        self.select = select
        self.enabled = sys.stdin.isatty()
        self.lock = threading.Lock()
        self.pressed = set()
        self.running = False
        self.orig_settings = None
        if self.enabled:
            try:
                self.orig_settings = termios.tcgetattr(sys.stdin)
                tty.setcbreak(sys.stdin.fileno())
                self.running = True
                self.thread = threading.Thread(target=self._reader, daemon=True)
                self.thread.start()
            except Exception:
                self.enabled = False

    def _reader(self):
        while self.running:
            try:
                dr, _, _ = self.select.select([sys.stdin], [], [], 0.1)
                if dr:
                    ch = sys.stdin.read(1)
                    with self.lock:
                        self.pressed.add(ch)
                else:
                    time.sleep(0.01)
            except Exception:
                time.sleep(0.05)

    def is_pressed(self, key_name):
        with self.lock:
            if key_name == 'space':
                if ' ' in self.pressed:
                    self.pressed.remove(' ')
                    return True
            if key_name == 'esc':
                if '\x1b' in self.pressed:
                    self.pressed.remove('\x1b')
                    return True
        return False

    def stop(self):
        self.running = False
        try:
            if self.orig_settings:
                self.termios.tcsetattr(sys.stdin, self.termios.TCSADRAIN, self.orig_settings)
        except Exception:
            pass


# prepare keyboard or terminal fallback
terminal_key_listener = None
keyboard = None
if _keyboard is not None:
    try:
        # try a harmless call to verify permissions (may raise on linux if not root)
        _ = _keyboard.is_pressed('space')
        keyboard = _keyboard
    except Exception:
        keyboard = None

if keyboard is None:
    try:
        terminal_key_listener = TerminalKeyListener()
    except Exception:
        terminal_key_listener = None

def check_key_pressed(name):
    """Check if a key is pressed using keyboard lib or terminal fallback."""
    if keyboard:
        try:
            return keyboard.is_pressed(name)
        except Exception:
            return False
    if terminal_key_listener and terminal_key_listener.enabled:
        return terminal_key_listener.is_pressed(name)
    return False

class DuplexCallback(MultiModalCallback):
    def __init__(self, manager, robot_controller, camera_controller):
        self.manager = manager
        self.robot_controller = robot_controller
        self.camera_controller = camera_controller
        self.audio_player = AudioPlayer(self)
        # event-driven: register on_finished handler so when audio finishes, set manager state to IDLE
        try:
            self.audio_player.on_finished = self._on_audio_finished
        except Exception:
            pass
        self.is_responding = False
        self.response_text = ""
        self.last_user_text = ""
        self.last_ai_response = ""
        self.is_sending_silence = False
        self.silence_thread = None
        self.vqa_context_active = False
        self.last_vqa_response = None
        self.numeric_commands = [
            "move_forward", "move_back", "move_left", "move_right", 
            "move_up", "move_down"
        ]
        self.skill_map = {
            "move_forward": self.robot_controller.move_forward,
            "move_back": self.robot_controller.move_back,
            "move_left": self.robot_controller.move_left,
            "move_right": self.robot_controller.move_right,
            "back_zero": self.robot_controller.back_zero,
            "move_up": self.robot_controller.move_up,
            "move_down": self.robot_controller.move_down,
            "wave_hand": self.robot_controller.wave_hand,
            "face_default_face": self.robot_controller.face_default_face,
            "face_flustered": self.robot_controller.face_flustered,
            "face_happy_moon_eye": self.robot_controller.face_happy_moon_eye,
            "face_sad_tear": self.robot_controller.face_sad_tear,
            "face_scared": self.robot_controller.face_scared,
        }

    def _send_silence_worker(self):
        bytes_per_frame = config.CHANNELS * config.SAMPLE_WIDTH
        silence_packet = b'\x00' * (config.AUDIO_FRAME_SIZE * bytes_per_frame)
        print("❤️ [HB] 开始发送静音心跳包以保持连接...")
        while self.is_sending_silence:
            try:
                self.manager.convo.send_audio_data(silence_packet)
                time.sleep(0.04)
            except Exception as e:
                print(f"❌ [HB] 发送静音时出错: {e}"); break
        print("💔 [HB] 静音心跳包已停止。")

    def _audio_watcher(self):
        """Background thread: watch audio_player.is_playing and set manager.current_state to IDLE when playback stops."""
        last_playing = True
        while not getattr(self, '_audio_watcher_stop').is_set():
            try:
                current_playing = False
                if self.audio_player:
                    current_playing = self.audio_player.is_playing
                # detect transition from playing -> stopped
                if last_playing and not current_playing:
                    try:
                        if self.manager:
                            # set manager state to IDLE
                            print("🎤 音频播放已结束，进入空闲状态1。")
                            self.manager.current_state = DialogState.IDLE
                            # if there's an idle event, notify
                            if hasattr(self.manager, 'idle_speak_complete'):
                                try:
                                    self.manager.idle_speak_complete.set()
                                except Exception:
                                    pass
                    except Exception:
                        pass
                last_playing = current_playing
            except Exception:
                pass
            # poll interval
            time.sleep(0.2)

    def _on_audio_finished(self):
        """Called when AudioPlayer signals playback finished (queue drained or stopped)."""
        try:
            if self.manager:
                print("🎤 音频播放已结束，进入空闲状态2。")
                self.manager.current_state = DialogState.IDLE
                if hasattr(self.manager, 'idle_speak_complete'):
                    try:
                        self.manager.idle_speak_complete.set()
                    except Exception:
                        pass
        except Exception:
            pass

    def _handle_vqa_request(self):
        self.vqa_context_active = True
        print("📝 [CTX] VQA 上下文已激活。")
        self.is_sending_silence = True
        self.silence_thread = threading.Thread(target=self._send_silence_worker)
        self.silence_thread.daemon = True
        self.silence_thread.start()
        print("📸 [VQA] 流程启动，正在拍照...")
        image_b64 = self.camera_controller.capture_image_base64()
        
        if image_b64:
            self.is_sending_silence = False
            if self.silence_thread: self.silence_thread.join(timeout=0.5)
            vqa_prompt = self.last_user_text
            print(f"📸 [VQA] 发送图文请求，使用用户的原始问题: '{vqa_prompt}'")
            image_param = {"type": "base64", "value": image_b64}
            vqa_params = RequestToRespondParameters(images=[image_param])
            self.manager.convo.request_to_respond("prompt", vqa_prompt, parameters=vqa_params)
        else:
            self.is_sending_silence = False
            if self.silence_thread: self.silence_thread.join(timeout=0.5)
            print("❌ [VQA] 图像捕获失败，VQA流程取消。")
            self.vqa_context_active = False

    def _get_param_value(self, params_list, param_name):
        for param in params_list:
            if param.get("name") == param_name:
                return param.get("normValue")
        return None

    def _extract_number(self, text_with_units):
        if isinstance(text_with_units, (int, float)): return text_with_units
        if not isinstance(text_with_units, str): return None
        numbers = re.findall(r'\d+\.?\d*|\d+', text_with_units)
        if numbers: return float(numbers[0])
        return None
    
    def on_responding_content(self, payload):
        if not payload or "output" not in payload: return
        output = payload.get("output", {})
        text_chunk = output.get("text", "")
        match = re.search(r"robot_write\(text=(['\"])(.*?)\1\)", text_chunk)
        if match:
            text_to_write = match.group(2)
            print(f"🦾 [ROBOT] (从文本中捕获) 分发指令 [robot_write] (参数: '{text_to_write}')")
            self.robot_controller.stop_current_task()
            self.manager.dispatch_robot_task(self.robot_controller.robot_write, args=(text_to_write,), is_long_task=True)
            return

        extra_info = output.get("extra_info", {})
        commands_str = extra_info.get("commands")
        if commands_str:
            try:
                commands_list = json.loads(commands_str)
                for command in commands_list:
                    self.robot_controller.stop_current_task()
                    time.sleep(0.1) 
                    command_name = command.get("name")
                    if command_name == "visual_qa":
                        print("‼️ [CMD] 检测到 visual_qa 指令！")
                        vqa_thread = threading.Thread(target=self._handle_vqa_request)
                        vqa_thread.daemon = True
                        vqa_thread.start()
                        return
                        
                    if command_name in self.skill_map:
                        skill_func = self.skill_map[command_name]
                        params = command.get("params", [])
                        args_for_thread = ()
                        final_param = None
                        sig = inspect.signature(skill_func)
                        if len(sig.parameters) > 0:
                            param_name = list(sig.parameters.keys())[0]
                            original_param_value = self._get_param_value(params, param_name)
                            if command_name in self.numeric_commands:
                                final_param = self._extract_number(original_param_value)
                                log_msg = f"\n🦾 [ROBOT] 分发指令 [{command_name}] (净化后: {final_param})"
                                if final_param is None:
                                    print(f" (无法从参数 '{original_param_value}' 中提取有效数字，跳过执行)")
                                    return
                            else:
                                final_param = original_param_value
                                log_msg = f"\n🦾 [ROBOT] 分发指令 [{command_name}] (参数: '{final_param}')"
                            args_for_thread = (final_param,)
                        else:
                             log_msg = f"\n🦾 [ROBOT] 分发指令 [{command_name}] (无参数)"
                        print(log_msg)
                        is_long_task = (command_name == "robot_write")
                        self.manager.dispatch_robot_task(
                            target_func=skill_func, 
                            args=args_for_thread,
                            is_long_task=is_long_task
                        )
                        return
            except Exception as e: print(f"❌ 指令分发或执行失败: {e}")

        if not self.is_responding and not output.get("finished", False): self.response_text = ""
        self.response_text += text_chunk
        if output.get("finished", False) and self.response_text:
            print(f"\n🤖 AI回复：{self.response_text}")
            self.last_ai_response = self.response_text
            if self.vqa_context_active:
                self.last_vqa_response = self.response_text
                print(f"📝 [CTX] VQA 响应已捕获，可用于后续写入。")

    def on_speech_content(self, payload):
        if self.manager.is_in_autonomous_speech:
            return

        if payload and payload.get("output"):
            output = payload["output"]
            text = output.get("text", "")
            finished = output.get("finished", False)
            if text and not finished:
                print(f"\r🎤 识别中：{text}", end="", flush=True)
            
            if finished:
                print(f"\n🎤 用户说：{text}")
                self.last_user_text = text
                move_keywords = ["前", "后", "左", "右", "上", "下", "移动", "前进", "后退"]
                is_move_command = any(keyword in text for keyword in move_keywords)

                if not is_move_command:
                    self.robot_controller.start_thinking_animation()
                else:
                    print("⚙️ 检测到移动指令，跳过思考动画以确保相对定位。")

                write_keywords = ["写", "write", "记下来", "记录", "把它写"]
                is_write_command = any(keyword in text for keyword in write_keywords)

                if self.vqa_context_active and self.last_vqa_response and is_write_command:
                    print("✍️ [CTX] 检测到VQA后的写入指令...")
                    self.robot_controller.stop_current_task()
                    text_to_write = self.last_vqa_response
                    self.manager.dispatch_robot_task(self.robot_controller.robot_write, args=(text_to_write,), is_long_task=True)
                    self.vqa_context_active = False
                    self.last_vqa_response = None
                    print("📝 [CTX] VQA 上下文已重置。")
                    self.manager.convo.interrupt()
                    return

    def on_error(self, error):
        err = str(error)
        if "Internal server error" in err or "ping/pong" in err or "Task 'failed'" in err: self.manager.need_reconnect = True
        if self.manager.is_in_idle_mode:
            self.manager.idle_speak_complete.set()

    def on_connected(self): print("✅ WebSocket 已连接")
    def on_started(self, dialog_id: str): self.manager.dialog_id = dialog_id
    def on_state_changed(self, state: DialogState): print(f"📌 状态变化：{state}"); self.manager.current_state = state
    def on_speech_audio_data(self, data: bytes): self.audio_player.add_audio(data)
    
    def on_responding_started(self):
        self.is_responding = True
        self.response_text = ""
        print("\n🔊 AI开始回复...")
        # --- MODIFICATION START: Animation-TTS Sync Fix ---
        # 将audio_player实例传递给动画启动函数，以实现同步
        self.robot_controller.start_talking_animation(self.audio_player)
        # --- MODIFICATION END ---

    def on_responding_ended(self, payload):
        self.is_responding = False
        # 由于动画现在与音频缓冲区同步，此处的等待可以缩短，以获得更快的响应
        time.sleep(0.5) 
        self.robot_controller.stop_animation(return_to_neutral=True)
        if self.manager.is_in_idle_mode:
            self.manager.idle_speak_complete.set()
        
    def on_request_accepted(self): 
        print("\n✅ 打断成功")
        self.audio_player.clear_queue()
        self.robot_controller.stop_animation(return_to_neutral=False)

    def on_close(self, code, msg):
        print(f"\n📌 连接关闭：{code} — {msg}")
        if code != 1000: self.manager.need_reconnect = True
        if self.manager.is_in_idle_mode:
            self.manager.idle_speak_complete.set()

    def cleanup(self):
        if self.audio_player: self.audio_player.stop()
        if self.robot_controller: self.robot_controller.stop_current_task()
        # stop audio watcher thread
        try:
            if hasattr(self, '_audio_watcher_stop'):
                self._audio_watcher_stop.set()
            if hasattr(self, '_audio_watcher_thread') and self._audio_watcher_thread.is_alive():
                self._audio_watcher_thread.join(timeout=0.5)
        except Exception:
            pass

class DuplexVoiceAssistant:
    def __init__(self, robot_controller, camera_controller):
        self.robot_controller = robot_controller
        self.camera_controller = camera_controller
        self.need_reconnect, self.current_state = False, None
        self.pyaudio_instance = pyaudio.PyAudio()
        self.in_stream, self.audio_thread = None, None
        self.is_running = True
        self.last_interaction_time = time.time()
        self.is_in_idle_mode = False
        self.is_robot_busy = False
        self.idle_check_interval = 2
        self.idle_timeout = 100
        self.last_vqa_idle_time = 0
        self.idle_vqa_interval = 20
        self.idle_speak_complete = threading.Event()
        self.is_in_autonomous_speech = False
        self._build_dialog()
        self._init_audio_input()
        
    def _init_audio_input(self):
        if self.in_stream:
            try: self.in_stream.stop_stream(); self.in_stream.close()
            except: pass
        self.in_stream = self.pyaudio_instance.open(
            format=self.pyaudio_instance.get_format_from_width(config.SAMPLE_WIDTH),
            channels=config.CHANNELS, rate=config.SAMPLE_RATE, input=True, frames_per_buffer=config.AUDIO_FRAME_SIZE
        )
    
    def _build_dialog(self):
        up = Upstream(type="AudioAndVideo", mode="duplex", audio_format="pcm")
        down = Downstream(voice=config.VOICE_NAME, sample_rate=config.SAMPLE_RATE, audio_format="pcm")
        client = ClientInfo(user_id="sdk_user", device=Device(uuid="sdk_device"))
        params = RequestParameters(upstream=up, downstream=down, client_info=client)
        if hasattr(self, 'callback') and self.callback: self.callback.cleanup()
        self.callback = DuplexCallback(self, self.robot_controller, self.camera_controller)
        self.convo = MultiModalDialog(
            app_id=config.APP_ID, workspace_id=config.WORKSPACE_ID, api_key=config.API_KEY, url=config.WEBSOCKET_URL,
            request_params=params, multimodal_callback=self.callback, model=config.MODEL_NAME
        )
    
    def _recording_worker(self):
        recording = False
        print("\n💡 终端模式：按一次空格开始识别，再按一次空格结束。按空格可在AI回复时打断。按 ESC 退出")
        debounce_seconds = 0.25
        while self.is_running:
            try:
                if check_key_pressed('space'):
                    # simple debounce: ignore further presses for a short period
                    self.last_interaction_time = time.time()
                    if self.is_in_idle_mode:
                        print("\n👋 机器人已被唤醒！")
                        self.is_in_idle_mode = False
                        self.idle_speak_complete.set()
                        self.robot_controller.stop_current_task()
                        self.robot_controller.back_zero()
                        time.sleep(0.5)
                        time.sleep(debounce_seconds)
                        continue

                    if self.callback.is_responding:
                        # If AI is responding, space acts as interrupt
                        print("\n🔸 用户打断 AI（Space）...")
                        try:
                            self.convo.interrupt()
                        except Exception:
                            pass
                        time.sleep(debounce_seconds)
                        continue

                    # Toggle recording state
                    if not recording:
                        print("\n🎙️ 开始识别...")
                        try:
                            self.convo.start_speech()
                            recording = True
                        except Exception as e:
                            print(f"❌ 启动识别失败: {e}")
                            recording = False
                    else:
                        print("\n⏹️ 结束识别")
                        try:
                            self.convo.stop_speech()
                        except Exception as e:
                            print(f"❌ 停止识别时出错: {e}")
                        recording = False

                    # debounce to avoid double-toggles
                    time.sleep(debounce_seconds)
                    continue

                # While recording, continuously read audio and send
                if recording:
                    try:
                        if self.current_state != DialogState.RESPONDING:
                            data = self.in_stream.read(config.AUDIO_FRAME_SIZE, exception_on_overflow=False)
                            self.convo.send_audio_data(data)
                        else:
                            # If responding, just read and discard to avoid overflow
                            self.in_stream.read(self.in_stream.get_read_available(), exception_on_overflow=False)

                    except Exception as e:
                        print(f"❌ 读取音频/发送数据失败：{e}")
                        # if audio read fails, stop recording to recover
                        try:
                            self.convo.stop_speech()
                        except Exception:
                            pass
                        recording = False
                if check_key_pressed('esc'):
                    print("\n🛑 ESC 退出")
                    self.is_running = False
                    break

                time.sleep(0.01)
            except Exception as e:
                print(f"❌ 录音线程错误：{e}")
                time.sleep(0.1)

    def dispatch_robot_task(self, target_func, args=(), is_long_task=False):
        def task_wrapper():
            if is_long_task:
                self.is_robot_busy = True
                print(f"🚦 机器人进入忙碌状态，执行长任务: {target_func.__name__}")
            try:
                target_func(*args)
            finally:
                if is_long_task:
                    self.is_robot_busy = False
                    print(f"🚥 机器人忙碌状态解除。")
        
        thread = threading.Thread(target=task_wrapper, daemon=True)
        thread.start()

    def enter_idle_mode(self):
        self.is_in_idle_mode = True
        print(f"\n😴 {self.idle_timeout}秒无交互，进入空闲模式...")
        self.robot_controller.back_zero()
        self.last_vqa_idle_time = time.time()
        time.sleep(1)
        idle_actions = self.robot_controller.get_idle_action_list()

        while self.is_in_idle_mode and self.is_running:
            if not self.is_in_idle_mode: break
            
            if self.callback.is_responding:
                time.sleep(0.5)
                continue
            
            if time.time() - self.last_vqa_idle_time > self.idle_vqa_interval:
                self.perform_autonomous_vqa()
                self.last_vqa_idle_time = time.time()
            else:
                action = random.choice(idle_actions)
                action()
            
            pause_duration = random.uniform(2.0, 4.0)
            time.sleep(pause_duration)

        print("🧠 空闲模式结束，返回待命状态。")

    def perform_autonomous_vqa(self):
        print("🤔 (空闲) 我自己找点乐子...")
        
        self.is_in_autonomous_speech = True

        self.callback.is_sending_silence = True
        silence_thread = threading.Thread(target=self.callback._send_silence_worker, daemon=True)
        silence_thread.start()

        self.robot_controller.move_to_random_vqa_point()
        image_b64 = self.camera_controller.capture_image_base64()
        
        if not image_b64:
            print("❌ (空闲) 拍照失败，跳过本次VQA。")
            self.callback.is_sending_silence = False
            silence_thread.join(timeout=0.5)
            self.robot_controller.back_zero()
            self.is_in_autonomous_speech = False
            return
            
        print("🖼️ (空闲) 看看这是什么... 并想一句俏皮话...")
        comment_text = ""
        try:
            prompt = "你对当下照片生成一个让下一个大模型基于你看到的细节，仿佛是他看到的一样去评论20个字的prompt，把你在照片中看到的细节，传给下一个大模型，然后你也要告诉下一个大模型用幽默的语气评论这个照片中你提取出来的细节。。"
            messages = [{'role': 'user', 'content': [
                {'image': f'data:image/jpeg;base64,{image_b64}'},
                {'text': prompt}
            ]}]
            response = MultiModalConversation.call(model='qwen-vl-plus', messages=messages, api_key=config.API_KEY)
            comment_text = response.output.choices[0].message.content[0]['text']
        except Exception as e:
            print(f"❌ (空闲) VQA API 调用失败: {e}")
            self.callback.is_sending_silence = False
            silence_thread.join(timeout=0.5)
            self.robot_controller.back_zero()
            self.is_in_autonomous_speech = False
            return
        
        self.callback.is_sending_silence = False
        silence_thread.join(timeout=0.5)
        
        if self.current_state == DialogState.RESPONDING:
            print("ℹ️ 检测到系统在响应（可能为空），发送打断信号以重置状态...")
            try:
                self.convo.interrupt()
            except Exception as e:
                print(f"⚠️ 发送打断信号时出错: {e}")

        wait_start_time = time.time()
        while self.current_state != DialogState.LISTENING and (time.time() - wait_start_time) < 5.0:
            print(f"⏳ 等待系统返回 LISTENING 状态... 当前: {self.current_state}")
            time.sleep(0.2)

        if self.current_state != DialogState.LISTENING:
            print(f"❌ 等待 LISTENING 超时，无法继续。当前状态: {self.current_state}")
            self.robot_controller.back_zero()
            self.is_in_autonomous_speech = False
            return
        
        try:
            if comment_text:
                print(f"💬 (空闲) 准备通过主流程说出: “{comment_text}”")
                if self.current_state == DialogState.LISTENING and not self.callback.is_responding:
                    self.idle_speak_complete.clear()
                    self.convo.request_to_respond("prompt", comment_text)
                    print("...⏳ 等待自言自语完成...")
                    finished_in_time = self.idle_speak_complete.wait(timeout=20) 
                    if not finished_in_time:
                        print("⚠️ 等待自言自语超时！")
                        self.convo.interrupt()
                else:
                    print(f"⚠️ (空闲) 主对话系统未就绪，跳过本次自言自语。当前状态: {self.current_state}, 是否响应中: {self.callback.is_responding}")
        finally:
            self.is_in_autonomous_speech = False

        time.sleep(1)
        self.robot_controller.back_zero()

    def start(self):
        print("🔗 启动助手...")
        try: self.convo.start(""); time.sleep(1); return True
        except Exception as e: print(f"❌ 启动失败：{e}"); return False
        
    def start_continuous_recording(self):
        self.audio_thread = threading.Thread(target=self._recording_worker, daemon=True)
        self.audio_thread.start()
        
    def run(self):
        if not self.start(): return
        start_time = time.time()
        while time.time() - start_time < 10:
            if self.current_state == DialogState.LISTENING: break
            time.sleep(0.1)
        if self.current_state != DialogState.LISTENING:
            print("❌ 系统在10秒内未就绪，请检查网络或API-KEY。"); self.cleanup(); return
            
        print("✅ 系统就绪，开始多轮对话"); self.start_continuous_recording()
        try:
            while self.is_running:
                if self.need_reconnect:
                    print("\n⚡ 检测到需要重连，将在5秒后尝试..."); time.sleep(5)
                    self.reconnect()
                
                is_ready_for_idle = (not self.is_in_idle_mode and 
                                     not self.is_robot_busy and
                                     not self.callback.is_responding)
                
                if is_ready_for_idle and (time.time() - self.last_interaction_time > self.idle_timeout):
                    self.enter_idle_mode()
                
                time.sleep(self.idle_check_interval)
        except KeyboardInterrupt:
            print("\n🛑 手动中断")
        finally:
            self.cleanup()

    def reconnect(self):
        print("🔄 正在执行重连..."); self.need_reconnect = False; self.is_running = False
        if self.audio_thread and self.audio_thread.is_alive(): self.audio_thread.join(timeout=1)
        try: self.convo.stop()
        except Exception as e: print(f"⚠️ 停止旧连接时出错: {e}")
        time.sleep(1)
        print("🔄 重新初始化组件..."); self.is_running = True
        self._build_dialog(); self._init_audio_input()
        if self.start():
            start_time = time.time()
            while time.time() - start_time < 10:
                if self.current_state == DialogState.LISTENING: break
                time.sleep(0.1)
            if self.current_state == DialogState.LISTENING:
                self.start_continuous_recording(); print("✅ 重连成功，系统已就绪！"); return
        print("❌ 重连失败，程序将退出。"); self.is_running = False
        
    def cleanup(self):
        print("\n🧹 清理资源..."); self.is_running = False
        self.robot_controller.stop_current_task()
        if self.audio_thread and self.audio_thread.is_alive(): self.audio_thread.join(timeout=1)
        try: self.convo.stop()
        except: pass
        if self.in_stream:
            try: self.in_stream.stop_stream(); self.in_stream.close()
            except: pass
        if hasattr(self, 'callback') and self.callback: self.callback.cleanup()
        self.pyaudio_instance.terminate()
        # stop terminal key listener if used
        try:
            if terminal_key_listener:
                terminal_key_listener.stop()
        except Exception:
            pass
        print("✅ 清理完成")