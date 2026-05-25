import tkinter as tk
from tkinter import filedialog, messagebox, ttk # Use ttk for themed widgets like Combobox and Labelframe
import can
import os
import struct
import threading
import time
import platform
import binascii # Required for CRC calculation
import queue  # For inter-thread communication
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import ctypes
from ctypes import wintypes

# ============================================================================
# ZLG USBCAN2 支持 - 自定义 CAN 总线类，调用 zlgcan.dll
# ============================================================================

# ZLG 定义常量
VCI_USBCAN2 = 4          # 设备类型
VCI_USBCAN2_INDEX = 0    # 设备索引，一般从0开始
CHANNEL_0 = 0            # CAN通道0
CHANNEL_1 = 1

STATUS_OK = 1

# 帧类型掩码
CAN_ID_STANDARD = 0x00000000
CAN_ID_EXTENDED = 0x80000000

class ZLGCAN_STRUCT(ctypes.Structure):
    """ZLG CAN 帧结构体（与 CAN_OBJ 对应）"""
    _fields_ = [
        ("ID", ctypes.c_uint),           # 帧ID
        ("TimeStamp", ctypes.c_uint),    # 时间戳
        ("TimeFlag", ctypes.c_byte),     # 时间标志
        ("SendType", ctypes.c_byte),     # 发送类型
        ("RemoteFlag", ctypes.c_byte),   # 远程帧标志
        ("ExternFlag", ctypes.c_byte),   # 扩展帧标志
        ("DataLen", ctypes.c_byte),      # 数据长度 DLC
        ("Data", ctypes.c_byte * 8),     # 数据
        ("Reserved", ctypes.c_byte * 3), # 保留
    ]

class ZlgCanBus:
    """
    自定义 CAN 总线类，封装 ZLG USBCAN2 动态库调用，
    实现与 python-can.Bus 相似的接口，以便 BackendAPI 使用。
    """
    def __init__(self, channel=CHANNEL_0, bitrate=250000):
        self.channel = channel
        self.bitrate = bitrate
        self.dll = None

        script_dir = os.path.dirname(os.path.abspath(__file__))
        dll_dirs = [script_dir, os.path.join(script_dir, "kerneldlls")]
        for d in dll_dirs:
            if os.path.isdir(d):
                try:
                    os.add_dll_directory(d)
                except (AttributeError, Exception):
                    pass

        dll_path = os.path.join(script_dir, "zlgcan.dll")
        if not os.path.isfile(dll_path):
            dll_path = "zlgcan.dll"

        try:
            self.dll = ctypes.WinDLL(dll_path)
        except OSError as e:
            kernel_dlls_path = os.path.join(script_dir, "kerneldlls")
            if os.path.isdir(kernel_dlls_path):
                old_path = os.environ.get("PATH", "")
                os.environ["PATH"] = kernel_dlls_path + os.pathsep + old_path
                try:
                    self.dll = ctypes.WinDLL(dll_path)
                except OSError as e2:
                    # 检测位数不匹配
                    if e2.winerror == 193:  # ERROR_BAD_EXE_FORMAT
                        python_bits = ctypes.sizeof(ctypes.c_void_p) * 8
                        raise RuntimeError(
                            f"DLL 位数不匹配：当前 Python 是 {python_bits} 位，但 '{dll_path}' 可能为 {64 if python_bits==32 else 32} 位。\n"
                            f"请使用 {python_bits} 位版本的 zlgcan.dll（可从 ZLG 开发包中获取）。"
                        ) from e2
                    else:
                        raise RuntimeError(f"加载 zlgcan.dll 失败（已调整 PATH）：{e2}")
            else:
                raise RuntimeError(f"加载 zlgcan.dll 失败：{e}")

        # 定义函数原型 
        self.dll.OpenDevice.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        self.dll.OpenDevice.restype = ctypes.c_uint

        self.dll.CloseDevice.argtypes = [ctypes.c_uint, ctypes.c_uint]
        self.dll.CloseDevice.restype = ctypes.c_uint

        self.dll.InitCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        self.dll.InitCAN.restype = ctypes.c_uint

        self.dll.ReadCanMsg.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ZLGCAN_STRUCT), ctypes.c_uint, ctypes.c_uint]
        self.dll.ReadCanMsg.restype = ctypes.c_uint

        self.dll.WriteCanMsg.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.POINTER(ZLGCAN_STRUCT), ctypes.c_uint]
        self.dll.WriteCanMsg.restype = ctypes.c_uint

        self.dll.StartCAN = self.dll.StartCAN
        self.dll.StartCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        self.dll.StartCAN.restype = ctypes.c_uint

        self.dll.ResetCAN = self.dll.ResetCAN
        self.dll.ResetCAN.argtypes = [ctypes.c_uint, ctypes.c_uint, ctypes.c_uint]
        self.dll.ResetCAN.restype = ctypes.c_uint

        # 打开设备
        if self.dll.OpenDevice(VCI_USBCAN2, VCI_USBCAN2_INDEX, 0) != STATUS_OK:
            raise RuntimeError("OpenDevice failed")

        # 初始化 CAN 通道
        # 波特率换算：250k -> 0x06000C, 500k -> 0x060008, 1M -> 0x060004
        baud_map = {
            125000: 0x06000E,
            250000: 0x06000C,
            500000: 0x060008,
            1000000: 0x060004,
        }
        baud_code = baud_map.get(bitrate, 0x06000C)
        if self.dll.InitCAN(VCI_USBCAN2, VCI_USBCAN2_INDEX, self.channel, baud_code) != STATUS_OK:
            self.dll.CloseDevice(VCI_USBCAN2, VCI_USBCAN2_INDEX)
            raise RuntimeError("InitCAN failed")

        # 启动 CAN 通道
        if self.dll.StartCAN(VCI_USBCAN2, VCI_USBCAN2_INDEX, self.channel) != STATUS_OK:
            self.dll.CloseDevice(VCI_USBCAN2, VCI_USBCAN2_INDEX)
            raise RuntimeError("StartCAN failed")

    def send(self, msg):
        """
        发送 CAN 消息
        :param msg: python-can.Message 对象
        """
        zlg_msg = ZLGCAN_STRUCT()
        zlg_msg.ID = msg.arbitration_id
        zlg_msg.ExternFlag = 1 if msg.is_extended_id else 0
        zlg_msg.RemoteFlag = 0  # 数据帧
        zlg_msg.SendType = 0    # 正常发送
        zlg_msg.DataLen = msg.dlc if msg.dlc else len(msg.data)

        data_bytes = msg.data[:8]
        for i, b in enumerate(data_bytes):
            zlg_msg.Data[i] = b
        for i in range(len(data_bytes), 8):
            zlg_msg.Data[i] = 0

        if self.dll.WriteCanMsg(VCI_USBCAN2, VCI_USBCAN2_INDEX, self.channel, ctypes.byref(zlg_msg), 1) != STATUS_OK:
            raise can.CanError("WriteCanMsg failed")

    def recv(self, timeout=0.01):
        """
        接收 CAN 消息（非阻塞）
        :param timeout: 超时时间（秒），实际用于内部休眠，因为 ReadCanMsg 本身是立即返回的
        :return: python-can.Message 对象或 None
        """
        start = time.time()
        while True:
            buf = (ZLGCAN_STRUCT * 10)()
            count = self.dll.ReadCanMsg(VCI_USBCAN2, VCI_USBCAN2_INDEX, self.channel, buf, 10, 0)
            if count > 0:
                zlg_msg = buf[0]
                msg = can.Message(
                    arbitration_id=zlg_msg.ID,
                    data=bytes(zlg_msg.Data[:zlg_msg.DataLen]),
                    dlc=zlg_msg.DataLen,
                    is_extended_id=(zlg_msg.ExternFlag != 0)
                )
                return msg
            if timeout is not None and (time.time() - start) >= timeout:
                return None
            time.sleep(0.001)

    def shutdown(self):
        """关闭 CAN 设备"""
        try:
            self.dll.ResetCAN(VCI_USBCAN2, VCI_USBCAN2_INDEX, self.channel)
            self.dll.CloseDevice(VCI_USBCAN2, VCI_USBCAN2_INDEX)
        except:
            pass

# ============================================================================
# 协议配置类 - 集中管理所有CAN协议常量
# ============================================================================

@dataclass
class ProtocolConfig:
    """CAN协议配置 - 统一管理所有协议常量，避免硬编码分散"""

    # VIN 协议配置
    VIN_CMD_ID: int = 0x1BE          # VIN命令ID (标准帧)
    VIN_DATA_ID: int = 0x1BF         # VIN数据帧ID (标准帧)
    VIN_RESP_ID: int = 0x2BE         # VIN响应ID (标准帧)
    VIN_LENGTH: int = 17             # VIN码长度
    VIN_CMD_READ: List[int] = None   # 读取命令 [0x01, 0x00, ...]
    VIN_CMD_WRITE_START: List[int] = None  # 写入开始命令 [0x00, 0x01, ...]
    VIN_CMD_END: List[int] = None    # 结束命令 [0x00, 0x00, ...]
    VIN_TIMEOUT: float = 2.0         # VIN读取总超时(秒)
    VIN_MAX_RETRIES: int = 3         # VIN读取最大重试次数
    VIN_FRAME_INTERVAL: float = 0.01 # 帧间隔(秒)

    # 里程数协议配置
    MILEAGE_WRITE_ID: int = 0x700    # 里程数写入ID (标准帧)
    MILEAGE_RESP_ID: int = 0x710     # 里程数响应ID (标准帧)
    MILEAGE_BROADCAST_ID: int = 0x3D0  # 里程数广播ID (标准帧)
    MILEAGE_ADDRESS: int = 0x0001    # 里程数写入地址
    MILEAGE_MAX_VALUE: int = 0xFFFFFF  # 24bit最大值

    # 静默模式协议配置
    SILENCE_CMD_ID: int = 0x8200000  # 静默命令ID (扩展帧)
    SILENCE_ENTER_DATA: List[int] = None  # 进入静默 [0xAA, ...]
    SILENCE_EXIT_DATA: List[int] = None   # 退出静默 [0x55, ...]

    # VCU参数配置协议配置
    PARAM_CONFIG_ID: int = 0x700      # 参数配置请求ID (标准帧)
    PARAM_RESP_ID: int = 0x710        # 参数配置响应ID (标准帧)
    PARAM_TIMEOUT: float = 2.0        # 参数配置超时(秒)

    # 参数定义（14个参数）
    VCU_PARAMS: Dict[int, Dict[str, any]] = None

    # OTA设备映射
    DEVICE_PREFIX_MAP: Dict[str, int] = None

    # 帧类型
    FRAME_TYPE_STANDARD: bool = False
    FRAME_TYPE_EXTENDED: bool = True

    def __post_init__(self):
        """初始化列表和字典"""
        if self.VIN_CMD_READ is None:
            self.VIN_CMD_READ = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        if self.VIN_CMD_WRITE_START is None:
            self.VIN_CMD_WRITE_START = [0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        if self.VIN_CMD_END is None:
            self.VIN_CMD_END = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        if self.SILENCE_ENTER_DATA is None:
            self.SILENCE_ENTER_DATA = [0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        if self.SILENCE_EXIT_DATA is None:
            self.SILENCE_EXIT_DATA = [0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        if self.DEVICE_PREFIX_MAP is None:
            self.DEVICE_PREFIX_MAP = {
                "BMS": 0x42, "VCU": 0x43, "MCU": 0x44,
                "IC": 0x45, "ABS": 0x46, "T-Box": 0x47
            }
        if self.VCU_PARAMS is None:
            self.VCU_PARAMS = {
                0x01: {"name": "ODO总里程数", "unit": "km", "scale": 1.0, "type": "int"},
                0x02: {"name": "轮胎滚动半径", "unit": "mm", "scale": 1.0, "type": "int"},
                0x03: {"name": "电机齿轮减速比", "unit": "", "scale": 0.001, "type": "float"},
                0x04: {"name": "整车最大续航里程数", "unit": "km", "scale": 1.0, "type": "int"},
                0x05: {"name": "VCU定时唤醒补电时间", "unit": "min", "scale": 1.0, "type": "int"},
                0x06: {"name": "VCU下电休眠等待时间", "unit": "ms", "scale": 1.0, "type": "int"},
                0x07: {"name": "车辆倾斜切断动力角度", "unit": "°", "scale": 1.0, "type": "int"},
                0x08: {"name": "车辆扶正恢复动力角度", "unit": "°", "scale": 1.0, "type": "int"},
                0x09: {"name": "开启水泵电机温度", "unit": "℃", "scale": 1.0, "type": "int"},
                0x0A: {"name": "关闭水泵电机温度", "unit": "℃", "scale": 1.0, "type": "int"},
                0x0B: {"name": "开启水泵电控温度", "unit": "℃", "scale": 1.0, "type": "int"},
                0x0C: {"name": "关闭水泵电控温度", "unit": "℃", "scale": 1.0, "type": "int"},
                0x0D: {"name": "开启水泵OBC温度", "unit": "℃", "scale": 1.0, "type": "int"},
                0x0E: {"name": "关闭水泵OBC温度", "unit": "℃", "scale": 1.0, "type": "int"},
                0x0F: {"name": "请求水泵强制打开/关闭", "unit": "", "scale": 1.0, "type": "int"},
                0x10: {"name": "六轴IMU校准", "unit": "", "scale": 1.0, "type": "int"},
            }


# 全局协议配置实例
PROTOCOL = ProtocolConfig()


# ============================================================================
# 后端业务逻辑API - 统一的CAN通信和业务处理接口
# ============================================================================

class BackendAPI:
    """
    后端业务逻辑API - 统一的CAN通信和业务处理接口

    职责：
    - 封装所有CAN发送/接收操作
    - 提供统一的后端业务API（OTA、VIN、里程数等）
    - 管理发送队列和广播队列
    - 集中处理CAN消息接收和分发
    - 统一的响应等待机制（所有操作使用相同的response_registry）
    """
    def __init__(self, can_bus, log_callback):
        """
        初始化BackendAPI

        Args:
            can_bus: python-can Bus对象或 ZlgCanBus 对象
            log_callback: 日志回调函数，用于记录消息到GUI
        """
        self.can_bus = can_bus
        self.log = log_callback

        # 线程管理
        self.running = False
        self.recv_thread = None
        self.send_thread = None
        self._threads_stopped = threading.Event()  # 线程停止事件

        # 消息队列
        self.send_queue = queue.Queue()  # 发送队列
        self.broadcast_queue = queue.Queue()  # 广播消息队列（通知前端）

        # 日志队列（异步记录，避免阻塞接收线程）
        self.log_queue = queue.Queue()  # 日志队列
        self._start_log_worker()  # 启动日志处理线程

        # 统一的响应等待机制
        self.response_registry = {}  # {can_id: {'event': threading.Event, 'data': None, 'timestamp': float}}
        self.vin_session_active = False  # VIN会话活跃标志（用于过滤干扰帧）

    def start(self):
        """启动接收和发送线程"""
        if self.running:
            self._log_async("[Backend] Threads already running")
            return

        self.running = True
        self._threads_stopped.clear()
        self.recv_thread = threading.Thread(target=self._can_recv_worker, daemon=True, name="CAN-Recv")
        self.send_thread = threading.Thread(target=self._can_send_worker, daemon=True, name="CAN-Send")
        self.recv_thread.start()
        self.send_thread.start()
        self._log_async("[Backend] CAN threads started")

    def stop(self):
        """停止所有线程 - 改进的资源清理"""
        if not self.running:
            return

        self._log_async("[Backend] Stopping CAN threads...")
        self.running = False

        # 发送哨兵值通知发送线程退出
        self.send_queue.put(None)

        # 等待线程结束（增加超时时间）
        if self.send_thread and self.send_thread.is_alive():
            self.send_thread.join(timeout=3)
        if self.recv_thread and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=3)

        # 设置停止事件，通知日志线程
        self._threads_stopped.set()

        # 清空响应注册表
        self.response_registry.clear()

        self._log_async("[Backend] CAN threads stopped")

    def _start_log_worker(self):
        """启动日志处理线程 - 异步记录日志，避免阻塞接收线程"""
        def log_worker():
            while not self._threads_stopped.is_set():
                try:
                    log_msg = self.log_queue.get(timeout=0.5)
                    if log_msg:
                        self.log(log_msg)  # 实际调用GUI回调
                except queue.Empty:
                    continue
                except Exception as e:
                    # 日志线程本身出错，直接打印到stderr避免递归
                    print(f"[LogWorker Error] {e}")

        threading.Thread(target=log_worker, daemon=True, name="LogWorker").start()

    def _log_async(self, message: str):
        """异步记录日志 - 非阻塞"""
        try:
            self.log_queue.put(message, block=False)
        except queue.Full:
            # 队列满时直接记录（降级）
            self.log(message)

    # --- 前端调用的API ---

    def send_can(self, can_id, data, extended_id=True, rsp_id=0x7f0, wait_response=False, timeout=5):
        """
        发送CAN消息

        Args:
            can_id: CAN仲裁ID
            data: 数据列表（如[0xAA, 0xBB, ...]）
            extended_id: 是否为扩展帧
            wait_response: 是否等待响应
            timeout: 等待响应超时时间（秒）

        Returns:
            响应数据（如果wait_response=True），否则返回None
        """
        if wait_response:
            # 注册期望的响应
            self._register_response_wait(rsp_id)

        # 加入发送队列
        self.send_queue.put({'id': can_id, 'data': data, 'extended': extended_id})

        if wait_response:
            return self._wait_for_response(rsp_id, timeout)
        return None

    def get_broadcast_message(self, block=False, timeout=0.1):
        """
        从广播队列获取消息（供GUI轮询调用）

        Args:
            block: 是否阻塞等待
            timeout: 超时时间

        Returns:
            消息字典或None
        """
        try:
            return self.broadcast_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None

    # --- 内部工作方法 ---

    def _can_recv_worker(self):
        """CAN接收线程 - 集中接收所有消息"""
        while self.running:
            try:
                msg = self.can_bus.recv(timeout=0.01)
                if msg:
                    self._dispatch_can_message(msg)
            except can.CanError as e:
                if self.running:
                    self._log_async(f"[Recv-Error] {e}")
            except Exception as e:
                if self.running:
                    self._log_async(f"[Recv-Error] Unexpected: {e}")

    def _can_send_worker(self):
        """CAN发送线程 - 从队列取消息发送"""
        while self.running:
            try:
                item = self.send_queue.get(timeout=0.5)
                if item is None:  # 哨兵值，用于退出
                    break
                self._send_can_actual(item)
            except queue.Empty:
                continue
            except Exception as e:
                if self.running:
                    self._log_async(f"[Send-Error] {e}")

    def _send_can_actual(self, item):
        """实际发送CAN消息"""
        can_id = item['id']
        data = item['data']
        extended_id = item.get('extended', True)

        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            dlc=len(data),
            is_extended_id=extended_id
        )
        self.can_bus.send(msg)

        # 记录发送日志（异步）
        data_hex = ' '.join([f'{byte:02X}' for byte in data])
        if extended_id:
            self._log_async(f"[TX]<0x{can_id:08X}>({len(data)}){{{data_hex}}}")
        else:
            self._log_async(f"[TX]<0x{can_id:03X}>({len(data)}){{{data_hex}}}")

    def _dispatch_can_message(self, msg):
        """分发接收到的CAN消息 - 统一处理，使用response_registry"""
        can_id = msg.arbitration_id
        data_hex = ' '.join([f'{byte:02X}' for byte in msg.data])

        # 1. 检查是否为注册的响应（OTA、里程数写入等）
        if can_id in self.response_registry:
            self._handle_registered_response(msg)
            return

        # 2. VIN响应（0x2BE）处理 - 如果VIN会话活跃，则收集
        if can_id == PROTOCOL.VIN_RESP_ID:
            if self.vin_session_active:
                # VIN会话中，调用处理器
                self._handle_vin_response(msg)
            # VIN响应不记录通用日志，由处理器记录
            return

        # 3. 记录接收日志（异步，非阻塞）
        # if msg.is_extended_id:
            # self._log_async(f"[RX]<0x{can_id:08X}>({len(msg.data)}){{{data_hex}}}")
        # else:
            # self._log_async(f"[RX]<0x{can_id:03X}>({len(msg.data)}){{{data_hex}}}")

        # 4. 处理特定ID的广播消息
        if can_id == PROTOCOL.MILEAGE_BROADCAST_ID:  # 里程数广播
            self._handle_mileage_broadcast(msg)

    # --- 响应等待机制 ---

    def _register_response_wait(self, can_id):
        """注册等待特定CAN ID的响应"""
        event = threading.Event()
        self.response_registry[can_id] = {
            'event': event,
            'data': None,
            'timestamp': time.time()
        }

    def _wait_for_response(self, can_id, timeout=5):
        """等待响应"""
        if can_id not in self.response_registry:
            self.log(f"[Error] No response wait registered for ID 0x{can_id:08X}")
            return None

        registry = self.response_registry[can_id]
        event = registry['event']

        if event.wait(timeout=timeout):
            # 成功收到响应
            data = registry['data']
            del self.response_registry[can_id]
            return data
        else:
            # 超时
            self.log(f"[RX] Timeout waiting for response ID 0x{can_id:08X} after {timeout}s")
            del self.response_registry[can_id]
            return None

    def _handle_registered_response(self, msg):
        """处理注册的响应"""
        can_id = msg.arbitration_id

        if can_id in self.response_registry:
            # 记录接收日志（异步）
            data_hex = ' '.join([f'{byte:02X}' for byte in msg.data])
            if msg.is_extended_id:
                self._log_async(f"[RX]<0x{can_id:08X}>({len(msg.data)}){{{data_hex}}}")
            else:
                self._log_async(f"[RX]<0x{can_id:03X}>({len(msg.data)}){{{data_hex}}}")

            # 保存数据并设置事件
            self.response_registry[can_id]['data'] = msg.data
            self.response_registry[can_id]['event'].set()

    def _handle_vin_response(self, msg):
        """处理VIN响应 - 基于02脚本的成功实现"""
        # 将VIN帧放入内部收集器
        if hasattr(self, '_vin_frame_collector'):
            data_hex = ' '.join([f'{byte:02X}' for byte in msg.data])
            self._log_async(f"[RX]<0x{msg.arbitration_id:03X}>({len(msg.data)}){{{data_hex}}}")
            self._vin_frame_collector.append(msg.data)

    # --- 各ID消息处理器 ---

    def _handle_mileage_broadcast(self, msg):
        """处理里程数广播 - 放入broadcast_queue通知前端"""
        try:
            # 解析里程数: bit24开始，长度为24bit，小端
            mileage_bytes = msg.data[3:6]
            mileage_value = mileage_bytes[0] | (mileage_bytes[1] << 8) | (mileage_bytes[2] << 16)

            self._log_async(f"[RX] Mileage broadcast (0x3D0): {mileage_value} (0x{mileage_value:06X})")

            # 放入广播队列通知前端
            self.broadcast_queue.put({
                'type': 'mileage',
                'can_id': msg.arbitration_id,
                'data': mileage_value
            })
        except Exception as e:
            self._log_async(f"[Error] Failed to parse mileage broadcast: {e}")

    # --- 业务功能方法 ---

    def vin_write(self, vin_str):
        """
        写入 VIN 码到 ECU

        Args:
            vin_str: 17字符的VIN码

        Returns:
            (success: bool, message: str)
        """
        # 验证长度和格式
        vin_str = vin_str.strip()
        if len(vin_str) != PROTOCOL.VIN_LENGTH:
            return False, f"VIN码必须为{PROTOCOL.VIN_LENGTH}个字符! 当前: {len(vin_str)} 个字符"

        if not vin_str.isascii() or not vin_str.isalnum():
            return False, "VIN码必须为纯ASCII字母数字! 只允许: 0-9, A-Z"

        try:
            # 发送写入开始命令（使用协议配置）
            self.send_can(PROTOCOL.VIN_CMD_ID, PROTOCOL.VIN_CMD_WRITE_START, PROTOCOL.FRAME_TYPE_STANDARD)
            self._log_async("[VIN-WRITE] Start command sent")
            time.sleep(PROTOCOL.VIN_FRAME_INTERVAL)

            # 分3帧发送VIN数据（基于02脚本的成功实现）
            vin_bytes = vin_str.encode('ascii')
            for frame_num in [1, 2, 3]:
                # 计算本帧数据范围
                start_idx = (frame_num - 1) * 6
                end_idx = min(start_idx + 6, PROTOCOL.VIN_LENGTH)

                # 组装帧数据
                length = end_idx - start_idx
                if length == 6:
                    frame_data = [frame_num, length] + list(vin_bytes[start_idx:end_idx])
                else:
                    frame_data = [frame_num, length] + list(vin_bytes[start_idx:end_idx]) + [0]

                # 发送帧
                self.send_can(PROTOCOL.VIN_DATA_ID, frame_data, PROTOCOL.FRAME_TYPE_STANDARD)
                time.sleep(PROTOCOL.VIN_FRAME_INTERVAL)

            self._log_async(f"[VIN-WRITE] Data sent (3 frames): {vin_str}")
            time.sleep(PROTOCOL.VIN_FRAME_INTERVAL)

            # 发送结束命令
            self.send_can(PROTOCOL.VIN_CMD_ID, PROTOCOL.VIN_CMD_END, PROTOCOL.FRAME_TYPE_STANDARD)
            self._log_async("[VIN-WRITE] End command sent")

            return True, "VIN码写入成功!"

        except Exception as e:
            error_msg = f"VIN写入失败: {e}"
            self._log_async(f"[Error] {error_msg}")
            return False, error_msg

    def vin_read(self):
        """
        从 ECU 读取 VIN 码 - 基于02脚本的成功实现，增加重试机制

        Returns:
            (success: bool, vin_str: str or None, message: str)
        """
        # 重试机制
        for retry in range(PROTOCOL.VIN_MAX_RETRIES):
            try:
                self._log_async(f"[VIN-READ] Attempt {retry + 1}/{PROTOCOL.VIN_MAX_RETRIES}")

                # 初始化VIN帧收集器
                self._vin_frame_collector = []
                self.vin_session_active = True

                # 发送读取命令（使用协议配置）
                self.send_can(PROTOCOL.VIN_CMD_ID, PROTOCOL.VIN_CMD_READ, PROTOCOL.FRAME_TYPE_STANDARD)
                self._log_async("[VIN-READ] Read command sent")
                time.sleep(PROTOCOL.VIN_FRAME_INTERVAL)

                # 收集3帧 - 基于02脚本的成功实现
                frames = {}
                start_time = time.time()

                self._log_async(f"[VIN-READ] Waiting for response (timeout={PROTOCOL.VIN_TIMEOUT}s)...")

                while time.time() - start_time < PROTOCOL.VIN_TIMEOUT:
                    # 从收集器获取已接收的帧
                    while self._vin_frame_collector:
                        data = self._vin_frame_collector.pop(0)

                        if len(data) >= 2:
                            frame_num = data[0]
                            length = data[1]

                            # 只处理1-3帧，避免重复
                            if 1 <= frame_num <= 3:
                                if frame_num not in frames:
                                    frames[frame_num] = data
                                    self._log_async(f"[VIN-READ] Received frame {frame_num}, length={length}")

                                    if len(frames) == 3:
                                        self._log_async("[VIN-READ] All 3 frames received!")
                                        break

                    if len(frames) == 3:
                        break

                    time.sleep(0.01)  # 避免CPU占用过高

                # 发送结束命令
                time.sleep(PROTOCOL.VIN_FRAME_INTERVAL)
                self.send_can(PROTOCOL.VIN_CMD_ID, PROTOCOL.VIN_CMD_END, PROTOCOL.FRAME_TYPE_STANDARD)
                self._log_async("[VIN-READ] End command sent")

                # 清理会话状态
                self.vin_session_active = False
                delattr(self, '_vin_frame_collector')

                # 验证是否收到所有帧
                if len(frames) < 3:
                    missing = [i for i in [1, 2, 3] if i not in frames]
                    if retry < PROTOCOL.VIN_MAX_RETRIES - 1:
                        self._log_async(f"[VIN-READ] Retry needed - missing frames: {missing}")
                        time.sleep(0.1)  # 短暂延迟后重试
                        continue
                    else:
                        return False, None, f"读取失败（{PROTOCOL.VIN_MAX_RETRIES}次重试后仍缺失帧 {missing}）"

                # 组装VIN
                vin_bytes = b''
                for i in [1, 2, 3]:
                    frame = frames[i]
                    length = frame[1]
                    vin_bytes += bytes(frame[2:2+length])

                # 验证长度
                if len(vin_bytes) != PROTOCOL.VIN_LENGTH:
                    if retry < PROTOCOL.VIN_MAX_RETRIES - 1:
                        self._log_async(f"[VIN-READ] Retry needed - invalid length: {len(vin_bytes)}")
                        time.sleep(0.1)
                        continue
                    else:
                        return False, None, f"接收到的VIN长度不正确: {len(vin_bytes)} 字节 (应为 {PROTOCOL.VIN_LENGTH} 字节)"

                # 解析VIN
                vin_str = vin_bytes.decode('ascii')
                self._log_async(f"[VIN-READ] Success: {vin_str}")

                return True, vin_str, "VIN码读取成功!"

            except Exception as e:
                # 确保清理会话状态
                self.vin_session_active = False
                if hasattr(self, '_vin_frame_collector'):
                    delattr(self, '_vin_frame_collector')

                if retry < PROTOCOL.VIN_MAX_RETRIES - 1:
                    self._log_async(f"[VIN-READ] Retry needed - error: {e}")
                    time.sleep(0.1)
                    continue
                else:
                    error_msg = f"VIN读取失败（{PROTOCOL.VIN_MAX_RETRIES}次重试后仍失败）: {e}"
                    self._log_async(f"[Error] {error_msg}")
                    return False, None, error_msg

        return False, None, f"VIN读取失败：超过最大重试次数"

    def mileage_write(self, mileage_str, mileage_toggle):
        """
        写入里程数到 ECU

        Args:
            mileage_str: 里程数字符串（十进制或十六进制）
            mileage_toggle: 当前toggle值（将被取反）

        Returns:
            (success: bool, new_toggle: int, message: str)
        """
        try:
            # 将字符串转换为整数
            if mileage_str.startswith('0x') or mileage_str.startswith('0X'):
                mileage_value = int(mileage_str, 16)
            else:
                mileage_value = int(mileage_str)

            # 检查范围（使用协议配置）
            if mileage_value < 0 or mileage_value > PROTOCOL.MILEAGE_MAX_VALUE:
                return False, mileage_toggle, f"里程数超出24bit范围! 有效范围: 0 ~ {PROTOCOL.MILEAGE_MAX_VALUE} (0x{PROTOCOL.MILEAGE_MAX_VALUE:06X}), 当前值: {mileage_value}"

            self._log_async(f"准备写入里程数: {mileage_value} (0x{mileage_value:06X})")

            # 构建配置请求帧
            config_byte = mileage_toggle & 0x01
            new_toggle = mileage_toggle ^ 0x01

            address = PROTOCOL.MILEAGE_ADDRESS
            address_bytes = list(struct.pack('<H', address))
            reserved = 0x00
            data_bytes = list(struct.pack('<I', mileage_value))

            frame_data = [config_byte] + address_bytes + [reserved] + data_bytes

            # 发送并等待响应（使用协议配置）
            response = self.send_can(PROTOCOL.MILEAGE_WRITE_ID, frame_data, PROTOCOL.FRAME_TYPE_STANDARD, rsp_id=PROTOCOL.MILEAGE_RESP_ID, wait_response=True, timeout=2)
            self._log_async(f"[TX] Mileage Write: Value={mileage_value}, Toggle={config_byte:02X}, Addr=0x{address:04X}")

            if response and len(response) >= 8:
                resp_config = response[0]
                resp_addr = struct.unpack('<H', bytes(response[1:3]))[0]
                resp_data = struct.unpack('<I', bytes(response[4:8]))[0]

                if resp_config == config_byte:
                    self._log_async(f"[RX] Mileage write successful! Response: Data=0x{resp_data:08X}")
                    return True, new_toggle, f"里程数写入成功! 写入值: {mileage_value}"
                else:
                    error_msg = f"设备返回错误! 发送的配置位: 0x{config_byte:02X}, 返回的配置位: 0x{resp_config:02X}"
                    self._log_async(f"[RX] {error_msg}")
                    return False, new_toggle, error_msg
            else:
                return False, new_toggle, "未收到设备回应或超时"

        except ValueError:
            return False, mileage_toggle, "输入格式错误: 请输入有效的数字! 支持格式: 十进制(12345) 或 十六进制(0x3039)"
        except Exception as e:
            error_msg = f"里程数写入失败: {e}"
            self._log_async(f"[Error] {error_msg}")
            return False, mileage_toggle, error_msg

    def param_write(self, param_address, value_str, toggle):
        """
        写入VCU参数（通用方法）

        Args:
            param_address: 参数地址（0x01-0x0E）
            value_str: 用户输入的值（十进制字符串）
            toggle: 当前toggle值（将被取反）

        Returns:
            (success: bool, new_toggle: int, message: str)
        """
        # 1. 获取参数定义并验证
        if param_address not in PROTOCOL.VCU_PARAMS:
            return False, toggle, f"未知的参数地址: 0x{param_address:02X}"

        param_def = PROTOCOL.VCU_PARAMS[param_address]

        # 2. 解析输入（仅支持十进制）
        try:
            if param_def["type"] == "int":
                user_value = int(value_str)
            else:
                user_value = float(value_str)
        except ValueError:
            return False, toggle, f"输入格式错误！期望{param_def['type']}类型"

        # 3. 缩放系数转换（显示值 -> 报文值）
        if param_def["scale"] != 1.0:
            raw_value = int(round(user_value / param_def["scale"]))
        else:
            raw_value = int(user_value)

        # 4. 32bit范围检查
        if raw_value < 0 or raw_value > 0xFFFFFFFF:
            return False, toggle, f"报文值超出32bit范围: {raw_value}"

        # 5. 构建配置请求帧（使用小端序，与mileage_write一致）
        config_byte = toggle & 0x01
        new_toggle = toggle ^ 0x01
        address_bytes = list(struct.pack('<H', param_address))  # 小端序
        reserved = 0x00
        data_bytes = list(struct.pack('<I', raw_value))  # 小端序
        frame_data = [config_byte] + address_bytes + [reserved] + data_bytes

        self._log_async(f"[TX] Param Write: Addr=0x{param_address:02X}, Value={user_value}, Raw=0x{raw_value:08X}, Toggle={config_byte:02X}")

        # 6. 发送并等待响应
        response = self.send_can(
            PROTOCOL.PARAM_CONFIG_ID, frame_data,
            PROTOCOL.FRAME_TYPE_STANDARD,
            rsp_id=PROTOCOL.PARAM_RESP_ID,
            wait_response=True, timeout=PROTOCOL.PARAM_TIMEOUT
        )

        # 7. 验证响应（检查config位和数据回显）
        if response and len(response) >= 8:
            resp_config = response[0]
            resp_data = struct.unpack('<I', bytes(response[4:8]))[0]

            if resp_config == config_byte and resp_data == raw_value:
                self._log_async(f"[RX] Param write successful! Response: Data=0x{resp_data:08X}")
                return True, new_toggle, f"{param_def['name']}写入成功！写入值: {user_value}"
            else:
                # 配置错误或VCU返回本地数据
                error_msg = (
                    f"{param_def['name']}配置失败！\n"
                    f"尝试写入: {user_value}\n"
                    f"VCU返回: {resp_data}"
                )
                self._log_async(f"[RX] {error_msg}")
                return False, new_toggle, error_msg
        else:
            return False, new_toggle, "未收到设备回应或超时"

    def param_read(self, param_address, toggle):
        """
        读取VCU参数

        Args:
            param_address: 参数地址 (0x01-0x0F)
            toggle: 当前toggle值 (bit0取反)

        Returns:
            (success: bool, new_toggle: int, value: int/float, message: str)
        """
        # 1. 验证参数地址
        if param_address not in PROTOCOL.VCU_PARAMS:
            return False, toggle, None, f"未知的参数地址: 0x{param_address:02X}"

        param_def = PROTOCOL.VCU_PARAMS[param_address]

        # 2. 构建读取请求帧
        # config_byte: bit0取反, bit1=1(读取)
        config_byte = (toggle & 0x01) | 0x02  # bit1=1表示读取
        new_toggle = toggle ^ 0x01

        address_bytes = list(struct.pack('<H', param_address))  # 小端序
        reserved = 0x00
        data_bytes = [0x00, 0x00, 0x00, 0x00]  # 读取时数据字段填充0
        frame_data = [config_byte] + address_bytes + [reserved] + data_bytes

        self._log_async(f"[TX] Param Read: Addr=0x{param_address:02X}, Config=0x{config_byte:02X}")

        # 3. 发送并等待响应
        response = self.send_can(
            PROTOCOL.PARAM_CONFIG_ID, frame_data,
            PROTOCOL.FRAME_TYPE_STANDARD,
            rsp_id=PROTOCOL.PARAM_RESP_ID,
            wait_response=True, timeout=PROTOCOL.PARAM_TIMEOUT
        )

        # 4. 验证响应
        if response and len(response) >= 8:
            resp_config = response[0]
            resp_addr = struct.unpack('<H', bytes(response[1:3]))[0]
            resp_data = struct.unpack('<I', bytes(response[4:8]))[0]

            # 检查config位和地址匹配
            if resp_config == config_byte and resp_addr == param_address:
                # 应用缩放系数
                if param_def["scale"] != 1.0:
                    display_value = resp_data * param_def["scale"]
                else:
                    display_value = resp_data

                self._log_async(f"[RX] Param read successful! Addr=0x{param_address:02X}, Data=0x{resp_data:08X}, Value={display_value}")
                return True, new_toggle, display_value, f"{param_def['name']}读取成功！值: {display_value}"
            else:
                error_msg = f"参数读取失败！配置位或地址不匹配"
                self._log_async(f"[RX] {error_msg}")
                return False, new_toggle, None, error_msg
        else:
            return False, new_toggle, None, "未收到设备回应或超时"

    # --- OTA 功能方法 ---

    def ota_start(self, firmware_data, version_str, device_name, progress_callback=None):
        """
        OTA 升级流程

        Args:
            firmware_data: 固件数据 (bytes)
            version_str: 目标版本号字符串
            device_name: 目标设备名称
            progress_callback: 进度回调函数 (可选)

        Returns:
            (success: bool, message: str)
        """
        from datetime import datetime
        start_time = datetime.now()
        start_timestamp = start_time.strftime("%Y-%m-%d %H:%M:%S")

        self._log_async(f"\n{'='*60}")
        self._log_async(f"OTA Process Started at: {start_timestamp}")
        self._log_async(f"{'='*60}\n")

        # 检查固件数据
        if not firmware_data:
            return False, "No firmware file selected"

        # 解析版本号
        try:
            version_str = version_str.strip()
            if version_str.startswith('0x') or version_str.startswith('0X'):
                version = int(version_str, 16)
            else:
                version = int(version_str)
            self._log_async(f"Target firmware version: 0x{version:08X}")
        except ValueError:
            self._log_async(f"Invalid version format: {version_str}. Using default 0x0000000F.")
            version = 0x0000000F

        # 计算CRC32
        bin_crc32 = self._crc32_calc(0x04C11DB7, firmware_data, len(firmware_data))
        file_size = len(firmware_data)

        # 获取设备前缀（使用协议配置）
        device_prefix = PROTOCOL.DEVICE_PREFIX_MAP.get(device_name.upper(), 0x42)
        self._log_async(f"Target device: {device_name} (prefix: 0x{device_prefix:02X})")

        # 协议常量
        step_request = 0x00
        step_data = 0x01
        step_complete = 0x02

        # Step 1: Request Upgrade
        self._log_async("\n--- Step 1: Requesting Upgrade ---")
        request_id = self._build_can_id(device_prefix, step_request, 0)
        request_data = list(struct.pack('>I', version)) + list(struct.pack('>I', file_size))

        response = self.send_can(request_id, request_data, extended_id=True, timeout=5)
        success, reason = self._check_ota_response(response, "Step 1")
        if not success:
            return False, f"Step 1 Failed: {reason}"

        self._log_async("Step 1 Successful. Target ECU agreed to upgrade.")

        # Step 2: Send Firmware Data
        self._log_async("\n--- Step 2: Sending Firmware Data ---")
        frame_size = 8
        total_frames = (file_size + frame_size - 1) // frame_size
        current_offset = 0

        for i in range(total_frames):
            chunk = firmware_data[i * frame_size : (i + 1) * frame_size]
            if len(chunk) < frame_size:
                chunk += b'\xFF' * (frame_size - len(chunk))

            # Build CAN ID with current offset
            data_id = self._build_can_id(device_prefix, step_data, current_offset)
            data_payload = list(chunk)

            # 发送并等待响应
            response = self.send_can(data_id, data_payload, extended_id=True, timeout=5)

            progress = ((i + 1) / total_frames) * 100
            self._log_async(f"Sent Frame {i+1}/{total_frames} ({progress:.1f}%), Offset: 0x{current_offset:08X}, Size: {len(chunk)}")

            success, reason = self._check_ota_response(response, f"Step 2 (Frame {i+1})")
            if not success:
                return False, f"Step 2 Failed at frame {i+1}: {reason}"

            current_offset += 1  # 偏移量改为发送帧数

            # 进度回调
            if progress_callback:
                progress_callback(i + 1, total_frames, progress)

        self._log_async("Step 2 Successful. All firmware data frames sent.")

        # Step 3: Send Complete Frame
        self._log_async("\n--- Step 3: Sending Complete Frame ---")
        complete_id = self._build_can_id(device_prefix, step_complete, current_offset)
        complete_data = list(struct.pack('>I', bin_crc32))

        response = self.send_can(complete_id, complete_data, extended_id=True, timeout=5)
        success, reason = self._check_ota_response(response, "Step 3")
        if not success:
            return False, f"Step 3 Failed: {reason}"

        self._log_async("Step 3 Successful. Target ECU accepted final CRC and started new firmware.")
        self._log_async("OTA Update COMPLETED SUCCESSFULLY!")

        # 记录结束时间
        end_time = datetime.now()
        end_timestamp = end_time.strftime("%Y-%m-%d %H:%M:%S")
        elapsed_time = end_time - start_time
        total_seconds = elapsed_time.total_seconds()

        hours = int(total_seconds // 3600)
        minutes = int((total_seconds % 3600) // 60)
        seconds = total_seconds % 60

        self._log_async(f"\n{'='*60}")
        self._log_async(f"OTA Process Finished at: {end_timestamp}")
        self._log_async(f"{'='*60}")
        self._log_async(f"Start Time:    {start_timestamp}")
        self._log_async(f"End Time:      {end_timestamp}")
        self._log_async(f"Elapsed Time:  {elapsed_time}")
        if hours > 0:
            self._log_async(f"              ({hours}h {minutes}m {seconds:.3f}s)")
        elif minutes > 0:
            self._log_async(f"              ({minutes}m {seconds:.3f}s)")
        else:
            self._log_async(f"              ({seconds:.3f}s)")
        self._log_async(f"{'='*60}\n")

        return True, "OTA Update Completed Successfully"

    def _build_can_id(self, device_prefix, step_flag, offset=0):
        """Builds CAN ID according to OTA protocol."""
        offset_masked = offset & 0x7FFFF
        can_id = ((device_prefix & 0xFF) << 21) | ((offset_masked & 0x7FFFF) << 2) | (step_flag & 0x03)
        return can_id

    def _check_ota_response(self, response, step_name):
        """
        Checks if the response from target ECU indicates success.

        Returns:
            (success: bool, message: str)
        """
        if response is None:
            self._log_async(f"{step_name} Failed. No response from target ECU.")
            return False, "No response from target ECU"

        if len(response) < 2:
            self._log_async(f"{step_name} Failed. Response too short: {list(response)}")
            return False, f"Response too short: {list(response)}"

        if response[0] == 0xAA and response[1] == 0x00:
            return True, "Success"
        elif response[0] == 0x55:
            reason = response[1] if len(response) > 1 else 0xFF
            self._log_async(f"{step_name} Failed. Target ECU rejected request. Reason Code: 0x{reason:02X}")
            return False, f"ECU rejected, Reason Code: 0x{reason:02X}"
        else:
            self._log_async(f"{step_name} Failed. Unexpected response: {list(response)[:2]}")
            return False, f"Unexpected response: {list(response)[:2]}"

    def _crc32_calc(self, poly, data, data_len):
        """计算CRC32"""
        crc = 0xffffffff
        for i in range(data_len):
            crc ^= data[i] << 24
            for j in range(8):
                if crc & 0x80000000:
                    crc = (crc << 1) ^ poly
                else:
                    crc <<= 1
                crc &= 0xFFFFFFFF
        return crc


class OTAUpdaterGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("RQ TBOX OTA Updater (Simulated T-Box)")
        self.root.geometry("1100x900") # 增加宽度以适应左右分栏布局

        # --- Configuration Variables ---
        self.selected_interface = tk.StringVar(value="Not Selected")
        self.selected_bitrate = tk.StringVar(value="250000")
        self.selected_device = tk.StringVar(value="VCU") # Default to VCU
        self.firmware_version = tk.StringVar(value="0x0000000F") # Firmware version to upgrade to
        self.auto_silence_enabled = tk.BooleanVar(value=False)
        self.silence_state = tk.BooleanVar(value=False) # Tracks if bus is silenced
        self.ota_in_progress = False # Tracks if an OTA update is currently running

        # --- CAN状态管理 ---
        self.can_initialized = False  # 追踪CAN是否已手动打开
        self.log_path_var = tk.StringVar()  # 日志保存路径变量
        self.mileage_write_toggle = 0x00  # 里程数写入请求计数器（bit0取反）
        self.param_write_toggle = 0x00  # 参数配置toggle状态

        # --- BackendAPI ---
        self.backend = None  # BackendAPI实例（在打开CAN时初始化）

        # 设置广播消息轮询
        self._setup_broadcast_polling()

        # --- GUI Elements ---

        # 配置列权重 (左右分栏布局)
        self.root.grid_columnconfigure(0, weight=4)  # 左侧功能区
        self.root.grid_columnconfigure(1, weight=2)  # 右侧参数区

        # --- File Selection Frame ---
        file_frame = ttk.LabelFrame(self.root, text="Firmware File", padding=(10, 5))
        file_frame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=10, pady=5, ipadx=5, ipady=5)
        file_frame.grid_columnconfigure(1, weight=1)

        self.file_path_var = tk.StringVar()
        button_browse = tk.Button(file_frame, text="Browse", command=self.browse_file)
        self.entry_path_display = tk.Entry(file_frame, textvariable=self.file_path_var, state="normal") # Made normal for editing

        button_browse.grid(row=0, column=0, padx=(0, 5), pady=2)
        self.entry_path_display.grid(row=0, column=1, sticky="ew", padx=(0, 5), pady=2)
        file_frame.grid_columnconfigure(1, weight=1) # Make entry expandable


        # --- CAN & Device Configuration Frame ---
        config_frame = ttk.LabelFrame(self.root, text="CAN & Target Device Configuration", padding=(10, 5))
        config_frame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=10, pady=5, ipadx=5, ipady=5)
        config_frame.grid_columnconfigure(1, weight=1)
        config_frame.grid_columnconfigure(3, weight=1)
        config_frame.grid_columnconfigure(4, weight=1)

        label_iface = tk.Label(config_frame, text="CAN Interface:")
        self.combo_iface = ttk.Combobox(config_frame, textvariable=self.selected_interface, state="readonly")
        label_bitrate = tk.Label(config_frame, text="CAN Bitrate:")
        self.combo_bitrate = ttk.Combobox(config_frame, textvariable=self.selected_bitrate, values=["125000", "250000", "500000", "1000000"], state="readonly")
        label_device = tk.Label(config_frame, text="Target Device:")
        self.combo_device = ttk.Combobox(config_frame, textvariable=self.selected_device, values=["BMS", "VCU", "MCU", "IC", "ABS", "T-Box"], state="readonly")

        label_iface.grid(row=0, column=0, sticky="ew", padx=5, pady=2)
        self.combo_iface.grid(row=0, column=1, padx=5, pady=2)

        # 添加刷新按钮
        button_refresh_iface = tk.Button(config_frame, text="Refresh", command=self.refresh_interfaces)
        button_refresh_iface.grid(row=0, column=2, sticky="ew", padx=5, pady=2)

        label_bitrate.grid(row=0, column=3, sticky="ew", padx=5, pady=2)
        self.combo_bitrate.grid(row=0, column=4, padx=5, pady=2)
        label_device.grid(row=1, column=0, sticky="ew", padx=5, pady=2)
        self.combo_device.grid(row=1, column=1, padx=5, pady=2)
        label_version = tk.Label(config_frame, text="Firmware Version:")
        self.entry_version = tk.Entry(config_frame, textvariable=self.firmware_version, width=15)
        label_version.grid(row=1, column=2, sticky="ew", padx=5, pady=2)
        self.entry_version.grid(row=1, column=3, padx=5, pady=2)
        config_frame.grid_columnconfigure(1, weight=1)
        config_frame.grid_columnconfigure(3, weight=1)
        config_frame.grid_columnconfigure(4, weight=1)


        # 创建左右分栏面板
        # 左侧面板 - 功能控制区
        left_panel = tk.Frame(self.root)
        left_panel.grid(row=2, column=0, rowspan=5, sticky="nsew", padx=5, pady=5)

        # 右侧面板 - 参数配置区
        right_panel = tk.Frame(self.root)
        right_panel.grid(row=2, column=1, rowspan=5, sticky="nsew", padx=5, pady=5)


        # --- Silence Controls Frame (移到左侧面板) ---
        silence_frame = ttk.LabelFrame(left_panel, text="Bus Silence Control", padding=(10, 5))
        silence_frame.pack(fill=tk.X, pady=5)

        self.check_auto_silence = tk.Checkbutton(silence_frame, text="Auto Silence During OTA", variable=self.auto_silence_enabled, command=self.on_auto_silence_change)
        self.button_silence = tk.Button(silence_frame, text="Enter Silence Mode", command=self.enter_silence_mode)
        self.button_unsilence = tk.Button(silence_frame, text="Exit Silence Mode", command=self.exit_silence_mode)

        self.check_auto_silence.grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.button_silence.grid(row=0, column=1, padx=5, pady=5)
        self.button_unsilence.grid(row=0, column=2, padx=5, pady=5)
        silence_frame.grid_columnconfigure(1, weight=1)
        silence_frame.grid_columnconfigure(2, weight=1)


        # --- VIN Code Frame (移到左侧面板) ---
        vin_frame = ttk.LabelFrame(left_panel, text="VIN Code Operations", padding=(10, 5))
        vin_frame.pack(fill=tk.X, pady=5)

        self.vin_code = tk.StringVar(value="")
        label_vin = tk.Label(vin_frame, text="VIN Code (17 chars):")
        self.entry_vin = tk.Entry(vin_frame, textvariable=self.vin_code, width=30)
        self.button_write_vin = tk.Button(vin_frame, text="Write VIN", command=self.write_vin, width=12)
        self.button_read_vin = tk.Button(vin_frame, text="Read VIN", command=self.read_vin, width=12)

        label_vin.grid(row=0, column=0, sticky="ew", padx=5, pady=2)
        self.entry_vin.grid(row=0, column=1, sticky="ew", padx=5, pady=2)
        self.button_write_vin.grid(row=0, column=2, padx=5, pady=2)
        self.button_read_vin.grid(row=0, column=3, padx=5, pady=2)
        vin_frame.grid_columnconfigure(1, weight=1)


        # --- Mileage Frame (移到左侧面板) ---
        mileage_frame = ttk.LabelFrame(left_panel, text="Mileage Operations", padding=(10, 5))
        mileage_frame.pack(fill=tk.X, pady=5)

        self.mileage_write_value = tk.StringVar(value="")
        self.mileage_read_value = tk.StringVar(value="--")

        label_mileage_write = tk.Label(mileage_frame, text="Mileage (Write):")
        self.entry_mileage_write = tk.Entry(mileage_frame, textvariable=self.mileage_write_value, width=15)
        self.button_write_mileage = tk.Button(mileage_frame, text="Write Mileage", command=self.write_mileage, width=12)

        label_mileage_read = tk.Label(mileage_frame, text="Mileage (Auto-Read):")
        self.entry_mileage_read = tk.Entry(mileage_frame, textvariable=self.mileage_read_value, width=15, state="readonly")

        label_mileage_write.grid(row=0, column=0, sticky="ew", padx=5, pady=2)
        self.entry_mileage_write.grid(row=0, column=1, sticky="ew", padx=5, pady=2)
        self.button_write_mileage.grid(row=0, column=2, padx=5, pady=2)

        label_mileage_read.grid(row=0, column=3, sticky="ew", padx=5, pady=2)
        self.entry_mileage_read.grid(row=0, column=4, sticky="ew", padx=5, pady=2)

        mileage_frame.grid_columnconfigure(1, weight=1)
        mileage_frame.grid_columnconfigure(4, weight=1)


        # --- 参数读取控制框架 (在右侧面板顶部) ---
        param_control_frame = ttk.LabelFrame(right_panel, text="参数读取控制", padding=(10, 5))
        param_control_frame.pack(fill=tk.X, padx=5, pady=5)

        # 读取周期设置
        self.param_read_interval = tk.IntVar(value=100)  # 默认100ms
        self.param_write_toggle = 0  # 参数读写toggle状态
        self.param_read_running = False  # 参数读取是否正在运行

        interval_frame = tk.Frame(param_control_frame)
        interval_frame.pack(side=tk.LEFT, padx=5)

        tk.Label(interval_frame, text="读取周期:").pack(side=tk.LEFT, padx=2)
        spinbox_interval = tk.Spinbox(
            interval_frame,
            from_=50, to=1000, increment=50,
            textvariable=self.param_read_interval,
            width=8
        )
        spinbox_interval.pack(side=tk.LEFT, padx=2)
        tk.Label(interval_frame, text="ms").pack(side=tk.LEFT, padx=2)

        # 读取所有参数按钮
        self.button_read_all = tk.Button(
            param_control_frame,
            text="读取所有参数",
            command=self.read_all_parameters,
            bg="#4CAF50",
            fg="white",
            width=15
        )
        self.button_read_all.pack(side=tk.RIGHT, padx=5)


        # --- VCU参数配置Frame (移到右侧面板) ---
        param_frame = ttk.LabelFrame(right_panel, text="VCU参数配置", padding=(10, 5))
        param_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 使用ttk.Treeview创建表格
        param_columns = ("address", "name", "value", "unit")
        param_tree = ttk.Treeview(param_frame, columns=param_columns, show="headings", height=8)

        # 定义列标题
        param_tree.heading("address", text="地址")
        param_tree.heading("name", text="参数名称")
        param_tree.heading("value", text="当前值")
        param_tree.heading("unit", text="单位")

        # 设置列宽
        param_tree.column("address", width=60, anchor="center")
        param_tree.column("name", width=250, anchor="w")
        param_tree.column("value", width=120, anchor="center")
        param_tree.column("unit", width=60, anchor="center")

        # 添加滚动条
        param_scrollbar = ttk.Scrollbar(param_frame, orient=tk.VERTICAL, command=param_tree.yview)
        param_tree.configure(yscrollcommand=param_scrollbar.set)

        param_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        param_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 绑定双击事件
        param_tree.bind("<Double-1>", self.on_param_double_click)

        # 保存引用
        self.param_tree = param_tree

        # 初始化参数表格
        self._populate_param_table()


        # --- Main Action Buttons (移到左侧面板) ---
        button_frame = tk.Frame(left_panel)
        button_frame.pack(fill=tk.X, pady=10)

        self.button_open_can = tk.Button(button_frame, text="Open CAN", command=self.open_can_manual)
        self.button_close_can = tk.Button(button_frame, text="Close CAN", command=self.close_can_manual, state="disabled")
        self.button_start = tk.Button(button_frame, text="Start OTA Update", command=self.start_update_thread)
        self.button_stop = tk.Button(button_frame, text="Stop/Reset CAN", command=self.stop_can)
        self.button_save_log = tk.Button(button_frame, text="Save Log", command=self.save_log)

        self.button_open_can.pack(side=tk.LEFT, padx=5)
        self.button_close_can.pack(side=tk.LEFT, padx=5)
        self.button_start.pack(side=tk.LEFT, padx=5)
        self.button_stop.pack(side=tk.LEFT, padx=5)
        self.button_save_log.pack(side=tk.LEFT, padx=5)


        # --- Log Path Frame ---
        log_path_frame = ttk.LabelFrame(self.root, text="Log Save Path", padding=(10, 5))
        log_path_frame.grid(row=7, column=0, columnspan=2, sticky="ew", padx=10, pady=5, ipadx=5, ipady=5)
        log_path_frame.grid_columnconfigure(1, weight=1)

        label_log_path = tk.Label(log_path_frame, text="Default Path:")
        self.entry_log_path = tk.Entry(log_path_frame, textvariable=self.log_path_var, state="normal")
        button_browse_log = tk.Button(log_path_frame, text="Browse", command=self.browse_log_path)

        label_log_path.grid(row=0, column=0, sticky="ew", padx=5, pady=2)
        self.entry_log_path.grid(row=0, column=1, sticky="ew", padx=5, pady=2)
        button_browse_log.grid(row=0, column=2, padx=5, pady=2)
        log_path_frame.grid_columnconfigure(1, weight=1)

        # 设置默认路径
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_log_path = os.path.abspath(f"./OTA_Log_{timestamp}.txt")
        self.log_path_var.set(default_log_path)


        # --- Status Text Box ---
        status_frame = tk.Frame(self.root)
        status_frame.grid(row=8, column=0, columnspan=2, padx=10, pady=10, sticky="nsew")
        self.root.grid_rowconfigure(8, weight=1)

        self.status_text = tk.Text(status_frame, height=20, width=80)
        scrollbar = tk.Scrollbar(status_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        self.status_text.configure(yscrollcommand=scrollbar.set)

        self.status_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)


        # Populate CAN interfaces
        self.populate_interfaces()

        # --- CAN and OTA Variables ---
        self.can_bus = None
        self.firmware_data = b''

        # Silence Commands 使用协议配置（不再需要硬编码）

    # --- 广播消息轮询 ---

    def _setup_broadcast_polling(self):
        """设置定期检查broadcast_queue"""
        def check_broadcast():
            if self.backend:
                msg = self.backend.get_broadcast_message()
                if msg:
                    self._handle_broadcast_message(msg)
            self.root.after(100, check_broadcast)  # 每100ms检查一次
        check_broadcast()

    def _handle_broadcast_message(self, msg):
        """处理后端发来的广播消息"""
        if msg['type'] == 'mileage':
            mileage_value = msg['data']
            self.mileage_read_value.set(f"{mileage_value} (0x{mileage_value:06X})")

    # --- GUI 辅助方法 ---

    def populate_interfaces(self):
        """Populates the CAN interface combobox with available interfaces, including ZLG."""
        try:
            configs = can.detect_available_configs(interfaces=None)
            available_channels = []
            for config in configs:
                interface_name = config.get('interface', 'Unknown')
                channel = config.get('channel', 'N/A')
                bitrate = config.get('bitrate', 'N/A')
                description = f"{interface_name} - {channel} (Bitrate: {bitrate})"
                available_channels.append(description)

            # 添加 ZLG USBCAN2 选项
            if platform.system() == "Windows":
                available_channels.append("zlgcan - USBCAN2 (Channel 0)")
                available_channels.append("zlgcan - USBCAN2 (Channel 1)")

            if available_channels:
                self.combo_iface['values'] = available_channels
                self.selected_interface.set(available_channels[0])
            else:
                fallback_channels = []
                system = platform.system()
                if system == "Linux":
                    fallback_channels.extend(["socketcan - can0", "socketcan - can1", "virtual - test"])
                elif system == "Windows":
                    fallback_channels.extend(["pcan - PCAN_USBBUS1", "pcan - PCAN_USBBUS2", "virtual - test"])
                    fallback_channels.append("zlgcan - USBCAN2 (Channel 0)")
                else:
                    fallback_channels.append("virtual - test")

                self.combo_iface['values'] = fallback_channels
                if fallback_channels:
                    self.selected_interface.set(fallback_channels[0])

                self.log_status("Could not auto-detect CAN interfaces. Showing common/default options. Please select the correct one for your hardware.")
        except Exception as e:
            fallback_channels = ["virtual - test"]
            if platform.system() == "Windows":
                fallback_channels.append("zlgcan - USBCAN2 (Channel 0)")
            self.combo_iface['values'] = fallback_channels
            self.selected_interface.set(fallback_channels[0])
            self.log_status(f"Failed to detect CAN interfaces: {e}. Using default option: {fallback_channels[0]}")

    def refresh_interfaces(self):
        """刷新CAN接口列表"""
        self.log_status("Refreshing CAN interfaces...")
        self.populate_interfaces()
        self.log_status(f"Found {len(self.combo_iface['values'])} interface(s).")

    def log_status(self, message):
        """Appends a message to the status text box."""
        self.status_text.insert(tk.END, message + "\n")
        self.status_text.see(tk.END)
        self.root.update_idletasks()

    def _crc32_calc(self, poly, data, data_len):
        """计算CRC32"""
        crc = 0xffffffff
        for i in range(data_len):
            crc ^= data[i] << 24
            for j in range(8):
                if crc & 0x80000000:
                    crc = (crc << 1) ^ poly
                else:
                    crc <<= 1
                crc &= 0xFFFFFFFF
        return crc

    def browse_file(self):
        """Opens a file dialog to select the firmware .bin file."""
        filename = filedialog.askopenfilename(
            title="Select Firmware File",
            filetypes=(("Binary files", "*.bin"), ("All files", "*.*"))
        )
        if filename:
            abs_path = os.path.abspath(filename)
            self.file_path_var.set(abs_path)
            self.log_status(f"Selected file: {abs_path}")
            try:
                with open(filename, 'rb') as f:
                    self.firmware_data = f.read()
                    self.bin_crc32 = self._crc32_calc(0x04C11DB7, self.firmware_data, len(self.firmware_data))
                    file_size = len(self.firmware_data)
                    self.log_status(f"Firmware size: {file_size} bytes ({file_size/1024:.2f} KB), CRC32: 0x{self.bin_crc32:08X}")

                    # 每隔4K数据打印一次 CRC32 计算值
                    self.log_status(f"每隔4K数据打印一次 CRC32 计算值, 方便调试")

                    for offset in range(0, len(self.firmware_data), 4096):
                        chunk = self.firmware_data[0:offset+4096]
                        chunk_crc32 = self._crc32_calc(0x04C11DB7, chunk, len(chunk))
                        self.log_status(f"Block 0x{offset:06X} (size={len(chunk)}): CRC32 = 0x{chunk_crc32:08X}")

                    # Check file size limit (bit20-bit2 = 19 bits, max offset = 2^19 - 1 = 524287 bytes)
                    max_size = 524287  # ~512 KB
                    if file_size > max_size:
                        warn_msg = (f"Warning: Firmware file size ({file_size} bytes = {file_size/1024:.2f} KB) "
                                  f"exceeds protocol limit ({max_size} bytes = {max_size/1024:.2f} KB).\n\n"
                                  f"The OTA protocol uses 19 bits for offset (bit20-bit2), which can address "
                                  f"a maximum of {max_size} bytes.\n\n"
                                  f"Continue anyway?")
                        response = messagebox.askyesno("File Size Warning", warn_msg, icon='warning')
                        if not response:
                            self.file_path_var.set("")
                            self.firmware_data = b''
                            self.log_status("File selection cancelled due to size limit.")
                            return
            except Exception as e:
                messagebox.showerror("Error", f"Failed to read file:\n{e}")
                self.file_path_var.set("")
                self.firmware_data = b''

    def browse_log_path(self):
        """打开文件选择对话框，让用户选择日志保存路径"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_filename = f"OTA_Log_{timestamp}.txt"

        filename = filedialog.asksaveasfilename(
            title="Select Log Save Path",
            initialfile=default_filename,
            defaultextension=".txt",
            filetypes=(("Text files", "*.txt"), ("All files", "*.*"))
        )

        if filename:
            self.log_path_var.set(filename)
            self.log_status(f"Log save path set to: {filename}")

    def save_log(self):
        """Saves the current log content to a file."""
        log_content = self.status_text.get("1.0", tk.END)
        if not log_content.strip():
            messagebox.showinfo("Info", "No log content to save.")
            return

        # 获取路径框中的路径
        save_path = self.log_path_var.get().strip()

        # 如果路径框为空，弹出对话框选择路径
        if not save_path:
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            default_filename = f"OTA_Log_{timestamp}.txt"

            filename = filedialog.asksaveasfilename(
                title="Save Log File",
                initialfile=default_filename,
                defaultextension=".txt",
                filetypes=(("Text files", "*.txt"), ("All files", "*.*"))
            )

            if not filename:
                return  # 用户取消保存

            save_path = filename
            self.log_path_var.set(save_path)

        try:
            with open(save_path, 'w', encoding='utf-8') as f:
                f.write(log_content)
            self.log_status(f"Log saved to: {save_path}")
            messagebox.showinfo("Success", f"Log saved successfully to:\n{save_path}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save log:\n{e}")

    def on_auto_silence_change(self):
        """Handles changes to the Auto Silence checkbox."""
        if self.auto_silence_enabled.get():
            self.button_silence.config(state="disabled")
            self.button_unsilence.config(state="disabled")
            self.log_status("Auto Silence enabled. Manual silence controls disabled.")
        else:
            if not self.ota_in_progress:
                self.button_silence.config(state="normal")
                self.button_unsilence.config(state="normal")
            self.log_status("Auto Silence disabled. Manual silence controls enabled.")

    def enter_silence_mode(self):
        """Manually enters silence mode by sending command."""
        if not self.backend:
             self.log_status("提示: 请先打开CAN设备")
             messagebox.showinfo("提示", "请先打开CAN设备\n\n点击'Open CAN'按钮初始化CAN总线。")
             return
        try:
            self.log_status(">>> Manual: Entering Silence Mode")
            self.backend.send_can(PROTOCOL.SILENCE_CMD_ID, PROTOCOL.SILENCE_ENTER_DATA, PROTOCOL.FRAME_TYPE_EXTENDED)
            self.silence_state.set(True)
        except Exception as e:
            self.log_status(f"Error sending silence command: {e}")

    def exit_silence_mode(self):
        """Manually exits silence mode by sending command."""
        if not self.backend:
             self.log_status("提示: CAN总线未打开。无需退出静默模式。")
             return
        try:
            self.log_status(">>> Manual: Exiting Silence Mode")
            self.backend.send_can(PROTOCOL.SILENCE_CMD_ID, PROTOCOL.SILENCE_EXIT_DATA, PROTOCOL.FRAME_TYPE_EXTENDED)
            self.silence_state.set(False)
        except Exception as e:
            self.log_status(f"Error sending unsilence command: {e}")

    def auto_manage_silence(self, enter):
        """Automatically manages silence mode based on OTA state."""
        if not self.backend:
             self.log_status("Error: CAN bus not initialized. Cannot auto-manage silence.")
             return
        try:
            if enter:
                self.log_status(">>> Auto: Entering Silence Mode")
                self.backend.send_can(PROTOCOL.SILENCE_CMD_ID, PROTOCOL.SILENCE_ENTER_DATA, PROTOCOL.FRAME_TYPE_EXTENDED)
                self.silence_state.set(True)
            else:
                self.log_status(">>> Auto: Exiting Silence Mode")
                self.backend.send_can(PROTOCOL.SILENCE_CMD_ID, PROTOCOL.SILENCE_EXIT_DATA, PROTOCOL.FRAME_TYPE_EXTENDED)
                self.silence_state.set(False)
        except Exception as e:
            self.log_status(f"Error during auto silence management: {e}")


    def initialize_can(self):
        """Initializes the CAN bus connection based on user selection."""
        try:
            selected_str = self.selected_interface.get()
            # 解析接口类型和通道
            if selected_str.startswith("zlgcan"):
                # 使用 ZLG 自定义总线
                # 格式: "zlgcan - USBCAN2 (Channel X)"
                channel = 0  # 默认通道0
                if "Channel 1" in selected_str:
                    channel = 1
                bitrate = int(self.selected_bitrate.get())
                self.can_bus = ZlgCanBus(channel=channel, bitrate=bitrate)
                self.log_status(f"ZLG CAN bus initialized: USBCAN2 channel {channel} at {bitrate} baud.")
                return True
            else:
                # 原有 python-can 逻辑
                if " - " in selected_str:
                    parts = selected_str.split(" - ")
                    interface_type = parts[0].strip()
                    channel_part = parts[1].split(" ")[0]
                else:
                    interface_type = selected_str
                    channel_part = "auto_detect_or_default"

                bitrate = int(self.selected_bitrate.get())

                if interface_type.lower() == 'socketcan':
                    channel = channel_part
                    bustype = 'socketcan'
                elif interface_type.lower() == 'pcan':
                    channel = channel_part
                    bustype = 'pcan'
                elif interface_type.lower() == 'virtual':
                    channel = channel_part if channel_part != 'N/A' else 'test'
                    bustype = 'virtual'
                else:
                    channel = channel_part
                    bustype = interface_type.lower()

                self.can_bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=bitrate)
                self.log_status(f"CAN bus initialized: {bustype} on {channel} at {bitrate} baud.")
                return True
        except Exception as e:
            error_msg = f"Failed to initialize CAN bus with {self.selected_interface.get()} at {self.selected_bitrate.get()} baud: {e}"
            self.log_status(error_msg)
            messagebox.showerror("CAN Error", error_msg)
            return False

    def stop_can(self):
        """Stops the CAN bus connection."""
        # 停止 BackendAPI
        if self.backend:
            self.backend.stop()
            self.backend = None

        if self.can_bus:
            try:
                self.can_bus.shutdown()
                self.can_bus = None
                self.log_status("CAN bus stopped.")
                self.silence_state.set(False)
                if not self.auto_silence_enabled.get() and not self.ota_in_progress:
                    self.button_silence.config(state="normal")
                    self.button_unsilence.config(state="normal")
            except Exception as e:
                self.log_status(f"Error stopping CAN bus: {e}")

    def open_can_manual(self):
        """手动打开CAN总线"""
        if self.can_bus:
            self.log_status("CAN bus is already open.")
            return

        success = self.initialize_can()
        if success:
            self.can_initialized = True
            self.button_open_can.config(state="disabled")
            self.button_close_can.config(state="normal")
            self.combo_iface.config(state="disabled")
            self.combo_bitrate.config(state="disabled")
            self.combo_device.config(state="disabled")

            # 创建并启动 BackendAPI
            self.backend = BackendAPI(self.can_bus, self.log_status)
            self.backend.start()

            # 启用silence按钮（如果不在Auto Silence模式）
            if not self.auto_silence_enabled.get():
                self.button_silence.config(state="normal")
                self.button_unsilence.config(state="normal")

    def close_can_manual(self):
        """手动关闭CAN总线"""
        if not self.can_bus:
            self.log_status("CAN bus is not open.")
            return

        self.stop_can()
        self.can_initialized = False
        self.button_open_can.config(state="normal")
        self.button_close_can.config(state="disabled")
        self.combo_iface.config(state="readonly")
        self.combo_bitrate.config(state="readonly")
        self.combo_device.config(state="readonly")

        # 禁用silence按钮
        if not self.auto_silence_enabled.get():
            self.button_silence.config(state="disabled")
            self.button_unsilence.config(state="disabled")

    def run_ota_process(self):
        """Main OTA logic running in a separate thread."""
        self.ota_in_progress = True
        self.button_start.config(state='disabled')
        self.button_open_can.config(state='disabled')
        self.button_close_can.config(state='disabled')
        self.combo_iface.config(state='disabled')
        self.combo_bitrate.config(state='disabled')
        self.combo_device.config(state='disabled')
        self.check_auto_silence.config(state='disabled')

        # Auto silence
        if self.auto_silence_enabled.get():
             self.auto_manage_silence(enter=True)

        try:
            if not self.firmware_data:
                self.log_status("No firmware file selected.")
                return

            # 检查CAN是否已打开，如果未打开则自动打开
            auto_opened = False
            if not self.can_bus:
                self.log_status("CAN bus not initialized. Opening automatically...")
                if not self.initialize_can():
                    self.log_status("Failed to initialize CAN bus. OTA aborted.")
                    return
                # 创建并启动 BackendAPI
                self.backend = BackendAPI(self.can_bus, self.log_status)
                self.backend.start()
                auto_opened = True

            # 调用 BackendAPI 执行 OTA
            success, message = self.backend.ota_start(
                self.firmware_data,
                self.firmware_version.get(),
                self.selected_device.get()
            )

            if not success:
                self.log_status(f"OTA Update FAILED: {message}")
            else:
                self.log_status(message)

        except Exception as e:
            self.log_status(f"An error occurred during OTA: {e}")
        finally:
            # Auto silence
            if self.auto_silence_enabled.get():
                 self.auto_manage_silence(enter=False)

            # 只关闭自动打开的CAN（非手动打开的）
            if auto_opened:
                self.stop_can()

            self.button_start.config(state='normal')
            self.button_open_can.config(state='normal' if not self.can_bus else 'disabled')
            self.button_close_can.config(state='disabled' if not self.can_bus else 'normal')
            self.combo_iface.config(state='readonly')
            self.combo_bitrate.config(state='readonly')
            self.combo_device.config(state='readonly')
            self.check_auto_silence.config(state='normal')

            if self.auto_silence_enabled.get():
                 self.button_silence.config(state="disabled")
                 self.button_unsilence.config(state="disabled")
            else:
                 if self.can_bus:
                     self.button_silence.config(state="normal")
                     self.button_unsilence.config(state="normal")
                 else:
                     self.button_silence.config(state="disabled")
                     self.button_unsilence.config(state="disabled")

            self.ota_in_progress = False


    def start_update_thread(self):
        """Starts the OTA process in a separate thread to prevent GUI freezing."""
        if not self.firmware_data:
             messagebox.showwarning("Warning", "Please select a firmware file first.")
             return
        thread = threading.Thread(target=self.run_ota_process, daemon=True)
        thread.start()

    def write_vin(self):
        """写入 VIN 码到 ECU"""
        # 检查 Backend
        if not self.backend:
            messagebox.showinfo("提示", "请先打开CAN设备")
            return

        vin_str = self.vin_code.get().strip()

        try:
            success, message = self.backend.vin_write(vin_str)
            if success:
                messagebox.showinfo("成功", message)
            else:
                messagebox.showerror("写入失败", message)
        except Exception as e:
            self.log_status(f"Write VIN error: {e}")
            messagebox.showerror("错误", f"VIN写入失败: {e}")

    def read_vin(self):
        """从 ECU 读取 VIN 码"""
        # 检查 Backend
        if not self.backend:
            messagebox.showinfo("提示", "请先打开CAN设备")
            return

        old_vin = self.vin_code.get()  # 保存原有值

        try:
            success, vin_str, message = self.backend.vin_read()
            if success:
                self.vin_code.set(vin_str)
                messagebox.showinfo("成功", f"{message}\n\n{vin_str}")
            else:
                messagebox.showerror("读取失败", f"{message}\n\n(原有值已保留)")
        except Exception as e:
            self.log_status(f"Read VIN error: {e}")
            messagebox.showerror("错误", f"VIN读取失败: {e}")

    def write_mileage(self):
        """写入里程数到 ECU"""
        # 检查 Backend
        if not self.backend:
            messagebox.showinfo("提示", "请先打开CAN设备")
            return

        mileage_str = self.mileage_write_value.get().strip()
        if not mileage_str:
            messagebox.showwarning("输入错误", "请输入里程数!")
            return

        try:
            success, new_toggle, message = self.backend.mileage_write(mileage_str, self.mileage_write_toggle)
            if success:
                self.mileage_write_toggle = new_toggle
                messagebox.showinfo("成功", message)
            else:
                self.mileage_write_toggle = new_toggle
                messagebox.showerror("写入失败", message)
        except Exception as e:
            self.log_status(f"Write mileage error: {e}")
            messagebox.showerror("错误", f"里程数写入失败: {e}")

    def _populate_param_table(self):
        """初始化参数表格"""
        # 清空现有数据
        for item in self.param_tree.get_children():
            self.param_tree.delete(item)

        # 从PROTOCOL获取参数定义
        for addr in sorted(PROTOCOL.VCU_PARAMS.keys()):
            param_def = PROTOCOL.VCU_PARAMS[addr]

            # 插入行
            self.param_tree.insert("", tk.END, values=(
                f"0x{addr:02X}",
                param_def["name"],
                "",  # 初始值为空
                param_def["unit"]
            ))

    def on_param_double_click(self, event):
        """双击参数行时弹出编辑对话框"""
        selection = self.param_tree.selection()
        if not selection:
            return

        item = selection[0]
        values = self.param_tree.item(item, "values")
        addr_str = values[0]
        param_address = int(addr_str, 16)
        param_def = PROTOCOL.VCU_PARAMS[param_address]

        # 创建编辑对话框
        dialog = tk.Toplevel(self.root)
        dialog.title(f"编辑参数 - {param_def['name']}")
        dialog.geometry("400x250")
        dialog.transient(self.root)
        dialog.grab_set()

        # 参数信息
        info_frame = ttk.LabelFrame(dialog, text="参数信息", padding=(10, 5))
        info_frame.pack(fill=tk.X, padx=10, pady=10)

        tk.Label(info_frame, text=f"参数名称: {param_def['name']}").pack(anchor="w")
        tk.Label(info_frame, text=f"参数地址: 0x{param_address:02X}").pack(anchor="w")
        tk.Label(info_frame, text=f"参数单位: {param_def['unit']}").pack(anchor="w")

        if param_def["scale"] != 1.0:
            tk.Label(info_frame, text=f"缩放系数: {param_def['scale']}", foreground="blue").pack(anchor="w")

        # 输入区域
        input_frame = ttk.LabelFrame(dialog, text="输入参数值", padding=(10, 5))
        input_frame.pack(fill=tk.X, padx=10, pady=5)

        tk.Label(input_frame, text=f"请输入 {param_def['name']} ({param_def['type']}):").pack(anchor="w")

        value_var = tk.StringVar()
        entry = tk.Entry(input_frame, textvariable=value_var, width=30)
        entry.pack(fill=tk.X, pady=5)
        entry.focus()

        # 按钮区域
        button_frame = tk.Frame(dialog)
        button_frame.pack(fill=tk.X, padx=10, pady=10)

        def on_write():
            value_str = value_var.get().strip()
            if not value_str:
                messagebox.showwarning("输入错误", "请输入参数值！")
                return

            if not self.backend:
                messagebox.showinfo("提示", "请先打开CAN设备")
                return

            success, new_toggle, message = self.backend.param_write(
                param_address, value_str, self.param_write_toggle
            )

            if success:
                self.param_write_toggle = new_toggle
                messagebox.showinfo("写入成功", message)
                # 更新表格
                current_values = list(self.param_tree.item(item, "values"))
                current_values[2] = value_str
                self.param_tree.item(item, values=current_values)
                dialog.destroy()
            else:
                self.param_write_toggle = new_toggle
                messagebox.showerror("写入失败", message)

        tk.Button(button_frame, text="写入", command=on_write, width=10).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="取消", command=dialog.destroy, width=10).pack(side=tk.LEFT, padx=5)

        entry.bind("<Return>", lambda e: on_write())

    def read_all_parameters(self):
        """读取所有参数按钮点击事件"""
        if not self.backend:
            messagebox.showinfo("提示", "请先打开CAN设备")
            return

        if self.param_read_running:
            # 正在运行，停止读取
            self.param_read_running = False
            # 更新按钮为停止中状态
            self.button_read_all.config(text="正在停止...", state="disabled", bg="#FF9800")
        else:
            # 未运行，开始读取
            self.param_read_running = True
            # 更新按钮为停止状态
            self.button_read_all.config(text="停止读取", bg="#f44336")
            # 启动读取线程
            thread = threading.Thread(target=self._read_all_params_thread, daemon=True)
            thread.start()

    def _read_all_params_thread(self):
        """读取所有参数的线程函数"""
        try:
            param_count = len(PROTOCOL.VCU_PARAMS)
            current_idx = 0

            for addr in sorted(PROTOCOL.VCU_PARAMS.keys()):
                # 检查是否收到停止信号
                if not self.param_read_running:
                    self.log_status("=" * 60)
                    self.log_status("参数读取已手动停止！")
                    break

                current_idx += 1
                param_def = PROTOCOL.VCU_PARAMS[addr]

                self.log_status(f"[{current_idx}/{param_count}] 正在读取 {param_def['name']}...")

                # 调用后端读取
                success, new_toggle, value, message = self.backend.param_read(
                    addr, self.param_write_toggle
                )

                if success:
                    # 更新表格（线程安全的GUI更新）
                    self.root.after(0, self._update_param_table, addr, value)
                    self.log_status(f"  ✓ {param_def['name']} = {value}")
                else:
                    self.log_status(f"  ✗ {param_def['name']} 读取失败: {message}")

                self.param_write_toggle = new_toggle

                # 检查是否收到停止信号（延迟后）
                if not self.param_read_running:
                    self.log_status("=" * 60)
                    self.log_status("参数读取已手动停止！")
                    break

                # 延迟（避免总线拥塞）
                time.sleep(self.param_read_interval.get() / 1000.0)

            if self.param_read_running:
                # 正常完成
                self.log_status("=" * 60)
                self.log_status("所有参数读取完成！")

        except Exception as e:
            self.log_status(f"读取参数时出错: {e}")
        finally:
            # 恢复按钮状态
            self.param_read_running = False
            self.root.after(0, lambda: self.button_read_all.config(
                text="读取所有参数",
                state="normal",
                bg="#4CAF50"
            ))

    def _update_param_table(self, param_addr, value):
        """更新参数表格中的单个值"""
        addr_str = f"0x{param_addr:02X}"
        for item in self.param_tree.get_children():
            values = self.param_tree.item(item, "values")
            if values[0] == addr_str:
                new_values = list(values)
                new_values[2] = str(value)
                self.param_tree.item(item, values=new_values)
                break


if __name__ == "__main__":
    root = tk.Tk()
    app = OTAUpdaterGUI(root)

    # 添加启动说明
    app.log_status("=" * 80)
    app.log_status("RQ TBOX OTA Updater -- MEGMEET")
    app.log_status("=" * 80)
    app.log_status("INFO: Warnings about missing CAN backend drivers during import are NORMAL.")
    app.log_status("      The tool works as long as your specific CAN driver is available.")
    app.log_status("      Common drivers: PCAN (Windows), SocketCAN (Linux), Virtual (test), ZLG USBCAN2 (Windows)")
    app.log_status("=" * 80)
    app.log_status("CRC32 算法参考, C语言同理, 如果需要计算多包数据，需要将上次的CRC32值作为参数传入:")
    app.log_status("def _crc32_calc(self, poly, data, len):")
    app.log_status("    # 多项式：0x04C11DB7 初始值：FFFFFFFF 异或值：00000000")
    app.log_status("    crc = 0xffffffff")
    app.log_status("    for i in range(len):")
    app.log_status("        crc ^= data[i] << 24")
    app.log_status("        for j in range(8):")
    app.log_status("            if crc & 0x80000000:")
    app.log_status("                crc = (crc << 1) ^ poly")
    app.log_status("            else:")
    app.log_status("                crc <<= 1")
    app.log_status("            crc &= 0xFFFFFFFF")
    app.log_status("    return crc")
    app.log_status("=" * 80)
    app.log_status("该版本已集成 ZLG USBCAN2 支持，并修复 DLL 加载问题。")

    root.mainloop()