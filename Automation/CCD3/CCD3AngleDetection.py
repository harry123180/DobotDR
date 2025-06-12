import sys
import os
import time
import threading
import json
import logging
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import cv2
import numpy as np

# PyModbus imports
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

# Flask imports
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

# Import camera manager and angle detection
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'API'))
from camera_manager import OptimizedCamera, CameraConfig

# Import angle detection algorithm
current_dir = os.path.dirname(os.path.abspath(__file__))
opencv_module_path = os.path.join(current_dir, '..', '..')
sys.path.append(opencv_module_path)
from opencv_detect_module import get_obj_angle, get_pre_treatment_image

# 設置logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class StatusBits(Enum):
    READY = 0
    RUNNING = 1
    ALARM = 2
    INITIALIZED = 3

@dataclass
class AngleResult:
    success: bool
    center: Optional[Tuple[int, int]]
    angle: Optional[float]
    major_axis: Optional[float]
    minor_axis: Optional[float]
    rect_width: Optional[float]
    rect_height: Optional[float]
    contour_area: Optional[float]
    processing_time: float
    capture_time: float
    total_time: float
    error_message: Optional[str] = None

class SystemStateMachine:
    def __init__(self):
        self.status_register = 0b0001  # 初始狀態: Ready=1
        self.lock = threading.Lock()
    
    def set_bit(self, bit_pos: StatusBits, value: bool):
        with self.lock:
            if value:
                self.status_register |= (1 << bit_pos.value)
            else:
                self.status_register &= ~(1 << bit_pos.value)
    
    def get_bit(self, bit_pos: StatusBits) -> bool:
        with self.lock:
            return bool(self.status_register & (1 << bit_pos.value))
    
    def is_ready(self) -> bool:
        return self.get_bit(StatusBits.READY)
    
    def is_running(self) -> bool:
        return self.get_bit(StatusBits.RUNNING)
    
    def is_alarm(self) -> bool:
        return self.get_bit(StatusBits.ALARM)
    
    def is_initialized(self) -> bool:
        return self.get_bit(StatusBits.INITIALIZED)
    
    def set_ready(self, ready: bool):
        self.set_bit(StatusBits.READY, ready)
    
    def set_running(self, running: bool):
        self.set_bit(StatusBits.RUNNING, running)
    
    def set_alarm(self, alarm: bool):
        self.set_bit(StatusBits.ALARM, alarm)
    
    def set_initialized(self, initialized: bool):
        self.set_bit(StatusBits.INITIALIZED, initialized)
    
    def reset_to_idle(self):
        with self.lock:
            self.status_register = 0b1001  # Ready=1, Initialized=1

class AngleDetector:
    def __init__(self):
        self.min_area_rate = 0.05
        self.sequence_mode = False
        self.gaussian_kernel = 3
        self.threshold_mode = 0  # 0=OTSU, 1=Manual
        self.manual_threshold = 127
    
    def update_params(self, **kwargs):
        """更新檢測參數"""
        if 'min_area_rate' in kwargs:
            self.min_area_rate = kwargs['min_area_rate'] / 1000.0
        if 'sequence_mode' in kwargs:
            self.sequence_mode = bool(kwargs['sequence_mode'])
        if 'gaussian_kernel' in kwargs:
            self.gaussian_kernel = kwargs['gaussian_kernel']
        if 'threshold_mode' in kwargs:
            self.threshold_mode = kwargs['threshold_mode']
        if 'manual_threshold' in kwargs:
            self.manual_threshold = kwargs['manual_threshold']
    
    def get_pre_treatment_image_enhanced(self, image):
        """增強版影像前處理"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (self.gaussian_kernel, self.gaussian_kernel), 0)
        
        if self.threshold_mode == 0:
            # OTSU自動閾值
            _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        else:
            # 手動閾值
            _, thresh = cv2.threshold(blur, self.manual_threshold, 255, cv2.THRESH_BINARY)
        
        return thresh
    
    def detect_angle(self, image, mode=0) -> AngleResult:
        """
        角度檢測主函數
        mode: 0=橢圓擬合模式, 1=最小外接矩形模式
        """
        start_time = time.time()
        
        try:
            print(f"開始角度檢測，模式: {mode}, 圖像尺寸: {image.shape}")
            
            # 檢查圖像格式並轉換為OpenCV算法所需的BGR格式
            if len(image.shape) == 2:
                # 灰度圖像，轉換為BGR
                print("檢測到灰度圖像，轉換為BGR格式")
                bgr_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            elif len(image.shape) == 3 and image.shape[2] == 1:
                # 單通道圖像，轉換為BGR
                print("檢測到單通道圖像，轉換為BGR格式")
                gray_image = image.squeeze()
                bgr_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
            elif len(image.shape) == 3 and image.shape[2] == 3:
                # 已經是3通道圖像
                print("檢測到3通道圖像，直接使用")
                bgr_image = image
            else:
                raise Exception(f"不支援的圖像格式: {image.shape}")
            
            print(f"轉換後圖像尺寸: {bgr_image.shape}")
            
            # 調用核心算法 - 傳入BGR格式圖像
            result = get_obj_angle(bgr_image.copy(), mode=mode)
            
            if result is None:
                print("角度檢測失敗: 未檢測到有效物體")
                return AngleResult(
                    success=False,
                    center=None,
                    angle=None,
                    major_axis=None,
                    minor_axis=None,
                    rect_width=None,
                    rect_height=None,
                    contour_area=None,
                    processing_time=0,
                    capture_time=0,
                    total_time=time.time() - start_time,
                    error_message="未檢測到有效物體"
                )
            
            center, angle = result
            processing_time = (time.time() - start_time) * 1000
            
            print(f"角度檢測成功: 中心座標({center[0]}, {center[1]}), 角度{angle:.2f}度, 處理時間{processing_time:.2f}ms")
            
            # 從opencv_detect_module.py獲取額外資訊需要增強算法
            # 目前先返回基本結果，後續整合內外徑算法時擴展
            
            return AngleResult(
                success=True,
                center=center,
                angle=angle,
                major_axis=None,  # 待整合
                minor_axis=None,  # 待整合
                rect_width=None,  # 待整合
                rect_height=None, # 待整合
                contour_area=None, # 待整合
                processing_time=processing_time,
                capture_time=0,
                total_time=processing_time
            )
            
        except Exception as e:
            print(f"角度檢測異常: {str(e)}")
            return AngleResult(
                success=False,
                center=None,
                angle=None,
                major_axis=None,
                minor_axis=None,
                rect_width=None,
                rect_height=None,
                contour_area=None,
                processing_time=0,
                capture_time=0,
                total_time=time.time() - start_time,
                error_message=str(e)
            )

class CCD3AngleDetectionService:
    def __init__(self):
        self.base_address = 800
        self.modbus_client = None
        self.server_ip = "127.0.0.1"
        self.server_port = 502
        
        # 組件初始化
        self.state_machine = SystemStateMachine()
        self.angle_detector = AngleDetector()
        self.camera = None
        
        # 控制變量
        self.last_control_command = 0
        self.command_processing = False
        self.handshake_thread = None
        self.stop_handshake = False
        
        # 統計資訊
        self.operation_count = 0
        self.error_count = 0
        self.connection_count = 0
        self.start_time = time.time()
        
        # 配置檔案
        self.config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ccd3_config.json')
        self.load_config()
    
    def load_config(self):
        """載入配置檔案"""
        default_config = {
            "module_id": "CCD3_Angle_Detection",
            "camera_config": {
                "name": "ccd3_camera",
                "ip": "192.168.1.10",
                "exposure_time": 20000.0,
                "gain": 200.0,
                "frame_rate": 30.0,
                "width": 2592,
                "height": 1944
            },
            "tcp_server": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1
            },
            "modbus_mapping": {
                "base_address": 800
            },
            "detection_params": {
                "min_area_rate": 50,
                "sequence_mode": 0,
                "gaussian_kernel": 3,
                "threshold_mode": 0,
                "manual_threshold": 127
            }
        }
        
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
            else:
                config = default_config
                with open(self.config_file, 'w', encoding='utf-8') as f:
                    json.dump(config, f, indent=2, ensure_ascii=False)
            
            # 應用配置
            self.server_ip = config['tcp_server']['host']
            self.server_port = config['tcp_server']['port']
            self.base_address = config['modbus_mapping']['base_address']
            
        except Exception as e:
            print(f"配置檔案載入錯誤: {e}")
    
    def connect_modbus(self) -> bool:
        """連接Modbus TCP服務器"""
        try:
            print("正在連接Modbus TCP服務器...")
            
            if self.modbus_client:
                self.modbus_client.close()
            
            self.modbus_client = ModbusTcpClient(
                host=self.server_ip,
                port=self.server_port,
                timeout=3
            )
            
            if self.modbus_client.connect():
                self.connection_count += 1
                print(f"CCD3角度檢測模組已連接到Modbus服務器: {self.server_ip}:{self.server_port}")
                return True
            else:
                print(f"Modbus連接失敗: 無法連接到 {self.server_ip}:{self.server_port}")
                self.state_machine.set_alarm(True)
                return False
                
        except Exception as e:
            print(f"Modbus連接錯誤: {e}")
            self.state_machine.set_alarm(True)
            return False
    
    def initialize_camera(self, ip_address: str = "192.168.1.10") -> bool:
        """初始化相機"""
        try:
            print(f"正在初始化相機，IP地址: {ip_address}")
            
            if self.camera:
                print("關閉現有相機連接...")
                self.camera.disconnect()
                self.camera = None
            
            # 使用camera_manager.py，需要提供logger參數
            config = CameraConfig(
                name="ccd3_camera",
                ip=ip_address,
                exposure_time=20000.0,
                gain=200.0,
                frame_rate=30.0,
                width=2592,
                height=1944
            )
            
            print(f"相機配置: 曝光時間={config.exposure_time}, 增益={config.gain}, 分辨率={config.width}x{config.height}")
            
            self.camera = OptimizedCamera(config, logger)
            
            print("正在連接相機...")
            if self.camera.connect():
                print(f"CCD3相機已成功連接: {ip_address}")
                
                # 先啟動串流
                print("啟動相機串流...")
                if self.camera.start_streaming():
                    print("相機串流啟動成功")
                    
                    # 測試圖像捕獲能力來驗證相機是否真正可用
                    print("測試相機圖像捕獲能力...")
                    try:
                        test_image = self.camera.capture_frame()
                        if test_image is not None:
                            print(f"相機測試成功，可以捕獲圖像，測試圖像尺寸: {test_image.data.shape}")
                            self.state_machine.set_initialized(True)
                            self.state_machine.set_alarm(False)
                            return True
                        else:
                            print("相機測試失敗: 無法捕獲圖像")
                            self.state_machine.set_alarm(True)
                            self.state_machine.set_initialized(False)
                            return False
                    except Exception as e:
                        print(f"相機測試異常: {e}")
                        self.state_machine.set_alarm(True)
                        self.state_machine.set_initialized(False)
                        return False
                else:
                    print("相機串流啟動失敗")
                    self.state_machine.set_alarm(True)
                    self.state_machine.set_initialized(False)
                    return False
            else:
                print(f"相機連接失敗: {ip_address}")
                self.state_machine.set_alarm(True)
                self.state_machine.set_initialized(False)
                return False
                
        except Exception as e:
            print(f"相機初始化錯誤: {e}")
            self.state_machine.set_alarm(True)
            self.state_machine.set_initialized(False)
            return False
    
    def capture_and_detect_angle(self, mode: int = 0) -> AngleResult:
        """拍照並檢測角度"""
        print(f"開始拍照+角度檢測，檢測模式: {mode}")
        
        if not self.camera:
            print("錯誤: 相機未初始化")
            return AngleResult(
                success=False, center=None, angle=None,
                major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                contour_area=None, processing_time=0, capture_time=0, total_time=0,
                error_message="相機未初始化"
            )
        
        # 檢查相機狀態 - 使用實際捕獲測試而不是device屬性
        if not self.camera:
            print("錯誤: 相機未初始化")
            return AngleResult(
                success=False, center=None, angle=None,
                major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                contour_area=None, processing_time=0, capture_time=0, total_time=0,
                error_message="相機未初始化"
            )
        
        capture_start = time.time()
        
        try:
            # 拍照
            print("正在捕獲圖像...")
            frame_data = self.camera.capture_frame()
            
            if frame_data is None:
                print("錯誤: 圖像捕獲失敗，返回None")
                raise Exception("圖像捕獲失敗")
            
            image = frame_data.data
            capture_time = (time.time() - capture_start) * 1000
            print(f"圖像捕獲成功，耗時: {capture_time:.2f}ms, 圖像尺寸: {image.shape}")
            
            # 更新檢測參數
            detection_params = self.read_detection_parameters()
            if detection_params:
                print(f"檢測參數: {detection_params}")
                self.angle_detector.update_params(**detection_params)
            else:
                print("使用預設檢測參數")
            
            # 角度檢測
            detect_start = time.time()
            print("開始角度檢測...")
            result = self.angle_detector.detect_angle(image, mode)
            result.capture_time = capture_time
            result.total_time = (time.time() - capture_start) * 1000
            
            if result.success:
                self.operation_count += 1
                print(f"角度檢測完成 - 總耗時: {result.total_time:.2f}ms")
            else:
                self.error_count += 1
                print(f"角度檢測失敗: {result.error_message}")
            
            return result
            
        except Exception as e:
            self.error_count += 1
            error_msg = str(e)
            print(f"捕獲或檢測過程異常: {error_msg}")
            return AngleResult(
                success=False, center=None, angle=None,
                major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                contour_area=None, processing_time=0,
                capture_time=(time.time() - capture_start) * 1000,
                total_time=(time.time() - capture_start) * 1000,
                error_message=error_msg
            )
    
    def read_detection_parameters(self) -> Dict[str, Any]:
        """讀取檢測參數寄存器"""
        params = {}
        try:
            if self.modbus_client and self.modbus_client.connected:
                result = self.modbus_client.read_holding_registers(
                    address=self.base_address + 10, count=10, slave=1
                )
                if not result.isError():
                    registers = result.registers
                    params['detection_mode'] = registers[0]
                    params['min_area_rate'] = registers[1]
                    params['sequence_mode'] = registers[2]
                    params['gaussian_kernel'] = registers[3]
                    params['threshold_mode'] = registers[4]
                    params['manual_threshold'] = registers[5]
        except Exception as e:
            print(f"讀取檢測參數錯誤: {e}")
        
        return params
    
    def write_detection_result(self, result: AngleResult):
        """寫入檢測結果到寄存器"""
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                print("警告: Modbus未連接，無法寫入檢測結果")
                return
            
            # 檢測結果寄存器 (840-859)
            result_registers = [0] * 20
            
            if result.success and result.center and result.angle is not None:
                result_registers[0] = 1  # 檢測成功標誌
                result_registers[1] = int(result.center[0])  # 中心X座標
                result_registers[2] = int(result.center[1])  # 中心Y座標
                
                # 角度32位存儲 (高低位)
                angle_int = int(result.angle * 100)  # 保留2位小數
                result_registers[3] = (angle_int >> 16) & 0xFFFF  # 角度高位
                result_registers[4] = angle_int & 0xFFFF          # 角度低位
                
                print(f"寫入檢測結果: 成功標誌=1, 中心=({result_registers[1]}, {result_registers[2]}), 角度={result.angle:.2f}度")
                
                # 其他參數 (待整合內外徑算法時實現)
                if result.major_axis:
                    result_registers[5] = int(result.major_axis)
                if result.minor_axis:
                    result_registers[6] = int(result.minor_axis)
                if result.rect_width:
                    result_registers[7] = int(result.rect_width)
                if result.rect_height:
                    result_registers[8] = int(result.rect_height)
                if result.contour_area:
                    result_registers[9] = int(result.contour_area)
            else:
                print("寫入檢測結果: 檢測失敗")
            
            # 寫入檢測結果 (840-859)
            self.modbus_client.write_registers(
                address=self.base_address + 40, values=result_registers, slave=1
            )
            
            # 寫入統計資訊 (880-899)
            stats_registers = [
                int(result.capture_time),      # 880: 拍照耗時
                int(result.processing_time),   # 881: 處理耗時
                int(result.total_time),        # 882: 總耗時
                self.operation_count,          # 883: 操作計數
                self.error_count,              # 884: 錯誤計數
                self.connection_count,         # 885: 連接計數
                0, 0, 0, 0,                   # 886-889: 保留
                3,                            # 890: 軟體版本主號
                0,                            # 891: 軟體版本次號
                int((time.time() - self.start_time) // 3600),  # 892: 運行小時
                int((time.time() - self.start_time) % 3600 // 60), # 893: 運行分鐘
                0, 0, 0, 0, 0, 0             # 894-899: 保留
            ]
            
            self.modbus_client.write_registers(
                address=self.base_address + 80, values=stats_registers, slave=1
            )
            
            print(f"統計資訊已更新: 成功次數={self.operation_count}, 錯誤次數={self.error_count}")
            
        except Exception as e:
            print(f"寫入檢測結果錯誤: {e}")
    
    def _handshake_sync_loop(self):
        """握手同步循環"""
        print("CCD3握手同步線程啟動")
        
        while not self.stop_handshake:
            try:
                if self.modbus_client and self.modbus_client.connected:
                    # 更新狀態寄存器
                    self._update_status_register()
                    
                    # 處理控制指令
                    self._process_control_commands()
                
                time.sleep(0.05)  # 50ms循環
                
            except Exception as e:
                print(f"握手同步錯誤: {e}")
                time.sleep(1)
        
        print("CCD3握手同步線程停止")
    
    def _update_status_register(self):
        """更新狀態寄存器"""
        try:
            # 更新初始化狀態 - 檢查相機串流狀態
            camera_ok = self.camera is not None and getattr(self.camera, 'is_streaming', False)
            modbus_ok = self.modbus_client is not None and self.modbus_client.connected
            
            self.state_machine.set_initialized(camera_ok)
            if not (camera_ok and modbus_ok):
                if not camera_ok:
                    self.state_machine.set_alarm(True)
                    # print("狀態更新: 相機未正確初始化")  # 避免過多輸出
            
            # 寫入狀態寄存器 (801)
            self.modbus_client.write_register(
                address=self.base_address + 1,
                value=self.state_machine.status_register,
                slave=1
            )
            
        except Exception as e:
            print(f"狀態寄存器更新錯誤: {e}")
    
    def _process_control_commands(self):
        """處理控制指令"""
        try:
            # 讀取控制指令 (800)
            result = self.modbus_client.read_holding_registers(
                address=self.base_address, count=1, slave=1
            )
            
            if result.isError():
                return
            
            control_command = result.registers[0]
            
            # 檢查新指令
            if control_command != self.last_control_command and control_command != 0:
                if not self.command_processing:
                    print(f"收到新控制指令: {control_command} (上次: {self.last_control_command})")
                    self._handle_control_command(control_command)
                    self.last_control_command = control_command
            
            # PLC清零指令後恢復Ready
            elif control_command == 0 and self.last_control_command != 0:
                print("PLC已清零指令，恢復Ready狀態")
                self.state_machine.set_ready(True)
                self.last_control_command = 0
                
        except Exception as e:
            print(f"控制指令處理錯誤: {e}")
    
    def _handle_control_command(self, command: int):
        """處理控制指令"""
        if not self.state_machine.is_ready():
            print(f"系統未Ready，無法執行指令 {command}")
            return
        
        print(f"開始處理控制指令: {command}")
        self.command_processing = True
        self.state_machine.set_ready(False)
        self.state_machine.set_running(True)
        
        # 異步執行指令
        threading.Thread(target=self._execute_command_async, args=(command,), daemon=True).start()
    
    def _execute_command_async(self, command: int):
        """異步執行指令"""
        try:
            if command == 8:
                # 單純拍照
                print("執行拍照指令")
                if self.camera and getattr(self.camera, 'is_streaming', False):
                    frame_data = self.camera.capture_frame()
                    if frame_data is not None:
                        print(f"拍照完成，圖像尺寸: {frame_data.data.shape}")
                    else:
                        print("拍照失敗: 無法捕獲圖像")
                        self.error_count += 1
                else:
                    print("拍照失敗: 相機未初始化或串流未啟動")
                    self.error_count += 1
                        
            elif command == 16:
                # 拍照+角度檢測
                print("執行拍照+角度檢測指令")
                
                # 讀取檢測模式 (810)
                mode_result = self.modbus_client.read_holding_registers(
                    address=self.base_address + 10, count=1, slave=1
                )
                detection_mode = 0
                if not mode_result.isError():
                    detection_mode = mode_result.registers[0]
                
                print(f"檢測模式: {detection_mode}")
                
                result = self.capture_and_detect_angle(detection_mode)
                self.write_detection_result(result)
                
                if result.success:
                    print(f"角度檢測完成: 中心{result.center}, 角度{result.angle:.2f}度")
                else:
                    print(f"角度檢測失敗: {result.error_message}")
                    
            elif command == 32:
                # 重新初始化
                print("執行重新初始化指令")
                success = self.initialize_camera()
                if success:
                    print("重新初始化成功")
                else:
                    print("重新初始化失敗")
            
            else:
                print(f"未知指令: {command}")
                
        except Exception as e:
            print(f"指令執行錯誤: {e}")
            self.error_count += 1
            self.state_machine.set_alarm(True)
        
        finally:
            print(f"控制指令 {command} 執行完成")
            self.command_processing = False
            self.state_machine.set_running(False)
    
    def start_handshake_service(self):
        """啟動握手服務"""
        if not self.handshake_thread or not self.handshake_thread.is_alive():
            self.stop_handshake = False
            self.handshake_thread = threading.Thread(target=self._handshake_sync_loop, daemon=True)
            self.handshake_thread.start()
            print("握手服務已啟動")
    
    def stop_handshake_service(self):
        """停止握手服務"""
        print("正在停止握手服務...")
        self.stop_handshake = True
        if self.handshake_thread:
            self.handshake_thread.join(timeout=2)
    
    def disconnect(self):
        """斷開連接"""
        print("正在斷開所有連接...")
        self.stop_handshake_service()
        
        if self.camera:
            print("正在關閉相機連接...")
            # 先停止串流
            if getattr(self.camera, 'is_streaming', False):
                print("停止相機串流...")
                self.camera.stop_streaming()
            # 然後斷開連接
            self.camera.disconnect()
            self.camera = None
        
        if self.modbus_client:
            print("正在關閉Modbus連接...")
            self.modbus_client.close()
            self.modbus_client = None
        
        print("CCD3角度檢測模組已斷開連接")

# Flask Web應用
app = Flask(__name__, template_folder='templates')
app.config['SECRET_KEY'] = 'ccd3_angle_detection_secret'
socketio = SocketIO(app, cors_allowed_origins="*")

# 全局服務實例
ccd3_service = CCD3AngleDetectionService()

@app.route('/')
def index():
    return render_template('ccd3_angle_detection.html')

@app.route('/api/modbus/set_server', methods=['POST'])
def set_modbus_server():
    data = request.json
    ip = data.get('ip', '127.0.0.1')
    port = data.get('port', 502)
    
    ccd3_service.server_ip = ip
    ccd3_service.server_port = port
    
    return jsonify({'success': True, 'message': f'Modbus服務器設置為 {ip}:{port}'})

@app.route('/api/modbus/connect', methods=['POST'])
def connect_modbus():
    success = ccd3_service.connect_modbus()
    if success:
        ccd3_service.start_handshake_service()
        return jsonify({'success': True, 'message': 'Modbus連接成功，握手服務已啟動'})
    else:
        return jsonify({'success': False, 'message': 'Modbus連接失敗'})

@app.route('/api/initialize', methods=['POST'])
def initialize_camera():
    data = request.json
    ip = data.get('ip', '192.168.1.10')
    
    success = ccd3_service.initialize_camera(ip)
    message = f'相機初始化{"成功" if success else "失敗"}'
    
    return jsonify({'success': success, 'message': message})

@app.route('/api/capture_and_detect', methods=['POST'])
def capture_and_detect():
    data = request.json
    mode = data.get('mode', 0)
    
    result = ccd3_service.capture_and_detect_angle(mode)
    
    # 將numpy類型轉換為Python原生類型以支援JSON序列化
    response_data = {
        'success': result.success,
        'center': [int(result.center[0]), int(result.center[1])] if result.center else None,
        'angle': float(result.angle) if result.angle is not None else None,
        'processing_time': float(result.processing_time),
        'capture_time': float(result.capture_time),
        'total_time': float(result.total_time)
    }
    
    if not result.success:
        response_data['error'] = result.error_message
    
    return jsonify(response_data)

@app.route('/api/status', methods=['GET'])
def get_status():
    return jsonify({
        'modbus_connected': ccd3_service.modbus_client and ccd3_service.modbus_client.connected,
        'camera_initialized': ccd3_service.state_machine.is_initialized(),
        'ready': ccd3_service.state_machine.is_ready(),
        'running': ccd3_service.state_machine.is_running(),
        'alarm': ccd3_service.state_machine.is_alarm(),
        'operation_count': ccd3_service.operation_count,
        'error_count': ccd3_service.error_count,
        'connection_count': ccd3_service.connection_count
    })

@app.route('/api/modbus/registers', methods=['GET'])
def get_registers():
    """讀取所有寄存器數值"""
    registers = {}
    
    try:
        if ccd3_service.modbus_client and ccd3_service.modbus_client.connected:
            # 讀取握手寄存器 (800-801)
            result = ccd3_service.modbus_client.read_holding_registers(
                address=ccd3_service.base_address, count=2, slave=1
            )
            if not result.isError():
                registers['control_command'] = result.registers[0]
                registers['status_register'] = result.registers[1]
            
            # 讀取檢測參數 (810-819)
            result = ccd3_service.modbus_client.read_holding_registers(
                address=ccd3_service.base_address + 10, count=10, slave=1
            )
            if not result.isError():
                registers['detection_params'] = result.registers
            
            # 讀取檢測結果 (840-859)
            result = ccd3_service.modbus_client.read_holding_registers(
                address=ccd3_service.base_address + 40, count=20, slave=1
            )
            if not result.isError():
                registers['detection_results'] = result.registers
            
            # 讀取統計資訊 (880-899)
            result = ccd3_service.modbus_client.read_holding_registers(
                address=ccd3_service.base_address + 80, count=20, slave=1
            )
            if not result.isError():
                registers['statistics'] = result.registers
                
    except Exception as e:
        print(f"寄存器讀取錯誤: {e}")
    
    return jsonify(registers)

@socketio.on('connect')
def handle_connect():
    emit('status_update', {'message': 'CCD3角度檢測系統已連接'})

@socketio.on('get_status')
def handle_get_status():
    status = get_status().data
    emit('status_update', status)

if __name__ == '__main__':
    print("CCD3角度辨識系統啟動中...")
    print(f"系統架構: Modbus TCP Client - 運動控制握手模式")
    print(f"基地址: {ccd3_service.base_address}")
    print(f"相機IP: 192.168.1.10")
    print(f"Web介面啟動中... http://localhost:5052")
    
    try:
        socketio.run(app, host='0.0.0.0', port=5052, debug=False)
    except KeyboardInterrupt:
        print("\n正在關閉CCD3角度檢測系統...")
        ccd3_service.disconnect()
    except Exception as e:
        print(f"系統錯誤: {e}")
        ccd3_service.disconnect()