# -*- coding: utf-8 -*-
"""
CCD1VisionCode_Terminal.py - CCD1視覺控制系統 Terminal版本
YOLOv11物件檢測，支援DR_F/STACK分類檢測
Modbus TCP Client架構，Terminal指令控制 + 自動握手功能
適配pymodbus 3.9.2
"""

import sys
import os
import time
import threading
import gc
from typing import Optional, Dict, Any, Tuple, List
import numpy as np
import cv2
import logging
from dataclasses import dataclass
from datetime import datetime
from enum import IntEnum

# 檢查YOLOv11可用性
YOLO_AVAILABLE = False
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
    print("✅ YOLOv11模組導入成功")
except ImportError as e:
    print(f"❌ YOLOv11模組導入失敗: {e}")
    YOLO_AVAILABLE = False

# 導入Modbus TCP Client
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
    print("✅ Modbus Client模組導入成功")
except ImportError as e:
    print(f"❌ Modbus Client模組導入失敗: {e}")
    MODBUS_AVAILABLE = False

# 導入相機管理模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'API'))
try:
    from camera_manager import OptimizedCameraManager, CameraConfig, CameraMode, PixelFormat
    CAMERA_MANAGER_AVAILABLE = True
    print("✅ 相機管理模組導入成功")
except ImportError as e:
    print(f"❌ 無法導入 camera_manager 模組: {e}")
    CAMERA_MANAGER_AVAILABLE = False


# ==================== 枚舉定義 ====================
class ControlCommand(IntEnum):
    CLEAR = 0
    CAPTURE = 8
    CAPTURE_DETECT = 16
    INITIALIZE = 32

class StatusBits(IntEnum):
    READY = 0
    RUNNING = 1
    ALARM = 2
    INITIALIZED = 3


# ==================== 數據結構 ====================
@dataclass
class YOLODetectionResult:
    success: bool = False
    dr_f_count: int = 0
    stack_count: int = 0
    dr_f_coords: List[Tuple[float, float]] = None
    stack_coords: List[Tuple[float, float]] = None
    dr_f_world_coords: List[Tuple[float, float]] = None
    total_detections: int = 0
    confidence_threshold: float = 0.8
    processing_time: float = 0.0
    capture_time: float = 0.0
    total_time: float = 0.0
    timestamp: str = ""
    error_message: Optional[str] = None
    model_id_used: int = 0

    def __post_init__(self):
        if self.dr_f_coords is None:
            self.dr_f_coords = []
        if self.stack_coords is None:
            self.stack_coords = []
        if self.dr_f_world_coords is None:
            self.dr_f_world_coords = []
        if not self.timestamp:
            self.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")


# ==================== YOLO模型管理器 ====================
class YOLOModelManager:
    def __init__(self, working_dir: str):
        self.working_dir = working_dir
        self.models = {}
        self.model_paths = {}
        self.current_model_id = 0
        self.model_switch_count = 0
        self.scan_model_files()
    
    def scan_model_files(self):
        print("🔍 掃描YOLO模型檔案...")
        for i in range(1, 21):
            patterns = [f"model_{i}.pt", f"best_{i}.pt", f"yolo_{i}.pt", f"dr_model_{i}.pt"]
            for pattern in patterns:
                model_path = os.path.join(self.working_dir, pattern)
                if os.path.exists(model_path):
                    self.model_paths[i] = model_path
                    print(f"✅ 發現模型{i}: {pattern}")
                    break
        
        # 檢查best.pt作為模型1的備選
        best_pt_path = os.path.join(self.working_dir, "best.pt")
        if os.path.exists(best_pt_path) and 1 not in self.model_paths:
            self.model_paths[1] = best_pt_path
            print(f"✅ 將best.pt指定為模型1")
        
        print(f"📊 總共發現 {len(self.model_paths)} 個模型檔案")
    
    def load_model(self, model_id: int) -> bool:
        try:
            if model_id == 0:
                self.current_model_id = 0
                print("🔄 卸載當前模型")
                return True
            
            if model_id < 1 or model_id > 20 or model_id not in self.model_paths:
                print(f"❌ 模型{model_id}不存在或ID超出範圍")
                return False
            
            print(f"🔄 載入模型{model_id}: {self.model_paths[model_id]}")
            
            if YOLO_AVAILABLE:
                model = YOLO(self.model_paths[model_id])
                self.models[model_id] = model
                self.current_model_id = model_id
                self.model_switch_count += 1
                print(f"✅ 模型{model_id}載入成功")
                return True
            else:
                print(f"❌ YOLOv11模組不可用")
                return False
                
        except Exception as e:
            print(f"❌ 載入模型{model_id}失敗: {e}")
            return False
    
    def get_current_model(self):
        if self.current_model_id in self.models:
            return self.models[self.current_model_id]
        return None
    
    def get_available_model_count(self) -> int:
        """獲取可用模型數量"""
        return len(self.model_paths)
    
    def is_model_loaded(self) -> bool:
        """檢查是否有模型已載入"""
        return self.current_model_id > 0 and self.current_model_id in self.models


# ==================== YOLO檢測器 ====================
class YOLOv11Detector:
    def __init__(self, working_dir: str, confidence_threshold: float = 0.8):
        self.working_dir = working_dir
        self.confidence_threshold = confidence_threshold
        self.model_manager = YOLOModelManager(working_dir)
        self.class_names = ['DR_F', 'stack']
        
        # 自動載入模型1
        if 1 in self.model_manager.model_paths:
            print(f"🎯 自動載入模型1作為預設模型")
            self.model_manager.load_model(1)
    
    @property
    def is_loaded(self) -> bool:
        return self.model_manager.is_model_loaded()
    
    def switch_model(self, model_id: int) -> bool:
        return self.model_manager.load_model(model_id)
    
    def update_confidence_threshold(self, threshold: float):
        self.confidence_threshold = max(0.1, min(1.0, threshold))
        print(f"🎯 置信度閾值更新為: {self.confidence_threshold}")
    
    def detect(self, image: np.ndarray) -> YOLODetectionResult:
        start_time = time.time()
        result = YOLODetectionResult()
        result.confidence_threshold = self.confidence_threshold
        result.model_id_used = self.model_manager.current_model_id
        
        try:
            current_model = self.model_manager.get_current_model()
            if current_model is None:
                result.error_message = f"模型{self.model_manager.current_model_id}未載入"
                return result
            
            # 執行推論
            results = current_model(image, conf=self.confidence_threshold, verbose=False)
            
            if results and len(results) > 0:
                detections = results[0]
                
                if detections.boxes is not None and len(detections.boxes) > 0:
                    boxes = detections.boxes.cpu().numpy()
                    
                    for box in boxes:
                        class_id = int(box.cls[0])
                        confidence = float(box.conf[0])
                        
                        if confidence >= self.confidence_threshold:
                            x1, y1, x2, y2 = box.xyxy[0]
                            center_x = float((x1 + x2) / 2)
                            center_y = float((y1 + y2) / 2)
                            
                            if class_id == 0:  # DR_F
                                result.dr_f_coords.append((center_x, center_y))
                                result.dr_f_count += 1
                            elif class_id == 1:  # stack
                                result.stack_coords.append((center_x, center_y))
                                result.stack_count += 1
                    
                    result.total_detections = result.dr_f_count + result.stack_count
                    result.success = True
            
            # 清理記憶體
            if results:
                for res in results:
                    for attr in ['boxes', 'masks', 'keypoints', 'probs']:
                        if hasattr(res, attr):
                            delattr(res, attr)
                results.clear()
                del results
                
        except Exception as e:
            result.error_message = f"YOLOv11檢測失敗: {e}"
            print(f"❌ YOLOv11檢測異常: {e}")
        
        result.processing_time = (time.time() - start_time) * 1000
        return result


# ==================== 座標轉換器 ====================
class CameraCoordinateTransformer:
    def __init__(self):
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvec = None
        self.tvec = None
        self.rotation_matrix = None
        self.is_valid_flag = False
    
    def load_calibration_data(self, intrinsic_file: str, extrinsic_file: str) -> bool:
        try:
            print(f"🔄 載入標定數據...")
            print(f"   內參檔案: {intrinsic_file}")
            print(f"   外參檔案: {extrinsic_file}")
            
            # 載入內參
            intrinsic_data = np.load(intrinsic_file, allow_pickle=True)
            if hasattr(intrinsic_data, 'shape') and intrinsic_data.shape == (3, 3):
                self.camera_matrix = intrinsic_data
                self.dist_coeffs = np.zeros((1, 5))
            elif isinstance(intrinsic_data, dict):
                self.camera_matrix = intrinsic_data['camera_matrix']
                self.dist_coeffs = intrinsic_data.get('dist_coeffs', np.zeros((1, 5)))
            else:
                self.camera_matrix = intrinsic_data
                self.dist_coeffs = np.zeros((1, 5))
            
            # 載入外參
            extrinsic_data = np.load(extrinsic_file, allow_pickle=True)
            if isinstance(extrinsic_data, dict):
                self.rvec = extrinsic_data['rvec']
                self.tvec = extrinsic_data['tvec']
            elif hasattr(extrinsic_data, 'item') and callable(extrinsic_data.item):
                dict_data = extrinsic_data.item()
                if isinstance(dict_data, dict):
                    self.rvec = dict_data['rvec']
                    self.tvec = dict_data['tvec']
            
            # 計算旋轉矩陣
            self.rotation_matrix, _ = cv2.Rodrigues(self.rvec)
            
            self.is_valid_flag = True
            print(f"✅ 座標轉換器載入成功")
            return True
            
        except Exception as e:
            print(f"❌ 座標轉換器載入失敗: {e}")
            return False
    
    def pixel_to_world(self, pixel_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        if not self.is_valid_flag:
            print(f"❌ 標定數據無效，無法進行座標轉換")
            return []
        
        try:
            world_coords = []
            
            for px, py in pixel_coords:
                # 去畸變處理
                pixel_point = np.array([[[float(px), float(py)]]], dtype=np.float32)
                undistorted_points = cv2.undistortPoints(pixel_point, self.camera_matrix, self.dist_coeffs)
                x_norm, y_norm = undistorted_points[0][0]
                
                # 計算世界座標
                normalized_coords = np.array([x_norm, y_norm, 1.0])
                R3 = self.rotation_matrix[2, :]
                denominator = np.dot(R3, normalized_coords)
                
                if abs(denominator) < 1e-6:
                    continue
                
                depth_scale = (0 - self.tvec[2, 0]) / denominator
                camera_point = depth_scale * normalized_coords
                tvec_3d = self.tvec.reshape(3)
                translated_point = camera_point - tvec_3d
                world_point_3d = np.dot(self.rotation_matrix.T, translated_point)
                
                world_x = world_point_3d[0]
                world_y = world_point_3d[1]
                
                world_coords.append((float(world_x), float(world_y)))
            
            return world_coords
            
        except Exception as e:
            print(f"❌ 座標轉換失敗: {e}")
            return []
    
    def is_valid(self) -> bool:
        return (self.is_valid_flag and 
                self.camera_matrix is not None and 
                self.rvec is not None and 
                self.tvec is not None)


# ==================== 狀態機 ====================
class SystemStateMachine:
    def __init__(self):
        self.lock = threading.Lock()
        self.status_register = 0b0001  # 初始Ready=1
    
    def get_bit(self, bit_pos: StatusBits) -> bool:
        with self.lock:
            return bool(self.status_register & (1 << bit_pos))
    
    def set_bit(self, bit_pos: StatusBits, value: bool):
        with self.lock:
            if value:
                self.status_register |= (1 << bit_pos)
            else:
                self.status_register &= ~(1 << bit_pos)
    
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


# ==================== Modbus客戶端服務（含握手功能）====================
class ModbusTcpClientService:
    def __init__(self, server_ip="127.0.0.1", server_port=502):
        self.server_ip = server_ip
        self.server_port = server_port
        self.client: Optional[ModbusTcpClient] = None
        self.connected = False
        self.vision_controller = None
        
        # 同步控制
        self.sync_thread = None
        self.sync_running = False
        self.sync_interval = 0.1  # 100ms輪詢
        
        # Modbus寄存器映射
        self.REGISTERS = {
            'CONTROL_COMMAND': 200,
            'STATUS_REGISTER': 201,
            'MODEL_SELECT': 202,
            'CAPTURE_COMPLETE': 203,
            'DETECT_COMPLETE': 204,
            'OPERATION_SUCCESS': 205,
            'ERROR_CODE': 206,
            'CONFIDENCE_HIGH': 210,
            'CONFIDENCE_LOW': 211,
            'DR_F_COUNT': 240,
            'STACK_COUNT': 242,
            'TOTAL_DETECTIONS': 243,
            'DETECTION_SUCCESS': 244,
            'DR_F_1_X': 245,
            'DR_F_1_Y': 246,
            'DR_F_2_X': 247,
            'DR_F_2_Y': 248,
            'DR_F_3_X': 249,
            'DR_F_3_Y': 250,
            'DR_F_4_X': 251,
            'DR_F_4_Y': 252,
            'DR_F_5_X': 253,
            'DR_F_5_Y': 254,
            'STACK_1_X': 257,
            'STACK_1_Y': 258,
            'MODEL_ID_USED': 259,
            'WORLD_COORD_VALID': 260,
            'VERSION_MAJOR': 290,
            'VERSION_MINOR': 291,
            'OPERATION_COUNT': 284,
            'ERROR_COUNT': 285,
            'CONNECTION_COUNT': 286,
            'MODEL_SWITCH_COUNT': 287,
        }
        
        # 狀態追蹤
        self.last_control_command = 0
        self.command_processing = False
        self.operation_count = 0
        self.error_count = 0
        self.connection_count = 0
        self.start_time = time.time()
        
        # 握手狀態控制
        self.command_execution_time = 0
        self.min_running_duration = 1.0
        self.completion_hold_time = 2.0
        self.completion_start_time = 0
        self.current_command = 0
    
    def set_vision_controller(self, controller):
        self.vision_controller = controller
    
    def connect(self) -> bool:
        if not MODBUS_AVAILABLE:
            print("❌ Modbus Client不可用")
            return False
        
        try:
            if self.client:
                self.client.close()
            
            print(f"🔗 正在連接Modbus TCP服務器: {self.server_ip}:{self.server_port}")
            
            self.client = ModbusTcpClient(
                host=self.server_ip,
                port=self.server_port,
                timeout=3.0
            )
            
            if self.client.connect():
                self.connected = True
                self.connection_count += 1
                self._write_initial_status()
                print(f"✅ Modbus TCP Client連接成功")
                return True
            else:
                print(f"❌ Modbus TCP連接失敗")
                self.connected = False
                return False
                
        except Exception as e:
            print(f"❌ Modbus TCP連接異常: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        self.stop_sync()
        if self.client and self.connected:
            try:
                self.client.close()
                print("🔌 Modbus TCP Client已斷開連接")
            except:
                pass
        self.connected = False
        self.client = None
    
    def start_sync(self):
        """啟動握手同步線程"""
        if self.sync_running:
            return
        
        self.sync_running = True
        self.sync_thread = threading.Thread(target=self._handshake_sync_loop, daemon=True)
        self.sync_thread.start()
        print("✅ Modbus握手同步線程已啟動")
    
    def stop_sync(self):
        """停止握手同步線程"""
        if self.sync_running:
            self.sync_running = False
            if self.sync_thread and self.sync_thread.is_alive():
                self.sync_thread.join(timeout=2.0)
            print("🛑 Modbus握手同步線程已停止")
    
    def _handshake_sync_loop(self):
        """握手同步循環"""
        print("🔄 Modbus握手同步線程開始運行...")
        
        while self.sync_running and self.connected:
            try:
                # 1. 更新狀態寄存器到PLC
                self._update_status_register()
                
                # 2. 處理模型管理指令
                self._handle_model_management()
                
                # 3. 讀取控制指令並處理握手邏輯
                self._handle_control_command()
                
                # 4. 更新統計資訊
                self._update_statistics()
                
                time.sleep(self.sync_interval)
                
            except ConnectionException:
                print("❌ Modbus連接中斷，同步線程退出")
                self.connected = False
                break
                
            except Exception as e:
                print(f"❌ 同步線程錯誤: {e}")
                self.error_count += 1
                time.sleep(1.0)
        
        self.sync_running = False
        print("⏹️ Modbus握手同步線程已退出")
    
    def _update_status_register(self):
        """更新狀態寄存器到PLC"""
        try:
            if self.vision_controller:
                status_value = self.vision_controller.state_machine.status_register
                self.write_register('STATUS_REGISTER', status_value)
        except:
            pass
    
    def _handle_model_management(self):
        """處理模型管理指令"""
        try:
            if not self.vision_controller or not self.vision_controller.yolo_detector:
                return
            
            model_select = self.read_register('MODEL_SELECT')
            if model_select is None:
                return
            
            current_model_id = self.vision_controller.yolo_detector.model_manager.current_model_id
            
            if model_select != current_model_id:
                print(f"📋 收到模型切換指令: 模型{current_model_id} → 模型{model_select}")
                
                if 0 <= model_select <= 20:
                    success = self.vision_controller.yolo_detector.switch_model(model_select)
                    if success:
                        print(f"✅ Modbus觸發模型切換成功: 當前模型{model_select}")
                        switch_count = self.vision_controller.yolo_detector.model_manager.model_switch_count
                        self.write_register('MODEL_SWITCH_COUNT', switch_count)
                    else:
                        print(f"❌ Modbus觸發模型切換失敗: 模型{model_select}")
                        self.write_register('ERROR_CODE', 10)
                else:
                    print(f"❌ 無效的模型ID: {model_select}")
                    self.write_register('ERROR_CODE', 11)
                    
        except Exception as e:
            print(f"❌ 處理模型管理指令失敗: {e}")
    
    def _handle_control_command(self):
        """處理控制指令握手邏輯"""
        try:
            current_command = self.read_register('CONTROL_COMMAND')
            if current_command is None:
                return
            
            # 檢查指令變化和防重複執行
            if (current_command != self.last_control_command and 
                current_command != 0 and 
                not self.command_processing):
                
                print(f"📋 收到新控制指令: {current_command}")
                self.last_control_command = current_command
                self.current_command = current_command
                self.command_processing = True
                
                # 清除完成標誌
                self._clear_completion_flags()
                
                # 異步執行指令
                command_thread = threading.Thread(
                    target=self._execute_command, 
                    args=(ControlCommand(current_command),),
                    daemon=True
                )
                command_thread.start()
                
            # 處理指令清零邏輯
            elif current_command == 0 and self.last_control_command != 0:
                print(f"📋 控制指令已清零，準備恢復Ready狀態")
                self._handle_command_clear()
                
        except Exception as e:
            print(f"❌ 處理控制指令失敗: {e}")
    
    def _execute_command(self, command: ControlCommand):
        """執行控制指令"""
        try:
            print(f"🚀 開始處理控制指令: {command}")
            
            if not self.vision_controller:
                print("❌ 視覺控制器不存在")
                self._set_error_state(1, "視覺控制器不存在")
                return
            
            # 檢查Ready狀態
            if not self.vision_controller.state_machine.is_ready():
                print("⚠️ 系統未Ready，忽略指令")
                self._set_error_state(2, "系統未Ready")
                return
            
            # 設置Running狀態，清除Ready
            self.vision_controller.state_machine.set_running(True)
            self.vision_controller.state_machine.set_ready(False)
            print(f"🔄 狀態變更: Ready=0, Running=1")
            
            result = None
            if command == ControlCommand.CAPTURE:
                result = self._handle_capture_command()
            elif command == ControlCommand.CAPTURE_DETECT:
                result = self._handle_capture_detect_command()
            elif command == ControlCommand.INITIALIZE:
                result = self._handle_initialize_command()
            else:
                print(f"⚠️ 未知控制指令: {command}")
                self._set_error_state(3, f"未知指令: {command}")
                return
            
            # 設置完成狀態
            if result:
                self._set_completion_state(True, command)
                print(f"✅ 指令 {command} 執行成功")
            else:
                self._set_completion_state(False, command)
                print(f"❌ 指令 {command} 執行失敗")
            
        except Exception as e:
            print(f"❌ 執行控制指令異常: {e}")
            self.error_count += 1
            self._set_error_state(99, f"執行異常: {e}")
        finally:
            # 清除Running狀態
            self.vision_controller.state_machine.set_running(False)
            self.command_processing = False
            self.completion_start_time = time.time()
            print(f"🔄 狀態變更: Running=0")
    
    def _handle_capture_command(self):
        """處理拍照指令"""
        try:
            print("📸 執行拍照指令")
            image, capture_time = self.vision_controller.capture_image()
            
            if image is not None:
                self.write_register('CAPTURE_COMPLETE', 1)
                print(f"✅ 拍照成功，耗時: {capture_time*1000:.2f}ms")
                return True
            else:
                print("❌ 拍照失敗")
                self.error_count += 1
                self.write_register('CAPTURE_COMPLETE', 0)
                return False
                
        except Exception as e:
            print(f"❌ 拍照指令執行失敗: {e}")
            self.error_count += 1
            self.write_register('CAPTURE_COMPLETE', 0)
            return False
    
    def _handle_capture_detect_command(self):
        """處理拍照+檢測指令"""
        try:
            print("🔍 執行拍照+YOLOv11檢測指令")
            result = self.vision_controller.capture_and_detect()
            
            if result and result.success:
                # 更新檢測結果到PLC
                self.update_detection_results(result)
                self.write_register('CAPTURE_COMPLETE', 1)
                self.write_register('DETECT_COMPLETE', 1)
                print(f"✅ YOLOv11檢測成功，DR_F={result.dr_f_count}, STACK={result.stack_count}")
                return True
            else:
                error_msg = result.error_message if result else "檢測結果為空"
                print(f"❌ YOLOv11檢測失敗: {error_msg}")
                self.error_count += 1
                self._clear_detection_results()
                self.write_register('CAPTURE_COMPLETE', 0)
                self.write_register('DETECT_COMPLETE', 0)
                return False
                
        except Exception as e:
            print(f"❌ 檢測指令執行失敗: {e}")
            self.error_count += 1
            self._clear_detection_results()
            return False
    
    def _handle_initialize_command(self):
        """處理初始化指令"""
        try:
            print("🔄 執行系統初始化指令")
            
            success = self.vision_controller.initialize_camera()
            
            if success:
                self.vision_controller.state_machine.set_initialized(True)
                self.vision_controller.state_machine.set_alarm(False)
                print("✅ 系統初始化成功")
                return True
            else:
                self.vision_controller.state_machine.set_initialized(False)
                self.vision_controller.state_machine.set_alarm(True)
                print("❌ 系統初始化失敗")
                self.error_count += 1
                return False
                
        except Exception as e:
            print(f"❌ 初始化指令執行失敗: {e}")
            self.error_count += 1
            self.vision_controller.state_machine.set_alarm(True)
            return False
    
    def _clear_completion_flags(self):
        """清除完成標誌"""
        try:
            self.write_register('CAPTURE_COMPLETE', 0)
            self.write_register('DETECT_COMPLETE', 0)
            self.write_register('OPERATION_SUCCESS', 0)
            self.write_register('ERROR_CODE', 0)
        except Exception as e:
            print(f"❌ 清除完成標誌失敗: {e}")
    
    def _set_error_state(self, error_code: int, error_msg: str):
        """設置錯誤狀態"""
        try:
            print(f"❌ 設置錯誤狀態: {error_code} - {error_msg}")
            self.write_register('ERROR_CODE', error_code)
            self.write_register('OPERATION_SUCCESS', 0)
            self.vision_controller.state_machine.set_alarm(True)
            self.vision_controller.state_machine.set_running(False)
            self.command_processing = False
        except Exception as e:
            print(f"❌ 設置錯誤狀態失敗: {e}")
    
    def _set_completion_state(self, success: bool, command: ControlCommand):
        """設置完成狀態"""
        try:
            self.write_register('OPERATION_SUCCESS', 1 if success else 0)
            
            if not success:
                error_code = 0
                if command == ControlCommand.CAPTURE:
                    error_code = 10
                elif command == ControlCommand.CAPTURE_DETECT:
                    error_code = 20
                elif command == ControlCommand.INITIALIZE:
                    error_code = 30
                
                self.write_register('ERROR_CODE', error_code)
                self.vision_controller.state_machine.set_alarm(True)
            else:
                self.write_register('ERROR_CODE', 0)
                
        except Exception as e:
            print(f"❌ 設置完成狀態失敗: {e}")
    
    def _handle_command_clear(self):
        """處理指令清零"""
        try:
            print("🔄 處理指令清零，恢復Ready狀態")
            
            self.last_control_command = 0
            self.current_command = 0
            
            if not self.vision_controller.state_machine.is_alarm():
                self.vision_controller.state_machine.set_ready(True)
                print("✅ Ready狀態已恢復")
            else:
                print("⚠️ 系統處於Alarm狀態，需要重置才能恢復Ready")
                
        except Exception as e:
            print(f"❌ 處理指令清零失敗: {e}")
    
    def _clear_detection_results(self):
        """清空檢測結果寄存器"""
        try:
            self.write_register('DR_F_COUNT', 0)
            self.write_register('STACK_COUNT', 0)
            self.write_register('TOTAL_DETECTIONS', 0)
            self.write_register('DETECTION_SUCCESS', 0)
            
            for i in range(1, 6):
                self.write_register(f'DR_F_{i}_X', 0)
                self.write_register(f'DR_F_{i}_Y', 0)
                
        except Exception as e:
            print(f"❌ 清空檢測結果失敗: {e}")
    
    def update_detection_results(self, result: YOLODetectionResult):
        """更新YOLOv11檢測結果到PLC"""
        try:
            # 寫入檢測數量
            self.write_register('DR_F_COUNT', result.dr_f_count)
            self.write_register('STACK_COUNT', result.stack_count)
            self.write_register('TOTAL_DETECTIONS', result.total_detections)
            self.write_register('DETECTION_SUCCESS', 1 if result.success else 0)
            self.write_register('MODEL_ID_USED', result.model_id_used)
            
            # 寫入DR_F座標
            for i in range(5):
                if i < len(result.dr_f_coords):
                    x, y = result.dr_f_coords[i]
                    self.write_register(f'DR_F_{i+1}_X', int(float(x)))
                    self.write_register(f'DR_F_{i+1}_Y', int(float(y)))
                else:
                    self.write_register(f'DR_F_{i+1}_X', 0)
                    self.write_register(f'DR_F_{i+1}_Y', 0)
            
            # 寫入STACK座標
            if result.stack_coords:
                x, y = result.stack_coords[0]
                self.write_register('STACK_1_X', int(float(x)))
                self.write_register('STACK_1_Y', int(float(y)))
            else:
                self.write_register('STACK_1_X', 0)
                self.write_register('STACK_1_Y', 0)
            
            # 更新世界座標有效性
            if result.dr_f_world_coords:
                self.write_register('WORLD_COORD_VALID', 1)
            else:
                self.write_register('WORLD_COORD_VALID', 0)
            
        except Exception as e:
            print(f"❌ 更新檢測結果到PLC失敗: {e}")
    
    def _update_statistics(self):
        """更新統計資訊"""
        try:
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            self.write_register('CONNECTION_COUNT', self.connection_count)
            
            # 更新模型切換次數
            if self.vision_controller and self.vision_controller.yolo_detector:
                switch_count = self.vision_controller.yolo_detector.model_manager.model_switch_count
                self.write_register('MODEL_SWITCH_COUNT', switch_count)
                
        except:
            pass
    
    def _write_initial_status(self):
        """寫入初始狀態到PLC"""
        try:
            # 版本資訊
            self.write_register('VERSION_MAJOR', 5)
            self.write_register('VERSION_MINOR', 1)
            
            # 初始化置信度閾值為0.8
            confidence_int = int(0.8 * 10000)
            self.write_register('CONFIDENCE_HIGH', (confidence_int >> 16) & 0xFFFF)
            self.write_register('CONFIDENCE_LOW', confidence_int & 0xFFFF)
            
            # 計數器
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            self.write_register('CONNECTION_COUNT', self.connection_count)
            
            print("📊 初始狀態已寫入PLC")
            
        except Exception as e:
            print(f"❌ 寫入初始狀態失敗: {e}")
    
    def write_register(self, register_name: str, value: int) -> bool:
        if not self.connected or not self.client or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.client.write_register(address, value, slave=1)
            return not result.isError()
        except:
            return False
    
    def read_register(self, register_name: str) -> Optional[int]:
        if not self.connected or not self.client or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.client.read_holding_registers(address, count=1, slave=1)
            if not result.isError():
                return result.registers[0]
            return None
        except:
            return None
    
    def read_confidence_threshold(self) -> float:
        """讀取置信度閾值"""
        try:
            high = self.read_register('CONFIDENCE_HIGH') or 0
            low = self.read_register('CONFIDENCE_LOW') or 8000
            confidence_int = (high << 16) + low
            return confidence_int / 10000.0
        except:
            return 0.8


# ==================== 主控制器 ====================
class CCD1VisionController:
    def __init__(self):
        self.working_dir = os.path.dirname(os.path.abspath(__file__))
        self.server_ip = "127.0.0.1"
        self.server_port = 502
        self.camera_ip = "192.168.1.8"
        
        # 核心組件
        self.state_machine = SystemStateMachine()
        self.camera_manager: Optional[OptimizedCameraManager] = None
        self.coordinate_transformer = CameraCoordinateTransformer()
        self.yolo_detector = None
        
        # 圖像緩存
        self.last_image: Optional[np.ndarray] = None
        self.last_result: Optional[YOLODetectionResult] = None
        
        # Modbus客戶端
        self.modbus_client = ModbusTcpClientService(self.server_ip, self.server_port)
        self.modbus_client.set_vision_controller(self)
        
        # 統計信息
        self.operation_count = 0
        self.error_count = 0
        
        # 初始化YOLO檢測器
        self._initialize_yolo()
    
    def _initialize_yolo(self):
        if not YOLO_AVAILABLE:
            print("❌ YOLOv11模組不可用")
            self.state_machine.set_alarm(True)
            return
        
        self.yolo_detector = YOLOv11Detector(self.working_dir, 0.8)
        
        if self.yolo_detector.model_manager.get_available_model_count() > 0:
            print("✅ YOLOv11檢測器初始化成功")
        else:
            print("❌ 未發現任何YOLOv11模型檔案")
            self.state_machine.set_alarm(True)
    
    def connect_modbus(self) -> bool:
        return self.modbus_client.connect()
    
    def disconnect_modbus(self):
        self.modbus_client.disconnect()
    
    def start_handshake(self):
        """啟動Modbus握手功能"""
        if self.modbus_client.connected:
            self.modbus_client.start_sync()
            print("✅ Modbus握手功能已啟動")
            return True
        else:
            print("❌ 請先連接Modbus服務器")
            return False
    
    def stop_handshake(self):
        """停止Modbus握手功能"""
        self.modbus_client.stop_sync()
        print("🛑 Modbus握手功能已停止")
    
    def save_image(self, image: np.ndarray, filename: str, save_dir: str = "dynamic_save") -> bool:
        """儲存圖片到指定目錄"""
        try:
            # 確保儲存目錄存在
            save_path = os.path.join(self.working_dir, save_dir)
            os.makedirs(save_path, exist_ok=True)
            
            # 完整檔案路徑
            full_path = os.path.join(save_path, filename)
            
            # 儲存圖片
            success = cv2.imwrite(full_path, image)
            if success:
                print(f"✅ 圖片已儲存: {full_path}")
                return True
            else:
                print(f"❌ 圖片儲存失敗: {full_path}")
                return False
                
        except Exception as e:
            print(f"❌ 儲存圖片異常: {e}")
            return False
    
    def create_detection_visualization(self, image: np.ndarray, result: YOLODetectionResult) -> np.ndarray:
        """創建檢測結果可視化圖片"""
        try:
            vis_image = np.copy(image)
            
            # 定義顏色
            colors = {
                'DR_F': (0, 0, 255),    # 紅色 (BGR格式)
                'STACK': (255, 0, 0),   # 藍色
            }
            
            # 繪製DR_F檢測結果
            for i, (x, y) in enumerate(result.dr_f_coords):
                cv2.circle(vis_image, (int(x), int(y)), 15, colors['DR_F'], -1)
                cv2.circle(vis_image, (int(x), int(y)), 20, (255, 255, 255), 3)
                
                label = f"DR_F{i+1}"
                cv2.putText(vis_image, label, (int(x-35), int(y-25)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            
            # 繪製STACK檢測結果
            for i, (x, y) in enumerate(result.stack_coords):
                cv2.circle(vis_image, (int(x), int(y)), 12, colors['STACK'], -1)
                cv2.circle(vis_image, (int(x), int(y)), 17, (255, 255, 255), 2)
                
                label = f"STACK{i+1}"
                cv2.putText(vis_image, label, (int(x-35), int(y-25)), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, colors['STACK'], 2)
            
            # 添加檢測統計信息
            stats_text = f"YOLOv11 Model{result.model_id_used}: DR_F={result.dr_f_count}, STACK={result.stack_count}"
            cv2.putText(vis_image, stats_text, (20, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
            
            # 添加時間戳
            timestamp_text = f"Time: {result.timestamp}"
            cv2.putText(vis_image, timestamp_text, (20, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            return vis_image
            
        except Exception as e:
            print(f"❌ 創建可視化失敗: {e}")
            return image
    
    def initialize_camera(self, ip_address: str = None) -> bool:
        try:
            if ip_address:
                self.camera_ip = ip_address
            
            if self.camera_manager:
                try:
                    self.camera_manager.shutdown()
                except:
                    pass
                finally:
                    self.camera_manager = None
            
            camera_config = CameraConfig(
                name="ccd1_camera",
                ip=self.camera_ip,
                exposure_time=20000.0,
                gain=200.0,
                frame_rate=5.0,
                pixel_format=PixelFormat.BAYER_GR8,
                width=2592,
                height=1944,
                trigger_mode=CameraMode.SOFTWARE_TRIGGER,
                auto_reconnect=True,
                bandwidth_limit_mbps=200,
                use_latest_frame_only=True,
                buffer_count=1
            )
            
            print(f"🔄 初始化相機: {self.camera_ip}")
            self.camera_manager = OptimizedCameraManager()
            
            success = self.camera_manager.add_camera("ccd1_camera", camera_config)
            if not success:
                raise Exception("添加相機失敗")
            
            connect_result = self.camera_manager.connect_camera("ccd1_camera")
            if not connect_result:
                raise Exception("相機連接失敗")
            
            stream_result = self.camera_manager.start_streaming(["ccd1_camera"])
            if not stream_result.get("ccd1_camera", False):
                raise Exception("開始串流失敗")
            
            time.sleep(1.0)
            
            self.state_machine.set_initialized(True)
            self.state_machine.set_alarm(False)
            self.state_machine.set_ready(True)
            print(f"✅ 相機初始化成功: {self.camera_ip}")
            return True
                
        except Exception as e:
            self.state_machine.set_alarm(True)
            self.state_machine.set_initialized(False)
            self.state_machine.set_ready(False)
            print(f"❌ 相機初始化失敗: {e}")
            return False
    
    def capture_image(self) -> Tuple[Optional[np.ndarray], float]:
        if not self.camera_manager:
            print(f"❌ 相機管理器不存在")
            return None, 0.0
        
        capture_start = time.time()
        
        try:
            trigger_result = self.camera_manager.trigger_software(["ccd1_camera"])
            if not trigger_result.get("ccd1_camera", False):
                print(f"❌ 軟體觸發失敗")
                return None, 0.0
            
            frame_data = self.camera_manager.capture_new_frame("ccd1_camera", timeout=2000)
            if frame_data is None:
                print(f"❌ 觸發後無法獲取圖像")
                return None, 0.0
            
            capture_time = time.time() - capture_start
            
            image_array = np.copy(frame_data.data)
            if hasattr(frame_data, 'data'):
                del frame_data.data
            del frame_data
            
            if len(image_array.shape) == 2:
                display_image = cv2.cvtColor(image_array, cv2.COLOR_GRAY2BGR)
                del image_array
            else:
                display_image = image_array
            
            print(f"✅ 拍照成功，耗時: {capture_time*1000:.2f}ms")
            return display_image, capture_time
            
        except Exception as e:
            capture_time = time.time() - capture_start
            print(f"❌ 拍照異常: {e}")
            return None, capture_time
    
    def capture_and_detect(self, save_original: bool = False, save_result: bool = False) -> YOLODetectionResult:
        total_start = time.time()
        
        try:
            if not self.yolo_detector or not self.yolo_detector.is_loaded:
                result = YOLODetectionResult()
                result.error_message = "YOLOv11檢測器未載入"
                result.total_time = (time.time() - total_start) * 1000
                return result
            
            # 拍照
            image, capture_time = self.capture_image()
            if image is None:
                result = YOLODetectionResult()
                result.error_message = "圖像捕獲失敗"
                result.capture_time = capture_time * 1000
                result.total_time = (time.time() - total_start) * 1000
                return result
            
            # 儲存原始圖片
            if save_original:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                original_filename = f"original_{timestamp}.jpg"
                self.save_image(image, original_filename)
            
            # YOLO檢測
            result = self.yolo_detector.detect(image)
            result.capture_time = capture_time * 1000
            result.total_time = (time.time() - total_start) * 1000
            
            # 世界座標轉換
            if result.success and result.dr_f_coords and self.coordinate_transformer.is_valid():
                world_coords = self.coordinate_transformer.pixel_to_world(result.dr_f_coords)
                if world_coords:
                    result.dr_f_world_coords = []
                    for wx, wy in world_coords:
                        result.dr_f_world_coords.append((float(wx), float(wy)))
            
            # 儲存檢測結果圖片
            if save_result and result.success:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                result_filename = f"result_{timestamp}.jpg"
                vis_image = self.create_detection_visualization(image, result)
                self.save_image(vis_image, result_filename)
            
            self.last_result = result
            gc.collect()
            return result
            
        except Exception as e:
            result = YOLODetectionResult()
            result.error_message = f"檢測失敗: {str(e)}"
            result.total_time = (time.time() - total_start) * 1000
            gc.collect()
            return result
    
    def load_calibration_data(self, intrinsic_file: str, extrinsic_file: str) -> bool:
        intrinsic_path = os.path.join(self.working_dir, intrinsic_file)
        extrinsic_path = os.path.join(self.working_dir, extrinsic_file)
        
        if not os.path.exists(intrinsic_path):
            print(f"❌ 內參檔案不存在: {intrinsic_path}")
            return False
        
        if not os.path.exists(extrinsic_path):
            print(f"❌ 外參檔案不存在: {extrinsic_path}")
            return False
        
        return self.coordinate_transformer.load_calibration_data(intrinsic_path, extrinsic_path)
    
    def get_status(self) -> Dict[str, Any]:
        return {
            'ready': self.state_machine.is_ready(),
            'running': self.state_machine.is_running(),
            'alarm': self.state_machine.is_alarm(),
            'initialized': self.state_machine.is_initialized(),
            'camera_connected': self.camera_manager is not None,
            'yolo_loaded': self.yolo_detector is not None and self.yolo_detector.is_loaded,
            'confidence_threshold': self.yolo_detector.confidence_threshold if self.yolo_detector else 0.8,
            'modbus_connected': self.modbus_client.connected,
            'modbus_handshake_running': self.modbus_client.sync_running,
            'calibration_valid': self.coordinate_transformer.is_valid(),
            'operation_count': self.operation_count,
            'error_count': self.error_count,
            'current_model_id': self.yolo_detector.model_manager.current_model_id if self.yolo_detector else 0,
            'available_models': len(self.yolo_detector.model_manager.model_paths) if self.yolo_detector else 0
        }
    
    def disconnect(self):
        if self.camera_manager:
            self.camera_manager.shutdown()
            self.camera_manager = None
        
        self.modbus_client.disconnect()
        print("✅ 所有連接已斷開")


# ==================== Terminal指令介面 ====================
class TerminalController:
    def __init__(self):
        self.controller = CCD1VisionController()
        self.running = True
        
        print("=" * 80)
        print("🚀 CCD1視覺控制系統 Terminal版本 (YOLOv11 + Modbus握手)")
        print("=" * 80)
        
        # 自動啟動Modbus連接和握手功能
        self._auto_initialize()
    
    def _auto_initialize(self):
        """自動初始化Modbus連接和握手功能"""
        print("🔄 自動初始化Modbus連接和握手功能...")
        
        # 自動連接Modbus
        success = self.controller.connect_modbus()
        if success:
            print("✅ Modbus自動連接成功")
            
            # 自動啟動握手功能
            handshake_success = self.controller.start_handshake()
            if handshake_success:
                print("✅ Modbus握手功能自動啟動成功")
            else:
                print("❌ Modbus握手功能自動啟動失敗")
        else:
            print("❌ Modbus自動連接失敗")
        
        print("🎯 系統已準備就緒，支援Terminal指令和Modbus自動握手")
        print()
    
    def show_help(self):
        print("\n📋 可用指令:")
        print("  help                    - 顯示此說明")
        print("  status                  - 顯示系統狀態")
        print("  connect_modbus          - 連接Modbus服務器")
        print("  disconnect_modbus       - 斷開Modbus連接")
        print("  start_handshake         - 啟動Modbus握手功能")
        print("  stop_handshake          - 停止Modbus握手功能")
        print("  init_camera [IP]        - 初始化相機 (預設: 192.168.1.8)")
        print("  capture                 - 拍照")
        print("  detect [--save] [--result] [--original] - 拍照並檢測")
        print("                            --save: 同時儲存原始和結果圖片")
        print("                            --result: 僅儲存檢測結果圖片")
        print("                            --original: 僅儲存原始圖片")
        print("  switch_model <ID>       - 切換YOLO模型 (1-20)")
        print("  set_confidence <VAL>    - 設置置信度閾值 (0.1-1.0)")
        print("  load_calib <INT> <EXT>  - 載入標定檔案")
        print("  models                  - 顯示可用模型")
        print("  exit                    - 退出系統")
        print()
        print("💡 系統支援Terminal指令和Modbus指令雙向控制")
        print("   - Terminal下指令，Modbus寄存器會同步更新")
        print("   - PLC透過Modbus下指令，Terminal會顯示執行結果")
        print()
    
    def show_status(self):
        status = self.controller.get_status()
        print(f"\n📊 系統狀態:")
        print(f"  Ready: {'✅' if status['ready'] else '❌'}")
        print(f"  Running: {'✅' if status['running'] else '❌'}")
        print(f"  Alarm: {'❌' if status['alarm'] else '✅'}")
        print(f"  Initialized: {'✅' if status['initialized'] else '❌'}")
        print(f"  Camera: {'✅' if status['camera_connected'] else '❌'}")
        print(f"  YOLO: {'✅' if status['yolo_loaded'] else '❌'}")
        print(f"  Modbus: {'✅' if status['modbus_connected'] else '❌'}")
        print(f"  Modbus握手: {'✅' if status['modbus_handshake_running'] else '❌'}")
        print(f"  Calibration: {'✅' if status['calibration_valid'] else '❌'}")
        print(f"  Current Model: {status['current_model_id']}")
        print(f"  Available Models: {status['available_models']}")
        print(f"  Confidence: {status['confidence_threshold']:.2f}")
        print(f"  Operations: {status['operation_count']}, Errors: {status['error_count']}")
        print()
    
    def show_models(self):
        if self.controller.yolo_detector:
            manager = self.controller.yolo_detector.model_manager
            print(f"\n🎯 可用模型:")
            print(f"  當前模型ID: {manager.current_model_id}")
            print(f"  可用模型: {list(manager.model_paths.keys())}")
            print(f"  切換次數: {manager.model_switch_count}")
            for model_id, path in manager.model_paths.items():
                status = "✅ (已載入)" if model_id == manager.current_model_id else ""
                print(f"    模型{model_id}: {os.path.basename(path)} {status}")
        else:
            print("❌ YOLO檢測器未初始化")
        print()
    
    def show_detection_result(self, result: YOLODetectionResult):
        print(f"\n🔍 檢測結果:")
        print(f"  成功: {'✅' if result.success else '❌'}")
        print(f"  DR_F數量: {result.dr_f_count}")
        print(f"  STACK數量: {result.stack_count}")
        print(f"  總檢測數: {result.total_detections}")
        print(f"  使用模型: {result.model_id_used}")
        print(f"  置信度閾值: {result.confidence_threshold}")
        print(f"  拍照耗時: {result.capture_time:.2f}ms")
        print(f"  處理耗時: {result.processing_time:.2f}ms")
        print(f"  總耗時: {result.total_time:.2f}ms")
        
        if result.dr_f_coords:
            print(f"  DR_F座標:")
            for i, (x, y) in enumerate(result.dr_f_coords):
                print(f"    DR_F{i+1}: ({x:.1f}, {y:.1f})")
        
        if result.stack_coords:
            print(f"  STACK座標:")
            for i, (x, y) in enumerate(result.stack_coords):
                print(f"    STACK{i+1}: ({x:.1f}, {y:.1f})")
        
        if result.dr_f_world_coords:
            print(f"  世界座標:")
            for i, (wx, wy) in enumerate(result.dr_f_world_coords):
                print(f"    DR_F{i+1}: ({wx:.2f}, {wy:.2f}) mm")
        
        if result.error_message:
            print(f"  錯誤: {result.error_message}")
        
        print()
    
    def run(self):
        self.show_help()
        
        while self.running:
            try:
                command = input("CCD1> ").strip().lower()
                
                if not command:
                    continue
                
                parts = command.split()
                cmd = parts[0]
                
                if cmd == "help":
                    self.show_help()
                
                elif cmd == "status":
                    self.show_status()
                
                elif cmd == "connect_modbus":
                    success = self.controller.connect_modbus()
                    print(f"{'✅' if success else '❌'} Modbus連接{'成功' if success else '失敗'}")
                    if success:
                        # 連接成功後自動啟動握手
                        handshake_success = self.controller.start_handshake()
                        print(f"{'✅' if handshake_success else '❌'} 握手功能自動{'啟動' if handshake_success else '啟動失敗'}")
                
                elif cmd == "disconnect_modbus":
                    self.controller.disconnect_modbus()
                    print("✅ Modbus已斷開")
                
                elif cmd == "start_handshake":
                    success = self.controller.start_handshake()
                    print(f"{'✅' if success else '❌'} Modbus握手功能{'啟動成功' if success else '啟動失敗'}")
                
                elif cmd == "stop_handshake":
                    self.controller.stop_handshake()
                    print("🛑 Modbus握手功能已停止")
                
                elif cmd == "init_camera":
                    ip = parts[1] if len(parts) > 1 else self.controller.camera_ip
                    success = self.controller.initialize_camera(ip)
                    print(f"{'✅' if success else '❌'} 相機初始化{'成功' if success else '失敗'}")
                
                elif cmd == "capture":
                    image, capture_time = self.controller.capture_image()
                    if image is not None:
                        print(f"✅ 拍照成功，耗時: {capture_time*1000:.2f}ms")
                    else:
                        print("❌ 拍照失敗")
                
                elif cmd == "detect":
                    # 解析儲存參數
                    save_original = False
                    save_result = False
                    
                    for part in parts[1:]:
                        if part == "--save":
                            save_original = True
                            save_result = True
                        elif part == "--original":
                            save_original = True
                        elif part == "--result":
                            save_result = True
                    
                    result = self.controller.capture_and_detect(save_original, save_result)
                    self.show_detection_result(result)
                    
                    # 同步結果到Modbus（如果連接）
                    if self.controller.modbus_client.connected and result.success:
                        self.controller.modbus_client.update_detection_results(result)
                        print("📤 檢測結果已同步到Modbus寄存器")
                    
                    if save_original or save_result:
                        save_info = []
                        if save_original:
                            save_info.append("原始圖片")
                        if save_result:
                            save_info.append("結果圖片")
                        print(f"💾 已儲存: {', '.join(save_info)} (存放於 dynamic_save/ 資料夾)")
                
                elif cmd == "switch_model":
                    if len(parts) < 2:
                        print("❌ 請指定模型ID (1-20)")
                        continue
                    
                    try:
                        model_id = int(parts[1])
                        if self.controller.yolo_detector:
                            success = self.controller.yolo_detector.switch_model(model_id)
                            print(f"{'✅' if success else '❌'} 模型切換{'成功' if success else '失敗'}")
                            
                            # 同步模型選擇到Modbus
                            if success and self.controller.modbus_client.connected:
                                self.controller.modbus_client.write_register('MODEL_SELECT', model_id)
                                print("📤 模型選擇已同步到Modbus寄存器")
                        else:
                            print("❌ YOLO檢測器未初始化")
                    except ValueError:
                        print("❌ 無效的模型ID")
                
                elif cmd == "set_confidence":
                    if len(parts) < 2:
                        print("❌ 請指定置信度閾值 (0.1-1.0)")
                        continue
                    
                    try:
                        threshold = float(parts[1])
                        if self.controller.yolo_detector:
                            self.controller.yolo_detector.update_confidence_threshold(threshold)
                            print(f"✅ 置信度閾值設置為: {threshold}")
                            
                            # 同步置信度到Modbus
                            if self.controller.modbus_client.connected:
                                confidence_int = int(threshold * 10000)
                                self.controller.modbus_client.write_register('CONFIDENCE_HIGH', (confidence_int >> 16) & 0xFFFF)
                                self.controller.modbus_client.write_register('CONFIDENCE_LOW', confidence_int & 0xFFFF)
                                print("📤 置信度閾值已同步到Modbus寄存器")
                        else:
                            print("❌ YOLO檢測器未初始化")
                    except ValueError:
                        print("❌ 無效的置信度值")
                
                elif cmd == "load_calib":
                    if len(parts) < 3:
                        print("❌ 請指定內參和外參檔案名")
                        continue
                    
                    intrinsic_file = parts[1]
                    extrinsic_file = parts[2]
                    success = self.controller.load_calibration_data(intrinsic_file, extrinsic_file)
                    print(f"{'✅' if success else '❌'} 標定檔案載入{'成功' if success else '失敗'}")
                
                elif cmd == "models":
                    self.show_models()
                
                elif cmd in ["exit", "quit"]:
                    self.running = False
                
                else:
                    print(f"❌ 未知指令: {cmd}，輸入 'help' 查看可用指令")
            
            except KeyboardInterrupt:
                print("\n收到中斷信號...")
                self.running = False
            
            except Exception as e:
                print(f"❌ 指令執行錯誤: {e}")
        
        print("正在關閉系統...")
        self.controller.disconnect()
        print("✅ 系統已安全關閉")


def main():
    if not CAMERA_MANAGER_AVAILABLE:
        print("❌ 相機管理器不可用，請檢查SDK導入")
        return
    
    try:
        terminal = TerminalController()
        terminal.run()
    except Exception as e:
        print(f"❌ 系統錯誤: {e}")


if __name__ == "__main__":
    main()