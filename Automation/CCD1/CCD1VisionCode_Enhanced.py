# -*- coding: utf-8 -*-
"""
CCD1VisionCode_YOLOv11_ModbusOnly.py - CCD1視覺控制系統 純Modbus版本
整合YOLOv11物件檢測功能，支援DR_F/STACK分類檢測
基於Modbus TCP Client架構，實現握手式狀態機控制
移除Web介面，支援自動初始化和自我重載功能
適配pymodbus 3.9.2
"""

import sys
import os
import time
import threading
import json
import gc
import psutil
from typing import Optional, Dict, Any, Tuple, List
import numpy as np
import cv2
import logging
from dataclasses import dataclass, asdict
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
    print("💡 請安裝: pip install ultralytics")
    YOLO_AVAILABLE = False

# 導入Modbus TCP Client (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
    print("✅ Modbus Client模組導入成功 (pymodbus 3.9.2)")
except ImportError as e:
    print(f"❌ Modbus Client模組導入失敗: {e}")
    print("💡 請安裝: pip install pymodbus>=3.0.0")
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
    """控制指令枚舉"""
    CLEAR = 0
    CAPTURE = 8
    CAPTURE_DETECT = 16
    INITIALIZE = 32


class StatusBits(IntEnum):
    """狀態位枚舉"""
    READY = 0
    RUNNING = 1
    ALARM = 2
    INITIALIZED = 3


# ==================== 數據結構定義 ====================
@dataclass
class YOLODetectionResult:
    """YOLOv11檢測結果 - DR_F/STACK分類"""
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


@dataclass
class CalibrationStatus:
    """標定狀態"""
    intrinsic_loaded: bool = False
    extrinsic_loaded: bool = False
    transformer_valid: bool = False
    intrinsic_file: str = ""
    extrinsic_file: str = ""
    dist_coeffs_file: str = ""
    working_dir: str = ""


# ==================== YOLO模型管理器 ====================
class YOLOModelManager:
    """YOLO模型管理器 - 支援多模型動態切換"""
    
    def __init__(self, working_dir: str):
        self.working_dir = working_dir
        self.models = {}
        self.model_paths = {}
        self.current_model_id = 0  # 預設為0（未載入）
        self.model_switch_count = 0
        self.scan_model_files()
    
    def scan_model_files(self):
        """掃描工作目錄中的模型檔案"""
        try:
            print("🔍 掃描YOLO模型檔案...")
            
            for i in range(1, 21):
                patterns = [
                    f"model_{i}.pt",
                    f"best_{i}.pt", 
                    f"yolo_{i}.pt",
                    f"dr_model_{i}.pt"
                ]
                
                for pattern in patterns:
                    model_path = os.path.join(self.working_dir, pattern)
                    if os.path.exists(model_path):
                        self.model_paths[i] = model_path
                        print(f"✅ 發現模型{i}: {pattern}")
                        break
            
            print(f"📊 總共發現 {len(self.model_paths)} 個模型檔案")
            
            # 檢查原有的best.pt檔案，作為模型1的備選
            best_pt_path = os.path.join(self.working_dir, "best.pt")
            if os.path.exists(best_pt_path) and 1 not in self.model_paths:
                self.model_paths[1] = best_pt_path
                print(f"✅ 將best.pt指定為模型1")
            
        except Exception as e:
            print(f"❌ 掃描模型檔案失敗: {e}")
    
    def load_model(self, model_id: int) -> bool:
        """載入指定模型"""
        try:
            if model_id == 0:
                self.current_model_id = 0
                print("🔄 設置為未載入模型狀態")
                return True
            
            if model_id < 1 or model_id > 20:
                print(f"❌ 模型ID超出範圍: {model_id} (應為1-20)")
                return False
            
            if model_id not in self.model_paths:
                print(f"❌ 模型{model_id}檔案不存在")
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
        """獲取當前模型"""
        if self.current_model_id in self.models:
            return self.models[self.current_model_id]
        return None
    
    def get_available_model_count(self) -> int:
        """獲取可用模型數量"""
        return len(self.model_paths)
    
    def is_model_loaded(self) -> bool:
        """檢查是否有模型已載入"""
        return self.current_model_id > 0 and self.current_model_id in self.models


# ==================== YOLOv11檢測器 ====================
class YOLOv11Detector:
    """YOLOv11物件檢測器 - 支援多模型切換"""
    
    def __init__(self, working_dir: str, confidence_threshold: float = 0.8):
        self.working_dir = working_dir
        self.confidence_threshold = confidence_threshold
        self.model_manager = YOLOModelManager(working_dir)
        self.class_names = ['DR_F', 'stack']
        self._detection_count = 0

    @property
    def is_loaded(self) -> bool:
        """檢查是否有模型已載入"""
        return self.model_manager.is_model_loaded()
    
    def switch_model(self, model_id: int) -> bool:
        """切換模型"""
        return self.model_manager.load_model(model_id)
    
    def update_confidence_threshold(self, threshold: float):
        """更新置信度閾值"""
        self.confidence_threshold = max(0.1, min(1.0, threshold))
        print(f"🎯 置信度閾值更新為: {self.confidence_threshold}")
    
    def detect(self, image: np.ndarray) -> YOLODetectionResult:
        """執行YOLOv11檢測 - 記憶體優化版"""
        start_time = time.time()
        result = YOLODetectionResult()
        result.confidence_threshold = self.confidence_threshold
        result.model_id_used = self.model_manager.current_model_id
        
        try:
            current_model = self.model_manager.get_current_model()
            if current_model is None:
                if self.model_manager.current_model_id == 0:
                    result.error_message = "未指定任何模型 (MODEL_SELECT=0)"
                else:
                    result.error_message = f"模型{self.model_manager.current_model_id}未載入"
                return result
            
            # 執行推論並立即處理結果
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
                    
                    # 立即清理boxes和detections
                    del boxes
                    if hasattr(detections, 'boxes') and detections.boxes is not None:
                        del detections.boxes
                
            # 強制清理模型輸出
            if results:
                for res in results:
                    for attr in ['boxes', 'masks', 'keypoints', 'probs']:
                        if hasattr(res, attr):
                            delattr(res, attr)
                results.clear()
                del results
            
            # 定期強制垃圾回收
            self._detection_count += 1
            if self._detection_count % 10 == 0:
                collected = gc.collect()
                print(f"🧹 YOLO記憶體清理: 回收{collected}個物件")
                
        except Exception as e:
            result.error_message = f"YOLOv11檢測失敗: {e}"
            print(f"❌ YOLOv11檢測異常: {e}")
        
        result.processing_time = (time.time() - start_time) * 1000
        return result


# ==================== 座標轉換器 ====================
class CameraCoordinateTransformer:
    """相機座標轉換器"""
    
    def __init__(self):
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvec = None
        self.tvec = None
        self.rotation_matrix = None
        self.is_valid_flag = False
    
    def load_calibration_data(self, intrinsic_file: str, extrinsic_file: str) -> bool:
        """載入標定數據"""
        try:
            print(f"🔄 CameraCoordinateTransformer載入標定數據...")
            print(f"   內參檔案: {intrinsic_file}")
            print(f"   外參檔案: {extrinsic_file}")
            
            # 載入內參
            try:
                intrinsic_data = np.load(intrinsic_file, allow_pickle=True)
                
                if hasattr(intrinsic_data, 'shape') and intrinsic_data.shape == (3, 3):
                    self.camera_matrix = intrinsic_data
                    self.dist_coeffs = np.zeros((1, 5))
                    print(f"   ✅ 載入3x3相機矩陣，使用零值畸變係數")
                elif isinstance(intrinsic_data, dict):
                    self.camera_matrix = intrinsic_data['camera_matrix']
                    self.dist_coeffs = intrinsic_data.get('dist_coeffs', np.zeros((1, 5)))
                    print(f"   ✅ 從字典載入相機矩陣和畸變係數")
                elif hasattr(intrinsic_data, 'item') and callable(intrinsic_data.item):
                    dict_data = intrinsic_data.item()
                    if isinstance(dict_data, dict):
                        self.camera_matrix = dict_data['camera_matrix']
                        self.dist_coeffs = dict_data.get('dist_coeffs', np.zeros((1, 5)))
                        print(f"   ✅ 從字典項目載入相機矩陣和畸變係數")
                else:
                    self.camera_matrix = intrinsic_data
                    self.dist_coeffs = np.zeros((1, 5))
                    print(f"   ✅ 直接使用數據作為相機矩陣")
                    
            except Exception as e1:
                print(f"   ❌ 內參載入失敗: {e1}")
                return False
            
            # 檢查畸變係數檔案
            dist_coeffs_file = intrinsic_file.replace('camera_matrix', 'dist_coeffs')
            if os.path.exists(dist_coeffs_file) and dist_coeffs_file != intrinsic_file:
                try:
                    dist_data = np.load(dist_coeffs_file, allow_pickle=True)
                    if hasattr(dist_data, 'shape'):
                        self.dist_coeffs = dist_data
                        print(f"   ✅ 載入單獨的畸變係數檔案: {dist_data.shape}")
                except Exception as e:
                    print(f"   ⚠️ 載入畸變係數檔案失敗，使用零值: {e}")
            
            # 載入外參
            try:
                extrinsic_data = np.load(extrinsic_file, allow_pickle=True)
                
                if isinstance(extrinsic_data, dict):
                    self.rvec = extrinsic_data['rvec']
                    self.tvec = extrinsic_data['tvec']
                    print(f"   ✅ 從字典載入外參")
                elif hasattr(extrinsic_data, 'item') and callable(extrinsic_data.item) and extrinsic_data.shape == ():
                    dict_data = extrinsic_data.item()
                    if isinstance(dict_data, dict):
                        self.rvec = dict_data['rvec']
                        self.tvec = dict_data['tvec']
                        print(f"   ✅ 從字典項目載入外參")
                else:
                    print(f"   ❌ 未知的外參檔案格式")
                    return False
                
                # 計算旋轉矩陣
                self.rotation_matrix, _ = cv2.Rodrigues(self.rvec)
                
            except Exception as e2:
                print(f"   ❌ 外參載入失敗: {e2}")
                return False
            
            # 驗證載入的數據
            print(f"   📊 載入數據驗證:")
            print(f"      相機矩陣: {self.camera_matrix.shape}, det={np.linalg.det(self.camera_matrix):.2f}")
            print(f"      畸變係數: {self.dist_coeffs.shape}, 非零個數: {np.count_nonzero(self.dist_coeffs)}")
            print(f"      旋轉向量: {self.rvec.shape}, 範圍: [{self.rvec.min():.3f}, {self.rvec.max():.3f}]")
            print(f"      平移向量: {self.tvec.shape}, 範圍: [{self.tvec.min():.3f}, {self.tvec.max():.3f}]")
            print(f"      旋轉矩陣: {self.rotation_matrix.shape}, det={np.linalg.det(self.rotation_matrix):.3f}")
            
            self.is_valid_flag = True
            print(f"   ✅ 座標轉換器載入成功")
            return True
            
        except Exception as e:
            print(f"   ❌ 座標轉換器載入失敗: {e}")
            import traceback
            print(f"   詳細錯誤: {traceback.format_exc()}")
            return False
    
    def pixel_to_world(self, pixel_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """像素座標轉世界座標"""
        if not self.is_valid_flag:
            print(f"❌ 標定數據無效，無法進行座標轉換")
            return []
        
        try:
            world_coords = []
            
            for px, py in pixel_coords:
                print(f"🔄 轉換像素座標: ({px:.1f}, {py:.1f})")
                
                # 去畸變處理
                pixel_point = np.array([[[float(px), float(py)]]], dtype=np.float32)
                undistorted_points = cv2.undistortPoints(
                    pixel_point, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                # 獲取歸一化座標
                x_norm, y_norm = undistorted_points[0][0]
                print(f"   去畸變後歸一化座標: ({x_norm:.6f}, {y_norm:.6f})")
                
                # 構建歸一化齊次座標
                normalized_coords = np.array([x_norm, y_norm, 1.0])
                
                # 計算深度係數
                R3 = self.rotation_matrix[2, :]
                denominator = np.dot(R3, normalized_coords)
                
                if abs(denominator) < 1e-6:
                    print(f"   ⚠️ 分母接近零，跳過此點: denominator={denominator}")
                    continue
                
                depth_scale = (0 - self.tvec[2, 0]) / denominator
                print(f"   深度係數: {depth_scale:.6f}")
                
                # 計算相機座標系中的3D點
                camera_point = depth_scale * normalized_coords
                print(f"   相機座標系點: ({camera_point[0]:.3f}, {camera_point[1]:.3f}, {camera_point[2]:.3f})")
                
                # 轉換到世界座標系
                tvec_3d = self.tvec.reshape(3)
                translated_point = camera_point - tvec_3d
                world_point_3d = np.dot(self.rotation_matrix.T, translated_point)
                
                world_x = world_point_3d[0]
                world_y = world_point_3d[1]
                
                print(f"   ✅ 世界座標: ({world_x:.3f}, {world_y:.3f}) mm")
                
                world_coords.append((float(world_x), float(world_y)))
            
            print(f"✅ 座標轉換完成，共轉換{len(world_coords)}個點")
            return world_coords
            
        except Exception as e:
            print(f"❌ 座標轉換失敗: {e}")
            import traceback
            print(f"詳細錯誤堆疊: {traceback.format_exc()}")
            return []
    
    def is_valid(self) -> bool:
        """檢查轉換器是否有效"""
        return (self.is_valid_flag and 
                self.camera_matrix is not None and 
                self.dist_coeffs is not None and 
                self.rvec is not None and 
                self.tvec is not None and 
                self.rotation_matrix is not None)


# ==================== 標定管理器 ====================
class CalibrationManager:
    """標定檔案管理器"""
    
    def __init__(self, working_dir: str):
        self.working_dir = working_dir
        self.transformer = CameraCoordinateTransformer()
        self.status = CalibrationStatus()
        self.status.working_dir = working_dir
    
    def auto_load_calibration_files(self) -> bool:
        """自動載入標定檔案"""
        try:
            print(f"🔍 自動掃描標定檔案目錄: {self.working_dir}")
            
            if not os.path.exists(self.working_dir):
                print(f"❌ 工作目錄不存在: {self.working_dir}")
                return False
            
            all_files = os.listdir(self.working_dir)
            npy_files = [f for f in all_files if f.endswith('.npy')]
            
            print(f"📁 發現 {len(npy_files)} 個NPY檔案")
            
            if not npy_files:
                print(f"❌ 未發現標定檔案")
                return False
            
            # 自動匹配標定檔案
            camera_matrix_file = None
            extrinsic_file = None
            
            # 尋找相機矩陣檔案
            for file in npy_files:
                if any(keyword in file.lower() for keyword in ['camera_matrix', 'camera', 'intrinsic']):
                    camera_matrix_file = file
                    break
            
            # 尋找外參檔案
            for file in npy_files:
                if any(keyword in file.lower() for keyword in ['extrinsic', '外参', 'external']):
                    extrinsic_file = file
                    break
            
            if not camera_matrix_file or not extrinsic_file:
                print(f"❌ 未找到必要的標定檔案")
                print(f"   相機矩陣檔案: {camera_matrix_file}")
                print(f"   外參檔案: {extrinsic_file}")
                return False
            
            print(f"📋 自動載入標定檔案:")
            print(f"   相機矩陣: {camera_matrix_file}")
            print(f"   外參檔案: {extrinsic_file}")
            
            # 載入標定數據
            success = self.transformer.load_calibration_data(
                os.path.join(self.working_dir, camera_matrix_file),
                os.path.join(self.working_dir, extrinsic_file)
            )
            
            if success:
                self.status.intrinsic_loaded = True
                self.status.extrinsic_loaded = True
                self.status.transformer_valid = True
                self.status.intrinsic_file = camera_matrix_file
                self.status.extrinsic_file = extrinsic_file
                
                print(f"✅ 標定數據自動載入成功")
                return True
            else:
                print(f"❌ 標定數據載入失敗")
                return False
                
        except Exception as e:
            print(f"❌ 自動載入標定檔案失敗: {e}")
            return False
    
    def get_status(self) -> Dict[str, Any]:
        """獲取標定狀態"""
        return {
            'intrinsic_loaded': self.status.intrinsic_loaded,
            'extrinsic_loaded': self.status.extrinsic_loaded,
            'transformer_valid': self.status.transformer_valid,
            'intrinsic_file': self.status.intrinsic_file,
            'extrinsic_file': self.status.extrinsic_file,
            'dist_coeffs_file': self.status.dist_coeffs_file,
            'working_dir': self.status.working_dir
        }


# ==================== 狀態機 ====================
class SystemStateMachine:
    """系統狀態機"""
    
    def __init__(self):
        self.lock = threading.Lock()
        self.status_register = 0b0001  # 初始Ready=1
    
    def get_bit(self, bit_pos: StatusBits) -> bool:
        """獲取狀態位"""
        with self.lock:
            return bool(self.status_register & (1 << bit_pos))
    
    def set_bit(self, bit_pos: StatusBits, value: bool):
        """設置狀態位"""
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
    
    def reset_to_idle(self):
        """重置到空閒狀態"""
        with self.lock:
            self.status_register = 0b0001  # Ready=1, 其他=0


# ==================== Modbus TCP Client服務 ====================
class EnhancedModbusTcpClientService:
    """增強版Modbus TCP Client服務 - 純Modbus版本"""
    
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
        self._sync_counter = 0
        self._cleanup_frequency = 100
        
        # 記憶體監控
        self.memory_monitor_thread = None
        self.memory_monitor_running = False
        self.last_memory_update = 0
        self.memory_update_interval = 60  # 1分鐘更新一次
        
        # CCD1 Modbus寄存器映射 (基地址200) - 新增寄存器
        self.REGISTERS = {
            # 控制寄存器 (200-201)
            'CONTROL_COMMAND': 200,
            'STATUS_REGISTER': 201,
            
            # 模型管理寄存器 (202)
            'MODEL_SELECT': 202,
            
            # 完成標誌寄存器 (203-206)
            'CAPTURE_COMPLETE': 203,
            'DETECT_COMPLETE': 204,
            'OPERATION_SUCCESS': 205,
            'ERROR_CODE': 206,
            
            # YOLOv11檢測參數寄存器 (210-219)
            'CONFIDENCE_HIGH': 210,
            'CONFIDENCE_LOW': 211,
            
            # YOLOv11檢測結果寄存器 (240-259)
            'DR_F_COUNT': 240,
            'STACK_COUNT': 242,
            'TOTAL_DETECTIONS': 243,
            'DETECTION_SUCCESS': 244,
            
            # DR_F座標寄存器 (245-254)
            'DR_F_1_X': 245, 'DR_F_1_Y': 246,
            'DR_F_2_X': 247, 'DR_F_2_Y': 248,
            'DR_F_3_X': 249, 'DR_F_3_Y': 250,
            'DR_F_4_X': 251, 'DR_F_4_Y': 252,
            'DR_F_5_X': 253, 'DR_F_5_Y': 254,
            
            'STACK_1_X': 257,
            'STACK_1_Y': 258,
            'MODEL_ID_USED': 259,
            
            # 世界座標寄存器 (260-279)
            'WORLD_COORD_VALID': 260,
            'DR_F_1_WORLD_X_HIGH': 261, 'DR_F_1_WORLD_X_LOW': 262,
            'DR_F_1_WORLD_Y_HIGH': 263, 'DR_F_1_WORLD_Y_LOW': 264,
            'DR_F_2_WORLD_X_HIGH': 265, 'DR_F_2_WORLD_X_LOW': 266,
            'DR_F_2_WORLD_Y_HIGH': 267, 'DR_F_2_WORLD_Y_LOW': 268,
            'DR_F_3_WORLD_X_HIGH': 269, 'DR_F_3_WORLD_X_LOW': 270,
            'DR_F_3_WORLD_Y_HIGH': 271, 'DR_F_3_WORLD_Y_LOW': 272,
            'DR_F_4_WORLD_X_HIGH': 273, 'DR_F_4_WORLD_X_LOW': 274,
            'DR_F_4_WORLD_Y_HIGH': 275, 'DR_F_4_WORLD_Y_LOW': 276,
            'DR_F_5_WORLD_X_HIGH': 277, 'DR_F_5_WORLD_X_LOW': 278,
            'DR_F_5_WORLD_Y_HIGH': 279, 'DR_F_5_WORLD_Y_LOW': 280,
            
            # 統計資訊寄存器 (281-299)
            'LAST_CAPTURE_TIME': 281,
            'LAST_PROCESS_TIME': 282,
            'LAST_TOTAL_TIME': 283,
            'OPERATION_COUNT': 284,
            'ERROR_COUNT': 285,
            'CONNECTION_COUNT': 286,
            'MODEL_SWITCH_COUNT': 287,
            
            # 版本資訊
            'VERSION_MAJOR': 290,
            'VERSION_MINOR': 291,
            'UPTIME_HOURS': 292,
            'UPTIME_MINUTES': 293,
            
            # 新增寄存器
            'MEMORY_USAGE_MB': 295,      # 記憶體使用量(MB)
            'SYSTEM_RELOAD': 296,        # 系統重載觸發
            'RELOAD_STATUS': 297,        # 重載狀態標誌
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
        
        # 系統重載控制
        self.last_reload_trigger = 0
    
    def set_vision_controller(self, controller):
        """設置視覺控制器引用"""
        self.vision_controller = controller
    
    def connect(self) -> bool:
        """連接到Modbus TCP服務器"""
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
                
                # 寫入初始狀態和預設值
                self._write_initial_status_and_defaults()
                
                print(f"✅ Modbus TCP Client連接成功: {self.server_ip}:{self.server_port}")
                return True
            else:
                print(f"❌ Modbus TCP連接失敗: {self.server_ip}:{self.server_port}")
                self.connected = False
                return False
                
        except Exception as e:
            print(f"❌ Modbus TCP連接異常: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        self.stop_sync()
        self.stop_memory_monitor()
        
        if self.client and self.connected:
            try:
                self.write_register('STATUS_REGISTER', 0)
                self.client.close()
                print("🔌 Modbus TCP Client已斷開連接")
            except:
                pass
        
        self.connected = False
        self.client = None
    
    def start_sync(self):
        """啟動同步線程"""
        if self.sync_running:
            return
        
        self.sync_running = True
        self.sync_thread = threading.Thread(target=self._handshake_sync_loop, daemon=True)
        self.sync_thread.start()
        
        # 同時啟動記憶體監控線程
        self.start_memory_monitor()
        
        print("✅ Modbus握手同步線程已啟動")
    
    def stop_sync(self):
        """停止同步線程"""
        if self.sync_running:
            self.sync_running = False
            if self.sync_thread and self.sync_thread.is_alive():
                self.sync_thread.join(timeout=2.0)
            print("🛑 Modbus握手同步線程已停止")
    
    def start_memory_monitor(self):
        """啟動記憶體監控線程"""
        if self.memory_monitor_running:
            return
        
        self.memory_monitor_running = True
        self.memory_monitor_thread = threading.Thread(target=self._memory_monitor_loop, daemon=True)
        self.memory_monitor_thread.start()
        print("✅ 記憶體監控線程已啟動")
    
    def stop_memory_monitor(self):
        """停止記憶體監控線程"""
        if self.memory_monitor_running:
            self.memory_monitor_running = False
            if self.memory_monitor_thread and self.memory_monitor_thread.is_alive():
                self.memory_monitor_thread.join(timeout=2.0)
            print("🛑 記憶體監控線程已停止")
    
    def _memory_monitor_loop(self):
        """記憶體監控循環 - 每分鐘更新一次"""
        print("🧹 記憶體監控線程開始運行...")
        
        while self.memory_monitor_running and self.connected:
            try:
                current_time = time.time()
                
                # 每分鐘更新一次記憶體使用量
                if current_time - self.last_memory_update >= self.memory_update_interval:
                    self._update_memory_usage()
                    self.last_memory_update = current_time
                
                # 短暫休眠
                time.sleep(5.0)  # 5秒檢查一次，但只在需要時更新
                
            except Exception as e:
                print(f"❌ 記憶體監控線程錯誤: {e}")
                time.sleep(10.0)
        
        print("⏹️ 記憶體監控線程已退出")
    
    def _update_memory_usage(self):
        """更新記憶體使用量到Modbus寄存器"""
        try:
            # 獲取當前進程記憶體使用量
            process = psutil.Process()
            memory_mb = process.memory_info().rss / 1024 / 1024
            
            # 無條件捨去，轉為整數
            memory_mb_int = int(memory_mb)
            
            # 寫入到Modbus寄存器
            self.write_register('MEMORY_USAGE_MB', memory_mb_int)
            
            print(f"🧹 記憶體使用量更新: {memory_mb_int}MB (原始值: {memory_mb:.2f}MB)")
            
        except Exception as e:
            print(f"❌ 更新記憶體使用量失敗: {e}")
    
    def _write_initial_status_and_defaults(self):
        """寫入初始狀態並檢查預設值"""
        try:
            print("📊 寫入初始狀態並檢查預設值...")
            
            # 檢查並寫入預設值
            self._check_and_write_defaults()
            
            # 版本資訊
            self.write_register('VERSION_MAJOR', 5)  # YOLOv11版本
            self.write_register('VERSION_MINOR', 1)  # 純Modbus版本
            
            # 計數器
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            self.write_register('CONNECTION_COUNT', self.connection_count)
            
            # 重載狀態初始化
            self.write_register('RELOAD_STATUS', 0)
            
            # 立即更新一次記憶體使用量
            self._update_memory_usage()
            
            print("📊 初始狀態和預設值寫入完成")
            
        except Exception as e:
            print(f"❌ 寫入初始狀態失敗: {e}")
    
    def _check_and_write_defaults(self):
        """檢查Modbus寄存器，如果為0則寫入預設值"""
        try:
            print("🔍 檢查Modbus寄存器預設值...")
            
            # 檢查模型選擇寄存器
            model_select = self.read_register('MODEL_SELECT')
            if model_select is None or model_select == 0:
                print("📝 MODEL_SELECT為0，寫入預設值: 0 (未載入模型)")
                self.write_register('MODEL_SELECT', 0)
            else:
                print(f"📋 MODEL_SELECT現有值: {model_select}")
            
            # 檢查置信度閾值
            conf_high = self.read_register('CONFIDENCE_HIGH')
            conf_low = self.read_register('CONFIDENCE_LOW')
            
            if (conf_high is None or conf_high == 0) and (conf_low is None or conf_low == 0):
                # 預設置信度0.8 = 8000
                confidence_int = 8000
                print(f"📝 置信度閾值為0，寫入預設值: {confidence_int} (0.8)")
                self.write_register('CONFIDENCE_HIGH', (confidence_int >> 16) & 0xFFFF)
                self.write_register('CONFIDENCE_LOW', confidence_int & 0xFFFF)
            else:
                current_confidence_int = (conf_high << 16) + conf_low if conf_high and conf_low else 0
                current_confidence = current_confidence_int / 10000.0
                print(f"📋 置信度閾值現有值: {current_confidence:.2f}")
            
            print("✅ 預設值檢查完成")
            
        except Exception as e:
            print(f"❌ 檢查預設值失敗: {e}")
    
    def _handshake_sync_loop(self):
        """握手同步循環"""
        print("🔄 增強版握手同步線程開始運行...")
        
        while self.sync_running and self.connected:
            try:
                # 1. 更新狀態寄存器到PLC
                self._update_status_register()
                
                # 2. 處理模型管理指令
                self._handle_model_management()
                
                # 3. 處理系統重載指令 (新增)
                self._handle_system_reload()
                
                # 4. 讀取控制指令並處理握手邏輯
                self._handle_control_command_enhanced()
                
                # 5. 處理完成狀態邏輯
                self._handle_completion_status()
                
                # 6. 更新統計資訊
                self._update_statistics()
                
                # 定期記憶體清理
                self._sync_counter += 1
                if self._sync_counter >= self._cleanup_frequency:
                    collected = gc.collect()
                    print(f"🧹 同步線程記憶體清理: 回收{collected}個物件")
                    self._sync_counter = 0
                
                # 短暫休眠
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
        print("⏹️ 增強版同步線程已退出")
    
    def _handle_system_reload(self):
        """處理系統重載指令"""
        try:
            reload_trigger = self.read_register('SYSTEM_RELOAD')
            if reload_trigger is None:
                return
            
            # 檢查是否有新的重載請求
            if reload_trigger == 1 and reload_trigger != self.last_reload_trigger:
                print("🔄 收到系統重載指令，開始重載程序...")
                
                # 設置重載狀態
                self.write_register('RELOAD_STATUS', 1)
                self.last_reload_trigger = reload_trigger
                
                # 異步執行重載
                reload_thread = threading.Thread(
                    target=self._execute_system_reload,
                    daemon=True
                )
                reload_thread.start()
                
            elif reload_trigger == 0 and self.last_reload_trigger == 1:
                # 重載觸發器已清零
                self.last_reload_trigger = 0
                
        except Exception as e:
            print(f"❌ 處理系統重載指令失敗: {e}")
    
    def _execute_system_reload(self):
        """執行系統重載"""
        try:
            print("🔄 執行系統重載程序...")
            
            # 1. 強制垃圾回收
            print("🧹 執行強制垃圾回收...")
            collected = gc.collect()
            print(f"🧹 垃圾回收完成，回收{collected}個物件")
            
            # 2. 重新初始化視覺控制器
            if self.vision_controller:
                print("🔄 重新初始化視覺控制器...")
                
                # 重新載入標定檔案
                calibration_success = self.vision_controller.calibration_manager.auto_load_calibration_files()
                if calibration_success:
                    print("✅ 標定檔案重新載入成功")
                else:
                    print("⚠️ 標定檔案重新載入失敗")
                
                # 重新初始化相機
                camera_success = self.vision_controller.initialize_camera()
                if camera_success:
                    print("✅ 相機重新初始化成功")
                    self.vision_controller.state_machine.set_initialized(True)
                    self.vision_controller.state_machine.set_alarm(False)
                    self.vision_controller.state_machine.set_ready(True)
                else:
                    print("❌ 相機重新初始化失敗")
                    self.vision_controller.state_machine.set_alarm(True)
                
                # 重新同步置信度閾值
                confidence = self.read_confidence_threshold()
                if self.vision_controller.yolo_detector:
                    self.vision_controller.yolo_detector.update_confidence_threshold(confidence)
                    print(f"🎯 置信度閾值重新同步: {confidence}")
            
            # 3. 更新記憶體使用量
            self._update_memory_usage()
            
            print("✅ 系統重載完成")
            
        except Exception as e:
            print(f"❌ 系統重載失敗: {e}")
            if self.vision_controller:
                self.vision_controller.state_machine.set_alarm(True)
        finally:
            # 清除重載狀態
            self.write_register('RELOAD_STATUS', 0)
            print("🔄 重載狀態已清除")
    
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
                        print(f"✅ 模型切換成功: 當前模型{model_select}")
                        switch_count = self.vision_controller.yolo_detector.model_manager.model_switch_count
                        self.write_register('MODEL_SWITCH_COUNT', switch_count)
                    else:
                        print(f"❌ 模型切換失敗: 模型{model_select}")
                        self.write_register('ERROR_CODE', 10)
                else:
                    print(f"❌ 無效的模型ID: {model_select}")
                    self.write_register('ERROR_CODE', 11)
                    
        except Exception as e:
            print(f"❌ 處理模型管理指令失敗: {e}")
    
    def _handle_control_command_enhanced(self):
        """處理控制指令握手邏輯 - 增強版"""
        try:
            current_command = self.read_register('CONTROL_COMMAND')
            if current_command is None:
                return
            
            if (current_command != self.last_control_command and 
                current_command != 0 and 
                not self.command_processing):
                
                print(f"📋 收到新控制指令: {current_command} (上次: {self.last_control_command})")
                self.last_control_command = current_command
                self.current_command = current_command
                self.command_processing = True
                self.command_execution_time = time.time()
                
                self._clear_completion_flags()
                
                command_thread = threading.Thread(
                    target=self._execute_command_enhanced, 
                    args=(ControlCommand(current_command),),
                    daemon=True
                )
                command_thread.start()
                
            elif current_command == 0 and self.last_control_command != 0:
                print(f"📋 控制指令已清零，準備恢復Ready狀態")
                self._handle_command_clear()
                
        except Exception as e:
            print(f"❌ 處理控制指令失敗: {e}")
    
    def _execute_command_enhanced(self, command: ControlCommand):
        """執行控制指令 - 增強版"""
        try:
            print(f"🚀 開始處理控制指令: {command}")
            
            if not self.vision_controller:
                print("❌ 視覺控制器不存在")
                self._set_error_state(1, "視覺控制器不存在")
                return
            
            if not self.vision_controller.state_machine.is_ready():
                print("⚠️ 系統未Ready，忽略指令")
                self._set_error_state(2, "系統未Ready")
                return
            
            self.vision_controller.state_machine.set_running(True)
            self.vision_controller.state_machine.set_ready(False)
            print(f"🔄 狀態變更: Ready=0, Running=1")
            
            running_start = time.time()
            
            result = None
            if command == ControlCommand.CAPTURE:
                result = self._handle_capture_command_enhanced()
            elif command == ControlCommand.CAPTURE_DETECT:
                result = self._handle_capture_detect_command_enhanced()
            elif command == ControlCommand.INITIALIZE:
                result = self._handle_initialize_command_enhanced()
            else:
                print(f"⚠️ 未知控制指令: {command}")
                self._set_error_state(3, f"未知指令: {command}")
                return
            
            running_duration = time.time() - running_start
            if running_duration < self.min_running_duration:
                remaining_time = self.min_running_duration - running_duration
                print(f"⏱️ Running狀態延長 {remaining_time:.2f} 秒以確保可見性")
                time.sleep(remaining_time)
            
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
            self.vision_controller.state_machine.set_running(False)
            self.command_processing = False
            self.completion_start_time = time.time()
            print(f"🔄 狀態變更: Running=0, 等待指令清零後恢復Ready")
    
    def _handle_capture_detect_command_enhanced(self):
        """處理拍照+檢測指令 - 增強版"""
        try:
            print("🔍 執行拍照+YOLOv11檢測指令")
            result = self.vision_controller.capture_and_detect()
            
            if result and result.success:
                self.update_detection_results(result)
                self.write_register('CAPTURE_COMPLETE', 1)
                self.write_register('DETECT_COMPLETE', 1)
                print(f"✅ YOLOv11檢測成功，DR_F={result.dr_f_count}")
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
            self.write_register('CAPTURE_COMPLETE', 0)
            self.write_register('DETECT_COMPLETE', 0)
            return False
    
    def _handle_capture_command_enhanced(self):
        """處理拍照指令 - 增強版"""
        try:
            print("📸 執行拍照指令")
            image, capture_time = self.vision_controller.capture_image()
            
            if image is not None:
                self.write_register('LAST_CAPTURE_TIME', int(capture_time * 1000))
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
    
    def _handle_initialize_command_enhanced(self):
        """處理初始化指令 - 增強版"""
        try:
            print("🔄 執行系統初始化指令")
            
            # 重新初始化相機
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
    
    def _clear_completion_flags(self):
        """清除完成標誌"""
        try:
            self.write_register('CAPTURE_COMPLETE', 0)
            self.write_register('DETECT_COMPLETE', 0)
            self.write_register('OPERATION_SUCCESS', 0)
            self.write_register('ERROR_CODE', 0)
        except Exception as e:
            print(f"❌ 清除完成標誌失敗: {e}")
    
    def _handle_completion_status(self):
        """處理完成狀態邏輯"""
        try:
            if (self.completion_start_time > 0 and 
                time.time() - self.completion_start_time > self.completion_hold_time):
                
                current_command = self.read_register('CONTROL_COMMAND')
                if current_command == 0:
                    print("⏰ 完成狀態保持時間結束，準備下一次操作")
                    self.completion_start_time = 0
                    
        except Exception as e:
            print(f"❌ 處理完成狀態失敗: {e}")
    
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
                
            self.write_register('STACK_1_X', 0)
            self.write_register('STACK_1_Y', 0)
                
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
            
            print(f"📊 檢測數量寫入寄存器:")
            print(f"   240(DR_F_COUNT) = {result.dr_f_count}")
            print(f"   242(STACK_COUNT) = {result.stack_count}")
            print(f"   243(TOTAL_DETECTIONS) = {result.total_detections}")
            print(f"   244(DETECTION_SUCCESS) = {1 if result.success else 0}")
            print(f"   259(MODEL_ID_USED) = {result.model_id_used}")

            # 寫入DR_F座標 (最多5個)
            for i in range(5):
                if i < len(result.dr_f_coords):
                    x, y = result.dr_f_coords[i]
                    self.write_register(f'DR_F_{i+1}_X', int(float(x)))
                    self.write_register(f'DR_F_{i+1}_Y', int(float(y)))
                    print(f"   {245+i*2}(DR_F_{i+1}_X) = {int(float(x))}")
                    print(f"   {246+i*2}(DR_F_{i+1}_Y) = {int(float(y))}")
                else:
                    self.write_register(f'DR_F_{i+1}_X', 0)
                    self.write_register(f'DR_F_{i+1}_Y', 0)
            
            # 寫入STACK座標 (第1個)
            if result.stack_coords:
                x, y = result.stack_coords[0]
                self.write_register('STACK_1_X', int(float(x)))
                self.write_register('STACK_1_Y', int(float(y)))
                print(f"   257(STACK_1_X) = {int(float(x))}")
                print(f"   258(STACK_1_Y) = {int(float(y))}")
            else:
                self.write_register('STACK_1_X', 0)
                self.write_register('STACK_1_Y', 0)
            
            # 寫入時間統計
            self.write_register('LAST_CAPTURE_TIME', int(float(result.capture_time)))
            self.write_register('LAST_PROCESS_TIME', int(float(result.processing_time)))
            self.write_register('LAST_TOTAL_TIME', int(float(result.total_time)))
            
            # 世界座標轉換
            if (result.success and result.dr_f_coords and
                self.vision_controller and 
                self.vision_controller.calibration_manager.transformer.is_valid()):
                
                world_coords = result.dr_f_world_coords
                
                if world_coords:
                    self.write_register('WORLD_COORD_VALID', 1)
                    print(f"🌍 世界座標轉換成功，共{len(world_coords)}個DR_F目標")
                    
                    # 寫入前5個世界座標 (×100存儲)
                    for i in range(min(5, len(world_coords))):
                        world_x, world_y = world_coords[i]
                        
                        # 轉換為32位整數並分高低位存儲
                        world_x_int = int(float(world_x) * 100)
                        world_y_int = int(float(world_y) * 100)
                        
                        print(f"   處理 DR_F {i+1}: 原始值=({world_x:.2f}, {world_y:.2f})")
                        print(f"   ×100後整數值: x_int={world_x_int}, y_int={world_y_int}")
                        
                        # 處理負數
                        if world_x_int < 0:
                            world_x_uint32 = (2**32) + world_x_int
                        else:
                            world_x_uint32 = world_x_int
                            
                        if world_y_int < 0:
                            world_y_uint32 = (2**32) + world_y_int
                        else:
                            world_y_uint32 = world_y_int
                        
                        print(f"   32位無符號值: x_uint32={world_x_uint32}, y_uint32={world_y_uint32}")
                        
                        # 分割32位為高16位和低16位
                        world_x_high = (world_x_uint32 >> 16) & 0xFFFF
                        world_x_low = world_x_uint32 & 0xFFFF
                        world_y_high = (world_y_uint32 >> 16) & 0xFFFF
                        world_y_low = world_y_uint32 & 0xFFFF
                        
                        print(f"   分割結果: x_high={world_x_high}, x_low={world_x_low}")
                        print(f"   分割結果: y_high={world_y_high}, y_low={world_y_low}")
                        
                        # 驗證重組
                        reconstructed_x = (world_x_high << 16) | world_x_low
                        reconstructed_y = (world_y_high << 16) | world_y_low
                        
                        if reconstructed_x >= 2**31:
                            reconstructed_x_signed = reconstructed_x - 2**32
                        else:
                            reconstructed_x_signed = reconstructed_x
                            
                        if reconstructed_y >= 2**31:
                            reconstructed_y_signed = reconstructed_y - 2**32
                        else:
                            reconstructed_y_signed = reconstructed_y
                        
                        reconstructed_x_mm = reconstructed_x_signed / 100.0
                        reconstructed_y_mm = reconstructed_y_signed / 100.0
                        
                        print(f"   驗證重組: ({reconstructed_x_mm:.2f}, {reconstructed_y_mm:.2f}) mm")
                        
                        # 寫入世界座標寄存器
                        self.write_register(f'DR_F_{i+1}_WORLD_X_HIGH', world_x_high)
                        self.write_register(f'DR_F_{i+1}_WORLD_X_LOW', world_x_low)
                        self.write_register(f'DR_F_{i+1}_WORLD_Y_HIGH', world_y_high)
                        self.write_register(f'DR_F_{i+1}_WORLD_Y_LOW', world_y_low)
                        
                        print(f"   DR_F {i+1} 世界座標寫入:")
                        print(f"     {261+i*4}(WORLD_X_HIGH) = {world_x_high}")
                        print(f"     {262+i*4}(WORLD_X_LOW) = {world_x_low}")
                        print(f"     {263+i*4}(WORLD_Y_HIGH) = {world_y_high}")
                        print(f"     {264+i*4}(WORLD_Y_LOW) = {world_y_low}")
                        print(f"     實際值: ({world_x:.2f}, {world_y:.2f}) mm")
                        print(f"     驗證值: ({reconstructed_x_mm:.2f}, {reconstructed_y_mm:.2f}) mm")
                        print()
                    
                    print(f"✅ 共寫入{len(world_coords)}個DR_F世界座標到Modbus寄存器")
                    
                    # 清空未使用的世界座標寄存器
                    for i in range(len(world_coords), 5):
                        self.write_register(f'DR_F_{i+1}_WORLD_X_HIGH', 0)
                        self.write_register(f'DR_F_{i+1}_WORLD_X_LOW', 0)
                        self.write_register(f'DR_F_{i+1}_WORLD_Y_HIGH', 0)
                        self.write_register(f'DR_F_{i+1}_WORLD_Y_LOW', 0)
                else:
                    print(f"❌ 世界座標轉換失敗")
                    self.write_register('WORLD_COORD_VALID', 0)
            else:
                self.write_register('WORLD_COORD_VALID', 0)
            
        except Exception as e:
            print(f"❌ 更新檢測結果到PLC失敗: {e}")
    
    def _update_status_register(self):
        """更新狀態寄存器到PLC"""
        try:
            if self.vision_controller:
                status_value = self.vision_controller.state_machine.status_register
                self.write_register('STATUS_REGISTER', status_value)
        except:
            pass
    
    def _update_statistics(self):
        """更新統計資訊"""
        try:
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            self.write_register('CONNECTION_COUNT', self.connection_count)
            
            # 更新運行時間
            uptime_total_minutes = int((time.time() - self.start_time) / 60)
            uptime_hours = uptime_total_minutes // 60
            uptime_minutes = uptime_total_minutes % 60
            
            self.write_register('UPTIME_HOURS', uptime_hours)
            self.write_register('UPTIME_MINUTES', uptime_minutes)
            
        except:
            pass
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器"""
        if not self.connected or not self.client or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            else:
                return None
                
        except Exception as e:
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器"""
        if not self.connected or not self.client or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.client.write_register(address, value, slave=1)
            
            if not result.isError():
                return True
            else:
                return False
                
        except Exception as e:
            return False
    
    def read_confidence_threshold(self) -> float:
        """讀取置信度閾值"""
        try:
            high = self.read_register('CONFIDENCE_HIGH') or 0
            low = self.read_register('CONFIDENCE_LOW') or 8000
            confidence_int = (high << 16) + low
            return confidence_int / 10000.0
        except:
            return 0.8
    
    def get_connection_status(self) -> Dict[str, Any]:
        """獲取連接狀態"""
        return {
            'connected': self.connected,
            'server_ip': self.server_ip,
            'server_port': self.server_port,
            'sync_running': self.sync_running,
            'memory_monitor_running': self.memory_monitor_running,
            'operation_count': self.operation_count,
            'error_count': self.error_count,
            'connection_count': self.connection_count,
            'uptime_seconds': int(time.time() - self.start_time)
        }


# ==================== 主控制器 ====================
class CCD1VisionController:
    """CCD1視覺檢測主控制器 - 純Modbus版本"""
    
    def __init__(self):
        # 基本配置
        self.working_dir = os.path.dirname(os.path.abspath(__file__))
        self.server_ip = "127.0.0.1"
        self.server_port = 502
        self.camera_ip = "192.168.1.8"
        
        # 核心組件
        self.state_machine = SystemStateMachine()
        self.camera_manager: Optional[OptimizedCameraManager] = None
        self.calibration_manager = CalibrationManager(self.working_dir)
        self.yolo_detector = None
        
        # 圖像緩存
        self.last_image: Optional[np.ndarray] = None
        self.last_result: Optional[YOLODetectionResult] = None
        
        # Modbus客戶端
        self.modbus_client = EnhancedModbusTcpClientService(self.server_ip, self.server_port)
        self.modbus_client.set_vision_controller(self)
        
        # 統計信息
        self.operation_count = 0
        self.error_count = 0
        
        # 記憶體管理
        self._memory_cleanup_counter = 0
        self._memory_cleanup_frequency = 50
        self._last_comprehensive_cleanup = time.time()
        self._comprehensive_cleanup_interval = 300  # 5分鐘
        
        # 設置日誌
        self.logger = logging.getLogger("CCD1Vision")
        self.logger.setLevel(logging.INFO)
        
        # 檢查並初始化YOLOv11
        self._check_yolo_availability()
        
        # 自動初始化所有組件
        self._auto_initialize_all_components()
    
    def _check_yolo_availability(self):
        """檢查YOLOv11模型可用性"""
        if not YOLO_AVAILABLE:
            error_msg = "YOLOv11模組不可用，請安裝ultralytics"
            print(f"❌ {error_msg}")
            self.state_machine.set_alarm(True)
            raise RuntimeError(error_msg)
        
        print(f"✅ YOLOv11模組可用")
        self.yolo_detector = YOLOv11Detector(self.working_dir, 0.8)
        
        if self.yolo_detector.model_manager.get_available_model_count() > 0:
            print("✅ YOLOv11檢測器初始化成功")
        else:
            error_msg = "未發現任何YOLOv11模型檔案"
            print(f"❌ {error_msg}")
            self.state_machine.set_alarm(True)
            raise RuntimeError(error_msg)
    
    def _auto_initialize_all_components(self):
        """自動初始化所有組件"""
        print("🚀 開始自動初始化所有組件...")
        
        try:
            # 1. 自動連接Modbus
            print("📡 自動連接Modbus TCP服務器...")
            modbus_success = self.modbus_client.connect()
            if modbus_success:
                self.modbus_client.start_sync()
                print("✅ Modbus TCP連接成功並啟動同步")
                
                # 讀取並同步置信度閾值
                confidence = self.modbus_client.read_confidence_threshold()
                if self.yolo_detector:
                    self.yolo_detector.update_confidence_threshold(confidence)
                    print(f"🎯 置信度閾值已同步: {confidence}")
            else:
                print("❌ Modbus TCP連接失敗")
            
            # 2. 自動載入標定檔案
            print("📐 自動載入標定檔案...")
            calibration_success = self.calibration_manager.auto_load_calibration_files()
            if calibration_success:
                print("✅ 標定檔案自動載入成功")
            else:
                print("⚠️ 標定檔案自動載入失敗，但系統可以繼續運行")
            
            # 3. 自動初始化相機
            print("📷 自動初始化相機...")
            camera_success = self.initialize_camera()
            if camera_success:
                print("✅ 相機自動初始化成功")
                self.state_machine.set_initialized(True)
                
                # 只有在沒有Alarm時才設置Ready
                if not self.state_machine.is_alarm():
                    self.state_machine.set_ready(True)
                    print("🎯 系統完全就緒，進入Ready狀態")
                else:
                    print("⚠️ 系統有Alarm，等待解決後才能進入Ready狀態")
            else:
                print("❌ 相機自動初始化失敗")
                self.state_machine.set_alarm(True)
            
            # 4. 檢查整體初始化狀態
            if modbus_success and calibration_success and camera_success:
                print("🎉 所有組件自動初始化成功！")
                self.state_machine.set_alarm(False)
            elif modbus_success and camera_success:
                print("🎯 核心組件初始化成功，系統可正常運行")
                self.state_machine.set_alarm(False)
            else:
                print("⚠️ 部分組件初始化失敗，但系統嘗試繼續運行")
            
        except Exception as e:
            print(f"❌ 自動初始化過程中發生異常: {e}")
            self.state_machine.set_alarm(True)
    
    def initialize_camera(self, ip_address: str = None) -> bool:
        """初始化相機連接 - 軟體觸發模式"""
        try:
            if ip_address:
                self.camera_ip = ip_address
            
            # 安全關閉現有相機管理器
            if self.camera_manager:
                try:
                    self.camera_manager.shutdown()
                except:
                    pass
                finally:
                    self.camera_manager = None
            
            # 創建相機配置
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
            
            print(f"🔄 初始化相機: {self.camera_ip} (軟體觸發模式)")
            self.camera_manager = OptimizedCameraManager()
            
            # 添加相機
            success = self.camera_manager.add_camera("ccd1_camera", camera_config)
            if not success:
                raise Exception("添加相機失敗")
            
            # 連接相機
            connect_result = self.camera_manager.connect_camera("ccd1_camera")
            if not connect_result:
                raise Exception("相機連接失敗")
            
            # 啟動串流
            stream_result = self.camera_manager.start_streaming(["ccd1_camera"])
            if not stream_result.get("ccd1_camera", False):
                raise Exception("開始串流失敗")
            
            # 等待相機穩定
            time.sleep(1.0)
            
            self.state_machine.set_initialized(True)
            self.state_machine.set_alarm(False)
            self.state_machine.set_ready(True)
            print(f"✅ 相機初始化成功: {self.camera_ip} (軟體觸發模式)")
            return True
                
        except Exception as e:
            self.state_machine.set_alarm(True)
            self.state_machine.set_initialized(False)
            self.state_machine.set_ready(False)
            print(f"❌ 相機初始化失敗: {e}")
            return False
    
    def capture_image(self) -> Tuple[Optional[np.ndarray], float]:
        """軟體觸發拍照 - 記憶體優化版"""
        print(f"📸 開始軟體觸發拍照程序...")
        
        if not self.camera_manager:
            print(f"❌ 相機管理器不存在")
            return None, 0.0
        
        capture_start = time.time()
        
        try:
            # 觸發拍照
            trigger_result = self.camera_manager.trigger_software(["ccd1_camera"])
            
            if not trigger_result.get("ccd1_camera", False):
                print(f"❌ 軟體觸發失敗")
                return None, 0.0
            
            # 獲取圖像
            frame_data = self.camera_manager.capture_new_frame("ccd1_camera", timeout=2000)
            
            if frame_data is None:
                print(f"❌ 觸發後無法獲取圖像")
                return None, 0.0
            
            capture_time = time.time() - capture_start
            
            # 立即複製圖像數據並清理原始frame_data
            image_array = np.copy(frame_data.data)
            
            # 清理frame_data引用
            if hasattr(frame_data, 'data'):
                del frame_data.data
            del frame_data
            
            # 格式轉換
            if len(image_array.shape) == 2:
                display_image = cv2.cvtColor(image_array, cv2.COLOR_GRAY2BGR)
                del image_array
            else:
                display_image = image_array
            
            print(f"✅ 軟體觸發拍照成功，耗時: {capture_time*1000:.2f}ms")
            return display_image, capture_time
            
        except Exception as e:
            capture_time = time.time() - capture_start
            print(f"❌ 軟體觸發拍照異常: {e}")
            return None, capture_time
    
    def capture_and_detect(self) -> YOLODetectionResult:
        """拍照並進行YOLOv11檢測"""
        total_start = time.time()
        
        try:
            # 檢查YOLOv11檢測器
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
            
            # YOLOv11檢測
            result = self.yolo_detector.detect(image)
            result.capture_time = capture_time * 1000
            result.total_time = (time.time() - total_start) * 1000
            
            # 世界座標轉換（如果有標定數據）
            if result.success and result.dr_f_coords:
                print(f"🌍 開始世界座標轉換...")
                self._add_world_coordinates_yolo(result)
            
            # 創建可視化圖像
            if result.success:
                self._create_yolo_visualization(image, result)
            
            self.last_result = result
            self._periodic_memory_cleanup()
            return result
            
        except Exception as e:
            result = YOLODetectionResult()
            result.error_message = f"檢測失敗: {str(e)}"
            result.total_time = (time.time() - total_start) * 1000
            self._periodic_memory_cleanup()
            return result
    
    def _add_world_coordinates_yolo(self, result: YOLODetectionResult):
        """為YOLOv11結果添加世界座標"""
        try:
            if not self.calibration_manager.transformer.is_valid():
                print(f"⚠️ 標定數據無效，跳過世界座標轉換")
                return
            
            print(f"🌍 執行YOLOv11結果世界座標轉換...")
            
            # 僅轉換DR_F座標
            if result.dr_f_coords:
                print(f"   轉換{len(result.dr_f_coords)}個DR_F目標座標")
                try:
                    world_coords = self.calibration_manager.transformer.pixel_to_world(result.dr_f_coords)
                    
                    if world_coords:
                        result.dr_f_world_coords = []
                        for wx, wy in world_coords:
                            world_x = float(wx)
                            world_y = float(wy)
                            result.dr_f_world_coords.append((world_x, world_y))
                        
                        for i, ((px, py), (wx, wy)) in enumerate(zip(result.dr_f_coords, result.dr_f_world_coords)):
                            print(f"   DR_F {i+1}: 像素({px:.1f}, {py:.1f}) → 世界({wx:.2f}, {wy:.2f}) mm")
                        
                        print(f"✅ 世界座標轉換成功，共轉換{len(world_coords)}個DR_F目標")
                    else:
                        print(f"❌ 世界座標轉換失敗：轉換結果為空")
                        result.dr_f_world_coords = []
                        
                except Exception as e:
                    print(f"   ❌ DR_F座標轉換失敗: {e}")
                    result.dr_f_world_coords = []
            else:
                print(f"   無DR_F目標需要轉換")
                result.dr_f_world_coords = []
                    
        except Exception as e:
            print(f"❌ YOLOv11世界座標轉換失敗: {e}")
            import traceback
            print(f"詳細錯誤: {traceback.format_exc()}")
            result.dr_f_world_coords = []
    
    def _create_yolo_visualization(self, image: np.ndarray, result: YOLODetectionResult):
        """創建YOLOv11檢測結果可視化 - 記憶體優化版"""
        try:
            # 使用原地操作減少記憶體複製
            vis_image = np.copy(image)
            
            # 定義顏色
            colors = {
                'DR_F': (255, 0, 0),
                'STACK': (0, 0, 255),
            }
            
            # 繪製DR_F檢測結果
            for i, (x, y) in enumerate(result.dr_f_coords):
                cv2.circle(vis_image, (int(x), int(y)), 15, colors['DR_F'], -1)
                cv2.circle(vis_image, (int(x), int(y)), 20, (255, 255, 255), 3)
                
                label = f"DR_F {i+1}"
                cv2.putText(vis_image, label, (int(x-35), int(y-25)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            
            # 繪製STACK檢測結果
            for i, (x, y) in enumerate(result.stack_coords):
                cv2.circle(vis_image, (int(x), int(y)), 12, colors['STACK'], -1)
                cv2.circle(vis_image, (int(x), int(y)), 17, (255, 255, 255), 2)
                
                label = f"STACK {i+1}"
                cv2.putText(vis_image, label, (int(x-35), int(y-25)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, colors['STACK'], 2)
            
            # 添加檢測統計信息
            stats_text = f"YOLOv11 (Model{result.model_id_used}): F={result.dr_f_count}, S={result.stack_count}"
            cv2.putText(vis_image, stats_text, (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
            
            # 清理輸入圖像引用
            del image
            
            # 限制圖像大小以減少記憶體佔用
            height, width = vis_image.shape[:2]
            if width > 1024:
                scale = 1024 / width
                new_size = (1024, int(height * scale))
                vis_image_resized = cv2.resize(vis_image, new_size)
                del vis_image
                vis_image = vis_image_resized
            
            # 如果之前有圖像，先清理
            if hasattr(self, 'last_image') and self.last_image is not None:
                del self.last_image
            
            self.last_image = vis_image
            
        except Exception as e:
            print(f"❌ 創建可視化失敗: {e}")
            # 發生錯誤時也要清理
            if 'image' in locals():
                del image
    
    def _periodic_memory_cleanup(self):
        """定期記憶體清理"""
        try:
            self._memory_cleanup_counter += 1
            current_time = time.time()
            
            # 常規清理
            if self._memory_cleanup_counter >= self._memory_cleanup_frequency:
                collected = gc.collect()
                print(f"🧹 控制器記憶體清理: 回收{collected}個物件")
                self._memory_cleanup_counter = 0
            
            # 綜合清理
            if current_time - self._last_comprehensive_cleanup > self._comprehensive_cleanup_interval:
                self._comprehensive_memory_cleanup()
                self._last_comprehensive_cleanup = current_time
                
        except Exception as e:
            print(f"❌ 定期記憶體清理失敗: {e}")

    def _comprehensive_memory_cleanup(self):
        """綜合記憶體清理"""
        try:
            print("🧹 執行綜合記憶體清理...")
            
            # 清理圖像緩存
            if hasattr(self, 'last_image') and self.last_image is not None:
                del self.last_image
                self.last_image = None
            
            # 清理檢測結果
            if hasattr(self, 'last_result') and self.last_result is not None:
                del self.last_result
                self.last_result = None
            
            # 強制垃圾回收
            collected = gc.collect()
            
            # 記憶體使用量統計
            try:
                process = psutil.Process()
                memory_mb = process.memory_info().rss / 1024 / 1024
                print(f"🧹 綜合清理完成: 回收{collected}個物件, 當前記憶體: {memory_mb:.1f}MB")
            except:
                print(f"🧹 綜合清理完成: 回收{collected}個物件")
                
        except Exception as e:
            print(f"❌ 綜合記憶體清理失敗: {e}")
    
    def get_status(self) -> Dict[str, Any]:
        """獲取系統狀態"""
        status = {
            'ready': self.state_machine.is_ready(),
            'running': self.state_machine.is_running(),
            'alarm': self.state_machine.is_alarm(),
            'initialized': self.state_machine.is_initialized(),
            'camera_connected': self.camera_manager is not None and "ccd1_camera" in self.camera_manager.cameras,
            'yolo_loaded': self.yolo_detector is not None and self.yolo_detector.is_loaded,
            'confidence_threshold': self.yolo_detector.confidence_threshold if self.yolo_detector else 0.8,
            'modbus_connected': self.modbus_client.connected,
            'calibration_valid': self.calibration_manager.transformer.is_valid(),
            'operation_count': self.operation_count,
            'error_count': self.error_count,
            'last_result': asdict(self.last_result) if self.last_result else None,
            'calibration_status': self.calibration_manager.get_status(),
            'modbus_status': self.modbus_client.get_connection_status()
        }
        
        return status
    
    def disconnect(self):
        """斷開所有連接"""
        # 斷開相機連接
        if self.camera_manager:
            self.camera_manager.shutdown()
            self.camera_manager = None
        
        # 斷開Modbus連接
        self.modbus_client.disconnect()
        
        self.logger.info("所有連接已斷開")


# ==================== 主函數 ====================
def main():
    """主函數 - 純Modbus版本"""
    print("=" * 80)
    print("🚀 CCD1視覺控制系統啟動中 (YOLOv11純Modbus版本)...")
    print("📊 功能特性:")
    print("   • YOLOv11物件檢測 (DR_F/STACK)")
    print("   • 握手式狀態機控制")
    print("   • Modbus TCP Client架構")
    print("   • 自動初始化所有組件")
    print("   • 自我重載功能")
    print("   • 記憶體使用量監控")
    print("   • 100ms高頻輪詢")
    print("   • 世界座標轉換")
    print("=" * 80)
    
    if not CAMERA_MANAGER_AVAILABLE:
        print("❌ 相機管理器不可用，請檢查SDK導入")
        return
    
    controller = None
    
    try:
        # 初始化控制器
        print("🔄 初始化CCD1視覺控制器...")
        controller = CCD1VisionController()
        print("✅ CCD1視覺控制器初始化完成")
        
        print("🎯 系統運行狀態:")
        status = controller.get_status()
        print(f"   • Ready: {status['ready']}")
        print(f"   • Initialized: {status['initialized']}")
        print(f"   • Alarm: {status['alarm']}")
        print(f"   • 相機連接: {status['camera_connected']}")
        print(f"   • YOLO載入: {status['yolo_loaded']}")
        print(f"   • Modbus連接: {status['modbus_connected']}")
        print(f"   • 標定有效: {status['calibration_valid']}")
        
        print("📋 Modbus寄存器映射:")
        print("   • 控制: 200-206 (指令/狀態/模型選擇)")
        print("   • 參數: 210-211 (置信度閾值)")
        print("   • 結果: 240-259 (檢測結果/座標)")
        print("   • 世界座標: 260-279 (座標轉換)")
        print("   • 統計: 280-299 (時間/計數/版本)")
        print("   • 新增: 295(記憶體MB), 296(重載觸發), 297(重載狀態)")
        
        print("🔗 控制方式:")
        print("   1. 透過PLC寫入寄存器200(控制指令)")
        print("   2. 透過寄存器202選擇YOLO模型(0-20)")
        print("   3. 透過寄存器296觸發系統重載")
        print("   4. 監控寄存器295獲取記憶體使用量")
        print("=" * 80)
        print("🎉 系統啟動完成，等待Modbus控制指令...")
        print("💡 系統將持續運行，按Ctrl+C退出")
        
        # 持續運行，等待Modbus控制
        try:
            while True:
                time.sleep(1.0)
                
                # 每10秒顯示一次基本狀態
                if int(time.time()) % 10 == 0:
                    status = controller.get_status()
                    modbus_status = controller.modbus_client.get_connection_status()
                    
                    print(f"📊 系統狀態 - Ready:{status['ready']} "
                          f"Running:{status['running']} "
                          f"Alarm:{status['alarm']} "
                          f"Modbus:{modbus_status['connected']} "
                          f"操作計數:{modbus_status['operation_count']} "
                          f"錯誤計數:{modbus_status['error_count']}")
                    
        except KeyboardInterrupt:
            print("\n🛑 用戶中斷，正在關閉系統...")
            
    except Exception as e:
        print(f"❌ 系統運行錯誤: {e}")
        import traceback
        print(f"詳細錯誤: {traceback.format_exc()}")
    finally:
        try:
            if controller:
                controller.disconnect()
                print("✅ 系統資源已清理")
        except:
            pass
        print("✅ 系統已安全關閉")


if __name__ == "__main__":
    main()