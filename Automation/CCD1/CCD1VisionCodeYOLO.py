# -*- coding: utf-8 -*-
"""
CCD1VisionCode_YOLOv11_Enhanced.py - CCD1視覺控制系統 YOLOv11版本
整合YOLOv11物件檢測功能，支援DR_F/STACK分類檢測
基於Modbus TCP Client架構，實現握手式狀態機控制
適配pymodbus 3.9.2
"""
import gc  # 新增這行
import sys
import os
import time
import threading
import json
import base64
from typing import Optional, Dict, Any, Tuple, List
import numpy as np
import cv2
from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
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
    
    stack_count: int = 0  # 新增
    dr_f_coords: List[Tuple[float, float]] = None
    stack_coords: List[Tuple[float, float]] = None  # 新增
    dr_f_world_coords: List[Tuple[float, float]] = None
    total_detections: int = 0
    confidence_threshold: float = 0.8
    processing_time: float = 0.0
    capture_time: float = 0.0
    total_time: float = 0.0
    timestamp: str = ""
    error_message: Optional[str] = None
    model_id_used: int = 0  # 新增：記錄使用的模型ID

    def __post_init__(self):
        if self.dr_f_coords is None:
            self.dr_f_coords = []
        if self.stack_coords is None:  # 新增
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
    dist_coeffs_file: str = ""  # 新增這行
    working_dir: str = ""
# ==================== 2. 新增YOLOModelManager類 ====================
# 位置：在YOLOv11Detector類之前新增
class YOLOModelManager:
    """YOLO模型管理器 - 支援多模型動態切換"""
    
    def __init__(self, working_dir: str):
        self.working_dir = working_dir
        self.models = {}  # {model_id: YOLO_model}
        self.model_paths = {}  # {model_id: file_path}
        self.current_model_id = 0  # 0=未指定模型
        self.model_switch_count = 0
        
        # 掃描模型檔案
        self.scan_model_files()
    
    def scan_model_files(self):
        """掃描工作目錄中的模型檔案"""
        try:
            print("🔍 掃描YOLO模型檔案...")
            
            # 支援的模型檔案命名模式：
            # model_1.pt, model_2.pt, ... model_20.pt
            # 或 best_1.pt, best_2.pt, ... best_20.pt
            
            for i in range(1, 21):  # 1-20
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
                # 卸載當前模型
                self.current_model_id = 0
                print("🔄 卸載當前模型")
                return True
            
            if model_id < 1 or model_id > 20:
                print(f"❌ 模型ID超出範圍: {model_id} (應為1-20)")
                return False
            
            if model_id not in self.model_paths:
                print(f"❌ 模型{model_id}檔案不存在")
                return False
            
            print(f"🔄 載入模型{model_id}: {self.model_paths[model_id]}")
            
            # 載入YOLO模型
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
    """YOLOv11物件檢測器 - 支援多模型切換和三種分類"""
    
    def __init__(self, working_dir: str, confidence_threshold: float = 0.8):
        self.working_dir = working_dir
        self.confidence_threshold = confidence_threshold
        self.model_manager = YOLOModelManager(working_dir)
        self.class_names = ['DR_F', 'stack']  # 2種類型
        
        # 檢查是否有可用模型，如果有則載入模型1
        if 1 in self.model_manager.model_paths:
            print(f"🎯 自動載入模型1作為預設模型")
            self.model_manager.load_model(1)
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
            
            # 🔥 修正：執行推論並立即處理結果
            results = current_model(image, conf=self.confidence_threshold, verbose=False)
            
            # 🔥 修正：立即提取必要數據
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
                    
                    # 🔥 修正：立即清理boxes和detections
                    del boxes
                    if hasattr(detections, 'boxes') and detections.boxes is not None:
                        del detections.boxes
                else:
                    # 🔥 新增：檢測成功但無目標
                    result.success = True
            else:
                # 🔥 新增：檢測成功但無結果
                result.success = True
            # 🔥 修正：強制清理模型輸出
            if results:
                for res in results:
                    for attr in ['boxes', 'masks', 'keypoints', 'probs']:
                        if hasattr(res, attr):
                            delattr(res, attr)
                results.clear()
                del results
            
            # 🔥 修正：定期強制垃圾回收
            if hasattr(self, '_detection_count'):
                self._detection_count += 1
            else:
                self._detection_count = 1
                
            if self._detection_count % 10 == 0:  # 每10次檢測清理一次
                collected = gc.collect()
                print(f"🧹 YOLO記憶體清理: 回收{collected}個物件")
                
        except Exception as e:
            result.error_message = f"YOLOv11檢測失敗: {e}"
            print(f"❌ YOLOv11檢測異常: {e}")
        
        result.processing_time = (time.time() - start_time) * 1000
        return result



# ==================== 座標轉換器 ====================
class CameraCoordinateTransformer:
    """相機座標轉換器 - 修正版，基於原版CCD1實現"""
    
    def __init__(self):
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvec = None
        self.tvec = None
        self.rotation_matrix = None
        self.is_valid_flag = False
    
    def load_calibration_data(self, intrinsic_file: str, extrinsic_file: str) -> bool:
        """載入標定數據 - 保持原有檔案載入邏輯"""
        try:
            print(f"🔄 CameraCoordinateTransformer載入標定數據...")
            print(f"   內參檔案: {intrinsic_file}")
            print(f"   外參檔案: {extrinsic_file}")
            
            # 載入內參 (相機矩陣) - 保持原有邏輯
            try:
                intrinsic_data = np.load(intrinsic_file, allow_pickle=True)
                
                if hasattr(intrinsic_data, 'shape') and intrinsic_data.shape == (3, 3):
                    # 直接是3x3數組
                    self.camera_matrix = intrinsic_data
                    self.dist_coeffs = np.zeros((1, 5))  # 使用零值畸變係數
                    print(f"   ✅ 載入3x3相機矩陣，使用零值畸變係數")
                elif isinstance(intrinsic_data, dict):
                    # 字典格式
                    self.camera_matrix = intrinsic_data['camera_matrix']
                    self.dist_coeffs = intrinsic_data.get('dist_coeffs', np.zeros((1, 5)))
                    print(f"   ✅ 從字典載入相機矩陣和畸變係數")
                elif hasattr(intrinsic_data, 'item') and callable(intrinsic_data.item):
                    # 字典項目格式
                    dict_data = intrinsic_data.item()
                    if isinstance(dict_data, dict):
                        self.camera_matrix = dict_data['camera_matrix']
                        self.dist_coeffs = dict_data.get('dist_coeffs', np.zeros((1, 5)))
                        print(f"   ✅ 從字典項目載入相機矩陣和畸變係數")
                else:
                    # 直接作為相機矩陣使用
                    self.camera_matrix = intrinsic_data
                    self.dist_coeffs = np.zeros((1, 5))
                    print(f"   ✅ 直接使用數據作為相機矩陣")
                    
            except Exception as e1:
                print(f"   ❌ 內參載入失敗: {e1}")
                return False
            
            # 檢查是否有單獨的畸變係數檔案
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
                    # 直接字典格式
                    self.rvec = extrinsic_data['rvec']
                    self.tvec = extrinsic_data['tvec']
                    print(f"   ✅ 從字典載入外參")
                elif hasattr(extrinsic_data, 'item') and callable(extrinsic_data.item) and extrinsic_data.shape == ():
                    # 0維數組包含字典
                    dict_data = extrinsic_data.item()
                    if isinstance(dict_data, dict):
                        self.rvec = dict_data['rvec']
                        self.tvec = dict_data['tvec']
                        print(f"   ✅ 從字典項目載入外參")
                else:
                    print(f"   ❌ 未知的外參檔案格式")
                    return False
                
                # 🔥 關鍵修正：計算旋轉矩陣
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
        """
        像素座標轉世界座標 - 修正版實現
        基於原版CCD1的正確數學模型
        """
        if not self.is_valid_flag:
            print(f"❌ 標定數據無效，無法進行座標轉換")
            return []
        
        try:
            world_coords = []
            
            for px, py in pixel_coords:
                print(f"🔄 轉換像素座標: ({px:.1f}, {py:.1f})")
                
                # 🔥 步驟1: 去畸變處理
                # 將像素座標轉換為正確的輸入格式
                pixel_point = np.array([[[float(px), float(py)]]], dtype=np.float32)
                
                # 使用cv2.undistortPoints進行去畸變，但不使用P參數
                undistorted_points = cv2.undistortPoints(
                    pixel_point, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                # 獲取歸一化座標
                x_norm, y_norm = undistorted_points[0][0]
                print(f"   去畸變後歸一化座標: ({x_norm:.6f}, {y_norm:.6f})")
                
                # 🔥 步驟2: 構建歸一化齊次座標
                normalized_coords = np.array([x_norm, y_norm, 1.0])
                
                # 🔥 步驟3: 計算深度係數 (Z=0平面投影)
                # 基於原版CCD1的數學模型：s = -t_z / (R3 · normalized_coords)
                # 其中R3是旋轉矩陣的第三行
                R3 = self.rotation_matrix[2, :]  # 旋轉矩陣第三行
                denominator = np.dot(R3, normalized_coords)
                
                if abs(denominator) < 1e-6:
                    print(f"   ⚠️ 分母接近零，跳過此點: denominator={denominator}")
                    continue
                
                # Z=0平面，所以目標Z座標為0
                depth_scale = (0 - self.tvec[2, 0]) / denominator
                print(f"   深度係數: {depth_scale:.6f}")
                
                # 🔥 步驟4: 計算相機座標系中的3D點
                camera_point = depth_scale * normalized_coords
                print(f"   相機座標系點: ({camera_point[0]:.3f}, {camera_point[1]:.3f}, {camera_point[2]:.3f})")
                
                # 🔥 步驟5: 轉換到世界座標系
                # world_point = R^(-1) * (camera_point - tvec)
                # 其中tvec需要reshape為3x1向量
                tvec_3d = self.tvec.reshape(3)
                
                # 計算：camera_point - tvec
                translated_point = camera_point - tvec_3d
                
                # 計算：R^(-1) * translated_point
                # R^(-1) = R^T (對於旋轉矩陣)
                world_point_3d = np.dot(self.rotation_matrix.T, translated_point)
                
                # 取X, Y座標 (忽略Z座標，因為投影到Z=0平面)
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
    """標定檔案管理器 - 修正版"""
    
    def __init__(self, working_dir: str):
        self.working_dir = working_dir
        self.transformer = CameraCoordinateTransformer()
        self.status = CalibrationStatus()
        self.status.working_dir = working_dir
    
    def scan_calibration_files(self) -> Dict[str, Any]:
        """修正版標定檔案掃描"""
        try:
            print(f"🔍 掃描標定檔案目錄: {self.working_dir}")
            
            if not os.path.exists(self.working_dir):
                return {
                    'success': False,
                    'error': f"工作目錄不存在: {self.working_dir}"
                }
            
            all_files = os.listdir(self.working_dir)
            npy_files = [f for f in all_files if f.endswith('.npy')]
            
            print(f"📁 發現 {len(all_files)} 個檔案，其中 {len(npy_files)} 個NPY檔案")
            
            if not npy_files:
                return {
                    'success': False,
                    'error': f"目錄中未發現任何NPY檔案: {self.working_dir}"
                }
            
            # 分類檔案
            camera_matrix_files = []
            dist_coeffs_files = []
            extrinsic_files = []
            unknown_files = []
            
            for file in npy_files:
                file_path = os.path.join(self.working_dir, file)
                file_type = self._classify_file_fixed(file, file_path)
                
                if file_type == 'camera_matrix':
                    camera_matrix_files.append(file)
                elif file_type == 'dist_coeffs':
                    dist_coeffs_files.append(file)
                elif file_type == 'extrinsic':
                    extrinsic_files.append(file)
                else:
                    unknown_files.append(file)
            
            print(f"📋 掃描結果:")
            print(f"   相機矩陣檔案: {len(camera_matrix_files)}個 - {camera_matrix_files}")
            print(f"   畸變係數檔案: {len(dist_coeffs_files)}個 - {dist_coeffs_files}")
            print(f"   外參檔案: {len(extrinsic_files)}個 - {extrinsic_files}")
            print(f"   未知檔案: {len(unknown_files)}個 - {unknown_files}")
            
            # 檢查必要檔案
            if not camera_matrix_files and not dist_coeffs_files:
                return {
                    'success': False,
                    'error': '未發現相機矩陣或畸變係數檔案'
                }
            
            if not extrinsic_files:
                return {
                    'success': False,
                    'error': '未發現外參檔案'
                }
            
            return {
                'success': True,
                'camera_matrix_files': camera_matrix_files,
                'dist_coeffs_files': dist_coeffs_files,
                'extrinsic_files': extrinsic_files,
                'unknown_files': unknown_files,
                'working_dir': self.working_dir,
                'scan_details': f"掃描{len(npy_files)}個NPY檔案，找到有效標定檔案"
            }
            
        except Exception as e:
            print(f"❌ 掃描標定檔案異常: {e}")
            return {
                'success': False,
                'error': f"掃描標定檔案異常: {e}"
            }

    def _classify_file_fixed(self, filename: str, file_path: str) -> str:
        """修正版檔案分類邏輯"""
        try:
            # 載入檔案
            data = np.load(file_path, allow_pickle=True)
            
            # 檔案名稱關鍵字分析
            file_lower = filename.lower()
            is_camera_matrix = any(keyword in file_lower for keyword in 
                                 ['camera_matrix', 'camera', 'intrinsic', 'calib'])
            is_dist_coeffs = any(keyword in file_lower for keyword in 
                               ['dist_coeffs', 'dist', 'distortion', 'coeffs'])
            is_extrinsic = any(keyword in file_lower for keyword in 
                             ['extrinsic', '外参', 'external', 'ext'])
            
            # 1. 檢查字典格式
            if isinstance(data, dict):
                if 'camera_matrix' in data and 'dist_coeffs' in data:
                    return 'camera_matrix'  # 完整內參檔案
                elif 'camera_matrix' in data:
                    return 'camera_matrix'
                elif 'rvec' in data and 'tvec' in data:
                    return 'extrinsic'
            
            # 2. 檢查字典項目格式 (0維數組包含字典)
            elif hasattr(data, 'item') and callable(data.item) and data.shape == ():
                try:
                    dict_data = data.item()
                    if isinstance(dict_data, dict):
                        if 'camera_matrix' in dict_data and 'dist_coeffs' in dict_data:
                            return 'camera_matrix'
                        elif 'camera_matrix' in dict_data:
                            return 'camera_matrix'
                        elif 'rvec' in dict_data and 'tvec' in dict_data:
                            return 'extrinsic'
                except:
                    pass
            
            # 3. 基於數組形狀和檔案名稱判斷
            if hasattr(data, 'shape'):
                # 3x3矩陣 - 相機內參矩陣
                if data.shape == (3, 3):
                    if is_camera_matrix or (not is_dist_coeffs and not is_extrinsic):
                        return 'camera_matrix'
                
                # 畸變係數向量
                elif data.shape in [(5,), (6,), (8,), (1, 5), (1, 8), (5, 1), (8, 1)]:
                    if is_dist_coeffs or (not is_camera_matrix and not is_extrinsic):
                        return 'dist_coeffs'
                
                # 外參矩陣
                elif data.shape in [(4, 4), (3, 4)]:
                    if is_extrinsic or (not is_camera_matrix and not is_dist_coeffs):
                        return 'extrinsic'
                
                # 4. 最終基於檔案名稱判斷
                if is_camera_matrix:
                    return 'camera_matrix'
                elif is_dist_coeffs:
                    return 'dist_coeffs'
                elif is_extrinsic:
                    return 'extrinsic'
            
            return 'unknown'
            
        except Exception as e:
            print(f"分類檔案 {filename} 失敗: {e}")
            return 'unknown'

    def load_calibration_data(self, camera_matrix_file: str = None, 
                            dist_coeffs_file: str = None, 
                            extrinsic_file: str = None) -> Dict[str, Any]:
        """修正版標定數據載入"""
        try:
            # 如果沒有指定檔案，自動選擇
            if not camera_matrix_file or not extrinsic_file:
                scan_result = self.scan_calibration_files()
                if not scan_result['success']:
                    return scan_result
                
                # 自動選擇檔案
                if not camera_matrix_file and scan_result.get('camera_matrix_files'):
                    camera_matrix_file = scan_result['camera_matrix_files'][0]
                
                if not dist_coeffs_file and scan_result.get('dist_coeffs_files'):
                    dist_coeffs_file = scan_result['dist_coeffs_files'][0]
                
                if not extrinsic_file and scan_result.get('extrinsic_files'):
                    extrinsic_file = scan_result['extrinsic_files'][0]
            
            print(f"🔄 載入標定數據:")
            print(f"   相機矩陣: {camera_matrix_file}")
            print(f"   畸變係數: {dist_coeffs_file or '使用零值'}")
            print(f"   外參數據: {extrinsic_file}")
            
            # 載入相機矩陣
            camera_matrix = None
            dist_coeffs = None
            
            if camera_matrix_file:
                camera_path = os.path.join(self.working_dir, camera_matrix_file)
                camera_data = np.load(camera_path, allow_pickle=True)
                
                # 修正: 根據測試結果，camera_matrix_DR.npy是直接的3x3數組
                if isinstance(camera_data, dict):
                    camera_matrix = camera_data.get('camera_matrix')
                    if 'dist_coeffs' in camera_data:
                        dist_coeffs = camera_data['dist_coeffs']
                        print(f"   📊 從字典載入相機矩陣和畸變係數")
                elif hasattr(camera_data, 'shape') and camera_data.shape == (3, 3):
                    # 直接是3x3數組的情況 (您的情況)
                    camera_matrix = camera_data
                    print(f"   📊 載入3x3相機矩陣數組")
                elif hasattr(camera_data, 'item') and callable(camera_data.item) and camera_data.shape == ():
                    # 0維數組包含字典的情況
                    dict_data = camera_data.item()
                    if isinstance(dict_data, dict):
                        camera_matrix = dict_data.get('camera_matrix')
                        if 'dist_coeffs' in dict_data:
                            dist_coeffs = dict_data['dist_coeffs']
                        print(f"   📊 從字典項目載入相機矩陣")
            
            # 載入畸變係數 (如果單獨提供且之前沒載入)
            if dist_coeffs_file and dist_coeffs is None:
                dist_path = os.path.join(self.working_dir, dist_coeffs_file)
                dist_data = np.load(dist_path, allow_pickle=True)
                
                # 修正: 根據測試結果，dist_coeffs_DR.npy是(1,5)數組
                if hasattr(dist_data, 'shape'):
                    dist_coeffs = dist_data
                    print(f"   📊 載入畸變係數: {dist_data.shape}")
            
            # 如果沒有畸變係數，使用零值
            if dist_coeffs is None:
                dist_coeffs = np.zeros((1, 5))
                print(f"   ⚠️ 使用零值畸變係數")
            
            # 載入外參
            rvec = None
            tvec = None
            
            if extrinsic_file:
                ext_path = os.path.join(self.working_dir, extrinsic_file)
                ext_data = np.load(ext_path, allow_pickle=True)
                
                # 修正: 根據測試結果，extrinsic_DR.npy是0維數組包含字典
                if isinstance(ext_data, dict):
                    rvec = ext_data.get('rvec')
                    tvec = ext_data.get('tvec')
                    print(f"   📊 從字典載入外參")
                elif hasattr(ext_data, 'item') and callable(ext_data.item) and ext_data.shape == ():
                    # 0維數組包含字典的情況 (您的情況)
                    dict_data = ext_data.item()
                    if isinstance(dict_data, dict):
                        rvec = dict_data.get('rvec')
                        tvec = dict_data.get('tvec')
                        print(f"   📊 從字典項目載入外參")
            
            # 驗證載入的數據
            if camera_matrix is None or camera_matrix.shape != (3, 3):
                return {
                    'success': False, 
                    'error': f'相機矩陣載入失敗或格式錯誤: {camera_matrix.shape if camera_matrix is not None else "None"}'
                }
            
            if rvec is None or tvec is None:
                return {
                    'success': False, 
                    'error': f'外參數據載入失敗: rvec={rvec is not None}, tvec={tvec is not None}'
                }
            
            # 創建座標轉換器 (使用您原有的CameraCoordinateTransformer)
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
                self.status.dist_coeffs_file = dist_coeffs_file or '內建零值'
                
                print(f"✅ 標定數據載入成功")
                print(f"   相機矩陣: {camera_matrix.shape}, fx={camera_matrix[0,0]:.2f}, fy={camera_matrix[1,1]:.2f}")
                print(f"   畸變係數: {dist_coeffs.shape}, 非零個數: {np.count_nonzero(dist_coeffs)}")
                print(f"   旋轉向量: {rvec.shape}, 範圍: [{rvec.min():.3f}, {rvec.max():.3f}]")
                print(f"   平移向量: {tvec.shape}, 範圍: [{tvec.min():.3f}, {tvec.max():.3f}]")
                
                return {
                    'success': True,
                    'message': '標定數據載入成功',
                    'camera_matrix_file': camera_matrix_file,
                    'dist_coeffs_file': dist_coeffs_file,
                    'extrinsic_file': extrinsic_file
                }
            else:
                return {'success': False, 'error': '座標轉換器創建失敗'}
                
        except Exception as e:
            print(f"❌ 載入標定數據失敗: {e}")
            import traceback
            print(f"詳細錯誤: {traceback.format_exc()}")
            return {'success': False, 'error': f"載入標定數據失敗: {e}"}
    
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
    """增強版Modbus TCP Client服務 - YOLOv11版本"""
    
    def __init__(self, server_ip="127.0.0.1", server_port=502):
        self.server_ip = server_ip
        self.server_port = server_port
        self.client: Optional[ModbusTcpClient] = None
        self.connected = False
        self.vision_controller = None
        
        # 同步控制
        self.sync_thread = None
        self.sync_running = False
        self.sync_interval = 0.1  # 50ms輪詢
        self._sync_counter = 0
        self._cleanup_frequency = 100  # 每100次循環清理一次
        # CCD1 Modbus寄存器映射 (基地址200) - YOLOv11版本
        self.REGISTERS = {
            # 控制寄存器 (200-201)
            'CONTROL_COMMAND': 200,        # 控制指令
            'STATUS_REGISTER': 201,        # 狀態寄存器
            
            # 模型管理寄存器 (202)
            'MODEL_SELECT': 202,           # 模型選擇 (0=未指定, 1-20=模型ID)
            
            # 完成標誌寄存器 (203-206)
            'CAPTURE_COMPLETE': 203,       # 拍照完成標誌
            'DETECT_COMPLETE': 204,        # 檢測完成標誌
            'OPERATION_SUCCESS': 205,      # 操作成功標誌
            'ERROR_CODE': 206,             # 錯誤代碼
            
            # YOLOv11檢測參數寄存器 (210-219)
            'CONFIDENCE_HIGH': 210,        # 置信度閾值高位
            'CONFIDENCE_LOW': 211,         # 置信度閾值低位
            'RESERVED_212': 212,
            'RESERVED_213': 213,
            'RESERVED_214': 214,
            'RESERVED_215': 215,
            
            # YOLOv11檢測結果寄存器 (240-259) - 方案A
            'DR_F_COUNT': 240,           # DR_F檢測數量
            'STACK_COUNT': 242,            # STACK檢測數量 (新增)
            'TOTAL_DETECTIONS': 243,       # 總檢測數量
            'DETECTION_SUCCESS': 244,      # 檢測成功標誌
            
            # DR_F座標寄存器 (245-254) - 最多5個
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

            'STACK_1_X': 257,             # STACK第1個座標X
            'STACK_1_Y': 258,             # STACK第1個座標Y
            'MODEL_ID_USED': 259,         # 本次檢測使用的模型ID
            
            # 世界座標寄存器 (260-279) - 保持不變
            'WORLD_COORD_VALID': 260,
            'DR_F_1_WORLD_X_HIGH': 261,
            'DR_F_1_WORLD_X_LOW': 262,
            'DR_F_1_WORLD_Y_HIGH': 263,
            'DR_F_1_WORLD_Y_LOW': 264,
            'DR_F_2_WORLD_X_HIGH': 265,
            'DR_F_2_WORLD_X_LOW': 266,
            'DR_F_2_WORLD_Y_HIGH': 267,
            'DR_F_2_WORLD_Y_LOW': 268,
            'DR_F_3_WORLD_X_HIGH': 269,
            'DR_F_3_WORLD_X_LOW': 270,
            'DR_F_3_WORLD_Y_HIGH': 271,
            'DR_F_3_WORLD_Y_LOW': 272,
            'DR_F_4_WORLD_X_HIGH': 273,
            'DR_F_4_WORLD_X_LOW': 274,
            'DR_F_4_WORLD_Y_HIGH': 275,
            'DR_F_4_WORLD_Y_LOW': 276,
            'DR_F_5_WORLD_X_HIGH': 277,
            'DR_F_5_WORLD_X_LOW': 278,
            'DR_F_5_WORLD_Y_HIGH': 279,
            'DR_F_5_WORLD_Y_LOW': 280,
            
            # 統計資訊寄存器 (281-299)
            'LAST_CAPTURE_TIME': 281,     # 最後拍照耗時
            'LAST_PROCESS_TIME': 282,     # 最後處理耗時
            'LAST_TOTAL_TIME': 283,       # 最後總耗時
            'OPERATION_COUNT': 284,       # 操作計數器
            'ERROR_COUNT': 285,           # 錯誤計數器
            'CONNECTION_COUNT': 286,      # 連接計數器
            'MODEL_SWITCH_COUNT': 287,    # 模型切換次數 (新增)
            'VERSION_MAJOR': 290,         # 軟體版本主版號
            'VERSION_MINOR': 291,         # 軟體版本次版號
            'UPTIME_HOURS': 292,          # 系統運行時間 (小時)
            'UPTIME_MINUTES': 293,        # 系統運行時間 (分鐘)
        }
        
        # 狀態追蹤
        self.last_control_command = 0
        self.command_processing = False
        self.operation_count = 0
        self.error_count = 0
        self.connection_count = 0
        self.start_time = time.time()
        # 握手狀態控制
        self.command_execution_time = 0     # 指令執行開始時間
        self.min_running_duration = 1.0     # 最小Running狀態持續時間(秒)
        self.completion_hold_time = 2.0     # 完成狀態保持時間(秒)
        self.completion_start_time = 0      # 完成狀態開始時間
        self.current_command = 0            # 當前執行的指令
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
                
                # 寫入初始狀態
                self._write_initial_status()
                
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
        
        if self.client and self.connected:
            try:
                # 寫入斷線狀態
                self.write_register('STATUS_REGISTER', 0)
                self.client.close()
                print("🔌 Modbus TCP Client已斷開連接")
            except:
                pass
        
        self.connected = False
        self.client = None
    def _handle_model_management(self):
        """處理模型管理指令"""
        try:
            if not self.vision_controller or not self.vision_controller.yolo_detector:
                return
            
            # 讀取模型選擇寄存器
            model_select = self.read_register('MODEL_SELECT')
            if model_select is None:
                return
            
            current_model_id = self.vision_controller.yolo_detector.model_manager.current_model_id
            
            # 檢查是否需要切換模型
            if model_select != current_model_id:
                print(f"📋 收到模型切換指令: 模型{current_model_id} → 模型{model_select}")
                
                if 0 <= model_select <= 20:
                    success = self.vision_controller.yolo_detector.switch_model(model_select)
                    
                    if success:
                        print(f"✅ 模型切換成功: 當前模型{model_select}")
                        # 更新模型切換計數
                        switch_count = self.vision_controller.yolo_detector.model_manager.model_switch_count
                        self.write_register('MODEL_SWITCH_COUNT', switch_count)
                    else:
                        print(f"❌ 模型切換失敗: 模型{model_select}")
                        self.write_register('ERROR_CODE', 10)  # 模型切換錯誤
                else:
                    print(f"❌ 無效的模型ID: {model_select}")
                    self.write_register('ERROR_CODE', 11)  # 無效模型ID錯誤
                    
        except Exception as e:
            print(f"❌ 處理模型管理指令失敗: {e}")
    def start_sync(self):
        """啟動同步線程"""
        if self.sync_running:
            return
        
        self.sync_running = True
        self.sync_thread = threading.Thread(target=self._handshake_sync_loop, daemon=True)
        self.sync_thread.start()
        print("✅ Modbus握手同步線程已啟動")
    
    def stop_sync(self):
        """停止同步線程"""
        if self.sync_running:
            self.sync_running = False
            if self.sync_thread and self.sync_thread.is_alive():
                self.sync_thread.join(timeout=2.0)
            print("🛑 Modbus握手同步線程已停止")
    
    def _handshake_sync_loop(self):
        """握手同步循環 - 50ms高頻輪詢"""
        """握手同步循環 - 修正版"""
        print("🔄 增強版握手同步線程開始運行...")
        
        while self.sync_running and self.connected:
            try:
                # 1. 更新狀態寄存器到PLC
                self._update_status_register()
                
                # 2. 讀取控制指令並處理握手邏輯
                self._handle_control_command_enhanced()
                
                # 3. 處理完成狀態邏輯
                self._handle_completion_status()
                
                # 4. 更新統計資訊
                self._update_statistics()
                # 🔥 修正：定期記憶體清理
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
    def _handle_control_command_enhanced(self):
        """處理控制指令握手邏輯 - 增強版"""
        try:
            current_command = self.read_register('CONTROL_COMMAND')
            if current_command is None:
                return
            
            # 檢查指令變化和防重複執行
            if (current_command != self.last_control_command and 
                current_command != 0 and 
                not self.command_processing):
                
                print(f"📋 收到新控制指令: {current_command} (上次: {self.last_control_command})")
                self.last_control_command = current_command
                self.current_command = current_command
                self.command_processing = True
                self.command_execution_time = time.time()
                
                # 清除完成標誌
                self._clear_completion_flags()
                
                # 異步執行指令
                command_thread = threading.Thread(
                    target=self._execute_command_enhanced, 
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
    def _handshake_sync_loop(self):
        """握手同步循環 - 修正版"""
        print("🔄 增強版握手同步線程開始運行...")
        
        while self.sync_running and self.connected:
            try:
                # 1. 更新狀態寄存器到PLC
                self._update_status_register()
                
                # 2. 處理模型管理指令 (新增)
                self._handle_model_management()
                
                # 3. 讀取控制指令並處理握手邏輯
                self._handle_control_command_enhanced()
                
                # 4. 處理完成狀態邏輯
                self._handle_completion_status()
                
                # 5. 更新統計資訊
                self._update_statistics()
                
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
    def _update_status_register(self):
        """更新狀態寄存器到PLC"""
        try:
            if self.vision_controller:
                status_value = self.vision_controller.state_machine.status_register
                self.write_register('STATUS_REGISTER', status_value)
        except:
            pass
    def _execute_command_enhanced(self, command: ControlCommand):
        """執行控制指令 - 增強版"""
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
            
            # 確保Running狀態持續足夠時間讓外部系統看到
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
            
            # 確保Running狀態至少持續指定時間
            running_duration = time.time() - running_start
            if running_duration < self.min_running_duration:
                remaining_time = self.min_running_duration - running_duration
                print(f"⏱️ Running狀態延長 {remaining_time:.2f} 秒以確保可見性")
                time.sleep(remaining_time)
            
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
            # 清除Running狀態，但不立即設置Ready
            # Ready狀態將在指令清零後設置
            self.vision_controller.state_machine.set_running(False)
            self.command_processing = False
            self.completion_start_time = time.time()
            print(f"🔄 狀態變更: Running=0, 等待指令清零後恢復Ready")
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
    def get_completion_status(self) -> Dict[str, Any]:
        """獲取完成狀態"""
        try:
            return {
                'capture_complete': self.read_register('CAPTURE_COMPLETE') or 0,
                'detect_complete': self.read_register('DETECT_COMPLETE') or 0,
                'operation_success': self.read_register('OPERATION_SUCCESS') or 0,
                'error_code': self.read_register('ERROR_CODE') or 0,
                'current_command': self.current_command
            }
        except:
            return {
                'capture_complete': 0,
                'detect_complete': 0,
                'operation_success': 0,
                'error_code': 0,
                'current_command': 0
            }
    def _handle_completion_status(self):
        """處理完成狀態邏輯"""
        try:
            # 如果有完成狀態且超過保持時間，自動清除某些標誌
            if (self.completion_start_time > 0 and 
                time.time() - self.completion_start_time > self.completion_hold_time):
                
                # 檢查控制指令是否已清零
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
            
            # 重置指令追蹤
            self.last_control_command = 0
            self.current_command = 0
            
            # 如果沒有Alarm，恢復Ready狀態
            if not self.vision_controller.state_machine.is_alarm():
                self.vision_controller.state_machine.set_ready(True)
                print("✅ Ready狀態已恢復")
            else:
                print("⚠️ 系統處於Alarm狀態，需要重置才能恢復Ready")
                
        except Exception as e:
            print(f"❌ 處理指令清零失敗: {e}")
    def _handle_capture_detect_command_enhanced(self):
        """處理拍照+檢測指令 - 增強版"""
        try:
            print("🔍 執行拍照+YOLOv11檢測指令")
            result = self.vision_controller.capture_and_detect()
            
            if result and result.success:
                # 更新檢測結果到PLC
                self.update_detection_results(result)
                self.write_register('CAPTURE_COMPLETE', 1)
                self.write_register('DETECT_COMPLETE', 1)
                print(f"✅ YOLOv11檢測成功，DR_F={result.dr_f_count}")
                return True
            else:
                error_msg = result.error_message if result else "檢測結果為空"
                print(f"❌ YOLOv11檢測失敗: {error_msg}")
                self.error_count += 1
                # 清空檢測結果
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
    def _clear_completion_flags(self):
        """清除完成標誌"""
        try:
            self.write_register('CAPTURE_COMPLETE', 0)
            self.write_register('DETECT_COMPLETE', 0)
            self.write_register('OPERATION_SUCCESS', 0)
            self.write_register('ERROR_CODE', 0)
        except Exception as e:
            print(f"❌ 清除完成標誌失敗: {e}")
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
    def _set_completion_state(self, success: bool, command: ControlCommand):
        """設置完成狀態"""
        try:
            self.write_register('OPERATION_SUCCESS', 1 if success else 0)
            
            if not success:
                # 設置錯誤代碼
                error_code = 0
                if command == ControlCommand.CAPTURE:
                    error_code = 10  # 拍照錯誤
                elif command == ControlCommand.CAPTURE_DETECT:
                    error_code = 20  # 檢測錯誤
                elif command == ControlCommand.INITIALIZE:
                    error_code = 30  # 初始化錯誤
                
                self.write_register('ERROR_CODE', error_code)
                self.vision_controller.state_machine.set_alarm(True)
            else:
                self.write_register('ERROR_CODE', 0)
                # 成功時不設置Alarm
                
        except Exception as e:
            print(f"❌ 設置完成狀態失敗: {e}")
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
                
                print(f"📋 收到新控制指令: {current_command} (上次: {self.last_control_command})")
                self.last_control_command = current_command
                self.command_processing = True
                
                # 異步執行指令
                command_thread = threading.Thread(
                    target=self._execute_command, 
                    args=(ControlCommand(current_command),),
                    daemon=True
                )
                command_thread.start()
                
        except Exception as e:
            print(f"❌ 處理控制指令失敗: {e}")
    
    def _execute_command(self, command: ControlCommand):
        """執行控制指令"""
        try:
            print(f"🚀 開始處理控制指令: {command}")
            
            if not self.vision_controller:
                print("❌ 視覺控制器不存在")
                return
            
            # 檢查Ready狀態
            if not self.vision_controller.state_machine.is_ready():
                print("⚠️ 系統未Ready，忽略指令")
                return
            
            # 設置Running狀態，清除Ready
            self.vision_controller.state_machine.set_running(True)
            self.vision_controller.state_machine.set_ready(False)
            
            if command == ControlCommand.CAPTURE:
                self._handle_capture_command()
            elif command == ControlCommand.CAPTURE_DETECT:
                self._handle_capture_detect_command()
            elif command == ControlCommand.INITIALIZE:
                self._handle_initialize_command()
            else:
                print(f"⚠️ 未知控制指令: {command}")
            
        except Exception as e:
            print(f"❌ 執行控制指令異常: {e}")
            self.error_count += 1
            self.vision_controller.state_machine.set_alarm(True)
        finally:
            # 恢復Ready狀態，清除Running
            self.vision_controller.state_machine.set_running(False)
            self.command_processing = False
            self.operation_count += 1
            print(f"✅ 控制指令 {command} 執行完成")
    
    def _handle_capture_command(self):
        """處理拍照指令"""
        try:
            print("📸 執行拍照指令")
            image, capture_time = self.vision_controller.capture_image()
            
            if image is not None:
                self.write_register('LAST_CAPTURE_TIME', int(capture_time * 1000))
                print(f"✅ 拍照成功，耗時: {capture_time*1000:.2f}ms")
            else:
                print("❌ 拍照失敗")
                self.error_count += 1
                
        except Exception as e:
            print(f"❌ 拍照指令執行失敗: {e}")
            self.error_count += 1
    
    def _handle_capture_detect_command(self):
        """處理拍照+檢測指令"""
        try:
            print("🔍 執行拍照+YOLOv11檢測指令")
            result = self.vision_controller.capture_and_detect()
            
            if result and result.success:
                # 更新檢測結果到PLC
                self.update_detection_results(result)
                print(f"✅ YOLOv11檢測成功，DR_F={result.dr_f_count}")
            else:
                error_msg = result.error_message if result else "檢測結果為空"
                print(f"❌ YOLOv11檢測失敗: {error_msg}")
                self.error_count += 1
                # 清空檢測結果
                self._clear_detection_results()
                
        except Exception as e:
            print(f"❌ 檢測指令執行失敗: {e}")
            self.error_count += 1
            self._clear_detection_results()
    
    def _handle_initialize_command(self):
        """處理初始化指令"""
        try:
            print("🔄 執行系統初始化指令")
            
            # 重新初始化相機
            success = self.vision_controller.initialize_camera()
            
            if success:
                self.vision_controller.state_machine.set_initialized(True)
                self.vision_controller.state_machine.set_alarm(False)
                print("✅ 系統初始化成功")
            else:
                self.vision_controller.state_machine.set_initialized(False)
                self.vision_controller.state_machine.set_alarm(True)
                print("❌ 系統初始化失敗")
                self.error_count += 1
                
        except Exception as e:
            print(f"❌ 初始化指令執行失敗: {e}")
            self.error_count += 1
    
    def _clear_detection_results(self):
        """清空檢測結果寄存器"""
        try:
            self.write_register('DR_F_COUNT', 0)
            
            self.write_register('TOTAL_DETECTIONS', 0)
            self.write_register('DETECTION_SUCCESS', 0)
            
            # 清空DR_F座標
            for i in range(1, 6):
                self.write_register(f'DR_F_{i}_X', 0)
                self.write_register(f'DR_F_{i}_Y', 0)
                
        except Exception as e:
            print(f"❌ 清空檢測結果失敗: {e}")
    
    def update_detection_results(self, result: YOLODetectionResult):
        """更新YOLOv11檢測結果到PLC"""
        try:
            # 寫入檢測數量 (包含STACK)
            self.write_register('DR_F_COUNT', result.dr_f_count)
            
            self.write_register('STACK_COUNT', result.stack_count)  # 新增
            self.write_register('TOTAL_DETECTIONS', result.total_detections)
            self.write_register('DETECTION_SUCCESS', 1 if result.success else 0)
            self.write_register('MODEL_ID_USED', result.model_id_used)  # 新增
            
            # 加入檢測數量寫入的打印訊息
            print(f"📊 檢測數量寫入寄存器:")
            print(f"   240(DR_F_COUNT) = {result.dr_f_count}")
            
            print(f"   242(STACK_COUNT) = {result.stack_count}")  # 新增
            print(f"   243(TOTAL_DETECTIONS) = {result.total_detections}")
            print(f"   244(DETECTION_SUCCESS) = {1 if result.success else 0}")
            print(f"   259(MODEL_ID_USED) = {result.model_id_used}")  # 新增

            # 寫入DR_F座標 (最多5個) - 地址更新為245-254
            for i in range(5):
                if i < len(result.dr_f_coords):
                    x, y = result.dr_f_coords[i]
                    self.write_register(f'DR_F_{i+1}_X', int(float(x)))
                    self.write_register(f'DR_F_{i+1}_Y', int(float(y)))
                    print(f"   {245+i*2}(DR_F_{i+1}_X) = {int(float(x))}")
                    print(f"   {246+i*2}(DR_F_{i+1}_Y) = {int(float(y))}")
                else:
                    self.write_register(f'DR_F_{i+1}_X', 0)
                    self.write_register(f'CDR_F_{i+1}_Y', 0)
            
            if result.stack_coords:
                x, y = result.stack_coords[0]
                self.write_register('STACK_1_X', int(float(x)))
                self.write_register('STACK_1_Y', int(float(y)))
                print(f"   257(STACK_1_X) = {int(float(x))}")
                print(f"   258(STACK_1_Y) = {int(float(y))}")
            else:
                self.write_register('STACK_1_X', 0)
                self.write_register('STACK_1_Y', 0)
            
            # 寫入時間統計 - 確保為整數類型
            self.write_register('LAST_CAPTURE_TIME', int(float(result.capture_time)))
            self.write_register('LAST_PROCESS_TIME', int(float(result.processing_time)))
            self.write_register('LAST_TOTAL_TIME', int(float(result.total_time)))
            
            # 世界座標轉換（如果可用）
            if (result.success and result.dr_f_coords and
                self.vision_controller and 
                self.vision_controller.calibration_manager.transformer.is_valid()):
                
                world_coords = result.dr_f_world_coords
                
                if world_coords:
                    self.write_register('WORLD_COORD_VALID', 1)
                    print(f"🌍 世界座標轉換成功，共{len(world_coords)}個DR_F目標")
                    
                    # 寫入前5個世界座標 (×100存儲，參考原CCD1方式)
                    # 修正世界座標寫入部分 - 在 update_detection_results 方法中

                    # 寫入前5個世界座標 (×100存儲，參考原CCD1方式)
                    for i in range(min(5, len(world_coords))):
                        world_x, world_y = world_coords[i]
                        
                        # 轉換為32位整數並分高低位存儲（×100精度）
                        world_x_int = int(float(world_x) * 100)
                        world_y_int = int(float(world_y) * 100)
                        
                        print(f"   處理 DR_F {i+1}: 原始值=({world_x:.2f}, {world_y:.2f})")
                        print(f"   ×100後整數值: x_int={world_x_int}, y_int={world_y_int}")
                        
                        # 🎯 修正：處理負數（使用補碼表示法）
                        if world_x_int < 0:
                            world_x_uint32 = (2**32) + world_x_int  # 轉換為無符號32位
                        else:
                            world_x_uint32 = world_x_int
                            
                        if world_y_int < 0:
                            world_y_uint32 = (2**32) + world_y_int  # 轉換為無符號32位
                        else:
                            world_y_uint32 = world_y_int
                        
                        print(f"   32位無符號值: x_uint32={world_x_uint32}, y_uint32={world_y_uint32}")
                        
                        # 🎯 修正：正確分割32位為高16位和低16位
                        world_x_high = (world_x_uint32 >> 16) & 0xFFFF  # 高16位
                        world_x_low = world_x_uint32 & 0xFFFF           # 低16位
                        world_y_high = (world_y_uint32 >> 16) & 0xFFFF  # 高16位
                        world_y_low = world_y_uint32 & 0xFFFF           # 低16位
                        
                        print(f"   分割結果: x_high={world_x_high}, x_low={world_x_low}")
                        print(f"   分割結果: y_high={world_y_high}, y_low={world_y_low}")
                        
                        # 🎯 驗證：重新組合檢查
                        reconstructed_x = (world_x_high << 16) | world_x_low
                        reconstructed_y = (world_y_high << 16) | world_y_low
                        
                        # 如果是負數，轉換回有符號整數
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
                        
                        # 確保分割值在16位範圍內
                        if world_x_high > 65535 or world_x_low > 65535 or world_y_high > 65535 or world_y_low > 65535:
                            print(f"❌ 警告：分割值超出16位範圍！")
                            print(f"   x_high={world_x_high}, x_low={world_x_low}")
                            print(f"   y_high={world_y_high}, y_low={world_y_low}")
                        
                        # 寫入世界座標寄存器
                        self.write_register(f'DR_F_{i+1}_WORLD_X_HIGH', world_x_high)
                        self.write_register(f'DR_F_{i+1}_WORLD_X_LOW', world_x_low)
                        self.write_register(f'DR_F_{i+1}_WORLD_Y_HIGH', world_y_high)
                        self.write_register(f'DR_F_{i+1}_WORLD_Y_LOW', world_y_low)
                        
                        # 加入詳細的寄存器寫入打印訊息
                        print(f"   DR_F {i+1} 世界座標寫入:")
                        print(f"     {261+i*4}(WORLD_X_HIGH) = {world_x_high}")
                        print(f"     {262+i*4}(WORLD_X_LOW) = {world_x_low}")
                        print(f"     {263+i*4}(WORLD_Y_HIGH) = {world_y_high}")
                        print(f"     {264+i*4}(WORLD_Y_LOW) = {world_y_low}")
                        print(f"     實際值: ({world_x:.2f}, {world_y:.2f}) mm")
                        print(f"     驗證值: ({reconstructed_x_mm:.2f}, {reconstructed_y_mm:.2f}) mm")
                        print()  # 空行分隔
                    
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
    
    def _update_world_coord_status(self):
        """更新世界座標有效性狀態"""
        try:
            if (self.vision_controller and 
                self.vision_controller.calibration_manager.transformer.is_valid()):
                # 如果標定數據有效但還沒設置標誌，設置為有效
                current_status = self.read_register('WORLD_COORD_VALID')
                if current_status is None:
                    self.write_register('WORLD_COORD_VALID', 0)
            else:
                self.write_register('WORLD_COORD_VALID', 0)
        except:
            pass
    
    def _write_initial_status(self):
        """寫入初始狀態到PLC"""
        try:
            # 版本資訊
            self.write_register('VERSION_MAJOR', 5)  # YOLOv11版本
            self.write_register('VERSION_MINOR', 0)
            
            # 初始化置信度閾值為0.8 (×10000存儲)
            confidence_int = int(0.8 * 10000)  # 8000
            self.write_register('CONFIDENCE_HIGH', (confidence_int >> 16) & 0xFFFF)
            self.write_register('CONFIDENCE_LOW', confidence_int & 0xFFFF)
            
            # 計數器
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            self.write_register('CONNECTION_COUNT', self.connection_count)
            
            print("📊 初始狀態已寫入PLC")
            
        except Exception as e:
            print(f"❌ 寫入初始狀態失敗: {e}")
    
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
            'operation_count': self.operation_count,
            'error_count': self.error_count,
            'connection_count': self.connection_count,
            'uptime_seconds': int(time.time() - self.start_time)
        }


# ==================== 主控制器 ====================
class CCD1VisionController:
    """CCD1視覺檢測主控制器 - YOLOv11版本"""
    
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
        
        # 檢查並初始化YOLOv11
        self._check_yolo_availability()
        
        # 設置日誌
        self.logger = logging.getLogger("CCD1Vision")
        self.logger.setLevel(logging.INFO)
        
        # 檢查和初始化各組件
        component_status = self._check_and_initialize_components()
        self._memory_cleanup_counter = 0
        self._memory_cleanup_frequency = 50  # 每50次操作清理一次
        self._last_comprehensive_cleanup = time.time()
        self._comprehensive_cleanup_interval = 300  # 5分鐘綜合清理一次
        # 根據組件狀態執行相應的後續操作
        if component_status['modbus_connected'] and component_status['calibration_loaded']:
            # 嘗試自動初始化相機
            print("📷 嘗試自動初始化相機...")
            try:
                camera_success = self.initialize_camera()
                if camera_success:
                    print("✅ 相機自動初始化成功")
                    self.state_machine.set_initialized(True)
                    self.state_machine.set_ready(True)
                    print("🎯 系統完全就緒，進入Ready狀態")
                else:
                    print("⚠️ 相機自動初始化失敗")
                    if not component_status['modbus_connected']:
                        print("💡 請透過Web介面手動連接Modbus服務器")
            except Exception as e:
                print(f"⚠️ 相機自動初始化異常: {e}")
        else:
            print("⚠️ 核心組件未完全就緒，跳過相機自動初始化")
    def _check_yolo_availability(self):
        """檢查YOLOv11模型可用性 - 支援多模型"""
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
    def _check_and_initialize_components(self):
        """檢查和初始化各組件狀態"""
        print("🔍 檢查系統組件狀態...")
        
        # 1. 檢查Modbus客戶端連接狀態
        print("📡 檢查Modbus客戶端連接狀態...")
        modbus_connected = False
        try:
            modbus_connected = self.modbus_client.connect()
            if modbus_connected:
                self.modbus_client.start_sync()
                print("✅ Modbus客戶端已連接並啟動同步")
                
                # 讀取並同步置信度閾值
                confidence = self.modbus_client.read_confidence_threshold()
                if self.yolo_detector:
                    self.yolo_detector.update_confidence_threshold(confidence)
                    print(f"🎯 置信度閾值已同步: {confidence}")
            else:
                print("❌ Modbus客戶端連接失敗")
        except Exception as e:
            print(f"❌ Modbus客戶端連接異常: {e}")
        
        # 2. 檢查內外參檔案載入狀態
        print("📐 檢查內外參檔案載入狀態...")
        calibration_loaded = False
        try:
            # 掃描標定檔案
            scan_result = self.calibration_manager.scan_calibration_files()
            if scan_result['success']:
                print(f"✅ 標定檔案掃描成功: {scan_result.get('scan_details', '')}")
                
                # 載入標定數據
                load_result = self.calibration_manager.load_calibration_data()
                if load_result['success']:
                    calibration_loaded = True
                    print("✅ 內外參檔案已成功載入")
                    print(f"   內參檔案: {load_result.get('intrinsic_file', 'N/A')}")
                    print(f"   外參檔案: {load_result.get('extrinsic_file', 'N/A')}")
                    print(f"   座標轉換器: 已啟用")
                else:
                    print(f"❌ 標定數據載入失敗: {load_result['error']}")
            else:
                print(f"❌ 標定檔案掃描失敗: {scan_result['error']}")
                
        except Exception as e:
            print(f"❌ 標定檔案檢查異常: {e}")
        
        # 3. 設置系統初始狀態
        if modbus_connected and calibration_loaded:
            print("🚀 所有核心組件檢查通過")
            self.state_machine.set_alarm(False)
            # 注意：相機初始化將在後續手動或自動執行
        else:
            print("⚠️ 部分核心組件檢查失敗")
            missing_components = []
            if not modbus_connected:
                missing_components.append("Modbus連接")
            if not calibration_loaded:
                missing_components.append("標定檔案")
            
            print(f"   缺失組件: {', '.join(missing_components)}")
            
            # 根據需求決定是否設置Alarm
            if not calibration_loaded:  # 標定檔案是強制要求
                print("❌ 標定檔案是強制要求，設置系統Alarm狀態")
                self.state_machine.set_alarm(True)
            elif not modbus_connected:  # Modbus連接失敗但可以繼續
                print("⚠️ Modbus連接失敗，但系統可以繼續運行")
        
        print("📊 組件檢查完成")
        return {
            'modbus_connected': modbus_connected,
            'calibration_loaded': calibration_loaded
        }
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
                import psutil
                process = psutil.Process()
                memory_mb = process.memory_info().rss / 1024 / 1024
                print(f"🧹 綜合清理完成: 回收{collected}個物件, 當前記憶體: {memory_mb:.1f}MB")
            except:
                print(f"🧹 綜合清理完成: 回收{collected}個物件")
                
        except Exception as e:
            print(f"❌ 綜合記憶體清理失敗: {e}")
    def _add_world_coordinates_yolo(self, result: YOLODetectionResult):
        """為YOLOv11結果添加世界座標 - 修正版"""
        try:
            if not self.calibration_manager.transformer.is_valid():
                print(f"⚠️ 標定數據無效，跳過世界座標轉換")
                return
            
            print(f"🌍 執行YOLOv11結果世界座標轉換...")
            
            # 僅轉換DR_F座標（作為圓心處理）
            if result.dr_f_coords:
                print(f"   轉換{len(result.dr_f_coords)}個DR_F目標座標")
                try:
                    # 🔥 使用修正版的像素到世界座標轉換
                    world_coords = self.calibration_manager.transformer.pixel_to_world(result.dr_f_coords)
                    
                    if world_coords:
                        # 將世界座標轉換為Python原生類型並存儲到結果中
                        result.dr_f_world_coords = []
                        for wx, wy in world_coords:
                            # 確保轉換為Python原生float類型
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
    def connect_modbus(self) -> Dict[str, Any]:
        """連接Modbus TCP服務器"""
        try:
            if self.modbus_client.connect():
                # 🔥 修改：確保同步線程啟動
                if not self.modbus_client.sync_running:
                    self.modbus_client.start_sync()
                    print("🔄 Modbus同步線程已啟動")
                else:
                    print("🔄 Modbus同步線程已在運行中")
                
                # 讀取並同步置信度閾值
                confidence = self.modbus_client.read_confidence_threshold()
                if self.yolo_detector:
                    self.yolo_detector.update_confidence_threshold(confidence)
                    print(f"🎯 置信度閾值已同步: {confidence}")
                
                return {
                    'success': True,
                    'message': f'Modbus TCP連接成功: {self.server_ip}:{self.server_port}',
                    'connection_status': self.modbus_client.get_connection_status(),
                    'sync_thread_running': self.modbus_client.sync_running  # 🔥 新增同步狀態
                }
                
        except Exception as e:
            return {
                'success': False,
                'message': f'Modbus連接異常: {str(e)}'
            }
    
    def disconnect_modbus(self) -> Dict[str, Any]:
        """斷開Modbus連接"""
        try:
            self.modbus_client.disconnect()
            
            return {
                'success': True,
                'message': 'Modbus連接已斷開'
            }
            
        except Exception as e:
            return {
                'success': False,
                'message': f'斷開Modbus連接失敗: {str(e)}'
            }
    
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
            
            # 創建相機配置 - 修正：使用軟體觸發模式
            camera_config = CameraConfig(
                name="ccd1_camera",
                ip=self.camera_ip,
                exposure_time=20000.0,  # 增加曝光時間配合軟體觸發
                gain=200.0,
                frame_rate=5.0,
                pixel_format=PixelFormat.BAYER_GR8,
                width=2592,
                height=1944,
                trigger_mode=CameraMode.SOFTWARE_TRIGGER,  # 修正：軟體觸發模式
                auto_reconnect=True,
                # 頻寬控制參數
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
            
            # 修正：軟體觸發模式也需要啟動串流才能接收觸發後的圖像
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
            
            # 🔥 修正：立即複製圖像數據並清理原始frame_data
            image_array = np.copy(frame_data.data)
            
            # 🔥 修正：清理frame_data引用
            if hasattr(frame_data, 'data'):
                del frame_data.data
            del frame_data
            
            # 格式轉換
            if len(image_array.shape) == 2:
                display_image = cv2.cvtColor(image_array, cv2.COLOR_GRAY2BGR)
                del image_array  # 🔥 修正：清理原始圖像
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
    
    def _create_yolo_visualization(self, image: np.ndarray, result: YOLODetectionResult):
        """創建YOLOv11檢測結果可視化 - 記憶體優化版"""
        try:
            # 🔥 修正：使用原地操作減少記憶體複製
            vis_image = np.copy(image)  # 明確使用np.copy
            
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
            
            # 🔥 修正：清理輸入圖像引用
            del image
            
            # 🔥 修正：限制圖像大小以減少記憶體佔用
            height, width = vis_image.shape[:2]
            if width > 1024:  # 限制最大寬度
                scale = 1024 / width
                new_size = (1024, int(height * scale))
                vis_image_resized = cv2.resize(vis_image, new_size)
                del vis_image  # 清理原始大圖
                vis_image = vis_image_resized
            
            # 🔥 修正：如果之前有圖像，先清理
            if hasattr(self, 'last_image') and self.last_image is not None:
                del self.last_image
            
            self.last_image = vis_image
            
        except Exception as e:
            print(f"❌ 創建可視化失敗: {e}")
            # 🔥 修正：發生錯誤時也要清理
            if 'image' in locals():
                del image
    
    def get_image_base64(self) -> Optional[str]:
        """獲取當前圖像的base64編碼"""
        if self.last_image is None:
            return None
        
        try:
            height, width = self.last_image.shape[:2]
            if width > 800:
                scale = 800 / width
                new_width = 800
                new_height = int(height * scale)
                display_image = cv2.resize(self.last_image, (new_width, new_height))
            else:
                display_image = self.last_image
            
            _, buffer = cv2.imencode('.jpg', display_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            image_base64 = base64.b64encode(buffer).decode('utf-8')
            return f"data:image/jpeg;base64,{image_base64}"
            
        except Exception as e:
            self.logger.error(f"圖像編碼失敗: {e}")
            return None
    
    def update_confidence_threshold(self, threshold: float):
        """更新置信度閾值"""
        if self.yolo_detector:
            self.yolo_detector.update_confidence_threshold(threshold)
            
            # 同步到Modbus寄存器
            if self.modbus_client.connected:
                confidence_int = int(threshold * 10000)
                self.modbus_client.write_register('CONFIDENCE_HIGH', (confidence_int >> 16) & 0xFFFF)
                self.modbus_client.write_register('CONFIDENCE_LOW', confidence_int & 0xFFFF)
    
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
            'calibration_status': self.calibration_manager.get_status()
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


# ==================== Flask Web應用 ====================
app = Flask(__name__)
app.config['SECRET_KEY'] = 'ccd1_yolo_vision_v5'
socketio = SocketIO(app, cors_allowed_origins="*")

# 全局控制器實例
controller = None

def initialize_controller():
    """初始化控制器並自動連接所有組件"""
    global controller
    try:
        print("🚀 正在初始化CCD1視覺控制器 (YOLOv11版本)...")
        controller = CCD1VisionController()
        print("✅ CCD1視覺控制器初始化成功")
        print("🎯 所有組件已自動初始化完成")
        return True
    except Exception as e:
        print(f"❌ CCD1視覺控制器初始化失敗: {e}")
        import traceback
        print(f"詳細錯誤: {traceback.format_exc()}")
        return False


@app.route('/')
def index():
    """主頁面"""
    return render_template('ccd1_yolo_enhanced.html')

@app.route('/api/switch_model', methods=['POST'])
def switch_model():
    """切換YOLO模型"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        data = request.get_json() if request.get_json() else {}
        model_id = int(data.get('model_id', 0))
        
        if 0 <= model_id <= 20:
            success = controller.yolo_detector.switch_model(model_id)
            
            if success:
                current_model = controller.yolo_detector.model_manager.current_model_id
                available_count = controller.yolo_detector.model_manager.get_available_model_count()
                
                # 同步到Modbus寄存器
                if controller.modbus_client and controller.modbus_client.connected:
                    controller.modbus_client.write_register('MODEL_SELECT', current_model)
                
                return jsonify({
                    'success': True,
                    'current_model_id': current_model,
                    'available_model_count': available_count,
                    'message': f'模型切換成功: 模型{model_id}'
                })
            else:
                return jsonify({
                    'success': False,
                    'error': f'模型{model_id}切換失敗'
                })
        else:
            return jsonify({
                'success': False,
                'error': f'無效的模型ID: {model_id} (應為0-20)'
            })
            
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})
@app.route('/api/model_status', methods=['GET'])
def get_model_status():
    """獲取模型狀態"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        manager = controller.yolo_detector.model_manager
        
        return jsonify({
            'success': True,
            'current_model_id': manager.current_model_id,
            'available_model_count': manager.get_available_model_count(),
            'available_models': list(manager.model_paths.keys()),
            'model_switch_count': manager.model_switch_count,
            'is_model_loaded': manager.is_model_loaded()
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})
@app.route('/api/status')
def get_status():
    """獲取系統狀態"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        status = controller.get_status()
        return jsonify({'success': True, 'status': status})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/connect_modbus', methods=['POST'])
def connect_modbus():
    """連接Modbus服務器"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        result = controller.connect_modbus()
        socketio.emit('status_update', controller.get_status())
        return jsonify(result)
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/disconnect_modbus', methods=['POST'])
def disconnect_modbus():
    """斷開Modbus連接"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        result = controller.disconnect_modbus()
        socketio.emit('status_update', controller.get_status())
        return jsonify(result)
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/initialize_camera', methods=['POST'])
def initialize_camera():
    """初始化相機"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        data = request.get_json() if request.get_json() else {}
        ip = data.get('ip', controller.camera_ip)
        
        success = controller.initialize_camera(ip)
        
        result = {
            'success': success,
            'message': f'相機初始化{"成功" if success else "失敗"}: {ip}',
            'camera_ip': ip
        }
        
        socketio.emit('status_update', controller.get_status())
        return jsonify(result)
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})

@app.route('/api/modbus/completion', methods=['GET'])
def get_completion_status():
    """獲取完成狀態"""
    if not controller or not controller.modbus_client.connected:
        return jsonify({'success': False, 'error': 'Modbus未連接'})
    
    try:
        completion_status = controller.modbus_client.get_completion_status()
        
        return jsonify({
            'success': True,
            'completion_status': completion_status,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})




@app.route('/api/capture_and_detect', methods=['POST'])
def capture_and_detect():
    """拍照並檢測 - 修正JSON序列化 + Modbus寄存器寫入"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        result = controller.capture_and_detect()
        
        # 🎯 Web API調用後也寫入Modbus寄存器
        if controller.modbus_client and controller.modbus_client.connected:
            print(f"📤 Flask API - 準備寫入檢測結果到Modbus寄存器...")
            try:
                controller.modbus_client.update_detection_results(result)
                print(f"✅ Flask API - Modbus寄存器寫入完成")
            except Exception as modbus_error:
                print(f"❌ Flask API - Modbus寄存器寫入失敗: {modbus_error}")
        
        # 確保所有數據類型都能序列化為JSON
        def convert_to_serializable(data):
            """轉換數據為JSON可序列化格式"""
            if isinstance(data, (list, tuple)):
                return [convert_to_serializable(item) for item in data]
            elif isinstance(data, dict):
                return {key: convert_to_serializable(value) for key, value in data.items()}
            elif hasattr(data, 'item'):  # NumPy scalar
                return data.item()
            elif isinstance(data, (np.float32, np.float64)):
                return float(data)
            elif isinstance(data, (np.int32, np.int64)):
                return int(data)
            else:
                return data
        
        response = {
            'success': result.success,
            'dr_f_count': int(result.dr_f_count),
            'stack_count': int(result.stack_count),  # 🔥 確保包含STACK數量
            'total_detections': int(result.total_detections),
            'dr_f_coords': convert_to_serializable(result.dr_f_coords),
            'stack_coords': convert_to_serializable(result.stack_coords),  # 🔥 確保包含STACK座標
            'dr_f_world_coords': convert_to_serializable(getattr(result, 'dr_f_world_coords', [])),
            'confidence_threshold': float(result.confidence_threshold),
            'capture_time': float(result.capture_time),
            'processing_time': float(result.processing_time),
            'total_time': float(result.total_time),
            'timestamp': str(result.timestamp),
            'image': controller.get_image_base64(),
            'error_message': result.error_message,
            'world_coord_valid': len(getattr(result, 'dr_f_world_coords', [])) > 0,
            'model_id_used': int(result.model_id_used),  # 🔥 確保模型ID包含在響應中
            'modbus_write_success': controller.modbus_client.connected if controller.modbus_client else False
        }
        
        # 🔥 調試：打印響應數據
        print(f"📤 API響應數據檢查:")
        print(f"   DR_F: {response['dr_f_count']}")
        print(f"   STACK: {response['stack_count']}")
        print(f"   模型ID: {response['model_id_used']}")
        
        socketio.emit('detection_result', response)
        return jsonify(response)
        
    except Exception as e:
        print(f"❌ Flask API capture_and_detect 異常: {e}")
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/update_confidence', methods=['POST'])
def update_confidence():
    """更新置信度閾值"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        data = request.json
        threshold = float(data.get('threshold', 0.8))
        
        controller.update_confidence_threshold(threshold)
        
        return jsonify({
            'success': True,
            'confidence_threshold': controller.yolo_detector.confidence_threshold if controller.yolo_detector else threshold,
            'message': f'置信度閾值更新為: {threshold}'
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/scan_calibration', methods=['GET'])
def scan_calibration():
    """掃描標定檔案"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        result = controller.calibration_manager.scan_calibration_files()
        return jsonify(result)
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/load_calibration', methods=['POST'])
def load_calibration():
    """載入標定數據"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        data = request.get_json() if request.get_json() else {}
        intrinsic_file = data.get('intrinsic_file')
        extrinsic_file = data.get('extrinsic_file')
        
        result = controller.calibration_manager.load_calibration_data(intrinsic_file, extrinsic_file)
        socketio.emit('status_update', controller.get_status())
        return jsonify(result)
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/modbus/registers', methods=['GET'])
def get_modbus_registers():
    """獲取Modbus寄存器狀態 - 修正版包含STACK"""
    if not controller or not controller.modbus_client.connected:
        return jsonify({'success': False, 'error': 'Modbus未連接'})
    
    try:
        registers = {}
        modbus_client = controller.modbus_client
        
        # 控制寄存器
        registers['200_控制指令'] = modbus_client.read_register('CONTROL_COMMAND')
        registers['201_狀態寄存器'] = modbus_client.read_register('STATUS_REGISTER')
        registers['202_模型選擇'] = modbus_client.read_register('MODEL_SELECT')  # 新增模型選擇
        
        # 完成標誌寄存器
        registers['203_拍照完成'] = modbus_client.read_register('CAPTURE_COMPLETE')
        registers['204_檢測完成'] = modbus_client.read_register('DETECT_COMPLETE')
        registers['205_操作成功'] = modbus_client.read_register('OPERATION_SUCCESS')
        registers['206_錯誤代碼'] = modbus_client.read_register('ERROR_CODE')
        
        # 置信度閾值
        conf_high = modbus_client.read_register('CONFIDENCE_HIGH') or 0
        conf_low = modbus_client.read_register('CONFIDENCE_LOW') or 8000
        confidence = (conf_high << 16) + conf_low
        registers['210-211_置信度閾值'] = f"{confidence/10000.0:.2f}"

        # 🔥 修正檢測結果 - 正確的地址映射
        registers['240_DR_F數量'] = modbus_client.read_register('DR_F_COUNT')
        registers['242_STACK數量'] = modbus_client.read_register('STACK_COUNT')  # 🔥 修正：這是STACK不是總檢測
        registers['243_總檢測數量'] = modbus_client.read_register('TOTAL_DETECTIONS')  # 🔥 修正：這才是總檢測數量
        registers['244_檢測成功標誌'] = modbus_client.read_register('DETECTION_SUCCESS')
        
        # 🔥 新增：DR_F座標寄存器 (245-254)
        for i in range(1, 6):
            dr_f_x = modbus_client.read_register(f'DR_F_{i}_X')
            dr_f_y = modbus_client.read_register(f'DR_F_{i}_Y')
            if dr_f_x is not None and dr_f_y is not None and (dr_f_x != 0 or dr_f_y != 0):
                registers[f'{245+(i-1)*2}_DR_F_{i}_X'] = dr_f_x
                registers[f'{246+(i-1)*2}_DR_F_{i}_Y'] = dr_f_y
        
        stack_x = modbus_client.read_register('STACK_1_X')
        stack_y = modbus_client.read_register('STACK_1_Y')
        if stack_x is not None and stack_y is not None and (stack_x != 0 or stack_y != 0):
            registers['257_STACK_1_X'] = stack_x
            registers['258_STACK_1_Y'] = stack_y
        
        used_model_id = modbus_client.read_register('MODEL_ID_USED')
        if used_model_id is not None:
            registers['259_檢測使用模型ID'] = used_model_id
        
        # 世界座標寄存器 (260-279)
        world_coord_valid = modbus_client.read_register('WORLD_COORD_VALID')
        registers['260_世界座標有效'] = world_coord_valid
        
        if world_coord_valid:
            for i in range(1, 6):
                x_high = modbus_client.read_register(f'DR_F_{i}_WORLD_X_HIGH') or 0
                x_low = modbus_client.read_register(f'DR_F_{i}_WORLD_X_LOW') or 0
                y_high = modbus_client.read_register(f'DR_F_{i}_WORLD_Y_HIGH') or 0
                y_low = modbus_client.read_register(f'DR_F_{i}_WORLD_Y_LOW') or 0
                
                if x_high != 0 or x_low != 0 or y_high != 0 or y_low != 0:
                    # 組合32位座標值並轉換為實際座標
                    world_x_int = (x_high << 16) + x_low
                    world_y_int = (y_high << 16) + y_low
                    
                    # 處理負數（補碼轉換）
                    if world_x_int >= 2**31:
                        world_x_int -= 2**32
                    if world_y_int >= 2**31:
                        world_y_int -= 2**32
                    
                    world_x_mm = world_x_int / 100.0
                    world_y_mm = world_y_int / 100.0
                    
                    registers[f'{261+(i-1)*4}_DR_F_{i}_世界X'] = f"{world_x_mm:.2f}mm"
                    registers[f'{263+(i-1)*4}_DR_F_{i}_世界Y'] = f"{world_y_mm:.2f}mm"
        
        # 統計資訊
        registers['281_拍照耗時ms'] = modbus_client.read_register('LAST_CAPTURE_TIME')
        registers['282_處理耗時ms'] = modbus_client.read_register('LAST_PROCESS_TIME')
        registers['283_總耗時ms'] = modbus_client.read_register('LAST_TOTAL_TIME')
        registers['284_操作計數'] = modbus_client.read_register('OPERATION_COUNT')
        registers['285_錯誤計數'] = modbus_client.read_register('ERROR_COUNT')
        registers['287_模型切換次數'] = modbus_client.read_register('MODEL_SWITCH_COUNT')  # 新增
        
        # 版本資訊
        version_major = modbus_client.read_register('VERSION_MAJOR')
        version_minor = modbus_client.read_register('VERSION_MINOR')
        if version_major is not None and version_minor is not None:
            registers['290-291_軟體版本'] = f"v{version_major}.{version_minor}"
        
        return jsonify({
            'success': True,
            'registers': registers,
            'timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        })
        
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    """斷開所有連接"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        controller.disconnect()
        socketio.emit('status_update', controller.get_status())
        return jsonify({'success': True, 'message': '所有連接已斷開'})
    except Exception as e:
        return jsonify({'success': False, 'error': str(e)})


@socketio.on('connect')
def handle_connect(auth=None):
    """客戶端連接"""
    if controller:
        emit('status_update', controller.get_status())


@socketio.on('disconnect')
def handle_disconnect():
    """客戶端斷開"""
    pass


def main():
    """主函數"""
    print("=" * 80)
    print("🚀 CCD1視覺控制系統啟動中 (YOLOv11版本)...")
    print("📊 功能特性:")
    print("   • YOLOv11物件檢測 (DR_F/STACK)")
    print("   • 握手式狀態機控制")
    print("   • Modbus TCP Client架構")
    print("   • 50ms高頻輪詢")
    print("   • 世界座標轉換")
    print("   • 置信度閾值動態調整")
    print("=" * 80)
    
    if not CAMERA_MANAGER_AVAILABLE:
        print("❌ 相機管理器不可用，請檢查SDK導入")
        return
    
    try:
        # 設置模板文件夾路徑
        app.template_folder = os.path.join(os.path.dirname(__file__), 'templates')
        
        # 初始化控制器
        if initialize_controller():
            print("🌐 Web介面啟動中...")
            print("📱 訪問地址: http://localhost:5051")
            print("🎯 系統功能:")
            print("   • 相機連接管理 (192.168.1.8)")
            print("   • YOLOv11檢測參數調整")
            print("   • DR_F/STACK分類檢測")
            print("   • Modbus TCP握手協議")
            print("   • 即時狀態監控")
            print("   • 標定檔案管理")
            print("🔗 使用說明:")
            print("   1. 連接到Modbus服務器 (127.0.0.1:502)")
            print("   2. 初始化相機連接")
            print("   3. 調整置信度閾值")
            print("   4. 載入標定檔案 (可選)")
            print("   5. 通過PLC控制拍照和檢測")
            print("📋 寄存器映射:")
            print("   • 控制: 200-201 (指令/狀態)")
            print("   • 參數: 210-219 (置信度等)")
            print("   • 結果: 240-259 (檢測結果)")
            print("   • 世界座標: 260-279 (座標轉換)")
            print("   • 統計: 280-299 (時間/計數)")
            print("=" * 80)
            
            socketio.run(app, host='0.0.0.0', port=5051, debug=False)
            
        else:
            print("❌ 系統初始化失敗，無法啟動")
            print("請檢查以下項目:")
            print("  1. YOLOv11模型檔案 (best.pt) 是否存在")
            print("  2. ultralytics模組是否已安裝")
            print("  3. 相機管理模組是否可用")
            return
        
    except KeyboardInterrupt:
        print("\n🛑 用戶中斷，正在關閉系統...")
    except Exception as e:
        print(f"❌ 系統運行錯誤: {e}")
    finally:
        try:
            if controller:
                controller.disconnect()
        except:
            pass
        print("✅ 系統已安全關閉")


if __name__ == "__main__":
    main()