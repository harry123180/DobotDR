# -*- coding: utf-8 -*-
"""
CCD1VisionCode_YOLOv11_Enhanced.py - CCD1視覺控制系統 YOLOv11版本 (Logging增強版) - 修正版
整合YOLOv11物件檢測功能，支援DR_F/STACK分類檢測
基於Modbus TCP Client架構，實現握手式狀態機控制
適配pymodbus 3.9.2
完整logging系統，支援日誌輪替和追蹤
"""

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
from logging.handlers import RotatingFileHandler
from dataclasses import dataclass, asdict
from datetime import datetime
from enum import IntEnum

# ==================== Logging配置系統 ====================
def setup_logging(module_name: str):
    """統一設置logging配置"""
    # 日誌目錄：執行檔同層目錄下的logs資料夾
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
    os.makedirs(log_dir, exist_ok=True)
    
    # 格式化器
    formatter = logging.Formatter(
        '%(asctime)s [%(levelname)s] %(name)s:%(funcName)s:%(lineno)d - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # 文件處理器 (輪替日誌，保存一週)
    file_handler = RotatingFileHandler(
        os.path.join(log_dir, f'{module_name}.log'),
        maxBytes=10*1024*1024,  # 10MB
        backupCount=7,          # 保留7個檔案
        encoding='utf-8'
    )
    file_handler.setFormatter(formatter)
    
    # 控制台處理器
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    
    # 配置logger
    logger = logging.getLogger(module_name)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger

# 初始化模組logger
logger = setup_logging('CCD1Vision')

# 檢查YOLOv11可用性
YOLO_AVAILABLE = False
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
    logger.info("YOLOv11模組導入成功")
except ImportError as e:
    logger.error(f"YOLOv11模組導入失敗: {e}")
    logger.info("請安裝: pip install ultralytics")
    YOLO_AVAILABLE = False

# 導入Modbus TCP Client (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
    logger.info("Modbus Client模組導入成功 (pymodbus 3.9.2)")
except ImportError as e:
    logger.error(f"Modbus Client模組導入失敗: {e}")
    logger.info("請安裝: pip install pymodbus>=3.0.0")
    MODBUS_AVAILABLE = False

# 導入相機管理模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'API'))
try:
    from camera_manager import OptimizedCameraManager, CameraConfig, CameraMode, PixelFormat
    CAMERA_MANAGER_AVAILABLE = True
    logger.info("相機管理模組導入成功")
except ImportError as e:
    logger.error(f"無法導入 camera_manager 模組: {e}")
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
        self.models = {}  # {model_id: YOLO_model}
        self.model_paths = {}  # {model_id: file_path}
        self.current_model_id = 0  # 0=未指定模型
        self.model_switch_count = 0
        self.logger = setup_logging('YOLOModelManager')
        
        # 掃描模型檔案
        self.scan_model_files()
    
    def scan_model_files(self):
        """掃描工作目錄中的模型檔案"""
        try:
            self.logger.info("掃描YOLO模型檔案...")
            
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
                        self.logger.info(f"發現模型{i}: {pattern}")
                        break
            
            self.logger.info(f"總共發現 {len(self.model_paths)} 個模型檔案")
            
            # 檢查原有的best.pt檔案，作為模型1的備選
            best_pt_path = os.path.join(self.working_dir, "best.pt")
            if os.path.exists(best_pt_path) and 1 not in self.model_paths:
                self.model_paths[1] = best_pt_path
                self.logger.info("將best.pt指定為模型1")
            
        except Exception as e:
            self.logger.error(f"掃描模型檔案失敗: {e}", exc_info=True)
    
    def load_model(self, model_id: int) -> bool:
        """載入指定模型"""
        try:
            if model_id == 0:
                # 卸載當前模型
                self.current_model_id = 0
                self.logger.info("卸載當前模型")
                return True
            
            if model_id < 1 or model_id > 20:
                self.logger.error(f"模型ID超出範圍: {model_id} (應為1-20)")
                return False
            
            if model_id not in self.model_paths:
                self.logger.error(f"模型{model_id}檔案不存在")
                return False
            
            self.logger.info(f"載入模型{model_id}: {self.model_paths[model_id]}")
            
            # 載入YOLO模型
            if YOLO_AVAILABLE:
                model = YOLO(self.model_paths[model_id])
                self.models[model_id] = model
                self.current_model_id = model_id
                self.model_switch_count += 1
                self.logger.info(f"模型{model_id}載入成功")
                return True
            else:
                self.logger.error("YOLOv11模組不可用")
                return False
            
        except Exception as e:
            self.logger.error(f"載入模型{model_id}失敗: {e}", exc_info=True)
            return False
    
    def load_highest_model(self) -> bool:
        """載入編號最大的模型 - 新增功能"""
        try:
            if not self.model_paths:
                self.logger.warning("未發現任何模型檔案")
                return False
            
            # 找到最大的模型ID
            max_model_id = max(self.model_paths.keys())
            self.logger.info(f"嘗試載入最大編號模型: 模型{max_model_id}")
            
            success = self.load_model(max_model_id)
            if success:
                self.logger.info(f"成功載入最大編號模型{max_model_id}")
            else:
                self.logger.error(f"載入最大編號模型{max_model_id}失敗")
            
            return success
            
        except Exception as e:
            self.logger.error(f"載入最大編號模型失敗: {e}", exc_info=True)
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
        self.logger = setup_logging('YOLOv11Detector')
        
        # 檢查是否有可用模型，如果有則載入最大編號模型 - 新增功能
        if self.model_manager.get_available_model_count() > 0:
            self.logger.info("自動載入編號最大的模型作為預設模型")
            self.model_manager.load_highest_model()
    
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
        self.logger.info(f"置信度閾值更新為: {self.confidence_threshold}")
    
    def detect(self, image: np.ndarray) -> YOLODetectionResult:
        """執行YOLOv11檢測 - 支援三種分類"""
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
            
            # 執行推論
            results = current_model(image, conf=self.confidence_threshold, verbose=False)
            
            # 處理檢測結果
            if results and len(results) > 0:
                detections = results[0]
                
                if detections.boxes is not None and len(detections.boxes) > 0:
                    boxes = detections.boxes.cpu().numpy()
                    
                    for box in boxes:
                        # 獲取類別ID和置信度
                        class_id = int(box.cls[0])
                        confidence = float(box.conf[0])
                        
                        if confidence >= self.confidence_threshold:
                            # 計算中心點座標
                            x1, y1, x2, y2 = box.xyxy[0]
                            center_x = float((x1 + x2) / 2)
                            center_y = float((y1 + y2) / 2)
                            
                            # 根據類別分類
                            if class_id == 0:  # DR_F
                                result.dr_f_coords.append((center_x, center_y))
                                result.dr_f_count += 1
                            elif class_id == 1:  # stack  
                                result.stack_coords.append((center_x, center_y))
                                result.stack_count += 1
                    
                    result.total_detections = result.dr_f_count + result.stack_count
                    result.success = True
                    self.logger.debug(f"檢測成功: DR_F={result.dr_f_count}, STACK={result.stack_count}")
                else:
                    result.success = True  # 檢測成功但無目標
                    self.logger.debug("檢測成功但無目標")
            else:
                result.success = True  # 檢測成功但無結果
                self.logger.debug("檢測成功但無結果")
                
        except Exception as e:
            result.error_message = f"YOLOv11檢測失敗: {e}"
            self.logger.error(f"YOLOv11檢測異常: {e}", exc_info=True)
        
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
        self.logger = setup_logging('CoordinateTransformer')
    
    def load_calibration_data(self, intrinsic_file: str, extrinsic_file: str) -> bool:
        """載入標定數據 - 保持原有檔案載入邏輯"""
        try:
            self.logger.info("CameraCoordinateTransformer載入標定數據...")
            self.logger.info(f"內參檔案: {intrinsic_file}")
            self.logger.info(f"外參檔案: {extrinsic_file}")
            
            # 載入內參 (相機矩陣) - 保持原有邏輯
            try:
                intrinsic_data = np.load(intrinsic_file, allow_pickle=True)
                
                if hasattr(intrinsic_data, 'shape') and intrinsic_data.shape == (3, 3):
                    # 直接是3x3數組
                    self.camera_matrix = intrinsic_data
                    self.dist_coeffs = np.zeros((1, 5))  # 使用零值畸變係數
                    self.logger.info("載入3x3相機矩陣，使用零值畸變係數")
                elif isinstance(intrinsic_data, dict):
                    # 字典格式
                    self.camera_matrix = intrinsic_data['camera_matrix']
                    self.dist_coeffs = intrinsic_data.get('dist_coeffs', np.zeros((1, 5)))
                    self.logger.info("從字典載入相機矩陣和畸變係數")
                elif hasattr(intrinsic_data, 'item') and callable(intrinsic_data.item):
                    # 字典項目格式
                    dict_data = intrinsic_data.item()
                    if isinstance(dict_data, dict):
                        self.camera_matrix = dict_data['camera_matrix']
                        self.dist_coeffs = dict_data.get('dist_coeffs', np.zeros((1, 5)))
                        self.logger.info("從字典項目載入相機矩陣和畸變係數")
                else:
                    # 直接作為相機矩陣使用
                    self.camera_matrix = intrinsic_data
                    self.dist_coeffs = np.zeros((1, 5))
                    self.logger.info("直接使用數據作為相機矩陣")
                    
            except Exception as e1:
                self.logger.error(f"內參載入失敗: {e1}", exc_info=True)
                return False
            
            # 檢查是否有單獨的畸變係數檔案
            dist_coeffs_file = intrinsic_file.replace('camera_matrix', 'dist_coeffs')
            if os.path.exists(dist_coeffs_file) and dist_coeffs_file != intrinsic_file:
                try:
                    dist_data = np.load(dist_coeffs_file, allow_pickle=True)
                    if hasattr(dist_data, 'shape'):
                        self.dist_coeffs = dist_data
                        self.logger.info(f"載入單獨的畸變係數檔案: {dist_data.shape}")
                except Exception as e:
                    self.logger.warning(f"載入畸變係數檔案失敗，使用零值: {e}")
            
            # 載入外參
            try:
                extrinsic_data = np.load(extrinsic_file, allow_pickle=True)
                
                if isinstance(extrinsic_data, dict):
                    # 直接字典格式
                    self.rvec = extrinsic_data['rvec']
                    self.tvec = extrinsic_data['tvec']
                    self.logger.info("從字典載入外參")
                elif hasattr(extrinsic_data, 'item') and callable(extrinsic_data.item) and extrinsic_data.shape == ():
                    # 0維數組包含字典
                    dict_data = extrinsic_data.item()
                    if isinstance(dict_data, dict):
                        self.rvec = dict_data['rvec']
                        self.tvec = dict_data['tvec']
                        self.logger.info("從字典項目載入外參")
                else:
                    self.logger.error("未知的外參檔案格式")
                    return False
                
                # 計算旋轉矩陣
                self.rotation_matrix, _ = cv2.Rodrigues(self.rvec)
                
            except Exception as e2:
                self.logger.error(f"外參載入失敗: {e2}", exc_info=True)
                return False
            
            # 驗證載入的數據
            self.logger.info("載入數據驗證:")
            self.logger.info(f"相機矩陣: {self.camera_matrix.shape}, det={np.linalg.det(self.camera_matrix):.2f}")
            self.logger.info(f"畸變係數: {self.dist_coeffs.shape}, 非零個數: {np.count_nonzero(self.dist_coeffs)}")
            self.logger.info(f"旋轉向量: {self.rvec.shape}, 範圍: [{self.rvec.min():.3f}, {self.rvec.max():.3f}]")
            self.logger.info(f"平移向量: {self.tvec.shape}, 範圍: [{self.tvec.min():.3f}, {self.tvec.max():.3f}]")
            self.logger.info(f"旋轉矩陣: {self.rotation_matrix.shape}, det={np.linalg.det(self.rotation_matrix):.3f}")
            
            self.is_valid_flag = True
            self.logger.info("座標轉換器載入成功")
            return True
            
        except Exception as e:
            self.logger.error(f"座標轉換器載入失敗: {e}", exc_info=True)
            return False
    
    def pixel_to_world(self, pixel_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """像素座標轉世界座標 - 修正版實現"""
        if not self.is_valid_flag:
            self.logger.error("標定數據無效，無法進行座標轉換")
            return []
        
        try:
            world_coords = []
            
            for px, py in pixel_coords:
                self.logger.debug(f"轉換像素座標: ({px:.1f}, {py:.1f})")
                
                # 步驟1: 去畸變處理
                pixel_point = np.array([[[float(px), float(py)]]], dtype=np.float32)
                
                undistorted_points = cv2.undistortPoints(
                    pixel_point, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                # 獲取歸一化座標
                x_norm, y_norm = undistorted_points[0][0]
                self.logger.debug(f"去畸變後歸一化座標: ({x_norm:.6f}, {y_norm:.6f})")
                
                # 步驟2: 構建歸一化齊次座標
                normalized_coords = np.array([x_norm, y_norm, 1.0])
                
                # 步驟3: 計算深度係數 (Z=0平面投影)
                R3 = self.rotation_matrix[2, :]  # 旋轉矩陣第三行
                denominator = np.dot(R3, normalized_coords)
                
                if abs(denominator) < 1e-6:
                    self.logger.warning(f"分母接近零，跳過此點: denominator={denominator}")
                    continue
                
                # Z=0平面，所以目標Z座標為0
                depth_scale = (0 - self.tvec[2, 0]) / denominator
                self.logger.debug(f"深度係數: {depth_scale:.6f}")
                
                # 步驟4: 計算相機座標系中的3D點
                camera_point = depth_scale * normalized_coords
                self.logger.debug(f"相機座標系點: ({camera_point[0]:.3f}, {camera_point[1]:.3f}, {camera_point[2]:.3f})")
                
                # 步驟5: 轉換到世界座標系
                tvec_3d = self.tvec.reshape(3)
                translated_point = camera_point - tvec_3d
                world_point_3d = np.dot(self.rotation_matrix.T, translated_point)
                
                # 取X, Y座標 (忽略Z座標，因為投影到Z=0平面)
                world_x = world_point_3d[0]
                world_y = world_point_3d[1]
                
                self.logger.debug(f"世界座標: ({world_x:.3f}, {world_y:.3f}) mm")
                
                world_coords.append((float(world_x), float(world_y)))
            
            self.logger.info(f"座標轉換完成，共轉換{len(world_coords)}個點")
            return world_coords
            
        except Exception as e:
            self.logger.error(f"座標轉換失敗: {e}", exc_info=True)
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
        self.logger = setup_logging('CalibrationManager')
    
    def scan_calibration_files(self) -> Dict[str, Any]:
        """修正版標定檔案掃描"""
        try:
            self.logger.info(f"掃描標定檔案目錄: {self.working_dir}")
            
            if not os.path.exists(self.working_dir):
                return {
                    'success': False,
                    'error': f"工作目錄不存在: {self.working_dir}"
                }
            
            all_files = os.listdir(self.working_dir)
            npy_files = [f for f in all_files if f.endswith('.npy')]
            
            self.logger.info(f"發現 {len(all_files)} 個檔案，其中 {len(npy_files)} 個NPY檔案")
            
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
            
            self.logger.info("掃描結果:")
            self.logger.info(f"相機矩陣檔案: {len(camera_matrix_files)}個 - {camera_matrix_files}")
            self.logger.info(f"畸變係數檔案: {len(dist_coeffs_files)}個 - {dist_coeffs_files}")
            self.logger.info(f"外參檔案: {len(extrinsic_files)}個 - {extrinsic_files}")
            self.logger.info(f"未知檔案: {len(unknown_files)}個 - {unknown_files}")
            
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
            self.logger.error(f"掃描標定檔案異常: {e}", exc_info=True)
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
            self.logger.error(f"分類檔案 {filename} 失敗: {e}", exc_info=True)
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
            
            self.logger.info("載入標定數據:")
            self.logger.info(f"相機矩陣: {camera_matrix_file}")
            self.logger.info(f"畸變係數: {dist_coeffs_file or '使用零值'}")
            self.logger.info(f"外參數據: {extrinsic_file}")
            
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
                        self.logger.info("從字典載入相機矩陣和畸變係數")
                elif hasattr(camera_data, 'shape') and camera_data.shape == (3, 3):
                    # 直接是3x3數組的情況 (您的情況)
                    camera_matrix = camera_data
                    self.logger.info("載入3x3相機矩陣數組")
                elif hasattr(camera_data, 'item') and callable(camera_data.item) and camera_data.shape == ():
                    # 0維數組包含字典的情況
                    dict_data = camera_data.item()
                    if isinstance(dict_data, dict):
                        camera_matrix = dict_data.get('camera_matrix')
                        if 'dist_coeffs' in dict_data:
                            dist_coeffs = dict_data['dist_coeffs']
                        self.logger.info("從字典項目載入相機矩陣")
            
            # 載入畸變係數 (如果單獨提供且之前沒載入)
            if dist_coeffs_file and dist_coeffs is None:
                dist_path = os.path.join(self.working_dir, dist_coeffs_file)
                dist_data = np.load(dist_path, allow_pickle=True)
                
                # 修正: 根據測試結果，dist_coeffs_DR.npy是(1,5)數組
                if hasattr(dist_data, 'shape'):
                    dist_coeffs = dist_data
                    self.logger.info(f"載入畸變係數: {dist_data.shape}")
            
            # 如果沒有畸變係數，使用零值
            if dist_coeffs is None:
                dist_coeffs = np.zeros((1, 5))
                self.logger.warning("使用零值畸變係數")
            
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
                    self.logger.info("從字典載入外參")
                elif hasattr(ext_data, 'item') and callable(ext_data.item) and ext_data.shape == ():
                    # 0維數組包含字典的情況 (您的情況)
                    dict_data = ext_data.item()
                    if isinstance(dict_data, dict):
                        rvec = dict_data.get('rvec')
                        tvec = dict_data.get('tvec')
                        self.logger.info("從字典項目載入外參")
            
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
                
                self.logger.info("標定數據載入成功")
                self.logger.info(f"相機矩陣: {camera_matrix.shape}, fx={camera_matrix[0,0]:.2f}, fy={camera_matrix[1,1]:.2f}")
                self.logger.info(f"畸變係數: {dist_coeffs.shape}, 非零個數: {np.count_nonzero(dist_coeffs)}")
                self.logger.info(f"旋轉向量: {rvec.shape}, 範圍: [{rvec.min():.3f}, {rvec.max():.3f}]")
                self.logger.info(f"平移向量: {tvec.shape}, 範圍: [{tvec.min():.3f}, {tvec.max():.3f}]")
                
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
            self.logger.error(f"載入標定數據失敗: {e}", exc_info=True)
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
        self.logger = setup_logging('StateMachine')
    
    def get_bit(self, bit_pos: StatusBits) -> bool:
        """獲取狀態位"""
        with self.lock:
            return bool(self.status_register & (1 << bit_pos))
    
    def set_bit(self, bit_pos: StatusBits, value: bool):
        """設置狀態位"""
        with self.lock:
            old_value = bool(self.status_register & (1 << bit_pos))
            if value:
                self.status_register |= (1 << bit_pos)
            else:
                self.status_register &= ~(1 << bit_pos)
            
            if old_value != value:
                self.logger.debug(f"狀態位變更: {bit_pos.name}={value}")
    
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
            self.logger.info("狀態機重置到空閒狀態")

# ==================== Modbus TCP Client服務 - 完整版 ====================
class EnhancedModbusTcpClientService:
    """增強版Modbus TCP Client服務 - YOLOv11版本 - 完整實現"""
    
    def __init__(self, server_ip="127.0.0.1", server_port=502):
        self.server_ip = server_ip
        self.server_port = server_port
        self.client: Optional[ModbusTcpClient] = None
        self.connected = False
        self.vision_controller = None
        self.logger = setup_logging('ModbusClient')
        
        # 同步控制
        self.sync_thread = None
        self.sync_running = False
        self.sync_interval = 0.05  # 50ms輪詢
        
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
        self.logger.info("視覺控制器引用已設置")
    
    def connect(self) -> bool:
        """連接到Modbus TCP服務器"""
        try:
            if not MODBUS_AVAILABLE:
                self.logger.error("Modbus TCP Client模組不可用")
                return False
            
            self.logger.info(f"正在連接Modbus TCP服務器: {self.server_ip}:{self.server_port}")
            
            # 修正：適配pymodbus 3.9.2的參數
            self.client = ModbusTcpClient(
                host=self.server_ip,
                port=self.server_port,
                timeout=3.0
            )
            
            if self.client.connect():
                self.connected = True
                self.connection_count += 1
                self.logger.info("Modbus TCP連接建立成功")
                
                # 初始化寄存器值
                self._initialize_registers()
                
                return True
            else:
                self.logger.error("Modbus TCP連接失敗")
                return False
                
        except Exception as e:
            self.logger.error(f"Modbus TCP連接異常: {e}", exc_info=True)
            return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        try:
            # 停止同步線程
            if self.sync_running:
                self.stop_sync()
            
            # 關閉連接
            if self.client and self.connected:
                self.client.close()
                self.logger.info("Modbus TCP連接已斷開")
            
            self.connected = False
            self.client = None
            
        except Exception as e:
            self.logger.error(f"斷開Modbus連接異常: {e}", exc_info=True)
    
    def _initialize_registers(self):
        """初始化寄存器"""
        try:
            self.logger.info("初始化Modbus寄存器...")
            
            # 初始化控制和狀態寄存器
            self.write_register('CONTROL_COMMAND', 0)
            self.write_register('STATUS_REGISTER', 1)  # Ready=1 (0b0001)
            self.write_register('MODEL_SELECT', 0)
            
            # 初始化完成標誌
            self.write_register('CAPTURE_COMPLETE', 0)
            self.write_register('DETECT_COMPLETE', 0)
            self.write_register('OPERATION_SUCCESS', 0)
            self.write_register('ERROR_CODE', 0)
            
            # 初始化置信度閾值
            confidence_int = int(0.8 * 10000)  # 預設0.8
            self.write_register('CONFIDENCE_HIGH', (confidence_int >> 16) & 0xFFFF)
            self.write_register('CONFIDENCE_LOW', confidence_int & 0xFFFF)
            
            # 初始化版本資訊
            self.write_register('VERSION_MAJOR', 5)
            self.write_register('VERSION_MINOR', 0)
            
            # 初始化統計計數器
            self.write_register('OPERATION_COUNT', 0)
            self.write_register('ERROR_COUNT', 0)
            self.write_register('CONNECTION_COUNT', self.connection_count)
            
            self.logger.info("Modbus寄存器初始化完成")
            self.logger.info("狀態寄存器201=1 (0b0001) -> Ready=1, Running=0, Alarm=0, Init=0")
            
        except Exception as e:
            self.logger.error(f"初始化寄存器失敗: {e}", exc_info=True)
    
    def start_sync(self):
        """啟動同步線程"""
        if self.sync_running:
            self.logger.warning("同步線程已在運行中")
            return
        
        self.sync_running = True
        self.sync_thread = threading.Thread(target=self._sync_worker, daemon=True)
        self.sync_thread.start()
        self.logger.info("Modbus同步線程已啟動")
    
    def stop_sync(self):
        """停止同步線程"""
        if not self.sync_running:
            return
        
        self.sync_running = False
        if self.sync_thread and self.sync_thread.is_alive():
            self.sync_thread.join(timeout=2.0)
        self.logger.info("Modbus同步線程已停止")
    
    def _sync_worker(self):
        """同步工作線程 - 握手式狀態機"""
        self.logger.info("Modbus同步工作線程已啟動")
        
        while self.sync_running:
            try:
                if not self.connected or not self.client:
                    time.sleep(self.sync_interval)
                    continue
                
                # 讀取控制指令
                current_command = self.read_register('CONTROL_COMMAND')
                if current_command is None:
                    time.sleep(self.sync_interval)
                    continue
                
                # 檢查是否有新指令
                if current_command != self.last_control_command and current_command != 0:
                    self.logger.info(f"收到新指令: {current_command}")
                    self._handle_command(current_command)
                    self.last_control_command = current_command
                
                # 處理完成狀態的超時
                self._handle_completion_timeout()
                
                # 更新運行時間統計
                self._update_uptime()
                
                time.sleep(self.sync_interval)
                
            except Exception as e:
                self.logger.error(f"同步線程異常: {e}", exc_info=True)
                time.sleep(self.sync_interval)
        
        self.logger.info("Modbus同步工作線程已結束")
    
    def _handle_command(self, command: int):
        """處理控制指令"""
        try:
            self.current_command = command
            self.command_execution_time = time.time()
            
            # 設置Running狀態
            self.write_register('STATUS_REGISTER', 0b0010)  # Running=1
            self.logger.info(f"開始執行指令{command}，設置Running狀態")
            
            # 清除之前的完成標誌
            self.write_register('CAPTURE_COMPLETE', 0)
            self.write_register('DETECT_COMPLETE', 0)
            self.write_register('OPERATION_SUCCESS', 0)
            self.write_register('ERROR_CODE', 0)
            
            # 讀取並應用模型選擇
            model_select = self.read_register('MODEL_SELECT') or 0
            if model_select != 0 and self.vision_controller and self.vision_controller.yolo_detector:
                current_model = self.vision_controller.yolo_detector.model_manager.current_model_id
                if model_select != current_model:
                    self.logger.info(f"切換模型: {current_model} -> {model_select}")
                    success = self.vision_controller.yolo_detector.switch_model(model_select)
                    if success:
                        # 更新模型切換計數
                        switch_count = self.read_register('MODEL_SWITCH_COUNT') or 0
                        self.write_register('MODEL_SWITCH_COUNT', switch_count + 1)
            
            if command == ControlCommand.CAPTURE:
                # 單純拍照
                self._execute_capture()
            elif command == ControlCommand.CAPTURE_DETECT:
                # 拍照並檢測
                self._execute_capture_detect()
            elif command == ControlCommand.INITIALIZE:
                # 初始化
                self._execute_initialize()
            elif command == ControlCommand.CLEAR:
                # 清除指令
                self._execute_clear()
            else:
                self.logger.warning(f"未知指令: {command}")
                self.error_count += 1
                self.write_register('ERROR_CODE', 1)  # 未知指令錯誤
                self._complete_command(False)
                
        except Exception as e:
            self.logger.error(f"處理指令{command}異常: {e}", exc_info=True)
            self.error_count += 1
            self.write_register('ERROR_CODE', 99)  # 通用執行錯誤
            self._complete_command(False)
    
    def _execute_capture(self):
        """執行拍照指令"""
        try:
            self.logger.info("執行拍照指令")
            
            if not self.vision_controller:
                raise Exception("視覺控制器未設置")
            
            # 拍照
            image, capture_time = self.vision_controller.capture_image()
            
            if image is not None:
                # 更新時間統計
                self.write_register('LAST_CAPTURE_TIME', int(capture_time * 1000))
                self.write_register('LAST_PROCESS_TIME', 0)  # 純拍照無處理時間
                self.write_register('LAST_TOTAL_TIME', int(capture_time * 1000))
                
                # 設置完成標誌
                self.write_register('CAPTURE_COMPLETE', 1)
                self._complete_command(True)
                
                self.logger.info(f"拍照成功，耗時: {capture_time*1000:.2f}ms")
            else:
                raise Exception("圖像捕獲失敗")
                
        except Exception as e:
            self.logger.error(f"拍照指令執行失敗: {e}", exc_info=True)
            self.error_count += 1
            self.write_register('ERROR_CODE', 10)  # 拍照失敗
            self._complete_command(False)
    
    def _execute_capture_detect(self):
        """執行拍照檢測指令"""
        try:
            self.logger.info("執行拍照檢測指令")
            
            if not self.vision_controller:
                raise Exception("視覺控制器未設置")
            
            # 拍照並檢測
            result = self.vision_controller.capture_and_detect()
            
            if result.success:
                # 寫入檢測結果到寄存器
                self.update_detection_results(result)
                
                # 設置完成標誌
                self.write_register('CAPTURE_COMPLETE', 1)
                self.write_register('DETECT_COMPLETE', 1)
                self._complete_command(True)
                
                self.logger.info(f"拍照檢測成功: DR_F={result.dr_f_count}, STACK={result.stack_count}")
            else:
                raise Exception(f"檢測失敗: {result.error_message}")
                
        except Exception as e:
            self.logger.error(f"拍照檢測指令執行失敗: {e}", exc_info=True)
            self.error_count += 1
            self.write_register('ERROR_CODE', 20)  # 檢測失敗
            self._complete_command(False)
    
    def _execute_initialize(self):
        """執行初始化指令"""
        try:
            self.logger.info("執行初始化指令")
            
            if not self.vision_controller:
                raise Exception("視覺控制器未設置")
            
            # 初始化相機
            success = self.vision_controller.initialize_camera()
            
            if success:
                # 設置初始化完成 - 同步狀態機和Modbus寄存器
                self.vision_controller.state_machine.set_initialized(True)
                status = 0b1001  # Ready=1, Initialized=1
                self.write_register('STATUS_REGISTER', status)
                self.logger.info(f"初始化成功，狀態寄存器更新為: {status} (0b{status:04b})")
                self._complete_command(True)
            else:
                raise Exception("相機初始化失敗")
                
        except Exception as e:
            self.logger.error(f"初始化指令執行失敗: {e}", exc_info=True)
            self.error_count += 1
            self.write_register('ERROR_CODE', 30)  # 初始化失敗
            self._complete_command(False)
    
    def _execute_clear(self):
        """執行清除指令"""
        try:
            self.logger.info("執行清除指令")
            
            # 清除所有完成標誌和結果
            self.write_register('CAPTURE_COMPLETE', 0)
            self.write_register('DETECT_COMPLETE', 0)
            self.write_register('OPERATION_SUCCESS', 0)
            self.write_register('ERROR_CODE', 0)
            
            # 清除檢測結果
            self.write_register('DR_F_COUNT', 0)
            self.write_register('STACK_COUNT', 0)
            self.write_register('TOTAL_DETECTIONS', 0)
            self.write_register('DETECTION_SUCCESS', 0)
            
            # 清除座標數據
            for i in range(1, 6):
                self.write_register(f'DR_F_{i}_X', 0)
                self.write_register(f'DR_F_{i}_Y', 0)
            
            self.write_register('STACK_1_X', 0)
            self.write_register('STACK_1_Y', 0)
            
            # 清除世界座標
            self.write_register('WORLD_COORD_VALID', 0)
            for i in range(1, 6):
                self.write_register(f'DR_F_{i}_WORLD_X_HIGH', 0)
                self.write_register(f'DR_F_{i}_WORLD_X_LOW', 0)
                self.write_register(f'DR_F_{i}_WORLD_Y_HIGH', 0)
                self.write_register(f'DR_F_{i}_WORLD_Y_LOW', 0)
            
            # 重置狀態到Ready
            self.write_register('STATUS_REGISTER', 0b0001)  # Ready=1
            self.write_register('CONTROL_COMMAND', 0)  # 清除指令
            
            self.logger.info("清除指令執行完成")
            
        except Exception as e:
            self.logger.error(f"清除指令執行失敗: {e}", exc_info=True)
            self.error_count += 1
            self.write_register('ERROR_CODE', 40)  # 清除失敗
    
    def _complete_command(self, success: bool):
        """完成指令執行"""
        try:
            # 檢查最小運行時間
            elapsed = time.time() - self.command_execution_time
            if elapsed < self.min_running_duration:
                time.sleep(self.min_running_duration - elapsed)
            
            # 設置操作成功標誌
            self.write_register('OPERATION_SUCCESS', 1 if success else 0)
            
            # 更新統計
            self.operation_count += 1
            self.write_register('OPERATION_COUNT', self.operation_count)
            self.write_register('ERROR_COUNT', self.error_count)
            
            # 設置完成狀態並開始計時
            self.completion_start_time = time.time()
            
            # 保持Running狀態一段時間後回到Ready
            # 同步狀態機狀態到Modbus寄存器
            status = 0b0001  # Ready=1
            if self.vision_controller and self.vision_controller.state_machine.is_initialized():
                status |= 0b1000  # Initialized=1
                self.logger.debug("設置狀態寄存器包含Initialized位")
            
            self.write_register('STATUS_REGISTER', status)
            self.logger.info(f"指令{self.current_command}執行完成，成功: {success}, 狀態寄存器: {status} (0b{status:04b})")
            
        except Exception as e:
            self.logger.error(f"完成指令執行異常: {e}", exc_info=True)
    
    def _handle_completion_timeout(self):
        """處理完成狀態超時"""
        try:
            if self.completion_start_time > 0:
                elapsed = time.time() - self.completion_start_time
                if elapsed >= self.completion_hold_time:
                    # 清除指令寄存器
                    self.write_register('CONTROL_COMMAND', 0)
                    self.completion_start_time = 0
                    self.current_command = 0
                    self.last_control_command = 0
                    
        except Exception as e:
            self.logger.error(f"處理完成狀態超時異常: {e}", exc_info=True)
    
    def _update_uptime(self):
        """更新運行時間統計"""
        try:
            uptime_seconds = int(time.time() - self.start_time)
            hours = uptime_seconds // 3600
            minutes = (uptime_seconds % 3600) // 60
            
            self.write_register('UPTIME_HOURS', hours)
            self.write_register('UPTIME_MINUTES', minutes)
            
        except Exception as e:
            self.logger.debug(f"更新運行時間異常: {e}")
    
    def update_detection_results(self, result: YOLODetectionResult):
        """更新檢測結果到Modbus寄存器"""
        try:
            self.logger.info("寫入YOLOv11檢測結果到Modbus寄存器...")
            
            # 基本檢測結果
            self.write_register('DR_F_COUNT', result.dr_f_count)
            self.write_register('STACK_COUNT', result.stack_count)
            self.write_register('TOTAL_DETECTIONS', result.total_detections)
            self.write_register('DETECTION_SUCCESS', 1 if result.success else 0)
            self.write_register('MODEL_ID_USED', result.model_id_used)
            
            # 時間統計
            self.write_register('LAST_CAPTURE_TIME', int(result.capture_time))
            self.write_register('LAST_PROCESS_TIME', int(result.processing_time))
            self.write_register('LAST_TOTAL_TIME', int(result.total_time))
            
            # 清除之前的座標數據
            for i in range(1, 6):
                self.write_register(f'DR_F_{i}_X', 0)
                self.write_register(f'DR_F_{i}_Y', 0)
            
            self.write_register('STACK_1_X', 0)
            self.write_register('STACK_1_Y', 0)
            
            # 寫入DR_F座標 (最多5個)
            for i, (x, y) in enumerate(result.dr_f_coords[:5], 1):
                self.write_register(f'DR_F_{i}_X', int(x))
                self.write_register(f'DR_F_{i}_Y', int(y))
                self.logger.debug(f"寫入DR_F_{i}座標: ({int(x)}, {int(y)})")
            
            # 寫入STACK座標 (第1個)
            if result.stack_coords:
                x, y = result.stack_coords[0]
                self.write_register('STACK_1_X', int(x))
                self.write_register('STACK_1_Y', int(y))
                self.logger.debug(f"寫入STACK_1座標: ({int(x)}, {int(y)})")
            
            # 寫入世界座標 (如果有)
            if hasattr(result, 'dr_f_world_coords') and result.dr_f_world_coords:
                self.write_register('WORLD_COORD_VALID', 1)
                
                for i, (wx, wy) in enumerate(result.dr_f_world_coords[:5], 1):
                    # 轉換為整數格式 (單位: 0.01mm)
                    world_x_int = int(wx * 100)
                    world_y_int = int(wy * 100)
                    
                    # 處理負數 (轉換為補碼)
                    if world_x_int < 0:
                        world_x_int += 2**32
                    if world_y_int < 0:
                        world_y_int += 2**32
                    
                    # 分解為高位和低位
                    x_high = (world_x_int >> 16) & 0xFFFF
                    x_low = world_x_int & 0xFFFF
                    y_high = (world_y_int >> 16) & 0xFFFF
                    y_low = world_y_int & 0xFFFF
                    
                    self.write_register(f'DR_F_{i}_WORLD_X_HIGH', x_high)
                    self.write_register(f'DR_F_{i}_WORLD_X_LOW', x_low)
                    self.write_register(f'DR_F_{i}_WORLD_Y_HIGH', y_high)
                    self.write_register(f'DR_F_{i}_WORLD_Y_LOW', y_low)
                    
                    self.logger.debug(f"寫入DR_F_{i}世界座標: ({wx:.2f}, {wy:.2f}) mm")
            else:
                self.write_register('WORLD_COORD_VALID', 0)
            
            self.logger.info("檢測結果寫入Modbus寄存器完成")
            
        except Exception as e:
            self.logger.error(f"寫入檢測結果到Modbus寄存器失敗: {e}", exc_info=True)
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器 - 適配pymodbus 3.9.2"""
        try:
            if not self.connected or not self.client:
                return None
            
            address = self.REGISTERS.get(register_name)
            if address is None:
                self.logger.error(f"未知寄存器: {register_name}")
                return None
            
            # 修正：適配pymodbus 3.9.2的API
            response = self.client.read_holding_registers(address, count=1, slave=1)
            
            if response.isError():
                self.logger.warning(f"讀取寄存器{register_name}失敗: {response}")
                return None
            
            return response.registers[0]
            
        except Exception as e:
            self.logger.error(f"讀取寄存器{register_name}異常: {e}", exc_info=True)
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器 - 適配pymodbus 3.9.2"""
        try:
            if not self.connected or not self.client:
                return False
            
            address = self.REGISTERS.get(register_name)
            if address is None:
                self.logger.error(f"未知寄存器: {register_name}")
                return False
            
            # 確保值在16位範圍內
            value = max(0, min(65535, int(value)))
            
            # 修正：適配pymodbus 3.9.2的API
            response = self.client.write_register(address, value, slave=1)
            
            if response.isError():
                self.logger.warning(f"寫入寄存器{register_name}失敗: {response}")
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"寫入寄存器{register_name}異常: {e}", exc_info=True)
            return False
    
    def read_confidence_threshold(self) -> float:
        """讀取置信度閾值"""
        try:
            high = self.read_register('CONFIDENCE_HIGH') or 0
            low = self.read_register('CONFIDENCE_LOW') or 8000
            
            confidence_int = (high << 16) + low
            confidence = confidence_int / 10000.0
            
            return max(0.1, min(1.0, confidence))
            
        except Exception as e:
            self.logger.error(f"讀取置信度閾值異常: {e}", exc_info=True)
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
            'last_command': self.last_control_command,
            'current_command': self.current_command,
            'command_processing': self.command_processing
        }
    
    def get_completion_status(self) -> Dict[str, Any]:
        """獲取完成狀態"""
        try:
            return {
                'capture_complete': bool(self.read_register('CAPTURE_COMPLETE')),
                'detect_complete': bool(self.read_register('DETECT_COMPLETE')),
                'operation_success': bool(self.read_register('OPERATION_SUCCESS')),
                'error_code': self.read_register('ERROR_CODE') or 0
            }
        except Exception as e:
            self.logger.error(f"獲取完成狀態異常: {e}", exc_info=True)
            return {
                'capture_complete': False,
                'detect_complete': False,
                'operation_success': False,
                'error_code': 99
            }

# ==================== 主控制器 ====================
class CCD1VisionController:
    """CCD1視覺檢測主控制器 - YOLOv11版本 (Logging增強版)"""
    
    def __init__(self):
        # 基本配置
        self.working_dir = os.path.dirname(os.path.abspath(__file__))
        self.server_ip = "127.0.0.1"
        self.server_port = 502
        self.camera_ip = "192.168.1.8"
        self.logger = setup_logging('CCD1VisionController')
        
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
        
        # 檢查和初始化各組件
        component_status = self._check_and_initialize_components()
        
        # 根據組件狀態執行相應的後續操作
        if component_status['modbus_connected'] or component_status['calibration_loaded']:
            # 嘗試自動初始化相機（即使部分組件未連接）
            self.logger.info("嘗試自動初始化相機...")
            try:
                camera_success = self.initialize_camera()
                if camera_success:
                    self.logger.info("相機自動初始化成功")
                    self.state_machine.set_initialized(True)
                    self.state_machine.set_ready(True)
                    self.logger.info("系統就緒，進入Ready狀態")
                else:
                    self.logger.warning("相機自動初始化失敗，請透過Web介面手動初始化")
            except Exception as e:
                self.logger.warning(f"相機自動初始化異常: {e}")
        else:
            self.logger.info("等待手動初始化各組件")
            self.logger.info("請透過Web介面:")
            self.logger.info("1. 連接Modbus服務器")
            self.logger.info("2. 初始化相機連接")
            self.logger.info("3. 載入標定檔案（可選）")
    
    def _check_yolo_availability(self):
        """檢查YOLOv11模型可用性 - 支援多模型"""
        if not YOLO_AVAILABLE:
            error_msg = "YOLOv11模組不可用，請安裝ultralytics"
            self.logger.error(error_msg)
            self.state_machine.set_alarm(True)
            raise RuntimeError(error_msg)
        
        self.logger.info("YOLOv11模組可用")
        self.yolo_detector = YOLOv11Detector(self.working_dir, 0.8)
        
        if self.yolo_detector.model_manager.get_available_model_count() > 0:
            self.logger.info("YOLOv11檢測器初始化成功")
        else:
            error_msg = "未發現任何YOLOv11模型檔案"
            self.logger.error(error_msg)
            self.state_machine.set_alarm(True)
            raise RuntimeError(error_msg)
    
    def _check_and_initialize_components(self):
        """檢查和初始化各組件狀態"""
        self.logger.info("檢查系統組件狀態...")
        
        # 1. 檢查Modbus客戶端連接狀態
        self.logger.info("檢查Modbus客戶端連接狀態...")
        modbus_connected = False
        try:
            modbus_connected = self.modbus_client.connect()
            if modbus_connected:
                self.modbus_client.start_sync()
                self.logger.info("Modbus客戶端已連接並啟動同步")
                
                # 讀取並同步置信度閾值
                confidence = self.modbus_client.read_confidence_threshold()
                if self.yolo_detector:
                    self.yolo_detector.update_confidence_threshold(confidence)
                    self.logger.info(f"置信度閾值已同步: {confidence}")
            else:
                self.logger.warning("Modbus客戶端連接失敗，將在需要時手動連接")
        except Exception as e:
            self.logger.warning(f"Modbus客戶端連接異常，將在需要時手動連接: {e}")
        
        # 2. 檢查內外參檔案載入狀態
        self.logger.info("檢查內外參檔案載入狀態...")
        calibration_loaded = False
        try:
            # 掃描標定檔案
            scan_result = self.calibration_manager.scan_calibration_files()
            if scan_result['success']:
                self.logger.info(f"標定檔案掃描成功: {scan_result.get('scan_details', '')}")
                
                # 載入標定數據
                load_result = self.calibration_manager.load_calibration_data()
                if load_result['success']:
                    calibration_loaded = True
                    self.logger.info("內外參檔案已成功載入")
                    self.logger.info(f"內參檔案: {load_result.get('camera_matrix_file', 'N/A')}")
                    self.logger.info(f"外參檔案: {load_result.get('extrinsic_file', 'N/A')}")
                    self.logger.info("座標轉換器: 已啟用")
                else:
                    self.logger.warning(f"標定數據載入失敗: {load_result['error']}")
            else:
                self.logger.warning(f"標定檔案掃描失敗: {scan_result['error']}")
                
        except Exception as e:
            self.logger.warning(f"標定檔案檢查異常: {e}")
        
        # 3. 設置系統初始狀態
        if modbus_connected or calibration_loaded:
            self.logger.info("核心組件檢查完成")
            self.state_machine.set_alarm(False)
            # 注意：相機初始化將在後續手動或自動執行
        else:
            self.logger.warning("核心組件檢查：某些組件可能需要手動初始化")
            # 不設置Alarm，允許手動初始化
        
        self.logger.info("組件檢查完成")
        return {
            'modbus_connected': modbus_connected,
            'calibration_loaded': calibration_loaded
        }
    
    def _add_world_coordinates_yolo(self, result: YOLODetectionResult):
        """為YOLOv11結果添加世界座標 - 修正版"""
        try:
            if not self.calibration_manager.transformer.is_valid():
                self.logger.warning("標定數據無效，跳過世界座標轉換")
                return
            
            self.logger.info("執行YOLOv11結果世界座標轉換...")
            
            # 僅轉換DR_F座標（作為圓心處理）
            if result.dr_f_coords:
                self.logger.info(f"轉換{len(result.dr_f_coords)}個DR_F目標座標")
                try:
                    # 使用修正版的像素到世界座標轉換
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
                            self.logger.debug(f"DR_F {i+1}: 像素({px:.1f}, {py:.1f}) → 世界({wx:.2f}, {wy:.2f}) mm")
                        
                        self.logger.info(f"世界座標轉換成功，共轉換{len(world_coords)}個DR_F目標")
                    else:
                        self.logger.error("世界座標轉換失敗：轉換結果為空")
                        result.dr_f_world_coords = []
                        
                except Exception as e:
                    self.logger.error(f"DR_F座標轉換失敗: {e}", exc_info=True)
                    result.dr_f_world_coords = []
            else:
                self.logger.debug("無DR_F目標需要轉換")
                result.dr_f_world_coords = []
                    
        except Exception as e:
            self.logger.error(f"YOLOv11世界座標轉換失敗: {e}", exc_info=True)
            result.dr_f_world_coords = []
    
    def connect_modbus(self) -> Dict[str, Any]:
        """連接Modbus TCP服務器"""
        try:
            if self.modbus_client.connect():
                # 確保同步線程啟動
                if not self.modbus_client.sync_running:
                    self.modbus_client.start_sync()
                    self.logger.info("Modbus同步線程已啟動")
                else:
                    self.logger.info("Modbus同步線程已在運行中")
                
                # 讀取並同步置信度閾值
                confidence = self.modbus_client.read_confidence_threshold()
                if self.yolo_detector:
                    self.yolo_detector.update_confidence_threshold(confidence)
                    self.logger.info(f"置信度閾值已同步: {confidence}")
                
                return {
                    'success': True,
                    'message': f'Modbus TCP連接成功: {self.server_ip}:{self.server_port}',
                    'connection_status': self.modbus_client.get_connection_status(),
                    'sync_thread_running': self.modbus_client.sync_running
                }
                
        except Exception as e:
            self.logger.error(f"Modbus連接異常: {e}", exc_info=True)
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
            self.logger.error(f"斷開Modbus連接失敗: {e}", exc_info=True)
            return {
                'success': False,
                'message': f'斷開Modbus連接失敗: {str(e)}'
            }
    
    def initialize_camera(self, ip_address: str = None) -> bool:
        """初始化相機連接"""
        try:
            if ip_address:
                self.camera_ip = ip_address
            
            # 安全關閉現有相機管理器
            if self.camera_manager:
                try:
                    self.camera_manager.shutdown()
                except Exception as e:
                    self.logger.warning(f"關閉舊相機管理器時發生異常: {e}")
                finally:
                    self.camera_manager = None
            
            # 創建相機配置 - 使用新版本的配置參數
            camera_config = CameraConfig(
                name="ccd1_camera",
                ip=self.camera_ip,
                exposure_time=5000.0,  # 增加曝光時間配合5FPS
                gain=200.0,
                frame_rate=5.0,  # 修改為5FPS
                pixel_format=PixelFormat.BAYER_GR8,
                width=2592,
                height=1944,
                trigger_mode=CameraMode.CONTINUOUS,
                auto_reconnect=True,
                # 新增頻寬控制參數
                bandwidth_limit_mbps=200,  # 200Mbps頻寬限制
                use_latest_frame_only=True,  # 啟用最新幀模式
                buffer_count=1  # 最小緩存
            )
            
            self.logger.info(f"初始化相機: {self.camera_ip} (5FPS, 200Mbps)")
            self.camera_manager = OptimizedCameraManager()
            
            # 添加相機
            success = self.camera_manager.add_camera("ccd1_camera", camera_config)
            if not success:
                raise Exception("添加相機失敗")
            
            # 連接相機
            connect_result = self.camera_manager.connect_camera("ccd1_camera")
            if not connect_result:
                raise Exception("相機連接失敗")
            
            # 開始串流
            stream_result = self.camera_manager.start_streaming(["ccd1_camera"])
            if not stream_result.get("ccd1_camera", False):
                raise Exception("開始串流失敗")
            
            # 等待相機穩定
            time.sleep(1.0)
            
            # 設置狀態機狀態
            self.state_machine.set_initialized(True)
            self.state_machine.set_alarm(False)
            self.state_machine.set_ready(True)
            
            # 同步到Modbus寄存器（如果已連接）
            if self.modbus_client and self.modbus_client.connected:
                status = 0b1001  # Ready=1, Initialized=1
                self.modbus_client.write_register('STATUS_REGISTER', status)
                self.logger.info(f"相機初始化成功，同步狀態到Modbus: {status} (0b{status:04b})")
            
            self.logger.info(f"相機初始化成功: {self.camera_ip} (頻寬控制: 200Mbps, 5FPS)")
            return True
                
        except Exception as e:
            self.state_machine.set_alarm(True)
            self.state_machine.set_initialized(False)
            self.state_machine.set_ready(False)
            
            # 同步錯誤狀態到Modbus寄存器
            if self.modbus_client and self.modbus_client.connected:
                status = 0b0100  # Alarm=1
                self.modbus_client.write_register('STATUS_REGISTER', status)
                self.logger.error(f"相機初始化失敗，同步錯誤狀態到Modbus: {status} (0b{status:04b})")
            
            self.logger.error(f"相機初始化失敗: {e}", exc_info=True)
            return False
    
    def capture_image(self) -> Tuple[Optional[np.ndarray], float]:
        """拍照"""
        self.logger.debug("開始拍照程序...")
        
        if not self.camera_manager:
            self.logger.error("相機管理器不存在")
            return None, 0.0
        
        # 檢查串流狀態
        try:
            if "ccd1_camera" not in self.camera_manager.cameras:
                self.logger.error("相機 ccd1_camera 不在管理器中")
                return None, 0.0
            
            camera = self.camera_manager.cameras["ccd1_camera"]
            if not camera.is_streaming:
                self.logger.warning("相機未在串流中，嘗試重新啟動串流...")
                restart_result = self.camera_manager.start_streaming(["ccd1_camera"])
                if not restart_result.get("ccd1_camera", False):
                    self.logger.error("重新啟動串流失敗")
                    return None, 0.0
                else:
                    self.logger.info("重新啟動串流成功")
                    time.sleep(0.5)  # 等待串流穩定
            
        except Exception as stream_check_error:
            self.logger.error(f"檢查串流狀態失敗: {stream_check_error}", exc_info=True)
            return None, 0.0
        
        # 檢查相機連接狀態
        try:
            camera_status = self.camera_manager.get_camera_status("ccd1_camera")
            self.logger.debug(f"相機狀態檢查: {camera_status}")
        except Exception as status_error:
            self.logger.warning(f"無法獲取相機狀態: {status_error}")
        
        capture_start = time.time()
        
        try:
            self.logger.debug("調用 capture_new_frame，超時時間: 1000ms")
            frame_data = self.camera_manager.capture_new_frame("ccd1_camera", timeout=1000)
            
            if frame_data is None:
                self.logger.error("capture_new_frame 返回 None")
                self.logger.info("可能原因:")
                self.logger.info("- 相機串流未啟動")
                self.logger.info("- 網路連接問題")
                self.logger.info("- 超時時間過短 (1000ms)")
                self.logger.info("- 相機幀率問題 (設置5FPS)")
                return None, 0.0
            
            capture_time = time.time() - capture_start
            self.logger.info(f"成功捕獲幀，耗時: {capture_time*1000:.2f}ms")
            
            image_array = frame_data.data
            self.logger.debug(f"圖像數據: 形狀={image_array.shape}, 類型={image_array.dtype}")
            
            if len(image_array.shape) == 2:
                display_image = cv2.cvtColor(image_array, cv2.COLOR_GRAY2BGR)
                self.logger.debug("轉換灰度圖像為BGR格式")
            else:
                display_image = image_array
                self.logger.debug("使用原始BGR圖像")
            
            return display_image, capture_time
            
        except Exception as e:
            capture_time = time.time() - capture_start
            self.logger.error(f"拍照異常: {e}", exc_info=True)
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
                self.logger.info("開始世界座標轉換...")
                self._add_world_coordinates_yolo(result)
            
            # 創建可視化圖像
            if result.success:
                self._create_yolo_visualization(image, result)
            
            self.last_result = result
            return result
            
        except Exception as e:
            result = YOLODetectionResult()
            result.error_message = f"檢測失敗: {str(e)}"
            result.total_time = (time.time() - total_start) * 1000
            self.logger.error(f"拍照檢測流程異常: {e}", exc_info=True)
            return result
    
    def _create_yolo_visualization(self, image: np.ndarray, result: YOLODetectionResult):
        """創建YOLOv11檢測結果可視化 - 支援三種分類"""
        try:
            vis_image = image.copy()
            
            # 定義顏色 (BGR格式)
            colors = {
                'DR_F': (255, 0, 0),    # 藍色
                'STACK': (0, 0, 255),     # 紅色
            }
            
            # 繪製DR_F檢測結果
            for i, (x, y) in enumerate(result.dr_f_coords):
                cv2.circle(vis_image, (int(x), int(y)), 15, colors['DR_F'], -1)
                cv2.circle(vis_image, (int(x), int(y)), 20, (255, 255, 255), 3)
                
                label = f"DR_F {i+1}"
                label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)[0]
                cv2.rectangle(vis_image, (int(x-label_size[0]//2-5), int(y-40)), 
                             (int(x+label_size[0]//2+5), int(y-10)), colors['DR_F'], -1)
                cv2.putText(vis_image, label, (int(x-label_size[0]//2), int(y-20)), 
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
            
            # 添加置信度閾值信息
            conf_text = f"Confidence >= {result.confidence_threshold:.1f}"
            cv2.putText(vis_image, conf_text, (20, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
            
            # 保存可視化圖像
            self.last_image = vis_image
            
        except Exception as e:
            self.logger.error(f"創建可視化失敗: {e}", exc_info=True)
    
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
            self.logger.error(f"圖像編碼失敗: {e}", exc_info=True)
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
            try:
                self.camera_manager.shutdown()
            except Exception as e:
                self.logger.warning(f"關閉相機管理器時發生異常: {e}")
            finally:
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
        logger.info("正在初始化CCD1視覺控制器 (YOLOv11版本)...")
        controller = CCD1VisionController()
        logger.info("CCD1視覺控制器初始化成功")
        logger.info("所有組件已自動初始化完成")
        return True
    except Exception as e:
        logger.error(f"CCD1視覺控制器初始化失敗: {e}", exc_info=True)
        return False

@app.route('/')
def index():
    """主頁面"""
    return render_template('ccd1_yolo_enhanced.html')

# ==================== Flask API端點完整實現 ====================

@app.route('/api/status')
def get_status():
    """獲取系統狀態"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        status = controller.get_status()
        return jsonify({'success': True, 'status': status})
    except Exception as e:
        logger.error(f"獲取系統狀態API異常: {e}", exc_info=True)
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
        logger.error(f"連接Modbus API異常: {e}", exc_info=True)
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
        logger.error(f"斷開Modbus API異常: {e}", exc_info=True)
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
        logger.error(f"初始化相機API異常: {e}", exc_info=True)
        return jsonify({'success': False, 'error': str(e)})

@app.route('/api/capture_and_detect', methods=['POST'])
def capture_and_detect():
    """拍照並檢測 - 修正JSON序列化 + Modbus寄存器寫入"""
    if not controller:
        return jsonify({'success': False, 'error': '控制器未初始化'})
    
    try:
        result = controller.capture_and_detect()
        
        # Web API調用後也寫入Modbus寄存器
        if controller.modbus_client and controller.modbus_client.connected:
            logger.info("Flask API - 準備寫入檢測結果到Modbus寄存器...")
            try:
                controller.modbus_client.update_detection_results(result)
                logger.info("Flask API - Modbus寄存器寫入完成")
            except Exception as modbus_error:
                logger.error(f"Flask API - Modbus寄存器寫入失敗: {modbus_error}", exc_info=True)
        
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
            'stack_count': int(result.stack_count),
            'total_detections': int(result.total_detections),
            'dr_f_coords': convert_to_serializable(result.dr_f_coords),
            'stack_coords': convert_to_serializable(result.stack_coords),
            'dr_f_world_coords': convert_to_serializable(getattr(result, 'dr_f_world_coords', [])),
            'confidence_threshold': float(result.confidence_threshold),
            'capture_time': float(result.capture_time),
            'processing_time': float(result.processing_time),
            'total_time': float(result.total_time),
            'timestamp': str(result.timestamp),
            'image': controller.get_image_base64(),
            'error_message': result.error_message,
            'world_coord_valid': len(getattr(result, 'dr_f_world_coords', [])) > 0,
            'model_id_used': int(result.model_id_used),
            'modbus_write_success': controller.modbus_client.connected if controller.modbus_client else False
        }
        
        # 調試：打印響應數據
        logger.debug("API響應數據檢查:")
        logger.debug(f"DR_F: {response['dr_f_count']}")
        logger.debug(f"STACK: {response['stack_count']}")
        logger.debug(f"模型ID: {response['model_id_used']}")
        
        socketio.emit('detection_result', response)
        return jsonify(response)
        
    except Exception as e:
        logger.error(f"Flask API capture_and_detect 異常: {e}", exc_info=True)
        return jsonify({'success': False, 'error': str(e)})

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
        logger.error(f"切換模型API異常: {e}", exc_info=True)
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
        logger.error(f"獲取模型狀態API異常: {e}", exc_info=True)
        return jsonify({'success': False, 'error': str(e)})

# ==================== SocketIO事件處理 ====================

@socketio.on('connect')
def handle_connect(auth=None):
    """客戶端連接"""
    logger.info("新的SocketIO客戶端已連接")
    if controller:
        emit('status_update', controller.get_status())

@socketio.on('disconnect')
def handle_disconnect():
    """客戶端斷開"""
    logger.info("SocketIO客戶端已斷開")

@socketio.on('request_status')
def handle_request_status():
    """處理狀態請求"""
    if controller:
        emit('status_update', controller.get_status())

def main():
    """主函數"""
    logger.info("=" * 80)
    logger.info("CCD1視覺控制系統啟動中 (YOLOv11版本 - Logging增強版)...")
    logger.info("功能特性:")
    logger.info("• YOLOv11物件檢測 (DR_F/STACK)")
    logger.info("• 握手式狀態機控制")
    logger.info("• Modbus TCP Client架構")
    logger.info("• 50ms高頻輪詢")
    logger.info("• 世界座標轉換")
    logger.info("• 置信度閾值動態調整")
    logger.info("• 完整日誌系統 (輪替保存)")
    logger.info("• 自動載入最大編號模型")
    logger.info("=" * 80)
    
    if not CAMERA_MANAGER_AVAILABLE:
        logger.error("相機管理器不可用，請檢查SDK導入")
        return
    
    try:
        # 設置模板文件夾路徑
        app.template_folder = os.path.join(os.path.dirname(__file__), 'templates')
        
        # 初始化控制器
        if initialize_controller():
            logger.info("Web介面啟動中...")
            logger.info("訪問地址: http://localhost:5051")
            logger.info("系統功能:")
            logger.info("• 相機連接管理 (192.168.1.8)")
            logger.info("• YOLOv11檢測參數調整")
            logger.info("• DR_F/STACK分類檢測")
            logger.info("• Modbus TCP握手協議")
            logger.info("• 即時狀態監控")
            logger.info("• 標定檔案管理")
            logger.info("• 完整執行緒安全日誌")
            logger.info("使用說明:")
            logger.info("1. 連接到Modbus服務器 (127.0.0.1:502)")
            logger.info("2. 初始化相機連接")
            logger.info("3. 調整置信度閾值")
            logger.info("4. 載入標定檔案 (可選)")
            logger.info("5. 通過PLC控制拍照和檢測")
            logger.info("寄存器映射:")
            logger.info("• 控制: 200-201 (指令/狀態)")
            logger.info("• 參數: 210-219 (置信度等)")
            logger.info("• 結果: 240-259 (檢測結果)")
            logger.info("• 世界座標: 260-279 (座標轉換)")
            logger.info("• 統計: 280-299 (時間/計數)")
            logger.info("日誌系統:")
            logger.info("• 日誌位置: ./logs/")
            logger.info("• 檔案輪替: 10MB/檔，保留7檔")
            logger.info("• 記錄級別: DEBUG/INFO/WARNING/ERROR/CRITICAL")
            logger.info("• 執行緒安全: 支援並發寫入")
            logger.info("=" * 80)
            
            socketio.run(app, host='0.0.0.0', port=5051, debug=False)
            
        else:
            logger.error("系統初始化失敗，無法啟動")
            logger.error("請檢查以下項目:")
            logger.error("1. YOLOv11模型檔案 (best.pt) 是否存在")
            logger.error("2. ultralytics模組是否已安裝")
            logger.error("3. 相機管理模組是否可用")
            return
        
    except KeyboardInterrupt:
        logger.info("用戶中斷，正在關閉系統...")
    except Exception as e:
        logger.error(f"系統運行錯誤: {e}", exc_info=True)
    finally:
        try:
            if controller:
                controller.disconnect()
        except Exception as e:
            logger.warning(f"關閉控制器時發生異常: {e}")
        logger.info("系統已安全關閉")

if __name__ == "__main__":
    main()