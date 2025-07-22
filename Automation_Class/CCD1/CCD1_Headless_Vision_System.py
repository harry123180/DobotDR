# -*- coding: utf-8 -*-
"""
CCD1_Headless_Vision_System.py - CCD1無頭視覺檢測系統 (記憶體優化版本)
基於原有架構，移除Flask可視化，提供純Class接口
支援多標籤YOLO檢測、動態參數配置、保護區域等功能
適配pymodbus 3.9.2
重點修復記憶體洩漏問題並增加記憶體監控節點
"""

import os
import time
import threading
import gc
import json
import psutil  # 添加 psutil 導入
from typing import Optional, Dict, Any, Tuple, List, Union
from dataclasses import dataclass, asdict
from datetime import datetime
from enum import IntEnum
import numpy as np
import cv2
import logging
import weakref
from contextlib import contextmanager

# 檢查YOLO可用性
YOLO_AVAILABLE = False
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

# 導入相機管理模組 - 根據項目結構調整
try:
    import sys
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'API'))
    from camera_manager import OptimizedCameraManager, CameraConfig, CameraMode, PixelFormat
    CAMERA_MANAGER_AVAILABLE = True
except ImportError:
    CAMERA_MANAGER_AVAILABLE = False


# ==================== 記憶體監控器 ====================
class MemoryMonitor:
    """記憶體監控器 - 追蹤不同步驟的記憶體使用"""
    
    def __init__(self):
        self.process = psutil.Process()
        self.baseline_memory = self._get_current_memory()
        self.checkpoints = {}
        
    def _get_current_memory(self) -> float:
        """獲取當前記憶體使用量 (MB)"""
        try:
            return self.process.memory_info().rss / 1024 / 1024
        except Exception as e:
            print(f"獲取記憶體使用量失敗: {e}")
            # 使用替代方法
            import os
            import resource
            try:
                # Unix/Linux 系統
                return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
            except:
                # Windows 系統或其他情況，返回預設值
                return 0.0
    
    def create_checkpoint(self, name: str) -> float:
        """創建記憶體檢查點"""
        current_memory = self._get_current_memory()
        self.checkpoints[name] = {
            'memory': current_memory,
            'timestamp': datetime.now(),
            'diff_from_baseline': current_memory - self.baseline_memory
        }
        return current_memory
    
    def get_memory_report(self) -> Dict[str, Any]:
        """獲取記憶體報告"""
        current = self._get_current_memory()
        return {
            'baseline': self.baseline_memory,
            'current': current,
            'total_change': current - self.baseline_memory,
            'growth_rate': ((current - self.baseline_memory) / self.baseline_memory) * 100,
            'checkpoints': self.checkpoints.copy()
        }
    
    def reset_baseline(self):
        """重置基準記憶體"""
        self.baseline_memory = self._get_current_memory()
        self.checkpoints.clear()


# ==================== 資源管理器 ====================
@contextmanager
def auto_cleanup(*objects):
    """自動資源清理上下文管理器"""
    try:
        yield
    finally:
        for obj in objects:
            if obj is not None:
                if hasattr(obj, '__del__'):
                    try:
                        obj.__del__()
                    except:
                        pass
                del obj
        gc.collect()


class ResourceTracker:
    """資源追蹤器"""
    
    def __init__(self):
        self._tracked_objects = weakref.WeakSet()
        self._allocation_count = 0
        
    def track(self, obj):
        """追蹤對象"""
        self._tracked_objects.add(obj)
        self._allocation_count += 1
        
    def get_stats(self):
        """獲取資源統計"""
        return {
            'active_objects': len(self._tracked_objects),
            'total_allocated': self._allocation_count
        }
    
    def force_cleanup(self):
        """強制清理"""
        count = 0
        for obj in list(self._tracked_objects):
            try:
                if hasattr(obj, 'cleanup'):
                    obj.cleanup()
                count += 1
            except:
                pass
        
        collected = gc.collect()
        return count, collected


# ==================== 配置結構定義 ====================
@dataclass
class CCD1VisionConfig:
    """CCD1視覺系統配置"""
    # 相機配置
    camera_ip: str = "192.168.1.8"
    exposure_time: float = 20000.0
    gain: float = 200.0
    frame_rate: float = 5.0
    width: int = 2592
    height: int = 1944
    pixel_format: str = "BAYER_GR8"
    trigger_mode: str = "SOFTWARE_TRIGGER"  
    bandwidth_limit_mbps: int = 200
    
    # YOLO模型配置
    model_path: str = ""  
    confidence_threshold: float = 0.8
    max_model_id: int = 20 
    
    # 檢測標籤配置
    num_classes: int = 2  
    class_names: Dict[int, str] = None  
    class_colors: Dict[int, Tuple[int, int, int]] = None 
    
    # 標定檔案配置
    calibration_dir: str = ""  
    intrinsic_file: str = "camera_matrix_DR.npy"    
    extrinsic_file: str = "extrinsic_DR3.npy"       
    dist_coeffs_file: str = "dist_coeffs_DR.npy"    
    
    # 保護區域配置
    protection_zones: List[Dict] = None  
    
    # 圖像保存配置
    save_raw_image: bool = False
    save_result_image: bool = True
    save_dir: str = "./images"
    auto_cleanup_days: int = 7  
    
    # 系統配置
    working_dir: str = ""
    enable_world_coord: bool = True
    memory_cleanup_frequency: int = 10  # 降低頻率到10次
    auto_retry_on_error: bool = True
    max_retry_count: int = 3
    
    # 記憶體管理配置
    enable_memory_monitoring: bool = True  # 啟用記憶體監控
    memory_warning_threshold: float = 50.0  # 記憶體增長警告閾值(MB)
    force_gc_frequency: int = 5  # 強制垃圾回收頻率
    max_cached_images: int = 3  # 最大快取圖像數量
    
    def __post_init__(self):
        if self.working_dir == "":
            self.working_dir = os.path.dirname(os.path.abspath(__file__))
        
        if self.calibration_dir == "":
            self.calibration_dir = self.working_dir
        
        if self.class_names is None:
            self.class_names = {0: "DR_F", 1: "STACK"}
        
        if self.class_colors is None:
            self.class_colors = {
                0: (255, 0, 0),   
                1: (0, 0, 255),   
                2: (0, 255, 0),   
            }


@dataclass 
class DetectionResult:
    """檢測結果數據結構 (記憶體優化版本)"""
    success: bool = False
    error_message: str = ""
    
    # 檢測數據
    total_detections: int = 0
    detections_by_class: Dict[int, int] = None  
    coordinates_by_class: Dict[int, List[Tuple[float, float]]] = None  
    confidences_by_class: Dict[int, List[float]] = None  
    
    # 世界座標 
    world_coordinates_by_class: Dict[int, List[Tuple[float, float]]] = None
    world_coord_valid: bool = False
    
    # 時間統計
    capture_time: float = 0.0
    processing_time: float = 0.0
    total_time: float = 0.0
    timestamp: str = ""
    
    # 使用的模型
    model_id_used: int = 0
    confidence_threshold: float = 0.8
    
    # 圖像保存路徑
    raw_image_path: str = ""
    result_image_path: str = ""
    
    # 記憶體信息
    memory_usage: Dict[str, float] = None
    
    def __post_init__(self):
        if self.detections_by_class is None:
            self.detections_by_class = {}
        if self.coordinates_by_class is None:
            self.coordinates_by_class = {}
        if self.confidences_by_class is None:
            self.confidences_by_class = {}
        if self.world_coordinates_by_class is None:
            self.world_coordinates_by_class = {}
        if self.memory_usage is None:
            self.memory_usage = {}
        if not self.timestamp:
            self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    def cleanup(self):
        """清理檢測結果中的大型數據"""
        self.coordinates_by_class.clear()
        self.confidences_by_class.clear()
        self.world_coordinates_by_class.clear()
        self.memory_usage.clear()


# ==================== YOLO模型管理器 (記憶體優化版本) ====================
class YOLOModelManager:
    """YOLO模型管理器 - 支援動態多模型切換 (記憶體優化)"""
    
    def __init__(self, config: CCD1VisionConfig):
        self.config = config
        self.working_dir = config.working_dir
        self.models = {}  
        self.model_paths = {}  
        self.current_model_id = 0
        self.model_switch_count = 0
        self.memory_monitor = MemoryMonitor()
        self.resource_tracker = ResourceTracker()
        
        self.scan_model_files()
    
    def scan_model_files(self):
        """掃描模型檔案"""
        try:
            print(f"掃描YOLO模型檔案: {self.working_dir}")
            
            patterns = [
                "best.pt",  
                "model_{}.pt",
                "best_{}.pt", 
                "yolo_{}.pt",
                "dr_model_{}.pt"
            ]
            
            for i in range(1, self.config.max_model_id + 1):
                for pattern in patterns:
                    if "{}" in pattern:
                        filename = pattern.format(i)
                    else:
                        filename = pattern
                        if i > 1: 
                            continue
                    
                    model_path = os.path.join(self.working_dir, filename)
                    if os.path.exists(model_path):
                        self.model_paths[i] = model_path
                        print(f"發現模型{i}: {filename}")
                        break
            
            if self.config.model_path and os.path.exists(self.config.model_path):
                self.model_paths[1] = self.config.model_path
                print(f"使用指定模型: {self.config.model_path}")
            
            print(f"總共發現 {len(self.model_paths)} 個模型檔案")
            
        except Exception as e:
            print(f"掃描模型檔案失敗: {e}")
    
    def load_model(self, model_id: int) -> bool:
        """載入指定模型 (記憶體優化)"""
        try:
            # 記憶體檢查點 - 模型載入前
            self.memory_monitor.create_checkpoint(f"before_load_model_{model_id}")
            
            if model_id == 0:
                # 卸載當前模型
                if self.current_model_id in self.models:
                    old_model = self.models[self.current_model_id]
                    # 強制清理模型
                    if hasattr(old_model, 'model'):
                        del old_model.model
                    del self.models[self.current_model_id]
                    del old_model
                    
                self.current_model_id = 0
                print("卸載當前模型")
                
                # 強制垃圾回收
                collected = gc.collect()
                print(f"模型卸載後回收: {collected}個物件")
                
                # 記憶體檢查點 - 模型卸載後
                self.memory_monitor.create_checkpoint(f"after_unload_model")
                return True
            
            if model_id < 1 or model_id > self.config.max_model_id:
                print(f"模型ID超出範圍: {model_id}")
                return False
            
            if model_id not in self.model_paths:
                print(f"模型{model_id}檔案不存在")
                return False
            
            if not YOLO_AVAILABLE:
                print(f"YOLOv11模組不可用")
                return False
            
            print(f"載入模型{model_id}: {self.model_paths[model_id]}")
            
            # 卸載舊模型並清理記憶體
            if self.current_model_id in self.models and self.current_model_id != model_id:
                print(f"卸載舊模型{self.current_model_id}")
                old_model = self.models[self.current_model_id]
                
                # 深度清理舊模型
                if hasattr(old_model, 'model'):
                    del old_model.model
                if hasattr(old_model, 'predictor'):
                    del old_model.predictor
                
                del self.models[self.current_model_id]
                del old_model
                
                # 強制垃圾回收
                collected = gc.collect()
                print(f"舊模型清理回收: {collected}個物件")
                
                # 記憶體檢查點 - 舊模型清理後
                self.memory_monitor.create_checkpoint(f"after_cleanup_old_model")
            
            # 載入YOLO模型
            model = YOLO(self.model_paths[model_id])
            
            # 追蹤模型對象
            self.resource_tracker.track(model)
            
            self.models[model_id] = model
            self.current_model_id = model_id
            self.model_switch_count += 1
            
            # 記憶體檢查點 - 新模型載入後
            self.memory_monitor.create_checkpoint(f"after_load_model_{model_id}")
            
            print(f"模型{model_id}載入成功")
            return True
            
        except Exception as e:
            print(f"載入模型{model_id}失敗: {e}")
            return False
    
    def get_current_model(self):
        """獲取當前模型"""
        if self.current_model_id in self.models:
            return self.models[self.current_model_id]
        return None
    
    def get_available_models(self) -> List[int]:
        """獲取可用模型ID列表"""
        return list(self.model_paths.keys())
    
    def is_model_loaded(self) -> bool:
        """檢查是否有模型已載入"""
        return self.current_model_id > 0 and self.current_model_id in self.models
    
    def cleanup_all_models(self):
        """清理所有模型"""
        for model_id, model in list(self.models.items()):
            try:
                if hasattr(model, 'model'):
                    del model.model
                if hasattr(model, 'predictor'):
                    del model.predictor
                del self.models[model_id]
                del model
            except:
                pass
        
        self.models.clear()
        self.current_model_id = 0
        collected = gc.collect()
        print(f"清理所有模型，回收: {collected}個物件")
    
    def get_memory_report(self):
        """獲取記憶體報告"""
        return {
            'memory_monitor': self.memory_monitor.get_memory_report(),
            'resource_tracker': self.resource_tracker.get_stats(),
            'loaded_models': list(self.models.keys())
        }


# ==================== YOLO檢測器 (記憶體優化版本) ====================
class YOLODetector:
    """YOLO檢測器 - 支援多標籤動態配置 (記憶體優化)"""
    
    def __init__(self, config: CCD1VisionConfig):
        self.config = config
        self.model_manager = YOLOModelManager(config)
        self.confidence_threshold = config.confidence_threshold
        self.detection_count = 0
        self.memory_monitor = MemoryMonitor()
        
        # 圖像快取管理
        self.image_cache = []
        self.max_cache_size = config.max_cached_images
        
        # 自動載入模型1
        if 1 in self.model_manager.model_paths:
            self.model_manager.load_model(1)
    
    def switch_model(self, model_id: int) -> bool:
        """切換模型"""
        # 記憶體檢查點
        self.memory_monitor.create_checkpoint(f"before_switch_to_{model_id}")
        result = self.model_manager.load_model(model_id)
        self.memory_monitor.create_checkpoint(f"after_switch_to_{model_id}")
        return result
    
    def update_confidence_threshold(self, threshold: float):
        """更新置信度閾值"""
        self.confidence_threshold = max(0.1, min(1.0, threshold))
        print(f"置信度閾值更新為: {self.confidence_threshold}")
    
    def _manage_image_cache(self, image: np.ndarray):
        """管理圖像快取"""
        # 清理舊的快取圖像
        while len(self.image_cache) >= self.max_cache_size:
            old_image = self.image_cache.pop(0)
            del old_image
        
        # 添加新圖像到快取 (創建副本)
        self.image_cache.append(image.copy())
    
    def _clear_image_cache(self):
        """清理圖像快取"""
        for img in self.image_cache:
            del img
        self.image_cache.clear()
        collected = gc.collect()
        if collected > 0:
            print(f"清理圖像快取，回收: {collected}個物件")
    
    def detect(self, image: np.ndarray) -> DetectionResult:
        """執行YOLO檢測 (記憶體優化)"""
        start_time = time.time()
        result = DetectionResult()
        result.confidence_threshold = self.confidence_threshold
        result.model_id_used = self.model_manager.current_model_id
        
        # 記憶體檢查點 - 檢測開始
        self.memory_monitor.create_checkpoint("detection_start")
        
        try:
            current_model = self.model_manager.get_current_model()
            if current_model is None:
                result.error_message = f"模型{self.model_manager.current_model_id}未載入"
                return result
            
            # 記憶體檢查點 - 推論前
            self.memory_monitor.create_checkpoint("before_inference")
            
            # 執行推論 - 使用上下文管理器
            with auto_cleanup() as _:
                results = current_model(image, conf=self.confidence_threshold, verbose=False)
                
                # 記憶體檢查點 - 推論後
                self.memory_monitor.create_checkpoint("after_inference")
                
                if results and len(results) > 0:
                    detections = results[0]
                    
                    if detections.boxes is not None and len(detections.boxes) > 0:
                        # 立即轉換為numpy並處理
                        boxes_cpu = detections.boxes.cpu().numpy()
                        
                        # 處理檢測結果
                        for box in boxes_cpu:
                            class_id = int(box.cls[0])
                            confidence = float(box.conf[0])
                            
                            if class_id >= self.config.num_classes:
                                continue
                            
                            if confidence >= self.confidence_threshold:
                                x1, y1, x2, y2 = box.xyxy[0]
                                center_x = float((x1 + x2) / 2)
                                center_y = float((y1 + y2) / 2)
                                
                                if class_id not in result.detections_by_class:
                                    result.detections_by_class[class_id] = 0
                                    result.coordinates_by_class[class_id] = []
                                    result.confidences_by_class[class_id] = []
                                
                                result.detections_by_class[class_id] += 1
                                result.coordinates_by_class[class_id].append((center_x, center_y))
                                result.confidences_by_class[class_id].append(confidence)
                        
                        result.total_detections = sum(result.detections_by_class.values())
                        result.success = True
                        
                        # 立即清理boxes數據
                        del boxes_cpu
                        
                    # 強制清理檢測結果
                    if hasattr(detections, 'boxes') and detections.boxes is not None:
                        detections.boxes = None
                    
                    # 清理results
                    for res in results:
                        for attr in ['boxes', 'masks', 'keypoints', 'probs']:
                            if hasattr(res, attr):
                                setattr(res, attr, None)
                    
                    results.clear()
                    del results
            
            # 記憶體檢查點 - 處理完成
            self.memory_monitor.create_checkpoint("processing_complete")
            
            # 記憶體管理
            self.detection_count += 1
            
            # 頻繁的記憶體清理
            if self.detection_count % self.config.force_gc_frequency == 0:
                collected = gc.collect()
                if collected > 0:
                    print(f"YOLO定期清理: 回收{collected}個物件")
                
                # 記憶體檢查點 - 垃圾回收後
                self.memory_monitor.create_checkpoint("after_gc")
            
            # 清理圖像快取
            if self.detection_count % self.config.memory_cleanup_frequency == 0:
                self._clear_image_cache()
            
            # 記憶體警告檢查
            memory_report = self.memory_monitor.get_memory_report()
            if memory_report['total_change'] > self.config.memory_warning_threshold:
                print(f"記憶體警告: 增長{memory_report['total_change']:.2f}MB "
                      f"({memory_report['growth_rate']:.1f}%)")
                
                # 強制記憶體清理
                self.model_manager.resource_tracker.force_cleanup()
                collected = gc.collect()
                print(f"強制清理回收: {collected}個物件")
                
                # 重置記憶體基準
                self.memory_monitor.reset_baseline()
                
        except Exception as e:
            result.error_message = f"YOLO檢測失敗: {e}"
            print(f"YOLO檢測異常: {e}")
            
            # 異常情況下強制清理
            collected = gc.collect()
            print(f"異常清理回收: {collected}個物件")
        
        result.processing_time = (time.time() - start_time) * 1000
        
        # 記錄記憶體使用情況到結果中
        if self.config.enable_memory_monitoring:
            result.memory_usage = self.memory_monitor.get_memory_report()
        
        return result
    
    def get_memory_stats(self):
        """獲取記憶體統計"""
        return {
            'detector': self.memory_monitor.get_memory_report(),
            'model_manager': self.model_manager.get_memory_report(),
            'detection_count': self.detection_count,
            'cache_size': len(self.image_cache)
        }
    
    def cleanup(self):
        """清理檢測器資源"""
        self._clear_image_cache()
        self.model_manager.cleanup_all_models()
        collected = gc.collect()
        print(f"檢測器清理完成，回收: {collected}個物件")


# ==================== 座標轉換器 (記憶體優化版本) ====================
class CameraCoordinateTransformer:
    """相機座標轉換器 (記憶體優化)"""
    
    def __init__(self):
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvec = None
        self.tvec = None
        self.rotation_matrix = None
        self.is_valid_flag = False
        self.memory_monitor = MemoryMonitor()
    
    def load_calibration_data(self, intrinsic_file: str, extrinsic_file: str, dist_coeffs_file: str = "") -> bool:
        """載入標定數據 (記憶體優化)"""
        try:
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("before_load_calibration")
            
            print(f"載入標定數據...")
            print(f"   內參檔案: {intrinsic_file}")
            print(f"   外參檔案: {extrinsic_file}")
            print(f"   畸變係數檔案: {dist_coeffs_file if dist_coeffs_file else '使用零值'}")
            
            # 使用上下文管理器載入數據
            with auto_cleanup() as _:
                # 載入內參
                intrinsic_data = np.load(intrinsic_file, allow_pickle=True)
                
                if hasattr(intrinsic_data, 'shape') and intrinsic_data.shape == (3, 3):
                    self.camera_matrix = intrinsic_data.copy()  # 創建副本
                    self.dist_coeffs = np.zeros((1, 5), dtype=np.float32)  
                elif isinstance(intrinsic_data, dict):
                    self.camera_matrix = intrinsic_data['camera_matrix'].copy()
                    self.dist_coeffs = intrinsic_data.get('dist_coeffs', np.zeros((1, 5), dtype=np.float32)).copy()
                elif hasattr(intrinsic_data, 'item') and callable(intrinsic_data.item):
                    dict_data = intrinsic_data.item()
                    if isinstance(dict_data, dict):
                        self.camera_matrix = dict_data['camera_matrix'].copy()
                        self.dist_coeffs = dict_data.get('dist_coeffs', np.zeros((1, 5), dtype=np.float32)).copy()
                else:
                    self.camera_matrix = intrinsic_data.copy()
                    self.dist_coeffs = np.zeros((1, 5), dtype=np.float32)
                
                # 清理原始數據
                del intrinsic_data
                if 'dict_data' in locals():
                    del dict_data
                
                # 載入單獨的畸變係數檔案
                if dist_coeffs_file and os.path.exists(dist_coeffs_file):
                    try:
                        dist_data = np.load(dist_coeffs_file, allow_pickle=True)
                        if hasattr(dist_data, 'shape'):
                            self.dist_coeffs = dist_data.copy()
                            print(f"   載入畸變係數: {dist_data.shape}")
                        del dist_data
                    except Exception as e:
                        print(f"   載入畸變係數失敗，使用零值: {e}")
                
                # 載入外參
                extrinsic_data = np.load(extrinsic_file, allow_pickle=True)
                
                if isinstance(extrinsic_data, dict):
                    self.rvec = extrinsic_data['rvec'].copy()
                    self.tvec = extrinsic_data['tvec'].copy()
                elif hasattr(extrinsic_data, 'item') and callable(extrinsic_data.item) and extrinsic_data.shape == ():
                    dict_data = extrinsic_data.item()
                    if isinstance(dict_data, dict):
                        self.rvec = dict_data['rvec'].copy()
                        self.tvec = dict_data['tvec'].copy()
                    del dict_data
                else:
                    del extrinsic_data
                    return False
                
                del extrinsic_data
            
            # 計算旋轉矩陣
            self.rotation_matrix, _ = cv2.Rodrigues(self.rvec)
            
            self.is_valid_flag = True
            
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("after_load_calibration")
            
            print(f"座標轉換器載入成功")
            return True
            
        except Exception as e:
            print(f"座標轉換器載入失敗: {e}")
            return False
    
    def pixel_to_world(self, pixel_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """像素座標轉世界座標 (記憶體優化)"""
        if not self.is_valid_flag:
            return []
        
        try:
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("before_coordinate_transform")
            
            world_coords = []
            
            # 批量處理座標轉換
            for px, py in pixel_coords:
                with auto_cleanup() as _:
                    # 去畸變處理
                    pixel_point = np.array([[[float(px), float(py)]]], dtype=np.float32)
                    undistorted_points = cv2.undistortPoints(
                        pixel_point, 
                        self.camera_matrix, 
                        self.dist_coeffs
                    )
                    
                    x_norm, y_norm = undistorted_points[0][0]
                    normalized_coords = np.array([x_norm, y_norm, 1.0], dtype=np.float32)
                    
                    # 計算深度係數
                    R3 = self.rotation_matrix[2, :]
                    denominator = np.dot(R3, normalized_coords)
                    
                    if abs(denominator) < 1e-6:
                        continue
                    
                    depth_scale = (0 - self.tvec[2, 0]) / denominator
                    camera_point = depth_scale * normalized_coords
                    
                    # 轉換到世界座標系
                    tvec_3d = self.tvec.reshape(3)
                    translated_point = camera_point - tvec_3d
                    world_point_3d = np.dot(self.rotation_matrix.T, translated_point)
                    
                    world_x = float(world_point_3d[0])
                    world_y = float(world_point_3d[1])
                    
                    world_coords.append((world_x, world_y))
                    
                    # 清理臨時數組
                    del pixel_point, undistorted_points, normalized_coords
                    del camera_point, translated_point, world_point_3d
            
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("after_coordinate_transform")
            
            return world_coords
            
        except Exception as e:
            print(f"座標轉換失敗: {e}")
            return []
    
    def is_valid(self) -> bool:
        """檢查轉換器是否有效"""
        return (self.is_valid_flag and 
                self.camera_matrix is not None and 
                self.dist_coeffs is not None and 
                self.rvec is not None and 
                self.tvec is not None and 
                self.rotation_matrix is not None)
    
    def cleanup(self):
        """清理轉換器資源"""
        arrays_to_clear = ['camera_matrix', 'dist_coeffs', 'rvec', 'tvec', 'rotation_matrix']
        for attr in arrays_to_clear:
            if hasattr(self, attr) and getattr(self, attr) is not None:
                delattr(self, attr)
        
        collected = gc.collect()
        print(f"座標轉換器清理完成，回收: {collected}個物件")


# ==================== 標定管理器 (記憶體優化版本) ====================
class CalibrationManager:
    """標定檔案管理器 (記憶體優化)"""
    
    def __init__(self, config: CCD1VisionConfig):
        self.config = config
        self.transformer = CameraCoordinateTransformer()
        self.calibration_loaded = False
        self.memory_monitor = MemoryMonitor()
        
        # 自動載入標定數據
        self.auto_load_calibration()
    
    def auto_load_calibration(self):
        """自動載入標定數據 (記憶體優化)"""
        try:
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("before_auto_load")
            
            intrinsic_file = ""
            extrinsic_file = ""
            dist_coeffs_file = ""
            
            if self.config.intrinsic_file and self.config.extrinsic_file:
                intrinsic_file = os.path.join(self.config.calibration_dir, self.config.intrinsic_file)
                extrinsic_file = os.path.join(self.config.calibration_dir, self.config.extrinsic_file)
                if self.config.dist_coeffs_file:
                    dist_coeffs_file = os.path.join(self.config.calibration_dir, self.config.dist_coeffs_file)
            else:
                scan_result = self.scan_calibration_files()
                if scan_result['success']:
                    if scan_result.get('camera_matrix_files'):
                        intrinsic_file = os.path.join(self.config.calibration_dir, 
                                                    scan_result['camera_matrix_files'][0])
                    if scan_result.get('extrinsic_files'):
                        extrinsic_file = os.path.join(self.config.calibration_dir,
                                                    scan_result['extrinsic_files'][0])
                    if scan_result.get('dist_coeffs_files'):
                        dist_coeffs_file = os.path.join(self.config.calibration_dir,
                                                      scan_result['dist_coeffs_files'][0])
            
            if intrinsic_file and extrinsic_file:
                if self.transformer.load_calibration_data(intrinsic_file, extrinsic_file, dist_coeffs_file):
                    self.calibration_loaded = True
                    print("自動載入標定數據成功")
                else:
                    print("自動載入標定數據失敗")
            else:
                print("未找到標定檔案，跳過世界座標轉換")
            
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("after_auto_load")
                
        except Exception as e:
            print(f"自動載入標定數據異常: {e}")
    
    def scan_calibration_files(self) -> Dict[str, Any]:
        """掃描標定檔案 (記憶體優化)"""
        try:
            if not os.path.exists(self.config.calibration_dir):
                return {'success': False, 'error': f"標定目錄不存在: {self.config.calibration_dir}"}
            
            all_files = os.listdir(self.config.calibration_dir)
            npy_files = [f for f in all_files if f.endswith('.npy')]
            
            if not npy_files:
                return {'success': False, 'error': "未發現NPY標定檔案"}
            
            camera_matrix_files = []
            extrinsic_files = []
            dist_coeffs_files = []
            
            for file in npy_files:
                file_lower = file.lower()
                if any(keyword in file_lower for keyword in ['camera_matrix', 'intrinsic', 'calib']):
                    camera_matrix_files.append(file)
                elif any(keyword in file_lower for keyword in ['extrinsic', '外参', 'external']):
                    extrinsic_files.append(file)
                elif any(keyword in file_lower for keyword in ['dist_coeffs', 'dist_coffs', 'distortion', 'coeffs']):
                    dist_coeffs_files.append(file)
            
            return {
                'success': True,
                'camera_matrix_files': camera_matrix_files,
                'extrinsic_files': extrinsic_files,
                'dist_coeffs_files': dist_coeffs_files
            }
            
        except Exception as e:
            return {'success': False, 'error': str(e)}
    
    def cleanup(self):
        """清理標定管理器資源"""
        if self.transformer:
            self.transformer.cleanup()
        collected = gc.collect()
        print(f"標定管理器清理完成，回收: {collected}個物件")


# ==================== 圖像保存管理器 (記憶體優化版本) ====================
class ImageSaveManager:
    """圖像保存管理器 (記憶體優化)"""
    
    def __init__(self, config: CCD1VisionConfig):
        self.config = config
        self.memory_monitor = MemoryMonitor()
        self.ensure_save_dir()
    
    def ensure_save_dir(self):
        """確保保存目錄存在"""
        if not os.path.exists(self.config.save_dir):
            os.makedirs(self.config.save_dir)
    
    def save_raw_image(self, image: np.ndarray, timestamp: str) -> str:
        """保存原始圖像 (記憶體優化)"""
        if not self.config.save_raw_image:
            return ""
        
        try:
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("before_save_raw")
            
            filename = f"raw_{timestamp}.jpg"
            filepath = os.path.join(self.config.save_dir, filename)
            
            # 使用較低的JPEG品質以節省記憶體和存儲空間
            cv2.imwrite(filepath, image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("after_save_raw")
            
            return filepath
            
        except Exception as e:
            print(f"保存原始圖像失敗: {e}")
            return ""
    
    def save_result_image(self, image: np.ndarray, result: DetectionResult, timestamp: str) -> str:
        """保存檢測結果圖像 (記憶體優化)"""
        if not self.config.save_result_image:
            return ""
        
        try:
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("before_create_vis")
            
            # 創建可視化圖像
            with auto_cleanup() as _:
                vis_image = self.create_visualization(image.copy(), result)
                
                # 記憶體檢查點
                self.memory_monitor.create_checkpoint("after_create_vis")
                
                filename = f"result_{timestamp}.jpg"
                filepath = os.path.join(self.config.save_dir, filename)
                
                # 使用較低的JPEG品質
                cv2.imwrite(filepath, vis_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
                
                # 立即清理可視化圖像
                del vis_image
            
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("after_save_result")
            
            return filepath
            
        except Exception as e:
            print(f"保存結果圖像失敗: {e}")
            return ""
    
    def create_visualization(self, image: np.ndarray, result: DetectionResult) -> np.ndarray:
        """創建檢測結果可視化 (記憶體優化)"""
        try:
            # 繪製檢測結果
            for class_id, coords in result.coordinates_by_class.items():
                if class_id not in self.config.class_names:
                    continue
                
                class_name = self.config.class_names[class_id]
                color = self.config.class_colors.get(class_id, (255, 255, 255))
                
                for i, (x, y) in enumerate(coords):
                    # 繪製檢測點
                    cv2.circle(image, (int(x), int(y)), 15, color, -1)
                    cv2.circle(image, (int(x), int(y)), 20, (255, 255, 255), 3)
                    
                    # 繪製標籤
                    label = f"{class_name} {i+1}"
                    confidence = result.confidences_by_class.get(class_id, [0.0])[i] if i < len(result.confidences_by_class.get(class_id, [])) else 0.0
                    label += f" ({confidence:.2f})"
                    
                    cv2.putText(image, label, (int(x-50), int(y-25)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # 繪製統計資訊
            stats_text = f"Model{result.model_id_used}: Total={result.total_detections}"
            cv2.putText(image, stats_text, (20, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            
            # 繪製各類別數量
            y_offset = 80
            for class_id, count in result.detections_by_class.items():
                class_name = self.config.class_names.get(class_id, f"Class{class_id}")
                count_text = f"{class_name}: {count}"
                cv2.putText(image, count_text, (20, y_offset), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                y_offset += 30
            
            # 繪製保護區域
            if self.config.protection_zones:
                for zone in self.config.protection_zones:
                    points = np.array(zone['points'], np.int32)
                    cv2.polylines(image, [points], True, (0, 255, 255), 2)
                    cv2.putText(image, zone['name'], tuple(points[0]), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    # 立即清理points數組
                    del points
            
            # 繪製記憶體信息 (如果啟用記憶體監控)
            if self.config.enable_memory_monitoring and result.memory_usage:
                memory_text = f"Memory: {result.memory_usage.get('current', 0):.1f}MB"
                cv2.putText(image, memory_text, (20, image.shape[0] - 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            
            return image
            
        except Exception as e:
            print(f"創建可視化失敗: {e}")
            return image
    
    def cleanup_old_images(self):
        """清理舊圖像 (記憶體優化)"""
        try:
            if self.config.auto_cleanup_days <= 0:
                return
            
            current_time = time.time()
            cutoff_time = current_time - (self.config.auto_cleanup_days * 24 * 3600)
            deleted_count = 0
            
            for filename in os.listdir(self.config.save_dir):
                filepath = os.path.join(self.config.save_dir, filename)
                if os.path.isfile(filepath):
                    file_time = os.path.getctime(filepath)
                    if file_time < cutoff_time:
                        os.remove(filepath)
                        deleted_count += 1
            
            if deleted_count > 0:
                print(f"清理了 {deleted_count} 個舊圖像檔案")
                        
        except Exception as e:
            print(f"清理舊圖像失敗: {e}")
    
    def cleanup(self):
        """清理圖像保存管理器資源"""
        collected = gc.collect()
        print(f"圖像保存管理器清理完成，回收: {collected}個物件")


# ==================== 主控制器 (記憶體優化版本) ====================
class CCD1VisionSystem:
    """CCD1無頭視覺檢測系統主控制器 (記憶體優化版本)"""
    
    def __init__(self, config: CCD1VisionConfig):
        self.config = config
        self.logger = logging.getLogger("CCD1Vision")
        self.logger.setLevel(logging.INFO)
        
        # 記憶體監控
        self.memory_monitor = MemoryMonitor()
        self.resource_tracker = ResourceTracker()
        
        # 核心組件
        self.camera_manager: Optional[OptimizedCameraManager] = None
        self.yolo_detector: Optional[YOLODetector] = None
        self.calibration_manager: Optional[CalibrationManager] = None
        self.image_save_manager: Optional[ImageSaveManager] = None
        
        # 狀態管理
        self.initialized = False
        self.camera_connected = False
        self.last_result: Optional[DetectionResult] = None
        
        # 錯誤重試
        self.retry_count = 0
        
        # 記憶體管理
        self.operation_count = 0
        self.last_cleanup_count = 0
        
        # 初始化組件
        self._initialize_components()
    
    def _initialize_components(self):
        """初始化各組件 (記憶體優化)"""
        try:
            print("初始化CCD1視覺檢測系統...")
            
            # 記憶體檢查點 - 初始化開始
            self.memory_monitor.create_checkpoint("init_start")
            
            # 1. 初始化YOLO檢測器
            if not YOLO_AVAILABLE:
                raise RuntimeError("YOLOv11模組不可用")
                
            self.yolo_detector = YOLODetector(self.config)
            self.resource_tracker.track(self.yolo_detector)
            
            if not self.yolo_detector.model_manager.get_available_models():
                raise RuntimeError("未發現任何YOLO模型檔案")
            
            self.memory_monitor.create_checkpoint("after_yolo_init")
            print("YOLO檢測器初始化成功")
            
            # 2. 初始化標定管理器
            if self.config.enable_world_coord:
                self.calibration_manager = CalibrationManager(self.config)
                self.resource_tracker.track(self.calibration_manager)
                self.memory_monitor.create_checkpoint("after_calibration_init")
                print("標定管理器初始化成功")
            
            # 3. 初始化圖像保存管理器
            self.image_save_manager = ImageSaveManager(self.config)
            self.resource_tracker.track(self.image_save_manager)
            self.memory_monitor.create_checkpoint("after_image_save_init")
            print("圖像保存管理器初始化成功")
            
            # 4. 初始化相機
            if CAMERA_MANAGER_AVAILABLE:
                if self.initialize_camera():
                    self.memory_monitor.create_checkpoint("after_camera_init")
                    print("相機自動初始化成功")
                else:
                    print("相機自動初始化失敗，需手動初始化")
            else:
                print("相機管理器不可用")
            
            self.initialized = True
            self.memory_monitor.create_checkpoint("init_complete")
            print("CCD1視覺檢測系統初始化完成")
            
            # 輸出初始化後的記憶體報告
            if self.config.enable_memory_monitoring:
                memory_report = self.memory_monitor.get_memory_report()
                print(f"初始化記憶體使用: {memory_report['current']:.2f}MB "
                      f"(增長: {memory_report['total_change']:.2f}MB)")
            
        except Exception as e:
            print(f"系統初始化失敗: {e}")
            self.initialized = False
            raise
    
    def initialize_camera(self, ip_address: str = None) -> bool:
        """初始化相機連接 (記憶體優化)"""
        try:
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("before_camera_init")
            
            if ip_address:
                self.config.camera_ip = ip_address
            
            # 安全關閉現有相機
            if self.camera_manager:
                try:
                    self.camera_manager.shutdown()
                except:
                    pass
                finally:
                    del self.camera_manager
                    self.camera_manager = None
                    collected = gc.collect()
                    print(f"舊相機管理器清理，回收: {collected}個物件")
            
            # 獲取像素格式枚舉
            pixel_format_map = {
                "BAYER_GR8": PixelFormat.BAYER_GR8,
                "BAYER_RG8": PixelFormat.BAYER_RG8,
                "BAYER_GB8": PixelFormat.BAYER_GB8,
                "BAYER_BG8": PixelFormat.BAYER_BG8,
                "MONO8": PixelFormat.MONO8,
                "RGB8": PixelFormat.RGB8,
            }
            
            trigger_mode_map = {
                "SOFTWARE_TRIGGER": CameraMode.SOFTWARE_TRIGGER,
                "CONTINUOUS": CameraMode.CONTINUOUS,
            }
            
            # 創建相機配置
            camera_config = CameraConfig(
                name="ccd1_camera",
                ip=self.config.camera_ip,
                exposure_time=self.config.exposure_time,
                gain=self.config.gain,
                frame_rate=self.config.frame_rate,
                pixel_format=pixel_format_map.get(self.config.pixel_format, PixelFormat.BAYER_GR8),
                width=self.config.width,
                height=self.config.height,
                trigger_mode=trigger_mode_map.get(self.config.trigger_mode, CameraMode.SOFTWARE_TRIGGER),
                auto_reconnect=True,
                bandwidth_limit_mbps=self.config.bandwidth_limit_mbps,
                use_latest_frame_only=True,
                buffer_count=1
            )
            
            print(f"初始化相機: {self.config.camera_ip}")
            self.camera_manager = OptimizedCameraManager()
            self.resource_tracker.track(self.camera_manager)
            
            # 添加並連接相機
            if not self.camera_manager.add_camera("ccd1_camera", camera_config):
                raise Exception("添加相機失敗")
            
            if not self.camera_manager.connect_camera("ccd1_camera"):
                raise Exception("相機連接失敗")
            
            # 啟動串流
            stream_result = self.camera_manager.start_streaming(["ccd1_camera"])
            if not stream_result.get("ccd1_camera", False):
                raise Exception("開始串流失敗")
            
            time.sleep(1.0)  # 等待相機穩定
            
            self.camera_connected = True
            
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("after_camera_init")
            
            print(f"相機初始化成功: {self.config.camera_ip}")
            return True
            
        except Exception as e:
            self.camera_connected = False
            print(f"相機初始化失敗: {e}")
            
            # 自動重試邏輯
            if (self.config.auto_retry_on_error and 
                self.retry_count < self.config.max_retry_count):
                self.retry_count += 1
                print(f"第{self.retry_count}次重試初始化相機...")
                time.sleep(2.0)
                return self.initialize_camera(ip_address)
            
            return False
    
    def capture_image(self) -> Tuple[Optional[np.ndarray], float]:
        """軟體觸發拍照 (記憶體優化)"""
        if not self.camera_manager or not self.camera_connected:
            print("相機未連接")
            return None, 0.0
        
        capture_start = time.time()
        
        try:
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("before_capture")
            
            # 觸發拍照
            trigger_result = self.camera_manager.trigger_software(["ccd1_camera"])
            
            if not trigger_result.get("ccd1_camera", False):
                raise Exception("軟體觸發失敗")
            
            # 獲取圖像
            frame_data = self.camera_manager.capture_new_frame("ccd1_camera", timeout=2000)
            
            if frame_data is None:
                raise Exception("觸發後無法獲取圖像")
            
            capture_time = time.time() - capture_start
            
            # 使用上下文管理器處理圖像數據
            with auto_cleanup(frame_data) as _:
                # 複製圖像數據
                image_array = np.copy(frame_data.data)
                
                # 清理frame_data引用
                if hasattr(frame_data, 'data'):
                    delattr(frame_data, 'data')
                
                # 格式轉換
                if len(image_array.shape) == 2:
                    display_image = cv2.cvtColor(image_array, cv2.COLOR_GRAY2BGR)
                    del image_array  # 立即清理原始數據
                else:
                    display_image = image_array
            
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("after_capture")
            
            return display_image, capture_time
            
        except Exception as e:
            capture_time = time.time() - capture_start
            print(f"拍照失敗: {e}")
            
            # 相機錯誤重新初始化
            if self.config.auto_retry_on_error:
                print("嘗試重新初始化相機...")
                if self.initialize_camera():
                    print("相機重新初始化成功")
                
            return None, capture_time
    
    def detect_objects(self, image: Optional[np.ndarray] = None) -> DetectionResult:
        """執行物件檢測 (記憶體優化)"""
        total_start = time.time()
        result = DetectionResult()
        
        try:
            # 記憶體檢查點 - 檢測開始
            self.memory_monitor.create_checkpoint("detection_start")
            
            # 如果沒提供圖像，自動拍照
            if image is None:
                image, capture_time = self.capture_image()
                result.capture_time = capture_time * 1000
                
                if image is None:
                    result.error_message = "圖像捕獲失敗"
                    result.total_time = (time.time() - total_start) * 1000
                    return result
            
            # 檢查YOLO檢測器
            if not self.yolo_detector or not self.yolo_detector.model_manager.is_model_loaded():
                result.error_message = "YOLO檢測器未載入"
                result.total_time = (time.time() - total_start) * 1000
                return result
            
            # 記憶體檢查點 - 檢測準備完成
            self.memory_monitor.create_checkpoint("detection_ready")
            
            # 檢查保護區域
            if self.config.protection_zones:
                self._check_protection_zones(image, result)
            
            # 執行YOLO檢測
            yolo_result = self.yolo_detector.detect(image)
            
            # 記憶體檢查點 - YOLO檢測完成
            self.memory_monitor.create_checkpoint("yolo_complete")
            
            if yolo_result.success:
                # 複製檢測結果
                result.success = True
                result.total_detections = yolo_result.total_detections
                result.detections_by_class = yolo_result.detections_by_class.copy()
                result.coordinates_by_class = yolo_result.coordinates_by_class.copy()
                result.confidences_by_class = yolo_result.confidences_by_class.copy()
                result.processing_time = yolo_result.processing_time
                result.model_id_used = yolo_result.model_id_used
                result.confidence_threshold = yolo_result.confidence_threshold
                
                # 世界座標轉換
                if (self.config.enable_world_coord and 
                    self.calibration_manager and 
                    self.calibration_manager.transformer.is_valid()):
                    self._convert_to_world_coordinates(result)
                    
                # 記憶體檢查點 - 座標轉換完成
                self.memory_monitor.create_checkpoint("coord_transform_complete")
                
                # 保存圖像 (如果需要)
                if self.image_save_manager:
                    result.raw_image_path = self.image_save_manager.save_raw_image(image, result.timestamp)
                    result.result_image_path = self.image_save_manager.save_result_image(image, result, result.timestamp)
                
                # 記憶體檢查點 - 圖像保存完成
                self.memory_monitor.create_checkpoint("image_save_complete")
                
            else:
                result.error_message = yolo_result.error_message or "檢測失敗"
                print(f"檢測失敗: {result.error_message}")
            
            # 清理YOLO檢測結果
            yolo_result.cleanup()
            del yolo_result
            
            # 更新統計
            self.last_result = result
            self.operation_count += 1
            
            # 記憶體管理策略
            self._manage_memory()
            
            # 定期清理舊圖像
            if self.image_save_manager and self.operation_count % 100 == 0:
                self.image_save_manager.cleanup_old_images()
            
        except Exception as e:
            result.error_message = f"檢測異常: {str(e)}"
            print(f"檢測異常: {e}")
            
            # 異常情況下強制清理
            self._emergency_cleanup()
        
        finally:
            # 清理輸入圖像 (如果是我們創建的)
            if image is not None:
                del image
        
        result.total_time = (time.time() - total_start) * 1000
        
        # 記錄記憶體使用情況
        if self.config.enable_memory_monitoring:
            result.memory_usage = self.memory_monitor.get_memory_report()
        
        # 記憶體檢查點 - 檢測完成
        self.memory_monitor.create_checkpoint("detection_complete")
        
        return result
    
    def _manage_memory(self):
        """智能記憶體管理"""
        try:
            # 檢查記憶體增長
            memory_report = self.memory_monitor.get_memory_report()
            current_growth = memory_report['total_change']
            
            # 根據記憶體增長採取不同清理策略
            if current_growth > self.config.memory_warning_threshold:
                print(f"記憶體警告: 增長{current_growth:.2f}MB")
                self._aggressive_cleanup()
            elif self.operation_count % self.config.force_gc_frequency == 0:
                self._routine_cleanup()
            elif self.operation_count % self.config.memory_cleanup_frequency == 0:
                self._light_cleanup()
                
        except Exception as e:
            print(f"記憶體管理失敗: {e}")
    
    def _light_cleanup(self):
        """輕量級清理"""
        collected = gc.collect()
        if collected > 0:
            print(f"輕量清理: 回收{collected}個物件")
    
    def _routine_cleanup(self):
        """常規清理"""
        # 清理檢測器快取
        if self.yolo_detector:
            self.yolo_detector._clear_image_cache()
        
        # 垃圾回收
        collected = gc.collect()
        if collected > 0:
            print(f"常規清理: 回收{collected}個物件")
        
        # 清理歷史檢測結果
        if self.last_result:
            self.last_result.cleanup()
    
    def _aggressive_cleanup(self):
        """激進清理"""
        print("執行激進記憶體清理...")
        
        # 資源追蹤器強制清理
        active, collected_resources = self.resource_tracker.force_cleanup()
        
        # YOLO檢測器深度清理
        if self.yolo_detector:
            self.yolo_detector._clear_image_cache()
            if hasattr(self.yolo_detector, 'model_manager'):
                # 強制清理模型管理器的記憶體監控器
                self.yolo_detector.model_manager.resource_tracker.force_cleanup()
        
        # 標定管理器清理
        if self.calibration_manager and hasattr(self.calibration_manager, 'transformer'):
            # 重新載入標定數據以清理記憶體
            if self.calibration_manager.calibration_loaded:
                self.calibration_manager.transformer.cleanup()
        
        # 多次垃圾回收
        total_collected = 0
        for _ in range(3):
            collected = gc.collect()
            total_collected += collected
            if collected == 0:
                break
        
        print(f"激進清理: 資源{active}個, 垃圾回收{total_collected}個物件")
        
        # 重置記憶體基準
        self.memory_monitor.reset_baseline()
        print("記憶體基準已重置")
    
    def _emergency_cleanup(self):
        """緊急清理"""
        print("執行緊急記憶體清理...")
        
        try:
            # 清理所有組件
            if self.yolo_detector:
                self.yolo_detector.cleanup()
            
            if self.calibration_manager:
                self.calibration_manager.cleanup()
            
            if self.image_save_manager:
                self.image_save_manager.cleanup()
            
            # 強制垃圾回收
            for _ in range(5):
                collected = gc.collect()
                if collected == 0:
                    break
            
            print("緊急清理完成")
            
        except Exception as e:
            print(f"緊急清理失敗: {e}")
    
    def _convert_to_world_coordinates(self, result: DetectionResult):
        """轉換到世界座標 (記憶體優化)"""
        try:
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("before_world_coord")
            
            for class_id, coords in result.coordinates_by_class.items():
                if coords:
                    world_coords = self.calibration_manager.transformer.pixel_to_world(coords)
                    if world_coords:
                        result.world_coordinates_by_class[class_id] = world_coords
                        result.world_coord_valid = True
                        
                        print(f"{self.config.class_names.get(class_id, f'Class{class_id}')}世界座標:")
                        for i, ((px, py), (wx, wy)) in enumerate(zip(coords, world_coords)):
                            print(f"   {i+1}: 像素({px:.1f}, {py:.1f}) → 世界({wx:.2f}, {wy:.2f}) mm")
            
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("after_world_coord")
                            
        except Exception as e:
            print(f"世界座標轉換失敗: {e}")
    
    def _check_protection_zones(self, image: np.ndarray, result: DetectionResult):
        """檢查保護區域"""
        # 這裡可以實現保護區域檢查邏輯
        pass
    
    def switch_yolo_model(self, model_id: int) -> bool:
        """切換YOLO模型"""
        if self.yolo_detector:
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint(f"before_model_switch_{model_id}")
            
            result = self.yolo_detector.switch_model(model_id)
            
            # 模型切換後強制清理
            if result:
                self._routine_cleanup()
                self.memory_monitor.create_checkpoint(f"after_model_switch_{model_id}")
            
            return result
        return False
    
    def update_confidence_threshold(self, threshold: float):
        """更新置信度閾值"""
        if self.yolo_detector:
            self.yolo_detector.update_confidence_threshold(threshold)
    
    def update_image_save_settings(self, save_raw: bool = None, save_result: bool = None):
        """動態更新圖像保存設置"""
        if save_raw is not None:
            self.config.save_raw_image = save_raw
            print(f"原始圖像保存: {'開啟' if save_raw else '關閉'}")
        
        if save_result is not None:
            self.config.save_result_image = save_result  
            print(f"結果圖像保存: {'開啟' if save_result else '關閉'}")
    
    def get_image_save_status(self) -> Dict[str, bool]:
        """獲取當前圖像保存狀態"""
        return {
            'save_raw_image': self.config.save_raw_image,
            'save_result_image': self.config.save_result_image
        }
    
    def get_memory_stats(self) -> Dict[str, Any]:
        """獲取詳細記憶體統計"""
        stats = {
            'system': self.memory_monitor.get_memory_report(),
            'resource_tracker': self.resource_tracker.get_stats(),
            'operation_count': self.operation_count
        }
        
        if self.yolo_detector:
            stats['yolo_detector'] = self.yolo_detector.get_memory_stats()
        
        if self.calibration_manager:
            stats['calibration'] = self.calibration_manager.memory_monitor.get_memory_report()
        
        if self.image_save_manager:
            stats['image_save'] = self.image_save_manager.memory_monitor.get_memory_report()
        
        return stats
    
    def get_status(self) -> Dict[str, Any]:
        """獲取系統狀態"""
        yolo_models = []
        current_model_id = 0
        
        if self.yolo_detector:
            yolo_models = self.yolo_detector.model_manager.get_available_models()
            current_model_id = self.yolo_detector.model_manager.current_model_id
        
        status = {
            'initialized': self.initialized,
            'camera_connected': self.camera_connected,
            'camera_ip': self.config.camera_ip,
            'yolo_model_loaded': self.yolo_detector and self.yolo_detector.model_manager.is_model_loaded(),
            'current_model_id': current_model_id,
            'available_models': yolo_models,
            'confidence_threshold': self.yolo_detector.confidence_threshold if self.yolo_detector else 0.8,
            'calibration_loaded': (self.calibration_manager and 
                                 self.calibration_manager.calibration_loaded) if self.calibration_manager else False,
            'world_coord_enabled': self.config.enable_world_coord,
            'num_classes': self.config.num_classes,
            'class_names': self.config.class_names,
            'operation_count': self.operation_count,
            'last_result_summary': {
                'success': self.last_result.success if self.last_result else False,
                'total_detections': self.last_result.total_detections if self.last_result else 0,
                'detections_by_class': self.last_result.detections_by_class if self.last_result else {},
                'timestamp': self.last_result.timestamp if self.last_result else ""
            } if self.last_result else None,
            'image_save_settings': {
                'save_raw_image': self.config.save_raw_image,
                'save_result_image': self.config.save_result_image,
                'save_dir': self.config.save_dir
            },
            'memory_monitoring': self.config.enable_memory_monitoring
        }
        
        # 添加記憶體統計
        if self.config.enable_memory_monitoring:
            status['memory_stats'] = self.get_memory_stats()
        
        return status
    
    def reset_memory_baseline(self):
        """重置記憶體基準"""
        self.memory_monitor.reset_baseline()
        if self.yolo_detector:
            self.yolo_detector.memory_monitor.reset_baseline()
        print("所有記憶體基準已重置")
    
    def force_cleanup(self):
        """強制清理所有資源"""
        self._aggressive_cleanup()
    
    def disconnect(self):
        """斷開連接並清理所有資源"""
        try:
            print("開始斷開CCD1視覺系統...")
            
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("before_disconnect")
            
            # 斷開相機
            if self.camera_manager:
                try:
                    self.camera_manager.shutdown()
                except:
                    pass
                finally:
                    del self.camera_manager
                    self.camera_manager = None
            
            # 清理所有組件
            if self.yolo_detector:
                self.yolo_detector.cleanup()
                del self.yolo_detector
                self.yolo_detector = None
            
            if self.calibration_manager:
                self.calibration_manager.cleanup()
                del self.calibration_manager
                self.calibration_manager = None
            
            if self.image_save_manager:
                self.image_save_manager.cleanup()
                del self.image_save_manager
                self.image_save_manager = None
            
            # 清理檢測結果
            if self.last_result:
                self.last_result.cleanup()
                del self.last_result
                self.last_result = None
            
            self.camera_connected = False
            self.initialized = False
            
            # 最終清理
            self.resource_tracker.force_cleanup()
            
            # 多次垃圾回收
            total_collected = 0
            for _ in range(5):
                collected = gc.collect()
                total_collected += collected
                if collected == 0:
                    break
            
            # 記憶體檢查點
            self.memory_monitor.create_checkpoint("after_disconnect")
            
            # 輸出最終記憶體報告
            final_report = self.memory_monitor.get_memory_report()
            
            print("CCD1視覺系統已斷開連接")
            print(f"最終記憶體使用: {final_report['current']:.2f}MB")
            print(f"總記憶體變化: {final_report['total_change']:.2f}MB")
            print(f"總共回收: {total_collected}個物件")
            
        except Exception as e:
            print(f"斷開連接失敗: {e}")


# ==================== 記憶體測試專用函數 ====================
def create_memory_test_system(enable_image_save=False) -> CCD1VisionSystem:
    """創建記憶體測試專用的視覺系統"""
    config = CCD1VisionConfig(
        # 相機配置
        camera_ip="192.168.1.8",
        exposure_time=20000.0,
        gain=200.0,
        
        # 圖像保存配置 - 測試時通常關閉
        save_raw_image=enable_image_save,
        save_result_image=enable_image_save,
        save_dir="./test_images",
        
        # 記憶體優化配置
        enable_memory_monitoring=True,
        memory_warning_threshold=30.0,  # 30MB警告
        force_gc_frequency=5,  # 每5次強制GC
        max_cached_images=2,  # 最大快取2張圖像
        memory_cleanup_frequency=10,  # 每10次檢測清理
        
        # 標定配置 - 測試時可能關閉
        enable_world_coord=False,
        
        # 其他配置
        auto_retry_on_error=False  # 測試時關閉重試
    )
    
    return CCD1VisionSystem(config)


# ==================== 使用範例 ====================
def example_usage():
    """使用範例 (記憶體優化版本)"""
    
    # 創建配置
    config = CCD1VisionConfig(
        # 相機配置
        camera_ip="192.168.1.8",
        exposure_time=20000.0,
        gain=200.0,
        
        # YOLO配置
        confidence_threshold=0.8,
        num_classes=3,
        class_names={0: "DR_F", 1: "DR_B", 2: "STACK"},
        class_colors={
            0: (255, 0, 0),    
            1: (0, 255, 0),     
            2: (0, 0, 255),    
        },
        
        # 標定配置
        calibration_dir="./calibration",
        intrinsic_file="camera_matrix_DR.npy",
        extrinsic_file="extrinsic_DR.npy",
        
        # 圖像保存配置
        save_raw_image=True,
        save_result_image=True,
        save_dir="./detection_images",
        
        # 記憶體優化配置
        enable_memory_monitoring=True,
        memory_warning_threshold=50.0,
        force_gc_frequency=5,
        max_cached_images=3,
        
        # 保護區域配置
        protection_zones=[
            {
                "name": "Zone1", 
                "points": [(100, 100), (500, 100), (500, 400), (100, 400)]
            }
        ]
    )
    
    try:
        # 初始化系統
        vision_system = CCD1VisionSystem(config)
        
        # 檢查狀態
        status = vision_system.get_status()
        print(f"系統狀態: {status}")
        
        # 輸出初始記憶體統計
        if config.enable_memory_monitoring:
            memory_stats = vision_system.get_memory_stats()
            print(f"初始記憶體統計: {memory_stats['system']}")
        
        # 執行多次檢測來測試記憶體穩定性
        for i in range(10):
            print(f"\n=== 第{i+1}次檢測 ===")
            
            result = vision_system.detect_objects()
            
            if result.success:
                print(f"檢測成功! 總檢測數: {result.total_detections}")
                print(f"各類別數量: {result.detections_by_class}")
                print(f"處理時間: {result.processing_time:.2f}ms")
                
                # 輸出記憶體使用情況
                if config.enable_memory_monitoring and result.memory_usage:
                    print(f"記憶體使用: {result.memory_usage['current']:.2f}MB "
                          f"(變化: {result.memory_usage['total_change']:+.2f}MB)")
                
            else:
                print(f"檢測失敗: {result.error_message}")
            
            # 每5次檢測輸出詳細記憶體統計
            if (i + 1) % 5 == 0:
                memory_stats = vision_system.get_memory_stats()
                print(f"記憶體統計: {memory_stats['system']['current']:.2f}MB "
                      f"(增長: {memory_stats['system']['total_change']:+.2f}MB)")
        
        # 切換模型測試
        print(f"\n=== 模型切換測試 ===")
        if vision_system.switch_yolo_model(2):
            print("模型切換成功")
            
            # 切換後的記憶體統計
            memory_stats = vision_system.get_memory_stats()
            print(f"模型切換後記憶體: {memory_stats['system']['current']:.2f}MB")
            
        # 更新置信度測試
        vision_system.update_confidence_threshold(0.9)
        
        # 最終記憶體統計
        final_memory_stats = vision_system.get_memory_stats()
        print(f"\n=== 最終記憶體統計 ===")
        print(f"系統記憶體: {final_memory_stats['system']}")
        print(f"資源追蹤: {final_memory_stats['resource_tracker']}")
        
        # 斷開連接
        vision_system.disconnect()
        
    except Exception as e:
        print(f"範例執行失敗: {e}")
        import traceback
        print(f"詳細錯誤: {traceback.format_exc()}")


def memory_stress_test():
    """記憶體壓力測試"""
    print("開始記憶體壓力測試...")
    
    # 創建測試系統
    vision_system = create_memory_test_system(enable_image_save=False)
    
    try:
        baseline_memory = vision_system.memory_monitor.get_memory_report()['current']
        print(f"基準記憶體: {baseline_memory:.2f}MB")
        
        # 執行1000次檢測
        for i in range(1000):
            result = vision_system.detect_objects()
            
            # 每100次輸出統計
            if (i + 1) % 100 == 0:
                memory_stats = vision_system.get_memory_stats()
                current_memory = memory_stats['system']['current']
                growth = current_memory - baseline_memory
                
                print(f"第{i+1:4d}次 | 記憶體: {current_memory:7.2f}MB | "
                      f"增長: {growth:+6.2f}MB | 成功: {result.success}")
                
                # 記憶體增長超過100MB時強制清理
                if growth > 100:
                    print("記憶體增長過多，執行強制清理...")
                    vision_system.force_cleanup()
                    vision_system.reset_memory_baseline()
                    baseline_memory = vision_system.memory_monitor.get_memory_report()['current']
        
        # 最終統計
        final_stats = vision_system.get_memory_stats()
        print(f"\n壓力測試完成")
        print(f"最終記憶體: {final_stats['system']['current']:.2f}MB")
        print(f"總增長: {final_stats['system']['total_change']:+.2f}MB")
        
    finally:
        vision_system.disconnect()


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "stress":
        memory_stress_test()
    else:
        example_usage()