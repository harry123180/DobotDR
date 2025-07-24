# -*- coding: utf-8 -*-
"""
CCD1_Simple_Vision_System.py - CCD1簡化視覺檢測系統
移除複雜記憶體管理，保持核心功能
Simple is Beautiful
"""

import os
import time
import gc
from typing import Optional, Dict, Any, Tuple, List
from dataclasses import dataclass
from datetime import datetime
import numpy as np
import cv2

# 檢查YOLO可用性
YOLO_AVAILABLE = False
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

# 導入相機管理模組
try:
    import sys
    sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'API'))
    from camera_manager import OptimizedCameraManager, CameraConfig, CameraMode, PixelFormat
    CAMERA_MANAGER_AVAILABLE = True
except ImportError:
    CAMERA_MANAGER_AVAILABLE = False


# ==================== 配置結構 ====================
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
    
    # 圖像保存配置
    save_raw_image: bool = False
    save_result_image: bool = True
    save_dir: str = "./images"
    auto_cleanup_days: int = 7
    
    # 系統配置
    working_dir: str = ""
    enable_world_coord: bool = True
    
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


# ==================== 檢測結果 ====================
@dataclass
class DetectionResult:
    """檢測結果數據結構"""
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
    
    def __post_init__(self):
        if self.detections_by_class is None:
            self.detections_by_class = {}
        if self.coordinates_by_class is None:
            self.coordinates_by_class = {}
        if self.confidences_by_class is None:
            self.confidences_by_class = {}
        if self.world_coordinates_by_class is None:
            self.world_coordinates_by_class = {}
        if not self.timestamp:
            self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")


# ==================== 資源追蹤器 ====================
class ResourceTracker:
    """簡單資源追蹤器"""
    
    def __init__(self):
        self._tracked_objects = []
        
    def track(self, obj):
        """追蹤對象"""
        self._tracked_objects.append(obj)
        
    def cleanup(self):
        """清理追蹤的對象"""
        count = 0
        for obj in self._tracked_objects:
            try:
                if hasattr(obj, 'cleanup'):
                    obj.cleanup()
                count += 1
            except:
                pass
        
        self._tracked_objects.clear()
        return count


# ==================== YOLO模型管理器 ====================
class YOLOModelManager:
    """YOLO模型管理器"""
    
    def __init__(self, config: CCD1VisionConfig):
        self.config = config
        self.working_dir = config.working_dir
        self.models = {}
        self.model_paths = {}
        self.current_model_id = 0
        
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
        """載入指定模型"""
        try:
            if model_id == 0:
                # 卸載當前模型
                if self.current_model_id in self.models:
                    del self.models[self.current_model_id]
                self.current_model_id = 0
                print("卸載當前模型")
                gc.collect()
                return True
            
            if model_id not in self.model_paths:
                print(f"模型{model_id}檔案不存在")
                return False
            
            if not YOLO_AVAILABLE:
                print("YOLO模組不可用")
                return False
            
            print(f"載入模型{model_id}: {self.model_paths[model_id]}")
            
            # 卸載舊模型
            if self.current_model_id in self.models and self.current_model_id != model_id:
                print(f"卸載舊模型{self.current_model_id}")
                del self.models[self.current_model_id]
                gc.collect()
            
            # 載入新模型
            model = YOLO(self.model_paths[model_id])
            self.models[model_id] = model
            self.current_model_id = model_id
            
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
    
    def cleanup(self):
        """清理所有模型"""
        for model_id in list(self.models.keys()):
            del self.models[model_id]
        self.models.clear()
        self.current_model_id = 0
        gc.collect()


# ==================== YOLO檢測器 ====================
class YOLODetector:
    """YOLO檢測器"""
    
    def __init__(self, config: CCD1VisionConfig):
        self.config = config
        self.model_manager = YOLOModelManager(config)
        self.confidence_threshold = config.confidence_threshold
        
        # 自動載入模型1
        if 1 in self.model_manager.model_paths:
            self.model_manager.load_model(1)
    
    def switch_model(self, model_id: int) -> bool:
        """切換模型"""
        return self.model_manager.load_model(model_id)
    
    def update_confidence_threshold(self, threshold: float):
        """更新置信度閾值"""
        self.confidence_threshold = max(0.1, min(1.0, threshold))
        print(f"置信度閾值更新為: {self.confidence_threshold}")
    
    def detect(self, image: np.ndarray) -> DetectionResult:
        """執行YOLO檢測"""
        start_time = time.time()
        result = DetectionResult()
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
            
        except Exception as e:
            result.error_message = f"YOLO檢測失敗: {e}"
            print(f"YOLO檢測異常: {e}")
        
        result.processing_time = (time.time() - start_time) * 1000
        return result
    
    def cleanup(self):
        """清理檢測器資源"""
        self.model_manager.cleanup()


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
    
    def load_calibration_data(self, intrinsic_file: str, extrinsic_file: str, dist_coeffs_file: str = "") -> bool:
        """載入標定數據"""
        try:
            print(f"載入標定數據...")
            print(f"   內參檔案: {intrinsic_file}")
            print(f"   外參檔案: {extrinsic_file}")
            print(f"   畸變係數檔案: {dist_coeffs_file if dist_coeffs_file else '使用零值'}")
            
            # 載入內參
            intrinsic_data = np.load(intrinsic_file, allow_pickle=True)
            
            if hasattr(intrinsic_data, 'shape') and intrinsic_data.shape == (3, 3):
                self.camera_matrix = intrinsic_data
                self.dist_coeffs = np.zeros((1, 5), dtype=np.float32)
            elif isinstance(intrinsic_data, dict):
                self.camera_matrix = intrinsic_data['camera_matrix']
                self.dist_coeffs = intrinsic_data.get('dist_coeffs', np.zeros((1, 5), dtype=np.float32))
            elif hasattr(intrinsic_data, 'item') and callable(intrinsic_data.item):
                dict_data = intrinsic_data.item()
                if isinstance(dict_data, dict):
                    self.camera_matrix = dict_data['camera_matrix']
                    self.dist_coeffs = dict_data.get('dist_coeffs', np.zeros((1, 5), dtype=np.float32))
            else:
                self.camera_matrix = intrinsic_data
                self.dist_coeffs = np.zeros((1, 5), dtype=np.float32)
            
            # 載入單獨的畸變係數檔案
            if dist_coeffs_file and os.path.exists(dist_coeffs_file):
                try:
                    dist_data = np.load(dist_coeffs_file, allow_pickle=True)
                    if hasattr(dist_data, 'shape'):
                        self.dist_coeffs = dist_data
                        print(f"   載入畸變係數: {dist_data.shape}")
                except Exception as e:
                    print(f"   載入畸變係數失敗，使用零值: {e}")
            
            # 載入外參
            extrinsic_data = np.load(extrinsic_file, allow_pickle=True)
            
            if isinstance(extrinsic_data, dict):
                self.rvec = extrinsic_data['rvec']
                self.tvec = extrinsic_data['tvec']
            elif hasattr(extrinsic_data, 'item') and callable(extrinsic_data.item) and extrinsic_data.shape == ():
                dict_data = extrinsic_data.item()
                if isinstance(dict_data, dict):
                    self.rvec = dict_data['rvec']
                    self.tvec = dict_data['tvec']
            else:
                return False
            
            # 計算旋轉矩陣
            self.rotation_matrix, _ = cv2.Rodrigues(self.rvec)
            
            self.is_valid_flag = True
            print(f"座標轉換器載入成功")
            return True
            
        except Exception as e:
            print(f"座標轉換器載入失敗: {e}")
            return False
    
    def pixel_to_world(self, pixel_coords: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """像素座標轉世界座標"""
        if not self.is_valid_flag:
            return []
        
        try:
            world_coords = []
            
            for px, py in pixel_coords:
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


# ==================== 標定管理器 ====================
class CalibrationManager:
    """標定檔案管理器"""
    
    def __init__(self, config: CCD1VisionConfig):
        self.config = config
        self.transformer = CameraCoordinateTransformer()
        self.calibration_loaded = False
        
        # 自動載入標定數據
        self.auto_load_calibration()
    
    def auto_load_calibration(self):
        """自動載入標定數據"""
        try:
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
                
        except Exception as e:
            print(f"自動載入標定數據異常: {e}")
    
    def scan_calibration_files(self) -> Dict[str, Any]:
        """掃描標定檔案"""
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


# ==================== 圖像保存管理器 ====================
class ImageSaveManager:
    """圖像保存管理器"""
    
    def __init__(self, config: CCD1VisionConfig):
        self.config = config
        self.ensure_save_dir()
    
    def ensure_save_dir(self):
        """確保保存目錄存在"""
        if not os.path.exists(self.config.save_dir):
            os.makedirs(self.config.save_dir)
    
    def save_raw_image(self, image: np.ndarray, timestamp: str) -> str:
        """保存原始圖像"""
        if not self.config.save_raw_image:
            return ""
        
        try:
            filename = f"raw_{timestamp}.jpg"
            filepath = os.path.join(self.config.save_dir, filename)
            cv2.imwrite(filepath, image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            return filepath
        except Exception as e:
            print(f"保存原始圖像失敗: {e}")
            return ""
    
    def save_result_image(self, image: np.ndarray, result: DetectionResult, timestamp: str) -> str:
        """保存檢測結果圖像"""
        if not self.config.save_result_image:
            return ""
        
        try:
            vis_image = self.create_visualization(image.copy(), result)
            filename = f"result_{timestamp}.jpg"
            filepath = os.path.join(self.config.save_dir, filename)
            cv2.imwrite(filepath, vis_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            return filepath
        except Exception as e:
            print(f"保存結果圖像失敗: {e}")
            return ""
    
    def create_visualization(self, image: np.ndarray, result: DetectionResult) -> np.ndarray:
        """創建檢測結果可視化"""
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
            
            return image
            
        except Exception as e:
            print(f"創建可視化失敗: {e}")
            return image
    
    def cleanup_old_images(self):
        """清理舊圖像"""
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


# ==================== 主控制器 ====================
class CCD1VisionSystem:
    """CCD1簡化視覺檢測系統主控制器"""
    
    def __init__(self, config: CCD1VisionConfig):
        self.config = config
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
        
        # 初始化組件
        self._initialize_components()
    
    def _initialize_components(self):
        """初始化各組件"""
        try:
            print("初始化CCD1視覺檢測系統...")
            
            # 1. 初始化YOLO檢測器
            if not YOLO_AVAILABLE:
                raise RuntimeError("YOLO模組不可用")
                
            self.yolo_detector = YOLODetector(self.config)
            self.resource_tracker.track(self.yolo_detector)
            
            if not self.yolo_detector.model_manager.get_available_models():
                raise RuntimeError("未發現任何YOLO模型檔案")
            
            print("YOLO檢測器初始化成功")
            
            # 2. 初始化標定管理器
            if self.config.enable_world_coord:
                self.calibration_manager = CalibrationManager(self.config)
                self.resource_tracker.track(self.calibration_manager)
                print("標定管理器初始化成功")
            
            # 3. 初始化圖像保存管理器
            self.image_save_manager = ImageSaveManager(self.config)
            self.resource_tracker.track(self.image_save_manager)
            print("圖像保存管理器初始化成功")
            
            # 4. 初始化相機
            if CAMERA_MANAGER_AVAILABLE:
                if self.initialize_camera():
                    print("相機自動初始化成功")
                else:
                    print("相機自動初始化失敗，需手動初始化")
            else:
                print("相機管理器不可用")
            
            self.initialized = True
            print("CCD1視覺檢測系統初始化完成")
            
        except Exception as e:
            print(f"系統初始化失敗: {e}")
            self.initialized = False
            raise
    
    def initialize_camera(self, ip_address: str = None) -> bool:
        """初始化相機連接"""
        try:
            if ip_address:
                self.config.camera_ip = ip_address
            
            # 安全關閉現有相機
            if self.camera_manager:
                try:
                    self.camera_manager.shutdown()
                except:
                    pass
                finally:
                    self.camera_manager = None
                    gc.collect()
            
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
            print(f"相機初始化成功: {self.config.camera_ip}")
            return True
            
        except Exception as e:
            self.camera_connected = False
            print(f"相機初始化失敗: {e}")
            return False
    
    def capture_image(self) -> Tuple[Optional[np.ndarray], float]:
        """軟體觸發拍照"""
        if not self.camera_manager or not self.camera_connected:
            print("相機未連接")
            return None, 0.0
        
        capture_start = time.time()
        
        try:
            # 觸發拍照
            trigger_result = self.camera_manager.trigger_software(["ccd1_camera"])
            
            if not trigger_result.get("ccd1_camera", False):
                raise Exception("軟體觸發失敗")
            
            # 獲取圖像
            frame_data = self.camera_manager.capture_new_frame("ccd1_camera", timeout=2000)
            
            if frame_data is None:
                raise Exception("觸發後無法獲取圖像")
            
            capture_time = time.time() - capture_start
            
            # 複製圖像數據
            image_array = np.copy(frame_data.data)
            
            # 格式轉換
            if len(image_array.shape) == 2:
                display_image = cv2.cvtColor(image_array, cv2.COLOR_GRAY2BGR)
            else:
                display_image = image_array
            
            return display_image, capture_time
            
        except Exception as e:
            capture_time = time.time() - capture_start
            print(f"拍照失敗: {e}")
            return None, capture_time
    
    def detect_objects(self, image: Optional[np.ndarray] = None) -> DetectionResult:
        """執行物件檢測"""
        total_start = time.time()
        result = DetectionResult()
        
        try:
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
            
            # 執行YOLO檢測
            yolo_result = self.yolo_detector.detect(image)
            
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
                    
                # 保存圖像 (如果需要)
                if self.image_save_manager:
                    result.raw_image_path = self.image_save_manager.save_raw_image(image, result.timestamp)
                    result.result_image_path = self.image_save_manager.save_result_image(image, result, result.timestamp)
                
            else:
                result.error_message = yolo_result.error_message or "檢測失敗"
                print(f"檢測失敗: {result.error_message}")
            
            # 更新統計
            self.last_result = result
            
        except Exception as e:
            result.error_message = f"檢測異常: {str(e)}"
            print(f"檢測異常: {e}")
        
        result.total_time = (time.time() - total_start) * 1000
        return result
    
    def _convert_to_world_coordinates(self, result: DetectionResult):
        """轉換到世界座標"""
        try:
            for class_id, coords in result.coordinates_by_class.items():
                if coords:
                    world_coords = self.calibration_manager.transformer.pixel_to_world(coords)
                    if world_coords:
                        result.world_coordinates_by_class[class_id] = world_coords
                        result.world_coord_valid = True
                        
                        print(f"{self.config.class_names.get(class_id, f'Class{class_id}')}世界座標:")
                        for i, ((px, py), (wx, wy)) in enumerate(zip(coords, world_coords)):
                            print(f"   {i+1}: 像素({px:.1f}, {py:.1f}) → 世界({wx:.2f}, {wy:.2f}) mm")
                            
        except Exception as e:
            print(f"世界座標轉換失敗: {e}")
    
    def switch_yolo_model(self, model_id: int) -> bool:
        """切換YOLO模型"""
        if self.yolo_detector:
            return self.yolo_detector.switch_model(model_id)
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
            }
        }
        
        return status
    
    def disconnect(self):
        """斷開連接並清理所有資源"""
        try:
            print("開始斷開CCD1視覺系統...")
            
            # 斷開相機
            if self.camera_manager:
                try:
                    self.camera_manager.shutdown()
                except:
                    pass
                finally:
                    self.camera_manager = None
            
            # 清理所有組件
            self.resource_tracker.cleanup()
            
            # 清理檢測結果
            self.last_result = None
            
            self.camera_connected = False
            self.initialized = False
            
            # 最終清理
            gc.collect()
            
            print("CCD1視覺系統已斷開連接")
            
        except Exception as e:
            print(f"斷開連接失敗: {e}")


# ==================== 使用範例 ====================
def example_usage():
    """使用範例"""
    
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
        save_dir="./detection_images"
    )
    
    try:
        # 初始化系統
        vision_system = CCD1VisionSystem(config)
        
        # 檢查狀態
        status = vision_system.get_status()
        print(f"系統狀態: {status}")
        
        # 執行檢測
        for i in range(5):
            print(f"\n=== 第{i+1}次檢測 ===")
            
            result = vision_system.detect_objects()
            
            if result.success:
                print(f"檢測成功! 總檢測數: {result.total_detections}")
                print(f"各類別數量: {result.detections_by_class}")
                print(f"處理時間: {result.processing_time:.2f}ms")
                
            else:
                print(f"檢測失敗: {result.error_message}")
        
        # 切換模型測試
        print(f"\n=== 模型切換測試 ===")
        if vision_system.switch_yolo_model(2):
            print("模型切換成功")
            
        # 更新置信度測試
        vision_system.update_confidence_threshold(0.9)
        
        # 斷開連接
        vision_system.disconnect()
        
    except Exception as e:
        print(f"範例執行失敗: {e}")


if __name__ == "__main__":
    example_usage()