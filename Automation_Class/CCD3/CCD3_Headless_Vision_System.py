import os
import sys
import time
import gc
import threading
import json
import psutil
from typing import Dict, Any, Optional, Tuple, List, Union
from dataclasses import dataclass, asdict
from enum import Enum
import cv2
import numpy as np
import math

# 導入相機管理模組
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'API'))
from camera_manager import OptimizedCamera, CameraConfig, CameraMode, PixelFormat

# 記憶體管理工具
class MemoryManager:
    """記憶體監控和管理工具"""
    
    def __init__(self, warning_threshold_mb=1024, critical_threshold_mb=2048):
        self.warning_threshold = warning_threshold_mb * 1024 * 1024
        self.critical_threshold = critical_threshold_mb * 1024 * 1024
        self.process = psutil.Process()
        self._lock = threading.Lock()
        
    def get_memory_usage(self) -> Dict[str, Union[int, float]]:
        """獲取當前記憶體使用情況"""
        with self._lock:
            memory_info = self.process.memory_info()
            return {
                'rss_bytes': memory_info.rss,
                'rss_mb': memory_info.rss / 1024 / 1024,
                'vms_bytes': memory_info.vms,
                'vms_mb': memory_info.vms / 1024 / 1024,
                'percent': self.process.memory_percent()
            }
    
    def force_cleanup(self):
        """強制清理記憶體"""
        try:
            gc.collect()
            cv2.destroyAllWindows()
        except Exception as e:
            pass
    
    def check_memory_threshold(self) -> Tuple[bool, str]:
        """檢查記憶體使用是否超過閾值"""
        memory_info = self.get_memory_usage()
        rss_bytes = memory_info['rss_bytes']
        
        if rss_bytes > self.critical_threshold:
            return False, f"記憶體使用超過臨界值: {memory_info['rss_mb']:.1f}MB"
        elif rss_bytes > self.warning_threshold:
            return True, f"記憶體使用接近警告值: {memory_info['rss_mb']:.1f}MB"
        
        return True, f"記憶體使用正常: {memory_info['rss_mb']:.1f}MB"

# 檢測方法枚舉
class DetectionMethod(Enum):
    CASE_ELLIPSE = 0        # CASE模式：橢圓擬合 + 複雜遮罩處理
    DR_MIN_RECT = 1         # DR模式：最小外接矩形
    SIMPLE_CONTOUR = 2      # 預留：簡單輪廓中心點
    MOMENTS_CENTER = 3      # 預留：重心計算
    
    @classmethod
    def get_description(cls, method):
        descriptions = {
            cls.CASE_ELLIPSE: "橢圓擬合複雜遮罩",
            cls.DR_MIN_RECT: "最小外接矩形",
            cls.SIMPLE_CONTOUR: "簡單輪廓中心",
            cls.MOMENTS_CENTER: "重心計算"
        }
        return descriptions.get(method, "未知方法")

# 檢測參數
@dataclass
class DetectionParams:
    """檢測參數結構"""
    detection_method: DetectionMethod = DetectionMethod.DR_MIN_RECT
    min_area_rate: float = 0.05                # 最小面積比例
    sequence_mode: bool = False                 # 序列模式：False=第一個輪廓, True=最後一個輪廓
    gaussian_kernel: int = 3                    # 高斯模糊核大小
    threshold_mode: int = 0                     # 0=OTSU自動, 1=手動
    manual_threshold: int = 127                 # 手動閾值
    
    def to_dict(self) -> Dict[str, Any]:
        """轉換為字典"""
        data = asdict(self)
        data['detection_method'] = self.detection_method.value
        return data
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'DetectionParams':
        """從字典創建"""
        if 'detection_method' in data:
            data['detection_method'] = DetectionMethod(data['detection_method'])
        return cls(**data)

# 檢測結果
@dataclass
class DetectionResult:
    """檢測結果結構"""
    success: bool
    center: Optional[Tuple[int, int]] = None        # 中心點像素座標
    angle: Optional[float] = None                   # 角度
    detection_region: Optional[Dict] = None         # 檢測區塊資訊
    
    # 基於方法的額外數據
    major_axis: Optional[float] = None              # 長軸 (橢圓擬合)
    minor_axis: Optional[float] = None              # 短軸 (橢圓擬合)
    rect_width: Optional[float] = None              # 矩形寬度 (最小外接矩形)
    rect_height: Optional[float] = None             # 矩形高度 (最小外接矩形)
    contour_area: Optional[float] = None            # 輪廓面積
    
    # 性能數據
    capture_time: float = 0.0                       # 拍照時間
    processing_time: float = 0.0                    # 處理時間
    total_time: float = 0.0                         # 總時間
    
    # 錯誤資訊
    error_message: Optional[str] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """轉換為字典"""
        return asdict(self)

# 圖像儲存選項
@dataclass
class ImageSaveOptions:
    """圖像儲存選項"""
    save_original: bool = False                     # 儲存原始圖像
    save_processed: bool = False                    # 儲存處理過程圖像
    save_result: bool = False                       # 儲存結果圖像
    save_directory: str = "debug_images"            # 儲存目錄
    filename_prefix: str = "ccd3"                   # 檔名前綴
    
    def __post_init__(self):
        """確保儲存目錄存在"""
        if any([self.save_original, self.save_processed, self.save_result]):
            # 使用執行檔同層目錄
            if not os.path.isabs(self.save_directory):
                script_dir = os.path.dirname(os.path.abspath(__file__))
                self.save_directory = os.path.join(script_dir, self.save_directory)
            
            os.makedirs(self.save_directory, exist_ok=True)

# 主要檢測器類
class CCD3HeadlessDetector:
    """CCD3無頭模式角度檢測器"""
    
    def __init__(self, camera_ip: str = "192.168.1.10"):
        """
        初始化檢測器
        
        Args:
            camera_ip: 相機IP地址
        """
        # 基本配置
        self.camera_ip = camera_ip
        self.camera = None
        self.is_initialized = False
        
        # 檢測參數
        self.params = DetectionParams()
        
        # 記憶體管理
        self.memory_manager = MemoryManager()
        
        # 圖像儲存
        self.save_options = ImageSaveOptions()
        
        # 線程鎖
        self._lock = threading.Lock()
        
        # 快取優化
        self._kernel_cache = {}
        self._last_image_shape = None
        self._min_area_cache = None
        
        # 統計資訊
        self.stats = {
            'total_detections': 0,
            'successful_detections': 0,
            'failed_detections': 0,
            'reconnection_count': 0,
            'memory_cleanups': 0
        }
        
    def initialize_camera(self) -> bool:
        """
        初始化相機連接
        
        Returns:
            bool: 初始化是否成功
        """
        try:
            self._log_info(f"初始化相機，IP地址: {self.camera_ip}")
            
            # 關閉現有連接
            if self.camera:
                self._log_info("關閉現有相機連接")
                self.camera.disconnect()
                self.camera = None
            
            # 檢查記憶體狀況
            memory_ok, memory_msg = self.memory_manager.check_memory_threshold()
            if not memory_ok:
                self._log_error(f"記憶體不足，無法初始化相機: {memory_msg}")
                raise Exception(memory_msg)
            
            # 創建相機配置
            config = CameraConfig(
                name="ccd3_headless_camera",
                ip=self.camera_ip,
                exposure_time=20000.0,
                gain=200.0,
                frame_rate=5.0,  # 修正：使用5FPS符合API要求
                width=2592,
                height=1944,
                trigger_mode=CameraMode.SOFTWARE_TRIGGER,
                bandwidth_limit_mbps=200,  # 添加頻寬限制
                enable_bandwidth_control=True,
                use_latest_frame_only=True
            )
            
            # 創建簡單的logger
            import logging
            logger = logging.getLogger("CCD3Camera")
            logger.setLevel(logging.INFO)
            if not logger.handlers:
                handler = logging.StreamHandler()
                logger.addHandler(handler)
            
            # 創建相機實例
            self.camera = OptimizedCamera(config, logger)
            
            # 連接相機
            if not self.camera.connect():
                raise Exception(f"相機連接失敗: {self.camera_ip}")
            
            # 啟動串流
            if not self.camera.start_streaming():
                raise Exception("相機串流啟動失敗")
            
            # 測試軟體觸發
            if not self.camera.trigger_software():
                raise Exception("軟體觸發測試失敗")
            
            # 測試圖像捕獲
            test_frame = self.camera.capture_latest_frame(timeout=3000)
            if test_frame is None:
                raise Exception("測試圖像捕獲失敗")
            
            self.is_initialized = True
            self._log_info(f"相機初始化成功，測試圖像尺寸: {test_frame.data.shape}")
            
            return True
            
        except Exception as e:
            self.is_initialized = False
            self._log_error(f"相機初始化失敗: {e}")
            return False
    
    def reconnect_camera(self) -> bool:
        """
        重新連接相機
        
        Returns:
            bool: 重連是否成功
        """
        self._log_info("嘗試重新連接相機")
        self.stats['reconnection_count'] += 1
        
        # 強制清理記憶體
        self.cleanup_memory()
        
        # 重新初始化
        return self.initialize_camera()
    
    def set_detection_params(self, **kwargs) -> bool:
        """
        設置檢測參數
        
        Args:
            **kwargs: 檢測參數
            
        Returns:
            bool: 設置是否成功
        """
        try:
            with self._lock:
                # 更新參數
                for key, value in kwargs.items():
                    if hasattr(self.params, key):
                        setattr(self.params, key, value)
                    else:
                        self._log_warning(f"未知參數: {key}")
                
                # 清除快取
                self._min_area_cache = None
                
                self._log_info(f"檢測參數已更新: {kwargs}")
                return True
                
        except Exception as e:
            self._log_error(f"設置檢測參數失敗: {e}")
            return False
    
    def get_detection_params(self) -> Dict[str, Any]:
        """
        獲取當前檢測參數
        
        Returns:
            Dict: 當前參數字典
        """
        return self.params.to_dict()
    
    def set_image_save_options(self, **kwargs) -> bool:
        """
        設置圖像儲存選項
        
        Args:
            **kwargs: 儲存選項
            
        Returns:
            bool: 設置是否成功
        """
        try:
            for key, value in kwargs.items():
                if hasattr(self.save_options, key):
                    setattr(self.save_options, key, value)
                else:
                    self._log_warning(f"未知儲存選項: {key}")
            
            # 重新初始化儲存目錄
            self.save_options.__post_init__()
            
            self._log_info(f"圖像儲存選項已更新: {kwargs}")
            return True
            
        except Exception as e:
            self._log_error(f"設置圖像儲存選項失敗: {e}")
            return False
    
    def capture_and_detect(self, detection_method: Optional[DetectionMethod] = None) -> DetectionResult:
        """
        執行拍照和檢測
        
        Args:
            detection_method: 檢測方法，None使用當前設定
            
        Returns:
            DetectionResult: 檢測結果
        """
        start_time = time.perf_counter()
        self.stats['total_detections'] += 1
        
        # 檢查初始化狀態
        if not self.is_initialized or not self.camera:
            result = DetectionResult(
                success=False,
                error_message="相機未初始化",
                total_time=(time.perf_counter() - start_time) * 1000
            )
            self.stats['failed_detections'] += 1
            return result
        
        # 檢查記憶體狀況
        memory_ok, memory_msg = self.memory_manager.check_memory_threshold()
        if not memory_ok:
            self._log_warning(memory_msg)
            self.cleanup_memory()
        
        original_image = None
        processed_images = {}
        result_image = None
        
        try:
            # 軟體觸發拍照
            capture_start = time.perf_counter()
            
            if not self.camera.trigger_software():
                raise Exception("軟體觸發失敗")
            
            # 等待觸發後的圖像
            frame_data = self.camera.capture_latest_frame(timeout=3000)
            if frame_data is None:
                raise Exception("圖像捕獲失敗")
            
            capture_time = (time.perf_counter() - capture_start) * 1000
            original_image = frame_data.data.copy()
            
            # 執行檢測
            process_start = time.perf_counter()
            
            method = detection_method or self.params.detection_method
            result = self._detect_angle(original_image, method)
            
            processing_time = (time.perf_counter() - process_start) * 1000
            total_time = (time.perf_counter() - start_time) * 1000
            
            # 更新結果時間
            result.capture_time = capture_time
            result.processing_time = processing_time
            result.total_time = total_time
            
            # 創建結果圖像
            if result.success:
                result_image = self._create_result_image(original_image, result)
                self.stats['successful_detections'] += 1
                self._log_info(f"檢測成功: 中心{result.center}, 角度{result.angle:.2f}°, 耗時{total_time:.1f}ms")
            else:
                result_image = original_image.copy()
                cv2.putText(result_image, f"FAILED: {result.error_message}", 
                           (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                self.stats['failed_detections'] += 1
                self._log_error(f"檢測失敗: {result.error_message}")
            
            # 儲存圖像
            if any([self.save_options.save_original, self.save_options.save_processed, self.save_options.save_result]):
                self._save_debug_images(original_image, processed_images, result_image, result)
            
            return result
            
        except Exception as e:
            error_result = DetectionResult(
                success=False,
                error_message=str(e),
                capture_time=(time.perf_counter() - start_time) * 1000,
                total_time=(time.perf_counter() - start_time) * 1000
            )
            self.stats['failed_detections'] += 1
            self._log_error(f"檢測過程異常: {e}")
            
            # 嘗試重新連接
            if "觸發失敗" in str(e) or "捕獲失敗" in str(e):
                self._log_info("檢測到相機問題，嘗試重新連接")
                if self.reconnect_camera():
                    self._log_info("相機重連成功")
                else:
                    self._log_error("相機重連失敗")
            
            return error_result
        
        finally:
            # 記憶體清理
            if original_image is not None:
                del original_image
            if result_image is not None:
                del result_image
            for img in processed_images.values():
                if img is not None:
                    del img
            
            # 定期執行垃圾回收
            if self.stats['total_detections'] % 10 == 0:
                self.cleanup_memory()
    
    def _detect_angle(self, image: np.ndarray, method: DetectionMethod) -> DetectionResult:
        """內部角度檢測方法"""
        try:
            # 圖像前處理
            binary_image = self._preprocess_image(image)
            
            # 輪廓檢測
            contour = self._find_main_contour(binary_image)
            if contour is None:
                return DetectionResult(
                    success=False,
                    error_message="未檢測到有效輪廓"
                )
            
            # 檢測區塊資訊
            detection_region = {
                'image_width': image.shape[1],
                'image_height': image.shape[0],
                'contour_area': cv2.contourArea(contour),
                'contour_perimeter': cv2.arcLength(contour, True)
            }
            
            # 根據方法執行檢測
            if method == DetectionMethod.CASE_ELLIPSE:
                return self._detect_ellipse_method(contour, detection_region)
            elif method == DetectionMethod.DR_MIN_RECT:
                return self._detect_min_rect_method(contour, detection_region)
            elif method == DetectionMethod.SIMPLE_CONTOUR:
                return self._detect_simple_contour_method(contour, detection_region)
            elif method == DetectionMethod.MOMENTS_CENTER:
                return self._detect_moments_method(contour, detection_region)
            else:
                return DetectionResult(
                    success=False,
                    error_message=f"不支援的檢測方法: {method}"
                )
                
        except Exception as e:
            return DetectionResult(
                success=False,
                error_message=f"檢測過程異常: {e}"
            )
    
    def _preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """圖像前處理"""
        # 轉灰度
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # 高斯模糊
        kernel_size = self.params.gaussian_kernel
        if kernel_size not in self._kernel_cache:
            if kernel_size <= 0 or kernel_size % 2 == 0:
                kernel_size = 3
            self._kernel_cache[kernel_size] = (kernel_size, kernel_size)
        
        blur = cv2.GaussianBlur(gray, self._kernel_cache[kernel_size], 0)
        
        # 閾值處理
        if self.params.threshold_mode == 0:  # OTSU
            _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        else:  # 手動
            _, thresh = cv2.threshold(blur, self.params.manual_threshold, 255, cv2.THRESH_BINARY_INV)
        
        return thresh
    
    def _find_main_contour(self, binary_image: np.ndarray):
        """尋找主要輪廓"""
        # 計算最小面積
        if self._min_area_cache is None or self._last_image_shape != binary_image.shape:
            self._min_area_cache = binary_image.shape[0] * binary_image.shape[1] * self.params.min_area_rate
            self._last_image_shape = binary_image.shape
        
        min_area = self._min_area_cache
        
        # 尋找輪廓
        contours, _ = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return None
        
        # 篩選符合面積要求的輪廓
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
        
        if not valid_contours:
            return None
        
        # 根據序列模式選擇輪廓
        if self.params.sequence_mode:
            return valid_contours[-1]  # 最後一個
        else:
            return valid_contours[0]   # 第一個
    
    def _detect_ellipse_method(self, contour, detection_region: Dict) -> DetectionResult:
        """橢圓擬合方法 (CASE模式)"""
        if len(contour) < 5:
            return DetectionResult(
                success=False,
                error_message="輪廓點數不足進行橢圓擬合",
                detection_region=detection_region
            )
        
        try:
            # 橢圓擬合
            ellipse = cv2.fitEllipse(contour)
            (x, y), (MA, ma), angle = ellipse
            
            center = (int(x), int(y))
            
            return DetectionResult(
                success=True,
                center=center,
                angle=angle,
                major_axis=MA,
                minor_axis=ma,
                contour_area=cv2.contourArea(contour),
                detection_region=detection_region
            )
            
        except cv2.error as e:
            return DetectionResult(
                success=False,
                error_message=f"橢圓擬合失敗: {e}",
                detection_region=detection_region
            )
    
    def _detect_min_rect_method(self, contour, detection_region: Dict) -> DetectionResult:
        """最小外接矩形方法 (DR模式)"""
        try:
            rect = cv2.minAreaRect(contour)
            center, size, angle = rect
            
            center_int = (int(center[0]), int(center[1]))
            
            return DetectionResult(
                success=True,
                center=center_int,
                angle=angle,
                rect_width=size[0],
                rect_height=size[1],
                contour_area=cv2.contourArea(contour),
                detection_region=detection_region
            )
            
        except Exception as e:
            return DetectionResult(
                success=False,
                error_message=f"最小外接矩形計算失敗: {e}",
                detection_region=detection_region
            )
    
    def _detect_simple_contour_method(self, contour, detection_region: Dict) -> DetectionResult:
        """簡單輪廓中心點方法"""
        try:
            # 計算輪廓邊界矩形
            x, y, w, h = cv2.boundingRect(contour)
            center = (x + w // 2, y + h // 2)
            
            # 計算主方向 (使用PCA或矩形角度)
            rect = cv2.minAreaRect(contour)
            _, _, angle = rect
            
            return DetectionResult(
                success=True,
                center=center,
                angle=angle,
                contour_area=cv2.contourArea(contour),
                detection_region=detection_region
            )
            
        except Exception as e:
            return DetectionResult(
                success=False,
                error_message=f"簡單輪廓方法失敗: {e}",
                detection_region=detection_region
            )
    
    def _detect_moments_method(self, contour, detection_region: Dict) -> DetectionResult:
        """重心計算方法"""
        try:
            # 計算輪廓矩
            M = cv2.moments(contour)
            
            if M["m00"] == 0:
                return DetectionResult(
                    success=False,
                    error_message="輪廓面積為零，無法計算重心",
                    detection_region=detection_region
                )
            
            # 計算重心
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            center = (cx, cy)
            
            # 計算主方向
            rect = cv2.minAreaRect(contour)
            _, _, angle = rect
            
            return DetectionResult(
                success=True,
                center=center,
                angle=angle,
                contour_area=cv2.contourArea(contour),
                detection_region=detection_region
            )
            
        except Exception as e:
            return DetectionResult(
                success=False,
                error_message=f"重心計算方法失敗: {e}",
                detection_region=detection_region
            )
    
    def _create_result_image(self, original_image: np.ndarray, result: DetectionResult) -> np.ndarray:
        """創建結果圖像"""
        result_image = original_image.copy()
        
        if result.success and result.center:
            # 畫中心點
            cv2.circle(result_image, result.center, 5, (255, 0, 0), -1)
            
            # 畫角度資訊
            if result.angle is not None:
                cv2.putText(result_image, f"Angle: {result.angle:.2f}°", 
                           (result.center[0] - 70, result.center[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # 畫面積資訊
            if result.contour_area:
                cv2.putText(result_image, f"Area: {result.contour_area:.0f}", 
                           (result.center[0] - 50, result.center[1] + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            # 畫方法名稱
            method_name = DetectionMethod.get_description(self.params.detection_method)
            cv2.putText(result_image, f"Method: {method_name}", 
                       (50, result_image.shape[0] - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return result_image
    
    def _save_debug_images(self, original_image: np.ndarray, processed_images: Dict, 
                          result_image: np.ndarray, result: DetectionResult):
        """儲存調試圖像"""
        try:
            timestamp = int(time.time())
            base_filename = f"{self.save_options.filename_prefix}_{timestamp}"
            
            if self.save_options.save_original and original_image is not None:
                original_path = os.path.join(self.save_options.save_directory, f"{base_filename}_original.jpg")
                cv2.imwrite(original_path, original_image)
            
            if self.save_options.save_processed and processed_images:
                for name, img in processed_images.items():
                    if img is not None:
                        processed_path = os.path.join(self.save_options.save_directory, f"{base_filename}_{name}.jpg")
                        cv2.imwrite(processed_path, img)
            
            if self.save_options.save_result and result_image is not None:
                result_path = os.path.join(self.save_options.save_directory, f"{base_filename}_result.jpg")
                cv2.imwrite(result_path, result_image)
                
                # 同時儲存檢測資訊JSON
                info_path = os.path.join(self.save_options.save_directory, f"{base_filename}_info.json")
                with open(info_path, 'w', encoding='utf-8') as f:
                    json.dump(result.to_dict(), f, indent=2, ensure_ascii=False)
            
        except Exception as e:
            self._log_error(f"儲存調試圖像失敗: {e}")
    
    def cleanup_memory(self):
        """清理記憶體"""
        try:
            self.memory_manager.force_cleanup()
            self.stats['memory_cleanups'] += 1
            memory_info = self.memory_manager.get_memory_usage()
            self._log_info(f"記憶體清理完成，當前使用: {memory_info['rss_mb']:.1f}MB")
        except Exception as e:
            self._log_error(f"記憶體清理失敗: {e}")
    
    def get_memory_status(self) -> Dict[str, Any]:
        """獲取記憶體狀態"""
        return self.memory_manager.get_memory_usage()
    
    def get_statistics(self) -> Dict[str, Any]:
        """獲取統計資訊"""
        stats = self.stats.copy()
        stats['memory_status'] = self.get_memory_status()
        stats['is_initialized'] = self.is_initialized
        stats['success_rate'] = (
            self.stats['successful_detections'] / max(self.stats['total_detections'], 1) * 100
        )
        return stats
    
    def disconnect(self):
        """斷開連接並清理資源"""
        try:
            self._log_info("正在斷開連接並清理資源")
            
            if self.camera:
                if getattr(self.camera, 'is_streaming', False):
                    self.camera.stop_streaming()
                self.camera.disconnect()
                self.camera = None
            
            self.is_initialized = False
            self.cleanup_memory()
            
            self._log_info("資源清理完成")
            
        except Exception as e:
            self._log_error(f"斷開連接失敗: {e}")
    
    def _log_info(self, message: str):
        """記錄資訊"""
        print(f"[CCD3HeadlessDetector INFO] {message}")
    
    def _log_warning(self, message: str):
        """記錄警告"""
        print(f"[CCD3HeadlessDetector WARNING] {message}")
    
    def _log_error(self, message: str):
        """記錄錯誤"""
        print(f"[CCD3HeadlessDetector ERROR] {message}")

# 使用範例
def example_usage():
    """使用範例"""
    # 創建檢測器
    detector = CCD3HeadlessDetector(camera_ip="192.168.1.10")
    
    try:
        # 初始化相機
        if not detector.initialize_camera():
            print("相機初始化失敗")
            return
        
        # 設置檢測參數
        detector.set_detection_params(
            detection_method=DetectionMethod.DR_MIN_RECT,
            min_area_rate=0.05,
            gaussian_kernel=3,
            threshold_mode=0
        )
        
        # 設置圖像儲存
        detector.set_image_save_options(
            save_original=True,
            save_result=True,
            save_directory="ccd3_debug",
            filename_prefix="test"
        )
        
        # 執行檢測
        result = detector.capture_and_detect()
        
        if result.success:
            print(f"檢測成功:")
            print(f"  中心點: {result.center}")
            print(f"  角度: {result.angle:.2f}°")
            print(f"  處理時間: {result.total_time:.1f}ms")
        else:
            print(f"檢測失敗: {result.error_message}")
        
        # 查看統計資訊
        stats = detector.get_statistics()
        print(f"統計資訊: {stats}")
        
    finally:
        # 清理資源
        detector.disconnect()

if __name__ == "__main__":
    example_usage()