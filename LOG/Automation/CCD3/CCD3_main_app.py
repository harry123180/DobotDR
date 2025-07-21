import sys
import os
import time
import threading
import json
import logging
from logging.handlers import RotatingFileHandler
import statistics
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import cv2
import numpy as np
import math

# PyModbus imports
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

# Flask imports
from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_socketio import SocketIO, emit

# Import camera manager
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'API'))
from camera_manager import OptimizedCamera, CameraConfig

def setup_logging(module_name):
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

# 設置logger
logger = setup_logging("CCD3_AngleDetection")

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
        logger.debug("SystemStateMachine已初始化，初始狀態: Ready=1")
    
    def set_bit(self, bit_pos: StatusBits, value: bool):
        with self.lock:
            old_value = self.status_register
            if value:
                self.status_register |= (1 << bit_pos.value)
            else:
                self.status_register &= ~(1 << bit_pos.value)
            #logger.debug(f"狀態位{bit_pos.name}設置為{value}: 0x{old_value:04X} -> 0x{self.status_register:04X}")
    
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
            old_value = self.status_register
            self.status_register = 0b1001  # Ready=1, Initialized=1
            logger.info(f"狀態機重置為閒置: 0x{old_value:04X} -> 0x{self.status_register:04X}")

class PerformanceMonitor:
    """性能監控類"""
    def __init__(self):
        self.times = []
        self.capture_times = []
        self.process_times = []
        self.lock = threading.Lock()
        logger.debug("PerformanceMonitor已初始化")
        
    def add_result(self, result: AngleResult):
        """添加檢測結果用於性能分析"""
        if result.success:
            with self.lock:
                self.times.append(result.total_time)
                self.capture_times.append(result.capture_time)
                self.process_times.append(result.processing_time)
                
                # 保持最近100次記錄
                if len(self.times) > 100:
                    self.times.pop(0)
                    self.capture_times.pop(0)
                    self.process_times.pop(0)
                
                # 每50次記錄性能統計
                if len(self.times) % 50 == 0:
                    logger.debug(f"性能監控：已記錄{len(self.times)}次成功檢測")
    
    def get_stats(self):
        """獲取性能統計"""
        with self.lock:
            if not self.times:
                return {}
            
            stats = {
                'avg_total_time': statistics.mean(self.times),
                'avg_capture_time': statistics.mean(self.capture_times),
                'avg_process_time': statistics.mean(self.process_times),
                'min_total_time': min(self.times),
                'max_total_time': max(self.times),
                'sample_count': len(self.times)
            }
            
            # 每次獲取統計時記錄性能摘要
            if len(self.times) >= 10:
                logger.debug(f"性能統計摘要: 平均總時間{stats['avg_total_time']:.1f}ms, 樣本數{stats['sample_count']}")
            
            return stats

class AngleDetector:
    def __init__(self):
        self.min_area_rate = 0.05
        self.sequence_mode = False
        self.gaussian_kernel = 3
        self.threshold_mode = 0  # 0=OTSU, 1=Manual
        self.manual_threshold = 127
        
        # 性能優化：預編譯快取
        self._kernel_cache = {}
        self._last_image_shape = None
        self._min_area_cache = None
        
        logger.debug("AngleDetector已初始化，使用默認參數")
    
    def update_params(self, **kwargs):
        """更新檢測參數 - 優化：減少不必要的更新"""
        changed = False
        old_params = {
            'min_area_rate': self.min_area_rate,
            'sequence_mode': self.sequence_mode,
            'gaussian_kernel': self.gaussian_kernel,
            'threshold_mode': self.threshold_mode,
            'manual_threshold': self.manual_threshold
        }
        
        if 'min_area_rate' in kwargs and kwargs['min_area_rate'] != self.min_area_rate * 1000:
            self.min_area_rate = kwargs['min_area_rate'] / 1000.0
            self._min_area_cache = None  # 清除面積快取
            changed = True
        if 'sequence_mode' in kwargs and bool(kwargs['sequence_mode']) != self.sequence_mode:
            self.sequence_mode = bool(kwargs['sequence_mode'])
            changed = True
        if 'gaussian_kernel' in kwargs and kwargs['gaussian_kernel'] != self.gaussian_kernel:
            self.gaussian_kernel = kwargs['gaussian_kernel']
            changed = True
        if 'threshold_mode' in kwargs and kwargs['threshold_mode'] != self.threshold_mode:
            self.threshold_mode = kwargs['threshold_mode']
            changed = True
        if 'manual_threshold' in kwargs and kwargs['manual_threshold'] != self.manual_threshold:
            self.manual_threshold = kwargs['manual_threshold']
            changed = True
        
        if changed:
            logger.info(f"檢測參數已更新：面積比={self.min_area_rate:.3f}, 高斯核={self.gaussian_kernel}, 閾值模式={self.threshold_mode}")
            logger.debug(f"參數變更詳情: {old_params} -> {kwargs}")
    
    def get_pre_treatment_image_optimized(self, image):
        """優化版影像前處理 - 修正為使用固定閾值210"""
        try:
            # 優化1：跳過不必要的顏色空間轉換
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image
            
            # 優化2：使用固定核尺寸避免重複計算
            kernel_size = self.gaussian_kernel
            if kernel_size not in self._kernel_cache:
                if kernel_size <= 0 or kernel_size % 2 == 0:
                    kernel_size = 3
                self._kernel_cache[kernel_size] = (kernel_size, kernel_size)
            
            blur = cv2.GaussianBlur(gray, self._kernel_cache[kernel_size], 0)
            
            # 修正關鍵：強制使用固定閾值210，不使用OTSU
            _, thresh = cv2.threshold(blur, 210, 255, cv2.THRESH_BINARY_INV)
            logger.debug("使用固定閾值210進行二值化處理")
            
            return thresh
            
        except Exception as e:
            logger.error(f"影像前處理失敗: {e}", exc_info=True)
            raise
    
    def get_main_contour_optimized(self, image, sequence=False):
        """優化版輪廓檢測 - 修正面積計算邏輯"""
        try:
            # 修正關鍵：直接使用0.05，不從快取讀取以避免參數錯誤
            min_area = image.shape[0] * image.shape[1] * 0.05  # 直接使用paste.txt的邏輯
            logger.debug(f"輪廓檢測參數: 圖像尺寸={image.shape}, 最小面積比率=0.05, 最小面積={min_area:.0f}")
            
            # 使用RETR_TREE保持與paste.txt一致
            contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) == 0:
                logger.warning("未檢測到任何輪廓")
                return None
            
            # 篩選符合面積要求的輪廓 (完全參考paste.txt邏輯)
            valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
            logger.debug(f"檢測到 {len(contours)} 個輪廓，符合面積要求的輪廓數量: {len(valid_contours)}")
            
            if not valid_contours:
                logger.warning("沒有輪廓符合最小面積要求")
                return None
            
            # 根據sequence模式選擇輪廓 (完全參考paste.txt邏輯)
            if sequence:
                # sequence=True: 選擇最後一個輪廓 (paste.txt中CASE模式使用)
                contour = valid_contours[-1]
                logger.debug(f"CASE模式: 選擇最後一個輪廓，面積: {cv2.contourArea(contour):.0f}")
            else:
                # sequence=False: 選擇第一個輪廓 (paste.txt中DR模式使用)
                contour = valid_contours[0]
                logger.debug(f"DR模式: 選擇第一個輪廓，面積: {cv2.contourArea(contour):.0f}")
            
            return contour
            
        except Exception as e:
            logger.error(f"輪廓檢測失敗: {e}", exc_info=True)
            return None
    
    def _detect_angle_dr_mode(self, contour):
        """DR模式角度檢測 - 完全參考paste.txt邏輯"""
        try:
            logger.debug("執行DR模式角度檢測 (mode=1)")
            
            rect = cv2.minAreaRect(contour)
            center, size, angle = rect
            
            logger.debug(f"minAreaRect結果: 中心=({center[0]:.2f}, {center[1]:.2f}), 尺寸=({size[0]:.2f}, {size[1]:.2f}), 角度={angle:.2f}")
            
            # 完全按照paste.txt: 直接使用rect[2]的角度，不做任何修正
            corrected_angle = angle  # paste.txt: angle = rect[2]
            
            # 中心點轉換: 完全按照paste.txt邏輯
            center_int = (int(center[0]), int(center[1]))  # paste.txt: center = tuple(np.int_(rect[0]))
            
            extra_data = {
                'rect_width': size[0],
                'rect_height': size[1]
            }
            
            logger.debug(f"DR模式最終結果: 中心={center_int}, 角度={corrected_angle:.2f}度")
            return center_int, corrected_angle, extra_data
            
        except Exception as e:
            logger.error(f"DR模式角度檢測失敗: {e}", exc_info=True)
            return None
    
    def _detect_angle_case_mode(self, contour, original_image):
        """CASE模式角度檢測 - 使用橢圓擬合複雜邏輯"""
        if len(contour) < 5:
            logger.warning("CASE模式: 輪廓點數不足(<5)，無法進行橢圓擬合")
            return None
        
        try:
            logger.debug("執行CASE模式角度檢測 (mode=0)")
            
            # 建立遮罩
            mask_1 = np.zeros((original_image.shape[0], original_image.shape[1]), dtype=np.uint8)
            mask_2 = np.zeros((original_image.shape[0], original_image.shape[1]), dtype=np.uint8)
            
            # 填充輪廓
            cv2.drawContours(mask_1, [contour], -1, (255, 255, 255), -1)
            
            # 橢圓擬合
            ellipse = cv2.fitEllipse(contour)
            (x, y), (MA, ma), angle = ellipse
            
            center = (int(x), int(y))
            logger.debug(f"橢圓擬合結果: 中心=({x:.2f}, {y:.2f}), 長軸={MA:.2f}, 短軸={ma:.2f}, 角度={angle:.2f}")
            
            # 橢圓遮罩處理
            cv2.ellipse(mask_1, ellipse, (0, 0, 0), -1)
            
            # 外接圓
            center_circle, radius = cv2.minEnclosingCircle(contour)
            center_circle = (int(center_circle[0]), int(center_circle[1]))
            cv2.circle(mask_2, center_circle, int(radius), (255, 255, 255), -1)
            
            # 形態學處理
            kernel = np.ones((11, 11), np.uint8)
            mask_1 = cv2.dilate(mask_1, kernel, iterations=1)
            mask_1 = cv2.bitwise_not(mask_1)
            rst = cv2.bitwise_and(mask_1, mask_1, mask=mask_2)
            
            # 找到處理後的輪廓
            rst_contour = self.get_main_contour_optimized(rst)
            if rst_contour is None:
                # 如果處理後沒有輪廓，回退到原始輪廓
                logger.warning("CASE模式: 處理後無輪廓，回退到原始輪廓")
                rst_contour = contour
            
            # 對處理後的輪廓使用minAreaRect
            rect = cv2.minAreaRect(rst_contour)
            final_center, size, final_angle = rect
            
            center_int = (int(final_center[0]), int(final_center[1]))
            
            extra_data = {
                'major_axis': MA,
                'minor_axis': ma,
                'ellipse_angle': angle,
                'final_angle': final_angle
            }
            
            logger.debug(f"CASE模式最終結果: 中心={center_int}, 最終角度={final_angle:.2f}度")
            return center_int, final_angle, extra_data
            
        except cv2.error as e:
            logger.error(f"CASE模式檢測錯誤: {e}", exc_info=True)
            return None
        except Exception as e:
            logger.error(f"CASE模式檢測異常: {e}", exc_info=True)
            return None
    
    def detect_angle(self, image, mode=0) -> AngleResult:
        """優化版角度檢測主函數 - 加入調試圖像保存"""
        start_time = time.perf_counter()  # 優化7：使用高精度計時器
        
        # 準備調試圖像變量
        original_image = None
        binary_image = None
        result_image = None
        
        try:
            logger.debug(f"開始角度檢測: mode={mode}")
            
            # 優化8：減少不必要的格式轉換
            if len(image.shape) == 2:
                bgr_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            elif len(image.shape) == 3 and image.shape[2] == 1:
                gray_image = image.squeeze()
                bgr_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
            elif len(image.shape) == 3 and image.shape[2] == 3:
                bgr_image = image
            else:
                raise Exception(f"不支援的圖像格式: {image.shape}")
            
            # 保存原始圖像用於調試
            original_image = bgr_image.copy()
            
            # 優化9：直接調用優化版前處理
            pt_img = self.get_pre_treatment_image_optimized(bgr_image)
            binary_image = pt_img.copy()  # 保存二值化圖像
            
            # 根據模式選擇不同的輪廓檢測策略
            if mode == 0:
                # CASE模式：使用sequence=True
                rst_contour = self.get_main_contour_optimized(pt_img, sequence=True)
            else:
                # DR模式：使用sequence=False
                rst_contour = self.get_main_contour_optimized(pt_img, sequence=False)
            
            # 準備結果圖像
            result_image = bgr_image.copy()
            
            if rst_contour is None:
                # 在結果圖像上標註失敗訊息
                cv2.putText(result_image, "No Valid Contour Found", (50, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                logger.warning("角度檢測失敗: 未檢測到有效輪廓")
                
                return AngleResult(
                    success=False, center=None, angle=None,
                    major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                    contour_area=None, processing_time=0, capture_time=0,
                    total_time=(time.perf_counter() - start_time) * 1000,
                    error_message="未檢測到有效輪廓"
                )
            
            contour_area = cv2.contourArea(rst_contour)
            logger.debug(f"檢測到輪廓面積: {contour_area:.0f} 像素")
            
            # 降低面積檢查閾值，原本100太大
            min_area_threshold = 50  # 降低閾值
            if contour_area < min_area_threshold:
                # 在結果圖像上標註面積太小
                cv2.putText(result_image, f"Area Too Small: {contour_area:.0f} < {min_area_threshold}", 
                           (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                # 仍然畫出檢測到的輪廓
                cv2.drawContours(result_image, [rst_contour], -1, (255, 0, 0), 2)
                
                logger.warning(f"角度檢測失敗: 輪廓面積太小 {contour_area:.0f} < {min_area_threshold}")
                
                return AngleResult(
                    success=False, center=None, angle=None,
                    major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                    contour_area=contour_area, processing_time=0, capture_time=0,
                    total_time=(time.perf_counter() - start_time) * 1000,
                    error_message=f"輪廓面積太小: {contour_area:.0f} < {min_area_threshold}"
                )
            
            # 優化12：角度檢測算法選擇 (修正模式對應)
            if mode == 0:
                # CASE模式：複雜的橢圓+遮罩處理
                result = self._detect_angle_case_mode(rst_contour, bgr_image)
            else:
                # DR模式：簡單的最小外接矩形
                result = self._detect_angle_dr_mode(rst_contour)
            
            if result is None:
                # 在結果圖像上標註角度計算失敗
                cv2.putText(result_image, "Angle Calculation Failed", (50, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.drawContours(result_image, [rst_contour], -1, (255, 0, 0), 2)
                
                logger.error("角度檢測失敗: 角度計算失敗")
                
                return AngleResult(
                    success=False, center=None, angle=None,
                    major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                    contour_area=contour_area, processing_time=0, capture_time=0,
                    total_time=(time.perf_counter() - start_time) * 1000,
                    error_message="角度計算失敗"
                )
            
            center, angle, extra_data = result
            processing_time = (time.perf_counter() - start_time) * 1000
            
            # 在結果圖像上標註成功結果
            cv2.drawContours(result_image, [rst_contour], -1, (0, 255, 0), 2)
            cv2.circle(result_image, center, 5, (255, 0, 0), -1)
            
            # 添加最小外接矩形框 (如果是DR模式)
            if mode == 1:  # DR模式
                rect = cv2.minAreaRect(rst_contour)
                box = cv2.boxPoints(rect)
                box = np.int_(box)
                cv2.drawContours(result_image, [box], 0, (0, 255, 0), 2)
            
            cv2.putText(result_image, f"Angle: {angle:.2f} deg", 
                       (center[0] - 70, center[1] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(result_image, f"Area: {contour_area:.0f}", 
                       (center[0] - 50, center[1] + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            cv2.putText(result_image, f"Mode: {'CASE' if mode == 0 else 'DR'}", 
                       (50, result_image.shape[0] - 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            logger.info(f"角度檢測成功: 中心{center}, 角度{angle:.2f}度, 面積{contour_area:.0f}, 耗時{processing_time:.1f}ms")
            
            return AngleResult(
                success=True,
                center=center,
                angle=angle,
                major_axis=extra_data.get('major_axis'),
                minor_axis=extra_data.get('minor_axis'),
                rect_width=extra_data.get('rect_width'),
                rect_height=extra_data.get('rect_height'),
                contour_area=contour_area,
                processing_time=processing_time,
                capture_time=0,
                total_time=processing_time
            )
            
        except Exception as e:
            # 錯誤情況也保存調試圖像
            if result_image is not None:
                cv2.putText(result_image, f"Exception: {str(e)[:50]}", (50, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            logger.error(f"角度檢測異常: {e}", exc_info=True)
            
            return AngleResult(
                success=False, center=None, angle=None,
                major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                contour_area=None, processing_time=0, capture_time=0,
                total_time=(time.perf_counter() - start_time) * 1000,
                error_message=str(e)
            )
        
        finally:
            # 無論成功還是失敗，都保存調試圖像
            if original_image is not None and binary_image is not None and result_image is not None:
                # 從外部服務類調用保存函數
                pass  # 將在capture_and_detect_angle中處理

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
        
        # 性能優化：參數快取和監控
        self._last_params = {}
        self._params_changed = True
        self.perf_monitor = PerformanceMonitor()
        
        # 調試圖像儲存
        self.debug_enabled = True  # 啟用調試圖像
        self.debug_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'debug_images')
        self._ensure_debug_dir()
        
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
        
        # 預設檢測參數 - 設定DR模式為默認
        self.default_detection_params = {
            'detection_mode': 1,        # 改為DR模式1 (最小外接矩形模式)
            'min_area_rate': 50,        # 0.05 → 50 (存儲時×1000)
            'sequence_mode': 0,         # 0=最大輪廓, 1=序列輪廓
            'gaussian_kernel': 3,       # 高斯模糊核大小
            'threshold_mode': 0,        # 0=OTSU自動, 1=手動
            'manual_threshold': 127     # 手動閾值 (0-255)
        }
        
        # 配置檔案
        self.config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ccd3_config.json')
        self.load_config()
        
        # 預設參數已寫入標誌
        self.default_params_written = False
        
        logger.info("CCD3AngleDetectionService已初始化")
        logger.debug(f"基地址: {self.base_address}, 服務器: {self.server_ip}:{self.server_port}")
    
    def _ensure_debug_dir(self):
        """確保調試圖像目錄存在"""
        if not os.path.exists(self.debug_dir):
            os.makedirs(self.debug_dir)
            logger.info(f"已創建調試圖像目錄: {self.debug_dir}")
    
    def save_debug_images(self, original_image, binary_image, result_image, detection_success):
        """保存調試圖像 - 每次覆蓋，不重複產生"""
        if not self.debug_enabled:
            return
        
        try:
            # 固定檔名，每次覆蓋
            original_path = os.path.join(self.debug_dir, '1_original.jpg')
            binary_path = os.path.join(self.debug_dir, '2_binary.jpg')
            result_path = os.path.join(self.debug_dir, '3_result.jpg')
            
            # 保存原始圖像
            cv2.imwrite(original_path, original_image)
            
            # 保存二值化圖像 (轉為3通道方便查看)
            binary_bgr = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)
            cv2.imwrite(binary_path, binary_bgr)
            
            # 保存結果圖像
            cv2.imwrite(result_path, result_image)
            
            status = "成功" if detection_success else "失敗"
            logger.debug(f"調試圖像已保存 (檢測{status}): {self.debug_dir}")
            
        except Exception as e:
            logger.error(f"保存調試圖像失敗: {e}", exc_info=True)
    
    def load_config(self):
        """載入配置檔案"""
        default_config = {
            "module_id": "CCD3_Angle_Detection_Optimized",
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
                logger.info(f"配置檔案載入成功: {self.config_file}")
            else:
                config = default_config
                with open(self.config_file, 'w', encoding='utf-8') as f:
                    json.dump(config, f, indent=2, ensure_ascii=False)
                logger.info(f"創建默認配置檔案: {self.config_file}")
            
            # 應用配置
            self.server_ip = config['tcp_server']['host']
            self.server_port = config['tcp_server']['port']
            self.base_address = config['modbus_mapping']['base_address']
            
            logger.debug(f"配置應用成功: IP={self.server_ip}, Port={self.server_port}, Base={self.base_address}")
            
        except Exception as e:
            logger.error(f"配置檔案載入錯誤: {e}", exc_info=True)
    
    def connect_modbus(self) -> bool:
        """連接Modbus TCP服務器"""
        try:
            logger.info(f"正在連接Modbus TCP服務器: {self.server_ip}:{self.server_port}")
            
            if self.modbus_client:
                self.modbus_client.close()
            
            self.modbus_client = ModbusTcpClient(
                host=self.server_ip,
                port=self.server_port,
                timeout=3
            )
            
            if self.modbus_client.connect():
                self.connection_count += 1
                logger.info(f"CCD3角度檢測模組已連接到Modbus服務器，連接計數: {self.connection_count}")
                return True
            else:
                logger.error(f"Modbus連接失敗: 無法連接到 {self.server_ip}:{self.server_port}")
                self.state_machine.set_alarm(True)
                return False
                
        except Exception as e:
            logger.error(f"Modbus連接異常: {e}", exc_info=True)
            self.state_machine.set_alarm(True)
            return False
    
    def initialize_camera(self, ip_address: str = "192.168.1.10") -> bool:
        """初始化相機"""
        try:
            logger.info(f"正在初始化相機，IP地址: {ip_address}")
            
            if self.camera:
                logger.info("關閉現有相機連接...")
                self.camera.disconnect()
                self.camera = None
            
            config = CameraConfig(
                name="ccd3_camera",
                ip=ip_address,
                exposure_time=20000.0,
                gain=200.0,
                frame_rate=30.0,
                width=2592,
                height=1600
            )
            
            logger.debug(f"相機配置: 曝光時間={config.exposure_time}, 增益={config.gain}, 分辨率={config.width}x{config.height}")
            
            self.camera = OptimizedCamera(config, logger)
            
            logger.info("正在連接相機...")
            if self.camera.connect():
                logger.info(f"CCD3相機已成功連接: {ip_address}")
                
                logger.info("啟動相機串流...")
                if self.camera.start_streaming():
                    logger.info("相機串流啟動成功")
                    
                    logger.info("測試相機圖像捕獲能力...")
                    try:
                        test_image = self.camera.capture_latest_frame(timeout=3000)
                        if test_image is not None:
                            logger.info(f"相機測試成功，可以捕獲圖像，測試圖像尺寸: {test_image.data.shape}")
                            self.state_machine.set_initialized(True)
                            self.state_machine.set_alarm(False)
                            return True
                        else:
                            logger.error("相機測試失敗: 無法捕獲圖像")
                            self.state_machine.set_alarm(True)
                            self.state_machine.set_initialized(False)
                            return False
                    except Exception as e:
                        logger.error(f"相機測試異常: {e}", exc_info=True)
                        self.state_machine.set_alarm(True)
                        self.state_machine.set_initialized(False)
                        return False
                else:
                    logger.error("相機串流啟動失敗")
                    self.state_machine.set_alarm(True)
                    self.state_machine.set_initialized(False)
                    return False
            else:
                logger.error(f"相機連接失敗: {ip_address}")
                self.state_machine.set_alarm(True)
                self.state_machine.set_initialized(False)
                return False
                
        except Exception as e:
            logger.error(f"相機初始化異常: {e}", exc_info=True)
            self.state_machine.set_alarm(True)
            self.state_machine.set_initialized(False)
            return False
    
    def _immediate_status_update(self):
        """立即更新狀態寄存器到Modbus服務器"""
        try:
            if self.modbus_client and self.modbus_client.connected:
                result = self.modbus_client.write_register(
                    address=self.base_address + 1,
                    value=self.state_machine.status_register,
                    slave=1
                )
                if not result.isError():
                    logger.debug(f"立即更新狀態寄存器801: 0x{self.state_machine.status_register:04X}")
                    logger.debug(f"狀態詳情: Ready={self.state_machine.is_ready()}, Running={self.state_machine.is_running()}, Alarm={self.state_machine.is_alarm()}, Initialized={self.state_machine.is_initialized()}")
                else:
                    logger.error(f"狀態寄存器更新失敗: {result}")
        except Exception as e:
            logger.error(f"立即狀態更新異常: {e}", exc_info=True)
    
    def write_default_detection_params(self) -> bool:
        """寫入預設檢測參數到ModbusTCP Server"""
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                logger.error("無法寫入預設參數: Modbus Client未連接")
                return False
            
            logger.info("開始寫入預設檢測參數到ModbusTCP Server")
            logger.debug(f"基地址: {self.base_address}, 參數寄存器範圍: {self.base_address + 10} ~ {self.base_address + 15}")
            
            # 準備寄存器數據 (810-815，共6個寄存器)
            params_registers = [
                self.default_detection_params['detection_mode'],     # 810: 檢測模式
                self.default_detection_params['min_area_rate'],      # 811: 最小面積比例
                self.default_detection_params['sequence_mode'],      # 812: 序列模式
                self.default_detection_params['gaussian_kernel'],    # 813: 高斯模糊核大小
                self.default_detection_params['threshold_mode'],     # 814: 閾值處理模式
                self.default_detection_params['manual_threshold']    # 815: 手動閾值
            ]
            
            logger.info("準備寫入預設參數:")
            logger.info(f"檢測模式 = {params_registers[0]} ({'CASE橢圓擬合' if params_registers[0] == 0 else 'DR最小外接矩形'})")
            logger.info(f"最小面積比例 = {params_registers[1]} (實際比例: {params_registers[1]/1000.0:.3f})")
            logger.info(f"序列模式 = {params_registers[2]} ({'最大輪廓' if params_registers[2] == 0 else '序列輪廓'})")
            logger.info(f"高斯模糊核 = {params_registers[3]}")
            logger.info(f"閾值模式 = {params_registers[4]} ({'OTSU自動' if params_registers[4] == 0 else '手動'})")
            logger.info(f"手動閾值 = {params_registers[5]}")
            
            # 批次寫入檢測參數
            logger.debug(f"開始批次寫入: 地址{self.base_address + 10}-{self.base_address + 15}, 數量6個寄存器")
            
            write_result = self.modbus_client.write_registers(
                address=self.base_address + 10, 
                values=params_registers, 
                slave=1
            )
            
            if write_result.isError():
                logger.error(f"預設參數寫入失敗: {write_result}")
                return False
            else:
                logger.info("預設檢測參數已成功批次寫入到ModbusTCP Server")
                logger.info("默認使用: DR模式1 (最小外接矩形角度檢測)")
                self.default_params_written = True
                
                # 立即更新本地檢測器參數
                logger.debug("同步更新本地檢測器參數")
                self.angle_detector.update_params(**self.default_detection_params)
                
                return True
                
        except Exception as e:
            logger.error(f"寫入預設參數發生異常: {e}", exc_info=True)
            logger.error(f"基地址: {self.base_address}, 連接狀態: {self.modbus_client.connected if self.modbus_client else 'None'}")
            return False
    
    def capture_and_detect_angle(self, mode: int = 1) -> AngleResult:
        """優化版拍照並檢測角度 - 默認DR模式1"""
        if not self.camera:
            logger.error("相機未初始化，無法執行檢測")
            return AngleResult(
                success=False, center=None, angle=None,
                major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                contour_area=None, processing_time=0, capture_time=0, total_time=0,
                error_message="相機未初始化"
            )
        if not getattr(self.camera, 'is_streaming', False):
            logger.error("相機串流未啟動，無法執行檢測")
            return AngleResult(
                success=False, center=None, angle=None,
                major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                contour_area=None, processing_time=0, capture_time=0, total_time=0,
                error_message="相機串流未啟動"
            )
        
        capture_start = time.perf_counter()  # 高精度計時
        
        try:
            logger.debug(f"開始執行拍照並檢測角度，模式: {mode}")
            
            # 優化13：移除不必要的日誌輸出，只保留關鍵訊息
            frame_data = self.camera.capture_latest_frame(timeout=3000)
            
            if frame_data is None:
                raise Exception("圖像捕獲失敗")
            
            image = frame_data.data
            capture_time = (time.perf_counter() - capture_start) * 1000
            
            logger.debug(f"圖像捕獲成功，尺寸: {image.shape}, 捕獲耗時: {capture_time:.1f}ms")
            
            # 優化14：參數快取機制
            detection_params = self.read_detection_parameters_cached()
            if detection_params and self._params_changed:
                logger.debug("檢測參數已變更，更新本地檢測器")
                self.angle_detector.update_params(**detection_params)
                self._params_changed = False
            
            # 準備調試圖像變量
            original_image = image.copy()
            binary_image = None
            result_image = None
            
            # 優化15：使用優化版檢測算法，並獲取調試圖像
            class DebugAngleDetector(AngleDetector):
                def __init__(self, parent_detector):
                    # 複製父檢測器的所有屬性
                    self.__dict__.update(parent_detector.__dict__)
                    self.debug_images = {}
                
                def get_pre_treatment_image_optimized(self, image):
                    result = super().get_pre_treatment_image_optimized(image)
                    self.debug_images['binary'] = result.copy()
                    return result
                
                def detect_angle(self, image, mode=1):  # 默認DR模式1
                    result = super().detect_angle(image, mode)
                    return result
            
            # 創建調試版檢測器
            debug_detector = DebugAngleDetector(self.angle_detector)
            result = debug_detector.detect_angle(image, mode)
            
            # 獲取調試圖像
            binary_image = debug_detector.debug_images.get('binary')
            
            # 創建結果圖像
            result_image = image.copy()
            if result.success and result.center:
                # 成功情況：畫出檢測結果
                cv2.circle(result_image, result.center, 5, (255, 0, 0), -1)
                cv2.putText(result_image, f"Angle: {result.angle:.2f} deg", 
                           (result.center[0] - 70, result.center[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(result_image, f"Area: {result.contour_area:.0f}", 
                           (result.center[0] - 50, result.center[1] + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                cv2.putText(result_image, f"Mode: {'CASE' if mode == 0 else 'DR'}", 
                           (50, result_image.shape[0] - 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            else:
                # 失敗情況：標註錯誤訊息
                cv2.putText(result_image, f"FAILED: {result.error_message}", (50, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.putText(result_image, f"Mode: {'CASE' if mode == 0 else 'DR'}", 
                           (50, result_image.shape[0] - 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # 保存調試圖像
            if binary_image is not None:
                self.save_debug_images(original_image, binary_image, result_image, result.success)
            
            result.capture_time = capture_time
            result.total_time = (time.perf_counter() - capture_start) * 1000
            
            # 性能監控
            self.perf_monitor.add_result(result)
            
            if result.success:
                self.operation_count += 1
                # 每50次成功後輸出性能統計
                if self.operation_count % 50 == 0:
                    stats = self.perf_monitor.get_stats()
                    logger.info(f"性能統計(最近{stats.get('sample_count', 0)}次): 平均總時間={stats.get('avg_total_time', 0):.1f}ms, 平均處理時間={stats.get('avg_process_time', 0):.1f}ms")
            else:
                self.error_count += 1
                logger.warning(f"檢測失敗: {result.error_message}")
            
            return result
            
        except Exception as e:
            self.error_count += 1
            logger.error(f"拍照檢測異常: {e}", exc_info=True)
            
            error_result = AngleResult(
                success=False, center=None, angle=None,
                major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                contour_area=None, processing_time=0,
                capture_time=(time.perf_counter() - capture_start) * 1000,
                total_time=(time.perf_counter() - capture_start) * 1000,
                error_message=str(e)
            )
            
            # 錯誤情況也嘗試保存調試圖像
            if 'image' in locals():
                try:
                    error_image = image.copy()
                    cv2.putText(error_image, f"ERROR: {str(e)[:50]}", (50, 50),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    self.save_debug_images(image, 
                                         np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8),
                                         error_image, False)
                except Exception as save_error:
                    logger.debug(f"保存錯誤調試圖像失敗: {save_error}")
            
            return error_result
    
    def read_detection_parameters_cached(self) -> Dict[str, Any]:
        """優化版參數讀取 - 使用快取機制"""
        params = {}
        try:
            if self.modbus_client and self.modbus_client.connected:
                result = self.modbus_client.read_holding_registers(
                    address=self.base_address + 10, count=6, slave=1  # 只讀取需要的寄存器
                )
                if not result.isError():
                    registers = result.registers
                    current_params = {
                        'detection_mode': registers[0],
                        'min_area_rate': registers[1],
                        'sequence_mode': registers[2],
                        'gaussian_kernel': registers[3],
                        'threshold_mode': registers[4],
                        'manual_threshold': registers[5]
                    }
                    
                    # 檢查參數是否改變
                    if current_params != self._last_params:
                        logger.debug(f"檢測參數變更: {self._last_params} -> {current_params}")
                        self._last_params = current_params.copy()
                        self._params_changed = True
                        params = current_params
                else:
                    logger.debug("讀取檢測參數失敗")
                    
        except Exception as e:
            logger.error(f"讀取檢測參數異常: {e}", exc_info=True)
        
        return params
    
    def write_detection_result(self, result: AngleResult):
        """優化版結果寫入 - 批次寫入減少通訊次數 + 調試訊息"""
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                logger.error("無法寫入檢測結果: Modbus Client未連接")
                return
            
            logger.debug("開始寫入CCD3檢測結果到ModbusTCP Server")
            logger.debug(f"基地址: {self.base_address}, 檢測成功: {result.success}")
            
            # 優化16：一次性準備所有寄存器數據
            all_registers = [0] * 40  # 結果區(20) + 統計區(20)
            
            # 檢測結果區 (840-859對應0-19)
            if result.success and result.center and result.angle is not None:
                logger.debug("檢測成功，準備寫入結果")
                
                all_registers[0] = 1  # 成功標誌
                all_registers[1] = int(result.center[0])  # X座標
                all_registers[2] = int(result.center[1])  # Y座標
                
                # 角度32位存儲
                angle_int = int(result.angle * 100)
                angle_high = (angle_int >> 16) & 0xFFFF
                angle_low = angle_int & 0xFFFF
                all_registers[3] = angle_high
                all_registers[4] = angle_low
                
                logger.debug(f"寫入結果: 中心({result.center[0]}, {result.center[1]}), 角度{result.angle:.2f}度(存儲值{angle_int})")
                
                # 額外參數 - 添加範圍檢查
                if result.major_axis:
                    all_registers[5] = min(int(result.major_axis), 65535)
                if result.minor_axis:
                    all_registers[6] = min(int(result.minor_axis), 65535)
                if result.rect_width:
                    all_registers[7] = min(int(result.rect_width), 65535)
                if result.rect_height:
                    all_registers[8] = min(int(result.rect_height), 65535)
                if result.contour_area:
                    # 輪廓面積可能很大，需要截斷或使用32位存儲
                    area_value = int(result.contour_area)
                    if area_value > 65535:
                        # 使用32位存儲輪廓面積
                        area_high = (area_value >> 16) & 0xFFFF
                        area_low = area_value & 0xFFFF
                        all_registers[9] = area_high  # 高位
                        all_registers[10] = area_low  # 低位
                        logger.debug(f"輪廓面積{result.contour_area:.0f}使用32位存儲")
                    else:
                        all_registers[9] = area_value
            else:
                logger.debug("檢測失敗，寫入失敗標誌")
                all_registers[0] = 0  # 失敗標誌
                if result.error_message:
                    logger.debug(f"錯誤訊息: {result.error_message}")
            
            # 統計資訊區 (880-899對應20-39) - 添加範圍檢查
            all_registers[20] = min(int(result.capture_time), 65535)
            all_registers[21] = min(int(result.processing_time), 65535)
            all_registers[22] = min(int(result.total_time), 65535)
            all_registers[23] = self.operation_count & 0xFFFF  # 只取低16位
            all_registers[24] = min(self.error_count, 65535)
            all_registers[25] = min(self.connection_count, 65535)
            
            all_registers[30] = 3  # 版本號
            all_registers[31] = 1  # 次版本號(優化版)
            uptime_hours = min(int((time.time() - self.start_time) // 3600), 65535)
            uptime_minutes = min(int((time.time() - self.start_time) % 3600 // 60), 65535)
            all_registers[32] = uptime_hours  # 運行小時
            all_registers[33] = uptime_minutes  # 運行分鐘
            
            logger.debug(f"統計資訊: 操作{self.operation_count}次, 錯誤{self.error_count}次, 運行{uptime_hours}小時{uptime_minutes}分鐘")
            
            # 優化17：批次寫入減少Modbus通訊次數
            logger.debug(f"開始批次寫入: 地址{self.base_address + 40}-{self.base_address + 79}, 40個寄存器")
            
            write_result = self.modbus_client.write_registers(
                address=self.base_address + 40, values=all_registers, slave=1
            )
            
            if write_result.isError():
                logger.error(f"檢測結果寫入失敗: {write_result}")
            else:
                logger.debug("檢測結果已成功批次寫入到ModbusTCP Server")
            
        except Exception as e:
            logger.error(f"寫入檢測結果發生異常: {e}", exc_info=True)
            logger.error(f"基地址: {self.base_address}, 連接狀態: {self.modbus_client.connected if self.modbus_client else 'None'}")
    
    def _handshake_sync_loop(self):
        """握手同步循環 - 修改版：包含參數寫入重試邏輯"""
        logger.info("CCD3握手同步線程啟動")
        retry_count = 0
        max_retries = 3
        cycle_count = 0
        
        while not self.stop_handshake:
            try:
                cycle_count += 1
                
                if self.modbus_client and self.modbus_client.connected:
                    # 檢查並重試寫入預設參數
                    if not self.default_params_written and retry_count < max_retries:
                        logger.warning(f"重試寫入預設參數 (第{retry_count + 1}次)")
                        success = self.write_default_detection_params()
                        if success:
                            logger.info("預設參數重試寫入成功")
                        else:
                            retry_count += 1
                            if retry_count >= max_retries:
                                logger.warning("預設參數寫入重試已達上限，停止重試")
                    
                    # 更新狀態寄存器 - 每50個循環記錄一次詳細狀態
                    if cycle_count % 50 == 0:
                        logger.debug(f"握手循環狀態更新 (第{cycle_count}次)")
                    self._update_status_register()
                    
                    # 處理控制指令
                    self._process_control_commands()
                else:
                    # 連接斷開時的警告，每100個循環記錄一次避免日誌洪水
                    if cycle_count % 100 == 0:
                        logger.warning("握手循環: Modbus客戶端未連接")
                
                time.sleep(0.05)  # 50ms循環
                
            except Exception as e:
                logger.error(f"握手同步異常: {e}", exc_info=True)
                time.sleep(1)
        
        logger.info("CCD3握手同步線程停止")
    
    def _update_status_register(self):
        """更新狀態寄存器"""
        try:
            # 更新初始化狀態
            camera_ok = self.camera is not None and getattr(self.camera, 'is_streaming', False)
            modbus_ok = self.modbus_client is not None and self.modbus_client.connected
            
            old_initialized = self.state_machine.is_initialized()
            old_alarm = self.state_machine.is_alarm()
            
            self.state_machine.set_initialized(camera_ok)
            if not (camera_ok and modbus_ok):
                if not camera_ok:
                    self.state_machine.set_alarm(True)
            
            # 只在狀態變化時記錄詳細信息
            if (old_initialized != self.state_machine.is_initialized() or 
                old_alarm != self.state_machine.is_alarm()):
                logger.debug(f"狀態變更: Initialized={old_initialized}->{self.state_machine.is_initialized()}, Alarm={old_alarm}->{self.state_machine.is_alarm()}")
            
            # 寫入狀態寄存器 (801)
            result = self.modbus_client.write_register(
                address=self.base_address + 1,
                value=self.state_machine.status_register,
                slave=1
            )
            
            if result.isError():
                logger.debug(f"狀態寄存器寫入失敗: {result}")
            
        except Exception as e:
            logger.error(f"狀態寄存器更新異常: {e}", exc_info=True)
    
    def _process_control_commands(self):
        """處理控制指令 - 增加調試訊息"""
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
                    logger.info(f"收到新控制指令: {control_command}")
                    logger.debug(f"指令詳情: 地址={self.base_address}, 上次指令={self.last_control_command}, 處理中={self.command_processing}")
                    self._handle_control_command(control_command)
                    self.last_control_command = control_command
                else:
                    logger.warning(f"收到新指令 {control_command} 但系統正在處理指令中，忽略")
            
            # PLC清零指令後恢復Ready
            elif control_command == 0 and self.last_control_command != 0:
                logger.info(f"PLC已清零指令，恢復Ready狀態 (指令變化: {self.last_control_command} → 0)")
                self.state_machine.set_ready(True)
                self.last_control_command = 0
                # 立即更新狀態寄存器
                self._immediate_status_update()
                
        except Exception as e:
            logger.error(f"控制指令處理異常: {e}", exc_info=True)
    
    def _handle_control_command(self, command: int):
        """處理控制指令 - 修正狀態同步問題"""
        if not self.state_machine.is_ready():
            logger.warning(f"系統未Ready，無法執行指令 {command}")
            logger.debug(f"當前狀態: Ready={self.state_machine.is_ready()}, Running={self.state_machine.is_running()}, Alarm={self.state_machine.is_alarm()}")
            return
        
        # 修正：將字典拆分出來，避免 f-string 解析錯誤
        command_mapping = {8: '拍照', 16: '拍照+檢測', 32: '重新初始化'}
        command_desc = command_mapping.get(command, '未知指令')
        logger.info(f"開始處理控制指令: {command} ({command_desc})")
        
        self.command_processing = True
        
        # 關鍵修正：立即設置狀態並更新到Modbus寄存器
        logger.debug("立即設置Running狀態並更新寄存器801")
        self.state_machine.set_ready(False)
        self.state_machine.set_running(True)
        
        # 立即寫入狀態寄存器，確保外部系統能夠看到Running狀態
        self._immediate_status_update()
        
        logger.debug("狀態變更完成: Ready=False, Running=True, 寄存器801已立即更新")
        
        # 異步執行指令
        threading.Thread(target=self._execute_command_async, args=(command,), daemon=True).start()
        logger.debug("已啟動異步執行線程")
    
    def _execute_command_async(self, command: int):
        """異步執行指令 - 修正版：確保狀態同步"""
        try:
            logger.info(f"開始異步執行指令: {command}")
            
            if command == 8:
                # 單純拍照
                logger.info("執行拍照指令")
                if self.camera and getattr(self.camera, 'is_streaming', False):
                    frame_data = self.camera.capture_latest_frame(timeout=3000)
                    if frame_data is not None:
                        logger.info(f"拍照完成，圖像尺寸: {frame_data.data.shape}")
                    else:
                        logger.error("拍照失敗: 無法捕獲圖像")
                        self.error_count += 1
                else:
                    logger.error("拍照失敗: 相機未初始化或串流未啟動")
                    self.error_count += 1
                        
            elif command == 16:
                # 拍照+角度檢測
                logger.info("執行拍照+角度檢測指令")
                
                # 讀取檢測模式 (810) - 默認使用DR模式1
                mode_result = self.modbus_client.read_holding_registers(
                    address=self.base_address + 10, count=1, slave=1
                )
                detection_mode = 1  # 默認DR模式1
                if not mode_result.isError():
                    detection_mode = mode_result.registers[0]
                    logger.debug(f"從寄存器讀取檢測模式: {detection_mode}")
                else:
                    logger.debug(f"寄存器讀取失敗，使用默認檢測模式: {detection_mode}")
                
                logger.info(f"使用檢測模式: {detection_mode} ({'CASE橢圓擬合' if detection_mode == 0 else 'DR最小外接矩形'})")
                
                # 執行檢測
                result = self.capture_and_detect_angle(detection_mode)
                
                # 寫入結果
                logger.debug("準備將檢測結果寫入ModbusTCP Server")
                self.write_detection_result(result)
                
                if result.success:
                    logger.info(f"角度檢測完成: 中心{result.center}, 角度{result.angle:.2f}度, 耗時{result.total_time:.1f}ms")
                else:
                    logger.warning(f"角度檢測失敗: {result.error_message}")
                    
            elif command == 32:
                # 重新初始化
                logger.info("執行重新初始化指令")
                success = self.initialize_camera()
                if success:
                    logger.info("重新初始化成功")
                    # 重新初始化後重新寫入預設參數
                    self.default_params_written = False
                else:
                    logger.error("重新初始化失敗")
            
            else:
                logger.warning(f"未知指令: {command}")
                
        except Exception as e:
            logger.error(f"指令執行發生異常: {e}", exc_info=True)
            self.error_count += 1
            self.state_machine.set_alarm(True)
        
        finally:
            logger.info(f"控制指令 {command} 執行完成")
            
            # 關鍵修正：完成後立即設置狀態並更新寄存器
            logger.debug("設置執行完成狀態並更新寄存器801")
            self.command_processing = False
            self.state_machine.set_running(False)
            
            if not self.state_machine.is_alarm():
                # 注意：這裡不立即設置Ready=True，等待PLC清零指令後再設置
                logger.debug("狀態設置: Running=False, 等待PLC清零指令後設置Ready=True")
            else:
                logger.debug("狀態設置: Running=False, Alarm=True")
            
            # 立即更新狀態寄存器，確保外部系統能夠看到Running=False
            self._immediate_status_update()
            logger.debug("寄存器801已立即更新，外部系統可檢測到Running=False")
    
    def start_handshake_service(self):
        """啟動握手服務 - 修改版：自動寫入預設參數"""
        if not self.handshake_thread or not self.handshake_thread.is_alive():
            self.stop_handshake = False
            self.handshake_thread = threading.Thread(target=self._handshake_sync_loop, daemon=True)
            self.handshake_thread.start()
            logger.info("握手服務已啟動")
            
            # 如果還未寫入預設參數，則自動寫入
            if not self.default_params_written:
                logger.info("自動寫入預設檢測參數")
                success = self.write_default_detection_params()
                if success:
                    logger.info("預設參數自動寫入成功")
                else:
                    logger.warning("預設參數自動寫入失敗，將在下次握手循環中重試")
    
    def stop_handshake_service(self):
        """停止握手服務"""
        logger.info("正在停止握手服務")
        self.stop_handshake = True
        if self.handshake_thread:
            self.handshake_thread.join(timeout=2)
            if self.handshake_thread.is_alive():
                logger.warning("握手線程停止超時")
            else:
                logger.info("握手服務已停止")
    
    def disconnect(self):
        """斷開連接"""
        logger.info("正在斷開所有連接")
        self.stop_handshake_service()
        
        if self.camera:
            logger.info("正在關閉相機連接")
            if getattr(self.camera, 'is_streaming', False):
                logger.debug("停止相機串流")
                self.camera.stop_streaming()
            self.camera.disconnect()
            self.camera = None
            logger.info("相機連接已關閉")
        
        if self.modbus_client:
            logger.info("正在關閉Modbus連接")
            self.modbus_client.close()
            self.modbus_client = None
            logger.info("Modbus連接已關閉")
        
        logger.info("CCD3角度檢測模組已斷開連接")

# Flask Web應用
app = Flask(__name__, template_folder='templates')
app.config['SECRET_KEY'] = 'ccd3_angle_detection_optimized'
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
    
    logger.info(f"Modbus服務器設置更新: {ip}:{port}")
    
    return jsonify({'success': True, 'message': f'Modbus服務器設置為 {ip}:{port}'})

@app.route('/api/modbus/connect', methods=['POST'])
def connect_modbus():
    logger.info("收到Modbus連接請求")
    success = ccd3_service.connect_modbus()
    if success:
        ccd3_service.start_handshake_service()
        logger.info("Modbus連接成功，握手服務已啟動")
        return jsonify({'success': True, 'message': 'Modbus連接成功，握手服務已啟動'})
    else:
        logger.error("Modbus連接失敗")
        return jsonify({'success': False, 'message': 'Modbus連接失敗'})

@app.route('/api/initialize', methods=['POST'])
def initialize_camera():
    data = request.json
    ip = data.get('ip', '192.168.1.10')
    
    logger.info(f"收到相機初始化請求: {ip}")
    success = ccd3_service.initialize_camera(ip)
    message = f'相機初始化{"成功" if success else "失敗"}'
    
    logger.info(message)
    
    return jsonify({'success': success, 'message': message})

@app.route('/api/capture_and_detect', methods=['POST'])
def capture_and_detect():
    data = request.json
    mode = data.get('mode', 0)
    
    logger.info(f"收到檢測請求: mode={mode}")
    result = ccd3_service.capture_and_detect_angle(mode)
    
    # 將numpy類型轉換為Python原生類型
    response_data = {
        'success': result.success,
        'center': [int(result.center[0]), int(result.center[1])] if result.center else None,
        'angle': float(result.angle) if result.angle is not None else None,
        'major_axis': float(result.major_axis) if result.major_axis else None,
        'minor_axis': float(result.minor_axis) if result.minor_axis else None,
        'rect_width': float(result.rect_width) if result.rect_width else None,
        'rect_height': float(result.rect_height) if result.rect_height else None,
        'contour_area': float(result.contour_area) if result.contour_area else None,
        'processing_time': float(result.processing_time),
        'capture_time': float(result.capture_time),
        'total_time': float(result.total_time)
    }
    
    if not result.success:
        response_data['error'] = result.error_message
    
    logger.debug(f"API檢測結果: 成功={result.success}, 耗時={result.total_time:.1f}ms")
    
    return jsonify(response_data)

@app.route('/api/performance_stats', methods=['GET'])
def get_performance_stats():
    """獲取性能統計"""
    stats = ccd3_service.perf_monitor.get_stats()
    logger.debug(f"API性能統計請求: 樣本數={stats.get('sample_count', 0)}")
    return jsonify(stats)

@app.route('/api/debug_images', methods=['GET'])
def get_debug_images():
    """獲取調試圖像列表 - 簡化版"""
    debug_dir = ccd3_service.debug_dir
    
    try:
        if os.path.exists(debug_dir):
            files = os.listdir(debug_dir)
            debug_files = [f for f in files if f.endswith(('.jpg', '.png', '.bmp'))]
            logger.debug(f"API調試圖像請求: 找到{len(debug_files)}個圖像文件")
            return jsonify({
                'images': debug_files,
                'debug_dir': debug_dir,
                'message': f'調試圖像已保存到: {debug_dir}'
            })
        else:
            logger.debug("API調試圖像請求: 調試目錄不存在")
            return jsonify({
                'images': [],
                'debug_dir': debug_dir,
                'message': '調試目錄不存在'
            })
    except Exception as e:
        logger.error(f"API調試圖像請求異常: {e}", exc_info=True)
        return jsonify({'images': [], 'error': str(e)})

@app.route('/api/toggle_debug', methods=['POST'])
def toggle_debug():
    """切換調試模式"""
    data = request.json
    enable = data.get('enable', True)
    
    ccd3_service.debug_enabled = enable
    
    status = "已啟用" if enable else "已關閉"
    logger.info(f"調試圖像保存{status}")
    
    return jsonify({
        'success': True,
        'message': f'調試圖像保存{status}，圖像將保存到: {ccd3_service.debug_dir}',
        'enabled': enable,
        'debug_dir': ccd3_service.debug_dir
    })

@app.route('/api/status', methods=['GET'])
def get_status():
    # 獲取性能統計
    perf_stats = ccd3_service.perf_monitor.get_stats()
    
    status_data = {
        'modbus_connected': ccd3_service.modbus_client and ccd3_service.modbus_client.connected,
        'camera_initialized': ccd3_service.state_machine.is_initialized(),
        'ready': ccd3_service.state_machine.is_ready(),
        'running': ccd3_service.state_machine.is_running(),
        'alarm': ccd3_service.state_machine.is_alarm(),
        'operation_count': ccd3_service.operation_count,
        'error_count': ccd3_service.error_count,
        'connection_count': ccd3_service.connection_count,
        'performance': perf_stats
    }
    
    # 每10次狀態請求記錄一次詳細狀態
    if hasattr(get_status, 'call_count'):
        get_status.call_count += 1
    else:
        get_status.call_count = 1
    
    if get_status.call_count % 10 == 0:
        logger.debug(f"API狀態請求: Ready={status_data['ready']}, 操作數={status_data['operation_count']}, 錯誤數={status_data['error_count']}")
    
    return jsonify(status_data)

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
        
        logger.debug(f"API寄存器讀取: 成功讀取{len(registers)}個寄存器組")
                
    except Exception as e:
        logger.error(f"API寄存器讀取異常: {e}", exc_info=True)
    
    return jsonify(registers)

@socketio.on('connect')
def handle_connect():
    logger.debug("WebSocket客戶端已連接")
    emit('status_update', {'message': 'CCD3角度檢測系統已連接 (狀態同步修正版)'})

@socketio.on('get_status')
def handle_get_status():
    status = get_status().data
    emit('status_update', status)
    logger.debug("WebSocket狀態更新已發送")

def auto_initialize_system():
    logger.info("=== CCD3角度檢測系統自動初始化開始 (狀態同步修正版) ===")
    
    # 1. 自動連接Modbus服務器
    logger.info("步驟1: 自動連接Modbus服務器")
    modbus_success = ccd3_service.connect_modbus()
    if modbus_success:
        logger.info("Modbus服務器連接成功")
        logger.info("握手服務將在相機初始化完成後啟動")
    else:
        logger.error("Modbus服務器連接失敗")
        return False
    
    # 2. 自動連接相機
    logger.info("步驟2: 自動連接相機")
    camera_success = ccd3_service.initialize_camera("192.168.1.10")
    if camera_success:
        logger.info("相機連接成功")
    else:
        logger.error("相機連接失敗")
    
    # 3. 啟動握手服務並自動寫入預設參數
    logger.info("步驟3: 啟動握手服務並寫入預設參數")
    ccd3_service.start_handshake_service()
    logger.info("握手服務已啟動")
    
    # 4. 等待參數寫入完成
    logger.info("步驟4: 等待預設參數寫入完成")
    import time
    for i in range(10):  # 最多等待5秒
        if ccd3_service.default_params_written:
            logger.info("預設參數寫入完成")
            break
        time.sleep(0.5)
        logger.debug(f"等待參數寫入中... ({i+1}/10)")
    
    if not ccd3_service.default_params_written:
        logger.warning("預設參數寫入超時，但系統仍可手動設置")
    
    logger.info("=== CCD3角度檢測系統自動初始化完成 ===")
    logger.info(f"狀態摘要: Ready={ccd3_service.state_machine.is_ready()}, Initialized={ccd3_service.state_machine.is_initialized()}, Alarm={ccd3_service.state_machine.is_alarm()}")
    logger.info(f"預設參數: 已寫入={ccd3_service.default_params_written}")
    logger.info("默認模式: DR模式1 (最小外接矩形角度檢測)")
    logger.info("狀態同步修正: 收到指令後立即更新Running狀態到寄存器801")
    logger.info("狀態同步修正: 指令執行完成後立即更新Running=False到寄存器801")
    logger.info("狀態同步修正: 等待PLC清零指令後才設置Ready=True")
    
    # 強制設置Ready狀態以確保系統可以接收指令
    logger.info("強制設置系統為Ready狀態")
    ccd3_service.state_machine.set_ready(True)
    ccd3_service.state_machine.set_alarm(False)
    # 立即更新到寄存器
    ccd3_service._immediate_status_update()
    logger.info(f"最終狀態: Ready={ccd3_service.state_machine.is_ready()}")
    return True

if __name__ == '__main__':
    logger.info("CCD3角度辨識系統啟動中 (狀態同步修正版)")
    logger.info(f"系統架構: Modbus TCP Client - 運動控制握手模式")
    logger.info(f"基地址: {ccd3_service.base_address}")
    logger.info(f"Modbus服務器: {ccd3_service.server_ip}:{ccd3_service.server_port}")
    logger.info(f"相機IP: 192.168.1.10")
    logger.info(f"檢測模式: 默認DR模式1 (最小外接矩形)，支援CASE模式(0)切換")
    logger.info(f"預設參數: 將自動寫入到寄存器810-815")
    logger.info(f"關鍵修正: 狀態同步問題已修正，Running狀態會立即更新到寄存器801")
    logger.info(f"完整交握: 支援AngleHighLevel.py的完整狀態機交握邏輯")
    
    # 執行自動初始化
    auto_success = auto_initialize_system()
    if auto_success:
        logger.info("系統已就緒，等待PLC指令")
        logger.info("預設使用DR模式1進行角度檢測")
        logger.info("當收到指令16時，將立即設置Running=True並顯示詳細的檢測過程")
        logger.info("檢測完成後，將立即設置Running=False，等待PLC清零指令")
    else:
        logger.warning("系統初始化失敗，但Web介面仍可使用")
    
    logger.info(f"Web介面啟動中... http://localhost:5052")
    
    try:
        socketio.run(app, host='0.0.0.0', port=5052, debug=False)
    except KeyboardInterrupt:
        logger.info("收到中斷信號，正在關閉CCD3角度檢測系統")
        ccd3_service.disconnect()
        logger.info("系統已安全關閉")
    except Exception as e:
        logger.critical(f"系統嚴重錯誤: {e}", exc_info=True)
        ccd3_service.disconnect()
        logger.info("系統已關閉")