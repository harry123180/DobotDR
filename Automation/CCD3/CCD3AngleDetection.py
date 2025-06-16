import sys
import os
import time
import threading
import json
import logging
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

class PerformanceMonitor:
    """性能監控類"""
    def __init__(self):
        self.times = []
        self.capture_times = []
        self.process_times = []
        self.lock = threading.Lock()
        
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
    
    def get_stats(self):
        """獲取性能統計"""
        with self.lock:
            if not self.times:
                return {}
            
            return {
                'avg_total_time': statistics.mean(self.times),
                'avg_capture_time': statistics.mean(self.capture_times),
                'avg_process_time': statistics.mean(self.process_times),
                'min_total_time': min(self.times),
                'max_total_time': max(self.times),
                'sample_count': len(self.times)
            }

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
    
    def update_params(self, **kwargs):
        """更新檢測參數 - 優化：減少不必要的更新"""
        changed = False
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
            print(f"參數已更新：面積比={self.min_area_rate:.3f}, 高斯核={self.gaussian_kernel}, 閾值模式={self.threshold_mode}")
    
    def get_pre_treatment_image_optimized(self, image):
        """優化版影像前處理 - 修正為使用固定閾值210 (參考paste.txt)"""
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
        
        # 修正關鍵：強制使用固定閾值210，不使用OTSU (參考paste.txt)
        _, thresh = cv2.threshold(blur, 210, 255, cv2.THRESH_BINARY)
        print(f"使用固定閾值210進行二值化 (working版本邏輯)")
        
        return thresh
    
    def get_main_contour_optimized(self, image, sequence=False):
        """優化版輪廓檢測 - 修正面積計算邏輯 (參考paste.txt)"""
        # 修正關鍵：直接使用0.05，不從快取讀取以避免參數錯誤
        min_area = image.shape[0] * image.shape[1] * 0.05  # 直接使用paste.txt的邏輯
        print(f"輪廓檢測參數: 圖像尺寸={image.shape}, 最小面積比率=0.05, 最小面積={min_area:.0f}")
        
        # 使用RETR_TREE保持與paste.txt一致
        contours, _ = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return None
        
        # 篩選符合面積要求的輪廓 (完全參考paste.txt邏輯)
        valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
        print(f"檢測到 {len(contours)} 個輪廓，符合面積要求的輪廓數量: {len(valid_contours)}")
        
        if not valid_contours:
            print("警告: 沒有輪廓符合最小面積要求")
            return None
        
        # 根據sequence模式選擇輪廓 (完全參考paste.txt邏輯)
        if sequence:
            # sequence=True: 選擇最後一個輪廓 (paste.txt中CASE模式使用)
            contour = valid_contours[-1]
            print(f"CASE模式: 選擇最後一個輪廓，面積: {cv2.contourArea(contour):.0f}")
        else:
            # sequence=False: 選擇第一個輪廓 (paste.txt中DR模式使用)
            contour = valid_contours[0]
            print(f"DR模式: 選擇第一個輪廓，面積: {cv2.contourArea(contour):.0f}")
        
        return contour
    
    def _detect_angle_dr_mode(self, contour):
        """DR模式角度檢測 - 完全參考paste.txt邏輯"""
        print("執行DR模式角度檢測 (mode=1)")
        
        rect = cv2.minAreaRect(contour)
        center, size, angle = rect
        
        print(f"minAreaRect結果: 中心=({center[0]:.2f}, {center[1]:.2f}), 尺寸=({size[0]:.2f}, {size[1]:.2f}), 角度={angle:.2f}")
        
        # 完全按照paste.txt: 直接使用rect[2]的角度，不做任何修正
        corrected_angle = angle  # paste.txt: angle = rect[2]
        
        # 中心點轉換: 完全按照paste.txt邏輯
        center_int = (int(center[0]), int(center[1]))  # paste.txt: center = tuple(np.int_(rect[0]))
        
        extra_data = {
            'rect_width': size[0],
            'rect_height': size[1]
        }
        
        print(f"DR模式最終結果: 中心={center_int}, 角度={corrected_angle:.2f}度")
        return center_int, corrected_angle, extra_data
    
    def _detect_angle_case_mode(self, contour, original_image):
        """CASE模式角度檢測 - 使用橢圓擬合複雜邏輯 (參考paste.txt mode=0)"""
        if len(contour) < 5:
            return None
        
        try:
            # 建立遮罩
            mask_1 = np.zeros((original_image.shape[0], original_image.shape[1]), dtype=np.uint8)
            mask_2 = np.zeros((original_image.shape[0], original_image.shape[1]), dtype=np.uint8)
            
            # 填充輪廓
            cv2.drawContours(mask_1, [contour], -1, (255, 255, 255), -1)
            
            # 橢圓擬合
            ellipse = cv2.fitEllipse(contour)
            (x, y), (MA, ma), angle = ellipse
            
            center = (int(x), int(y))
            
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
            
            return center_int, final_angle, extra_data
            
        except cv2.error as e:
            print(f"CASE模式檢測錯誤: {e}")
            return None
    
    def detect_angle(self, image, mode=0) -> AngleResult:
        """優化版角度檢測主函數 - 加入調試圖像保存"""
        start_time = time.perf_counter()  # 優化7：使用高精度計時器
        
        # 準備調試圖像變量
        original_image = None
        binary_image = None
        result_image = None
        
        try:
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
                # CASE模式：使用sequence=True (參考paste.txt)
                rst_contour = self.get_main_contour_optimized(pt_img, sequence=True)
            else:
                # DR模式：使用sequence=False (參考paste.txt)
                rst_contour = self.get_main_contour_optimized(pt_img, sequence=False)
            
            # 準備結果圖像
            result_image = bgr_image.copy()
            
            if rst_contour is None:
                # 在結果圖像上標註失敗訊息
                cv2.putText(result_image, "No Valid Contour Found", (50, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                return AngleResult(
                    success=False, center=None, angle=None,
                    major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                    contour_area=None, processing_time=0, capture_time=0,
                    total_time=(time.perf_counter() - start_time) * 1000,
                    error_message="未檢測到有效輪廓"
                )
            
            contour_area = cv2.contourArea(rst_contour)
            print(f"檢測到輪廓面積: {contour_area:.0f} 像素")
            
            # 降低面積檢查閾值，原本100太大
            min_area_threshold = 50  # 降低閾值
            if contour_area < min_area_threshold:
                # 在結果圖像上標註面積太小
                cv2.putText(result_image, f"Area Too Small: {contour_area:.0f} < {min_area_threshold}", 
                           (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                # 仍然畫出檢測到的輪廓
                cv2.drawContours(result_image, [rst_contour], -1, (255, 0, 0), 2)
                
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
        
        # 配置檔案
        self.config_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ccd3_config.json')
        self.load_config()
    
    def _ensure_debug_dir(self):
        """確保調試圖像目錄存在"""
        if not os.path.exists(self.debug_dir):
            os.makedirs(self.debug_dir)
            print(f"已創建調試圖像目錄: {self.debug_dir}")
    
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
            print(f"調試圖像已保存 (檢測{status}): {self.debug_dir}")
            
        except Exception as e:
            print(f"保存調試圖像失敗: {e}")
    
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
                
                print("啟動相機串流...")
                if self.camera.start_streaming():
                    print("相機串流啟動成功")
                    
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
        """優化版拍照並檢測角度 - 加入調試圖像保存"""
        if not self.camera:
            return AngleResult(
                success=False, center=None, angle=None,
                major_axis=None, minor_axis=None, rect_width=None, rect_height=None,
                contour_area=None, processing_time=0, capture_time=0, total_time=0,
                error_message="相機未初始化"
            )
        
        capture_start = time.perf_counter()  # 高精度計時
        
        try:
            # 優化13：移除不必要的日誌輸出，只保留關鍵訊息
            frame_data = self.camera.capture_frame()
            
            if frame_data is None:
                raise Exception("圖像捕獲失敗")
            
            image = frame_data.data
            capture_time = (time.perf_counter() - capture_start) * 1000
            
            # 優化14：參數快取機制
            detection_params = self.read_detection_parameters_cached()
            if detection_params and self._params_changed:
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
                
                def detect_angle(self, image, mode=0):
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
                    print(f"性能統計(最近{stats.get('sample_count', 0)}次): 平均總時間={stats.get('avg_total_time', 0):.1f}ms, 平均處理時間={stats.get('avg_process_time', 0):.1f}ms")
            else:
                self.error_count += 1
                print(f"檢測失敗: {result.error_message}")
            
            return result
            
        except Exception as e:
            self.error_count += 1
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
                error_image = image.copy()
                cv2.putText(error_image, f"ERROR: {str(e)[:50]}", (50, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                try:
                    self.save_debug_images(image, 
                                         np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8),
                                         error_image, False)
                except:
                    pass
            
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
                        self._last_params = current_params.copy()
                        self._params_changed = True
                        params = current_params
                    
        except Exception as e:
            print(f"讀取檢測參數錯誤: {e}")
        
        return params
    
    def write_detection_result(self, result: AngleResult):
        """優化版結果寫入 - 批次寫入減少通訊次數"""
        try:
            if not self.modbus_client or not self.modbus_client.connected:
                return
            
            # 優化16：一次性準備所有寄存器數據
            all_registers = [0] * 40  # 結果區(20) + 統計區(20)
            
            # 檢測結果區 (840-859對應0-19)
            if result.success and result.center and result.angle is not None:
                all_registers[0] = 1  # 成功標誌
                all_registers[1] = int(result.center[0])  # X座標
                all_registers[2] = int(result.center[1])  # Y座標
                
                # 角度32位存儲
                angle_int = int(result.angle * 100)
                all_registers[3] = (angle_int >> 16) & 0xFFFF
                all_registers[4] = angle_int & 0xFFFF
                
                # 額外參數
                if result.major_axis:
                    all_registers[5] = int(result.major_axis)
                if result.minor_axis:
                    all_registers[6] = int(result.minor_axis)
                if result.rect_width:
                    all_registers[7] = int(result.rect_width)
                if result.rect_height:
                    all_registers[8] = int(result.rect_height)
                if result.contour_area:
                    all_registers[9] = int(result.contour_area)
            
            # 統計資訊區 (880-899對應20-39)
            all_registers[20] = int(result.capture_time)
            all_registers[21] = int(result.processing_time)
            all_registers[22] = int(result.total_time)
            all_registers[23] = self.operation_count
            all_registers[24] = self.error_count
            all_registers[25] = self.connection_count
            all_registers[30] = 3  # 版本號
            all_registers[31] = 1  # 次版本號(優化版)
            all_registers[32] = int((time.time() - self.start_time) // 3600)  # 運行小時
            all_registers[33] = int((time.time() - self.start_time) % 3600 // 60)  # 運行分鐘
            
            # 優化17：批次寫入減少Modbus通訊次數
            self.modbus_client.write_registers(
                address=self.base_address + 40, values=all_registers, slave=1
            )
            
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
            # 更新初始化狀態
            camera_ok = self.camera is not None and getattr(self.camera, 'is_streaming', False)
            modbus_ok = self.modbus_client is not None and self.modbus_client.connected
            
            self.state_machine.set_initialized(camera_ok)
            if not (camera_ok and modbus_ok):
                if not camera_ok:
                    self.state_machine.set_alarm(True)
            
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
                    print(f"角度檢測完成: 中心{result.center}, 角度{result.angle:.2f}度, 耗時{result.total_time:.1f}ms")
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
            if getattr(self.camera, 'is_streaming', False):
                print("停止相機串流...")
                self.camera.stop_streaming()
            self.camera.disconnect()
            self.camera = None
        
        if self.modbus_client:
            print("正在關閉Modbus連接...")
            self.modbus_client.close()
            self.modbus_client = None
        
        print("CCD3角度檢測模組已斷開連接")

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
    
    return jsonify(response_data)

@app.route('/api/performance_stats', methods=['GET'])
def get_performance_stats():
    """獲取性能統計"""
    stats = ccd3_service.perf_monitor.get_stats()
    return jsonify(stats)

@app.route('/api/debug_images', methods=['GET'])
def get_debug_images():
    """獲取調試圖像列表 - 簡化版"""
    debug_dir = ccd3_service.debug_dir
    
    try:
        if os.path.exists(debug_dir):
            files = os.listdir(debug_dir)
            debug_files = [f for f in files if f.endswith(('.jpg', '.png', '.bmp'))]
            return jsonify({
                'images': debug_files,
                'debug_dir': debug_dir,
                'message': f'調試圖像已保存到: {debug_dir}'
            })
        else:
            return jsonify({
                'images': [],
                'debug_dir': debug_dir,
                'message': '調試目錄不存在'
            })
    except Exception as e:
        return jsonify({'images': [], 'error': str(e)})

@app.route('/api/toggle_debug', methods=['POST'])
def toggle_debug():
    """切換調試模式"""
    data = request.json
    enable = data.get('enable', True)
    
    ccd3_service.debug_enabled = enable
    
    status = "已啟用" if enable else "已關閉"
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
    
    return jsonify({
        'modbus_connected': ccd3_service.modbus_client and ccd3_service.modbus_client.connected,
        'camera_initialized': ccd3_service.state_machine.is_initialized(),
        'ready': ccd3_service.state_machine.is_ready(),
        'running': ccd3_service.state_machine.is_running(),
        'alarm': ccd3_service.state_machine.is_alarm(),
        'operation_count': ccd3_service.operation_count,
        'error_count': ccd3_service.error_count,
        'connection_count': ccd3_service.connection_count,
        'performance': perf_stats
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
    emit('status_update', {'message': 'CCD3角度檢測系統已連接 (優化版)'})

@socketio.on('get_status')
def handle_get_status():
    status = get_status().data
    emit('status_update', status)

def auto_initialize_system():
    """系統自動初始化"""
    print("=== CCD3角度檢測系統自動初始化開始 (優化版) ===")
    
    # 1. 自動連接Modbus服務器
    print("步驟1: 自動連接Modbus服務器...")
    modbus_success = ccd3_service.connect_modbus()
    if modbus_success:
        print("✓ Modbus服務器連接成功")
        # 啟動握手服務
        ccd3_service.start_handshake_service()
        print("✓ 握手服務已啟動")
    else:
        print("✗ Modbus服務器連接失敗")
        return False
    
    # 2. 自動連接相機
    print("步驟2: 自動連接相機...")
    camera_success = ccd3_service.initialize_camera("192.168.1.10")
    if camera_success:
        print("✓ 相機連接成功")
    else:
        print("✗ 相機連接失敗")
        return False
    
    print("=== CCD3角度檢測系統自動初始化完成 ===")
    print(f"狀態: Ready={ccd3_service.state_machine.is_ready()}")
    print(f"狀態: Initialized={ccd3_service.state_machine.is_initialized()}")
    print(f"狀態: Alarm={ccd3_service.state_machine.is_alarm()}")
    print("性能優化: 啟用快取機制、批次寫入、高精度計時")
    return True

if __name__ == '__main__':
    print("CCD3角度辨識系統啟動中 (性能優化版)...")
    print(f"系統架構: Modbus TCP Client - 運動控制握手模式")
    print(f"基地址: {ccd3_service.base_address}")
    print(f"Modbus服務器: {ccd3_service.server_ip}:{ccd3_service.server_port}")
    print(f"相機IP: 192.168.1.10")
    print(f"檢測模式: 支援CASE模式(0)和DR模式(1)")
    print(f"性能優化: 圖像處理快取、參數快取、批次寫入、高精度計時")
    
    # 執行自動初始化
    auto_success = auto_initialize_system()
    if auto_success:
        print("系統已就緒，等待PLC指令...")
    else:
        print("系統初始化失敗，但Web介面仍可使用")
    
    print(f"Web介面啟動中... http://localhost:5052")
    
    try:
        socketio.run(app, host='0.0.0.0', port=5052, debug=False)
    except KeyboardInterrupt:
        print("\n正在關閉CCD3角度檢測系統...")
        ccd3_service.disconnect()
    except Exception as e:
        print(f"系統錯誤: {e}")
        ccd3_service.disconnect()