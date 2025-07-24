# -*- coding: utf-8 -*-
"""
VisionModule.py - 極致簡潔CCD1/CCD3視覺模組與Modbus整合
統合CCD1 YOLO檢測與CCD3角度檢測，提供Modbus通訊接口
"""

import os
import sys
import time
import threading
import logging
from typing import Dict, List, Tuple, Optional, Any
from datetime import datetime, timedelta

# Modbus客戶端
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException, ConnectionException

# 導入CCD模組
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.append(os.path.join(project_root, 'CCD1'))
sys.path.append(os.path.join(project_root, 'CCD3'))
sys.path.append(current_dir)

from CCD1_Simple_Vison_System import CCD1VisionSystem, CCD1VisionConfig
from CCD3_Headless_Vision_System import CCD3HeadlessDetector, DetectionMethod

# ==================== LOG管理器 ====================
class VMLogger:
    """簡化LOG管理器"""
    
    def __init__(self, name: str = "VM"):
        self.name = name
        self.logger = logging.getLogger(name)
        self.logger.setLevel(logging.DEBUG)
        
        # 確保只有一個handler
        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)
        
        self.log_level = 0  # 0=不記錄, 1=INFO, 2=WARNING, 3=ERROR
        self.silent_mode = False
    
    def set_level(self, level: int):
        """設置LOG等級"""
        self.log_level = level
    
    def set_silent(self, silent: bool):
        """設置靜默模式"""
        self.silent_mode = silent
    
    def info(self, msg: str):
        if not self.silent_mode and self.log_level >= 1:
            self.logger.info(msg)
    
    def warning(self, msg: str):
        if not self.silent_mode and self.log_level >= 2:
            self.logger.warning(msg)
    
    def error(self, msg: str):
        if not self.silent_mode and self.log_level >= 3:
            self.logger.error(msg)

# ==================== Modbus客戶端管理器 ====================
class VMModbusClient:
    """VM專用Modbus客戶端"""
    
    def __init__(self, host="127.0.0.1", port=502):
        self.host = host
        self.port = port
        self.client = None
        self.connected = False
        self._connect()
    
    def _connect(self):
        """連接到Modbus服務器"""
        try:
            self.client = ModbusTcpClient(self.host, port=self.port)
            self.connected = self.client.connect()
            if self.connected:
                print(f"Modbus客戶端連接成功: {self.host}:{self.port}")
            else:
                print(f"Modbus客戶端連接失敗: {self.host}:{self.port}")
        except Exception as e:
            print(f"Modbus連接異常: {e}")
            self.connected = False
    
    def reconnect(self):
        """重新連接"""
        if self.client:
            self.client.close()
        self._connect()
    
    def read_holding_register(self, address: int) -> int:
        """讀取保持寄存器"""
        if not self.connected:
            self.reconnect()
        
        try:
            response = self.client.read_holding_registers(address, 1)
            if response.isError():
                return 0
            return response.registers[0]
        except Exception:
            return 0
    
    def write_holding_register(self, address: int, value: int) -> bool:
        """寫入保持寄存器"""
        if not self.connected:
            self.reconnect()
        
        try:
            response = self.client.write_register(address, value)
            return not response.isError()
        except Exception:
            return False
    
    def close(self):
        """關閉連接"""
        if self.client:
            self.client.close()
            self.connected = False

# ==================== 主要視覺模組 ====================
class VisionModule:
    """極致簡潔視覺模組"""
    
    def __init__(self):
        # LOG管理
        self.ccd1_logger = VMLogger("CCD1")
        self.ccd3_logger = VMLogger("CCD3")
        
        # 核心組件
        self.ccd1: Optional[CCD1VisionSystem] = None
        self.ccd3: Optional[CCD3HeadlessDetector] = None
        
        # Modbus客戶端
        self.modbus = VMModbusClient()
        
        # 內部變數
        self.ccd1_detections = {}  # 存儲檢測結果 {class_id: [(x, y, world_x, world_y), ...]}
        self.ccd3_angle = 0.0
        self.ccd3_has_contour = False
        
        # 雙執行緒架構的緩存
        self.current_values = {}      # Modbus同步執行緒更新的當前值
        self.last_processed_values = {}  # 主處理執行緒記錄的上次處理值
        
        # 運行控制
        self.running = False
        self.main_thread = None
        self.sync_thread = None
        
        # 初始化預設值
        self._write_default_values()
        
        # 等待寫入完成後清空緩存
        time.sleep(0.1)
        self.current_values.clear()
        self.last_processed_values.clear()
        
        # 設置運行標志
        self.running = True
        
        # 啟動主循環（包含Modbus讀取和邏輯處理）
        self._start_main_loop()
    
    def _write_default_values(self):
        """寫入所有預設值到Modbus"""
        print("正在寫入VM預設值...")
        
        try:
            # 基本狀態初始化
            self.modbus.write_holding_register(100, 0)   # CCD1狀態: 未初始化
            self.modbus.write_holding_register(101, 0)   # CCD3狀態: 未初始化
            self.modbus.write_holding_register(102, 0)   # CCD1初始化控制: 停止
            self.modbus.write_holding_register(103, 0)   # CCD3初始化控制: 停止
            
            # CCD1預設值
            self.modbus.write_holding_register(104, 1)   # CCD1選用模型: 預設模型1
            self.modbus.write_holding_register(105, 800) # CCD1模型置信度: 預設0.8 (*1000)
            self.modbus.write_holding_register(106, 0)   # CCD1標定文件狀態: 未載入
            
            # CCD3預設值  
            self.modbus.write_holding_register(107, 0)   # CCD3檢測方法: 預設0(DR模式)
            
            # LOG控制預設值
            self.modbus.write_holding_register(108, 0)   # CCD1Log控制: 預設0(不記錄)
            self.modbus.write_holding_register(109, 0)   # CCD3Log控制: 預設0(不記錄)
            
            # CCD1檢測數量初始化
            self.modbus.write_holding_register(110, 0)   # CCD1標籤0數量
            self.modbus.write_holding_register(111, 0)   # CCD1標籤1數量  
            self.modbus.write_holding_register(112, 0)   # CCD1標籤2數量
            
            # CCD1物件請求相關初始化
            self.modbus.write_holding_register(113, 0)   # 保留
            self.modbus.write_holding_register(114, 0)   # 保留
            self.modbus.write_holding_register(115, 0)   # 保留
            self.modbus.write_holding_register(116, 0)   # CCD1物件請求: 停止
            self.modbus.write_holding_register(117, 0)   # CCD1物件編號: 0
            self.modbus.write_holding_register(118, 0)   # CCD1標籤編號: 0
            
            # 圖像存儲控制預設值
            self.modbus.write_holding_register(119, 0)   # CCD1原始存圖: 預設0(不存)
            self.modbus.write_holding_register(120, 0)   # YOLO圖: 預設0(不存)
            self.modbus.write_holding_register(121, 0)   # CCD3原始圖: 預設0(不存)
            self.modbus.write_holding_register(122, 0)   # 二值化圖: 預設0(不存)
            self.modbus.write_holding_register(123, 0)   # 可視化圖: 預設0(不存)
            
            # CCD3相關初始化
            self.modbus.write_holding_register(124, 0)   # CCD3檢測請求: 停止
            self.modbus.write_holding_register(125, 0)   # CCD1物件世界座標X: 0
            self.modbus.write_holding_register(126, 0)   # CCD1物件世界座標Y: 0
            
            # 靜默模式預設值
            self.modbus.write_holding_register(127, 0)   # CCD1靜默: 預設0(非靜默)
            self.modbus.write_holding_register(128, 0)   # CCD3靜默: 預設0(非靜默)
            
            # CCD3檢測結果初始化
            self.modbus.write_holding_register(129, 0)   # CCD3角度: 0
            self.modbus.write_holding_register(130, 0)   # CCD3有無有效輪廓: 0(無)
            
            print("VM預設值寫入完成")
            print("  - CCD1模型: 1, 置信度: 0.8")
            print("  - CCD3檢測方法: DR模式") 
            print("  - LOG等級: 不記錄")
            print("  - 圖像存儲: 全部關閉")
            print("  - 靜默模式: 關閉")
            
        except Exception as e:
            print(f"寫入預設值失敗: {e}")
    
    def _start_main_loop(self):
        """啟動主循環"""
        self.main_thread = threading.Thread(target=self._main_loop, daemon=True)
        self.main_thread.start()
        print("VM主循環已啟動")
    
    def _main_loop(self):
        """主循環 - 整合Modbus讀取和邏輯處理"""
        print("VM主循環開始，50ms掃描週期")
        
        while self.running:
            try:
                start_time = time.time()
                
                # 1. 讀取Modbus當前值到緩存
                self._read_modbus_values()
                
                # 2. 更新系統狀態到Modbus  
                self._update_status()
                
                # 3. 處理控制信號變化
                self._process_control_signals()
                
                # 執行時間統計
                loop_time = (time.time() - start_time) * 1000
                if loop_time > 10:  # 超過10ms才記錄
                    print(f"VM主循環耗時: {loop_time:.1f}ms")
                
                time.sleep(0.05)  # 50ms週期
                
            except Exception as e:
                print(f"主循環異常: {e}")
                time.sleep(0.1)
    
    def _read_modbus_values(self):
        """讀取Modbus值到緩存"""
        scan_addresses = [102, 103, 104, 105, 107, 108, 109, 116, 117, 118, 
                         119, 120, 121, 122, 123, 124, 127, 128]
        
        try:
            for address in scan_addresses:
                current_value = self.modbus.read_holding_register(address)
                self.current_values[address] = current_value
                
        except Exception as e:
            print(f"讀取Modbus值異常: {e}")
    
    def _update_status(self):
        """更新系統狀態到Modbus"""
        # 更新CCD1狀態
        if self.ccd1 is None:
            self.modbus.write_holding_register(100, 0)  # 未初始化
        elif not self.ccd1.camera_connected:
            self.modbus.write_holding_register(100, 99)  # 斷開連接
        elif not self.ccd1.initialized:
            self.modbus.write_holding_register(100, 12)  # 錯誤
        else:
            self.modbus.write_holding_register(100, 9)   # 準備好
        
        # 更新CCD3狀態
        if self.ccd3 is None:
            self.modbus.write_holding_register(101, 0)   # 未初始化
        elif not self.ccd3.is_initialized:
            self.modbus.write_holding_register(101, 99)  # 斷開連接
        else:
            self.modbus.write_holding_register(101, 9)   # 準備好
        
        # 更新標定文件狀態
        calibration_status = 0
        if (self.ccd1 and 
            hasattr(self.ccd1, 'calibration_manager') and 
            self.ccd1.calibration_manager and
            hasattr(self.ccd1.calibration_manager, 'calibration_loaded') and
            self.ccd1.calibration_manager.calibration_loaded and
            hasattr(self.ccd1.calibration_manager, 'transformer') and
            self.ccd1.calibration_manager.transformer and
            self.ccd1.calibration_manager.transformer.is_valid()):
            calibration_status = 1
            
        self.modbus.write_holding_register(106, calibration_status)
    
    def _process_control_signals(self):
        """處理控制信號變化"""
        # 定義控制地址映射
        control_addresses = {
            102: 'ccd1_init_control',
            103: 'ccd3_init_control', 
            104: 'ccd1_model_select',
            105: 'ccd1_confidence',
            107: 'ccd3_detection_method',
            108: 'ccd1_log_control',
            109: 'ccd3_log_control',
            116: 'ccd1_object_request',
            117: 'ccd1_object_number',
            118: 'ccd1_label_number',
            119: 'ccd1_raw_image_save',
            120: 'ccd1_yolo_image_save',
            121: 'ccd3_raw_image_save',
            122: 'ccd3_binary_image_save',
            123: 'ccd3_result_image_save',
            124: 'ccd3_detection_request',
            127: 'ccd1_silent_mode',
            128: 'ccd3_silent_mode'
        }
        
        for address, name in control_addresses.items():
            try:
                current_value = self.current_values.get(address, 0)
                last_value = self.last_processed_values.get(address, -999)
                
                # 調試信息：每10秒輸出一次關鍵地址的值
                if address in [102, 103, 116, 124]:
                    if not hasattr(self, '_last_debug_time'):
                        self._last_debug_time = time.time()
                    elif time.time() - self._last_debug_time > 10:
                        print(f"調試 - 地址{address}({name}): 當前值={current_value}, 上次處理值={last_value}")
                        if address == 102:
                            self._last_debug_time = time.time()
                
                # 觸發型信號：值為1或2時立即處理，不管上次處理值
                if address in [102, 103, 124] and current_value == 1:
                    print(f"檢測到觸發信號 地址{address}({name}) = {current_value}")
                    self._handle_control_signal(address, current_value)
                    self.last_processed_values[address] = current_value
                elif address == 116 and current_value in [1, 2]:
                    print(f"檢測到CCD1物件請求信號 地址{address}({name}) = {current_value}")
                    self._handle_control_signal(address, current_value)
                    self.last_processed_values[address] = current_value
                # 配置型信號：值變化時處理
                elif current_value != last_value:
                    print(f"檢測到地址{address}({name})變化: {last_value} -> {current_value}")
                    self._handle_control_signal(address, current_value)
                    self.last_processed_values[address] = current_value
                    
            except Exception as e:
                print(f"處理地址{address}異常: {e}")
    
    def _handle_control_signal(self, address: int, value: int):
        """處理具體控制信號"""
        try:
            if address == 102 and value == 1:  # CCD1初始化控制
                print("收到CCD1初始化請求")
                self._init_ccd1()
                self.modbus.write_holding_register(102, 0)  # 重置控制信號
                
            elif address == 103 and value == 1:  # CCD3初始化控制
                print("收到CCD3初始化請求")
                self._init_ccd3()
                self.modbus.write_holding_register(103, 0)  # 重置控制信號
                
            elif address == 104:  # CCD1選用模型
                if self.ccd1 and value > 0:
                    print(f"CCD1切換模型請求: 模型{value}")
                    self.modbus.write_holding_register(100, 8)  # 運作中
                    if self.ccd1.switch_yolo_model(value):
                        self.ccd1_logger.info(f"成功切換到模型{value}")
                        self.modbus.write_holding_register(100, 9)  # 準備好
                    else:
                        self.ccd1_logger.error(f"切換模型{value}失敗")
                        self.modbus.write_holding_register(100, 12)  # 錯誤
                    
            elif address == 105:  # CCD1模型置信度
                if self.ccd1 and value > 0:
                    confidence = value / 1000.0  # 800 -> 0.8
                    if hasattr(self.ccd1, 'update_confidence_threshold'):
                        self.ccd1.update_confidence_threshold(confidence)
                        self.ccd1_logger.info(f"置信度更新為{confidence}")
                    
            elif address == 107:  # CCD3檢測方法
                if self.ccd3:
                    method = DetectionMethod.DR_MIN_RECT if value == 0 else DetectionMethod.CASE_ELLIPSE
                    self.ccd3.set_detection_params(detection_method=method)
                    self.ccd3_logger.info(f"檢測方法切換為{'DR' if value == 0 else 'CASE'}")
                    
            elif address == 108:  # CCD1Log控制
                self.ccd1_logger.set_level(value)
                if value > 0:
                    self.ccd1_logger.info(f"CCD1 LOG等級設為{value}")
                    
            elif address == 109:  # CCD3Log控制
                self.ccd3_logger.set_level(value)
                if value > 0:
                    self.ccd3_logger.info(f"CCD3 LOG等級設為{value}")
                    
            elif address == 116:  # CCD1物件請求
                if value == 1:
                    print("收到CCD1物件座標查詢請求")
                    self._handle_ccd1_coordinate_query()
                elif value == 2:
                    print("收到CCD1重新檢測+查詢請求")
                    self._handle_ccd1_redetect_and_query()
                    
            elif address == 124 and value == 1:  # CCD3檢測請求
                print("收到CCD3檢測請求")
                self._process_ccd3_detection()
                self.modbus.write_holding_register(124, 0)  # 重置觸發信號
                
            elif address == 119:  # CCD1原始存圖
                if self.ccd1 and hasattr(self.ccd1, 'update_image_save_settings'):
                    self.ccd1.update_image_save_settings(save_raw=(value == 1))
                    self.ccd1_logger.info(f"CCD1原始圖存儲: {'開啟' if value == 1 else '關閉'}")
                    
            elif address == 120:  # YOLO圖
                if self.ccd1 and hasattr(self.ccd1, 'update_image_save_settings'):
                    self.ccd1.update_image_save_settings(save_result=(value == 1))
                    self.ccd1_logger.info(f"CCD1 YOLO圖存儲: {'開啟' if value == 1 else '關閉'}")
                    
            elif address == 121:  # CCD3原始圖
                if self.ccd3 and hasattr(self.ccd3, 'set_image_save_options'):
                    self.ccd3.set_image_save_options(save_original=(value == 1))
                    self.ccd3_logger.info(f"CCD3原始圖存儲: {'開啟' if value == 1 else '關閉'}")
                    
            elif address == 122:  # 二值化圖
                if self.ccd3 and hasattr(self.ccd3, 'set_image_save_options'):
                    self.ccd3.set_image_save_options(save_processed=(value == 1))
                    self.ccd3_logger.info(f"CCD3二值化圖存儲: {'開啟' if value == 1 else '關閉'}")
                    
            elif address == 123:  # 可視化圖
                if self.ccd3 and hasattr(self.ccd3, 'set_image_save_options'):
                    self.ccd3.set_image_save_options(save_result=(value == 1))
                    self.ccd3_logger.info(f"CCD3可視化圖存儲: {'開啟' if value == 1 else '關閉'}")
                    
            elif address == 127:  # CCD1靜默
                self.ccd1_logger.set_silent(value == 1)
                print(f"CCD1靜默模式: {'開啟' if value == 1 else '關閉'}")
                
            elif address == 128:  # CCD3靜默
                self.ccd3_logger.set_silent(value == 1)
                print(f"CCD3靜默模式: {'開啟' if value == 1 else '關閉'}")
                
        except Exception as e:
            print(f"處理控制信號異常 地址{address} 值{value}: {e}")
    
    def _init_ccd1(self):
        """初始化CCD1"""
        try:
            print("開始CCD1初始化...")
            self.modbus.write_holding_register(100, 8)  # 狀態: 運作中
            
            config = CCD1VisionConfig(
                camera_ip="192.168.1.8",
                confidence_threshold=0.8,
                num_classes=3,
                class_names={0: "DR_F", 1: "STACK", 2: "DR_B"},
                enable_world_coord=True,
                save_raw_image=False,
                save_result_image=False,
                save_dir=os.path.dirname(__file__)
            )
            
            self.ccd1 = CCD1VisionSystem(config)
            
            # 檢查初始化結果
            if self.ccd1.initialized and self.ccd1.camera_connected:
                self.modbus.write_holding_register(100, 9)  # 狀態: 準備好
                self.ccd1_logger.info("CCD1初始化成功")
                print("CCD1初始化成功 - 狀態更新為準備好(9)")
                self._update_calibration_status()
            elif self.ccd1.initialized and not self.ccd1.camera_connected:
                self.modbus.write_holding_register(100, 99)  # 狀態: 相機斷線
                self.ccd1_logger.error("CCD1初始化成功但相機未連接")
                print("CCD1初始化成功但相機未連接 - 狀態更新為斷線(99)")
            else:
                self.modbus.write_holding_register(100, 12)  # 狀態: 錯誤
                self.ccd1_logger.error("CCD1初始化失敗")
                print("CCD1初始化失敗 - 狀態更新為錯誤(12)")
                
        except Exception as e:
            self.modbus.write_holding_register(100, 12)  # 狀態: 錯誤
            self.ccd1_logger.error(f"CCD1初始化異常: {e}")
            print(f"CCD1初始化異常: {e} - 狀態更新為錯誤(12)")
    
    def _init_ccd3(self):
        """初始化CCD3"""
        try:
            print("開始CCD3初始化...")
            self.modbus.write_holding_register(101, 8)  # 狀態: 運作中
            
            self.ccd3 = CCD3HeadlessDetector(camera_ip="192.168.1.10")
            
            # 檢查初始化結果
            if self.ccd3.initialize_camera():
                self.modbus.write_holding_register(101, 9)  # 狀態: 準備好
                self.ccd3_logger.info("CCD3初始化成功")
                print("CCD3初始化成功 - 狀態更新為準備好(9)")
            else:
                if hasattr(self.ccd3, 'camera') and self.ccd3.camera:
                    self.modbus.write_holding_register(101, 99)  # 狀態: 相機斷線
                    self.ccd3_logger.error("CCD3相機連接失敗")
                    print("CCD3相機連接失敗 - 狀態更新為斷線(99)")
                else:
                    self.modbus.write_holding_register(101, 12)  # 狀態: 錯誤
                    self.ccd3_logger.error("CCD3初始化失敗")
                    print("CCD3初始化失敗 - 狀態更新為錯誤(12)")
                
        except Exception as e:
            self.modbus.write_holding_register(101, 12)  # 狀態: 錯誤
            self.ccd3_logger.error(f"CCD3初始化異常: {e}")
            print(f"CCD3初始化異常: {e} - 狀態更新為錯誤(12)")
    
    def _update_calibration_status(self):
        """更新標定文件狀態"""
        calibration_status = 0
        
        try:
            if (self.ccd1 and 
                hasattr(self.ccd1, 'calibration_manager') and 
                self.ccd1.calibration_manager and
                hasattr(self.ccd1.calibration_manager, 'calibration_loaded') and
                self.ccd1.calibration_manager.calibration_loaded and
                hasattr(self.ccd1.calibration_manager, 'transformer') and
                self.ccd1.calibration_manager.transformer and
                self.ccd1.calibration_manager.transformer.is_valid()):
                
                calibration_status = 1
                print("標定文件檢查: 內外參相機畸變參數文件已載入 - 狀態更新為1")
            else:
                print("標定文件檢查: 內外參相機畸變參數文件未載入或無效 - 狀態保持為0")
                
        except Exception as e:
            print(f"標定文件狀態檢查異常: {e}")
            
        self.modbus.write_holding_register(106, calibration_status)
    
    def _handle_ccd1_coordinate_query(self):
        """處理CCD1物件座標查詢請求 (使用現有檢測結果)"""
        if not self.ccd1 or not self.ccd1.initialized:
            self.ccd1_logger.warning("CCD1未初始化，無法查詢座標")
            self.modbus.write_holding_register(100, 12)
            self.modbus.write_holding_register(116, 0)
            return
        
        try:
            self.modbus.write_holding_register(100, 8)  # 運作中
            
            # 從緩存讀取請求參數
            obj_num = self.current_values.get(117, 0)  # 物件編號 (1-based)
            class_id = self.current_values.get(118, 0)  # 標籤編號 (0-based)
            
            self.ccd1_logger.info(f"查詢座標: 標籤{class_id} 物件編號{obj_num}")
            
            # 檢查是否有檢測結果
            if not self.ccd1_detections:
                self.ccd1_logger.error("無檢測結果，請先執行檢測")
                self.modbus.write_holding_register(100, 12)
                self.modbus.write_holding_register(125, 0)
                self.modbus.write_holding_register(126, 0)
                self.modbus.write_holding_register(116, 0)
                return
            
            # 檢查指定標籤是否存在
            if class_id not in self.ccd1_detections or len(self.ccd1_detections[class_id]) == 0:
                self.ccd1_logger.error(f"標籤{class_id}無檢測結果")
                self.modbus.write_holding_register(100, 12)
                self.modbus.write_holding_register(125, 0)
                self.modbus.write_holding_register(126, 0)
                self.modbus.write_holding_register(116, 0)
                return
            
            detections = self.ccd1_detections[class_id]
            
            # 檢查物件編號是否有效
            if obj_num < 1 or obj_num > len(detections):
                self.ccd1_logger.error(f"物件編號{obj_num}超出範圍，標籤{class_id}共有{len(detections)}個物件")
                self.modbus.write_holding_register(100, 12)
                self.modbus.write_holding_register(125, 0)
                self.modbus.write_holding_register(126, 0)
                self.modbus.write_holding_register(116, 0)
                return
            
            # 獲取座標
            px, py, wx, wy = detections[obj_num - 1]  # 轉為0-based索引
            
            # 轉換為整數並寫入 (*100)
            world_x_int = int(wx * 100)
            world_y_int = int(wy * 100)
            
            self.modbus.write_holding_register(125, world_x_int)
            self.modbus.write_holding_register(126, world_y_int)
            
            self.ccd1_logger.info(f"成功返回座標: 標籤{class_id} 編號{obj_num} "
                                f"世界座標({wx:.2f}, {wy:.2f}) "
                                f"Modbus值({world_x_int}, {world_y_int})")
            
            self.modbus.write_holding_register(100, 9)  # 準備好
            self.modbus.write_holding_register(116, 0)  # 重置請求地址
            
        except Exception as e:
            self.modbus.write_holding_register(100, 12)
            self.modbus.write_holding_register(125, 0)
            self.modbus.write_holding_register(126, 0)
            self.modbus.write_holding_register(116, 0)
            self.ccd1_logger.error(f"座標查詢異常: {e}")
    
    def _handle_ccd1_redetect_and_query(self):
        """處理CCD1重新檢測+座標查詢請求"""
        if not self.ccd1 or not self.ccd1.initialized:
            self.ccd1_logger.warning("CCD1未初始化，無法執行重新檢測")
            self.modbus.write_holding_register(100, 12)
            self.modbus.write_holding_register(116, 0)
            return
        
        try:
            self.modbus.write_holding_register(100, 8)  # 運作中
            self.ccd1_logger.info("開始重新檢測+座標查詢")
            
            # 先執行檢測
            result = self.ccd1.detect_objects()
            
            if result.success:
                # 清空舊數據
                self.ccd1_detections.clear()
                
                # 更新檢測數量到Modbus (地址110-112)
                counts = [0, 0, 0]  # 標籤0,1,2的數量
                for class_id in range(3):
                    count = result.detections_by_class.get(class_id, 0)
                    counts[class_id] = count
                    self.modbus.write_holding_register(110 + class_id, count)
                
                self.ccd1_logger.info(f"重新檢測完成，數量: DR_F={counts[0]}, STACK={counts[1]}, DR_B={counts[2]}")
                
                # 存儲檢測結果
                for class_id, coords in result.coordinates_by_class.items():
                    if class_id not in self.ccd1_detections:
                        self.ccd1_detections[class_id] = []
                    
                    world_coords = result.world_coordinates_by_class.get(class_id, [])
                    
                    for i, (px, py) in enumerate(coords):
                        if i < len(world_coords):
                            wx, wy = world_coords[i]
                        else:
                            wx, wy = 0.0, 0.0
                        
                        self.ccd1_detections[class_id].append((px, py, wx, wy))
                
                # 執行座標查詢
                self._query_coordinate_after_detection()
                
            else:
                # 檢測失敗，清空數量
                for class_id in range(3):
                    self.modbus.write_holding_register(110 + class_id, 0)
                self.modbus.write_holding_register(125, 0)
                self.modbus.write_holding_register(126, 0)
                self.ccd1_logger.error(f"重新檢測失敗: {result.error_message}")
                self.modbus.write_holding_register(100, 12)
                self.modbus.write_holding_register(116, 0)
                
        except Exception as e:
            self.modbus.write_holding_register(100, 12)
            self.modbus.write_holding_register(125, 0)
            self.modbus.write_holding_register(126, 0)
            self.modbus.write_holding_register(116, 0)
            self.ccd1_logger.error(f"重新檢測+查詢異常: {e}")
    
    def _query_coordinate_after_detection(self):
        """檢測完成後執行座標查詢"""
        try:
            # 從緩存讀取請求參數
            obj_num = self.current_values.get(117, 0)  # 物件編號 (1-based)
            class_id = self.current_values.get(118, 0)  # 標籤編號 (0-based)
            
            self.ccd1_logger.info(f"檢測後查詢座標: 標籤{class_id} 物件編號{obj_num}")
            
            # 檢查指定標籤是否存在
            if class_id not in self.ccd1_detections or len(self.ccd1_detections[class_id]) == 0:
                self.ccd1_logger.error(f"檢測後標籤{class_id}無結果")
                self.modbus.write_holding_register(100, 12)
                self.modbus.write_holding_register(125, 0)
                self.modbus.write_holding_register(126, 0)
                self.modbus.write_holding_register(116, 0)
                return
            
            detections = self.ccd1_detections[class_id]
            
            # 檢查物件編號是否有效
            if obj_num < 1 or obj_num > len(detections):
                self.ccd1_logger.error(f"檢測後物件編號{obj_num}超出範圍，標籤{class_id}共有{len(detections)}個物件")
                self.modbus.write_holding_register(100, 12)
                self.modbus.write_holding_register(125, 0)
                self.modbus.write_holding_register(126, 0)
                self.modbus.write_holding_register(116, 0)
                return
            
            # 獲取座標
            px, py, wx, wy = detections[obj_num - 1]  # 轉為0-based索引
            
            # 轉換為整數並寫入 (*100)
            world_x_int = int(wx * 100)
            world_y_int = int(wy * 100)
            
            self.modbus.write_holding_register(125, world_x_int)
            self.modbus.write_holding_register(126, world_y_int)
            
            self.ccd1_logger.info(f"檢測後成功返回座標: 標籤{class_id} 編號{obj_num} "
                                f"世界座標({wx:.2f}, {wy:.2f}) "
                                f"Modbus值({world_x_int}, {world_y_int})")
            
            self.modbus.write_holding_register(100, 9)  # 準備好
            self.modbus.write_holding_register(116, 0)  # 重置請求地址
            
        except Exception as e:
            self.modbus.write_holding_register(100, 12)
            self.modbus.write_holding_register(125, 0)
            self.modbus.write_holding_register(126, 0)
            self.modbus.write_holding_register(116, 0)
            self.ccd1_logger.error(f"檢測後座標查詢異常: {e}")
    
    def _process_ccd3_detection(self):
        """處理CCD3檢測請求"""
        if not self.ccd3 or not self.ccd3.is_initialized:
            self.ccd3_logger.warning("CCD3未初始化，無法執行檢測")
            return
        
        try:
            self.modbus.write_holding_register(101, 8)  # 運作中
            self.ccd3_logger.info("開始CCD3檢測")
            
            result = self.ccd3.capture_and_detect()
            
            if result.success:
                if result.center and result.angle is not None:
                    # 有效檢測
                    self.ccd3_angle = result.angle
                    self.ccd3_has_contour = True
                    
                    # 角度寫入 (*100)
                    angle_int = int(result.angle * 100)
                    self.modbus.write_holding_register(129, angle_int)  # CCD3角度
                    self.modbus.write_holding_register(130, 1)          # 有效輪廓
                    
                    self.ccd3_logger.info(f"CCD3檢測成功，角度: {result.angle:.2f}° "
                                        f"Modbus值: {angle_int}")
                else:
                    # 無有效輪廓但檢測成功
                    self.ccd3_has_contour = False
                    self.modbus.write_holding_register(130, 0)  # 無有效輪廓
                    
                    self.ccd3_logger.info("CCD3檢測成功，但無有效輪廓")
            else:
                # 檢測失敗
                self.ccd3_logger.error(f"CCD3檢測失敗: {result.error_message}")
            
            self.modbus.write_holding_register(101, 9)  # 準備好
            
        except Exception as e:
            self.modbus.write_holding_register(101, 12)  # 錯誤
            self.ccd3_logger.error(f"CCD3檢測處理異常: {e}")
    
    def get_status(self) -> Dict[str, Any]:
        """獲取系統狀態"""
        return {
            'running': self.running,
            'modbus_connected': self.modbus.connected,
            'ccd1_initialized': self.ccd1 is not None and self.ccd1.initialized,
            'ccd3_initialized': self.ccd3 is not None and self.ccd3.is_initialized,
            'last_ccd1_detections': len(sum(self.ccd1_detections.values(), [])),
            'last_ccd3_angle': self.ccd3_angle,
            'ccd3_has_contour': self.ccd3_has_contour
        }
    
    def shutdown(self):
        """關閉系統"""
        print("正在關閉VisionModule...")
        
        self.running = False
        
        if self.ccd1:
            self.ccd1.disconnect()
            self.ccd1 = None
        
        if self.ccd3:
            self.ccd3.disconnect()
            self.ccd3 = None
        
        if self.modbus:
            self.modbus.close()
        
        print("VisionModule已關閉")

# ==================== 主程序 ====================
def main():
    """主程序"""
    print("啟動VisionModule...")
    
    try:
        vm = VisionModule()
        
        print("VisionModule運行中，按Ctrl+C停止")
        
        while True:
            time.sleep(1)
            status = vm.get_status()
            if not status['running']:
                break
                
    except KeyboardInterrupt:
        print("\n收到停止信號")
    except Exception as e:
        print(f"運行異常: {e}")
    finally:
        if 'vm' in locals():
            vm.shutdown()

if __name__ == "__main__":
    main()