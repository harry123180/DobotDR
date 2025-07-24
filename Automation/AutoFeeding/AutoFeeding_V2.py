#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoFeeding_main.py - DR專案獨立入料檢測模組 (增強版)
基地址：900-999
功能：持續檢測確保DR_F存在於保護區域內
增強功能：
1. 外部控制啟動/停止 (920寄存器)
2. 可調整VP和Flow4參數 (930-959寄存器)
3. 自動初始化硬編碼參數
4. 記憶體使用量監控 (906寄存器)
5. 監控1201當值為1時暫停自動進料程序  
6. 監控1202當前進度，<44時停止運作，>=44時恢復
7. 異常數量檢測：檢測到大量物件後突然驟減時進行二次檢測
保護區域：(-112, 243) 到 (-4, 339)四點矩形
檢測對象：DR_F (正面物件) 和 STACK (堆疊物件) 二分類檢測
"""

import time
import math
import os
import json
import logging
import threading
import psutil
from logging.handlers import RotatingFileHandler
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

# Modbus TCP Client (pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    MODBUS_AVAILABLE = False


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


class AutoFeedingStatus(Enum):
    """AutoFeeding狀態"""
    STOPPED = 0
    RUNNING = 1
    FLOW1_PAUSED = 2  # Flow1執行時暫停
    DETECTING = 3
    VP_VIBRATING = 4
    ERROR = 5


class OperationStatus(Enum):
    """操作狀態"""
    IDLE = 0
    CCD_DETECTING = 1
    VP_CONTROLLING = 2
    FLOW4_TRIGGERING = 3


@dataclass
class CCD1DetectionResult:
    """CCD1檢測結果"""
    dr_f_count: int = 0
    total_detections: int = 0
    dr_f_world_coords: List[Tuple[float, float]] = None
    operation_success: bool = False
    
    def __post_init__(self):
        if self.dr_f_world_coords is None:
            self.dr_f_world_coords = []


@dataclass
class AnomalyDetection:
    """異常檢測記錄"""
    previous_dr_f_count: int = 0
    previous_total_count: int = 0
    has_large_detection: bool = False
    large_detection_threshold: int = 5
    sudden_drop_threshold: int = 1


class ProtectionZone:
    """DR保護區域判斷"""
    
    @staticmethod
    def is_point_in_rect(x: float, y: float) -> bool:
        """判斷點是否在DR保護區域矩形內"""
        # DR保護區域四點座標
        x_min = -112.0
        x_max = -4.0
        y_min = 243.0
        y_max = 339.21
        
        return x_min <= x <= x_max and y_min <= y <= y_max


class AutoFeedingModule:
    """DR AutoFeeding獨立模組 (增強版)"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        # 初始化logging
        self.logger = setup_logging('AutoFeeding')
        
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 基地址範圍 900-999
        self.BASE_ADDRESS = 900
        
        # 外部模組地址
        self.CCD1_BASE = 200
        self.VP_BASE = 300
        self.FLOW4_ADDRESS = 448
        
        # 監控地址
        self.CURRENT_MOTION_FLOW = 1201  # 當前運動Flow (0=無, 1=Flow1, 2=Flow2, 5=Flow5)
        self.CURRENT_PROGRESS = 1202     # 當前進度
        
        # 寄存器地址定義
        self.CONTROL_START = 920         # 控制寄存器起始
        self.VP_PARAMS_START = 930       # VP參數寄存器起始
        self.FLOW4_PARAMS_START = 950    # Flow4參數寄存器起始
        self.COORD_START = 960           # 座標交握寄存器起始
        self.SYSTEM_PARAMS_START = 980   # 系統參數寄存器起始
        
        # 硬編碼預設參數
        self.DEFAULT_VP_PARAMS = {
            "action1": {"action_code": 4, "strength": 45, "frequency": 43, "duration": 800},
            "action2": {"action_code": 11, "strength": 98, "frequency": 64, "duration": 800},
            "stop": {"command_code": 3, "delay": 100}
        }
        
        self.DEFAULT_FLOW4_PARAMS = {
            "pulse_duration": 100,
            "pulse_interval": 50
        }
        
        self.DEFAULT_SYSTEM_PARAMS = {
            "cycle_interval": 1000,
            "ccd1_timeout": 5000,
            "flow4_consecutive_limit": 5,
            "vp_empty_check_count": 3,
            "progress_threshold": 44,
            "large_count_threshold": 5,
            "sudden_drop_threshold": 1,
            "redetection_delay": 500,
            "vp_stabilize_delay": 150,
            "flow1_check_interval": 100
        }
        
        # 保護區域判斷
        self.protection_zone = ProtectionZone()
        
        # 系統狀態
        self.status = AutoFeedingStatus.STOPPED
        self.operation_status = OperationStatus.IDLE
        self.running = False
        self.external_control_running = False  # 外部控制運行狀態
        self.flow1_active = False
        self.progress_blocking = False
        self.vp_clearing_mode = False
        
        # DR_F狀態
        self.dr_f_available = False
        self.dr_f_coords = (0.0, 0.0)
        self.dr_f_taken = False
        
        # 統計資訊
        self.cycle_count = 0
        self.dr_f_found_count = 0
        self.flow4_trigger_count = 0
        self.vp_vibration_count = 0
        self.flow4_consecutive_count = 0
        self.vp_empty_detection_count = 0
        self.error_code = 0
        self.anomaly_redetection_count = 0
        
        # 異常檢測
        self.anomaly_detection = AnomalyDetection()
        
        # 記憶體監控
        self.memory_monitor_thread = None
        self.memory_monitor_running = False
        
        self.logger.info(f"DR AutoFeeding獨立模組初始化 (增強版) - 基地址{self.BASE_ADDRESS}")
        self.logger.info(f"新增功能: 外部控制(920), VP參數(930-949), Flow4參數(950-959), 記憶體監控(906)")
        self.logger.info(f"保護區域: X(-112.0 ~ -4.0mm), Y(243.0 ~ 339.21mm)")
        self.logger.info(f"監控當前執行Flow地址: {self.CURRENT_MOTION_FLOW}")
        self.logger.info(f"監控當前進度地址: {self.CURRENT_PROGRESS}")
    
    def connect(self) -> bool:
        """連接Modbus服務器"""
        try:
            if not MODBUS_AVAILABLE:
                self.logger.error("Modbus功能不可用")
                return False
            
            self.logger.info(f"正在連接 {self.modbus_host}:{self.modbus_port}")
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.connected = self.modbus_client.connect()
            
            if self.connected:
                self.logger.info("連接建立成功")
                
                # 檢查是否需要初始化參數
                if self.check_need_initialization():
                    self.logger.info("檢測到參數未初始化，執行硬編碼參數初始化...")
                    self.initialize_default_parameters()
                
                self.init_registers()
                self.start_memory_monitor()
                return True
            else:
                self.logger.warning("連接失敗，將在主循環中重試")
                return False
        except Exception as e:
            self.logger.error(f"連接異常: {e}", exc_info=True)
            self.connected = False
            return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        if self.modbus_client and self.connected:
            self.stop_memory_monitor()
            self.modbus_client.close()
            self.connected = False
            self.logger.info("Modbus連接已斷開")
    
    def check_need_initialization(self) -> bool:
        """檢查是否需要初始化參數"""
        try:
            # 檢查關鍵寄存器是否全為0
            vp_params_zero = all(self.read_register(self.VP_PARAMS_START + i) == 0 for i in range(10))
            flow4_params_zero = all(self.read_register(self.FLOW4_PARAMS_START + i) == 0 for i in range(2))
            system_params_zero = all(self.read_register(self.SYSTEM_PARAMS_START + i) == 0 for i in range(10))
            
            need_init = vp_params_zero and flow4_params_zero and system_params_zero
            
            if need_init:
                self.logger.info("檢測到關鍵寄存器全為0，需要初始化")
            else:
                self.logger.info("檢測到參數已存在，跳過初始化")
            
            return need_init
            
        except Exception as e:
            self.logger.error(f"初始化檢查失敗: {e}", exc_info=True)
            return True  # 發生錯誤時執行初始化
    
    def initialize_default_parameters(self) -> bool:
        """初始化硬編碼預設參數到寄存器"""
        try:
            self.logger.info("開始寫入硬編碼預設參數...")
            
            # VP參數初始化 (930-949)
            vp_success = True
            vp_success &= self.write_register(930, self.DEFAULT_VP_PARAMS["action1"]["action_code"])
            vp_success &= self.write_register(931, self.DEFAULT_VP_PARAMS["action1"]["strength"])
            vp_success &= self.write_register(932, self.DEFAULT_VP_PARAMS["action1"]["frequency"])
            vp_success &= self.write_register(933, self.DEFAULT_VP_PARAMS["action1"]["duration"])
            vp_success &= self.write_register(934, self.DEFAULT_VP_PARAMS["action2"]["action_code"])
            vp_success &= self.write_register(935, self.DEFAULT_VP_PARAMS["action2"]["strength"])
            vp_success &= self.write_register(936, self.DEFAULT_VP_PARAMS["action2"]["frequency"])
            vp_success &= self.write_register(937, self.DEFAULT_VP_PARAMS["action2"]["duration"])
            vp_success &= self.write_register(938, self.DEFAULT_VP_PARAMS["stop"]["command_code"])
            vp_success &= self.write_register(939, self.DEFAULT_VP_PARAMS["stop"]["delay"])
            
            if vp_success:
                self.logger.info("VP參數初始化成功 (930-939)")
            else:
                self.logger.error("VP參數初始化失敗")
            
            # Flow4參數初始化 (950-959)
            flow4_success = True
            flow4_success &= self.write_register(950, self.DEFAULT_FLOW4_PARAMS["pulse_duration"])
            flow4_success &= self.write_register(951, self.DEFAULT_FLOW4_PARAMS["pulse_interval"])
            
            if flow4_success:
                self.logger.info("Flow4參數初始化成功 (950-951)")
            else:
                self.logger.error("Flow4參數初始化失敗")
            
            # 系統參數初始化 (980-999)
            system_success = True
            system_success &= self.write_register(980, self.DEFAULT_SYSTEM_PARAMS["cycle_interval"])
            system_success &= self.write_register(981, self.DEFAULT_SYSTEM_PARAMS["ccd1_timeout"])
            system_success &= self.write_register(982, self.DEFAULT_SYSTEM_PARAMS["flow4_consecutive_limit"])
            system_success &= self.write_register(983, self.DEFAULT_SYSTEM_PARAMS["vp_empty_check_count"])
            system_success &= self.write_register(984, self.DEFAULT_SYSTEM_PARAMS["progress_threshold"])
            system_success &= self.write_register(985, self.DEFAULT_SYSTEM_PARAMS["large_count_threshold"])
            system_success &= self.write_register(986, self.DEFAULT_SYSTEM_PARAMS["sudden_drop_threshold"])
            system_success &= self.write_register(987, self.DEFAULT_SYSTEM_PARAMS["redetection_delay"])
            system_success &= self.write_register(988, self.DEFAULT_SYSTEM_PARAMS["vp_stabilize_delay"])
            system_success &= self.write_register(989, self.DEFAULT_SYSTEM_PARAMS["flow1_check_interval"])
            
            if system_success:
                self.logger.info("系統參數初始化成功 (980-989)")
            else:
                self.logger.error("系統參數初始化失敗")
            
            overall_success = vp_success and flow4_success and system_success
            
            if overall_success:
                self.logger.info("硬編碼參數初始化完成")
            else:
                self.logger.error("硬編碼參數初始化部分失敗")
            
            return overall_success
            
        except Exception as e:
            self.logger.error(f"參數初始化異常: {e}", exc_info=True)
            return False
    
    def start_memory_monitor(self):
        """啟動記憶體監控執行緒"""
        try:
            if self.memory_monitor_thread is None or not self.memory_monitor_thread.is_alive():
                self.memory_monitor_running = True
                self.memory_monitor_thread = threading.Thread(target=self._memory_monitor_worker, daemon=True)
                self.memory_monitor_thread.start()
                self.logger.info("記憶體監控執行緒已啟動 (每10分鐘更新)")
        except Exception as e:
            self.logger.error(f"記憶體監控啟動失敗: {e}", exc_info=True)
    
    def stop_memory_monitor(self):
        """停止記憶體監控執行緒"""
        self.memory_monitor_running = False
        if self.memory_monitor_thread and self.memory_monitor_thread.is_alive():
            self.memory_monitor_thread.join(timeout=1.0)
            self.logger.info("記憶體監控執行緒已停止")
    
    def _memory_monitor_worker(self):
        """記憶體監控工作執行緒"""
        while self.memory_monitor_running:
            try:
                if self.connected:
                    # 獲取當前進程記憶體使用量
                    process = psutil.Process()
                    memory_bytes = process.memory_info().rss
                    memory_mb = int(memory_bytes / (1024 * 1024))  # 無條件捨去小數點
                    
                    # 更新到906寄存器
                    if self.write_register(906, memory_mb):
                        self.logger.debug(f"記憶體使用量更新: {memory_mb} MB")
                    else:
                        self.logger.warning("記憶體使用量寫入寄存器失敗")
                
                # 等待10分鐘 (600秒)
                for _ in range(600):
                    if not self.memory_monitor_running:
                        break
                    time.sleep(1)
                    
            except Exception as e:
                self.logger.error(f"記憶體監控執行緒異常: {e}", exc_info=True)
                time.sleep(60)  # 發生錯誤時等待1分鐘後重試
    
    def init_registers(self):
        """初始化寄存器"""
        try:
            # 狀態寄存器 (900-919)
            self.write_register(900, AutoFeedingStatus.STOPPED.value)  # 模組狀態
            self.write_register(901, self.cycle_count)                  # 週期計數
            self.write_register(902, self.dr_f_found_count)            # DR_F找到次數
            self.write_register(903, self.flow4_trigger_count)         # Flow4觸發次數
            self.write_register(904, self.vp_vibration_count)          # VP震動次數
            self.write_register(905, self.anomaly_redetection_count)   # 異常重檢次數
            # 906 記憶體使用量由監控執行緒更新
            self.write_register(907, 0)  # 錯誤代碼
            self.write_register(908, OperationStatus.IDLE.value)       # 操作狀態
            self.write_register(909, 0)  # Flow1監控狀態
            self.write_register(910, 0)  # 進度阻擋狀態
            
            # 控制寄存器 (920-929) - 保持現有值或設為0
            current_control = self.read_register(920)
            if current_control is None:
                self.write_register(920, 0)  # 預設停止狀態
            
            # DR_F狀態寄存器 (960-979) - 座標交握區域
            self.write_register(960, 0)  # DR_F可用標誌
            self.write_register(961, 0)  # X座標高位
            self.write_register(962, 0)  # X座標低位
            self.write_register(963, 0)  # Y座標高位
            self.write_register(964, 0)  # Y座標低位
            self.write_register(965, 0)  # 座標已讀取標誌
            
            self.logger.info("寄存器初始化完成")
        except Exception as e:
            self.logger.error(f"寄存器初始化失敗: {e}", exc_info=True)
    
    def read_register(self, address: int) -> Optional[int]:
        """讀取單個寄存器"""
        try:
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            if not result.isError():
                return result.registers[0]
            return None
        except Exception:
            return None
    
    def write_register(self, address: int, value: int) -> bool:
        """寫入單個寄存器"""
        try:
            result = self.modbus_client.write_register(address, value, slave=1)
            return not result.isError()
        except Exception:
            return False
    
    def read_32bit_register(self, high_addr: int, low_addr: int) -> float:
        """讀取32位世界座標並轉換為實際值"""
        high_val = self.read_register(high_addr)
        low_val = self.read_register(low_addr)
        
        if high_val is None or low_val is None:
            return 0.0
        
        # 合併32位值
        combined = (high_val << 16) + low_val
        
        # 處理補碼(負數)
        if combined >= 2147483648:  # 2^31
            combined = combined - 4294967296  # 2^32
        
        # 轉換為毫米(除以100)
        return combined / 100.0
    
    def write_32bit_register(self, high_addr: int, low_addr: int, value: float) -> bool:
        """寫入32位世界座標"""
        # 轉換為整數形式(×100)
        value_int = int(value * 100)
        
        # 處理負數(補碼)
        if value_int < 0:
            value_int = value_int + 4294967296  # 2^32
        
        # 分解為高低位
        high_val = (value_int >> 16) & 0xFFFF
        low_val = value_int & 0xFFFF
        
        success = True
        success &= self.write_register(high_addr, high_val)
        success &= self.write_register(low_addr, low_val)
        
        return success
    
    def read_vp_params(self) -> Dict[str, Any]:
        """從寄存器讀取VP參數"""
        try:
            params = {
                "action1": {
                    "action_code": max(0, min(11, self.read_register(930) or 4)),
                    "strength": max(0, min(100, self.read_register(931) or 45)),
                    "frequency": max(0, min(100, self.read_register(932) or 43)),
                    "duration": max(100, min(5000, self.read_register(933) or 800))
                },
                "action2": {
                    "action_code": max(0, min(11, self.read_register(934) or 11)),
                    "strength": max(0, min(100, self.read_register(935) or 98)),
                    "frequency": max(0, min(100, self.read_register(936) or 64)),
                    "duration": max(100, min(5000, self.read_register(937) or 800))
                },
                "stop": {
                    "command_code": max(0, min(255, self.read_register(938) or 3)),
                    "delay": max(10, min(1000, self.read_register(939) or 100))
                }
            }
            return params
        except Exception as e:
            self.logger.error(f"VP參數讀取失敗: {e}", exc_info=True)
            return self.DEFAULT_VP_PARAMS
    
    def read_flow4_params(self) -> Dict[str, int]:
        """從寄存器讀取Flow4參數"""
        try:
            params = {
                "pulse_duration": max(10, min(5000, self.read_register(950) or 100)),
                "pulse_interval": max(10, min(1000, self.read_register(951) or 50))
            }
            return params
        except Exception as e:
            self.logger.error(f"Flow4參數讀取失敗: {e}", exc_info=True)
            return self.DEFAULT_FLOW4_PARAMS
    
    def read_system_params(self) -> Dict[str, int]:
        """從寄存器讀取系統參數"""
        try:
            params = {
                "cycle_interval": max(100, min(10000, self.read_register(980) or 1000)),
                "ccd1_timeout": max(1000, min(30000, self.read_register(981) or 5000)),
                "flow4_consecutive_limit": max(1, min(20, self.read_register(982) or 5)),
                "vp_empty_check_count": max(1, min(10, self.read_register(983) or 3)),
                "progress_threshold": max(0, min(100, self.read_register(984) or 44)),
                "large_count_threshold": max(1, min(20, self.read_register(985) or 5)),
                "sudden_drop_threshold": max(0, min(10, self.read_register(986) or 1)),
                "redetection_delay": max(100, min(5000, self.read_register(987) or 500)),
                "vp_stabilize_delay": max(50, min(2000, self.read_register(988) or 150)),
                "flow1_check_interval": max(50, min(1000, self.read_register(989) or 100))
            }
            return params
        except Exception as e:
            self.logger.error(f"系統參數讀取失敗: {e}", exc_info=True)
            return self.DEFAULT_SYSTEM_PARAMS
    
    def check_external_control(self) -> bool:
        """檢查外部控制狀態"""
        try:
            control_value = self.read_register(920)
            if control_value is None:
                return False
            
            should_run = (control_value == 1)
            
            # 檢查狀態變化
            if should_run != self.external_control_running:
                self.external_control_running = should_run
                if should_run:
                    self.logger.info("外部控制啟動AutoFeeding (920=1)")
                    return True
                else:
                    self.logger.info("外部控制停止AutoFeeding (920=0)")
                    return False
            
            return should_run
            
        except Exception as e:
            self.logger.error(f"外部控制檢查失敗: {e}", exc_info=True)
            return False
    
    def update_status_registers(self):
        """更新狀態寄存器"""
        try:
            self.write_register(900, self.status.value)
            self.write_register(901, self.cycle_count)
            self.write_register(902, self.dr_f_found_count)
            self.write_register(903, self.flow4_trigger_count)
            self.write_register(904, self.vp_vibration_count)
            self.write_register(905, self.anomaly_redetection_count)
            self.write_register(907, self.error_code)
            self.write_register(908, self.operation_status.value)
            self.write_register(909, 1 if self.flow1_active else 0)
            self.write_register(910, 1 if self.progress_blocking else 0)
            
            # 更新DR_F狀態
            self.write_register(960, 1 if self.dr_f_available else 0)
            if self.dr_f_available:
                self.write_32bit_register(961, 962, self.dr_f_coords[0])
                self.write_32bit_register(963, 964, self.dr_f_coords[1])
        except Exception as e:
            self.logger.error(f"狀態寄存器更新失敗: {e}", exc_info=True)
    
    def check_flow_and_progress_status(self) -> bool:
        """監控Flow和進度狀態"""
        try:
            current_motion_flow = self.read_register(self.CURRENT_MOTION_FLOW)
            current_progress = self.read_register(self.CURRENT_PROGRESS)
            
            if current_motion_flow is None or current_progress is None:
                return False
            
            # 獲取當前進度阻擋閾值
            system_params = self.read_system_params()
            progress_threshold = system_params["progress_threshold"]
            
            # 檢查Flow1狀態變化
            flow1_now_active = (current_motion_flow == 1)
            
            # 檢查進度阻擋條件：1201=1 且 1202<threshold
            progress_now_blocking = (current_motion_flow == 1 and current_progress < progress_threshold)
            
            # 記錄狀態變化
            if flow1_now_active != self.flow1_active:
                self.flow1_active = flow1_now_active
                if self.flow1_active:
                    self.logger.info(f"檢測到Flow1正在執行 ({self.CURRENT_MOTION_FLOW}=1)")
                else:
                    self.logger.info(f"檢測到Flow1執行完成 ({self.CURRENT_MOTION_FLOW}=0)")
                    # Flow1完成後，檢查座標是否被讀取
                    self.check_coords_taken()
            
            if progress_now_blocking != self.progress_blocking:
                self.progress_blocking = progress_now_blocking
                if self.progress_blocking:
                    self.logger.info(f"進度阻擋啟動 (1201={current_motion_flow}, 1202={current_progress}<{progress_threshold})，停止所有操作")
                    if self.status == AutoFeedingStatus.RUNNING:
                        self.status = AutoFeedingStatus.FLOW1_PAUSED
                else:
                    self.logger.info(f"進度阻擋解除 (1201={current_motion_flow}, 1202={current_progress}>={progress_threshold})，恢復操作")
                    if self.status == AutoFeedingStatus.FLOW1_PAUSED:
                        self.status = AutoFeedingStatus.RUNNING
            
            # 調試日誌 - 減少頻率
            if self.cycle_count % 100 == 1:
                self.logger.debug(f"Flow監控: 1201={current_motion_flow}, 1202={current_progress}, 阻擋={self.progress_blocking}")
            
            return True
        except Exception as e:
            self.logger.error(f"Flow和進度狀態檢查失敗: {e}", exc_info=True)
            return False
    
    def check_coords_taken(self):
        """檢查座標是否被Flow1讀取"""
        if not self.dr_f_available:
            return
        
        try:
            coords_taken = self.read_register(965)  # Flow1設置此標誌表示已讀取座標
            if coords_taken == 1:
                self.logger.info("座標已被Flow1讀取，清除DR_F狀態")
                # 清除DR_F狀態，繼續檢測新的
                self.dr_f_available = False
                self.dr_f_coords = (0.0, 0.0)
                self.dr_f_taken = True
                
                # 清除相關寄存器
                self.write_register(960, 0)  # DR_F可用標誌
                self.write_register(965, 0)  # 座標已讀取標誌
                for addr in [961, 962, 963, 964]:
                    self.write_register(addr, 0)
                
                self.logger.info("DR_F狀態已清除，繼續檢測新的正面物件")
        except Exception as e:
            self.logger.error(f"座標讀取檢查失敗: {e}", exc_info=True)
    
    def check_modules_status(self) -> bool:
        """檢查CCD1、VP模組狀態"""
        # 檢查CCD1狀態
        ccd1_status = self.read_register(201)
        if ccd1_status is None:
            if self.cycle_count % 50 == 1:
                self.logger.debug("CCD1模組無回應 (寄存器201無法讀取)")
            self.error_code = 101
            return False
        
        # 解析CCD1狀態位
        ccd1_ready = bool(ccd1_status & 0x01)
        ccd1_running = bool(ccd1_status & 0x02)
        ccd1_alarm = bool(ccd1_status & 0x04)
        ccd1_initialized = bool(ccd1_status & 0x08)
        
        if self.cycle_count % 100 == 1:  # 減少打印頻率
            self.logger.debug(f"CCD1狀態: {ccd1_status} (Ready={ccd1_ready}, Running={ccd1_running}, Alarm={ccd1_alarm}, Init={ccd1_initialized})")
        
        # 檢查CCD1是否準備就緒 (應該是狀態9: Ready=1, Initialized=1)
        if ccd1_alarm:
            if self.cycle_count % 50 == 1:
                error_code = self.read_register(206)
                self.logger.debug(f"CCD1處於警報狀態，錯誤代碼: {error_code}")
            self.error_code = 102
            return False
        
        if not ccd1_initialized:
            if self.cycle_count % 50 == 1:
                self.logger.debug("CCD1尚未初始化完成，等待...")
            self.error_code = 102
            return False
        
        if not ccd1_ready:
            if self.cycle_count % 50 == 1:
                self.logger.debug("CCD1未Ready (可能正在執行其他任務)，等待...")
            self.error_code = 102
            return False
        
        # 檢查VP狀態
        vp_status = self.read_register(300)
        vp_connected = self.read_register(301)
        
        if vp_status is None or vp_connected is None:
            if self.cycle_count % 50 == 1:
                self.logger.debug("VP模組無回應")
            self.error_code = 103
            return False
        
        if vp_status != 1 or vp_connected != 1:
            if self.cycle_count % 50 == 1:
                self.logger.debug(f"VP模組狀態異常: status={vp_status}, connected={vp_connected}")
            self.error_code = 103
            return False
        
        return True
    
    def check_for_anomaly(self, current_result: CCD1DetectionResult) -> bool:
        """檢查異常數量變化，決定是否需要重檢"""
        # 獲取當前異常檢測參數
        system_params = self.read_system_params()
        large_threshold = system_params["large_count_threshold"]
        drop_threshold = system_params["sudden_drop_threshold"]
        
        # 檢查前一次是否有大量檢測
        prev_had_large = (self.anomaly_detection.previous_dr_f_count >= large_threshold or 
                         self.anomaly_detection.previous_total_count >= large_threshold)
        
        # 檢查當前是否驟減
        current_has_drop = (current_result.dr_f_count <= drop_threshold and 
                           current_result.total_detections <= drop_threshold)
        
        # 判斷是否需要重檢
        if prev_had_large and current_has_drop:
            self.logger.warning(f"檢測到異常：前次大量檢測(DR_F={self.anomaly_detection.previous_dr_f_count}, 總數={self.anomaly_detection.previous_total_count})，當前驟減(DR_F={current_result.dr_f_count}, 總數={current_result.total_detections})")
            return True
        
        return False
    
    def update_anomaly_detection(self, result: CCD1DetectionResult):
        """更新異常檢測記錄"""
        self.anomaly_detection.previous_dr_f_count = result.dr_f_count
        self.anomaly_detection.previous_total_count = result.total_detections
    
    def trigger_ccd1_detection(self) -> CCD1DetectionResult:
        """觸發CCD1檢測 - 適配DR專案的YOLO檢測"""
        self.operation_status = OperationStatus.CCD_DETECTING
        result = CCD1DetectionResult()
        
        # 獲取當前CCD1超時參數
        system_params = self.read_system_params()
        timeout = system_params["ccd1_timeout"] / 1000.0  # 轉換為秒
        
        # 觸發拍照+檢測
        if not self.write_register(200, 16):
            self.logger.error("無法寫入控制指令到寄存器200")
            self.error_code = 201
            return result
        
        self.logger.debug("已發送控制指令16到寄存器200")
        
        # 等待檢測完成
        start_time = time.time()
        check_interval = 0.02
        
        while (time.time() - start_time) < timeout:
            capture_complete = self.read_register(203)
            detect_complete = self.read_register(204)
            operation_success = self.read_register(205)
            current_status = self.read_register(201)
            
            if capture_complete == 1 and detect_complete == 1 and operation_success == 1:
                result.operation_success = True
                self.logger.debug("CCD1檢測完成標誌確認")
                break
            
            time.sleep(check_interval)
        
        if not result.operation_success:
            elapsed_time = time.time() - start_time
            self.logger.error(f"CCD1檢測超時 ({elapsed_time:.2f}s)，最終狀態:")
            self.logger.error(f"  201狀態: {self.read_register(201)}")
            self.logger.error(f"  203拍照完成: {self.read_register(203)}")
            self.logger.error(f"  204檢測完成: {self.read_register(204)}")
            self.logger.error(f"  205操作成功: {self.read_register(205)}")
            self.logger.error(f"  206錯誤代碼: {self.read_register(206)}")
            self.error_code = 202
            return result
        
        # 讀取DR YOLO檢測結果
        result.dr_f_count = self.read_register(240) or 0        # DR_F數量
        stack_count = self.read_register(242) or 0              # STACK數量  
        result.total_detections = self.read_register(243) or 0  # 總檢測數量 (DR_F + STACK)
        
        # 提取DR_F世界座標
        if result.dr_f_count > 0:
            for i in range(min(result.dr_f_count, 5)):
                # DR YOLO版本世界座標地址 (261-280)
                base_addr = 261 + (i * 4)
                world_x = self.read_32bit_register(base_addr, base_addr + 1)
                world_y = self.read_32bit_register(base_addr + 2, base_addr + 3)
                result.dr_f_world_coords.append((world_x, world_y))
        
        # 清空CCD1寄存器
        self.write_register(200, 0)
        self.write_register(203, 0)
        self.write_register(204, 0)
        self.write_register(205, 0)
        
        return result
    
    def find_dr_f_in_protection_zone(self, detection_result: CCD1DetectionResult) -> Optional[Tuple[float, float]]:
        """尋找保護區域內的DR_F"""
        if detection_result.dr_f_count == 0:
            return None
        
        for world_x, world_y in detection_result.dr_f_world_coords:
            if self.protection_zone.is_point_in_rect(world_x, world_y):
                return (world_x, world_y)
        
        return None
    
    def trigger_vp_vibration(self) -> bool:
        """觸發VP震動 - 使用寄存器參數"""
        self.operation_status = OperationStatus.VP_CONTROLLING
        
        # 讀取當前VP參數
        vp_params = self.read_vp_params()
        
        self.logger.debug(f"VP震動參數: action1={vp_params['action1']}, action2={vp_params['action2']}")
        
        # 第一組震動
        success = True
        success &= self.write_register(320, 5)  # execute_action
        success &= self.write_register(321, vp_params["action1"]["action_code"])
        success &= self.write_register(322, vp_params["action1"]["strength"])
        success &= self.write_register(323, vp_params["action1"]["frequency"])
        success &= self.write_register(324, int(time.time()) % 65535)
                
        if not success:
            self.error_code = 301
            return False
        
        # 第一組震動持續時間
        time.sleep(vp_params["action1"]["duration"] / 1000.0)
        
        # 第二組震動 (如果動作碼不為0)
        if vp_params["action2"]["action_code"] > 0:
            success = True
            success &= self.write_register(320, 5)  # execute_action
            success &= self.write_register(321, vp_params["action2"]["action_code"])
            success &= self.write_register(322, vp_params["action2"]["strength"])
            success &= self.write_register(323, vp_params["action2"]["frequency"])
            success &= self.write_register(324, int(time.time()) % 65535)
            
            if not success:
                self.error_code = 301  
                return False
            
            # 第二組震動持續時間
            time.sleep(vp_params["action2"]["duration"] / 1000.0)
        
        # 停止震動
        return self.stop_vp_vibration()
    
    def stop_vp_vibration(self) -> bool:
        """停止VP震動 - 使用寄存器參數"""
        vp_params = self.read_vp_params()
        
        success = True
        success &= self.write_register(320, vp_params["stop"]["command_code"])
        success &= self.write_register(321, 0)
        success &= self.write_register(322, 0)
        success &= self.write_register(323, 0)
        success &= self.write_register(324, 99)
        
        if success:
            time.sleep(vp_params["stop"]["delay"] / 1000.0)
        
        return success
    
    def trigger_flow4_feeding(self) -> bool:
        """觸發Flow4送料 - 使用寄存器參數"""
        self.operation_status = OperationStatus.FLOW4_TRIGGERING
        
        # 讀取當前Flow4參數
        flow4_params = self.read_flow4_params()
        pulse_duration = flow4_params["pulse_duration"] / 1000.0  # 轉換為秒
        
        if not self.write_register(self.FLOW4_ADDRESS, 1):
            self.error_code = 401
            return False
        
        time.sleep(pulse_duration)
        
        if not self.write_register(self.FLOW4_ADDRESS, 0):
            self.error_code = 402
            return False
        
        return True
    
    def set_dr_f_available(self, coords: Tuple[float, float]):
        """設置DR_F可用狀態"""
        self.dr_f_available = True
        self.dr_f_coords = coords
        self.dr_f_taken = False
        
        # 立即更新寄存器讓Flow1可以讀取
        self.write_register(960, 1)  # DR_F可用標誌
        self.write_32bit_register(961, 962, coords[0])  # X座標
        self.write_32bit_register(963, 964, coords[1])  # Y座標
        
        self.logger.info(f"DR_F已就緒: {coords}, Flow1可直接讀取座標")
    
    def feeding_cycle(self) -> bool:
        """執行一次入料檢測週期 - 使用寄存器參數"""
        try:
            self.cycle_count += 1
            self.status = AutoFeedingStatus.DETECTING
            
            # 快速檢查模組狀態
            if not self.check_modules_status():
                if self.error_code == 102:  # CCD1初始化中
                    self.status = AutoFeedingStatus.RUNNING
                    return True
                return False
            
            # CCD1檢測
            detection_result = self.trigger_ccd1_detection()
            if not detection_result.operation_success:
                self.logger.warning(f"週期{self.cycle_count} CCD1檢測失敗")
                return False
            
            self.logger.info(f"週期{self.cycle_count} 檢測結果: DR_F={detection_result.dr_f_count}, 總數={detection_result.total_detections}")
            
            # 異常檢測 - 檢查是否需要重檢
            need_redetection = self.check_for_anomaly(detection_result)
            if need_redetection:
                self.logger.warning("執行異常重檢...")
                system_params = self.read_system_params()
                redetection_delay = system_params["redetection_delay"] / 1000.0
                time.sleep(redetection_delay)
                
                retry_result = self.trigger_ccd1_detection()
                if retry_result.operation_success:
                    self.logger.info(f"重檢結果: DR_F={retry_result.dr_f_count}, 總數={retry_result.total_detections}")
                    detection_result = retry_result  # 使用重檢結果
                    self.anomaly_redetection_count += 1
                else:
                    self.logger.warning("重檢失敗，使用原檢測結果")
            
            # 更新異常檢測記錄
            self.update_anomaly_detection(detection_result)
            
            # 尋找保護區域內的DR_F
            target_coords = self.find_dr_f_in_protection_zone(detection_result)
            
            if target_coords:
                # 找到正面物件 - 設置可用狀態，但繼續檢測
                self.dr_f_found_count += 1
                self.flow4_consecutive_count = 0
                self.logger.info(f"找到保護區內DR_F: {target_coords}")
                
                # 設置DR_F可用狀態
                self.set_dr_f_available(target_coords)
                
            elif detection_result.total_detections < 4:
                # 料件不足，觸發Flow4送料
                self.logger.info(f"料件不足 (總數={detection_result.total_detections}<4)，觸發Flow4送料")
                
                if self.trigger_flow4_feeding():
                    self.flow4_trigger_count += 1
                    self.flow4_consecutive_count += 1
                    self.logger.info(f"Flow4送料完成 (連續{self.flow4_consecutive_count}次)")
                    
                    # 檢查連續直振限制
                    system_params = self.read_system_params()
                    if self.flow4_consecutive_count >= system_params["flow4_consecutive_limit"]:
                        self.logger.warning("達到連續直振限制，需要VP清空")
                else:
                    self.logger.warning("Flow4送料失敗")
                
            else:
                # 料件充足但無正面，VP震動重檢
                self.logger.info(f"料件充足 (總數={detection_result.total_detections}>=4) 但無正面，VP震動重檢")
                self.flow4_consecutive_count = 0
                
                if self.trigger_vp_vibration():
                    self.vp_vibration_count += 1
                    self.logger.info("VP震動完成，等待穩定後重新檢測")
                    
                    # 等待穩定
                    system_params = self.read_system_params()
                    time.sleep(system_params["vp_stabilize_delay"] / 1000.0)
                    
                    # 立即重新檢測
                    retry_result = self.trigger_ccd1_detection()
                    if retry_result.operation_success:
                        self.logger.info(f"震動後重檢: DR_F={retry_result.dr_f_count}, 總數={retry_result.total_detections}")
                        retry_coords = self.find_dr_f_in_protection_zone(retry_result)
                        if retry_coords:
                            self.dr_f_found_count += 1
                            self.logger.info(f"震動後找到DR_F: {retry_coords}")
                            self.set_dr_f_available(retry_coords)
            
            self.operation_status = OperationStatus.IDLE
            self.status = AutoFeedingStatus.RUNNING
            return True
            
        except Exception as e:
            self.logger.error(f"入料週期異常: {e}", exc_info=True)
            self.error_code = 999
            return False
    
    def start_feeding(self):
        """啟動入料檢測"""
        if self.running:
            return
        
        self.logger.info("啟動持續入料檢測")
        self.logger.info("目標：保持保護區域內始終有DR_F可用")
        
        self.running = True
        self.status = AutoFeedingStatus.RUNNING
        self.error_code = 0
        
        # 重置DR_F狀態
        self.dr_f_available = False
        self.dr_f_coords = (0.0, 0.0)
        self.dr_f_taken = False
    
    def stop_feeding(self):
        """停止入料檢測"""
        self.running = False
        self.status = AutoFeedingStatus.STOPPED
        self.dr_f_available = False
        self.emergency_stop_vp()
        self.logger.info("入料檢測已停止")
    
    def emergency_stop_vp(self):
        """緊急停止VP"""
        try:
            self.stop_vp_vibration()
            self.logger.info("VP緊急停止")
        except:
            pass
    
    def main_loop(self):
        """主循環"""
        self.logger.info("主循環啟動 (增強版)")
        self.logger.info("新增特性：")
        self.logger.info("  ✓ 外部控制啟動/停止 (920寄存器)")
        self.logger.info("  ✓ 可調整VP參數 (930-949寄存器)")
        self.logger.info("  ✓ 可調整Flow4參數 (950-959寄存器)")
        self.logger.info("  ✓ 可調整系統參數 (980-999寄存器)")
        self.logger.info("  ✓ 記憶體使用量監控 (906寄存器)")
        self.logger.info("  ✓ 自動初始化硬編碼參數")
        self.logger.info("  ✓ 監控當前執行Flow狀態 (1201) 和進度 (1202)")
        self.logger.info("  ✓ 異常數量檢測和二次重檢")
        self.logger.info("  ✓ 持續檢測確保DR_F可用")
        self.logger.info("  ✓ 簡化交握邏輯")
        self.logger.info("  ✓ Flow1直接讀取座標")
        self.logger.info("  ✓ DR保護區域: X(-112~-4mm), Y(243~339.21mm)")
        
        loop_count = 0
        
        while True:
            try:
                loop_count += 1
                
                # 定期打印狀態
                if loop_count % 200 == 1:
                    self.logger.debug(f"主循環 {loop_count}: running={self.running}, status={self.status.name}, external_control={self.external_control_running}, flow1_active={self.flow1_active}, progress_blocking={self.progress_blocking}, dr_f_available={self.dr_f_available}")
                
                # 檢查連接狀態
                if not self.connected:
                    self.logger.debug("Modbus連接斷開，嘗試重連")
                    if not self.connect():
                        time.sleep(5.0)
                        continue
                
                # 檢查外部控制狀態
                should_run_external = self.check_external_control()
                
                # 外部控制啟動/停止邏輯
                if should_run_external and not self.running:
                    self.start_feeding()
                elif not should_run_external and self.running:
                    self.stop_feeding()
                
                # 監控Flow和進度狀態
                self.check_flow_and_progress_status()
                
                # 檢查座標是否被讀取
                if self.dr_f_available:
                    self.check_coords_taken()
                
                # 更新狀態寄存器
                self.update_status_registers()
                
                # 執行入料檢測 - 只有在運行且未被阻擋時
                if self.running and not self.progress_blocking and not self.vp_clearing_mode:
                    if not self.feeding_cycle():
                        self.logger.debug(f"入料檢測失敗，錯誤碼: {self.error_code}")
                        self.status = AutoFeedingStatus.ERROR
                        time.sleep(0.5)
                    else:
                        # 檢測成功，使用寄存器參數決定檢測間隔
                        system_params = self.read_system_params()
                        cycle_interval = system_params["cycle_interval"] / 1000.0
                        time.sleep(cycle_interval)
                else:
                    # 非運行狀態或被阻擋，短間隔檢查
                    system_params = self.read_system_params()
                    check_interval = system_params["flow1_check_interval"] / 1000.0
                    time.sleep(check_interval)
                    
            except KeyboardInterrupt:
                self.logger.info("收到中斷信號，準備退出")
                break
            except Exception as e:
                self.logger.error(f"主循環異常: {e}", exc_info=True)
                time.sleep(1.0)
        
        # 清理資源
        self.stop_feeding()
        self.disconnect()
        self.logger.info("程序已退出")


def main():
    """主程序入口"""
    print("=== DR AutoFeeding獨立模組啟動 (增強版) ===")
    print("基地址範圍: 900-999")
    print("主要功能:")
    print("  ✓ 外部控制啟動/停止 (920=0停止, 920=1啟動)")
    print("  ✓ 可調整VP震動參數 (930-949寄存器)")
    print("  ✓ 可調整Flow4直振參數 (950-959寄存器)")
    print("  ✓ 可調整系統運行參數 (980-999寄存器)")
    print("  ✓ 記憶體使用量監控 (906寄存器，每10分鐘更新)")
    print("  ✓ 自動初始化硬編碼參數 (檢測到寄存器全為0時)")
    print("  ✓ 監控當前執行Flow地址(1201)和進度(1202)")
    print("  ✓ 當1201=1且1202<44時停止所有操作")
    print("  ✓ 異常數量檢測和二次重檢")
    print("  ✓ 持續檢測保持DR_F可用")
    print("  ✓ 簡化交握邏輯")
    print("  ✓ Flow1直接讀取座標(960-964)")
    print("  ✓ DR保護區域: X(-112~-4mm), Y(243~339.21mm)")
    print("  ✓ 完整logging記錄")
    print()
    print("*** 重要提醒 ***")
    print("請確保以下模組已啟動:")
    print("  1. 主Modbus TCP Server (端口502)")
    print("  2. CCD1視覺檢測模組 (CCD1VisionCodeYOLO.py)")
    print("  3. VP震動盤模組")
    print("  4. 然後啟動本AutoFeeding模組")
    print()
    print("*** 外部控制方式 ***") 
    print("  • 啟動AutoFeeding: 寫入920寄存器值為1")
    print("  • 停止AutoFeeding: 寫入920寄存器值為0")
    print("  • 調整VP參數: 寫入930-949寄存器")
    print("  • 調整Flow4參數: 寫入950-959寄存器")
    print("  • 調整系統參數: 寫入980-999寄存器")
    print("  • 監控記憶體使用: 讀取906寄存器(MB)")
    print()
    
    # 檢查Modbus可用性
    if not MODBUS_AVAILABLE:
        print("[ERROR] pymodbus未安裝，請安裝: pip install pymodbus==3.9.2")
        return
    
    # 檢查psutil可用性
    try:
        import psutil
    except ImportError:
        print("[ERROR] psutil未安裝，請安裝: pip install psutil")
        return
    
    # 創建AutoFeeding模組
    autofeeding = AutoFeedingModule()
    
    # 連接Modbus
    if not autofeeding.connect():
        print("[ERROR] Modbus連接失敗，程序退出")
        print("[建議] 請檢查主Modbus TCP Server是否在127.0.0.1:502運行")
        return
    
    # 啟動主循環
    autofeeding.main_loop()


if __name__ == "__main__":
    main()