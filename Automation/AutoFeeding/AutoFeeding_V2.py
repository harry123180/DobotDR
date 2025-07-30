#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoFeeding_main.py - DR專案獨立入料檢測模組 (超強化版)
基地址：900-999
新增功能：
1. 監控200地址是否卡16，自動歸零
2. 監控CCD1報警狀態，自動初始化
3. 監控記憶體使用量，自動重載
4. 增強的CCD1狀態監控
"""

import time
import math
import os
import json
import logging
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
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
    os.makedirs(log_dir, exist_ok=True)
    
    formatter = logging.Formatter(
        '%(asctime)s [%(levelname)s] %(name)s:%(funcName)s:%(lineno)d - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    file_handler = RotatingFileHandler(
        os.path.join(log_dir, f'{module_name}.log'),
        maxBytes=10*1024*1024,
        backupCount=7,
        encoding='utf-8'
    )
    file_handler.setFormatter(formatter)
    
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    
    logger = logging.getLogger(module_name)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger


class AutoFeedingStatus(Enum):
    """AutoFeeding狀態"""
    STOPPED = 0
    RUNNING = 1
    FLOW1_PAUSED = 2
    DETECTING = 3
    VP_VIBRATING = 4
    ERROR = 5
    CCD1_RECOVERY = 6    # 新增：CCD1恢復中
    MEMORY_RECOVERY = 7  # 新增：記憶體恢復中


class OperationStatus(Enum):
    """操作狀態"""
    IDLE = 0
    CCD_DETECTING = 1
    VP_CONTROLLING = 2
    FLOW4_TRIGGERING = 3
    CCD1_INIT_RECOVERY = 4  # 新增：CCD1初始化恢復
    MEMORY_RELOAD = 5       # 新增：記憶體重載


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
    
    def __init__(self, points=None):
        if points is None:
            points = [
                (-127.83, 194.7),
                (-113.75, 348.31),
                (6.21, 348.25),
                (6.20, 194.76)
            ]
        self.points = points

    def is_point_in_rect(self, x, y):
        """射線投射法判斷點是否在多邊形內"""
        n = len(self.points)
        inside = False
        j = n - 1
        
        for i in range(n):
            xi, yi = self.points[i]
            xj, yj = self.points[j]
            
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
            
        return inside


class AutoFeedingModule:
    """DR AutoFeeding獨立模組 (超強化版)"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
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
        self.CURRENT_MOTION_FLOW = 1201
        self.CURRENT_PROGRESS = 1202
        
        # 載入配置
        self.config = self.load_config()
        
        # 保護區域判斷
        self.protection_zone = ProtectionZone()
        
        # 系統狀態
        self.status = AutoFeedingStatus.STOPPED
        self.operation_status = OperationStatus.IDLE
        self.running = False
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
        
        # 🔥 新增：CCD1監控相關變數
        self.ccd1_command_stuck_count = 0       # 指令卡住計數
        self.ccd1_alarm_recovery_count = 0      # 報警恢復計數
        self.last_ccd1_command = 0              # 上次CCD1指令
        self.last_ccd1_status = None            # 上次CCD1狀態
        self.ccd1_stuck_threshold = 5           # 指令卡住閾值
        
        # 🔥 新增：記憶體監控相關變數
        self.memory_usage_check_interval = 10   # 記憶體檢查間隔(秒)
        self.last_memory_check_time = 0         # 上次記憶體檢查時間
        self.memory_threshold = 500             # 記憶體閾值(MB)
        self.memory_reload_in_progress = False  # 記憶體重載進行中
        self.memory_reload_count = 0            # 記憶體重載計數
        
        # 異常檢測
        self.anomaly_detection = AnomalyDetection()
        
        self.logger.info(f"DR AutoFeeding獨立模組初始化 (超強化版) - 基地址{self.BASE_ADDRESS}")
        self.logger.info(f"新增功能:")
        self.logger.info(f"  ✓ CCD1指令卡住監控(200地址)")
        self.logger.info(f"  ✓ CCD1報警狀態監控與自動初始化")
        self.logger.info(f"  ✓ 記憶體使用量監控(295地址)與自動重載(296地址)")
        self.logger.info(f"  ✓ 增強的狀態恢復機制")
    
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "autofeeding": {
                "cycle_interval": 1.0,
                "ccd1_timeout": 5.0,
                "flow4_consecutive_limit": 5,
                "vp_empty_check_count": 3,
                "auto_start": True,
                "progress_threshold": 44
            },
            "anomaly_detection": {
                "large_count_threshold": 5,
                "sudden_drop_threshold": 1,
                "redetection_delay": 0.5
            },
            "ccd1_monitoring": {                    # 🔥 新增：CCD1監控配置
                "command_stuck_threshold": 5,       # 指令卡住閾值
                "alarm_recovery_timeout": 10.0,     # 報警恢復超時
                "status_check_interval": 0.5        # 狀態檢查間隔
            },
            "memory_monitoring": {                  # 🔥 新增：記憶體監控配置
                "check_interval": 10,               # 檢查間隔(秒)
                "threshold_mb": 500,                # 記憶體閾值(MB)
                "reload_timeout": 30.0              # 重載超時
            },
            "vp_params": {
                "spread_action_code": 4,
                "spread_strength": 45,
                "spread_frequency": 43,
                "spread_duration": 0.3,
                "stop_command_code": 3,
                "stop_delay": 0.1
            },
            "flow4_params": {
                "pulse_duration": 0.1,
                "pulse_interval": 0.05
            },
            "timing": {
                "command_delay": 0.05,
                "status_check_interval": 0.05,
                "register_clear_delay": 0.02,
                "vp_stabilize_delay": 0.15,
                "flow1_check_interval": 0.1
            },
            "coordination": {
                "coords_taken_timeout": 10.0
            }
        }
        
        try:
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'autofeeding_config.json')
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                self.logger.info(f"配置檔案已載入: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                self.logger.info(f"預設配置檔案已創建: {config_path}")
        except Exception as e:
            self.logger.error(f"配置檔案處理失敗: {e}", exc_info=True)
            
        return default_config
    
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
                self.init_registers()
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
            self.modbus_client.close()
            self.connected = False
            self.logger.info("Modbus連接已斷開")
    
    def init_registers(self):
        """初始化寄存器"""
        try:
            # 狀態寄存器 (900-919)
            self.write_register(900, AutoFeedingStatus.STOPPED.value)
            self.write_register(901, self.cycle_count)
            self.write_register(902, self.dr_f_found_count)
            self.write_register(903, self.flow4_trigger_count)
            self.write_register(904, self.vp_vibration_count)
            self.write_register(905, self.anomaly_redetection_count)
            self.write_register(906, self.ccd1_alarm_recovery_count)    # 🔥 新增
            self.write_register(907, self.error_code)
            self.write_register(908, OperationStatus.IDLE.value)
            self.write_register(909, 0)
            self.write_register(910, 0)
            
            # 🔥 新增：CCD1監控寄存器 (911-915)
            self.write_register(911, 0)  # CCD1指令卡住標誌
            self.write_register(912, 0)  # CCD1報警恢復標誌
            self.write_register(913, 0)  # 記憶體重載標誌
            self.write_register(914, self.memory_reload_count)
            self.write_register(915, 0)  # 保留
            
            # DR_F狀態寄存器 (940-959)
            self.write_register(940, 0)
            self.write_register(941, 0)
            self.write_register(942, 0)
            self.write_register(943, 0)
            self.write_register(944, 0)
            self.write_register(945, 0)
            self.write_register(946, 0)
            self.write_register(947, 0)
            
            # 配置參數寄存器 (960-979)
            self.write_register(960, int(self.config['autofeeding']['cycle_interval'] * 1000))
            self.write_register(961, int(self.config['autofeeding']['ccd1_timeout'] * 1000))
            self.write_register(962, self.config['vp_params']['spread_strength'])
            self.write_register(963, self.config['vp_params']['spread_frequency'])
            self.write_register(964, int(self.config['vp_params']['spread_duration'] * 1000))
            
            self.logger.info("寄存器初始化完成 (新增CCD1監控和記憶體管理)")
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
        
        combined = (high_val << 16) + low_val
        
        if combined >= 2147483648:
            combined = combined - 4294967296
        
        return combined / 100.0
    
    def write_32bit_register(self, high_addr: int, low_addr: int, value: float) -> bool:
        """寫入32位世界座標"""
        value_int = int(value * 100)
        
        if value_int < 0:
            value_int = value_int + 4294967296
        
        high_val = (value_int >> 16) & 0xFFFF
        low_val = value_int & 0xFFFF
        
        success = True
        success &= self.write_register(high_addr, high_val)
        success &= self.write_register(low_addr, low_val)
        
        return success
    
    def check_ccd1_command_stuck(self) -> bool:
        """🔥 新增：檢查CCD1指令是否卡住"""
        try:
            current_command = self.read_register(200)  # 控制指令
            
            if current_command is None:
                return False
            
            # 檢查指令是否卡在16
            if current_command == 16:
                if self.last_ccd1_command == 16:
                    self.ccd1_command_stuck_count += 1
                    self.logger.debug(f"CCD1指令卡住檢測: 連續{self.ccd1_command_stuck_count}次檢測到16")
                    
                    # 達到閾值，執行清除操作
                    if self.ccd1_command_stuck_count >= self.ccd1_stuck_threshold:
                        self.logger.warning(f"檢測到CCD1指令卡住(200={current_command})，執行歸零操作")
                        success = self.write_register(200, 0)
                        if success:
                            self.logger.info("CCD1指令已歸零，重置卡住計數")
                            self.ccd1_command_stuck_count = 0
                            self.write_register(911, 1)  # 設置卡住標誌
                            return True
                        else:
                            self.logger.error("CCD1指令歸零失敗")
                else:
                    self.ccd1_command_stuck_count = 1
            else:
                self.ccd1_command_stuck_count = 0
                
            self.last_ccd1_command = current_command
            return False
            
        except Exception as e:
            self.logger.error(f"CCD1指令檢查失敗: {e}", exc_info=True)
            return False
    
    def check_ccd1_alarm_status(self) -> bool:
        """🔥 新增：檢查CCD1報警狀態並自動初始化"""
        try:
            ccd1_status = self.read_register(201)
            
            if ccd1_status is None:
                return False
            
            # 解析狀態位
            ccd1_alarm = bool(ccd1_status & 0x04)  # bit2=Alarm
            
            if ccd1_alarm:
                # 狀態變化時記錄
                if self.last_ccd1_status is None or not bool(self.last_ccd1_status & 0x04):
                    self.logger.warning(f"檢測到CCD1進入報警狀態 (201={ccd1_status})")
                    
                # 執行初始化恢復
                self.logger.info("執行CCD1自動初始化恢復...")
                success = self.write_register(200, 32)  # 初始化指令
                
                if success:
                    self.logger.info("CCD1初始化指令已發送(200=32)")
                    self.ccd1_alarm_recovery_count += 1
                    self.write_register(912, 1)  # 設置報警恢復標誌
                    
                    # 等待初始化完成
                    timeout = self.config['ccd1_monitoring']['alarm_recovery_timeout']
                    start_time = time.time()
                    
                    while (time.time() - start_time) < timeout:
                        time.sleep(0.5)
                        current_status = self.read_register(201)
                        if current_status is not None:
                            current_alarm = bool(current_status & 0x04)
                            if not current_alarm:
                                self.logger.info("CCD1報警狀態已清除，初始化成功")
                                # 清除控制指令
                                self.write_register(200, 0)
                                return True
                    
                    self.logger.warning("CCD1初始化超時，報警狀態未清除")
                else:
                    self.logger.error("CCD1初始化指令發送失敗")
                    
            self.last_ccd1_status = ccd1_status
            return False
            
        except Exception as e:
            self.logger.error(f"CCD1報警檢查失敗: {e}", exc_info=True)
            return False
    
    def check_memory_usage(self) -> bool:
        """🔥 新增：檢查記憶體使用量並觸發重載"""
        try:
            current_time = time.time()
            
            # 檢查時間間隔
            if current_time - self.last_memory_check_time < self.memory_monitoring_check_interval:
                return False
            
            self.last_memory_check_time = current_time
            
            # 讀取記憶體使用量(MB)
            memory_usage = self.read_register(295)
            
            if memory_usage is None:
                return False
            
            # 檢查是否超過閾值
            if memory_usage > self.memory_threshold and not self.memory_reload_in_progress:
                self.logger.warning(f"記憶體使用量超過閾值: {memory_usage}MB > {self.memory_threshold}MB")
                self.logger.info("觸發CCD1系統重載...")
                
                # 觸發重載
                success = self.write_register(296, 1)  # 系統重載觸發
                
                if success:
                    self.logger.info("系統重載指令已發送(296=1)")
                    self.memory_reload_in_progress = True
                    self.write_register(913, 1)  # 設置記憶體重載標誌
                    
                    # 等待重載完成
                    timeout = self.config['memory_monitoring']['reload_timeout']
                    start_time = time.time()
                    
                    while (time.time() - start_time) < timeout:
                        time.sleep(1.0)
                        reload_status = self.read_register(297)  # 重載狀態
                        
                        if reload_status == 1:
                            self.logger.info("檢測到重載狀態變為1，清除重載觸發")
                            # 清除重載觸發
                            self.write_register(296, 0)
                            
                            # 等待重載完成(狀態回到0)
                            while (time.time() - start_time) < timeout:
                                time.sleep(1.0)
                                reload_status = self.read_register(297)
                                if reload_status == 0:
                                    self.logger.info("重載完成，記憶體恢復正常")
                                    self.memory_reload_in_progress = False
                                    self.memory_reload_count += 1
                                    # 檢查重載後記憶體
                                    new_memory = self.read_register(295)
                                    if new_memory:
                                        self.logger.info(f"重載後記憶體使用量: {new_memory}MB")
                                    return True
                            
                            self.logger.warning("重載狀態未回到0，可能重載失敗")
                            break
                    
                    self.logger.error("記憶體重載超時")
                    self.memory_reload_in_progress = False
                else:
                    self.logger.error("記憶體重載觸發失敗")
            
            return False
            
        except Exception as e:
            self.logger.error(f"記憶體檢查失敗: {e}", exc_info=True)
            return False
    
    @property 
    def memory_monitoring_check_interval(self):
        """獲取記憶體監控間隔"""
        return self.config['memory_monitoring']['check_interval']
    
    def update_status_registers(self):
        """更新狀態寄存器"""
        try:
            self.write_register(900, self.status.value)
            self.write_register(901, self.cycle_count)
            self.write_register(902, self.dr_f_found_count)
            self.write_register(903, self.flow4_trigger_count)
            self.write_register(904, self.vp_vibration_count)
            self.write_register(905, self.anomaly_redetection_count)
            self.write_register(906, self.ccd1_alarm_recovery_count)
            self.write_register(907, self.error_code)
            self.write_register(908, self.operation_status.value)
            self.write_register(909, 1 if self.flow1_active else 0)
            self.write_register(910, 1 if self.progress_blocking else 0)
            self.write_register(914, self.memory_reload_count)
            
            # 更新DR_F狀態
            self.write_register(940, 1 if self.dr_f_available else 0)
            if self.dr_f_available:
                self.write_32bit_register(941, 942, self.dr_f_coords[0])
                self.write_32bit_register(943, 944, self.dr_f_coords[1])
        except Exception as e:
            self.logger.error(f"狀態寄存器更新失敗: {e}", exc_info=True)
    
    def check_flow_and_progress_status(self) -> bool:
        """監控Flow和進度狀態"""
        try:
            current_motion_flow = self.read_register(self.CURRENT_MOTION_FLOW)
            current_progress = self.read_register(self.CURRENT_PROGRESS)
            
            if current_motion_flow is None or current_progress is None:
                return False
            
            flow1_now_active = (current_motion_flow == 1)
            progress_threshold = self.config['autofeeding'].get('progress_threshold', 44)
            progress_now_blocking = (current_motion_flow == 1 and current_progress < progress_threshold)
            
            if flow1_now_active != self.flow1_active:
                self.flow1_active = flow1_now_active
                if self.flow1_active:
                    self.logger.info(f"檢測到Flow1正在執行 ({self.CURRENT_MOTION_FLOW}=1)")
                else:
                    self.logger.info(f"檢測到Flow1執行完成 ({self.CURRENT_MOTION_FLOW}=0)")
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
            coords_taken = self.read_register(945)
            if coords_taken == 1:
                self.logger.info("座標已被Flow1讀取，清除DR_F狀態")
                self.dr_f_available = False
                self.dr_f_coords = (0.0, 0.0)
                self.dr_f_taken = True
                
                self.write_register(940, 0)
                self.write_register(945, 0)
                for addr in [941, 942, 943, 944]:
                    self.write_register(addr, 0)
                
                self.logger.info("DR_F狀態已清除，繼續檢測新的正面物件")
        except Exception as e:
            self.logger.error(f"座標讀取檢查失敗: {e}", exc_info=True)
    
    def check_modules_status(self) -> bool:
        """檢查CCD1、VP模組狀態 - 增強版"""
        # 🔥 新增：檢查CCD1指令是否卡住
        if self.check_ccd1_command_stuck():
            self.logger.info("CCD1指令卡住問題已處理")
        
        # 🔥 新增：檢查CCD1報警狀態
        if self.check_ccd1_alarm_status():
            self.logger.info("CCD1報警狀態已處理")
            return False  # 報警恢復中，暫停檢測
        
        # 檢查CCD1狀態
        ccd1_status = self.read_register(201)
        if ccd1_status is None:
            if self.cycle_count % 50 == 1:
                self.logger.debug("CCD1模組無回應 (寄存器201無法讀取)")
            self.error_code = 101
            return False
        
        ccd1_ready = bool(ccd1_status & 0x01)
        ccd1_running = bool(ccd1_status & 0x02)
        ccd1_alarm = bool(ccd1_status & 0x04)
        ccd1_initialized = bool(ccd1_status & 0x08)
        
        if self.cycle_count % 100 == 1:
            self.logger.debug(f"CCD1狀態: {ccd1_status} (Ready={ccd1_ready}, Running={ccd1_running}, Alarm={ccd1_alarm}, Init={ccd1_initialized})")
        
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
        large_threshold = self.config['anomaly_detection'].get('large_count_threshold', 5)
        drop_threshold = self.config['anomaly_detection'].get('sudden_drop_threshold', 1)
        
        prev_had_large = (self.anomaly_detection.previous_dr_f_count >= large_threshold or 
                         self.anomaly_detection.previous_total_count >= large_threshold)
        
        current_has_drop = (current_result.dr_f_count <= drop_threshold and 
                           current_result.total_detections <= drop_threshold)
        
        if prev_had_large and current_has_drop:
            self.logger.warning(f"檢測到異常：前次大量檢測(DR_F={self.anomaly_detection.previous_dr_f_count}, 總數={self.anomaly_detection.previous_total_count})，當前驟減(DR_F={current_result.dr_f_count}, 總數={current_result.total_detections})")
            return True
        
        return False
    
    def update_anomaly_detection(self, result: CCD1DetectionResult):
        """更新異常檢測記錄"""
        self.anomaly_detection.previous_dr_f_count = result.dr_f_count
        self.anomaly_detection.previous_total_count = result.total_detections
    
    def trigger_ccd1_detection(self) -> CCD1DetectionResult:
        """觸發CCD1檢測"""
        self.operation_status = OperationStatus.CCD_DETECTING
        result = CCD1DetectionResult()
        
        initial_status = self.read_register(201)
        self.logger.debug(f"發送檢測指令前 - 201狀態: {initial_status}")
        time.sleep(1)
        
        if not self.write_register(200, 16):
            self.logger.error("無法寫入控制指令到寄存器200")
            self.error_code = 201
            return result
        
        self.logger.debug("已發送控制指令16到寄存器200")
        
        timeout = self.config['autofeeding']['ccd1_timeout']
        start_time = time.time()
        check_interval = 0.02
        
        while (time.time() - start_time) < timeout:
            capture_complete = self.read_register(203)
            detect_complete = self.read_register(204)
            operation_success = self.read_register(205)
            current_status = self.read_register(201)
            
            if self.cycle_count <= 5:
                self.logger.debug(f"檢測等待: 201={current_status}, 203={capture_complete}, 204={detect_complete}, 205={operation_success}")
            
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
        result.dr_f_count = self.read_register(240) or 0
        stack_count = self.read_register(242) or 0
        result.total_detections = self.read_register(243) or 0
        
        # 提取DR_F世界座標
        if result.dr_f_count > 0:
            for i in range(min(result.dr_f_count, 5)):
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
        """觸發VP震動"""
        self.operation_status = OperationStatus.VP_CONTROLLING
        self.logger.debug(f"VP震動參數: {self.config['vp_params']['spread_action_code']}, {self.config['vp_params']['spread_strength']}, {self.config['vp_params']['spread_frequency']}")
        
        success = True
        success &= self.write_register(320, 5)
        success &= self.write_register(321, self.config['vp_params']['spread_action_code'])
        success &= self.write_register(322, self.config['vp_params']['spread_strength'])
        success &= self.write_register(323, self.config['vp_params']['spread_frequency'])
        success &= self.write_register(324, int(time.time()) % 65535)
                
        if not success:
            self.error_code = 301
            return False
        
        time.sleep(1.2)
        success = True
        success &= self.write_register(320, 5)
        success &= self.write_register(321, 11)
        success &= self.write_register(322, 132)
        success &= self.write_register(323, 49)
        success &= self.write_register(324, int(time.time()) % 65535)
        
        if not success:
            self.error_code = 301
            return False
        
        time.sleep(1.8)
        success = True
        success &= self.write_register(320, 5)
        success &= self.write_register(321, 0)
        success &= self.write_register(322, 0)
        success &= self.write_register(323, 0)
        success &= self.write_register(324, int(time.time()) % 65535)
        if not success:
            self.error_code = 301
            return False
        
        time.sleep(0.8)
        
        return self.stop_vp_vibration()
    
    def stop_vp_vibration(self) -> bool:
        """停止VP震動"""
        success = True
        success &= self.write_register(320, self.config['vp_params']['stop_command_code'])
        success &= self.write_register(321, 0)
        success &= self.write_register(322, 0)
        success &= self.write_register(323, 0)
        success &= self.write_register(324, 99)
        
        if success:
            time.sleep(self.config['vp_params']['stop_delay'])
        
        return success
    
    def trigger_flow4_feeding(self) -> bool:
        """觸發Flow4送料"""
        self.operation_status = OperationStatus.FLOW4_TRIGGERING
        
        pulse_duration = self.config['flow4_params']['pulse_duration']
        
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
        
        self.write_register(940, 1)
        self.write_32bit_register(941, 942, coords[0])
        self.write_32bit_register(943, 944, coords[1])
        
        self.logger.info(f"DR_F已就緒: {coords}, Flow1可直接讀取座標")
    
    def feeding_cycle(self) -> bool:
        """執行一次入料檢測週期 - 超強化版本"""
        try:
            self.cycle_count += 1
            self.status = AutoFeedingStatus.DETECTING
            
            # 🔥 新增：記憶體使用量檢查
            if self.check_memory_usage():
                self.logger.info("記憶體重載操作已處理")
                return False  # 重載中暫停檢測
            
            # 快速檢查模組狀態（包含CCD1監控）
            if not self.check_modules_status():
                if self.error_code == 102:
                    self.status = AutoFeedingStatus.RUNNING
                    return True
                return False
            
            # CCD1檢測
            detection_result = self.trigger_ccd1_detection()
            if not detection_result.operation_success:
                self.logger.warning(f"週期{self.cycle_count} CCD1檢測失敗")
                return False
            
            self.logger.info(f"週期{self.cycle_count} 檢測結果: DR_F={detection_result.dr_f_count}, 總數={detection_result.total_detections}")
            
            # 異常檢測
            need_redetection = self.check_for_anomaly(detection_result)
            if need_redetection:
                self.logger.warning("執行異常重檢...")
                redetection_delay = self.config['anomaly_detection'].get('redetection_delay', 0.5)
                time.sleep(redetection_delay)
                
                retry_result = self.trigger_ccd1_detection()
                if retry_result.operation_success:
                    self.logger.info(f"重檢結果: DR_F={retry_result.dr_f_count}, 總數={retry_result.total_detections}")
                    detection_result = retry_result
                    self.anomaly_redetection_count += 1
                else:
                    self.logger.warning("重檢失敗，使用原檢測結果")
            
            self.update_anomaly_detection(detection_result)
            
            # 尋找保護區域內的DR_F
            target_coords = self.find_dr_f_in_protection_zone(detection_result)
            
            if target_coords:
                self.dr_f_found_count += 1
                self.flow4_consecutive_count = 0
                self.logger.info(f"找到保護區內DR_F: {target_coords}")
                self.set_dr_f_available(target_coords)
                
            elif detection_result.total_detections < 4:
                self.logger.info(f"料件不足 (總數={detection_result.total_detections}<4)，觸發Flow4送料")
                
                if self.trigger_flow4_feeding():
                    self.flow4_trigger_count += 1
                    self.flow4_consecutive_count += 1
                    self.logger.info(f"Flow4送料完成 (連續{self.flow4_consecutive_count}次)")
                    time.sleep(2)
                    self.trigger_vp_vibration()
                    
                    if self.flow4_consecutive_count >= self.config['autofeeding']['flow4_consecutive_limit']:
                        self.logger.warning("達到連續直振限制，需要VP清空")
                else:
                    self.logger.warning("Flow4送料失敗")
                
            else:
                self.logger.info(f"料件充足 (總數={detection_result.total_detections}>=4) 但無正面，VP震動重檢")
                self.flow4_consecutive_count = 0
                
                if self.trigger_vp_vibration():
                    self.vp_vibration_count += 1
                    self.logger.info("VP震動完成，等待穩定後重新檢測")
                    
                    time.sleep(self.config['timing']['vp_stabilize_delay'])
                    
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
        
        self.logger.info("啟動持續入料檢測 (超強化版)")
        self.logger.info("新增監控功能：")
        self.logger.info("  ✓ CCD1指令卡住檢測與自動清除")
        self.logger.info("  ✓ CCD1報警狀態監控與自動初始化")
        self.logger.info("  ✓ 記憶體使用量監控與自動重載")
        
        self.running = True
        self.status = AutoFeedingStatus.RUNNING
        self.error_code = 0
        
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
        """主循環 - 超強化版"""
        self.logger.info("主循環啟動 (超強化版)")
        self.logger.info("新增特性：")
        self.logger.info("  ✓ CCD1指令卡住監控 (200地址自動歸零)")
        self.logger.info("  ✓ CCD1報警狀態監控 (自動32初始化)")
        self.logger.info("  ✓ 記憶體使用量監控 (295>500MB觸發296重載)")
        self.logger.info("  ✓ 重載狀態確認 (297=1→0完成恢復)")
        self.logger.info("  ✓ 增強的異常恢復機制")
        
        auto_start = self.config['autofeeding'].get('auto_start', True)
        if auto_start:
            self.start_feeding()
        
        loop_count = 0
        
        while True:
            try:
                loop_count += 1
                
                if loop_count % 200 == 1:
                    self.logger.debug(f"主循環 {loop_count}: running={self.running}, status={self.status.name}, flow1_active={self.flow1_active}, progress_blocking={self.progress_blocking}, dr_f_available={self.dr_f_available}")
                
                if not self.connected:
                    self.logger.debug("Modbus連接斷開，嘗試重連")
                    if not self.connect():
                        time.sleep(5.0)
                        continue
                
                # 監控Flow和進度狀態
                self.check_flow_and_progress_status()
                
                # 檢查座標是否被讀取
                if self.dr_f_available:
                    self.check_coords_taken()
                
                # 更新狀態寄存器
                self.update_status_registers()
                
                # 執行入料檢測
                if self.running and not self.progress_blocking and not self.vp_clearing_mode and not self.memory_reload_in_progress:
                    if not self.feeding_cycle():
                        self.logger.debug(f"入料檢測失敗，錯誤碼: {self.error_code}")
                        self.status = AutoFeedingStatus.ERROR
                        time.sleep(0.5)
                    else:
                        cycle_interval = self.config['autofeeding']['cycle_interval']
                        time.sleep(cycle_interval)
                else:
                    time.sleep(self.config['timing']['flow1_check_interval'])
                    
            except KeyboardInterrupt:
                self.logger.info("收到中斷信號，準備退出")
                break
            except Exception as e:
                self.logger.error(f"主循環異常: {e}", exc_info=True)
                time.sleep(1.0)
        
        self.stop_feeding()
        self.disconnect()
        self.logger.info("程序已退出")


def main():
    """主程序入口 - 超強化版"""
    print("=== DR AutoFeeding獨立模組啟動 (超強化版) ===")
    print("基地址範圍: 900-999")
    print("新增監控功能:")
    print("  ✓ CCD1指令卡住檢測: 監控200地址，卡16時自動歸零")
    print("  ✓ CCD1報警狀態監控: 檢測201 bit2，報警時自動發送32初始化")
    print("  ✓ 記憶體使用量監控: 監控295地址，>500MB時觸發296重載")
    print("  ✓ 重載狀態確認: 確認297=1→0完成後恢復Feeding循環")
    print("  ✓ 增強的狀態寄存器: 911-915新增CCD1和記憶體監控狀態")
    print()
    print("*** 監控邏輯 ***")
    print("1. CCD1指令監控: 連續5次檢測到200=16時執行歸零")
    print("2. CCD1報警監控: 檢測到201 bit2=1時發送200=32初始化")
    print("3. 記憶體監控: 每10秒檢查295，>500MB時觸發296=1重載")
    print("4. 重載確認: 296=1→等待297=1→296=0→等待297=0→恢復")
    print()
    
    if not MODBUS_AVAILABLE:
        print("[ERROR] pymodbus未安裝，請安裝: pip install pymodbus==3.9.2")
        return
    
    autofeeding = AutoFeedingModule()
    
    if not autofeeding.connect():
        print("[ERROR] Modbus連接失敗，程序退出")
        print("[建議] 請檢查主Modbus TCP Server是否在127.0.0.1:502運行")
        return
    
    autofeeding.main_loop()


if __name__ == "__main__":
    main()