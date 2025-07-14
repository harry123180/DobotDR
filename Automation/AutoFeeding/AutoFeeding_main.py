#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoFeeding_main.py - DR專案獨立入料檢測模組
基地址：900-999
功能：持續檢測確保DR_F存在於保護區域內，監控1201當值為1時暫停自動進料程序
保護區域：(-112, 243) 到 (-4, 339)四點矩形
檢測對象：DR_F (正面物件) 和 STACK (堆疊物件) 二分類檢測
"""

import time
import math
import os
import json
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

# Modbus TCP Client (pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    print("[ERROR] pymodbus未安裝，請安裝: pip install pymodbus==3.9.2")
    MODBUS_AVAILABLE = False


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
    """DR AutoFeeding獨立模組"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
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
        
        # 監控1201當前執行Flow
        self.CURRENT_MOTION_FLOW = 1201  # 當前運動Flow (0=無, 1=Flow1, 2=Flow2, 5=Flow5)
        
        # 載入配置
        self.config = self.load_config()
        
        # 保護區域判斷
        self.protection_zone = ProtectionZone()
        
        # 系統狀態
        self.status = AutoFeedingStatus.STOPPED
        self.operation_status = OperationStatus.IDLE
        self.running = False
        self.flow1_active = False  # 監控Flow1是否正在執行
        self.vp_clearing_mode = False
        
        # DR_F狀態
        self.dr_f_available = False  # 保護區內是否有DR_F
        self.dr_f_coords = (0.0, 0.0)  # 當前DR_F座標
        self.dr_f_taken = False  # 座標是否已被讀取
        
        # 統計資訊
        self.cycle_count = 0
        self.dr_f_found_count = 0
        self.flow4_trigger_count = 0
        self.vp_vibration_count = 0
        self.flow4_consecutive_count = 0
        self.vp_empty_detection_count = 0
        self.error_code = 0
        
        print(f"[DR AutoFeeding] 獨立模組初始化 - 基地址{self.BASE_ADDRESS}")
        print(f"[DR AutoFeeding] 保護區域: X(-112.0 ~ -4.0mm), Y(243.0 ~ 339.21mm)")
        print(f"[DR AutoFeeding] 監控當前執行Flow地址: {self.CURRENT_MOTION_FLOW}")
        print(f"[DR AutoFeeding] 當1201=1時暫停自動進料程序")
    
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "autofeeding": {
                "cycle_interval": 1.0,     # 1秒檢測週期
                "ccd1_timeout": 5.0,       # CCD1檢測超時
                "flow4_consecutive_limit": 5,
                "vp_empty_check_count": 3,
                "auto_start": True         # 啟動後自動開始檢測
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
                "flow1_check_interval": 0.1  # Flow1監控間隔
            },
            "coordination": {
                "coords_taken_timeout": 10.0  # 座標被讀取超時
            }
        }
        
        try:
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'autofeeding_config.json')
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                print(f"[DR AutoFeeding] 配置檔案已載入: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"[DR AutoFeeding] 預設配置檔案已創建: {config_path}")
        except Exception as e:
            print(f"[ERROR] 配置檔案處理失敗: {e}")
            
        return default_config
    
    def connect(self) -> bool:
        """連接Modbus服務器"""
        try:
            if not MODBUS_AVAILABLE:
                print("[ERROR] Modbus功能不可用")
                return False
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.connected = self.modbus_client.connect()
            
            if self.connected:
                print(f"[DR AutoFeeding] Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
                self.init_registers()
                return True
            else:
                print(f"[ERROR] Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
                return False
        except Exception as e:
            print(f"[ERROR] Modbus連接異常: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        if self.modbus_client and self.connected:
            self.modbus_client.close()
            self.connected = False
            print("[DR AutoFeeding] Modbus連接已斷開")
    
    def init_registers(self):
        """初始化寄存器"""
        try:
            # 狀態寄存器 (900-919)
            self.write_register(900, AutoFeedingStatus.STOPPED.value)  # 模組狀態
            self.write_register(901, self.cycle_count)                  # 週期計數
            self.write_register(902, self.dr_f_found_count)            # DR_F找到次數
            self.write_register(903, self.flow4_trigger_count)         # Flow4觸發次數
            self.write_register(904, self.vp_vibration_count)          # VP震動次數
            self.write_register(905, 0)  # 保留
            self.write_register(906, 0)  # 保留
            self.write_register(907, 0)  # 錯誤代碼
            self.write_register(908, OperationStatus.IDLE.value)       # 操作狀態
            self.write_register(909, 0)  # Flow1監控狀態
            
            # DR_F狀態寄存器 (940-959) - 簡化交握
            self.write_register(940, 0)  # DR_F可用標誌 (0=無, 1=有)
            self.write_register(941, 0)  # DR_F座標X高位
            self.write_register(942, 0)  # DR_F座標X低位
            self.write_register(943, 0)  # DR_F座標Y高位
            self.write_register(944, 0)  # DR_F座標Y低位
            self.write_register(945, 0)  # 座標已讀取標誌 (Flow1設置)
            self.write_register(946, 0)  # 保留
            self.write_register(947, 0)  # 保留
            
            # 配置參數寄存器 (960-979)
            self.write_register(960, int(self.config['autofeeding']['cycle_interval'] * 1000))
            self.write_register(961, int(self.config['autofeeding']['ccd1_timeout'] * 1000))
            self.write_register(962, self.config['vp_params']['spread_strength'])
            self.write_register(963, self.config['vp_params']['spread_frequency'])
            self.write_register(964, int(self.config['vp_params']['spread_duration'] * 1000))
            
            print("[DR AutoFeeding] 寄存器初始化完成")
        except Exception as e:
            print(f"[ERROR] 寄存器初始化失敗: {e}")
    
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
    
    def update_status_registers(self):
        """更新狀態寄存器"""
        try:
            self.write_register(900, self.status.value)
            self.write_register(901, self.cycle_count)
            self.write_register(902, self.dr_f_found_count)
            self.write_register(903, self.flow4_trigger_count)
            self.write_register(904, self.vp_vibration_count)
            self.write_register(907, self.error_code)
            self.write_register(908, self.operation_status.value)
            self.write_register(909, 1 if self.flow1_active else 0)
            
            # 更新DR_F狀態
            self.write_register(940, 1 if self.dr_f_available else 0)
            if self.dr_f_available:
                self.write_32bit_register(941, 942, self.dr_f_coords[0])
                self.write_32bit_register(943, 944, self.dr_f_coords[1])
        except Exception as e:
            print(f"[ERROR] 狀態寄存器更新失敗: {e}")
    
    def check_flow1_status(self) -> bool:
        """主動監控當前執行Flow狀態 - 監控1201"""
        try:
            current_motion_flow = self.read_register(self.CURRENT_MOTION_FLOW)
            if current_motion_flow is None:
                return False
            
            # 檢查當前執行Flow狀態變化
            flow1_now_active = (current_motion_flow == 1)
            
            if flow1_now_active != self.flow1_active:
                # Flow1狀態變化
                self.flow1_active = flow1_now_active
                if self.flow1_active:
                    print(f"[DR AutoFeeding] 檢測到Flow1正在執行 ({self.CURRENT_MOTION_FLOW}=1)，暫停檢測")
                    if self.status == AutoFeedingStatus.RUNNING:
                        self.status = AutoFeedingStatus.FLOW1_PAUSED
                else:
                    print(f"[DR AutoFeeding] 檢測到Flow1執行完成 ({self.CURRENT_MOTION_FLOW}=0)，恢復檢測")
                    if self.status == AutoFeedingStatus.FLOW1_PAUSED:
                        self.status = AutoFeedingStatus.RUNNING
                        # Flow1完成後，檢查座標是否被讀取
                        self.check_coords_taken()
            
            return True
        except Exception as e:
            print(f"[ERROR] Flow1狀態檢查失敗: {e}")
            return False
    
    def check_coords_taken(self):
        """檢查座標是否被Flow1讀取"""
        if not self.dr_f_available:
            return
        
        try:
            coords_taken = self.read_register(945)  # Flow1設置此標誌表示已讀取座標
            if coords_taken == 1:
                print(f"[DR AutoFeeding] 座標已被Flow1讀取，清除DR_F狀態")
                # 清除DR_F狀態，繼續檢測新的
                self.dr_f_available = False
                self.dr_f_coords = (0.0, 0.0)
                self.dr_f_taken = True
                
                # 清除相關寄存器
                self.write_register(940, 0)  # DR_F可用標誌
                self.write_register(945, 0)  # 座標已讀取標誌
                for addr in [941, 942, 943, 944]:
                    self.write_register(addr, 0)
                
                print(f"[DR AutoFeeding] DR_F狀態已清除，繼續檢測新的正面物件")
        except Exception as e:
            print(f"[ERROR] 座標讀取檢查失敗: {e}")
    
    def check_modules_status(self) -> bool:
        """檢查CCD1、VP模組狀態"""
        # 檢查CCD1狀態
        ccd1_status = self.read_register(201)
        if ccd1_status is None:
            if self.cycle_count % 50 == 1:
                print(f"[DEBUG] CCD1模組無回應 (寄存器201無法讀取)")
            self.error_code = 101
            return False
        
        # 解析CCD1狀態位
        ccd1_ready = bool(ccd1_status & 0x01)
        ccd1_running = bool(ccd1_status & 0x02)
        ccd1_alarm = bool(ccd1_status & 0x04)
        ccd1_initialized = bool(ccd1_status & 0x08)
        
        if self.cycle_count % 100 == 1:  # 減少打印頻率
            print(f"[DEBUG] CCD1狀態: {ccd1_status} (Ready={ccd1_ready}, Running={ccd1_running}, Alarm={ccd1_alarm}, Init={ccd1_initialized})")
        
        # 檢查CCD1是否準備就緒 (應該是狀態9: Ready=1, Initialized=1)
        if ccd1_alarm:
            if self.cycle_count % 50 == 1:
                error_code = self.read_register(206)
                print(f"[DEBUG] CCD1處於警報狀態，錯誤代碼: {error_code}")
            self.error_code = 102
            return False
        
        if not ccd1_initialized:
            if self.cycle_count % 50 == 1:
                print(f"[DEBUG] CCD1尚未初始化完成，等待...")
            self.error_code = 102
            return False
        
        if not ccd1_ready:
            if self.cycle_count % 50 == 1:
                print(f"[DEBUG] CCD1未Ready (可能正在執行其他任務)，等待...")
            self.error_code = 102
            return False
        
        # 檢查VP狀態
        vp_status = self.read_register(300)
        vp_connected = self.read_register(301)
        
        if vp_status is None or vp_connected is None:
            if self.cycle_count % 50 == 1:
                print(f"[DEBUG] VP模組無回應")
            self.error_code = 103
            return False
        
        if vp_status != 1 or vp_connected != 1:
            if self.cycle_count % 50 == 1:
                print(f"[DEBUG] VP模組狀態異常: status={vp_status}, connected={vp_connected}")
            self.error_code = 103
            return False
        
        return True
    
    def trigger_ccd1_detection(self) -> CCD1DetectionResult:
        """觸發CCD1檢測 - 適配DR專案的YOLO檢測"""
        self.operation_status = OperationStatus.CCD_DETECTING
        result = CCD1DetectionResult()
        
        # 調試: 檢查指令發送前的狀態
        initial_status = self.read_register(201)
        print(f"[DEBUG] 發送檢測指令前 - 201狀態: {initial_status}")
        
        # 觸發拍照+檢測
        if not self.write_register(200, 16):
            print(f"[ERROR] 無法寫入控制指令到寄存器200")
            self.error_code = 201
            return result
        
        print(f"[DEBUG] 已發送控制指令16到寄存器200")
        
        # 等待檢測完成
        timeout = self.config['autofeeding']['ccd1_timeout']
        start_time = time.time()
        check_interval = 0.02
        
        while (time.time() - start_time) < timeout:
            capture_complete = self.read_register(203)
            detect_complete = self.read_register(204)
            operation_success = self.read_register(205)
            current_status = self.read_register(201)
            
            # 調試信息
            if self.cycle_count <= 5:  # 只在前5次打印調試信息
                print(f"[DEBUG] 檢測等待: 201={current_status}, 203={capture_complete}, 204={detect_complete}, 205={operation_success}")
            
            if capture_complete == 1 and detect_complete == 1 and operation_success == 1:
                result.operation_success = True
                print(f"[DEBUG] CCD1檢測完成標誌確認")
                break
            
            time.sleep(check_interval)
        
        if not result.operation_success:
            elapsed_time = time.time() - start_time
            print(f"[ERROR] CCD1檢測超時 ({elapsed_time:.2f}s)，最終狀態:")
            print(f"[ERROR]   201狀態: {self.read_register(201)}")
            print(f"[ERROR]   203拍照完成: {self.read_register(203)}")
            print(f"[ERROR]   204檢測完成: {self.read_register(204)}")
            print(f"[ERROR]   205操作成功: {self.read_register(205)}")
            print(f"[ERROR]   206錯誤代碼: {self.read_register(206)}")
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
        """觸發VP震動"""
        self.operation_status = OperationStatus.VP_CONTROLLING
        print(self.config['vp_params']['spread_action_code'],self.config['vp_params']['spread_strength'],self.config['vp_params']['spread_frequency'])
        # 啟動震動
        success = True
        success &= self.write_register(320, 5)  # execute_action
        success &= self.write_register(321, self.config['vp_params']['spread_action_code'])
        success &= self.write_register(322, self.config['vp_params']['spread_strength'])
        success &= self.write_register(323, self.config['vp_params']['spread_frequency'])
        success &= self.write_register(324, int(time.time()) % 65535)
        
        if not success:
            self.error_code = 301
            return False
        
        # 震動持續時間
        time.sleep(self.config['vp_params']['spread_duration'])
        success = True
        success &= self.write_register(320, 5)  # execute_action
        success &= self.write_register(321, 11) #擴散
        success &= self.write_register(322, 82)#強度
        success &= self.write_register(323, 41)#頻率
        success &= self.write_register(324, int(time.time()) % 65535)
        
        if not success:
            self.error_code = 301
            return False
        
        # 震動持續時間
        time.sleep(self.config['vp_params']['spread_duration'])
        
        # 停止震動
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
        
        # 立即更新寄存器讓Flow1可以讀取
        self.write_register(940, 1)  # DR_F可用標誌
        self.write_32bit_register(941, 942, coords[0])  # X座標
        self.write_32bit_register(943, 944, coords[1])  # Y座標
        
        print(f"[DR AutoFeeding] DR_F已就緒: {coords}, Flow1可直接讀取座標")
    
    def feeding_cycle(self) -> bool:
        """執行一次入料檢測週期"""
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
                print(f"[DR AutoFeeding] 週期{self.cycle_count} CCD1檢測失敗")
                return False
            
            print(f"[DR AutoFeeding] 週期{self.cycle_count} 檢測結果: DR_F={detection_result.dr_f_count}, 總數={detection_result.total_detections}")
            
            # 尋找保護區域內的DR_F
            target_coords = self.find_dr_f_in_protection_zone(detection_result)
            
            if target_coords:
                # 找到正面物件 - 設置可用狀態，但繼續檢測
                self.dr_f_found_count += 1
                self.flow4_consecutive_count = 0
                print(f"[DR AutoFeeding] 找到保護區內DR_F: {target_coords}")
                
                # 設置DR_F可用狀態
                self.set_dr_f_available(target_coords)
                
                # 不暫停，繼續檢測確保持續有料件
                
            elif detection_result.total_detections < 4:
                # 料件不足，觸發Flow4送料
                print(f"[DR AutoFeeding] 料件不足 (總數={detection_result.total_detections}<4)，觸發Flow4送料")
                
                if self.trigger_flow4_feeding():
                    self.flow4_trigger_count += 1
                    self.flow4_consecutive_count += 1
                    print(f"[DR AutoFeeding] Flow4送料完成 (連續{self.flow4_consecutive_count}次)")
                    
                    # 檢查連續直振限制
                    if self.flow4_consecutive_count >= self.config['autofeeding']['flow4_consecutive_limit']:
                        print("[DR AutoFeeding] 達到連續直振限制，需要VP清空")
                        # 這裡可以加入VP清空流程或報警
                else:
                    print(f"[DR AutoFeeding] Flow4送料失敗")
                
            else:
                # 料件充足但無正面，VP震動重檢
                print(f"[DR AutoFeeding] 料件充足 (總數={detection_result.total_detections}>=4) 但無正面，VP震動重檢")
                self.flow4_consecutive_count = 0
                
                if self.trigger_vp_vibration():
                    self.vp_vibration_count += 1
                    print(f"[DR AutoFeeding] VP震動完成，等待穩定後重新檢測")
                    
                    # 等待穩定
                    time.sleep(self.config['timing']['vp_stabilize_delay'])
                    
                    # 立即重新檢測
                    retry_result = self.trigger_ccd1_detection()
                    if retry_result.operation_success:
                        print(f"[DR AutoFeeding] 震動後重檢: DR_F={retry_result.dr_f_count}, 總數={retry_result.total_detections}")
                        retry_coords = self.find_dr_f_in_protection_zone(retry_result)
                        if retry_coords:
                            self.dr_f_found_count += 1
                            print(f"[DR AutoFeeding] 震動後找到DR_F: {retry_coords}")
                            self.set_dr_f_available(retry_coords)
            
            self.operation_status = OperationStatus.IDLE
            self.status = AutoFeedingStatus.RUNNING
            return True
            
        except Exception as e:
            print(f"[ERROR] 入料週期異常: {e}")
            self.error_code = 999
            return False
    
    def start_feeding(self):
        """啟動入料檢測"""
        if self.running:
            return
        
        print("[DR AutoFeeding] 啟動持續入料檢測")
        print("[DR AutoFeeding] 目標：保持保護區域內始終有DR_F可用")
        
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
        print("[DR AutoFeeding] 入料檢測已停止")
    
    def emergency_stop_vp(self):
        """緊急停止VP"""
        try:
            self.stop_vp_vibration()
            print("[DR AutoFeeding] VP緊急停止")
        except:
            pass
    
    def main_loop(self):
        """主循環"""
        print("[DR AutoFeeding] 主循環啟動")
        print("[DR AutoFeeding] 特性：")
        print("  ✓ 主動監控當前執行Flow狀態 (1201)")
        print("  ✓ 當1201=1時暫停自動進料程序")
        print("  ✓ 持續檢測確保DR_F可用")
        print("  ✓ 簡化交握邏輯")
        print("  ✓ Flow1直接讀取座標")
        print("  ✓ DR保護區域: X(-112~-4mm), Y(243~339.21mm)")
        
        # 自動啟動檢測
        auto_start = self.config['autofeeding'].get('auto_start', True)
        if auto_start:
            self.start_feeding()
        
        loop_count = 0
        
        while True:
            try:
                loop_count += 1
                
                # 定期打印狀態
                if loop_count % 200 == 1:
                    print(f"[DEBUG] 主循環 {loop_count}: running={self.running}, status={self.status.name}, flow1_active={self.flow1_active}, dr_f_available={self.dr_f_available}")
                
                # 檢查連接狀態
                if not self.connected:
                    print("[DEBUG] Modbus連接斷開，嘗試重連")
                    if not self.connect():
                        time.sleep(5.0)
                        continue
                
                # 主動監控Flow1狀態 - 監控1201
                self.check_flow1_status()
                
                # 檢查座標是否被讀取
                if self.dr_f_available:
                    self.check_coords_taken()
                
                # 更新狀態寄存器
                self.update_status_registers()
                
                # 執行入料檢測 - 只有在運行且Flow1未啟動時
                if self.running and not self.flow1_active and not self.vp_clearing_mode:
                    if not self.feeding_cycle():
                        print(f"[DEBUG] 入料檢測失敗，錯誤碼: {self.error_code}")
                        self.status = AutoFeedingStatus.ERROR
                        time.sleep(0.5)
                    else:
                        # 檢測成功，快速進入下一輪
                        cycle_interval = self.config['autofeeding']['cycle_interval']
                        time.sleep(cycle_interval)
                else:
                    # 非運行狀態或Flow1執行中，短間隔檢查
                    time.sleep(self.config['timing']['flow1_check_interval'])
                    
            except KeyboardInterrupt:
                print("\n[DR AutoFeeding] 收到中斷信號，準備退出")
                break
            except Exception as e:
                print(f"[ERROR] 主循環異常: {e}")
                time.sleep(1.0)
        
        # 清理資源
        self.stop_feeding()
        self.disconnect()
        print("[DR AutoFeeding] 程序已退出")


def main():
    """主程序入口"""
    print("=== DR AutoFeeding獨立模組啟動 ===")
    print("基地址範圍: 900-999")
    print("主要功能:")
    print("  ✓ 監控當前執行Flow地址(1201)")
    print("  ✓ 當1201=1時暫停自動進料程序")
    print("  ✓ 持續檢測保持DR_F可用")
    print("  ✓ 簡化交握邏輯")
    print("  ✓ Flow1直接讀取座標(940-944)")
    print("  ✓ 自動啟動檢測")
    print("  ✓ DR保護區域: X(-112~-4mm), Y(243~339.21mm)")
    print()
    print("*** 重要提醒 ***")
    print("請確保以下模組已啟動:")
    print("  1. 主Modbus TCP Server (端口502)")
    print("  2. CCD1視覺檢測模組 (CCD1VisionCodeYOLO.py)")
    print("  3. VP震動盤模組")
    print("  4. 然後啟動本AutoFeeding模組")
    print()
    
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