#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoFeeding_main.py - DR專案獨立入料檢測模組
基地址：900-999
功能：CCD1檢測協調、VP控制、Flow4觸發、DR_F保護區域判斷
專案：DR (檢測DR_F/STACK物件)
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
    PAUSED = 2
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
    """CCD1檢測結果 - DR專案版本"""
    dr_f_count: int = 0
    stack_count: int = 0
    total_detections: int = 0
    dr_f_world_coords: List[Tuple[float, float]] = None
    operation_success: bool = False
    
    def __post_init__(self):
        if self.dr_f_world_coords is None:
            self.dr_f_world_coords = []


class ProtectionZone:
    """DR專案保護區域判斷"""
    
    @staticmethod
    def is_point_in_quad(x_a: float, y_a: float) -> bool:
        """射線法判斷點是否在DR專案保護區域四邊形內"""
        # DR專案保護區域座標 (來自原始程式碼)
        points = [
            (-112, 243),     # 左上
            (-112, 339.21),  # 左下  
            (-4, 243),       # 右上
            (-4, 339)        # 右下
        ]
        
        # 找中心點並按極角排序
        cx = sum(p[0] for p in points) / 4
        cy = sum(p[1] for p in points) / 4
        
        def angle(p):
            return math.atan2(p[1] - cy, p[0] - cx)
        
        sorted_points = sorted(points, key=angle)
        
        # 射線法檢查
        def point_in_polygon(x, y, polygon):
            n = len(polygon)
            inside = False
            px, py = polygon[0]
            for i in range(1, n + 1):
                qx, qy = polygon[i % n]
                if ((py > y) != (qy > y)):
                    cross = (qx - px) * (y - py) / (qy - py + 1e-9) + px
                    if x < cross:
                        inside = not inside
                px, py = qx, qy
            return inside
        
        return point_in_polygon(x_a, y_a, sorted_points)


class AutoFeedingModule:
    """AutoFeeding獨立模組 - DR專案版本"""
    
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
        
        # 載入配置
        self.config = self.load_config()
        
        # 保護區域判斷
        self.protection_zone = ProtectionZone()
        
        # 系統狀態
        self.status = AutoFeedingStatus.STOPPED
        self.operation_status = OperationStatus.IDLE
        self.running = False
        self.paused = False
        self.vp_clearing_mode = False
        
        # 統計資訊
        self.cycle_count = 0
        self.dr_f_found_count = 0
        self.flow4_trigger_count = 0
        self.vp_vibration_count = 0
        self.flow4_consecutive_count = 0
        self.vp_empty_detection_count = 0
        self.error_code = 0
        
        print(f"[AutoFeeding] DR專案獨立模組初始化 - 基地址{self.BASE_ADDRESS}")
        print(f"[AutoFeeding] 檢測目標: DR_F/STACK物件")
    
    def load_config(self) -> Dict[str, Any]:
        """載入DR專案配置檔案"""
        default_config = {
            "autofeeding": {
                "cycle_interval": 1.5,     # DR專案週期間隔 (從原始代碼)
                "ccd1_timeout": 8.0,       # DR專案CCD1超時時間
                "flow4_consecutive_limit": 5,
                "vp_empty_check_count": 3,
                "material_shortage_threshold": 3,  # DR專案料件不足閾值
                "max_dr_f_check": 5        # DR專案最多檢查5個DR_F
            },
            "vp_params": {
                "spread_action_code": 11,
                "right_action_code": 4,    # DR專案向右震動
                "spread_strength": 50,     # DR專案震動強度 (來自原始代碼)
                "spread_frequency": 43,    # DR專案震動頻率 (來自原始代碼)
                "spread_duration": 0.4,    # DR專案震動持續時間 (來自原始代碼)
                "stop_command_code": 3,
                "stop_delay": 0.15         # DR專案停止延遲 (來自原始代碼)
            },
            "flow4_params": {
                "pulse_duration": 0.08,    # DR專案Flow4脈衝持續時間
                "pulse_interval": 0.05
            },
            "timing": {
                "command_delay": 0.08,         # DR專案指令延遲
                "status_check_interval": 0.08,  # DR專案狀態檢查間隔
                "register_clear_delay": 0.03,   # DR專案寄存器清空延遲
                "vp_stabilize_delay": 0.3       # DR專案VP穩定延遲
            }
        }
        
        try:
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dr_autofeeding_config.json')
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                print(f"[AutoFeeding] DR專案配置檔案已載入: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"[AutoFeeding] DR專案預設配置檔案已創建: {config_path}")
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
                print(f"[AutoFeeding] Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
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
            print("[AutoFeeding] Modbus連接已斷開")
    
    def init_registers(self):
        """初始化寄存器"""
        try:
            # 初始化狀態寄存器
            self.write_register(900, AutoFeedingStatus.STOPPED.value)  # 模組狀態
            self.write_register(907, 0)  # 錯誤代碼
            self.write_register(908, OperationStatus.IDLE.value)  # 操作狀態
            self.write_register(909, 0)  # VP清空模式
            
            # 清空控制寄存器
            self.write_register(920, 0)  # 運行控制
            self.write_register(921, 0)  # 暫停控制
            
            # 清空交握寄存器
            self.write_register(940, 0)  # 入料完成標誌
            self.write_register(945, 0)  # AutoProgram確認
            
            # 載入DR專案參數寄存器
            self.write_register(960, int(self.config['autofeeding']['cycle_interval'] * 1000))
            self.write_register(961, int(self.config['autofeeding']['ccd1_timeout'] * 1000))
            self.write_register(962, self.config['vp_params']['spread_strength'])
            self.write_register(963, self.config['vp_params']['spread_frequency'])
            self.write_register(964, int(self.config['vp_params']['spread_duration'] * 1000))
            self.write_register(965, self.config['autofeeding']['flow4_consecutive_limit'])
            self.write_register(966, self.config['autofeeding']['vp_empty_check_count'])
            self.write_register(967, int(self.config['flow4_params']['pulse_duration'] * 1000))
            self.write_register(968, int(self.config['flow4_params']['pulse_interval'] * 1000))
            self.write_register(969, self.config['autofeeding']['material_shortage_threshold'])  # DR專案料件不足閾值
            
            print("[AutoFeeding] DR專案寄存器初始化完成")
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
            self.write_register(902, self.dr_f_found_count)     # DR_F找到次數
            self.write_register(903, self.flow4_trigger_count)
            self.write_register(904, self.vp_vibration_count)
            self.write_register(905, self.flow4_consecutive_count)
            self.write_register(906, self.vp_empty_detection_count)
            self.write_register(907, self.error_code)
            self.write_register(908, self.operation_status.value)
            self.write_register(909, 1 if self.vp_clearing_mode else 0)
        except Exception as e:
            print(f"[ERROR] 狀態寄存器更新失敗: {e}")
    
    def check_control_registers(self):
        """檢查控制寄存器"""
        try:
            # 檢查運行控制 - 由AutoProgram觸發啟動
            run_control = self.read_register(920)
            if run_control == 1 and not self.running:
                print(f"[DEBUG] 檢測到920=1，啟動入料檢測")
                self.start_feeding()
            elif run_control == 0 and self.running:
                print(f"[DEBUG] 檢測到920=0，停止入料檢測")
                self.stop_feeding()
            
            # 檢查暫停控制 - AutoProgram執行Flow1時暫停AutoFeeding
            pause_control = self.read_register(921)
            if pause_control == 1 and not self.paused:
                print(f"[DEBUG] 檢測到921=1，暫停入料檢測")
                self.pause_feeding()
            elif pause_control == 0 and self.paused:
                print(f"[DEBUG] 檢測到921=0，恢復入料檢測")
                self.resume_feeding()
            
            # 檢查單次檢測觸發
            single_detect = self.read_register(922)
            if single_detect == 1:
                print(f"[DEBUG] 檢測到922=1，執行單次檢測")
                self.write_register(922, 0)  # 清除觸發
                if not self.running:
                    self.single_detection_cycle()
            
            # 檢查錯誤清除
            error_clear = self.read_register(923)
            if error_clear == 1:
                print(f"[DEBUG] 檢測到923=1，清除錯誤")
                self.write_register(923, 0)  # 清除觸發
                self.clear_errors()
            
            # 檢查VP強制停止
            vp_force_stop = self.read_register(924)
            if vp_force_stop == 1:
                print(f"[DEBUG] 檢測到924=1，VP強制停止")
                self.write_register(924, 0)  # 清除觸發
                self.emergency_stop_vp()
            
            # 檢查Flow4強制觸發
            flow4_force = self.read_register(925)
            if flow4_force == 1:
                print(f"[DEBUG] 檢測到925=1，Flow4強制觸發")
                self.write_register(925, 0)  # 清除觸發
                self.trigger_flow4_feeding_fast()
            
            # 檢查重置統計
            reset_stats = self.read_register(926)
            if reset_stats == 1:
                print(f"[DEBUG] 檢測到926=1，重置統計")
                self.write_register(926, 0)  # 清除觸發
                self.reset_statistics()
                
        except Exception as e:
            print(f"[ERROR] 控制寄存器檢查失敗: {e}")
    
    def check_modules_status(self) -> bool:
        """檢查CCD1、VP模組狀態 - 等待CCD1自行初始化"""
        # 檢查CCD1狀態 - 等待其自行初始化完成
        ccd1_status = self.read_register(201)
        if ccd1_status is None:
            print(f"[DEBUG] CCD1模組無回應")
            self.error_code = 101
            return False
        
        ccd1_ready = bool(ccd1_status & 0x01)        # bit0=Ready
        ccd1_running = bool(ccd1_status & 0x02)      # bit1=Running  
        ccd1_alarm = bool(ccd1_status & 0x04)        # bit2=Alarm
        ccd1_initialized = bool(ccd1_status & 0x08)  # bit3=Initialized
        
        # 只在第一次或狀態變化時打印詳細資訊
        if self.cycle_count % 20 == 1:  # 每20次打印一次狀態
            print(f"[DEBUG] CCD1狀態: {ccd1_status} (Ready={ccd1_ready}, Alarm={ccd1_alarm}, Initialized={ccd1_initialized})")
        
        # CCD1仍在初始化中，等待其完成
        if ccd1_alarm or not ccd1_initialized:
            if self.cycle_count % 50 == 1:  # 減少打印頻率
                print(f"[DEBUG] CCD1仍在初始化中，等待完成...")
            self.error_code = 102
            return False
        
        # CCD1必須Ready且Initialized，且無Alarm
        if not (ccd1_ready and ccd1_initialized):
            if self.cycle_count % 50 == 1:
                print(f"[DEBUG] CCD1未準備就緒，等待Ready狀態...")
            self.error_code = 102
            return False
        
        # 檢查VP狀態
        vp_status = self.read_register(300)
        vp_connected = self.read_register(301)
        
        if vp_status is None or vp_connected is None:
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
        """觸發CCD1檢測 - DR專案版本"""
        self.operation_status = OperationStatus.CCD_DETECTING
        result = CCD1DetectionResult()
        
        # 觸發拍照+檢測
        if not self.write_register(200, 16):
            self.error_code = 201
            return result
        
        # 等待檢測完成 - 高頻輪詢
        timeout = self.config['autofeeding']['ccd1_timeout']
        start_time = time.time()
        check_interval = self.config['timing']['status_check_interval']
        
        while (time.time() - start_time) < timeout:
            capture_complete = self.read_register(203)
            detect_complete = self.read_register(204)
            operation_success = self.read_register(205)
            
            if capture_complete == 1 and detect_complete == 1 and operation_success == 1:
                result.operation_success = True
                break
            
            time.sleep(check_interval)
        
        if not result.operation_success:
            self.error_code = 202
            return result
        
        # 讀取DR專案檢測結果
        result.dr_f_count = self.read_register(240) or 0       # DR_F_COUNT
        result.stack_count = self.read_register(242) or 0      # STACK_COUNT
        result.total_detections = self.read_register(243) or 0 # TOTAL_DETECTIONS
        
        # 提取DR_F世界座標
        if result.dr_f_count > 0:
            max_check = min(result.dr_f_count, self.config['autofeeding']['max_dr_f_check'])
            for i in range(max_check):
                base_addr = 261 + (i * 4)  # 每個物件佔4個寄存器
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
            if self.protection_zone.is_point_in_quad(world_x, world_y):
                return (world_x, world_y)
        
        return None
    
    def trigger_vp_vibration_and_redetect(self) -> Optional[Tuple[float, float]]:
        """觸發VP震動並重新檢測DR_F - DR專案版本(向右+擴散交互使用)"""
        self.operation_status = OperationStatus.VP_CONTROLLING
        
        # VP指令參數 - 來自DR專案原始代碼
        command_code = 5  # execute_action
        strength = self.config['vp_params']['spread_strength']      # 50
        frequency = self.config['vp_params']['spread_frequency']    # 43
        duration = self.config['vp_params']['spread_duration']      # 0.4秒
        
        # 第一步：向右震動
        print("[AutoFeeding] 第一步：執行向右震動")
        right_action_code = self.config['vp_params']['right_action_code']  # 4
        
        # 啟動向右震動
        success = True
        success &= self.write_register(320, command_code)
        success &= self.write_register(321, right_action_code)
        success &= self.write_register(322, strength)
        success &= self.write_register(323, frequency)
        success &= self.write_register(324, int(time.time()) % 65535)  # command_id
        
        if not success:
            print("[AutoFeeding] ✗ 向右震動指令發送失敗")
            self.error_code = 301
            return None
        
        # 等待向右震動
        time.sleep(duration)
        
        # 停止震動
        if not self.stop_vp_vibration():
            print("[AutoFeeding] ✗ 停止向右震動失敗")
            return None
        
        # 等待穩定
        time.sleep(0.3)
        
        # 重新檢測是否有DR_F在保護區
        detection_result = self.trigger_ccd1_detection()
        
        if detection_result.operation_success:
            target_coords = self.find_dr_f_in_protection_zone(detection_result)
            if target_coords:
                print(f"[AutoFeeding] ✓ 向右震動後找到DR_F: {target_coords}")
                return target_coords
        
        # 第二步：擴散震動
        print("[AutoFeeding] 第二步：執行擴散震動")
        spread_action_code = self.config['vp_params']['spread_action_code']  # 11
        
        # 啟動擴散震動
        success = True
        success &= self.write_register(320, command_code)
        success &= self.write_register(321, spread_action_code)
        success &= self.write_register(322, strength)
        success &= self.write_register(323, frequency)
        success &= self.write_register(324, int(time.time()) % 65535)  # command_id
        
        if not success:
            print("[AutoFeeding] ✗ 擴散震動指令發送失敗")
            self.error_code = 302
            return None
        
        # 等待擴散震動
        time.sleep(duration)
        
        # 停止震動
        if not self.stop_vp_vibration():
            print("[AutoFeeding] ✗ 停止擴散震動失敗")
            return None
        
        # 等待穩定
        time.sleep(0.3)
        
        # 最終檢測
        detection_result = self.trigger_ccd1_detection()
        
        if not detection_result.operation_success:
            print("[AutoFeeding] ✗ 擴散震動後檢測失敗")
            return None
        
        target_coords = self.find_dr_f_in_protection_zone(detection_result)
        
        if target_coords:
            print(f"[AutoFeeding] ✓ 擴散震動後找到DR_F: {target_coords}")
            return target_coords
        else:
            print("[AutoFeeding] 向右+擴散震動後仍未找到保護區域內的DR_F")
            return None
    
    def stop_vp_vibration(self) -> bool:
        """停止VP震動"""
        success = True
        success &= self.write_register(320, self.config['vp_params']['stop_command_code'])  # 3
        success &= self.write_register(321, 0)
        success &= self.write_register(322, 0)
        success &= self.write_register(323, 0)
        success &= self.write_register(324, 99)  # emergency stop id
        
        if success:
            time.sleep(self.config['vp_params']['stop_delay'])
        
        return success
    
    def trigger_flow4_feeding_fast(self) -> bool:
        """快速觸發Flow4送料 - DR專案版本"""
        self.operation_status = OperationStatus.FLOW4_TRIGGERING
        
        # 使用DR專案Flow4脈衝時間
        pulse_duration = self.config['flow4_params']['pulse_duration']
        print(f"[DEBUG] Flow4送料脈衝: 持續時間{pulse_duration}秒")
        
        # 立即觸發
        if not self.write_register(self.FLOW4_ADDRESS, 1):
            self.error_code = 401
            return False
        
        time.sleep(pulse_duration)
        
        if not self.write_register(self.FLOW4_ADDRESS, 0):
            self.error_code = 402
            return False
        
        return True
    
    def set_feeding_complete_fast(self, target_coords: Tuple[float, float], total_detections: int, dr_f_count: int) -> bool:
        """快速設置入料完成交握 - DR專案版本"""
        world_x, world_y = target_coords
        
        # 快速寫入座標和檢測結果
        self.write_32bit_register(941, 942, world_x)
        self.write_32bit_register(943, 944, world_y)
        self.write_register(946, total_detections)
        self.write_register(947, dr_f_count)
        self.write_register(940, 1)  # 設置完成標誌
        
        # 等待確認
        timeout = 2.0
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self.read_register(945) == 1:
                # 快速清空交握寄存器
                for addr in [940, 941, 942, 943, 944, 945, 946, 947]:
                    self.write_register(addr, 0)
                return True
            time.sleep(0.02)
        
        # 超時快速清空
        self.write_register(940, 0)
        return False
    
    def check_vp_empty(self) -> bool:
        """檢查VP是否為空"""
        detection_result = self.trigger_ccd1_detection()
        return detection_result.operation_success and detection_result.total_detections == 0
    
    def execute_vp_clearing_process(self):
        """執行VP清空流程 - DR專案版本"""
        print("[AutoFeeding] 開始VP清空流程")
        self.vp_clearing_mode = True
        self.vp_empty_detection_count = 0
        
        try:
            # VP震動嘗試清空料件
            for attempt in range(6):
                print(f"[AutoFeeding] VP清空嘗試 {attempt + 1}/6")
                
                # VP震動翻正料件
                success = True
                success &= self.write_register(320, 5)  # execute_action
                success &= self.write_register(321, self.config['vp_params']['spread_action_code'])
                success &= self.write_register(322, self.config['vp_params']['spread_strength'])
                success &= self.write_register(323, self.config['vp_params']['spread_frequency'])
                success &= self.write_register(324, int(time.time()) % 65535)
                
                if success:
                    time.sleep(self.config['vp_params']['spread_duration'])
                    self.stop_vp_vibration()
                
                time.sleep(0.5)
                
                # 檢測VP是否為空
                if self.check_vp_empty():
                    self.vp_empty_detection_count += 1
                    print(f"[AutoFeeding] VP空檢測 {self.vp_empty_detection_count}/3")
                    
                    if self.vp_empty_detection_count >= 3:
                        print("[AutoFeeding] VP連續3次檢測為空，清空完成")
                        self.status = AutoFeedingStatus.STOPPED
                        self.running = False
                        return
                else:
                    self.vp_empty_detection_count = 0
                
                time.sleep(1.0)
                
        except Exception as e:
            print(f"[ERROR] VP清空流程異常: {e}")
        finally:
            self.vp_clearing_mode = False
            print("[AutoFeeding] VP清空流程結束")
    
    def feeding_cycle(self) -> bool:
        """執行一次入料檢測週期 - DR專案版本"""
        try:
            self.cycle_count += 1
            self.status = AutoFeedingStatus.DETECTING
            
            # 檢查模組狀態
            if not self.check_modules_status():
                # 如果是CCD1還在初始化，不算錯誤，繼續等待
                if self.error_code == 102:
                    self.status = AutoFeedingStatus.RUNNING
                    return True
                return False
            
            # CCD1檢測
            detection_result = self.trigger_ccd1_detection()
            if not detection_result.operation_success:
                print(f"[AutoFeeding] 週期{self.cycle_count} CCD1檢測失敗")
                return False
            
            # 打印檢測結果
            print(f"[AutoFeeding] 週期{self.cycle_count} 檢測結果: DR_F={detection_result.dr_f_count}, STACK={detection_result.stack_count}, 總數={detection_result.total_detections}")
            
            # 尋找保護區域內的DR_F
            target_coords = self.find_dr_f_in_protection_zone(detection_result)
            
            if target_coords:
                # 找到保護區域內的DR_F
                self.dr_f_found_count += 1
                self.flow4_consecutive_count = 0  # 重置連續直振計數
                print(f"[AutoFeeding] ✅ 找到保護區內DR_F: {target_coords}")
                
                # 設置完成交握
                if self.set_feeding_complete_fast(target_coords, detection_result.total_detections, detection_result.dr_f_count):
                    print("[AutoFeeding] 入料完成，等待AutoProgram處理")
                    self.status = AutoFeedingStatus.PAUSED
                    return True
                
            elif detection_result.total_detections < self.config['autofeeding']['material_shortage_threshold']:
                # 料件不足，觸發Flow4送料
                print(f"[AutoFeeding] 料件不足 (總數={detection_result.total_detections}<{self.config['autofeeding']['material_shortage_threshold']})，觸發Flow4送料")
                
                if self.trigger_flow4_feeding_fast():
                    self.flow4_trigger_count += 1
                    self.flow4_consecutive_count += 1
                    print(f"[AutoFeeding] Flow4送料完成 (連續{self.flow4_consecutive_count}次)")
                    
                    # 檢查連續直振限制
                    if self.flow4_consecutive_count >= self.config['autofeeding']['flow4_consecutive_limit']:
                        print("[AutoFeeding] 達到連續直振限制，開始VP清空")
                        self.execute_vp_clearing_process()
                        return False
                else:
                    print(f"[AutoFeeding] Flow4送料失敗")
                
            else:
                # 料件充足但無DR_F在保護區，震動散開並重新檢測
                print(f"[AutoFeeding] 料件充足 (總數={detection_result.total_detections}>={self.config['autofeeding']['material_shortage_threshold']}) 但無DR_F在保護區，VP震動重檢")
                self.flow4_consecutive_count = 0  # 重置連續直振計數
                
                target_coords_after_vp = self.trigger_vp_vibration_and_redetect()
                if target_coords_after_vp:
                    # 震動後找到DR_F
                    self.dr_f_found_count += 1
                    print(f"[AutoFeeding] ✅ 震動後找到DR_F: {target_coords_after_vp}")
                    if self.set_feeding_complete_fast(target_coords_after_vp, detection_result.total_detections, detection_result.dr_f_count):
                        self.status = AutoFeedingStatus.PAUSED
                        return True
                    self.vp_vibration_count += 1
                else:
                    print("[AutoFeeding] 震動後仍未找到保護區域內的DR_F")
                    self.vp_vibration_count += 1
            
            self.operation_status = OperationStatus.IDLE
            self.status = AutoFeedingStatus.RUNNING
            return True
            
        except Exception as e:
            print(f"[ERROR] 入料週期異常: {e}")
            self.error_code = 999
            return False
    
    def single_detection_cycle(self):
        """執行單次檢測週期"""
        print("[AutoFeeding] 執行單次檢測")
        self.feeding_cycle()
    
    def start_feeding(self):
        """啟動入料檢測"""
        print("[AutoFeeding] 啟動DR專案入料檢測")
        print("[AutoFeeding] 等待CCD1自行初始化完成...")
        
        self.running = True
        self.status = AutoFeedingStatus.RUNNING
        self.error_code = 0
    
    def stop_feeding(self):
        """停止入料檢測"""
        self.running = False
        self.status = AutoFeedingStatus.STOPPED
        self.emergency_stop_vp()
        print("[AutoFeeding] DR專案入料檢測已停止")
    
    def pause_feeding(self):
        """暫停入料檢測"""
        self.paused = True
        self.status = AutoFeedingStatus.PAUSED
        print("[AutoFeeding] DR專案入料檢測已暫停")
    
    def resume_feeding(self):
        """恢復入料檢測"""
        self.paused = False
        self.status = AutoFeedingStatus.RUNNING if self.running else AutoFeedingStatus.STOPPED
        print("[AutoFeeding] DR專案入料檢測已恢復")
    
    def clear_errors(self):
        """清除錯誤"""
        self.error_code = 0
        if self.status == AutoFeedingStatus.ERROR:
            self.status = AutoFeedingStatus.STOPPED
        print("[AutoFeeding] 錯誤已清除")
    
    def emergency_stop_vp(self):
        """緊急停止VP"""
        try:
            self.stop_vp_vibration()
            print("[AutoFeeding] VP緊急停止")
        except:
            pass
    
    def reset_statistics(self):
        """重置統計資訊"""
        self.cycle_count = 0
        self.dr_f_found_count = 0
        self.flow4_trigger_count = 0
        self.vp_vibration_count = 0
        self.flow4_consecutive_count = 0
        self.vp_empty_detection_count = 0
        print("[AutoFeeding] DR專案統計資訊已重置")
    
    def main_loop(self):
        """主循環"""
        print("[AutoFeeding] DR專案主循環啟動")
        print("[AutoFeeding] 等待AutoProgram透過920=1觸發啟動")
        
        loop_count = 0
        
        while True:
            try:
                loop_count += 1
                
                # 每100次循環打印一次狀態
                if loop_count % 100 == 1:
                    run_control = self.read_register(920)
                    print(f"[DEBUG] DR專案主循環 {loop_count}: running={self.running}, paused={self.paused}, 920={run_control}")
                
                # 檢查連接狀態
                if not self.connected:
                    print("[DEBUG] Modbus連接斷開，嘗試重連")
                    if not self.connect():
                        time.sleep(5.0)
                        continue
                
                # 檢查控制寄存器
                self.check_control_registers()
                
                # 更新狀態寄存器
                self.update_status_registers()
                
                # 執行入料檢測 - 只有在運行且未暫停時
                if self.running and not self.paused and not self.vp_clearing_mode:
                    if not self.feeding_cycle():
                        # 檢測失敗，設置錯誤狀態
                        print(f"[DEBUG] 入料檢測失敗，錯誤碼: {self.error_code}")
                        self.status = AutoFeedingStatus.ERROR
                        time.sleep(0.2)
                    else:
                        # 檢測成功
                        if self.status == AutoFeedingStatus.PAUSED:
                            # 找到DR_F已暫停，等待AutoProgram處理
                            time.sleep(0.1)
                        else:
                            # 未找到DR_F，進入下一輪檢測
                            cycle_interval = self.config['autofeeding']['cycle_interval']
                            time.sleep(cycle_interval)  # DR專案1.5秒週期
                else:
                    # 非運行狀態或暫停狀態，短間隔檢查
                    time.sleep(0.1)
                    
            except KeyboardInterrupt:
                print("\n[AutoFeeding] 收到中斷信號，準備退出")
                break
            except Exception as e:
                print(f"[ERROR] 主循環異常: {e}")
                time.sleep(1.0)
        
        # 清理資源
        self.stop_feeding()
        self.disconnect()
        print("[AutoFeeding] DR專案程序已退出")


def main():
    """主程序入口"""
    print("=== AutoFeeding DR專案獨立模組啟動 ===")
    print("基地址範圍: 900-999")
    print("功能: CCD1檢測協調、VP控制、Flow4觸發、DR_F保護區域判斷")
    print("檢測目標: DR_F/STACK物件")
    print("保護區域: (-112,243) → (-112,339.21) → (-4,243) → (-4,339)")
    print("寄存器交握: 940-947 (與AutoProgram協調)")
    
    # 創建AutoFeeding模組
    autofeeding = AutoFeedingModule()
    
    # 連接Modbus
    if not autofeeding.connect():
        print("[ERROR] Modbus連接失敗，程序退出")
        return
    
    # 啟動主循環
    autofeeding.main_loop()


if __name__ == "__main__":
    main()