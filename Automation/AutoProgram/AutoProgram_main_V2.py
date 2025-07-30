#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoProgram_main_enhanced.py - DR專案機械臂協調控制模組 (SQLite批量座標處理版)
基地址：1300-1399
新功能：
1. CoordinateSupporter整合，批量讀取AutoFeeding SQLite篩選結果
2. AutoFeeding統一控制 (920寄存器)
3. CCD1記憶體監控與重啟控制
4. 逐一處理座標list，每個座標完成完整Flow1→Flow2循環
5. 批次完成後重啟AutoFeeding
"""

import time
import os
import sys
import json
import threading
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

# Modbus TCP Client (pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    print("pymodbus未安裝，請安裝: pip install pymodbus==3.9.2")
    MODBUS_AVAILABLE = False

# CoordinateSupporter integration
coordinate_supporter_path = os.path.join(os.path.dirname(__file__), '..', 'API')
if coordinate_supporter_path not in sys.path:
    sys.path.append(coordinate_supporter_path)
try:
    from CoordinateSupporter import CoordinateSupporter, CoordinatePoint
    COORDINATE_SUPPORTER_AVAILABLE = True
    print("✅ CoordinateSupporter模組導入成功")
except ImportError as e:
    print(f"❌ CoordinateSupporter模組導入失敗: {e}")
    COORDINATE_SUPPORTER_AVAILABLE = False


class SystemStatus(Enum):
    """系統狀態"""
    STOPPED = 0
    RUNNING = 1
    PROCESSING_COORDINATES = 2
    FLOW1_TRIGGERED = 3
    FLOW2_COMPLETED = 4
    CCD1_RELOADING = 5
    ERROR = 6


class ProtectionZone:
    """DR保護區域判斷 - 射線投射法"""
    
    def __init__(self, points=None):
        # 預設四點座標 (逆時針)
        if points is None:
            points = [
                (-127.83, 194.7),   # 左下
                (-113.75, 348.31),     # 右下  
                (6.21, 348.25),    # 右上
                (6.20, 194.76)   # 左上
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


@dataclass
class CoordinateBatch:
    """座標批次資料結構"""
    coordinates: List[CoordinatePoint]
    current_index: int = 0
    total_count: int = 0
    processed_count: int = 0
    failed_count: int = 0
    
    def __post_init__(self):
        self.total_count = len(self.coordinates)
    
    def get_current_coordinate(self) -> Optional[CoordinatePoint]:
        """獲取當前要處理的座標"""
        if self.current_index < len(self.coordinates):
            return self.coordinates[self.current_index]
        return None
    
    def move_to_next(self):
        """移動到下一個座標"""
        self.current_index += 1
    
    def mark_processed(self):
        """標記當前座標已處理"""
        self.processed_count += 1
        self.move_to_next()
    
    def mark_failed(self):
        """標記當前座標處理失敗"""
        self.failed_count += 1
        self.move_to_next()
    
    def is_complete(self) -> bool:
        """檢查批次是否完成"""
        return self.current_index >= self.total_count
    
    def get_progress(self) -> Dict[str, Any]:
        """獲取處理進度"""
        return {
            'current_index': self.current_index,
            'total_count': self.total_count,
            'processed_count': self.processed_count,
            'failed_count': self.failed_count,
            'remaining': self.total_count - self.current_index,
            'progress_percent': (self.current_index / self.total_count * 100) if self.total_count > 0 else 0
        }


class AutoProgramEnhancedController:
    """DR專案機械臂協調控制模組 (SQLite批量座標處理版)"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 基地址配置
        self.BASE_ADDRESS = 1300
        
        # AutoFeeding控制地址
        self.AF_CONTROL = 920                    # AutoFeeding控制 (1=啟動, 0=停止)
        self.AF_SCREENING_COMPLETE = 970         # AutoFeeding篩選完成旗標
        
        # Dobot M1Pro地址
        self.DOBOT_FLOW1_CONTROL = 1240          # Flow1控制
        self.DOBOT_FLOW1_COMPLETE = 1204         # Flow1完成狀態
        self.DOBOT_FLOW2_COMPLETE = 1205         # Flow2完成狀態
        self.DOBOT_CURRENT_MOTION_FLOW = 1201    # 當前運動Flow
        self.DOBOT_MOTION_PROGRESS = 1202        # 運動進度
        
        # CCD1監控地址
        self.CCD1_MEMORY_USAGE = 295             # CCD1記憶體使用量(MB)
        self.CCD1_SYSTEM_RELOAD = 296            # CCD1系統重載觸發
        self.CCD1_RELOAD_STATUS = 297            # CCD1重載狀態
        
        # AutoProgram座標存儲地址
        self.AP_COORDS_AVAILABLE = 1340          # AutoProgram座標可用標誌
        self.AP_TARGET_X_HIGH = 1341             # AutoProgram座標X高位
        self.AP_TARGET_X_LOW = 1342              # AutoProgram座標X低位
        self.AP_TARGET_Y_HIGH = 1343             # AutoProgram座標Y高位
        self.AP_TARGET_Y_LOW = 1344              # AutoProgram座標Y低位
        
        # CoordinateSupporter初始化
        self.coordinate_supporter: Optional[CoordinateSupporter] = None
        self.sqlite_db_path = r"C:\Users\user\Documents\GitHub\DobotDR\Automation\CCD1\ccd1_coordinate_supporter.db"
        
        # 保護區域判斷
        self.protection_zone = ProtectionZone()
        
        # 載入配置
        self.config = self.load_config()
        
        # 系統狀態
        self.system_status = SystemStatus.STOPPED
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # 核心狀態變數
        self.prepare_done = False
        self.auto_program_enabled = True
        self.autofeeding_stopped_by_memory = False  # CCD1記憶體原因停止AutoFeeding標誌
        
        # 座標批次處理
        self.current_batch: Optional[CoordinateBatch] = None
        self.processing_coordinates = False
        
        # CCD1記憶體監控
        self.ccd1_memory_limit = 800  # MB
        self.ccd1_reloading = False
        self.memory_monitor_enabled = True
        
        # 統計資訊
        self.coordination_cycle_count = 0
        self.flow1_trigger_count = 0
        self.flow2_complete_count = 0
        self.coordinates_processed_count = 0
        self.coordinates_failed_count = 0
        self.batch_completed_count = 0
        self.ccd1_reload_count = 0
        self.autofeeding_control_count = 0
        
        print("DR專案機械臂協調控制模組初始化完成 (SQLite批量座標處理版)")
        print(f"新功能:")
        print(f"  ✓ CoordinateSupporter整合")
        print(f"  ✓ AutoFeeding統一控制 (920寄存器)")
        print(f"  ✓ CCD1記憶體監控 ({self.ccd1_memory_limit}MB限制)")
        print(f"  ✓ 批量座標逐一處理")
        print(f"  ✓ 保護區域檢查")
        print(f"SQLite數據庫: {self.sqlite_db_path}")
    
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "autoprogram": {
                "coordination_interval": 0.2,
                "auto_program_enabled": True,
                "flow1_trigger_delay": 0.1,
                "coords_confirm_delay": 0.1,
                "batch_complete_delay": 1.0,
                "coordinates_timeout": 30.0,
            },
            "ccd1_monitoring": {
                "memory_check_interval": 5.0,
                "memory_limit_mb": 800,
                "reload_timeout": 30.0,
                "reload_check_interval": 1.0,
            },
            "autofeeding_control": {
                "stop_confirm_delay": 0.2,
                "start_confirm_delay": 0.5,
                "screening_flag_timeout": 10.0,
            },
            "protection_zone": {
                "x_min": -112.0,
                "x_max": -4.0,
                "y_min": 243.0,
                "y_max": 339.21
            },
            "timing": {
                "register_clear_delay": 0.05,
                "flow1_response_timeout": 10.0,
                "flow2_response_timeout": 30.0,
            }
        }
        
        try:
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dr_autoprogram_enhanced_config.json')
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                print(f"已載入增強版配置檔案: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"已創建增強版預設配置檔案: {config_path}")
        except Exception as e:
            print(f"配置檔案處理失敗: {e}")
            
        return default_config
    
    def connect(self) -> bool:
        """連接Modbus服務器"""
        try:
            if not MODBUS_AVAILABLE:
                print("Modbus功能不可用")
                return False
            
            if not COORDINATE_SUPPORTER_AVAILABLE:
                print("CoordinateSupporter模組不可用")
                return False
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.connected = self.modbus_client.connect()
            
            if self.connected:
                print(f"✓ Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
                
                # 初始化CoordinateSupporter
                try:
                    self.coordinate_supporter = CoordinateSupporter(db_path=self.sqlite_db_path)
                    print("✓ CoordinateSupporter初始化成功")
                except Exception as e:
                    print(f"✗ CoordinateSupporter初始化失敗: {e}")
                    return False
                
                self.init_system_registers()
                print("✓ 系統寄存器初始化完成")
            else:
                print(f"✗ Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
            
            return self.connected
        except Exception as e:
            print(f"Modbus連接異常: {e}")
            self.connected = False
            return False
    
    def init_system_registers(self):
        """初始化系統寄存器"""
        try:
            # AutoProgram狀態寄存器 (1300-1319)
            self.write_register(1300, SystemStatus.STOPPED.value)    # 系統狀態
            self.write_register(1301, 0)                             # prepare_done狀態
            self.write_register(1302, 1 if self.auto_program_enabled else 0)  # 自動程序啟用
            self.write_register(1303, 0)                             # AutoFeeding控制狀態
            self.write_register(1304, 0)                             # 當前批次座標數量
            self.write_register(1305, 0)                             # 當前處理索引
            self.write_register(1306, 0)                             # 已處理數量
            self.write_register(1307, 0)                             # 失敗數量
            self.write_register(1308, 0)                             # CCD1記憶體使用量
            self.write_register(1309, 0)                             # 錯誤代碼
            self.write_register(1310, 0)                             # CCD1重載狀態
            
            # AutoProgram控制寄存器 (1320-1339)
            self.write_register(1320, 0)                             # 系統控制
            self.write_register(1321, 1 if self.auto_program_enabled else 0)  # 自動程序控制
            self.write_register(1322, 0)                             # 錯誤清除
            self.write_register(1323, 0)                             # 強制重置
            self.write_register(1324, 1 if self.memory_monitor_enabled else 0)  # 記憶體監控啟用
            
            # AutoProgram座標寄存器 (1340-1344)
            self.write_register(self.AP_COORDS_AVAILABLE, 0)         # 座標可用標誌
            self.write_register(self.AP_TARGET_X_HIGH, 0)            # X座標高位
            self.write_register(self.AP_TARGET_X_LOW, 0)             # X座標低位
            self.write_register(self.AP_TARGET_Y_HIGH, 0)            # Y座標高位
            self.write_register(self.AP_TARGET_Y_LOW, 0)             # Y座標低位
            
            # 統計寄存器 (1350-1369)
            self.write_register(1350, 0)                             # 協調週期計數
            self.write_register(1351, 0)                             # Flow1觸發次數
            self.write_register(1352, 0)                             # Flow2完成次數
            self.write_register(1353, 0)                             # 座標處理次數
            self.write_register(1354, 0)                             # 座標失敗次數
            self.write_register(1355, 0)                             # 批次完成次數
            self.write_register(1356, 0)                             # CCD1重載次數
            self.write_register(1357, 0)                             # AutoFeeding控制次數
            
            print("✓ 系統寄存器初始化完成")
        except Exception as e:
            print(f"✗ 系統寄存器初始化失敗: {e}")
    
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
    
    def read_32bit_coordinate(self, high_addr: int, low_addr: int) -> float:
        """讀取32位座標"""
        high_val = self.read_register(high_addr) or 0
        low_val = self.read_register(low_addr) or 0
        
        # 合併32位值
        combined = (high_val << 16) + low_val
        
        # 處理補碼(負數)
        if combined >= 2147483648:  # 2^31
            combined = combined - 4294967296  # 2^32
        
        # 轉換為毫米(除以100)
        return combined / 100.0
    
    def write_32bit_coordinate(self, high_addr: int, low_addr: int, value: float) -> bool:
        """寫入32位座標"""
        try:
            # 轉換為整數形式(×100)
            value_int = int(value * 100)
            
            # 處理負數(補碼)
            if value_int < 0:
                value_int = value_int + 4294967296  # 2^32
            
            # 分解為高低位
            high_val = (value_int >> 16) & 0xFFFF
            low_val = value_int & 0xFFFF
            
            # 寫入寄存器
            high_success = self.write_register(high_addr, high_val)
            low_success = self.write_register(low_addr, low_val)
            
            return high_success and low_success
            
        except Exception as e:
            print(f"[AutoProgram] 32位座標寫入異常: {e}")
            return False
    
    # ==================== AutoFeeding控制功能 ====================
    
    def stop_autofeeding(self) -> bool:
        """停止AutoFeeding"""
        try:
            print("[AutoProgram] 停止AutoFeeding...")
            success = self.write_register(self.AF_CONTROL, 0)
            if success:
                self.autofeeding_control_count += 1
                print("✓ AutoFeeding已停止 (920=0)")
                time.sleep(self.config['autofeeding_control']['stop_confirm_delay'])
            else:
                print("✗ AutoFeeding停止失敗")
            return success
        except Exception as e:
            print(f"✗ AutoFeeding停止異常: {e}")
            return False
    
    def start_autofeeding(self) -> bool:
        """啟動AutoFeeding"""
        try:
            print("[AutoProgram] 啟動AutoFeeding...")
            success = self.write_register(self.AF_CONTROL, 1)
            if success:
                self.autofeeding_control_count += 1
                print("✓ AutoFeeding已啟動 (920=1)")
                time.sleep(self.config['autofeeding_control']['start_confirm_delay'])
            else:
                print("✗ AutoFeeding啟動失敗")
            return success
        except Exception as e:
            print(f"✗ AutoFeeding啟動異常: {e}")
            return False
    
    def clear_autofeeding_screening_flag(self) -> bool:
        """清除AutoFeeding篩選完成旗標"""
        try:
            success = self.write_register(self.AF_SCREENING_COMPLETE, 0)
            if success:
                print("✓ AutoFeeding篩選完成旗標已清除 (970=0)")
            else:
                print("✗ AutoFeeding篩選完成旗標清除失敗")
            return success
        except Exception as e:
            print(f"✗ AutoFeeding篩選完成旗標清除異常: {e}")
            return False
    
    # ==================== CoordinateSupporter整合功能 ====================
    
    def check_autofeeding_screening_complete(self) -> bool:
        """檢查AutoFeeding篩選是否完成"""
        screening_flag = self.read_register(self.AF_SCREENING_COMPLETE)
        return screening_flag == 1
    
    def read_sqlite_coordinates(self) -> Optional[List[CoordinatePoint]]:
        """從SQLite讀取AutoFeeding篩選結果"""
        try:
            if not self.coordinate_supporter:
                print("✗ CoordinateSupporter未初始化")
                return None
            
            print("[AutoProgram] 從SQLite讀取篩選結果...")
            
            # 讀取Find算法結果（label_id=0表示DR_F）
            coordinates = self.coordinate_supporter.get_find_algorithm_results(label_id=0)
            
            if not coordinates:
                print("[AutoProgram] SQLite中無篩選結果")
                return []
            
            print(f"[AutoProgram] 從SQLite讀取到 {len(coordinates)} 個篩選結果")
            
            # 驗證座標並篩選保護區域內的點
            valid_coordinates = []
            for i, coord in enumerate(coordinates):
                # 檢查世界座標是否有效
                if coord.world_x is None or coord.world_y is None:
                    print(f"[AutoProgram] 座標{i+1} 世界座標無效，跳過")
                    continue
                
                # 檢查是否在保護區域內
                if not self.protection_zone.is_point_in_rect(coord.world_x, coord.world_y):
                    print(f"[AutoProgram] 座標{i+1} ({coord.world_x:.2f}, {coord.world_y:.2f}) 不在保護區域內，跳過")
                    continue
                
                valid_coordinates.append(coord)
                print(f"[AutoProgram] 座標{i+1}: ({coord.world_x:.2f}, {coord.world_y:.2f}mm) ✓")
            
            print(f"[AutoProgram] 有效座標數量: {len(valid_coordinates)}/{len(coordinates)}")
            return valid_coordinates
            
        except Exception as e:
            print(f"✗ SQLite座標讀取異常: {e}")
            return None
    
    def create_coordinate_batch(self, coordinates: List[CoordinatePoint]) -> CoordinateBatch:
        """創建座標批次"""
        batch = CoordinateBatch(coordinates=coordinates)
        print(f"[AutoProgram] 創建座標批次，共 {batch.total_count} 個座標")
        return batch
    
    # ==================== CCD1記憶體監控功能 ====================
    
    def check_ccd1_memory_usage(self) -> int:
        """檢查CCD1記憶體使用量"""
        memory_usage = self.read_register(self.CCD1_MEMORY_USAGE) or 0
        return memory_usage
    
    def is_ccd1_memory_over_limit(self) -> bool:
        """檢查CCD1記憶體是否超限"""
        memory_usage = self.check_ccd1_memory_usage()
        return memory_usage > self.ccd1_memory_limit
    
    def reload_ccd1_system(self) -> bool:
        """重載CCD1系統"""
        try:
            print(f"[AutoProgram] 開始重載CCD1系統...")
            self.ccd1_reloading = True
            self.system_status = SystemStatus.CCD1_RELOADING
            
            # 1. 觸發重載指令
            if not self.write_register(self.CCD1_SYSTEM_RELOAD, 1):
                print("✗ 無法寫入CCD1重載指令")
                return False
            
            # 2. 等待重載狀態變為1（重載中）
            reload_start_timeout = 10.0
            start_time = time.time()
            
            while (time.time() - start_time) < reload_start_timeout:
                reload_status = self.read_register(self.CCD1_RELOAD_STATUS)
                if reload_status == 1:
                    print("✓ CCD1重載已開始")
                    break
                time.sleep(0.1)
            else:
                print("✗ CCD1重載啟動超時")
                return False
            
            # 3. 清除重載指令
            self.write_register(self.CCD1_SYSTEM_RELOAD, 0)
            
            # 4. 等待重載完成（重載狀態變回0）
            reload_complete_timeout = self.config['ccd1_monitoring']['reload_timeout']
            start_time = time.time()
            
            while (time.time() - start_time) < reload_complete_timeout:
                reload_status = self.read_register(self.CCD1_RELOAD_STATUS)
                if reload_status == 0:
                    print("✓ CCD1重載完成")
                    self.ccd1_reload_count += 1
                    self.ccd1_reloading = False
                    
                    # 等待系統穩定
                    time.sleep(2.0)
                    return True
                time.sleep(self.config['ccd1_monitoring']['reload_check_interval'])
            
            print("✗ CCD1重載完成超時")
            self.ccd1_reloading = False
            return False
            
        except Exception as e:
            print(f"✗ CCD1重載失敗: {e}")
            self.ccd1_reloading = False
            return False
    
    def handle_ccd1_memory_overlimit(self) -> bool:
        """處理CCD1記憶體超限"""
        try:
            memory_usage = self.check_ccd1_memory_usage()
            print(f"[AutoProgram] CCD1記憶體超限: {memory_usage}MB > {self.ccd1_memory_limit}MB")
            
            # 1. 停止AutoFeeding
            if not self.stop_autofeeding():
                print("✗ 停止AutoFeeding失敗")
                return False
            
            self.autofeeding_stopped_by_memory = True
            
            # 2. 重載CCD1系統
            if not self.reload_ccd1_system():
                print("✗ CCD1重載失敗")
                return False
            
            print("✓ CCD1記憶體超限處理完成")
            print("📌 注意: AutoFeeding將在當前批次完成後重新啟用")
            
            return True
            
        except Exception as e:
            print(f"✗ CCD1記憶體超限處理異常: {e}")
            return False
    
    # ==================== Flow控制功能 ====================
    
    def store_coordinate_to_autoprogram(self, coord: CoordinatePoint) -> bool:
        """將座標存儲到AutoProgram寄存器供Flow1讀取"""
        try:
            print(f"[AutoProgram] 存儲座標到寄存器: ({coord.world_x:.2f}, {coord.world_y:.2f})")
            
            # 寫入座標
            success = True
            success &= self.write_32bit_coordinate(self.AP_TARGET_X_HIGH, self.AP_TARGET_X_LOW, coord.world_x)
            success &= self.write_32bit_coordinate(self.AP_TARGET_Y_HIGH, self.AP_TARGET_Y_LOW, coord.world_y)
            
            if success:
                # 設置座標可用標誌
                success = self.write_register(self.AP_COORDS_AVAILABLE, 1)
                
                if success:
                    print(f"✓ 座標已存儲: ({coord.world_x:.2f}, {coord.world_y:.2f})")
                    return True
                else:
                    print("✗ 座標可用標誌設置失敗")
            else:
                print("✗ 座標寫入失敗")
            
            return False
            
        except Exception as e:
            print(f"✗ 座標存儲異常: {e}")
            return False
    
    def clear_autoprogram_coordinates(self):
        """清除AutoProgram座標狀態"""
        self.write_register(self.AP_COORDS_AVAILABLE, 0)
        self.write_register(self.AP_TARGET_X_HIGH, 0)
        self.write_register(self.AP_TARGET_X_LOW, 0)
        self.write_register(self.AP_TARGET_Y_HIGH, 0)
        self.write_register(self.AP_TARGET_Y_LOW, 0)
        print("[AutoProgram] ✓ AutoProgram座標狀態已清除")
    
    def trigger_flow1(self) -> bool:
        """觸發Flow1取料作業"""
        try:
            print("[AutoProgram] 觸發Flow1取料作業")
            
            # 檢查Flow1是否正在運行
            if self.check_flow1_running():
                print("✗ Flow1正在運行中，無法重複觸發")
                return False
            
            # 觸發Flow1控制
            if not self.write_register(self.DOBOT_FLOW1_CONTROL, 1):
                print("✗ Flow1觸發失敗")
                return False
            
            time.sleep(self.config['autoprogram']['flow1_trigger_delay'])
            
            # 清除Flow1控制狀態
            self.write_register(self.DOBOT_FLOW1_CONTROL, 0)
            
            self.flow1_trigger_count += 1
            self.system_status = SystemStatus.FLOW1_TRIGGERED
            
            print(f"✓ Flow1已觸發 (第{self.flow1_trigger_count}次)")
            return True
            
        except Exception as e:
            print(f"✗ Flow1觸發異常: {e}")
            return False
    
    def check_flow1_running(self) -> bool:
        """檢查Flow1是否正在執行"""
        current_motion_flow = self.read_register(self.DOBOT_CURRENT_MOTION_FLOW)
        return current_motion_flow == 1
    
    def check_flow1_complete(self) -> bool:
        """檢查Flow1是否完成"""
        flow1_complete = self.read_register(self.DOBOT_FLOW1_COMPLETE)
        return flow1_complete == 1
    
    def check_flow2_complete(self) -> bool:
        """檢查Flow2是否完成"""
        flow2_complete = self.read_register(self.DOBOT_FLOW2_COMPLETE)
        return flow2_complete == 1
    
    def clear_flow2_complete(self):
        """清除Flow2完成狀態"""
        self.write_register(self.DOBOT_FLOW2_COMPLETE, 0)
        self.flow2_complete_count += 1
        self.system_status = SystemStatus.FLOW2_COMPLETED
        print(f"[AutoProgram] Flow2完成狀態已清除 (第{self.flow2_complete_count}次)")
    
    # ==================== 批量座標處理邏輯 ====================
    
    def start_coordinate_batch_processing(self) -> bool:
        """開始批量座標處理"""
        try:
            # 1. 檢查AutoFeeding篩選是否完成
            if not self.check_autofeeding_screening_complete():
                return False
            
            # 2. 讀取SQLite座標
            coordinates = self.read_sqlite_coordinates()
            if not coordinates:
                print("[AutoProgram] 無有效座標，清除篩選旗標")
                self.clear_autofeeding_screening_flag()
                return False
            
            # 3. 創建座標批次
            self.current_batch = self.create_coordinate_batch(coordinates)
            self.processing_coordinates = True
            self.system_status = SystemStatus.PROCESSING_COORDINATES
            
            # 4. 清除AutoFeeding篩選完成旗標
            self.clear_autofeeding_screening_flag()
            
            print(f"[AutoProgram] 開始批量座標處理: {self.current_batch.total_count} 個座標")
            return True
            
        except Exception as e:
            print(f"✗ 批量座標處理啟動失敗: {e}")
            return False
    
    def process_current_coordinate(self) -> bool:
        """處理當前座標"""
        try:
            if not self.current_batch:
                return False
            
            current_coord = self.current_batch.get_current_coordinate()
            if not current_coord:
                return False
            
            progress = self.current_batch.get_progress()
            print(f"[AutoProgram] 處理座標 {progress['current_index']+1}/{progress['total_count']}: "
                  f"({current_coord.world_x:.2f}, {current_coord.world_y:.2f})")
            
            # 1. 存儲座標到AutoProgram寄存器
            if not self.store_coordinate_to_autoprogram(current_coord):
                print("✗ 座標存儲失敗，跳過此座標")
                self.current_batch.mark_failed()
                self.coordinates_failed_count += 1
                return False
            
            # 2. 停止AutoFeeding
            if not self.stop_autofeeding():
                print("✗ AutoFeeding停止失敗，跳過此座標")
                self.current_batch.mark_failed()
                self.coordinates_failed_count += 1
                return False
            
            # 3. 觸發Flow1
            if not self.trigger_flow1():
                print("✗ Flow1觸發失敗，跳過此座標")
                self.current_batch.mark_failed()
                self.coordinates_failed_count += 1
                return False
            
            print(f"✓ 座標 {progress['current_index']+1} 處理啟動成功，等待Flow1完成")
            return True
            
        except Exception as e:
            print(f"✗ 座標處理異常: {e}")
            if self.current_batch:
                self.current_batch.mark_failed()
                self.coordinates_failed_count += 1
            return False
    
    def complete_coordinate_processing(self):
        """完成當前座標處理"""
        if self.current_batch:
            self.current_batch.mark_processed()
            self.coordinates_processed_count += 1
            
            progress = self.current_batch.get_progress()
            print(f"✓ 座標處理完成 ({progress['processed_count']}/{progress['total_count']})")
    
    def complete_batch_processing(self):
        """完成批次處理"""
        try:
            if not self.current_batch:
                return
            
            progress = self.current_batch.get_progress()
            self.batch_completed_count += 1
            
            print(f"[AutoProgram] 批次處理完成:")
            print(f"  ✓ 總數量: {progress['total_count']}")
            print(f"  ✓ 成功處理: {progress['processed_count']}")
            print(f"  ✗ 失敗數量: {progress['failed_count']}")
            print(f"  📈 成功率: {progress['processed_count']/progress['total_count']*100:.1f}%")
            
            # 清除當前批次
            self.current_batch = None
            self.processing_coordinates = False
            
            # 如果AutoFeeding因為記憶體問題被停止，現在重新啟用
            if self.autofeeding_stopped_by_memory:
                print("[AutoProgram] 檢測到AutoFeeding因記憶體問題停止，現在重新啟用")
                if self.start_autofeeding():
                    self.autofeeding_stopped_by_memory = False
                    print("✓ AutoFeeding已重新啟用")
                else:
                    print("✗ AutoFeeding重新啟用失敗")
            else:
                # 正常情況下重新啟用AutoFeeding
                if self.start_autofeeding():
                    print("✓ AutoFeeding已重新啟用，等待下批座標")
                else:
                    print("✗ AutoFeeding重新啟用失敗")
            
            self.system_status = SystemStatus.RUNNING
            
        except Exception as e:
            print(f"✗ 批次處理完成異常: {e}")
    
    # ==================== 主協調邏輯 ====================
    
    def coordination_cycle(self):
        """機械臂協調控制週期 - SQLite批量座標處理版"""
        try:
            self.coordination_cycle_count += 1
            
            # DEBUG: 每50個週期輸出一次狀態
            if self.coordination_cycle_count % 50 == 0:
                current_status = f"週期{self.coordination_cycle_count}: status={self.system_status.name}, " \
                            f"prepare_done={self.prepare_done}, processing={self.processing_coordinates}"
                if self.current_batch:
                    progress = self.current_batch.get_progress()
                    current_status += f", batch_progress={progress['current_index']}/{progress['total_count']}"
                print(f"[AutoProgram] DEBUG - {current_status}")
            
            # 1. CCD1記憶體監控 (最高優先級，背景監控)
            if self.memory_monitor_enabled and not self.ccd1_reloading:
                if self.is_ccd1_memory_over_limit():
                    print(f"[AutoProgram] 檢測到CCD1記憶體超限")
                    self.handle_ccd1_memory_overlimit()
                    return  # 處理記憶體問題後返回
            
            # 2. 修改：Flow1進度監控與AutoFeeding重啟邏輯 (增加批次完成條件)
            flow1_running = self.check_flow1_running()
            if flow1_running:
                # Flow1正在運行，檢查進度
                flow1_progress = self.read_register(self.DOBOT_MOTION_PROGRESS) or 0
                
                # 當Flow1進度>44且AutoFeeding未運行時，檢查是否可以重新啟用AutoFeeding
                if flow1_progress > 44:
                    autofeeding_status = self.read_register(self.AF_CONTROL) or 0
                    if autofeeding_status == 0:  # AutoFeeding未運行
                        # 新增條件：檢查當前批次是否已完成
                        can_restart_autofeeding = False
                        
                        if self.current_batch is None:
                            # 沒有正在處理的批次，可以重啟AutoFeeding
                            can_restart_autofeeding = True
                            restart_reason = "無正在處理的批次"
                        elif self.current_batch.is_complete():
                            # 當前批次已完成，可以重啟AutoFeeding
                            can_restart_autofeeding = True
                            restart_reason = f"當前批次已完成 ({self.current_batch.processed_count}/{self.current_batch.total_count})"
                        else:
                            # 當前批次未完成，不能重啟AutoFeeding
                            remaining = self.current_batch.total_count - self.current_batch.current_index
                            restart_reason = f"當前批次未完成，剩餘{remaining}個座標"
                            if self.coordination_cycle_count % 25 == 0:  # 減少日誌頻率
                                print(f"[AutoProgram] Flow1進度={flow1_progress}% > 44%，但{restart_reason}，暫不重啟AutoFeeding")
                        
                        if can_restart_autofeeding:
                            print(f"[AutoProgram] Flow1進度={flow1_progress}% > 44%，{restart_reason}，重新啟用AutoFeeding")
                            if self.start_autofeeding():
                                print(f"[AutoProgram] ✓ AutoFeeding已在Flow1進度{flow1_progress}%時重新啟用")
                                # 重置記憶體停止標誌(如果有的話)
                                if self.autofeeding_stopped_by_memory:
                                    self.autofeeding_stopped_by_memory = False
                                    print(f"[AutoProgram] ✓ 記憶體停止標誌已重置")
                            else:
                                print(f"[AutoProgram] ✗ Flow1進度{flow1_progress}%時AutoFeeding重啟失敗")
                
                # Flow1正在運行，等待完成
                if self.coordination_cycle_count % 50 == 0:
                    print(f"[AutoProgram] Flow1正在運行中，進度={flow1_progress}%，等待完成...")
                return
            
            # 3. 檢查Flow2完成狀態 (第二優先級，始終檢查)
            if self.check_flow2_complete():
                print("[AutoProgram] 檢測到Flow2完成")
                self.clear_flow2_complete()
                self.prepare_done = False
                print("[AutoProgram] ✓ Flow2完成 → prepare_done=False")
                
                # 如果正在處理批次座標，完成當前座標
                if self.processing_coordinates and self.current_batch:
                    self.complete_coordinate_processing()
                    self.clear_autoprogram_coordinates()
                
                return  # Flow2完成處理後返回
            
            # 4. 檢查Flow1完成狀態
            if self.check_flow1_complete() and not self.prepare_done:
                print("[AutoProgram] 檢測到Flow1完成")
                self.prepare_done = True
                print("[AutoProgram] ✓ Flow1完成 → prepare_done=True，等待外部交握觸發Flow2")
                return  # Flow1完成處理後返回
            
            # 5. 主要協調邏輯（只在prepare_done=False時執行）
            if not self.prepare_done:
                # === prepare_done=False：執行座標處理邏輯 ===
                
                if self.processing_coordinates and self.current_batch:
                    # 正在處理批次座標
                    if self.current_batch.is_complete():
                        # 批次處理完成
                        print("[AutoProgram] 批次座標處理完成")
                        self.complete_batch_processing()
                        return
                    else:
                        # 處理下一個座標
                        if not self.check_flow1_running():  # 確保Flow1未運行
                            if not self.process_current_coordinate():
                                print("[AutoProgram] 當前座標處理失敗，繼續下一個")
                            return
                else:
                    # 未在處理批次，檢查是否有新的SQLite結果
                    if self.start_coordinate_batch_processing():

                        print("[AutoProgram] 新批次座標處理已開始")
                        return
                    else:
                        # 無新座標，等待
                        if self.coordination_cycle_count % 100 == 0:
                            self.start_autofeeding()
                            print("[AutoProgram] 等待AutoFeeding新的篩選結果...")
            else:
                # === prepare_done=True：等待Flow2完成 ===
                if self.coordination_cycle_count % 100 == 0:
                    print("[AutoProgram] prepare_done=True，等待外部交握觸發Flow2...")
            
        except Exception as e:
            print(f"[AutoProgram] 協調週期異常: {e}")
            import traceback
            traceback.print_exc()
    
    def update_system_registers(self):
        """更新系統寄存器"""
        try:
            if not self.connected:
                return
            
            # 更新系統狀態
            self.write_register(1300, self.system_status.value)
            self.write_register(1301, 1 if self.prepare_done else 0)
            self.write_register(1302, 1 if self.auto_program_enabled else 0)
            self.write_register(1303, self.read_register(self.AF_CONTROL) or 0)
            
            # 更新批次處理狀態
            if self.current_batch:
                progress = self.current_batch.get_progress()
                self.write_register(1304, progress['total_count'])
                self.write_register(1305, progress['current_index'])
                self.write_register(1306, progress['processed_count'])
                self.write_register(1307, progress['failed_count'])
            else:
                self.write_register(1304, 0)
                self.write_register(1305, 0)
                self.write_register(1306, 0)
                self.write_register(1307, 0)
            
            # 更新CCD1狀態
            ccd1_memory = self.check_ccd1_memory_usage()
            self.write_register(1308, ccd1_memory)
            self.write_register(1310, 1 if self.ccd1_reloading else 0)
            
            # 更新統計資訊
            self.write_register(1350, self.coordination_cycle_count)
            self.write_register(1351, self.flow1_trigger_count)
            self.write_register(1352, self.flow2_complete_count)
            self.write_register(1353, self.coordinates_processed_count)
            self.write_register(1354, self.coordinates_failed_count)
            self.write_register(1355, self.batch_completed_count)
            self.write_register(1356, self.ccd1_reload_count)
            self.write_register(1357, self.autofeeding_control_count)
            
        except Exception as e:
            print(f"系統寄存器更新失敗: {e}")
    
    def check_control_registers(self):
        """檢查控制寄存器變更"""
        try:
            # 檢查系統控制寄存器 (1320)
            system_control = self.read_register(1320)
            if system_control == 1 and not self.running:
                print("[AutoProgram] 檢測到系統啟動指令 (1320=1)")
                self.start()
            elif system_control == 0 and self.running:
                print("[AutoProgram] 檢測到系統停止指令 (1320=0)")
                self.stop()
            
            # 檢查自動程序控制寄存器 (1321)
            auto_control = self.read_register(1321)
            if auto_control is not None:
                if auto_control != (1 if self.auto_program_enabled else 0):
                    self.auto_program_enabled = (auto_control == 1)
                    print(f"[AutoProgram] 自動程序啟用狀態更新: {self.auto_program_enabled} (1321={auto_control})")
            
            # 檢查記憶體監控控制寄存器 (1324)
            memory_monitor_control = self.read_register(1324)
            if memory_monitor_control is not None:
                if memory_monitor_control != (1 if self.memory_monitor_enabled else 0):
                    self.memory_monitor_enabled = (memory_monitor_control == 1)
                    print(f"[AutoProgram] 記憶體監控啟用狀態更新: {self.memory_monitor_enabled} (1324={memory_monitor_control})")
            
        except Exception as e:
            print(f"[AutoProgram] 控制寄存器檢查異常: {e}")
    
    def start(self):
        """啟動機械臂協調控制系統"""
        if self.running:
            return
        
        print("[AutoProgram] === 啟動DR專案機械臂協調控制系統 (SQLite批量座標處理版) ===")
        self.running = True
        self.system_status = SystemStatus.RUNNING
        
        # 重置狀態
        self.prepare_done = False
        self.processing_coordinates = False
        self.current_batch = None
        self.autofeeding_stopped_by_memory = False
        self.ccd1_reloading = False
        
        # 重置統計
        self.coordination_cycle_count = 0
        self.flow1_trigger_count = 0
        self.flow2_complete_count = 0
        self.coordinates_processed_count = 0
        self.coordinates_failed_count = 0
        self.batch_completed_count = 0
        self.ccd1_reload_count = 0
        self.autofeeding_control_count = 0
        
        # 更新狀態寄存器
        self.write_register(1300, SystemStatus.RUNNING.value)
        self.write_register(1301, 0)  # prepare_done=False
        
        self.thread = threading.Thread(target=self._coordination_loop, daemon=True)
        self.thread.start()
        
        print("[AutoProgram] DR專案協調控制系統已啟動")
        print("[AutoProgram] 新功能特性:")
        print(f"[AutoProgram]   ✓ SQLite批量座標處理")
        print(f"[AutoProgram]   ✓ AutoFeeding統一控制 (920寄存器)")
        print(f"[AutoProgram]   ✓ CCD1記憶體監控 ({self.ccd1_memory_limit}MB限制)")
        print(f"[AutoProgram]   ✓ 保護區域檢查")
        print(f"[AutoProgram]   ✓ 逐一座標處理 (Flow1→Flow2循環)")
    
    def stop(self):
        """停止機械臂協調控制系統"""
        if not self.running:
            return
        
        print("[AutoProgram] === 停止DR專案機械臂協調控制系統 ===")
        self.running = False
        self.system_status = SystemStatus.STOPPED
        
        # 清除批次處理狀態
        self.processing_coordinates = False
        self.current_batch = None
        
        # 清除座標狀態
        self.clear_autoprogram_coordinates()
        
        # 重新啟用AutoFeeding（如果被停止）
        if self.autofeeding_stopped_by_memory:
            self.start_autofeeding()
            self.autofeeding_stopped_by_memory = False
        
        # 更新狀態寄存器
        self.write_register(1300, SystemStatus.STOPPED.value)
        self.update_system_registers()
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        
        print("[AutoProgram] DR專案協調控制系統已停止")
        self.print_statistics()
    
    def _coordination_loop(self):
        """協調控制主循環"""
        interval = self.config['autoprogram']['coordination_interval']
        
        print("[AutoProgram] DR專案協調控制主循環已啟動 (SQLite批量座標處理版)")
        
        loop_count = 0
        while True:  # 移除 self.running 檢查，讓循環持續運行
            try:
                loop_count += 1
                
                # DEBUG: 每100次循環輸出一次心跳
                if loop_count % 100 == 0:
                    print(f"[AutoProgram] 控制循環心跳 - 第{loop_count}次, running={self.running}, auto_enabled={self.auto_program_enabled}")
                
                # 總是檢查控制寄存器變更
                self.check_control_registers()
                
                # 只有在系統運行且自動程序啟用時才執行協調邏輯
                if self.running and self.auto_program_enabled:
                    self.coordination_cycle()
                
                # 總是更新系統寄存器
                self.update_system_registers()
                
                time.sleep(interval)
                
            except Exception as e:
                print(f"[AutoProgram] 協調循環異常: {e}")
                time.sleep(1.0)
    
    def disconnect(self):
        """斷開Modbus連接"""
        if self.modbus_client and self.connected:
            self.modbus_client.close()
            self.connected = False
            print("Modbus連接已斷開")
    
    def print_statistics(self):
        """輸出統計資訊"""
        print(f"\n=== DR專案AutoProgram統計資訊 (SQLite批量座標處理版) ===")
        print(f"協調週期數: {self.coordination_cycle_count}")
        print(f"Flow1觸發次數: {self.flow1_trigger_count}")
        print(f"Flow2完成次數: {self.flow2_complete_count}")
        print(f"座標處理成功: {self.coordinates_processed_count}")
        print(f"座標處理失敗: {self.coordinates_failed_count}")
        print(f"批次完成次數: {self.batch_completed_count}")
        print(f"CCD1重載次數: {self.ccd1_reload_count}")
        print(f"AutoFeeding控制次數: {self.autofeeding_control_count}")
        print(f"座標處理成功率: {self.coordinates_processed_count/(self.coordinates_processed_count+self.coordinates_failed_count)*100:.1f}%" if (self.coordinates_processed_count+self.coordinates_failed_count) > 0 else "N/A")
        print(f"功能特性: SQLite整合, 批量處理, 記憶體監控, 統一控制")
    
    def get_status_info(self) -> Dict[str, Any]:
        """獲取狀態資訊"""
        batch_info = None
        if self.current_batch:
            batch_info = self.current_batch.get_progress()
        
        return {
            "connected": self.connected,
            "system_status": self.system_status.name,
            "running": self.running,
            "auto_program_enabled": self.auto_program_enabled,
            "prepare_done": self.prepare_done,
            "processing_coordinates": self.processing_coordinates,
            "current_batch": batch_info,
            "ccd1_memory_usage": self.check_ccd1_memory_usage(),
            "ccd1_memory_limit": self.ccd1_memory_limit,
            "ccd1_reloading": self.ccd1_reloading,
            "autofeeding_stopped_by_memory": self.autofeeding_stopped_by_memory,
            "memory_monitor_enabled": self.memory_monitor_enabled,
            "flow1_complete": self.check_flow1_complete(),
            "flow2_complete": self.check_flow2_complete(),
            "flow1_running": self.check_flow1_running(),
            "sqlite_db_path": self.sqlite_db_path,
            "statistics": {
                "coordination_cycle_count": self.coordination_cycle_count,
                "flow1_trigger_count": self.flow1_trigger_count,
                "flow2_complete_count": self.flow2_complete_count,
                "coordinates_processed_count": self.coordinates_processed_count,
                "coordinates_failed_count": self.coordinates_failed_count,
                "batch_completed_count": self.batch_completed_count,
                "ccd1_reload_count": self.ccd1_reload_count,
                "autofeeding_control_count": self.autofeeding_control_count
            }
        }


def main():
    """主程序"""
    print("=" * 80)
    print("DR專案機械臂協調控制模組啟動 (SQLite批量座標處理版)")
    print("主要新功能:")
    print("  ✓ CoordinateSupporter整合，批量讀取AutoFeeding SQLite篩選結果")
    print("  ✓ AutoFeeding統一控制 (920寄存器)")
    print("  ✓ CCD1記憶體監控與自動重啟 (800MB限制)")
    print("  ✓ 逐一處理座標list，每個座標完成完整Flow1→Flow2循環")
    print("  ✓ 批次完成後自動重啟AutoFeeding")
    print("  ✓ 保護區域檢查與座標驗證")
    print("=" * 80)
    
    controller = AutoProgramEnhancedController()
    
    if not controller.connect():
        print("✗ Modbus連接失敗，程序退出")
        return
    
  
    print("[AutoProgram] 🚀 啟動Modbus控制循環...")
    
    # 啟動協調控制循環
    controller.thread = threading.Thread(target=controller._coordination_loop, daemon=True)
    controller.thread.start()
    
    # 啟動寄存器更新線程
    def update_registers():
        while True:
            try:
                if controller.connected:
                    controller.update_system_registers()
                time.sleep(1.0)
            except Exception as e:
                print(f"寄存器更新異常: {e}")
                time.sleep(2.0)
    
    update_thread = threading.Thread(target=update_registers, daemon=True)
    update_thread.start()
    
    print("\n✅ 系統已啟動，等待Modbus寄存器控制指令...")
    print("📋 控制寄存器說明:")
    print("  • 系統控制: 1320寄存器 (0=停止, 1=啟動)")
    print("  • 自動程序控制: 1321寄存器 (0=停用, 1=啟用)")
    print("  • 記憶體監控控制: 1324寄存器 (0=停用, 1=啟用)")
    print("  • AutoFeeding控制: 920寄存器 (0=停止, 1=啟動)")
    print("\n📊 狀態監控寄存器:")
    print("  • 系統狀態: 1300 (0=停止, 1=運行, 2=處理座標中, 3=Flow1觸發, 4=Flow2完成, 5=CCD1重載中, 6=錯誤)")
    print("  • prepare_done狀態: 1301 (0=False, 1=True)")
    print("  • 當前批次總數: 1304")
    print("  • 當前處理索引: 1305")
    print("  • 已處理數量: 1306")
    print("  • 失敗數量: 1307")
    print("  • CCD1記憶體使用量: 1308 (MB)")
    print("  • 協調週期計數: 1350")
    print("  • Flow1觸發次數: 1351")
    print("  • Flow2完成次數: 1352")
    print("  • 座標處理成功次數: 1353")
    print("  • 座標處理失敗次數: 1354")
    print("  • 批次完成次數: 1355")
    print("  • CCD1重載次數: 1356")
    print("  • AutoFeeding控制次數: 1357")
    print("\n🔄 程序持續運行中，透過Modbus寄存器進行控制...")
    print("   按 Ctrl+C 停止程序")
    
    # 主循環：僅維護Modbus地址循環
    while True:
        try:
            # 每10秒輸出一次心跳信息
            time.sleep(10.0)
            
            # 簡單的心跳狀態輸出
            status = controller.get_status_info()
            print(f"[心跳] 系統狀態={status['system_status']}, "
                    f"運行={status['running']}, "
                    f"自動程序={'啟用' if status['auto_program_enabled'] else '停用'}, "
                    f"處理座標中={status['processing_coordinates']}, "
                    f"週期={status['statistics']['coordination_cycle_count']}")
            
            if status['current_batch']:
                batch_info = status['current_batch']
                print(f"[批次] 進度={batch_info['current_index']}/{batch_info['total_count']}, "
                        f"成功={batch_info['processed_count']}, "
                        f"失敗={batch_info['failed_count']}")
            
        except KeyboardInterrupt:
            print("\n[AutoProgram] 收到中斷信號，準備停止...")
            break
        except Exception as e:
            print(f"[AutoProgram] 主循環異常: {e}")
            time.sleep(5.0)
    


if __name__ == "__main__":
    main()