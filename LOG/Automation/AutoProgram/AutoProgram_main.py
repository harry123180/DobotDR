#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoProgram_main.py - DR專案機械臂協調控制模組 (修正prepare_done邏輯版)
基地址：1300-1399
修正策略：嚴格控制prepare_done狀態機，防止Flow1重複觸發
Flow2 done->AutoFeeding done ->AutoProgram get coordinates-> Flow1->Flow1 get AutoProgram coordinates->work flow
增加保護區域檢查和重複座標檢測機制，防止重複檢查AutoFeeding
"""

import time
import os
import json
import threading
import logging
from logging.handlers import RotatingFileHandler
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

# Modbus TCP Client (pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    MODBUS_AVAILABLE = False


def setup_logging(module_name: str) -> logging.Logger:
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


class SystemStatus(Enum):
    """系統狀態"""
    STOPPED = 0
    RUNNING = 1
    WAITING_COORDINATES = 2
    FLOW1_TRIGGERED = 3
    FLOW2_COMPLETED = 4
    ERROR = 5


class ProtectionZone:
    """保護區域判斷 - 與AutoFeeding相同的定義"""
    
    @staticmethod
    def is_point_in_rect(x: float, y: float) -> bool:
        """判斷點是否在DR保護區域矩形內"""
        # DR保護區域四點座標
        x_min = -112.0
        x_max = -4.0
        y_min = 243.0
        y_max = 339.21
        
        return x_min <= x <= x_max and y_min <= y <= y_max


class AutoProgramController:
    """DR專案機械臂協調控制模組 (提前座標讀取版)"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        # 初始化logger
        self.logger = setup_logging('AutoProgram')
        
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 基地址配置
        self.BASE_ADDRESS = 1300
        
        # AutoFeeding模組地址 (DR專案使用940-945)
        self.AF_DR_F_AVAILABLE = 940         # AutoFeeding DR_F可用標誌
        self.AF_TARGET_X_HIGH = 941          # 目標座標X高位
        self.AF_TARGET_X_LOW = 942           # 目標座標X低位
        self.AF_TARGET_Y_HIGH = 943          # 目標座標Y高位
        self.AF_TARGET_Y_LOW = 944           # 目標座標Y低位
        self.AF_COORDS_TAKEN = 945           # 座標已讀取標誌
        
        # Dobot M1Pro地址
        self.DOBOT_FLOW1_CONTROL = 1240      # Flow1控制
        self.DOBOT_FLOW1_COMPLETE = 1204     # Flow1完成狀態
        self.DOBOT_FLOW2_COMPLETE = 1205     # Flow2完成狀態
        self.DOBOT_CURRENT_MOTION_FLOW = 1201 # 當前運動Flow (0=無, 1=Flow1, 2=Flow2, 5=Flow5)
        
        # AutoProgram座標存儲地址 (新增 1350-1354)
        self.AP_COORDS_AVAILABLE = 1350      # AutoProgram座標可用標誌
        self.AP_TARGET_X_HIGH = 1351         # AutoProgram座標X高位
        self.AP_TARGET_X_LOW = 1352          # AutoProgram座標X低位
        self.AP_TARGET_Y_HIGH = 1353         # AutoProgram座標Y高位
        self.AP_TARGET_Y_LOW = 1354          # AutoProgram座標Y低位
        
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
        
        # 座標狀態管理 (新增)
        self.current_coords: Optional[Tuple[float, float]] = None
        self.last_coords: Optional[Tuple[float, float]] = None
        self.coords_ready = False
        self.last_autofeeding_check_time = 0.0  # 新增：防止重複檢查AutoFeeding
        
        # 統計資訊
        self.coordination_cycle_count = 0
        self.flow1_trigger_count = 0
        self.flow2_complete_count = 0
        self.dr_f_taken_count = 0
        self.coords_duplicate_count = 0
        
        self.logger.info("DR專案機械臂協調控制模組初始化完成 (提前座標讀取版)")
        self.logger.info(f"Modbus服務器: {modbus_host}:{modbus_port}")
        self.logger.info(f"AutoProgram基地址: {self.BASE_ADDRESS}")
        self.logger.info("新策略: AutoProgram提前讀取座標，Flow1直接從AutoProgram拿取")
        self.logger.info(f"座標存儲地址: {self.AP_COORDS_AVAILABLE}-{self.AP_TARGET_Y_LOW}")
        self.logger.info("保護區域檢查: X(-112~-4), Y(243~339.21)")
        self.logger.info("重複座標檢測: 啟用")
    
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "autoprogram": {
                "coordination_interval": 0.2,
                "auto_program_enabled": True,
                "flow1_trigger_delay": 0.1,
                "coords_confirm_delay": 0.1,
                "duplicate_check_delay": 3.0,     # 重複座標檢查延遲
                "coords_tolerance": 0.1,          # 座標容忍度(mm)
            },
            "monitoring": {
                "dr_f_check_interval": 0.1,
                "flow2_check_interval": 0.2,
                "status_update_interval": 1.0,
            },
            "timing": {
                "register_clear_delay": 0.05,
                "flow1_response_timeout": 10.0,
            },
            "protection_zone": {
                "x_min": -112.0,
                "x_max": -4.0,
                "y_min": 243.0,
                "y_max": 339.21
            },
            "project_info": {
                "project_name": "DR",
                "detection_types": ["DR_F", "STACK"],
                "flow_config": "Flow1+Flow2",
                "coordinate_strategy": "AutoProgram_PreRead"
            }
        }
        
        try:
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dr_autoprogram_config.json')
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                self.logger.info(f"已載入DR專案配置檔案: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                self.logger.info(f"已創建DR專案預設配置檔案: {config_path}")
        except Exception as e:
            self.logger.error(f"配置檔案處理失敗: {e}", exc_info=True)
            
        return default_config
    
    def connect(self) -> bool:
        """連接Modbus服務器"""
        try:
            if not MODBUS_AVAILABLE:
                self.logger.critical("pymodbus未安裝，請安裝: pip install pymodbus==3.9.2")
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
                self.logger.info(f"Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
                self.init_system_registers()
            else:
                self.logger.warning("連接失敗，將在主循環中重試")
                self.logger.warning(f"Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
            
            return self.connected
        except Exception as e:
            self.logger.error(f"Modbus連接異常: {e}", exc_info=True)
            self.connected = False
            return False
    
    def init_system_registers(self):
        """初始化系統寄存器"""
        try:
            self.logger.debug("開始初始化系統寄存器")
            
            # AutoProgram狀態寄存器 (1300-1319)
            self.write_register(1300, SystemStatus.STOPPED.value)
            self.write_register(1301, 0)  # prepare_done狀態
            self.write_register(1302, 1 if self.auto_program_enabled else 0)
            self.write_register(1303, 0)  # AutoFeeding DR_F狀態
            self.write_register(1304, 0)  # Flow2完成狀態
            self.write_register(1305, 0)  # 協調週期計數
            self.write_register(1306, 0)  # Flow1觸發次數
            self.write_register(1307, 0)  # Flow2完成次數
            self.write_register(1308, 0)  # DR_F取得次數
            self.write_register(1309, 0)  # 錯誤代碼
            self.write_register(1310, 0)  # 重複座標計數
            
            # AutoProgram控制寄存器 (1320-1339)
            self.write_register(1320, 0)  # 系統控制
            self.write_register(1321, 1 if self.auto_program_enabled else 0)
            self.write_register(1322, 0)  # 錯誤清除
            self.write_register(1323, 0)  # 強制重置
            
            # AutoProgram座標寄存器 (1350-1354) - 新增
            self.write_register(1350, 0)  # 座標可用標誌
            self.write_register(1351, 0)  # X高位
            self.write_register(1352, 0)  # X低位
            self.write_register(1353, 0)  # Y高位
            self.write_register(1354, 0)  # Y低位
            
            self.logger.info("DR專案AutoProgram系統寄存器初始化完成")
            self.logger.info(f"座標存儲寄存器: {self.AP_COORDS_AVAILABLE}-{self.AP_TARGET_Y_LOW}")
            
        except Exception as e:
            self.logger.error(f"系統寄存器初始化失敗: {e}", exc_info=True)
    
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
    
    def write_32bit_coordinate(self, high_addr: int, low_addr: int, value: float) -> bool:
        """寫入32位世界座標"""
        try:
            self.logger.debug(f"寫入32位座標: 值={value:.2f}, 高地址={high_addr}, 低地址={low_addr}")
            
            # 轉換為整數形式(×100)
            value_int = int(value * 100)
            self.logger.debug(f"整數轉換: {value:.2f} -> {value_int}")
            
            # 處理負數(補碼)
            if value_int < 0:
                value_int = value_int + 4294967296  # 2^32
                self.logger.debug(f"負數補碼處理: -> {value_int}")
            
            # 分解為高低位
            high_val = (value_int >> 16) & 0xFFFF
            low_val = value_int & 0xFFFF
            self.logger.debug(f"高低位分解: 高={high_val}, 低={low_val}")
            
            # 寫入寄存器
            high_success = self.write_register(high_addr, high_val)
            low_success = self.write_register(low_addr, low_val)
            
            self.logger.debug(f"寄存器寫入結果: 高地址{high_addr}={high_success}, 低地址{low_addr}={low_success}")
            
            return high_success and low_success
            
        except Exception as e:
            self.logger.error(f"32位座標寫入異常: {e}", exc_info=True)
            return False
    
    def get_autofeeding_status(self) -> Dict[str, Any]:
        """獲取AutoFeeding模組狀態"""
        dr_f_available = self.read_register(self.AF_DR_F_AVAILABLE) or 0
        coords_taken = self.read_register(self.AF_COORDS_TAKEN) or 0
        
        target_x = 0.0
        target_y = 0.0
        
        if dr_f_available == 1:
            target_x = self.read_32bit_coordinate(self.AF_TARGET_X_HIGH, self.AF_TARGET_X_LOW)
            target_y = self.read_32bit_coordinate(self.AF_TARGET_Y_HIGH, self.AF_TARGET_Y_LOW)
        
        return {
            'dr_f_available': bool(dr_f_available),
            'coords_taken': bool(coords_taken),
            'target_x': target_x,
            'target_y': target_y
        }
    
    def coords_equal(self, coords1: Tuple[float, float], coords2: Tuple[float, float]) -> bool:
        """比較兩個座標是否相等(在容忍度內)"""
        tolerance = self.config['autoprogram']['coords_tolerance']
        return (abs(coords1[0] - coords2[0]) < tolerance and 
                abs(coords1[1] - coords2[1]) < tolerance)
    
    def read_and_validate_autofeeding_coordinates(self) -> Optional[Tuple[float, float]]:
        """讀取並驗證AutoFeeding座標 - 修正版：加強重複檢查"""
        self.logger.debug("開始讀取AutoFeeding座標...")
        
        af_status = self.get_autofeeding_status()
        self.logger.debug(f"AutoFeeding狀態: DR_F可用={af_status['dr_f_available']}, 座標已讀取={af_status['coords_taken']}")
        
        if not af_status['dr_f_available']:
            self.logger.debug("AutoFeeding DR_F不可用 (940=0)")
            return None
        
        new_coords = (af_status['target_x'], af_status['target_y'])
        self.logger.debug(f"讀取到座標: {new_coords}")
        
        # 1. 保護區域檢查
        if not self.protection_zone.is_point_in_rect(new_coords[0], new_coords[1]):
            self.logger.warning(f"座標不在保護區域內: {new_coords}")
            self.logger.warning("保護區域: X(-112~-4), Y(243~339.21)")
            return None
        else:
            self.logger.debug("座標在保護區域內")
        
        # 2. 重複座標檢查 - 與上次使用的座標比較
        if self.last_coords and self.coords_equal(new_coords, self.last_coords):
            self.logger.warning(f"檢測到重複座標: {new_coords}")
            self.logger.warning(f"前次座標: {self.last_coords}")
            self.logger.warning("這表示AutoFeeding尚未更新座標")
            
            self.coords_duplicate_count += 1
            self.logger.warning(f"重複座標檢測次數: {self.coords_duplicate_count}")
            
            # 如果重複次數過多，暫停自動程序
            if self.coords_duplicate_count >= 3:
                self.logger.error(f"重複座標次數過多({self.coords_duplicate_count})，暫停自動程序")
                self.logger.error("建議檢查AutoFeeding是否正常更新座標")
                self.auto_program_enabled = False
                self.system_status = SystemStatus.ERROR
                self.write_register(1321, 0)  # 更新自動程序停用狀態
                self.write_register(1309, 101)  # 錯誤代碼: 重複座標過多
                
            return None
        else:
            if self.last_coords:
                self.logger.debug("座標已更新，與前次不同")
                self.logger.debug(f"前次: {self.last_coords}, 本次: {new_coords}")
            else:
                self.logger.debug("首次讀取座標")
        
        # 3. 確認座標已讀取
        self.logger.debug("設置座標已讀取標誌 (945=1)...")
        if not self.write_register(self.AF_COORDS_TAKEN, 1):
            self.logger.error("無法設置座標讀取標誌")
            return None
        
        time.sleep(self.config['autoprogram']['coords_confirm_delay'])
        self.logger.debug("座標讀取確認完成")
        
        self.logger.info(f"座標驗證通過: {new_coords}")
        return new_coords
    
    def store_coordinates_to_autoprogram(self, coords: Tuple[float, float]) -> bool:
        """將座標存儲到AutoProgram寄存器供Flow1讀取"""
        try:
            self.logger.debug(f"開始存儲座標到寄存器: {coords}")
            
            # 存儲到AutoProgram座標寄存器
            success = True
            
            # 逐個存儲並驗證
            x_success = self.write_32bit_coordinate(self.AP_TARGET_X_HIGH, self.AP_TARGET_X_LOW, coords[0])
            y_success = self.write_32bit_coordinate(self.AP_TARGET_Y_HIGH, self.AP_TARGET_Y_LOW, coords[1])
            
            success = x_success and y_success
            
            self.logger.debug(f"座標寫入結果: X={x_success}, Y={y_success}")
            
            if success:
                # 設置座標可用標誌
                flag_success = self.write_register(self.AP_COORDS_AVAILABLE, 1)
                self.logger.debug(f"可用標誌寫入結果: {flag_success}")
                
                if flag_success:
                    self.current_coords = coords
                    self.coords_ready = True
                    
                    # 立即驗證寫入結果
                    self.logger.debug("驗證寫入結果...")
                    verify_x = self.read_32bit_coordinate(self.AP_TARGET_X_HIGH, self.AP_TARGET_X_LOW)
                    verify_y = self.read_32bit_coordinate(self.AP_TARGET_Y_HIGH, self.AP_TARGET_Y_LOW)
                    verify_flag = self.read_register(self.AP_COORDS_AVAILABLE)
                    
                    self.logger.debug(f"驗證結果: X={verify_x:.2f}, Y={verify_y:.2f}, Flag={verify_flag}")
                    
                    if abs(verify_x - coords[0]) < 0.01 and abs(verify_y - coords[1]) < 0.01 and verify_flag == 1:
                        self.logger.info(f"座標已成功存儲並驗證: {coords}")
                        self.logger.info(f"Flow1可從地址{self.AP_COORDS_AVAILABLE}-{self.AP_TARGET_Y_LOW}讀取")
                        return True
                    else:
                        self.logger.error("座標驗證失敗")
                        return False
                else:
                    self.logger.error("可用標誌寫入失敗")
                    return False
            else:
                self.logger.error("座標寫入失敗")
                return False
            
        except Exception as e:
            self.logger.error(f"座標存儲異常: {e}", exc_info=True)
            return False
    
    def clear_autoprogram_coordinates(self):
        """清除AutoProgram座標狀態"""
        self.write_register(self.AP_COORDS_AVAILABLE, 0)
        for addr in [self.AP_TARGET_X_HIGH, self.AP_TARGET_X_LOW, 
                     self.AP_TARGET_Y_HIGH, self.AP_TARGET_Y_LOW]:
            self.write_register(addr, 0)
        
        # 更新內部狀態
        self.last_coords = self.current_coords
        self.current_coords = None
        self.coords_ready = False
        
        self.logger.debug("AutoProgram座標狀態已清除")
    
    def trigger_flow1(self) -> bool:
        """觸發Flow1取料作業"""
        if not self.coords_ready:
            self.logger.error("座標未準備就緒，無法觸發Flow1")
            return False
        
        self.logger.info("觸發Flow1取料作業")
        self.logger.info(f"目標座標: {self.current_coords}")
        
        # 觸發Flow1控制
        if not self.write_register(self.DOBOT_FLOW1_CONTROL, 1):
            self.logger.error(f"Flow1觸發失敗 (寫入{self.DOBOT_FLOW1_CONTROL}=1失敗)")
            return False
        
        time.sleep(self.config['autoprogram']['flow1_trigger_delay'])
        
        # 清除Flow1控制狀態
        self.write_register(self.DOBOT_FLOW1_CONTROL, 0)
        
        self.flow1_trigger_count += 1
        self.system_status = SystemStatus.FLOW1_TRIGGERED
        
        self.logger.info(f"Flow1已觸發 (第{self.flow1_trigger_count}次)")
        self.logger.info("Flow1將從AutoProgram寄存器讀取座標")
        return True
    
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
        self.logger.info(f"Flow2完成狀態已清除 ({self.DOBOT_FLOW2_COMPLETE}=0，第{self.flow2_complete_count}次)")
    
    def check_control_registers(self):
        """檢查控制寄存器變更"""
        try:
            # 檢查系統控制寄存器 (1320)
            system_control = self.read_register(1320)
            if system_control == 1 and not self.running:
                self.logger.info("檢測到系統啟動指令 (1320=1)")
                self.start()
            elif system_control == 0 and self.running:
                self.logger.info("檢測到系統停止指令 (1320=0)")
                self.stop()
            
            # 檢查自動程序控制寄存器 (1321)
            auto_control = self.read_register(1321)
            if auto_control is not None:
                if auto_control != (1 if self.auto_program_enabled else 0):
                    self.auto_program_enabled = (auto_control == 1)
                    self.logger.info(f"自動程序啟用狀態更新: {self.auto_program_enabled} (1321={auto_control})")
            
        except Exception as e:
            self.logger.error(f"控制寄存器檢查異常: {e}", exc_info=True)
    
    def coordination_cycle(self):
        """機械臂協調控制週期 - 修正版：不清空Flow1完成狀態，讓自動交握檢查"""
        try:
            self.coordination_cycle_count += 1
            
            # DEBUG: 每50個週期輸出一次狀態
            if self.coordination_cycle_count % 50 == 0:
                af_status = self.get_autofeeding_status()
                flow1_running = self.check_flow1_running()
                self.logger.debug(f"週期{self.coordination_cycle_count}: "
                    f"prepare_done={self.prepare_done}, "
                    f"coords_ready={self.coords_ready}, "
                    f"Flow1運行中={flow1_running}, "
                    f"DR_F可用={af_status['dr_f_available']}, "
                    f"自動程序啟用={self.auto_program_enabled}")
            
            # === 修正關鍵邏輯：檢測到Flow1完成但不清空！！！ ===
            if self.check_flow1_complete():
                self.logger.info("檢測到Flow1完成")
                # 🔥 關鍵修正：不再清空Flow1完成狀態！讓自動交握系統檢查
                # self.clear_flow1_complete()  # ← 註解掉這行！！！
                self.logger.warning("不清空Flow1完成狀態(1204)，讓自動交握檢查")
                
                self.clear_autoprogram_coordinates()  # 只清除座標狀態
                self.prepare_done = True
                self.logger.info("Flow1完成 → prepare_done=True")
                self.logger.info("機台準備就緒，等待Flow2完成")
                self.logger.info("Flow1完成狀態(1204=1)保持不變，供自動交握檢查")
                return  # 立即返回，避免在同一週期內觸發新操作
            
            # === 第二階段：檢查Flow2完成狀態 ===
            if self.check_flow2_complete():
                self.logger.info("檢測到Flow2完成，料件已送至組立區")
                self.clear_flow2_complete()
                self.prepare_done = False
                self.logger.info("Flow2完成 → prepare_done=False")
                self.logger.info("開始新週期，準備檢查AutoFeeding新座標")
                return  # 立即返回，下一週期才開始新的座標檢查
            
            # === 第三階段：主要協調邏輯 - 只在prepare_done=False時執行 ===
            if not self.prepare_done:
                self.logger.debug("prepare_done=False，開始處理新週期")
                
                # 首先檢查Flow1是否正在運行
                flow1_running = self.check_flow1_running()
                if flow1_running:
                    # Flow1正在運行，等待完成
                    if self.coordination_cycle_count % 50 == 0:
                        self.logger.debug("Flow1正在運行中(1201=1)，等待完成...")
                    return
                
                if not self.coords_ready:
                    # === 只有在prepare_done=False且coords_ready=False時才檢查AutoFeeding ===
                    self.logger.debug("座標未準備，開始檢查AutoFeeding新座標...")
                    
                    current_time = time.time()
                    
                    # 防止頻繁檢查AutoFeeding (至少間隔2秒)
                    if current_time - self.last_autofeeding_check_time < 2.0:
                        return
                    
                    self.system_status = SystemStatus.WAITING_COORDINATES
                    self.last_autofeeding_check_time = current_time
                    
                    # 檢查AutoFeeding新座標
                    coords = self.read_and_validate_autofeeding_coordinates()
                    if coords:
                        self.logger.info(f"從AutoFeeding讀取到新座標: {coords}")
                        
                        # 座標讀取成功，存儲到AutoProgram
                        if self.store_coordinates_to_autoprogram(coords):
                            self.dr_f_taken_count += 1
                            self.logger.info("座標已存儲到AutoProgram寄存器")
                            self.logger.info("準備觸發Flow1...")
                            
                            # 再次確認Flow1未運行才觸發
                            if not self.check_flow1_running():
                                if self.trigger_flow1():
                                    self.logger.info("Flow1已觸發，進入執行階段")
                                    self.logger.info("等待Flow1完成，完成後不會清空1204狀態")
                                else:
                                    self.logger.error("Flow1觸發失敗")
                            else:
                                self.logger.warning("座標準備完成但Flow1已在運行，跳過觸發")
                        else:
                            self.logger.error("座標存儲失敗，下週期重試")
                    else:
                        # 座標讀取失敗，等待下一週期
                        if self.coordination_cycle_count % 25 == 0:
                            af_status = self.get_autofeeding_status()
                            self.logger.debug(f"等待AutoFeeding新座標... (940={af_status.get('dr_f_available', 'N/A')})")
                            self.logger.debug("當前狀態: prepare_done=False, coords_ready=False")
                else:
                    # 座標已準備，檢查Flow1是否需要觸發
                    self.logger.debug("座標已準備，檢查Flow1觸發狀態...")
                    if not flow1_running:
                        # Flow1未運行，可以觸發
                        if self.trigger_flow1():
                            self.logger.info("Flow1已觸發 (座標已準備)")
                        else:
                            self.logger.error("Flow1觸發失敗")
                    # 如果Flow1正在運行，等待完成
            else:
                # === prepare_done=True：等待Flow2完成，完全不檢查AutoFeeding ===
                if self.coordination_cycle_count % 100 == 0:
                    self.logger.debug("prepare_done=True，等待Flow2完成...")
                    self.logger.debug("期間不檢查AutoFeeding，避免重複座標問題")
                    self.logger.debug("Flow1完成狀態(1204)保持不變，供自動交握檢查")
            
        except Exception as e:
            self.logger.error(f"協調週期異常: {e}", exc_info=True)
    
    def clear_flow1_complete(self):
        """
        清除Flow1完成狀態 - 已停用！！！
        ⚠️ 重要：不應該由AutoProgram清除Flow1完成狀態
        ⚠️ Flow1完成狀態應該由自動交握系統檢查後再清除
        """
        # 🔥 關鍵修正：完全註解掉清除邏輯
        # self.write_register(self.DOBOT_FLOW1_COMPLETE, 0)
        # self.logger.info(f"Flow1完成狀態已清除 ({self.DOBOT_FLOW1_COMPLETE}=0)")
        
        self.logger.warning("clear_flow1_complete() 已停用")
        self.logger.warning("Flow1完成狀態(1204)不由AutoProgram清除")
        self.logger.warning("讓自動交握系統檢查後清除")
    
    def update_system_registers(self):
        """更新系統寄存器"""
        try:
            if not self.connected:
                return
            
            # 更新系統狀態
            self.write_register(1300, self.system_status.value)
            self.write_register(1301, 1 if self.prepare_done else 0)
            self.write_register(1302, 1 if self.auto_program_enabled else 0)
            
            # 更新AutoFeeding狀態
            af_status = self.get_autofeeding_status()
            self.write_register(1303, 1 if af_status['dr_f_available'] else 0)
            self.write_register(1304, 1 if self.check_flow2_complete() else 0)
            
            # 更新統計資訊
            self.write_register(1305, self.coordination_cycle_count)
            self.write_register(1306, self.flow1_trigger_count)
            self.write_register(1307, self.flow2_complete_count)
            self.write_register(1308, self.dr_f_taken_count)
            self.write_register(1310, self.coords_duplicate_count)
            
            # 更新座標可用狀態
            self.write_register(1350, 1 if self.coords_ready else 0)
            
        except Exception as e:
            self.logger.error(f"系統寄存器更新失敗: {e}", exc_info=True)
    
    def start(self):
        """啟動機械臂協調控制系統"""
        if self.running:
            return
        
        self.logger.info("=== 啟動DR專案機械臂協調控制系統 (防重複觸發版) ===")
        self.running = True
        self.system_status = SystemStatus.RUNNING
        
        # 重置狀態
        self.prepare_done = False
        self.coords_ready = False
        self.current_coords = None
        self.coordination_cycle_count = 0
        self.flow1_trigger_count = 0
        self.flow2_complete_count = 0
        self.dr_f_taken_count = 0
        self.coords_duplicate_count = 0
        
        # 立即更新狀態寄存器
        self.write_register(1300, SystemStatus.RUNNING.value)
        self.write_register(1301, 0)  # prepare_done=False
        self.write_register(1350, 0)  # coords_ready=False
        
        self.thread = threading.Thread(target=self._coordination_loop, daemon=True)
        self.thread.start()
        
        self.logger.info("主循環執行緒已啟動")
        self.logger.info("DR專案協調控制系統已啟動")
        self.logger.info("防重複觸發策略:")
        self.logger.info(f"  - 檢查當前運動Flow狀態 (地址{self.DOBOT_CURRENT_MOTION_FLOW})")
        self.logger.info("  - 當Flow1運行中時(1201=1)不觸發新的Flow1")
        self.logger.info("  - prepare_done=False時才會觸發Flow1")
        self.logger.info("  - prepare_done=True時等待Flow2完成")
        self.logger.info("  - Flow2完成後prepare_done重置為False")
        self.logger.info("  - 保護區域檢查: X(-112~-4), Y(243~339.21)")
        self.logger.info("  - 重複座標檢測與自動停止")
        self.logger.info(f"  - 座標存儲地址: {self.AP_COORDS_AVAILABLE}-{self.AP_TARGET_Y_LOW}")
        self.logger.info(f"  - Flow1控制地址: {self.DOBOT_FLOW1_CONTROL}")
        self.logger.info("執行邏輯: 座標準備完成 → 檢查Flow1狀態 → 觸發Flow1 → 等待完成 → prepare_done=True → 等待Flow2")
    
    def stop(self):
        """停止機械臂協調控制系統"""
        if not self.running:
            return
        
        self.logger.info("=== 停止DR專案機械臂協調控制系統 ===")
        self.running = False
        self.system_status = SystemStatus.STOPPED
        
        # 清除座標狀態
        self.clear_autoprogram_coordinates()
        
        # 立即更新狀態寄存器
        self.write_register(1300, SystemStatus.STOPPED.value)
        
        # 更新系統寄存器
        self.update_system_registers()
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        
        self.logger.info("主循環執行緒已停止")
        self.logger.info("DR專案協調控制系統已停止")
        self.print_statistics()
    
    def _coordination_loop(self):
        """協調控制主循環"""
        interval = self.config['autoprogram']['coordination_interval']
        
        self.logger.info("DR專案協調控制主循環已啟動")
        
        loop_count = 0
        while True:
            try:
                loop_count += 1
                
                # DEBUG: 每100次循環輸出一次心跳
                if loop_count % 100 == 0:
                    self.logger.debug(f"控制循環心跳 - 第{loop_count}次, running={self.running}, auto_enabled={self.auto_program_enabled}")
                
                # 總是檢查控制寄存器變更
                self.check_control_registers()
                
                # 只有在系統運行且自動程序啟用時才執行協調邏輯
                if self.running and self.auto_program_enabled:
                    self.coordination_cycle()
                
                time.sleep(interval)
                
            except Exception as e:
                self.logger.error(f"協調循環異常: {e}", exc_info=True)
                self.logger.error("執行緒意外終止", exc_info=True)
                time.sleep(1.0)
    
    def disconnect(self):
        """斷開Modbus連接"""
        if self.modbus_client and self.connected:
            self.modbus_client.close()
            self.connected = False
            self.logger.info("Modbus連接已斷開")
    
    def print_statistics(self):
        """輸出統計資訊"""
        self.logger.info("=== DR專案AutoProgram統計資訊 (防重複觸發版) ===")
        self.logger.info(f"協調週期數: {self.coordination_cycle_count}")
        self.logger.info(f"Flow1觸發次數: {self.flow1_trigger_count}")
        self.logger.info(f"Flow2完成次數: {self.flow2_complete_count}")
        self.logger.info(f"DR_F取得次數: {self.dr_f_taken_count}")
        self.logger.info(f"重複座標檢測次數: {self.coords_duplicate_count}")
        self.logger.info(f"當前prepare_done狀態: {self.prepare_done}")
        self.logger.info(f"當前coords_ready狀態: {self.coords_ready}")
        self.logger.info(f"自動程序啟用: {self.auto_program_enabled}")
        self.logger.info("座標策略: AutoProgram讀取後主動觸發Flow1")
        self.logger.info("保護區域檢查: 啟用")
        self.logger.info("重複座標檢測: 啟用")
        self.logger.info("Flow1控制方式: 主動觸發 (1240地址)")
        self.logger.info("防重複觸發: 啟用 (檢查1201當前運動Flow)")
        self.logger.info("狀態機邏輯: prepare_done=False時觸發Flow1, prepare_done=True時等待Flow2")
    
    def get_status_info(self) -> Dict[str, Any]:
        """獲取狀態資訊"""
        af_status = self.get_autofeeding_status()
        
        return {
            "connected": self.connected,
            "system_status": self.system_status.name,
            "running": self.running,
            "auto_program_enabled": self.auto_program_enabled,
            "prepare_done": self.prepare_done,
            "coords_ready": self.coords_ready,
            "current_coords": self.current_coords,
            "last_coords": self.last_coords,
            "project_info": self.config.get("project_info", {}),
            "autofeeding_status": af_status,
            "flow1_complete": self.check_flow1_complete(),
            "flow2_complete": self.check_flow2_complete(),
            "statistics": {
                "coordination_cycle_count": self.coordination_cycle_count,
                "flow1_trigger_count": self.flow1_trigger_count,
                "flow2_complete_count": self.flow2_complete_count,
                "dr_f_taken_count": self.dr_f_taken_count,
                "coords_duplicate_count": self.coords_duplicate_count
            }
        }


def main():
    """主程序"""
    # 設置主程序logger
    main_logger = setup_logging('AutoProgram_Main')
    
    main_logger.info("DR專案機械臂協調控制模組啟動 (主動觸發Flow1版)")
    main_logger.info("主動觸發策略: AutoProgram讀取座標後立即主動觸發Flow1")
    main_logger.info("執行流程: Flow2 done->AutoFeeding done->AutoProgram get coordinates->AutoProgram trigger Flow1->Flow1 get coordinates->work flow")
    
    controller = AutoProgramController()
    
    if not controller.connect():
        main_logger.critical("Modbus連接失敗，程序退出")
        return
    
    try:
        main_logger.info("啟動控制循環，等待指令...")
        controller.thread = threading.Thread(target=controller._coordination_loop, daemon=True)
        controller.thread.start()
        
        def update_registers():
            update_logger = setup_logging('AutoProgram_RegisterUpdate')
            while True:
                try:
                    controller.update_system_registers()
                    time.sleep(1.0)
                except Exception as e:
                    update_logger.error(f"寄存器更新異常: {e}", exc_info=True)
                    time.sleep(2.0)
        
        update_thread = threading.Thread(target=update_registers, daemon=True)
        update_thread.start()
        
        main_logger.info("DR專案指令說明 (主動觸發Flow1版):")
        main_logger.info("  s - 顯示狀態")
        main_logger.info("  start - 手動啟動系統")
        main_logger.info("  stop - 手動停止系統")
        main_logger.info("  enable - 啟用自動程序")
        main_logger.info("  disable - 停用自動程序")
        main_logger.info("  flow1 - 手動觸發Flow1")
        main_logger.info("  coords - 手動讀取並存儲座標")
        main_logger.info("  clear_coords - 清除AutoProgram座標")
        main_logger.info("  check_af - 檢查AutoFeeding狀態")
        main_logger.info("  check_ap_coords - 檢查AutoProgram座標")
        main_logger.info("  q - 退出程序")
        
        while True:
            try:
                cmd = input("\n請輸入指令: ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 's':
                    status = controller.get_status_info()
                    main_logger.info("DR專案系統狀態 (主動觸發Flow1版):")
                    for key, value in status.items():
                        if isinstance(value, dict):
                            main_logger.info(f"  {key}:")
                            for sub_key, sub_value in value.items():
                                main_logger.info(f"    {sub_key}: {sub_value}")
                        else:
                            main_logger.info(f"  {key}: {value}")
                elif cmd == 'start':
                    controller.write_register(1320, 1)
                    main_logger.info("系統啟動指令已發送 (1320=1)")
                elif cmd == 'stop':
                    controller.write_register(1320, 0)
                    main_logger.info("系統停止指令已發送 (1320=0)")
                elif cmd == 'enable':
                    controller.auto_program_enabled = True
                    controller.write_register(1321, 1)
                    main_logger.info("自動程序已啟用")
                elif cmd == 'disable':
                    controller.auto_program_enabled = False
                    controller.write_register(1321, 0)
                    main_logger.info("自動程序已停用")
                elif cmd == 'flow1':
                    if controller.trigger_flow1():
                        main_logger.info("Flow1已觸發")
                    else:
                        main_logger.error("Flow1觸發失敗")
                elif cmd == 'coords':
                    coords = controller.read_and_validate_autofeeding_coordinates()
                    if coords:
                        if controller.store_coordinates_to_autoprogram(coords):
                            main_logger.info(f"座標已讀取並存儲: {coords}")
                            # 立即觸發Flow1
                            if controller.trigger_flow1():
                                main_logger.info("Flow1已自動觸發")
                            else:
                                main_logger.error("Flow1自動觸發失敗")
                        else:
                            main_logger.error("座標存儲失敗")
                    else:
                        main_logger.warning("無可用的有效座標")
                elif cmd == 'clear_coords':
                    controller.clear_autoprogram_coordinates()
                    main_logger.info("AutoProgram座標已清除")
                elif cmd == 'check_af':
                    af_status = controller.get_autofeeding_status()
                    main_logger.info(f"AutoFeeding狀態: {af_status}")
                elif cmd == 'check_ap_coords':
                    coords_available = controller.read_register(1350)
                    if coords_available == 1:
                        x = controller.read_32bit_coordinate(1351, 1352)
                        y = controller.read_32bit_coordinate(1353, 1354)
                        main_logger.info(f"AutoProgram座標: ({x:.2f}, {y:.2f})")
                    else:
                        main_logger.info("AutoProgram座標不可用")
                else:
                    main_logger.warning("無效指令")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
    
    finally:
        if controller.running:
            controller.stop()
        controller.disconnect()
        main_logger.info("DR專案程序已退出")


if __name__ == "__main__":
    main()