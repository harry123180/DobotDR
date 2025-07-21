#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GripperHighLevel.py - PGC夾爪高層API模組 (PyModbus 3.9.2)
保持完全向後相容，移除PGE支援，專注PGC夾爪
實現完整logging系統和狀態監控
"""

import time
import threading
import os
from typing import Optional, Dict, Any
from enum import IntEnum
import logging
from logging.handlers import RotatingFileHandler

# 導入Modbus TCP Client (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError as e:
    print(f"Modbus Client模組導入失敗: {e}")
    MODBUS_AVAILABLE = False


# ==================== 夾爪指令枚舉 ====================
class GripperCommand(IntEnum):
    """夾爪指令枚舉"""
    NOP = 0             # 無操作
    INITIALIZE = 1      # 初始化/回零
    STOP = 2           # 停止
    MOVE_ABS = 3       # 絕對位置移動
    SET_FORCE = 5      # 設定力道
    SET_SPEED = 6      # 設定速度
    QUICK_OPEN = 7     # 快速開啟
    QUICK_CLOSE = 8    # 快速關閉


# ==================== 夾爪狀態枚舉 ====================
class GripperStatus(IntEnum):
    """夾爪狀態枚舉"""
    MOVING = 0         # 運動中
    REACHED = 1        # 到達位置
    GRIPPED = 2        # 夾住物體
    DROPPED = 3        # 掉落


# ==================== 夾爪類型枚舉 (保持相容) ====================
class GripperType(IntEnum):
    """夾爪類型枚舉 - 向後相容，僅支援PGC"""
    PGC = 1           # PGC夾爪


# ==================== 夾爪高層API類 (保持原類名) ====================
class GripperHighLevelAPI:
    """
    夾爪高層API - 向後相容版本，專注PGC夾爪支援
    
    主要功能:
    1. 智能夾取 - 自動判斷夾取成功
    2. 快速指令 - 發了就走不等確認  
    3. 確認指令 - 等待動作完成
    4. 位置控制 - 精確位置移動
    5. 完整狀態監控和日誌記錄
    """
    
    def __init__(self, gripper_type: GripperType = GripperType.PGC, 
                 modbus_host: str = "127.0.0.1", modbus_port: int = 502,
                 auto_initialize: bool = True):
        """
        初始化夾爪高層API - 保持向後相容
        
        Args:
            gripper_type: 夾爪類型 (僅支援PGC，其他類型會拋出異常)
            modbus_host: Modbus TCP服務器IP
            modbus_port: Modbus TCP服務器端口
            auto_initialize: 是否自動初始化夾爪
        """
        # 向後相容檢查
        if gripper_type != GripperType.PGC:
            raise ValueError(f"此版本僅支援PGC夾爪，不支援類型: {gripper_type}")
        
        self.gripper_type = gripper_type
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        self.auto_initialize = auto_initialize
        
        # 線程鎖保護
        self.modbus_lock = threading.Lock()
        
        # PGC寄存器映射
        self._setup_pgc_registers()
        
        # 指令ID計數器
        self.command_id_counter = 1
        
        # 操作超時設定
        self.operation_timeout = 10.0  # 動作超時時間(秒)
        self.quick_timeout = 0.5       # 快速指令超時時間(秒)
        
        # 設置日誌
        self.logger = self._setup_logging()
        
        # 初始化狀態
        self.initialized = False
        
        # 自動連接
        self.connect()
        
    def _setup_logging(self):
        """設置logging配置"""
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
            os.path.join(log_dir, 'GripperHighLevel.log'),
            maxBytes=10*1024*1024,  # 10MB
            backupCount=7,          # 保留7個檔案
            encoding='utf-8'
        )
        file_handler.setFormatter(formatter)
        
        # 控制台處理器
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        
        # 配置logger
        logger = logging.getLogger("GripperHighLevel")
        logger.setLevel(logging.DEBUG)
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
        
        return logger
        
    def _setup_pgc_registers(self):
        """設定PGC夾爪寄存器映射 (基於專案技術文檔)"""
        self.REGISTERS = {
            # 狀態寄存器 (500-519)
            'MODULE_STATUS': 500,      # 模組狀態: 0=離線, 1=在線
            'CONNECT_STATUS': 501,     # 連接狀態: 0=斷開, 1=已連接
            'DEVICE_STATUS': 502,      # 設備狀態(初始化狀態)
            'ERROR_COUNT': 503,        # 錯誤計數
            'GRIP_STATUS': 504,        # 夾持狀態
            'CURRENT_POSITION': 505,   # 當前位置
            
            # 指令寄存器 (520-529)
            'COMMAND': 520,            # 指令代碼
            'PARAM1': 521,             # 參數1
            'PARAM2': 522,             # 參數2
            'COMMAND_ID': 523,         # 指令ID
        }
        
    def connect(self) -> bool:
        """
        連接到Modbus TCP服務器 - PyModbus 3.9.2修正版
        
        Returns:
            bool: 連接是否成功
        """
        if not MODBUS_AVAILABLE:
            self.logger.error("Modbus Client不可用")
            return False
        
        with self.modbus_lock:
            try:
                if self.modbus_client:
                    self.modbus_client.close()
                
                self.logger.info(f"正在連接Modbus TCP服務器: {self.modbus_host}:{self.modbus_port}")
                
                self.modbus_client = ModbusTcpClient(
                    host=self.modbus_host,
                    port=self.modbus_port,
                    timeout=3.0
                )
                
                if self.modbus_client.connect():
                    self.connected = True
                    self.logger.info("PGC夾爪Modbus連接成功")
                    
                    # 可選的自動初始化
                    if self.auto_initialize:
                        self.logger.info("開始自動初始化PGC夾爪")
                        if self.initialize():
                            self.initialized = True
                            self.logger.info("PGC夾爪初始化成功")
                        else:
                            self.logger.warning("PGC夾爪初始化失敗，但連接正常")
                    else:
                        self.logger.info("跳過自動初始化，需手動調用initialize()")
                    
                    return True
                else:
                    self.logger.error(f"Modbus TCP連接失敗: {self.modbus_host}:{self.modbus_port}")
                    self.connected = False
                    return False
                    
            except Exception as e:
                self.logger.error(f"Modbus TCP連接異常: {e}", exc_info=True)
                self.connected = False
                return False
    
    def disconnect(self):
        """斷開Modbus連接"""
        with self.modbus_lock:
            if self.modbus_client and self.connected:
                try:
                    self.modbus_client.close()
                    self.logger.info("Modbus TCP連接已斷開")
                except Exception as e:
                    self.logger.error(f"斷開連接時發生異常: {e}", exc_info=True)
            
            self.connected = False
            self.modbus_client = None
    
    def _write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器 - PyModbus 3.9.2修正版"""
        if not self.connected or not self.modbus_client:
            self.logger.error("Modbus未連接")
            return False
        
        with self.modbus_lock:
            try:
                address = self.REGISTERS[register_name]
                result = self.modbus_client.write_register(address=address, value=value)
                
                if not (hasattr(result, 'isError') and result.isError()):
                    self.logger.debug(f"寫入寄存器 {register_name}[{address}] = {value}")
                    return True
                else:
                    self.logger.error(f"寫入寄存器失敗: {register_name}[{address}] = {value}, 錯誤: {result}")
                    return False
                    
            except Exception as e:
                self.logger.error(f"寫入寄存器異常: {register_name} = {value}, 錯誤: {e}", exc_info=True)
                return False
    
    def _read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器 - PyModbus 3.9.2修正版"""
        if not self.connected or not self.modbus_client:
            self.logger.error("Modbus未連接")
            return None
        
        with self.modbus_lock:
            try:
                address = self.REGISTERS[register_name]
                result = self.modbus_client.read_holding_registers(address=address, count=1)
                
                if not (hasattr(result, 'isError') and result.isError()) and len(result.registers) > 0:
                    value = result.registers[0]
                    self.logger.debug(f"讀取寄存器 {register_name}[{address}] = {value}")
                    return value
                else:
                    self.logger.error(f"讀取寄存器失敗: {register_name}[{address}], 錯誤: {result}")
                    return None
                    
            except Exception as e:
                self.logger.error(f"讀取寄存器異常: {register_name}, 錯誤: {e}", exc_info=True)
                return None
    
    def _log_gripper_status(self, operation: str, success: bool):
        """記錄夾爪狀態 - 用於失敗時的詳細狀態記錄"""
        try:
            position = self.get_current_position()
            grip_status = self._read_register('GRIP_STATUS')
            device_status = self._read_register('DEVICE_STATUS')
            error_count = self._read_register('ERROR_COUNT')
            
            status_msg = (
                f"{operation} {'成功' if success else '失敗'} - "
                f"位置: {position}, 夾持狀態: {grip_status}, "
                f"初始化狀態: {device_status}, 錯誤計數: {error_count}"
            )
            
            if success:
                self.logger.info(status_msg)
            else:
                self.logger.error(status_msg)
                
        except Exception as e:
            self.logger.error(f"記錄夾爪狀態時發生異常: {e}", exc_info=True)
    
    # ==================== 初始化API (保持相容) ====================
    
    def initialize(self, wait_completion: bool = True, timeout: float = None) -> bool:
        """
        初始化夾爪
        
        Args:
            wait_completion: 是否等待初始化完成
            timeout: 超時時間(秒)，None使用預設值
            
        Returns:
            bool: 初始化是否成功
        """
        self.logger.info("開始初始化PGC夾爪")
        
        # 使用自定義超時時間或預設值
        actual_timeout = timeout if timeout is not None else self.operation_timeout
        
        try:
            if not self._send_command(GripperCommand.INITIALIZE):
                self._log_gripper_status("初始化指令發送", False)
                return False
                
            if wait_completion:
                success = self._wait_for_completion(actual_timeout)
                self._log_gripper_status("初始化", success)
                if success:
                    self.initialized = True
                return success
            
            self.logger.info("初始化指令已發送，不等待完成")
            return True
                
        except Exception as e:
            self.logger.error(f"初始化失敗: {e}", exc_info=True)
            self._log_gripper_status("初始化", False)
            return False
    
    # ==================== 智能夾取API (保持相容) ====================
    
    def smart_grip(self, target_position: int = 420, max_attempts: int = 3) -> bool:
        """
        智能夾取 - 自動判斷夾取成功
        
        Args:
            target_position: 目標位置 (0-1000)
            max_attempts: 最大嘗試次數
            
        Returns:
            bool: 夾取是否成功
        """
        self.logger.info(f"PGC智能夾取到位置: {target_position}, 最大嘗試次數: {max_attempts}")
        
        for attempt in range(max_attempts):
            try:
                # 記錄初始位置
                initial_pos = self.get_current_position()
                if initial_pos is None:
                    self.logger.warning("無法讀取初始位置")
                    initial_pos = 0
                else:
                    self.logger.debug(f"初始位置: {initial_pos}")
                
                # 移動到目標位置
                if not self._send_command(GripperCommand.MOVE_ABS, target_position):
                    self.logger.warning(f"第{attempt + 1}次嘗試：移動指令發送失敗")
                    continue
                
                # 等待運動完成
                if not self._wait_for_completion(self.operation_timeout):
                    self.logger.warning(f"第{attempt + 1}次嘗試：等待運動完成超時")
                    continue
                
                # 檢查是否夾到物體
                final_pos = self.get_current_position()
                if final_pos is None:
                    self.logger.warning(f"第{attempt + 1}次嘗試：無法讀取最終位置")
                    continue
                
                # 如果位置差異大於閾值，表示夾到物體
                position_diff = abs(target_position - final_pos)
                self.logger.debug(f"目標位置: {target_position}, 最終位置: {final_pos}, 位置差異: {position_diff}")
                
                if position_diff > 20:  # 閾值可調整
                    self.logger.info(f"PGC智能夾取成功 (嘗試{attempt + 1}/{max_attempts}, 位置差異: {position_diff})")
                    self._log_gripper_status("智能夾取", True)
                    return True
                else:
                    self.logger.warning(f"第{attempt + 1}次嘗試：未夾到物體 (位置差異: {position_diff})")
                    
            except Exception as e:
                self.logger.error(f"第{attempt + 1}次嘗試：智能夾取異常: {e}", exc_info=True)
                
        self.logger.error(f"PGC智能夾取失敗，已用完{max_attempts}次嘗試")
        self._log_gripper_status("智能夾取", False)
        return False
    
    # ==================== 快速指令API (保持相容) ====================
    
    def quick_open(self, position: int = None) -> bool:
        """
        快速開啟 - 發了就走不等確認
        
        Args:
            position: 開啟位置，None使用預設
            
        Returns:
            bool: 指令發送是否成功
        """
        if position is None:
            self.logger.info("PGC快速開啟 (預設位置)")
            success = self._send_command(GripperCommand.QUICK_OPEN)
        else:
            self.logger.info(f"PGC快速開啟到位置: {position}")
            success = self._send_command(GripperCommand.MOVE_ABS, position)
        
        if success:
            self.logger.info("快速開啟指令發送成功")
        else:
            self._log_gripper_status("快速開啟", False)
            
        return success
    
    def quick_close(self) -> bool:
        """快速關閉 - 檢測位置變化確認動作"""
        self.logger.info("PGC快速關閉")
        
        # 記錄發送指令前的夾爪位置
        position_before = self.get_current_position()
        if position_before is None:
            self.logger.warning("無法讀取快速關閉前的位置")
            position_before = 0
        else:
            self.logger.debug(f"快速關閉前位置: {position_before}")
        
        # 發送快速關閉指令
        command_success = self._send_command(GripperCommand.QUICK_CLOSE)
        
        if not command_success:
            self.logger.error("快速關閉指令發送失敗")
            self._log_gripper_status("快速關閉", False)
            return True
        
        # 等待0.1秒讓夾爪開始動作
        time.sleep(0.1)
        return True
        # # 記錄發送指令後的位置
        # position_after = self.get_current_position()
        # if position_after is None:
        #     self.logger.warning("無法讀取快速關閉後的位置")
        #     position_after = position_before
        # else:
        #     self.logger.debug(f"快速關閉後位置: {position_after}")
        
        # # 計算位置變化：前位置 - 後位置
        # position_change = position_before - position_after
        # self.logger.debug(f"位置變化: {position_before} - {position_after} = {position_change}")
        
        # # 判斷夾爪是否有在關閉動作
        # if position_change > 0:
        #     self.logger.info(f"快速關閉成功，位置變化: {position_change}")
        #     return True
        # else:
        #     self.logger.error(f"快速關閉失敗，位置無變化或反向移動: {position_change}")
        #     self._log_gripper_status("快速關閉", False)
        #     return False
    
    def smart_release(self, release_position: int = 50) -> bool:
        """智能釋放 - 移動到指定位置並等待完成"""
        self.logger.info(f"PGC智能釋放到位置: {release_position}")
        success = self.move_to_and_wait(release_position)
        self._log_gripper_status("智能釋放", success)
        return success
    
    # ==================== 位置控制API (保持相容) ====================
    
    def move_to_and_wait(self, position: int) -> bool:
        """
        移動到指定位置並等待完成
        
        Args:
            position: 目標位置 (0-1000)
            
        Returns:
            bool: 移動是否成功
        """
        self.logger.info(f"PGC移動到位置: {position}")
        
        if not self._send_command(GripperCommand.MOVE_ABS, position):
            self._log_gripper_status("移動指令發送", False)
            return False
        
        success = self._wait_for_completion(self.operation_timeout)
        self._log_gripper_status("位置移動", success)
        return success
    
    def set_force(self, force_percent: int) -> bool:
        """
        設定PGC夾爪力道
        
        Args:
            force_percent: 力道百分比 (20-100)
            
        Returns:
            bool: 設定是否成功
        """
        if not 20 <= force_percent <= 100:
            self.logger.error(f"PGC力道超出範圍: {force_percent} (應為20-100)")
            return False
        
        self.logger.info(f"設定PGC夾爪力道: {force_percent}%")
        success = self._send_command(GripperCommand.SET_FORCE, force_percent)
        
        if not success:
            self._log_gripper_status("設定力道", False)
            
        return success
    
    def set_speed(self, speed_percent: int) -> bool:
        """
        設定PGC夾爪速度
        
        Args:
            speed_percent: 速度百分比 (1-100)
            
        Returns:
            bool: 設定是否成功
        """
        if not 1 <= speed_percent <= 100:
            self.logger.error(f"PGC速度超出範圍: {speed_percent} (應為1-100)")
            return False
        
        self.logger.info(f"設定PGC夾爪速度: {speed_percent}%")
        success = self._send_command(GripperCommand.SET_SPEED, speed_percent)
        
        if not success:
            self._log_gripper_status("設定速度", False)
            
        return success
    
    # ==================== 狀態查詢API (保持相容) ====================
    
    def get_current_position(self) -> Optional[int]:
        """取得當前位置"""
        return self._read_register('CURRENT_POSITION')
    
    def get_grip_status(self) -> Optional[int]:
        """取得夾持狀態"""
        return self._read_register('GRIP_STATUS')
    
    def get_device_status(self) -> Optional[int]:
        """取得設備狀態(初始化狀態)"""
        return self._read_register('DEVICE_STATUS')
    
    def get_error_count(self) -> Optional[int]:
        """取得錯誤計數"""
        return self._read_register('ERROR_COUNT')
    
    def get_status(self) -> Dict[str, Any]:
        """取得夾爪狀態資訊 - 保持原格式相容"""
        status_info = {
            'gripper_type': self.gripper_type.name,
            'connected': self.connected,
            'initialized': self.initialized,
            'current_position': self.get_current_position(),
            'device_status': self.get_device_status(),
            'grip_status': self.get_grip_status(),
            'error_count': self.get_error_count(),
            'module_status': self._read_register('MODULE_STATUS'),
            'connect_status': self._read_register('CONNECT_STATUS')
        }
        
        return status_info
    
    # ==================== 向後相容方法 ====================
    
    def is_initialized(self) -> bool:
        """檢查是否已初始化 - 向後相容方法"""
        return self.initialized
    
    # ==================== 內部方法 ====================
    
    def _send_command(self, command: GripperCommand, param1: int = 0, param2: int = 0) -> bool:
        """發送PGC夾爪指令"""
        try:
            # 獲取指令ID
            cmd_id = self.command_id_counter
            self.command_id_counter += 1
            if self.command_id_counter > 65535:
                self.command_id_counter = 1
            
            # 寫入指令參數
            if not self._write_register('PARAM1', param1):
                return False
            if not self._write_register('PARAM2', param2):
                return False
            if not self._write_register('COMMAND_ID', cmd_id):
                return False
            
            # 發送指令
            if not self._write_register('COMMAND', int(command)):
                return False
            
            self.logger.debug(f"發送PGC指令: {command.name}({param1}, {param2}) ID={cmd_id}")
            return True
            
        except Exception as e:
            self.logger.error(f"發送PGC指令失敗: {command.name}({param1}, {param2}), 錯誤: {e}", exc_info=True)
            return False
    
    def _wait_for_completion(self, timeout: float) -> bool:
        """等待PGC夾爪動作完成"""
        start_time = time.time()
        self.logger.debug(f"開始等待動作完成，超時時間: {timeout}秒")
        
        while time.time() - start_time < timeout:
            # 檢查夾爪狀態
            status = self._read_register('GRIP_STATUS')
            if status in [GripperStatus.REACHED, GripperStatus.GRIPPED]:
                elapsed_time = time.time() - start_time
                self.logger.debug(f"動作完成，耗時: {elapsed_time:.2f}秒，狀態: {status}")
                return True
            elif status == GripperStatus.DROPPED:
                elapsed_time = time.time() - start_time
                self.logger.warning(f"動作失敗(掉落)，耗時: {elapsed_time:.2f}秒")
                return False
            
            time.sleep(0.1)
        
        self.logger.warning(f"PGC動作完成等待超時 (超時時間: {timeout}秒)")
        return False


# ==================== 使用範例 (保持相容) ====================
if __name__ == "__main__":
    # 向後相容的使用方式 - 跳過自動初始化避免卡住
    gripper = GripperHighLevelAPI(gripper_type=GripperType.PGC, auto_initialize=False)
    
    try:
        if gripper.connected:
            print("PGC夾爪連接成功，開始手動初始化...")
            
            # 手動初始化，使用較短超時時間
            if gripper.initialize(wait_completion=True, timeout=3.0):
                print("PGC夾爪初始化成功")
            else:
                print("PGC夾爪初始化失敗，但可繼續測試")
            
            # 測試智能夾取
            print("\n=== 智能夾取測試 ===")
            if gripper.smart_grip(420):
                print("智能夾取成功")
            else:
                print("智能夾取失敗")
            
            time.sleep(2)
            
            # 測試智能釋放
            print("\n=== 智能釋放測試 ===")
            if gripper.smart_release(50):
                print("智能釋放成功")
            else:
                print("智能釋放失敗")
            
            # 顯示狀態
            print("\n=== 當前狀態 ===")
            status = gripper.get_status()
            for key, value in status.items():
                print(f"{key}: {value}")
                
        else:
            print("PGC夾爪連接失敗")
            
    except KeyboardInterrupt:
        print("\n程序中斷")
    finally:
        gripper.disconnect()
        print("PGC夾爪已斷開連接")