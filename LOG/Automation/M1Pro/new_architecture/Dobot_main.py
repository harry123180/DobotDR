#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_main.py - 機械臂主控制器 (DR專案新架構混合交握協議版)
實現混合交握協議：
- 運動類Flow (Flow1,Flow2,Flow5): 基地址1200-1249，狀態機交握
- IO類Flow (Flow3,Flow4): 地址447-449，專用佇列併行執行
確保運動安全性的同時提供IO操作並行能力
含詳細調試訊息用於排查1200之後地址讀寫問題
輪詢速度優化：50ms → 10ms
修正版：解決高頻logger創建導致的文件句柄洩漏問題
"""

import json
import os
import time
import threading
import traceback
import queue
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field
from enum import Enum, IntEnum
import logging
from logging.handlers import RotatingFileHandler

# 導入流程架構模組
from Dobot_Flow1 import DrFlow1VisionPickExecutor
from Dobot_Flow2 import DrFlow2UnloadExecutor  
from Dobot_Flow4 import Flow4VibrationFeedExecutor

# 導入高階API模組
from CCD1HighLevel import CCD1HighLevelAPI
from GripperHighLevel import GripperHighLevelAPI, GripperType
from AngleHighLevel import AngleHighLevel

from pymodbus.client.tcp import ModbusTcpClient
from dobot_api import DobotApiDashboard, DobotApiMove

# 配置常數
CONFIG_FILE = "dobot_config.json"

# ==================== 日誌設置 ====================

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

# 初始化主模組logger
logger = setup_logging('DobotMain')

# ==================== 調試控制開關 ====================
ENABLE_HANDSHAKE_DEBUG = False  # True=開啟HandshakeLoop調試訊息, False=關閉HandshakeLoop調試訊息

# ==================== 新架構寄存器映射 ====================

class MotionRegisters:
    """運動類Flow寄存器映射 (基地址1200-1249) - 修正地址衝突版本"""
    
    # 運動狀態寄存器 (1200-1219) - 只讀
    MOTION_STATUS = 1200          # 運動狀態寄存器 (bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized)
    CURRENT_MOTION_FLOW = 1201    # 當前運動Flow (0=無, 1=Flow1, 2=Flow2, 5=Flow5)
    MOTION_PROGRESS = 1202        # 運動進度 (0-100百分比)
    MOTION_ERROR_CODE = 1203      # 運動錯誤碼
    FLOW1_COMPLETE = 1204         # Flow1完成狀態 (0=未完成, 1=完成且角度校正成功)
    FLOW2_COMPLETE = 1205         # Flow2完成狀態
    FLOW5_COMPLETE = 1206         # Flow5完成狀態
    MOTION_OP_COUNT = 1207        # 運動操作計數
    MOTION_ERR_COUNT = 1208       # 運動錯誤計數
    MOTION_RUN_TIME = 1209        # 運動系統運行時間(分鐘)
    # 1210-1219 保留狀態寄存器
    
    # 運動控制寄存器 (1240-1249) - 讀寫
    FLOW1_CONTROL = 1240          # Flow1控制 (0=清空, 1=啟動VP視覺抓取)
    FLOW2_CONTROL = 1241          # Flow2控制 (0=清空, 1=啟動出料流程)
    FLOW5_CONTROL = 1242          # Flow5控制 (0=清空, 1=啟動機械臂運轉)
    MOTION_CLEAR_ALARM = 1243     # 運動清除警報 (0=無動作, 1=清除Alarm)
    MOTION_EMERGENCY_STOP = 1244  # 運動緊急停止 (0=正常, 1=緊急停止)
    # 1245-1249 保留控制寄存器

class IORegisters:
    """IO類Flow寄存器映射 (447-449) - 保持不變"""
    
    FLOW3_CONTROL = 447           # Flow3控制 (0=清空, 1=啟動翻轉站)
    FLOW4_CONTROL = 448           # Flow4控制 (0=清空, 1=啟動震動投料)
    IO_RESERVED = 449             # 保留IO控制

# ==================== 指令系統 ====================

class CommandType(Enum):
    """指令類型"""
    MOTION = "motion"               # 運動類指令 (Flow1,2,5)
    DIO_FLIP = "dio_flip"          # IO類翻轉站指令 (Flow3)
    DIO_VIBRATION = "dio_vibration" # IO類震動投料指令 (Flow4)
    EXTERNAL = "external"          # 外部模組指令
    EMERGENCY = "emergency"        # 緊急指令

class CommandPriority(IntEnum):
    """指令優先權"""
    EMERGENCY = 0
    MOTION = 1
    DIO_FLIP = 2
    DIO_VIBRATION = 2
    EXTERNAL = 3

@dataclass
class Command:
    """統一指令格式"""
    command_type: CommandType
    command_data: Dict[str, Any]
    priority: CommandPriority
    timestamp: float = field(default_factory=time.time)
    command_id: int = field(default=0)
    callback: Optional[callable] = None

    def __lt__(self, other):
        return self.priority < other.priority if self.priority != other.priority else self.timestamp < other.timestamp

# ==================== 專用指令佇列系統 ====================

class DedicatedCommandQueue:
    """專用指令佇列 - 避免執行緒間競爭"""
    
    def __init__(self, name: str, max_size: int = 50):
        self.name = name
        self.queue = queue.Queue(max_size)
        self.command_id_counter = 1
        self._lock = threading.Lock()
        self.put_count = 0
        self.get_count = 0
        self.logger = setup_logging(f'Queue_{name}')
        
    def put_command(self, command: Command) -> bool:
        """加入指令到專用佇列"""
        try:
            with self._lock:
                command.command_id = self.command_id_counter
                self.command_id_counter += 1
                
            self.queue.put_nowait(command)
            self.put_count += 1
            
            self.logger.info(f"指令已加入 - ID:{command.command_id}, 類型:{command.command_type.value}, 佇列大小:{self.queue.qsize()}")
            return True
            
        except queue.Full:
            self.logger.warning(f"佇列已滿，丟棄指令: {command.command_type}")
            return False
        except Exception as e:
            self.logger.error(f"加入指令失敗: {e}", exc_info=True)
            return False
            
    def get_command(self, timeout: Optional[float] = None) -> Optional[Command]:
        """取得指令"""
        try:
            command = self.queue.get(timeout=timeout)
            self.get_count += 1
            
            if command:
                self.logger.info(f"指令已取出 - ID:{command.command_id}, 類型:{command.command_type.value}, 剩餘:{self.queue.qsize()}")
            
            return command
            
        except queue.Empty:
            return None
        except Exception as e:
            self.logger.error(f"取得指令失敗: {e}", exc_info=True)
            return None
            
    def size(self) -> int:
        return self.queue.qsize()
        
    def get_stats(self) -> Dict[str, int]:
        return {
            'current_size': self.size(),
            'total_put': self.put_count,
            'total_get': self.get_count,
            'pending': self.put_count - self.get_count
        }

# ==================== 運動類狀態機 ====================

class MotionStateMachine:
    """運動類Flow狀態機 - 修正地址1200版本"""
    
    def __init__(self, modbus_client: ModbusTcpClient):
        self.modbus_client = modbus_client
        self.status_register = 0x08  # 初始化位=1
        self.current_flow = 0
        self.progress = 0
        self.error_code = 0
        self.operation_count = 0
        self.error_count = 0
        self.run_time_minutes = 0
        self._lock = threading.Lock()
        
        # Flow完成狀態
        self.flow1_complete = 0
        self.flow2_complete = 0  
        self.flow5_complete = 0
        
        self.logger = setup_logging('MotionStateMachine')
        self.logger.info(f"MotionStateMachine初始化完成 - 新基地址: {MotionRegisters.MOTION_STATUS}")
        
    def _update_status_register_only(self):
        """只更新狀態寄存器，不影響Flow完成狀態 - 新增方法"""
        try:
            self.logger.debug("只更新狀態寄存器")
            self.logger.debug(f"狀態寄存器: 地址{MotionRegisters.MOTION_STATUS} = {self.status_register} ({self.status_register:04b})")
            
            # 只寫入狀態寄存器
            status_result = self.modbus_client.write_register(address=MotionRegisters.MOTION_STATUS, value=self.status_register)
            if hasattr(status_result, 'isError') and status_result.isError():
                self.logger.error(f"狀態寄存器寫入失敗: {status_result}")
            else:
                self.logger.debug("狀態寄存器寫入成功 (保持Flow完成狀態不變)")
                
        except Exception as e:
            self.logger.error(f"只更新狀態寄存器失敗: {e}", exc_info=True)     
            
    def set_ready(self, ready: bool = True):
        """設置Ready狀態 - 使用新地址1200"""
        try:
            old_register = self.status_register
            with self._lock:
                if ready:
                    self.status_register |= 0x01   # 設置Ready位
                    self.status_register &= ~0x06  # 清除Running和Alarm位
                else:
                    self.status_register &= ~0x01  # 清除Ready位
                    
            self.logger.debug(f"set_ready({ready}): {old_register:04b} -> {self.status_register:04b}")
            self._update_status_register_only()
        except Exception as e:
            self.logger.error(f"設置運動Ready狀態失敗: {e}", exc_info=True)
            
    def set_running(self, running: bool = True):
        """設置Running狀態 - 使用新地址1200"""
        try:
            old_register = self.status_register
            with self._lock:
                if running:
                    self.status_register |= 0x02   # 設置Running位
                    self.status_register &= ~0x05  # 清除Ready和Alarm位
                else:
                    self.status_register &= ~0x02  # 清除Running位
                    
            self.logger.debug(f"set_running({running}): {old_register:04b} -> {self.status_register:04b}")
            self._update_status_to_plc()
        except Exception as e:
            self.logger.error(f"設置運動Running狀態失敗: {e}", exc_info=True)
            
    def set_alarm(self, alarm: bool = True):
        """設置Alarm狀態 - 使用新地址1200"""
        try:
            old_register = self.status_register
            with self._lock:
                if alarm:
                    self.status_register |= 0x04   # 設置Alarm位
                    self.status_register &= ~0x03  # 清除Ready和Running位
                    self.error_count += 1
                else:
                    self.status_register &= ~0x04  # 清除Alarm位
                    
            self.logger.warning(f"set_alarm({alarm}): {old_register:04b} -> {self.status_register:04b}")
            if alarm:
                self.logger.warning(f"錯誤計數增加至: {self.error_count}")
            self._update_status_to_plc()
        except Exception as e:
            self.logger.error(f"設置運動Alarm狀態失敗: {e}", exc_info=True)
            
    def set_current_flow(self, flow_id: int):
        """設置當前流程ID - 使用新地址1201"""
        try:
            with self._lock:
                old_flow = self.current_flow
                self.current_flow = flow_id
            
            self.logger.info(f"set_current_flow({flow_id}): {old_flow} -> {flow_id}")
            self.logger.debug(f"寫入寄存器 {MotionRegisters.CURRENT_MOTION_FLOW} = {flow_id}")
            
            result = self.modbus_client.write_register(address=MotionRegisters.CURRENT_MOTION_FLOW, value=flow_id)
            if hasattr(result, 'isError') and result.isError():
                self.logger.error(f"寫入失敗: {result}")
            else:
                self.logger.debug(f"寫入成功: 地址{MotionRegisters.CURRENT_MOTION_FLOW} = {flow_id}")
                
            # 驗證寫入結果
            verify_result = self.modbus_client.read_holding_registers(address=MotionRegisters.CURRENT_MOTION_FLOW, count=1)
            if hasattr(verify_result, 'registers') and len(verify_result.registers) > 0:
                actual_value = verify_result.registers[0]
                self.logger.debug(f"驗證讀取: 地址{MotionRegisters.CURRENT_MOTION_FLOW} = {actual_value}")
            else:
                self.logger.error(f"驗證讀取失敗: {verify_result}")
                
        except Exception as e:
            self.logger.error(f"設置運動流程ID失敗: {e}", exc_info=True)
            
    def set_progress(self, progress: int):
        """設置進度 - 使用新地址1202"""
        try:
            with self._lock:
                old_progress = self.progress
                self.progress = max(0, min(100, progress))
            
            self.logger.debug(f"set_progress({progress}): {old_progress} -> {self.progress}")
            self.logger.debug(f"寫入寄存器 {MotionRegisters.MOTION_PROGRESS} = {self.progress}")
            
            result = self.modbus_client.write_register(address=MotionRegisters.MOTION_PROGRESS, value=self.progress)
            if hasattr(result, 'isError') and result.isError():
                self.logger.error(f"寫入失敗: {result}")
            else:
                self.logger.debug(f"寫入成功: 地址{MotionRegisters.MOTION_PROGRESS} = {self.progress}")
                
        except Exception as e:
            self.logger.error(f"設置運動進度失敗: {e}", exc_info=True)
            
    def set_flow_complete(self, flow_id: int, complete: bool = True):
        """設置Flow完成狀態 - 修正版：確保Flow1(1204)寫入成功"""
        try:
            value = 1 if complete else 0
            address = None
            
            # 更新內部狀態
            with self._lock:
                if flow_id == 1:
                    old_value = self.flow1_complete
                    self.flow1_complete = value
                    address = MotionRegisters.FLOW1_COMPLETE  # 1204
                    self.logger.debug(f"Flow1內部狀態: {old_value} → {value}")
                elif flow_id == 2:
                    old_value = self.flow2_complete
                    self.flow2_complete = value
                    address = MotionRegisters.FLOW2_COMPLETE  # 1205
                    self.logger.debug(f"Flow2內部狀態: {old_value} → {value}")
                elif flow_id == 5:
                    old_value = self.flow5_complete
                    self.flow5_complete = value
                    address = MotionRegisters.FLOW5_COMPLETE  # 1206
                    self.logger.debug(f"Flow5內部狀態: {old_value} → {value}")
                else:
                    self.logger.error(f"未知Flow ID: {flow_id}")
                    return False
                    
            self.logger.info(f"=== 設置Flow{flow_id}完成狀態 (地址{address}) ===")
            self.logger.info(f"目標地址: {address}")
            self.logger.info(f"設置值: {value} ({'完成' if complete else '清除'})")
            
            # 關鍵修正：立即寫入到Modbus，重試機制確保成功
            max_retries = 3
            write_success = False
            
            for retry in range(max_retries):
                try:
                    self.logger.debug(f"寫入嘗試 {retry + 1}/{max_retries}")
                    
                    result = self.modbus_client.write_register(address=address, value=value)
                    if hasattr(result, 'isError') and result.isError():
                        self.logger.warning(f"寫入失敗 (嘗試{retry + 1}): {result}")
                        if retry < max_retries - 1:
                            time.sleep(0.1)  # 短暫等待後重試
                            continue
                        else:
                            break
                    else:
                        self.logger.debug(f"寫入成功 (嘗試{retry + 1}): 地址{address} = {value}")
                        write_success = True
                        break
                        
                except Exception as e:
                    self.logger.error(f"寫入異常 (嘗試{retry + 1}): {e}", exc_info=True)
                    if retry < max_retries - 1:
                        time.sleep(0.1)
                        continue
                    else:
                        break
            
            if not write_success:
                self.logger.error(f"Flow{flow_id}完成狀態寫入失敗 (所有重試都失敗)")
                return False
            
            # 關鍵修正：立即驗證寫入結果，多次驗證確保可靠性
            verification_success = False
            for verify_attempt in range(3):
                try:
                    time.sleep(0.05)  # 短暫延遲確保寫入完成
                    verify_result = self.modbus_client.read_holding_registers(address=address, count=1)
                    
                    if hasattr(verify_result, 'registers') and len(verify_result.registers) > 0:
                        actual_value = verify_result.registers[0]
                        self.logger.debug(f"驗證結果 (嘗試{verify_attempt + 1}): 地址{address} = {actual_value}")
                        
                        if actual_value == value:
                            self.logger.info(f"Flow{flow_id}完成狀態驗證成功")
                            verification_success = True
                            break
                        else:
                            self.logger.warning(f"Flow{flow_id}完成狀態驗證失敗！期望{value}，實際{actual_value}")
                            if verify_attempt < 2:
                                # 如果驗證失敗，嘗試重新寫入
                                self.logger.debug("重新寫入以修正驗證失敗...")
                                self.modbus_client.write_register(address=address, value=value)
                            continue
                    else:
                        self.logger.warning(f"無法驗證Flow{flow_id}完成狀態寫入 (嘗試{verify_attempt + 1})")
                        continue
                        
                except Exception as e:
                    self.logger.error(f"驗證異常 (嘗試{verify_attempt + 1}): {e}", exc_info=True)
                    continue
            
            if not verification_success:
                self.logger.warning(f"Flow{flow_id}完成狀態驗證失敗，可能有其他程序覆蓋寄存器")
                return False
                
            # 更新操作計數
            if complete:
                self.operation_count += 1
                self.logger.info(f"更新操作計數: {self.operation_count}")
                
                try:
                    op_result = self.modbus_client.write_register(address=MotionRegisters.MOTION_OP_COUNT, value=self.operation_count)
                    if hasattr(op_result, 'isError') and op_result.isError():
                        self.logger.error(f"操作計數寫入失敗: {op_result}")
                    else:
                        self.logger.debug(f"操作計數寫入成功: 地址{MotionRegisters.MOTION_OP_COUNT} = {self.operation_count}")
                except Exception as e:
                    self.logger.error(f"操作計數寫入異常: {e}", exc_info=True)
            
            self.logger.info(f"=== Flow{flow_id}完成狀態設置完成 ===")
            return True
                
        except Exception as e:
            self.logger.error(f"設置Flow{flow_id}完成狀態失敗: {e}", exc_info=True)
            return False
            
    def is_ready_for_command(self) -> bool:
        """檢查是否可接受新的運動指令"""
        ready = (self.status_register & 0x01) != 0
        self.logger.debug(f"is_ready_for_command(): 狀態寄存器={self.status_register:04b}, Ready位={ready}")
        return ready
        
    def _verify_critical_registers_only(self):
        """只驗證關鍵寄存器，不讀取Flow完成狀態避免混淆"""
        try:
            self.logger.debug("驗證關鍵寄存器寫入結果")
            
            # 只讀取和驗證狀態相關寄存器 (1200-1203)
            status_result = self.modbus_client.read_holding_registers(address=MotionRegisters.MOTION_STATUS, count=4)
            if hasattr(status_result, 'registers') and len(status_result.registers) >= 4:
                registers = status_result.registers
                self.logger.debug("關鍵狀態寄存器驗證:")
                self.logger.debug(f"  1200: {registers[0]} ({registers[0]:04b}) - 運動狀態")
                self.logger.debug(f"  1201: {registers[1]} - 當前Flow")
                self.logger.debug(f"  1202: {registers[2]} - 進度%")
                self.logger.debug(f"  1203: {registers[3]} - 錯誤碼")
                self.logger.debug("Flow完成狀態(1204-1206)未讀取，避免干擾")
            else:
                self.logger.error(f"關鍵寄存器驗證失敗: {status_result}")
                
        except Exception as e:
            self.logger.error(f"驗證關鍵寄存器失敗: {e}", exc_info=True)
    
    def _update_status_to_plc(self):
        """更新狀態到PLC - 修正版：絕對不覆蓋Flow完成狀態"""
        try:
            self.logger.debug("更新狀態到PLC (保護Flow完成狀態)")
            self.logger.debug(f"狀態寄存器: 地址{MotionRegisters.MOTION_STATUS} = {self.status_register} ({self.status_register:04b})")
            self.logger.debug(f"錯誤計數: 地址{MotionRegisters.MOTION_ERR_COUNT} = {self.error_count}")
            self.logger.debug(f"Flow完成狀態保護: F1={self.flow1_complete}, F2={self.flow2_complete}, F5={self.flow5_complete}")
            
            # 1. 只寫入狀態寄存器
            status_result = self.modbus_client.write_register(address=MotionRegisters.MOTION_STATUS, value=self.status_register)
            if hasattr(status_result, 'isError') and status_result.isError():
                self.logger.error(f"狀態寄存器寫入失敗: {status_result}")
            else:
                self.logger.debug("狀態寄存器寫入成功")
                
            # 2. 只寫入錯誤計數
            err_result = self.modbus_client.write_register(address=MotionRegisters.MOTION_ERR_COUNT, value=self.error_count)
            if hasattr(err_result, 'isError') and err_result.isError():
                self.logger.error(f"錯誤計數寫入失敗: {err_result}")
            else:
                self.logger.debug("錯誤計數寫入成功")
                
            # 3. 關鍵修正：保護Flow完成狀態，不進行無意義的重寫
            self.logger.debug("Flow完成狀態保護中 - 不進行自動重寫")
            self.logger.debug(f"當前內部狀態: F1={self.flow1_complete}, F2={self.flow2_complete}, F5={self.flow5_complete}")
                
            # 4. 驗證寫入結果 - 修正版：不打印Flow完成狀態值（避免誤導）
            self._verify_critical_registers_only()
            
        except Exception as e:
            self.logger.error(f"更新運動狀態到PLC失敗: {e}", exc_info=True)
            
    def _verify_register_writes(self):
        """驗證寄存器寫入結果 - 使用新地址範圍"""
        try:
            self.logger.debug("驗證寄存器寫入結果")
            
            # 讀取狀態寄存器範圍 (1200-1209)
            status_result = self.modbus_client.read_holding_registers(address=MotionRegisters.MOTION_STATUS, count=10)
            if hasattr(status_result, 'registers') and len(status_result.registers) >= 10:
                registers = status_result.registers
                self.logger.debug("狀態寄存器驗證 (1200-1209):")
                for i, reg_value in enumerate(registers):
                    addr = MotionRegisters.MOTION_STATUS + i
                    if addr == MotionRegisters.MOTION_STATUS:
                        self.logger.debug(f"  {addr}: {reg_value} ({reg_value:04b}) - 運動狀態")
                    elif addr == MotionRegisters.CURRENT_MOTION_FLOW:
                        self.logger.debug(f"  {addr}: {reg_value} - 當前Flow")
                    elif addr == MotionRegisters.MOTION_PROGRESS:
                        self.logger.debug(f"  {addr}: {reg_value} - 進度%")
                    elif addr == MotionRegisters.MOTION_ERROR_CODE:
                        self.logger.debug(f"  {addr}: {reg_value} - 錯誤碼")
                    elif addr == MotionRegisters.FLOW1_COMPLETE:
                        self.logger.debug(f"  {addr}: {reg_value} - Flow1完成")
                    elif addr == MotionRegisters.FLOW2_COMPLETE:
                        self.logger.debug(f"  {addr}: {reg_value} - Flow2完成")
                    elif addr == MotionRegisters.FLOW5_COMPLETE:
                        self.logger.debug(f"  {addr}: {reg_value} - Flow5完成")
                    elif addr == MotionRegisters.MOTION_OP_COUNT:
                        self.logger.debug(f"  {addr}: {reg_value} - 操作計數")
                    elif addr == MotionRegisters.MOTION_ERR_COUNT:
                        self.logger.debug(f"  {addr}: {reg_value} - 錯誤計數")
                    elif addr == MotionRegisters.MOTION_RUN_TIME:
                        self.logger.debug(f"  {addr}: {reg_value} - 運行時間")
            else:
                self.logger.error(f"狀態寄存器驗證失敗: {status_result}")
                
        except Exception as e:
            self.logger.error(f"驗證寄存器寫入失敗: {e}", exc_info=True)

# ==================== 真實機械臂控制器 ====================

class RealRobotController:
    """真實機械臂控制器 - 修正Sync()缺失問題"""
    
    def __init__(self, ip: str, dashboard_port: int = 29999, move_port: int = 30003):
        self.ip = ip
        self.dashboard_port = dashboard_port
        self.move_port = move_port
        self.is_connected = False
        self.dashboard_api = None
        self.move_api = None
        self.global_speed = 100
        
        self.logger = setup_logging('RobotController')
        
    def _parse_api_response(self, response: str) -> bool:
        """解析API響應"""
        if not response:
            return False
        try:
            parts = response.strip().split(',')
            if len(parts) >= 1:
                error_code = int(parts[0])
                return error_code == 0
            return False
        except (ValueError, IndexError):
            return False
    
    def _extract_mode_from_response(self, response: str) -> Optional[int]:
        """從RobotMode響應中提取模式值"""
        try:
            if not response:
                return None
            
            parts = response.strip().split(',')
            if len(parts) >= 2:
                mode_part = parts[1].strip()
                
                if mode_part.startswith('{') and mode_part.endswith('}'):
                    mode_part = mode_part[1:-1]
                
                return int(mode_part)
            return None
        except (ValueError, IndexError):
            return None
            
    def initialize(self) -> bool:
        """初始化機械臂連接"""
        try:
            self.logger.info(f"正在初始化機械臂: {self.ip}")
            
            self.dashboard_api = DobotApiDashboard(self.ip, self.dashboard_port)
            self.move_api = DobotApiMove(self.ip, self.move_port)
            
            clear_result = self.dashboard_api.ClearError()
            if self._parse_api_response(clear_result):
                self.logger.info("清除錯誤成功")
            else:
                self.logger.warning(f"清除錯誤失敗: {clear_result}")
                
            enable_result = self.dashboard_api.EnableRobot()
            if self._parse_api_response(enable_result):
                self.logger.info("機械臂啟用成功")
            else:
                self.logger.error(f"機械臂啟用失敗: {enable_result}")
                return False
            
            time.sleep(2.0)
            
            if self.set_global_speed(self.global_speed):
                self.logger.info(f"初始速度設定成功: {self.global_speed}%")
            else:
                self.logger.warning("初始速度設定失敗")
            
            self.is_connected = True
            self.logger.info(f"機械臂初始化成功: {self.ip}")
            return True
            
        except Exception as e:
            self.logger.error(f"機械臂初始化失敗: {e}", exc_info=True)
            return False
    
    def set_arm_orientation(self, orientation: int) -> bool:
        """設置機械臂座標系方向"""
        try:
            result = self.dashboard_api.SetArmOrientation(orientation)
            success = self._parse_api_response(result)
            if success:
                self.logger.info(f"座標系切換成功: {'左手系' if orientation == 0 else '右手系'}")
            else:
                self.logger.error(f"座標系切換失敗: {result}")
            return success
        except Exception as e:
            self.logger.error(f"座標系切換異常: {e}", exc_info=True)
            return False
    
    def set_global_speed(self, speed_percent: int) -> bool:
        """設定全局速度"""
        try:
            if not 1 <= speed_percent <= 100:
                self.logger.error(f"速度超出範圍: {speed_percent}")
                return False
                
            result = self.dashboard_api.SpeedFactor(speed_percent)
            success = self._parse_api_response(result)
            if success:
                self.global_speed = speed_percent
                self.logger.info(f"全局速度設定成功: {speed_percent}%")
            else:
                self.logger.error(f"全局速度設定失敗: {result}")
            return success
        except Exception as e:
            self.logger.error(f"設定全局速度異常: {e}", exc_info=True)
            return False
    
    def move_j(self, x: float, y: float, z: float, r: float) -> bool:
        """關節運動 - 修正版，加入Sync()調用"""
        try:
            self.logger.info(f"開始MovJ: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
            
            # 發送運動指令到隊列
            result = self.move_api.MovJ(x, y, z, r)
            success = self._parse_api_response(result)
            
            if not success:
                self.logger.error(f"MovJ指令發送失敗: {result}")
                return False
            
            self.logger.debug("MovJ指令發送成功，調用Sync()執行...")
            
            # 關鍵修正：調用Sync()執行隊列中的指令
            sync_result = self.move_api.Sync()
            sync_success = self._parse_api_response(sync_result)
            
            if sync_success:
                self.logger.info(f"MovJ完成: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
                return True
            else:
                self.logger.error(f"MovJ同步執行失敗: {sync_result}")
                return False
                
        except Exception as e:
            self.logger.error(f"MovJ執行異常: {e}", exc_info=True)
            return False
    
    def move_l(self, x: float, y: float, z: float, r: float) -> bool:
        """直線運動 - 修正版，加入Sync()調用"""
        try:
            self.logger.info(f"開始MovL: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
            
            # 發送運動指令到隊列
            result = self.move_api.MovL(x, y, z, r)
            success = self._parse_api_response(result)
            
            if not success:
                self.logger.error(f"MovL指令發送失敗: {result}")
                return False
            
            self.logger.debug("MovL指令發送成功，調用Sync()執行...")
            
            # 關鍵修正：調用Sync()執行隊列中的指令
            sync_result = self.move_api.Sync()
            sync_success = self._parse_api_response(sync_result)
            
            if sync_success:
                self.logger.info(f"MovL完成: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
                return True
            else:
                self.logger.error(f"MovL同步執行失敗: {sync_result}")
                return False
                
        except Exception as e:
            self.logger.error(f"MovL執行異常: {e}", exc_info=True)
            return False
    
    def joint_move_j(self, j1: float, j2: float, j3: float, j4: float) -> bool:
        """關節角度運動 - 修正版，加入Sync()調用"""
        try:
            self.logger.info(f"開始JointMovJ: (j1:{j1:.1f}, j2:{j2:.1f}, j3:{j3:.1f}, j4:{j4:.1f})")
            
            # 發送關節運動指令到隊列
            result = self.move_api.JointMovJ(j1, j2, j3, j4)
            success = self._parse_api_response(result)
            
            if not success:
                self.logger.error(f"JointMovJ指令發送失敗: {result}")
                return False
            
            self.logger.debug("JointMovJ指令發送成功，調用Sync()執行...")
            
            # 關鍵修正：調用Sync()執行隊列中的指令
            sync_result = self.move_api.Sync()
            sync_success = self._parse_api_response(sync_result)
            
            if sync_success:
                self.logger.info(f"JointMovJ完成: (j1:{j1:.1f}, j2:{j2:.1f}, j3:{j3:.1f}, j4:{j4:.1f})")
                return True
            else:
                self.logger.error(f"JointMovJ同步執行失敗: {sync_result}")
                return False
                
        except Exception as e:
            self.logger.error(f"JointMovJ執行異常: {e}", exc_info=True)
            return False
    
    def sync(self) -> bool:
        """同步等待所有運動完成 - 修正版"""
        try:
            self.logger.debug("執行Sync()同步等待...")
            result = self.move_api.Sync()
            success = self._parse_api_response(result)
            
            if success:
                self.logger.debug("Sync()同步完成")
                return True
            else:
                self.logger.error(f"Sync()同步失敗: {result}")
                return False
                
        except Exception as e:
            self.logger.error(f"同步等待失敗: {e}", exc_info=True)
            return False
    
    def set_do(self, pin: int, value: int) -> bool:
        """設定數位輸出"""
        try:
            result = self.dashboard_api.DOExecute(pin, value)
            success = self._parse_api_response(result)
            if success:
                self.logger.info(f"DO{pin}設定為{value}")
            else:
                self.logger.error(f"DO{pin}設定失敗: {result}")
            return success
        except Exception as e:
            self.logger.error(f"設定DO失敗: {e}", exc_info=True)
            return False
    
    def get_di(self, pin: int) -> Optional[int]:
        """讀取數位輸入"""
        try:
            result = self.dashboard_api.DI(pin)
            if self._parse_api_response(result):
                parts = result.strip().split(',')
                if len(parts) >= 2:
                    di_part = parts[1].strip()
                    if di_part.startswith('{') and di_part.endswith('}'):
                        di_part = di_part[1:-1]
                    return int(di_part)
            return None
        except Exception as e:
            self.logger.error(f"讀取DI失敗: {e}", exc_info=True)
            return None
    
    def emergency_stop(self) -> bool:
        """緊急停止"""
        try:
            result = self.dashboard_api.EmergencyStop()
            success = self._parse_api_response(result)
            if success:
                self.logger.critical("緊急停止執行成功")
            else:
                self.logger.error(f"緊急停止執行失敗: {result}")
            return success
        except Exception as e:
            self.logger.error(f"緊急停止失敗: {e}", exc_info=True)
            return False
    
    def get_current_pose(self) -> Optional[Dict[str, float]]:
        """獲取當前位置"""
        try:
            result = self.dashboard_api.GetPose()
            if self._parse_api_response(result):
                parts = result.strip().split(',')
                if len(parts) >= 5:
                    return {
                        'x': float(parts[1]),
                        'y': float(parts[2]), 
                        'z': float(parts[3]),
                        'r': float(parts[4])
                    }
            return None
        except Exception as e:
            self.logger.error(f"獲取位置失敗: {e}", exc_info=True)
            return None
    
    def disconnect(self) -> bool:
        """斷開機械臂連接"""
        try:
            if self.dashboard_api:
                disable_result = self.dashboard_api.DisableRobot()
                if self._parse_api_response(disable_result):
                    self.logger.info("機械臂已停用")
                else:
                    self.logger.warning(f"機械臂停用失敗: {disable_result}")
                self.dashboard_api.close()
            if self.move_api:
                self.move_api.close()
            self.is_connected = False
            return True
        except Exception as e:
            self.logger.error(f"機械臂斷開連接失敗: {e}", exc_info=True)
            return False

# ==================== 執行緒基類 ====================

class BaseFlowThread(threading.Thread):
    """執行緒基類"""
    
    def __init__(self, name: str, command_queue: DedicatedCommandQueue):
        super().__init__(daemon=True, name=name)
        self.command_queue = command_queue
        self.running = False
        self.status = "停止"
        self.last_error = ""
        self.operation_count = 0
        self.logger = setup_logging(f'Thread_{name}')
        
    def start_thread(self):
        self.running = True
        self.start()
        self.logger.info("執行緒已啟動")
        
    def stop_thread(self):
        self.running = False
        self.logger.info("執行緒停止信號已發送")
        
    def get_status(self) -> Dict[str, Any]:
        return {
            'name': self.name,
            'running': self.running,
            'status': self.status,
            'last_error': self.last_error,
            'operation_count': self.operation_count,
            'queue_stats': self.command_queue.get_stats()
        }

# ==================== 運動控制執行緒 ====================

class MotionFlowThread(BaseFlowThread):
    """運動控制執行緒 - 處理Flow1、Flow2、Flow5"""
    
    def __init__(self, robot: RealRobotController, command_queue: DedicatedCommandQueue, 
                 motion_state_machine: MotionStateMachine, external_modules: Dict):
        super().__init__("MotionFlow", command_queue)
        self.robot = robot
        self.motion_state_machine = motion_state_machine
        self.external_modules = external_modules
        self.flow_executors = {}
        
    def initialize_flows(self):
        """初始化Flow執行器"""
        try:
            # Flow1: VP視覺抓取 - DR專案版本
            flow1 = DrFlow1VisionPickExecutor()
            flow1.initialize(self.robot, self.motion_state_machine, self.external_modules)
            self.flow_executors[1] = flow1
            
            # Flow2: CV出料流程 - DR專案版本
            flow2 = DrFlow2UnloadExecutor()
            flow2.initialize(self.robot, self.motion_state_machine, self.external_modules)
            self.flow_executors[2] = flow2
            
            # 註：DR專案通常不需要Flow5，如需要可加入
            
            self.logger.info("運動Flow執行器初始化完成 (Flow1, Flow2) - DR專案版本")
            
        except Exception as e:
            self.logger.error(f"運動Flow執行器初始化失敗: {e}", exc_info=True)
            self.last_error = str(e)
    
    def run(self):
        """運動控制執行緒主循環"""
        self.status = "運行中"
        self.logger.info("執行緒啟動 - 處理運動類Flow")
        
        while self.running:
            try:
                command = self.command_queue.get_command(timeout=0.1)
                
                if command and command.command_type == CommandType.MOTION:
                    self.logger.info(f"收到運動指令，ID: {command.command_id}")
                    self._handle_motion_command(command)
                    
            except Exception as e:
                self.last_error = f"運動控制執行緒錯誤: {e}"
                self.logger.error(self.last_error, exc_info=True)
                
        self.status = "已停止"
        self.logger.info("執行緒結束")
    
    def _handle_motion_command(self, command: Command):
        """處理運動指令"""
        try:
            cmd_data = command.command_data
            cmd_type = cmd_data.get('type', '')
            
            if cmd_type == 'flow1_vp_vision_pick':
                self._execute_flow1()
            elif cmd_type == 'flow2_unload':
                self._execute_flow2()
            elif cmd_type == 'flow5_assembly':
                self._execute_flow5()
            else:
                self.logger.warning(f"未知運動指令類型: {cmd_type}")
                
            self.operation_count += 1
            
        except Exception as e:
            self.last_error = f"處理運動指令失敗: {e}"
            self.logger.error(self.last_error, exc_info=True)
    
    def _execute_flow1(self):
        """執行Flow1 - VP視覺抓取 - DR專案版本 (修正完成狀態寫入)"""
        try:
            self.logger.info("開始執行Flow1 - VP視覺抓取 (DR專案)")
            self.motion_state_machine.set_running(True)
            self.motion_state_machine.set_current_flow(1)
            self.motion_state_machine.set_progress(0)
            
            # 關鍵修正：Flow1開始時先清除之前的完成狀態
            self.logger.info("清除Flow1之前的完成狀態...")
            self.motion_state_machine.set_flow_complete(1, False)
            
            flow1 = self.flow_executors.get(1)
            if flow1:
                result = flow1.execute()
                
                if result.success:
                    self.logger.info("Flow1執行成功")
                    
                    # 關鍵修正：Flow1成功完成後確保設置完成狀態
                    self.logger.info("=== 設置Flow1完成狀態 ===")
                    
                    # 方法1：直接調用motion_state_machine（主要方法）
                    try:
                        self.motion_state_machine.set_flow_complete(1, True)
                        self.logger.info("透過MotionStateMachine設置Flow1完成狀態")
                        
                        # 額外驗證：確保寫入成功
                        time.sleep(0.1)
                        if hasattr(self.motion_state_machine, 'modbus_client'):
                            verify_result = self.motion_state_machine.modbus_client.read_holding_registers(
                                address=1204, count=1  # Flow1完成狀態寄存器
                            )
                            if hasattr(verify_result, 'registers') and len(verify_result.registers) > 0:
                                actual_value = verify_result.registers[0]
                                self.logger.debug(f"驗證Flow1完成狀態寄存器1204: {actual_value}")
                                if actual_value == 1:
                                    self.logger.info("Flow1完成狀態寫入驗證成功")
                                else:
                                    self.logger.warning(f"Flow1完成狀態寫入驗證失敗，實際值: {actual_value}")
                                    # 如果驗證失敗，嘗試重新寫入
                                    self.logger.info("嘗試重新寫入Flow1完成狀態...")
                                    self.motion_state_machine.modbus_client.write_register(address=1204, value=1)
                            else:
                                self.logger.warning("無法驗證Flow1完成狀態寫入")
                        else:
                            self.logger.warning("無法訪問modbus_client進行驗證")
                            
                    except Exception as e:
                        self.logger.error(f"Flow1完成狀態設置異常: {e}", exc_info=True)
                    
                    # 方法2：如果Flow1執行器有自己的設置方法，也調用一次
                    if hasattr(flow1, '_safe_set_flow1_complete_status'):
                        try:
                            flow1._safe_set_flow1_complete_status(True)
                            self.logger.info("透過Flow1執行器自有方法設置完成狀態")
                        except Exception as e:
                            self.logger.error(f"Flow1執行器自有方法設置失敗: {e}", exc_info=True)
                    
                    # 設置系統狀態
                    self.motion_state_machine.set_progress(100)
                    self.motion_state_machine.set_running(False)
                    self.motion_state_machine.set_current_flow(0)
                    self.motion_state_machine.set_ready(True)
                    
                    self.logger.info("=== Flow1完成狀態設置完成 ===")
                    
                else:
                    self.logger.error(f"Flow1執行失敗: {result.error_message}")
                    
                    # 失敗時清除完成狀態
                    self.motion_state_machine.set_flow_complete(1, False)
                    self.motion_state_machine.set_alarm(True)
                    self.motion_state_machine.set_running(False)
                    self.motion_state_machine.set_current_flow(0)
            else:
                self.logger.error("Flow1執行器未初始化")
                
                # 執行器未初始化時清除完成狀態
                self.motion_state_machine.set_flow_complete(1, False)
                self.motion_state_machine.set_alarm(True)
                
        except Exception as e:
            self.logger.error(f"Flow1執行異常: {e}", exc_info=True)
            
            # 異常時清除完成狀態
            self.motion_state_machine.set_flow_complete(1, False)
            self.motion_state_machine.set_alarm(True)
            self.motion_state_machine.set_running(False)
            self.motion_state_machine.set_current_flow(0)
    
    def _execute_flow2(self):
        """執行Flow2 - CV出料流程 - DR專案版本"""
        try:
            self.logger.info("開始執行Flow2 - CV出料流程 (DR專案)")
            self.motion_state_machine.set_running(True)
            self.motion_state_machine.set_current_flow(2)
            self.motion_state_machine.set_progress(0)
            
            flow2 = self.flow_executors.get(2)
            if flow2:
                result = flow2.execute()
                
                if result.success:
                    self.logger.info("Flow2執行成功")
                    self.motion_state_machine.set_flow_complete(2, True)
                    self.motion_state_machine.set_progress(100)
                    self.motion_state_machine.set_running(False)
                    self.motion_state_machine.set_current_flow(0)
                    self.motion_state_machine.set_ready(True)
                else:
                    self.logger.error(f"Flow2執行失敗: {result.error_message}")
                    self.motion_state_machine.set_alarm(True)
                    self.motion_state_machine.set_running(False)
                    self.motion_state_machine.set_current_flow(0)
            else:
                self.logger.error("Flow2執行器未初始化")
                self.motion_state_machine.set_alarm(True)
                
        except Exception as e:
            self.logger.error(f"Flow2執行異常: {e}", exc_info=True)
            self.motion_state_machine.set_alarm(True)
            self.motion_state_machine.set_running(False)
            self.motion_state_machine.set_current_flow(0)
    
    def _execute_flow5(self):
        """執行Flow5 - 機械臂運轉流程 (保留接口)"""
        try:
            self.logger.warning("Flow5在DR專案中尚未實現")
            self.motion_state_machine.set_alarm(True)
                
        except Exception as e:
            self.logger.error(f"Flow5執行異常: {e}", exc_info=True)
            self.motion_state_machine.set_alarm(True)
            self.motion_state_machine.set_running(False)
            self.motion_state_machine.set_current_flow(0)

# ==================== IO類Flow執行緒 ====================

class Flow4VibrationFeedThread(BaseFlowThread):
    """Flow4震動投料控制專用執行緒 - IO類併行"""
    
    def __init__(self, robot: RealRobotController, command_queue: DedicatedCommandQueue):
        super().__init__("Flow4VibrationFeed", command_queue)
        self.robot = robot
        self.flow4_executor = None
        
    def initialize_flows(self):
        """初始化Flow4執行器"""
        try:
            flow4 = Flow4VibrationFeedExecutor()
            flow4.initialize(self.robot, None, {})
            self.flow4_executor = flow4
            self.logger.info("Flow4震動投料執行器初始化完成")
        except Exception as e:
            self.logger.error(f"Flow4執行器初始化失敗: {e}", exc_info=True)
            self.last_error = str(e)
    
    def run(self):
        """Flow4執行緒主循環 - IO類併行處理"""
        self.status = "運行中"
        self.logger.info("執行緒啟動，專用佇列接收DIO_VIBRATION指令")
        
        while self.running:
            try:
                command = self.command_queue.get_command(timeout=0.2)
                
                if command:
                    self.logger.info(f"收到指令 - ID:{command.command_id}, 類型:{command.command_type.value}")
                    
                    if command.command_type == CommandType.DIO_VIBRATION:
                        cmd_type = command.command_data.get('type', '')
                        if cmd_type == 'flow_vibration_feed':
                            self.logger.info(f"開始處理震動投料指令，ID: {command.command_id}")
                            self._execute_vibration_feed()
                        else:
                            self.logger.warning(f"未知指令子類型: {cmd_type}")
                    else:
                        self.logger.warning(f"收到非DIO_VIBRATION指令，忽略: {command.command_type}")
                        
            except Exception as e:
                self.last_error = f"Flow4執行緒錯誤: {e}"
                self.logger.error(self.last_error, exc_info=True)
                time.sleep(0.1)
                
        self.status = "已停止"
        self.logger.info("執行緒結束")
    
    def _execute_vibration_feed(self):
        """執行震動投料控制 - IO類併行"""
        try:
            self.logger.info("=== 開始執行震動投料控制 (IO類併行) ===")
            start_time = time.time()
            
            if not self.flow4_executor:
                self.logger.error("Flow4執行器未初始化")
                return
            
            result = self.flow4_executor.execute()
            execution_time = time.time() - start_time
            
            if result.success:
                self.logger.info(f"震動投料控制執行成功，耗時: {execution_time:.2f}秒")
                self.logger.info(f"完成步驟: {result.steps_completed}/{result.total_steps}")
            else:
                self.logger.error(f"震動投料控制執行失敗: {result.error_message}")
                self.logger.error(f"完成步驟: {result.steps_completed}/{result.total_steps}")
                
            self.operation_count += 1
            self.logger.info("=== 震動投料控制執行完成 ===")
                
        except Exception as e:
            self.logger.error(f"震動投料控制執行異常: {e}", exc_info=True)

# ==================== 外部模組執行緒 ====================

class ExternalModuleThread(BaseFlowThread):
    """外部模組交握執行緒"""
    
    def __init__(self, command_queue: DedicatedCommandQueue, external_modules: Dict):
        super().__init__("ExternalModule", command_queue)
        self.external_modules = external_modules
        
    def run(self):
        """外部模組執行緒主循環"""
        self.status = "運行中"
        self.logger.info("執行緒啟動")
        
        while self.running:
            try:
                command = self.command_queue.get_command(timeout=0.1)
                
                if command and command.command_type == CommandType.EXTERNAL:
                    self._handle_external_command(command)
                    
            except Exception as e:
                self.last_error = f"外部模組執行緒錯誤: {e}"
                self.logger.error(self.last_error, exc_info=True)
                
        self.status = "已停止"
        self.logger.info("執行緒結束")
    
    def _handle_external_command(self, command: Command):
        """處理外部模組指令"""
        try:
            cmd_data = command.command_data
            module_name = cmd_data.get('module', '')
            operation = cmd_data.get('operation', '')
            params = cmd_data.get('params', {})
            
            if module_name in self.external_modules:
                module = self.external_modules[module_name]
                success = self._handle_module_operation(module, module_name, operation, params)
                
                if success:
                    self.logger.info(f"{module_name}.{operation} 執行成功")
                else:
                    self.logger.error(f"{module_name}.{operation} 執行失敗")
                    
                self.operation_count += 1
            else:
                self.logger.warning(f"未知外部模組: {module_name}")
                
        except Exception as e:
            self.last_error = f"執行外部模組指令失敗: {e}"
            self.logger.error(self.last_error, exc_info=True)
            
    def _handle_module_operation(self, module, module_name: str, operation: str, params: Dict) -> bool:
        """處理模組操作"""
        try:
            if hasattr(module, operation):
                method = getattr(module, operation)
                if callable(method):
                    if params:
                        return method(**params)
                    else:
                        return method()
            return True
        except Exception as e:
            self.logger.error(f"模組操作執行失敗: {e}", exc_info=True)
            return False

# ==================== 主控制器 ====================

class DobotNewArchController:
    """Dobot新架構混合交握控制器 - DR專案版本"""
    
    def __init__(self, config_file: str = CONFIG_FILE):
        # 首先初始化logger
        self.logger = setup_logging('DobotController')
        
        # 關鍵修正：在初始化時創建HandshakeLoop logger，避免高頻創建
        self.handshake_logger = setup_logging('HandshakeLoop')
        
        self.config_file = config_file
        self.config = self._load_config()
        
        # 專用指令佇列 - 每個執行緒一個佇列
        self.motion_queue = DedicatedCommandQueue("Motion")
        self.flow4_queue = DedicatedCommandQueue("Flow4")
        self.external_queue = DedicatedCommandQueue("External")
        
        # 核心組件
        self.robot = None
        self.modbus_client = None
        self.motion_state_machine = None
        
        # 執行緒 - DR專案通常不需要Flow3
        self.motion_thread = None
        self.flow4_thread = None
        self.external_thread = None
        self.handshake_thread = None
        
        # 狀態
        self.running = False
        self.external_modules = {}
        
        # 控制狀態緩存
        self.last_flow1_control = 0
        self.last_flow2_control = 0
        self.last_flow5_control = 0
        self.last_flow4_control = 0
        self.last_motion_clear_alarm = 0
        
        # 統計資訊
        self.start_time = time.time()
        self.loop_count = 0
        
    def _load_config(self) -> Dict[str, Any]:
        """載入配置"""
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), self.config_file)
        
        default_config = {
            "robot": {
                "ip": "192.168.1.6",
                "dashboard_port": 29999,
                "move_port": 30003,
                "default_speed": 100
            },
            "modbus": {
                "server_ip": "127.0.0.1", 
                "server_port": 502,
                "timeout": 3.0
            },
            "gripper": {
                "type": "PGE",
                "enabled": True
            },
            "vision": {
                "ccd1_enabled": True
            },
            "flows": {
                "flow1_enabled": True,
                "flow2_enabled": True,
                "flow3_enabled": False,  # DR專案通常不需要
                "flow4_enabled": True,
                "flow5_enabled": False   # DR專案通常不需要
            }
        }
        
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    user_config = json.load(f)
                    self._deep_update(default_config, user_config)
                self.logger.info(f"載入配置檔案成功: {config_path}")
            except Exception as e:
                self.logger.error(f"載入配置檔案失敗，使用預設配置: {e}", exc_info=True)
        else:
            try:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                self.logger.info(f"創建預設配置檔案: {config_path}")
            except Exception as e:
                self.logger.error(f"創建配置檔案失敗: {e}", exc_info=True)
                
        return default_config
    
    def _deep_update(self, base_dict: Dict, update_dict: Dict):
        """深度更新字典"""
        for key, value in update_dict.items():
            if key in base_dict and isinstance(base_dict[key], dict) and isinstance(value, dict):
                self._deep_update(base_dict[key], value)
            else:
                base_dict[key] = value
    
    def start(self) -> bool:
        """啟動控制器"""
        self.logger.info("=== 啟動Dobot新架構混合交握控制器 (DR專案) ===")
        self.logger.info("運動類Flow (Flow1,2): 基地址1200-1249，狀態機交握")
        self.logger.info("IO類Flow (Flow4): 地址448，專用佇列併行")
        
        if not self._initialize_robot():
            return False
            
        if not self._initialize_modbus():
            return False
            
        self._initialize_motion_state_machine()
        self._initialize_external_modules()
        
        if not self._initialize_threads():
            return False
            
        self.running = True
        self._start_handshake_loop()
        
        self.logger.info("Dobot新架構混合交握控制器啟動成功 (DR專案)")
        return True
    
    def _initialize_robot(self) -> bool:
        """初始化機械臂連接"""
        try:
            robot_config = self.config["robot"]
            
            self.robot = RealRobotController(
                robot_config["ip"],
                robot_config["dashboard_port"],
                robot_config["move_port"]
            )
            
            if not self.robot.initialize():
                return False
                
            self.logger.info("機械臂控制器初始化成功")
            return True
            
        except Exception as e:
            self.logger.error(f"機械臂初始化失敗: {e}", exc_info=True)
            return False
    
    def _initialize_modbus(self) -> bool:
        """初始化Modbus連接 - 含連接測試"""
        try:
            modbus_config = self.config["modbus"]
            self.logger.info(f"連接Modbus服務器: {modbus_config['server_ip']}:{modbus_config['server_port']}")
            
            self.modbus_client = ModbusTcpClient(
                host=modbus_config["server_ip"],
                port=modbus_config["server_port"],
                timeout=modbus_config["timeout"]
            )
            
            if self.modbus_client.connect():
                self.logger.info("Modbus客戶端連接成功")
                
                # 測試運動類寄存器範圍讀寫
                self._test_motion_register_access()
                
                # 測試IO類寄存器範圍讀寫
                self._test_io_register_access()
                
                return True
            else:
                self.logger.error("Modbus客戶端連接失敗")
                return False
                
        except Exception as e:
            self.logger.error(f"Modbus初始化失敗: {e}", exc_info=True)
            return False
    
    def _test_motion_register_access(self):
        """測試運動類寄存器範圍 (1200-1249) 讀寫權限"""
        try:
            self.logger.info("驗證運動類寄存器範圍 (1200-1249) 讀寫權限...")
            
            # 測試讀取運動狀態寄存器範圍
            self.logger.debug(f"讀取運動狀態寄存器 {MotionRegisters.MOTION_STATUS}-{MotionRegisters.MOTION_STATUS+9}")
            read_result = self.modbus_client.read_holding_registers(address=MotionRegisters.MOTION_STATUS, count=10)
            
            if hasattr(read_result, 'isError') and read_result.isError():
                self.logger.error(f"讀取運動狀態寄存器失敗: {read_result}")
            else:
                self.logger.info(f"讀取運動狀態寄存器成功: {len(read_result.registers)}個寄存器")
                for i, value in enumerate(read_result.registers):
                    addr = MotionRegisters.MOTION_STATUS + i
                    self.logger.debug(f"寄存器{addr}: {value}")
            
            # 測試寫入運動狀態寄存器
            self.logger.debug(f"測試寫入運動狀態寄存器 {MotionRegisters.MOTION_STATUS}")
            test_value = 0x08  # Initialized位
            write_result = self.modbus_client.write_register(address=MotionRegisters.MOTION_STATUS, value=test_value)
            
            if hasattr(write_result, 'isError') and write_result.isError():
                self.logger.error(f"寫入運動狀態寄存器失敗: {write_result}")
            else:
                self.logger.info(f"寫入運動狀態寄存器成功: {MotionRegisters.MOTION_STATUS} = {test_value}")
                
                # 驗證寫入結果
                verify_result = self.modbus_client.read_holding_registers(address=MotionRegisters.MOTION_STATUS, count=1)
                if hasattr(verify_result, 'registers') and len(verify_result.registers) > 0:
                    actual_value = verify_result.registers[0]
                    self.logger.debug(f"驗證寫入結果: {MotionRegisters.MOTION_STATUS} = {actual_value}")
                    if actual_value == test_value:
                        self.logger.info("運動狀態寄存器寫入驗證成功")
                    else:
                        self.logger.warning(f"運動狀態寄存器寫入驗證失敗: 期望{test_value}，實際{actual_value}")
            
            # 測試讀取運動控制寄存器範圍
            self.logger.debug(f"讀取運動控制寄存器 {MotionRegisters.FLOW1_CONTROL}-{MotionRegisters.FLOW1_CONTROL+4}")
            control_result = self.modbus_client.read_holding_registers(address=MotionRegisters.FLOW1_CONTROL, count=5)
            
            if hasattr(control_result, 'isError') and control_result.isError():
                self.logger.error(f"讀取運動控制寄存器失敗: {control_result}")
            else:
                self.logger.info(f"讀取運動控制寄存器成功: {len(control_result.registers)}個寄存器")
                for i, value in enumerate(control_result.registers):
                    addr = MotionRegisters.FLOW1_CONTROL + i
                    self.logger.debug(f"寄存器{addr}: {value}")
                    
        except Exception as e:
            self.logger.error(f"運動類寄存器測試異常: {e}", exc_info=True)
    
    def _test_io_register_access(self):
        """測試IO類寄存器範圍 (447-449) 讀寫權限"""
        try:
            self.logger.info("驗證IO類寄存器範圍 (447-449) 讀寫權限...")
            
            # 測試讀取IO控制寄存器範圍 - DR專案只需要Flow4 (448)
            self.logger.debug(f"讀取IO控制寄存器 {IORegisters.FLOW4_CONTROL}")
            read_result = self.modbus_client.read_holding_registers(address=IORegisters.FLOW4_CONTROL, count=1)
            
            if hasattr(read_result, 'isError') and read_result.isError():
                self.logger.error(f"讀取IO控制寄存器失敗: {read_result}")
            else:
                self.logger.info(f"讀取IO控制寄存器成功: {len(read_result.registers)}個寄存器")
                for i, value in enumerate(read_result.registers):
                    addr = IORegisters.FLOW4_CONTROL + i
                    self.logger.debug(f"寄存器{addr}: {value}")
            
            # 測試寫入IO控制寄存器
            self.logger.debug(f"測試寫入IO控制寄存器 {IORegisters.FLOW4_CONTROL}")
            test_value = 0
            write_result = self.modbus_client.write_register(address=IORegisters.FLOW4_CONTROL, value=test_value)
            
            if hasattr(write_result, 'isError') and write_result.isError():
                self.logger.error(f"寫入IO控制寄存器失敗: {write_result}")
            else:
                self.logger.info(f"寫入IO控制寄存器成功: {IORegisters.FLOW4_CONTROL} = {test_value}")
                    
        except Exception as e:
            self.logger.error(f"IO類寄存器測試異常: {e}", exc_info=True)
    
    def _initialize_motion_state_machine(self):
        """初始化運動類狀態機 - 新地址版本"""
        self.logger.info("=== 初始化運動類狀態機 (DR專案) ===")
        self.logger.info(f"新架構地址範圍: {MotionRegisters.MOTION_STATUS}-{MotionRegisters.MOTION_STATUS+49}")
        self.logger.info(f"狀態寄存器: {MotionRegisters.MOTION_STATUS}-{MotionRegisters.MOTION_RUN_TIME}")
        self.logger.info(f"控制寄存器: {MotionRegisters.FLOW1_CONTROL}-{MotionRegisters.MOTION_EMERGENCY_STOP}")
        self.logger.info("解決地址衝突: 避開CCD2模組1000-1099範圍")
        
        self.motion_state_machine = MotionStateMachine(self.modbus_client)
        self.motion_state_machine.set_ready(True)
        self.logger.info("運動類狀態機初始化完成 - 新基地址1200 (DR專案)")
    
    def _initialize_external_modules(self):
        """初始化外部模組"""
        try:
            if self.config["vision"]["ccd1_enabled"]:
                try:
                    ccd1_api = CCD1HighLevelAPI(
                        modbus_host=self.config["modbus"]["server_ip"],
                        modbus_port=self.config["modbus"]["server_port"]
                    )
                    if ccd1_api.connected:
                        self.external_modules['ccd1'] = ccd1_api
                        self.logger.info("CCD1高階API連接成功")
                    else:
                        self.logger.warning("CCD1高階API連接失敗")
                except Exception as e:
                    self.logger.warning(f"CCD1高階API初始化失敗: {e}", exc_info=True)
            
            if self.config["gripper"]["enabled"]:
                try:
                    gripper_type = GripperType.PGE if self.config["gripper"]["type"] == "PGE" else GripperType.PGC
                    
                    gripper_api = GripperHighLevelAPI(
                        gripper_type=gripper_type,
                        modbus_host=self.config["modbus"]["server_ip"],
                        modbus_port=self.config["modbus"]["server_port"]
                    )
                    if gripper_api.connected:
                        self.external_modules['gripper'] = gripper_api
                        self.logger.info("夾爪高階API連接成功")
                    else:
                        self.logger.warning("夾爪高階API連接失敗")
                except Exception as e:
                    self.logger.warning(f"夾爪高階API初始化失敗: {e}", exc_info=True)
            
            try:
                angle_api = AngleHighLevel(
                    host=self.config["modbus"]["server_ip"],
                    port=self.config["modbus"]["server_port"]
                )
                if angle_api.connect():
                    self.external_modules['angle'] = angle_api
                    self.logger.info("角度校正API連接成功")
                else:
                    self.logger.warning("角度校正API連接失敗")
            except Exception as e:
                self.logger.warning(f"角度校正API初始化失敗: {e}", exc_info=True)
                
            self.logger.info("外部模組初始化完成")
            
        except Exception as e:
            self.logger.error(f"外部模組初始化異常: {e}", exc_info=True)
    
    def _initialize_threads(self) -> bool:
        """初始化執行緒"""
        try:
            # 運動控制執行緒 (運動類Flow: Flow1,2)
            self.motion_thread = MotionFlowThread(
                self.robot, self.motion_queue, self.motion_state_machine, self.external_modules
            )
            self.motion_thread.initialize_flows()
            
            # Flow4專用執行緒 (IO類)
            self.flow4_thread = Flow4VibrationFeedThread(self.robot, self.flow4_queue)
            self.flow4_thread.initialize_flows()
            
            # 外部模組執行緒
            self.external_thread = ExternalModuleThread(self.external_queue, self.external_modules)
            
            # 啟動所有執行緒
            self.motion_thread.start_thread()
            self.flow4_thread.start_thread()
            self.external_thread.start_thread()
            
            self.logger.info("執行緒初始化完成 - 新架構混合交握 (DR專案)")
            return True
            
        except Exception as e:
            self.logger.error(f"執行緒初始化失敗: {e}", exc_info=True)
            return False
    
    def _start_handshake_loop(self):
        """啟動握手循環"""
        self.handshake_thread = threading.Thread(target=self._handshake_loop, daemon=True, name="HandshakeLoop")
        self.handshake_thread.start()
        self.logger.info("新架構混合交握循環啟動 (DR專案)")
    
    def _handshake_loop(self):
        """新架構混合交握循環 - 輪詢速度優化版 (10ms) - 修正版：移除高頻logger創建"""
        # 關鍵修正：使用預先初始化的logger，避免高頻創建
        if ENABLE_HANDSHAKE_DEBUG:
            self.handshake_logger.info("新架構混合交握循環啟動 (DR專案)")
            self.handshake_logger.info("運動類寄存器: 1200-1249 (狀態機交握)")
            self.handshake_logger.info("IO類寄存器: 448 (專用佇列併行)")
            self.handshake_logger.info("循環間隔: 10ms (優化版)")
        
        loop_count = 0
        last_status_print = 0
        
        while self.running:
            try:
                loop_count += 1
                self.loop_count = loop_count
                current_time = time.time()
                
                # 每10秒打印一次系統狀態
                if current_time - last_status_print >= 10.0:
                    self._print_system_status(loop_count)
                    last_status_print = current_time
                
                # 處理運動類控制寄存器 (1240-1249)
                self._process_motion_control_registers()
                
                # 處理IO類控制寄存器 (448)
                self._process_io_control_registers()
                
                time.sleep(0.01)  # 10ms循環 (優化版)
                
            except Exception as e:
                # 關鍵修正：使用預先初始化的logger
                self.handshake_logger.error(f"混合交握循環錯誤: {e}", exc_info=True)
                time.sleep(1.0)
                
        if ENABLE_HANDSHAKE_DEBUG:
            self.handshake_logger.info("新架構混合交握循環結束")
    
    def _process_io_control_registers(self):
        """處理IO類控制寄存器 (448) - DR專案只有Flow4"""
        try:
            if ENABLE_HANDSHAKE_DEBUG:
                self.handshake_logger.debug(f"讀取IO控制寄存器 {IORegisters.FLOW4_CONTROL}")
            
            # 讀取IO控制寄存器 (448)
            result = self.modbus_client.read_holding_registers(address=IORegisters.FLOW4_CONTROL, count=1)
            
            if hasattr(result, 'isError') and result.isError():
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.error(f"讀取IO控制寄存器失敗: {result}")
                return
            
            if not hasattr(result, 'registers') or len(result.registers) < 1:
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.error(f"IO控制寄存器數據不足: {result}")
                return
                
            registers = result.registers
            flow4_control = registers[0]  # 448
            
            if ENABLE_HANDSHAKE_DEBUG:
                self.handshake_logger.debug("IO控制寄存器讀取成功:")
                self.handshake_logger.debug(f"  Flow4控制 (448): {flow4_control}")
                
            # 處理Flow4控制 (IO類震動投料)
            if flow4_control == 1 and self.last_flow4_control == 0:
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.debug(f"檢測到Flow4控制指令: {self.last_flow4_control} -> {flow4_control}")
                command = Command(
                    command_type=CommandType.DIO_VIBRATION,
                    command_data={'type': 'flow_vibration_feed'},
                    priority=CommandPriority.DIO_VIBRATION
                )
                if self.flow4_queue.put_command(command):
                    self.last_flow4_control = 1
                    if ENABLE_HANDSHAKE_DEBUG:
                        self.handshake_logger.info("Flow4指令已加入震動投料佇列")
                else:
                    if ENABLE_HANDSHAKE_DEBUG:
                        self.handshake_logger.error("Flow4指令加入震動投料佇列失敗")
                
            elif flow4_control == 0 and self.last_flow4_control == 1:
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.debug(f"Flow4控制指令已清零: {self.last_flow4_control} -> {flow4_control}")
                self.last_flow4_control = 0
                
        except Exception as e:
            self.logger.error(f"處理IO類控制寄存器失敗: {e}", exc_info=True)
    
    def _process_motion_control_registers(self):
        """處理運動類控制寄存器 (1240-1249) - 修正地址版本"""
        try:
            if ENABLE_HANDSHAKE_DEBUG:
                self.handshake_logger.debug(f"讀取運動控制寄存器 {MotionRegisters.FLOW1_CONTROL}-{MotionRegisters.FLOW1_CONTROL+4}")
            
            # 讀取運動控制寄存器 (1240-1244)
            result = self.modbus_client.read_holding_registers(address=MotionRegisters.FLOW1_CONTROL, count=5)
            
            if hasattr(result, 'isError') and result.isError():
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.error(f"讀取運動控制寄存器失敗: {result}")
                return
            
            if not hasattr(result, 'registers') or len(result.registers) < 5:
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.error(f"運動控制寄存器數據不足: {result}")
                return
                
            registers = result.registers
            
            flow1_control = registers[0]  # 1240
            flow2_control = registers[1]  # 1241
            flow5_control = registers[2]  # 1242
            motion_clear_alarm = registers[3]  # 1243
            motion_emergency_stop = registers[4]  # 1244
            
            if ENABLE_HANDSHAKE_DEBUG:
                self.handshake_logger.debug("運動控制寄存器讀取成功:")
                self.handshake_logger.debug(f"  Flow1控制 (1240): {flow1_control}")
                self.handshake_logger.debug(f"  Flow2控制 (1241): {flow2_control}")
                self.handshake_logger.debug(f"  Flow5控制 (1242): {flow5_control}")
                self.handshake_logger.debug(f"  清除警報 (1243): {motion_clear_alarm}")
                self.handshake_logger.debug(f"  緊急停止 (1244): {motion_emergency_stop}")
            
            # 處理Flow1控制 (運動類)
            if flow1_control == 1 and self.last_flow1_control == 0:
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.debug(f"檢測到Flow1控制指令: {self.last_flow1_control} -> {flow1_control}")
                if self.motion_state_machine.is_ready_for_command():
                    if ENABLE_HANDSHAKE_DEBUG:
                        self.handshake_logger.info("運動系統Ready，接受Flow1指令")
                    command = Command(
                        command_type=CommandType.MOTION,
                        command_data={'type': 'flow1_vp_vision_pick'},
                        priority=CommandPriority.MOTION
                    )
                    if self.motion_queue.put_command(command):
                        self.last_flow1_control = 1
                        if ENABLE_HANDSHAKE_DEBUG:
                            self.handshake_logger.info("Flow1指令已加入運動佇列")
                    else:
                        if ENABLE_HANDSHAKE_DEBUG:
                            self.handshake_logger.error("Flow1指令加入運動佇列失敗")
                else:
                    if ENABLE_HANDSHAKE_DEBUG:
                        self.handshake_logger.warning("運動系統非Ready狀態，拒絕Flow1指令")
                
            elif flow1_control == 0 and self.last_flow1_control == 1:
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.debug(f"Flow1控制指令已清零: {self.last_flow1_control} -> {flow1_control}")
                self.last_flow1_control = 0
                
            # 處理Flow2控制 (運動類)
            if flow2_control == 1 and self.last_flow2_control == 0:
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.debug(f"檢測到Flow2控制指令: {self.last_flow2_control} -> {flow2_control}")
                if self.motion_state_machine.is_ready_for_command():
                    if ENABLE_HANDSHAKE_DEBUG:
                        self.handshake_logger.info("運動系統Ready，接受Flow2指令")
                    command = Command(
                        command_type=CommandType.MOTION,
                        command_data={'type': 'flow2_unload'},
                        priority=CommandPriority.MOTION
                    )
                    if self.motion_queue.put_command(command):
                        self.last_flow2_control = 1
                        if ENABLE_HANDSHAKE_DEBUG:
                            self.handshake_logger.info("Flow2指令已加入運動佇列")
                    else:
                        if ENABLE_HANDSHAKE_DEBUG:
                            self.handshake_logger.error("Flow2指令加入運動佇列失敗")
                else:
                    if ENABLE_HANDSHAKE_DEBUG:
                        self.handshake_logger.warning("運動系統非Ready狀態，拒絕Flow2指令")
                
            elif flow2_control == 0 and self.last_flow2_control == 1:
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.debug(f"Flow2控制指令已清零: {self.last_flow2_control} -> {flow2_control}")
                self.last_flow2_control = 0
                
            # 處理Flow5控制 (運動類) - DR專案保留接口但不實現
            if flow5_control == 1 and self.last_flow5_control == 0:
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.debug(f"檢測到Flow5控制指令 (DR專案未實現): {self.last_flow5_control} -> {flow5_control}")
                self.handshake_logger.warning("Flow5在DR專案中未實現，忽略指令")
                self.last_flow5_control = 1
                
            elif flow5_control == 0 and self.last_flow5_control == 1:
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.debug(f"Flow5控制指令已清零: {self.last_flow5_control} -> {flow5_control}")
                self.last_flow5_control = 0
                
            # 處理運動清除警報
            if motion_clear_alarm == 1 and self.last_motion_clear_alarm == 0:
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.info(f"收到運動清除警報指令: {self.last_motion_clear_alarm} -> {motion_clear_alarm}")
                self.motion_state_machine.set_alarm(False)
                self.motion_state_machine.set_ready(True)
                self.last_motion_clear_alarm = 1
                
                # 自動清零警報控制寄存器
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.debug(f"自動清零警報控制寄存器 {MotionRegisters.MOTION_CLEAR_ALARM}")
                clear_result = self.modbus_client.write_register(address=MotionRegisters.MOTION_CLEAR_ALARM, value=0)
                if hasattr(clear_result, 'isError') and clear_result.isError():
                    if ENABLE_HANDSHAKE_DEBUG:
                        self.handshake_logger.error(f"清零警報控制寄存器失敗: {clear_result}")
                else:
                    if ENABLE_HANDSHAKE_DEBUG:
                        self.handshake_logger.debug("清零警報控制寄存器成功")
                
            elif motion_clear_alarm == 0 and self.last_motion_clear_alarm == 1:
                self.last_motion_clear_alarm = 0
                
            # 處理運動緊急停止
            if motion_emergency_stop == 1:
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.critical(f"收到運動緊急停止指令: {motion_emergency_stop}")
                if self.robot and self.robot.is_connected:
                    if ENABLE_HANDSHAKE_DEBUG:
                        self.handshake_logger.critical("執行機械臂緊急停止")
                    self.robot.emergency_stop()
                self.motion_state_machine.set_alarm(True)
                
                # 自動清零緊急停止寄存器
                if ENABLE_HANDSHAKE_DEBUG:
                    self.handshake_logger.debug(f"自動清零緊急停止寄存器 {MotionRegisters.MOTION_EMERGENCY_STOP}")
                stop_result = self.modbus_client.write_register(address=MotionRegisters.MOTION_EMERGENCY_STOP, value=0)
                if hasattr(stop_result, 'isError') and stop_result.isError():
                    if ENABLE_HANDSHAKE_DEBUG:
                        self.handshake_logger.error(f"清零緊急停止寄存器失敗: {stop_result}")
                else:
                    if ENABLE_HANDSHAKE_DEBUG:
                        self.handshake_logger.debug("清零緊急停止寄存器成功")
                
        except Exception as e:
            self.logger.error(f"處理運動類控制寄存器失敗: {e}", exc_info=True)
    
    def _print_system_status(self, loop_count: int):
        """打印系統狀態摘要 - 使用新地址範圍"""
        try:
            # 降低系統狀態日誌級別，避免過於頻繁的info級別輸出
            if loop_count % 100 == 0:  # 每1000次循環才打印一次詳細狀態
                self.logger.info(f"系統狀態 (循環計數: {loop_count}) - DR專案")
            else:
                self.logger.debug(f"系統狀態 (循環計數: {loop_count}) - DR專案")
            
            # 讀取並顯示運動狀態寄存器 (1200-1209)
            motion_status_result = self.modbus_client.read_holding_registers(address=MotionRegisters.MOTION_STATUS, count=10)
            if hasattr(motion_status_result, 'registers') and len(motion_status_result.registers) >= 10:
                registers = motion_status_result.registers
                status_reg = registers[0]
                current_flow = registers[1] 
                progress = registers[2]
                flow1_complete = registers[4] if len(registers) > 4 else 0
                flow2_complete = registers[5] if len(registers) > 5 else 0
                
                if loop_count % 100 == 0:
                    self.logger.info(f"運動狀態: {status_reg} ({status_reg:04b}) - 地址1200")
                    self.logger.info(f"當前Flow: {current_flow}, 進度: {progress}% - 地址1201-1202")
                    self.logger.info(f"Flow完成狀態: F1={flow1_complete}, F2={flow2_complete} - 地址1204-1205")
                else:
                    self.logger.debug(f"運動狀態: {status_reg} ({status_reg:04b}), 當前Flow: {current_flow}, 進度: {progress}%")
            else:
                self.logger.warning("無法讀取運動狀態寄存器(1200-1209)")
                
            # 顯示執行緒狀態
            if loop_count % 100 == 0:
                if self.motion_thread:
                    self.logger.info(f"Motion執行緒: {self.motion_thread.status}, 操作計數: {self.motion_thread.operation_count}")
                if self.flow4_thread:
                    self.logger.info(f"Flow4執行緒: {self.flow4_thread.status}, 操作計數: {self.flow4_thread.operation_count}")
                    
                # 顯示佇列狀態
                self.logger.info(f"佇列大小: Motion={self.motion_queue.size()}, Flow4={self.flow4_queue.size()}")
                self.logger.info(f"機械臂連接: {'連接' if self.robot and self.robot.is_connected else '斷開'}")
                self.logger.info(f"Modbus連接: {'連接' if self.modbus_client and self.modbus_client.connected else '斷開'}")
                self.logger.info(f"新架構地址: 狀態1200-1209, 控制1240-1249")
                self.logger.info(f"輪詢間隔: 10ms (優化版)")
                
                # 運行時間統計
                run_time = time.time() - self.start_time
                self.logger.info(f"系統運行時間: {run_time/3600:.1f}小時")
            
        except Exception as e:
            self.logger.error(f"打印系統狀態失敗: {e}", exc_info=True)
    
    def stop(self):
        """停止控制器"""
        self.logger.info("=== 停止Dobot新架構混合交握控制器 (DR專案) ===")
        
        self.running = False
        
        if self.motion_thread:
            self.motion_thread.stop_thread()
        if self.flow4_thread:
            self.flow4_thread.stop_thread()
        if self.external_thread:
            self.external_thread.stop_thread()
            
        if self.robot:
            self.robot.disconnect()
        if self.modbus_client:
            self.modbus_client.close()
            
        for name, module in self.external_modules.items():
            try:
                if hasattr(module, 'disconnect'):
                    module.disconnect()
                    self.logger.info(f"外部模組 {name} 已斷開")
            except Exception as e:
                self.logger.error(f"斷開{name}失敗: {e}", exc_info=True)
        
        self.logger.info("Dobot新架構混合交握控制器已停止 (DR專案)")
    
    def get_system_status(self) -> Dict[str, Any]:
        """取得系統狀態"""
        motion_status = "未知"
        if self.motion_state_machine:
            if self.motion_state_machine.status_register & 0x04:
                motion_status = "警報"
            elif self.motion_state_machine.status_register & 0x02:
                motion_status = "運行中"
            elif self.motion_state_machine.status_register & 0x01:
                motion_status = "準備就緒"
            else:
                motion_status = "空閒"
        
        return {
            'running': self.running,
            'motion_status': motion_status,
            'current_motion_flow': self.motion_state_machine.current_flow if self.motion_state_machine else 0,
            'motion_thread': self.motion_thread.get_status() if self.motion_thread else None,
            'flow4_thread': self.flow4_thread.get_status() if self.flow4_thread else None,
            'external_thread': self.external_thread.get_status() if self.external_thread else None,
            'robot_connected': self.robot.is_connected if self.robot else False,
            'modbus_connected': self.modbus_client.connected if self.modbus_client else False,
            'loop_count': self.loop_count,
            'run_time': time.time() - self.start_time
        }

# ==================== 主程序 ====================

def main():
    """主程序 - DR專案地址修正版本"""
    logger.info("="*80)
    logger.info("Dobot M1Pro 新架構混合交握控制器啟動 (DR專案)")
    logger.info("運動類Flow (Flow1,2): 基地址1200-1249，狀態機交握，序列化執行")
    logger.info("IO類Flow (Flow4): 地址448，專用佇列併行執行")
    logger.info("混合交握協議：確保運動安全性，提供IO操作並行能力")
    logger.info("地址衝突解決：原1100-1149 → 新1200-1249，避開CCD2模組")
    logger.info("輪詢速度優化：50ms → 10ms")
    logger.info("修正版：解決高頻logger創建導致的文件句柄洩漏問題")
    logger.info("="*80)
    
    controller = DobotNewArchController()
    
    try:
        if controller.start():
            logger.info("系統運行中，按 Ctrl+C 停止...")
            logger.info("")
            logger.info("寄存器地址映射 (DR專案修正版):")
            logger.info("運動類狀態機: 1200-1249")
            logger.info("  - 運動狀態: 1200 (bit0=Ready, bit1=Running, bit2=Alarm)")
            logger.info("  - 當前Flow: 1201 (1=Flow1, 2=Flow2)")
            logger.info("  - Flow控制: 1240(Flow1), 1241(Flow2)")
            logger.info("IO類併行控制: 448")
            logger.info("  - Flow4震動投料: 448")
            logger.info("")
            logger.info("DR專案特點:")
            logger.info("  - 支援AutoFeeding座標交握 (940-945)")
            logger.info("  - Flow1使用DR專項執行器")
            logger.info("  - Flow2使用DR專項執行器")
            logger.info("  - 不包含Flow3翻轉站")
            logger.info("  - 不包含Flow5機械臂運轉")
            logger.info("  - 輪詢間隔優化到10ms")
            logger.info("  - 修正高頻logger創建問題")
            
            while True:
                time.sleep(5)
                
                # 每5秒顯示系統狀態
                status = controller.get_system_status()
                logger.info(f"[{time.strftime('%H:%M:%S')}] 系統狀態 (DR專案-新地址1200):")
                logger.info(f"  運動系統: {status['motion_status']}")
                logger.info(f"  當前運動Flow: {status['current_motion_flow']}")
                logger.info(f"  Motion執行緒: {status['motion_thread']['status'] if status['motion_thread'] else 'None'}")
                logger.info(f"  Flow4執行緒: {status['flow4_thread']['status'] if status['flow4_thread'] else 'None'}")
                logger.info(f"  機械臂連接: {'連接' if status['robot_connected'] else '斷開'}")
                logger.info(f"  Modbus連接: {'連接' if status['modbus_connected'] else '斷開'}")
                logger.info(f"  系統運行時間: {status['run_time']/3600:.1f}小時")
                
        else:
            logger.error("控制器啟動失敗")
            
    except KeyboardInterrupt:
        logger.info("收到停止信號...")
    except Exception as e:
        logger.error(f"系統錯誤: {e}", exc_info=True)
    finally:
        controller.stop()
        logger.info("程序結束")

if __name__ == "__main__":
    main()