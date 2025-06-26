#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DR_Dobot_main.py - 機械臂主控制器 (新架構混合交握協議版)
實現混合交握協議：
- 運動類Flow (Flow1,Flow2,Flow5): 基地址1200-1249，狀態機交握
- IO類Flow (Flow3,Flow4): 地址447-449，專用佇列併行執行
確保運動安全性的同時提供IO操作並行能力
含詳細調試訊息用於排查1200之後地址讀寫問題
DR版本：支援Flow1、Flow2，Flow3暫未實作
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

# 導入高階API模組
from CCD1HighLevel import CCD1HighLevelAPI
from GripperHighLevel import GripperHighLevelAPI, GripperType
from AngleHighLevel import AngleHighLevel

from pymodbus.client.tcp import ModbusTcpClient
from dobot_api import DobotApiDashboard, DobotApiMove

# 配置常數
CONFIG_FILE = "dr_dobot_config.json"

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

# ==================== 流程執行結果 ====================

@dataclass
class FlowResult:
    """流程執行結果"""
    success: bool
    error_message: str = ""
    steps_completed: int = 0
    total_steps: int = 0
    execution_time: float = 0.0
    extra_data: Dict[str, Any] = field(default_factory=dict)

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
        
    def put_command(self, command: Command) -> bool:
        """加入指令到專用佇列"""
        try:
            with self._lock:
                command.command_id = self.command_id_counter
                self.command_id_counter += 1
                
            self.queue.put_nowait(command)
            self.put_count += 1
            
            print(f"[{self.name}Queue] 指令已加入 - ID:{command.command_id}, 類型:{command.command_type.value}, 佇列大小:{self.queue.qsize()}")
            return True
            
        except queue.Full:
            print(f"[{self.name}Queue] 佇列已滿，丟棄指令: {command.command_type}")
            return False
        except Exception as e:
            print(f"[{self.name}Queue] 加入指令失敗: {e}")
            return False
            
    def get_command(self, timeout: Optional[float] = None) -> Optional[Command]:
        """取得指令"""
        try:
            command = self.queue.get(timeout=timeout)
            self.get_count += 1
            
            if command:
                print(f"[{self.name}Queue] 指令已取出 - ID:{command.command_id}, 類型:{command.command_type.value}, 剩餘:{self.queue.qsize()}")
            
            return command
            
        except queue.Empty:
            return None
        except Exception as e:
            print(f"[{self.name}Queue] 取得指令失敗: {e}")
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
        
        print(f"✓ MotionStateMachine初始化完成 - 新基地址: {MotionRegisters.MOTION_STATUS}")
        
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
                    
            print(f"[MotionStateMachine] set_ready({ready}): {old_register:04b} -> {self.status_register:04b}")
            self._update_status_to_plc()
        except Exception as e:
            print(f"[MotionStateMachine] 設置運動Ready狀態失敗: {e}")
            
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
                    
            print(f"[MotionStateMachine] set_running({running}): {old_register:04b} -> {self.status_register:04b}")
            self._update_status_to_plc()
        except Exception as e:
            print(f"[MotionStateMachine] 設置運動Running狀態失敗: {e}")
            
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
                    
            print(f"[MotionStateMachine] set_alarm({alarm}): {old_register:04b} -> {self.status_register:04b}")
            self._update_status_to_plc()
        except Exception as e:
            print(f"[MotionStateMachine] 設置運動Alarm狀態失敗: {e}")
            
    def set_current_flow(self, flow_id: int):
        """設置當前流程ID - 使用新地址1201"""
        try:
            with self._lock:
                old_flow = self.current_flow
                self.current_flow = flow_id
            
            print(f"[MotionStateMachine] set_current_flow({flow_id}): {old_flow} -> {flow_id}")
            print(f"[MotionStateMachine] 寫入寄存器 {MotionRegisters.CURRENT_MOTION_FLOW} = {flow_id}")
            
            result = self.modbus_client.write_register(address=MotionRegisters.CURRENT_MOTION_FLOW, value=flow_id)
            if hasattr(result, 'isError') and result.isError():
                print(f"[MotionStateMachine] ✗ 寫入失敗: {result}")
            else:
                print(f"[MotionStateMachine] ✓ 寫入成功: 地址{MotionRegisters.CURRENT_MOTION_FLOW} = {flow_id}")
                
        except Exception as e:
            print(f"[MotionStateMachine] 設置運動流程ID失敗: {e}")
            
    def set_progress(self, progress: int):
        """設置進度 - 使用新地址1202"""
        try:
            with self._lock:
                old_progress = self.progress
                self.progress = max(0, min(100, progress))
            
            print(f"[MotionStateMachine] set_progress({progress}): {old_progress} -> {self.progress}")
            
            result = self.modbus_client.write_register(address=MotionRegisters.MOTION_PROGRESS, value=self.progress)
            if hasattr(result, 'isError') and result.isError():
                print(f"[MotionStateMachine] ✗ 寫入失敗: {result}")
            else:
                print(f"[MotionStateMachine] ✓ 寫入成功: 地址{MotionRegisters.MOTION_PROGRESS} = {self.progress}")
                
        except Exception as e:
            print(f"[MotionStateMachine] 設置運動進度失敗: {e}")
            
    def set_flow_complete(self, flow_id: int, complete: bool = True):
        """設置Flow完成狀態 - 使用新地址1204-1206"""
        try:
            value = 1 if complete else 0
            address = None
            
            if flow_id == 1:
                self.flow1_complete = value
                address = MotionRegisters.FLOW1_COMPLETE
            elif flow_id == 2:
                self.flow2_complete = value
                address = MotionRegisters.FLOW2_COMPLETE
            elif flow_id == 5:
                self.flow5_complete = value
                address = MotionRegisters.FLOW5_COMPLETE
            else:
                print(f"[MotionStateMachine] ✗ 未知Flow ID: {flow_id}")
                return
                
            print(f"[MotionStateMachine] set_flow_complete(Flow{flow_id}, {complete}): 值={value}")
            
            result = self.modbus_client.write_register(address=address, value=value)
            if hasattr(result, 'isError') and result.isError():
                print(f"[MotionStateMachine] ✗ 寫入失敗: {result}")
            else:
                print(f"[MotionStateMachine] ✓ 寫入成功: 地址{address} = {value}")
                
            if complete:
                self.operation_count += 1
                op_result = self.modbus_client.write_register(address=MotionRegisters.MOTION_OP_COUNT, value=self.operation_count)
                
        except Exception as e:
            print(f"[MotionStateMachine] 設置Flow{flow_id}完成狀態失敗: {e}")
            
    def is_ready_for_command(self) -> bool:
        """檢查是否可接受新的運動指令"""
        ready = (self.status_register & 0x01) != 0
        print(f"[MotionStateMachine] is_ready_for_command(): 狀態寄存器={self.status_register:04b}, Ready位={ready}")
        return ready
        
    def _update_status_to_plc(self):
        """更新狀態到PLC - 使用新地址1200"""
        try:
            print(f"[MotionStateMachine] 更新狀態到PLC:")
            print(f"[MotionStateMachine]   狀態寄存器: 地址{MotionRegisters.MOTION_STATUS} = {self.status_register} ({self.status_register:04b})")
            
            # 寫入狀態寄存器
            status_result = self.modbus_client.write_register(address=MotionRegisters.MOTION_STATUS, value=self.status_register)
            if hasattr(status_result, 'isError') and status_result.isError():
                print(f"[MotionStateMachine] ✗ 狀態寄存器寫入失敗: {status_result}")
            else:
                print(f"[MotionStateMachine] ✓ 狀態寄存器寫入成功")
                
            # 寫入錯誤計數
            err_result = self.modbus_client.write_register(address=MotionRegisters.MOTION_ERR_COUNT, value=self.error_count)
                
        except Exception as e:
            print(f"[MotionStateMachine] 更新運動狀態到PLC失敗: {e}")

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
    
    def initialize(self) -> bool:
        """初始化機械臂連接"""
        try:
            self.dashboard_api = DobotApiDashboard(self.ip, self.dashboard_port)
            self.move_api = DobotApiMove(self.ip, self.move_port)
            
            clear_result = self.dashboard_api.ClearError()
            if self._parse_api_response(clear_result):
                print("✓ 清除錯誤成功")
            else:
                print(f"清除錯誤失敗: {clear_result}")
                
            enable_result = self.dashboard_api.EnableRobot()
            if self._parse_api_response(enable_result):
                print("✓ 機械臂啟用成功")
            else:
                print(f"機械臂啟用失敗: {enable_result}")
                return False
            
            time.sleep(2.0)
            
            if self.set_global_speed(self.global_speed):
                print(f"✓ 初始速度設定成功: {self.global_speed}%")
            else:
                print(f"⚠️ 初始速度設定失敗")
            
            self.is_connected = True
            print(f"✓ 機械臂初始化成功: {self.ip}")
            return True
            
        except Exception as e:
            print(f"機械臂初始化失敗: {e}")
            return False
    
    def set_global_speed(self, speed_percent: int) -> bool:
        """設定全局速度"""
        try:
            if not 1 <= speed_percent <= 100:
                print(f"速度超出範圍: {speed_percent}")
                return False
                
            result = self.dashboard_api.SpeedFactor(speed_percent)
            success = self._parse_api_response(result)
            if success:
                self.global_speed = speed_percent
                print(f"✓ 全局速度設定成功: {speed_percent}%")
            else:
                print(f"全局速度設定失敗: {result}")
            return success
        except Exception as e:
            print(f"設定全局速度異常: {e}")
            return False
    
    def move_j(self, x: float, y: float, z: float, r: float) -> bool:
        """關節運動 - 修正版，加入Sync()調用"""
        try:
            print(f"開始MovJ: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
            
            result = self.move_api.MovJ(x, y, z, r)
            success = self._parse_api_response(result)
            
            if not success:
                print(f"✗ MovJ指令發送失敗: {result}")
                return False
            
            print(f"MovJ指令發送成功，調用Sync()執行...")
            
            sync_result = self.move_api.Sync()
            sync_success = self._parse_api_response(sync_result)
            
            if sync_success:
                print(f"✓ MovJ完成: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
                return True
            else:
                print(f"✗ MovJ同步執行失敗: {sync_result}")
                return False
                
        except Exception as e:
            print(f"MovJ執行異常: {e}")
            return False
    
    def move_l(self, x: float, y: float, z: float, r: float) -> bool:
        """直線運動 - 修正版，加入Sync()調用"""
        try:
            print(f"開始MovL: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
            
            result = self.move_api.MovL(x, y, z, r)
            success = self._parse_api_response(result)
            
            if not success:
                print(f"✗ MovL指令發送失敗: {result}")
                return False
            
            print(f"MovL指令發送成功，調用Sync()執行...")
            
            sync_result = self.move_api.Sync()
            sync_success = self._parse_api_response(sync_result)
            
            if sync_success:
                print(f"✓ MovL完成: ({x:.1f}, {y:.1f}, {z:.1f}, {r:.1f})")
                return True
            else:
                print(f"✗ MovL同步執行失敗: {sync_result}")
                return False
                
        except Exception as e:
            print(f"MovL執行異常: {e}")
            return False
    
    def set_do(self, pin: int, value: int) -> bool:
        """設定數位輸出"""
        try:
            result = self.dashboard_api.DOExecute(pin, value)
            success = self._parse_api_response(result)
            if success:
                print(f"✓ DO{pin}設定為{value}")
            else:
                print(f"✗ DO{pin}設定失敗: {result}")
            return success
        except Exception as e:
            print(f"設定DO失敗: {e}")
            return False
    
    def emergency_stop(self) -> bool:
        """緊急停止"""
        try:
            result = self.dashboard_api.EmergencyStop()
            success = self._parse_api_response(result)
            if success:
                print("✓ 緊急停止執行成功")
            else:
                print(f"緊急停止執行失敗: {result}")
            return success
        except Exception as e:
            print(f"緊急停止失敗: {e}")
            return False
    
    def disconnect(self) -> bool:
        """斷開機械臂連接"""
        try:
            if self.dashboard_api:
                disable_result = self.dashboard_api.DisableRobot()
                if self._parse_api_response(disable_result):
                    print("✓ 機械臂已停用")
                else:
                    print(f"⚠️ 機械臂停用失敗: {disable_result}")
                self.dashboard_api.close()
            if self.move_api:
                self.move_api.close()
            self.is_connected = False
            return True
        except Exception as e:
            print(f"機械臂斷開連接失敗: {e}")
            return False

# ==================== Flow執行器基類 ====================

class BaseFlowExecutor:
    """Flow執行器基類"""
    
    def __init__(self):
        self.robot = None
        self.motion_state_machine = None
        self.external_modules = {}
        
    def initialize(self, robot: RealRobotController, motion_state_machine: MotionStateMachine, external_modules: Dict):
        """初始化Flow執行器"""
        self.robot = robot
        self.motion_state_machine = motion_state_machine
        self.external_modules = external_modules
        
    def execute(self) -> FlowResult:
        """執行Flow - 子類需要重寫此方法"""
        raise NotImplementedError("子類必須實現execute方法")

# ==================== DR Flow執行器 ====================

class DrFlow1VisionPickExecutor(BaseFlowExecutor):
    """DR Flow1: VP視覺抓取執行器 - DR版本簡化版"""
    
    def execute(self) -> FlowResult:
        """執行DR Flow1 VP視覺抓取流程"""
        start_time = time.time()
        
        try:
            print("[DR Flow1] === 開始VP視覺抓取流程 ===")
            
            # 步驟1: 移動到安全位置
            print("[DR Flow1] 步驟1: 移動到安全位置")
            if not self.robot.move_j(300, 0, 200, 0):
                return FlowResult(success=False, error_message="移動到安全位置失敗", steps_completed=0, total_steps=5)
            
            # 步驟2: 移動到視覺檢測位置  
            print("[DR Flow1] 步驟2: 移動到視覺檢測位置")
            if not self.robot.move_j(250, 100, 150, 0):
                return FlowResult(success=False, error_message="移動到視覺檢測位置失敗", steps_completed=1, total_steps=5)
            
            # 步驟3: 視覺檢測 (如果CCD1可用)
            print("[DR Flow1] 步驟3: 執行視覺檢測")
            ccd1_api = self.external_modules.get('ccd1')
            if ccd1_api:
                # 使用真實CCD1檢測
                detection_result = ccd1_api.trigger_detection_and_wait(timeout=10.0)
                if not detection_result:
                    return FlowResult(success=False, error_message="CCD1視覺檢測失敗", steps_completed=2, total_steps=5)
                print("[DR Flow1] CCD1檢測成功")
            else:
                print("[DR Flow1] CCD1未連接，跳過視覺檢測")
            
            # 步驟4: 夾爪抓取 (如果夾爪可用)
            print("[DR Flow1] 步驟4: 夾爪抓取")
            gripper_api = self.external_modules.get('gripper')
            if gripper_api:
                if not gripper_api.grip_with_feedback(force=50, timeout=5.0):
                    return FlowResult(success=False, error_message="夾爪抓取失敗", steps_completed=3, total_steps=5)
                print("[DR Flow1] 夾爪抓取成功")
            else:
                print("[DR Flow1] 夾爪未連接，跳過抓取動作")
            
            # 步驟5: 移動到目標位置
            print("[DR Flow1] 步驟5: 移動到目標位置")
            if not self.robot.move_j(200, 200, 180, 0):
                return FlowResult(success=False, error_message="移動到目標位置失敗", steps_completed=4, total_steps=5)
            
            execution_time = time.time() - start_time
            print(f"[DR Flow1] === VP視覺抓取流程完成，耗時: {execution_time:.2f}秒 ===")
            
            return FlowResult(
                success=True,
                steps_completed=5,
                total_steps=5,
                execution_time=execution_time,
                extra_data={'detection_used': ccd1_api is not None, 'gripper_used': gripper_api is not None}
            )
            
        except Exception as e:
            execution_time = time.time() - start_time
            print(f"[DR Flow1] 執行異常: {e}")
            traceback.print_exc()
            return FlowResult(
                success=False,
                error_message=f"執行異常: {str(e)}",
                execution_time=execution_time
            )

class DrFlow2UnloadExecutor(BaseFlowExecutor):
    """DR Flow2: 出料流程執行器 - DR版本簡化版"""
    
    def execute(self) -> FlowResult:
        """執行DR Flow2 出料流程"""
        start_time = time.time()
        
        try:
            print("[DR Flow2] === 開始出料流程 ===")
            
            # 步驟1: 移動到出料位置
            print("[DR Flow2] 步驟1: 移動到出料位置")
            if not self.robot.move_j(150, 250, 200, 0):
                return FlowResult(success=False, error_message="移動到出料位置失敗", steps_completed=0, total_steps=3)
            
            # 步驟2: 夾爪放開 (如果夾爪可用)
            print("[DR Flow2] 步驟2: 夾爪放開")
            gripper_api = self.external_modules.get('gripper')
            if gripper_api:
                if not gripper_api.release_with_feedback(timeout=3.0):
                    return FlowResult(success=False, error_message="夾爪放開失敗", steps_completed=1, total_steps=3)
                print("[DR Flow2] 夾爪放開成功")
            else:
                print("[DR Flow2] 夾爪未連接，跳過放開動作")
            
            # 步驟3: 返回安全位置
            print("[DR Flow2] 步驟3: 返回安全位置")
            if not self.robot.move_j(300, 0, 200, 0):
                return FlowResult(success=False, error_message="返回安全位置失敗", steps_completed=2, total_steps=3)
            
            execution_time = time.time() - start_time
            print(f"[DR Flow2] === 出料流程完成，耗時: {execution_time:.2f}秒 ===")
            
            return FlowResult(
                success=True,
                steps_completed=3,
                total_steps=3,
                execution_time=execution_time,
                extra_data={'gripper_used': gripper_api is not None}
            )
            
        except Exception as e:
            execution_time = time.time() - start_time
            print(f"[DR Flow2] 執行異常: {e}")
            traceback.print_exc()
            return FlowResult(
                success=False,
                error_message=f"執行異常: {str(e)}",
                execution_time=execution_time
            )

# ==================== DR Flow4執行器 (震動投料) ====================

# 導入flow_base模組
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from flow_base import FlowExecutor, FlowResult, FlowStatus
except ImportError:
    # 如果無法導入，定義簡化版本
    class FlowStatus:
        IDLE = 0
        RUNNING = 1
        COMPLETED = 2
        ERROR = 3
        PAUSED = 4
    
    class FlowExecutor:
        def __init__(self, flow_id: int, flow_name: str):
            self.flow_id = flow_id
            self.flow_name = flow_name
            self.status = FlowStatus.IDLE
            self.current_step = 0
            self.total_steps = 0
            self.start_time = 0.0
            self.last_error = ""
            self.robot = None
            self.state_machine = None
            self.external_modules = {}
        
        def initialize(self, robot, state_machine, external_modules):
            self.robot = robot
            self.state_machine = state_machine  
            self.external_modules = external_modules

class DrFlow4VibrationFeedExecutor(FlowExecutor):
    """DR Flow4: 震動投料流程執行器 (DIO控制架構版)"""
    
    def __init__(self):
        super().__init__(flow_id=4, flow_name="震動投料流程")
        self.dio_steps = []
        
        # DIO腳位定義
        self.DIO_PINS = {
            'VIBRATION_CONTROL': 1,    # DO1: 震動控制 (HIGH-LOW脈衝)
            'FEED_ENABLE': 4,          # DO4: 投料使能 (持續HIGH)
        }
        
        # 時間延遲設定
        self.TIMING_CONFIG = {
            'FEED_DURATION': 0.3,      # DO4持續時間 (秒)
            'PULSE_HIGH_TIME': 0.3,    # DO1 HIGH持續時間 (秒)
            'PULSE_LOW_TIME': 0.3,     # DO1 LOW持續時間 (秒)
            'PULSE_COUNT': 1           # DO1脈衝次數
        }
        
        # 執行緒控制
        self.pulse_thread = None
        self.pulse_thread_running = False
        
        # 建構流程步驟
        self.build_flow_steps()
        
        print("✓ DrFlow4VibrationFeedExecutor初始化完成 (flow_base架構)")
    
    def build_flow_steps(self):
        """建構Flow4步驟"""
        self.dio_steps = [
            # 1. 同時啟動投料使能和震動脈衝
            {'type': 'start_vibration_feed', 'params': {}},
            
            # 2. 等待流程完成
            {'type': 'wait_completion', 'params': {'duration': self.TIMING_CONFIG['FEED_DURATION']}},
            
            # 3. 確保所有輸出關閉
            {'type': 'stop_all_outputs', 'params': {}}
        ]
        
        self.total_steps = len(self.dio_steps)
    
    def execute(self) -> FlowResult:
        """執行Flow4主邏輯"""
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        
        print(f"\n[DR Flow4] === 開始執行Flow4震動投料流程 ===")
        print(f"[DR Flow4] 總步驟數: {self.total_steps}")
        
        # 檢查初始化
        if not self.robot or not hasattr(self.robot, 'is_connected'):
            print(f"[DR Flow4] ✗ 機械臂未連接或未初始化")
            return FlowResult(
                success=False,
                error_message="機械臂未連接或未初始化",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        if not self.robot.is_connected:
            print(f"[DR Flow4] ✗ 機械臂未連接")
            return FlowResult(
                success=False,
                error_message="機械臂未連接",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        # 檢查dashboard_api連接
        if not hasattr(self.robot, 'dashboard_api') or self.robot.dashboard_api is None:
            print(f"[DR Flow4] ✗ dashboard_api未初始化")
            return FlowResult(
                success=False,
                error_message="dashboard_api未初始化",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        print(f"[DR Flow4] ✓ 機械臂連接檢查通過")
        print(f"[DR Flow4] ✓ dashboard_api連接檢查通過")
        
        try:
            for step in self.dio_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                print(f"\n[DR Flow4] 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = False
                
                if step['type'] == 'start_vibration_feed':
                    success = self._execute_start_vibration_feed()
                elif step['type'] == 'wait_completion':
                    success = self._execute_wait_completion(step['params'])
                elif step['type'] == 'stop_all_outputs':
                    success = self._execute_stop_all_outputs()
                else:
                    print(f"[DR Flow4] ✗ 未知步驟類型: {step['type']}")
                    success = False
                
                if not success:
                    self.status = FlowStatus.ERROR
                    print(f"[DR Flow4] ✗ 步驟 {self.current_step + 1}/{self.total_steps} 失敗")
                    return FlowResult(
                        success=False,
                        error_message=f"步驟 {step['type']} 執行失敗",
                        execution_time=time.time() - self.start_time,
                        steps_completed=self.current_step,
                        total_steps=self.total_steps
                    )
                
                print(f"[DR Flow4] ✓ 步驟 {self.current_step + 1}/{self.total_steps} 完成")
                self.current_step += 1
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            print(f"\n[DR Flow4] === Flow4震動投料流程執行完成 ===")
            print(f"[DR Flow4] 執行時間: {execution_time:.2f}秒")
            print(f"[DR Flow4] 完成步驟: {self.current_step}/{self.total_steps}")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                flow_data={'vibration_feed_completed': True}
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            print(f"[DR Flow4] ✗ Flow4執行異常: {str(e)}")
            return FlowResult(
                success=False,
                error_message=f"Flow4執行異常: {str(e)}",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
    
    def _execute_start_vibration_feed(self) -> bool:
        """執行震動投料啟動 - 並行控制DO1和DO4"""
        try:
            print(f"[DR Flow4]   震動投料流程啟動")
            print(f"[DR Flow4]   DO4將持續HIGH {self.TIMING_CONFIG['FEED_DURATION']}秒")
            print(f"[DR Flow4]   DO1將執行{self.TIMING_CONFIG['PULSE_COUNT']}次脈衝操作")
            
            # 啟動DO4 (投料使能) - 增加API調用打印
            print(f"[DR Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['FEED_ENABLE']}, 1)")
            try:
                if hasattr(self.robot, 'set_do'):
                    success = self.robot.set_do(self.DIO_PINS['FEED_ENABLE'], 1)
                    if not success:
                        print(f"[DR Flow4]     ✗ DO{self.DIO_PINS['FEED_ENABLE']}啟動失敗")
                        return False
                    print(f"[DR Flow4]     ✓ DO{self.DIO_PINS['FEED_ENABLE']} = 1 執行成功")
                else:
                    result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['FEED_ENABLE'], 1)
                    print(f"[DR Flow4]     API返回結果: {result}")
                    print(f"[DR Flow4]     ✓ DO{self.DIO_PINS['FEED_ENABLE']} = 1 執行成功")
            except Exception as e:
                print(f"[DR Flow4]     ✗ DO{self.DIO_PINS['FEED_ENABLE']}啟動失敗: {e}")
                return False
            
            print(f"[DR Flow4]   ✓ DO{self.DIO_PINS['FEED_ENABLE']}已啟動")
            
            # 創建執行緒執行DO1脈衝操作
            self.pulse_thread_running = True
            self.pulse_thread = threading.Thread(target=self._execute_do1_pulses, daemon=True)
            self.pulse_thread.start()
            
            print(f"[DR Flow4]   ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝執行緒已啟動")
            return True
            
        except Exception as e:
            print(f"[DR Flow4]   ✗ 震動投料啟動失敗: {e}")
            return False
    
    def _execute_do1_pulses(self):
        """執行DO1脈衝操作 - 在獨立執行緒中運行"""
        try:
            print(f"[DR Flow4]   DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝操作開始")
            
            for pulse_num in range(self.TIMING_CONFIG['PULSE_COUNT']):
                if not self.pulse_thread_running:
                    break
                    
                print(f"[DR Flow4]   DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝 {pulse_num + 1}/{self.TIMING_CONFIG['PULSE_COUNT']}")
                
                # DO1 HIGH - 增加API調用打印
                print(f"[DR Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['VIBRATION_CONTROL']}, 1)")
                try:
                    if hasattr(self.robot, 'set_do'):
                        success = self.robot.set_do(self.DIO_PINS['VIBRATION_CONTROL'], 1)
                        if success:
                            print(f"[DR Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']} = 1 執行成功")
                        else:
                            print(f"[DR Flow4]     ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝{pulse_num + 1} HIGH失敗")
                            continue
                    else:
                        result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['VIBRATION_CONTROL'], 1)
                        print(f"[DR Flow4]     API返回結果: {result}")
                        print(f"[DR Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']} = 1 執行成功")
                    
                    print(f"[DR Flow4]     DO{self.DIO_PINS['VIBRATION_CONTROL']} HIGH (持續{self.TIMING_CONFIG['PULSE_HIGH_TIME']}秒)")
                except Exception as e:
                    print(f"[DR Flow4]     ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝{pulse_num + 1} HIGH失敗: {e}")
                    continue
                
                time.sleep(self.TIMING_CONFIG['PULSE_HIGH_TIME'])
                
                # DO1 LOW - 增加API調用打印
                print(f"[DR Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['VIBRATION_CONTROL']}, 0)")
                try:
                    if hasattr(self.robot, 'set_do'):
                        success = self.robot.set_do(self.DIO_PINS['VIBRATION_CONTROL'], 0)
                        if success:
                            print(f"[DR Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']} = 0 執行成功")
                        else:
                            print(f"[DR Flow4]     ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝{pulse_num + 1} LOW失敗")
                            continue
                    else:
                        result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['VIBRATION_CONTROL'], 0)
                        print(f"[DR Flow4]     API返回結果: {result}")
                        print(f"[DR Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']} = 0 執行成功")
                    
                    print(f"[DR Flow4]     DO{self.DIO_PINS['VIBRATION_CONTROL']} LOW (持續{self.TIMING_CONFIG['PULSE_LOW_TIME']}秒)")
                except Exception as e:
                    print(f"[DR Flow4]     ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝{pulse_num + 1} LOW失敗: {e}")
                    continue
                
                time.sleep(self.TIMING_CONFIG['PULSE_LOW_TIME'])
                
                print(f"[DR Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝{pulse_num + 1}完成")
            
            print(f"[DR Flow4]   ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']}所有脈衝操作完成")
            
        except Exception as e:
            print(f"[DR Flow4]   ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝操作失敗: {e}")
        finally:
            self.pulse_thread_running = False
    
    def _execute_wait_completion(self, params: Dict[str, Any]) -> bool:
        """等待流程完成"""
        try:
            duration = params.get('duration', 0.3)
            print(f"[DR Flow4]   等待流程完成 ({duration}秒)")
            print(f"[DR Flow4]   此期間DO4保持HIGH，DO1執行脈衝操作")
            
            time.sleep(duration)
            
            # 等待脈衝執行緒結束
            if self.pulse_thread and self.pulse_thread.is_alive():
                print(f"[DR Flow4]   等待DO1脈衝執行緒完成...")
                self.pulse_thread_running = False
                self.pulse_thread.join(timeout=2.0)
                
                if self.pulse_thread.is_alive():
                    print(f"[DR Flow4]   ⚠️ DO1脈衝執行緒未正常結束")
                else:
                    print(f"[DR Flow4]   ✓ DO1脈衝執行緒已結束")
            
            print(f"[DR Flow4]   ✓ 流程完成等待結束")
            return True
            
        except Exception as e:
            print(f"[DR Flow4]   ✗ 等待流程完成失敗: {e}")
            return False
    
    def _execute_stop_all_outputs(self) -> bool:
        """確保所有輸出關閉"""
        try:
            print(f"[DR Flow4]   關閉所有DO輸出")
            success = True
            
            # 關閉DO4 (投料使能) - 增加API調用打印
            print(f"[DR Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['FEED_ENABLE']}, 0)")
            try:
                if hasattr(self.robot, 'set_do'):
                    if not self.robot.set_do(self.DIO_PINS['FEED_ENABLE'], 0):
                        print(f"[DR Flow4]     ✗ DO{self.DIO_PINS['FEED_ENABLE']}關閉失敗")
                        success = False
                    else:
                        print(f"[DR Flow4]     ✓ DO{self.DIO_PINS['FEED_ENABLE']} = 0 執行成功")
                else:
                    result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['FEED_ENABLE'], 0)
                    print(f"[DR Flow4]     API返回結果: {result}")
                    print(f"[DR Flow4]     ✓ DO{self.DIO_PINS['FEED_ENABLE']} = 0 執行成功")
            except Exception as e:
                print(f"[DR Flow4]     ✗ DO{self.DIO_PINS['FEED_ENABLE']}關閉失敗: {e}")
                success = False
            
            # 關閉DO1 (震動控制) - 增加API調用打印
            print(f"[DR Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['VIBRATION_CONTROL']}, 0)")
            try:
                if hasattr(self.robot, 'set_do'):
                    if not self.robot.set_do(self.DIO_PINS['VIBRATION_CONTROL'], 0):
                        print(f"[DR Flow4]     ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}關閉失敗")
                        success = False
                    else:
                        print(f"[DR Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']} = 0 執行成功")
                else:
                    result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['VIBRATION_CONTROL'], 0)
                    print(f"[DR Flow4]     API返回結果: {result}")
                    print(f"[DR Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']} = 0 執行成功")
            except Exception as e:
                print(f"[DR Flow4]     ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}關閉失敗: {e}")
                success = False
            
            # 確保脈衝執行緒停止
            if self.pulse_thread_running:
                self.pulse_thread_running = False
                if self.pulse_thread and self.pulse_thread.is_alive():
                    self.pulse_thread.join(timeout=1.0)
            
            if success:
                print(f"[DR Flow4]   ✓ 所有DO輸出已關閉")
            else:
                print(f"[DR Flow4]   ⚠️ 部分DO輸出關閉失敗")
                
            return success
            
        except Exception as e:
            print(f"[DR Flow4]   ✗ 關閉DO輸出失敗: {e}")
            return False
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        print("[DR Flow4] Flow4已暫停")
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("[DR Flow4] Flow4已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow4執行"""
        try:
            print(f"[DR Flow4] 正在停止Flow4執行...")
            self.status = FlowStatus.ERROR
            
            # 停止脈衝執行緒
            if self.pulse_thread_running:
                self.pulse_thread_running = False
                if self.pulse_thread and self.pulse_thread.is_alive():
                    self.pulse_thread.join(timeout=2.0)
            
            # 關閉所有輸出
            self._execute_stop_all_outputs()
            
            print(f"[DR Flow4] ✓ Flow4已停止")
            return True
            
        except Exception as e:
            print(f"[DR Flow4] ✗ Flow4停止過程出錯: {e}")
            return False
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
class EmptyFlowExecutor(BaseFlowExecutor):
    """空白Flow執行器 - 用於未實作的Flow"""
    
    def __init__(self, flow_name: str):
        super().__init__()
        self.flow_name = flow_name
    
    def execute(self) -> FlowResult:
        print(f"[{self.flow_name}] 此Flow尚未實作")
        return FlowResult(
            success=False,
            error_message=f"{self.flow_name}尚未實作",
            steps_completed=0,
            total_steps=0
        )

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
        
    def start_thread(self):
        self.running = True
        self.start()
        
    def stop_thread(self):
        self.running = False
        
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
            # DR Flow1: VP視覺抓取
            flow1 = DrFlow1VisionPickExecutor()
            flow1.initialize(self.robot, self.motion_state_machine, self.external_modules)
            self.flow_executors[1] = flow1
            
            # DR Flow2: 出料流程
            flow2 = DrFlow2UnloadExecutor()
            flow2.initialize(self.robot, self.motion_state_machine, self.external_modules)
            self.flow_executors[2] = flow2
            
            # Flow5: 暫未實作，使用空白執行器
            flow5 = EmptyFlowExecutor("Flow5機械臂運轉")
            flow5.initialize(self.robot, self.motion_state_machine, self.external_modules)
            self.flow_executors[5] = flow5
            
            print("✓ DR運動Flow執行器初始化完成 (Flow1, Flow2已實作, Flow5未實作)")
            
        except Exception as e:
            print(f"運動Flow執行器初始化失敗: {e}")
            self.last_error = str(e)
    
    def run(self):
        """運動控制執行緒主循環"""
        self.status = "運行中"
        print(f"[{self.name}] 執行緒啟動 - 處理運動類Flow")
        
        while self.running:
            try:
                command = self.command_queue.get_command(timeout=0.1)
                
                if command and command.command_type == CommandType.MOTION:
                    print(f"[Motion] 收到運動指令，ID: {command.command_id}")
                    self._handle_motion_command(command)
                    
            except Exception as e:
                self.last_error = f"運動控制執行緒錯誤: {e}"
                print(f"[Motion] {self.last_error}")
                traceback.print_exc()
                
        self.status = "已停止"
        print(f"[{self.name}] 執行緒結束")
    
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
                print(f"[Motion] 未知運動指令類型: {cmd_type}")
                
            self.operation_count += 1
            
        except Exception as e:
            self.last_error = f"處理運動指令失敗: {e}"
            print(f"[Motion] {self.last_error}")
    
    def _execute_flow1(self):
        """執行Flow1 - VP視覺抓取"""
        try:
            print("[Motion] 開始執行Flow1 - VP視覺抓取")
            self.motion_state_machine.set_running(True)
            self.motion_state_machine.set_current_flow(1)
            self.motion_state_machine.set_progress(0)
            
            flow1 = self.flow_executors.get(1)
            if flow1:
                result = flow1.execute()
                
                if result.success:
                    print("[Motion] ✓ Flow1執行成功")
                    self.motion_state_machine.set_flow_complete(1, True)
                    self.motion_state_machine.set_progress(100)
                    self.motion_state_machine.set_running(False)
                    self.motion_state_machine.set_current_flow(0)
                    self.motion_state_machine.set_ready(True)
                else:
                    print(f"[Motion] ✗ Flow1執行失敗: {result.error_message}")
                    self.motion_state_machine.set_alarm(True)
                    self.motion_state_machine.set_running(False)
                    self.motion_state_machine.set_current_flow(0)
            else:
                print("[Motion] ✗ Flow1執行器未初始化")
                self.motion_state_machine.set_alarm(True)
                
        except Exception as e:
            print(f"[Motion] Flow1執行異常: {e}")
            self.motion_state_machine.set_alarm(True)
            self.motion_state_machine.set_running(False)
            self.motion_state_machine.set_current_flow(0)
    
    def _execute_flow2(self):
        """執行Flow2 - 出料流程"""
        try:
            print("[Motion] 開始執行Flow2 - 出料流程")
            self.motion_state_machine.set_running(True)
            self.motion_state_machine.set_current_flow(2)
            self.motion_state_machine.set_progress(0)
            
            flow2 = self.flow_executors.get(2)
            if flow2:
                result = flow2.execute()
                
                if result.success:
                    print("[Motion] ✓ Flow2執行成功")
                    self.motion_state_machine.set_flow_complete(2, True)
                    self.motion_state_machine.set_progress(100)
                    self.motion_state_machine.set_running(False)
                    self.motion_state_machine.set_current_flow(0)
                    self.motion_state_machine.set_ready(True)
                else:
                    print(f"[Motion] ✗ Flow2執行失敗: {result.error_message}")
                    self.motion_state_machine.set_alarm(True)
                    self.motion_state_machine.set_running(False)
                    self.motion_state_machine.set_current_flow(0)
            else:
                print("[Motion] ✗ Flow2執行器未初始化")
                self.motion_state_machine.set_alarm(True)
                
        except Exception as e:
            print(f"[Motion] Flow2執行異常: {e}")
            self.motion_state_machine.set_alarm(True)
            self.motion_state_machine.set_running(False)
            self.motion_state_machine.set_current_flow(0)
    
    def _execute_flow5(self):
        """執行Flow5 - 機械臂運轉流程 (未實作)"""
        try:
            print("[Motion] 開始執行Flow5 - 機械臂運轉流程 (未實作)")
            self.motion_state_machine.set_running(True)
            self.motion_state_machine.set_current_flow(5)
            self.motion_state_machine.set_progress(0)
            
            flow5 = self.flow_executors.get(5)
            if flow5:
                result = flow5.execute()
                
                if result.success:
                    print("[Motion] ✓ Flow5執行成功")
                    self.motion_state_machine.set_flow_complete(5, True)
                    self.motion_state_machine.set_progress(100)
                    self.motion_state_machine.set_running(False)
                    self.motion_state_machine.set_current_flow(0)
                    self.motion_state_machine.set_ready(True)
                else:
                    print(f"[Motion] ✗ Flow5執行失敗: {result.error_message}")
                    self.motion_state_machine.set_alarm(True)
                    self.motion_state_machine.set_running(False)
                    self.motion_state_machine.set_current_flow(0)
            else:
                print("[Motion] ✗ Flow5執行器未初始化")
                self.motion_state_machine.set_alarm(True)
                
        except Exception as e:
            print(f"[Motion] Flow5執行異常: {e}")
            self.motion_state_machine.set_alarm(True)
            self.motion_state_machine.set_running(False)
            self.motion_state_machine.set_current_flow(0)

# ==================== IO類Flow執行緒 (Flow3未實作) ====================

class Flow3FlipStationThread(BaseFlowThread):
    """Flow3翻轉站控制專用執行緒 - DR版本未實作"""
    
    def __init__(self, robot: RealRobotController, command_queue: DedicatedCommandQueue):
        super().__init__("Flow3FlipStation", command_queue)
        self.robot = robot
        self.flow3_executor = None
        
    def initialize_flows(self):
        """初始化Flow3執行器 - DR版本未實作"""
        try:
            # DR版本：Flow3未實作
            self.flow3_executor = EmptyFlowExecutor("Flow3翻轉站")
            print("⚠️ DR版本Flow3翻轉站未實作")
        except Exception as e:
            print(f"Flow3執行器初始化失敗: {e}")
            self.last_error = str(e)
    
    def run(self):
        """Flow3執行緒主循環 - DR版本未實作處理"""
        self.status = "運行中"
        print(f"[{self.name}] 執行緒啟動，專用佇列接收DIO_FLIP指令 (DR版本未實作)")
        
        while self.running:
            try:
                command = self.command_queue.get_command(timeout=0.2)
                
                if command:
                    print(f"[Flow3] 收到指令 - ID:{command.command_id}, 類型:{command.command_type.value}")
                    
                    if command.command_type == CommandType.DIO_FLIP:
                        print(f"[Flow3] DR版本Flow3未實作，跳過執行")
                    else:
                        print(f"[Flow3] 收到非DIO_FLIP指令，忽略: {command.command_type}")
                        
            except Exception as e:
                self.last_error = f"Flow3執行緒錯誤: {e}"
                print(f"[Flow3] {self.last_error}")
                time.sleep(0.1)
                
        self.status = "已停止"
        print(f"[{self.name}] 執行緒結束")

class Flow4VibrationFeedThread(BaseFlowThread):
    """Flow4震動投料控制專用執行緒 - DR版本已實作"""
    
    def __init__(self, robot: RealRobotController, command_queue: DedicatedCommandQueue):
        super().__init__("Flow4VibrationFeed", command_queue)
        self.robot = robot
        self.flow4_executor = None
        
    def initialize_flows(self):
        """初始化Flow4執行器 - DR版本已實作"""
        try:
            flow4 = DrFlow4VibrationFeedExecutor()
            flow4.initialize(self.robot, None, {})
            self.flow4_executor = flow4
            print("✓ DR版本Flow4震動投料執行器初始化完成 (flow_base架構)")
        except Exception as e:
            print(f"Flow4執行器初始化失敗: {e}")
            self.last_error = str(e)
    
    def run(self):
        """Flow4執行緒主循環 - DR版本已實作處理"""
        self.status = "運行中"
        print(f"[{self.name}] 執行緒啟動，專用佇列接收DIO_VIBRATION指令 (DR版本已實作)")
        
        while self.running:
            try:
                command = self.command_queue.get_command(timeout=0.2)
                
                if command:
                    print(f"[Flow4] 收到指令 - ID:{command.command_id}, 類型:{command.command_type.value}")
                    
                    if command.command_type == CommandType.DIO_VIBRATION:
                        cmd_type = command.command_data.get('type', '')
                        if cmd_type == 'flow_vibration_feed':
                            print(f"[Flow4] 開始處理震動投料指令，ID: {command.command_id}")
                            self._execute_vibration_feed()
                        else:
                            print(f"[Flow4] 未知指令子類型: {cmd_type}")
                    else:
                        print(f"[Flow4] 收到非DIO_VIBRATION指令，忽略: {command.command_type}")
                        
            except Exception as e:
                self.last_error = f"Flow4執行緒錯誤: {e}"
                print(f"[Flow4] {self.last_error}")
                traceback.print_exc()
                time.sleep(0.1)
                
        self.status = "已停止"
        print(f"[{self.name}] 執行緒結束")
    
    def _execute_vibration_feed(self):
        """執行震動投料控制 - DR版本已實作"""
        try:
            print("[Flow4] === 開始執行震動投料控制 (DR版本已實作) ===")
            start_time = time.time()
            
            if not self.flow4_executor:
                print("[Flow4] ✗ Flow4執行器未初始化")
                return
            
            result = self.flow4_executor.execute()
            execution_time = time.time() - start_time
            
            if result.success:
                print(f"[Flow4] ✓ 震動投料控制執行成功，耗時: {execution_time:.2f}秒")
                print(f"[Flow4] 完成步驟: {result.steps_completed}/{result.total_steps}")
            else:
                print(f"[Flow4] ✗ 震動投料控制執行失敗: {result.error_message}")
                print(f"[Flow4] 完成步驟: {result.steps_completed}/{result.total_steps}")
                
            self.operation_count += 1
            print("[Flow4] === 震動投料控制執行完成 ===")
                
        except Exception as e:
            print(f"[Flow4] 震動投料控制執行異常: {e}")
            traceback.print_exc()

# ==================== 外部模組執行緒 ====================

class ExternalModuleThread(BaseFlowThread):
    """外部模組交握執行緒"""
    
    def __init__(self, command_queue: DedicatedCommandQueue, external_modules: Dict):
        super().__init__("ExternalModule", command_queue)
        self.external_modules = external_modules
        
    def run(self):
        """外部模組執行緒主循環"""
        self.status = "運行中"
        print(f"[{self.name}] 執行緒啟動")
        
        while self.running:
            try:
                command = self.command_queue.get_command(timeout=0.1)
                
                if command and command.command_type == CommandType.EXTERNAL:
                    self._handle_external_command(command)
                    
            except Exception as e:
                self.last_error = f"外部模組執行緒錯誤: {e}"
                print(self.last_error)
                
        self.status = "已停止"
        print(f"[{self.name}] 執行緒結束")
    
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
                    print(f"{module_name}.{operation} 執行成功")
                else:
                    print(f"{module_name}.{operation} 執行失敗")
                    
                self.operation_count += 1
            else:
                print(f"未知外部模組: {module_name}")
                
        except Exception as e:
            self.last_error = f"執行外部模組指令失敗: {e}"
            print(self.last_error)
            
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
            print(f"模組操作執行失敗: {e}")
            return False

# ==================== 主控制器 ====================

class DrDobotNewArchController:
    """DR Dobot新架構混合交握控制器"""
    
    def __init__(self, config_file: str = CONFIG_FILE):
        self.config_file = config_file
        self.config = self._load_config()
        
        # 專用指令佇列 - 每個執行緒一個佇列
        self.motion_queue = DedicatedCommandQueue("Motion")
        self.flow3_queue = DedicatedCommandQueue("Flow3")
        self.flow4_queue = DedicatedCommandQueue("Flow4")
        self.external_queue = DedicatedCommandQueue("External")
        
        # 核心組件
        self.robot = None
        self.modbus_client = None
        self.motion_state_machine = None
        
        # 執行緒
        self.motion_thread = None
        self.flow3_thread = None
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
        self.last_flow3_control = 0
        self.last_flow4_control = 0
        self.last_motion_clear_alarm = 0
        
    def _load_config(self) -> Dict[str, Any]:
        """載入配置"""
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), self.config_file)
        
        default_config = {
            "robot": {
                "ip": "192.168.1.6",
                "dashboard_port": 29999,
                "move_port": 30003,
                "default_speed": 40
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
                "flow3_enabled": False,  # DR版本未實作
                "flow4_enabled": True,   # DR版本已實作震動投料
                "flow5_enabled": False   # DR版本未實作
            }
        }
        
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    user_config = json.load(f)
                    self._deep_update(default_config, user_config)
            except Exception as e:
                print(f"載入配置檔案失敗，使用預設配置: {e}")
        else:
            try:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"創建預設配置檔案: {config_path}")
            except Exception as e:
                print(f"創建配置檔案失敗: {e}")
                
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
        print("=== 啟動DR Dobot新架構混合交握控制器 ===")
        print("運動類Flow (Flow1,2,5): 基地址1200-1249，狀態機交握")
        print("IO類Flow (Flow3,4): 地址447-449，專用佇列併行")
        print("DR版本：已實作Flow1、Flow2，Flow3/4/5未實作")
        
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
        
        print("✓ DR Dobot新架構混合交握控制器啟動成功")
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
                
            print("✓ 機械臂控制器初始化成功")
            return True
            
        except Exception as e:
            print(f"✗ 機械臂初始化失敗: {e}")
            return False
    
    def _initialize_modbus(self) -> bool:
        """初始化Modbus連接 - 含連接測試"""
        try:
            modbus_config = self.config["modbus"]
            print(f"[初始化] 連接Modbus服務器: {modbus_config['server_ip']}:{modbus_config['server_port']}")
            
            self.modbus_client = ModbusTcpClient(
                host=modbus_config["server_ip"],
                port=modbus_config["server_port"],
                timeout=modbus_config["timeout"]
            )
            
            if self.modbus_client.connect():
                print("✓ Modbus客戶端連接成功")
                
                # 測試運動類寄存器範圍讀寫
                self._test_motion_register_access()
                
                # 測試IO類寄存器範圍讀寫
                self._test_io_register_access()
                
                return True
            else:
                print("✗ Modbus客戶端連接失敗")
                return False
                
        except Exception as e:
            print(f"✗ Modbus初始化失敗: {e}")
            traceback.print_exc()
            return False
    
    def _test_motion_register_access(self):
        """測試運動類寄存器範圍 (1200-1249) 讀寫權限"""
        try:
            print("[測試] 驗證運動類寄存器範圍 (1200-1249) 讀寫權限...")
            
            # 測試讀取運動狀態寄存器範圍
            read_result = self.modbus_client.read_holding_registers(address=MotionRegisters.MOTION_STATUS, count=10)
            
            if hasattr(read_result, 'isError') and read_result.isError():
                print(f"[測試] ✗ 讀取運動狀態寄存器失敗: {read_result}")
            else:
                print(f"[測試] ✓ 讀取運動狀態寄存器成功: {len(read_result.registers)}個寄存器")
            
            # 測試寫入運動狀態寄存器
            test_value = 0x08  # Initialized位
            write_result = self.modbus_client.write_register(address=MotionRegisters.MOTION_STATUS, value=test_value)
            
            if hasattr(write_result, 'isError') and write_result.isError():
                print(f"[測試] ✗ 寫入運動狀態寄存器失敗: {write_result}")
            else:
                print(f"[測試] ✓ 寫入運動狀態寄存器成功: {MotionRegisters.MOTION_STATUS} = {test_value}")
                    
        except Exception as e:
            print(f"[測試] 運動類寄存器測試異常: {e}")
            traceback.print_exc()
    
    def _test_io_register_access(self):
        """測試IO類寄存器範圍 (447-449) 讀寫權限"""
        try:
            print("[測試] 驗證IO類寄存器範圍 (447-449) 讀寫權限...")
            
            # 測試讀取IO控制寄存器範圍
            read_result = self.modbus_client.read_holding_registers(address=IORegisters.FLOW3_CONTROL, count=2)
            
            if hasattr(read_result, 'isError') and read_result.isError():
                print(f"[測試] ✗ 讀取IO控制寄存器失敗: {read_result}")
            else:
                print(f"[測試] ✓ 讀取IO控制寄存器成功: {len(read_result.registers)}個寄存器")
                    
        except Exception as e:
            print(f"[測試] IO類寄存器測試異常: {e}")
            traceback.print_exc()
    
    def _initialize_motion_state_machine(self):
        """初始化運動類狀態機 - 新地址版本"""
        print(f"=== 初始化運動類狀態機 ===")
        print(f"新架構地址範圍: {MotionRegisters.MOTION_STATUS}-{MotionRegisters.MOTION_STATUS+49}")
        print(f"狀態寄存器: {MotionRegisters.MOTION_STATUS}-{MotionRegisters.MOTION_RUN_TIME}")
        print(f"控制寄存器: {MotionRegisters.FLOW1_CONTROL}-{MotionRegisters.MOTION_EMERGENCY_STOP}")
        print(f"解決地址衝突: 避開CCD2模組1000-1099範圍")
        
        self.motion_state_machine = MotionStateMachine(self.modbus_client)
        self.motion_state_machine.set_ready(True)
        print("✓ 運動類狀態機初始化完成 - 新基地址1200")
    
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
                        print("✓ CCD1高階API連接成功")
                    else:
                        print("⚠️ CCD1高階API連接失敗")
                except Exception as e:
                    print(f"⚠️ CCD1高階API初始化失敗: {e}")
            
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
                        print("✓ 夾爪高階API連接成功")
                    else:
                        print("⚠️ 夾爪高階API連接失敗")
                except Exception as e:
                    print(f"⚠️ 夾爪高階API初始化失敗: {e}")
            
            try:
                angle_api = AngleHighLevel(
                    host=self.config["modbus"]["server_ip"],
                    port=self.config["modbus"]["server_port"]
                )
                if angle_api.connect():
                    self.external_modules['angle'] = angle_api
                    print("✓ 角度校正API連接成功")
                else:
                    print("⚠️ 角度校正API連接失敗")
            except Exception as e:
                print(f"⚠️ 角度校正API初始化失敗: {e}")
                
            print("✓ 外部模組初始化完成")
            
        except Exception as e:
            print(f"外部模組初始化異常: {e}")
    
    def _initialize_threads(self) -> bool:
        """初始化執行緒"""
        try:
            # 運動控制執行緒 (運動類Flow: Flow1,2,5)
            self.motion_thread = MotionFlowThread(
                self.robot, self.motion_queue, self.motion_state_machine, self.external_modules
            )
            self.motion_thread.initialize_flows()
            
            # Flow3專用執行緒 (IO類 - DR版本未實作)
            self.flow3_thread = Flow3FlipStationThread(self.robot, self.flow3_queue)
            self.flow3_thread.initialize_flows()
            
            # Flow4專用執行緒 (IO類 - DR版本未實作)
            self.flow4_thread = Flow4VibrationFeedThread(self.robot, self.flow4_queue)
            self.flow4_thread.initialize_flows()
            
            # 外部模組執行緒
            self.external_thread = ExternalModuleThread(self.external_queue, self.external_modules)
            
            # 啟動所有執行緒
            self.motion_thread.start_thread()
            self.flow3_thread.start_thread()
            self.flow4_thread.start_thread()
            self.external_thread.start_thread()
            
            print("✓ 執行緒初始化完成 - DR新架構混合交握")
            return True
            
        except Exception as e:
            print(f"✗ 執行緒初始化失敗: {e}")
            traceback.print_exc()
            return False
    
    def _start_handshake_loop(self):
        """啟動握手循環"""
        self.handshake_thread = threading.Thread(target=self._handshake_loop, daemon=True)
        self.handshake_thread.start()
        print("✓ DR新架構混合交握循環啟動")
    
    def _handshake_loop(self):
        """新架構混合交握循環 - 可控制調試訊息"""
        if ENABLE_HANDSHAKE_DEBUG:
            print("[HandshakeLoop] DR新架構混合交握循環啟動")
            print("[HandshakeLoop] 運動類寄存器: 1200-1249 (狀態機交握)")
            print("[HandshakeLoop] IO類寄存器: 447-449 (專用佇列併行)")
            print("[HandshakeLoop] 循環間隔: 50ms")
        
        loop_count = 0
        last_status_print = 0
        
        while self.running:
            try:
                loop_count += 1
                current_time = time.time()
                
                # 每10秒打印一次系統狀態
                if current_time - last_status_print >= 10.0:
                    self._print_system_status(loop_count)
                    last_status_print = current_time
                
                # 處理運動類控制寄存器 (1240-1249)
                self._process_motion_control_registers()
                
                # 處理IO類控制寄存器 (447-449)
                self._process_io_control_registers()
                
                time.sleep(0.05)  # 50ms循環
                
            except Exception as e:
                print(f"[HandshakeLoop] 混合交握循環錯誤: {e}")
                traceback.print_exc()
                time.sleep(1.0)
                
        if ENABLE_HANDSHAKE_DEBUG:
            print("[HandshakeLoop] DR新架構混合交握循環結束")
    
    def _process_motion_control_registers(self):
        """處理運動類控制寄存器 (1240-1249) - 修正地址版本"""
        try:
            if ENABLE_HANDSHAKE_DEBUG:
                print(f"[HandshakeLoop] 讀取運動控制寄存器 {MotionRegisters.FLOW1_CONTROL}-{MotionRegisters.FLOW1_CONTROL+4}")
            
            # 讀取運動控制寄存器 (1240-1244)
            result = self.modbus_client.read_holding_registers(address=MotionRegisters.FLOW1_CONTROL, count=5)
            
            if hasattr(result, 'isError') and result.isError():
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] ✗ 讀取運動控制寄存器失敗: {result}")
                return
            
            if not hasattr(result, 'registers') or len(result.registers) < 5:
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] ✗ 運動控制寄存器數據不足: {result}")
                return
                
            registers = result.registers
            
            flow1_control = registers[0]  # 1240
            flow2_control = registers[1]  # 1241
            flow5_control = registers[2]  # 1242
            motion_clear_alarm = registers[3]  # 1243
            motion_emergency_stop = registers[4]  # 1244
            
            if ENABLE_HANDSHAKE_DEBUG:
                print(f"[HandshakeLoop] 運動控制寄存器讀取成功:")
                print(f"[HandshakeLoop]   Flow1控制 (1240): {flow1_control}")
                print(f"[HandshakeLoop]   Flow2控制 (1241): {flow2_control}")
                print(f"[HandshakeLoop]   Flow5控制 (1242): {flow5_control}")
                print(f"[HandshakeLoop]   清除警報 (1243): {motion_clear_alarm}")
                print(f"[HandshakeLoop]   緊急停止 (1244): {motion_emergency_stop}")
            
            # 處理Flow1控制 (運動類)
            if flow1_control == 1 and self.last_flow1_control == 0:
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] 檢測到Flow1控制指令: {self.last_flow1_control} -> {flow1_control}")
                if self.motion_state_machine.is_ready_for_command():
                    if ENABLE_HANDSHAKE_DEBUG:
                        print("[HandshakeLoop] 運動系統Ready，接受Flow1指令")
                    command = Command(
                        command_type=CommandType.MOTION,
                        command_data={'type': 'flow1_vp_vision_pick'},
                        priority=CommandPriority.MOTION
                    )
                    if self.motion_queue.put_command(command):
                        self.last_flow1_control = 1
                        if ENABLE_HANDSHAKE_DEBUG:
                            print("[HandshakeLoop] ✓ Flow1指令已加入運動佇列")
                    else:
                        if ENABLE_HANDSHAKE_DEBUG:
                            print("[HandshakeLoop] ✗ Flow1指令加入運動佇列失敗")
                else:
                    if ENABLE_HANDSHAKE_DEBUG:
                        print("[HandshakeLoop] ✗ 運動系統非Ready狀態，拒絕Flow1指令")
                
            elif flow1_control == 0 and self.last_flow1_control == 1:
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] Flow1控制指令已清零: {self.last_flow1_control} -> {flow1_control}")
                self.last_flow1_control = 0
                
            # 處理Flow2控制 (運動類)
            if flow2_control == 1 and self.last_flow2_control == 0:
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] 檢測到Flow2控制指令: {self.last_flow2_control} -> {flow2_control}")
                if self.motion_state_machine.is_ready_for_command():
                    if ENABLE_HANDSHAKE_DEBUG:
                        print("[HandshakeLoop] 運動系統Ready，接受Flow2指令")
                    command = Command(
                        command_type=CommandType.MOTION,
                        command_data={'type': 'flow2_unload'},
                        priority=CommandPriority.MOTION
                    )
                    if self.motion_queue.put_command(command):
                        self.last_flow2_control = 1
                        if ENABLE_HANDSHAKE_DEBUG:
                            print("[HandshakeLoop] ✓ Flow2指令已加入運動佇列")
                    else:
                        if ENABLE_HANDSHAKE_DEBUG:
                            print("[HandshakeLoop] ✗ Flow2指令加入運動佇列失敗")
                else:
                    if ENABLE_HANDSHAKE_DEBUG:
                        print("[HandshakeLoop] ✗ 運動系統非Ready狀態，拒絕Flow2指令")
                
            elif flow2_control == 0 and self.last_flow2_control == 1:
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] Flow2控制指令已清零: {self.last_flow2_control} -> {flow2_control}")
                self.last_flow2_control = 0
                
            # 處理Flow5控制 (運動類 - DR版本未實作)
            if flow5_control == 1 and self.last_flow5_control == 0:
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] 檢測到Flow5控制指令: {self.last_flow5_control} -> {flow5_control} (DR版本未實作)")
                if self.motion_state_machine.is_ready_for_command():
                    command = Command(
                        command_type=CommandType.MOTION,
                        command_data={'type': 'flow5_assembly'},
                        priority=CommandPriority.MOTION
                    )
                    if self.motion_queue.put_command(command):
                        self.last_flow5_control = 1
                        if ENABLE_HANDSHAKE_DEBUG:
                            print("[HandshakeLoop] ✓ Flow5指令已加入運動佇列 (未實作)")
                else:
                    if ENABLE_HANDSHAKE_DEBUG:
                        print("[HandshakeLoop] ✗ 運動系統非Ready狀態，拒絕Flow5指令")
                
            elif flow5_control == 0 and self.last_flow5_control == 1:
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] Flow5控制指令已清零: {self.last_flow5_control} -> {flow5_control}")
                self.last_flow5_control = 0
                
            # 處理運動清除警報
            if motion_clear_alarm == 1 and self.last_motion_clear_alarm == 0:
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] 收到運動清除警報指令: {self.last_motion_clear_alarm} -> {motion_clear_alarm}")
                self.motion_state_machine.set_alarm(False)
                self.motion_state_machine.set_ready(True)
                self.last_motion_clear_alarm = 1
                
                # 自動清零警報控制寄存器
                clear_result = self.modbus_client.write_register(address=MotionRegisters.MOTION_CLEAR_ALARM, value=0)
                
            elif motion_clear_alarm == 0 and self.last_motion_clear_alarm == 1:
                self.last_motion_clear_alarm = 0
                
            # 處理運動緊急停止
            if motion_emergency_stop == 1:
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] 收到運動緊急停止指令: {motion_emergency_stop}")
                if self.robot and self.robot.is_connected:
                    self.robot.emergency_stop()
                self.motion_state_machine.set_alarm(True)
                
                # 自動清零緊急停止寄存器
                stop_result = self.modbus_client.write_register(address=MotionRegisters.MOTION_EMERGENCY_STOP, value=0)
                
        except Exception as e:
            print(f"[HandshakeLoop] 處理運動類控制寄存器失敗: {e}")
            traceback.print_exc()
    
    def _process_io_control_registers(self):
        """處理IO類控制寄存器 (447-449) - DR版本未實作Flow3/4"""
        try:
            if ENABLE_HANDSHAKE_DEBUG:
                print(f"[HandshakeLoop] 讀取IO控制寄存器 {IORegisters.FLOW3_CONTROL}-{IORegisters.FLOW4_CONTROL}")
            
            # 讀取IO控制寄存器 (447-448)
            result = self.modbus_client.read_holding_registers(address=IORegisters.FLOW3_CONTROL, count=2)
            
            if hasattr(result, 'isError') and result.isError():
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] ✗ 讀取IO控制寄存器失敗: {result}")
                return
            
            if not hasattr(result, 'registers') or len(result.registers) < 2:
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] ✗ IO控制寄存器數據不足: {result}")
                return
                
            registers = result.registers
            
            flow3_control = registers[0]  # 447
            flow4_control = registers[1]  # 448
            
            if ENABLE_HANDSHAKE_DEBUG:
                print(f"[HandshakeLoop] IO控制寄存器讀取成功:")
                print(f"[HandshakeLoop]   Flow3控制 (447): {flow3_control} (DR版本未實作)")
                print(f"[HandshakeLoop]   Flow4控制 (448): {flow4_control} (DR版本未實作)")
            
            # 處理Flow3控制 (IO類翻轉站 - DR版本未實作)
            if flow3_control == 1 and self.last_flow3_control == 0:
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] 檢測到Flow3控制指令: {self.last_flow3_control} -> {flow3_control} (DR版本未實作)")
                command = Command(
                    command_type=CommandType.DIO_FLIP,
                    command_data={'type': 'flow_flip_station'},
                    priority=CommandPriority.DIO_FLIP
                )
                if self.flow3_queue.put_command(command):
                    self.last_flow3_control = 1
                    if ENABLE_HANDSHAKE_DEBUG:
                        print("[HandshakeLoop] ✓ Flow3指令已加入翻轉站佇列 (未實作)")
                
            elif flow3_control == 0 and self.last_flow3_control == 1:
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] Flow3控制指令已清零: {self.last_flow3_control} -> {flow3_control}")
                self.last_flow3_control = 0
                
            # 處理Flow4控制 (IO類震動投料 - DR版本未實作)
            if flow4_control == 1 and self.last_flow4_control == 0:
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] 檢測到Flow4控制指令: {self.last_flow4_control} -> {flow4_control} (DR版本未實作)")
                command = Command(
                    command_type=CommandType.DIO_VIBRATION,
                    command_data={'type': 'flow_vibration_feed'},
                    priority=CommandPriority.DIO_VIBRATION
                )
                if self.flow4_queue.put_command(command):
                    self.last_flow4_control = 1
                    if ENABLE_HANDSHAKE_DEBUG:
                        print("[HandshakeLoop] ✓ Flow4指令已加入震動投料佇列 (未實作)")
                
            elif flow4_control == 0 and self.last_flow4_control == 1:
                if ENABLE_HANDSHAKE_DEBUG:
                    print(f"[HandshakeLoop] Flow4控制指令已清零: {self.last_flow4_control} -> {flow4_control}")
                self.last_flow4_control = 0
                
        except Exception as e:
            print(f"[HandshakeLoop] 處理IO類控制寄存器失敗: {e}")
            traceback.print_exc()
    
    def _print_system_status(self, loop_count: int):
        """打印系統狀態摘要 - 使用新地址範圍"""
        try:
            print(f"\n[DR系統狀態] 循環計數: {loop_count}")
            
            # 讀取並顯示運動狀態寄存器 (1200-1209)
            motion_status_result = self.modbus_client.read_holding_registers(address=MotionRegisters.MOTION_STATUS, count=10)
            if hasattr(motion_status_result, 'registers') and len(motion_status_result.registers) >= 10:
                registers = motion_status_result.registers
                status_reg = registers[0]
                current_flow = registers[1] 
                progress = registers[2]
                flow1_complete = registers[4] if len(registers) > 4 else 0
                flow2_complete = registers[5] if len(registers) > 5 else 0
                flow5_complete = registers[6] if len(registers) > 6 else 0
                
                print(f"[DR系統狀態] 運動狀態: {status_reg} ({status_reg:04b}) - 地址1200")
                print(f"[DR系統狀態] 當前Flow: {current_flow}, 進度: {progress}% - 地址1201-1202")
                print(f"[DR系統狀態] Flow完成狀態: F1={flow1_complete}, F2={flow2_complete}, F5={flow5_complete} - 地址1204-1206")
            else:
                print(f"[DR系統狀態] ✗ 無法讀取運動狀態寄存器(1200-1209)")
                
            # 顯示執行緒狀態
            if self.motion_thread:
                print(f"[DR系統狀態] Motion執行緒: {self.motion_thread.status}, 操作計數: {self.motion_thread.operation_count}")
            if self.flow3_thread:
                print(f"[DR系統狀態] Flow3執行緒: {self.flow3_thread.status} (未實作)")
            if self.flow4_thread:
                print(f"[DR系統狀態] Flow4執行緒: {self.flow4_thread.status} (已實作)")
                
            # 顯示佇列狀態
            print(f"[DR系統狀態] 佇列大小: Motion={self.motion_queue.size()}, Flow3={self.flow3_queue.size()}, Flow4={self.flow4_queue.size()}")
            print(f"[DR系統狀態] 機械臂連接: {'✓' if self.robot and self.robot.is_connected else '✗'}")
            print(f"[DR系統狀態] Modbus連接: {'✓' if self.modbus_client and self.modbus_client.connected else '✗'}")
            print(f"[DR系統狀態] DR版本實作狀態: Flow1=✓, Flow2=✓, Flow3=✗, Flow4=✓, Flow5=✗")
            print("")
            
        except Exception as e:
            print(f"[DR系統狀態] 打印系統狀態失敗: {e}")
    
    def stop(self):
        """停止控制器"""
        print("\n=== 停止DR Dobot新架構混合交握控制器 ===")
        
        self.running = False
        
        if self.motion_thread:
            self.motion_thread.stop_thread()
        if self.flow3_thread:
            self.flow3_thread.stop_thread()
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
            except Exception as e:
                print(f"斷開{name}失敗: {e}")
        
        print("✓ DR Dobot新架構混合交握控制器已停止")
    
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
            'flow3_thread': self.flow3_thread.get_status() if self.flow3_thread else None,
            'flow4_thread': self.flow4_thread.get_status() if self.flow4_thread else None,
            'external_thread': self.external_thread.get_status() if self.external_thread else None,
            'robot_connected': self.robot.is_connected if self.robot else False,
            'modbus_connected': self.modbus_client.connected if self.modbus_client else False,
            'dr_implementation_status': {
                'flow1': True,
                'flow2': True,
                'flow3': False,
                'flow4': True,   # Flow4已實作
                'flow5': False
            }
        }

# ==================== 主程序 ====================

def main():
    """主程序 - DR版本地址修正版本"""
    print("="*80)
    print("DR Dobot M1Pro 新架構混合交握控制器啟動")
    print("運動類Flow (Flow1,2,5): 基地址1200-1249，狀態機交握，序列化執行")
    print("IO類Flow (Flow3,4): 地址447-449，專用佇列併行執行")
    print("混合交握協議：確保運動安全性，提供IO操作並行能力")
    print("地址衝突解決：原1100-1149 → 新1200-1249，避開CCD2模組")
    print("DR版本實作狀態：Flow1=✓, Flow2=✓, Flow3=✗, Flow4=✓, Flow5=✗")
    print("="*80)
    
    controller = DrDobotNewArchController()
    
    try:
        if controller.start():
            print("\nDR系統運行中，按 Ctrl+C 停止...")
            print("\n寄存器地址映射 (修正版):")
            print("運動類狀態機: 1200-1249")
            print("  - 運動狀態: 1200 (bit0=Ready, bit1=Running, bit2=Alarm)")
            print("  - 當前Flow: 1201 (1=Flow1, 2=Flow2, 5=Flow5)")
            print("  - Flow控制: 1240(Flow1), 1241(Flow2), 1242(Flow5)")
            print("IO類併行控制: 447-449 (保持不變)")
            print("  - Flow3翻轉站: 447 (DR版本未實作)")
            print("  - Flow4震動投料: 448 (DR版本未實作)")
            print("\nDR版本特點:")
            print("  - 已實作: Flow1 VP視覺抓取, Flow2 出料流程, Flow4 震動投料")
            print("  - 未實作: Flow3 翻轉站, Flow5 機械臂運轉")
            print("  - 支援CCD1視覺檢測和夾爪控制")
            print("  - 支援DIO控制震動投料功能")
            print("  - 使用新架構混合交握協議")
            
            while True:
                time.sleep(5)
                
                # 每5秒顯示系統狀態
                status = controller.get_system_status()
                print(f"\n[{time.strftime('%H:%M:%S')}] DR系統狀態 (新地址1200):")
                print(f"  運動系統: {status['motion_status']}")
                print(f"  當前運動Flow: {status['current_motion_flow']}")
                print(f"  Motion執行緒: {status['motion_thread']['status'] if status['motion_thread'] else 'None'}")
                print(f"  機械臂連接: {'✓' if status['robot_connected'] else '✗'}")
                print(f"  Modbus連接: {'✓' if status['modbus_connected'] else '✗'}")
                print(f"  DR實作狀態: F1=✓, F2=✓, F3=✗, F4=✓, F5=✗")
                
        else:
            print("DR控制器啟動失敗")
            
    except KeyboardInterrupt:
        print("\n\n收到停止信號...")
    except Exception as e:
        print(f"\nDR系統錯誤: {e}")
        traceback.print_exc()
    finally:
        controller.stop()
        print("DR程序結束")

if __name__ == "__main__":
    main()