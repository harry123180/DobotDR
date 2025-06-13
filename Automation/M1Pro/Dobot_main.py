#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_main.py - 機械臂主控制器 (修正版)
整合狀態機管理、外部模組通訊、運動控制等功能
基地址400，支援多流程執行與外部設備整合
"""

import json
import os
import time
import threading
import traceback
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from enum import Enum

# 導入流程模組
from Dobot_Flow1 import Flow1Executor

from pymodbus.client.tcp import ModbusTcpClient
from dobot_api import DobotApiDashboard, DobotApiMove


# 配置常數
DOBOT_BASE_ADDR = 400
CONFIG_FILE = "dobot_config.json"

# Modbus寄存器映射 (基地址400)
class DobotRegisters:
    # 控制寄存器
    CONTROL_CMD = DOBOT_BASE_ADDR + 0      # 400: 控制指令
    ROBOT_STATE = DOBOT_BASE_ADDR + 1      # 401: 機械臂狀態
    CURRENT_FLOW = DOBOT_BASE_ADDR + 2     # 402: 當前流程ID
    FLOW_PROGRESS = DOBOT_BASE_ADDR + 3    # 403: 流程執行進度
    ERROR_CODE = DOBOT_BASE_ADDR + 4       # 404: 錯誤代碼
    ROBOT_MODE = DOBOT_BASE_ADDR + 5       # 405: 機械臂模式
    
    # 位置資訊寄存器
    POS_X = DOBOT_BASE_ADDR + 6           # 406: 當前X座標
    POS_Y = DOBOT_BASE_ADDR + 7           # 407: 當前Y座標  
    POS_Z = DOBOT_BASE_ADDR + 8           # 408: 當前Z座標
    POS_R = DOBOT_BASE_ADDR + 9           # 409: 當前R座標
    
    # 關節角度寄存器
    JOINT_J1 = DOBOT_BASE_ADDR + 10       # 410: J1角度
    JOINT_J2 = DOBOT_BASE_ADDR + 11       # 411: J2角度
    JOINT_J3 = DOBOT_BASE_ADDR + 12       # 412: J3角度
    JOINT_J4 = DOBOT_BASE_ADDR + 13       # 413: J4角度
    
    # IO狀態寄存器
    DI_STATUS = DOBOT_BASE_ADDR + 14      # 414: 數位輸入狀態
    DO_STATUS = DOBOT_BASE_ADDR + 15      # 415: 數位輸出狀態
    
    # 統計寄存器
    OP_COUNTER = DOBOT_BASE_ADDR + 16     # 416: 操作計數器
    ERR_COUNTER = DOBOT_BASE_ADDR + 17    # 417: 錯誤計數器
    RUN_TIME = DOBOT_BASE_ADDR + 18       # 418: 運行時間(分鐘)
    RESERVED = DOBOT_BASE_ADDR + 19       # 419: 保留


# 外部模組寄存器地址
class ExternalModules:
    # CCD1視覺檢測模組
    CCD1_BASE = 200
    CCD1_CONTROL = 200         # 握手控制
    CCD1_STATUS = 201          # 狀態寄存器
    CCD1_COUNT = 240          # 檢測數量
    CCD1_WORLD_VALID = 256    # 世界座標有效標誌
    
    # VP震動盤模組
    VP_BASE = 300
    VP_STATUS = 300           # 模組狀態
    VP_COMMAND = 320          # 指令代碼
    VP_PARAM1 = 321           # 參數1
    VP_CMD_ID = 323           # 指令ID
    
    # CCD3角度檢測模組
    CCD3_BASE = 800
    CCD3_CONTROL = 800        # 握手控制
    CCD3_STATUS = 801         # 狀態寄存器
    CCD3_SUCCESS = 840        # 檢測成功標誌
    CCD3_ANGLE_H = 843        # 角度高位
    CCD3_ANGLE_L = 844        # 角度低位
    
    # PGC夾爪模組
    PGC_BASE = 500
    PGC_STATUS = 500          # 模組狀態
    PGC_POSITION = 505        # 當前位置
    PGC_COMMAND = 520         # 指令代碼
    PGC_PARAM1 = 521          # 位置參數
    PGC_CMD_ID = 523          # 指令ID


# 狀態枚舉
class RobotState(Enum):
    IDLE = 0          # 空閒狀態
    RUNNING = 1       # 運行狀態
    PAUSED = 2        # 暫停狀態
    ERROR = 3         # 錯誤狀態
    EMERGENCY = 4     # 緊急停止狀態


class FlowType(Enum):
    NONE = 0          # 無流程
    FLOW_1 = 1        # 流程1 - VP視覺抓取
    FLOW_2 = 2        # 流程2 - CCD3角度檢測
    FLOW_3 = 3        # 流程3 - 完整加工流程


class CommandType(Enum):
    CLEAR = 0         # 清空指令
    FLOW_1 = 1        # 執行流程1
    FLOW_2 = 2        # 執行流程2
    FLOW_3 = 3        # 執行流程3
    EMERGENCY_STOP = 99  # 緊急停止


@dataclass
class RobotPoint:
    """機械臂點位數據結構"""
    name: str
    x: float
    y: float
    z: float
    r: float
    j1: float
    j2: float
    j3: float
    j4: float


class PointsManager:
    """點位管理器"""
    
    def __init__(self, points_file: str = "saved_points/robot_points.json"):
        # 確保使用絕對路徑，相對於當前執行檔案的目錄
        if not os.path.isabs(points_file):
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.points_file = os.path.join(current_dir, points_file)
        else:
            self.points_file = points_file
        self.points: Dict[str, RobotPoint] = {}
        
    def load_points(self) -> bool:
        """載入點位數據"""
        try:
            print(f"嘗試載入點位檔案: {self.points_file}")
            
            if not os.path.exists(self.points_file):
                print(f"點位檔案不存在: {self.points_file}")
                # 列出當前目錄和saved_points目錄的內容以供調試
                current_dir = os.path.dirname(self.points_file)
                parent_dir = os.path.dirname(current_dir)
                print(f"當前目錄: {parent_dir}")
                if os.path.exists(parent_dir):
                    print(f"目錄內容: {os.listdir(parent_dir)}")
                if os.path.exists(current_dir):
                    print(f"saved_points目錄內容: {os.listdir(current_dir)}")
                return False
                
            with open(self.points_file, "r", encoding="utf-8") as f:
                points_list = json.load(f)
            
            self.points.clear()
            for point_data in points_list:
                point = RobotPoint(
                    name=point_data["name"],
                    x=point_data["pose"]["x"],
                    y=point_data["pose"]["y"],
                    z=point_data["pose"]["z"],
                    r=point_data["pose"]["r"],
                    j1=point_data["joint"]["j1"],
                    j2=point_data["joint"]["j2"],
                    j3=point_data["joint"]["j3"],
                    j4=point_data["joint"]["j4"]
                )
                self.points[point.name] = point
                
            print(f"載入點位數據成功，共{len(self.points)}個點位: {list(self.points.keys())}")
            return True
            
        except Exception as e:
            print(f"載入點位數據失敗: {e}")
            return False
    
    def get_point(self, name: str) -> Optional[RobotPoint]:
        """獲取指定點位"""
        return self.points.get(name)
    
    def list_points(self) -> List[str]:
        """列出所有點位名稱"""
        return list(self.points.keys())


class ExternalModuleController:
    """外部模組控制器基類"""
    
    def __init__(self, modbus_client: ModbusTcpClient, base_address: int):
        self.modbus_client = modbus_client
        self.base_address = base_address
        self.command_id_counter = 1
        
    def read_register(self, offset: int) -> Optional[int]:
        """讀取寄存器"""
        try:
            result = self.modbus_client.read_holding_registers(
                self.base_address + offset, count=1
            )
            if hasattr(result, 'registers') and len(result.registers) > 0:
                return result.registers[0]
            return None
        except Exception as e:
            print(f"讀取寄存器{self.base_address + offset}失敗: {e}")
            return None
    
    def write_register(self, offset: int, value: int) -> bool:
        """寫入寄存器"""
        try:
            result = self.modbus_client.write_register(
                self.base_address + offset, value
            )
            return not (hasattr(result, 'isError') and result.isError())
        except Exception as e:
            print(f"寫入寄存器{self.base_address + offset}失敗: {e}")
            return False
    
    def get_next_command_id(self) -> int:
        """獲取下一個指令ID"""
        self.command_id_counter += 1
        return self.command_id_counter


class PGCGripperController(ExternalModuleController):
    """PGC夾爪控制器"""
    
    def __init__(self, modbus_client: ModbusTcpClient):
        super().__init__(modbus_client, ExternalModules.PGC_BASE)
        
    def initialize(self) -> bool:
        """初始化夾爪"""
        return self.send_command(1, 0, timeout=10.0)  # 初始化指令
        
    def open_to_position(self, position: int, timeout: float = 15.0) -> bool:
        """打開到指定位置"""
        return self.send_command(3, position, timeout=timeout)
        
    def close_fast(self) -> bool:
        """快速關閉"""
        return self.send_command(8, 0, wait_completion=False)  # 快速關閉不等待完成
        
    def check_module_status(self) -> bool:
        """檢查夾爪模組狀態"""
        try:
            module_status = self.read_register(0)  # 500: 模組狀態
            connect_status = self.read_register(1)  # 501: 連接狀態
            
            if module_status == 1 and connect_status == 1:
                return True
            else:
                print(f"夾爪模組狀態異常: module={module_status}, connect={connect_status}")
                return False
                
        except Exception as e:
            print(f"檢查夾爪模組狀態失敗: {e}")
            return False
        
    def send_command(self, command: int, param1: int = 0, 
                    timeout: float = 5.0, wait_completion: bool = True) -> bool:
        """發送夾爪指令"""
        try:
            cmd_id = self.get_next_command_id()
            
            # 發送指令
            self.write_register(20, command)      # 520: 指令代碼
            self.write_register(21, param1)       # 521: 參數1
            self.write_register(23, cmd_id)       # 523: 指令ID
            
            if not wait_completion:
                return True
                
            # 等待指令完成
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.read_register(23) == 0:  # 指令ID清零表示完成
                    return True
                time.sleep(0.05)
                
            print(f"PGC夾爪指令{command}執行超時")
            return False
            
        except Exception as e:
            print(f"PGC夾爪指令{command}執行失敗: {e}")
            return False
    
    def get_current_position(self) -> int:
        """獲取當前位置"""
        return self.read_register(5) or 0  # 505: 當前位置
        
    def check_reached(self) -> bool:
        """檢查是否到達位置"""
        grip_status = self.read_register(4)  # 504: 夾持狀態
        return grip_status in [1, 2]  # 1=到達, 2=夾住


class CCD1VisionController(ExternalModuleController):
    """CCD1視覺檢測控制器"""
    
    def __init__(self, modbus_client: ModbusTcpClient):
        super().__init__(modbus_client, ExternalModules.CCD1_BASE)
        
    def initialize(self) -> bool:
        """初始化視覺系統"""
        return self.send_vision_command(32, timeout=10.0)  # 重新初始化
        
    def capture_and_detect(self, timeout: float = 10.0) -> bool:
        """拍照並檢測"""
        return self.send_vision_command(16, timeout=timeout)  # 拍照+檢測
        
    def send_vision_command(self, command: int, timeout: float = 5.0) -> bool:
        """發送視覺指令"""
        try:
            # 檢查Ready狀態
            status = self.read_register(1)  # 201: 狀態寄存器
            if not (status and (status & 0x01)):  # bit0=Ready
                print("CCD1視覺系統未準備好")
                return False
                
            # 發送指令
            self.write_register(0, command)  # 200: 控制指令
            
            # 等待執行完成
            start_time = time.time()
            while time.time() - start_time < timeout:
                status = self.read_register(1)  # 201: 狀態寄存器
                if status and not (status & 0x02):  # bit1=Running變為0
                    # 清除控制指令
                    self.write_register(0, 0)
                    return True
                time.sleep(0.1)
                
            print(f"CCD1視覺指令{command}執行超時")
            return False
            
        except Exception as e:
            print(f"CCD1視覺指令{command}執行失敗: {e}")
            return False
    
    def get_detection_count(self) -> int:
        """獲取檢測到的物體數量"""
        return self.read_register(40) or 0  # 240: 檢測數量
        
    def get_object_center_world(self, object_index: int) -> Optional[List[float]]:
        """獲取物體世界座標中心點"""
        try:
            # 檢查世界座標有效性
            world_valid = self.read_register(56)  # 256: 世界座標有效標誌
            if not world_valid:
                return None
                
            if object_index < 1 or object_index > 5:
                return None
                
            # 計算寄存器偏移 (每個物體占用4個寄存器，X高位/低位, Y高位/低位)
            base_offset = 57 + (object_index - 1) * 4  # 257開始
            
            # 讀取X座標 (高位/低位)
            x_high = self.read_register(base_offset) or 0
            x_low = self.read_register(base_offset + 1) or 0
            x_int = (x_high << 16) | x_low
            x_world = x_int / 100.0  # 恢復小數點
            
            # 讀取Y座標 (高位/低位)
            y_high = self.read_register(base_offset + 2) or 0
            y_low = self.read_register(base_offset + 3) or 0
            y_int = (y_high << 16) | y_low
            y_world = y_int / 100.0  # 恢復小數點
            
            return [x_world, y_world, 0.0]  # Z=0平面
            
        except Exception as e:
            print(f"獲取物體{object_index}世界座標失敗: {e}")
            return None
    
    def is_ready(self) -> bool:
        """檢查是否準備好"""
        status = self.read_register(1)  # 201: 狀態寄存器
        return bool(status and (status & 0x01))  # bit0=Ready


class CCD3AngleController(ExternalModuleController):
    """CCD3角度檢測控制器"""
    
    def __init__(self, modbus_client: ModbusTcpClient):
        super().__init__(modbus_client, ExternalModules.CCD3_BASE)
        
    def initialize(self) -> bool:
        """初始化角度檢測系統"""
        return self.send_angle_command(32, timeout=10.0)  # 重新初始化
        
    def detect_angle(self, timeout: float = 10.0) -> bool:
        """檢測角度"""
        return self.send_angle_command(16, timeout=timeout)  # 拍照+角度檢測
        
    def send_angle_command(self, command: int, timeout: float = 5.0) -> bool:
        """發送角度檢測指令"""
        try:
            # 檢查Ready狀態
            status = self.read_register(1)  # 801: 狀態寄存器
            if not (status and (status & 0x01)):  # bit0=Ready
                print("CCD3角度檢測系統未準備好")
                return False
                
            # 發送指令
            self.write_register(0, command)  # 800: 控制指令
            
            # 等待執行完成
            start_time = time.time()
            while time.time() - start_time < timeout:
                status = self.read_register(1)  # 801: 狀態寄存器
                if status and not (status & 0x02):  # bit1=Running變為0
                    # 清除控制指令
                    self.write_register(0, 0)
                    return True
                time.sleep(0.1)
                
            print(f"CCD3角度指令{command}執行超時")
            return False
            
        except Exception as e:
            print(f"CCD3角度指令{command}執行失敗: {e}")
            return False
    
    def get_detected_angle(self) -> Optional[float]:
        """獲取檢測到的角度"""
        try:
            # 檢查檢測成功標誌
            success = self.read_register(40)  # 840: 檢測成功標誌
            if not success:
                return None
                
            # 讀取角度 (高位/低位)
            angle_high = self.read_register(43) or 0  # 843: 角度高位
            angle_low = self.read_register(44) or 0   # 844: 角度低位
            
            angle_int = (angle_high << 16) | angle_low
            angle = angle_int / 100.0  # 恢復小數點
            
            return angle
            
        except Exception as e:
            print(f"獲取檢測角度失敗: {e}")
            return None
    
    def is_ready(self) -> bool:
        """檢查是否準備好"""
        status = self.read_register(1)  # 801: 狀態寄存器
        return bool(status and (status & 0x01))  # bit0=Ready


class DobotM1Pro:
    """Dobot M1Pro機械臂核心控制類"""
    
    def __init__(self, ip: str = "192.168.1.6"):
        self.ip = ip
        self.dashboard_api: Optional[DobotApiDashboard] = None
        self.move_api: Optional[DobotApiMove] = None
        # 修正：使用執行檔案目錄的相對路徑
        current_dir = os.path.dirname(os.path.abspath(__file__))
        points_file = os.path.join(current_dir, "saved_points", "robot_points.json")
        self.points_manager = PointsManager(points_file)
        self.is_connected = False
        self.global_speed = 50
        
    def initialize(self) -> bool:
        """初始化機械臂連接"""
        try:
            self.dashboard_api = DobotApiDashboard(self.ip, 29999)
            self.move_api = DobotApiMove(self.ip, 30003)
            
            # 機械臂初始化設置
            self.dashboard_api.ClearError()
            self.dashboard_api.EnableRobot()
            self.dashboard_api.SpeedFactor(self.global_speed)
            self.dashboard_api.SpeedJ(self.global_speed)
            self.dashboard_api.AccJ(self.global_speed)
            
            self.is_connected = True
            print(f"機械臂初始化成功: {self.ip}")
            return True
            
        except Exception as e:
            print(f"機械臂初始化失敗: {e}")
            return False
    
    def disconnect(self) -> bool:
        """斷開機械臂連接"""
        try:
            if self.dashboard_api:
                self.dashboard_api.DisableRobot()
                self.dashboard_api.close()
            if self.move_api:
                self.move_api.close()
            self.is_connected = False
            return True
        except Exception as e:
            print(f"機械臂斷開連接失敗: {e}")
            return False
    
    def emergency_stop(self) -> bool:
        """緊急停止"""
        try:
            if self.dashboard_api:
                self.dashboard_api.EmergencyStop()
            return True
        except Exception as e:
            print(f"緊急停止失敗: {e}")
            return False
    
    def clear_error(self) -> bool:
        """清除錯誤"""
        try:
            if self.dashboard_api:
                self.dashboard_api.ClearError()
            return True
        except Exception as e:
            print(f"清除錯誤失敗: {e}")
            return False
    
    def set_global_speed(self, speed: int) -> bool:
        """設置全局速度"""
        try:
            if self.dashboard_api:
                self.dashboard_api.SpeedFactor(speed)
                self.global_speed = speed
            return True
        except Exception as e:
            print(f"設置全局速度失敗: {e}")
            return False
    
    def MovJ(self, point_name: str, **kwargs) -> bool:
        """關節運動到指定點位"""
        point = self.points_manager.get_point(point_name)
        if not point:
            print(f"點位{point_name}不存在")
            return False
            
        try:
            if self.move_api:
                result = self.move_api.JointMovJ(point.j1, point.j2, point.j3, point.j4)
                return True
        except Exception as e:
            print(f"MovJ到{point_name}失敗: {e}")
            return False
    
    def MovL(self, point_name: str, **kwargs) -> bool:
        """直線運動到指定點位"""
        point = self.points_manager.get_point(point_name)
        if not point:
            print(f"點位{point_name}不存在")
            return False
            
        try:
            if self.move_api:
                result = self.move_api.MovL(point.x, point.y, point.z, point.r)
                return True
        except Exception as e:
            print(f"MovL到{point_name}失敗: {e}")
            return False
    
    def MovL_coord(self, x: float, y: float, z: float, r: float, **kwargs) -> bool:
        """直線運動到指定座標"""
        try:
            if self.move_api:
                result = self.move_api.MovL(x, y, z, r)
                return True
        except Exception as e:
            print(f"MovL到座標({x},{y},{z},{r})失敗: {e}")
            return False
    
    def sync(self) -> bool:
        """等待運動完成"""
        try:
            if self.move_api:
                self.move_api.Sync()
            return True
        except Exception as e:
            print(f"同步等待失敗: {e}")
            return False
    
    def get_robot_mode(self) -> int:
        """獲取機械臂模式"""
        try:
            if self.dashboard_api:
                result = self.dashboard_api.RobotMode()
                # 解析返回結果，提取模式數值
                if result and ',' in result:
                    parts = result.split(',')
                    if len(parts) > 1:
                        return int(parts[1])
            return -1
        except Exception as e:
            print(f"獲取機械臂模式失敗: {e}")
            return -1
    
    def get_current_pose(self) -> Dict[str, float]:
        """獲取當前位姿"""
        try:
            if self.dashboard_api:
                result = self.dashboard_api.GetPose()
                # 解析座標數據
                if result and ',' in result:
                    parts = result.split(',')
                    if len(parts) >= 5:
                        return {
                            'x': float(parts[1]),
                            'y': float(parts[2]),
                            'z': float(parts[3]),
                            'r': float(parts[4])
                        }
            return {'x': 0, 'y': 0, 'z': 0, 'r': 0}
        except Exception as e:
            print(f"獲取當前位姿失敗: {e}")
            return {'x': 0, 'y': 0, 'z': 0, 'r': 0}
    
    def get_current_joints(self) -> Dict[str, float]:
        """獲取當前關節角度"""
        try:
            if self.dashboard_api:
                result = self.dashboard_api.GetAngle()
                # 解析關節角度數據
                if result and ',' in result:
                    parts = result.split(',')
                    if len(parts) >= 5:
                        return {
                            'j1': float(parts[1]),
                            'j2': float(parts[2]),
                            'j3': float(parts[3]),
                            'j4': float(parts[4])
                        }
            return {'j1': 0, 'j2': 0, 'j3': 0, 'j4': 0}
        except Exception as e:
            print(f"獲取當前關節角度失敗: {e}")
            return {'j1': 0, 'j2': 0, 'j3': 0, 'j4': 0}
    
    def is_ready(self) -> bool:
        """檢查機械臂是否準備好"""
        if not self.is_connected:
            return False
        robot_mode = self.get_robot_mode()
        return robot_mode in [5, 9]  # 5=Idle, 9=Ready等可用狀態
    
    def is_running(self) -> bool:
        """檢查機械臂是否運行中"""
        robot_mode = self.get_robot_mode()
        return robot_mode in [7, 8]  # 運行狀態


class DobotStateMachine:
    """Dobot狀態機管理"""
    
    def __init__(self, modbus_client: ModbusTcpClient):
        self.modbus_client = modbus_client
        self.current_state = RobotState.IDLE
        self.current_flow = FlowType.NONE
        self.operation_count = 0
        self.error_count = 0
        self.start_time = time.time()
        
    def set_state(self, new_state: RobotState):
        """設置機械臂狀態"""
        self.current_state = new_state
        self.update_status_to_plc()
        
    def set_flow(self, flow_type: FlowType):
        """設置當前流程"""
        self.current_flow = flow_type
        self.update_status_to_plc()
        
    def update_status_to_plc(self):
        """更新狀態到PLC"""
        try:
            # 更新機械臂狀態
            self.modbus_client.write_register(
                DobotRegisters.ROBOT_STATE, self.current_state.value
            )
            
            # 更新當前流程
            self.modbus_client.write_register(
                DobotRegisters.CURRENT_FLOW, self.current_flow.value
            )
            
            # 更新統計資訊
            self.modbus_client.write_register(
                DobotRegisters.OP_COUNTER, self.operation_count
            )
            self.modbus_client.write_register(
                DobotRegisters.ERR_COUNTER, self.error_count
            )
            
            # 更新運行時間(分鐘)
            run_time_minutes = int((time.time() - self.start_time) / 60)
            self.modbus_client.write_register(
                DobotRegisters.RUN_TIME, run_time_minutes
            )
            
        except Exception as e:
            print(f"更新狀態到PLC失敗: {e}")
    
    def read_control_from_plc(self) -> Dict[str, Any]:
        """從PLC讀取控制指令"""
        try:
            result = self.modbus_client.read_holding_registers(
                DobotRegisters.CONTROL_CMD, count=1
            )
            if hasattr(result, 'registers') and len(result.registers) > 0:
                command = result.registers[0]
                return {'command': command}
            return {'command': 0}
        except Exception as e:
            print(f"讀取PLC控制指令失敗: {e}")
            return {'command': 0}
    
    def update_robot_info(self, robot: DobotM1Pro):
        """更新機械臂資訊到寄存器"""
        try:
            # 更新機械臂模式
            robot_mode = robot.get_robot_mode()
            self.modbus_client.write_register(DobotRegisters.ROBOT_MODE, robot_mode)
            
            # 更新位置資訊
            pose = robot.get_current_pose()
            self.modbus_client.write_register(DobotRegisters.POS_X, int(pose['x']))
            self.modbus_client.write_register(DobotRegisters.POS_Y, int(pose['y']))
            self.modbus_client.write_register(DobotRegisters.POS_Z, int(pose['z']))
            self.modbus_client.write_register(DobotRegisters.POS_R, int(pose['r']))
            
            # 更新關節角度
            joints = robot.get_current_joints()
            self.modbus_client.write_register(DobotRegisters.JOINT_J1, int(joints['j1'] * 100))
            self.modbus_client.write_register(DobotRegisters.JOINT_J2, int(joints['j2'] * 100))
            self.modbus_client.write_register(DobotRegisters.JOINT_J3, int(joints['j3'] * 100))
            self.modbus_client.write_register(DobotRegisters.JOINT_J4, int(joints['j4'] * 100))
            
        except Exception as e:
            print(f"更新機械臂資訊失敗: {e}")
    
    def is_ready_for_command(self) -> bool:
        """檢查是否準備好接受指令"""
        return self.current_state == RobotState.IDLE


class DobotMotionController:
    """Dobot運動控制主控制器 (修正版)"""
    
    def __init__(self, config_file: str = CONFIG_FILE):
        self.config_file = config_file
        self.config = self._load_config()
        
        # 核心組件
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.robot = DobotM1Pro(self.config["robot"]["ip"])
        self.state_machine: Optional[DobotStateMachine] = None
        
        # 外部模組控制器
        self.gripper: Optional[PGCGripperController] = None
        self.ccd1: Optional[CCD1VisionController] = None
        self.ccd3: Optional[CCD3AngleController] = None
        
        # 流程執行器 - 修正：使用統一的Any類型
        self.flows: Dict[int, Any] = {}
        self.current_flow: Optional[Any] = None
        
        # 運行狀態
        self.is_running = False
        self.handshake_thread: Optional[threading.Thread] = None
        
    def _load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), self.config_file)
        
        # 預設配置
        default_config = {
            "robot": {
                "ip": "192.168.1.6",
                "dashboard_port": 29999,
                "move_port": 30003,
                "default_speed": 50,
                "default_acceleration": 50,
                "enable_collision_detection": True,
                "collision_level": 3
            },
            "modbus": {
                "server_ip": "127.0.0.1",
                "server_port": 502,
                "robot_base_address": 400,
                "timeout": 3.0
            },
            "gripper": {
                "type": "PGC",
                "base_address": 520,
                "status_address": 500,
                "default_force": 50,
                "default_speed": 80
            },
            "vision": {
                "ccd1_base_address": 200,
                "ccd3_base_address": 800,
                "detection_timeout": 10.0
            },
            "flows": {
                "flow1_enabled": True,
                "flow2_enabled": False,  # 暫時關閉未實現的流程
                "flow3_enabled": False   # 暫時關閉未實現的流程
            },
            "safety": {
                "enable_emergency_stop": True,
                "max_error_count": 5,
                "auto_recovery": False
            }
        }
        
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    user_config = json.load(f)
                    # 合併配置
                    self._deep_update(default_config, user_config)
            except Exception as e:
                print(f"載入配置檔案失敗，使用預設配置: {e}")
        else:
            # 創建預設配置檔案
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
    
    def initialize_system(self) -> bool:
        """初始化系統"""
        print("=== 初始化Dobot運動控制系統 ===")
        
        # 1. 連接Modbus服務器
        if not self._connect_modbus():
            return False
        
        # 2. 初始化狀態機
        self.state_machine = DobotStateMachine(self.modbus_client)
        
        # 3. 初始化外部模組控制器
        self.gripper = PGCGripperController(self.modbus_client)
        self.ccd1 = CCD1VisionController(self.modbus_client)
        self.ccd3 = CCD3AngleController(self.modbus_client)
        
        # 4. 初始化流程執行器 - 修正：使用Flow1Executor
        if self.config["flows"]["flow1_enabled"]:
            self.flows[1] = Flow1Executor(
                robot=self.robot,
                gripper=self.gripper, 
                ccd1=self.ccd1,
                ccd3=self.ccd3,
                state_machine=self.state_machine
            )
            print("Flow1 執行器初始化完成")
        
        # 其他流程暫時註釋，等實現後再啟用
        # if self.config["flows"]["flow2_enabled"]:
        #     from Dobot_Flow2 import Flow2Executor
        #     self.flows[2] = Flow2Executor(...)
        
        # 5. 載入點位數據
        if not self.robot.points_manager.load_points():
            print("載入點位數據失敗，但繼續運行")
        
        print("系統初始化完成")
        return True
    
    def _connect_modbus(self) -> bool:
        """連接Modbus服務器"""
        try:
            self.modbus_client = ModbusTcpClient(
                self.config["modbus"]["server_ip"],
                port=self.config["modbus"]["server_port"]
            )
            
            if self.modbus_client.connect():
                print(f"Modbus服務器連接成功: {self.config['modbus']['server_ip']}:{self.config['modbus']['server_port']}")
                return True
            else:
                print("Modbus服務器連接失敗")
                return False
                
        except Exception as e:
            print(f"Modbus連接異常: {e}")
            return False
    
    def connect_all_devices(self) -> bool:
        """連接所有設備"""
        print("=== 連接所有設備 ===")
        
        # 1. 連接機械臂
        if not self.robot.initialize():
            print("機械臂連接失敗")
            return False
        
        # 2. 初始化夾爪
        if not self.gripper.initialize():
            print("夾爪初始化失敗")
            return False
        
        # 3. 初始化視覺系統
        if not self.ccd1.initialize():
            print("CCD1視覺系統初始化失敗，但繼續運行")
        
        if not self.ccd3.initialize():
            print("CCD3角度檢測系統初始化失敗，但繼續運行")
        
        print("設備連接完成")
        return True
    
    def start_handshake_sync(self):
        """啟動狀態機交握同步"""
        if self.is_running:
            return
            
        self.is_running = True
        self.handshake_thread = threading.Thread(
            target=self._handshake_loop,
            daemon=True
        )
        self.handshake_thread.start()
        print("狀態機交握同步啟動")
    
    def stop_handshake_sync(self):
        """停止狀態機交握同步"""
        self.is_running = False
        if self.handshake_thread and self.handshake_thread.is_alive():
            self.handshake_thread.join(timeout=2.0)
        print("狀態機交握同步停止")
    
    def _handshake_loop(self):
        """狀態機交握主循環"""
        print("狀態機交握循環開始")
        
        while self.is_running:
            try:
                # 讀取PLC控制指令
                control_data = self.state_machine.read_control_from_plc()
                command = control_data.get('command', 0)
                
                # 處理控制指令
                if command != 0:
                    self._handle_plc_command(command)
                
                # 更新機械臂資訊
                if self.robot.is_connected:
                    self.state_machine.update_robot_info(self.robot)
                
                # 更新狀態到PLC
                self.state_machine.update_status_to_plc()
                
                time.sleep(0.05)  # 50ms循環
                
            except KeyboardInterrupt:
                print("收到中斷信號，停止狀態機循環")
                break
            except Exception as e:
                print(f"狀態機循環錯誤: {e}")
                traceback.print_exc()
                time.sleep(1)
        
        print("狀態機交握循環結束")
    
    def _handle_plc_command(self, command: int):
        """處理PLC指令"""
        try:
            if command == CommandType.EMERGENCY_STOP.value:
                print("收到緊急停止指令")
                self.emergency_stop_all()
                
            elif command in [CommandType.FLOW_1.value, CommandType.FLOW_2.value, CommandType.FLOW_3.value]:
                if self.state_machine.is_ready_for_command():
                    print(f"收到流程{command}執行指令")
                    self.execute_flow(command)
                else:
                    print(f"系統忙碌，無法執行流程{command}")
                    
            elif command == CommandType.CLEAR.value:
                print("收到清空指令")
                self._clear_command()
                
        except Exception as e:
            print(f"處理PLC指令{command}失敗: {e}")
    
    def execute_flow(self, flow_id: int) -> bool:
        """執行指定流程"""
        if flow_id not in self.flows:
            print(f"流程{flow_id}未啟用或不存在")
            return False
        
        if self.current_flow and hasattr(self.current_flow, 'is_running') and self.current_flow.is_running:
            print("有流程正在執行中")
            return False
        
        try:
            # 設置狀態
            self.state_machine.set_state(RobotState.RUNNING)
            self.state_machine.set_flow(FlowType(flow_id))
            
            # 執行流程
            self.current_flow = self.flows[flow_id]
            
            # 在新線程中執行流程
            flow_thread = threading.Thread(
                target=self._execute_flow_thread,
                args=(self.current_flow,),
                daemon=True
            )
            flow_thread.start()
            
            return True
            
        except Exception as e:
            print(f"啟動流程{flow_id}失敗: {e}")
            self.state_machine.set_state(RobotState.ERROR)
            return False
    
    def _execute_flow_thread(self, flow_executor):
        """流程執行線程 - 修正版"""
        try:
            # 調用流程的execute方法，Flow1返回FlowResult對象
            result = flow_executor.execute()
            
            # 處理FlowResult對象
            if hasattr(result, 'success'):
                if result.success:
                    print(f"流程執行成功，耗時: {result.execution_time:.2f}秒")
                    print(f"完成步驟: {result.steps_completed}/{result.total_steps}")
                    self.state_machine.operation_count += 1
                    self.state_machine.set_state(RobotState.IDLE)
                else:
                    print(f"流程執行失敗: {result.error_message}")
                    print(f"失敗於步驟: {result.steps_completed}/{result.total_steps}")
                    self.state_machine.set_state(RobotState.ERROR)
            else:
                # 處理舊版本的bool返回值
                if result:
                    print("流程執行成功")
                    self.state_machine.operation_count += 1
                    self.state_machine.set_state(RobotState.IDLE)
                else:
                    print("流程執行失敗")
                    self.state_machine.set_state(RobotState.ERROR)
                
        except Exception as e:
            print(f"流程執行異常: {e}")
            traceback.print_exc()
            self.state_machine.set_state(RobotState.ERROR)
        finally:
            self.state_machine.set_flow(FlowType.NONE)
            self.current_flow = None
    
    def emergency_stop_all(self) -> bool:
        """緊急停止所有設備"""
        try:
            print("執行緊急停止...")
            
            # 停止機械臂
            if self.robot:
                self.robot.emergency_stop()
            
            # 停止夾爪
            if self.gripper:
                self.gripper.send_command(2, wait_completion=False)  # 停止指令
            
            # 停止當前流程
            if self.current_flow and hasattr(self.current_flow, 'stop'):
                self.current_flow.stop()
            
            # 設置緊急停止狀態
            self.state_machine.set_state(RobotState.EMERGENCY)
            
            print("緊急停止完成")
            return True
            
        except Exception as e:
            print(f"緊急停止失敗: {e}")
            return False
    
    def _clear_command(self):
        """清空指令"""
        try:
            # 清除控制指令寄存器
            self.modbus_client.write_register(DobotRegisters.CONTROL_CMD, 0)
            print("指令已清空")
        except Exception as e:
            print(f"清空指令失敗: {e}")
    
    def get_system_status(self) -> Dict[str, Any]:
        """獲取系統狀態"""
        return {
            "robot_connected": self.robot.is_connected,
            "robot_ready": self.robot.is_ready() if self.robot.is_connected else False,
            "current_state": self.state_machine.current_state.name,
            "current_flow": self.state_machine.current_flow.name,
            "operation_count": self.state_machine.operation_count,
            "error_count": self.state_machine.error_count,
            "is_running": self.is_running,
            "flows_enabled": list(self.flows.keys())
        }
    
    def cleanup(self):
        """清理資源"""
        print("清理系統資源...")
        
        # 停止狀態機交握
        self.stop_handshake_sync()
        
        # 斷開機械臂連接
        if self.robot:
            self.robot.disconnect()
        
        # 關閉Modbus連接
        if self.modbus_client:
            self.modbus_client.close()
        
        print("資源清理完成")


def main():
    """主函數"""
    controller = DobotMotionController()
    
    try:
        # 初始化系統
        if not controller.initialize_system():
            print("系統初始化失敗")
            return
        
        # 連接所有設備
        if not controller.connect_all_devices():
            print("設備連接失敗")
            return
        
        # 啟動狀態機交握
        controller.start_handshake_sync()
        
        print("\n=== Dobot運動控制系統啟動完成 ===")
        print(f"基地址: {DOBOT_BASE_ADDR}")
        print(f"控制指令地址: {DobotRegisters.CONTROL_CMD}")
        print(f"機械臂狀態地址: {DobotRegisters.ROBOT_STATE}")
        print("指令映射:")
        print("  0 = 清空指令")
        print("  1 = 執行流程1 (VP視覺抓取)")
        print("  2 = 執行流程2 (CCD3角度檢測) - 未實現")
        print("  3 = 執行流程3 (完整加工流程) - 未實現")
        print(" 99 = 緊急停止")
        print(f"\n啟用的流程: {list(controller.flows.keys())}")
        print("\n系統準備完成，等待PLC指令...")
        
        # 主循環
        while True:
            try:
                time.sleep(1)
                status = controller.get_system_status()
                if status["current_state"] != "IDLE":
                    print(f"系統狀態: {status['current_state']}, 當前流程: {status['current_flow']}")
                    
            except KeyboardInterrupt:
                print("\n收到中斷信號，準備退出...")
                break
                
    except Exception as e:
        print(f"系統運行異常: {e}")
        traceback.print_exc()
    finally:
        controller.cleanup()


if __name__ == "__main__":
    main()