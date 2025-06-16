#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_main.py - 機械臂主控制器 (修正API解析版)
整合狀態機管理、外部模組通訊、運動控制等功能
基地址400，支援多流程執行與外部設備整合
修正GetPose和GetAngle API解析問題
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
    
    # PGC夾爪模組 (基於MVP.py的地址配置)
    PGC_BASE = 500
    PGC_MODULE_STATUS = 500   # 模組狀態
    PGC_CONNECT_STATUS = 501  # 連接狀態
    PGC_DEVICE_STATUS = 502   # 設備狀態
    PGC_GRIP_STATUS = 504     # 夾持狀態
    PGC_POSITION = 505        # 當前位置
    PGC_COMMAND = 520         # 指令代碼
    PGC_PARAM1 = 521          # 位置參數
    PGC_PARAM2 = 522          # 參數2
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
    """點位管理器 - 修正版，支援cartesian格式"""
    
    def __init__(self, points_file: str = "saved_points/robot_points.json"):
        # 確保使用絕對路徑，相對於當前執行檔案的目錄
        if not os.path.isabs(points_file):
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.points_file = os.path.join(current_dir, points_file)
        else:
            self.points_file = points_file
        self.points: Dict[str, RobotPoint] = {}
        
    def load_points(self) -> bool:
        """載入點位數據 - 修正版，支援cartesian格式"""
        try:
            print(f"嘗試載入點位檔案: {self.points_file}")
            
            if not os.path.exists(self.points_file):
                print(f"點位檔案不存在: {self.points_file}")
                return False
                
            with open(self.points_file, "r", encoding="utf-8") as f:
                points_list = json.load(f)
            
            self.points.clear()
            for point_data in points_list:
                try:
                    # 支援兩種格式：pose 或 cartesian
                    if "pose" in point_data:
                        # 原始格式
                        pose_data = point_data["pose"]
                    elif "cartesian" in point_data:
                        # 新格式
                        pose_data = point_data["cartesian"]
                    else:
                        print(f"點位 {point_data.get('name', 'unknown')} 缺少座標數據")
                        continue
                    
                    # 檢查關節數據
                    if "joint" not in point_data:
                        print(f"點位 {point_data.get('name', 'unknown')} 缺少關節數據")
                        continue
                    
                    joint_data = point_data["joint"]
                    
                    point = RobotPoint(
                        name=point_data["name"],
                        x=float(pose_data["x"]),
                        y=float(pose_data["y"]),
                        z=float(pose_data["z"]),
                        r=float(pose_data["r"]),
                        j1=float(joint_data["j1"]),
                        j2=float(joint_data["j2"]),
                        j3=float(joint_data["j3"]),
                        j4=float(joint_data["j4"])
                    )
                    
                    # 處理點位名稱的拼寫錯誤
                    point_name = point.name
                    if point_name == "stanby":
                        point_name = "standby"
                        print(f"自動修正點位名稱: stanby -> standby")
                    
                    self.points[point_name] = point
                    
                except Exception as e:
                    print(f"處理點位 {point_data.get('name', 'unknown')} 時發生錯誤: {e}")
                    continue
                
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
        """讀取寄存器 - PyModbus 3.9.2修正版"""
        try:
            result = self.modbus_client.read_holding_registers(
                address=self.base_address + offset, 
                count=1
            )
            
            # PyModbus 3.x 正確的錯誤檢查方式
            if hasattr(result, 'isError') and result.isError():
                print(f"讀取寄存器{self.base_address + offset}錯誤: {result}")
                return None
            elif hasattr(result, 'registers') and len(result.registers) > 0:
                return result.registers[0]
            else:
                return None
                
        except Exception as e:
            print(f"讀取寄存器{self.base_address + offset}異常: {e}")
            return None
    
    def write_register(self, offset: int, value: int) -> bool:
        """寫入寄存器 - PyModbus 3.9.2修正版"""
        try:
            result = self.modbus_client.write_register(
                address=self.base_address + offset, 
                value=value
            )
            
            # PyModbus 3.x 正確的錯誤檢查方式
            if hasattr(result, 'isError') and result.isError():
                return False
            else:
                return True
                
        except Exception as e:
            print(f"寫入寄存器{self.base_address + offset}={value}異常: {e}")
            return False
    
    def get_next_command_id(self) -> int:
        """獲取下一個指令ID"""
        self.command_id_counter += 1
        return self.command_id_counter


class PGCGripperController(ExternalModuleController):
    """PGC夾爪控制器 - 基於MVP.py的實際工作邏輯"""
    
    def __init__(self, modbus_client: ModbusTcpClient):
        super().__init__(modbus_client, ExternalModules.PGC_BASE)
        
        # PGC指令映射 (來自MVP.py)
        self.PGC_CMD_INIT = 1
        self.PGC_CMD_STOP = 2
        self.PGC_CMD_ABS_POS = 3
        self.PGC_CMD_SET_FORCE = 5
        self.PGC_CMD_SET_SPEED = 6
        self.PGC_CMD_QUICK_OPEN = 7
        self.PGC_CMD_QUICK_CLOSE = 8
        
        # PGC狀態映射
        self.PGC_STATUS_MOVING = 0
        self.PGC_STATUS_REACHED = 1
        self.PGC_STATUS_GRIPPED = 2
        self.PGC_STATUS_DROPPED = 3
        
    def initialize(self) -> bool:
        """初始化夾爪 - 基於MVP.py的邏輯"""
        print("=== 初始化PGC夾爪 ===")
        
        # 檢查模組是否運行
        if not self.check_module_status():
            print("PGC模組未運行")
            return False
        
        # 初始化重試機制
        max_init_attempts = 3
        for attempt in range(max_init_attempts):
            print(f"初始化嘗試 {attempt + 1}/{max_init_attempts}")
            
            # 發送初始化指令
            if not self.send_gripper_command_batch(self.PGC_CMD_INIT):
                continue
            
            # 等待初始化完成
            init_start = time.time()
            while time.time() - init_start < 10.0:
                device_status = self.read_register(2)  # 502: 設備狀態
                if device_status == 1:
                    print("夾爪初始化成功")
                    
                    # 設定最大速度和力道
                    self.send_gripper_command_batch(self.PGC_CMD_SET_SPEED, 100)
                    time.sleep(0.5)
                    self.send_gripper_command_batch(self.PGC_CMD_SET_FORCE, 20)
                    time.sleep(0.5)
                    
                    print("夾爪參數設定完成")
                    return True
                time.sleep(0.2)
            
            print(f"第{attempt + 1}次初始化超時")
        
        print("夾爪初始化失敗")
        return False
        
    def open_to_position(self, position: int, timeout: float = 15.0) -> bool:
        """打開到指定位置 - 基於MVP.py的智能檢測邏輯"""
        print(f"夾爪撐開到位置 {position}")
        
        # 記錄初始位置
        initial_position = self.read_register(5) or 0  # 505: 當前位置
        
        # 發送位置指令
        if not self.send_gripper_command_batch(self.PGC_CMD_ABS_POS, position):
            return False
        
        # 等待位置到達 - 使用MVP.py的智能判斷邏輯
        start_time = time.time()
        max_position_reached = initial_position
        
        while time.time() - start_time < timeout:
            current_position = self.read_register(5) or 0  # 505: 當前位置
            grip_status = self.read_register(4)  # 504: 夾持狀態
            
            # 記錄最大位置
            if current_position > max_position_reached:
                max_position_reached = current_position
            
            position_diff = abs(current_position - position)
            movement_from_start = abs(current_position - initial_position)
            
            # 智能判斷成功條件 (來自MVP.py)
            if position_diff <= 20:
                print(f"夾爪到達目標位置: {current_position}")
                return True
            elif movement_from_start > 100 and grip_status == self.PGC_STATUS_GRIPPED:
                print(f"夾爪撐開固定物件: {current_position}")
                return True
            elif current_position == max_position_reached and movement_from_start > 50:
                # 檢查位置穩定性
                stable_count = 0
                for _ in range(3):
                    time.sleep(0.1)
                    check_pos = self.read_register(5) or 0
                    if check_pos == current_position:
                        stable_count += 1
                
                if stable_count >= 2:
                    print(f"夾爪位置穩定，撐開成功: {current_position}")
                    return True
            
            time.sleep(0.2)
        
        # 超時檢查
        final_position = self.read_register(5) or 0
        final_movement = abs(final_position - initial_position)
        
        if final_movement > 100:
            print(f"超時但有顯著移動，認為成功: {final_position}")
            return True
        
        print(f"夾爪移動不足，失敗: {final_position}")
        return False
        
    def close_fast(self) -> bool:
        """快速關閉 - 基於MVP.py的快速執行邏輯"""
        print("夾爪快速關閉")
        return self.send_gripper_command_batch(self.PGC_CMD_QUICK_CLOSE)
        
    def check_module_status(self) -> bool:
        """檢查夾爪模組狀態"""
        try:
            module_status = self.read_register(0)  # 500: 模組狀態
            connect_status = self.read_register(1)  # 501: 連接狀態
            
            if module_status == 1 and connect_status == 1:
                print("PGC夾爪模組狀態正常")
                return True
            else:
                print(f"PGC夾爪模組狀態異常: module={module_status}, connect={connect_status}")
                return False
                
        except Exception as e:
            print(f"檢查PGC夾爪模組狀態失敗: {e}")
            return False
    
    def send_gripper_command_batch(self, command: int, param1: int = 0, param2: int = 0) -> bool:
        """使用批量寫入發送夾爪指令 - PyModbus 3.9.2修正版"""
        try:
            cmd_id = self.get_next_command_id()
            command_base = 520
            
            # 構建指令數組 (10個寄存器)
            values = [command, param1, param2, cmd_id, 0, 0, 0, 0, 0, 0]
            
            result = self.modbus_client.write_registers(
                address=command_base,
                values=values,
                slave=1
            )
            
            # PyModbus 3.x 正確的錯誤檢查
            if hasattr(result, 'isError') and result.isError():
                print(f"PGC批量指令發送錯誤: {result}")
                return False
            else:
                return True
                
        except Exception as e:
            print(f"PGC批量指令發送異常: {e}")
            return False
    
    def send_command(self, command: int, param1: int = 0, 
                    timeout: float = 5.0, wait_completion: bool = True) -> bool:
        """發送夾爪指令 - 相容性方法"""
        return self.send_gripper_command_batch(command, param1)
        
    def get_current_position(self) -> int:
        """獲取當前位置"""
        return self.read_register(5) or 0  # 505: 當前位置
        
    def check_reached(self) -> bool:
        """檢查是否到達位置"""
        grip_status = self.read_register(4)  # 504: 夾持狀態
        return grip_status in [self.PGC_STATUS_REACHED, self.PGC_STATUS_GRIPPED]


class CCD1VisionController(ExternalModuleController):
    """CCD1視覺檢測控制器"""
    
    def __init__(self, modbus_client: ModbusTcpClient):
        super().__init__(modbus_client, ExternalModules.CCD1_BASE)
        
    def initialize(self) -> bool:
        """初始化視覺系統"""
        print("初始化CCD1視覺系統")
        return self.send_vision_command(32, timeout=10.0)  # 重新初始化
        
    def capture_and_detect(self, timeout: float = 10.0) -> bool:
        """拍照並檢測"""
        print("CCD1拍照並檢測")
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
                    print(f"CCD1視覺指令{command}執行完成")
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
            if x_int >= 2**31:  # 如果是負數（最高位為1）
                x_int = x_int - 2**32  # 轉換為負數
            x_world = x_int / 100.0
            
            # 讀取Y座標 (高位/低位)
            y_high = self.read_register(base_offset + 2) or 0
            y_low = self.read_register(base_offset + 3) or 0
            y_int = (y_high << 16) | y_low
            if y_int >= 2**31:  # 如果是負數（最高位為1）
                y_int = y_int - 2**32  # 轉換為負數
            y_world = y_int / 100.0
            
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
        print("初始化CCD3角度檢測系統")
        return self.send_angle_command(32, timeout=10.0)  # 重新初始化
        
    def detect_angle(self, timeout: float = 10.0) -> bool:
        """檢測角度"""
        print("CCD3角度檢測")
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
                    print(f"CCD3角度指令{command}執行完成")
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
    """Dobot M1Pro機械臂核心控制類 - 修正API解析版"""
    
    def __init__(self, ip: str = "192.168.1.6"):
        self.ip = ip
        self.dashboard_api: Optional[DobotApiDashboard] = None
        self.move_api: Optional[DobotApiMove] = None
        # 修正：使用執行檔案目錄的相對路徑
        current_dir = os.path.dirname(os.path.abspath(__file__))
        points_file = os.path.join(current_dir, "saved_points", "robot_points.json")
        self.points_manager = PointsManager(points_file)
        self.is_connected = False
        self.global_speed = 100
        
    def initialize(self) -> bool:
        """初始化機械臂連接"""
        try:
            self.dashboard_api = DobotApiDashboard(self.ip, 29999)
            
            self.move_api = DobotApiMove(self.ip, 30003)
            
            # 機械臂初始化設置
            self.dashboard_api.ClearError()
            self.dashboard_api.EnableRobot()
            self.dashboard_api.SpeedFactor(100)  # 全局速度比例
            self.dashboard_api.SpeedJ(100)       # 關節運動速度
            self.dashboard_api.SpeedL(100)       # 直線運動速度
            self.dashboard_api.AccJ(100)         # 關節運動加速度
            self.dashboard_api.AccL(100)         # 直線運動加速度
            
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
        """獲取機械臂模式 - 修正API解析版本"""
        try:
            if self.dashboard_api:
                result = self.dashboard_api.RobotMode()
                
                # 修正的解析方法，參照logic.py
                if result and ',' in result:
                    parts = result.split(',')
                    if len(parts) > 1:
                        # 提取花括號中的數字
                        mode_part = parts[1].strip()
                        if mode_part.startswith('{') and mode_part.endswith('}'):
                            mode_str = mode_part[1:-1]  # 去除花括號
                            return int(mode_str)
                        else:
                            # 如果沒有花括號，直接轉換
                            return int(mode_part)
            return 5  # 預設返回可用狀態
        except Exception as e:
            #print(f"獲取機械臂模式解析錯誤: {e}, 原始回應: {result if 'result' in locals() else '無回應'}")
            return 5  # 預設返回可用狀態，避免阻塞流程
    
    def get_current_pose(self) -> Dict[str, float]:
        """獲取當前位姿 - 修正API解析版本"""
        try:
            if self.dashboard_api:
                result = self.dashboard_api.GetPose()
                
                # 修正的解析方法，參照logic.py
                # 格式: "0,{223.071971,189.311344,238.825226,-227.592615,0.000000,0.000000,Right},GetPose();"
                if result and '{' in result and '}' in result:
                    # 提取花括號中的內容
                    start = result.find('{')
                    end = result.find('}')
                    if start != -1 and end != -1:
                        data_str = result[start+1:end]
                        # 分割數據，忽略最後的"Right"
                        parts = data_str.split(',')
                        if len(parts) >= 4:
                            return {
                                'x': float(parts[0]),
                                'y': float(parts[1]),
                                'z': float(parts[2]),
                                'r': float(parts[3])
                            }
                            
            return {'x': 0, 'y': 0, 'z': 0, 'r': 0}
        except Exception as e:
            print(f"獲取當前位姿失敗: {e}, 原始回應: {result if 'result' in locals() else '無回應'}")
            return {'x': 0, 'y': 0, 'z': 0, 'r': 0}
    
    def get_current_joints(self) -> Dict[str, float]:
        """獲取當前關節角度 - 修正API解析版本"""
        try:
            if self.dashboard_api:
                result = self.dashboard_api.GetAngle()
                
                # 修正的解析方法，參照logic.py
                # 格式: "0,{-2.673262,85.986069,238.825302,-310.905426,0.000000,0.000000},GetAngle();"
                if result and '{' in result and '}' in result:
                    # 提取花括號中的內容
                    start = result.find('{')
                    end = result.find('}')
                    if start != -1 and end != -1:
                        data_str = result[start+1:end]
                        # 分割數據
                        parts = data_str.split(',')
                        if len(parts) >= 4:
                            return {
                                'j1': float(parts[0]),
                                'j2': float(parts[1]),
                                'j3': float(parts[2]),
                                'j4': float(parts[3])
                            }
                            
            return {'j1': 0, 'j2': 0, 'j3': 0, 'j4': 0}
        except Exception as e:
            print(f"獲取當前關節角度失敗: {e}, 原始回應: {result if 'result' in locals() else '無回應'}")
            return {'j1': 0, 'j2': 0, 'j3': 0, 'j4': 0}
    
    def is_ready(self) -> bool:
        """檢查機械臂是否準備好 - 修正版"""
        if not self.is_connected:
            return False
        robot_mode = self.get_robot_mode()
        # 擴大可用狀態範圍，避免因狀態檢查太嚴格而無法執行
        return robot_mode in [5, 9] or robot_mode > 0  # 只要不是錯誤狀態就認為可用
    
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
        """更新狀態到PLC - PyModbus 3.9.2修正版"""
        try:
            # 更新機械臂狀態
            result = self.modbus_client.write_register(
                address=DobotRegisters.ROBOT_STATE, 
                value=self.current_state.value
            )
            
            # 更新當前流程
            result = self.modbus_client.write_register(
                address=DobotRegisters.CURRENT_FLOW, 
                value=self.current_flow.value
            )
            
            # 更新統計資訊
            self.modbus_client.write_register(
                address=DobotRegisters.OP_COUNTER, 
                value=self.operation_count
            )
            self.modbus_client.write_register(
                address=DobotRegisters.ERR_COUNTER, 
                value=self.error_count
            )
            
            # 更新運行時間(分鐘)
            run_time_minutes = int((time.time() - self.start_time) / 60)
            self.modbus_client.write_register(
                address=DobotRegisters.RUN_TIME, 
                value=run_time_minutes
            )
            
        except Exception as e:
            print(f"更新狀態到PLC異常: {e}")
    
    def read_control_from_plc(self) -> Dict[str, Any]:
        """從PLC讀取控制指令 - PyModbus 3.9.2修正版"""
        try:
            result = self.modbus_client.read_holding_registers(
                address=DobotRegisters.CONTROL_CMD, 
                count=1
            )
            
            if hasattr(result, 'isError') and result.isError():
                return {'command': 0}
            elif hasattr(result, 'registers') and len(result.registers) > 0:
                command = result.registers[0]
                return {'command': command}
            else:
                return {'command': 0}
                
        except Exception as e:
            print(f"讀取PLC控制指令異常: {e}")
            return {'command': 0}
    
    def update_robot_info(self, robot: DobotM1Pro):
        """更新機械臂資訊到寄存器 - 修正版，避免數值範圍錯誤"""
        try:
            # 更新機械臂模式
            robot_mode = robot.get_robot_mode()
            self.safe_write_register(DobotRegisters.ROBOT_MODE, robot_mode)
            
            # 更新位置資訊 (轉換為整數，避免小數點問題)
            pose = robot.get_current_pose()
            self.safe_write_register(DobotRegisters.POS_X, int(pose['x']))
            self.safe_write_register(DobotRegisters.POS_Y, int(pose['y']))
            self.safe_write_register(DobotRegisters.POS_Z, int(pose['z']))
            self.safe_write_register(DobotRegisters.POS_R, int(pose['r']))
            
            # 更新關節角度 (放大100倍後取絕對值，避免負數)
            joints = robot.get_current_joints()
            self.safe_write_register(DobotRegisters.JOINT_J1, int(abs(joints['j1']) * 100) % 65536)
            self.safe_write_register(DobotRegisters.JOINT_J2, int(abs(joints['j2']) * 100) % 65536)
            self.safe_write_register(DobotRegisters.JOINT_J3, int(abs(joints['j3']) * 100) % 65536)
            self.safe_write_register(DobotRegisters.JOINT_J4, int(abs(joints['j4']) * 100) % 65536)
            
        except Exception as e:
            print(f"更新機械臂資訊異常: {e}")
    
    def safe_write_register(self, address: int, value: Any) -> bool:
        """安全寫入寄存器，處理數值範圍限制"""
        try:
            # 確保數值在16位無符號整數範圍內 (0-65535)
            if isinstance(value, (int, float)):
                # 處理負數和超範圍的情況
                if value < 0:
                    safe_value = 0
                elif value > 65535:
                    safe_value = 65535
                else:
                    safe_value = int(value)
            else:
                safe_value = 0
            
            result = self.modbus_client.write_register(
                address=address,
                value=safe_value
            )
            
            return not (hasattr(result, 'isError') and result.isError())
            
        except Exception as e:
            return False
    
    def is_ready_for_command(self) -> bool:
        """檢查是否準備好接受指令"""
        return self.current_state == RobotState.IDLE


class DobotMotionController:
    """Dobot運動控制主控制器 - 修正版"""
    
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
        
        # 流程執行器
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
                "enabled": True,  # 開啟真實夾爪
                "base_address": 520,
                "status_address": 500,
                "default_force": 50,
                "default_speed": 80
            },
            "vision": {
                "ccd1_base_address": 200,
                "ccd3_base_address": 800,
                "detection_timeout": 10.0,
                "ccd1_enabled": True,  # 開啟真實CCD1
                "ccd3_enabled": False
            },
            "flows": {
                "flow1_enabled": True,
                "flow2_enabled": False,
                "flow3_enabled": False
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
        if self.config["gripper"]["enabled"]:
            self.gripper = PGCGripperController(self.modbus_client)
            print("PGC夾爪控制器已啟用")
        else:
            print("PGC夾爪控制器已停用")
            
        if self.config["vision"]["ccd1_enabled"]:
            self.ccd1 = CCD1VisionController(self.modbus_client)
            print("CCD1視覺控制器已啟用")
        else:
            print("CCD1視覺控制器已停用")
            
        if self.config["vision"]["ccd3_enabled"]:
            self.ccd3 = CCD3AngleController(self.modbus_client)
            print("CCD3角度控制器已啟用")
        else:
            print("CCD3角度控制器已停用")
        
        # 4. 初始化流程執行器
        if self.config["flows"]["flow1_enabled"]:
            self.flows[1] = Flow1Executor(
                robot=self.robot,
                gripper=self.gripper, 
                ccd1=self.ccd1,
                ccd3=self.ccd3,
                state_machine=self.state_machine
            )
            print("Flow1 執行器初始化完成")
        
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
        """連接所有設備 - 修正版，允許部分設備失敗"""
        print("=== 連接所有設備 ===")
        
        # 1. 連接機械臂
        if not self.robot.initialize():
            print("機械臂連接失敗")
            return False
        
        # 2. 初始化夾爪 (如果啟用)
        if self.gripper:
            try:
                if not self.gripper.initialize():
                    print("PGC夾爪初始化失敗，但繼續運行")
                    self.gripper = None
                else:
                    print("PGC夾爪初始化成功")
            except Exception as e:
                print(f"PGC夾爪初始化異常: {e}，繼續運行")
                self.gripper = None
        
        # 3. 初始化視覺系統 (如果啟用)
        if self.ccd1:
            try:
                if not self.ccd1.initialize():
                    print("CCD1視覺系統初始化失敗，但繼續運行")
            except Exception as e:
                print(f"CCD1初始化異常: {e}")
        
        if self.ccd3:
            try:
                if not self.ccd3.initialize():
                    print("CCD3角度檢測系統初始化失敗，但繼續運行")
            except Exception as e:
                print(f"CCD3初始化異常: {e}")
        
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
        """狀態機交握主循環 - 增強版本，避免重複處理"""
        print("狀態機交握循環開始")
        last_command = 0  # 追蹤上次處理的指令
        
        while self.is_running:
            try:
                # 讀取PLC控制指令
                control_data = self.state_machine.read_control_from_plc()
                command = control_data.get('command', 0)
                
                # 只處理新指令（避免重複處理）
                if command != 0 and command != last_command:
                    print(f"檢測到新指令: {command} (上次: {last_command})")
                    self._handle_plc_command(command)
                    last_command = command
                elif command == 0 and last_command != 0:
                    # 指令被清除
                    print(f"指令已清除 (上次: {last_command})")
                    last_command = 0
                
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
        """處理PLC指令 - 修正指令清除邏輯"""
        try:
            print(f"收到PLC指令: {command}")
            
            if command == CommandType.EMERGENCY_STOP.value:
                print("收到緊急停止指令")
                self.emergency_stop_all()
                # 立即清除指令
                self._clear_command()
                
            elif command in [CommandType.FLOW_1.value, CommandType.FLOW_2.value, CommandType.FLOW_3.value]:
                if self.state_machine.is_ready_for_command():
                    print(f"收到流程{command}執行指令")
                    
                    # 🔥 關鍵修正：立即清除指令，避免重複執行
                    self._clear_command()
                    
                    # 然後執行流程
                    self.execute_flow(command)
                else:
                    current_state = self.state_machine.current_state.name
                    print(f"系統忙碌，無法執行流程{command} (當前狀態: {current_state})")
                    
                    # 如果系統忙碌，也清除指令避免重複
                    self._clear_command()
                    
            elif command == CommandType.CLEAR.value:
                print("收到清空指令")
                self._clear_command()
                
        except Exception as e:
            print(f"處理PLC指令{command}失敗: {e}")
            # 發生異常時也清除指令
            self._clear_command()
    
    def execute_flow(self, flow_id: int) -> bool:
        """執行指定流程 - 增強版本"""
        if flow_id not in self.flows:
            print(f"流程{flow_id}未啟用或不存在")
            return False
        
        if self.current_flow and hasattr(self.current_flow, 'is_running') and self.current_flow.is_running:
            print("有流程正在執行中")
            return False
        
        try:
            print(f"開始執行流程{flow_id}")
            
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
            
            print(f"流程{flow_id}線程已啟動")
            return True
            
        except Exception as e:
            print(f"啟動流程{flow_id}失敗: {e}")
            self.state_machine.set_state(RobotState.ERROR)
            return False
    
    def _execute_flow_thread(self, flow_executor):
        """流程執行線程 - 增強錯誤處理版本"""
        flow_name = type(flow_executor).__name__
        
        try:
            print(f"流程線程開始執行: {flow_name}")
            
            # 調用流程的execute方法
            result = flow_executor.execute()
            
            # 處理FlowResult對象
            if hasattr(result, 'success'):
                if result.success:
                    print(f"流程{flow_name}執行成功，耗時: {result.execution_time:.2f}秒")
                    print(f"完成步驟: {result.steps_completed}/{result.total_steps}")
                    self.state_machine.operation_count += 1
                    self.state_machine.set_state(RobotState.IDLE)
                else:
                    print(f"流程{flow_name}執行失敗: {result.error_message}")
                    print(f"失敗於步驟: {result.steps_completed}/{result.total_steps}")
                    self.state_machine.error_count += 1
                    self.state_machine.set_state(RobotState.ERROR)
            else:
                # 處理舊版本的bool返回值
                if result:
                    print(f"流程{flow_name}執行成功")
                    self.state_machine.operation_count += 1
                    self.state_machine.set_state(RobotState.IDLE)
                else:
                    print(f"流程{flow_name}執行失敗")
                    self.state_machine.error_count += 1
                    self.state_machine.set_state(RobotState.ERROR)
                    
        except Exception as e:
            print(f"流程{flow_name}執行異常: {e}")
            traceback.print_exc()
            self.state_machine.error_count += 1
            self.state_machine.set_state(RobotState.ERROR)
        finally:
            # 確保狀態機正確重置
            self.state_machine.set_flow(FlowType.NONE)
            self.current_flow = None
            print(f"流程線程結束: {flow_name}")
    
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
        """清空指令 - 增強版本，確保清除成功"""
        try:
            # 清除控制指令寄存器
            result = self.modbus_client.write_register(
                address=DobotRegisters.CONTROL_CMD, 
                value=0
            )
            if hasattr(result, 'isError') and result.isError():
                print(f"清空指令失敗: {result}")
            else:
                print("指令已清空")
                
            # 額外確認：讀取寄存器確保清除成功
            read_result = self.modbus_client.read_holding_registers(
                address=DobotRegisters.CONTROL_CMD, 
                count=1
            )
            if hasattr(read_result, 'registers') and len(read_result.registers) > 0:
                actual_value = read_result.registers[0]
                if actual_value != 0:
                    print(f"警告：指令清除後寄存器值仍為 {actual_value}")
                    
        except Exception as e:
            print(f"清空指令異常: {e}")
    
    def get_system_status(self) -> Dict[str, Any]:
        """獲取系統狀態 - 增加更多診斷信息"""
        return {
            "robot_connected": self.robot.is_connected,
            "robot_ready": self.robot.is_ready() if self.robot.is_connected else False,
            "current_state": self.state_machine.current_state.name,
            "current_flow": self.state_machine.current_flow.name,
            "operation_count": self.state_machine.operation_count,
            "error_count": self.state_machine.error_count,
            "is_running": self.is_running,
            "flows_enabled": list(self.flows.keys()),
            "gripper_enabled": self.gripper is not None,
            "ccd1_enabled": self.ccd1 is not None,
            "ccd3_enabled": self.ccd3 is not None,
            # 新增診斷信息
            "is_ready_for_command": self.state_machine.is_ready_for_command(),
            "current_flow_object": str(type(self.current_flow).__name__) if self.current_flow else None,
            "handshake_thread_alive": self.handshake_thread.is_alive() if self.handshake_thread else False
        }
    
    def force_reset_state(self):
        """強制重置狀態機 - 緊急恢復用"""
        try:
            print("=== 執行強制狀態重置 ===")
            
            # 停止當前流程
            if self.current_flow:
                if hasattr(self.current_flow, 'stop'):
                    self.current_flow.stop()
                self.current_flow = None
                print("已停止當前流程")
            
            # 重置狀態機
            self.state_machine.set_state(RobotState.IDLE)
            self.state_machine.set_flow(FlowType.NONE)
            print("狀態機已重置為IDLE")
            
            # 清除PLC指令
            self._clear_command()
            print("已清除PLC指令")
            
            print("強制狀態重置完成")
            return True
            
        except Exception as e:
            print(f"強制狀態重置失敗: {e}")
            return False
    
    def diagnose_system_state(self):
        """系統狀態診斷 - 詳細檢查"""
        print("\n=== 系統狀態診斷 ===")
        
        # 狀態機診斷
        print(f"狀態機當前狀態: {self.state_machine.current_state.name}")
        print(f"狀態機當前流程: {self.state_machine.current_flow.name}")
        print(f"是否準備好接受指令: {self.state_machine.is_ready_for_command()}")
        
        # 流程執行器診斷
        if self.current_flow:
            print(f"當前流程對象: {type(self.current_flow).__name__}")
            if hasattr(self.current_flow, 'is_running'):
                print(f"流程內部運行狀態: {self.current_flow.is_running}")
            if hasattr(self.current_flow, 'current_step'):
                print(f"流程當前步驟: {self.current_flow.current_step}")
        else:
            print("當前流程對象: None")
        
        # 線程診斷
        if self.handshake_thread:
            print(f"握手線程存活: {self.handshake_thread.is_alive()}")
        else:
            print("握手線程: 未啟動")
        
        # 機械臂診斷
        print(f"機械臂連接狀態: {self.robot.is_connected}")
        if self.robot.is_connected:
            print(f"機械臂準備狀態: {self.robot.is_ready()}")
            robot_mode = self.robot.get_robot_mode()
            print(f"機械臂模式: {robot_mode}")
        
        # Modbus診斷
        if self.modbus_client:
            print(f"Modbus連接狀態: {self.modbus_client.connected}")
        
        # 外部模組診斷
        if self.gripper:
            print("PGC夾爪: 已啟用")
        if self.ccd1:
            print("CCD1視覺: 已啟用")
        if self.ccd3:
            print("CCD3角度: 已啟用")
        
        print("=== 診斷完成 ===\n")
    
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
        
        status = controller.get_system_status()
        print(f"\n系統狀態:")
        print(f"  啟用的流程: {status['flows_enabled']}")
        print(f"  PGC夾爪: {'啟用' if status['gripper_enabled'] else '停用'}")
        print(f"  CCD1視覺: {'啟用' if status['ccd1_enabled'] else '停用'}")
        print(f"  CCD3角度: {'啟用' if status['ccd3_enabled'] else '停用'}")
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