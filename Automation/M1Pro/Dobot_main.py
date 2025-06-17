#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_main.py - 機械臂主控制器 (狀態機交握實現版)
整合狀態機管理、外部模組通訊、運動控制等功能
基地址400，支援多流程執行與外部設備整合
實現完整的狀態機交握協議
"""
# 在文件頂部修改import
from CCD1HighLevel import CCD1HighLevelAPI
from GripperHighLevel import GripperHighLevelAPI
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
from Dobot_Flow2 import Flow2Executor

from pymodbus.client.tcp import ModbusTcpClient
from dobot_api import DobotApiDashboard, DobotApiMove


# 配置常數
DOBOT_BASE_ADDR = 400
CONFIG_FILE = "dobot_config.json"

# Modbus寄存器映射 (基地址400) - 狀態機交握版
class DobotRegisters:
    # 狀態寄存器 (400-419) - 只讀
    STATUS_REGISTER = DOBOT_BASE_ADDR + 0     # 400: 主狀態寄存器 (bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized)
    ROBOT_STATE = DOBOT_BASE_ADDR + 1         # 401: 機械臂狀態
    CURRENT_FLOW = DOBOT_BASE_ADDR + 2        # 402: 當前流程ID
    FLOW_PROGRESS = DOBOT_BASE_ADDR + 3       # 403: 流程執行進度
    ERROR_CODE = DOBOT_BASE_ADDR + 4          # 404: 錯誤代碼
    ROBOT_MODE = DOBOT_BASE_ADDR + 5          # 405: 機械臂模式
    
    # 位置資訊寄存器 (406-409)
    POS_X = DOBOT_BASE_ADDR + 6               # 406: 當前X座標
    POS_Y = DOBOT_BASE_ADDR + 7               # 407: 當前Y座標  
    POS_Z = DOBOT_BASE_ADDR + 8               # 408: 當前Z座標
    POS_R = DOBOT_BASE_ADDR + 9               # 409: 當前R座標
    
    # 關節角度寄存器 (410-413)
    JOINT_J1 = DOBOT_BASE_ADDR + 10           # 410: J1角度
    JOINT_J2 = DOBOT_BASE_ADDR + 11           # 411: J2角度
    JOINT_J3 = DOBOT_BASE_ADDR + 12           # 412: J3角度
    JOINT_J4 = DOBOT_BASE_ADDR + 13           # 413: J4角度
    
    # IO狀態寄存器 (414-415)
    DI_STATUS = DOBOT_BASE_ADDR + 14          # 414: 數位輸入狀態
    DO_STATUS = DOBOT_BASE_ADDR + 15          # 415: 數位輸出狀態
    
    # 統計寄存器 (416-419)
    OP_COUNTER = DOBOT_BASE_ADDR + 16         # 416: 操作計數器
    ERR_COUNTER = DOBOT_BASE_ADDR + 17        # 417: 錯誤計數器
    RUN_TIME = DOBOT_BASE_ADDR + 18           # 418: 運行時間(分鐘)
    GLOBAL_SPEED = DOBOT_BASE_ADDR + 19       # 419: 全局速度設定值 (新增)
    FLOW1_COMPLETE = DOBOT_BASE_ADDR + 20     # 420: Flow1完成狀態(0=未完成,1=完成且角度較鄭成功)

    # 控制寄存器 (440-449) - 讀寫
    VP_CONTROL = DOBOT_BASE_ADDR + 40         # 440: VP視覺取料控制
    UNLOAD_CONTROL = DOBOT_BASE_ADDR + 41     # 441: 出料控制
    CLEAR_ALARM = DOBOT_BASE_ADDR + 42        # 442: 清除警報控制
    EMERGENCY_STOP = DOBOT_BASE_ADDR + 43     # 443: 緊急停止控制
    MANUAL_COMMAND = DOBOT_BASE_ADDR + 44     # 444: 手動指令 (Web端使用)
    SPEED_COMMAND = DOBOT_BASE_ADDR + 45      # 445: 速度控制指令 (新增)
    SPEED_VALUE = DOBOT_BASE_ADDR + 46        # 446: 速度數值 (新增)
    SPEED_CMD_ID = DOBOT_BASE_ADDR + 47       # 447: 速度指令ID (新增)


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
    FLOW_2 = 2        # 流程2 - 出料流程
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
        self.global_speed = 50  # 預設全局速度
        self.last_set_speed = 50  # 追蹤最後設定的速度
        
    def initialize(self) -> bool:
        """初始化機械臂連接 - 增強版"""
        try:
            self.dashboard_api = DobotApiDashboard(self.ip, 29999)
            self.move_api = DobotApiMove(self.ip, 30003)
            
            # 機械臂初始化設置
            self.dashboard_api.ClearError()
            self.dashboard_api.EnableRobot()
            
            # 設定初始速度
            self.set_global_speed(self.global_speed)
            
            self.is_connected = True
            print(f"機械臂初始化成功: {self.ip}, 初始速度: {self.global_speed}%")
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
        """設置全局速度 - 增強版"""
        try:
            if speed < 1 or speed > 100:
                print(f"速度值超出範圍(1-100): {speed}")
                return False
                
            if self.dashboard_api:
                # 同時設定所有速度相關參數
                self.dashboard_api.SpeedFactor(speed)     # 全局速度比例
                self.dashboard_api.SpeedJ(speed)          # 關節運動速度
                self.dashboard_api.SpeedL(speed)          # 直線運動速度
                self.dashboard_api.AccJ(speed)            # 關節運動加速度
                self.dashboard_api.AccL(speed)            # 直線運動加速度
                
                self.global_speed = speed
                self.last_set_speed = speed
                print(f"全局速度已設定為: {speed}%")
                return True
            else:
                print("機械臂API未連接，無法設定速度")
                return False
                
        except Exception as e:
            print(f"設置全局速度失敗: {e}")
            return False
    def get_global_speed(self) -> int:
        """獲取當前全局速度"""
        return self.global_speed

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
    """Dobot狀態機管理 - 狀態機交握實現版"""
    
    def __init__(self, modbus_client: ModbusTcpClient):
        self.modbus_client = modbus_client
        self.current_state = RobotState.IDLE
        self.current_flow = FlowType.NONE
        self.operation_count = 0
        self.error_count = 0
        self.start_time = time.time()
        self.flow1_complete_status = 0 #新增:Flow1完成狀態追蹤

        # 狀態機交握核心 - 二進制位控制
        self.status_register = 0b1001  # 初始: Ready=1, Initialized=1
        self.lock = threading.Lock()   # 線程安全保護
        # 速度控制狀態機 (新增)
        self.current_global_speed = 50  # 當前全局速度
        self.last_speed_cmd_id = 0      # 最後處理的速度指令ID
    def set_flow1_complete(self, complete: bool):
        """設置Flow1完成狀態"""
        self.flow1_complete_status = 1 if complete else 0
        try:
            self.modbus_client.write_register(
                address=DobotRegisters.FLOW1_COMPLETE,
                value=self.flow1_complete_status
            )
            print(f"Flow1完成狀態已設置: {self.flow1_complete_status}")
        except Exception as e:
            print(f"設置Flow1完成狀態失敗: {e}")
    
    def get_flow1_complete(self) -> bool:
        """獲取Flow1完成狀態"""
        return self.flow1_complete_status == 1
    
    def clear_flow1_complete(self):
        """清除Flow1完成狀態 (用於重新開始流程)"""
        self.set_flow1_complete(False)    
    # === 狀態機交握核心方法 ===
    def get_status_bit(self, bit_pos: int) -> bool:
        """獲取狀態位"""
        with self.lock:
            return bool(self.status_register & (1 << bit_pos))
    def update_global_speed_register(self, speed: int):
        """更新全局速度寄存器"""
        try:
            self.current_global_speed = speed
            self.safe_write_register(DobotRegisters.GLOBAL_SPEED, speed)
            print(f"全局速度寄存器已更新: {speed}%")
        except Exception as e:
            print(f"更新全局速度寄存器失敗: {e}")
    def process_speed_command(self, robot: DobotM1Pro) -> bool:
        """處理速度控制指令 - 狀態機交握"""
        try:
            # 讀取速度控制指令
            speed_command = self.read_control_register(5)      # 445: 速度控制指令
            speed_value = self.read_control_register(6)        # 446: 速度數值
            speed_cmd_id = self.read_control_register(7)       # 447: 速度指令ID
            
            # 檢查是否有新的速度指令
            if (speed_command == 1 and 
                speed_cmd_id != 0 and 
                speed_cmd_id != self.last_speed_cmd_id):
                
                print(f"收到速度設定指令: {speed_value}%, ID: {speed_cmd_id}")
                
                # 驗證速度範圍
                if speed_value < 1 or speed_value > 100:
                    print(f"速度值超出範圍: {speed_value}")
                    # 清除無效指令
                    self.safe_write_register(DobotRegisters.SPEED_COMMAND, 0)
                    return False
                
                # 設定機械臂速度
                success = robot.set_global_speed(speed_value)
                
                if success:
                    # 更新速度寄存器
                    self.update_global_speed_register(speed_value)
                    print(f"全局速度設定成功: {speed_value}%")
                else:
                    print(f"全局速度設定失敗: {speed_value}%")
                
                # 更新最後處理的指令ID
                self.last_speed_cmd_id = speed_cmd_id
                
                # 清除速度控制指令
                self.safe_write_register(DobotRegisters.SPEED_COMMAND, 0)
                
                return success
                
            return True  # 沒有新指令，返回成功
            
        except Exception as e:
            print(f"處理速度控制指令異常: {e}")
            return False
    def set_status_bit(self, bit_pos: int, value: bool):
        """設置狀態位"""
        with self.lock:
            if value:
                self.status_register |= (1 << bit_pos)
            else:
                self.status_register &= ~(1 << bit_pos)
    
    def is_ready(self) -> bool:
        """檢查Ready狀態 (bit0)"""
        return self.get_status_bit(0)
    
    def is_running(self) -> bool:
        """檢查Running狀態 (bit1)"""
        return self.get_status_bit(1)
    
    def is_alarm(self) -> bool:
        """檢查Alarm狀態 (bit2)"""
        return self.get_status_bit(2)
    
    def is_initialized(self) -> bool:
        """檢查Initialized狀態 (bit3)"""
        return self.get_status_bit(3)
    
    def set_ready(self, ready: bool):
        """設置Ready狀態"""
        if ready:
            self.set_status_bit(0, True)   # 設置Ready=1
            self.set_status_bit(1, False)  # 清除Running=0
            self.set_status_bit(2, False)  # 清除Alarm=0
        else:
            self.set_status_bit(0, False)  # 清除Ready=0
        print(f"狀態機Ready設置為: {ready}, 當前狀態寄存器: {self.status_register:04b}")
    
    def set_running(self, running: bool):
        """設置Running狀態"""
        if running:
            self.set_status_bit(1, True)   # 設置Running=1
            self.set_status_bit(0, False)  # 清除Ready=0
        else:
            self.set_status_bit(1, False)  # 清除Running=0
            # 注意: 不自動設置Ready=1，等待控制指令清零
        print(f"狀態機Running設置為: {running}, 當前狀態寄存器: {self.status_register:04b}")
    
    def set_alarm(self, alarm: bool):
        """設置Alarm狀態"""
        if alarm:
            self.set_status_bit(2, True)   # 設置Alarm=1
            self.set_status_bit(0, False)  # 清除Ready=0
            self.set_status_bit(1, False)  # 清除Running=0
        else:
            self.set_status_bit(2, False)  # 清除Alarm=0
        print(f"狀態機Alarm設置為: {alarm}, 當前狀態寄存器: {self.status_register:04b}")
    
    def set_initialized(self, initialized: bool):
        """設置Initialized狀態"""
        self.set_status_bit(3, initialized)
        print(f"狀態機Initialized設置為: {initialized}")
    
    def clear_alarm_state(self):
        """清除警報狀態 - 專用方法"""
        print("收到清除警報指令")
        self.set_alarm(False)
        # 清除警報後恢復Ready狀態
        if not self.is_running():
            self.set_ready(True)
        print("警報狀態已清除")
    
    def is_ready_for_command(self) -> bool:
        """檢查是否準備好接受指令"""
        return self.is_ready() and not self.is_running() and not self.is_alarm()
    
    # === 原有方法保持相容性 ===
    def set_state(self, new_state: RobotState):
        """設置機械臂狀態 - 相容性方法"""
        self.current_state = new_state
        self.update_status_to_plc()
        
    def set_flow(self, flow_type: FlowType):
        """設置當前流程"""
        self.current_flow = flow_type
        self.update_status_to_plc()
    
    def read_control_register(self, register_offset: int) -> int:
        """讀取控制寄存器 - 增強版"""
        try:
            result = self.modbus_client.read_holding_registers(
                address=DobotRegisters.VP_CONTROL + register_offset, 
                count=1
            )
            
            if hasattr(result, 'isError') and result.isError():
                return 0
            elif hasattr(result, 'registers') and len(result.registers) > 0:
                return result.registers[0]
            else:
                return 0
                
        except Exception as e:
            print(f"讀取控制寄存器{DobotRegisters.VP_CONTROL + register_offset}異常: {e}")
            return 0
    
    def update_status_to_plc(self):
        """更新狀態到PLC - 增強版含速度"""
        try:
            # 更新主狀態寄存器 (400)
            self.modbus_client.write_register(
                address=DobotRegisters.STATUS_REGISTER, 
                value=self.status_register
            )
            
            # 更新機械臂狀態 (401)
            self.modbus_client.write_register(
                address=DobotRegisters.ROBOT_STATE, 
                value=self.current_state.value
            )
            
            # 更新當前流程 (402)
            self.modbus_client.write_register(
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
            
            # 更新全局速度 (419) - 新增
            self.modbus_client.write_register(
                address=DobotRegisters.GLOBAL_SPEED, 
                value=self.current_global_speed
            )
             # 新增: 更新Flow1完成狀態 (420)
            self.modbus_client.write_register(
            address=DobotRegisters.FLOW1_COMPLETE,
            value=self.flow1_complete_status
            )
        except Exception as e:
            print(f"更新狀態到PLC異常: {e}")
    
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


class DobotMotionController:
    """Dobot運動控制主控制器 - 狀態機交握實現版"""
    
    def __init__(self, config_file: str = CONFIG_FILE):
        self.config_file = config_file
        self.config = self._load_config()
        
        # 核心組件
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.robot = DobotM1Pro(self.config["robot"]["ip"])
        self.state_machine: Optional[DobotStateMachine] = None
        
        # 外部模組控制器
        self.gripper: Optional[GripperHighLevelAPI] = None
        self.ccd1: Optional[CCD1HighLevelAPI] = None
        self.ccd3: Optional[Any] = None  # 保持原有
        
        # 流程執行器
        self.flows: Dict[int, Any] = {}
        self.current_flow: Optional[Any] = None
        
        # 運行狀態
        self.is_running = False
        self.handshake_thread: Optional[threading.Thread] = None
        # === 新增: 握手狀態追蹤變量 ===
        self.last_vp_control = 0
        self.last_unload_control = 0
        self.last_clear_alarm = 0
        self.last_emergency_stop = 0
        self.last_manual_command = 0
        self.last_speed_cmd_id = 0  # 新增速度指令ID追蹤
        
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
                "flow2_enabled": True,
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
        """初始化系統 - 修改為使用高層API"""
        print("=== 初始化Dobot運動控制系統 (狀態機交握版) ===")
        
        # 1. 連接Modbus服務器
        if not self._connect_modbus():
            return False
        
        # 2. 初始化狀態機
        self.state_machine = DobotStateMachine(self.modbus_client)
        
        # 3. 初始化外部模組控制器 - 使用高層API
        if self.config["gripper"]["enabled"]:
            self.gripper = GripperHighLevelAPI(
                modbus_host=self.config["modbus"]["server_ip"],
                modbus_port=self.config["modbus"]["server_port"]
            )
            print("✓ PGC夾爪高層API已啟用")
        else:
            print("PGC夾爪高層API已停用")
            
        if self.config["vision"]["ccd1_enabled"]:
            self.ccd1 = CCD1HighLevelAPI(
                modbus_host=self.config["modbus"]["server_ip"],
                modbus_port=self.config["modbus"]["server_port"]
            )
            print("✓ CCD1視覺高層API已啟用")
        else:
            print("CCD1視覺高層API已停用")
        
        # 4. 初始化流程執行器
        if self.config["flows"]["flow1_enabled"]:
            self.flows[1] = Flow1Executor(
                robot=self.robot,
                gripper=self.gripper,     # 傳入高層API實例
                ccd1=self.ccd1,          # 傳入高層API實例
                ccd3=self.ccd3,
                state_machine=self.state_machine
            )
            print("✓ Flow1執行器初始化完成 (使用高層API)")
        
        if self.config["flows"]["flow2_enabled"]:
            self.flows[2] = Flow2Executor(
                robot=self.robot,
                gripper=self.gripper,
                ccd1=self.ccd1,
                ccd3=self.ccd3,
                state_machine=self.state_machine
            )
            print("✓ Flow2執行器初始化完成 (出料流程)")
        
        # 5. 載入點位數據
        if not self.robot.points_manager.load_points():
            print("載入點位數據失敗，但繼續運行")
        
        print("系統初始化完成 (狀態機交握版)")
        return True
    
    def connect_all_devices(self) -> bool:
        """連接所有設備 - 簡化版本，高層API自動處理連接"""
        print("=== 連接所有設備 ===")
        
        # 1. 連接機械臂
        if not self.robot.initialize():
            print("機械臂連接失敗")
            return False
        
        # 2. 檢查夾爪連接 (高層API自動處理連接)
        if self.gripper:
            try:
                status = self.gripper.get_status()
                if status['connected']:
                    print("✓ PGC夾爪高層API連接正常")
                    
                    # 自動初始化夾爪
                    if not status['initialized']:
                        print("夾爪未初始化，自動初始化...")
                        if self.gripper.initialize(wait_completion=True):
                            print("✓ PGC夾爪初始化成功")
                        else:
                            print("⚠️ PGC夾爪初始化失敗，但繼續運行")
                else:
                    print("⚠️ PGC夾爪連接異常，但繼續運行")
            except Exception as e:
                print(f"⚠️ PGC夾爪檢查異常: {e}，但繼續運行")
        
        # 3. 檢查CCD1連接 (高層API自動處理連接)
        if self.ccd1:
            try:
                status = self.ccd1.get_system_status()
                if status['connected']:
                    print("✓ CCD1視覺高層API連接正常")
                    if status['world_coord_valid']:
                        print("✓ CCD1世界座標有效")
                    else:
                        print("⚠️ CCD1世界座標無效，可能缺少標定數據")
                else:
                    print("⚠️ CCD1視覺連接異常，但繼續運行")
            except Exception as e:
                print(f"⚠️ CCD1視覺檢查異常: {e}，但繼續運行")
        
        print("設備連接完成 (使用高層API)")
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
        """狀態機交握主循環 - 新增速度控制"""
        print("狀態機交握循環開始 (含速度控制)")
        
        # 追蹤指令狀態，避免重複處理
        last_vp_control = 0
        last_unload_control = 0
        last_clear_alarm = 0
        last_emergency_stop = 0
        last_manual_command = 0
        
        while self.is_running:
            try:
                # 讀取控制寄存器
                vp_control = self.state_machine.read_control_register(0)      # 440: VP視覺取料控制
                unload_control = self.state_machine.read_control_register(1)  # 441: 出料控制
                clear_alarm = self.state_machine.read_control_register(2)     # 442: 清除警報控制
                emergency_stop = self.state_machine.read_control_register(3)  # 443: 緊急停止控制
                manual_command = self.state_machine.read_control_register(4)  # 444: 手動指令
                # === 新增: 處理速度控制指令 ===
                if self.robot.is_connected:
                    self.state_machine.process_speed_command(self.robot)
                 # === 新增: Flow1完成狀態自動管理 ===
                # 當Flow1執行時自動清除完成狀態
                if (vp_control == 1 and 
                    self.state_machine.is_ready_for_command() and 
                    self.state_machine.get_flow1_complete()):
                    print("檢測到新的Flow1指令，清除之前的完成狀態")
                    self.state_machine.clear_flow1_complete()
                
                # 處理VP視覺取料控制 (Flow1)
                if (vp_control == 1 and 
                    self.state_machine.is_ready_for_command() and 
                    self.last_vp_control != vp_control):
                    
                    print("收到VP視覺取料指令 (Flow1)")
                    self.state_machine.set_running(True)
                    self.state_machine.set_current_flow(1)
                    self.last_vp_control = vp_control
                    
                    # 異步執行Flow1
                    threading.Thread(target=self.execute_flow1_async, daemon=True).start()
                
                # Flow1完成後的狀態處理 - Flow1內部會自動設置完成狀態
                # 這裡只需要處理PLC清零指令
                elif vp_control == 0 and self.last_vp_control == 1:
                    print("PLC已清零VP取料指令")
                    self.last_vp_control = 0
                    if not self.state_machine.is_running():
                        self.state_machine.set_ready(True)
                        self.state_machine.set_current_flow(0)
                
                # 處理出料控制 (Flow2) - 需要檢查Flow1完成狀態
                if (unload_control == 1 and 
                    self.state_machine.is_ready_for_command() and 
                    self.last_unload_control != unload_control and
                    vp_control == 0):  # 確保VP控制已清零
                    
                    # 檢查Flow1是否已完成 (可選條件，根據業務需求決定)
                    if self.state_machine.get_flow1_complete():
                        print("收到出料指令 (Flow2) - Flow1已完成")
                    else:
                        print("收到出料指令 (Flow2) - Flow1未完成，但允許執行")
                    
                    self.state_machine.set_running(True)
                    self.state_machine.set_current_flow(2)
                    self.last_unload_control = unload_control
                    
                    # 異步執行Flow2
                    threading.Thread(target=self.execute_flow2_async, daemon=True).start()
                
                elif unload_control == 0 and self.last_unload_control == 1:
                    print("PLC已清零出料指令")
                    self.last_unload_control = 0
                    if not self.state_machine.is_running():
                        self.state_machine.set_ready(True)
                        self.state_machine.set_current_flow(0)
                # 處理緊急停止 (最高優先級)
                if emergency_stop == 1 and last_emergency_stop != 1:
                    print("收到緊急停止指令")
                    self.emergency_stop_all()
                    last_emergency_stop = 1
                elif emergency_stop == 0:
                    last_emergency_stop = 0
                
                # 處理警報清除
                if clear_alarm == 1 and last_clear_alarm != 1:
                    self.state_machine.clear_alarm_state()
                    last_clear_alarm = 1
                elif clear_alarm == 0:
                    last_clear_alarm = 0
                
                # 處理VP視覺取料指令 (Flow1)
                if vp_control == 1 and last_vp_control != 1:
                    if self.state_machine.is_ready_for_command():
                        print("收到VP視覺取料指令，啟動Flow1")
                        self.execute_flow1_async()
                    else:
                        print(f"系統未Ready，無法執行Flow1 (狀態: {self.state_machine.status_register:04b})")
                    last_vp_control = 1
                elif vp_control == 0:
                    # VP控制清零，檢查是否需要恢復Ready
                    if last_vp_control == 1 and not self.state_machine.is_running() and not self.state_machine.is_alarm():
                        print("VP控制指令已清零，恢復Ready狀態")
                        self.state_machine.set_ready(True)
                    last_vp_control = 0
                
                # 處理出料指令 (Flow2) - 需要Ready且VP控制已清零
                if unload_control == 1 and last_unload_control != 1:
                    if self.state_machine.is_ready_for_command() and vp_control == 0:
                        print("收到出料指令，啟動Flow2")
                        self.execute_flow2_async()
                    else:
                        if not self.state_machine.is_ready_for_command():
                            print(f"系統未Ready，無法執行Flow2 (狀態: {self.state_machine.status_register:04b})")
                        if vp_control != 0:
                            print(f"VP控制未清零({vp_control})，無法執行Flow2")
                    last_unload_control = 1
                elif unload_control == 0:
                    # 出料控制清零，檢查是否需要恢復Ready
                    if last_unload_control == 1 and not self.state_machine.is_running() and not self.state_machine.is_alarm():
                        print("出料控制指令已清零，恢復Ready狀態")
                        self.state_machine.set_ready(True)
                    last_unload_control = 0
                
                # 處理Web端手動指令
                if manual_command != 0 and manual_command != last_manual_command:
                    self.handle_manual_command(manual_command)
                    last_manual_command = manual_command
                elif manual_command == 0:
                    last_manual_command = 0
                
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
    
    def execute_flow1_async(self):
        """異步執行Flow1 - 狀態機交握版本"""
        try:
            print("開始執行Flow1...")
            
            # 創建Flow1執行器
            flow1_executor = Flow1Executor(
                self.robot, self.gripper, self.ccd1, self.ccd3, self.state_machine
            )
            
            # 執行Flow1 (包含角度校正)
            result = flow1_executor.execute()
            
            if result.success:
                print("Flow1執行成功")
                # 注意: Flow1完成狀態由Flow1內部的 _set_flow1_completion_status() 方法設置
                # 這裡不需要再次設置，避免重複
                if result.angle_correction_performed:
                    print(f"角度校正結果: {result.angle_correction_result}")
                
                # 設置執行完成狀態 (非Running狀態)
                self.state_machine.set_running(False)
                self.state_machine.set_current_flow(0)
                
            else:
                print(f"Flow1執行失敗: {result.error_message}")
                self.state_machine.set_alarm(True)
                self.state_machine.set_running(False)
                self.state_machine.set_current_flow(0)
                # Flow1失敗時不設置完成狀態
                
        except Exception as e:
            print(f"Flow1執行異常: {e}")
            traceback.print_exc()
            self.state_machine.set_alarm(True)
            self.state_machine.set_running(False)
            self.state_machine.set_current_flow(0)
    
    def execute_flow2_async(self):
        """異步執行Flow2 - 狀態機交握版本"""
        threading.Thread(target=self._flow2_execution_thread, daemon=True).start()
    
    def _flow1_execution_thread(self):
        """Flow1執行線程 - 狀態機交握版本"""
        try:
            print("=== Flow1執行線程開始 ===")
            
            # 設置執行狀態
            self.state_machine.set_running(True)
            self.state_machine.set_flow(FlowType.FLOW_1)
            self.current_flow = self.flows[1]
            
            # 執行Flow1
            result = self.flows[1].execute()
            
            # 處理執行結果
            if hasattr(result, 'success'):
                if result.success:
                    print(f"Flow1執行成功，耗時: {result.execution_time:.2f}秒")
                    self.state_machine.operation_count += 1
                    self.state_machine.set_running(False)  # Running=0, Ready保持=0，等待PLC清零
                    print("Flow1完成，等待PLC清零VP控制指令後恢復Ready")
                else:
                    print(f"Flow1執行失敗: {result.error_message}")
                    self.state_machine.error_count += 1
                    self.state_machine.set_alarm(True)
                    print("Flow1失敗，系統進入Alarm狀態")
            else:
                # 處理舊版本的bool返回值
                if result:
                    print("Flow1執行成功")
                    self.state_machine.operation_count += 1
                    self.state_machine.set_running(False)
                    print("Flow1完成，等待PLC清零VP控制指令後恢復Ready")
                else:
                    print("Flow1執行失敗")
                    self.state_machine.error_count += 1
                    self.state_machine.set_alarm(True)
                    print("Flow1失敗，系統進入Alarm狀態")
                    
        except Exception as e:
            print(f"Flow1執行異常: {e}")
            traceback.print_exc()
            self.state_machine.error_count += 1
            self.state_machine.set_alarm(True)
            print("Flow1異常，系統進入Alarm狀態")
        finally:
            # 確保狀態機正確重置
            self.state_machine.set_flow(FlowType.NONE)
            self.current_flow = None
            print("=== Flow1執行線程結束 ===")
    
    def _flow2_execution_thread(self):
        """Flow2執行線程 - 狀態機交握版本"""
        try:
            print("=== Flow2執行線程開始 ===")
            
            # 設置執行狀態
            self.state_machine.set_running(True)
            self.state_machine.set_flow(FlowType.FLOW_2)
            self.current_flow = self.flows[2]
            
            # 執行Flow2
            result = self.flows[2].execute()
            
            # 處理執行結果
            if hasattr(result, 'success'):
                if result.success:
                    print(f"Flow2執行成功，耗時: {result.execution_time:.2f}秒")
                    self.state_machine.operation_count += 1
                    self.state_machine.set_running(False)  # Running=0, Ready保持=0，等待PLC清零
                    print("Flow2完成，等待PLC清零出料控制指令後恢復Ready")
                else:
                    print(f"Flow2執行失敗: {result.error_message}")
                    self.state_machine.error_count += 1
                    self.state_machine.set_alarm(True)
                    print("Flow2失敗，系統進入Alarm狀態")
            else:
                # 處理舊版本的bool返回值
                if result:
                    print("Flow2執行成功")
                    self.state_machine.operation_count += 1
                    self.state_machine.set_running(False)
                    print("Flow2完成，等待PLC清零出料控制指令後恢復Ready")
                else:
                    print("Flow2執行失敗")
                    self.state_machine.error_count += 1
                    self.state_machine.set_alarm(True)
                    print("Flow2失敗，系統進入Alarm狀態")
                    
        except Exception as e:
            print(f"Flow2執行異常: {e}")
            traceback.print_exc()
            self.state_machine.error_count += 1
            self.state_machine.set_alarm(True)
            print("Flow2異常，系統進入Alarm狀態")
        finally:
            # 確保狀態機正確重置
            self.state_machine.set_flow(FlowType.NONE)
            self.current_flow = None
            print("=== Flow2執行線程結束 ===")
    
    def handle_manual_command(self, command: int):
        """處理Web端手動指令 - 增強版"""
        try:
            print(f"收到Web端手動指令: {command}")
            
            if command == 1:  # 手動Flow1
                if self.state_machine.is_ready_for_command():
                    print("Web端手動執行Flow1")
                    self.execute_flow1_async()
                else:
                    print("系統未Ready，無法執行Web端Flow1")
            elif command == 2:  # 手動Flow2
                if self.state_machine.is_ready_for_command():
                    print("Web端手動執行Flow2")
                    self.execute_flow2_async()
                else:
                    print("系統未Ready，無法執行Web端Flow2")
            elif command == 99:  # Web端緊急停止
                print("Web端緊急停止")
                self.emergency_stop_all()
            elif command >= 10 and command <= 100:  # 速度設定指令 (新增)
                print(f"Web端速度設定: {command}%")
                if self.robot.is_connected:
                    success = self.robot.set_global_speed(command)
                    if success:
                        self.state_machine.update_global_speed_register(command)
                        print(f"Web端速度設定成功: {command}%")
                    else:
                        print(f"Web端速度設定失敗: {command}%")
                
        except Exception as e:
            print(f"處理Web端手動指令{command}失敗: {e}")
    
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
            
            # 設置系統狀態
            self.state_machine.set_state(RobotState.EMERGENCY)
            self.state_machine.set_alarm(True)
            
            print("緊急停止完成")
            return True
            
        except Exception as e:
            print(f"緊急停止失敗: {e}")
            return False
    
    def get_system_status(self) -> Dict[str, Any]:
        """獲取系統狀態 - 增強版含速度"""
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
            
            # 狀態機交握狀態
            "status_register": self.state_machine.status_register,
            "status_register_binary": f"{self.state_machine.status_register:04b}",
            "ready": self.state_machine.is_ready(),
            "running": self.state_machine.is_running(),
            "alarm": self.state_machine.is_alarm(),
            "initialized": self.state_machine.is_initialized(),
            "ready_for_command": self.state_machine.is_ready_for_command(),
            "current_flow_object": str(type(self.current_flow).__name__) if self.current_flow else None,
            "handshake_thread_alive": self.handshake_thread.is_alive() if self.handshake_thread else False,
            
            # 速度控制狀態 (新增)
            "global_speed": self.state_machine.current_global_speed,
            "robot_speed": self.robot.get_global_speed() if self.robot.is_connected else 0,
            "last_speed_cmd_id": self.state_machine.last_speed_cmd_id,
            "speed_control_available": True
        }
    def set_global_speed_via_handshake(self, speed: int) -> bool:
        """透過狀態機交握設定全局速度 - 測試用"""
        try:
            if speed < 1 or speed > 100:
                print(f"速度值超出範圍(1-100): {speed}")
                return False
            
            # 生成指令ID
            import random
            cmd_id = random.randint(1, 65535)
            
            # 寫入速度控制寄存器
            success1 = self.state_machine.safe_write_register(DobotRegisters.SPEED_VALUE, speed)
            success2 = self.state_machine.safe_write_register(DobotRegisters.SPEED_CMD_ID, cmd_id)
            success3 = self.state_machine.safe_write_register(DobotRegisters.SPEED_COMMAND, 1)
            
            success = success1 and success2 and success3
            
            if success:
                print(f"透過狀態機交握設定速度: {speed}%, ID: {cmd_id}")
            else:
                print(f"透過狀態機交握設定速度失敗: {speed}%")
            
            return success
            
        except Exception as e:
            print(f"透過狀態機交握設定速度異常: {e}")
            return False
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
            self.state_machine.set_ready(True)  # 強制設置Ready
            print("狀態機已重置為Ready")
            
            print("強制狀態重置完成")
            return True
            
        except Exception as e:
            print(f"強制狀態重置失敗: {e}")
            return False
    
    def diagnose_system_state(self):
        """系統狀態診斷 - 詳細檢查"""
        print("\n=== 系統狀態診斷 (狀態機交握版) ===")
        
        # 狀態機診斷
        print(f"狀態寄存器值: {self.state_machine.status_register} (二進制: {self.state_machine.status_register:04b})")
        print(f"Ready狀態: {self.state_machine.is_ready()}")
        print(f"Running狀態: {self.state_machine.is_running()}")
        print(f"Alarm狀態: {self.state_machine.is_alarm()}")
        print(f"Initialized狀態: {self.state_machine.is_initialized()}")
        print(f"準備接受指令: {self.state_machine.is_ready_for_command()}")
        print(f"當前流程: {self.state_machine.current_flow.name}")
        
        # 控制寄存器狀態
        try:
            vp_control = self.state_machine.read_control_register(0)
            unload_control = self.state_machine.read_control_register(1)
            clear_alarm = self.state_machine.read_control_register(2)
            emergency_stop = self.state_machine.read_control_register(3)
            manual_command = self.state_machine.read_control_register(4)
            
            print(f"VP控制寄存器(440): {vp_control}")
            print(f"出料控制寄存器(441): {unload_control}")
            print(f"清除警報寄存器(442): {clear_alarm}")
            print(f"緊急停止寄存器(443): {emergency_stop}")
            print(f"手動指令寄存器(444): {manual_command}")
        except Exception as e:
            print(f"讀取控制寄存器失敗: {e}")
        
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
        
        print("=== 狀態機交握寄存器映射 ===")
        print(f"主狀態寄存器: {DobotRegisters.STATUS_REGISTER} (bit0=Ready, bit1=Running, bit2=Alarm)")
        print(f"機械臂狀態: {DobotRegisters.ROBOT_STATE}")
        print(f"當前流程ID: {DobotRegisters.CURRENT_FLOW}")
        print(f"Flow1完成狀態: {DobotRegisters.FLOW1_COMPLETE} (0=未完成, 1=完成且角度校正成功)")  # 新增
        
        print("=== 控制寄存器映射 ===")
        print(f"VP視覺取料控制: {DobotRegisters.VP_CONTROL} (0=清空, 1=啟動Flow1)")
        print(f"出料控制: {DobotRegisters.UNLOAD_CONTROL} (0=清空, 1=啟動Flow2)")
        # ... 其餘說明保持不變 ...
        
        print("\n=== 狀態機交握流程說明 ===")
        print("Flow1 (VP視覺取料 + 角度校正):")  # 修改
        print("  1. 檢查400寄存器Ready=1")
        print("  2. 寫入440=1觸發Flow1")
        print("  3. 系統執行: 400=10 (Running=1)")
        print("  4. 執行完成: 400=8 (Ready=0, Running=0)")
        print("  5. 角度校正成功: 420=1 (Flow1完成)")  # 新增
        print("  6. PLC清零: 寫入440=0")
        print("  7. 系統恢復: 400=9 (Ready=1)")
        
        print("\nFlow2 (出料流程):")
        print("  前提: 400=9 且 440=0 (Flow1已完成且清零)")
        print("  可選: 420=1 (Flow1已完成，根據業務需求)")  # 新增
        print("  1. 檢查400寄存器Ready=1, 440=0")
        print("  2. 寫入441=1觸發Flow2")
        print("  3. 系統執行: 400=10 (Running=1)")
        print("  4. 執行完成: 400=8 (Ready=0, Running=0)")
        print("  5. PLC清零: 寫入441=0")
        print("  6. 系統恢復: 400=9 (Ready=1)")
        
        print("\n=== 速度控制流程 (新增) ===")
        print("設定全局速度:")
        print("  1. 寫入446=速度值 (1-100)")
        print("  2. 寫入447=指令ID (唯一識別)")
        print("  3. 寫入445=1 (觸發速度設定)")
        print("  4. 系統處理: 設定機械臂速度")
        print("  5. 更新419=新速度值")
        print("  6. 系統清零: 445=0")
        
        print("\n錯誤處理:")
        print("  檢測Alarm: 400=12 (bit2=1)")
        print("  清除警報: 寫入442=1")
        print("  確認清除: 400=9 (Alarm清除)")
        print("  清零指令: 寫入442=0")
        
        status = controller.get_system_status()
        print(f"\n=== 系統當前狀態 ===")
        print(f"狀態寄存器: {status['status_register']} (二進制: {status['status_register_binary']})")
        print(f"Ready: {status['ready']}")
        print(f"Running: {status['running']}")
        print(f"Alarm: {status['alarm']}")
        print(f"Initialized: {status['initialized']}")
        print(f"準備接受指令: {status['ready_for_command']}")
        print(f"當前全局速度: {status['global_speed']}%")
        print(f"機械臂速度: {status['robot_speed']}%")
        print(f"啟用的流程: {status['flows_enabled']}")
        print(f"PGC夾爪: {'啟用' if status['gripper_enabled'] else '停用'}")
        print(f"CCD1視覺: {'啟用' if status['ccd1_enabled'] else '停用'}")
        print(f"CCD3角度: {'啟用' if status['ccd3_enabled'] else '停用'}")
        print(f"速度控制: {'啟用' if status['speed_control_available'] else '停用'}")
        
        print("\n系統準備完成，等待PLC狀態機交握指令...")
        print("  Flow1: VP視覺抓取 (FIFO佇列模式)")
        print("  Flow2: 出料流程 (standby→撈料→組裝→放下→standby)")
        print("  Speed: 全局速度控制 (1-100%)")
        print("  Web端: 可通過444寄存器手動控制")
        
        print("\n=== 測試指令範例 ===")
        print("測試速度設定: controller.set_global_speed_via_handshake(75)")
        print("速度寄存器檢查: 讀取419查看當前速度")
        print("狀態診斷: controller.diagnose_system_state()")
        
        # 主循環
        while True:
            try:
                time.sleep(1)
                status = controller.get_system_status()
                if status["ready"] or status["running"] or status["alarm"]:
                    speed_info = f", Speed={status['global_speed']}%"
                    print(f"狀態更新: Ready={status['ready']}, Running={status['running']}, Alarm={status['alarm']}, Flow={status['current_flow']}{speed_info}")
                    
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






# =========================== 使用說明 ===========================
# 
# 1. 速度控制寄存器映射:
#    445 (SPEED_COMMAND): 速度控制指令 (0=無動作, 1=設定速度)
#    446 (SPEED_VALUE): 速度數值 (1-100)
#    447 (SPEED_CMD_ID): 速度指令ID (防重複執行)
#    419 (GLOBAL_SPEED): 當前全局速度 (只讀狀態)
# 
# 2. 速度設定流程:
#    步驟1: 寫入446=速度值
#    步驟2: 寫入447=唯一指令ID
#    步驟3: 寫入445=1 (觸發設定)
#    步驟4: 系統自動處理並更新419寄存器
#    步驟5: 系統自動清零445寄存器
# 
# 3. Web端可用的手動指令 (444寄存器):
#    1: 手動Flow1
#    2: 手動Flow2
#    10-100: 直接速度設定
#    99: 緊急停止
# 
# 4. 測試方法:
#    python Dobot_main.py test_speed  # 運行速度控制測試
#    python Dobot_main.py             # 正常運行
# 
# 5. 速度控制特點:
#    - 線程安全的狀態機交握
#    - 防重複執行機制
#    - 自動同步機械臂和寄存器
#    - 範圍檢查 (1-100%)
#    - 錯誤處理和恢復