#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1.py - Flow1 VP視覺抓取流程執行器 (DR專案版 - 整合快速角度檢測)
- 使用外部點位檔案，禁止硬編碼座標
- 從AutoProgram模組讀取座標 (地址1350-1354)
- 支援sync_enable控制高速/精準兩種模式
- 整合Flow2的快速角度檢測功能
- 支援帶參數的運動控制
"""

import time
import os
import json
import threading
import queue
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

# 導入Modbus TCP Client (適配pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    MODBUS_AVAILABLE = False


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


@dataclass
class FlowResult:
    """流程執行結果"""
    success: bool
    error_message: str = ""
    execution_time: float = 0.0
    steps_completed: int = 0
    total_steps: int = 0
    detected_position: Optional[Dict[str, float]] = None
    target_angle: Optional[float] = None
    command_angle: Optional[float] = None
    angle_acquisition_success: bool = False
    extra_data: Dict[str, Any] = None

    def __post_init__(self):
        if self.extra_data is None:
            self.extra_data = {}


class FlowStatus(Enum):
    """流程狀態"""
    READY = "ready"
    RUNNING = "running" 
    PAUSED = "paused"
    COMPLETED = "completed"
    ERROR = "error"


class AutoProgramInterface:
    """AutoProgram座標交握接口 - DR專案專用"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # AutoProgram座標寄存器映射 (1350-1354)
        self.REGISTERS = {
            'AP_COORDS_AVAILABLE': 1350,     # AutoProgram座標可用標誌 (0=無, 1=有)
            'AP_TARGET_X_HIGH': 1351,        # AutoProgram座標X高位
            'AP_TARGET_X_LOW': 1352,         # AutoProgram座標X低位  
            'AP_TARGET_Y_HIGH': 1353,        # AutoProgram座標Y高位
            'AP_TARGET_Y_LOW': 1354,         # AutoProgram座標Y低位
        }
        
        # 初始連接
        self.ensure_connection()
    
    def ensure_connection(self) -> bool:
        """確保連接已建立"""
        if self.connected and self.modbus_client:
            try:
                # 快速連接測試
                test_result = self.modbus_client.read_holding_registers(self.REGISTERS['AP_COORDS_AVAILABLE'], 1, slave=1)
                if not test_result.isError():
                    return True
            except:
                pass
        
        # 需要重新連接
        return self._establish_connection()
    
    def _establish_connection(self) -> bool:
        """建立連接"""
        try:
            if self.modbus_client:
                try:
                    self.modbus_client.close()
                except:
                    pass
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=2.0
            )
            
            if self.modbus_client.connect():
                self.connected = True
                print("✓ AutoProgram連接已建立")
                return True
            else:
                self.connected = False
                print("✗ AutoProgram連接失敗")
                return False
                
        except Exception as e:
            self.connected = False
            print(f"AutoProgram連接異常: {e}")
            return False
    
    def disconnect(self):
        """斷開連接"""
        if self.modbus_client and self.connected:
            try:
                self.modbus_client.close()
            except:
                pass
        self.connected = False
        self.modbus_client = None
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器"""
        if not self.ensure_connection() or register_name not in self.REGISTERS:
            return None
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            
            if not result.isError():
                return result.registers[0]
            else:
                return None
                
        except Exception:
            self.connected = False
            return None
    
    def check_autoprogram_coords_available(self) -> bool:
        """檢查AutoProgram座標是否可用"""
        coords_available = self.read_register('AP_COORDS_AVAILABLE')
        print(f"AutoProgram座標可用檢查: 寄存器1350={coords_available}")
        return coords_available == 1
    
    def read_autoprogram_coordinates(self) -> Optional[Dict[str, float]]:
        """讀取AutoProgram座標"""
        try:
            # 檢查座標是否可用
            if not self.check_autoprogram_coords_available():
                print("AutoProgram: 座標不可用")
                return None
            
            # 讀取32位座標寄存器 - 逐個讀取確保可靠性
            try:
                x_high = self.read_register('AP_TARGET_X_HIGH')
                x_low = self.read_register('AP_TARGET_X_LOW')
                y_high = self.read_register('AP_TARGET_Y_HIGH')
                y_low = self.read_register('AP_TARGET_Y_LOW')
                
                print(f"AutoProgram原始寄存器讀取:")
                print(f"  X_HIGH(1351)={x_high}, X_LOW(1352)={x_low}")
                print(f"  Y_HIGH(1353)={y_high}, Y_LOW(1354)={y_low}")
                
                if any(val is None for val in [x_high, x_low, y_high, y_low]):
                    print("AutoProgram: 座標寄存器讀取失敗")
                    return None
                
            except Exception as e:
                print(f"AutoProgram座標寄存器讀取異常: {e}")
                return None
            
            # 32位合併並轉換精度
            world_x_int = (x_high << 16) | x_low
            world_y_int = (y_high << 16) | y_low
            
            print(f"32位合併結果:")
            print(f"  X_INT={world_x_int}, Y_INT={world_y_int}")
            
            # 處理負數 (補碼轉換)
            if world_x_int >= 2**31:
                world_x_int -= 2**32
            if world_y_int >= 2**31:
                world_y_int -= 2**32
                
            print(f"補碼處理後:")
            print(f"  X_INT={world_x_int}, Y_INT={world_y_int}")
            
            # 恢復精度 (÷100)
            world_x = world_x_int / 100.0
            world_y = world_y_int / 100.0
            
            print(f"AutoProgram座標讀取成功: X={world_x:.2f}, Y={world_y:.2f}")
            
            # 驗證座標不為零
            if world_x == 0.0 and world_y == 0.0:
                print("⚠️ AutoProgram座標為零，可能是無效數據")
                return None
            
            return {
                'x': world_x,
                'y': world_y,
                'source': 'autoprogram'
            }
            
        except Exception as e:
            print(f"讀取AutoProgram座標異常: {e}")
            return None


class PointsManager:
    """點位管理器 - 支援cartesian和pose格式"""
    
    def __init__(self, points_file: str = "saved_points/robot_points.json"):
        if not os.path.isabs(points_file):
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.points_file = os.path.join(current_dir, points_file)
        else:
            self.points_file = points_file
        self.points: Dict[str, RobotPoint] = {}
        
    def load_points(self) -> bool:
        """載入點位數據 - 支援cartesian和pose格式"""
        try:
            print(f"嘗試載入點位檔案: {self.points_file}")
            
            if not os.path.exists(self.points_file):
                print(f"錯誤: 點位檔案不存在: {self.points_file}")
                return False
                
            with open(self.points_file, "r", encoding="utf-8") as f:
                points_list = json.load(f)
            
            self.points.clear()
            for point_data in points_list:
                try:
                    # 支援兩種格式：pose 或 cartesian
                    if "pose" in point_data:
                        pose_data = point_data["pose"]
                    elif "cartesian" in point_data:
                        pose_data = point_data["cartesian"]
                    else:
                        print(f"點位 {point_data.get('name', 'unknown')} 缺少座標數據")
                        continue
                    
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
            print(f"錯誤: 載入點位數據失敗: {e}")
            return False
    
    def get_point(self, name: str) -> Optional[RobotPoint]:
        """獲取指定點位"""
        return self.points.get(name)
    
    def list_points(self) -> List[str]:
        """列出所有點位名稱"""
        return list(self.points.keys())
    
    def has_point(self, name: str) -> bool:
        """檢查是否存在指定點位"""
        return name in self.points


class DrFlow1VisionPickExecutor:
    """Flow1: VP視覺抓取流程執行器 - DR專案整合快速角度檢測版"""
    
    def __init__(self, sync_enable: bool = False):
        # 核心組件 (通過initialize方法設置)
        self.robot = None
        self.motion_state_machine = None
        self.external_modules = {}
        
        # 流程配置
        self.flow_id = 1
        self.flow_name = "VP視覺抓取流程(整合角度檢測)"
        self.status = FlowStatus.READY
        self.current_step = 0
        self.start_time = 0.0
        self.last_error = ""
        
        # 同步控制參數
        self.sync_enable = sync_enable
        
        # 流程參數
        self.PICKUP_HEIGHT = 147.52  # 夾取高度
        
        # 角度檢測參數 (新增 - 從Flow2移植)
        self.target_angle = None      # 從AngleHighLevel獲取的角度
        self.command_angle = None     # 計算後的指令角度 (target_angle + 45)
        self.angle_acquisition_success = False
        self.ANGLE_OFFSET = 45.0      # 角度偏移量
        
        # 優化參數 (從Flow2移植)
        self.angle_detection_timeout = 3.0  # 角度檢測超時時間
        self.use_fast_angle_detection = True  # 啟用快速角度檢測
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # 初始化AutoProgram接口
        self.autoprogram_interface = AutoProgramInterface()
        
        # Flow1完成狀態管理
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.FLOW1_COMPLETE_REGISTER = 1204  # Flow1完成狀態寄存器
        
        # Flow1需要的點位名稱 (修改為新流程)
        self.REQUIRED_POINTS = [
            "standby",       # 待機點
            "VP_TOPSIDE",    # VP震動盤上方點
            "Rotate_top",    # 翻轉頂部點
            "Rotate_down",   # 翻轉底部點
            "Rotate_V2",     # 翻轉預備點 (角度檢測位置)
            "put_asm_top"    # 組裝頂部位置
        ]
        
        # 建構流程步驟
        self.motion_steps = []
        self.total_steps = 0
        
        # 嘗試載入點位檔案
        self._load_and_validate_points()
        
        # 只有點位載入成功才建構流程步驟
        if self.points_loaded:
            self.build_flow_steps()
        
        print(f"✓ DrFlow1VisionPickExecutor初始化完成 (整合角度檢測版)")
        print(f"  sync控制: {'啟用' if sync_enable else '停用'}")
        print(f"  座標來源: AutoProgram模組 (地址1350-1354)")
        print(f"  角度檢測: 超時{self.angle_detection_timeout}秒")
        print(f"  完成狀態寄存器: {self.FLOW1_COMPLETE_REGISTER}")
        
    def initialize(self, robot, motion_state_machine, external_modules):
        """初始化Flow執行器"""
        self.robot = robot
        self.motion_state_machine = motion_state_machine
        self.external_modules = external_modules
        
        print(f"✓ Flow1執行器初始化完成")
        print(f"  可用模組: Gripper={self.external_modules.get('gripper') is not None}, "
              f"Angle={self.external_modules.get('angle') is not None}")
        print(f"  座標交握: AutoProgram寄存器1350-1354")
        print(f"  同步控制: {'啟用' if self.sync_enable else '停用'}")
        
    def _load_and_validate_points(self):
        """載入並驗證點位檔案"""
        print("Flow1正在載入外部點位檔案...")
        
        # 載入點位檔案
        if not self.points_manager.load_points():
            print("錯誤: 無法載入點位檔案，Flow1無法執行")
            self.points_loaded = False
            return
        
        # 檢查所有必要點位是否存在
        missing_points = []
        for point_name in self.REQUIRED_POINTS:
            if not self.points_manager.has_point(point_name):
                missing_points.append(point_name)
        
        if missing_points:
            print(f"錯誤: 缺少必要點位: {missing_points}")
            print(f"可用點位: {self.points_manager.list_points()}")
            self.points_loaded = False
            return
        
        print("✓ 所有必要點位載入成功")
        self.points_loaded = True
        
    def build_flow_steps(self):
        """建構Flow1步驟 - 整合角度檢測版"""
        if not self.points_loaded:
            print("警告: 點位未載入，無法建構流程步驟")
            self.motion_steps = []
            self.total_steps = 0
            return
            
        # 定義新的流程步驟
        self.motion_steps = [
            # 1. AutoProgram座標讀取
            {'type': 'autoprogram_coordinates_read', 'params': {}},
            
            # 2. 初始準備
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J', 'sync': False}},
            {'type': 'gripper_close', 'params': {}},
            
            # 3. 移動到VP上方檢測位置
            {'type': 'move_to_point', 'params': {'point_name': 'VP_TOPSIDE', 'move_type': 'J', 'sync': False}},
            
            # 4. 移動到檢測物件位置（與VP_TOPSIDE同高）
            {'type': 'move_to_detected_position_high', 'params': {'sync': False}},
            {'type': 'move_to_detected_position_low', 'params': {'sync': True}},
            {'type': 'gripper_smart_release', 'params': {'position': 205}},
            {'type': 'move_to_point', 'params': {'point_name': 'VP_TOPSIDE', 'move_type': 'L', 'sync': False}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J', 'sync': False}},
            
            # 5. 翻轉站序列
            {'type': 'move_to_point', 'params': {'point_name': 'Rotate_top', 'move_type': 'J', 'sync': False}},
            {'type': 'move_to_point', 'params': {'point_name': 'Rotate_down', 'move_type': 'J', 'sync': True}},
            {'type': 'gripper_close', 'params': {}},
            {'type': 'move_to_point', 'params': {'point_name': 'Rotate_top', 'move_type': 'J', 'sync': False}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J', 'sync': True}},
            
            # 6. 快速角度檢測 (從Flow2移植)
            {'type': 'angle_detection_fast', 'params': {}},
            
            # 7. 移動到組裝位置
            {'type': 'move_to_point', 'params': {'point_name': 'Rotate_top', 'move_type': 'J', 'sync': False}},
            {'type': 'move_to_point', 'params': {'point_name': 'Rotate_down', 'move_type': 'J', 'sync': True}},
            {'type': 'gripper_smart_release', 'params': {'position': 205}},
            {'type': 'move_to_point', 'params': {'point_name': 'Rotate_top', 'move_type': 'J', 'sync': False}},
            
            # 8. 帶J4角度控制的組裝位置
            {'type': 'move_to_point_with_j4', 'params': {'point_name': 'put_asm_top', 'move_type': 'J', 'sync': True}},
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"Flow1新流程步驟建構完成，共{self.total_steps}步")
        print("✓ 新增功能: 快速角度檢測 + J4角度控制組裝")
    
    def execute(self) -> FlowResult:
        """執行Flow1主邏輯 - 整合角度檢測版"""
        # 檢查點位是否已載入
        if not self.points_loaded:
            return FlowResult(
                success=False,
                error_message="點位檔案載入失敗，無法執行Flow1",
                execution_time=0.0,
                steps_completed=0,
                total_steps=0
            )
        
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        
        # 重置角度檢測參數
        self.target_angle = None
        self.command_angle = None
        self.angle_acquisition_success = False
        
        # 檢查初始化
        if not self.robot or not self.robot.is_connected:
            return FlowResult(
                success=False,
                error_message="機械臂未連接或未初始化",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        # 檢查AutoProgram連接
        if not self.autoprogram_interface.ensure_connection():
            return FlowResult(
                success=False,
                error_message="AutoProgram連接失敗",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        detected_position = None
        
        try:
            for step in self.motion_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                print(f"Flow1 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 更新進度到motion_state_machine
                if self.motion_state_machine:
                    progress = int((self.current_step / self.total_steps) * 100)
                    self.motion_state_machine.set_progress(progress)
                
                # 執行步驟
                success = False
                
                if step['type'] == 'move_to_point':
                    success = self._execute_move_to_point_with_parameters(step['params'])
                elif step['type'] == 'move_to_point_with_j4':
                    success = self._execute_move_to_point_with_j4(step['params'])
                elif step['type'] == 'gripper_close':
                    success = self._execute_gripper_close()
                elif step['type'] == 'gripper_smart_release':
                    success = self._execute_gripper_smart_release(step['params'])
                elif step['type'] == 'autoprogram_coordinates_read':
                    detected_position = self._execute_autoprogram_coordinates_read()
                    success = detected_position is not None
                elif step['type'] == 'move_to_detected_position_high':
                    success = self._execute_move_to_detected_position_with_parameters(detected_position, step['params'])
                elif step['type'] == 'move_to_detected_position_low':
                    success = self._execute_move_to_detected_position_with_parameters(detected_position, step['params'])
                elif step['type'] == 'angle_detection_fast':
                    success = self._execute_angle_detection_fast()
                else:
                    print(f"未知步驟類型: {step['type']}")
                    success = False
                
                if not success:
                    self.status = FlowStatus.ERROR
                    return FlowResult(
                        success=False,
                        error_message=f"步驟 {step['type']} 執行失敗",
                        execution_time=time.time() - self.start_time,
                        steps_completed=self.current_step,
                        total_steps=self.total_steps,
                        target_angle=self.target_angle,
                        command_angle=self.command_angle,
                        angle_acquisition_success=self.angle_acquisition_success
                    )
                
                self.current_step += 1
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            # 設置Flow1完成狀態
            self._safe_set_flow1_complete_status(True)
            
            # 最終進度設置
            if self.motion_state_machine:
                self.motion_state_machine.set_progress(100)
            
            print(f"✓ Flow1執行完成！總耗時: {execution_time:.2f}秒")
            if self.angle_acquisition_success:
                print(f"角度控制: 目標角度={self.target_angle:.2f}°, 指令角度={self.command_angle:.2f}°")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                detected_position=detected_position,
                target_angle=self.target_angle,
                command_angle=self.command_angle,
                angle_acquisition_success=self.angle_acquisition_success
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            return FlowResult(
                success=False,
                error_message=f"Flow1執行異常: {str(e)}",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                target_angle=self.target_angle,
                command_angle=self.command_angle,
                angle_acquisition_success=self.angle_acquisition_success
            )
    
    def _execute_move_to_point_with_parameters(self, params: Dict[str, Any]) -> bool:
        """執行移動到點位 - 支援參數版本 (從paste3.txt移植並簡化)"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                print(f"錯誤: 點位管理器中找不到點位: {point_name}")
                return False
            
            print(f"移動到點位 {point_name} ({move_type})")
            print(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{point.j4:.1f})")
            print(f"  笛卡爾座標: ({point.x:.2f}, {point.y:.2f}, {point.z:.2f}, {point.r:.2f})")
            
            # 執行移動
            success = False
            if move_type in ['J', 'JointMovJ']:
                # 使用關節角度運動
                success = self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4)
            elif move_type in ['L', 'MovL']:
                # 直線運動使用笛卡爾座標
                success = self.robot.move_l(point.x, point.y, point.z, point.r)
            else:
                print(f"未支援的移動類型: {move_type}")
                return False
            
            # Sync控制
            sync_enabled = params.get('sync', self.sync_enable)
            if success and sync_enabled:
                self.robot.sync()
                print(f"  ✓ 移動到 {point_name} 成功 ({move_type}) (含Sync)")
            elif success:
                print(f"  ✓ 移動到 {point_name} 成功 ({move_type})")
            else:
                print(f"  ✗ 移動到 {point_name} 失敗")
                
            return success
                
        except Exception as e:
            print(f"移動到點位失敗: {e}")
            return False
    
    def _execute_move_to_point_with_j4(self, params: Dict[str, Any]) -> bool:
        """執行移動到點位並使用計算的J4角度 (從Flow2移植)"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                print(f"錯誤: 點位管理器中找不到點位: {point_name}")
                return False
            
            # 使用計算的command_angle作為J4值
            if self.command_angle is not None:
                j4_value = self.command_angle
                print(f"移動到點位 {point_name} (使用計算J4角度)")
                print(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{j4_value:.1f})")
                print(f"  計算角度: target={self.target_angle:.1f}° + offset={self.ANGLE_OFFSET}° = command={j4_value:.1f}°")
            else:
                # 沒有角度數據，使用原始J4值
                j4_value = point.j4
                print(f"移動到點位 {point_name} (使用原始J4角度)")
                print(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{j4_value:.1f})")
                print(f"  ⚠️ 未獲取角度數據，使用原始J4值")
            
            success = False
            if move_type in ['J', 'JointMovJ']:
                # 使用關節角度運動，J4使用計算角度
                success = self.robot.joint_move_j(point.j1, point.j2, point.j3, j4_value)
            else:
                print(f"J4角度控制僅支援關節運動(J)，當前類型: {move_type}")
                return False
            
            # Sync控制
            sync_enabled = params.get('sync', self.sync_enable)
            if success and sync_enabled:
                self.robot.sync()
                print(f"  ✓ 移動到 {point_name} 成功 (J4:{j4_value:.1f}°) (含Sync)")
            elif success:
                print(f"  ✓ 移動到 {point_name} 成功 (J4:{j4_value:.1f}°)")
                
            return success
                
        except Exception as e:
            print(f"移動到點位(J4角度控制)失敗: {e}")
            return False
    
    def _execute_angle_detection_fast(self) -> bool:
        """執行AngleHighLevel角度檢測 - 快速版本 (從Flow2移植)"""
        try:
            print("  [快速角度檢測] 開始檢測...")
            detection_start_time = time.time()
            
            # 優先使用external_modules中的angle模組
            angle_controller = self.external_modules.get('angle')
            
            if not angle_controller:
                print("  [快速角度檢測] ✗ 角度模組未連接，快速使用預設值")
                self._set_default_angle()
                return True
            
            print("  [快速角度檢測] ✓ 使用外部模組中的角度API")
            
            # 檢查角度控制器連接狀態
            if hasattr(angle_controller, 'connected') and not angle_controller.connected:
                print("  [快速角度檢測] ⚠️ 角度控制器未連接，嘗試快速重連...")
                if not angle_controller.connect():
                    print("  [快速角度檢測] ✗ 快速重連失敗，使用預設值")
                    self._set_default_angle()
                    return True
            
            # 執行快速角度檢測
            print("  [快速角度檢測] 執行CCD3角度檢測(DR模式)...")
            
            try:
                # 使用超時機制執行角度檢測
                detection_result = self._execute_angle_detection_with_timeout(angle_controller)
                
                if detection_result is None:
                    print(f"  [快速角度檢測] ✗ 檢測超時({self.angle_detection_timeout}秒)，使用預設值")
                    self._set_default_angle()
                    return True
                
                detection_time = time.time() - detection_start_time
                
                if (hasattr(detection_result, 'result') and 
                    detection_result.result.value == "SUCCESS" and 
                    detection_result.target_angle is not None):
                    
                    self.target_angle = detection_result.target_angle
                    self.command_angle = self.target_angle + self.ANGLE_OFFSET
                    self.angle_acquisition_success = True
                    
                    print(f"  [快速角度檢測] ✓ 角度檢測成功 (耗時: {detection_time:.2f}秒):")
                    print(f"    目標角度: {self.target_angle:.2f}°")
                    print(f"    指令角度: {self.command_angle:.2f}°")
                    
                    return True
                else:
                    error_msg = getattr(detection_result, 'message', '未知錯誤') if detection_result else '檢測失敗'
                    print(f"  [快速角度檢測] ✗ 檢測失敗: {error_msg} (耗時: {detection_time:.2f}秒)")
                    self._set_default_angle()
                    return True
                    
            except Exception as detection_error:
                detection_time = time.time() - detection_start_time
                print(f"  [快速角度檢測] ✗ 檢測異常: {detection_error} (耗時: {detection_time:.2f}秒)")
                self._set_default_angle()
                return True
                
        except Exception as e:
            detection_time = time.time() - detection_start_time
            print(f"  [快速角度檢測] ✗ 系統異常: {e} (耗時: {detection_time:.2f}秒)")
            self._set_default_angle()
            return True
    
    def _execute_angle_detection_with_timeout(self, angle_controller) -> Optional[Any]:
        """帶超時的角度檢測執行 (從Flow2移植)"""
        import threading
        import queue
        
        result_queue = queue.Queue()
        
        def detection_thread():
            try:
                result = angle_controller.detect_angle(detection_mode=1)  # DR模式
                result_queue.put(result)
            except Exception as e:
                result_queue.put(None)
        
        # 啟動檢測線程
        thread = threading.Thread(target=detection_thread, daemon=True)
        thread.start()
        
        # 等待結果或超時
        try:
            result = result_queue.get(timeout=self.angle_detection_timeout)
            return result
        except queue.Empty:
            print(f"  [快速角度檢測] 檢測超時 ({self.angle_detection_timeout}秒)")
            return None
    
    def _set_default_angle(self):
        """設置預設角度值 (從Flow2移植)"""
        self.target_angle = 0.0
        self.command_angle = self.target_angle + self.ANGLE_OFFSET
        self.angle_acquisition_success = False
        print(f"  [快速角度檢測] 使用預設角度: target={self.target_angle}°, command={self.command_angle}°")
    
    def _execute_move_to_detected_position_with_parameters(self, detected_position: Optional[Dict[str, float]], params: Dict[str, Any]) -> bool:
        """移動到檢測位置 - 支援參數版本"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            # 取得VP_TOPSIDE點位的Z高度和R值
            vp_topside_point = self.points_manager.get_point('VP_TOPSIDE')
            if not vp_topside_point:
                print("錯誤: 無法獲取VP_TOPSIDE點位")
                return False
            
            # 決定目標高度 (檢測高度或夾取高度)
            target_height = vp_topside_point.z if 'high' in str(params) else self.PICKUP_HEIGHT
            
            print(f"移動到檢測位置:")
            print(f"  AutoProgram座標XY: ({detected_position['x']:.2f}, {detected_position['y']:.2f})")
            print(f"  目標高度Z: {target_height:.2f}")
            print(f"  繼承R: {vp_topside_point.r:.2f} (VP_TOPSIDE角度)")
            
            # 執行移動
            success = self.robot.move_l(
                detected_position['x'],
                detected_position['y'],
                target_height,
                vp_topside_point.r
            )
            
            # Sync控制
            sync_enabled = params.get('sync', self.sync_enable)
            if success and sync_enabled:
                self.robot.sync()
                print(f"  ✓ 移動到檢測位置完成 (含Sync)")
            elif success:
                print(f"  ✓ 移動到檢測位置完成")
            else:
                print(f"  ✗ 移動到檢測位置失敗")
                
            return success
                
        except Exception as e:
            print(f"移動到檢測位置失敗: {e}")
            return False
    
    def _execute_gripper_close(self) -> bool:
        """執行夾爪關閉"""
        try:
            gripper_api = self.external_modules.get('gripper')
            if gripper_api:
                return gripper_api.quick_close()
            else:
                print("夾爪API未初始化")
                return False
        except Exception as e:
            print(f"夾爪關閉失敗: {e}")
            return False
    
    def _execute_gripper_smart_release(self, params: Dict[str, Any]) -> bool:
        """執行夾爪智能撐開"""
        try:
            position = params.get('position', 370)
            print(f"夾爪智能撐開到位置: {position}")
            
            gripper_api = self.external_modules.get('gripper')
            if not gripper_api:
                print("夾爪API未初始化")
                return False
            
            # 執行智能撐開操作
            success = gripper_api.smart_release(position)
            
            if success:
                print(f"✓ 夾爪智能撐開指令發送成功")
                
                # 等待夾爪撐開操作完全完成
                print("  等待夾爪撐開動作完成...")
                time.sleep(1.5)  # 等待1.5秒確保夾爪完全撐開
                
                print(f"✓ 夾爪智能撐開完成 - 位置{position}")
                return True
            else:
                print(f"✗ 夾爪智能撐開失敗")
                return False
                
        except Exception as e:
            print(f"夾爪智能撐開異常: {e}")
            return False
    
    def _execute_autoprogram_coordinates_read(self) -> Optional[Dict[str, float]]:
        """執行AutoProgram座標讀取"""
        try:
            print("開始從AutoProgram讀取預先驗證的座標...")
            print(f"AutoProgram寄存器地址: {self.autoprogram_interface.REGISTERS}")
            
            # 等待AutoProgram提供座標 (最多等待10秒)
            timeout = 10.0
            start_time = time.time()
            
            coord_data = None
            while time.time() - start_time < timeout:
                coord_data = self.autoprogram_interface.read_autoprogram_coordinates()
                if coord_data:
                    break
                print(f"  等待AutoProgram座標... ({time.time() - start_time:.1f}s)")
                time.sleep(0.5)  # 500ms檢查間隔
            
            if not coord_data:
                print("等待AutoProgram座標超時或讀取失敗")
                return None
            
            # 獲取VP_TOPSIDE點位的Z高度和R值
            vp_topside_point = self.points_manager.get_point('VP_TOPSIDE')
            if not vp_topside_point:
                print("錯誤: 無法獲取VP_TOPSIDE點位")
                return None
            
            detected_pos = {
                'x': coord_data['x'],                # AutoProgram提供的X座標
                'y': coord_data['y'],                # AutoProgram提供的Y座標
                'z': vp_topside_point.z,             # 繼承VP_TOPSIDE的Z高度
                'r': vp_topside_point.r,             # 繼承VP_TOPSIDE的R角度
                'source': 'autoprogram'
            }
            
            print(f"AutoProgram座標讀取成功:")
            print(f"  預先驗證座標: ({detected_pos['x']:.2f}, {detected_pos['y']:.2f})")
            print(f"  繼承VP_TOPSIDE - Z:{detected_pos['z']:.2f}, R:{detected_pos['r']:.2f}")
            print(f"  座標來源: AutoProgram模組 (已通過保護區域和重複檢查)")
            
            # 最終驗證座標不為零
            if detected_pos['x'] == 0.0 and detected_pos['y'] == 0.0:
                print("✗ 最終驗證失敗: AutoProgram座標為零")
                return None
            
            return detected_pos
            
        except Exception as e:
            print(f"AutoProgram座標讀取異常: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def _safe_set_flow1_complete_status(self, complete: bool) -> bool:
        """安全設置Flow1完成狀態"""
        try:
            print(f"[Flow1] 設置Flow1完成狀態: {complete}")
            
            # 方法1: 透過motion_state_machine設置 (優先且推薦)
            if self.motion_state_machine:
                try:
                    self.motion_state_machine.set_flow_complete(1, complete)
                    print(f"[Flow1] ✓ 透過狀態機設置Flow1完成狀態: {complete}")
                    return True
                except Exception as e:
                    print(f"[Flow1] ✗ 狀態機設置失敗: {e}")
            
            # 方法2: 直接Modbus寫入 (備用方案)
            if not self.modbus_client:
                self.modbus_client = ModbusTcpClient(host="127.0.0.1", port=502, timeout=2.0)
                if not self.modbus_client.connect():
                    print("[Flow1] ✗ Flow1完成狀態設置失敗：無法連接Modbus")
                    return False
            
            value = 1 if complete else 0
            result = self.modbus_client.write_register(
                address=self.FLOW1_COMPLETE_REGISTER, 
                value=value
            )
            
            if hasattr(result, 'isError') and result.isError():
                print(f"[Flow1] ✗ Flow1完成狀態直接寫入失敗: {result}")
                return False
            else:
                print(f"[Flow1] ✓ Flow1完成狀態直接寫入成功: 地址{self.FLOW1_COMPLETE_REGISTER} = {value}")
                return True
                
        except Exception as e:
            print(f"[Flow1] ✗ 設置Flow1完成狀態異常: {e}")
            return False
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        print("Flow1已暫停")
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("Flow1已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow"""
        self.status = FlowStatus.ERROR
        print("Flow1已停止")
        return True
    
    def get_target_angle(self) -> Optional[float]:
        """供Flow2調用：獲取target_angle"""
        return self.target_angle
        
    def get_command_angle(self) -> Optional[float]:
        """供Flow2調用：獲取command_angle"""
        return self.command_angle
    
    def has_valid_angle_data(self) -> bool:
        """供Flow2調用：檢查角度數據是否有效"""
        return (self.angle_acquisition_success and 
                self.target_angle is not None and 
                self.command_angle is not None)
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
    
    def is_ready(self) -> bool:
        """檢查Flow1是否準備好執行"""
        return (self.points_loaded and 
                self.total_steps > 0 and 
                self.autoprogram_interface.connected)
    
    def cleanup(self):
        """清理資源"""
        if hasattr(self, 'autoprogram_interface'):
            self.autoprogram_interface.disconnect()
        
        # 關閉Flow1的Modbus連接
        if hasattr(self, 'modbus_client') and self.modbus_client:
            try:
                self.modbus_client.close()
                print("✓ Flow1 Modbus連接已關閉")
            except:
                pass


# 兼容性別名
class Flow1Executor(DrFlow1VisionPickExecutor):
    """Flow1執行器 - 兼容性包裝器"""
    pass