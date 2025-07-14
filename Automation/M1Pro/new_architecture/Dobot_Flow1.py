#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1.py - Flow1 VP視覺抓取流程執行器 (DR專案)
- 使用外部點位檔案，禁止硬編碼座標
- 整合AutoFeeding模組座標交握機制 (寄存器940-945)
- 支援sync_enable控制高速/精準兩種模式
"""

import time
import os
import json
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


class AutoFeedingInterface:
    """AutoFeeding座標交握接口 - DR專案專用"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # AutoFeeding交握寄存器映射 (基地址940-945)
        self.REGISTERS = {
            'DR_F_AVAILABLE': 940,      # DR_F可用標誌 (0=無, 1=有)
            'TARGET_X_HIGH': 941,       # DR_F座標X高位
            'TARGET_X_LOW': 942,        # DR_F座標X低位  
            'TARGET_Y_HIGH': 943,       # DR_F座標Y高位
            'TARGET_Y_LOW': 944,        # DR_F座標Y低位
            'COORDS_READ_CONFIRM': 945, # 座標已讀取確認 (Flow1設置)
        }
        
        # 初始連接
        self.ensure_connection()
    
    def ensure_connection(self) -> bool:
        """確保連接已建立"""
        if self.connected and self.modbus_client:
            try:
                # 快速連接測試
                test_result = self.modbus_client.read_holding_registers(self.REGISTERS['DR_F_AVAILABLE'], 1, slave=1)
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
                print("✓ AutoFeeding連接已建立")
                return True
            else:
                self.connected = False
                print("✗ AutoFeeding連接失敗")
                return False
                
        except Exception as e:
            self.connected = False
            print(f"AutoFeeding連接異常: {e}")
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
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器"""
        if not self.ensure_connection() or register_name not in self.REGISTERS:
            return False
        
        try:
            address = self.REGISTERS[register_name]
            result = self.modbus_client.write_register(address, value, slave=1)
            return not result.isError()
        except Exception:
            self.connected = False
            return False
    
    def check_dr_f_available(self) -> bool:
        """檢查DR_F是否可用"""
        dr_f_available = self.read_register('DR_F_AVAILABLE')
        return dr_f_available == 1
    
    def read_dr_f_coordinates(self) -> Optional[Dict[str, float]]:
        """讀取AutoFeeding DR_F座標"""
        try:
            # 檢查DR_F是否可用
            if not self.check_dr_f_available():
                print("AutoFeeding: DR_F不可用")
                return None
            
            # 讀取32位座標寄存器
            try:
                result = self.modbus_client.read_holding_registers(
                    self.REGISTERS['TARGET_X_HIGH'], 4, slave=1
                )
                if result.isError():
                    return None
                
                x_high, x_low, y_high, y_low = result.registers
                
            except Exception:
                # 批量讀取失敗，回退到單個讀取
                x_high = self.read_register('TARGET_X_HIGH') or 0
                x_low = self.read_register('TARGET_X_LOW') or 0
                y_high = self.read_register('TARGET_Y_HIGH') or 0
                y_low = self.read_register('TARGET_Y_LOW') or 0
            
            # 32位合併並轉換精度
            world_x_int = (x_high << 16) | x_low
            world_y_int = (y_high << 16) | y_low
            
            # 處理負數 (補碼轉換)
            if world_x_int >= 2**31:
                world_x_int -= 2**32
            if world_y_int >= 2**31:
                world_y_int -= 2**32
            
            # 恢復精度 (÷100)
            world_x = world_x_int / 100.0
            world_y = world_y_int / 100.0
            
            print(f"AutoFeeding座標讀取成功: X={world_x:.2f}, Y={world_y:.2f}")
            return {
                'x': world_x,
                'y': world_y,
                'source': 'autofeeding'
            }
            
        except Exception as e:
            print(f"讀取AutoFeeding座標異常: {e}")
            return None
    
    def confirm_coordinates_read(self) -> bool:
        """確認座標已讀取 (設置945=1)"""
        return self.write_register('COORDS_READ_CONFIRM', 1)


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
    """Flow1: VP視覺抓取流程執行器 - DR專案AutoFeeding整合版"""
    
    def __init__(self, sync_enable: bool = False):
        # 核心組件 (通過initialize方法設置)
        self.robot = None
        self.motion_state_machine = None
        self.external_modules = {}
        
        # 流程配置
        self.flow_id = 1
        self.flow_name = "VP視覺抓取流程"
        self.status = FlowStatus.READY
        self.current_step = 0
        self.start_time = 0.0
        self.last_error = ""
        
        # 同步控制參數 - 新增
        self.sync_enable = sync_enable
        
        # 流程參數
        self.PICKUP_HEIGHT = 137.52  # 夾取高度
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # 初始化AutoFeeding接口
        self.autofeeding_interface = AutoFeedingInterface()
        
        # Flow1需要的點位名稱
        self.REQUIRED_POINTS = [
            "standby",      # 待機點
            "VP_TOPSIDE",   # VP震動盤上方點
            "Rotate_V2",    # 翻轉預備點
            "Rotate_top",   # 翻轉頂部點
            "Rotate_down"   # 翻轉底部點
        ]
        
        # 建構流程步驟
        self.motion_steps = []
        self.total_steps = 0
        
        # 嘗試載入點位檔案
        self._load_and_validate_points()
        
        # 只有點位載入成功才建構流程步驟
        if self.points_loaded:
            self.build_flow_steps()
        
        print(f"✓ DrFlow1VisionPickExecutor初始化完成 (sync={'啟用' if sync_enable else '停用'})")
        
    def initialize(self, robot, motion_state_machine, external_modules):
        """初始化Flow執行器"""
        self.robot = robot
        self.motion_state_machine = motion_state_machine
        self.external_modules = external_modules
        
        print(f"✓ Flow1執行器初始化完成")
        print(f"  可用模組: Gripper={self.external_modules.get('gripper') is not None}")
        print(f"  AutoFeeding交握: 寄存器940-945")
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
        """建構Flow1步驟"""
        if not self.points_loaded:
            print("警告: 點位未載入，無法建構流程步驟")
            self.motion_steps = []
            self.total_steps = 0
            return
            
        # 定義流程步驟
        self.motion_steps = [
            # 1. AutoFeeding座標讀取
            {'type': 'autofeeding_coordinates_read', 'params': {}},
            
            # 2. 初始準備
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            {'type': 'gripper_close', 'params': {}},
            
            # 3. 移動到VP上方檢測位置
            {'type': 'move_to_point', 'params': {'point_name': 'VP_TOPSIDE', 'move_type': 'J'}},
            
            # 4. 移動到檢測物件位置（與VP_TOPSIDE同高）
            {'type': 'move_to_detected_position_high', 'params': {}},
            
            # 5. 下降到夾取高度
            {'type': 'move_to_detected_position_low', 'params': {}},
            
            # 6. 夾爪智能撐開
            {'type': 'gripper_smart_release', 'params': {'position': 370}},
            
            # 7. 上升離開
            {'type': 'move_to_point', 'params': {'point_name': 'VP_TOPSIDE', 'move_type': 'L'}},
            
            # 8. 回到待機點
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 9. 翻轉站序列
            {'type': 'move_to_point', 'params': {'point_name': 'Rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'Rotate_down', 'move_type': 'J'}},
            {'type': 'gripper_close', 'params': {}},
            {'type': 'move_to_point', 'params': {'point_name': 'Rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"Flow1流程步驟建構完成，共{self.total_steps}步")
    
    def execute(self) -> FlowResult:
        """執行Flow1主邏輯"""
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
        
        # 檢查初始化
        if not self.robot or not self.robot.is_connected:
            return FlowResult(
                success=False,
                error_message="機械臂未連接或未初始化",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        # 檢查AutoFeeding連接
        if not self.autofeeding_interface.ensure_connection():
            return FlowResult(
                success=False,
                error_message="AutoFeeding連接失敗",
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
                    success = self._execute_move_to_point(step['params'])
                elif step['type'] == 'gripper_close':
                    success = self._execute_gripper_close()
                elif step['type'] == 'gripper_smart_release':
                    success = self._execute_gripper_smart_release(step['params'])
                elif step['type'] == 'autofeeding_coordinates_read':
                    detected_position = self._execute_autofeeding_coordinates_read()
                    success = detected_position is not None
                elif step['type'] == 'move_to_detected_position_high':
                    success = self._execute_move_to_detected_high(detected_position)
                elif step['type'] == 'move_to_detected_position_low':
                    success = self._execute_move_to_detected_low(detected_position)
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
                        total_steps=self.total_steps
                    )
                
                self.current_step += 1
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                detected_position=detected_position
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            return FlowResult(
                success=False,
                error_message=f"Flow1執行異常: {str(e)}",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
    
    def _execute_move_to_point(self, params: Dict[str, Any]) -> bool:
        """執行移動到外部點位檔案的點位 - 支援sync控制"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                print(f"錯誤: 點位管理器中找不到點位: {point_name}")
                return False
            
            print(f"移動到點位 {point_name} (sync={'啟用' if self.sync_enable else '停用'})")
            print(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{point.j4:.1f})")
            print(f"  笛卡爾座標: ({point.x:.2f}, {point.y:.2f}, {point.z:.2f}, {point.r:.2f})")
            
            success = False
            if move_type == 'J':
                # 使用關節角度運動
                success = self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4)
            elif move_type == 'L':
                # 直線運動使用笛卡爾座標
                success = self.robot.move_l(point.x, point.y, point.z, point.r)
            else:
                print(f"未支援的移動類型: {move_type}")
                return False
            
            # 可選同步控制
            if success and self.sync_enable:
                self.robot.sync()
                
            return success
                
        except Exception as e:
            print(f"移動到點位失敗: {e}")
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
    
    def _execute_autofeeding_coordinates_read(self) -> Optional[Dict[str, float]]:
        """執行AutoFeeding座標讀取"""
        try:
            print("開始讀取AutoFeeding座標...")
            
            # 等待AutoFeeding提供DR_F座標 (最多等待10秒)
            timeout = 10.0
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                coord_data = self.autofeeding_interface.read_dr_f_coordinates()
                if coord_data:
                    break
                time.sleep(0.1)  # 100ms檢查間隔
            else:
                print("等待AutoFeeding座標超時")
                return None
            
            # 確認座標讀取
            if not self.autofeeding_interface.confirm_coordinates_read():
                print("確認座標讀取失敗")
                return None
            
            # 獲取VP_TOPSIDE點位的Z高度和R值
            vp_topside_point = self.points_manager.get_point('VP_TOPSIDE')
            if not vp_topside_point:
                print("錯誤: 無法獲取VP_TOPSIDE點位")
                return None
            
            detected_pos = {
                'x': coord_data['x'],                # AutoFeeding提供的X座標
                'y': coord_data['y'],                # AutoFeeding提供的Y座標
                'z': vp_topside_point.z,             # 繼承VP_TOPSIDE的Z高度
                'r': vp_topside_point.r,             # 繼承VP_TOPSIDE的R角度
                'source': 'autofeeding'
            }
            
            print(f"AutoFeeding座標讀取成功:")
            print(f"  DR_F座標: ({detected_pos['x']:.2f}, {detected_pos['y']:.2f})")
            print(f"  繼承VP_TOPSIDE - Z:{detected_pos['z']:.2f}, R:{detected_pos['r']:.2f}")
            
            return detected_pos
            
        except Exception as e:
            print(f"AutoFeeding座標讀取異常: {e}")
            return None
    
    def _execute_move_to_detected_high(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(與VP_TOPSIDE同高) - 支援sync控制"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            # 座標系切換到右手系
            print("  切換到右手系...")
            if hasattr(self.robot, 'dashboard_api') and self.robot.dashboard_api:
                try:
                    result = self.robot.dashboard_api.SetArmOrientation(1)  
                    if self.robot._parse_api_response(result):
                        print("  ✓ 已切換到右手系")
                    else:
                        print(f"  ⚠️ 切換到右手系失敗: {result}")
                except Exception as e:
                    print(f"  ⚠️ 切換座標系異常: {e}")
            else:
                print("  ⚠️ 無法訪問座標系切換API，跳過")
            
            print(f"移動到檢測位置(與VP_TOPSIDE同高) (sync={'啟用' if self.sync_enable else '停用'}):")
            print(f"  檢測XY: ({detected_position['x']:.2f}, {detected_position['y']:.2f})")
            print(f"  繼承Z: {detected_position['z']:.2f} (VP_TOPSIDE高度)")
            print(f"  繼承R: {detected_position['r']:.2f} (VP_TOPSIDE角度)")
            
            # 使用MovL移動到檢測位置
            success = self.robot.move_l(
                detected_position['x'],
                detected_position['y'],
                detected_position['z'],
                detected_position['r']
            )
            
            # 可選同步控制
            if success and self.sync_enable:
                self.robot.sync()
            
            if success:
                print(f"✓ 移動到檢測位置完成")
                return True
            else:
                print(f"✗ 移動到檢測位置失敗")
                return False
                
        except Exception as e:
            print(f"移動到檢測位置失敗: {e}")
            return False
    
    def _execute_move_to_detected_low(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(夾取高度) - 支援sync控制"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            print(f"下降到夾取高度 (sync={'啟用' if self.sync_enable else '停用'}):")
            print(f"  檢測XY: ({detected_position['x']:.2f}, {detected_position['y']:.2f})")
            print(f"  夾取高度Z: {self.PICKUP_HEIGHT:.2f}")
            print(f"  繼承R: {detected_position['r']:.2f} (VP_TOPSIDE角度)")
            
            # 使用夾取高度
            success = self.robot.move_l(
                detected_position['x'],
                detected_position['y'],
                self.PICKUP_HEIGHT,
                detected_position['r']
            )
            
            # 可選同步控制
            if success and self.sync_enable:
                self.robot.sync()
                
            if success:
                print(f"✓ 下降到夾取位置完成")
                return True
            else:
                print(f"✗ 下降到夾取位置失敗")
                return False
                
        except Exception as e:
            print(f"移動到夾取位置失敗: {e}")
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
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
    
    def is_ready(self) -> bool:
        """檢查Flow1是否準備好執行"""
        return (self.points_loaded and 
                self.total_steps > 0 and 
                self.autofeeding_interface.connected)
    
    def cleanup(self):
        """清理資源"""
        if hasattr(self, 'autofeeding_interface'):
            self.autofeeding_interface.disconnect()


# 兼容性別名
class Flow1Executor(DrFlow1VisionPickExecutor):
    """Flow1執行器 - 兼容性包裝器"""
    pass