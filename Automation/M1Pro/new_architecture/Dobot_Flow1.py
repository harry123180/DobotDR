#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1.py - Flow1 VP視覺抓取流程執行器
使用外部點位檔案，禁止硬編碼座標
整合CCD1HighLevel API進行智能檢測
"""

import time
import os
import json
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum


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


class PointsManager:
    """點位管理器 - 支援cartesian和pose格式"""
    
    def __init__(self, points_file: str = "saved_points/robot_points.json"):
        # 確保使用絕對路徑，相對於當前執行檔案的目錄
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
    """Flow1: VP視覺抓取流程執行器"""
    
    def __init__(self):
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
        
        # 流程參數
        self.PICKUP_HEIGHT = 137.52  # 夾取高度
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
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
        
        print("✓ DrFlow1VisionPickExecutor初始化完成")
        
    def initialize(self, robot, motion_state_machine, external_modules):
        """初始化Flow執行器"""
        self.robot = robot
        self.motion_state_machine = motion_state_machine
        self.external_modules = external_modules
        
        print(f"✓ Flow1執行器初始化完成")
        print(f"  可用模組: Gripper={self.external_modules.get('gripper') is not None}")
        print(f"  CCD1檢測模式: 直接寄存器讀取 (基地址257-260)")
        # 移除了對CCD1高層API的依賴檢查
        
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
            # 1. 初始準備
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            {'type': 'gripper_close', 'params': {}},
            # 2. CCD1視覺檢測
            {'type': 'ccd1_smart_detection', 'params': {}},
            
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
                elif step['type'] == 'ccd1_smart_detection':
                    detected_position = self._execute_ccd1_smart_detection()
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
        """執行移動到外部點位檔案的點位"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                print(f"錯誤: 點位管理器中找不到點位: {point_name}")
                return False
            
            print(f"移動到點位 {point_name}")
            print(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{point.j4:.1f})")
            print(f"  笛卡爾座標: ({point.x:.2f}, {point.y:.2f}, {point.z:.2f}, {point.r:.2f})")
            
            if move_type == 'J':
                # 使用關節角度運動
                return self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4)
            elif move_type == 'L':
                # 直線運動使用笛卡爾座標
                return self.robot.move_l(point.x, point.y, point.z, point.r)
            else:
                print(f"未支援的移動類型: {move_type}")
                return False
                
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
                if hasattr(gripper_api, 'get_current_position'):
                    try:
                        current_pos = gripper_api.get_current_position()
                        if current_pos is not None:
                            print(f"  夾爪當前位置: {current_pos}")
                            if abs(current_pos - position) <= 20:  # 容差20
                                print(f"  ✓ 夾爪已撐開到目標位置 (誤差: {abs(current_pos - position)})")
                            else:
                                print(f"  ⚠️ 夾爪位置偏差較大 (目標: {position}, 實際: {current_pos})")
                    except Exception as e:
                        print(f"  無法讀取夾爪位置: {e}")
                print(f"✓ 夾爪智能撐開完成 - 位置{position}")
                return True
            else:
                print(f"✗ 夾爪智能撐開失敗")
                return False
                
        except Exception as e:
            print(f"夾爪智能撐開異常: {e}")
            return False
    def _execute_move_to_detected_high(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(與VP_TOPSIDE同高) - 僅同步XY，R繼承VP_TOPSIDE"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            # 🔥 關鍵新增：在MovL前切換到左手系
            print("  切換到左手系（LorR=0）...")
            if hasattr(self.robot, 'dashboard_api') and self.robot.dashboard_api:
                # 直接調用底層API進行座標系切換
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
            
            print(f"移動到檢測位置(與VP_TOPSIDE同高):")
            print(f"  檢測XY: ({detected_position['x']:.2f}, {detected_position['y']:.2f})")
            print(f"  繼承Z: {detected_position['z']:.2f} (VP_TOPSIDE高度)")
            print(f"  繼承R: {detected_position['r']:.2f} (VP_TOPSIDE角度)")
            
            # 使用MovL移動到檢測位置：檢測XY + VP_TOPSIDE的Z和R
            success = self.robot.move_l(
                detected_position['x'],    # CCD1檢測的X座標
                detected_position['y'],    # CCD1檢測的Y座標
                detected_position['z'],    # VP_TOPSIDE的Z高度
                detected_position['r']     # VP_TOPSIDE的R角度
            )
            
            if success:
                print(f"✓ 移動到檢測位置完成 - XY同步檢測結果，ZR繼承VP_TOPSIDE，已切換左手系")
                return True
            else:
                print(f"✗ 移動到檢測位置失敗")
                return False
                
        except Exception as e:
            print(f"移動到檢測位置失敗: {e}")
            return False
    def _execute_ccd1_smart_detection(self) -> Optional[Dict[str, float]]:
        """執行CCD1智能檢測 - 修正版：讀取正確的DR_F世界座標地址"""
        try:
            # 導入pymodbus進行直接寄存器讀取
            from pymodbus.client import ModbusTcpClient
            
            print("  使用CCD1寄存器直接讀取...")
            
            # 連接到Modbus服務器
            modbus_client = ModbusTcpClient(
                host="127.0.0.1",  # 根據你的Modbus服務器配置調整
                port=502,
                timeout=3.0
            )
            
            if not modbus_client.connect():
                print("  ⚠️ 無法連接到Modbus服務器")
                return None
            
            try:
                # 🔥 修正：讀取DR_F第1個目標的世界座標寄存器 (261-264)
                # 261: DR_F_1_WORLD_X_HIGH, 262: DR_F_1_WORLD_X_LOW
                # 263: DR_F_1_WORLD_Y_HIGH, 264: DR_F_1_WORLD_Y_LOW
                result = modbus_client.read_holding_registers(
                    address=261,    # 🔥 修正：起始地址改為261
                    count=4,        # 讀取4個寄存器 (261-264)
                    slave=1
                )
                
                if hasattr(result, 'isError') and result.isError():
                    print(f"  ✗ 讀取CCD1世界座標寄存器失敗: {result}")
                    return None
                
                if not hasattr(result, 'registers') or len(result.registers) < 4:
                    print("  ✗ 讀取的寄存器數據不足")
                    return None
                
                # 解析32位世界座標 (×100精度)
                x_high, x_low, y_high, y_low = result.registers
                
                print(f"  讀取寄存器261-264成功:")
                print(f"    X_HIGH(261)={x_high}, X_LOW(262)={x_low}")
                print(f"    Y_HIGH(263)={y_high}, Y_LOW(264)={y_low}")
                
                # 合併高低位並轉換為有符號32位整數
                world_x_int = ((x_high << 16) | x_low)
                world_y_int = ((y_high << 16) | y_low)
                
                # 處理有符號數 (如果最高位為1，則為負數)
                if world_x_int & 0x80000000:
                    world_x_int = world_x_int - 0x100000000
                if world_y_int & 0x80000000:
                    world_y_int = world_y_int - 0x100000000
                
                # 轉換為實際座標 (÷100恢復小數)
                world_x = world_x_int / 100.0
                world_y = world_y_int / 100.0
                
                print(f"  合併後32位整數: X={world_x_int}, Y={world_y_int}")
                print(f"  實際世界座標: X={world_x:.2f}mm, Y={world_y:.2f}mm")
                
                # 檢查座標是否有效 (不為0)
                if world_x == 0.0 and world_y == 0.0:
                    print("  ⚠️ CCD1世界座標為零，可能無有效檢測結果")
                    return None
                
                # 🔥 額外檢查：先確認DR_F數量是否>0
                dr_f_count_result = modbus_client.read_holding_registers(
                    address=240,    # DR_F_COUNT
                    count=1,
                    slave=1
                )
                
                if hasattr(dr_f_count_result, 'registers') and len(dr_f_count_result.registers) > 0:
                    dr_f_count = dr_f_count_result.registers[0]
                    print(f"  DR_F檢測數量: {dr_f_count}")
                    
                    if dr_f_count == 0:
                        print("  ⚠️ DR_F檢測數量為0，無有效目標")
                        return None
                else:
                    print("  ⚠️ 無法讀取DR_F檢測數量")
                
                # 獲取VP_TOPSIDE點位的Z高度和R值
                vp_topside_point = self.points_manager.get_point('VP_TOPSIDE')
                if not vp_topside_point:
                    print("錯誤: 無法獲取VP_TOPSIDE點位")
                    return None
                
                detected_pos = {
                    'x': world_x,                 # 使用CCD1檢測的DR_F世界X座標
                    'y': world_y,                 # 使用CCD1檢測的DR_F世界Y座標
                    'z': vp_topside_point.z,      # 使用VP_TOPSIDE的Z高度
                    'r': vp_topside_point.r       # 繼承VP_TOPSIDE的R角度
                }
                
                print(f"CCD1 DR_F世界座標讀取成功:")
                print(f"  寄存器值: X_high={x_high}, X_low={x_low}, Y_high={y_high}, Y_low={y_low}")
                print(f"  DR_F世界座標: ({detected_pos['x']:.2f}, {detected_pos['y']:.2f})mm")
                print(f"  繼承VP_TOPSIDE - Z:{detected_pos['z']:.2f}, R:{detected_pos['r']:.2f}")
                
                return detected_pos
                
            finally:
                # 確保關閉Modbus連接
                modbus_client.close()
                
        except ImportError:
            print("  ✗ 無法導入pymodbus，請確認pymodbus已安裝")
            return None
        except Exception as e:
            print(f"CCD1寄存器讀取異常: {e}")
            return None
    
    def _execute_move_to_detected_low(self, detected_position: Optional[Dict[str, float]]) -> bool:
        """移動到檢測位置(夾取高度) - 僅同步XY，R繼承VP_TOPSIDE"""
        try:
            if not detected_position:
                print("檢測位置為空，無法移動")
                return False
            
            print(f"下降到夾取高度:")
            print(f"  檢測XY: ({detected_position['x']:.2f}, {detected_position['y']:.2f})")
            print(f"  夾取高度Z: {self.PICKUP_HEIGHT:.2f}")
            print(f"  繼承R: {detected_position['r']:.2f} (VP_TOPSIDE角度)")
            
            # 使用夾取高度：檢測XY + 固定夾取Z + VP_TOPSIDE的R
            success = self.robot.move_l(
                detected_position['x'],    # CCD1檢測的X座標
                detected_position['y'],    # CCD1檢測的Y座標
                self.PICKUP_HEIGHT,        # 固定夾取高度137.52
                detected_position['r']     # VP_TOPSIDE的R角度
            )
            
            if success:
                # 確保機械臂到位後才繼續
                if hasattr(self.robot, 'sync'):
                    self.robot.sync()
                print(f"✓ 下降到夾取位置完成 - XY同步檢測結果，R繼承VP_TOPSIDE，夾取高度={self.PICKUP_HEIGHT:.2f}mm")
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
        return self.points_loaded and self.total_steps > 0


# 兼容性別名
class Flow1Executor(DrFlow1VisionPickExecutor):
    """Flow1執行器 - 兼容性包裝器"""
    pass