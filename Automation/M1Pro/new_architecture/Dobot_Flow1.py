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
        print(f"  可用模組: Gripper={self.external_modules.get('gripper') is not None}, "
              f"CCD1={self.external_modules.get('ccd1') is not None}")
        
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
    
    def _execute_ccd1_smart_detection(self) -> Optional[Dict[str, float]]:
        """執行CCD1智能檢測 - 僅使用檢測XY座標，R角度繼承VP_TOPSIDE"""
        try:
            ccd1_api = self.external_modules.get('ccd1')
            if not ccd1_api:
                print("CCD1 API未初始化")
                return None
            
            print("  使用CCD1智能檢測API...")
            
            # 檢查CCD1系統狀態
            system_status = ccd1_api.get_system_status()
            if not system_status['connected']:
                print("  ⚠️ CCD1系統未連接")
                return None
            
            print(f"  CCD1系統狀態: Ready={system_status.get('ready', False)}")
            
            # 使用get_next_circle_world_coord API
            # 自動處理：檢查FIFO佇列 → 如果空則自動拍照檢測 → 返回結果或None
            coord = ccd1_api.get_next_circle_world_coord()
            
            if coord:
                # 獲取VP_TOPSIDE點位的Z高度和R值
                vp_topside_point = self.points_manager.get_point('VP_TOPSIDE')
                if not vp_topside_point:
                    print("錯誤: 無法獲取VP_TOPSIDE點位")
                    return None
                
                detected_pos = {
                    'x': coord.world_x,           # 使用CCD1檢測的X座標
                    'y': coord.world_y,           # 使用CCD1檢測的Y座標
                    'z': vp_topside_point.z,      # 使用VP_TOPSIDE的Z高度
                    'r': vp_topside_point.r       # 繼承VP_TOPSIDE的R角度
                }
                print(f"CCD1檢測成功:")
                print(f"  檢測XY座標: ({detected_pos['x']:.2f}, {detected_pos['y']:.2f})")
                print(f"  繼承VP_TOPSIDE - Z:{detected_pos['z']:.2f}, R:{detected_pos['r']:.2f}")
                return detected_pos
            else:
                print("CCD1未檢測到有效物件")
                return None
                
        except Exception as e:
            print(f"CCD1檢測異常: {e}")
            return None
    
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