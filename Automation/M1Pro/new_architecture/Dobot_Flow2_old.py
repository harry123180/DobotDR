#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow2.py - Flow2 出料流程執行器 (DR專案優化版)
整合AngleHighLevel角度檢測，使用外部點位檔案
實現完整的出料作業流程，包含角度計算和J4角度控制
優化角度檢測性能，減少等待時間
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
                    
                    self.points[point.name] = point
                    
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


class DrFlow2UnloadExecutor:
    """Flow2: 出料流程執行器 - DR專案優化版"""
    
    def __init__(self):
        # 核心組件 (通過initialize方法設置)
        self.robot = None
        self.motion_state_machine = None
        self.external_modules = {}
        
        # 流程配置
        self.flow_id = 2
        self.flow_name = "出料流程"
        self.status = FlowStatus.READY
        self.current_step = 0
        self.start_time = 0.0
        self.last_error = ""
        
        # 角度控制參數
        self.target_angle = None      # 從AngleHighLevel獲取的角度
        self.command_angle = None     # 計算後的指令角度 (target_angle + 45)
        self.angle_acquisition_success = False
        self.ANGLE_OFFSET = 45.0      # 角度偏移量
        
        # 優化參數
        self.angle_detection_timeout = 3.0  # 角度檢測超時時間縮短到3秒
        self.use_fast_angle_detection = True  # 啟用快速角度檢測
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # Flow2需要的點位名稱
        self.REQUIRED_POINTS = [
            "standby",                  # 待機點
            "Rotate_V2",               # 翻轉預備點
            "Rotate_top",              # 翻轉頂部點
            "Rotate_down",             # 翻轉底部點
            "put_asm_Pre",             # 組裝預備位置
            "put_asm_top",             # 組裝頂部位置  
            "put_asm_down",            # 組裝放下位置
            "back_standby_from_asm"    # 從組裝區回程的中轉點
        ]
        
        # 建構流程步驟
        self.motion_steps = []
        self.total_steps = 0
        
        # 嘗試載入點位檔案
        self._load_and_validate_points()
        
        # 只有點位載入成功才建構流程步驟
        if self.points_loaded:
            self.build_flow_steps()
        
        print("✓ DrFlow2UnloadExecutor初始化完成 (優化版)")
        
    def initialize(self, robot, motion_state_machine, external_modules):
        """初始化Flow執行器"""
        self.robot = robot
        self.motion_state_machine = motion_state_machine
        self.external_modules = external_modules
        
        print(f"✓ Flow2執行器初始化完成")
        print(f"  可用模組: Gripper={self.external_modules.get('gripper') is not None}, "
              f"Angle={self.external_modules.get('angle') is not None}")
        print(f"  優化設置: 角度檢測超時={self.angle_detection_timeout}秒, 快速檢測={'啟用' if self.use_fast_angle_detection else '停用'}")
        
    def _load_and_validate_points(self):
        """載入並驗證點位檔案"""
        print("Flow2正在載入外部點位檔案...")
        
        # 載入點位檔案
        if not self.points_manager.load_points():
            print("錯誤: 無法載入點位檔案，Flow2無法執行")
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
        """建構Flow2步驟"""
        if not self.points_loaded:
            print("警告: 點位未載入，無法建構流程步驟")
            self.motion_steps = []
            self.total_steps = 0
            return
            
        # 定義流程步驟
        self.motion_steps = [
            # 1. 移動到待機點
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 2. AngleHighLevel角度檢測 (優化版)
            {'type': 'angle_detection_fast', 'params': {}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
            
            # 3. 翻轉站序列
            {'type': 'move_to_point', 'params': {'point_name': 'Rotate_V2', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'Rotate_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'Rotate_down', 'move_type': 'J'}},
            
            # 4. 夾爪智能撐開
            {'type': 'gripper_smart_release', 'params': {'position': 370}},
            
            # 5. 移動到翻轉頂部
            {'type': 'move_to_point', 'params': {'point_name': 'Rotate_top', 'move_type': 'J'}},
            
            # 6. 組裝序列 (帶J4角度控制)
            {'type': 'move_to_point', 'params': {'point_name': 'put_asm_Pre', 'move_type': 'J'}},
            {'type': 'move_to_point_with_j4', 'params': {'point_name': 'put_asm_top', 'move_type': 'J'}},
            {'type': 'move_to_point_with_j4', 'params': {'point_name': 'put_asm_down', 'move_type': 'J'}},

            # 7. 夾爪快速關閉
            {'type': 'gripper_close', 'params': {}},
            {'type': 'gripper_close', 'params': {}},
            
            # 8. 回程序列
            {'type': 'move_to_point_with_j4', 'params': {'point_name': 'put_asm_top', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'back_standby_from_asm', 'move_type': 'J'}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J'}},
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"Flow2流程步驟建構完成，共{self.total_steps}步")
    
    def execute(self) -> FlowResult:
        """執行Flow2主邏輯"""
        # 檢查點位是否已載入
        if not self.points_loaded:
            return FlowResult(
                success=False,
                error_message="點位檔案載入失敗，無法執行Flow2",
                execution_time=0.0,
                steps_completed=0,
                total_steps=0
            )
        
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        
        # 重置角度參數
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
        
        try:
            for step in self.motion_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                print(f"Flow2 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 更新進度到motion_state_machine
                if self.motion_state_machine:
                    progress = int((self.current_step / self.total_steps) * 100)
                    self.motion_state_machine.set_progress(progress)
                
                # 執行步驟
                success = False
                
                if step['type'] == 'move_to_point':
                    success = self._execute_move_to_point(step['params'])
                elif step['type'] == 'move_to_point_with_j4':
                    success = self._execute_move_to_point_with_j4(step['params'])
                elif step['type'] == 'angle_detection_fast':
                    success = self._execute_angle_detection_fast()
                elif step['type'] == 'gripper_close':
                    success = self._execute_gripper_close()
                elif step['type'] == 'gripper_smart_release':
                    success = self._execute_gripper_smart_release(step['params'])
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
            
            print(f"✓ Flow2執行完成！總耗時: {execution_time:.2f}秒")
            if self.angle_acquisition_success:
                print(f"角度控制: 目標角度={self.target_angle:.2f}°, 指令角度={self.command_angle:.2f}°")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                target_angle=self.target_angle,
                command_angle=self.command_angle,
                angle_acquisition_success=self.angle_acquisition_success
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            return FlowResult(
                success=False,
                error_message=f"Flow2執行異常: {str(e)}",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                target_angle=self.target_angle,
                command_angle=self.command_angle,
                angle_acquisition_success=self.angle_acquisition_success
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
    
    def _execute_move_to_point_with_j4(self, params: Dict[str, Any]) -> bool:
        """執行移動到點位並使用計算的J4角度"""
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
            
            if move_type == 'J':
                # 使用關節角度運動，J4使用計算角度
                return self.robot.joint_move_j(point.j1, point.j2, point.j3, j4_value)
            else:
                print(f"J4角度控制僅支援關節運動(J)，當前類型: {move_type}")
                return False
                
        except Exception as e:
            print(f"移動到點位(J4角度控制)失敗: {e}")
            return False
    
    def _execute_angle_detection_fast(self) -> bool:
        """執行AngleHighLevel角度檢測 - 優化版"""
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
        """帶超時的角度檢測執行"""
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
        """設置預設角度值"""
        self.target_angle = 0.0
        self.command_angle = self.target_angle + self.ANGLE_OFFSET
        self.angle_acquisition_success = False
        print(f"  [快速角度檢測] 使用預設角度: target={self.target_angle}°, command={self.command_angle}°")
    
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
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        print("Flow2已暫停")
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("Flow2已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow"""
        self.status = FlowStatus.ERROR
        print("Flow2已停止")
        return True
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
    
    def is_ready(self) -> bool:
        """檢查Flow2是否準備好執行"""
        return self.points_loaded and self.total_steps > 0


# 兼容性別名
class Flow2Executor(DrFlow2UnloadExecutor):
    """Flow2執行器 - 兼容性包裝器"""
    pass