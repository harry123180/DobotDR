#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow2.py - Flow2 出料流程執行器 (DR專案版 - 支援從Flow1獲取角度數據)
整合AngleHighLevel角度檢測，使用外部點位檔案
實現完整的出料作業流程，包含角度計算和J4角度控制
支援從Flow1執行器獲取角度數據，實現角度數據跨Flow傳遞
"""

import time
import os
import json
import threading
import queue
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
    angle_source: str = "unknown"  # 新增：角度來源
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
    """Flow2: 出料流程執行器 - DR專案版 (支援從Flow1獲取角度數據)"""
    
    def __init__(self):
        # 核心組件 (通過initialize方法設置)
        self.robot = None
        self.motion_state_machine = None
        self.external_modules = {}
        
        # 流程配置
        self.flow_id = 2
        self.flow_name = "出料流程(支援Flow1角度數據)"
        self.status = FlowStatus.READY
        self.current_step = 0
        self.start_time = 0.0
        self.last_error = ""
        
        # 角度控制參數
        self.target_angle = None      # 從AngleHighLevel或Flow1獲取的角度
        self.command_angle = None     # 計算後的指令角度 (target_angle + 45)
        self.angle_acquisition_success = False
        self.angle_source = "none"    # 角度來源: "flow1", "self_detect", "default"
        self.ANGLE_OFFSET = 45.0      # 角度偏移量
        
        # Flow1角度數據來源 (新增)
        self.flow1_executor_ref = None  # Flow1執行器引用
        self.prefer_flow1_angle = True  # 優先使用Flow1角度數據
        
        # 優化參數
        self.angle_detection_timeout = 3.0  # 角度檢測超時時間縮短到3秒
        self.use_fast_angle_detection = True  # 啟用快速角度檢測
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # Flow2需要的點位名稱 - 精簡版
        self.REQUIRED_POINTS = [
            "standby",                  # 待機點
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
        
        print("✓ DrFlow2UnloadExecutor初始化完成 (精簡組裝流程版)")
        print("✓ 精簡流程: 僅保留組裝序列，put_asm_top和put_asm_down使用Flow1角度值")
        
    def initialize(self, robot, motion_state_machine, external_modules, flow1_executor_ref=None):
        """初始化Flow執行器 - 新增Flow1執行器引用"""
        self.robot = robot
        self.motion_state_machine = motion_state_machine
        self.external_modules = external_modules
        self.flow1_executor_ref = flow1_executor_ref  # 新增：Flow1執行器引用
        
        print(f"✓ Flow2執行器初始化完成 (精簡組裝流程版)")
        print(f"  可用模組: Gripper={self.external_modules.get('gripper') is not None}")
        print(f"  Flow1角度數據: {'✓ 可用' if self.flow1_executor_ref else '✗ 不可用'}")
        print(f"  精簡流程: put_asm_top → put_asm_down → gripper_close → put_asm_top → back_standby_from_asm → standby")
        
    def set_flow1_executor_reference(self, flow1_executor):
        """設置Flow1執行器引用 - 外部調用"""
        self.flow1_executor_ref = flow1_executor
        print(f"✓ Flow2已設置Flow1執行器引用")
        
        # 檢查Flow1角度數據可用性
        if hasattr(flow1_executor, 'has_valid_angle_data'):
            if flow1_executor.has_valid_angle_data():
                print(f"✓ Flow1有有效角度數據可供使用")
            else:
                print(f"⚠️ Flow1角度數據無效，Flow2將使用自檢測或預設值")
    
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
        """建構Flow2步驟 - 精簡組裝流程版 (移除獨立角度獲取步驟)"""
        if not self.points_loaded:
            print("警告: 點位未載入，無法建構流程步驟")
            self.motion_steps = []
            self.total_steps = 0
            return
            
        # 定義精簡流程步驟 - 移除angle_data_acquisition
        self.motion_steps = [
            # 1. 精簡組裝序列 (帶J4角度控制，內嵌角度獲取)
            {'type': 'move_to_point_with_j4', 'params': {'point_name': 'put_asm_top', 'move_type': 'J', 'sync': False}},
            {'type': 'move_to_point_with_j4', 'params': {'point_name': 'put_asm_down', 'move_type': 'J', 'sync': True}},
            
            # 2. 夾爪關閉
            {'type': 'gripper_close', 'params': {}},
            
            # 3. 回程序列
            {'type': 'move_to_point_with_j4', 'params': {'point_name': 'put_asm_top', 'move_type': 'J', 'sync': False}},
            {'type': 'move_to_point', 'params': {'point_name': 'back_standby_from_asm', 'move_type': 'J', 'sync': False}},
            {'type': 'move_to_point', 'params': {'point_name': 'standby', 'move_type': 'J', 'sync': True}},
        ]
        
        self.total_steps = len(self.motion_steps)
        print(f"Flow2精簡流程步驟建構完成，共{self.total_steps}步")
        print("✓ 精簡流程: put_asm_top → put_asm_down → gripper_close → put_asm_top → back_standby_from_asm → standby")
        print("✓ J4角度控制: 在move_to_point_with_j4中動態獲取Flow1角度值")
    
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
        self.angle_source = "none"
        
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
                    success = self._execute_move_to_point_with_parameters(step['params'])
                elif step['type'] == 'move_to_point_with_j4':
                    success = self._execute_move_to_point_with_j4(step['params'])
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
                        angle_acquisition_success=self.angle_acquisition_success,
                        angle_source=self.angle_source
                    )
                
                self.current_step += 1
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            print(f"✓ Flow2執行完成！總耗時: {execution_time:.2f}秒")
            print(f"✓ 精簡組裝流程完成")
            if self.angle_acquisition_success:
                print(f"角度控制: 目標角度={self.target_angle:.2f}°, 指令角度={self.command_angle:.2f}°")
                print(f"角度來源: {self.angle_source}")
                print(f"J4角度應用: put_asm_top和put_asm_down都使用指令角度{self.command_angle:.2f}°")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                target_angle=self.target_angle,
                command_angle=self.command_angle,
                angle_acquisition_success=self.angle_acquisition_success,
                angle_source=self.angle_source
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
                angle_acquisition_success=self.angle_acquisition_success,
                angle_source=self.angle_source
            )
    
    def _execute_angle_data_acquisition(self) -> bool:
        """執行角度數據獲取 - 優先Flow1，備用自檢測"""
        try:
            print("  [角度數據獲取] 開始...")
            
            # 策略1: 嘗試從Flow1獲取角度數據
            if self.prefer_flow1_angle and self._try_get_angle_from_flow1():
                return True
            
            # 策略2: 備用快速角度檢測
            print("  [角度數據獲取] Flow1數據不可用，嘗試自檢測...")
            return self._execute_angle_detection_fast()
            
        except Exception as e:
            print(f"  [角度數據獲取] ✗ 異常: {e}")
            self._set_default_angle()
            return True
    
    def _try_get_angle_from_flow1(self) -> bool:
        """嘗試從Flow1獲取角度數據"""
        try:
            print("  [Flow1角度獲取] 檢查Flow1角度數據...")
            
            # 檢查Flow1執行器引用
            if not self.flow1_executor_ref:
                print("  [Flow1角度獲取] ✗ Flow1執行器引用不存在")
                return False
            
            # 檢查Flow1是否有有效角度數據
            if not hasattr(self.flow1_executor_ref, 'has_valid_angle_data'):
                print("  [Flow1角度獲取] ✗ Flow1執行器缺少has_valid_angle_data方法")
                return False
            
            if not self.flow1_executor_ref.has_valid_angle_data():
                print("  [Flow1角度獲取] ✗ Flow1角度數據無效")
                return False
            
            # 獲取Flow1角度數據
            flow1_target = self.flow1_executor_ref.get_target_angle()
            flow1_command = self.flow1_executor_ref.get_command_angle()
            
            if flow1_target is None or flow1_command is None:
                print("  [Flow1角度獲取] ✗ Flow1角度數據為空")
                return False
            
            # 應用Flow1角度數據
            self.target_angle = flow1_target
            self.command_angle = flow1_command
            self.angle_acquisition_success = True
            self.angle_source = "flow1"
            
            print(f"  [Flow1角度獲取] ✓ 成功獲取Flow1角度數據:")
            print(f"    目標角度: {self.target_angle:.2f}°")
            print(f"    指令角度: {self.command_angle:.2f}°")
            print(f"    角度來源: Flow1傳遞")
            
            return True
            
        except Exception as e:
            print(f"  [Flow1角度獲取] ✗ 異常: {e}")
            return False
    
    def _execute_move_to_point_with_parameters(self, params: Dict[str, Any]) -> bool:
        """執行移動到點位 - 支援參數版本"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                print(f"錯誤: 點位管理器中找不到點位: {point_name}")
                return False
            
            print(f"移動到點位 {point_name} ({move_type})")
            print(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{point.j4:.1f})")
            print(f"  笛卡爾座標: ({point.x:.2f}, {point.y:.2f}, {point.z:.2f}, {point.r:.2f})")
            
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
            sync_enabled = params.get('sync', False)
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
        """執行移動到點位並使用計算的J4角度 - 內嵌Flow1角度獲取"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                print(f"錯誤: 點位管理器中找不到點位: {point_name}")
                return False
            
            # 🔥 關鍵：在此處動態獲取Flow1角度數據
            j4_value = self._get_flow1_angle_or_default(point)
            
            print(f"移動到點位 {point_name} (動態角度獲取)")
            print(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{j4_value:.1f})")
            print(f"  角度來源: {self.angle_source}")
            
            success = False
            if move_type in ['J', 'JointMovJ']:
                # 使用關節角度運動，J4使用動態獲取的角度
                success = self.robot.joint_move_j(point.j1, point.j2, point.j3, j4_value)
            else:
                print(f"J4角度控制僅支援關節運動(J)，當前類型: {move_type}")
                return False
            
            # Sync控制
            sync_enabled = params.get('sync', False)
            if success and sync_enabled:
                self.robot.sync()
                print(f"  ✓ 移動到 {point_name} 成功 (J4:{j4_value:.1f}°) (含Sync)")
            elif success:
                print(f"  ✓ 移動到 {point_name} 成功 (J4:{j4_value:.1f}°)")
                
            return success
                
        except Exception as e:
            print(f"移動到點位(J4角度控制)失敗: {e}")
            return False
    
    def _get_flow1_angle_or_default(self, point: RobotPoint) -> float:
        """動態獲取Flow1角度或使用預設值"""
        try:
            # 策略1: 嘗試從Flow1獲取角度數據
            if self._try_get_angle_from_flow1_direct():
                print(f"  [動態角度獲取] ✓ 成功獲取Flow1角度: {self.command_angle:.2f}°")
                return self.command_angle
            
            # 策略2: 使用點位原始J4值
            print(f"  [動態角度獲取] ⚠️ Flow1角度不可用，使用點位原始J4值: {point.j4:.2f}°")
            self.angle_source = "point_original"
            return point.j4
            
        except Exception as e:
            print(f"  [動態角度獲取] ✗ 異常: {e}，使用點位原始J4值")
            self.angle_source = "error_fallback"
            return point.j4
    
    def _try_get_angle_from_flow1_direct(self) -> bool:
        """直接從Flow1獲取角度數據 - 簡化版本"""
        try:
            # 檢查Flow1執行器引用
            if not self.flow1_executor_ref:
                self.angle_source = "no_flow1_ref"
                return False
            
            # 檢查Flow1是否有有效角度數據的方法
            if not hasattr(self.flow1_executor_ref, 'has_valid_angle_data'):
                self.angle_source = "no_method"
                return False
            
            # 檢查Flow1角度數據是否有效
            if not self.flow1_executor_ref.has_valid_angle_data():
                self.angle_source = "flow1_invalid"
                return False
            
            # 獲取Flow1角度數據
            if hasattr(self.flow1_executor_ref, 'get_command_angle'):
                flow1_command = self.flow1_executor_ref.get_command_angle()
                if flow1_command is not None:
                    # 直接應用Flow1的command_angle
                    self.command_angle = flow1_command
                    self.target_angle = self.flow1_executor_ref.get_target_angle() if hasattr(self.flow1_executor_ref, 'get_target_angle') else None
                    self.angle_acquisition_success = True
                    self.angle_source = "flow1_direct"
                    return True
            
            self.angle_source = "flow1_no_command_angle"
            return False
            
        except Exception as e:
            print(f"  [Flow1角度獲取] ✗ 異常: {e}")
            self.angle_source = "flow1_exception"
            return False
    
    def _execute_angle_detection_fast(self) -> bool:
        """執行AngleHighLevel角度檢測 - 快速版本 (備用方案)"""
        try:
            print("  [快速角度檢測] 開始檢測...")
            detection_start_time = time.time()
            
            # 優先使用external_modules中的angle模組
            angle_controller = self.external_modules.get('angle')
            
            if not angle_controller:
                print("  [快速角度檢測] ✗ 角度模組未連接，快速使用預設值")
                self._set_default_angle("self_detect_failed")
                return True
            
            print("  [快速角度檢測] ✓ 使用外部模組中的角度API")
            
            # 檢查角度控制器連接狀態
            if hasattr(angle_controller, 'connected') and not angle_controller.connected:
                print("  [快速角度檢測] ⚠️ 角度控制器未連接，嘗試快速重連...")
                if not angle_controller.connect():
                    print("  [快速角度檢測] ✗ 快速重連失敗，使用預設值")
                    self._set_default_angle("self_detect_failed")
                    return True
            
            # 執行快速角度檢測
            print("  [快速角度檢測] 執行CCD3角度檢測(DR模式)...")
            
            try:
                # 使用超時機制執行角度檢測
                detection_result = self._execute_angle_detection_with_timeout(angle_controller)
                
                if detection_result is None:
                    print(f"  [快速角度檢測] ✗ 檢測超時({self.angle_detection_timeout}秒)，使用預設值")
                    self._set_default_angle("self_detect_timeout")
                    return True
                
                detection_time = time.time() - detection_start_time
                
                if (hasattr(detection_result, 'result') and 
                    detection_result.result.value == "SUCCESS" and 
                    detection_result.target_angle is not None):
                    
                    self.target_angle = detection_result.target_angle
                    self.command_angle = self.target_angle + self.ANGLE_OFFSET
                    self.angle_acquisition_success = True
                    self.angle_source = "self_detect"
                    
                    print(f"  [快速角度檢測] ✓ 角度檢測成功 (耗時: {detection_time:.2f}秒):")
                    print(f"    目標角度: {self.target_angle:.2f}°")
                    print(f"    指令角度: {self.command_angle:.2f}°")
                    print(f"    角度來源: 自檢測")
                    
                    return True
                else:
                    error_msg = getattr(detection_result, 'message', '未知錯誤') if detection_result else '檢測失敗'
                    print(f"  [快速角度檢測] ✗ 檢測失敗: {error_msg} (耗時: {detection_time:.2f}秒)")
                    self._set_default_angle("self_detect_failed")
                    return True
                    
            except Exception as detection_error:
                detection_time = time.time() - detection_start_time
                print(f"  [快速角度檢測] ✗ 檢測異常: {detection_error} (耗時: {detection_time:.2f}秒)")
                self._set_default_angle("self_detect_error")
                return True
                
        except Exception as e:
            detection_time = time.time() - detection_start_time
            print(f"  [快速角度檢測] ✗ 系統異常: {e} (耗時: {detection_time:.2f}秒)")
            self._set_default_angle("system_error")
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
    
    def _set_default_angle(self, reason: str = "default"):
        """設置預設角度值"""
        self.target_angle = 0.0
        self.command_angle = self.target_angle + self.ANGLE_OFFSET
        self.angle_acquisition_success = False
        self.angle_source = reason
        print(f"  [角度設置] 使用預設角度: target={self.target_angle}°, command={self.command_angle}°")
        print(f"  [角度設置] 原因: {reason}")
    
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
    
    def get_angle_data_summary(self) -> Dict[str, Any]:
        """獲取角度數據摘要 - 供外部調用"""
        return {
            'target_angle': self.target_angle,
            'command_angle': self.command_angle,
            'acquisition_success': self.angle_acquisition_success,
            'angle_source': self.angle_source,
            'angle_offset': self.ANGLE_OFFSET,
            'flow1_ref_available': self.flow1_executor_ref is not None,
            'prefer_flow1_angle': self.prefer_flow1_angle
        }
        
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