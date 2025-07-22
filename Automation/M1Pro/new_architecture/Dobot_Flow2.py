#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow2.py - Flow2 出料流程執行器 (DR專案版 - 支援CASE參數傳入 + Flow1角度數據)
- 升級motion_steps支援速度、加速度、運動類型、sync調用、切換手勢等參數
- 支援從Flow1獲取角度數據，實現角度數據跨Flow傳遞
- 整合AngleHighLevel角度檢測，使用外部點位檔案
- 支援CASE專案格式的參數傳入：speed_j, acc_j, speed_l, acc_l, tool, sync等
- 支援MovJ、MovL、JointMovJ運動類型和SetArmOrientation手勢切換
"""

import time
import os
import json
import threading
import queue
import logging
from logging.handlers import RotatingFileHandler
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
    angle_source: str = "unknown"
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


def setup_logging(module_name):
    """統一設置logging配置"""
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
    os.makedirs(log_dir, exist_ok=True)
    
    formatter = logging.Formatter(
        '%(asctime)s [%(levelname)s] %(name)s:%(funcName)s:%(lineno)d - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    file_handler = RotatingFileHandler(
        os.path.join(log_dir, f'{module_name}.log'),
        maxBytes=10*1024*1024,
        backupCount=7,
        encoding='utf-8'
    )
    file_handler.setFormatter(formatter)
    
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    
    logger = logging.getLogger(module_name)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger


class PointsManager:
    """點位管理器 - 支援cartesian和pose格式"""
    
    def __init__(self, points_file: str = "saved_points/robot_points.json"):
        self.logger = setup_logging("PointsManager")
        if not os.path.isabs(points_file):
            current_dir = os.path.dirname(os.path.abspath(__file__))
            self.points_file = os.path.join(current_dir, points_file)
        else:
            self.points_file = points_file
        self.points: Dict[str, RobotPoint] = {}
        
    def load_points(self) -> bool:
        """載入點位數據 - 支援cartesian和pose格式"""
        try:
            self.logger.info(f"嘗試載入點位檔案: {self.points_file}")
            
            if not os.path.exists(self.points_file):
                self.logger.error(f"點位檔案不存在: {self.points_file}")
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
                        self.logger.warning(f"點位 {point_data.get('name', 'unknown')} 缺少座標數據")
                        continue
                    
                    if "joint" not in point_data:
                        self.logger.warning(f"點位 {point_data.get('name', 'unknown')} 缺少關節數據")
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
                    self.logger.error(f"處理點位 {point_data.get('name', 'unknown')} 時發生錯誤: {e}")
                    continue
                
            self.logger.info(f"載入點位數據成功，共{len(self.points)}個點位: {list(self.points.keys())}")
            return True
            
        except Exception as e:
            self.logger.error(f"載入點位數據失敗: {e}", exc_info=True)
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
    """Flow2: 出料流程執行器 - DR專案版 (支援CASE參數傳入 + Flow1角度數據)"""
    
    def __init__(self):
        self.logger = setup_logging("DrFlow2UnloadExecutor")
        
        # 核心組件 (通過initialize方法設置)
        self.robot = None
        self.motion_state_machine = None
        self.external_modules = {}
        
        # 流程配置
        self.flow_id = 2
        self.flow_name = "出料流程(支援CASE參數+Flow1角度數據)"
        self.status = FlowStatus.READY
        self.current_step = 0
        self.start_time = 0.0
        self.last_error = ""
        
        # 角度控制參數
        self.target_angle = None
        self.command_angle = None
        self.angle_acquisition_success = False
        self.angle_source = "none"
        self.ANGLE_OFFSET = 45.0
        
        # Flow1角度數據來源
        self.flow1_executor_ref = None
        self.prefer_flow1_angle = True
        
        # 優化參數
        self.angle_detection_timeout = 3.0
        self.use_fast_angle_detection = True
        
        # 初始化點位管理器
        self.points_manager = PointsManager()
        self.points_loaded = False
        
        # Flow2需要的點位名稱
        self.REQUIRED_POINTS = [
            "standby",
            "put_asm_top",
            "put_asm_down",
            "back_standby_from_asm"
        ]
        
        # 建構流程步驟
        self.motion_steps = []
        self.total_steps = 0
        
        # 嘗試載入點位檔案
        self._load_and_validate_points()
        
        # 只有點位載入成功才建構流程步驟
        if self.points_loaded:
            self.build_flow_steps()
        
        self.logger.info("DrFlow2UnloadExecutor初始化完成 (支援CASE參數版)")
        self.logger.info("精簡流程: 僅保留組裝序列，put_asm_top和put_asm_down使用Flow1角度值")
        
    def initialize(self, robot, motion_state_machine, external_modules, flow1_executor_ref=None):
        """初始化Flow執行器 - 新增Flow1執行器引用"""
        self.robot = robot
        self.motion_state_machine = motion_state_machine
        self.external_modules = external_modules
        self.flow1_executor_ref = flow1_executor_ref
        
        self.logger.info("Flow2執行器初始化完成 (支援CASE參數版)")
        self.logger.info(f"  可用模組: Gripper={self.external_modules.get('gripper') is not None}")
        self.logger.info(f"  Flow1角度數據: {'可用' if self.flow1_executor_ref else '不可用'}")
        
    def set_flow1_executor_reference(self, flow1_executor):
        """設置Flow1執行器引用 - 外部調用"""
        self.flow1_executor_ref = flow1_executor
        self.logger.info("Flow2已設置Flow1執行器引用")
        
        # 檢查Flow1角度數據可用性
        if hasattr(flow1_executor, 'has_valid_angle_data'):
            if flow1_executor.has_valid_angle_data():
                self.logger.info("Flow1有有效角度數據可供使用")
            else:
                self.logger.warning("Flow1角度數據無效，Flow2將使用自檢測或預設值")
    
    def _load_and_validate_points(self):
        """載入並驗證點位檔案"""
        self.logger.info("Flow2正在載入外部點位檔案...")
        
        if not self.points_manager.load_points():
            self.logger.error("無法載入點位檔案，Flow2無法執行")
            self.points_loaded = False
            return
        
        missing_points = []
        for point_name in self.REQUIRED_POINTS:
            if not self.points_manager.has_point(point_name):
                missing_points.append(point_name)
        
        if missing_points:
            self.logger.error(f"缺少必要點位: {missing_points}")
            self.logger.debug(f"可用點位: {self.points_manager.list_points()}")
            self.points_loaded = False
            return
        
        self.logger.info("所有必要點位載入成功")
        self.points_loaded = True
        
    def build_flow_steps(self):
        """建構Flow2步驟 - 支援CASE參數版"""
        if not self.points_loaded:
            self.logger.warning("點位未載入，無法建構流程步驟")
            self.motion_steps = []
            self.total_steps = 0
            return
            
        # 定義支援CASE參數的流程步驟
        self.motion_steps = [
            # 1. 切換到左手手勢進行組裝 - 適合特定角度操作
            #{'type': 'arm_orientation_change', 'params': {'orientation': 1}},
            
            # 2. 精簡組裝序列 (帶J4角度控制，內嵌角度獲取) - 精準控制
            {'type': 'move_to_point_with_j4', 'params': {
                'point_name': 'put_asm_top', 
                'move_type': 'J', 
                'speed_j': 100,      
                'acc_j': 100,
                'sync': False
            }},
            {'type': 'move_to_point_with_j4', 'params': {
                'point_name': 'put_asm_down', 
                'move_type': 'J', 
                'speed_j': 100,      
                'acc_j': 100,
                'sync': True
            }},
            
            # 3. 夾爪關閉
            {'type': 'gripper_close', 'params': {}},
            
            # 4. 回程序列 - 速度優化
            {'type': 'move_to_point_with_j4', 'params': {
                'point_name': 'put_asm_top', 
                'move_type': 'J', 
                'speed_j': 100,    
                'acc_j': 100,
                
                'sync': False
            }},
            
            # 5. 切換回右手手勢
            #{'type': 'arm_orientation_change', 'params': {'orientation': 0}},
            
            # 6. 返回待機 - 高速運動
            {'type': 'move_to_point', 'params': {
                'point_name': 'standby', 
                'move_type': 'J', 
                'speed_j': 100,      
                'acc_j': 100,
              
                'sync': False
            }},
    
        ]
        
        self.total_steps = len(self.motion_steps)
        self.logger.info(f"Flow2支援CASE參數流程步驟建構完成，共{self.total_steps}步")
        self.logger.info("新增功能: 手勢切換 + 速度優化 + J4角度控制")
    
    def execute(self) -> FlowResult:
        """執行Flow2主邏輯 - 支援CASE參數版"""
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
                
                self.logger.info(f"Flow2 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
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
                elif step['type'] == 'arm_orientation_change':
                    success = self._execute_arm_orientation_change(step['params'])
                elif step['type'] == 'gripper_close':
                    success = self._execute_gripper_close()
                elif step['type'] == 'gripper_smart_release':
                    success = self._execute_gripper_smart_release(step['params'])
                else:
                    self.logger.warning(f"未知步驟類型: {step['type']}")
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
            
            self.logger.info(f"Flow2執行完成！總耗時: {execution_time:.2f}秒")
            self.logger.info("精簡組裝流程完成")
            if self.angle_acquisition_success:
                self.logger.info(f"角度控制: 目標角度={self.target_angle:.2f}°, 指令角度={self.command_angle:.2f}°")
                self.logger.info(f"角度來源: {self.angle_source}")
                self.logger.info(f"J4角度應用: put_asm_top和put_asm_down都使用指令角度{self.command_angle:.2f}°")
            
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
            self.logger.error(f"Flow2執行異常: {e}", exc_info=True)
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
    
    def _execute_move_to_point_with_parameters(self, params: Dict[str, Any]) -> bool:
        """執行移動到點位 - 支援CASE風格參數版本"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                self.logger.error(f"點位管理器中找不到點位: {point_name}")
                return False
            
            # 提取運動參數
            speed_j = params.get('speed_j', 80)
            acc_j = params.get('acc_j', 80)
            speed_l = params.get('speed_l', 80)
            acc_l = params.get('acc_l', 80)
            tool = params.get('tool', 0)
            sync_enabled = params.get('sync', False)
            
            self.logger.info(f"移動到點位 {point_name} ({move_type})")
            self.logger.debug(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{point.j4:.1f})")
            self.logger.debug(f"  笛卡爾座標: ({point.x:.2f}, {point.y:.2f}, {point.z:.2f}, {point.r:.2f})")
            self.logger.debug(f"  運動參數: speed_j={speed_j}, acc_j={acc_j}, speed_l={speed_l}, acc_l={acc_l}, tool={tool}")
            
            # 根據運動類型設置速度參數
            success = False
            if move_type in ['J', 'JointMovJ']:
                # 設置關節運動參數
                if hasattr(self.robot, 'set_speed_j'):
                    self.robot.set_speed_j(speed_j)
                if hasattr(self.robot, 'set_acc_j'):
                    self.robot.set_acc_j(acc_j)
                if hasattr(self.robot, 'set_tool'):
                    self.robot.set_tool(tool)
                
                # 執行關節運動
                success = self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4)
                
            elif move_type in ['L', 'MovL']:
                # 設置直線運動參數
                if hasattr(self.robot, 'set_speed_l'):
                    self.robot.set_speed_l(speed_l)
                if hasattr(self.robot, 'set_acc_l'):
                    self.robot.set_acc_l(acc_l)
                if hasattr(self.robot, 'set_tool'):
                    self.robot.set_tool(tool)
                
                # 執行直線運動
                success = self.robot.move_l(point.x, point.y, point.z, point.r)
                
            elif move_type == 'MovJ':
                # MovJ使用笛卡爾座標但關節插值
                if hasattr(self.robot, 'set_speed_j'):
                    self.robot.set_speed_j(speed_j)
                if hasattr(self.robot, 'set_acc_j'):
                    self.robot.set_acc_j(acc_j)
                if hasattr(self.robot, 'set_tool'):
                    self.robot.set_tool(tool)
                
                # 如果機械臂API支援MovJ
                if hasattr(self.robot, 'move_j'):
                    success = self.robot.move_j(point.x, point.y, point.z, point.r)
                else:
                    # 降級為JointMovJ
                    success = self.robot.joint_move_j(point.j1, point.j2, point.j3, point.j4)
                    
            else:
                self.logger.error(f"未支援的移動類型: {move_type}")
                return False
            
            # Sync控制
            if success and sync_enabled:
                self.robot.sync()
                self.logger.info(f"  移動到 {point_name} 成功 ({move_type}) (含Sync)")
            elif success:
                self.logger.info(f"  移動到 {point_name} 成功 ({move_type})")
            else:
                self.logger.error(f"  移動到 {point_name} 失敗")
                
            return success
                
        except Exception as e:
            self.logger.error(f"移動到點位失敗: {e}", exc_info=True)
            return False
    
    def _execute_move_to_point_with_j4(self, params: Dict[str, Any]) -> bool:
        """執行移動到點位並使用計算的J4角度 - 支援CASE參數版"""
        try:
            point_name = params['point_name']
            move_type = params['move_type']
            
            # 從點位管理器獲取點位
            point = self.points_manager.get_point(point_name)
            if not point:
                self.logger.error(f"點位管理器中找不到點位: {point_name}")
                return False
            
            # 提取運動參數
            speed_j = params.get('speed_j', 50)
            acc_j = params.get('acc_j', 50)
            tool = params.get('tool', 0)
            sync_enabled = params.get('sync', False)
            
            # 動態獲取Flow1角度數據
            j4_value = self._get_flow1_angle_or_default(point)
            
            self.logger.info(f"移動到點位 {point_name} (動態角度獲取)")
            self.logger.debug(f"  關節角度: (j1:{point.j1:.1f}, j2:{point.j2:.1f}, j3:{point.j3:.1f}, j4:{j4_value:.1f})")
            self.logger.debug(f"  角度來源: {self.angle_source}")
            self.logger.debug(f"  運動參數: speed_j={speed_j}, acc_j={acc_j}, tool={tool}")
            
            success = False
            if move_type in ['J', 'JointMovJ']:
                # 設置關節運動參數
                if hasattr(self.robot, 'set_speed_j'):
                    self.robot.set_speed_j(speed_j)
                if hasattr(self.robot, 'set_acc_j'):
                    self.robot.set_acc_j(acc_j)
                if hasattr(self.robot, 'set_tool'):
                    self.robot.set_tool(tool)
                
                # 使用關節角度運動，J4使用動態獲取的角度
                success = self.robot.joint_move_j(point.j1, point.j2, point.j3, j4_value)
            else:
                self.logger.error(f"J4角度控制僅支援關節運動(J)，當前類型: {move_type}")
                return False
            
            # Sync控制
            if success and sync_enabled:
                self.robot.sync()
                self.logger.info(f"  移動到 {point_name} 成功 (J4:{j4_value:.1f}°) (含Sync)")
            elif success:
                self.logger.info(f"  移動到 {point_name} 成功 (J4:{j4_value:.1f}°)")
                
            return success
                
        except Exception as e:
            self.logger.error(f"移動到點位(J4角度控制)失敗: {e}", exc_info=True)
            return False
    
    def _execute_arm_orientation_change(self, params: Dict[str, Any]) -> bool:
        """執行機械臂手勢切換 (SetArmOrientation)"""
        try:
            orientation = params.get('orientation', 0)
            
            # 手勢定義
            orientation_names = {
                0: "右手手勢 (Right)",
                1: "左手手勢 (Left)" 
            }
            
            orientation_name = orientation_names.get(orientation, f"未知手勢({orientation})")
            self.logger.info(f"切換機械臂手勢到: {orientation_name}")
            
            # 方法1: 直接調用機械臂API的SetArmOrientation()方法
            if hasattr(self.robot, 'dashboard_api') and self.robot.dashboard_api:
                try:
                    if hasattr(self.robot.dashboard_api, 'SetArmOrientation'):
                        result = self.robot.dashboard_api.SetArmOrientation(orientation)
                        self.logger.info(f"機械臂手勢切換成功: SetArmOrientation({orientation})")
                        self.logger.debug(f"API回應: {result}")
                        return True
                    else:
                        self.logger.warning("機械臂API不支援SetArmOrientation方法")
                        return True
                except Exception as e:
                    self.logger.error(f"機械臂手勢切換失敗: {e}")
                    return False
            
            # 方法2: 如果機械臂有set_arm_orientation方法
            elif hasattr(self.robot, 'set_arm_orientation'):
                try:
                    success = self.robot.set_arm_orientation(orientation)
                    if success:
                        self.logger.info(f"機械臂手勢切換成功: set_arm_orientation({orientation})")
                    else:
                        self.logger.error(f"機械臂手勢切換失敗: set_arm_orientation({orientation})")
                    return success
                except Exception as e:
                    self.logger.error(f"機械臂手勢切換異常: {e}")
                    return False
            
            # 方法3: 如果有move_api可以發送SetArmOrientation指令
            elif hasattr(self.robot, 'move_api') and self.robot.move_api:
                try:
                    command = f"SetArmOrientation({orientation})"
                    result = self.robot.move_api.sendRecvMsg(command)
                    self.logger.info(f"機械臂手勢切換指令發送: {command}")
                    self.logger.debug(f"API回應: {result}")
                    return True
                except Exception as e:
                    self.logger.error(f"機械臂手勢切換指令發送失敗: {e}")
                    return False
            
            else:
                self.logger.warning("機械臂API不支援手勢切換，跳過此步驟")
                return True
                
        except Exception as e:
            self.logger.error(f"機械臂手勢切換異常: {e}", exc_info=True)
            return False
    
    def _get_flow1_angle_or_default(self, point: RobotPoint) -> float:
        """動態獲取Flow1角度或使用預設值"""
        try:
            # 策略1: 嘗試從Flow1獲取角度數據
            if self._try_get_angle_from_flow1_direct():
                self.logger.debug(f"[動態角度獲取] 成功獲取Flow1角度: {self.command_angle:.2f}°")
                return self.command_angle
            
            # 策略2: 使用點位原始J4值
            self.logger.debug(f"[動態角度獲取] Flow1角度不可用，使用點位原始J4值: {point.j4:.2f}°")
            self.angle_source = "point_original"
            return point.j4
            
        except Exception as e:
            self.logger.error(f"[動態角度獲取] 異常: {e}，使用點位原始J4值", exc_info=True)
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
            self.logger.error(f"[Flow1角度獲取] 異常: {e}", exc_info=True)
            self.angle_source = "flow1_exception"
            return False
    
    def _execute_angle_detection_fast(self) -> bool:
        """執行AngleHighLevel角度檢測 - 快速版本 (備用方案)"""
        try:
            self.logger.info("[快速角度檢測] 開始檢測...")
            detection_start_time = time.time()
            
            # 優先使用external_modules中的angle模組
            angle_controller = self.external_modules.get('angle')
            
            if not angle_controller:
                self.logger.warning("[快速角度檢測] 角度模組未連接，快速使用預設值")
                self._set_default_angle("self_detect_failed")
                return True
            
            self.logger.info("[快速角度檢測] 使用外部模組中的角度API")
            
            # 檢查角度控制器連接狀態
            if hasattr(angle_controller, 'connected') and not angle_controller.connected:
                self.logger.warning("[快速角度檢測] 角度控制器未連接，嘗試快速重連...")
                if not angle_controller.connect():
                    self.logger.warning("[快速角度檢測] 快速重連失敗，使用預設值")
                    self._set_default_angle("self_detect_failed")
                    return True
            
            # 執行快速角度檢測
            self.logger.info("[快速角度檢測] 執行CCD3角度檢測(DR模式)...")
            
            try:
                # 使用超時機制執行角度檢測
                detection_result = self._execute_angle_detection_with_timeout(angle_controller)
                
                if detection_result is None:
                    self.logger.warning(f"[快速角度檢測] 檢測超時({self.angle_detection_timeout}秒)，使用預設值")
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
                    
                    self.logger.info(f"[快速角度檢測] 角度檢測成功 (耗時: {detection_time:.2f}秒):")
                    self.logger.info(f"    目標角度: {self.target_angle:.2f}°")
                    self.logger.info(f"    指令角度: {self.command_angle:.2f}°")
                    self.logger.info("    角度來源: 自檢測")
                    
                    return True
                else:
                    error_msg = getattr(detection_result, 'message', '未知錯誤') if detection_result else '檢測失敗'
                    self.logger.warning(f"[快速角度檢測] 檢測失敗: {error_msg} (耗時: {detection_time:.2f}秒)")
                    self._set_default_angle("self_detect_failed")
                    return True
                    
            except Exception as detection_error:
                detection_time = time.time() - detection_start_time
                self.logger.error(f"[快速角度檢測] 檢測異常: {detection_error} (耗時: {detection_time:.2f}秒)")
                self._set_default_angle("self_detect_error")
                return True
                
        except Exception as e:
            detection_time = time.time() - detection_start_time
            self.logger.error(f"[快速角度檢測] 系統異常: {e} (耗時: {detection_time:.2f}秒)", exc_info=True)
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
                self.logger.debug(f"角度檢測線程異常: {e}")
                result_queue.put(None)
        
        # 啟動檢測線程
        thread = threading.Thread(target=detection_thread, daemon=True)
        thread.start()
        
        # 等待結果或超時
        try:
            result = result_queue.get(timeout=self.angle_detection_timeout)
            return result
        except queue.Empty:
            self.logger.warning(f"[快速角度檢測] 檢測超時 ({self.angle_detection_timeout}秒)")
            return None
    
    def _set_default_angle(self, reason: str = "default"):
        """設置預設角度值"""
        self.target_angle = 0.0
        self.command_angle = self.target_angle + self.ANGLE_OFFSET
        self.angle_acquisition_success = False
        self.angle_source = reason
        self.logger.info(f"[角度設置] 使用預設角度: target={self.target_angle}°, command={self.command_angle}°")
        self.logger.debug(f"[角度設置] 原因: {reason}")
    
    def _execute_gripper_close(self) -> bool:
        """執行夾爪關閉"""
        try:
            gripper_api = self.external_modules.get('gripper')
            if gripper_api:
                success = gripper_api.quick_close()
                if success:
                    self.logger.info("夾爪關閉成功")
                else:
                    self.logger.error("夾爪關閉失敗")
                return success
            else:
                self.logger.error("夾爪API未初始化")
                return False
        except Exception as e:
            self.logger.error(f"夾爪關閉失敗: {e}", exc_info=True)
            return False
    
    def _execute_gripper_smart_release(self, params: Dict[str, Any]) -> bool:
        """執行夾爪智能撐開"""
        try:
            position = params.get('position', 370)
            self.logger.info(f"夾爪智能撐開到位置: {position}")
            
            gripper_api = self.external_modules.get('gripper')
            if not gripper_api:
                self.logger.error("夾爪API未初始化")
                return False
            
            # 執行智能撐開操作
            success = gripper_api.smart_release(position)
            
            if success:
                self.logger.info("夾爪智能撐開指令發送成功")
                
                # 等待夾爪撐開操作完全完成
                self.logger.debug("等待夾爪撐開動作完成...")
                time.sleep(0.3)
                
                self.logger.info(f"夾爪智能撐開完成 - 位置{position}")
                return True
            else:
                self.logger.error("夾爪智能撐開失敗")
                return False
                
        except Exception as e:
            self.logger.error(f"夾爪智能撐開異常: {e}", exc_info=True)
            return False
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        self.logger.info("Flow2已暫停")
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            self.logger.info("Flow2已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow"""
        self.status = FlowStatus.ERROR
        self.logger.info("Flow2已停止")
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