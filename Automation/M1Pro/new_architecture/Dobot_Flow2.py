#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DR_Dobot_Flow2.py - 出料流程 (新架構版本)
適配新架構混合交握協議，包含角度獲取和J4角度控制
基地址1200-1249，狀態機交握，序列化執行
"""

import time
from typing import Dict, Any, Optional
from dataclasses import dataclass


@dataclass
class FlowResult:
    """流程執行結果"""
    success: bool
    error_message: str = ""
    execution_time: float = 0.0
    steps_completed: int = 0
    total_steps: int = 15  # 新流程步驟數
    target_angle: Optional[float] = None      # 獲取的目標角度
    command_angle: Optional[float] = None     # 計算的指令角度
    angle_acquisition_success: bool = False   # 角度獲取是否成功
    extra_data: Dict[str, Any] = None         # 額外數據

    def __post_init__(self):
        if self.extra_data is None:
            self.extra_data = {}


class DrFlow2UnloadExecutor:
    """
    DR版本 出料流程執行器 (新架構版本)
    - 適配新架構混合交握協議
    - 包含AngleHighLevel角度獲取功能
    - 支援J4角度控制的組裝動作
    """
    
    def __init__(self):
        """初始化流程執行器 - 新架構版本"""
        # 核心組件 (通過initialize方法設置)
        self.robot = None
        self.motion_state_machine = None
        self.external_modules = {}
        
        # 流程配置
        self.flow_id = 2
        self.total_steps = 15  # 新流程步驟數
        self.current_step = 0
        self.is_running = False
        self.last_error = ""
        
        # 角度控制參數
        self.target_angle = None      # 從AngleHighLevel獲取的角度
        self.command_angle = None     # 計算後的指令角度 (target_angle + 45)
        self.angle_acquisition_success = False
        
        # 流程參數
        self.SPEED_RATIO = 100
        self.JOINT_TOLERANCE = 5.0  # 關節位置容差(%)
        self.GRIP_OPEN_POSITION = 370  # 撐開位置
        self.GRIP_CLOSE_POSITION = 0   # 關閉位置
        self.ANGLE_OFFSET = 45.0       # 角度偏移量
        
        # 必要點位列表 (按新流程順序)
        self.REQUIRED_POINTS = [
            "standby",               # 起點和終點
            "Rotate_V2",            # 第一個旋轉點
            "Rotate_top",           # 旋轉頂部點
            "Rotate_down",          # 旋轉底部點(撐開料件處)
            "put_asm_Pre",          # 組裝預備位置
            "put_asm_top",          # 組裝頂部位置
            "put_asm_down",         # 組裝放下位置
            "back_standby_from_asm" # 從組裝區回程的中轉點
        ]
        
        print("✓ DrFlow2UnloadExecutor初始化完成 (新架構版本)")
    
    def initialize(self, robot, motion_state_machine, external_modules):
        """初始化流程執行器 - 新架構版本"""
        self.robot = robot
        self.motion_state_machine = motion_state_machine
        self.external_modules = external_modules
        
        # 快速取得外部模組引用
        self.gripper = external_modules.get('gripper')
        self.ccd1 = external_modules.get('ccd1')
        self.ccd3 = external_modules.get('ccd3') 
        self.angle = external_modules.get('angle')
        
        print(f"✓ DR Flow2執行器初始化完成")
        print(f"  可用模組: Gripper={self.gripper is not None}, CCD1={self.ccd1 is not None}, "
              f"CCD3={self.ccd3 is not None}, Angle={self.angle is not None}")
    
    def execute(self) -> FlowResult:
        """執行DR 出料流程 (新架構版本)"""
        print("\n" + "="*60)
        print("開始執行DR Flow2 - 出料流程 (新架構版本)")
        print("="*60)
        
        start_time = time.time()
        self.is_running = True
        self.current_step = 0
        self.last_error = ""
        
        # 重置角度參數
        self.target_angle = None
        self.command_angle = None
        self.angle_acquisition_success = False
        
        try:
            # 步驟1: 系統檢查和起點位置驗證
            if not self._execute_step(1, "系統檢查和起點位置驗證", self._step_system_check_and_position):
                return self._create_result(False, start_time)
            
            # 步驟2: 移動到standby
            if not self._execute_step(2, "移動到standby", self._step_move_to_standby):
                return self._create_result(False, start_time)
            
            # 步驟3: 快速關閉夾爪
            if not self._execute_step(3, "快速關閉夾爪", self._step_quick_close_gripper):
                return self._create_result(False, start_time)
            
            # 步驟4: AngleHighLevel取得角度值並計算指令角度
            if not self._execute_step(4, "AngleHighLevel取得角度值", self._step_get_angle_from_angle_high_level):
                return self._create_result(False, start_time)
            
            # ===== 連續運動段1: 移動到撐開位置 =====
            print("  ▶ 開始連續運動段1 (步驟5-7): 移動到撐開位置...")
            
            # 步驟5: 移動到Rotate_V2
            if not self._execute_step(5, "移動到Rotate_V2", 
                                    lambda: self._step_move_to_point_no_sync("Rotate_V2")):
                return self._create_result(False, start_time)
            
            # 步驟6: 移動到Rotate_top
            if not self._execute_step(6, "移動到Rotate_top", 
                                    lambda: self._step_move_to_point_no_sync("Rotate_top")):
                return self._create_result(False, start_time)
            
            # 步驟7: 移動到Rotate_down
            if not self._execute_step(7, "移動到Rotate_down", 
                                    lambda: self._step_move_to_point_no_sync("Rotate_down")):
                return self._create_result(False, start_time)
            
            # 步驟8: 智慧撐開夾爪370
            if not self._execute_step(8, "智慧撐開夾爪370", self._step_smart_grip_open_sync):
                return self._create_result(False, start_time)
            
            # ===== 連續運動段2: 移動到組裝位置 =====
            print("  ▶ 開始連續運動段2 (步驟9-11): 移動到組裝位置...")
            
            # 步驟9: 移動到Rotate_top
            if not self._execute_step(9, "移動到Rotate_top", 
                                    lambda: self._step_move_to_point_no_sync("Rotate_top")):
                return self._create_result(False, start_time)
            
            # 步驟10: 移動到put_asm_Pre
            if not self._execute_step(10, "移動到put_asm_Pre", 
                                     lambda: self._step_move_to_point_no_sync("put_asm_Pre")):
                return self._create_result(False, start_time)
            
            # 步驟11: 移動到put_asm_top (帶入J4角度)
            if not self._execute_step(11, "移動到put_asm_top (J4角度控制)", 
                                     lambda: self._step_move_to_put_asm_top_with_angle_no_sync()):
                return self._create_result(False, start_time)
            
            # 步驟12: 移動到put_asm_down (帶入J4角度)
            if not self._execute_step(12, "移動到put_asm_down (J4角度控制)", 
                                     lambda: self._step_move_to_put_asm_down_with_angle_no_sync()):
                return self._create_result(False, start_time)
            
            # 步驟13: 夾爪快速關閉
            if not self._execute_step(13, "夾爪快速關閉", self._step_quick_close_release_sync):
                return self._create_result(False, start_time)
            
            # ===== 連續運動段3: 回到standby =====
            print("  ▶ 開始連續運動段3 (步驟14-15): 回到standby...")
            
            # 步驟14: 移動到put_asm_top (帶入J4角度)
            if not self._execute_step(14, "移動到put_asm_top (J4角度控制)", 
                                     lambda: self._step_move_to_put_asm_top_with_angle_no_sync()):
                return self._create_result(False, start_time)
            
            final_movements = [
                (15, "移動到put_asm_Pre", "put_asm_Pre"),
                (16, "移動到back_standby_from_asm", "back_standby_from_asm"),
                (17, "回到standby", "standby")
            ]
            
            # 注意：這裡需要調整步驟編號，因為我們總步驟是15，但實際有17個動作
            # 我們將15步驟拆分為多個子動作
            for i, (sub_step, step_name, point_name) in enumerate(final_movements):
                sub_progress = 15 + (i * 0.33)  # 在步驟15內部分配進度
                print(f"[{sub_progress:.1f}/15] {step_name}...")
                
                if not self._step_move_to_point_no_sync(point_name):
                    self.last_error = f"{step_name}失敗"
                    return self._create_result(False, start_time)
                print(f"  ✓ {step_name}指令已發送")
            
            # 最終sync確保所有運動完成
            if self.robot:
                if hasattr(self.robot, 'sync'):
                    self.robot.sync()
                print("  ✓ 所有運動已完成，機械臂已回到standby點")
            
            # 流程完成
            execution_time = time.time() - start_time
            print(f"\n✓ DR Flow2執行完成！總耗時: {execution_time:.2f}秒")
            
            # 顯示角度控制資訊
            if self.angle_acquisition_success:
                print(f"角度控制: 目標角度={self.target_angle:.2f}°, 指令角度={self.command_angle:.2f}°")
            else:
                print("⚠️ 角度獲取失敗，使用預設角度控制")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.total_steps,
                total_steps=self.total_steps,
                target_angle=self.target_angle,
                command_angle=self.command_angle,
                angle_acquisition_success=self.angle_acquisition_success
            )
            
        except Exception as e:
            self.last_error = f"DR Flow2執行異常: {str(e)}"
            print(f"✗ {self.last_error}")
            return FlowResult(
                success=False,
                error_message=self.last_error,
                execution_time=time.time() - start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                target_angle=self.target_angle,
                command_angle=self.command_angle,
                angle_acquisition_success=self.angle_acquisition_success
            )
        
        finally:
            self.is_running = False
    
    # =================================================================
    # 新增步驟：AngleHighLevel角度獲取
    # =================================================================
    
    def _step_get_angle_from_angle_high_level(self) -> bool:
        """步驟4: AngleHighLevel取得角度值並計算指令角度"""
        print("  正在從AngleHighLevel獲取角度值...")
        
        # 優先使用external_modules中的angle模組
        angle_controller = self.angle
        
        if not angle_controller:
            print("  ✗ 角度模組未連接，嘗試動態導入...")
            # 動態導入AngleHighLevel
            try:
                from AngleHighLevel import AngleHighLevel
                angle_controller = AngleHighLevel()
                print("  ✓ 成功導入AngleHighLevel")
                
                # 測試連接
                if not angle_controller.connect():
                    print("  ✗ AngleHighLevel連接失敗，使用預設角度")
                    self.target_angle = 0.0
                    self.command_angle = self.target_angle + self.ANGLE_OFFSET
                    self.angle_acquisition_success = False
                    print(f"  使用預設: target_angle={self.target_angle}°, command_angle={self.command_angle}°")
                    return True
                    
            except ImportError as e:
                print(f"  ✗ 無法導入AngleHighLevel: {e}")
                self.target_angle = 0.0
                self.command_angle = self.target_angle + self.ANGLE_OFFSET
                self.angle_acquisition_success = False
                print(f"  使用預設: target_angle={self.target_angle}°, command_angle={self.command_angle}°")
                return True
        else:
            print("  ✓ 使用外部模組中的角度API")
        
        try:
            # 獲取當前角度值
            angle_result = angle_controller.get_current_angle()
            
            if angle_result and hasattr(angle_result, 'success') and angle_result.success:
                self.target_angle = angle_result.angle
                self.command_angle = self.target_angle + self.ANGLE_OFFSET
                self.angle_acquisition_success = True
                
                print(f"  ✓ 角度獲取成功:")
                print(f"    目標角度 (target_angle): {self.target_angle:.2f}°")
                print(f"    指令角度 (command_angle): {self.command_angle:.2f}°")
                print(f"    角度偏移: {self.ANGLE_OFFSET}°")
                
                return True
            else:
                error_msg = getattr(angle_result, 'message', '未知錯誤') if angle_result else '無響應'
                print(f"  ✗ 角度獲取失敗: {error_msg}")
                
                # 使用預設角度
                self.target_angle = 0.0
                self.command_angle = self.target_angle + self.ANGLE_OFFSET
                self.angle_acquisition_success = False
                
                print(f"  使用預設角度: target_angle={self.target_angle}°, command_angle={self.command_angle}°")
                return True
                
        except Exception as e:
            print(f"  ✗ 角度獲取過程異常: {e}")
            
            # 使用預設角度
            self.target_angle = 0.0
            self.command_angle = self.target_angle + self.ANGLE_OFFSET
            self.angle_acquisition_success = False
            
            print(f"  使用預設角度: target_angle={self.target_angle}°, command_angle={self.command_angle}°")
            return True
        
        finally:
            # 確保斷開連接 (如果是動態創建的)
            if angle_controller and angle_controller != self.angle:
                try:
                    angle_controller.disconnect()
                    print("  角度控制系統連接已斷開")
                except:
                    pass
    
    # =================================================================
    # 新增步驟：帶J4角度控制的移動
    # =================================================================
    
    def _step_move_to_put_asm_top_with_angle_no_sync(self) -> bool:
        """移動到put_asm_top但J4角度帶入command_angle (無sync版本)"""
        if not self.robot:
            self.last_error = "機械臂控制器不可用"
            return False
        
        # 獲取put_asm_top點位
        if hasattr(self.robot, 'points_manager'):
            point = self.robot.points_manager.get_point("put_asm_top")
            if not point:
                self.last_error = "找不到put_asm_top點位"
                return False
            
            # 使用原點位的J1,J2,J3，但J4使用command_angle
            target_j4 = self.command_angle if self.command_angle is not None else 0.0
            
            print(f"    put_asm_top: J1={point.j1:.2f}°, J2={point.j2:.2f}°, J3={point.j3:.2f}°, J4={target_j4:.2f}° (角度控制)")
            
            # 使用關節運動
            if hasattr(self.robot, 'JointMovJ'):
                if not self.robot.JointMovJ(point.j1, point.j2, point.j3, target_j4):
                    self.last_error = "關節運動到put_asm_top失敗"
                    return False
            elif hasattr(self.robot, 'joint_move_j'):
                if not self.robot.joint_move_j(point.j1, point.j2, point.j3, target_j4):
                    self.last_error = "關節運動到put_asm_top失敗"
                    return False
            else:
                # 降級處理：使用基本MovJ
                if not self._step_move_to_point_no_sync("put_asm_top"):
                    return False
                print("    ⚠️ 使用基本MovJ，未能應用J4角度控制")
                return True
                
        else:
            # 簡化版本：使用預設座標
            target_j4 = self.command_angle if self.command_angle is not None else 0.0
            if hasattr(self.robot, 'joint_move_j'):
                # 使用預設的put_asm_top關節角度
                if not self.robot.joint_move_j(150.0, 45.0, 30.0, target_j4):
                    self.last_error = "關節運動到put_asm_top失敗"
                    return False
            else:
                return self._step_move_to_point_no_sync("put_asm_top")
        
        print(f"  移動到put_asm_top指令已發送 (J4={target_j4:.2f}°)")
        return True
    
    def _step_move_to_put_asm_down_with_angle_no_sync(self) -> bool:
        """移動到put_asm_down但J4角度帶入command_angle (無sync版本)"""
        if not self.robot:
            self.last_error = "機械臂控制器不可用"
            return False
        
        # 獲取put_asm_down點位
        if hasattr(self.robot, 'points_manager'):
            point = self.robot.points_manager.get_point("put_asm_down")
            if not point:
                self.last_error = "找不到put_asm_down點位"
                return False
            
            # 使用原點位的J1,J2,J3，但J4使用command_angle
            target_j4 = self.command_angle if self.command_angle is not None else 0.0
            
            print(f"    put_asm_down: J1={point.j1:.2f}°, J2={point.j2:.2f}°, J3={point.j3:.2f}°, J4={target_j4:.2f}° (角度控制)")
            
            # 使用關節運動
            if hasattr(self.robot, 'JointMovJ'):
                if not self.robot.JointMovJ(point.j1, point.j2, point.j3, target_j4):
                    self.last_error = "關節運動到put_asm_down失敗"
                    return False
            elif hasattr(self.robot, 'joint_move_j'):
                if not self.robot.joint_move_j(point.j1, point.j2, point.j3, target_j4):
                    self.last_error = "關節運動到put_asm_down失敗"
                    return False
            else:
                # 降級處理：使用基本MovJ
                if not self._step_move_to_point_no_sync("put_asm_down"):
                    return False
                print("    ⚠️ 使用基本MovJ，未能應用J4角度控制")
                return True
                
        else:
            # 簡化版本：使用預設座標
            target_j4 = self.command_angle if self.command_angle is not None else 0.0
            if hasattr(self.robot, 'joint_move_j'):
                # 使用預設的put_asm_down關節角度
                if not self.robot.joint_move_j(150.0, 55.0, 10.0, target_j4):
                    self.last_error = "關節運動到put_asm_down失敗"
                    return False
            else:
                return self._step_move_to_point_no_sync("put_asm_down")
        
        print(f"  移動到put_asm_down指令已發送 (J4={target_j4:.2f}°)")
        return True
    
    # =================================================================
    # 輔助方法 (新架構版本適配)
    # =================================================================
    
    def _execute_step(self, step_num: int, step_name: str, step_func) -> bool:
        """執行單個步驟並更新進度 - 新架構版本"""
        self.current_step = step_num
        self._update_progress()
        
        print(f"[{step_num}/{self.total_steps}] {step_name}...")
        
        step_start = time.time()
        success = step_func()
        step_time = time.time() - step_start
        
        if success:
            print(f"  ✓ {step_name}完成 (耗時: {step_time*1000:.1f}ms)")
            return True
        else:
            print(f"  ✗ {step_name}失敗")
            return False
    
    def _update_progress(self):
        """更新進度到狀態機 - 新架構版本"""
        if (self.motion_state_machine and 
            hasattr(self.motion_state_machine, 'modbus_client') and 
            self.motion_state_machine.modbus_client is not None):
            try:
                progress = int((self.current_step / self.total_steps) * 100)
                # 使用新架構地址1202
                try:
                    from Dobot_main import MotionRegisters
                    self.motion_state_machine.modbus_client.write_register(MotionRegisters.MOTION_PROGRESS, progress)
                except ImportError:
                    # 降級處理：使用固定地址
                    self.motion_state_machine.modbus_client.write_register(1202, progress)
            except Exception:
                pass
    
    def _create_result(self, success: bool, start_time: float) -> FlowResult:
        """創建流程結果 - 新架構版本"""
        return FlowResult(
            success=success,
            error_message=self.last_error,
            execution_time=time.time() - start_time,
            steps_completed=self.current_step,
            total_steps=self.total_steps,
            target_angle=self.target_angle,
            command_angle=self.command_angle,
            angle_acquisition_success=self.angle_acquisition_success
        )
    
    # =================================================================
    # 流程步驟實現 (新架構版本適配)
    # =================================================================
    
    def _step_system_check_and_position(self) -> bool:
        """步驟1: 系統檢查和起點位置驗證 - 新架構版本"""
        # 1. 系統檢查
        if not self.robot or not hasattr(self.robot, 'is_connected'):
            self.last_error = "機械臂控制器未初始化"
            return False
        
        if not self.robot.is_connected:
            self.last_error = "機械臂未連接"
            return False
        
        # 2. 檢查必要點位 (如果points_manager可用)
        if hasattr(self.robot, 'points_manager'):
            for point_name in self.REQUIRED_POINTS:
                if not self.robot.points_manager.get_point(point_name):
                    self.last_error = f"缺少必要點位: {point_name}"
                    return False
        else:
            print("  ⚠️ 無法檢查點位 (points_manager不可用)")
        
        # 3. 檢查夾爪狀態
        if self.gripper:
            status = self.gripper.get_status()
            if not status['connected']:
                self.last_error = "PGC夾爪未連接"
                return False
            print("  PGC夾爪狀態正常")
        
        print("  系統檢查通過")
        return True
    
    def _step_move_to_standby(self) -> bool:
        """步驟2: 移動到standby - 新架構版本"""
        if not self.robot:
            self.last_error = "機械臂控制器不可用"
            return False
        
        self.robot.set_global_speed(self.SPEED_RATIO)
        
        # 新架構版本：使用座標點或簡化移動
        if hasattr(self.robot, 'MovJ') and hasattr(self.robot, 'points_manager'):
            if not self.robot.MovJ("standby"):
                self.last_error = "移動到standby失敗"
                return False
        else:
            # 簡化版本：移動到固定座標
            if not self.robot.move_j(300, 0, 200, 0):
                self.last_error = "移動到standby失敗"
                return False
        
        if hasattr(self.robot, 'sync'):
            self.robot.sync()
        print("  移動到standby完成")
        return True
    
    def _step_quick_close_gripper(self) -> bool:
        """步驟3: 快速關閉夾爪 - 新架構版本"""
        if not self.gripper:
            print("  跳過夾爪關閉 (夾爪未啟用)")
            return True
        
        success = self.gripper.quick_close()
        
        if success:
            print("  PGC夾爪快速關閉完成")
        else:
            self.last_error = "PGC夾爪快速關閉失敗"
        
        return success
    
    # =================================================================
    # 關鍵sync點 - 夾爪調用前
    # =================================================================
    
    def _step_smart_grip_open_sync(self) -> bool:
        """步驟8: 智慧撐開夾爪370 (關鍵sync點)"""
        # 夾爪調用前先sync等待前面運動完成
        if hasattr(self.robot, 'sync'):
            self.robot.sync()
        print("  到達Rotate_down點")
        
        if not self.gripper:
            print("  跳過夾爪撐開 (夾爪未啟用)")
            return True
        
        # 智能張開撐開料件
        if not self.gripper.smart_grip(target_position=self.GRIP_OPEN_POSITION):
            self.last_error = f"智能張開夾爪到{self.GRIP_OPEN_POSITION}位置失敗"
            return False
        
        print(f"  智能張開夾爪到位置{self.GRIP_OPEN_POSITION}完成 (料件已撐開)")
        return True
    
    def _step_quick_close_release_sync(self) -> bool:
        """步驟13: 夾爪快速關閉 (關鍵sync點)"""
        # 夾爪調用前先sync等待前面運動完成
        if hasattr(self.robot, 'sync'):
            self.robot.sync()
        print("  到達put_asm_down點")
        
        if not self.gripper:
            print("  跳過夾爪關閉 (夾爪未啟用)")
            return True
        
        # 快速關閉放下料件
        if not self.gripper.quick_close():
            self.last_error = "快速關閉夾爪失敗"
            return False
        
        print(f"  快速關閉夾爪完成 (料件已放下)")
        return True
    
    # =================================================================
    # 無sync版本 - 連續運動優化
    # =================================================================
    
    def _step_move_to_point_no_sync(self, point_name: str) -> bool:
        """通用點位移動 (無sync版本) - 新架構版本"""
        if not self.robot:
            self.last_error = "機械臂控制器不可用"
            return False
        
        # 新架構版本：使用座標點或簡化移動
        if hasattr(self.robot, 'MovJ') and hasattr(self.robot, 'points_manager'):
            if not self.robot.MovJ(point_name):
                self.last_error = f"移動到{point_name}失敗"
                return False
        else:
            # 簡化版本：根據點位名稱使用預設座標
            point_coords = {
                "Rotate_V2": (200, 200, 180, 0),
                "Rotate_top": (200, 200, 250, 0),
                "Rotate_down": (200, 200, 120, 0),
                "put_asm_Pre": (150, 250, 200, 0),
                "put_asm_top": (150, 250, 250, 0),
                "put_asm_down": (150, 250, 120, 0),
                "back_standby_from_asm": (250, 150, 200, 0),
                "standby": (300, 0, 200, 0)
            }
            
            if point_name in point_coords:
                x, y, z, r = point_coords[point_name]
                if not self.robot.move_j(x, y, z, r):
                    self.last_error = f"移動到{point_name}失敗"
                    return False
            else:
                print(f"  ⚠️ 未知點位{point_name}，跳過")
                return True
        
        # 移除sync()，純指令發送
        print(f"  移動到{point_name}指令已發送")
        return True
    
    # =================================================================
    # 狀態查詢和控制方法 - 新架構版本
    # =================================================================
    
    def get_progress(self) -> int:
        """獲取當前進度百分比"""
        return int((self.current_step / self.total_steps) * 100)
    
    def get_status(self) -> Dict[str, Any]:
        """獲取流程狀態 - 新架構版本"""
        return {
            "flow_id": self.flow_id,
            "is_running": self.is_running,
            "current_step": self.current_step,
            "total_steps": self.total_steps,
            "progress_percent": self.get_progress(),
            "last_error": self.last_error,
            "required_points": self.REQUIRED_POINTS,
            "gripper_enabled": self.gripper is not None,
            "joint_tolerance_percent": self.JOINT_TOLERANCE,
            "grip_open_position": self.GRIP_OPEN_POSITION,
            "grip_close_position": self.GRIP_CLOSE_POSITION,
            "angle_offset": self.ANGLE_OFFSET,
            "target_angle": self.target_angle,
            "command_angle": self.command_angle,
            "angle_acquisition_success": self.angle_acquisition_success,
            "new_architecture_version": True,  # 標識新架構版本
            "angle_control_enabled": True,     # 標識角度控制功能
            "continuous_movement_segments": [
                "步驟5-7: Rotate_V2→Rotate_top→Rotate_down", 
                "步驟9-11: Rotate_top→put_asm_Pre→put_asm_top(J4控制)",
                "步驟14-17: put_asm_top(J4控制)→put_asm_Pre→back_standby_from_asm→standby"
            ]
        }
    
    def stop(self) -> bool:
        """停止流程執行 - 新架構版本"""
        try:
            self.is_running = False
            
            if self.robot:
                self.robot.emergency_stop()
            
            if self.gripper:
                self.gripper.stop()
            
            self.last_error = "DR Flow2流程已停止"
            return True
            
        except Exception as e:
            print(f"停止DR Flow2流程失敗: {e}")
            return False


# 兼容性別名
class Flow2Executor(DrFlow2UnloadExecutor):
    """Flow2執行器 - 兼容性包裝器"""
    pass


# ============================= 新架構修改說明 ===============================
# 
# 主要修改項目：
# 1. 類別名稱：DobotFlow2 → DrFlow2UnloadExecutor
# 2. 初始化方式：移除多參數構造，改為initialize()方法設置
# 3. 流程重構：按照指定順序重新設計15步驟流程
# 4. 角度控制：新增AngleHighLevel角度獲取和J4角度控制
# 5. 機械臂API適配：支援新架構的RealRobotController
# 6. 進度更新：使用新架構地址1202 (MotionRegisters.MOTION_PROGRESS)
# 7. 錯誤處理：適配新架構的錯誤管理機制
# 8. 模組引用：通過external_modules字典獲取外部模組
# 
# 新增功能：
# 1. AngleHighLevel角度獲取：步驟4獲取target_angle，計算command_angle
# 2. J4角度控制：put_asm_top和put_asm_down使用command_angle
# 3. 關節運動支援：優先使用JointMovJ或joint_move_j進行角度控制
# 4. 降級處理：如果角度控制失敗，自動降級到基本MovJ
# 5. 預設角度：如果AngleHighLevel不可用，使用預設角度0°
# 
# 流程改進：
# 1. 15步驟優化流程，包含3個連續運動段
# 2. 關鍵sync點：在夾爪操作前確保運動到位
# 3. 角度控制精確性：J4角度使用AngleHighLevel獲取的值+45°偏移
# 4. 完整的執行狀態記錄和錯誤處理
# 
# 相容性特點：
# 1. 支援新舊機械臂API (MovJ/JointMovJ 和 move_j/joint_move_j)
# 2. 兼容points_manager或使用預設座標
# 3. 外部模組未連接時的跳過邏輯
# 4. 完整的FlowResult包含角度控制資訊
# 
# 使用方式：
# 1. 在新架構主程序中創建：flow2 = DrFlow2UnloadExecutor()
# 2. 初始化：flow2.initialize(robot, motion_state_machine, external_modules)
# 3. 執行：result = flow2.execute()
# 4. 結果包含角度控制狀態和完整執行資訊