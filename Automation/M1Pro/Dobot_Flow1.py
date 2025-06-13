#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1.py - VP震動盤視覺抓取流程
被Dobot_main.py調用的流程執行器，不獨立運行
接收共享的機械臂、夾爪、視覺系統實例來執行具體流程
"""

import time
from typing import Dict, Any, Optional, List
from dataclasses import dataclass


@dataclass
class FlowResult:
    """流程執行結果"""
    success: bool
    error_message: str = ""
    execution_time: float = 0.0
    steps_completed: int = 0
    total_steps: int = 12


class DobotFlow1:
    """
    VP震動盤視覺抓取流程執行器
    基於MVP.py的流程邏輯，整合機械臂運動、夾爪控制和CCD視覺檢測
    """
    
    def __init__(self, robot, gripper, ccd1, ccd3, state_machine):
        """
        初始化流程執行器
        
        參數:
            robot: DobotM1Pro實例 (來自Dobot_main.py)
            gripper: PGCGripperController實例
            ccd1: CCD1VisionController實例  
            ccd3: CCD3AngleController實例 (本流程未使用)
            state_machine: DobotStateMachine實例
        """
        # 共享資源
        self.robot = robot
        self.gripper = gripper
        self.ccd1 = ccd1
        self.ccd3 = ccd3  # 備用，本流程暫未使用
        self.state_machine = state_machine
        
        # 流程配置
        self.flow_id = 1
        self.total_steps = 12
        self.current_step = 0
        self.is_running = False
        self.last_error = ""
        
        # 流程參數 (基於MVP.py的設定)
        self.SPEED_RATIO = 100          # 最大速度執行
        self.POINT_DELAY = 0.5          # 點位間延遲500ms
        self.GRIPPER_OPEN_POSITION = 370 # 夾爪撐開位置
        self.CCD1_DETECT_HEIGHT = 238.86 # CCD1檢測高度
        self.PICKUP_HEIGHT = 137.52     # 物體抓取高度
        
        # 必要點位列表
        self.REQUIRED_POINTS = [
            "standby",      # 待機點
            "Rotate_V2",    # 旋轉測試點1
            "Rotate_top",   # 旋轉上方點
            "Rotate_down",  # 旋轉下方點
            "VP_TOPSIDE"    # VP檢測位置
        ]
    
    def execute(self) -> FlowResult:
        """
        執行VP震動盤視覺抓取流程
        
        返回:
            FlowResult: 包含執行結果的數據結構
        """
        print("\n" + "="*60)
        print("開始執行流程1 - VP震動盤視覺抓取流程")
        print("="*60)
        
        start_time = time.time()
        self.is_running = True
        self.current_step = 0
        self.last_error = ""
        
        try:
            # 步驟1: 初始化檢查
            if not self._execute_step(1, "初始化檢查", self._step_initialize_check):
                return self._create_result(False, start_time)
            
            # 步驟2: 機械臂回到待機點  
            if not self._execute_step(2, "移動到待機點", self._step_move_to_standby):
                return self._create_result(False, start_time)
            
            # 步驟3: 夾爪關閉
            if not self._execute_step(3, "夾爪快速關閉", self._step_gripper_close):
                return self._create_result(False, start_time)
            
            # 步驟4-6: 測試動作序列 (基於MVP.py)
            test_points = ["Rotate_V2", "Rotate_top", "Rotate_down"]
            for i, point in enumerate(test_points):
                step_num = 4 + i
                if not self._execute_step(step_num, f"移動到{point}", 
                                        lambda p=point: self._step_move_to_point(p)):
                    return self._create_result(False, start_time)
            
            # 步驟7: 夾爪撐開測試
            if not self._execute_step(7, "夾爪撐開測試", self._step_gripper_open_test):
                return self._create_result(False, start_time)
            
            # 步驟8: 移動序列 - Rotate_top -> standby
            if not self._execute_step(8, "移動準備序列", self._step_move_preparation):
                return self._create_result(False, start_time)
            
            # 步驟9: 移動到VP檢測位置
            if not self._execute_step(9, "移動到VP檢測位置", self._step_move_to_vp):
                return self._create_result(False, start_time)
            
            # 步驟10: CCD1視覺檢測並抓取 (核心步驟)
            if not self._execute_step(10, "視覺檢測並抓取", self._step_vision_detection_pickup):
                return self._create_result(False, start_time)
            
            # 步驟11: 移動到安全位置
            if not self._execute_step(11, "移動到安全位置", self._step_move_to_safe):
                return self._create_result(False, start_time)
            
            # 步驟12: 回到待機點
            if not self._execute_step(12, "回到待機點", self._step_return_standby):
                return self._create_result(False, start_time)
            
            # 流程完成
            execution_time = time.time() - start_time
            print(f"\n✓ 流程1執行完成！總耗時: {execution_time:.2f}秒")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.total_steps,
                total_steps=self.total_steps
            )
            
        except Exception as e:
            self.last_error = f"流程執行異常: {str(e)}"
            print(f"✗ {self.last_error}")
            return self._create_result(False, start_time)
        
        finally:
            self.is_running = False
    
    def _execute_step(self, step_num: int, step_name: str, step_func) -> bool:
        """
        執行單個步驟並更新進度
        
        參數:
            step_num: 步驟編號
            step_name: 步驟名稱
            step_func: 步驟執行函數
            
        返回:
            bool: 執行成功返回True
        """
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
        """更新進度到狀態機"""
        if self.state_machine and hasattr(self.state_machine, 'modbus_client'):
            try:
                progress = int((self.current_step / self.total_steps) * 100)
                # 更新到寄存器403 (流程執行進度)
                self.state_machine.modbus_client.write_register(403, progress)
            except Exception as e:
                print(f"更新進度失敗: {e}")
    
    def _create_result(self, success: bool, start_time: float) -> FlowResult:
        """創建流程結果"""
        return FlowResult(
            success=success,
            error_message=self.last_error,
            execution_time=time.time() - start_time,
            steps_completed=self.current_step,
            total_steps=self.total_steps
        )
    
    # =================================================================
    # 流程步驟實現 (基於MVP.py的邏輯)
    # =================================================================
    
    def _step_initialize_check(self) -> bool:
        """步驟1: 初始化檢查"""
        # 檢查機械臂是否準備好
        if not self.robot.is_ready():
            self.last_error = "機械臂未準備好"
            return False
        
        # 檢查必要點位是否存在
        for point_name in self.REQUIRED_POINTS:
            if not self.robot.points_manager.get_point(point_name):
                self.last_error = f"缺少必要點位: {point_name}"
                return False
        
        # 檢查夾爪模組狀態 (如果有的話)
        if hasattr(self.gripper, 'check_module_status'):
            if not self.gripper.check_module_status():
                self.last_error = "夾爪模組狀態異常"
                return False
        
        return True
    
    def _step_move_to_standby(self) -> bool:
        """步驟2: 移動到待機點"""
        # 設置全局速度為最大
        self.robot.set_global_speed(self.SPEED_RATIO)
        
        # 移動到待機點
        if not self.robot.MovL("standby"):
            self.last_error = "移動到待機點失敗"
            return False
        
        # 等待運動完成
        self.robot.sync()
        return True
    
    def _step_gripper_close(self) -> bool:
        """步驟3: 夾爪快速關閉 (基於MVP.py優化邏輯)"""
        # 快速關閉不等待完成，提高執行效率
        if not self.gripper.close_fast():
            self.last_error = "夾爪快速關閉失敗"
            return False
        return True
    
    def _step_move_to_point(self, point_name: str) -> bool:
        """移動到指定點位"""
        if not self.robot.MovL(point_name):
            self.last_error = f"移動到{point_name}失敗"
            return False
        
        self.robot.sync()
        time.sleep(self.POINT_DELAY)  # 點位間延遲
        return True
    
    def _step_gripper_open_test(self) -> bool:
        """步驟7: 夾爪撐開測試 (基於MVP.py智能檢測邏輯)"""
        # 撐開到位置0進行測試
        if not self.gripper.open_to_position(0):
            self.last_error = "夾爪撐開測試失敗"
            return False
        return True
    
    def _step_move_preparation(self) -> bool:
        """步驟8: 移動準備序列 (基於MVP.py流程)"""
        # Rotate_top -> standby 移動序列
        if not self.robot.MovL("Rotate_top"):
            self.last_error = "移動到Rotate_top失敗"
            return False
        
        self.robot.sync()
        time.sleep(self.POINT_DELAY)
        
        if not self.robot.MovL("standby"):
            self.last_error = "移動到standby失敗"  
            return False
        
        self.robot.sync()
        return True
    
    def _step_move_to_vp(self) -> bool:
        """步驟9: 移動到VP檢測位置"""
        if not self.robot.MovL("VP_TOPSIDE"):
            self.last_error = "移動到VP_TOPSIDE失敗"
            return False
        
        self.robot.sync()
        return True
    
    def _step_vision_detection_pickup(self) -> bool:
        """步驟10: CCD1視覺檢測並抓取 (核心步驟)"""
        # 1. 觸發CCD1視覺檢測
        if not self.ccd1.capture_and_detect():
            self.last_error = "CCD1視覺檢測失敗"
            return False
        
        # 2. 獲取檢測結果
        detection_count = self.ccd1.get_detection_count()
        if detection_count == 0:
            self.last_error = "未檢測到物體"
            return False
        
        print(f"    檢測到 {detection_count} 個物體")
        
        # 3. 獲取第一個物體的世界座標
        world_coord = self.ccd1.get_object_center_world(1)
        if not world_coord or len(world_coord) < 2:
            self.last_error = "獲取物體世界座標失敗"
            return False
        
        print(f"    物體世界座標: X={world_coord[0]:.2f}mm, Y={world_coord[1]:.2f}mm")
        
        # 4. 移動到物體上方 (CCD1檢測高度)
        world_coord[2] = self.CCD1_DETECT_HEIGHT
        if not self.robot.MovL_coord(world_coord[0], world_coord[1], world_coord[2], 0):
            self.last_error = "移動到物體上方失敗"
            return False
        
        self.robot.sync()
        print(f"    已移動到物體上方，高度: {self.CCD1_DETECT_HEIGHT}mm")
        
        # 5. 下降到抓取高度
        world_coord[2] = self.PICKUP_HEIGHT  
        if not self.robot.MovL_coord(world_coord[0], world_coord[1], world_coord[2], 0):
            self.last_error = "下降到抓取高度失敗"
            return False
        
        self.robot.sync()
        print(f"    已下降到抓取高度: {self.PICKUP_HEIGHT}mm")
        
        # 6. 夾爪撐開抓取 (基於MVP.py智能檢測邏輯)
        if not self.gripper.open_to_position(self.GRIPPER_OPEN_POSITION):
            self.last_error = "夾爪撐開抓取失敗"
            return False
        
        print("    夾爪撐開抓取完成")
        return True
    
    def _step_move_to_safe(self) -> bool:
        """步驟11: 移動到安全位置"""
        if not self.robot.MovL("VP_TOPSIDE"):
            self.last_error = "移動到安全位置失敗"
            return False
        
        self.robot.sync()
        return True
    
    def _step_return_standby(self) -> bool:
        """步驟12: 回到待機點"""
        if not self.robot.MovL("standby"):
            self.last_error = "回到待機點失敗"
            return False
        
        self.robot.sync()
        return True
    
    # =================================================================
    # 狀態查詢和控制方法
    # =================================================================
    
    def get_progress(self) -> int:
        """獲取當前進度百分比"""
        return int((self.current_step / self.total_steps) * 100)
    
    def get_status(self) -> Dict[str, Any]:
        """獲取流程狀態"""
        return {
            "flow_id": self.flow_id,
            "is_running": self.is_running,
            "current_step": self.current_step,
            "total_steps": self.total_steps,
            "progress_percent": self.get_progress(),
            "last_error": self.last_error,
            "required_points": self.REQUIRED_POINTS
        }
    
    def stop(self) -> bool:
        """停止流程執行"""
        try:
            self.is_running = False
            
            # 緊急停止機械臂
            if self.robot:
                self.robot.emergency_stop()
            
            # 停止夾爪
            if hasattr(self.gripper, 'send_command'):
                self.gripper.send_command(2, wait_completion=False)  # 停止指令
            
            self.last_error = "流程已停止"
            return True
            
        except Exception as e:
            print(f"停止流程失敗: {e}")
            return False
    
    def handle_error(self, error_msg: str) -> bool:
        """處理錯誤"""
        self.last_error = error_msg
        print(f"流程錯誤: {error_msg}")
        
        # 自動執行緊急停止
        self.stop()
        return False


# =================================================================
# 兼容性包裝器 (為了與Dobot_main.py中的Flow1Executor兼容)
# =================================================================

class Flow1Executor(DobotFlow1):
    """
    Flow1執行器 - 兼容Dobot_main.py中的調用方式
    這是一個包裝器，確保與主控制器的接口一致
    """
    pass


# =================================================================
# 測試和調試功能
# =================================================================

def test_flow1_logic():
    """
    測試Flow1流程邏輯 (不需要實際硬體)
    用於開發階段的邏輯驗證
    """
    print("=== Flow1邏輯測試 ===")
    
    class MockRobot:
        def __init__(self):
            self.points_manager = MockPointsManager()
        
        def is_ready(self): return True
        def set_global_speed(self, speed): return True
        def MovL(self, point): return True
        def MovL_coord(self, x, y, z, r): return True
        def sync(self): pass
        def emergency_stop(self): return True
    
    class MockPointsManager:
        def get_point(self, name):
            # 模擬所有必要點位都存在
            return True if name in ["standby", "Rotate_V2", "Rotate_top", "Rotate_down", "VP_TOPSIDE"] else None
    
    class MockGripper:
        def check_module_status(self): return True
        def close_fast(self): return True
        def open_to_position(self, pos): return True
    
    class MockCCD1:
        def capture_and_detect(self): return True
        def get_detection_count(self): return 1
        def get_object_center_world(self, idx): return [100.0, 200.0, 0.0]
    
    class MockStateMachine:
        def __init__(self):
            self.modbus_client = None
    
    # 創建模擬對象
    mock_robot = MockRobot()
    mock_gripper = MockGripper()
    mock_ccd1 = MockCCD1()
    mock_state_machine = MockStateMachine()
    
    # 測試流程
    flow1 = DobotFlow1(mock_robot, mock_gripper, mock_ccd1, None, mock_state_machine)
    result = flow1.execute()
    
    print(f"測試結果: {'成功' if result.success else '失敗'}")
    print(f"執行時間: {result.execution_time:.2f}秒")
    print(f"完成步驟: {result.steps_completed}/{result.total_steps}")
    
    return result.success


# if __name__ == "__main__":
#     # 如果直接執行此檔案，運行邏輯測試
#     print("注意: 此模組設計為被Dobot_main.py調用")
#     print("正在執行邏輯測試...")
#     test_flow1_logic()