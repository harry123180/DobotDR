#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow1.py - VP震動盤視覺抓取流程 (連續運動優化版)
基於原版程式碼，優化連續運動段，減少sync()卡頓
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
    total_steps: int = 16


class DobotFlow1:
    """
    VP震動盤視覺抓取流程執行器 (連續運動優化版)
    減少sync()使用，提升連續運動流暢度
    """
    
    def __init__(self, robot, gripper, ccd1, ccd3, state_machine):
        """初始化流程執行器"""
        # 共享資源
        self.robot = robot
        self.gripper = gripper
        self.ccd1 = ccd1
        self.ccd3 = ccd3
        self.state_machine = state_machine
        
        # 流程配置
        self.flow_id = 1
        self.total_steps = 16
        self.current_step = 0
        self.is_running = False
        self.last_error = ""
        
        # 流程參數 - 優化後
        self.SPEED_RATIO = 100
        self.POINT_DELAY = 0.1  # 從0.5秒優化為0.1秒
        self.CCD1_DETECT_HEIGHT = 238.86
        self.PICKUP_HEIGHT = 137.52
        
        # 必要點位列表
        self.REQUIRED_POINTS = [
            "standby",
            "Rotate_V2",
            "Rotate_top", 
            "Rotate_down",
            "VP_TOPSIDE"
        ]
    
    def execute(self) -> FlowResult:
        """執行VP震動盤視覺抓取流程 (連續運動優化版)"""
        print("\n" + "="*60)
        print("開始執行流程1 - VP震動盤視覺抓取流程 (連續運動優化版)")
        print("="*60)
        
        start_time = time.time()
        self.is_running = True
        self.current_step = 0
        self.last_error = ""
        
        detected_coord = None
        
        try:
            # 步驟1: 系統檢查
            if not self._execute_step(1, "系統檢查", self._step_system_check):
                return self._create_result(False, start_time)
            
            # 步驟2: 夾爪快速關閉 (關鍵sync點)
            if not self._execute_step(2, "夾爪快速關閉", self._step_gripper_quick_close_sync):
                return self._create_result(False, start_time)
            
            # 步驟3: 移動到待機點 (CCD1檢測前sync)
            if not self._execute_step(3, "移動到待機點", self._step_move_to_standby_sync):
                return self._create_result(False, start_time)
            
            # 步驟4: CCD1檢測 (關鍵sync點)
            coord_result = self._execute_step_with_return(4, "CCD1視覺檢測", self._step_ccd1_detection)
            if coord_result is False:
                return self._create_result(False, start_time)
            detected_coord = coord_result
            
            # 步驟5-8: 視覺抓取流程 (必要時sync)
            if detected_coord:
                print(f"  檢測到物體 (FIFO佇列ID: {detected_coord.id})")
                print(f"  世界座標: ({detected_coord.world_x:.2f}, {detected_coord.world_y:.2f})mm, R={getattr(detected_coord, 'r', 0.0)}°")
                
                # 步驟5: 移動到VP_TOPSIDE (無sync，開始連續運動)
                if not self._execute_step(5, "移動到VP_TOPSIDE", self._step_move_to_vp_topside_no_sync):
                    return self._create_result(False, start_time)
                
                # 步驟6: 移動到物體上方 (無sync，連續運動)
                if not self._execute_step(6, "移動到物體上方", 
                                        lambda: self._step_move_to_object_above_no_sync(detected_coord)):
                    return self._create_result(False, start_time)
                
                # 步驟7: 下降並智能夾取 (關鍵sync點 - 夾爪調用前)
                if not self._execute_step(7, "下降並智能夾取", 
                                        lambda: self._step_descend_and_grip_sync(detected_coord)):
                    return self._create_result(False, start_time)
                
                # 步驟8: 上升並移動到VP_TOPSIDE (夾取後開始連續運動)
                if not self._execute_step(8, "上升並移動到VP_TOPSIDE", 
                                        lambda: self._step_ascend_and_move_to_vp_no_sync(detected_coord)):
                    return self._create_result(False, start_time)
            else:
                print("  未檢測到物體，跳過抓取流程")
                for step in range(5, 9):
                    self._execute_step(step, f"跳過步驟{step}", lambda: True)
            
            # 🔥 步驟9-12: 連續運動段優化 (夾取完成後的連續動作)
            print("  ▶ 開始連續運動段 (步驟9-12)...")
            
            # 步驟9: 移動到待機點 (無sync，連續運動開始)
            if not self._execute_step(9, "移動到待機點", self._step_move_to_standby_no_sync):
                return self._create_result(False, start_time)
            
            # 步驟10-12: 連續運動序列 (無中間sync)
            continuous_movements = [
                (10, "移動到Rotate_V2", "Rotate_V2"),
                (11, "移動到Rotate_top", "Rotate_top"), 
                (12, "移動到Rotate_down", "Rotate_down")
            ]
            
            for step_num, step_name, point_name in continuous_movements:
                if not self._execute_step(step_num, step_name, 
                                        lambda p=point_name: self._step_move_to_point_no_sync(p)):
                    return self._create_result(False, start_time)
            
            # 步驟13: 智能關閉 (關鍵sync點 - 夾爪調用前)
            if not self._execute_step(13, "智能關閉", self._step_smart_close_sync):
                return self._create_result(False, start_time)
            
            # 步驟14-16: 最後連續運動段 (夾爪操作後的連續運動)
            print("  ▶ 開始最後連續運動段 (步驟14-16)...")
            
            final_movements = [
                (14, "移動到Rotate_top", "Rotate_top"),
                (15, "移動到Rotate_V2", "Rotate_V2"),
                (16, "移動到standby", "standby")
            ]
            
            for step_num, step_name, point_name in final_movements:
                if not self._execute_step(step_num, step_name,
                                        lambda p=point_name: self._step_move_to_point_no_sync(p)):
                    return self._create_result(False, start_time)
            
            # 最終sync確保所有運動完成
            self.robot.sync()
            print("  ✓ 所有運動已完成")
            
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
        """執行單個步驟並更新進度"""
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
    
    def _execute_step_with_return(self, step_num: int, step_name: str, step_func):
        """執行單個步驟並返回結果"""
        self.current_step = step_num
        self._update_progress()
        
        print(f"[{step_num}/{self.total_steps}] {step_name}...")
        
        step_start = time.time()
        result = step_func()
        step_time = time.time() - step_start
        
        if result is not False:
            print(f"  ✓ {step_name}完成 (耗時: {step_time*1000:.1f}ms)")
            return result
        else:
            print(f"  ✗ {step_name}失敗")
            return False
    
    def _update_progress(self):
        """更新進度到狀態機"""
        if (self.state_machine and 
            hasattr(self.state_machine, 'modbus_client') and 
            self.state_machine.modbus_client is not None):
            try:
                progress = int((self.current_step / self.total_steps) * 100)
                self.state_machine.modbus_client.write_register(403, progress)
            except Exception:
                pass
    
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
    # 流程步驟實現 - 區分sync和no_sync版本
    # =================================================================
    
    def _step_system_check(self) -> bool:
        """步驟1: 系統檢查"""
        if not self.robot.is_ready():
            self.last_error = "機械臂未準備好"
            return False
        
        for point_name in self.REQUIRED_POINTS:
            if not self.robot.points_manager.get_point(point_name):
                self.last_error = f"缺少必要點位: {point_name}"
                return False
        
        if self.gripper:
            status = self.gripper.get_status()
            if not status['connected']:
                self.last_error = "PGC夾爪未連接"
                return False
            print("  PGC夾爪狀態正常")
        
        if self.ccd1:
            status = self.ccd1.get_system_status()
            if not status['connected']:
                print("  CCD1視覺系統未連接，但繼續執行")
            else:
                print("  CCD1視覺系統準備就緒")
        
        return True
    
    # =================================================================
    # 關鍵sync點 - 夾爪和CCD1調用前
    # =================================================================
    
    def _step_gripper_quick_close_sync(self) -> bool:
        """步驟2: 夾爪快速關閉 (關鍵sync點)"""
        if not self.gripper:
            print("  跳過夾爪關閉 (夾爪未啟用)")
            return True
        
        success = self.gripper.quick_close()
        
        if success:
            print("  PGC夾爪快速關閉完成")
        else:
            self.last_error = "PGC夾爪快速關閉失敗"
        
        return success
    
    def _step_move_to_standby_sync(self) -> bool:
        """步驟3: 移動到待機點 (CCD1檢測前sync)"""
        self.robot.set_global_speed(self.SPEED_RATIO)
        
        if not self.robot.MovJ("standby"):
            self.last_error = "移動到待機點失敗"
            return False
        
        # CCD1檢測前必須sync確保到位
        self.robot.sync()
        print("  移動到待機點完成 (CCD1檢測前)")
        return True
    
    def _step_ccd1_detection(self):
        """步驟4: CCD1檢測 (關鍵sync點)"""
        if not self.ccd1:
            print("  跳過CCD1檢測 (CCD1未啟用)")
            return None
        
        print("  在待機點進行CCD1視覺檢測...")
        
        # 優先從FIFO佇列獲取已有的檢測結果
        # 如果佇列為空，會自動觸發新的檢測
        coord = self.ccd1.get_next_circle_world_coord()
        
        if coord:
            # 🔥 修正R值 - 繼承VP_TOPSIDE的R值
            vp_topside_point = self.robot.points_manager.get_point("VP_TOPSIDE")
            if vp_topside_point and hasattr(vp_topside_point, 'r'):
                coord.r = vp_topside_point.r
                print(f"    繼承VP_TOPSIDE的R值: {coord.r}°")
            else:
                # 如果無法獲取VP_TOPSIDE的R值，使用預設值
                coord.r = 0.0
                print(f"    使用預設R值: {coord.r}°")
            
            print(f"    檢測成功: 世界座標=({coord.world_x:.2f}, {coord.world_y:.2f})mm, R={coord.r}°")
            print(f"    來源: FIFO佇列第{coord.id}個物體")
            return coord
        else:
            print("    未檢測到物體或佇列已空")
            return None
    
    def _step_descend_and_grip_sync(self, coord) -> bool:
        """步驟7: 下降並智能夾取 (關鍵sync點 - 夾爪調用前)"""
        if not coord:
            self.last_error = "沒有有效的物體座標"
            return False
        
        # 使用coord中的R值（已從VP_TOPSIDE繼承）
        r_value = getattr(coord, 'r', 0.0)
        
        # 下降到抓取高度
        if not self.robot.MovL_coord(coord.world_x, coord.world_y, self.PICKUP_HEIGHT, r_value):
            self.last_error = "下降到抓取高度失敗"
            return False
        
        # 夾爪調用前必須sync確保精確定位
        self.robot.sync()
        print(f"    下降到抓取高度完成: {self.PICKUP_HEIGHT}mm (R={r_value}°)")
        
        # 智能夾取
        if self.gripper:
            if not self.gripper.smart_grip(target_position=420):
                self.last_error = "智能夾取失敗"
                return False
            print("    智能夾取完成")
        
        return True
    
    def _step_smart_close_sync(self) -> bool:
        """步驟13: 智能關閉 (關鍵sync點 - 夾爪調用前)"""
        # 夾爪調用前先sync等待前面運動完成
        self.robot.sync()
        
        if not self.gripper:
            print("  跳過智能關閉 (夾爪未啟用)")
            return True
        
        if not self.gripper.smart_release(release_position=50):
            self.last_error = "智能關閉失敗"
            return False
        
        print("  智能關閉完成")
        return True
    
    # =================================================================
    # 無sync版本 - 連續運動優化
    # =================================================================
    
    def _step_move_to_vp_topside_no_sync(self) -> bool:
        """步驟5: 移動到VP_TOPSIDE (無sync版本)"""
        if not self.robot.MovJ("VP_TOPSIDE"):
            self.last_error = "移動到VP_TOPSIDE失敗"
            return False
        
        # 移除sync()，讓運動連續進行
        print("  移動到VP_TOPSIDE指令已發送")
        return True
    
    def _step_move_to_object_above_no_sync(self, coord) -> bool:
        """步驟6: 移動到物體上方 (無sync版本)"""
        if not coord:
            self.last_error = "沒有有效的物體座標"
            return False
        
        # 使用coord中的R值（已從VP_TOPSIDE繼承）
        r_value = getattr(coord, 'r', 0.0)
        
        if not self.robot.MovL_coord(coord.world_x, coord.world_y, self.CCD1_DETECT_HEIGHT, r_value):
            self.last_error = "移動到物體上方失敗"
            return False
        
        # 移除sync()，連續運動
        print(f"    移動到物體上方指令已發送 (R={r_value}°)")
        return True
    
    def _step_ascend_and_move_to_vp_no_sync(self, coord) -> bool:
        """步驟8: 上升並移動 (無sync版本)"""
        if not coord:
            self.last_error = "沒有有效的物體座標"
            return False
        
        # 使用coord中的R值（已從VP_TOPSIDE繼承）
        r_value = getattr(coord, 'r', 0.0)
        
        # 上升到安全高度
        if not self.robot.MovL_coord(coord.world_x, coord.world_y, self.CCD1_DETECT_HEIGHT, r_value):
            self.last_error = "上升到安全高度失敗"
            return False
        
        # 移動到VP_TOPSIDE
        if not self.robot.MovJ("VP_TOPSIDE"):
            self.last_error = "移動到VP_TOPSIDE失敗"
            return False
        
        # 移除sync()，讓運動連續
        print(f"    上升並移動指令已發送 (R={r_value}°)")
        return True
    
    def _step_move_to_standby_no_sync(self) -> bool:
        """步驟9&16: 移動到待機點 (無sync版本)"""
        self.robot.set_global_speed(self.SPEED_RATIO)
        
        if not self.robot.MovJ("standby"):
            self.last_error = "移動到待機點失敗"
            return False
        
        # 移除sync()，連續運動
        print("  移動到待機點指令已發送")
        return True
    
    def _step_move_to_point_no_sync(self, point_name: str) -> bool:
        """通用點位移動 (無sync版本) - 用於連續運動段"""
        if not self.robot.MovJ(point_name):
            self.last_error = f"移動到{point_name}失敗"
            return False
        
        # 移除sync()和sleep()，純指令發送
        print(f"  移動到{point_name}指令已發送")
        return True
    
    # =================================================================
    # 保留的sync版本 (向後兼容)
    # =================================================================
    
    def _step_move_to_point(self, point_name: str) -> bool:
        """通用點位移動方法 (保留sync版本)"""
        if not self.robot.MovJ(point_name):
            self.last_error = f"移動到{point_name}失敗"
            return False
        
        self.robot.sync()
        time.sleep(self.POINT_DELAY)
        print(f"  移動到{point_name}完成")
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
            "required_points": self.REQUIRED_POINTS,
            "gripper_enabled": self.gripper is not None,
            "ccd1_enabled": self.ccd1 is not None,
            "optimization_enabled": True,  # 標識已優化
            "continuous_movement_segments": ["步驟9-12: 待機點→Rotate_V2→Rotate_top→Rotate_down", "步驟14-16: Rotate_top→Rotate_V2→standby"]
        }
    
    def stop(self) -> bool:
        """停止流程執行"""
        try:
            self.is_running = False
            
            if self.robot:
                self.robot.emergency_stop()
            
            if self.gripper:
                self.gripper.stop()
            
            self.last_error = "流程已停止"
            return True
            
        except Exception as e:
            print(f"停止流程失敗: {e}")
            return False


class Flow1Executor(DobotFlow1):
    """Flow1執行器 - 兼容性包裝器"""
    pass


