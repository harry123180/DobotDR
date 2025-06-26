#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DR_Dobot_Flow1.py - VP震動盤視覺抓取流程 (新架構版本)
適配新架構混合交握協議，使用CCD1HighLevel.py的自動拍照檢測和FIFO管理機制
基地址1200-1249，狀態機交握，序列化執行
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
    total_steps: int = 17  # 包含角度校正步驟
    angle_correction_performed: bool = False  # 是否執行了角度校正
    angle_correction_success: bool = False    # 角度校正是否成功
    detected_angle: Optional[float] = None    # 檢測到的角度
    angle_difference: Optional[float] = None  # 角度差
    motor_position: Optional[int] = None      # 馬達位置
    angle_correction_error: Optional[str] = None  # 角度校正錯誤訊息
    # 新增CCD1相關結果
    ccd1_objects_processed: int = 0           # 本次流程處理的物體數量
    ccd1_detection_triggered: bool = False    # 是否觸發了自動拍照檢測
    need_refill: bool = False                 # 是否需要補料
    extra_data: Dict[str, Any] = None         # 額外數據

    def __post_init__(self):
        if self.extra_data is None:
            self.extra_data = {}


class DrFlow1VisionPickExecutor:
    """
    DR版本 VP震動盤視覺抓取流程執行器 (新架構版本)
    - 適配新架構混合交握協議
    - 使用新的CCD1HighLevel API自動拍照檢測和FIFO管理
    - 連續運動優化
    - 自動清零角度校正機制
    """
    
    def __init__(self):
        """初始化流程執行器 - 新架構版本"""
        # 核心組件 (通過initialize方法設置)
        self.robot = None
        self.motion_state_machine = None
        self.external_modules = {}
        
        # 流程配置
        self.flow_id = 1
        self.total_steps = 17  # 包含角度校正步驟
        self.current_step = 0
        self.is_running = False
        self.last_error = ""
        
        # 角度校正執行結果記錄
        self.angle_correction_performed = False
        self.angle_correction_success = False
        self.detected_angle = None
        self.angle_difference = None
        self.motor_position = None
        self.angle_correction_error = None
        
        # CCD1執行結果記錄
        self.ccd1_objects_processed = 0
        self.ccd1_detection_triggered = False
        self.need_refill = False
        
        # 流程參數
        self.SPEED_RATIO = 100
        self.POINT_DELAY = 0.1
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
        
        print("✓ DrFlow1VisionPickExecutor初始化完成 (新架構版本)")
    
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
        
        print(f"✓ DR Flow1執行器初始化完成")
        print(f"  可用模組: Gripper={self.gripper is not None}, CCD1={self.ccd1 is not None}, "
              f"CCD3={self.ccd3 is not None}, Angle={self.angle is not None}")
    
    def execute(self) -> FlowResult:
        """執行DR VP震動盤視覺抓取流程 (新架構版本)"""
        print("\n" + "="*60)
        print("開始執行DR Flow1 - VP震動盤視覺抓取流程 (新架構版本)")
        print("="*60)
        
        start_time = time.time()
        self.is_running = True
        self.current_step = 0
        self.last_error = ""
        
        # 重置執行記錄
        self.angle_correction_performed = False
        self.angle_correction_success = False
        self.detected_angle = None
        self.angle_difference = None
        self.motor_position = None
        self.angle_correction_error = None
        self.ccd1_objects_processed = 0
        self.ccd1_detection_triggered = False
        self.need_refill = False
        
        detected_coord = None
        
        try:
            # 步驟1-16: 視覺抓取流程
            # ===== 系統檢查和初始化 =====
            if not self._execute_step(1, "系統檢查", self._step_system_check):
                return self._create_result(False, start_time)
            
            if not self._execute_step(2, "夾爪快速關閉", self._step_gripper_quick_close_sync):
                return self._create_result(False, start_time)
            
            if not self._execute_step(3, "移動到待機點", self._step_move_to_standby_sync):
                return self._create_result(False, start_time)
            
            # ===== CCD1視覺檢測 (使用新API) =====
            coord_result = self._execute_step_with_return(4, "CCD1智能檢測", self._step_ccd1_smart_detection)
            if coord_result is False:
                return self._create_result(False, start_time)
            detected_coord = coord_result
            
            # ===== 視覺抓取流程 =====
            if detected_coord:
                print(f"  檢測到物體 (FIFO佇列ID: {detected_coord.id})")
                print(f"  世界座標: ({detected_coord.world_x:.2f}, {detected_coord.world_y:.2f})mm, R={getattr(detected_coord, 'r', 0.0)}°")
                self.ccd1_objects_processed = 1
                
                # 步驟5-8: 抓取動作序列
                if not self._execute_step(5, "移動到VP_TOPSIDE", self._step_move_to_vp_topside_no_sync):
                    return self._create_result(False, start_time)
                
                if not self._execute_step(6, "移動到物體上方", 
                                        lambda: self._step_move_to_object_above_no_sync(detected_coord)):
                    return self._create_result(False, start_time)
                
                if not self._execute_step(7, "下降並智能夾取", 
                                        lambda: self._step_descend_and_grip_sync(detected_coord)):
                    return self._create_result(False, start_time)
                
                if not self._execute_step(8, "上升並移動到VP_TOPSIDE", 
                                        lambda: self._step_ascend_and_move_to_vp_no_sync(detected_coord)):
                    return self._create_result(False, start_time)
            else:
                print("  未檢測到物體，需要補料 - 跳過抓取流程")
                self.need_refill = True
                for step in range(5, 9):
                    self._execute_step(step, f"跳過步驟{step} (需要補料)", lambda: True)
            
            # ===== 連續運動段 =====
            print("  ▶ 開始連續運動段 (步驟9-12)...")
            
            if not self._execute_step(9, "移動到待機點", self._step_move_to_standby_no_sync):
                return self._create_result(False, start_time)
            
            continuous_movements = [
                (10, "移動到Rotate_V2", "Rotate_V2"),
                (11, "移動到Rotate_top", "Rotate_top"),
                (12, "移動到Rotate_down", "Rotate_down")
            ]
            
            for step_num, step_name, point_name in continuous_movements:
                if not self._execute_step(step_num, step_name, 
                                        lambda p=point_name: self._step_move_to_point_no_sync(p)):
                    return self._create_result(False, start_time)
            
            if not self._execute_step(13, "智能關閉", self._step_smart_close_sync):
                return self._create_result(False, start_time)
            
            # ===== 最後連續運動段 =====
            print("  ▶ 開始最後連續運動段 (步驟14-16)...")
            
            final_movements = [
                (14, "移動到Rotate_top", "Rotate_top"),
                (15, "移動到Rotate_V2", "Rotate_V2"),
                (16, "回到待機點(角度校正前)", "standby")
            ]
            
            for step_num, step_name, point_name in final_movements:
                if not self._execute_step(step_num, step_name,
                                        lambda p=point_name: self._step_move_to_point_no_sync(p)):
                    return self._create_result(False, start_time)
            
            # 步驟17: 角度校正到90度 (新架構版本 - 使用自動清零機制)
            self.angle_correction_performed = True
            print(f"[17/{self.total_steps}] 角度校正到90度 (新架構版本 - 自動清零機制)...")
            
            angle_correction_result = self.execute_angle_correction_with_auto_clear()
            
            if not angle_correction_result:
                # 角度校正失敗，整體流程失敗
                print("  ✗ 角度校正失敗，DR Flow1整體執行失敗")
                execution_time = time.time() - start_time
                return FlowResult(
                    success=False,
                    error_message=f"角度校正失敗: {self.angle_correction_error or self.last_error}",
                    execution_time=execution_time,
                    steps_completed=self.current_step,
                    total_steps=self.total_steps,
                    angle_correction_performed=True,
                    angle_correction_success=False,
                    detected_angle=self.detected_angle,
                    angle_difference=self.angle_difference,
                    motor_position=self.motor_position,
                    angle_correction_error=self.angle_correction_error,
                    ccd1_objects_processed=self.ccd1_objects_processed,
                    ccd1_detection_triggered=self.ccd1_detection_triggered,
                    need_refill=self.need_refill
                )
            
            # 最終sync確保所有運動完成
            if self.robot:
                self.robot.sync()
                print("  ✓ 所有運動已完成")
            
            # 流程完成
            execution_time = time.time() - start_time
            print(f"\n✓ DR Flow1執行完成！總耗時: {execution_time:.2f}秒")
            
            # 顯示CCD1統計資訊
            if self.ccd1:
                queue_status = self.ccd1.get_queue_status()
                print(f"CCD1狀態: 佇列剩餘={queue_status['queue_length']}, 檢測觸發={self.ccd1_detection_triggered}")
                if self.need_refill:
                    print("⚠️ 需要補料：CCD1未檢測到物體")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.total_steps,
                total_steps=self.total_steps,
                angle_correction_performed=self.angle_correction_performed,
                angle_correction_success=self.angle_correction_success,
                detected_angle=self.detected_angle,
                angle_difference=self.angle_difference,
                motor_position=self.motor_position,
                ccd1_objects_processed=self.ccd1_objects_processed,
                ccd1_detection_triggered=self.ccd1_detection_triggered,
                need_refill=self.need_refill
            )
            
        except Exception as e:
            self.last_error = f"DR Flow1執行異常: {str(e)}"
            print(f"✗ {self.last_error}")
            return FlowResult(
                success=False,
                error_message=self.last_error,
                execution_time=time.time() - start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                angle_correction_performed=self.angle_correction_performed,
                angle_correction_success=self.angle_correction_success,
                detected_angle=self.detected_angle,
                angle_difference=self.angle_difference,
                motor_position=self.motor_position,
                angle_correction_error=self.angle_correction_error,
                ccd1_objects_processed=self.ccd1_objects_processed,
                ccd1_detection_triggered=self.ccd1_detection_triggered,
                need_refill=self.need_refill
            )
        
        finally:
            self.is_running = False
    
    def execute_angle_correction_with_auto_clear(self) -> bool:
        """
        執行角度校正到90度 (新架構版本 - 使用自動清零機制)
        """
        max_retries = 2
        retry_count = 0
        
        print("  正在初始化角度校正系統 (新架構版本 - 含自動清零機制)...")
        
        # 優先使用external_modules中的angle模組
        angle_controller = self.angle
        
        if not angle_controller:
            print("  ✗ 角度校正模組未連接，嘗試動態導入...")
            # 動態導入修正版AngleHighLevel
            try:
                AngleOperationResult = None
                
                # 嘗試導入修正版AngleHighLevel
                try:
                    from AngleHighLevel import AngleHighLevel, AngleOperationResult
                    angle_controller = AngleHighLevel()
                    print("  ✓ 成功導入修正版AngleHighLevel (含自動清零機制)")
                except ImportError as e:
                    print(f"  ✗ 無法導入AngleHighLevel: {e}")
                    print("  嘗試使用備用的直接ModbusTCP方案...")
                    return self._execute_angle_correction_direct_modbus()
                
                # 測試連接
                if not angle_controller.connect():
                    print("  ✗ 角度校正系統連接失敗")
                    self.angle_correction_error = "角度校正系統連接失敗"
                    return False
                
                print("  ✓ 角度校正系統連接成功 (含自動清零機制)")
                
            except Exception as e:
                print(f"  ✗ 角度校正系統初始化失敗: {e}")
                self.angle_correction_error = f"系統初始化失敗: {e}"
                return self._execute_angle_correction_direct_modbus()
        else:
            print("  ✓ 使用外部模組中的角度校正API")
            try:
                from AngleHighLevel import AngleOperationResult
            except ImportError:
                print("  ⚠️ 無法導入AngleOperationResult，使用備用方案")
                return self._execute_angle_correction_direct_modbus()
        
        try:
            while retry_count < max_retries:
                retry_count += 1
                print(f"  嘗試第 {retry_count}/{max_retries} 次角度校正 (新架構版本+自動清零機制)...")
                
                # 修正：簡化狀態檢查，信任自動清零機制
                if angle_controller.is_system_ready():
                    print("  角度校正系統已準備就緒，開始校正...")
                    
                    # 執行角度校正 (含自動清零機制)
                    result = angle_controller.adjust_to_90_degrees()
                    
                    if result.result == AngleOperationResult.SUCCESS:
                        print(f"  ✓ 角度校正成功！(新架構版本+自動清零機制生效)")
                        
                        # 記錄角度校正結果
                        self.angle_correction_success = True
                        if result.original_angle is not None:
                            self.detected_angle = result.original_angle
                            print(f"    檢測角度: {result.original_angle:.2f}度")
                        if result.angle_diff is not None:
                            self.angle_difference = result.angle_diff
                            print(f"    角度差: {result.angle_diff:.2f}度")
                        if result.motor_position is not None:
                            self.motor_position = result.motor_position
                            print(f"    馬達位置: {result.motor_position}")
                        if result.execution_time is not None:
                            print(f"    執行時間: {result.execution_time:.2f}秒")
                        
                        print("  ✓ 角度校正完成，系統狀態已自動清零")
                        return True
                    else:
                        error_msg = result.message
                        if result.error_details:
                            error_msg += f" (詳細: {result.error_details})"
                        
                        print(f"  ✗ 角度校正失敗: {error_msg}")
                        self.angle_correction_error = error_msg
                        
                        if retry_count < max_retries:
                            print("  等待系統穩定後重試...")
                            reset_result = angle_controller.reset_errors()
                            if reset_result == AngleOperationResult.SUCCESS:
                                print("  ✓ 錯誤重置成功 (含自動清零)")
                            time.sleep(2.0)
                            continue
                        else:
                            return False
                else:
                    print("  角度校正系統未準備就緒，執行錯誤重置...")
                    
                    reset_result = angle_controller.reset_errors()
                    if reset_result == AngleOperationResult.SUCCESS:
                        print("  ✓ 錯誤重置成功 (含自動清零)")
                        time.sleep(1.5)
                        continue
                    else:
                        print("  ✗ 錯誤重置失敗")
                        if retry_count >= max_retries:
                            self.angle_correction_error = "錯誤重置失敗"
                            return False
                        time.sleep(1.0)
                        continue
            
            # 如果所有重試都失敗
            self.angle_correction_error = "角度校正所有重試都失敗"
            return False
            
        except Exception as e:
            self.angle_correction_error = f"角度校正過程發生異常: {e}"
            print(f"  ✗ 角度校正過程發生異常: {e}")
            return False
        
        finally:
            # 確保斷開連接 (如果是動態創建的)
            if angle_controller and angle_controller != self.angle:
                try:
                    angle_controller.disconnect()
                    print("  角度校正系統連接已斷開")
                except:
                    pass
    
    def _execute_angle_correction_direct_modbus(self) -> bool:
        """
        備用方案: 直接通過ModbusTCP執行角度校正 (新架構版本 - 含自動清零機制)
        """
        try:
            from pymodbus.client import ModbusTcpClient
            import threading
            
            print("  使用備用ModbusTCP方案執行角度校正 (新架構版本+含自動清零機制)...")
            
            # 連接到主服務器
            modbus_client = ModbusTcpClient(host="127.0.0.1", port=502, timeout=3)
            if not modbus_client.connect():
                print("  ✗ 無法連接到Modbus服務器")
                self.angle_correction_error = "無法連接到Modbus服務器"
                return False
            
            angle_base_address = 700
            max_retries = 2
            retry_count = 0
            
            # 定義自動清零函數
            def auto_clear_command():
                try:
                    time.sleep(0.5)
                    clear_result = modbus_client.write_register(
                        address=angle_base_address + 40, value=0, slave=1
                    )
                    if not clear_result.isError():
                        print("  ✓ 角度校正指令已自動清零")
                    else:
                        print("  ⚠️ 角度校正指令自動清零失敗")
                except Exception as e:
                    print(f"  ⚠️ 自動清零過程異常: {e}")
            
            try:
                while retry_count < max_retries:
                    retry_count += 1
                    print(f"  嘗試第 {retry_count}/{max_retries} 次角度校正 (新架構版本+備用方案+自動清零)...")
                    
                    # 檢查系統狀態
                    status_result = modbus_client.read_holding_registers(
                        address=angle_base_address, count=1, slave=1
                    )
                    
                    if not status_result.isError():
                        status_register = status_result.registers[0]
                        ready = bool(status_register & (1 << 0))
                        running = bool(status_register & (1 << 1))
                        alarm = bool(status_register & (1 << 2))
                        initialized = bool(status_register & (1 << 3))
                        
                        print(f"    系統狀態: Ready={ready}, Running={running}, Alarm={alarm}, Init={initialized}")
                        
                        if ready and not alarm and initialized:
                            print("  系統已準備就緒，發送角度校正指令...")
                            
                            # 發送角度校正指令並啟動自動清零
                            cmd_result = modbus_client.write_register(
                                address=angle_base_address + 40, value=1, slave=1
                            )
                            
                            if cmd_result.isError():
                                print("  ✗ 發送角度校正指令失敗")
                                if retry_count < max_retries:
                                    time.sleep(1.0)
                                    continue
                                else:
                                    self.angle_correction_error = "發送角度校正指令失敗"
                                    return False
                            
                            print("  ✓ 角度校正指令已發送")
                            
                            # 啟動自動清零機制
                            threading.Thread(target=auto_clear_command, daemon=True).start()
                            print("  ✓ 自動清零機制已啟動")
                            
                            # 等待執行完成 (最多15秒)
                            completion_timeout = 15.0
                            start_time = time.time()
                            
                            while time.time() - start_time < completion_timeout:
                                # 檢查狀態
                                status_check = modbus_client.read_holding_registers(
                                    address=angle_base_address, count=1, slave=1
                                )
                                
                                if not status_check.isError():
                                    check_status = status_check.registers[0]
                                    check_ready = bool(check_status & (1 << 0))
                                    check_running = bool(check_status & (1 << 1))
                                    check_alarm = bool(check_status & (1 << 2))
                                    
                                    # 檢查是否有錯誤
                                    if check_alarm:
                                        print("  ✗ 執行過程發生錯誤，系統進入Alarm狀態")
                                        if retry_count < max_retries:
                                            # 嘗試錯誤重置
                                            print("  執行錯誤重置 (含自動清零)...")
                                            modbus_client.write_register(
                                                address=angle_base_address + 40, value=7, slave=1
                                            )
                                            # 啟動錯誤重置的自動清零
                                            def auto_clear_reset():
                                                time.sleep(0.5)
                                                modbus_client.write_register(
                                                    address=angle_base_address + 40, value=0, slave=1
                                                )
                                                print("  ✓ 錯誤重置指令已自動清零")
                                            threading.Thread(target=auto_clear_reset, daemon=True).start()
                                            time.sleep(2.0)
                                            break
                                        else:
                                            self.angle_correction_error = "角度校正執行過程發生錯誤"
                                            return False
                                    
                                    # 檢查是否完成
                                    if check_ready and not check_running:
                                        print("  ✓ 角度校正執行完成 (新架構版本+自動清零機制已生效)")
                                        
                                        # 讀取執行結果
                                        result_data = modbus_client.read_holding_registers(
                                            address=angle_base_address + 20, count=7, slave=1
                                        )
                                        
                                        if not result_data.isError():
                                            registers = result_data.registers
                                            success = bool(registers[0])
                                            
                                            if success:
                                                # 解析結果
                                                angle_int = (registers[1] << 16) | registers[2]
                                                if angle_int >= 2**31:
                                                    angle_int -= 2**32
                                                original_angle = angle_int / 100.0
                                                
                                                diff_int = (registers[3] << 16) | registers[4]
                                                if diff_int >= 2**31:
                                                    diff_int -= 2**32
                                                angle_diff = diff_int / 100.0
                                                
                                                pos_int = (registers[5] << 16) | registers[6]
                                                if pos_int >= 2**31:
                                                    pos_int -= 2**32
                                                motor_position = pos_int
                                                
                                                print(f"  ✓ 角度校正成功！(新架構版本+備用方案+自動清零)")
                                                print(f"    檢測角度: {original_angle:.2f}度")
                                                print(f"    角度差: {angle_diff:.2f}度")
                                                print(f"    馬達位置: {motor_position}")
                                                
                                                # 記錄角度校正結果
                                                self.angle_correction_success = True
                                                self.detected_angle = original_angle
                                                self.angle_difference = angle_diff
                                                self.motor_position = motor_position
                                                
                                                return True
                                            else:
                                                print("  ✗ 角度校正執行失敗，無有效結果")
                                                if retry_count < max_retries:
                                                    time.sleep(2.0)
                                                    break
                                                else:
                                                    self.angle_correction_error = "角度校正執行失敗，無有效結果"
                                                    return False
                                
                                time.sleep(0.5)
                            else:
                                print(f"  ✗ 角度校正執行超時 ({completion_timeout}秒)")
                                if retry_count < max_retries:
                                    try:
                                        modbus_client.write_register(
                                            address=angle_base_address + 40, value=0, slave=1
                                        )
                                        print("  ✓ 超時情況下已清零指令")
                                    except:
                                        pass
                                    time.sleep(1.0)
                                    continue
                                else:
                                    self.angle_correction_error = "角度校正執行超時"
                                    return False
                        else:
                            # 系統未準備就緒，執行錯誤重置
                            print("  系統未準備就緒，嘗試錯誤重置 (含自動清零)...")
                            
                            reset_result = modbus_client.write_register(
                                address=angle_base_address + 40, value=7, slave=1
                            )
                            
                            if not reset_result.isError():
                                print("  ✓ 錯誤重置指令已發送")
                                
                                # 啟動錯誤重置的自動清零
                                def auto_clear_reset():
                                    time.sleep(0.5)
                                    modbus_client.write_register(
                                        address=angle_base_address + 40, value=0, slave=1
                                    )
                                    print("  ✓ 錯誤重置指令已自動清零")
                                
                                threading.Thread(target=auto_clear_reset, daemon=True).start()
                                time.sleep(2.0)
                                continue
                            else:
                                print("  ✗ 錯誤重置指令發送失敗")
                                if retry_count >= max_retries:
                                    self.angle_correction_error = "錯誤重置指令發送失敗"
                                    return False
                    else:
                        print("  ✗ 無法讀取系統狀態")
                        if retry_count >= max_retries:
                            self.angle_correction_error = "無法讀取系統狀態"
                            return False
                
                # 所有重試都失敗
                self.angle_correction_error = "角度校正所有重試都失敗 (新架構版本+備用方案)"
                return False
                
            finally:
                # 確保最終清除指令並斷開連接
                try:
                    modbus_client.write_register(
                        address=angle_base_address + 40, value=0, slave=1
                    )
                    modbus_client.close()
                    print("  ModbusTCP連接已斷開")
                except:
                    pass
            
        except ImportError:
            print("  ✗ 無法導入pymodbus，請確認pymodbus已安裝")
            self.angle_correction_error = "無法導入pymodbus"
            return False
        except Exception as e:
            print(f"  ✗ 新架構版本備用方案執行異常: {e}")
            self.angle_correction_error = f"新架構版本備用方案執行異常: {e}"
            return False
    
    # =================================================================
    # 新的CCD1檢測步驟 (使用新API) - 新架構版本
    # =================================================================
    
    def _step_ccd1_smart_detection(self):
        """步驟4: CCD1智能檢測 (使用新的CCD1HighLevel API) - 新架構版本"""
        if not self.ccd1:
            print("  跳過CCD1檢測 (CCD1未啟用)")
            return None
        
        print("  使用CCD1智能檢測API (新架構版本)...")
        
        # 檢查CCD1系統狀態
        system_status = self.ccd1.get_system_status()
        if not system_status['connected']:
            print("  ⚠️ CCD1系統未連接")
            return None
        
        print(f"  CCD1系統狀態: Ready={system_status.get('ready', False)}, "
              f"檢測需求={system_status.get('detection_needed', True)}")
        
        # 🔥 關鍵：使用新的get_next_object API
        # 自動處理：檢查FIFO佇列 → 如果空則自動拍照檢測 → 返回結果或None
        coord = self.ccd1.get_next_object()
        
        if coord:
            # 檢查是否觸發了自動拍照檢測
            queue_status = self.ccd1.get_queue_status()
            if queue_status['last_detection_count'] > 0:
                self.ccd1_detection_triggered = True
                print(f"  ✓ 自動拍照檢測觸發，新增 {queue_status['last_detection_count']} 個物體到佇列")
            
            # 設定R值 (繼承VP_TOPSIDE點位的R值) - 新架構版本適配
            vp_topside_point = None
            if self.robot and hasattr(self.robot, 'points_manager'):
                vp_topside_point = self.robot.points_manager.get_point("VP_TOPSIDE")
            
            if vp_topside_point and hasattr(vp_topside_point, 'r'):
                coord.r = vp_topside_point.r
                print(f"  繼承VP_TOPSIDE的R值: {coord.r}°")
            else:
                coord.r = 0.0
                print(f"  使用預設R值: {coord.r}°")
            
            print(f"  ✓ 智能檢測成功: 世界座標=({coord.world_x:.2f}, {coord.world_y:.2f})mm, R={coord.r}°")
            print(f"  來源: FIFO佇列ID={coord.id}, 佇列剩餘={queue_status['queue_length']}個物體")
            
            return coord
        else:
            # coord為None表示CCD1檢測不到任何物體，需要補料
            print("  ✗ CCD1智能檢測：未檢測到物體，需要補料")
            self.need_refill = True
            
            # 檢查是否有觸發拍照檢測
            queue_status = self.ccd1.get_queue_status()
            if queue_status['last_detection_count'] == 0:
                self.ccd1_detection_triggered = True
                print("  (已自動執行拍照檢測，但無物體)")
            
            return None
    
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
    
    def _execute_step_with_return(self, step_num: int, step_name: str, step_func):
        """執行單個步驟並返回結果 - 新架構版本"""
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
        """更新進度到狀態機 - 新架構版本"""
        if (self.motion_state_machine and 
            hasattr(self.motion_state_machine, 'modbus_client') and 
            self.motion_state_machine.modbus_client is not None):
            try:
                progress = int((self.current_step / self.total_steps) * 100)
                # 使用新架構地址1202
                from Dobot_main import MotionRegisters
                self.motion_state_machine.modbus_client.write_register(MotionRegisters.MOTION_PROGRESS, progress)
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
            angle_correction_performed=self.angle_correction_performed,
            angle_correction_success=self.angle_correction_success,
            detected_angle=self.detected_angle,
            angle_difference=self.angle_difference,
            motor_position=self.motor_position,
            angle_correction_error=self.angle_correction_error,
            ccd1_objects_processed=self.ccd1_objects_processed,
            ccd1_detection_triggered=self.ccd1_detection_triggered,
            need_refill=self.need_refill
        )
    
    # =================================================================
    # 原有步驟實現 (新架構版本適配)
    # =================================================================
    
    def _step_system_check(self) -> bool:
        """步驟1: 系統檢查 - 新架構版本"""
        if not self.robot or not hasattr(self.robot, 'is_connected'):
            self.last_error = "機械臂控制器未初始化"
            return False
        
        if not self.robot.is_connected:
            self.last_error = "機械臂未連接"
            return False
        
        # 檢查必要點位 (簡化版，因為新架構可能沒有points_manager)
        if hasattr(self.robot, 'points_manager'):
            for point_name in self.REQUIRED_POINTS:
                if not self.robot.points_manager.get_point(point_name):
                    self.last_error = f"缺少必要點位: {point_name}"
                    return False
        else:
            print("  ⚠️ 無法檢查點位 (points_manager不可用)")
        
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
                print("  CCD1視覺系統準備就緒 (新API)")
        
        return True
    
    def _step_gripper_quick_close_sync(self) -> bool:
        """步驟2: 夾爪快速關閉 - 新架構版本"""
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
        """步驟3: 移動到待機點 - 新架構版本"""
        if not self.robot:
            self.last_error = "機械臂控制器不可用"
            return False
        
        self.robot.set_global_speed(self.SPEED_RATIO)
        
        # 新架構版本：使用座標點或簡化移動
        if hasattr(self.robot, 'MovJ') and hasattr(self.robot, 'points_manager'):
            if not self.robot.MovJ("standby"):
                self.last_error = "移動到待機點失敗"
                return False
        else:
            # 簡化版本：移動到固定座標
            if not self.robot.move_j(300, 0, 200, 0):
                self.last_error = "移動到待機點失敗"
                return False
        
        if hasattr(self.robot, 'sync'):
            self.robot.sync()
        print("  移動到待機點完成")
        return True
    
    def _step_move_to_vp_topside_no_sync(self) -> bool:
        """步驟5: 移動到VP_TOPSIDE - 新架構版本"""
        if not self.robot:
            self.last_error = "機械臂控制器不可用"
            return False
        
        # 新架構版本：使用座標點或簡化移動
        if hasattr(self.robot, 'MovJ') and hasattr(self.robot, 'points_manager'):
            if not self.robot.MovJ("VP_TOPSIDE"):
                self.last_error = "移動到VP_TOPSIDE失敗"
                return False
        else:
            # 簡化版本：移動到固定座標
            if not self.robot.move_j(250, 100, 200, 0):
                self.last_error = "移動到VP_TOPSIDE失敗"
                return False
        
        print("  移動到VP_TOPSIDE指令已發送")
        return True
    
    def _step_move_to_object_above_no_sync(self, coord) -> bool:
        """步驟6: 移動到物體上方 - 新架構版本"""
        if not coord:
            self.last_error = "沒有有效的物體座標"
            return False
        
        if not self.robot:
            self.last_error = "機械臂控制器不可用"
            return False
        
        r_value = getattr(coord, 'r', 0.0)
        
        # 新架構版本：使用座標移動
        if hasattr(self.robot, 'MovL_coord'):
            if not self.robot.MovL_coord(coord.world_x, coord.world_y, self.CCD1_DETECT_HEIGHT, r_value):
                self.last_error = "移動到物體上方失敗"
                return False
        else:
            # 簡化版本：使用基本移動
            if not self.robot.move_l(coord.world_x, coord.world_y, self.CCD1_DETECT_HEIGHT, r_value):
                self.last_error = "移動到物體上方失敗"
                return False
        
        print(f"    移動到物體上方指令已發送 (R={r_value}°)")
        return True
    
    def _step_descend_and_grip_sync(self, coord) -> bool:
        """步驟7: 下降並智能夾取 - 新架構版本"""
        if not coord:
            self.last_error = "沒有有效的物體座標"
            return False
        
        if not self.robot:
            self.last_error = "機械臂控制器不可用"
            return False
        
        r_value = getattr(coord, 'r', 0.0)
        
        # 新架構版本：使用座標移動
        if hasattr(self.robot, 'MovL_coord'):
            if not self.robot.MovL_coord(coord.world_x, coord.world_y, self.PICKUP_HEIGHT, r_value):
                self.last_error = "下降到抓取高度失敗"
                return False
        else:
            # 簡化版本：使用基本移動
            if not self.robot.move_l(coord.world_x, coord.world_y, self.PICKUP_HEIGHT, r_value):
                self.last_error = "下降到抓取高度失敗"
                return False
        
        if hasattr(self.robot, 'sync'):
            self.robot.sync()
        print(f"    下降到抓取高度完成: {self.PICKUP_HEIGHT}mm (R={r_value}°)")
        
        if self.gripper:
            if not self.gripper.smart_grip(target_position=300):
                self.last_error = "智能夾取失敗"
                return False
            print("    智能夾取完成")
        
        return True
    
    def _step_ascend_and_move_to_vp_no_sync(self, coord) -> bool:
        """步驟8: 上升並移動 - 新架構版本"""
        if not coord:
            self.last_error = "沒有有效的物體座標"
            return False
        
        if not self.robot:
            self.last_error = "機械臂控制器不可用"
            return False
        
        r_value = getattr(coord, 'r', 0.0)
        
        # 新架構版本：使用座標移動
        if hasattr(self.robot, 'MovL_coord'):
            if not self.robot.MovL_coord(coord.world_x, coord.world_y, self.CCD1_DETECT_HEIGHT, r_value):
                self.last_error = "上升到安全高度失敗"
                return False
        else:
            # 簡化版本：使用基本移動
            if not self.robot.move_l(coord.world_x, coord.world_y, self.CCD1_DETECT_HEIGHT, r_value):
                self.last_error = "上升到安全高度失敗"
                return False
        
        if hasattr(self.robot, 'MovJ') and hasattr(self.robot, 'points_manager'):
            if not self.robot.MovJ("VP_TOPSIDE"):
                self.last_error = "移動到VP_TOPSIDE失敗"
                return False
        else:
            # 簡化版本：移動到固定座標
            if not self.robot.move_j(250, 100, 200, 0):
                self.last_error = "移動到VP_TOPSIDE失敗"
                return False
        
        print(f"    上升並移動指令已發送 (R={r_value}°)")
        return True
    
    def _step_move_to_standby_no_sync(self) -> bool:
        """步驟9&16: 移動到待機點 - 新架構版本"""
        if not self.robot:
            self.last_error = "機械臂控制器不可用"
            return False
        
        self.robot.set_global_speed(self.SPEED_RATIO)
        
        # 新架構版本：使用座標點或簡化移動
        if hasattr(self.robot, 'MovJ') and hasattr(self.robot, 'points_manager'):
            if not self.robot.MovJ("standby"):
                self.last_error = "移動到待機點失敗"
                return False
        else:
            # 簡化版本：移動到固定座標
            if not self.robot.move_j(300, 0, 200, 0):
                self.last_error = "移動到待機點失敗"
                return False
        
        print("  移動到待機點指令已發送")
        return True
    
    def _step_move_to_point_no_sync(self, point_name: str) -> bool:
        """通用點位移動 - 新架構版本"""
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
                "Rotate_down": (200, 200, 120, 0)
            }
            
            if point_name in point_coords:
                x, y, z, r = point_coords[point_name]
                if not self.robot.move_j(x, y, z, r):
                    self.last_error = f"移動到{point_name}失敗"
                    return False
            else:
                print(f"  ⚠️ 未知點位{point_name}，跳過")
                return True
        
        print(f"  移動到{point_name}指令已發送")
        return True
    
    def _step_smart_close_sync(self) -> bool:
        """步驟13: 智能關閉 - 新架構版本"""
        if hasattr(self.robot, 'sync'):
            self.robot.sync()
        
        if not self.gripper:
            print("  跳過智能關閉 (夾爪未啟用)")
            return True
        
        if not self.gripper.smart_release(release_position=50):
            self.last_error = "智能關閉失敗"
            return False
        
        print("  智能關閉完成")
        return True
    
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
            "ccd1_enabled": self.ccd1 is not None,
            "angle_correction_enabled": True,
            "auto_clear_enabled": True,
            "ccd1_new_api_enabled": True,
            "ccd1_objects_processed": self.ccd1_objects_processed,
            "ccd1_detection_triggered": self.ccd1_detection_triggered,
            "need_refill": self.need_refill,
            "new_architecture_version": True  # 新增：標識新架構版本
        }
    
    def stop(self) -> bool:
        """停止流程執行 - 新架構版本"""
        try:
            self.is_running = False
            
            if self.robot:
                self.robot.emergency_stop()
            
            if self.gripper:
                self.gripper.stop()
            
            self.last_error = "DR Flow1流程已停止"
            return True
            
        except Exception as e:
            print(f"停止DR Flow1流程失敗: {e}")
            return False


# 兼容性別名
class Flow1Executor(DrFlow1VisionPickExecutor):
    """Flow1執行器 - 兼容性包裝器"""
    pass


# ============================= 新架構修改說明 ===============================
# 
# 主要修改項目：
# 1. 類別名稱：DobotFlow1Enhanced → DrFlow1VisionPickExecutor
# 2. 初始化方式：移除狀態機等參數，改為initialize()方法設置
# 3. 機械臂API適配：支援新架構的RealRobotController
# 4. 進度更新：使用新架構地址1202 (MotionRegisters.MOTION_PROGRESS)
# 5. 錯誤處理：適配新架構的錯誤管理機制
# 6. 模組引用：通過external_modules字典獲取外部模組
# 7. 座標移動：兼容新舊API (MovJ/MovL_coord 和 move_j/move_l)
# 8. 狀態檢查：適配新架構的is_connected屬性
# 9. 點位管理：兼容points_manager或使用預設座標
# 10. 同步機制：適配新架構的sync()方法
# 
# 新架構特點：
# - 支援基地址1200-1249的寄存器映射
# - 適配混合交握協議的狀態機管理
# - 兼容新的RealRobotController API
# - 保持原有的角度校正和CCD1檢測功能
# - 增強錯誤處理和狀態回報
# 
# 使用方式：
# 1. 在新架構主程序中創建實例：flow1 = DrFlow1VisionPickExecutor()
# 2. 初始化：flow1.initialize(robot, motion_state_machine, external_modules)
# 3. 執行：result = flow1.execute()
# 4. 結果包含完整的執行資訊和角度校正狀態
# 
# 相容性：
# - 保持原有Flow1的所有功能
# - 支援CCD1HighLevel API的FIFO管理
# - 支援自動清零角度校正機制
# - 適配新架構的寄存器地址和狀態管理