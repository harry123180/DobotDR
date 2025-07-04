#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AutoProgram_main.py - DR專案版本
VP入料+機械臂協調控制系統 (DR_F/STACK二分類)
雙執行緒架構：AutoFeeding執行緒 + RobotJob執行緒
基地址：1300-1349
"""

import time
import math
import os
import json
import threading
from typing import Dict, Any, Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

# Modbus TCP Client (pymodbus 3.9.2)
try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError:
    print("pymodbus未安裝，請安裝: pip install pymodbus==3.9.2")
    MODBUS_AVAILABLE = False


class SystemStatus(Enum):
    """系統狀態"""
    STOPPED = 0
    AUTO_FEEDING_RUNNING = 1
    ROBOT_JOB_RUNNING = 2
    FLOW1_EXECUTING = 3
    FLOW2_EXECUTING = 4
    ERROR = 5


@dataclass
class CCD1DetectionResult:
    """CCD1檢測結果 - DR_F/STACK二分類"""
    dr_f_count: int = 0
    stack_count: int = 0
    total_detections: int = 0
    dr_f_world_coords: List[Tuple[float, float]] = None
    capture_success: bool = False
    detect_success: bool = False
    operation_success: bool = False
    
    def __post_init__(self):
        if self.dr_f_world_coords is None:
            self.dr_f_world_coords = []


class ProtectionZone:
    """保護區域判斷"""
    
    @staticmethod
    def is_point_in_quad(x_a: float, y_a: float) -> bool:
        """判斷點是否在保護區域四邊形內"""
        points = [
            (-112, 243),   # x1, y1
            (-112, 341.21),   # x2, y2
            (-4, 243),  # x3, y3
            (-4, 341)  # x4, y4
        ]
        
        # 找中心點，並以中心為基準對四點極角排序
        cx = sum(p[0] for p in points) / 4
        cy = sum(p[1] for p in points) / 4
        
        def angle(p):
            return math.atan2(p[1] - cy, p[0] - cx)
        
        sorted_points = sorted(points, key=angle)
        
        # 使用射線法檢查點是否在排序後四邊形內
        def point_in_polygon(x, y, polygon):
            n = len(polygon)
            inside = False
            px, py = polygon[0]
            for i in range(1, n + 1):
                qx, qy = polygon[i % n]
                if ((py > y) != (qy > y)):
                    cross = (qx - px) * (y - py) / (qy - py + 1e-9) + px
                    if x < cross:
                        inside = not inside
                px, py = qx, qy
            return inside
        
        return point_in_polygon(x_a, y_a, sorted_points)


class AutoFeedingThread:
    """自動入料執行緒 - DR專案版本"""
    
    def __init__(self, modbus_client: ModbusTcpClient, config: Dict):
        self.modbus_client = modbus_client
        self.config = config
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.protection_zone = ProtectionZone()
        
        # 模組基地址
        self.CCD1_BASE = 200
        self.VP_BASE = 300
        self.FLOW4_BASE = 448
        
        # 統計資訊
        self.cycle_count = 0
        self.dr_f_found_count = 0
        self.flow4_trigger_count = 0
        self.vp_vibration_count = 0
        
        # 入料完成標誌
        self.feeding_ready = False
        self.feeding_ready_callback = None
        
        # 暫停標誌 (用於Flow1執行時暫停)
        self.pause_for_robot = False
        
    def set_feeding_ready_callback(self, callback):
        """設置入料完成回調函數"""
        self.feeding_ready_callback = callback
    
    def pause_for_robot_operation(self):
        """為機械臂作業暫停入料檢測"""
        print("[AutoFeeding] 為機械臂作業暫停入料檢測")
        self.pause_for_robot = True
        self.feeding_ready = False
    
    def resume_after_robot_operation(self):
        """機械臂作業完成後恢復入料檢測"""
        print("[AutoFeeding] 機械臂作業完成，恢復入料檢測")
        self.pause_for_robot = False
        self.feeding_ready = False
    
    
    
    def read_register(self, address: int) -> Optional[int]:
        """讀取單個寄存器"""
        try:
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            if not result.isError():
                return result.registers[0]
            return None
        except Exception:
            return None
    
    def write_register(self, address: int, value: int) -> bool:
        """寫入單個寄存器"""
        try:
            result = self.modbus_client.write_register(address, value, slave=1)
            return not result.isError()
        except Exception:
            return False
    
    def read_32bit_register(self, high_addr: int, low_addr: int) -> float:
        """讀取32位世界座標並轉換為實際值"""
        high_val = self.read_register(high_addr)
        low_val = self.read_register(low_addr)
        
        if high_val is None or low_val is None:
            return 0.0
        
        # 合併32位值
        combined = (high_val << 16) + low_val
        
        # 處理補碼(負數)
        if combined >= 2147483648:  # 2^31
            combined = combined - 4294967296  # 2^32
        
        # 轉換為毫米(除以100)
        return combined / 100.0
    
    def clear_ccd1_registers(self) -> bool:
        """清空CCD1控制寄存器"""
        success = True
        success &= self.write_register(200, 0)  # CONTROL_COMMAND
        success &= self.write_register(203, 0)  # CAPTURE_COMPLETE
        success &= self.write_register(204, 0)  # DETECT_COMPLETE
        success &= self.write_register(205, 0)  # OPERATION_SUCCESS
        return success
    
    def check_modules_status(self) -> bool:
        """檢查VP、CCD1模組狀態"""
        print("[AutoFeeding] 檢查模組狀態...")
        
        # 檢查CCD1狀態
        ccd1_status = self.read_register(201)  # STATUS_REGISTER
        if ccd1_status is None:
            print("[AutoFeeding] ✗ CCD1模組無回應")
            return False
        
        ccd1_ready = bool(ccd1_status & 0x01)  # bit0=Ready
        ccd1_initialized = bool(ccd1_status & 0x08)  # bit3=Initialized
        
        print(f"[AutoFeeding] CCD1狀態: {ccd1_status:04b}, Ready={ccd1_ready}, Initialized={ccd1_initialized}")
        
        # 如果CCD1未準備就緒但已初始化，嘗試重置
        if not ccd1_ready and ccd1_initialized:
            print("[AutoFeeding] CCD1未Ready但已初始化，嘗試重置...")
            
            # 清零CCD1控制寄存器
            reset_success = True
            reset_success &= self.write_register(200, 0)  # CONTROL_COMMAND
            reset_success &= self.write_register(203, 0)  # CAPTURE_COMPLETE
            reset_success &= self.write_register(204, 0)  # DETECT_COMPLETE
            reset_success &= self.write_register(205, 0)  # OPERATION_SUCCESS
            
            if reset_success:
                print("[AutoFeeding] CCD1寄存器已清零，等待狀態更新...")
                time.sleep(0.5)  # 等待狀態更新
                
                # 重新檢查CCD1狀態
                ccd1_status = self.read_register(201)
                if ccd1_status is not None:
                    ccd1_ready = bool(ccd1_status & 0x01)
                    ccd1_initialized = bool(ccd1_status & 0x08)
                    print(f"[AutoFeeding] CCD1重置後狀態: {ccd1_status:04b}, Ready={ccd1_ready}, Initialized={ccd1_initialized}")
                else:
                    print("[AutoFeeding] ✗ CCD1重置後無回應")
                    return False
            else:
                print("[AutoFeeding] ✗ CCD1寄存器重置失敗")
        
        if not (ccd1_ready and ccd1_initialized):
            print(f"[AutoFeeding] ✗ CCD1模組未準備就緒 (Ready={ccd1_ready}, Initialized={ccd1_initialized})")
            return False
        
        # 檢查VP狀態
        vp_status = self.read_register(300)  # module_status
        vp_connected = self.read_register(301)  # device_connection
        
        print(f"[AutoFeeding] VP狀態: module_status={vp_status}, device_connection={vp_connected}")
        
        if vp_status != 1:
            print(f"[AutoFeeding] ✗ VP模組狀態異常: {vp_status}")
            return False
        
        if vp_connected != 1:
            print(f"[AutoFeeding] ✗ VP設備未連接: {vp_connected}")
            return False
        
        print("[AutoFeeding] ✓ 所有模組狀態正常")
        return True
    
    def trigger_ccd1_detection(self) -> CCD1DetectionResult:
        """觸發CCD1拍照檢測並獲取結果 - DR專案版本"""
        print("[AutoFeeding] 觸發CCD1拍照檢測 (DR_F/STACK二分類)...")
        
        result = CCD1DetectionResult()
        
        # 向200地址寫入16觸發拍照+檢測
        if not self.write_register(200, 16):
            print("[AutoFeeding] ✗ CCD1指令寫入失敗")
            return result
        
        print("[AutoFeeding] CCD1指令已發送，等待檢測完成...")
        
        # 等待拍照、檢測、操作完成標誌
        timeout = self.config['auto_program']['ccd1_timeout']
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            capture_complete = self.read_register(203)  # CAPTURE_COMPLETE
            detect_complete = self.read_register(204)   # DETECT_COMPLETE  
            operation_success = self.read_register(205) # OPERATION_SUCCESS
            
            # 每秒輸出一次等待狀態
            elapsed = time.time() - start_time
            if int(elapsed) % 2 == 0:
                print(f"[AutoFeeding] 等待CCD1檢測... {elapsed:.1f}s (capture={capture_complete}, detect={detect_complete}, success={operation_success})")
            
            if capture_complete == 1 and detect_complete == 1 and operation_success == 1:
                result.capture_success = True
                result.detect_success = True
                result.operation_success = True
                print("[AutoFeeding] ✓ CCD1檢測完成")
                break
            
            time.sleep(self.config['timing']['status_check_interval'])
        
        if not result.operation_success:
            elapsed = time.time() - start_time
            print(f"[AutoFeeding] ✗ CCD1檢測超時或失敗 (耗時{elapsed:.1f}s)")
            return result
        
        # 讀取檢測結果 - DR專案版本 (DR_F/STACK二分類)
        result.dr_f_count = self.read_register(240) or 0       # DR_F_COUNT
        result.stack_count = self.read_register(242) or 0      # STACK_COUNT
        result.total_detections = self.read_register(243) or 0 # TOTAL_DETECTIONS
        
        print(f"[AutoFeeding] CCD1檢測結果: DR_F={result.dr_f_count}, STACK={result.stack_count}, 總檢測={result.total_detections}")
        
        # 提取DR_F世界座標
        if result.dr_f_count > 0:
            max_check = min(result.dr_f_count, self.config['auto_program']['max_dr_f_check'])
            
            for i in range(max_check):
                base_addr = 261 + (i * 4)  # 每個物件佔4個寄存器
                world_x = self.read_32bit_register(base_addr, base_addr + 1)
                world_y = self.read_32bit_register(base_addr + 2, base_addr + 3)
                result.dr_f_world_coords.append((world_x, world_y))
                print(f"[AutoFeeding] DR_F{i+1}座標: ({world_x:.2f}, {world_y:.2f})")
        
        return result
    
    def find_dr_f_in_protection_zone(self, detection_result: CCD1DetectionResult) -> Optional[Tuple[float, float]]:
        """尋找保護區域內的DR_F物件"""
        if detection_result.dr_f_count == 0:
            return None
        
        for i, (world_x, world_y) in enumerate(detection_result.dr_f_world_coords):
            if self.protection_zone.is_point_in_quad(world_x, world_y):
                return (world_x, world_y)
        
        return None
    
    def update_first_dr_f_coordinates(self, target_coords: Tuple[float, float]) -> bool:
        """將目標座標覆蓋到第一個DR_F位置(261-264)"""
        world_x, world_y = target_coords
        
        # 轉換為整數形式(×100)
        world_x_int = int(world_x * 100)
        world_y_int = int(world_y * 100)
        
        # 處理負數(補碼)
        if world_x_int < 0:
            world_x_int = world_x_int + 4294967296  # 2^32
        if world_y_int < 0:
            world_y_int = world_y_int + 4294967296  # 2^32
        
        # 分解為高低位
        x_high = (world_x_int >> 16) & 0xFFFF
        x_low = world_x_int & 0xFFFF
        y_high = (world_y_int >> 16) & 0xFFFF
        y_low = world_y_int & 0xFFFF
        
        # 寫入寄存器261-264
        success = True
        success &= self.write_register(261, x_high)  # DR_F_1_WORLD_X_HIGH
        success &= self.write_register(262, x_low)   # DR_F_1_WORLD_X_LOW
        success &= self.write_register(263, y_high)  # DR_F_1_WORLD_Y_HIGH
        success &= self.write_register(264, y_low)   # DR_F_1_WORLD_Y_LOW
        
        return success
    
    def stop_vp_vibration(self) -> bool:
        """停止VP震動盤"""
        success = True
        success &= self.write_register(320, 3)  # stop_all指令
        success &= self.write_register(321, 0)
        success &= self.write_register(322, 0)
        success &= self.write_register(323, 0)
        success &= self.write_register(324, 99)  # emergency stop id
        
        if success:
            time.sleep(self.config['auto_program']['vp_stop_delay'])
        
        return success
    
    def trigger_vp_vibration_and_redetect(self) -> Optional[Tuple[float, float]]:
        """觸發VP震動並重新檢測DR_F"""
        # VP指令參數
        command_code = 5  # execute_action
        action_code = self.config['vp_params']['spread_action_code']  # spread=11
        strength = self.config['vp_params']['spread_strength']        # 50
        frequency = self.config['vp_params']['spread_frequency']      # 43
        duration = self.config['vp_params']['spread_duration']        # 0.5秒
        
        # 啟動VP震動
        success = True
        success &= self.write_register(320, command_code)
        success &= self.write_register(321, action_code)
        success &= self.write_register(322, strength)
        success &= self.write_register(323, frequency)
        success &= self.write_register(324, int(time.time()) % 65535)  # command_id
        
        if not success:
            return None
        
        # 等待震動
        time.sleep(duration)
        
        # 停止震動
        if not self.stop_vp_vibration():
            return None
        
        # 等待穩定
        time.sleep(0.3)
        
        # 重新檢測
        detection_result = self.trigger_ccd1_detection()
        
        if not detection_result.operation_success:
            return None
        
        target_coords = self.find_dr_f_in_protection_zone(detection_result)
        
        if target_coords:
            self.dr_f_found_count += 1
            return target_coords
        
        return None
    
    def trigger_flow4_feeding(self) -> bool:
        """觸發Flow4送料"""
        if not self.write_register(448, 1):
            return False
        
        time.sleep(self.config['auto_program']['flow4_pulse_duration'])
        
        if not self.write_register(448, 0):
            return False
        
        return True
    
    def auto_feeding_cycle(self) -> bool:
        """執行一次自動入料週期 - DR專案版本"""
        try:
            self.cycle_count += 1
            print(f"\n[AutoFeeding] === 週期 {self.cycle_count} 開始 ===")
            
            # 檢查模組狀態
            if not self.check_modules_status():
                print(f"[AutoFeeding] 週期 {self.cycle_count} 跳過: 模組狀態檢查失敗")
                return False
            
            # 觸發CCD1檢測
            detection_result = self.trigger_ccd1_detection()
            
            if not detection_result.operation_success:
                print(f"[AutoFeeding] 週期 {self.cycle_count} 跳過: CCD1檢測失敗")
                return False
            
            # 尋找保護區域內的DR_F
            target_coords = self.find_dr_f_in_protection_zone(detection_result)
            
            if target_coords:
                # 找到保護區域內的DR_F
                self.dr_f_found_count += 1
                if self.update_first_dr_f_coordinates(target_coords):
                    print(f"[AutoFeeding] ✓ 找到DR_F在保護區域: {target_coords}")
                    self.feeding_ready = True
                    # 通知RobotJob
                    if self.feeding_ready_callback:
                        self.feeding_ready_callback()
                else:
                    print("[AutoFeeding] ✗ DR_F座標更新失敗")
            else:
                # 沒有DR_F在保護區域內
                if detection_result.total_detections < 4:
                    # 料件不足，觸發Flow4送料
                    if self.trigger_flow4_feeding():
                        self.flow4_trigger_count += 1
                        print(f"[AutoFeeding] 料件不足，觸發Flow4送料 (總檢測={detection_result.total_detections})")
                
                elif detection_result.total_detections >= 4:
                    # 料件充足但沒有DR_F在保護區，震動散開並重新檢測
                    print(f"[AutoFeeding] 料件充足但無DR_F在保護區 (總檢測={detection_result.total_detections})")
                    target_coords_after_vp = self.trigger_vp_vibration_and_redetect()
                    if target_coords_after_vp:
                        # 震動後找到DR_F，更新座標
                        if self.update_first_dr_f_coordinates(target_coords_after_vp):
                            print(f"[AutoFeeding] ✓ 震動後找到DR_F: {target_coords_after_vp}")
                            self.feeding_ready = True
                            # 通知RobotJob
                            if self.feeding_ready_callback:
                                self.feeding_ready_callback()
                        self.vp_vibration_count += 1
                    else:
                        print("[AutoFeeding] 震動後仍未找到保護區域內的DR_F")
                        self.vp_vibration_count += 1
            
            # 清空CCD1寄存器
            self.clear_ccd1_registers()
            time.sleep(self.config['timing']['register_clear_delay'])
            
            print(f"[AutoFeeding] 週期 {self.cycle_count} 完成")
            return True
            
        except Exception as e:
            print(f"[AutoFeeding] 週期 {self.cycle_count} 異常: {e}")
            return False
    
    def start(self):
        """啟動自動入料執行緒"""
        if self.running:
            return
        
        print("[AutoFeeding] 啟動自動入料執行緒")
        self.running = True
        self.cycle_count = 0
        self.dr_f_found_count = 0
        self.flow4_trigger_count = 0
        self.vp_vibration_count = 0
        self.feeding_ready = False
        self.pause_for_robot = False
        
        self.thread = threading.Thread(target=self._auto_feeding_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """停止自動入料執行緒"""
        if not self.running:
            return
        
        print("[AutoFeeding] 停止自動入料執行緒")
        self.running = False
        self.feeding_ready = False
        
        # 緊急停止VP
        try:
            self.stop_vp_vibration()
        except:
            pass
        
        # 只有在外部執行緒調用時才join，避免自己等待自己
        if self.thread and self.thread.is_alive() and threading.current_thread() != self.thread:
            self.thread.join(timeout=2.0)
        elif threading.current_thread() == self.thread:
            print("[AutoFeeding] 執行緒內部停止，跳過join操作")
    
    def _auto_feeding_loop(self):
        """自動入料主循環"""
        cycle_interval = self.config['auto_program']['cycle_interval']
        
        while self.running:
            try:
                # 檢查是否需要暫停
                if self.pause_for_robot:
                    print("[AutoFeeding] 已暫停，等待機械臂作業完成...")
                    time.sleep(0.5)
                    continue
                
                # 只有在feeding_ready=False時才執行檢測
                if not self.feeding_ready:
                    self.auto_feeding_cycle()
                
                time.sleep(cycle_interval)
                
            except Exception as e:
                print(f"[AutoFeeding] 循環異常: {e}")
                time.sleep(1.0)


class RobotJobThread:
    """機械臂作業執行緒 - DR專案版本 (Flow1+Flow2)"""
    
    def __init__(self, modbus_client: ModbusTcpClient, config: Dict):
        self.modbus_client = modbus_client
        self.config = config
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # 狀態變數
        self.prepare_done = False
        
        # AutoFeeding執行緒參考 (用於重啟)
        self.auto_feeding_ref = None
        
        # 模組基地址 - 基於new_architecture
        self.MOTION_STATUS = 1200     # 運動狀態寄存器
        self.FLOW1_CONTROL = 1240     # Flow1控制
        self.FLOW2_CONTROL = 1241     # Flow2控制 (DR專案使用Flow2出料)
        self.FLOW1_COMPLETE = 1204    # Flow1完成狀態
        self.FLOW2_COMPLETE = 1205    # Flow2完成狀態 (取代CASE專案的Flow5)
        
        # 統計資訊
        self.flow1_trigger_count = 0
        self.flow2_complete_count = 0
        
    def read_register(self, address: int) -> Optional[int]:
        """讀取單個寄存器"""
        try:
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            if not result.isError():
                return result.registers[0]
            return None
        except Exception:
            return None
    
    def write_register(self, address: int, value: int) -> bool:
        """寫入單個寄存器"""
        try:
            result = self.modbus_client.write_register(address, value, slave=1)
            return not result.isError()
        except Exception:
            return False
    
    def on_feeding_ready(self):
        """自動入料完成回調"""
        print("[RobotJob] 收到自動入料完成通知")
        
        # 只要prepare_done=False，就觸發Flow1
        if not self.prepare_done:
            print("[RobotJob] prepare_done=False，觸發Flow1去VP拿料")
            
            # 使用暫停標誌而不是停止執行緒
            if hasattr(self, 'auto_feeding_ref') and self.auto_feeding_ref:
                print("[RobotJob] 暫停AutoFeeding檢測避免干擾Flow1")
                self.auto_feeding_ref.pause_for_robot_operation()
            
            if self.write_register(self.FLOW1_CONTROL, 1):
                self.flow1_trigger_count += 1
                print("[RobotJob] Flow1觸發成功")
            else:
                print("[RobotJob] Flow1觸發失敗")
        else:
            # prepare_done=True，表示機台已準備好，等待出料指令
            print(f"[RobotJob] prepare_done=True，機台已準備好接受出料指令")
    
    def robot_job_cycle(self):
        """機械臂作業週期 - DR專案版本"""
        try:
            # 檢查機械臂是否Ready (1200=9)
            robot_status = self.read_register(self.MOTION_STATUS)  # 運動狀態寄存器
            robot_ready = (robot_status == 9) if robot_status is not None else False
            
            # 檢查Flow1完成狀態 (但不清空，供其他模組使用)
            flow1_status = self.read_register(self.FLOW1_COMPLETE)
            if flow1_status == 1:
                print("[RobotJob] 檢測到Flow1完成")
                
                # Flow1完成，設置prepare_done=True
                self.prepare_done = True
                
                # 恢復AutoFeeding檢測
                if hasattr(self, 'auto_feeding_ref') and self.auto_feeding_ref:
                    print("[RobotJob] Flow1完成，恢復AutoFeeding檢測")
                    self.auto_feeding_ref.resume_after_robot_operation()
                
                # 只清空Flow1控制狀態，保持Flow1完成狀態供其他模組使用
                self.write_register(self.FLOW1_CONTROL, 0)
                print("[RobotJob] Flow1控制狀態已清零，prepare_done=True，等待其他模組執行Flow2")
            
            # 檢查Flow2完成狀態 (DR專案使用Flow2出料，取代CASE專案的Flow5)
            flow2_status = self.read_register(self.FLOW2_COMPLETE)
            if flow2_status == 1:
                print("[RobotJob] 檢測到Flow2完成")
                
                # Flow2完成，重置prepare_done=False，準備下一輪
                self.prepare_done = False
                self.flow2_complete_count += 1
                
                # 重置AutoFeeding狀態開始新的週期
                if hasattr(self, 'auto_feeding_ref') and self.auto_feeding_ref:
                    print("[RobotJob] Flow2完成，重置AutoFeeding狀態開始新週期")
                    # 重置所有狀態
                    self.auto_feeding_ref.feeding_ready = False
                    self.auto_feeding_ref.pause_for_robot = False
                
                # 清除Flow2完成狀態
                self.write_register(self.FLOW2_COMPLETE, 0)
                print("[RobotJob] Flow2完成狀態已清零，prepare_done=False，系統準備新週期")
            
            # 核心邏輯：當prepare_done=False且機械臂Ready時，執行AutoFeeding+Flow1
            if not self.prepare_done and robot_ready:
                print(f"[RobotJob] 系統條件滿足：prepare_done=False, 機械臂Ready={robot_ready}")
                print("[RobotJob] 開始AutoFeeding+Flow1週期")
                
                # 確保AutoFeeding正在運行且未暫停
                if hasattr(self, 'auto_feeding_ref') and self.auto_feeding_ref:
                    if not self.auto_feeding_ref.running:
                        print("[RobotJob] 啟動AutoFeeding執行緒")
                        self.auto_feeding_ref.start()
                    
                    if self.auto_feeding_ref.pause_for_robot:
                        print("[RobotJob] 恢復AutoFeeding檢測")
                        self.auto_feeding_ref.resume_after_robot_operation()
        
        except Exception as e:
            print(f"[RobotJob] 機械臂作業週期異常: {e}")
    
    def start(self):
        """啟動機械臂作業執行緒"""
        if self.running:
            return
        
        print("[RobotJob] 啟動機械臂作業執行緒")
        self.running = True
        self.prepare_done = False
        self.flow1_trigger_count = 0
        self.flow2_complete_count = 0
        
        # 初始化Flow1完成狀態為0
        self.write_register(self.FLOW1_COMPLETE, 0)
        
        self.thread = threading.Thread(target=self._robot_job_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """停止機械臂作業執行緒"""
        if not self.running:
            return
        
        print("[RobotJob] 停止機械臂作業執行緒")
        self.running = False
        
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
    
    def _robot_job_loop(self):
        """機械臂作業主循環"""
        while self.running:
            try:
                self.robot_job_cycle()
                time.sleep(0.5)  # 0.5秒檢查間隔
                
            except Exception as e:
                print(f"[RobotJob] 循環異常: {e}")
                time.sleep(1.0)


class AutoProgramRobotJobController:
    """VP入料+機械臂協調控制系統 - DR專案版本"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected = False
        
        # 基地址 1300-1349
        self.base_address = 1300
        
        # 控制地址 - AutoProgram流程控制
        self.AUTO_PROGRAM_CONTROL = 1350  # AutoProgram流程控制 (0=停止, 1=啟動)
        
        # 載入配置
        self.config = self.load_config()
        
        # 建立執行緒
        self.auto_feeding_thread = None
        self.robot_job_thread = None
        
        # 系統狀態
        self.system_status = SystemStatus.STOPPED
        
        # 流程控制狀態
        self.last_auto_program_control = 0
        self.auto_program_enabled = False
        
        # 控制監控執行緒
        self.control_monitor_thread = None
        self.control_monitor_running = False
        
        print("VP入料+機械臂協調控制系統初始化完成 (DR專案版本)")
        print(f"Modbus服務器: {modbus_host}:{modbus_port}")
        print(f"系統基地址: {self.base_address}")
        print(f"流程控制地址: {self.AUTO_PROGRAM_CONTROL}")
        print("檢測類型: DR_F/STACK二分類")
        print("流程配置: Flow1(VP抓取) + Flow2(出料)")
        print("流程控制: 寫入1350=1啟動, 寫入1350=0停止")
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案 - DR專案版本"""
        default_config = {
            "auto_program": {
                "cycle_interval": 2.0,
                "ccd1_timeout": 10.0,
                "vp_vibration_duration": 0.5,
                "vp_stop_delay": 0.2,
                "flow4_pulse_duration": 0.1,
                "max_dr_f_check": 5  # DR專案最多檢查5個DR_F
            },
            "vp_params": {
                "spread_action_code": 11,
                "spread_strength": 50,
                "spread_frequency": 43,
                "spread_duration": 0.5,
                "stop_command_code": 3
            },
            "timing": {
                "command_delay": 0.1,
                "status_check_interval": 0.1,
                "register_clear_delay": 0.05
            },
            "modbus_mapping": {
                "base_address": 1300,
                "auto_program_control": 1350,  # AutoProgram流程控制地址
                "motion_status": 1200,
                "flow1_control": 1240,
                "flow2_control": 1241,
                "flow1_complete": 1204,
                "flow2_complete": 1205
            }
        }
        
        try:
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dr_autoprogram_config.json')
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                print(f"已載入DR專案配置檔案: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"已創建DR專案預設配置檔案: {config_path}")
        except Exception as e:
            print(f"配置檔案處理失敗: {e}")
            
        return default_config
    def read_register(self, address: int) -> Optional[int]:
        """讀取單個寄存器"""
        try:
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            if not result.isError():
                return result.registers[0]
            return None
        except Exception:
            return None
    
    def write_register(self, address: int, value: int) -> bool:
        """寫入單個寄存器"""
        try:
            result = self.modbus_client.write_register(address, value, slave=1)
            return not result.isError()
        except Exception:
            return False
    
    def start_control_monitor(self):
        """啟動控制監控執行緒"""
        if self.control_monitor_running:
            return
        
        print("[ControlMonitor] 啟動AutoProgram流程控制監控")
        self.control_monitor_running = True
        self.control_monitor_thread = threading.Thread(target=self._control_monitor_loop, daemon=True)
        self.control_monitor_thread.start()
    
    def stop_control_monitor(self):
        """停止控制監控執行緒"""
        if self.control_monitor_running:
            print("[ControlMonitor] 停止AutoProgram流程控制監控")
            self.control_monitor_running = False
            if self.control_monitor_thread and self.control_monitor_thread.is_alive():
                self.control_monitor_thread.join(timeout=1.0)
    
    def _control_monitor_loop(self):
        """控制監控主循環"""
        while self.control_monitor_running and self.connected:
            try:
                # 讀取AutoProgram控制地址
                current_control = self.read_register(self.AUTO_PROGRAM_CONTROL)
                
                if current_control is not None:
                    # 檢查控制指令變化
                    if current_control != self.last_auto_program_control:
                        print(f"[ControlMonitor] AutoProgram控制指令變化: {self.last_auto_program_control} → {current_control}")
                        
                        if current_control == 1 and not self.auto_program_enabled:
                            # 啟動AutoProgram流程
                            print("[ControlMonitor] 收到啟動指令，啟動AutoProgram流程")
                            self._start_auto_program_flow()
                            
                        elif current_control == 0 and self.auto_program_enabled:
                            # 停止AutoProgram流程
                            print("[ControlMonitor] 收到停止指令，停止AutoProgram流程")
                            self._stop_auto_program_flow()
                        
                        self.last_auto_program_control = current_control
                
                # 100ms檢查間隔
                time.sleep(0.1)
                
            except Exception as e:
                print(f"[ControlMonitor] 控制監控異常: {e}")
                time.sleep(1.0)
    
    def _start_auto_program_flow(self):
        """啟動AutoProgram流程"""
        try:
            if self.auto_program_enabled:
                print("[ControlMonitor] AutoProgram流程已在運行中")
                return
            
            print("[ControlMonitor] 啟動AutoProgram流程...")
            
            # 建立AutoFeeding執行緒
            if not self.auto_feeding_thread:
                self.auto_feeding_thread = AutoFeedingThread(self.modbus_client, self.config)
            
            # 建立RobotJob執行緒
            if not self.robot_job_thread:
                self.robot_job_thread = RobotJobThread(self.modbus_client, self.config)
                
                # 設置執行緒間的相互參考
                self.robot_job_thread.auto_feeding_ref = self.auto_feeding_thread
                
                # 設置回調連接
                self.auto_feeding_thread.set_feeding_ready_callback(self.robot_job_thread.on_feeding_ready)
            
            # 啟動執行緒
            if not self.auto_feeding_thread.running:
                self.auto_feeding_thread.start()
            
            if not self.robot_job_thread.running:
                self.robot_job_thread.start()
            
            # 更新狀態
            self.auto_program_enabled = True
            self.system_status = SystemStatus.AUTO_FEEDING_RUNNING
            
            print("[ControlMonitor] ✓ AutoProgram流程已啟動")
            
        except Exception as e:
            print(f"[ControlMonitor] 啟動AutoProgram流程失敗: {e}")
    
    def _stop_auto_program_flow(self):
        """停止AutoProgram流程"""
        try:
            if not self.auto_program_enabled:
                print("[ControlMonitor] AutoProgram流程未在運行")
                return
            
            print("[ControlMonitor] 停止AutoProgram流程...")
            
            # 停止執行緒
            if self.auto_feeding_thread and self.auto_feeding_thread.running:
                self.auto_feeding_thread.stop()
                print("[ControlMonitor] AutoFeeding執行緒已停止")
            
            if self.robot_job_thread and self.robot_job_thread.running:
                self.robot_job_thread.stop()
                print("[ControlMonitor] RobotJob執行緒已停止")
            
            # 更新狀態
            self.auto_program_enabled = False
            self.system_status = SystemStatus.STOPPED
            
            print("[ControlMonitor] ✓ AutoProgram流程已停止")
            
        except Exception as e:
            print(f"[ControlMonitor] 停止AutoProgram流程失敗: {e}")
        """載入配置檔案 - DR專案版本"""
        default_config = {
            "auto_program": {
                "cycle_interval": 2.0,
                "ccd1_timeout": 10.0,
                "vp_vibration_duration": 0.5,
                "vp_stop_delay": 0.2,
                "flow4_pulse_duration": 0.1,
                "max_dr_f_check": 5  # DR專案最多檢查5個DR_F
            },
            "vp_params": {
                "spread_action_code": 11,
                "spread_strength": 50,
                "spread_frequency": 43,
                "spread_duration": 0.5,
                "stop_command_code": 3
            },
            "timing": {
                "command_delay": 0.1,
                "status_check_interval": 0.1,
                "register_clear_delay": 0.05
            },
            "modbus_mapping": {
                "base_address": 1300,
                "auto_program_control": 1350,  # AutoProgram流程控制地址
                "motion_status": 1200,
                "flow1_control": 1240,
                "flow2_control": 1241,
                "flow1_complete": 1204,
                "flow2_complete": 1205
            }
        }
        
        try:
            config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dr_autoprogram_config.json')
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                print(f"已載入DR專案配置檔案: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"已創建DR專案預設配置檔案: {config_path}")
        except Exception as e:
            print(f"配置檔案處理失敗: {e}")
            
        return default_config
    
    def connect(self) -> bool:
        """連接Modbus服務器"""
        try:
            if not MODBUS_AVAILABLE:
                print("Modbus功能不可用")
                return False
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.connected = self.modbus_client.connect()
            
            if self.connected:
                print(f"Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
                self.init_system_registers()
            else:
                print(f"Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
            
            return self.connected
        except Exception as e:
            print(f"Modbus連接異常: {e}")
            self.connected = False
            return False
    
    def init_system_registers(self):
        """初始化系統寄存器"""
        try:
            # 1300: 系統狀態
            # 1301: AutoFeeding執行緒狀態 
            # 1302: RobotJob執行緒狀態
            # 1303: 系統錯誤代碼
            # 1304: AutoFeeding週期計數
            # 1305: DR_F找到次數
            # 1306: Flow4觸發次數  
            # 1307: VP震動次數
            # 1308: Flow1觸發次數
            # 1309: Flow2完成次數 (DR專案)
            # 1350: AutoProgram流程控制 (0=停止, 1=啟動)
            
            self.modbus_client.write_register(1300, SystemStatus.STOPPED.value, slave=1)
            self.modbus_client.write_register(1301, 0, slave=1)  # AutoFeeding停止
            self.modbus_client.write_register(1302, 0, slave=1)  # RobotJob停止
            self.modbus_client.write_register(1303, 0, slave=1)  # 無錯誤
            self.modbus_client.write_register(1350, 0, slave=1)  # AutoProgram流程停止
            print("DR專案系統寄存器初始化完成 (包含AutoProgram流程控制)")
        except Exception as e:
            print(f"系統寄存器初始化失敗: {e}")
    
    def update_system_registers(self):
        """更新系統寄存器"""
        try:
            if not self.connected:
                return
            
            # 更新系統狀態
            self.modbus_client.write_register(1300, self.system_status.value, slave=1)
            
            # 更新AutoProgram流程控制狀態
            current_auto_program_state = 1 if self.auto_program_enabled else 0
            self.modbus_client.write_register(1350, current_auto_program_state, slave=1)
            
            # 更新執行緒狀態
            auto_feeding_running = 1 if (self.auto_feeding_thread and self.auto_feeding_thread.running) else 0
            robot_job_running = 1 if (self.robot_job_thread and self.robot_job_thread.running) else 0
            
            self.modbus_client.write_register(1301, auto_feeding_running, slave=1)
            self.modbus_client.write_register(1302, robot_job_running, slave=1)
            
            # 更新統計資訊
            if self.auto_feeding_thread:
                self.modbus_client.write_register(1304, self.auto_feeding_thread.cycle_count, slave=1)
                self.modbus_client.write_register(1305, self.auto_feeding_thread.dr_f_found_count, slave=1)
                self.modbus_client.write_register(1306, self.auto_feeding_thread.flow4_trigger_count, slave=1)
                self.modbus_client.write_register(1307, self.auto_feeding_thread.vp_vibration_count, slave=1)
            
            if self.robot_job_thread:
                self.modbus_client.write_register(1308, self.robot_job_thread.flow1_trigger_count, slave=1)
                self.modbus_client.write_register(1309, self.robot_job_thread.flow2_complete_count, slave=1)  # DR專案使用Flow2
                
        except Exception as e:
            print(f"系統寄存器更新失敗: {e}")
    
    def disconnect(self):
        """斷開Modbus連接"""
        # 停止控制監控
        self.stop_control_monitor()
        
        if self.modbus_client and self.connected:
            self.modbus_client.close()
            self.connected = False
            print("Modbus連接已斷開")
    
    def start_system(self):
        """啟動協調控制系統 - 修改為僅啟動控制監控"""
        if not self.connected:
            print("請先連接Modbus服務器")
            return
        
        print("=== 啟動VP入料+機械臂協調控制系統 (DR專案版本) ===")
        print("檢測類型: DR_F/STACK二分類")
        print("流程配置: Flow1(VP抓取) + Flow2(出料)")
        print("流程控制: 通過寫入1350地址來控制流程開關")
        print("  - 寫入1350=1: 啟動AutoProgram流程")
        print("  - 寫入1350=0: 停止AutoProgram流程")
        
        # 啟動控制監控執行緒
        self.start_control_monitor()
        
        print("DR專案協調控制系統控制監控已啟動，等待流程控制指令...")
    
    def stop_system(self):
        """停止協調控制系統"""
        print("=== 停止VP入料+機械臂協調控制系統 (DR專案版本) ===")
        
        # 停止AutoProgram流程
        if self.auto_program_enabled:
            self._stop_auto_program_flow()
        
        # 停止控制監控
        self.stop_control_monitor()
        
        self.system_status = SystemStatus.STOPPED
        
        # 更新系統寄存器
        self.update_system_registers()
        
        print("DR專案協調控制系統已停止")
        self.print_statistics()
    
    def print_statistics(self):
        """輸出統計資訊 - DR專案版本"""
        print(f"\n=== DR專案協調控制系統統計 ===")
        
        if self.auto_feeding_thread:
            print(f"AutoFeeding統計:")
            print(f"  總週期數: {self.auto_feeding_thread.cycle_count}")
            print(f"  DR_F找到次數: {self.auto_feeding_thread.dr_f_found_count}")
            print(f"  Flow4觸發次數: {self.auto_feeding_thread.flow4_trigger_count}")
            print(f"  VP震動次數: {self.auto_feeding_thread.vp_vibration_count}")
            if self.auto_feeding_thread.cycle_count > 0:
                success_rate = (self.auto_feeding_thread.dr_f_found_count / self.auto_feeding_thread.cycle_count) * 100
                print(f"  DR_F找到率: {success_rate:.1f}%")
        
        if self.robot_job_thread:
            print(f"RobotJob統計:")
            print(f"  Flow1觸發次數: {self.robot_job_thread.flow1_trigger_count}")
            print(f"  Flow2完成次數: {self.robot_job_thread.flow2_complete_count}")  # DR專案使用Flow2
    
    def get_status_info(self) -> Dict[str, Any]:
        """獲取狀態資訊"""
        status = {
            "connected": self.connected,
            "system_status": self.system_status.name,
            "auto_program_enabled": self.auto_program_enabled,
            "auto_program_control_address": self.AUTO_PROGRAM_CONTROL,
            "auto_feeding_running": self.auto_feeding_thread.running if self.auto_feeding_thread else False,
            "robot_job_running": self.robot_job_thread.running if self.robot_job_thread else False,
            "control_monitor_running": self.control_monitor_running,
            "project_type": "DR",
            "detection_types": ["DR_F", "STACK"],
            "flow_config": "Flow1(VP抓取) + Flow2(出料)"
        }
        
        if self.auto_feeding_thread:
            status.update({
                "auto_feeding_cycle_count": self.auto_feeding_thread.cycle_count,
                "dr_f_found_count": self.auto_feeding_thread.dr_f_found_count,
                "flow4_trigger_count": self.auto_feeding_thread.flow4_trigger_count,
                "vp_vibration_count": self.auto_feeding_thread.vp_vibration_count,
                "feeding_ready": self.auto_feeding_thread.feeding_ready
            })
        
        if self.robot_job_thread:
            status.update({
                "prepare_done": self.robot_job_thread.prepare_done,
                "flow1_trigger_count": self.robot_job_thread.flow1_trigger_count,
                "flow2_complete_count": self.robot_job_thread.flow2_complete_count
            })
        
        return status


def main():
    """主程序"""
    print("VP入料+機械臂協調控制系統啟動 (DR專案版本)")
    print("檢測類型: DR_F/STACK二分類")
    print("流程配置: Flow1(VP抓取) + Flow2(出料)")
    
    # 創建控制器
    controller = AutoProgramRobotJobController()
    
    # 連接Modbus
    if not controller.connect():
        print("Modbus連接失敗，程序退出")
        return
    
    try:
        # 啟動協調控制系統
        controller.start_system()
        
        # 定期更新系統寄存器
        def update_registers():
            while True:
                controller.update_system_registers()
                time.sleep(2.0)
        
        update_thread = threading.Thread(target=update_registers, daemon=True)
        update_thread.start()
        
        # 主循環 - 等待用戶操作
        print("\nDR專案指令說明:")
        print("  s - 顯示狀態")
        print("  start - 手動啟動AutoProgram流程 (等同於寫入1350=1)")
        print("  stop - 手動停止AutoProgram流程 (等同於寫入1350=0)")
        print("  pause - 暫停自動入料")
        print("  resume - 恢復自動入料")
        print("  r - 重啟系統")
        print("  q - 退出程序")
        print("\n流程控制說明:")
        print("  - Modbus寫入1350=1: 啟動AutoProgram流程")
        print("  - Modbus寫入1350=0: 停止AutoProgram流程")
        print("  - 當前控制狀態將自動監控並執行對應動作")
        
        while True:
            try:
                cmd = input("\n請輸入指令: ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 's':
                    status = controller.get_status_info()
                    print(f"\nDR專案系統狀態:")
                    for key, value in status.items():
                        print(f"  {key}: {value}")
                elif cmd == 'start':
                    print("手動啟動AutoProgram流程...")
                    if controller.write_register(controller.AUTO_PROGRAM_CONTROL, 1):
                        print("✓ 已寫入1350=1，AutoProgram流程將被啟動")
                    else:
                        print("✗ 寫入1350=1失敗")
                elif cmd == 'stop':
                    print("手動停止AutoProgram流程...")
                    if controller.write_register(controller.AUTO_PROGRAM_CONTROL, 0):
                        print("✓ 已寫入1350=0，AutoProgram流程將被停止")
                    else:
                        print("✗ 寫入1350=0失敗")
                elif cmd == 'r':
                    controller.stop_system()
                    time.sleep(1.0)
                    controller.start_system()
                elif cmd == 'pause':
                    if controller.auto_feeding_thread:
                        controller.auto_feeding_thread.pause_for_robot_operation()
                elif cmd == 'resume':
                    if controller.auto_feeding_thread:
                        controller.auto_feeding_thread.resume_after_robot_operation()
                else:
                    print("無效指令")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
    
    finally:
        # 清理資源
        controller.stop_system()
        controller.disconnect()
        print("DR專案程序已退出")


if __name__ == "__main__":
    main()