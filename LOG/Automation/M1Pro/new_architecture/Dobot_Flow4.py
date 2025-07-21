#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_Flow4_vibration.py - Flow4 震動投料流程 (改良版 - 延遲百分比控制)
基於統一Flow架構的DIO控制執行器
控制震動投料：DO4先啟動，DO1在指定百分比延遲後執行脈衝
增加完整API調用打印和錯誤處理
"""

import time
import threading
from typing import Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

# 導入新架構基類
from flow_base import FlowExecutor, FlowResult, FlowStatus


class Flow4VibrationFeedExecutor(FlowExecutor):
    """Flow4: 震動投料流程執行器 (DIO控制 - 延遲百分比版)"""
    
    def __init__(self):
        super().__init__(flow_id=4, flow_name="震動投料流程(延遲百分比)")
        self.dio_steps = []
        
        # DIO腳位定義
        self.DIO_PINS = {
            'VIBRATION_CONTROL': 1,    # DO1: 震動控制 (延遲脈衝)
            'FEED_ENABLE': 4,          # DO4: 投料使能 (先啟動)
        }
        
        # 🟢 時間延遲設定 - 增加百分比控制
        self.TIMING_CONFIG = {
            'FEED_DURATION': 2.0,         # DO4持續時間 (秒)
            'PULSE_DELAY_PERCENT': 30,     # DO1延遲百分比 (0-100)
            'PULSE_HIGH_TIME': 0.3,        # DO1 HIGH持續時間 (秒)
            'PULSE_LOW_TIME': 0.3,         # DO1 LOW持續時間 (秒)
            'PULSE_COUNT': 1               # DO1脈衝次數
        }
        
        # 🟢 計算實際延遲時間
        self.calculated_delay = self._calculate_pulse_delay()
        
        # 執行緒控制
        self.pulse_thread = None
        self.pulse_thread_running = False
        self.do4_thread = None
        self.do4_thread_running = False
        
        # 建構流程步驟
        self.build_flow_steps()
        
    def _calculate_pulse_delay(self) -> float:
        """🟢 計算DO1脈衝延遲時間"""
        delay = (self.TIMING_CONFIG['FEED_DURATION'] * self.TIMING_CONFIG['PULSE_DELAY_PERCENT']) / 100.0
        return delay
    
    def build_flow_steps(self):
        """建構Flow4步驟"""
        self.dio_steps = [
            # 1. 啟動DO4投料使能
            {'type': 'start_do4_feed_enable', 'params': {}},
            
            # 2. 延遲後啟動DO1脈衝
            {'type': 'start_delayed_do1_pulse', 'params': {}},
            
            # 3. 等待流程完成
            {'type': 'wait_completion', 'params': {'duration': self.TIMING_CONFIG['FEED_DURATION']}},
            
            # 4. 確保所有輸出關閉
            {'type': 'stop_all_outputs', 'params': {}}
        ]
        
        self.total_steps = len(self.dio_steps)
        
        # 打印配置資訊
        print(f"[Flow4] 時間配置:")
        print(f"  DO4持續時間: {self.TIMING_CONFIG['FEED_DURATION']}秒")
        print(f"  DO1延遲百分比: {self.TIMING_CONFIG['PULSE_DELAY_PERCENT']}%")
        print(f"  DO1實際延遲: {self.calculated_delay:.2f}秒")
        print(f"  DO1脈衝次數: {self.TIMING_CONFIG['PULSE_COUNT']}次")
    
    def update_timing_config(self, **kwargs):
        """🟢 更新時間配置"""
        for key, value in kwargs.items():
            if key in self.TIMING_CONFIG:
                old_value = self.TIMING_CONFIG[key]
                self.TIMING_CONFIG[key] = value
                print(f"[Flow4] 配置更新: {key} = {old_value} → {value}")
        
        # 重新計算延遲時間
        old_delay = self.calculated_delay
        self.calculated_delay = self._calculate_pulse_delay()
        print(f"[Flow4] 延遲時間更新: {old_delay:.2f}s → {self.calculated_delay:.2f}s")
    
    def execute(self) -> FlowResult:
        """執行Flow4主邏輯"""
        self.status = FlowStatus.RUNNING
        self.start_time = time.time()
        self.current_step = 0
        
        print(f"\n[Flow4] === 開始執行Flow4震動投料流程 (延遲百分比版) ===")
        print(f"[Flow4] 流程序列:")
        print(f"  1. DO4先啟動 (持續{self.TIMING_CONFIG['FEED_DURATION']}秒)")
        print(f"  2. 延遲{self.calculated_delay:.2f}秒後DO1開始脈衝")
        print(f"  3. DO1執行{self.TIMING_CONFIG['PULSE_COUNT']}次脈衝")
        print(f"  4. 所有輸出關閉")
        print(f"[Flow4] 總步驟數: {self.total_steps}")
        
        # 檢查初始化
        if not self.robot or not self.robot.is_connected:
            print(f"[Flow4] ✗ 機械臂未連接或未初始化")
            return FlowResult(
                success=False,
                error_message="機械臂未連接或未初始化",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        # 檢查dashboard_api連接
        if not hasattr(self.robot, 'dashboard_api') or self.robot.dashboard_api is None:
            print(f"[Flow4] ✗ dashboard_api未初始化")
            return FlowResult(
                success=False,
                error_message="dashboard_api未初始化",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
        
        print(f"[Flow4] ✓ 機械臂連接檢查通過")
        print(f"[Flow4] ✓ dashboard_api連接檢查通過")
        
        try:
            for step in self.dio_steps:
                if self.status == FlowStatus.PAUSED:
                    time.sleep(0.1)
                    continue
                    
                if self.status == FlowStatus.ERROR:
                    break
                
                print(f"\n[Flow4] 步驟 {self.current_step + 1}/{self.total_steps}: {step['type']}")
                
                # 執行步驟
                success = False
                
                if step['type'] == 'start_do4_feed_enable':
                    success = self._execute_start_do4_feed_enable()
                elif step['type'] == 'start_delayed_do1_pulse':
                    success = self._execute_start_delayed_do1_pulse()
                elif step['type'] == 'wait_completion':
                    success = self._execute_wait_completion(step['params'])
                elif step['type'] == 'stop_all_outputs':
                    success = self._execute_stop_all_outputs()
                else:
                    print(f"[Flow4] ✗ 未知步驟類型: {step['type']}")
                    success = False
                
                if not success:
                    self.status = FlowStatus.ERROR
                    print(f"[Flow4] ✗ 步驟 {self.current_step + 1}/{self.total_steps} 失敗")
                    return FlowResult(
                        success=False,
                        error_message=f"步驟 {step['type']} 執行失敗",
                        execution_time=time.time() - self.start_time,
                        steps_completed=self.current_step,
                        total_steps=self.total_steps
                    )
                
                print(f"[Flow4] ✓ 步驟 {self.current_step + 1}/{self.total_steps} 完成")
                self.current_step += 1
            
            # 流程成功完成
            self.status = FlowStatus.COMPLETED
            execution_time = time.time() - self.start_time
            
            print(f"\n[Flow4] === Flow4震動投料流程執行完成 ===")
            print(f"[Flow4] 執行時間: {execution_time:.2f}秒")
            print(f"[Flow4] 完成步驟: {self.current_step}/{self.total_steps}")
            
            return FlowResult(
                success=True,
                execution_time=execution_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps,
                flow_data={
                    'vibration_feed_completed': True,
                    'actual_delay': self.calculated_delay,
                    'delay_percent': self.TIMING_CONFIG['PULSE_DELAY_PERCENT']
                }
            )
            
        except Exception as e:
            self.status = FlowStatus.ERROR
            print(f"[Flow4] ✗ Flow4執行異常: {str(e)}")
            return FlowResult(
                success=False,
                error_message=f"Flow4執行異常: {str(e)}",
                execution_time=time.time() - self.start_time,
                steps_completed=self.current_step,
                total_steps=self.total_steps
            )
    
    def _execute_start_do4_feed_enable(self) -> bool:
        """🟢 執行DO4投料使能啟動"""
        try:
            print(f"[Flow4]   DO4投料使能啟動")
            print(f"[Flow4]   DO4將持續HIGH {self.TIMING_CONFIG['FEED_DURATION']}秒")
            
            # 啟動DO4 (投料使能)
            print(f"[Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['FEED_ENABLE']}, 1)")
            try:
                result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['FEED_ENABLE'], 1)
                print(f"[Flow4]     API返回結果: {result}")
                print(f"[Flow4]     ✓ DO{self.DIO_PINS['FEED_ENABLE']} = 1 執行成功")
            except Exception as e:
                print(f"[Flow4]     ✗ DO{self.DIO_PINS['FEED_ENABLE']}啟動失敗: {e}")
                return False
            
            # 🟢 創建DO4持續執行緒 (負責在指定時間後關閉DO4)
            self.do4_thread_running = True
            self.do4_thread = threading.Thread(target=self._execute_do4_duration_control, daemon=True)
            self.do4_thread.start()
            
            print(f"[Flow4]   ✓ DO{self.DIO_PINS['FEED_ENABLE']}已啟動，持續時間控制執行緒已啟動")
            return True
            
        except Exception as e:
            print(f"[Flow4]   ✗ DO4投料使能啟動失敗: {e}")
            return False
    
    def _execute_do4_duration_control(self):
        """🟢 DO4持續時間控制執行緒"""
        try:
            print(f"[Flow4]   DO4持續時間控制執行緒開始")
            print(f"[Flow4]   DO4將在{self.TIMING_CONFIG['FEED_DURATION']}秒後自動關閉")
            
            # 等待指定的持續時間
            time.sleep(self.TIMING_CONFIG['FEED_DURATION'])
            
            if self.do4_thread_running:
                # 關閉DO4
                print(f"[Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['FEED_ENABLE']}, 0)")
                try:
                    result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['FEED_ENABLE'], 0)
                    print(f"[Flow4]     API返回結果: {result}")
                    print(f"[Flow4]     ✓ DO{self.DIO_PINS['FEED_ENABLE']} = 0 執行成功 (自動關閉)")
                except Exception as e:
                    print(f"[Flow4]     ✗ DO{self.DIO_PINS['FEED_ENABLE']}自動關閉失敗: {e}")
            
            print(f"[Flow4]   ✓ DO4持續時間控制完成")
            
        except Exception as e:
            print(f"[Flow4]   ✗ DO4持續時間控制失敗: {e}")
        finally:
            self.do4_thread_running = False
    
    def _execute_start_delayed_do1_pulse(self) -> bool:
        """🟢 執行延遲DO1脈衝啟動"""
        try:
            print(f"[Flow4]   DO1延遲脈衝啟動")
            print(f"[Flow4]   DO1將在{self.calculated_delay:.2f}秒後開始脈衝")
            print(f"[Flow4]   延遲百分比: {self.TIMING_CONFIG['PULSE_DELAY_PERCENT']}%")
            
            # 創建延遲脈衝執行緒
            self.pulse_thread_running = True
            self.pulse_thread = threading.Thread(target=self._execute_delayed_do1_pulses, daemon=True)
            self.pulse_thread.start()
            
            print(f"[Flow4]   ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']}延遲脈衝執行緒已啟動")
            return True
            
        except Exception as e:
            print(f"[Flow4]   ✗ DO1延遲脈衝啟動失敗: {e}")
            return False
    
    def _execute_delayed_do1_pulses(self):
        """🟢 執行延遲DO1脈衝操作 - 在獨立執行緒中運行"""
        try:
            print(f"[Flow4]   DO1延遲脈衝執行緒開始")
            
            # 🟢 先延遲指定的時間
            print(f"[Flow4]   等待延遲時間: {self.calculated_delay:.2f}秒 ({self.TIMING_CONFIG['PULSE_DELAY_PERCENT']}%)")
            time.sleep(self.calculated_delay)
            
            if not self.pulse_thread_running:
                print(f"[Flow4]   執行緒已停止，取消DO1脈衝")
                return
            
            print(f"[Flow4]   延遲時間結束，開始DO1脈衝操作")
            
            # 執行脈衝
            for pulse_num in range(self.TIMING_CONFIG['PULSE_COUNT']):
                if not self.pulse_thread_running:
                    break
                    
                print(f"[Flow4]   DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝 {pulse_num + 1}/{self.TIMING_CONFIG['PULSE_COUNT']}")
                
                # DO1 HIGH
                print(f"[Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['VIBRATION_CONTROL']}, 1)")
                try:
                    result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['VIBRATION_CONTROL'], 1)
                    print(f"[Flow4]     API返回結果: {result}")
                    print(f"[Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']} = 1 執行成功")
                    print(f"[Flow4]     DO{self.DIO_PINS['VIBRATION_CONTROL']} HIGH (持續{self.TIMING_CONFIG['PULSE_HIGH_TIME']}秒)")
                except Exception as e:
                    print(f"[Flow4]     ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝{pulse_num + 1} HIGH失敗: {e}")
                    continue
                
                time.sleep(self.TIMING_CONFIG['PULSE_HIGH_TIME'])
                
                # DO1 LOW
                print(f"[Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['VIBRATION_CONTROL']}, 0)")
                try:
                    result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['VIBRATION_CONTROL'], 0)
                    print(f"[Flow4]     API返回結果: {result}")
                    print(f"[Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']} = 0 執行成功")
                    print(f"[Flow4]     DO{self.DIO_PINS['VIBRATION_CONTROL']} LOW (持續{self.TIMING_CONFIG['PULSE_LOW_TIME']}秒)")
                except Exception as e:
                    print(f"[Flow4]     ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝{pulse_num + 1} LOW失敗: {e}")
                    continue
                
                time.sleep(self.TIMING_CONFIG['PULSE_LOW_TIME'])
                
                print(f"[Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']}脈衝{pulse_num + 1}完成")
            
            print(f"[Flow4]   ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']}所有延遲脈衝操作完成")
            
        except Exception as e:
            print(f"[Flow4]   ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}延遲脈衝操作失敗: {e}")
        finally:
            self.pulse_thread_running = False
    
    def _execute_wait_completion(self, params: Dict[str, Any]) -> bool:
        """等待流程完成"""
        try:
            duration = params.get('duration', 2.0)
            print(f"[Flow4]   等待流程完成 ({duration}秒)")
            print(f"[Flow4]   此期間監控所有執行緒狀態")
            
            # 🟢 監控所有執行緒完成
            start_time = time.time()
            while time.time() - start_time < duration:
                # 檢查是否所有執行緒都完成
                do4_alive = self.do4_thread and self.do4_thread.is_alive()
                pulse_alive = self.pulse_thread and self.pulse_thread.is_alive()
                
                if not do4_alive and not pulse_alive:
                    elapsed = time.time() - start_time
                    print(f"[Flow4]   所有執行緒提前完成 (耗時{elapsed:.2f}秒)")
                    break
                
                time.sleep(0.1)
            
            # 確保執行緒結束
            if self.do4_thread and self.do4_thread.is_alive():
                print(f"[Flow4]   等待DO4執行緒完成...")
                self.do4_thread_running = False
                self.do4_thread.join(timeout=2.0)
                
                if self.do4_thread.is_alive():
                    print(f"[Flow4]   ⚠️ DO4執行緒未正常結束")
                else:
                    print(f"[Flow4]   ✓ DO4執行緒已結束")
            
            if self.pulse_thread and self.pulse_thread.is_alive():
                print(f"[Flow4]   等待DO1脈衝執行緒完成...")
                self.pulse_thread_running = False
                self.pulse_thread.join(timeout=2.0)
                
                if self.pulse_thread.is_alive():
                    print(f"[Flow4]   ⚠️ DO1脈衝執行緒未正常結束")
                else:
                    print(f"[Flow4]   ✓ DO1脈衝執行緒已結束")
            
            print(f"[Flow4]   ✓ 流程完成等待結束")
            return True
            
        except Exception as e:
            print(f"[Flow4]   ✗ 等待流程完成失敗: {e}")
            return False
    
    def _execute_stop_all_outputs(self) -> bool:
        """確保所有輸出關閉"""
        try:
            print(f"[Flow4]   關閉所有DO輸出")
            success = True
            
            # 停止所有執行緒
            self.do4_thread_running = False
            self.pulse_thread_running = False
            
            # 關閉DO4 (投料使能)
            print(f"[Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['FEED_ENABLE']}, 0)")
            try:
                result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['FEED_ENABLE'], 0)
                print(f"[Flow4]     API返回結果: {result}")
                print(f"[Flow4]     ✓ DO{self.DIO_PINS['FEED_ENABLE']} = 0 執行成功")
            except Exception as e:
                print(f"[Flow4]     ✗ DO{self.DIO_PINS['FEED_ENABLE']}關閉失敗: {e}")
                success = False
            
            # 關閉DO1 (震動控制)
            print(f"[Flow4]     正在執行: dashboard_api.DOExecute({self.DIO_PINS['VIBRATION_CONTROL']}, 0)")
            try:
                result = self.robot.dashboard_api.DOExecute(self.DIO_PINS['VIBRATION_CONTROL'], 0)
                print(f"[Flow4]     API返回結果: {result}")
                print(f"[Flow4]     ✓ DO{self.DIO_PINS['VIBRATION_CONTROL']} = 0 執行成功")
            except Exception as e:
                print(f"[Flow4]     ✗ DO{self.DIO_PINS['VIBRATION_CONTROL']}關閉失敗: {e}")
                success = False
            
            # 等待執行緒結束
            for thread, name in [(self.do4_thread, "DO4"), (self.pulse_thread, "DO1脈衝")]:
                if thread and thread.is_alive():
                    thread.join(timeout=1.0)
                    if thread.is_alive():
                        print(f"[Flow4]     ⚠️ {name}執行緒強制結束")
                    else:
                        print(f"[Flow4]     ✓ {name}執行緒已正常結束")
            
            if success:
                print(f"[Flow4]   ✓ 所有DO輸出已關閉")
            else:
                print(f"[Flow4]   ⚠️ 部分DO輸出關閉失敗")
                
            return success
            
        except Exception as e:
            print(f"[Flow4]   ✗ 關閉DO輸出失敗: {e}")
            return False
    
    def pause(self) -> bool:
        """暫停Flow"""
        self.status = FlowStatus.PAUSED
        print("[Flow4] Flow4已暫停")
        return True
        
    def resume(self) -> bool:
        """恢復Flow"""
        if self.status == FlowStatus.PAUSED:
            self.status = FlowStatus.RUNNING
            print("[Flow4] Flow4已恢復")
            return True
        return False
        
    def stop(self) -> bool:
        """停止Flow4執行"""
        try:
            print(f"[Flow4] 正在停止Flow4執行...")
            self.status = FlowStatus.ERROR
            
            # 停止所有執行緒
            self.do4_thread_running = False
            self.pulse_thread_running = False
            
            # 等待執行緒結束
            for thread in [self.do4_thread, self.pulse_thread]:
                if thread and thread.is_alive():
                    thread.join(timeout=2.0)
            
            # 關閉所有輸出
            self._execute_stop_all_outputs()
            
            print(f"[Flow4] ✓ Flow4已停止")
            return True
            
        except Exception as e:
            print(f"[Flow4] ✗ Flow4停止過程出錯: {e}")
            return False
        
    def get_progress(self) -> int:
        """取得進度百分比"""
        if self.total_steps == 0:
            return 0
        return int((self.current_step / self.total_steps) * 100)
    
    def get_timing_config(self) -> Dict[str, Any]:
        """🟢 獲取當前時間配置"""
        config = self.TIMING_CONFIG.copy()
        config['calculated_delay'] = self.calculated_delay
        return config
    
    def print_timing_info(self):
        """🟢 打印時間配置資訊"""
        print(f"[Flow4] 當前時間配置:")
        print(f"  DO4持續時間: {self.TIMING_CONFIG['FEED_DURATION']}秒")
        print(f"  DO1延遲百分比: {self.TIMING_CONFIG['PULSE_DELAY_PERCENT']}%")
        print(f"  DO1實際延遲: {self.calculated_delay:.2f}秒")
        print(f"  DO1脈衝HIGH時間: {self.TIMING_CONFIG['PULSE_HIGH_TIME']}秒")
        print(f"  DO1脈衝LOW時間: {self.TIMING_CONFIG['PULSE_LOW_TIME']}秒")
        print(f"  DO1脈衝次數: {self.TIMING_CONFIG['PULSE_COUNT']}次")