# -*- coding: utf-8 -*-
"""
生產線震動盤控制模組 - ProductionVibrationPlate
專為生產線環境設計的震動盤控制類別
支援斷線重連、狀態保持、錯誤恢復機制
"""

import time
import threading
import json
import os
from typing import Dict, Any, Optional, Tuple
from pymodbus.client import ModbusTcpClient


class ProductionVibrationPlate:
    """生產線震動盤控制器 - 獨立Python模組"""
    
    # 震動模式定義
    VIBRATION_MODES = {
        'stop': 0,
        'up': 1,
        'down': 2,
        'left': 3,
        'right': 4,
        'upleft': 5,
        'downleft': 6,
        'upright': 7,
        'downright': 8,
        'horizontal': 9,
        'vertical': 10,
        'spread': 11
    }
    
    # 設備狀態定義
    STATUS_DISCONNECTED = 0
    STATUS_IDLE = 1
    STATUS_RUNNING = 2
    STATUS_ERROR = 3
    
    def __init__(self, config_file="production_vp_config.json"):
        """初始化生產線震動盤控制器"""
        # 載入配置
        self.config = self._load_config(config_file)
        
        # 連接組件
        self.client: Optional[ModbusTcpClient] = None
        self._connection_lock = threading.Lock()
        
        # 設備狀態
        self._connected = False
        self._device_status = self.STATUS_DISCONNECTED
        self._last_error = None
        self._connection_attempts = 0
        self._last_connection_time = 0
        
        # 震動參數狀態保持 (斷線時保存)
        self._vibration_state = {
            'mode': 'stop',
            'intensity': 0,
            'frequency': 100,
            'duration': 0,
            'backlight_enabled': False,
            'backlight_brightness': 28,
            'is_vibrating': False,
            'vibration_start_time': 0
        }
        
        # 統計數據
        self._operation_count = 0
        self._error_count = 0
        self._reconnection_count = 0
        self._start_time = time.time()
        
        # 自動重連控制
        self._auto_reconnect = True
        self._reconnect_interval = 2.0  # 重連間隔(秒)
        self._max_reconnect_attempts = 10
        
        print(f"生產線震動盤控制器初始化完成")
        print(f"設備地址: {self.config['connection']['ip']}:{self.config['connection']['port']}")
    
    def _load_config(self, config_file: str) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "connection": {
                "ip": "192.168.1.7",
                "port": 1000,
                "slave_id": 10,
                "timeout": 0.5
            },
            "operation_limits": {
                "max_intensity": 100,
                "min_intensity": 1,
                "max_frequency": 200,
                "min_frequency": 10,
                "max_duration": 300,
                "max_brightness": 100
            },
            "retry_settings": {
                "max_attempts": 3,
                "retry_delay": 0.1
            },
            "defaults": {
                "brightness": 28,
                "frequency": 100,
                "intensity": 50
            }
        }
        
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(current_dir, config_file)
            
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                print(f"已載入配置: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"已創建預設配置: {config_path}")
        except Exception as e:
            print(f"配置載入失敗: {e}")
            
        return default_config
    
    def connect(self) -> bool:
        """連接到震動盤設備"""
        with self._connection_lock:
            try:
                if self.client:
                    self.client.close()
                
                conn_config = self.config['connection']
                self.client = ModbusTcpClient(
                    host=conn_config['ip'],
                    port=conn_config['port'],
                    timeout=conn_config['timeout']
                )
                
                if self.client.connect():
                    self._connected = True
                    self._device_status = self.STATUS_IDLE
                    self._last_error = None
                    self._connection_attempts = 0
                    self._last_connection_time = time.time()
                    
                    # 初始化設備
                    self._initialize_device()
                    
                    print(f"震動盤連接成功: {conn_config['ip']}:{conn_config['port']}")
                    return True
                else:
                    self._connected = False
                    self._device_status = self.STATUS_DISCONNECTED
                    print("震動盤連接失敗")
                    return False
                    
            except Exception as e:
                self._connected = False
                self._device_status = self.STATUS_ERROR
                self._last_error = str(e)
                self._connection_attempts += 1
                print(f"震動盤連接異常: {e}")
                return False
    
    def disconnect(self):
        """斷開連接"""
        with self._connection_lock:
            try:
                # 停止震動
                if self._connected:
                    self.stop()
                
                if self.client:
                    self.client.close()
                    
                self._connected = False
                self._device_status = self.STATUS_DISCONNECTED
                print("震動盤已斷開連接")
                
            except Exception as e:
                print(f"斷開連接異常: {e}")
    
    def reconnect(self) -> bool:
        """手動重連"""
        print("嘗試重新連接震動盤...")
        
        # 斷開現有連接
        self.disconnect()
        time.sleep(0.5)
        
        # 重新連接
        if self.connect():
            self._reconnection_count += 1
            print(f"重連成功 (第{self._reconnection_count}次)")
            
            # 恢復斷線前的狀態
            self._restore_vibration_state()
            return True
        else:
            print("重連失敗")
            return False
    
    def _auto_reconnect_if_needed(self) -> bool:
        """自動重連機制"""
        if not self._auto_reconnect or self._connected:
            return self._connected
            
        current_time = time.time()
        if (current_time - self._last_connection_time) < self._reconnect_interval:
            return False
            
        if self._connection_attempts >= self._max_reconnect_attempts:
            return False
            
        return self.reconnect()
    
    def _initialize_device(self):
        """初始化設備狀態"""
        try:
            defaults = self.config['defaults']
            
            # 設置背光
            self.set_backlight_brightness(defaults['brightness'])
            self.set_backlight(False)  # 預設關閉背光
            
            # 確保停止狀態
            self.stop()
            
            print(f"設備初始化完成 - 預設亮度: {defaults['brightness']}")
            
        except Exception as e:
            print(f"設備初始化失敗: {e}")
    
    def _restore_vibration_state(self):
        """恢復震動狀態 (重連後)"""
        try:
            state = self._vibration_state
            
            # 恢復背光設置
            if state['backlight_enabled']:
                self.set_backlight_brightness(state['backlight_brightness'])
                self.set_backlight(True)
            
            # 如果之前在震動，嘗試恢復
            if state['is_vibrating'] and state['mode'] != 'stop':
                remaining_time = 0
                if state['duration'] > 0:
                    elapsed = time.time() - state['vibration_start_time']
                    remaining_time = max(0, state['duration'] - elapsed)
                
                if remaining_time > 0:
                    print(f"恢復震動狀態: {state['mode']}, 剩餘時間: {remaining_time:.1f}秒")
                    self.vibrate(
                        mode=state['mode'],
                        intensity=state['intensity'],
                        frequency=state['frequency'],
                        duration=remaining_time
                    )
                    
        except Exception as e:
            print(f"狀態恢復失敗: {e}")
    
    def _write_register(self, address: int, value: int, retries: int = None) -> bool:
        """寫入寄存器 (帶重試機制)"""
        if not self._connected and not self._auto_reconnect_if_needed():
            return False
        
        if retries is None:
            retries = self.config['retry_settings']['max_attempts']
        
        for attempt in range(retries):
            try:
                result = self.client.write_register(
                    address, value, 
                    slave=self.config['connection']['slave_id']
                )
                
                if not result.isError():
                    return True
                    
            except Exception as e:
                if attempt == retries - 1:
                    print(f"寫入寄存器失敗 (地址{address}): {e}")
                    self._error_count += 1
                    
                time.sleep(self.config['retry_settings']['retry_delay'])
        
        return False
    
    def _read_register(self, address: int, retries: int = None) -> Optional[int]:
        """讀取寄存器 (帶重試機制)"""
        if not self._connected and not self._auto_reconnect_if_needed():
            return None
        
        if retries is None:
            retries = self.config['retry_settings']['max_attempts']
        
        for attempt in range(retries):
            try:
                result = self.client.read_holding_registers(
                    address, count=1,
                    slave=self.config['connection']['slave_id']
                )
                
                if not result.isError():
                    return result.registers[0]
                    
            except Exception as e:
                if attempt == retries - 1:
                    print(f"讀取寄存器失敗 (地址{address}): {e}")
                    self._error_count += 1
                    
                time.sleep(self.config['retry_settings']['retry_delay'])
        
        return None
    
    def set_backlight(self, enabled: bool) -> bool:
        """設置背光開關"""
        try:
            value = 1 if enabled else 0
            success = self._write_register(5, value)
            
            if success:
                self._vibration_state['backlight_enabled'] = enabled
                self._operation_count += 1
                print(f"背光{'開啟' if enabled else '關閉'}成功")
            
            return success
            
        except Exception as e:
            print(f"設置背光失敗: {e}")
            return False
    
    def set_backlight_brightness(self, brightness: int) -> bool:
        """設置背光亮度 (1-100)"""
        try:
            limits = self.config['operation_limits']
            brightness = max(1, min(brightness, limits['max_brightness']))
            
            success = self._write_register(6, brightness)
            
            if success:
                self._vibration_state['backlight_brightness'] = brightness
                self._operation_count += 1
                print(f"背光亮度設置成功: {brightness}")
            
            return success
            
        except Exception as e:
            print(f"設置背光亮度失敗: {e}")
            return False
    
    def vibrate(self, mode: str, intensity: int, frequency: int = None, duration: float = 0) -> bool:
        """執行震動
        
        Args:
            mode: 震動模式 ('up', 'down', 'left', 'right', 'horizontal', 'vertical', 'spread' 等)
            intensity: 震動強度 (1-100)
            frequency: 震動頻率 (10-200 Hz), 預設使用配置值
            duration: 持續時間(秒), 0表示持續震動直到手動停止
        """
        try:
            # 參數驗證
            if mode not in self.VIBRATION_MODES:
                print(f"無效的震動模式: {mode}")
                return False
            
            limits = self.config['operation_limits']
            intensity = max(limits['min_intensity'], min(intensity, limits['max_intensity']))
            
            if frequency is None:
                frequency = self.config['defaults']['frequency']
            frequency = max(limits['min_frequency'], min(frequency, limits['max_frequency']))
            
            if duration > limits['max_duration']:
                print(f"持續時間超過限制，設置為最大值: {limits['max_duration']}秒")
                duration = limits['max_duration']
            
            # 更新狀態 (斷線時保存參數)
            self._vibration_state.update({
                'mode': mode,
                'intensity': intensity,
                'frequency': frequency,
                'duration': duration,
                'is_vibrating': True,
                'vibration_start_time': time.time()
            })
            
            # 執行震動
            mode_code = self.VIBRATION_MODES[mode]
            
            # 寫入震動參數寄存器
            success = (
                self._write_register(1, mode_code) and        # 震動模式
                self._write_register(2, intensity) and        # 震動強度  
                self._write_register(3, frequency) and        # 震動頻率
                self._write_register(4, 1)                    # 啟動震動
            )
            
            if success:
                self._operation_count += 1
                self._device_status = self.STATUS_RUNNING
                
                print(f"震動啟動成功: 模式={mode}, 強度={intensity}, 頻率={frequency}Hz", end="")
                if duration > 0:
                    print(f", 持續={duration}秒")
                    # 定時停止
                    threading.Timer(duration, self._auto_stop).start()
                else:
                    print(" (持續震動)")
            else:
                self._vibration_state['is_vibrating'] = False
                print("震動啟動失敗")
            
            return success
            
        except Exception as e:
            print(f"震動執行失敗: {e}")
            self._vibration_state['is_vibrating'] = False
            return False
    
    def _auto_stop(self):
        """自動停止震動 (定時器回調)"""
        self.stop()
        print("定時震動結束")
    
    def stop(self) -> bool:
        """停止震動"""
        try:
            success = self._write_register(4, 0)  # 停止震動
            
            if success:
                self._vibration_state['is_vibrating'] = False
                self._vibration_state['mode'] = 'stop'
                self._device_status = self.STATUS_IDLE
                self._operation_count += 1
                print("震動已停止")
            
            return success
            
        except Exception as e:
            print(f"停止震動失敗: {e}")
            return False
    
    def emergency_stop(self) -> bool:
        """緊急停止 (強制停止所有動作)"""
        try:
            # 多重停止指令確保可靠性
            success = (
                self._write_register(4, 0) and    # 停止震動
                self._write_register(1, 0) and    # 清除模式
                self._write_register(2, 0)        # 清除強度
            )
            
            if success:
                self._vibration_state['is_vibrating'] = False
                self._vibration_state['mode'] = 'stop'
                self._device_status = self.STATUS_IDLE
                print("緊急停止執行成功")
            
            return success
            
        except Exception as e:
            print(f"緊急停止失敗: {e}")
            return False
    
    def is_connected(self) -> bool:
        """檢查連接狀態"""
        return self._connected
    
    def is_vibrating(self) -> bool:
        """檢查是否正在震動"""
        return self._vibration_state['is_vibrating']
    
    def get_status(self) -> Dict[str, Any]:
        """獲取設備狀態"""
        current_time = time.time()
        uptime = current_time - self._start_time
        
        vibration_duration = 0
        if self._vibration_state['is_vibrating']:
            vibration_duration = current_time - self._vibration_state['vibration_start_time']
        
        return {
            'connected': self._connected,
            'device_status': self._device_status,
            'last_error': self._last_error,
            'connection_attempts': self._connection_attempts,
            'vibration_state': self._vibration_state.copy(),
            'vibration_duration': vibration_duration,
            'statistics': {
                'operation_count': self._operation_count,
                'error_count': self._error_count,
                'reconnection_count': self._reconnection_count,
                'uptime_seconds': int(uptime)
            },
            'config': self.config
        }
    
    def get_vibration_modes(self) -> Dict[str, int]:
        """獲取支援的震動模式"""
        return self.VIBRATION_MODES.copy()
    
    def reset_error_state(self):
        """重置錯誤狀態"""
        self._error_count = 0
        self._connection_attempts = 0
        self._last_error = None
        if self._connected:
            self._device_status = self.STATUS_IDLE
        print("錯誤狀態已重置")
    
    def enable_auto_reconnect(self, enabled: bool = True):
        """啟用/停用自動重連"""
        self._auto_reconnect = enabled
        print(f"自動重連{'啟用' if enabled else '停用'}")
    
    def __del__(self):
        """析構函數 - 確保資源釋放"""
        try:
            self.disconnect()
        except:
            pass


# 使用範例
if __name__ == "__main__":
    # 創建震動盤控制器
    vp = ProductionVibrationPlate()
    
    try:
        # 連接設備
        if vp.connect():
            # 設置背光
            vp.set_backlight_brightness(50)
            vp.set_backlight(True)
            
            # 執行震動 (水平震動, 強度70, 頻率120Hz, 持續5秒)
            vp.vibrate(mode='horizontal', intensity=70, frequency=120, duration=5)
            
            # 等待執行
            time.sleep(6)
            
            # 檢查狀態
            status = vp.get_status()
            print(f"設備狀態: {status}")
            
        else:
            print("設備連接失敗")
            
    except KeyboardInterrupt:
        print("收到中斷信號")
    finally:
        vp.disconnect()
        print("程序結束")