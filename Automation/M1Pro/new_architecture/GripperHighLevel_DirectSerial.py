#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GripperHighLevel_DirectSerial.py - PGC夾爪直接串口通信版本
取代原有的TCP架構，直接通過串口與PGC夾爪通信
移除對Gripper.py的依賴，大幅簡化架構並提升性能
"""

import time
import threading
import os
import json
import serial
from typing import Optional, Dict, Any
from enum import IntEnum
import logging
from logging.handlers import RotatingFileHandler

# 導入Modbus RTU Client
try:
    from pymodbus.client import ModbusSerialClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError as e:
    print(f"Modbus Client模組導入失敗: {e}")
    MODBUS_AVAILABLE = False


# ==================== 夾爪指令枚舉 ====================
class GripperCommand(IntEnum):
    """夾爪指令枚舉"""
    NOP = 0             # 無操作
    INITIALIZE = 1      # 初始化/回零
    STOP = 2           # 停止
    MOVE_ABS = 3       # 絕對位置移動
    SET_FORCE = 5      # 設定力道
    SET_SPEED = 6      # 設定速度
    QUICK_OPEN = 7     # 快速開啟
    QUICK_CLOSE = 8    # 快速關閉


# ==================== 夾爪狀態枚舉 ====================
class GripperStatus(IntEnum):
    """夾爪狀態枚舉"""
    MOVING = 0         # 運動中
    REACHED = 1        # 到達位置
    GRIPPED = 2        # 夾住物體
    DROPPED = 3        # 掉落


# ==================== 直接串口通信版夾爪API類 ====================
class GripperHighLevelAPI:
    """
    PGC夾爪高層API - 直接串口通信版本
    
    主要特點：
    1. 直接通過串口與PGC夾爪通信，移除TCP中介層
    2. 大幅降低CPU負載和系統複雜度
    3. 保持與原有API的完全兼容性
    4. 優化的性能和響應速度
    5. 內建配置管理和日誌系統
    """
    
    def __init__(self, config_file: str = "gripper_config.json", auto_initialize: bool = True):
        """
        初始化直接串口通信版夾爪API
        
        Args:
            config_file: 配置檔案路徑
            auto_initialize: 是否自動初始化夾爪
        """
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(self.current_dir, config_file)
        self.config = self.load_config()
        
        # 設置日誌
        self.logger = self._setup_logging()
        
        # 串口連接
        self.serial_client: Optional[ModbusSerialClient] = None
        self.connected = False
        self.auto_initialize = auto_initialize
        
        # 高性能線程鎖
        self.serial_lock = threading.RLock()
        
        # PGC夾爪配置
        self.pgc_unit_id = self.config["grippers"]["PGC"]["unit_id"]
        
        # 狀態快取
        self.status_cache = {}
        self.cache_timeout = 0.05  # 50ms快取有效期
        
        # 初始化狀態
        self.initialized = False
        
        # 性能統計
        self.performance_stats = {
            'command_count': 0,
            'avg_response_time': 0.0,
            'max_response_time': 0.0,
            'success_rate': 1.0,
            'error_count': 0
        }
        
        self.logger.info("PGC夾爪API初始化 - 直接串口通信模式")
        
        # 自動連接
        if self.connect():
            self.logger.info("PGC夾爪連接成功")
            if self.auto_initialize:
                self.logger.info("開始自動初始化PGC夾爪")
                if self.initialize(timeout=3.0):
                    self.initialized = True
                    self.logger.info("PGC夾爪初始化成功")
                else:
                    self.logger.warning("PGC夾爪初始化失敗，但連接正常")
        else:
            self.logger.error("PGC夾爪連接失敗")
    
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "module_id": "PGC夾爪直接串口模式",
            "rtu_connection": {
                "port": "COM7",
                "baudrate": 115200,
                "parity": "N",
                "stopbits": 1,
                "timeout": 0.5
            },
            "grippers": {
                "PGC": {
                    "unit_id": 6,
                    "enabled": True
                }
            },
            "timing": {
                "command_delay": 0.01,
                "status_cache_timeout": 0.05,
                "operation_timeout": 5.0,
                "quick_timeout": 2.0
            },
            "performance": {
                "enable_cache": True,
                "max_retries": 3,
                "retry_delay": 0.1
            }
        }
        
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    # 合併配置，確保必要的鍵存在
                    for key, value in default_config.items():
                        if key not in loaded_config:
                            loaded_config[key] = value
                        elif isinstance(value, dict):
                            for subkey, subvalue in value.items():
                                if subkey not in loaded_config[key]:
                                    loaded_config[key][subkey] = subvalue
                    return loaded_config
            except Exception as e:
                print(f"配置檔案讀取錯誤: {e}")
                return default_config
        else:
            # 創建預設配置檔案
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(default_config, f, indent=2, ensure_ascii=False)
            return default_config
    
    def _setup_logging(self):
        """設置日誌系統"""
        log_dir = os.path.join(self.current_dir, 'logs')
        os.makedirs(log_dir, exist_ok=True)
        
        formatter = logging.Formatter(
            '%(asctime)s [%(levelname)s] GripperDirect:%(funcName)s:%(lineno)d - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        file_handler = RotatingFileHandler(
            os.path.join(log_dir, 'gripper_direct_serial.log'),
            maxBytes=5*1024*1024,
            backupCount=3,
            encoding='utf-8'
        )
        file_handler.setFormatter(formatter)
        
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        
        logger = logging.getLogger("GripperDirectSerial")
        logger.setLevel(logging.INFO)
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
        
        return logger
    
    def connect(self) -> bool:
        """連接到PGC夾爪串口"""
        if not MODBUS_AVAILABLE:
            self.logger.error("Modbus Client不可用")
            return False
        
        with self.serial_lock:
            try:
                if self.serial_client:
                    self.serial_client.close()
                
                rtu_config = self.config["rtu_connection"]
                self.logger.info(f"正在連接PGC夾爪串口: {rtu_config['port']}")
                
                self.serial_client = ModbusSerialClient(
                    port=rtu_config["port"],
                    baudrate=rtu_config["baudrate"],
                    parity=rtu_config["parity"],
                    stopbits=rtu_config["stopbits"],
                    timeout=rtu_config["timeout"]
                )
                
                if self.serial_client.connect():
                    self.connected = True
                    
                    # 測試連接
                    test_result = self.serial_client.read_holding_registers(
                        address=0x0200, count=1, slave=self.pgc_unit_id
                    )
                    
                    if test_result and not test_result.isError():
                        self.logger.info(f"PGC夾爪串口連接成功: {rtu_config['port']}")
                        return True
                    else:
                        self.logger.error("PGC夾爪連接測試失敗")
                        self.connected = False
                        return False
                else:
                    self.logger.error(f"串口連接失敗: {rtu_config['port']}")
                    self.connected = False
                    return False
                    
            except Exception as e:
                self.logger.error(f"串口連接異常: {e}", exc_info=True)
                self.connected = False
                return False
    
    def disconnect(self):
        """斷開串口連接"""
        with self.serial_lock:
            if self.serial_client and self.connected:
                try:
                    self.serial_client.close()
                    self.logger.info("PGC夾爪串口連接已斷開")
                except Exception as e:
                    self.logger.error(f"斷開串口連接時發生異常: {e}")
            
            self.connected = False
            self.serial_client = None
    
    def _write_register(self, address: int, value: int, retry_count: int = 0) -> bool:
        """寫入PGC夾爪寄存器"""
        if not self.connected or not self.serial_client:
            if retry_count == 0:  # 嘗試重新連接
                self.logger.warning("連接中斷，嘗試重新連接")
                if self.connect():
                    return self._write_register(address, value, retry_count + 1)
            self.logger.error("PGC夾爪未連接")
            return False
        
        with self.serial_lock:
            try:
                max_retries = self.config["performance"]["max_retries"]
                
                for attempt in range(max_retries):
                    result = self.serial_client.write_register(
                        address=address, 
                        value=value, 
                        slave=self.pgc_unit_id
                    )
                    
                    if result and not result.isError():
                        return True
                    else:
                        if attempt < max_retries - 1:
                            time.sleep(self.config["performance"]["retry_delay"])
                
                self.logger.error(f"PGC寄存器寫入失敗: 地址0x{address:04X} = {value}")
                self.performance_stats['error_count'] += 1
                return False
                
            except Exception as e:
                self.logger.error(f"PGC寄存器寫入異常: 地址0x{address:04X} = {value}, 錯誤: {e}")
                self.performance_stats['error_count'] += 1
                return False
    
    def _read_register(self, address: int, count: int = 1, use_cache: bool = True) -> Optional[list]:
        """讀取PGC夾爪寄存器"""
        if not self.connected or not self.serial_client:
            self.logger.error("PGC夾爪未連接")
            return None
        
        current_time = time.time()
        cache_key = f"{address}_{count}"
        
        # 檢查快取
        if use_cache and self.config["performance"]["enable_cache"] and cache_key in self.status_cache:
            cached_data = self.status_cache[cache_key]
            if current_time - cached_data['timestamp'] < self.config["timing"]["status_cache_timeout"]:
                return cached_data['value']
        
        with self.serial_lock:
            try:
                result = self.serial_client.read_holding_registers(
                    address=address, 
                    count=count, 
                    slave=self.pgc_unit_id
                )
                
                if result and not result.isError() and len(result.registers) == count:
                    values = result.registers
                    
                    # 更新快取
                    if use_cache and self.config["performance"]["enable_cache"]:
                        self.status_cache[cache_key] = {
                            'value': values,
                            'timestamp': current_time
                        }
                    
                    return values
                else:
                    self.logger.error(f"PGC寄存器讀取失敗: 地址0x{address:04X}, 數量{count}")
                    return None
                    
            except Exception as e:
                self.logger.error(f"PGC寄存器讀取異常: 地址0x{address:04X}, 錯誤: {e}")
                return None
    
    # ==================== PGC夾爪控制方法 ====================
    
    def initialize(self, wait_completion: bool = True, timeout: float = None) -> bool:
        """初始化PGC夾爪"""
        self.logger.info("開始初始化PGC夾爪")
        
        start_time = time.time()
        actual_timeout = timeout if timeout is not None else self.config["timing"]["operation_timeout"]
        
        try:
            # 發送初始化指令
            if not self._write_register(0x0100, 0x01):
                self.logger.error("PGC初始化指令發送失敗")
                return False
            
            if wait_completion:
                success = self._wait_for_completion(actual_timeout)
                response_time = time.time() - start_time
                self._update_performance_stats(response_time, success)
                
                if success:
                    self.initialized = True
                    self.logger.info(f"PGC夾爪初始化成功，耗時: {response_time:.3f}秒")
                else:
                    self.logger.error(f"PGC夾爪初始化超時，耗時: {response_time:.3f}秒")
                
                return success
            
            return True
            
        except Exception as e:
            self.logger.error(f"PGC夾爪初始化異常: {e}", exc_info=True)
            return False
    
    def quick_close(self) -> bool:
        """PGC夾爪快速關閉"""
        self.logger.info("PGC夾爪快速關閉")
        
        start_time = time.time()
        
        try:
            # 設定到關閉位置 (通常為較小的數值)
            success = self._write_register(0x0103, 100)  # 關閉位置
            
            if success:
                # 等待動作完成
                success = self._wait_for_completion(self.config["timing"]["quick_timeout"])
            
            response_time = time.time() - start_time
            self._update_performance_stats(response_time, success)
            
            if success:
                self.logger.info(f"PGC快速關閉成功，耗時: {response_time:.3f}秒")
            else:
                self.logger.error(f"PGC快速關閉失敗，耗時: {response_time:.3f}秒")
            
            return success
            
        except Exception as e:
            self.logger.error(f"PGC快速關閉異常: {e}", exc_info=True)
            return False
    
    def quick_open(self) -> bool:
        """PGC夾爪快速開啟"""
        self.logger.info("PGC夾爪快速開啟")
        
        start_time = time.time()
        
        try:
            # 設定到開啟位置 (通常為較大的數值)
            success = self._write_register(0x0103, 900)  # 開啟位置
            
            if success:
                # 等待動作完成
                success = self._wait_for_completion(self.config["timing"]["quick_timeout"])
            
            response_time = time.time() - start_time
            self._update_performance_stats(response_time, success)
            
            if success:
                self.logger.info(f"PGC快速開啟成功，耗時: {response_time:.3f}秒")
            else:
                self.logger.error(f"PGC快速開啟失敗，耗時: {response_time:.3f}秒")
            
            return success
            
        except Exception as e:
            self.logger.error(f"PGC快速開啟異常: {e}", exc_info=True)
            return False
    
    def smart_release(self, release_position: int = 470) -> bool:
        """PGC夾爪智能釋放"""
        self.logger.info(f"PGC智能釋放到位置: {release_position}")
        
        start_time = time.time()
        
        try:
            success = self._write_register(0x0103, release_position)
            
            if success:
                success = self._wait_for_completion(self.config["timing"]["quick_timeout"])
            
            response_time = time.time() - start_time
            self._update_performance_stats(response_time, success)
            
            if success:
                self.logger.info(f"PGC智能釋放成功，耗時: {response_time:.3f}秒")
            else:
                self.logger.error(f"PGC智能釋放失敗，耗時: {response_time:.3f}秒")
            
            return success
            
        except Exception as e:
            self.logger.error(f"PGC智能釋放異常: {e}", exc_info=True)
            return False
    
    def smart_grip(self, target_position: int = 420, max_attempts: int = 3) -> bool:
        """PGC夾爪智能夾取"""
        self.logger.info(f"PGC智能夾取到位置: {target_position}, 最大嘗試次數: {max_attempts}")
        
        for attempt in range(max_attempts):
            try:
                initial_pos = self.get_current_position()
                
                if not self._write_register(0x0103, target_position):
                    continue
                
                if not self._wait_for_completion(self.config["timing"]["operation_timeout"]):
                    continue
                
                final_pos = self.get_current_position()
                if final_pos is None:
                    continue
                
                position_diff = abs(target_position - final_pos)
                
                if position_diff > 20:  # 位置差異超過20，表示夾住物體
                    self.logger.info(f"PGC智能夾取成功 (嘗試{attempt + 1}/{max_attempts})")
                    return True
                    
            except Exception as e:
                self.logger.error(f"第{attempt + 1}次嘗試異常: {e}")
                
        self.logger.error("PGC智能夾取失敗")
        return False
    
    def move_to_and_wait(self, position: int) -> bool:
        """移動到指定位置並等待完成"""
        self.logger.info(f"PGC移動到位置: {position}")
        
        start_time = time.time()
        
        try:
            success = self._write_register(0x0103, position)
            
            if success:
                success = self._wait_for_completion(self.config["timing"]["operation_timeout"])
            
            response_time = time.time() - start_time
            self._update_performance_stats(response_time, success)
            
            return success
            
        except Exception as e:
            self.logger.error(f"PGC移動異常: {e}", exc_info=True)
            return False
    
    def set_force(self, force: int) -> bool:
        """設定PGC夾爪力道"""
        if not 1 <= force <= 100:
            self.logger.error(f"力道值超出範圍: {force} (應為1-100)")
            return False
        
        self.logger.info(f"設定PGC夾爪力道: {force}")
        return self._write_register(0x0101, force)
    
    def set_speed(self, speed: int) -> bool:
        """設定PGC夾爪速度"""
        if not 1 <= speed <= 100:
            self.logger.error(f"速度值超出範圍: {speed} (應為1-100)")
            return False
        
        self.logger.info(f"設定PGC夾爪速度: {speed}")
        return self._write_register(0x0104, speed)
    
    def stop(self) -> bool:
        """停止PGC夾爪動作"""
        self.logger.info("停止PGC夾爪動作")
        return self._write_register(0x0100, 0)
    
    # ==================== 狀態讀取方法 ====================
    
    def get_current_position(self) -> Optional[int]:
        """取得PGC夾爪當前位置"""
        values = self._read_register(0x0202, 1, use_cache=True)
        return values[0] if values else None
    
    def get_grip_status(self) -> Optional[int]:
        """取得PGC夾爪夾持狀態"""
        values = self._read_register(0x0201, 1, use_cache=True)
        return values[0] if values else None
    
    def get_init_status(self) -> Optional[int]:
        """取得PGC夾爪初始化狀態"""
        values = self._read_register(0x0200, 1, use_cache=True)
        return values[0] if values else None
    
    def get_complete_status(self) -> Dict[str, Any]:
        """取得PGC夾爪完整狀態"""
        try:
            # 批量讀取狀態寄存器
            values = self._read_register(0x0200, 3, use_cache=False)
            
            if values and len(values) == 3:
                return {
                    'init_status': values[0],
                    'grip_status': values[1],
                    'current_position': values[2],
                    'connected': True,
                    'timestamp': time.time()
                }
            else:
                return {
                    'init_status': 0,
                    'grip_status': 0,
                    'current_position': 0,
                    'connected': False,
                    'timestamp': time.time()
                }
                
        except Exception as e:
            self.logger.error(f"讀取完整狀態異常: {e}")
            return {
                'init_status': 0,
                'grip_status': 0,
                'current_position': 0,
                'connected': False,
                'timestamp': time.time()
            }
    
    def is_initialized(self) -> bool:
        """檢查PGC夾爪是否已初始化"""
        return self.initialized and self.get_init_status() == 1
    
    def is_connected(self) -> bool:
        """檢查PGC夾爪是否已連接"""
        return self.connected
    
    # ==================== 內部輔助方法 ====================
    
    def _wait_for_completion(self, timeout: float) -> bool:
        """等待PGC夾爪動作完成"""
        start_time = time.time()
        
        # 適應性輪詢策略
        poll_intervals = [0.01, 0.02, 0.05, 0.1]
        poll_index = 0
        
        while time.time() - start_time < timeout:
            elapsed = time.time() - start_time
            
            # 動態調整輪詢間隔
            if elapsed > 0.3 and poll_index == 0:
                poll_index = 1
            elif elapsed > 0.8 and poll_index == 1:
                poll_index = 2
            elif elapsed > 2.0 and poll_index == 2:
                poll_index = 3
            
            time.sleep(poll_intervals[poll_index])
            
            # 檢查夾爪狀態
            status = self.get_grip_status()
            if status in [GripperStatus.REACHED, GripperStatus.GRIPPED]:
                elapsed_time = time.time() - start_time
                self.logger.debug(f"PGC動作完成，耗時: {elapsed_time:.3f}秒，狀態: {status}")
                return True
            elif status == GripperStatus.DROPPED:
                elapsed_time = time.time() - start_time
                self.logger.warning(f"PGC動作失敗(掉落)，耗時: {elapsed_time:.3f}秒")
                return True
        
        self.logger.warning(f"PGC動作完成等待超時 (超時時間: {timeout}秒)")
        return False
    
    def _update_performance_stats(self, response_time: float, success: bool):
        """更新性能統計"""
        self.performance_stats['command_count'] += 1
        
        self.performance_stats['avg_response_time'] = (
            self.performance_stats['avg_response_time'] * 0.9 + response_time * 0.1
        )
        
        if response_time > self.performance_stats['max_response_time']:
            self.performance_stats['max_response_time'] = response_time
        
        # 成功率統計
        if hasattr(self, '_success_history'):
            self._success_history.append(success)
            if len(self._success_history) > 100:
                self._success_history.pop(0)
            self.performance_stats['success_rate'] = sum(self._success_history) / len(self._success_history)
        else:
            self._success_history = [success]
            self.performance_stats['success_rate'] = 1.0 if success else 0.0
    
    def get_performance_stats(self) -> Dict[str, Any]:
        """取得性能統計"""
        return {
            **self.performance_stats,
            'cache_size': len(self.status_cache),
            'initialized': self.initialized,
            'connected': self.connected
        }
    
    def clear_cache(self):
        """清空狀態快取"""
        with self.serial_lock:
            self.status_cache.clear()
            self.logger.debug("狀態快取已清空")


# ==================== 使用範例和測試 ====================
def test_gripper_direct():
    """測試PGC夾爪直接串口通信"""
    print("=== PGC夾爪直接串口通信測試 ===")
    
    gripper = GripperHighLevelAPI(auto_initialize=False)
    
    try:
        if gripper.is_connected():
            print("✓ PGC夾爪連接成功")
            
            # 初始化測試
            print("\n--- 初始化測試 ---")
            if gripper.initialize():
                print("✓ PGC夾爪初始化成功")
            else:
                print("✗ PGC夾爪初始化失敗")
                return
            
            # 快速關閉測試
            print("\n--- 快速關閉測試 ---")
            start_time = time.time()
            if gripper.quick_close():
                elapsed = time.time() - start_time
                print(f"✓ 快速關閉成功，耗時: {elapsed:.3f}秒")
            else:
                print("✗ 快速關閉失敗")
            
            time.sleep(1)
            
            # 智能釋放測試
            print("\n--- 智能釋放測試 ---")
            start_time = time.time()
            if gripper.smart_release(470):
                elapsed = time.time() - start_time
                print(f"✓ 智能釋放成功，耗時: {elapsed:.3f}秒")
            else:
                print("✗ 智能釋放失敗")
            
            # 狀態讀取測試
            print("\n--- 狀態讀取測試 ---")
            status = gripper.get_complete_status()
            print(f"完整狀態: {status}")
            
            # 性能統計
            print("\n--- 性能統計 ---")
            stats = gripper.get_performance_stats()
            print(f"平均響應時間: {stats['avg_response_time']:.3f}秒")
            print(f"最大響應時間: {stats['max_response_time']:.3f}秒")
            print(f"成功率: {stats['success_rate']:.1%}")
            print(f"指令總數: {stats['command_count']}")
            print(f"錯誤計數: {stats['error_count']}")
            
        else:
            print("✗ PGC夾爪連接失敗")
            
    except KeyboardInterrupt:
        print("\n程序中斷")
    except Exception as e:
        print(f"測試異常: {e}")
    finally:
        gripper.disconnect()
        print("PGC夾爪已斷開連接")


if __name__ == "__main__":
    test_gripper_direct()