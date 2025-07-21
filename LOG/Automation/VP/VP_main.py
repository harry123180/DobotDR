# -*- coding: utf-8 -*-
"""
VP_main.py - 震動盤Modbus TCP Client主程序
實現震動盤RTU轉TCP橋接，狀態機交握，自動重連
適用於自動化設備對接流程
"""

import sys
import os
import time
import threading
import json
import signal
import logging
from logging.handlers import RotatingFileHandler
from typing import Dict, Any, Optional
from datetime import datetime
from pymodbus.client import ModbusTcpClient
from vibration_plate import VibrationPlate


def setup_logging(module_name: str):
    """統一設置logging配置"""
    # 日誌目錄：執行檔同層目錄下的logs資料夾
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
    os.makedirs(log_dir, exist_ok=True)
    
    # 格式化器
    formatter = logging.Formatter(
        '%(asctime)s [%(levelname)s] %(name)s:%(funcName)s:%(lineno)d - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # 文件處理器 (輪替日誌，保存一週)
    file_handler = RotatingFileHandler(
        os.path.join(log_dir, f'{module_name}.log'),
        maxBytes=10*1024*1024,  # 10MB
        backupCount=7,          # 保留7個檔案
        encoding='utf-8'
    )
    file_handler.setFormatter(formatter)
    
    # 控制台處理器
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    
    # 配置logger
    logger = logging.getLogger(module_name)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger


# 初始化模組logger
logger = setup_logging('VibrationPlate')


class VibrationPlateModbusClient:
    """震動盤Modbus TCP Client - RTU轉TCP橋接模組"""
    
    def __init__(self, config_file="vp_config.json"):
        # 載入配置
        self.config = self.load_config(config_file)
        
        # 核心組件
        self.vibration_plate: Optional[VibrationPlate] = None
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.running = False
        
        # 狀態變數
        self.connected_to_server = False
        self.connected_to_device = False
        self.last_command_id = 0
        self.executing_command = False
        
        # 執行緒控制
        self.main_loop_thread = None
        self.loop_lock = threading.Lock()
        
        # 統計計數
        self.operation_count = 0
        self.error_count = 0
        self.connection_count = 0
        self.start_time = time.time()
        self.loop_cycle_count = 0  # 循環計數，用於控制日誌頻率
        
        # 寄存器映射 (基地址 + 偏移)
        self.base_address = self.config['modbus_mapping']['base_address']
        self.init_register_mapping()
        
        logger.info("VibrationPlateModbusClient初始化完成")
        
    def load_config(self, config_file: str) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "module_id": "震動盤模組",
            "device_connection": {
                "ip": "192.168.1.7",
                "port": 1000,
                "slave_id": 10,
                "timeout": 0.2
            },
            "tcp_server": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 1.0
            },
            "modbus_mapping": {
                "base_address": 300
            },
            "timing": {
                "fast_loop_interval": 0.02,
                "movement_delay": 0.1,
                "command_delay": 0.02
            },
            "device_defaults": {
                "brightness": 28,
                "backlight_enabled": False
            }
        }
        
        try:
            # 取得當前執行檔案的目錄
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(current_dir, config_file)
            
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    # 合併配置
                    default_config.update(loaded_config)
                logger.info(f"已載入配置檔案: {config_path}")
            else:
                # 創建預設配置檔案在執行檔案同層目錄
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                logger.info(f"已創建預設配置檔案: {config_path}")
        except Exception as e:
            logger.error(f"載入配置檔案失敗: {e}", exc_info=True)
            
        return default_config
    
    def init_register_mapping(self):
        """初始化寄存器映射"""
        base = self.base_address
        
        # 狀態寄存器區 (只讀) base+0 ~ base+14
        self.status_registers = {
            'module_status': base + 0,          # 模組狀態
            'device_connection': base + 1,      # 設備連接狀態
            'device_status': base + 2,          # 設備狀態
            'error_code': base + 3,             # 錯誤代碼
            'current_action_low': base + 4,     # 當前動作低位
            'current_action_high': base + 5,    # 當前動作高位
            'target_action_low': base + 6,      # 目標動作低位
            'target_action_high': base + 7,     # 目標動作高位
            'command_status': base + 8,         # 指令執行狀態
            'comm_error_count': base + 9,       # 通訊錯誤計數
            'brightness_status': base + 10,     # 背光亮度狀態
            'backlight_status': base + 11,      # 背光開關狀態
            'vibration_status': base + 12,      # 震動狀態
            'frequency_status': base + 13,      # 頻率狀態
            'timestamp': base + 14              # 時間戳
        }
        
        # 指令寄存器區 (讀寫) base+20 ~ base+25
        self.command_registers = {
            'command_code': base + 20,          # 指令代碼
            'param1': base + 21,                # 參數1 (強度/亮度/動作碼)
            'param2': base + 22,                # 參數2 (頻率)
            'param3': base + 23,                # 參數3 (持續時間)
            'command_id': base + 24,            # 指令ID
            'reserved': base + 25               # 保留
        }
        
        # 所有寄存器
        self.all_registers = {**self.status_registers, **self.command_registers}
        
        logger.info(f"寄存器映射初始化完成 - 基地址: {base}")
        logger.debug(f"狀態寄存器範圍: {base} ~ {base + 14}")
        logger.debug(f"指令寄存器範圍: {base + 20} ~ {base + 25}")
        logger.debug(f"預設背光亮度: {self.config['device_defaults']['brightness']}")
        logger.debug(f"預設背光狀態: {'開啟' if self.config['device_defaults']['backlight_enabled'] else '關閉'}")
    
    def connect_main_server(self) -> bool:
        """連接到主Modbus TCP服務器"""
        try:
            if self.modbus_client:
                self.modbus_client.close()
            
            server_config = self.config['tcp_server']
            logger.info(f"正在連接主服務器: {server_config['host']}:{server_config['port']}")
            
            self.modbus_client = ModbusTcpClient(
                host=server_config['host'],
                port=server_config['port'],
                timeout=server_config['timeout']
            )
            
            if self.modbus_client.connect():
                self.connected_to_server = True
                self.connection_count += 1
                logger.info("連接到主服務器成功")
                
                # 初始化寄存器
                self.init_status_registers()
                return True
            else:
                logger.warning("連接到主服務器失敗")
                return False
                
        except Exception as e:
            logger.error(f"連接主服務器異常: {e}", exc_info=True)
            self.connected_to_server = False
            return False
    
    def connect_device(self) -> bool:
        """連接到震動盤設備"""
        try:
            if self.vibration_plate:
                self.vibration_plate.disconnect()
            
            device_config = self.config['device_connection']
            logger.info(f"正在連接震動盤設備: {device_config['ip']}:{device_config['port']}")
            
            self.vibration_plate = VibrationPlate(
                ip=device_config['ip'],
                port=device_config['port'],
                slave_id=device_config['slave_id'],
                auto_connect=True
            )
            
            if self.vibration_plate.is_connected():
                self.connected_to_device = True
                logger.info("連接到震動盤設備成功")
                
                # 初始化設備 - 使用配置中的預設值
                defaults = self.config['device_defaults']
                self.vibration_plate.set_backlight_brightness(defaults['brightness'])
                self.vibration_plate.set_backlight(defaults['backlight_enabled'])
                
                logger.info(f"設備初始化完成 - 亮度: {defaults['brightness']}, 背光: {'開啟' if defaults['backlight_enabled'] else '關閉'}")
                
                return True
            else:
                logger.warning("連接到震動盤設備失敗，將在主循環中重試")
                return False
                
        except Exception as e:
            logger.error(f"連接震動盤設備異常: {e}", exc_info=True)
            self.connected_to_device = False
            return False
    
    def init_status_registers(self):
        """初始化狀態寄存器"""
        try:
            # 寫入模組基本資訊
            self.write_register('module_status', 1)  # 閒置狀態
            self.write_register('error_code', 0)     # 無錯誤
            self.write_register('command_status', 0) # 空閒
            self.write_register('comm_error_count', self.error_count)
            self.write_register('brightness_status', self.config['device_defaults']['brightness'])
            self.write_register('backlight_status', 1 if self.config['device_defaults']['backlight_enabled'] else 0)
            self.write_register('frequency_status', 100)  # 預設頻率
            
            logger.info("狀態寄存器初始化完成")
        except Exception as e:
            logger.error(f"初始化狀態寄存器失敗: {e}", exc_info=True)
    
    def read_register(self, register_name: str) -> Optional[int]:
        """讀取寄存器"""
        if not self.connected_to_server or register_name not in self.all_registers:
            return None
        
        try:
            address = self.all_registers[register_name]
            result = self.modbus_client.read_holding_registers(
                address, count=1, slave=self.config['tcp_server']['unit_id']
            )
            
            if not result.isError():
                value = result.registers[0]
                #logger.debug(f"讀取寄存器 {register_name}({address}): {value}")
                
                return value
            else:
                logger.warning(f"讀取寄存器 {register_name}({address}) 失敗: {result}")
                return None
                
        except Exception as e:
            logger.error(f"讀取寄存器 {register_name} 異常: {e}", exc_info=True)
            return None
    
    def write_register(self, register_name: str, value: int) -> bool:
        """寫入寄存器"""
        if not self.connected_to_server or register_name not in self.all_registers:
            return False
        
        try:
            address = self.all_registers[register_name]
            result = self.modbus_client.write_register(
                address, value, slave=self.config['tcp_server']['unit_id']
            )
            
            success = not result.isError()
            if success:
                #logger.debug(f"寫入寄存器 {register_name}({address}): {value}")
                pass
            else:
                logger.warning(f"寫入寄存器 {register_name}({address}) 失敗: {result}")
            
            return success
                
        except Exception as e:
            logger.error(f"寫入寄存器 {register_name} 異常: {e}", exc_info=True)
            return False
    
    def execute_command(self, command: int, param1: int, param2: int, param3: int = 0) -> bool:
        """執行指令 - 支援頻率控制"""
        if not self.connected_to_device:
            logger.warning("設備未連接，無法執行指令")
            return False
        
        try:
            success = False
            logger.info(f"開始執行指令: cmd={command}, p1={param1}, p2={param2}, p3={param3}")
            
            if command == 0:  # NOP
                success = True
                
            elif command == 1:  # 設備啟用 (背光開啟)
                success = self.vibration_plate.set_backlight(True)
                
            elif command == 2:  # 設備停用 (背光關閉)
                success = self.vibration_plate.set_backlight(False)
                
            elif command == 3:  # 停止所有動作
                success = self.vibration_plate.stop()
                
            elif command == 4:  # 設定背光亮度
                success = self.vibration_plate.set_backlight_brightness(param1)
                
            elif command == 5:  # 執行動作 (param1=動作碼, param2=強度, param3=頻率)
                actions = ['stop', 'up', 'down', 'left', 'right', 'upleft', 'downleft',
                          'upright', 'downright', 'horizontal', 'vertical', 'spread']
                if 0 <= param1 < len(actions):
                    action = actions[param1]
                    if action == 'stop':
                        success = self.vibration_plate.stop()
                    else:
                        # 使用頻率參數
                        frequency = param3 if param3 > 0 else param2
                        success = self.vibration_plate.execute_action(action, param2, frequency)
                        logger.debug(f"執行動作: {action}, 強度: {param2}, 頻率: {frequency}")
                
            elif command == 6:  # 緊急停止
                success = self.vibration_plate.stop()
                logger.warning("執行緊急停止指令")
                
            elif command == 7:  # 錯誤重置
                self.error_count = 0
                success = True
                logger.info("錯誤計數已重置")
                
            # 震動盤專用指令 (11-30)
            elif 11 <= command <= 30:
                success = self.execute_vp_specific_command(command, param1, param2, param3)
            
            if success:
                self.operation_count += 1
                logger.info(f"指令執行成功: cmd={command}")
            else:
                self.error_count += 1
                logger.error(f"指令執行失敗: cmd={command}")
                
            return success
            
        except Exception as e:
            logger.error(f"執行指令異常: cmd={command}, 錯誤: {e}", exc_info=True)
            self.error_count += 1
            return False
    
    def execute_vp_specific_command(self, command: int, param1: int, param2: int, param3: int) -> bool:
        """執行震動盤專用指令 - 支援頻率控制"""
        try:
            logger.debug(f"執行震動盤專用指令: {command}")
            
            if command == 11:  # 設定動作參數 (含頻率)
                actions = ['up', 'down', 'left', 'right', 'upleft', 'downleft',
                          'upright', 'downright', 'horizontal', 'vertical', 'spread']
                if 0 <= param1 < len(actions):
                    frequency = param3 if param3 > 0 else param2
                    result = self.vibration_plate.set_action_parameters(actions[param1], param2, frequency)
                    logger.debug(f"設定動作參數: {actions[param1]}, 強度: {param2}, 頻率: {frequency}")
                    return result
                    
            elif command == 12:  # 背光切換
                result = self.vibration_plate.set_backlight(bool(param1))
                logger.debug(f"背光切換: {'開啟' if param1 else '關閉'}")
                return result
                
            elif command == 13:  # 執行特定動作並設定參數 (含頻率)
                actions = ['up', 'down', 'left', 'right', 'upleft', 'downleft',
                          'upright', 'downright', 'horizontal', 'vertical', 'spread']
                if 0 <= param1 < len(actions) and param1 > 0:
                    action = actions[param1]
                    frequency = param3 if param3 > 0 else param2
                    result = self.vibration_plate.execute_action(action, param2, frequency)
                    logger.debug(f"執行特定動作: {action}, 強度: {param2}, 頻率: {frequency}")
                    return result
                    
            elif command == 14:  # 設定頻率
                # 更新頻率狀態寄存器
                self.write_register('frequency_status', param1)
                logger.debug(f"設定頻率: {param1}")
                return True
                    
            return False
            
        except Exception as e:
            logger.error(f"震動盤專用指令執行失敗: cmd={command}, 錯誤: {e}", exc_info=True)
            return False
    
    def update_status_registers(self):
        """更新狀態寄存器"""
        try:
            # 更新連接狀態
            self.write_register('device_connection', 1 if self.connected_to_device else 0)
            
            # 更新設備狀態
            if self.connected_to_device:
                vp_status = self.vibration_plate.get_status()
                self.write_register('device_status', 1 if vp_status['connected'] else 0)
                self.write_register('vibration_status', 1 if vp_status['vibration_active'] else 0)
                self.write_register('brightness_status', vp_status.get('backlight_brightness', 0))
                self.write_register('backlight_status', 1 if vp_status.get('backlight_enabled', False) else 0)
            else:
                self.write_register('device_status', 0)
                self.write_register('vibration_status', 0)
            
            # 更新模組狀態
            if self.executing_command:
                self.write_register('module_status', 2)  # 執行中
            elif not self.connected_to_device:
                self.write_register('module_status', 0)  # 離線
            elif self.error_count > 10:
                self.write_register('module_status', 4)  # 錯誤
            else:
                self.write_register('module_status', 1)  # 閒置
            
            # 更新錯誤計數和時間戳
            self.write_register('comm_error_count', self.error_count)
            self.write_register('timestamp', int(time.time()) & 0xFFFF)
            
            # 每1000次循環輸出狀態統計
            if self.loop_cycle_count % 1000 == 0:
                logger.debug(f"狀態統計 - 操作次數: {self.operation_count}, 錯誤次數: {self.error_count}, 連接次數: {self.connection_count}")
            
        except Exception as e:
            logger.error(f"更新狀態寄存器失敗: {e}", exc_info=True)
    
    def process_commands(self):
        """處理指令 (狀態機交握)"""
        try:
            # 讀取新指令ID
            new_command_id = self.read_register('command_id')
            if new_command_id is None or new_command_id == self.last_command_id:
                return
            
            # 檢測到新指令
            command_code = self.read_register('command_code')
            param1 = self.read_register('param1')
            param2 = self.read_register('param2')
            param3 = self.read_register('param3')
            
            if command_code is None:
                return
            
            logger.info(f"收到新指令: ID={new_command_id}, CMD={command_code}, P1={param1}, P2={param2}, P3={param3}")
            
            # 設置執行狀態
            self.executing_command = True
            self.write_register('command_status', 1)  # 執行中
            
            # 執行指令
            success = self.execute_command(command_code, param1 or 0, param2 or 0, param3 or 0)
            
            # 更新結果
            if success:
                self.write_register('error_code', 0)
                logger.debug("指令執行完成，狀態正常")
            else:
                self.write_register('error_code', 1)  # 執行失敗
                logger.warning("指令執行失敗")
            
            # 清除執行狀態
            self.executing_command = False
            self.write_register('command_status', 0)  # 空閒
            
            # 清除指令寄存器
            self.write_register('command_code', 0)
            self.write_register('param1', 0)
            self.write_register('param2', 0)
            self.write_register('param3', 0)
            self.write_register('command_id', 0)
            
            # 更新指令ID
            self.last_command_id = new_command_id
            
        except Exception as e:
            logger.error(f"處理指令失敗: {e}", exc_info=True)
            self.executing_command = False
            self.write_register('command_status', 0)
            self.error_count += 1
    
    def main_loop(self):
        """主循環"""
        loop_interval = self.config['timing']['fast_loop_interval']
        logger.info("主循環執行緒已啟動")
        
        while self.running:
            try:
                with self.loop_lock:
                    self.loop_cycle_count += 1
                    
                    # 檢查連接狀態
                    if not self.connected_to_server:
                        logger.info("開始重連主服務器")
                        if not self.connect_main_server():
                            time.sleep(1)
                            continue
                    
                    if not self.connected_to_device:
                        logger.info("開始重連震動盤設備")
                        if not self.connect_device():
                            time.sleep(1)
                            continue
                    
                    # 處理指令
                    self.process_commands()
                    
                    # 更新狀態
                    self.update_status_registers()
                
                time.sleep(loop_interval)
                
            except Exception as e:
                logger.error(f"主循環異常: {e}", exc_info=True)
                self.error_count += 1
                time.sleep(0.5)
        
        logger.info("主循環執行緒已停止")
    
    def start(self) -> bool:
        """啟動模組"""
        if self.running:
            logger.warning("模組已在運行中")
            return False
        
        try:
            logger.info("開始啟動震動盤模組")
            
            # 自動連接服務器和設備
            logger.info("正在自動連接主服務器...")
            if not self.connect_main_server():
                logger.error("無法連接到主服務器")
                return False
            
            logger.info("正在自動連接震動盤設備...")
            if not self.connect_device():
                logger.warning("無法連接到震動盤設備，將在主循環中重試")
            
            # 啟動主循環
            self.running = True
            self.main_loop_thread = threading.Thread(target=self.main_loop, daemon=True)
            self.main_loop_thread.start()
            
            logger.info("震動盤模組啟動成功")
            return True
            
        except Exception as e:
            logger.error(f"啟動模組失敗: {e}", exc_info=True)
            return False
    
    def stop(self):
        """停止模組"""
        logger.info("正在停止震動盤模組...")
        
        self.running = False
        
        # 停止震動盤
        if self.vibration_plate:
            try:
                self.vibration_plate.stop()
                self.vibration_plate.disconnect()
                logger.info("震動盤設備已停止並斷開連接")
            except Exception as e:
                logger.error(f"停止震動盤設備失敗: {e}", exc_info=True)
        
        # 更新狀態為離線
        if self.connected_to_server:
            try:
                self.write_register('module_status', 0)  # 離線
                self.write_register('device_connection', 0)
                self.write_register('command_status', 0)
                logger.debug("狀態寄存器已更新為離線")
            except Exception as e:
                logger.error(f"更新離線狀態失敗: {e}", exc_info=True)
        
        # 關閉連接
        if self.modbus_client:
            try:
                self.modbus_client.close()
                logger.info("Modbus TCP連接已關閉")
            except Exception as e:
                logger.error(f"關閉Modbus連接失敗: {e}", exc_info=True)
        
        # 等待線程結束
        if self.main_loop_thread and self.main_loop_thread.is_alive():
            self.main_loop_thread.join(timeout=2)
            if self.main_loop_thread.is_alive():
                logger.warning("主循環執行緒未能在預期時間內停止")
            else:
                logger.info("主循環執行緒已正常停止")
        
        logger.info("震動盤模組已停止")
    
    def get_status(self) -> Dict[str, Any]:
        """獲取模組狀態"""
        uptime = time.time() - self.start_time
        
        status = {
            'module_id': self.config['module_id'],
            'running': self.running,
            'connected_to_server': self.connected_to_server,
            'connected_to_device': self.connected_to_device,
            'executing_command': self.executing_command,
            'operation_count': self.operation_count,
            'error_count': self.error_count,
            'connection_count': self.connection_count,
            'uptime_seconds': int(uptime),
            'last_command_id': self.last_command_id,
            'loop_cycle_count': self.loop_cycle_count,
            'config': self.config,
            'register_mapping': {
                'base_address': self.base_address,
                'status_registers': self.status_registers,
                'command_registers': self.command_registers
            }
        }
        
        logger.debug(f"模組狀態查詢 - 運行時間: {int(uptime)}秒, 操作次數: {self.operation_count}")
        return status


def main():
    """主函數"""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    logger.info("震動盤Modbus TCP Client啟動中...")
    logger.info(f"執行目錄: {current_dir}")
    
    # 創建模組實例
    vp_client = VibrationPlateModbusClient()
    
    # 信號處理
    def signal_handler(sig, frame):
        logger.info("收到停止信號")
        vp_client.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        # 啟動模組 (自動連接)
        if vp_client.start():
            logger.info(f"震動盤模組運行中 - 基地址: {vp_client.base_address}")
            logger.info(f"狀態寄存器範圍: {vp_client.base_address} ~ {vp_client.base_address + 14}")
            logger.info(f"指令寄存器範圍: {vp_client.base_address + 20} ~ {vp_client.base_address + 25}")
            logger.info("按 Ctrl+C 停止程序")
            
            # 保持運行
            while vp_client.running:
                time.sleep(1)
        else:
            logger.critical("模組啟動失敗")
            
    except KeyboardInterrupt:
        logger.info("收到中斷信號")
    except Exception as e:
        logger.critical(f"運行異常: {e}", exc_info=True)
    finally:
        vp_client.stop()


if __name__ == '__main__':
    main()