#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_main_app.py - Dobot主控制器Web介面 (狀態機交握版本) - 增強版
提供Web UI來控制和監控Dobot_main.py主控制器
完全使用狀態機交握協議進行控制
新增功能：
1. Angle模組整合 (基地址700)
2. Flow執行時間統計
3. Angle校正狀態顯示
4. 寄存器地址顯示
"""

import os
import json
import time
import threading
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from pymodbus.client.tcp import ModbusTcpClient

# 配置檔案
CONFIG_FILE = "dobot_main_app_config.json"

# Modbus寄存器映射 (狀態機交握版本)
class DobotRegisters:
    # 狀態寄存器 (400-419) - 只讀
    STATUS_REGISTER = 400     # 主狀態寄存器 (bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized)
    ROBOT_STATE = 401         # 機械臂狀態
    CURRENT_FLOW = 402        # 當前流程ID
    FLOW_PROGRESS = 403       # 流程執行進度
    ERROR_CODE = 404          # 錯誤代碼
    ROBOT_MODE = 405          # 機械臂模式
    POS_X = 406              # 當前X座標
    POS_Y = 407              # 當前Y座標
    POS_Z = 408              # 當前Z座標
    POS_R = 409              # 當前R座標
    JOINT_J1 = 410           # J1角度
    JOINT_J2 = 411           # J2角度
    JOINT_J3 = 412           # J3角度
    JOINT_J4 = 413           # J4角度
    DI_STATUS = 414          # 數位輸入狀態
    DO_STATUS = 415          # 數位輸出狀態
    OP_COUNTER = 416         # 操作計數器
    ERR_COUNTER = 417        # 錯誤計數器
    RUN_TIME = 418           # 運行時間(分鐘)
    GLOBAL_SPEED = 419       # 全局速度設定值
    
    # 控制寄存器 (440-449) - 讀寫
    VP_CONTROL = 440         # VP視覺取料控制
    UNLOAD_CONTROL = 441     # 出料控制
    CLEAR_ALARM = 442        # 清除警報控制
    EMERGENCY_STOP = 443     # 緊急停止控制
    MANUAL_COMMAND = 444     # 手動指令 (Web端使用)
    SPEED_COMMAND = 445      # 速度控制指令
    SPEED_VALUE = 446        # 速度數值
    SPEED_CMD_ID = 447       # 速度指令ID

# Angle模組寄存器映射 (基地址700)
class AngleRegisters:
    # 狀態寄存器 (700-714) - 只讀
    STATUS_REGISTER = 700    # Angle狀態寄存器 (bit0=Ready, bit1=Running, bit2=Alarm)
    MODBUS_CONNECTION = 701  # Modbus連接狀態
    MOTOR_CONNECTION = 702   # 馬達連接狀態
    ERROR_CODE = 703         # 錯誤代碼
    
    # 檢測結果寄存器 (720-739) - 只讀
    SUCCESS_FLAG = 720       # 成功標誌
    ANGLE_HIGH = 721         # 原始角度高位
    ANGLE_LOW = 722          # 原始角度低位
    DIFF_HIGH = 723          # 角度差高位
    DIFF_LOW = 724           # 角度差低位
    MOTOR_POS_HIGH = 725     # 馬達位置高位
    MOTOR_POS_LOW = 726      # 馬達位置低位
    
    # 控制指令寄存器 (740) - 讀寫
    CONTROL_COMMAND = 740    # 控制指令

# 狀態映射
ROBOT_STATES = {
    0: "空閒", 1: "運行中", 2: "暫停", 3: "錯誤", 4: "緊急停止"
}

FLOW_TYPES = {
    0: "無流程", 1: "VP視覺抓取(FIFO)", 2: "出料流程", 3: "完整加工流程"
}

# 狀態機交握指令
HANDSHAKE_COMMANDS = {
    "vp_pickup": 1,          # VP視覺取料
    "unload": 2,             # 出料流程
    "clear_alarm": 1,        # 清除警報
    "emergency_stop": 1,     # 緊急停止
    "manual_flow1": 1,       # 手動Flow1
    "manual_flow2": 2,       # 手動Flow2
    "set_speed": 1,          # 設定速度
    "angle_correction": 1    # 角度校正
}

class DobotHandshakeController:
    """Dobot狀態機交握控制器 - 增強版"""
    
    def __init__(self, config_file: str = CONFIG_FILE):
        self.config_file = config_file
        self.config = self._load_config()
        self.modbus_client = None
        self.is_connected = False
        self.monitoring = False
        self.monitor_thread = None
        self.speed_cmd_id_counter = 1
        
        # Flow執行時間統計
        self.flow_timers = {
            1: {'start_time': None, 'duration': 0.0, 'is_running': False},
            2: {'start_time': None, 'duration': 0.0, 'is_running': False}
        }
        
        # Angle模組狀態快取
        self.angle_status = {
            'ready': False,
            'running': False,
            'alarm': False,
            'modbus_connected': False,
            'motor_connected': False,
            'last_angle': None,
            'last_diff': None,
            'last_motor_pos': None,
            'last_success': False
        }
        
    def _load_config(self) -> dict:
        """載入配置檔案"""
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), self.config_file)
        
        default_config = {
            "modbus": {
                "server_ip": "127.0.0.1",
                "server_port": 502,
                "timeout": 3.0
            },
            "web_server": {
                "host": "0.0.0.0",
                "port": 5009,
                "debug": False
            },
            "monitoring": {
                "refresh_interval": 1.0,
                "auto_start": True
            },
            "handshake_settings": {
                "command_timeout": 5.0,
                "retry_count": 3,
                "auto_clear_commands": True
            },
            "speed_control": {
                "min_speed": 1,
                "max_speed": 100,
                "default_speed": 50
            },
            "angle_module": {
                "base_address": 700,
                "enabled": True,
                "web_api_port": 5087
            }
        }
        
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    user_config = json.load(f)
                    self._deep_update(default_config, user_config)
            except Exception as e:
                print(f"載入配置檔案失敗，使用預設配置: {e}")
        else:
            try:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                print(f"創建預設配置檔案: {config_path}")
            except Exception as e:
                print(f"創建配置檔案失敗: {e}")
                
        return default_config
    
    def _deep_update(self, base_dict: dict, update_dict: dict):
        """深度更新字典"""
        for key, value in update_dict.items():
            if key in base_dict and isinstance(base_dict[key], dict) and isinstance(value, dict):
                self._deep_update(base_dict[key], value)
            else:
                base_dict[key] = value
    
    def connect_modbus(self) -> bool:
        """連接Modbus服務器"""
        try:
            if self.modbus_client:
                self.modbus_client.close()
                
            self.modbus_client = ModbusTcpClient(
                self.config["modbus"]["server_ip"],
                port=self.config["modbus"]["server_port"]
            )
            
            if self.modbus_client.connect():
                self.is_connected = True
                print(f"Modbus連接成功: {self.config['modbus']['server_ip']}:{self.config['modbus']['server_port']}")
                return True
            else:
                self.is_connected = False
                print("Modbus連接失敗")
                return False
                
        except Exception as e:
            self.is_connected = False
            print(f"Modbus連接異常: {e}")
            return False
    
    def read_register(self, address: int) -> int:
        """讀取單個寄存器"""
        try:
            if not self.is_connected:
                return None
                
            result = self.modbus_client.read_holding_registers(address, count=1)
            if hasattr(result, 'registers') and len(result.registers) > 0:
                return result.registers[0]
            return None
        except Exception as e:
            print(f"讀取寄存器{address}失敗: {e}")
            return None
    
    def write_register(self, address: int, value: int) -> bool:
        """寫入單個寄存器"""
        try:
            if not self.is_connected:
                return False
                
            result = self.modbus_client.write_register(address, value)
            return not (hasattr(result, 'isError') and result.isError())
        except Exception as e:
            print(f"寫入寄存器{address}={value}失敗: {e}")
            return False
    
    def get_status_bits(self) -> dict:
        """解析狀態寄存器位元"""
        status_register = self.read_register(DobotRegisters.STATUS_REGISTER)
        if status_register is None:
            return {}
        
        return {
            "ready": bool(status_register & 0x01),      # bit0
            "running": bool(status_register & 0x02),    # bit1
            "alarm": bool(status_register & 0x04),      # bit2
            "initialized": bool(status_register & 0x08), # bit3
            "status_register_value": status_register,
            "status_register_binary": f"{status_register:04b}"
        }
    
    def is_ready_for_command(self) -> bool:
        """檢查系統是否準備好接受指令"""
        status_bits = self.get_status_bits()
        return (status_bits.get("ready", False) and 
                not status_bits.get("running", False) and 
                not status_bits.get("alarm", False))
    
    def read_angle_status(self) -> dict:
        """讀取Angle模組狀態 - 增強錯誤處理"""
        try:
            if not self.is_connected:
                return self._get_default_angle_status()
                
            # 讀取Angle狀態寄存器 (700-703)
            result = self.modbus_client.read_holding_registers(AngleRegisters.STATUS_REGISTER, count=4)
            
            if hasattr(result, 'registers') and len(result.registers) >= 4:
                registers = result.registers
                status_register = registers[0]
                
                self.angle_status.update({
                    'ready': bool(status_register & 0x01),
                    'running': bool(status_register & 0x02),
                    'alarm': bool(status_register & 0x04),
                    'modbus_connected': bool(registers[1]),
                    'motor_connected': bool(registers[2]),
                    'error_code': registers[3],
                    'available': True  # 標記Angle模組可用
                })
                
                # 讀取檢測結果 (720-726)
                try:
                    result_regs = self.modbus_client.read_holding_registers(AngleRegisters.SUCCESS_FLAG, count=7)
                    if hasattr(result_regs, 'registers') and len(result_regs.registers) >= 7:
                        res_data = result_regs.registers
                        self.angle_status['last_success'] = bool(res_data[0])
                        
                        if res_data[0]:  # 如果成功
                            # 合併32位角度數據
                            angle_int = (res_data[1] << 16) | res_data[2]
                            if angle_int >= 2**31:
                                angle_int -= 2**32
                            self.angle_status['last_angle'] = angle_int / 100.0
                            
                            # 合併32位角度差數據
                            diff_int = (res_data[3] << 16) | res_data[4]
                            if diff_int >= 2**31:
                                diff_int -= 2**32
                            self.angle_status['last_diff'] = diff_int / 100.0
                            
                            # 合併32位馬達位置
                            motor_pos = (res_data[5] << 16) | res_data[6]
                            if motor_pos >= 2**31:
                                motor_pos -= 2**32
                            self.angle_status['last_motor_pos'] = motor_pos
                except Exception as result_error:
                    # 結果寄存器讀取失敗，但狀態寄存器成功
                    print(f"讀取Angle結果寄存器失敗: {result_error}")
                
            else:
                # 狀態寄存器讀取失敗，標記為不可用
                self.angle_status.update(self._get_default_angle_status())
                self.angle_status['available'] = False
                
            return self.angle_status
            
        except Exception as e:
            print(f"讀取Angle狀態失敗: {e}")
            # 返回預設狀態，標記為不可用
            default_status = self._get_default_angle_status()
            default_status['available'] = False
            return default_status
    
    def _get_default_angle_status(self) -> dict:
        """獲取預設Angle狀態"""
        return {
            'ready': False,
            'running': False,
            'alarm': False,
            'modbus_connected': False,
            'motor_connected': False,
            'error_code': 0,
            'last_angle': None,
            'last_diff': None,
            'last_motor_pos': None,
            'last_success': False,
            'available': False
        }
    
    def read_all_status(self) -> dict:
        """讀取所有狀態寄存器"""
        try:
            if not self.is_connected:
                return {}
                
            # 讀取狀態寄存器 (400-419)
            result = self.modbus_client.read_holding_registers(DobotRegisters.STATUS_REGISTER, count=20)
            
            if not (hasattr(result, 'registers') and len(result.registers) >= 20):
                return {}
            
            registers = result.registers
            status_bits = self.get_status_bits()
            
            # 檢查Flow狀態變化並更新計時器
            current_flow = registers[2]  # CURRENT_FLOW
            running = status_bits.get("running", False)
            
            self._update_flow_timers(current_flow, running)
            
            # 讀取Angle模組狀態
            angle_status = self.read_angle_status()
            
            # 基本狀態數據
            status_data = {
                # 狀態機交握狀態
                "handshake_status": status_bits,
                "ready_for_command": self.is_ready_for_command(),
                
                # 機械臂狀態
                "robot_state": registers[1],
                "robot_state_text": ROBOT_STATES.get(registers[1], "未知"),
                "current_flow": registers[2],
                "current_flow_text": FLOW_TYPES.get(registers[2], "未知"),
                "flow_progress": registers[3],
                "error_code": registers[4],
                "robot_mode": registers[5],
                "global_speed": registers[19],
                
                # 位置資訊
                "position": {
                    "x": registers[6],
                    "y": registers[7],
                    "z": registers[8],
                    "r": registers[9]
                },
                
                # 關節角度
                "joints": {
                    "j1": registers[10] / 100.0,
                    "j2": registers[11] / 100.0,
                    "j3": registers[12] / 100.0,
                    "j4": registers[13] / 100.0
                },
                
                # IO狀態
                "io_status": {
                    "di": registers[14],
                    "do": registers[15]
                },
                
                # 統計資訊
                "statistics": {
                    "operation_count": registers[16],
                    "error_count": registers[17],
                    "run_time_minutes": registers[18]
                },
                
                # Flow計時器數據
                "flow_timers": {
                    "flow1": {
                        "duration": self.flow_timers[1]['duration'],
                        "is_running": self.flow_timers[1]['is_running']
                    },
                    "flow2": {
                        "duration": self.flow_timers[2]['duration'],
                        "is_running": self.flow_timers[2]['is_running']
                    }
                },
                
                # Angle模組狀態
                "angle_status": {
                    "angle_ready": angle_status.get('ready', False),
                    "angle_running": angle_status.get('running', False),
                    "angle_alarm": angle_status.get('alarm', False),
                    "ccd3_connected": angle_status.get('modbus_connected', False),
                    "motor_connected": angle_status.get('motor_connected', False),
                    "detected_angle": angle_status.get('last_angle'),
                    "angle_diff": angle_status.get('last_diff'),
                    "motor_position": angle_status.get('last_motor_pos'),
                    "last_correction_success": angle_status.get('last_success', False)
                },
                
                "connection_status": self.is_connected,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
            }
            
            return status_data
            
        except Exception as e:
            print(f"讀取狀態失敗: {e}")
            return {}
    
    def _update_flow_timers(self, current_flow: int, running: bool):
        """更新Flow計時器"""
        try:
            current_time = time.time()
            
            # 檢查Flow1狀態變化
            if current_flow == 1 and running and not self.flow_timers[1]['is_running']:
                # Flow1開始執行
                self.flow_timers[1]['start_time'] = current_time
                self.flow_timers[1]['is_running'] = True
                print("Flow1開始執行")
                
            elif self.flow_timers[1]['is_running'] and (not running or current_flow != 1):
                # Flow1執行結束
                if self.flow_timers[1]['start_time']:
                    self.flow_timers[1]['duration'] = current_time - self.flow_timers[1]['start_time']
                    print(f"Flow1執行完成，耗時: {self.flow_timers[1]['duration']:.1f}秒")
                self.flow_timers[1]['is_running'] = False
            
            # 檢查Flow2狀態變化
            if current_flow == 2 and running and not self.flow_timers[2]['is_running']:
                # Flow2開始執行
                self.flow_timers[2]['start_time'] = current_time
                self.flow_timers[2]['is_running'] = True
                print("Flow2開始執行")
                
            elif self.flow_timers[2]['is_running'] and (not running or current_flow != 2):
                # Flow2執行結束
                if self.flow_timers[2]['start_time']:
                    self.flow_timers[2]['duration'] = current_time - self.flow_timers[2]['start_time']
                    print(f"Flow2執行完成，耗時: {self.flow_timers[2]['duration']:.1f}秒")
                self.flow_timers[2]['is_running'] = False
                
        except Exception as e:
            print(f"更新Flow計時器錯誤: {e}")
    
    def read_control_registers(self) -> dict:
        """讀取控制寄存器狀態"""
        try:
            if not self.is_connected:
                return {}
            
            # 讀取Dobot控制寄存器
            dobot_control_data = {
                "vp_control": self.read_register(DobotRegisters.VP_CONTROL),
                "unload_control": self.read_register(DobotRegisters.UNLOAD_CONTROL),
                "clear_alarm": self.read_register(DobotRegisters.CLEAR_ALARM),
                "emergency_stop": self.read_register(DobotRegisters.EMERGENCY_STOP),
                "manual_command": self.read_register(DobotRegisters.MANUAL_COMMAND),
                "speed_command": self.read_register(DobotRegisters.SPEED_COMMAND),
                "speed_value": self.read_register(DobotRegisters.SPEED_VALUE),
                "speed_cmd_id": self.read_register(DobotRegisters.SPEED_CMD_ID),
                "status_register": self.read_register(DobotRegisters.STATUS_REGISTER),
                "robot_state": self.read_register(DobotRegisters.ROBOT_STATE),
                "current_flow": self.read_register(DobotRegisters.CURRENT_FLOW),
                "global_speed": self.read_register(DobotRegisters.GLOBAL_SPEED)
            }
            
            # 讀取Angle模組寄存器
            angle_registers = {
                "status": self.read_register(AngleRegisters.STATUS_REGISTER),
                "success": self.read_register(AngleRegisters.SUCCESS_FLAG),
                "command": self.read_register(AngleRegisters.CONTROL_COMMAND),
                "angle_high": self.read_register(AngleRegisters.ANGLE_HIGH),
                "angle_low": self.read_register(AngleRegisters.ANGLE_LOW)
            }
            
            dobot_control_data["angle_registers"] = angle_registers
            
            return dobot_control_data
            
        except Exception as e:
            print(f"讀取控制寄存器失敗: {e}")
            return {}
    
    # === 狀態機交握指令方法 ===
    
    def execute_vp_pickup(self) -> dict:
        """執行VP視覺取料 (Flow1)"""
        try:
            if not self.is_ready_for_command():
                return {
                    "success": False,
                    "message": "系統未Ready，無法執行VP視覺取料"
                }
            
            success = self.write_register(DobotRegisters.VP_CONTROL, 1)
            return {
                "success": success,
                "message": "VP視覺取料指令已發送" if success else "VP視覺取料指令發送失敗",
                "flow_description": "從CCD1佇列獲取物體座標並執行抓取動作"
            }
            
        except Exception as e:
            return {
                "success": False,
                "message": f"VP視覺取料執行失敗: {e}"
            }
    
    def execute_unload_flow(self) -> dict:
        """執行出料流程 (Flow2)"""
        try:
            if not self.is_ready_for_command():
                return {
                    "success": False,
                    "message": "系統未Ready，無法執行出料流程"
                }
            
            # 檢查VP控制是否已清零
            vp_control = self.read_register(DobotRegisters.VP_CONTROL)
            if vp_control != 0:
                return {
                    "success": False,
                    "message": f"VP控制未清零({vp_control})，無法執行出料流程"
                }
            
            success = self.write_register(DobotRegisters.UNLOAD_CONTROL, 1)
            return {
                "success": success,
                "message": "出料流程指令已發送" if success else "出料流程指令發送失敗",
                "flow_description": "從standby點開始的完整出料作業流程"
            }
            
        except Exception as e:
            return {
                "success": False,
                "message": f"出料流程執行失敗: {e}"
            }
    
    def clear_vp_pickup(self) -> dict:
        """清零VP視覺取料指令"""
        try:
            success = self.write_register(DobotRegisters.VP_CONTROL, 0)
            return {
                "success": success,
                "message": "VP視覺取料指令已清零" if success else "VP視覺取料指令清零失敗"
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"清零VP視覺取料指令失敗: {e}"
            }
    
    def clear_unload_flow(self) -> dict:
        """清零出料流程指令"""
        try:
            success = self.write_register(DobotRegisters.UNLOAD_CONTROL, 0)
            return {
                "success": success,
                "message": "出料流程指令已清零" if success else "出料流程指令清零失敗"
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"清零出料流程指令失敗: {e}"
            }
    
    def clear_alarm(self) -> dict:
        """清除警報"""
        try:
            success = self.write_register(DobotRegisters.CLEAR_ALARM, 1)
            return {
                "success": success,
                "message": "清除警報指令已發送" if success else "清除警報指令發送失敗"
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"清除警報失敗: {e}"
            }
    
    def emergency_stop(self) -> dict:
        """緊急停止"""
        try:
            success = self.write_register(DobotRegisters.EMERGENCY_STOP, 1)
            return {
                "success": success,
                "message": "緊急停止指令已發送" if success else "緊急停止指令發送失敗"
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"緊急停止失敗: {e}"
            }
    
    def manual_execute_flow(self, flow_id: int) -> dict:
        """Web端手動執行流程"""
        try:
            if not self.is_ready_for_command():
                return {
                    "success": False,
                    "message": "系統未Ready，無法執行手動指令"
                }
            
            success = self.write_register(DobotRegisters.MANUAL_COMMAND, flow_id)
            flow_name = FLOW_TYPES.get(flow_id, f"流程{flow_id}")
            
            return {
                "success": success,
                "message": f"手動執行{flow_name}指令已發送" if success else f"手動執行{flow_name}指令發送失敗"
            }
            
        except Exception as e:
            return {
                "success": False,
                "message": f"手動執行流程{flow_id}失敗: {e}"
            }
    
    def execute_angle_correction(self) -> dict:
        """執行Angle模組校正"""
        try:
            # 檢查Angle模組是否Ready
            angle_status = self.read_angle_status()
            if not angle_status.get('ready', False):
                return {
                    "success": False,
                    "message": "Angle模組未Ready，無法執行校正"
                }
            
            if angle_status.get('running', False):
                return {
                    "success": False,
                    "message": "Angle模組執行中，無法執行新的校正"
                }
            
            # 發送角度校正指令到Angle模組
            success = self.write_register(AngleRegisters.CONTROL_COMMAND, 1)
            return {
                "success": success,
                "message": "角度校正指令已發送" if success else "角度校正指令發送失敗",
                "description": "CCD3檢測 → 角度計算 → 馬達移動"
            }
            
        except Exception as e:
            return {
                "success": False,
                "message": f"角度校正執行失敗: {e}"
            }
    
    def set_global_speed(self, speed: int) -> dict:
        """設定全局速度"""
        try:
            # 檢查速度範圍
            min_speed = self.config["speed_control"]["min_speed"]
            max_speed = self.config["speed_control"]["max_speed"]
            
            if speed < min_speed or speed > max_speed:
                return {
                    "success": False,
                    "message": f"速度值超出範圍({min_speed}-{max_speed})"
                }
            
            # 生成指令ID
            self.speed_cmd_id_counter += 1
            if self.speed_cmd_id_counter > 65535:
                self.speed_cmd_id_counter = 1
            
            # 發送速度設定指令
            success1 = self.write_register(DobotRegisters.SPEED_VALUE, speed)
            success2 = self.write_register(DobotRegisters.SPEED_CMD_ID, self.speed_cmd_id_counter)
            success3 = self.write_register(DobotRegisters.SPEED_COMMAND, 1)
            
            success = success1 and success2 and success3
            
            return {
                "success": success,
                "message": f"全局速度設定為{speed}%" if success else "全局速度設定失敗",
                "speed_value": speed,
                "command_id": self.speed_cmd_id_counter
            }
            
        except Exception as e:
            return {
                "success": False,
                "message": f"設定全局速度失敗: {e}"
            }
    
    def get_global_speed(self) -> dict:
        """獲取當前全局速度"""
        try:
            current_speed = self.read_register(DobotRegisters.GLOBAL_SPEED)
            if current_speed is None:
                return {
                    "success": False,
                    "message": "讀取全局速度失敗"
                }
            
            return {
                "success": True,
                "current_speed": current_speed,
                "message": f"當前全局速度: {current_speed}%"
            }
            
        except Exception as e:
            return {
                "success": False,
                "message": f"獲取全局速度失敗: {e}"
            }
    
    def start_monitoring(self):
        """啟動狀態監控"""
        if self.monitoring:
            return
            
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        print("狀態監控已啟動 (狀態機交握版)")
    
    def stop_monitoring(self):
        """停止狀態監控"""
        self.monitoring = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        print("狀態監控已停止")
    
    def _monitor_loop(self):
        """監控循環"""
        while self.monitoring:
            try:
                if self.is_connected:
                    status = self.read_all_status()
                    if status:
                        # 通過SocketIO發送狀態更新
                        socketio.emit('status_update', status)
                        
                time.sleep(self.config["monitoring"]["refresh_interval"])
                
            except Exception as e:
                print(f"監控循環錯誤: {e}")
                time.sleep(1)

# 初始化Flask應用
app = Flask(__name__)
app.config['SECRET_KEY'] = 'dobot_handshake_controller_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# 初始化控制器
controller = DobotHandshakeController()

@app.route('/')
def index():
    """主頁面"""
    return render_template('dobot_handshake_index.html')

@app.route('/api/connect', methods=['POST'])
def connect():
    """連接Modbus服務器"""
    success = controller.connect_modbus()
    if success and controller.config["monitoring"]["auto_start"]:
        controller.start_monitoring()
    
    return jsonify({
        'success': success,
        'message': 'Modbus連接成功' if success else 'Modbus連接失敗'
    })

@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    """斷開Modbus連接"""
    controller.stop_monitoring()
    if controller.modbus_client:
        controller.modbus_client.close()
    controller.is_connected = False
    
    return jsonify({
        'success': True,
        'message': 'Modbus連接已斷開'
    })

@app.route('/api/status', methods=['GET'])
def get_status():
    """獲取系統狀態"""
    status = controller.read_all_status()
    return jsonify(status)

@app.route('/api/control_registers', methods=['GET'])
def get_control_registers():
    """獲取控制寄存器狀態"""
    control_status = controller.read_control_registers()
    return jsonify(control_status)

# === 狀態機交握API路由 ===

@app.route('/api/handshake/vp_pickup', methods=['POST'])
def handshake_vp_pickup():
    """執行VP視覺取料"""
    result = controller.execute_vp_pickup()
    return jsonify(result)

@app.route('/api/handshake/unload_flow', methods=['POST'])
def handshake_unload_flow():
    """執行出料流程"""
    result = controller.execute_unload_flow()
    return jsonify(result)

@app.route('/api/handshake/clear_vp', methods=['POST'])
def handshake_clear_vp():
    """清零VP視覺取料指令"""
    result = controller.clear_vp_pickup()
    return jsonify(result)

@app.route('/api/handshake/clear_unload', methods=['POST'])
def handshake_clear_unload():
    """清零出料流程指令"""
    result = controller.clear_unload_flow()
    return jsonify(result)

@app.route('/api/handshake/clear_alarm', methods=['POST'])
def handshake_clear_alarm():
    """清除警報"""
    result = controller.clear_alarm()
    return jsonify(result)

@app.route('/api/handshake/emergency_stop', methods=['POST'])
def handshake_emergency_stop():
    """緊急停止"""
    result = controller.emergency_stop()
    return jsonify(result)

@app.route('/api/handshake/manual_flow/<int:flow_id>', methods=['POST'])
def handshake_manual_flow(flow_id):
    """手動執行流程"""
    result = controller.manual_execute_flow(flow_id)
    return jsonify(result)

@app.route('/api/handshake/set_speed', methods=['POST'])
def handshake_set_speed():
    """設定全局速度"""
    data = request.get_json()
    speed = data.get('speed')
    
    if speed is None:
        return jsonify({
            'success': False,
            'message': '缺少speed參數'
        })
    
    try:
        speed = int(speed)
        result = controller.set_global_speed(speed)
        return jsonify(result)
    except ValueError:
        return jsonify({
            'success': False,
            'message': 'speed必須是數字'
        })

@app.route('/api/handshake/get_speed', methods=['GET'])
def handshake_get_speed():
    """獲取當前全局速度"""
    result = controller.get_global_speed()
    return jsonify(result)

# === Angle模組API路由 (通過Modbus直接操作) ===

@app.route('/api/angle/correction', methods=['POST'])
def angle_correction():
    """執行角度校正 - 直接操作Modbus寄存器"""
    result = controller.execute_angle_correction()
    return jsonify(result)

@app.route('/api/angle/status', methods=['GET'])
def angle_status():
    """獲取Angle模組狀態 - 直接讀取Modbus寄存器"""
    try:
        angle_status = controller.read_angle_status()
        
        # 格式化返回數據以符合前端期望
        formatted_status = {
            'success': True,
            'angle_ready': angle_status.get('ready', False),
            'angle_running': angle_status.get('running', False),
            'angle_alarm': angle_status.get('alarm', False),
            'ccd3_connected': angle_status.get('modbus_connected', False),
            'motor_connected': angle_status.get('motor_connected', False),
            'detected_angle': angle_status.get('last_angle'),
            'angle_diff': angle_status.get('last_diff'),
            'motor_position': angle_status.get('last_motor_pos'),
            'last_correction_success': angle_status.get('last_success', False)
        }
        
        return jsonify(formatted_status)
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'讀取Angle狀態失敗: {e}'
        })

@app.route('/api/angle/registers', methods=['GET'])
def angle_registers():
    """獲取Angle模組原始寄存器數據"""
    try:
        if not controller.is_connected:
            return jsonify({'success': False, 'message': 'Modbus未連接'})
        
        # 讀取所有Angle相關寄存器
        registers_data = {}
        
        # 狀態寄存器 (700-703)
        for addr in range(AngleRegisters.STATUS_REGISTER, AngleRegisters.STATUS_REGISTER + 4):
            registers_data[f'reg_{addr}'] = controller.read_register(addr)
        
        # 結果寄存器 (720-726)
        for addr in range(AngleRegisters.SUCCESS_FLAG, AngleRegisters.MOTOR_POS_LOW + 1):
            registers_data[f'reg_{addr}'] = controller.read_register(addr)
        
        # 控制寄存器 (740)
        registers_data[f'reg_{AngleRegisters.CONTROL_COMMAND}'] = controller.read_register(AngleRegisters.CONTROL_COMMAND)
        
        return jsonify({
            'success': True,
            'registers': registers_data
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'讀取Angle寄存器失敗: {e}'
        })

# === SocketIO事件處理 ===

@socketio.on('connect')
def on_connect():
    """客戶端連接"""
    print('客戶端已連接')
    emit('connected', {'message': '已連接到Dobot狀態機交握控制器'})

@socketio.on('disconnect')
def on_disconnect():
    """客戶端斷開"""
    print('客戶端已斷開')

@socketio.on('request_status')
def on_request_status():
    """請求狀態更新"""
    status = controller.read_all_status()
    emit('status_update', status)

@socketio.on('request_control_status')
def on_request_control_status():
    """請求控制寄存器狀態"""
    control_status = controller.read_control_registers()
    emit('control_status_update', control_status)

@socketio.on('handshake_vp_pickup')
def on_handshake_vp_pickup():
    """SocketIO VP視覺取料"""
    result = controller.execute_vp_pickup()
    emit('handshake_result', result)

@socketio.on('handshake_unload_flow')
def on_handshake_unload_flow():
    """SocketIO 出料流程"""
    result = controller.execute_unload_flow()
    emit('handshake_result', result)

@socketio.on('handshake_clear_alarm')
def on_handshake_clear_alarm():
    """SocketIO 清除警報"""
    result = controller.clear_alarm()
    emit('handshake_result', result)

@socketio.on('handshake_emergency_stop')
def on_handshake_emergency_stop():
    """SocketIO 緊急停止"""
    result = controller.emergency_stop()
    emit('handshake_result', result)

@socketio.on('handshake_manual_flow')
def on_handshake_manual_flow(data):
    """SocketIO 手動執行流程"""
    flow_id = data.get('flow_id')
    if flow_id is not None:
        result = controller.manual_execute_flow(int(flow_id))
        emit('handshake_result', result)

@socketio.on('handshake_set_speed')
def on_handshake_set_speed(data):
    """SocketIO 設定全局速度"""
    speed = data.get('speed')
    if speed is not None:
        result = controller.set_global_speed(int(speed))
        emit('speed_update_result', result)

@socketio.on('handshake_get_speed')
def on_handshake_get_speed():
    """SocketIO 獲取全局速度"""
    result = controller.get_global_speed()
    emit('speed_status_update', result)

@socketio.on('angle_correction')
def on_angle_correction():
    """SocketIO 角度校正"""
    result = controller.execute_angle_correction()
    emit('angle_result', result)

def main():
    """主函數"""
    print("=== Dobot狀態機交握控制器Web應用啟動中 (增強版) ===")
    print(f"Web服務器: http://{controller.config['web_server']['host']}:{controller.config['web_server']['port']}")
    print(f"Modbus服務器: {controller.config['modbus']['server_ip']}:{controller.config['modbus']['server_port']}")
    print("\n=== 狀態機交握寄存器映射 ===")
    print(f"主狀態寄存器: {DobotRegisters.STATUS_REGISTER} (bit0=Ready, bit1=Running, bit2=Alarm)")
    print(f"機械臂狀態: {DobotRegisters.ROBOT_STATE}")
    print(f"當前流程ID: {DobotRegisters.CURRENT_FLOW}")
    print(f"全局速度: {DobotRegisters.GLOBAL_SPEED}")
    print("\n=== Dobot控制寄存器映射 ===")
    print(f"VP視覺取料控制: {DobotRegisters.VP_CONTROL}")
    print(f"出料控制: {DobotRegisters.UNLOAD_CONTROL}")
    print(f"清除警報控制: {DobotRegisters.CLEAR_ALARM}")
    print(f"緊急停止控制: {DobotRegisters.EMERGENCY_STOP}")
    print(f"手動指令: {DobotRegisters.MANUAL_COMMAND}")
    print(f"速度控制指令: {DobotRegisters.SPEED_COMMAND}")
    print(f"速度數值: {DobotRegisters.SPEED_VALUE}")
    print(f"速度指令ID: {DobotRegisters.SPEED_CMD_ID}")
    
    print("\n=== Angle模組寄存器映射 ===")
    print(f"Angle狀態寄存器: {AngleRegisters.STATUS_REGISTER} (bit0=Ready, bit1=Running, bit2=Alarm)")
    print(f"校正成功標誌: {AngleRegisters.SUCCESS_FLAG}")
    print(f"檢測角度: {AngleRegisters.ANGLE_HIGH}-{AngleRegisters.ANGLE_LOW} (32位)")
    print(f"馬達位置: {AngleRegisters.MOTOR_POS_HIGH}-{AngleRegisters.MOTOR_POS_LOW} (32位)")
    print(f"Angle控制指令: {AngleRegisters.CONTROL_COMMAND}")
    
    print("\n=== 新增功能 ===")
    print("1. Angle模組整合 - 一鍵角度校正")
    print("2. Flow執行時間統計 - 自動計時")
    print("3. 寄存器地址顯示 - 按鈕上顯示地址")
    print("4. Angle校正狀態監控 - 即時顯示")
    print("5. 32位角度數據處理 - 保留2位小數")
    
    print("\n=== 狀態機交握流程 ===")
    print("VP視覺取料: 檢查Ready → 寫入440=1 → 執行 → 寫入440=0 → 恢復Ready")
    print("出料流程: 檢查Ready且440=0 → 寫入441=1 → 執行 → 寫入441=0 → 恢復Ready")
    print("角度校正: 檢查Ready → 寫入740=1 → CCD3檢測→計算→馬達移動 → 完成")
    print("速度設定: 寫入446=速度值 → 寫入447=指令ID → 寫入445=1 → 等待執行完成")
    
    print("\n請在瀏覽器中打開Web介面進行狀態機交握控制")
    
    try:
        socketio.run(
            app,
            host=controller.config['web_server']['host'],
            port=controller.config['web_server']['port'],
            debug=controller.config['web_server']['debug']
        )
    except KeyboardInterrupt:
        print("\n收到中斷信號，正在關閉...")
    finally:
        controller.stop_monitoring()
        if controller.modbus_client:
            controller.modbus_client.close()
        print("Web應用已關閉")

if __name__ == "__main__":
    main()

# ==================== 狀態機交握API路由總覽 (增強版) ====================
# 
# === Dobot流程控制 ===
# POST /api/handshake/vp_pickup         - 執行VP視覺取料 (寫入440=1)
# POST /api/handshake/unload_flow        - 執行出料流程 (寫入441=1)
# POST /api/handshake/clear_vp           - 清零VP指令 (寫入440=0)
# POST /api/handshake/clear_unload       - 清零出料指令 (寫入441=0)
# POST /api/handshake/manual_flow/<id>   - 手動執行流程 (寫入444=id)
# 
# === Dobot系統控制 ===
# POST /api/handshake/clear_alarm        - 清除警報 (寫入442=1)
# POST /api/handshake/emergency_stop     - 緊急停止 (寫入443=1)
# 
# === Dobot速度控制 ===
# POST /api/handshake/set_speed          - 設定全局速度 (寫入445=1, 446=速度, 447=ID)
# GET  /api/handshake/get_speed          - 獲取當前全局速度 (讀取419)
# 
# === Angle模組控制 (新增) ===
# POST /api/angle/correction             - 執行角度校正 (寫入740=1)
# GET  /api/angle/status                 - 獲取Angle模組狀態 (讀取700-726)
# 
# === 狀態查詢 ===
# GET  /api/status                       - 獲取完整系統狀態 (含Flow計時器和Angle狀態)
# GET  /api/control_registers            - 獲取控制寄存器狀態 (含Angle寄存器)
# 
# === SocketIO事件 (新增Angle支援) ===
# handshake_vp_pickup, handshake_unload_flow, handshake_clear_alarm
# handshake_emergency_stop, handshake_manual_flow, handshake_set_speed
# handshake_get_speed, request_status, request_control_status
# angle_correction (新增) - 角度校正