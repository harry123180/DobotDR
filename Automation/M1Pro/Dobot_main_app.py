#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dobot_main_app.py - Dobot主控制器Web介面 (增強版)
提供Web UI來控制和監控Dobot_main.py主控制器
基地址400的寄存器操作界面，新增FIFO佇列狀態顯示
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

# Modbus寄存器映射 (基地址400)
class DobotRegisters:
    CONTROL_CMD = 400      # 控制指令
    ROBOT_STATE = 401      # 機械臂狀態
    CURRENT_FLOW = 402     # 當前流程ID
    FLOW_PROGRESS = 403    # 流程執行進度
    ERROR_CODE = 404       # 錯誤代碼
    ROBOT_MODE = 405       # 機械臂模式
    POS_X = 406           # 當前X座標
    POS_Y = 407           # 當前Y座標
    POS_Z = 408           # 當前Z座標
    POS_R = 409           # 當前R座標
    JOINT_J1 = 410        # J1角度
    JOINT_J2 = 411        # J2角度
    JOINT_J3 = 412        # J3角度
    JOINT_J4 = 413        # J4角度
    DI_STATUS = 414       # 數位輸入狀態
    DO_STATUS = 415       # 數位輸出狀態
    OP_COUNTER = 416      # 操作計數器
    ERR_COUNTER = 417     # 錯誤計數器
    RUN_TIME = 418        # 運行時間(分鐘)

# CCD1寄存器映射 (用於FIFO佇列狀態查詢)
class CCD1Registers:
    STATUS_REGISTER = 201  # CCD1狀態寄存器
    CIRCLE_COUNT = 240     # 檢測圓形數量
    WORLD_COORD_VALID = 256 # 世界座標有效標誌

# 狀態映射
ROBOT_STATES = {
    0: "空閒", 1: "運行中", 2: "暫停", 3: "錯誤", 4: "緊急停止"
}

FLOW_TYPES = {
    0: "無流程", 1: "VP視覺抓取(FIFO)", 2: "CCD3角度檢測", 3: "完整加工流程"
}

COMMANDS = {
    0: "清空指令", 1: "執行流程1", 2: "執行流程2", 3: "執行流程3", 99: "緊急停止"
}

class DobotMainAppController:
    """Dobot主控制器Web應用控制器 (增強版)"""
    
    def __init__(self, config_file: str = CONFIG_FILE):
        self.config_file = config_file
        self.config = self._load_config()
        self.modbus_client = None
        self.is_connected = False
        self.monitoring = False
        self.monitor_thread = None
        
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
            "ui_settings": {
                "theme": "light",
                "show_advanced": True,  # 啟用進階功能顯示
                "show_ccd1_queue": True  # 新增：顯示CCD1佇列狀態
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
    
    def read_ccd1_queue_status(self) -> dict:
        """讀取CCD1佇列狀態 (新增功能)"""
        try:
            if not self.is_connected:
                return {}
            
            # 讀取CCD1相關寄存器
            ccd1_status = self.read_register(CCD1Registers.STATUS_REGISTER)
            circle_count = self.read_register(CCD1Registers.CIRCLE_COUNT)
            world_coord_valid = self.read_register(CCD1Registers.WORLD_COORD_VALID)
            
            return {
                "ccd1_ready": bool(ccd1_status & 0x01) if ccd1_status is not None else False,
                "ccd1_running": bool(ccd1_status & 0x02) if ccd1_status is not None else False,
                "ccd1_alarm": bool(ccd1_status & 0x04) if ccd1_status is not None else False,
                "circle_count": circle_count if circle_count is not None else 0,
                "world_coord_valid": bool(world_coord_valid) if world_coord_valid is not None else False,
                "queue_estimate": circle_count if circle_count is not None else 0  # 佇列估計值
            }
            
        except Exception as e:
            print(f"讀取CCD1佇列狀態失敗: {e}")
            return {}
    
    def read_all_status(self) -> dict:
        """讀取所有狀態寄存器 (增強版)"""
        try:
            if not self.is_connected:
                return {}
                
            # 讀取所有狀態寄存器 (400-418)
            result = self.modbus_client.read_holding_registers(DobotRegisters.CONTROL_CMD, count=19)
            
            if not (hasattr(result, 'registers') and len(result.registers) >= 19):
                return {}
            
            registers = result.registers
            
            # 基本狀態數據
            status_data = {
                "control_cmd": registers[0],
                "robot_state": registers[1],
                "robot_state_text": ROBOT_STATES.get(registers[1], "未知"),
                "current_flow": registers[2],
                "current_flow_text": FLOW_TYPES.get(registers[2], "未知"),
                "flow_progress": registers[3],
                "error_code": registers[4],
                "robot_mode": registers[5],
                "position": {
                    "x": registers[6],
                    "y": registers[7],
                    "z": registers[8],
                    "r": registers[9]
                },
                "joints": {
                    "j1": registers[10] / 100.0,  # 恢復小數點
                    "j2": registers[11] / 100.0,
                    "j3": registers[12] / 100.0,
                    "j4": registers[13] / 100.0
                },
                "io_status": {
                    "di": registers[14],
                    "do": registers[15]
                },
                "statistics": {
                    "operation_count": registers[16],
                    "error_count": registers[17],
                    "run_time_minutes": registers[18]
                },
                "connection_status": self.is_connected,
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
            }
            
            # 新增：CCD1佇列狀態 (如果啟用)
            if self.config["ui_settings"].get("show_ccd1_queue", True):
                ccd1_queue = self.read_ccd1_queue_status()
                status_data["ccd1_queue"] = ccd1_queue
            
            return status_data
            
        except Exception as e:
            print(f"讀取狀態失敗: {e}")
            return {}
    
    def send_command(self, command: int) -> bool:
        """發送控制指令"""
        try:
            success = self.write_register(DobotRegisters.CONTROL_CMD, command)
            if success:
                print(f"發送指令成功: {command} ({COMMANDS.get(command, '未知指令')})")
            else:
                print(f"發送指令失敗: {command}")
            return success
        except Exception as e:
            print(f"發送指令{command}異常: {e}")
            return False
    
    def start_monitoring(self):
        """啟動狀態監控"""
        if self.monitoring:
            return
            
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        print("狀態監控已啟動 (含CCD1佇列監控)")
    
    def stop_monitoring(self):
        """停止狀態監控"""
        self.monitoring = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)
        print("狀態監控已停止")
    
    def _monitor_loop(self):
        """監控循環 (增強版)"""
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
app.config['SECRET_KEY'] = 'dobot_main_controller_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*")

# 初始化控制器
controller = DobotMainAppController()

@app.route('/')
def index():
    """主頁面"""
    return render_template('dobot_main_index.html')

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
    """獲取系統狀態 (增強版)"""
    status = controller.read_all_status()
    return jsonify(status)

@app.route('/api/ccd1/queue', methods=['GET'])
def get_ccd1_queue_status():
    """獲取CCD1佇列狀態 (新增API)"""
    queue_status = controller.read_ccd1_queue_status()
    return jsonify(queue_status)

@app.route('/api/command', methods=['POST'])
def send_command():
    """發送控制指令"""
    data = request.get_json()
    command = data.get('command')
    
    if command is None:
        return jsonify({'success': False, 'message': '缺少command參數'})
    
    try:
        command = int(command)
        success = controller.send_command(command)
        return jsonify({
            'success': success,
            'message': f'指令{command}發送{"成功" if success else "失敗"}'
        })
    except ValueError:
        return jsonify({'success': False, 'message': 'command必須是數字'})

@app.route('/api/flow/<int:flow_id>', methods=['POST'])
def execute_flow(flow_id):
    """執行指定流程"""
    if flow_id not in [1, 2, 3]:
        return jsonify({'success': False, 'message': '無效的流程ID'})
    
    success = controller.send_command(flow_id)
    
    # 流程1的特殊處理提示
    if flow_id == 1:
        message = f'流程{flow_id}(VP視覺抓取FIFO){"啟動成功" if success else "啟動失敗"}'
        if success:
            message += " - 將從CCD1佇列獲取物體座標"
    else:
        message = f'流程{flow_id}{"啟動成功" if success else "啟動失敗"}'
    
    return jsonify({
        'success': success,
        'message': message
    })

@app.route('/api/emergency_stop', methods=['POST'])
def emergency_stop():
    """緊急停止"""
    success = controller.send_command(99)
    return jsonify({
        'success': success,
        'message': '緊急停止指令已發送' if success else '緊急停止指令發送失敗'
    })

@app.route('/api/clear', methods=['POST'])
def clear_command():
    """清空指令"""
    success = controller.send_command(0)
    return jsonify({
        'success': success,
        'message': '清空指令已發送' if success else '清空指令發送失敗'
    })

@app.route('/api/register/read/<int:address>', methods=['GET'])
def read_register_api(address):
    """讀取指定寄存器"""
    value = controller.read_register(address)
    return jsonify({
        'success': value is not None,
        'address': address,
        'value': value
    })

@app.route('/api/register/write', methods=['POST'])
def write_register_api():
    """寫入指定寄存器"""
    data = request.get_json()
    address = data.get('address')
    value = data.get('value')
    
    if address is None or value is None:
        return jsonify({'success': False, 'message': '缺少address或value參數'})
    
    try:
        address = int(address)
        value = int(value)
        success = controller.write_register(address, value)
        return jsonify({
            'success': success,
            'message': f'寄存器{address}寫入{"成功" if success else "失敗"}'
        })
    except ValueError:
        return jsonify({'success': False, 'message': 'address和value必須是數字'})

# SocketIO事件處理
@socketio.on('connect')
def on_connect():
    """客戶端連接"""
    print('客戶端已連接')
    emit('connected', {'message': '已連接到Dobot主控制器 (增強版)'})

@socketio.on('disconnect')
def on_disconnect():
    """客戶端斷開"""
    print('客戶端已斷開')

@socketio.on('request_status')
def on_request_status():
    """請求狀態更新"""
    status = controller.read_all_status()
    emit('status_update', status)

@socketio.on('request_ccd1_queue')
def on_request_ccd1_queue():
    """請求CCD1佇列狀態 (新增事件)"""
    queue_status = controller.read_ccd1_queue_status()
    emit('ccd1_queue_update', queue_status)

@socketio.on('send_command')
def on_send_command(data):
    """SocketIO發送指令"""
    command = data.get('command')
    if command is not None:
        success = controller.send_command(int(command))
        emit('command_result', {
            'success': success,
            'command': command,
            'message': f'指令{command}{"發送成功" if success else "發送失敗"}'
        })

def main():
    """主函數"""
    print("=== Dobot主控制器Web應用啟動中 (增強版) ===")
    print(f"Web服務器: http://{controller.config['web_server']['host']}:{controller.config['web_server']['port']}")
    print(f"Modbus服務器: {controller.config['modbus']['server_ip']}:{controller.config['modbus']['server_port']}")
    print("控制寄存器映射:")
    print("  400: 控制指令 (0=清空, 1=流程1, 2=流程2, 3=流程3, 99=緊急停止)")
    print("  401: 機械臂狀態 (0=空閒, 1=運行, 2=暫停, 3=錯誤, 4=緊急停止)")
    print("  402: 當前流程ID")
    print("  403: 流程執行進度 (0-100%)")
    print("新增功能:")
    print("  - CCD1佇列狀態監控 (寄存器201, 240, 256)")
    print("  - 流程1 FIFO模式支援")
    print("\n請在瀏覽器中打開Web介面進行控制")
    
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