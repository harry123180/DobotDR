#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LED_app.py - LED控制器Web UI應用
純ModbusTCP Client實現，參考VP_app.py架構
"""

import os
import json
import time
import threading
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from pymodbus.client import ModbusTcpClient
import logging
from logging.handlers import RotatingFileHandler

def setup_logging(module_name):
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

class LEDWebApp:
    """LED控制器Web應用 - 純ModbusTCP Client"""
    
    def __init__(self, config_file="led_app_config.json"):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(self.current_dir, config_file)
        
        # 設置日誌
        self.logger = setup_logging("LED_WebApp")
        
        self.load_config()
        
        # Flask應用初始化 - 模板路徑設為執行檔同層的templates目錄
        template_dir = os.path.join(self.current_dir, 'templates')
        self.app = Flask(__name__, template_folder=template_dir)
        self.app.config['SECRET_KEY'] = 'led_controller_web_secret_key'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # Modbus TCP Client (連接主服務器)
        self.modbus_client = None
        self.connected_to_server = False
        self.base_address = self.config["modbus_mapping"]["base_address"]
        
        # 狀態監控
        self.monitor_thread = None
        self.monitoring = False
        self.monitor_cycle_count = 0
        
        # 設置路由和事件
        self.setup_routes()
        self.setup_socketio_events()
        
        self.logger.info("LED Web應用初始化完成")
    
    def load_config(self):
        """載入配置檔案"""
        default_config = {
            "module_id": "LED控制器Web UI",
            "tcp_server": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 3.0
            },
            "modbus_mapping": {
                "base_address": 500
            },
            "web_server": {
                "host": "0.0.0.0", 
                "port": 5008,
                "debug": False
            },
            "ui_settings": {
                "refresh_interval": 2.0,
                "auto_refresh": True
            }
        }
        
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    self.config = json.load(f)
                self.logger.info(f"載入配置檔案成功: {self.config_file}")
            except Exception as e:
                self.logger.error(f"載入配置失敗: {e}", exc_info=True)
                self.config = default_config
                self.logger.info("使用預設配置")
        else:
            self.config = default_config
            self.save_config()
            self.logger.info("配置檔案不存在，已建立預設配置")
    
    def save_config(self):
        """保存配置檔案"""
        try:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(self.config, f, indent=2, ensure_ascii=False)
            self.logger.debug("配置檔案已保存")
        except Exception as e:
            self.logger.error(f"保存配置失敗: {e}", exc_info=True)
    
    def connect_server(self) -> bool:
        """連接Modbus TCP服務器"""
        try:
            if self.modbus_client and self.modbus_client.connected:
                return True
                
            tcp_config = self.config["tcp_server"]
            self.logger.info(f"正在連接 {tcp_config['host']}:{tcp_config['port']}")
            
            self.modbus_client = ModbusTcpClient(
                host=tcp_config["host"],
                port=tcp_config["port"],
                timeout=tcp_config["timeout"]
            )
            
            if self.modbus_client.connect():
                self.connected_to_server = True
                self.logger.info("連接建立成功")
                return True
            else:
                self.connected_to_server = False
                self.logger.warning("連接失敗，將在主循環中重試")
                return False
                
        except Exception as e:
            self.logger.error(f"連接服務器失敗: {e}", exc_info=True)
            self.connected_to_server = False
            return False
    
    def read_status(self) -> dict:
        """讀取LED狀態 (從LED_main.py的寄存器)"""
        try:
            if not self.connected_to_server:
                if not self.connect_server():
                    self.logger.warning("無法連接服務器，返回錯誤狀態")
                    return {"error": "無法連接服務器"}
            
            # 讀取狀態寄存器 (500-515)
            result = self.modbus_client.read_holding_registers(
                address=self.base_address,
                count=16,
                slave=self.config["tcp_server"]["unit_id"]
            )
            if result.isError():
                self.logger.error("讀取狀態失敗")
                return {"error": "讀取狀態失敗"}
            
            registers = result.registers
            
            status_map = {
                0: "離線", 1: "閒置", 2: "執行中", 3: "初始化", 4: "錯誤"
            }
            
            status_data = {
                "module_status": status_map.get(registers[0], "未知"),
                "device_connection": "已連接" if registers[1] else "斷開",
                "active_channels": registers[2],
                "error_code": registers[3],
                "channels": {
                    "L1": {"state": bool(registers[4]), "brightness": registers[8]},
                    "L2": {"state": bool(registers[5]), "brightness": registers[9]},
                    "L3": {"state": bool(registers[6]), "brightness": registers[10]},
                    "L4": {"state": bool(registers[7]), "brightness": registers[11]}
                },
                "operation_count": registers[12],
                "error_count": registers[13],
                "timestamp": registers[15],
                "base_address": self.base_address
            }
            
            self.logger.debug(f"讀取狀態成功: 模組狀態={status_data['module_status']}, 活躍通道={status_data['active_channels']}")
            return status_data
            
        except Exception as e:
            self.logger.error(f"讀取狀態失敗: {e}", exc_info=True)
            return {"error": str(e)}
    
    def send_command(self, command: int, param1: int = 0, param2: int = 0) -> bool:
        """發送指令到LED_main.py"""
        try:
            if not self.connected_to_server:
                if not self.connect_server():
                    self.logger.warning("無法連接服務器，指令發送失敗")
                    return False
            
            command_address = self.base_address + 20  # 520
            command_id = int(time.time() * 1000) % 65536
            
            values = [command, param1, param2, command_id, 0]
            
            self.logger.debug(f"發送指令: 命令={command}, 參數1={param1}, 參數2={param2}")
            
            result = self.modbus_client.write_registers(
                address=command_address,
                values=values,
                slave=self.config["tcp_server"]["unit_id"]
            )
            
            success = not result.isError()
            if success:
                self.logger.info(f"指令發送成功: 命令={command}")
            else:
                self.logger.error(f"指令發送失敗: 命令={command}")
            
            return success
            
        except Exception as e:
            self.logger.error(f"發送指令失敗: {e}", exc_info=True)
            return False
    
    def setup_routes(self):
        """設置Flask路由"""
        
        @self.app.route('/')
        def index():
            self.logger.debug("Web首頁被訪問")
            return render_template('led_index.html')
        
        @self.app.route('/api/status')
        def api_status():
            status = self.read_status()
            return jsonify(status)
        
        @self.app.route('/api/connect', methods=['POST'])
        def api_connect():
            success = self.connect_server()
            self.logger.info(f"API連接請求: {'成功' if success else '失敗'}")
            return jsonify({"success": success})
        
        @self.app.route('/api/channel/brightness', methods=['POST'])
        def api_set_brightness():
            data = request.get_json()
            channel = data.get('channel', 1)
            brightness = data.get('brightness', 0)
            
            if not (1 <= channel <= 4):
                self.logger.warning(f"API設定亮度失敗: 通道號錯誤 {channel}")
                return jsonify({"success": False, "error": "通道號必須在1-4之間"})
            if not (0 <= brightness <= 511):
                self.logger.warning(f"API設定亮度失敗: 亮度值錯誤 {brightness}")
                return jsonify({"success": False, "error": "亮度必須在0-511之間"})
            
            # 指令4: 設定單一通道亮度
            success = self.send_command(4, channel, brightness)
            self.logger.info(f"API設定通道{channel}亮度{brightness}: {'成功' if success else '失敗'}")
            return jsonify({"success": success})
        
        @self.app.route('/api/channel/on', methods=['POST'])
        def api_turn_on():
            data = request.get_json()
            channel = data.get('channel', 1)
            
            if not (1 <= channel <= 4):
                self.logger.warning(f"API開啟通道失敗: 通道號錯誤 {channel}")
                return jsonify({"success": False, "error": "通道號必須在1-4之間"})
            
            # 指令5: 開啟單一通道
            success = self.send_command(5, channel, 0)
            self.logger.info(f"API開啟通道{channel}: {'成功' if success else '失敗'}")
            return jsonify({"success": success})
        
        @self.app.route('/api/channel/off', methods=['POST'])
        def api_turn_off():
            data = request.get_json()
            channel = data.get('channel', 1)
            
            if not (1 <= channel <= 4):
                self.logger.warning(f"API關閉通道失敗: 通道號錯誤 {channel}")
                return jsonify({"success": False, "error": "通道號必須在1-4之間"})
            
            # 指令6: 關閉單一通道
            success = self.send_command(6, channel, 0)
            self.logger.info(f"API關閉通道{channel}: {'成功' if success else '失敗'}")
            return jsonify({"success": success})
        
        @self.app.route('/api/all_on', methods=['POST'])
        def api_all_on():
            # 指令1: 全部開啟
            success = self.send_command(1, 0, 0)
            self.logger.info(f"API全部開啟: {'成功' if success else '失敗'}")
            return jsonify({"success": success})
        
        @self.app.route('/api/all_off', methods=['POST'])
        def api_all_off():
            # 指令2: 全部關閉
            success = self.send_command(2, 0, 0)
            self.logger.info(f"API全部關閉: {'成功' if success else '失敗'}")
            return jsonify({"success": success})
        
        @self.app.route('/api/reset', methods=['POST'])
        def api_reset():
            # 指令3: 重置設備
            success = self.send_command(3, 0, 0)
            self.logger.info(f"API重置設備: {'成功' if success else '失敗'}")
            return jsonify({"success": success})
        
        @self.app.route('/api/error_reset', methods=['POST'])
        def api_error_reset():
            # 指令7: 錯誤重置
            success = self.send_command(7, 0, 0)
            self.logger.info(f"API錯誤重置: {'成功' if success else '失敗'}")
            return jsonify({"success": success})
    
    def setup_socketio_events(self):
        """設置SocketIO事件"""
        
        @self.socketio.on('connect')
        def handle_connect():
            self.logger.info('客戶端已連接')
            emit('status', self.read_status())
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            self.logger.info('客戶端已斷開')
        
        @self.socketio.on('get_status')
        def handle_get_status():
            status = self.read_status()
            emit('status', status)
        
        @self.socketio.on('set_brightness')
        def handle_set_brightness(data):
            channel = data.get('channel', 1)
            brightness = data.get('brightness', 0)
            
            if 1 <= channel <= 4 and 0 <= brightness <= 511:
                success = self.send_command(4, channel, brightness)
                self.logger.info(f"SocketIO設定通道{channel}亮度{brightness}: {'成功' if success else '失敗'}")
                emit('command_result', {"success": success, "command": "set_brightness"})
            else:
                self.logger.warning(f"SocketIO設定亮度參數錯誤: 通道={channel}, 亮度={brightness}")
                emit('command_result', {"success": False, "error": "參數範圍錯誤"})
        
        @self.socketio.on('channel_control')
        def handle_channel_control(data):
            channel = data.get('channel', 1)
            action = data.get('action', 'off')
            
            if not (1 <= channel <= 4):
                self.logger.warning(f"SocketIO通道控制失敗: 通道號錯誤 {channel}")
                emit('command_result', {"success": False, "error": "通道號錯誤"})
                return
            
            if action == 'on':
                success = self.send_command(5, channel, 0)
            else:
                success = self.send_command(6, channel, 0)
            
            self.logger.info(f"SocketIO通道{channel} {action}: {'成功' if success else '失敗'}")
            emit('command_result', {"success": success, "command": f"channel_{action}"})
        
        @self.socketio.on('global_control')
        def handle_global_control(data):
            action = data.get('action', 'all_off')
            
            if action == 'all_on':
                success = self.send_command(1, 0, 0)
            elif action == 'all_off':
                success = self.send_command(2, 0, 0)
            elif action == 'reset':
                success = self.send_command(3, 0, 0)
            elif action == 'error_reset':
                success = self.send_command(7, 0, 0)
            else:
                success = False
                self.logger.warning(f"SocketIO全域控制: 未知動作 {action}")
                
            self.logger.info(f"SocketIO全域控制 {action}: {'成功' if success else '失敗'}")
            emit('command_result', {"success": success, "command": action})
    
    def status_monitor(self):
        """狀態監控線程"""
        self.logger.info("狀態監控執行緒已啟動")
        
        while self.monitoring:
            try:
                status = self.read_status()
                self.socketio.emit('status_update', status)
                
                # 每5個週期記錄一次統計資訊 (避免過於頻繁的日誌)
                self.monitor_cycle_count += 1
                if self.monitor_cycle_count % 5 == 0:
                    self.logger.debug(f"狀態監控週期: {self.monitor_cycle_count}, 模組狀態: {status.get('module_status', 'unknown')}")
                
                time.sleep(self.config["ui_settings"]["refresh_interval"])
                
            except Exception as e:
                self.logger.error(f"狀態監控錯誤: {e}", exc_info=True)
                time.sleep(1)
        
        self.logger.info("狀態監控執行緒已停止")
    
    def start_monitoring(self):
        """啟動狀態監控"""
        if not self.monitoring:
            self.monitoring = True
            self.monitor_thread = threading.Thread(target=self.status_monitor, daemon=True)
            self.monitor_thread.start()
            self.logger.info("狀態監控已啟動")
        else:
            self.logger.warning("狀態監控已在運行中")
    
    def stop_monitoring(self):
        """停止狀態監控"""
        if self.monitoring:
            self.monitoring = False
            if self.monitor_thread:
                self.monitor_thread.join(timeout=2)
            self.logger.info("狀態監控已停止")
        else:
            self.logger.debug("狀態監控未運行")
    
    def run(self):
        """運行Web應用"""
        web_config = self.config["web_server"]
        
        self.logger.info("LED控制器Web應用啟動中")
        self.logger.info(f"模組ID: {self.config['module_id']}")
        self.logger.info(f"Web服務器: http://{web_config['host']}:{web_config['port']}")
        self.logger.info(f"Modbus服務器地址: {self.config['tcp_server']['host']}:{self.config['tcp_server']['port']}")
        self.logger.info(f"LED控制器基地址: {self.base_address}")
        self.logger.info("架構: 純ModbusTCP Client -> LED_main.py")
        
        # 嘗試連接服務器
        self.connect_server()
        
        # 啟動狀態監控
        if self.config["ui_settings"]["auto_refresh"]:
            self.start_monitoring()
        
        try:
            # 運行Flask應用
            self.logger.info("Flask應用開始運行")
            self.socketio.run(
                self.app,
                host=web_config["host"],
                port=web_config["port"],
                debug=web_config["debug"]
            )
        except Exception as e:
            self.logger.error(f"Flask應用運行錯誤: {e}", exc_info=True)
            raise

def main():
    """主函數"""
    logger = setup_logging("LED_Main")
    logger.info("LED控制器Web UI啟動中")
    logger.info("架構: Web UI -> ModbusTCP Client -> LED_main.py")
    
    app = LEDWebApp()
    try:
        app.run()
    except KeyboardInterrupt:
        logger.info("收到停止信號，正在關閉應用")
        app.stop_monitoring()
    except Exception as e:
        logger.error(f"應用錯誤: {e}", exc_info=True)
        app.stop_monitoring()
    finally:
        logger.info("LED控制器Web UI已停止")

if __name__ == "__main__":
    main()