# -*- coding: utf-8 -*-
"""
VP_app.py - 震動盤Web UI控制應用
作為純Modbus TCP Client連接主服務器，通過VP_main模組控制震動盤
"""

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import threading
import time
import json
import os
import logging
from logging.handlers import RotatingFileHandler
from datetime import datetime
from pymodbus.client import ModbusTcpClient
from typing import Dict, Any, Optional


def setup_logging(module_name: str) -> logging.Logger:
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


class VibrationPlateWebApp:
    """震動盤Web控制應用 - 純Modbus TCP Client"""
    
    def __init__(self):
        # 設置logger
        self.logger = setup_logging('VP_WebApp')
        self.logger.info("震動盤Web UI初始化開始")
        
        # 載入配置
        self.config = self.load_config()
        
        # Modbus TCP Client (連接主服務器)
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.connected_to_server = False
        
        # 狀態監控
        self.status_monitor_thread = None
        self.monitoring = False
        self.status_update_counter = 0  # 用於條件日誌
        
        # 寄存器映射 (與VP_main.py一致 - 基地址300)
        self.base_address = self.config['modbus_mapping']['base_address']
        self.init_register_mapping()
        
        # 狀態快取和指令計數
        self.command_id_counter = 1
        
        # 初始化Flask應用
        self.init_flask_app()
        
        self.logger.info("震動盤Web UI初始化完成")
        
    def load_config(self) -> Dict[str, Any]:
        """載入配置檔案"""
        default_config = {
            "module_id": "震動盤Web UI",
            "tcp_server": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 1.0
            },
            "modbus_mapping": {
                "base_address": 300
            },
            "web_server": {
                "host": "0.0.0.0",
                "port": 5053,
                "debug": False
            },
            "defaults": {
                "brightness": 28,
                "strength": 100,
                "frequency": 100
            }
        }
        
        try:
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(current_dir, "vp_app_config.json")
            
            if os.path.exists(config_path):
                with open(config_path, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    default_config.update(loaded_config)
                self.logger.info(f"已載入配置檔案: {config_path}")
            else:
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                self.logger.info(f"已創建預設配置檔案: {config_path}")
        except Exception as e:
            self.logger.error(f"載入配置檔案失敗: {e}", exc_info=True)
            
        return default_config
    
    def init_register_mapping(self):
        """初始化寄存器映射 (與VP_main.py一致)"""
        base = self.base_address  # 300
        
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
            'command_code': base + 20,          # 指令代碼 (320)
            'param1': base + 21,                # 參數1
            'param2': base + 22,                # 參數2
            'param3': base + 23,                # 參數3 (頻率)
            'command_id': base + 24,            # 指令ID
            'reserved': base + 25               # 保留
        }
        
        # VP_main指令映射
        self.command_map = {
            'nop': 0,                # 無操作
            'enable_device': 1,      # 設備啟用 (背光開啟)
            'disable_device': 2,     # 設備停用 (背光關閉)
            'stop_all': 3,           # 停止所有動作
            'set_brightness': 4,     # 設定背光亮度
            'execute_action': 5,     # 執行動作
            'emergency_stop': 6,     # 緊急停止
            'reset_error': 7,        # 錯誤重置
            'set_frequency': 14,     # 設定頻率
        }
        
        # 動作編碼映射 (用於execute_action指令的param1)
        self.action_map = {
            'stop': 0, 'up': 1, 'down': 2, 'left': 3, 'right': 4,
            'upleft': 5, 'downleft': 6, 'upright': 7, 'downright': 8,
            'horizontal': 9, 'vertical': 10, 'spread': 11
        }
        
        self.logger.info(f"震動盤Web UI寄存器映射初始化:")
        self.logger.info(f"  主服務器: {self.config['tcp_server']['host']}:{self.config['tcp_server']['port']}")
        self.logger.info(f"  基地址: {base}")
        self.logger.info(f"  指令寄存器: {base + 20} ~ {base + 25}")
        self.logger.debug(f"  預設亮度: {self.config['defaults']['brightness']}")
        self.logger.debug(f"  預設強度: {self.config['defaults']['strength']}")
        self.logger.debug(f"  預設頻率: {self.config['defaults']['frequency']}")
        
    def init_flask_app(self):
        """初始化Flask應用"""
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'vp_web_app_2024'
        
        # 初始化SocketIO
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        
        # 錯誤處理
        @self.app.errorhandler(404)
        def not_found_error(error):
            self.logger.warning(f"404錯誤 - 路徑不存在: {request.path}")
            return jsonify({'success': False, 'message': '路徑不存在'}), 404
        
        @self.app.errorhandler(500)
        def internal_error(error):
            self.logger.error(f"500錯誤 - 內部服務器錯誤: {error}", exc_info=True)
            return jsonify({'success': False, 'message': '內部服務器錯誤'}), 500
        
        @self.app.errorhandler(405)
        def method_not_allowed(error):
            self.logger.warning(f"405錯誤 - 請求方法不被允許: {request.method} {request.path}")
            return jsonify({'success': False, 'message': '請求方法不被允許'}), 405
        
        # 註冊路由
        self.register_routes()
        self.register_socketio_events()
        
    def register_routes(self):
        """註冊Flask路由"""
        
        @self.app.route('/')
        def index():
            """主頁面"""
            self.logger.debug("主頁面請求")
            return render_template('index.html', config=self.config)
        
        @self.app.route('/api/status')
        def get_status():
            """獲取系統狀態"""
            self.logger.debug("狀態API請求")
            return jsonify(self.get_current_status())
        
        @self.app.route('/api/connect', methods=['POST'])
        def connect_server():
            """連接到主Modbus TCP服務器"""
            try:
                data = request.get_json() or {}
                
                if 'host' in data:
                    self.config['tcp_server']['host'] = data['host']
                    self.logger.info(f"更新主服務器地址: {data['host']}")
                if 'port' in data:
                    self.config['tcp_server']['port'] = int(data['port'])
                    self.logger.info(f"更新主服務器端口: {data['port']}")
                
                result = self.connect_modbus_server()
                
                if result['success']:
                    self.start_monitoring()
                
                return jsonify(result)
            except Exception as e:
                self.logger.error(f"連接時發生錯誤: {e}", exc_info=True)
                return jsonify({
                    'success': False,
                    'message': f'連接時發生錯誤: {str(e)}'
                })
        
        @self.app.route('/api/disconnect', methods=['POST'])
        def disconnect_server():
            """斷開Modbus TCP連接"""
            try:
                result = self.disconnect_modbus_server()
                return jsonify(result)
            except Exception as e:
                self.logger.error(f"斷開連接時發生錯誤: {e}", exc_info=True)
                return jsonify({
                    'success': False,
                    'message': f'斷開連接時發生錯誤: {str(e)}'
                })
        
        @self.app.route('/api/action', methods=['POST'])
        def execute_action():
            """執行震動動作"""
            try:
                if not self.connected_to_server:
                    self.logger.warning("執行動作失敗 - 主服務器未連接")
                    return jsonify({
                        'success': False,
                        'message': '主服務器未連接'
                    })
                
                data = request.get_json()
                if not data:
                    self.logger.warning("執行動作失敗 - 無效的請求數據")
                    return jsonify({
                        'success': False,
                        'message': '無效的請求數據'
                    })
                
                action = data.get('action')
                strength = data.get('strength', self.config['defaults']['strength'])
                frequency = data.get('frequency', self.config['defaults']['frequency'])
                
                if not action or action not in self.action_map:
                    self.logger.warning(f"執行動作失敗 - 未知動作: {action}")
                    return jsonify({
                        'success': False,
                        'message': f'未知動作: {action}'
                    })
                
                # 發送execute_action指令 (指令5)
                action_code = self.action_map[action]
                self.logger.info(f"執行震動動作: {action} (code={action_code}, strength={strength}, frequency={frequency})")
                result = self.send_command('execute_action', action_code, strength, frequency)
                
                return jsonify(result)
            except Exception as e:
                self.logger.error(f"執行動作時發生錯誤: {e}", exc_info=True)
                return jsonify({
                    'success': False,
                    'message': f'執行動作時發生錯誤: {str(e)}'
                })
        
        @self.app.route('/api/execute_action', methods=['POST'])
        def execute_action_alt():
            """執行震動動作 (備用路徑)"""
            try:
                if not self.connected_to_server:
                    self.logger.warning("執行動作失敗 - 主服務器未連接 (備用路徑)")
                    return jsonify({
                        'success': False,
                        'message': '主服務器未連接'
                    })
                
                data = request.get_json()
                if not data:
                    self.logger.warning("執行動作失敗 - 無效的請求數據 (備用路徑)")
                    return jsonify({
                        'success': False,
                        'message': '無效的請求數據'
                    })
                
                action = data.get('action')
                strength = data.get('strength', self.config['defaults']['strength'])
                frequency = data.get('frequency', self.config['defaults']['frequency'])
                
                if not action or action not in self.action_map:
                    self.logger.warning(f"執行動作失敗 - 未知動作: {action} (備用路徑)")
                    return jsonify({
                        'success': False,
                        'message': f'未知動作: {action}'
                    })
                
                # 發送execute_action指令 (指令5)
                action_code = self.action_map[action]
                self.logger.info(f"執行震動動作 (備用路徑): {action} (code={action_code}, strength={strength}, frequency={frequency})")
                result = self.send_command('execute_action', action_code, strength, frequency)
                
                return jsonify(result)
            except Exception as e:
                self.logger.error(f"執行動作時發生錯誤 (備用路徑): {e}", exc_info=True)
                return jsonify({
                    'success': False,
                    'message': f'執行動作時發生錯誤: {str(e)}'
                })
        
        @self.app.route('/api/stop', methods=['POST'])
        def stop_action():
            """停止動作 - 發送停止指令到VP_main"""
            try:
                if not self.connected_to_server:
                    self.logger.warning("停止動作失敗 - 主服務器未連接")
                    return jsonify({
                        'success': False,
                        'message': '主服務器未連接'
                    })
                
                # 發送stop_all指令 (指令3) 到VP_main
                self.logger.info("發送停止指令到VP_main")
                result = self.send_command('stop_all')
                
                if result['success']:
                    result['message'] = '停止指令發送成功'
                    self.logger.info("停止指令發送成功")
                else:
                    result['message'] = '停止指令發送失敗'
                    self.logger.warning("停止指令發送失敗")
                
                return jsonify(result)
            except Exception as e:
                self.logger.error(f"停止動作時發生錯誤: {e}", exc_info=True)
                return jsonify({
                    'success': False,
                    'message': f'停止動作時發生錯誤: {str(e)}'
                })
        
        @self.app.route('/api/emergency_stop', methods=['POST'])
        def emergency_stop():
            """緊急停止"""
            try:
                if not self.connected_to_server:
                    self.logger.critical("緊急停止失敗 - 主服務器未連接")
                    return jsonify({
                        'success': False,
                        'message': '主服務器未連接'
                    })
                
                # 發送emergency_stop指令 (指令6) 到VP_main
                self.logger.critical("發送緊急停止指令到VP_main")
                result = self.send_command('emergency_stop')
                
                return jsonify(result)
            except Exception as e:
                self.logger.critical(f"緊急停止時發生錯誤: {e}", exc_info=True)
                return jsonify({
                    'success': False,
                    'message': f'緊急停止時發生錯誤: {str(e)}'
                })
        
        @self.app.route('/api/set_brightness', methods=['POST'])
        def set_brightness():
            """設定背光亮度"""
            try:
                if not self.connected_to_server:
                    self.logger.warning("設定亮度失敗 - 主服務器未連接")
                    return jsonify({
                        'success': False,
                        'message': '主服務器未連接'
                    })
                
                data = request.get_json()
                if not data:
                    self.logger.warning("設定亮度失敗 - 無效的請求數據")
                    return jsonify({
                        'success': False,
                        'message': '無效的請求數據'
                    })
                
                brightness = data.get('brightness', self.config['defaults']['brightness'])
                brightness = max(0, min(255, int(brightness)))
                
                # 發送set_brightness指令 (指令4)
                self.logger.info(f"設定背光亮度: {brightness}")
                result = self.send_command('set_brightness', brightness)
                
                if result['success']:
                    self.config['defaults']['brightness'] = brightness
                    self.logger.debug(f"更新預設亮度設定: {brightness}")
                
                return jsonify(result)
            except Exception as e:
                self.logger.error(f"設定亮度時發生錯誤: {e}", exc_info=True)
                return jsonify({
                    'success': False,
                    'message': f'設定亮度時發生錯誤: {str(e)}'
                })
        
        @self.app.route('/api/set_backlight', methods=['POST'])
        def set_backlight():
            """設定背光開關"""
            try:
                if not self.connected_to_server:
                    self.logger.warning("設定背光失敗 - 主服務器未連接")
                    return jsonify({
                        'success': False,
                        'message': '主服務器未連接'
                    })
                
                data = request.get_json()
                if not data:
                    self.logger.warning("設定背光失敗 - 無效的請求數據")
                    return jsonify({
                        'success': False,
                        'message': '無效的請求數據'
                    })
                
                state = data.get('state', True)
                
                # 發送背光控制指令
                command = 'enable_device' if state else 'disable_device'
                self.logger.info(f"設定背光開關: {'開啟' if state else '關閉'}")
                result = self.send_command(command)
                
                return jsonify(result)
            except Exception as e:
                self.logger.error(f"設定背光時發生錯誤: {e}", exc_info=True)
                return jsonify({
                    'success': False,
                    'message': f'設定背光時發生錯誤: {str(e)}'
                })
        
        @self.app.route('/api/set_frequency', methods=['POST'])
        def set_frequency():
            """設定震動頻率"""
            try:
                if not self.connected_to_server:
                    self.logger.warning("設定頻率失敗 - 主服務器未連接")
                    return jsonify({
                        'success': False,
                        'message': '主服務器未連接'
                    })
                
                data = request.get_json()
                if not data:
                    self.logger.warning("設定頻率失敗 - 無效的請求數據")
                    return jsonify({
                        'success': False,
                        'message': '無效的請求數據'
                    })
                
                frequency = data.get('frequency', self.config['defaults']['frequency'])
                frequency = max(0, min(255, int(frequency)))
                
                # 發送set_frequency指令 (指令14)
                self.logger.info(f"設定震動頻率: {frequency}")
                result = self.send_command('set_frequency', frequency)
                
                if result['success']:
                    self.config['defaults']['frequency'] = frequency
                    self.logger.debug(f"更新預設頻率設定: {frequency}")
                
                return jsonify(result)
            except Exception as e:
                self.logger.error(f"設定頻率時發生錯誤: {e}", exc_info=True)
                return jsonify({
                    'success': False,
                    'message': f'設定頻率時發生錯誤: {str(e)}'
                })
        
        @self.app.route('/api/reset_error', methods=['POST'])
        def reset_error():
            """重置錯誤"""
            try:
                if not self.connected_to_server:
                    self.logger.warning("重置錯誤失敗 - 主服務器未連接")
                    return jsonify({
                        'success': False,
                        'message': '主服務器未連接'
                    })
                
                self.logger.info("發送錯誤重置指令")
                result = self.send_command('reset_error')
                return jsonify(result)
            except Exception as e:
                self.logger.error(f"重置錯誤時發生錯誤: {e}", exc_info=True)
                return jsonify({
                    'success': False,
                    'message': f'重置錯誤時發生錯誤: {str(e)}'
                })
        
        @self.app.route('/api/get_register_values', methods=['GET'])
        def get_register_values():
            """獲取寄存器數值"""
            if not self.connected_to_server:
                self.logger.warning("獲取寄存器數值失敗 - 主服務器未連接")
                return jsonify({
                    'success': False,
                    'message': '主服務器未連接'
                })
            
            try:
                # 讀取狀態寄存器
                status_values = {}
                for name, addr in self.status_registers.items():
                    value = self.read_register(addr)
                    status_values[name] = value
                
                # 讀取指令寄存器
                command_values = {}
                for name, addr in self.command_registers.items():
                    value = self.read_register(addr)
                    command_values[name] = value
                
                self.logger.debug("成功讀取所有寄存器數值")
                return jsonify({
                    'success': True,
                    'status_registers': status_values,
                    'command_registers': command_values,
                    'base_address': self.base_address
                })
            except Exception as e:
                self.logger.error(f"讀取寄存器失敗: {e}", exc_info=True)
                return jsonify({
                    'success': False,
                    'message': f'讀取寄存器失敗: {str(e)}'
                })
        
        @self.app.route('/api/debug', methods=['GET'])
        def debug_info():
            """調試資訊"""
            try:
                debug_data = {
                    'success': True,
                    'server_status': {
                        'connected': self.connected_to_server,
                        'server_config': self.config['tcp_server'],
                        'base_address': self.base_address
                    },
                    'register_mapping': {
                        'status_registers': self.status_registers,
                        'command_registers': self.command_registers
                    },
                    'command_map': self.command_map,
                    'action_map': self.action_map,
                    'command_counter': self.command_id_counter,
                    'defaults': self.config['defaults']
                }
                self.logger.debug("調試資訊API請求")
                return jsonify(debug_data)
            except Exception as e:
                self.logger.error(f"獲取調試資訊失敗: {e}", exc_info=True)
                return jsonify({
                    'success': False,
                    'message': f'獲取調試資訊失敗: {str(e)}'
                })
        
        @self.app.route('/api/routes', methods=['GET'])
        def list_routes():
            """列出所有可用路由"""
            try:
                routes = []
                for rule in self.app.url_map.iter_rules():
                    if rule.endpoint != 'static':
                        routes.append({
                            'endpoint': rule.endpoint,
                            'methods': list(rule.methods),
                            'path': str(rule.rule)
                        })
                
                self.logger.debug(f"路由列表API請求 - 找到 {len(routes)} 個路由")
                return jsonify({
                    'success': True,
                    'routes': routes,
                    'message': f'找到 {len(routes)} 個路由'
                })
            except Exception as e:
                self.logger.error(f"獲取路由列表失敗: {e}", exc_info=True)
                return jsonify({
                    'success': False,
                    'message': f'獲取路由列表失敗: {str(e)}'
                })
    
    def register_socketio_events(self):
        """註冊SocketIO事件"""
        
        @self.socketio.on('connect')
        def handle_connect():
            """客戶端連接"""
            self.logger.info("Web客戶端已連接")
            emit('status_update', self.get_current_status())
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            """客戶端斷開"""
            self.logger.info("Web客戶端已斷開")
        
        @self.socketio.on('request_status')
        def handle_status_request():
            """狀態請求"""
            self.logger.debug("收到狀態請求")
            emit('status_update', self.get_current_status())
    
    def connect_modbus_server(self) -> Dict[str, Any]:
        """連接到主Modbus TCP服務器"""
        try:
            if self.modbus_client:
                self.modbus_client.close()
                self.logger.debug("關閉現有Modbus連接")
            
            server_config = self.config['tcp_server']
            self.logger.info(f"正在連接主Modbus服務器: {server_config['host']}:{server_config['port']}")
            
            self.modbus_client = ModbusTcpClient(
                host=server_config['host'],
                port=server_config['port'],
                timeout=server_config['timeout']
            )
            
            if self.modbus_client.connect():
                self.connected_to_server = True
                self.logger.info(f"連接到主Modbus服務器成功: {server_config['host']}:{server_config['port']}")
                
                return {
                    'success': True,
                    'message': '主Modbus服務器連接成功',
                    'server_info': server_config
                }
            else:
                self.connected_to_server = False
                self.logger.warning("主Modbus服務器連接失敗")
                return {
                    'success': False,
                    'message': '主Modbus服務器連接失敗'
                }
                
        except Exception as e:
            self.connected_to_server = False
            self.logger.error(f"連接主Modbus服務器失敗: {e}", exc_info=True)
            return {
                'success': False,
                'message': f'連接失敗: {str(e)}'
            }
    
    def disconnect_modbus_server(self) -> Dict[str, Any]:
        """斷開Modbus TCP連接"""
        try:
            self.stop_monitoring()
            
            if self.modbus_client:
                self.modbus_client.close()
                self.modbus_client = None
                self.logger.debug("Modbus Client對象已關閉")
            
            self.connected_to_server = False
            self.logger.info("主Modbus服務器連接已斷開")
            
            return {
                'success': True,
                'message': '主Modbus服務器連接已斷開'
            }
            
        except Exception as e:
            self.logger.error(f"斷開連接失敗: {e}", exc_info=True)
            return {
                'success': False,
                'message': f'斷開連接失敗: {str(e)}'
            }
    
    def read_register(self, address: int) -> Optional[int]:
        """讀取寄存器"""
        if not self.connected_to_server or not self.modbus_client:
            return None
        
        try:
            result = self.modbus_client.read_holding_registers(
                address, count=1, slave=self.config['tcp_server']['unit_id']
            )
            
            if not result.isError():
                value = result.registers[0]
                # 高頻操作使用條件日誌
                if address == self.status_registers['module_status'] and self.status_update_counter % 300 == 0:
                    self.logger.debug(f"讀取寄存器 {address}: {value}")
                return value
            else:
                self.logger.warning(f"讀取寄存器 {address} 失敗: {result}")
                return None
                
        except Exception as e:
            self.logger.error(f"讀取寄存器 {address} 異常: {e}", exc_info=True)
            self.connected_to_server = False
            return None
    
    def write_register(self, address: int, value: int) -> bool:
        """寫入寄存器"""
        if not self.connected_to_server or not self.modbus_client:
            self.logger.warning(f"寫入寄存器失敗 - 連接狀態: {self.connected_to_server}")
            return False
        
        try:
            result = self.modbus_client.write_register(
                address, value, slave=self.config['tcp_server']['unit_id']
            )
            
            if not result.isError():
                self.logger.debug(f"寫入寄存器 {address}: {value}")
                return True
            else:
                self.logger.warning(f"寫入寄存器 {address} 失敗: {result}")
                return False
                
        except Exception as e:
            self.logger.error(f"寫入寄存器 {address} 異常: {e}", exc_info=True)
            self.connected_to_server = False
            return False
    
    def send_command(self, command: str, param1: int = 0, param2: int = 0, param3: int = 0) -> Dict[str, Any]:
        """發送指令到VP_main模組 - 支援頻率參數"""
        if not self.connected_to_server:
            self.logger.warning(f"發送指令失敗 - 主服務器未連接: {command}")
            return {
                'success': False,
                'message': '主服務器未連接'
            }
        
        if command not in self.command_map:
            self.logger.error(f"未知指令: {command}")
            return {
                'success': False,
                'message': f'未知指令: {command}'
            }
        
        try:
            command_code = self.command_map[command]
            self.command_id_counter += 1
            
            # 寫入指令寄存器 (狀態機交握)
            write_results = []
            write_results.append(self.write_register(self.command_registers['command_code'], command_code))
            write_results.append(self.write_register(self.command_registers['param1'], param1))
            write_results.append(self.write_register(self.command_registers['param2'], param2))
            write_results.append(self.write_register(self.command_registers['param3'], param3))
            write_results.append(self.write_register(self.command_registers['command_id'], self.command_id_counter))
            
            success = all(write_results)
            
            if success:
                self.logger.info(f"發送指令成功: {command} (code={command_code}, p1={param1}, p2={param2}, p3={param3}, id={self.command_id_counter})")
                return {
                    'success': True,
                    'message': f'指令 {command} 發送成功',
                    'command': command,
                    'command_code': command_code,
                    'param1': param1,
                    'param2': param2,
                    'param3': param3,
                    'command_id': self.command_id_counter
                }
            else:
                failed_writes = [i for i, result in enumerate(write_results) if not result]
                self.logger.error(f"指令 {command} 發送失敗，寫入失敗的寄存器: {failed_writes}")
                return {
                    'success': False,
                    'message': f'指令 {command} 發送失敗，寫入失敗的寄存器: {failed_writes}'
                }
                
        except Exception as e:
            self.logger.error(f"發送指令異常: {command} - {e}", exc_info=True)
            return {
                'success': False,
                'message': f'發送指令異常: {str(e)}'
            }
    
    def start_monitoring(self):
        """開始狀態監控"""
        if self.monitoring:
            self.logger.debug("狀態監控已在運行中")
            return
        
        self.monitoring = True
        self.status_monitor_thread = threading.Thread(target=self.status_monitor_loop, daemon=True)
        self.status_monitor_thread.start()
        self.logger.info("狀態監控執行緒已啟動")
    
    def stop_monitoring(self):
        """停止狀態監控"""
        if not self.monitoring:
            self.logger.debug("狀態監控未在運行")
            return
            
        self.monitoring = False
        if self.status_monitor_thread and self.status_monitor_thread.is_alive():
            self.status_monitor_thread.join(timeout=1)
            if self.status_monitor_thread.is_alive():
                self.logger.warning("狀態監控執行緒停止超時")
            else:
                self.logger.info("狀態監控執行緒已停止")
        else:
            self.logger.info("狀態監控執行緒已停止")
    
    def status_monitor_loop(self):
        """狀態監控循環"""
        self.logger.info("狀態監控循環開始")
        
        while self.monitoring:
            try:
                self.status_update_counter += 1
                
                if self.connected_to_server:
                    # 檢查連接狀態 - 使用條件日誌避免日誌爆炸
                    test_read = self.read_register(self.status_registers['module_status'])
                    if test_read is None:
                        self.connected_to_server = False
                        self.logger.warning("主Modbus服務器連接已斷開")
                    
                    # 發送狀態更新 - 每10秒記錄一次統計
                    status = self.get_current_status()
                    self.socketio.emit('status_update', status)
                    
                    # 條件日誌：每60次更新（約1分鐘）記錄一次狀態
                    if self.status_update_counter % 60 == 0:
                        self.logger.debug(f"狀態監控正常運行 - 更新計數: {self.status_update_counter}")
                
                time.sleep(1)  # 1秒更新一次
                
            except Exception as e:
                self.logger.error(f"狀態監控異常: {e}", exc_info=True)
                time.sleep(2)
        
        self.logger.info("狀態監控循環結束")
    
    def get_current_status(self) -> Dict[str, Any]:
        """獲取當前狀態"""
        status = {
            'connected_to_server': self.connected_to_server,
            'config': self.config,
            'timestamp': datetime.now().isoformat(),
            'register_mapping': {
                'base_address': self.base_address,
                'status_registers': self.status_registers,
                'command_registers': self.command_registers
            },
            'vp_module_status': None
        }
        
        if self.connected_to_server:
            try:
                # 讀取VP模組狀態
                vp_status = {}
                for name, addr in self.status_registers.items():
                    value = self.read_register(addr)
                    vp_status[name] = value
                
                # 讀取指令寄存器狀態
                command_status = {}
                for name, addr in self.command_registers.items():
                    value = self.read_register(addr)
                    command_status[name] = value
                
                status['vp_module_status'] = vp_status
                status['command_status'] = command_status
                
                # 條件日誌：只在特定條件下記錄詳細狀態
                if self.status_update_counter % 100 == 0:
                    self.logger.debug(f"VP模組狀態更新 - 模組狀態: {vp_status.get('module_status')}")
                
            except Exception as e:
                self.logger.error(f"獲取VP模組狀態失敗: {e}", exc_info=True)
                status['connected_to_server'] = False
                self.connected_to_server = False
        
        return status
    
    def create_templates_directory(self):
        """創建templates目錄"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        templates_dir = os.path.join(current_dir, 'templates')
        if not os.path.exists(templates_dir):
            os.makedirs(templates_dir)
            self.logger.info(f"已創建templates目錄: {templates_dir}")
        else:
            self.logger.debug(f"templates目錄已存在: {templates_dir}")
    
    def run(self):
        """運行Web應用"""
        self.logger.info("震動盤Web控制應用啟動中...")
        
        # 創建templates目錄
        self.create_templates_directory()
        
        # 自動連接主服務器
        self.logger.info("嘗試自動連接主服務器...")
        result = self.connect_modbus_server()
        if result['success']:
            self.start_monitoring()
        
        web_config = self.config['web_server']
        self.logger.info(f"Web服務器啟動 - http://{web_config['host']}:{web_config['port']}")
        self.logger.info(f"主Modbus服務器: {self.config['tcp_server']['host']}:{self.config['tcp_server']['port']}")
        self.logger.info(f"VP模組基地址: {self.base_address}")
        self.logger.info("架構: VP_app → 主Modbus服務器 → VP_main → 震動盤(192.168.1.7:1000)")
        self.logger.info("功能列表:")
        self.logger.info("  - VP_main模組寄存器監控")
        self.logger.info("  - 震動動作控制 (11種震動模式)")
        self.logger.info("  - 強度/頻率調整")
        self.logger.info("  - 停止功能 (指令3→VP_main→震動盤寄存器4)")
        self.logger.info("  - 背光控制 (亮度調節/開關)")
        self.logger.info("  - 錯誤重置")
        self.logger.info("按 Ctrl+C 停止應用")
        
        try:
            self.socketio.run(
                self.app,
                host=web_config['host'],
                port=web_config['port'],
                debug=web_config['debug'],
                allow_unsafe_werkzeug=True
            )
        except Exception as e:
            self.logger.error(f"Web服務器啟動失敗: {e}", exc_info=True)
        finally:
            self.cleanup()
    
    def cleanup(self):
        """清理資源"""
        self.logger.info("正在清理資源...")
        self.stop_monitoring()
        if self.modbus_client:
            try:
                self.modbus_client.close()
                self.logger.info("主Modbus連接已安全斷開")
            except Exception as e:
                self.logger.error(f"關閉Modbus連接時發生錯誤: {e}", exc_info=True)
        self.logger.info("資源清理完成")


def create_index_html():
    """創建index.html檔案 (如果不存在)"""
    logger = logging.getLogger('VP_WebApp')
    current_dir = os.path.dirname(os.path.abspath(__file__))
    templates_dir = os.path.join(current_dir, 'templates')
    index_path = os.path.join(templates_dir, 'index.html')
    
    if not os.path.exists(templates_dir):
        os.makedirs(templates_dir)
        logger.info(f"已創建templates目錄: {templates_dir}")
    
    if not os.path.exists(index_path):
        logger.warning(f"未找到HTML模板檔案: {index_path}")
        logger.warning("請確保將 index.html 檔案放置在 templates/ 目錄中")
        return False
    
    logger.debug(f"HTML模板檔案存在: {index_path}")
    return True


def main():
    """主函數"""
    # 設置主程序logger
    logger = setup_logging('VP_WebApp_Main')
    
    logger.info("=" * 60)
    logger.info("震動盤Web控制應用 (純Modbus TCP Client)")
    logger.info("=" * 60)
    
    # 檢查HTML模板
    if not create_index_html():
        logger.warning("HTML模板檔案缺失，Web介面可能無法正常顯示")
        logger.info("繼續啟動應用...")
    
    # 創建應用實例
    app = VibrationPlateWebApp()
    
    try:
        # 運行應用
        app.run()
    except KeyboardInterrupt:
        logger.info("收到中斷信號，正在關閉...")
    except Exception as e:
        logger.critical(f"應用運行異常: {e}", exc_info=True)
    finally:
        app.cleanup()
        logger.info("應用已安全關閉")


if __name__ == '__main__':
    main()