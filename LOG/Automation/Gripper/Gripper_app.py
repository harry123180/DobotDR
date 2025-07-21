# Gripper_app.py - 夾爪Web控制應用 (Logging版本)
import os
import json
import time
import threading
import logging
from logging.handlers import RotatingFileHandler
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException

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

class GripperWebApp:
    def __init__(self, config_file="gripper_app_config.json"):
        # 設置logging
        self.logger = setup_logging("GripperWebApp")
        
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(self.current_dir, config_file)
        
        try:
            self.config = self.load_config()
            self.logger.info("配置檔案載入成功")
        except Exception as e:
            self.logger.error(f"配置檔案載入失敗: {e}", exc_info=True)
            raise
        
        # Flask應用初始化
        try:
            self.app = Flask(__name__)
            self.app.config['SECRET_KEY'] = 'gripper_secret_key'
            self.socketio = SocketIO(self.app, cors_allowed_origins="*")
            self.logger.info("Flask應用初始化完成")
        except Exception as e:
            self.logger.error(f"Flask應用初始化失敗: {e}", exc_info=True)
            raise
        
        # Modbus客戶端
        self.modbus_client = None
        self.is_connected = False
        
        # 監控線程控制
        self.monitoring_active = False
        self.monitor_thread = None
        self.status_update_counter = 0  # 用於控制日誌頻率
        
        # 寄存器映射
        self.register_mapping = {
            'PGC': {'status_base': 500, 'command_base': 520},
            'PGHL': {'status_base': 530, 'command_base': 550},
            'PGE': {'status_base': 560, 'command_base': 580}
        }
        
        # 設置路由和SocketIO
        try:
            self.setup_routes()
            self.setup_socketio()
            self.logger.info("路由和SocketIO設置完成")
        except Exception as e:
            self.logger.error(f"路由設置失敗: {e}", exc_info=True)
            raise
        
        self.logger.info("夾爪Web控制應用啟動中...")
        self.logger.info(f"Modbus服務器地址: {self.config['modbus_tcp']['host']}:{self.config['modbus_tcp']['port']}")
        self.logger.info("夾爪寄存器映射:")
        for gripper, mapping in self.register_mapping.items():
            self.logger.info(f"  {gripper}: 狀態 {mapping['status_base']}-{mapping['status_base']+19}, 指令 {mapping['command_base']}-{mapping['command_base']+9}")

    def load_config(self):
        default_config = {
            "module_id": "夾爪Web UI",
            "modbus_tcp": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 1.0
            },
            "web_server": {
                "host": "0.0.0.0",
                "port": 5054,
                "debug": False
            },
            "ui_settings": {
                "refresh_interval": 3.0,
                "manual_mode": False
            },
            "grippers": {
                "PGC": {
                    "name": "PGC夾爪",
                    "enabled": True,
                    "positions": {"open": 1000, "close": 0},
                    "max_force": 100,
                    "max_speed": 100
                },
                "PGHL": {
                    "name": "PGHL夾爪", 
                    "enabled": True,
                    "positions": {"open": 5000, "close": 0},
                    "max_force": 100,
                    "max_speed": 100
                },
                "PGE": {
                    "name": "PGE夾爪",
                    "enabled": True,
                    "positions": {"open": 1000, "close": 0},
                    "max_force": 100,
                    "max_speed": 100
                }
            }
        }
        
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    config = json.load(f)
                    self.logger.info(f"配置檔案讀取成功: {self.config_file}")
                    return config
            except Exception as e:
                self.logger.error(f"配置檔案讀取錯誤: {e}", exc_info=True)
                self.logger.warning("使用預設配置")
                return default_config
        else:
            try:
                with open(self.config_file, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                self.logger.info(f"創建預設配置檔案: {self.config_file}")
                return default_config
            except Exception as e:
                self.logger.error(f"創建配置檔案失敗: {e}", exc_info=True)
                return default_config

    def connect_modbus(self):
        try:
            if self.modbus_client and self.modbus_client.connected:
                self.logger.debug("Modbus已連接，跳過重複連接")
                return True
            
            host = self.config["modbus_tcp"]["host"]
            port = self.config["modbus_tcp"]["port"]
            timeout = self.config["modbus_tcp"]["timeout"]
            
            self.logger.info(f"正在連接Modbus服務器: {host}:{port}")
            
            self.modbus_client = ModbusTcpClient(
                host=host,
                port=port,
                timeout=timeout
            )
            
            if self.modbus_client.connect():
                self.is_connected = True
                self.logger.info(f"Modbus連接建立成功: {host}:{port}")
                return True
            else:
                self.is_connected = False
                self.logger.warning(f"Modbus連接失敗: {host}:{port}")
                return False
                
        except Exception as e:
            self.logger.error(f"Modbus連接異常: {e}", exc_info=True)
            self.is_connected = False
            return False

    def read_gripper_status(self, gripper_type):
        """讀取夾爪狀態"""
        try:
            if not self.is_connected:
                self.logger.debug(f"Modbus未連接，跳過{gripper_type}狀態讀取")
                return None
                
            status_base = self.register_mapping[gripper_type]['status_base']
            unit_id = self.config["modbus_tcp"]["unit_id"]
            
            result = self.modbus_client.read_holding_registers(
                address=status_base,
                count=20,
                slave=unit_id
            )
            
            if result.isError():
                self.logger.warning(f"讀取{gripper_type}狀態寄存器失敗: {result}")
                return None
                
            registers = result.registers
            
            status_data = {
                'module_status': registers[0],
                'connected': bool(registers[1]),
                'device_status': registers[2],
                'error_count': registers[3],
                'grip_status': registers[4],
                'position': registers[5],
                'current': registers[6] if len(registers) > 6 else 0,
                'timestamp': registers[14] if len(registers) > 14 else 0
            }
            
            # 每10次才記錄一次狀態讀取成功（避免日誌過多）
            if self.status_update_counter % 10 == 0:
                self.logger.debug(f"{gripper_type}狀態讀取成功: 位置={status_data['position']}, 狀態={status_data['grip_status']}")
            
            return status_data
            
        except Exception as e:
            self.logger.error(f"讀取{gripper_type}狀態異常: {e}", exc_info=True)
            return None

    def send_gripper_command(self, gripper_type, command, param1=0, param2=0):
        """發送夾爪指令"""
        try:
            if not self.is_connected:
                self.logger.warning(f"Modbus未連接，無法發送{gripper_type}指令")
                return False
                
            command_base = self.register_mapping[gripper_type]['command_base']
            command_id = int(time.time() * 1000) % 65535  # 生成唯一指令ID
            unit_id = self.config["modbus_tcp"]["unit_id"]
            
            values = [command, param1, param2, command_id, 0, 0, 0, 0, 0, 0]
            
            self.logger.info(f"發送{gripper_type}指令: 指令={command}, 參數1={param1}, 參數2={param2}, ID={command_id}")
            
            result = self.modbus_client.write_registers(
                address=command_base,
                values=values,
                slave=unit_id
            )
            
            success = not result.isError() if result else False
            
            if success:
                self.logger.info(f"{gripper_type}指令發送成功")
            else:
                self.logger.warning(f"{gripper_type}指令發送失敗: {result}")
            
            return success
            
        except Exception as e:
            self.logger.error(f"發送{gripper_type}指令異常: {e}", exc_info=True)
            return False

    def setup_routes(self):
        @self.app.route('/')
        def index():
            self.logger.debug("Web主頁訪問")
            return render_template('index.html', 
                                 config=self.config,
                                 register_mapping=self.register_mapping)

        @self.app.route('/api/connect', methods=['POST'])
        def connect():
            self.logger.info("收到連接請求")
            success = self.connect_modbus()
            message = 'Modbus連接成功' if success else 'Modbus連接失敗'
            self.logger.info(f"連接結果: {message}")
            return jsonify({
                'success': success,
                'message': message
            })

        @self.app.route('/api/status/<gripper_type>')
        def get_status(gripper_type):
            if gripper_type not in self.register_mapping:
                self.logger.warning(f"無效的夾爪類型請求: {gripper_type}")
                return jsonify({'error': '無效的夾爪類型'}), 400
                
            status = self.read_gripper_status(gripper_type)
            return jsonify({
                'success': status is not None,
                'status': status,
                'connected': self.is_connected
            })

        @self.app.route('/api/command/<gripper_type>', methods=['POST'])
        def send_command(gripper_type):
            if gripper_type not in self.register_mapping:
                self.logger.warning(f"無效的夾爪類型指令: {gripper_type}")
                return jsonify({'error': '無效的夾爪類型'}), 400
                
            data = request.get_json()
            command = data.get('command', 0)
            param1 = data.get('param1', 0)
            param2 = data.get('param2', 0)
            
            self.logger.info(f"收到{gripper_type}指令請求: {command}, 參數: {param1}, {param2}")
            
            success = self.send_gripper_command(gripper_type, command, param1, param2)
            message = '指令發送成功' if success else '指令發送失敗'
            
            return jsonify({
                'success': success,
                'message': message
            })

        @self.app.route('/api/initialize/<gripper_type>', methods=['POST'])
        def initialize_gripper(gripper_type):
            self.logger.info(f"收到{gripper_type}初始化請求")
            success = self.send_gripper_command(gripper_type, 1)  # 指令1: 初始化
            message = f'{gripper_type}初始化指令已發送' if success else '指令發送失敗'
            self.logger.info(f"{gripper_type}初始化結果: {message}")
            return jsonify({
                'success': success,
                'message': message
            })

        @self.app.route('/api/stop/<gripper_type>', methods=['POST'])
        def stop_gripper(gripper_type):
            self.logger.info(f"收到{gripper_type}停止請求")
            success = self.send_gripper_command(gripper_type, 2)  # 指令2: 停止
            message = f'{gripper_type}停止指令已發送' if success else '指令發送失敗'
            self.logger.info(f"{gripper_type}停止結果: {message}")
            return jsonify({
                'success': success,
                'message': message
            })

        @self.app.route('/api/move/<gripper_type>', methods=['POST'])
        def move_gripper(gripper_type):
            data = request.get_json()
            position = data.get('position', 0)
            
            self.logger.info(f"收到{gripper_type}移動請求: 位置={position}")
            success = self.send_gripper_command(gripper_type, 3, position)  # 指令3: 絕對位置
            message = f'{gripper_type}移動指令已發送' if success else '指令發送失敗'
            self.logger.info(f"{gripper_type}移動結果: {message}")
            return jsonify({
                'success': success,
                'message': message
            })

        @self.app.route('/api/set_force/<gripper_type>', methods=['POST'])
        def set_force(gripper_type):
            data = request.get_json()
            force = data.get('force', 50)
            
            self.logger.info(f"收到{gripper_type}力道設定請求: 力道={force}")
            success = self.send_gripper_command(gripper_type, 5, force)  # 指令5: 設定力道
            message = f'{gripper_type}力道設定已發送' if success else '指令發送失敗'
            self.logger.info(f"{gripper_type}力道設定結果: {message}")
            return jsonify({
                'success': success,
                'message': message
            })

        @self.app.route('/api/set_speed/<gripper_type>', methods=['POST'])
        def set_speed(gripper_type):
            data = request.get_json()
            speed = data.get('speed', 50)
            
            self.logger.info(f"收到{gripper_type}速度設定請求: 速度={speed}")
            success = self.send_gripper_command(gripper_type, 6, speed)  # 指令6: 設定速度
            message = f'{gripper_type}速度設定已發送' if success else '指令發送失敗'
            self.logger.info(f"{gripper_type}速度設定結果: {message}")
            return jsonify({
                'success': success,
                'message': message
            })

        @self.app.route('/api/open/<gripper_type>', methods=['POST'])
        def open_gripper(gripper_type):
            self.logger.info(f"收到{gripper_type}開啟請求")
            success = self.send_gripper_command(gripper_type, 7)  # 指令7: 開啟
            message = f'{gripper_type}開啟指令已發送' if success else '指令發送失敗'
            self.logger.info(f"{gripper_type}開啟結果: {message}")
            return jsonify({
                'success': success,
                'message': message
            })

        @self.app.route('/api/close/<gripper_type>', methods=['POST'])
        def close_gripper(gripper_type):
            self.logger.info(f"收到{gripper_type}關閉請求")
            success = self.send_gripper_command(gripper_type, 8)  # 指令8: 關閉
            message = f'{gripper_type}關閉指令已發送' if success else '指令發送失敗'
            self.logger.info(f"{gripper_type}關閉結果: {message}")
            return jsonify({
                'success': success,
                'message': message
            })

    def setup_socketio(self):
        @self.socketio.on('connect')
        def handle_connect():
            self.logger.info('SocketIO客戶端已連接')
            emit('status', {'connected': True})

        @self.socketio.on('disconnect')
        def handle_disconnect():
            self.logger.info('SocketIO客戶端已斷開')

        @self.socketio.on('start_monitoring')
        def handle_start_monitoring():
            self.logger.info('收到啟動監控請求')
            self.start_monitoring()
            emit('monitoring_status', {'active': True})

        @self.socketio.on('stop_monitoring')
        def handle_stop_monitoring():
            self.logger.info('收到停止監控請求')
            self.stop_monitoring()
            emit('monitoring_status', {'active': False})

        @self.socketio.on('request_status')
        def handle_request_status():
            self.logger.debug('收到狀態請求')
            self.emit_all_status()

    def start_monitoring(self):
        """啟動狀態監控"""
        try:
            if not self.monitoring_active:
                self.monitoring_active = True
                self.monitor_thread = threading.Thread(target=self.monitor_loop, daemon=True)
                self.monitor_thread.start()
                self.logger.info("狀態監控執行緒已啟動")
            else:
                self.logger.warning("狀態監控已在運行中")
        except Exception as e:
            self.logger.error(f"啟動狀態監控失敗: {e}", exc_info=True)

    def stop_monitoring(self):
        """停止狀態監控"""
        try:
            if self.monitoring_active:
                self.monitoring_active = False
                self.logger.info("狀態監控執行緒已停止")
            else:
                self.logger.debug("狀態監控已停止")
        except Exception as e:
            self.logger.error(f"停止狀態監控異常: {e}", exc_info=True)

    def monitor_loop(self):
        """監控循環"""
        self.logger.info("監控循環執行緒已啟動")
        loop_counter = 0
        
        try:
            while self.monitoring_active:
                try:
                    # 每10個循環記錄一次狀態
                    if loop_counter % 10 == 0:
                        self.logger.debug(f"監控循環運行中，計數: {loop_counter}")
                    
                    if self.connect_modbus():
                        self.emit_all_status()
                    else:
                        # 連接失敗時降低頻率記錄
                        if loop_counter % 30 == 0:  # 每30個循環記錄一次連接失敗
                            self.logger.warning("Modbus連接失敗，監控循環中將重試連接")
                    
                    loop_counter += 1
                    self.status_update_counter = loop_counter
                    
                    time.sleep(self.config["ui_settings"]["refresh_interval"])
                    
                except Exception as e:
                    self.logger.error(f"監控循環內部錯誤: {e}", exc_info=True)
                    time.sleep(1)  # 錯誤後短暫暫停
                    
        except Exception as e:
            self.logger.error(f"監控循環執行緒意外終止: {e}", exc_info=True)
        finally:
            self.logger.info("監控循環執行緒已結束")

    def emit_all_status(self):
        """發送所有夾爪狀態"""
        try:
            status_data = {}
            
            for gripper_type in ['PGC', 'PGHL', 'PGE']:
                if self.config["grippers"][gripper_type]["enabled"]:
                    status = self.read_gripper_status(gripper_type)
                    status_data[gripper_type] = status
            
            self.socketio.emit('status_update', {
                'timestamp': time.time(),
                'connected': self.is_connected,
                'grippers': status_data
            })
            
            # 每20次狀態更新記錄一次
            if self.status_update_counter % 20 == 0:
                active_grippers = [gt for gt, data in status_data.items() if data is not None]
                self.logger.debug(f"狀態更新已發送，活躍夾爪: {active_grippers}")
            
        except Exception as e:
            self.logger.error(f"狀態更新發送異常: {e}", exc_info=True)

    def run(self):
        """啟動Web應用"""
        try:
            host = self.config["web_server"]["host"]
            port = self.config["web_server"]["port"]
            debug = self.config["web_server"]["debug"]
            
            self.logger.info(f"Web服務器啟動 - http://{host}:{port}")
            
            # 嘗試連接Modbus
            if self.connect_modbus():
                self.logger.info("初始Modbus連接成功")
            else:
                self.logger.warning("初始Modbus連接失敗，將在監控循環中重試")
            
            # 啟動狀態監控
            if not self.config["ui_settings"]["manual_mode"]:
                self.start_monitoring()
                self.logger.info("自動狀態監控已啟動")
            else:
                self.logger.info("手動模式，狀態監控未自動啟動")
            
            self.logger.info("Flask SocketIO服務器即將啟動")
            
            self.socketio.run(
                self.app,
                host=host,
                port=port,
                debug=debug
            )
            
        except Exception as e:
            self.logger.critical(f"Web應用啟動失敗: {e}", exc_info=True)
            raise
        finally:
            self.logger.info("Web應用服務器已關閉")
            # 清理資源
            if self.monitoring_active:
                self.stop_monitoring()
            if self.modbus_client and self.modbus_client.connected:
                try:
                    self.modbus_client.close()
                    self.logger.info("Modbus連接已關閉")
                except Exception as e:
                    self.logger.error(f"關閉Modbus連接時發生錯誤: {e}", exc_info=True)

if __name__ == "__main__":
    try:
        app = GripperWebApp()
        app.run()
    except KeyboardInterrupt:
        app.logger.info("收到中斷信號，正在關閉應用...")
    except Exception as e:
        if 'app' in locals():
            app.logger.critical(f"應用程式致命錯誤: {e}", exc_info=True)
        else:
            print(f"應用程式初始化失敗: {e}")
        raise