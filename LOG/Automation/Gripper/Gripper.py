# Gripper.py - 夾爪主模組 (logging完整轉換版本)
import os
import json
import time
import threading
from datetime import datetime
from pymodbus.client import ModbusSerialClient, ModbusTcpClient
from pymodbus.exceptions import ModbusException
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

class GripperModule:
    def __init__(self, config_file="gripper_config.json"):
        # 初始化日誌系統
        self.logger = setup_logging("GripperModule")
        
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(self.current_dir, config_file)
        self.config = self.load_config()
        
        # 連接狀態
        self.main_server_client = None
        self.rtu_client = None
        self.is_running = False
        
        # 線程鎖 - 使用更輕量的鎖
        self.command_lock = threading.RLock()
        
        # 夾爪狀態 - 簡化數據結構
        self.gripper_states = {
            'PGC': {'connected': False, 'error_count': 0, 'last_status': None},
            'PGHL': {'connected': False, 'error_count': 0, 'last_status': None},
            'PGE': {'connected': False, 'error_count': 0, 'last_status': None}
        }
        
        # 指令ID追蹤 - 優化為更快的查找
        self.last_command_ids = {'PGC': 0, 'PGHL': 0, 'PGE': 0}
        self.command_in_progress = {'PGC': False, 'PGHL': False, 'PGE': False}
        
        # 性能監控 - 簡化統計
        self.performance_stats = {
            'total_commands': 0,
            'avg_response_time': 0.0,
            'last_update': time.time()
        }
        
        # 寄存器基地址配置
        self.register_mapping = {
            'PGC': {'status_base': 500, 'command_base': 520, 'unit_id': 6},
            'PGHL': {'status_base': 530, 'command_base': 550, 'unit_id': 5},
            'PGE': {'status_base': 560, 'command_base': 580, 'unit_id': 4}
        }
        
        # 預分配寄存器緩存 - 避免重複創建對象
        self.register_cache = {}
        for gripper in ['PGC', 'PGHL', 'PGE']:
            self.register_cache[gripper] = [0] * 20
        
        # 循環計數器 - 用於控制日誌頻率
        self.loop_count = 0
        
        self.logger.info("夾爪模組啟動 (增強優化版本) - 基地址: 500-589")

    def load_config(self):
        default_config = {
            "module_id": "夾爪模組",
            "rtu_connection": {
                "port": "COM5",
                "baudrate": 115200,
                "parity": "N",
                "stopbits": 1,
                "timeout": 0.3  # 進一步優化: 減少到0.3秒
            },
            "tcp_server": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 0.3  # 進一步優化: 減少到0.3秒
            },
            "timing": {
                "fast_loop_interval": 0.01,    # 超級優化: 10ms循環
                "command_delay": 0.001,        # 超級優化: 1ms延遲
                "status_update_interval": 5,   # 每5個循環更新一次狀態
                "connection_test_interval": 100 # 每100個循環測試一次連接
            },
            "grippers": {
                "PGC": {"unit_id": 6, "enabled": True},
                "PGHL": {"unit_id": 5, "enabled": True}, 
                "PGE": {"unit_id": 4, "enabled": True}
            },
            "ultra_optimization": {
                "immediate_response": True,     # 立即響應模式
                "complete_status_update": True, # 完整狀態更新 (新增)
                "command_priority": True,      # 指令優先處理
                "cache_enabled": True,         # 啟用緩存機制
                "parallel_processing": False   # 並行處理(謹慎使用)
            }
        }
        
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    loaded_config = json.load(f)
                    # 合併配置，確保新的優化選項存在
                    for key, value in default_config.items():
                        if key not in loaded_config:
                            loaded_config[key] = value
                        elif isinstance(value, dict):
                            for subkey, subvalue in value.items():
                                if subkey not in loaded_config[key]:
                                    loaded_config[key][subkey] = subvalue
                    self.logger.info("配置檔案載入成功")
                    return loaded_config
            except Exception as e:
                self.logger.error(f"配置檔案讀取錯誤: {e}", exc_info=True)
                return default_config
        else:
            try:
                with open(self.config_file, 'w', encoding='utf-8') as f:
                    json.dump(default_config, f, indent=2, ensure_ascii=False)
                self.logger.info("已建立預設配置檔案")
            except Exception as e:
                self.logger.error(f"建立配置檔案失敗: {e}", exc_info=True)
            return default_config

    def connect_main_server(self):
        """優化的主服務器連接"""
        if self.main_server_client and self.main_server_client.connected:
            return True
            
        try:
            self.logger.info(f"正在連接 {self.config['tcp_server']['host']}:{self.config['tcp_server']['port']}")
            
            self.main_server_client = ModbusTcpClient(
                host=self.config["tcp_server"]["host"],
                port=self.config["tcp_server"]["port"],
                timeout=self.config["tcp_server"]["timeout"]
            )
            
            connected = self.main_server_client.connect()
            if connected:
                self.logger.info("主服務器連接建立成功")
            else:
                self.logger.warning("主服務器連接失敗，將在主循環中重試")
            
            return connected
        except Exception as e:
            self.logger.error(f"主服務器連接異常: {e}", exc_info=True)
            return False

    def connect_rtu_devices(self):
        """優化的RTU設備連接"""
        if self.rtu_client and self.rtu_client.connected:
            return True
            
        try:
            self.logger.info(f"正在連接RTU設備 {self.config['rtu_connection']['port']}")
            
            self.rtu_client = ModbusSerialClient(
                port=self.config["rtu_connection"]["port"],
                baudrate=self.config["rtu_connection"]["baudrate"],
                parity=self.config["rtu_connection"]["parity"],
                stopbits=self.config["rtu_connection"]["stopbits"],
                timeout=self.config["rtu_connection"]["timeout"]
            )
            
            connected = self.rtu_client.connect()
            if connected:
                self.logger.info("RTU設備連接建立成功")
            else:
                self.logger.warning("RTU設備連接失敗，將在主循環中重試")
            
            return connected
        except Exception as e:
            self.logger.error(f"RTU設備連接異常: {e}", exc_info=True)
            return False

    def execute_pgc_command_ultra(self, command, param1):
        """PGC超級優化指令執行"""
        unit_id = 6  # 直接硬編碼避免查找
        
        try:
            # 根據指令類型直接執行，避免多重判斷
            if command == 1:  # 初始化
                result = self.rtu_client.write_register(address=0x0100, value=0x01, slave=unit_id)
                success = not result.isError()
            elif command == 3:  # 設定位置
                result = self.rtu_client.write_register(address=0x0103, value=param1, slave=unit_id)
                success = not result.isError()
            elif command == 5:  # 設定力道
                result = self.rtu_client.write_register(address=0x0101, value=param1, slave=unit_id)
                success = not result.isError()
            elif command == 6:  # 設定速度
                result = self.rtu_client.write_register(address=0x0104, value=param1, slave=unit_id)
                success = not result.isError()
            elif command == 7:  # 開啟
                result = self.rtu_client.write_register(address=0x0103, value=1000, slave=unit_id)
                success = not result.isError()
            elif command == 8:  # 關閉
                result = self.rtu_client.write_register(address=0x0103, value=0, slave=unit_id)
                success = not result.isError()
            else:
                success = True
                
            if not success:
                self.logger.warning(f"PGC指令執行失敗: cmd={command}, param1={param1}")
            
            return success
        except Exception as e:
            self.logger.error(f"PGC指令執行異常: cmd={command}, param1={param1}, error={e}", exc_info=True)
            return False

    def read_pgc_status_complete(self):
        """讀取PGC完整關鍵狀態 (增強版)"""
        try:
            # 讀取初始化狀態 (0x0200)
            init_result = self.rtu_client.read_holding_registers(address=0x0200, count=1, slave=6)
            init_status = init_result.registers[0] if not init_result.isError() else 0
            
            # 讀取夾持狀態 (0x0201) 
            grip_result = self.rtu_client.read_holding_registers(address=0x0201, count=1, slave=6)
            grip_status = grip_result.registers[0] if not grip_result.isError() else 0
            
            # 讀取位置反饋 (0x0202)
            pos_result = self.rtu_client.read_holding_registers(address=0x0202, count=1, slave=6)
            position = pos_result.registers[0] if not pos_result.isError() else 0
            
            return init_status, grip_status, position
        except Exception as e:
            self.logger.error(f"讀取PGC完整狀態錯誤: {e}", exc_info=True)
            return 0, 0, 0

    def read_pgc_status_minimal(self):
        """PGC最小化狀態讀取 (保留向後兼容)"""
        try:
            # 只讀取關鍵狀態寄存器 - 使用正確的PyModbus 3.x語法
            result = self.rtu_client.read_holding_registers(address=0x0201, count=1, slave=6)
            return result.registers[0] if not result.isError() else 0
        except Exception as e:
            self.logger.debug(f"PGC最小化狀態讀取失敗: {e}")
            return 0

    def ultra_fast_command_processor(self):
        """超快速指令處理器 - 專門處理PGC"""
        try:
            if not self.main_server_client or not self.main_server_client.connected:
                return
            
            # 只處理PGC夾爪 (520-523) - 使用正確的PyModbus 3.x語法
            result = self.main_server_client.read_holding_registers(address=520, count=4, slave=1)
            if result.isError():
                return
                
            command_id = result.registers[3]
            
            # 檢查新指令
            if command_id != 0 and command_id != self.last_command_ids['PGC']:
                command = result.registers[0]
                param1 = result.registers[1]
                
                start_time = time.time()
                
                self.logger.debug(f"超快速PGC指令: cmd={command}, param1={param1}, id={command_id}")
                
                # 標記指令進行中
                self.command_in_progress['PGC'] = True
                self.last_command_ids['PGC'] = command_id
                
                # 立即執行指令
                success = self.execute_pgc_command_ultra(command, param1)
                
                # 立即清除指令寄存器 - 使用正確的PyModbus 3.x語法
                self.main_server_client.write_registers(address=520, values=[0, 0, 0, 0], slave=1)
                
                response_time = (time.time() - start_time) * 1000
                
                if success:
                    self.logger.info(f"PGC指令執行成功 - 響應時間: {response_time:.1f}ms")
                else:
                    self.logger.warning(f"PGC指令執行失敗 - 響應時間: {response_time:.1f}ms")
                    self.gripper_states['PGC']['error_count'] += 1
                
                # 更新性能統計
                self.performance_stats['total_commands'] += 1
                self.performance_stats['avg_response_time'] = (
                    self.performance_stats['avg_response_time'] * 0.9 + response_time * 0.1
                )
                
                self.command_in_progress['PGC'] = False
                
        except Exception as e:
            self.logger.error(f"超快速指令處理錯誤: {e}", exc_info=True)
            self.command_in_progress['PGC'] = False

    def enhanced_status_update(self):
        """增強版狀態更新 - 包含完整關鍵狀態"""
        try:
            if not self.main_server_client or not self.main_server_client.connected:
                return
            
            # 讀取PGC完整狀態
            init_status, grip_status, position = self.read_pgc_status_complete()
            
            # 使用預分配的緩存，更新所有關鍵寄存器
            registers = self.register_cache['PGC']
            registers[0] = 1 if self.gripper_states['PGC']['connected'] else 0  # 寄存器500: 模組狀態
            registers[1] = 1 if self.gripper_states['PGC']['connected'] else 0  # 寄存器501: 連接狀態
            registers[2] = init_status                                          # 寄存器502: 設備狀態(初始化狀態) ← 重要！
            registers[3] = self.gripper_states['PGC']['error_count']           # 寄存器503: 錯誤計數
            registers[4] = grip_status                                         # 寄存器504: 夾持狀態
            registers[5] = position                                            # 寄存器505: 當前位置 ← 重要！
            registers[14] = int(time.time()) % 65535                          # 寄存器514: 時間戳
            
            # 快速寫入關鍵寄存器 - 使用正確的PyModbus 3.x語法
            self.main_server_client.write_registers(address=500, values=registers[:20], slave=1)
            
            # 除錯輸出 (降低頻率) - 每500次循環記錄一次
            if self.loop_count % 500 == 0:
                self.logger.debug(f"PGC狀態更新: init={init_status}, grip={grip_status}, pos={position}")
                
        except Exception as e:
            self.logger.error(f"增強狀態更新錯誤: {e}", exc_info=True)

    def minimal_status_update(self):
        """最小化狀態更新 - 只更新PGC關鍵狀態 (保留向後兼容)"""
        try:
            if not self.main_server_client or not self.main_server_client.connected:
                return
            
            # 只處理PGC夾爪狀態
            if self.config.get("ultra_optimization", {}).get("minimal_status_check", True):
                # 只讀取夾持狀態，不讀取完整狀態
                grip_status = self.read_pgc_status_minimal()
                
                # 使用預分配的緩存
                registers = self.register_cache['PGC']
                registers[0] = 1 if self.gripper_states['PGC']['connected'] else 0  # 模組狀態
                registers[1] = 1 if self.gripper_states['PGC']['connected'] else 0  # 連接狀態
                registers[4] = grip_status  # 夾持狀態
                registers[3] = self.gripper_states['PGC']['error_count']  # 錯誤計數
                
                # 快速寫入 - 使用正確的PyModbus 3.x語法
                self.main_server_client.write_registers(address=500, values=registers, slave=1)
                
        except Exception as e:
            self.logger.error(f"最小化狀態更新錯誤: {e}", exc_info=True)

    def ultra_fast_loop(self):
        """超級優化主循環"""
        self.logger.info("主循環執行緒已啟動")
        
        self.loop_count = 0
        status_interval = self.config["timing"].get("status_update_interval", 5)
        connection_interval = self.config["timing"].get("connection_test_interval", 100)
        
        # 檢查是否啟用完整狀態更新
        use_complete_status = self.config.get("ultra_optimization", {}).get("complete_status_update", True)
        
        # 記錄開始時間用於性能統計
        start_time = time.time()
        
        while self.is_running:
            try:
                loop_start = time.time()
                
                # 優先連接檢查 (降低頻率)
                if self.loop_count % connection_interval == 0:
                    if not self.connect_main_server() or not self.connect_rtu_devices():
                        time.sleep(0.1)
                        self.loop_count += 1
                        continue
                    
                    # 快速連接測試 PGC - 使用正確的PyModbus 3.x語法
                    try:
                        result = self.rtu_client.read_holding_registers(address=0x0200, count=1, slave=6)
                        self.gripper_states['PGC']['connected'] = not result.isError()
                    except Exception as e:
                        self.gripper_states['PGC']['connected'] = False
                        self.logger.debug(f"PGC連接測試失敗: {e}")
                
                # 最高優先級: 指令處理
                self.ultra_fast_command_processor()
                
                # 狀態更新 (降低頻率)
                if self.loop_count % status_interval == 0:
                    if use_complete_status:
                        self.enhanced_status_update()    # 使用增強版狀態更新
                    else:
                        self.minimal_status_update()     # 使用最小化狀態更新
                
                self.loop_count += 1
                
                # 性能統計輸出 (降低頻率) - 每2000次循環記錄一次
                if self.loop_count % 2000 == 0:
                    status_mode = "完整" if use_complete_status else "最小化"
                    runtime = time.time() - start_time
                    self.logger.info(f"性能統計 - 總指令: {self.performance_stats['total_commands']}, "
                                   f"平均響應: {self.performance_stats['avg_response_time']:.1f}ms, "
                                   f"狀態模式: {status_mode}, 運行時間: {runtime:.1f}秒")
                
                # 精確的循環時間控制
                elapsed = time.time() - loop_start
                sleep_time = self.config["timing"]["fast_loop_interval"] - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except KeyboardInterrupt:
                self.logger.info("收到中斷信號，停止主循環")
                break
            except Exception as e:
                self.logger.error(f"主循環錯誤: {e}", exc_info=True)
                time.sleep(0.01)
        
        self.logger.info("主循環執行緒已停止")

    def start(self):
        """啟動增強優化模組"""
        self.is_running = True
        
        # 啟動增強優化主循環
        self.main_thread = threading.Thread(target=self.ultra_fast_loop, daemon=True)
        self.main_thread.start()
        
        use_complete_status = self.config.get("ultra_optimization", {}).get("complete_status_update", True)
        status_mode = "完整狀態更新" if use_complete_status else "最小狀態更新"
        
        self.logger.info("夾爪模組已啟動 (增強優化版本)")
        self.logger.info(f"循環間隔: {self.config['timing']['fast_loop_interval']*1000}ms")
        self.logger.info(f"指令延遲: {self.config['timing']['command_delay']*1000}ms")
        self.logger.info(f"狀態更新模式: {status_mode}")
        self.logger.info("專注於PGC夾爪超快速響應 + 完整狀態更新")
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """停止模組"""
        self.logger.info("正在停止夾爪模組...")
        self.is_running = False
        
        # 等待主循環結束
        if hasattr(self, 'main_thread') and self.main_thread:
            self.main_thread.join(timeout=2.0)
            if self.main_thread.is_alive():
                self.logger.warning("主循環執行緒強制結束")
        
        # 關閉連接
        if self.main_server_client:
            try:
                self.main_server_client.close()
                self.logger.info("主服務器連接已關閉")
            except Exception as e:
                self.logger.error(f"關閉主服務器連接時發生錯誤: {e}", exc_info=True)
                
        if self.rtu_client:
            try:
                self.rtu_client.close()
                self.logger.info("RTU設備連接已關閉")
            except Exception as e:
                self.logger.error(f"關閉RTU設備連接時發生錯誤: {e}", exc_info=True)
            
        self.logger.info("夾爪模組已停止")

if __name__ == "__main__":
    gripper_module = GripperModule()
    gripper_module.start()