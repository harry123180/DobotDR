# Gripper.py - 夾爪主模組
import os
import json
import time
import threading
from datetime import datetime
from pymodbus.client import ModbusSerialClient, ModbusTcpClient
from pymodbus.exceptions import ModbusException

class GripperModule:
    def __init__(self, config_file="gripper_config.json"):
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_file = os.path.join(self.current_dir, config_file)
        self.config = self.load_config()
        
        # 連接狀態
        self.main_server_client = None
        self.rtu_client = None
        self.is_running = False
        
        # 線程鎖
        self.state_lock = threading.Lock()
        
        # 夾爪狀態
        self.gripper_states = {
            'PGC': {'connected': False, 'last_error': 0, 'error_count': 0},
            'PGHL': {'connected': False, 'last_error': 0, 'error_count': 0},
            'PGE': {'connected': False, 'last_error': 0, 'error_count': 0}
        }
        
        # 指令ID追蹤
        self.last_command_ids = {'PGC': 0, 'PGHL': 0, 'PGE': 0}
        
        # 寄存器基地址配置
        self.register_mapping = {
            'PGC': {'status_base': 500, 'command_base': 520, 'unit_id': 6},
            'PGHL': {'status_base': 530, 'command_base': 550, 'unit_id': 5},
            'PGE': {'status_base': 560, 'command_base': 580, 'unit_id': 4}
        }
        
        print(f"夾爪模組啟動 - 基地址: 500-589")
        print("寄存器映射:")
        for gripper, mapping in self.register_mapping.items():
            print(f"  {gripper}: 狀態 {mapping['status_base']}-{mapping['status_base']+19}, 指令 {mapping['command_base']}-{mapping['command_base']+9}")

    def load_config(self):
        default_config = {
            "module_id": "夾爪模組",
            "rtu_connection": {
                "port": "COM5",
                "baudrate": 115200,
                "parity": "N",
                "stopbits": 1,
                "timeout": 1.0
            },
            "tcp_server": {
                "host": "127.0.0.1",
                "port": 502,
                "unit_id": 1,
                "timeout": 1.0
            },
            "modbus_mapping": {
                "base_address": 500
            },
            "timing": {
                "fast_loop_interval": 0.05,
                "command_delay": 0.02,
                "reconnect_interval": 5.0
            },
            "grippers": {
                "PGC": {"unit_id": 6, "enabled": True},
                "PGHL": {"unit_id": 5, "enabled": True}, 
                "PGE": {"unit_id": 4, "enabled": True}
            }
        }
        
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception as e:
                print(f"配置檔案讀取錯誤: {e}")
                return default_config
        else:
            with open(self.config_file, 'w', encoding='utf-8') as f:
                json.dump(default_config, f, indent=2, ensure_ascii=False)
            return default_config

    def connect_main_server(self):
        try:
            if self.main_server_client and self.main_server_client.connected:
                return True
                
            self.main_server_client = ModbusTcpClient(
                host=self.config["tcp_server"]["host"],
                port=self.config["tcp_server"]["port"],
                timeout=self.config["tcp_server"]["timeout"]
            )
            
            if self.main_server_client.connect():
                print(f"已連接主服務器: {self.config['tcp_server']['host']}:{self.config['tcp_server']['port']}")
                return True
            else:
                print("主服務器連接失敗")
                return False
        except Exception as e:
            print(f"主服務器連接錯誤: {e}")
            return False

    def connect_rtu_devices(self):
        try:
            if self.rtu_client and self.rtu_client.connected:
                return True
                
            self.rtu_client = ModbusSerialClient(
                port=self.config["rtu_connection"]["port"],
                baudrate=self.config["rtu_connection"]["baudrate"],
                parity=self.config["rtu_connection"]["parity"],
                stopbits=self.config["rtu_connection"]["stopbits"],
                timeout=self.config["rtu_connection"]["timeout"]
            )
            
            if self.rtu_client.connect():
                print(f"已連接RTU設備: {self.config['rtu_connection']['port']}")
                return True
            else:
                print("RTU設備連接失敗")
                return False
        except Exception as e:
            print(f"RTU設備連接錯誤: {e}")
            return False

    def test_gripper_connection(self, gripper_type):
        """測試單個夾爪連接"""
        try:
            if not self.rtu_client or not self.rtu_client.connected:
                self.gripper_states[gripper_type]['connected'] = False
                return False
                
            unit_id = self.register_mapping[gripper_type]['unit_id']
            
            # 嘗試讀取狀態寄存器，使用正確的PyModbus 3.x語法
            result = self.rtu_client.read_holding_registers(address=0x0200, count=1, slave=unit_id)
            
            if result and not result.isError():
                self.gripper_states[gripper_type]['connected'] = True
                #print(f"{gripper_type}夾爪連接正常 (unit_id={unit_id})")
                return True
            else:
                self.gripper_states[gripper_type]['connected'] = False
                print(f"{gripper_type}夾爪連接失敗 (unit_id={unit_id})")
                return False
                
        except Exception as e:
            self.gripper_states[gripper_type]['connected'] = False
            self.gripper_states[gripper_type]['error_count'] += 1
            print(f"{gripper_type}夾爪連接測試異常: {e}")
            return False

    def read_gripper_status(self, gripper_type):
        """讀取夾爪狀態"""
        try:
            if not self.rtu_client or not self.rtu_client.connected:
                return None
                
            unit_id = self.register_mapping[gripper_type]['unit_id']
            status_data = {}
            
            if gripper_type == 'PGC':
                # PGC狀態讀取 - 使用正確的PyModbus 3.x語法
                init_status = self.rtu_client.read_holding_registers(address=0x0200, count=1, slave=unit_id)
                if init_status.isError():
                    return None
                    
                grip_status = self.rtu_client.read_holding_registers(address=0x0201, count=1, slave=unit_id) 
                if grip_status.isError():
                    return None
                    
                position = self.rtu_client.read_holding_registers(address=0x0202, count=1, slave=unit_id)
                if position.isError():
                    return None
                    
                status_data = {
                    'init_status': init_status.registers[0],
                    'grip_status': grip_status.registers[0],
                    'position': position.registers[0],
                    'connected': True
                }
                
            elif gripper_type == 'PGHL':
                # PGHL狀態讀取 - 使用正確的PyModbus 3.x語法
                home_status = self.rtu_client.read_holding_registers(address=0x0200, count=1, slave=unit_id)
                if home_status.isError():
                    return None
                    
                running_status = self.rtu_client.read_holding_registers(address=0x0201, count=1, slave=unit_id)
                if running_status.isError():
                    return None
                    
                position = self.rtu_client.read_holding_registers(address=0x0202, count=1, slave=unit_id)
                if position.isError():
                    return None
                    
                current = self.rtu_client.read_holding_registers(address=0x0204, count=1, slave=unit_id)
                if current.isError():
                    return None
                    
                status_data = {
                    'home_status': home_status.registers[0],
                    'running_status': running_status.registers[0], 
                    'position': position.registers[0],
                    'current': current.registers[0],
                    'connected': True
                }
                
            elif gripper_type == 'PGE':
                # PGE狀態讀取 - 使用正確的PyModbus 3.x語法
                init_status = self.rtu_client.read_holding_registers(address=0x0200, count=1, slave=unit_id)
                if init_status.isError():
                    return None
                    
                grip_status = self.rtu_client.read_holding_registers(address=0x0201, count=1, slave=unit_id)
                if grip_status.isError():
                    return None
                    
                position = self.rtu_client.read_holding_registers(address=0x0202, count=1, slave=unit_id)
                if position.isError():
                    return None
                    
                status_data = {
                    'init_status': init_status.registers[0],
                    'grip_status': grip_status.registers[0],
                    'position': position.registers[0],
                    'connected': True
                }
            
            return status_data
            
        except Exception as e:
            self.gripper_states[gripper_type]['error_count'] += 1
            return None

    def execute_gripper_command(self, gripper_type, command, param1=0, param2=0):
        """執行夾爪指令"""
        try:
            if not self.rtu_client or not self.rtu_client.connected:
                return False
                
            unit_id = self.register_mapping[gripper_type]['unit_id']
            success = False
            
            if gripper_type == 'PGC':
                success = self.execute_pgc_command(unit_id, command, param1, param2)
            elif gripper_type == 'PGHL':
                success = self.execute_pghl_command(unit_id, command, param1, param2)
            elif gripper_type == 'PGE':
                success = self.execute_pge_command(unit_id, command, param1, param2)
                
            if not success:
                self.gripper_states[gripper_type]['error_count'] += 1
                
            return success
            
        except Exception as e:
            self.gripper_states[gripper_type]['error_count'] += 1
            return False

    def execute_pgc_command(self, unit_id, command, param1, param2):
        """執行PGC夾爪指令"""
        try:
            if command == 1:  # 初始化
                result = self.rtu_client.write_register(address=0x0100, value=0x01, slave=unit_id)
            elif command == 2:  # 停止
                result = self.rtu_client.write_register(address=0x0100, value=0, slave=unit_id)
            elif command == 3:  # 設定位置(絕對)
                result = self.rtu_client.write_register(address=0x0103, value=param1, slave=unit_id)
            elif command == 5:  # 設定力道
                result = self.rtu_client.write_register(address=0x0101, value=param1, slave=unit_id)
            elif command == 6:  # 設定速度
                result = self.rtu_client.write_register(address=0x0104, value=param1, slave=unit_id)
            elif command == 7:  # 開啟
                result = self.rtu_client.write_register(address=0x0103, value=1000, slave=unit_id)
            elif command == 8:  # 關閉
                result = self.rtu_client.write_register(address=0x0103, value=0, slave=unit_id)
            else:
                return True  # NOP或未知指令
                
            return not result.isError() if result else False
            
        except Exception:
            return False

    def execute_pghl_command(self, unit_id, command, param1, param2):
        """執行PGHL夾爪指令"""
        try:
            if command == 1:  # 初始化
                result = self.rtu_client.write_register(address=0x0100, value=1, slave=unit_id)
            elif command == 2:  # 停止
                result = self.rtu_client.write_register(address=0x0100, value=0, slave=unit_id)
            elif command == 3:  # 設定位置(絕對)
                result = self.rtu_client.write_register(address=0x0103, value=param1, slave=unit_id)
            elif command == 4:  # 設定位置(相對)
                result = self.rtu_client.write_register(address=0x0106, value=param1, slave=unit_id)
            elif command == 5:  # 設定力道
                result = self.rtu_client.write_register(address=0x0101, value=param1, slave=unit_id)
            elif command == 6:  # 設定速度
                result = self.rtu_client.write_register(address=0x0104, value=param1, slave=unit_id)
            elif command == 7:  # 開啟
                result = self.rtu_client.write_register(address=0x0103, value=param1 if param1 > 0 else 5000, slave=unit_id)
            elif command == 8:  # 關閉
                result = self.rtu_client.write_register(address=0x0103, value=0, slave=unit_id)
            else:
                return True  # NOP或未知指令
                
            return not result.isError() if result else False
            
        except Exception:
            return False

    def execute_pge_command(self, unit_id, command, param1, param2):
        """執行PGE夾爪指令"""
        try:
            if command == 1:  # 初始化
                result = self.rtu_client.write_register(address=0x0100, value=0x01, slave=unit_id)
            elif command == 2:  # 停止
                result = self.rtu_client.write_register(address=0x0100, value=0, slave=unit_id)
            elif command == 3:  # 設定位置(絕對)
                result = self.rtu_client.write_register(address=0x0103, value=param1, slave=unit_id)
            elif command == 5:  # 設定力道
                result = self.rtu_client.write_register(address=0x0101, value=param1, slave=unit_id)
            elif command == 6:  # 設定速度
                result = self.rtu_client.write_register(address=0x0104, value=param1, slave=unit_id)
            elif command == 7:  # 開啟
                result = self.rtu_client.write_register(address=0x0103, value=1000, slave=unit_id)
            elif command == 8:  # 關閉
                result = self.rtu_client.write_register(address=0x0103, value=0, slave=unit_id)
            else:
                return True  # NOP或未知指令
                
            return not result.isError() if result else False
            
        except Exception:
            return False

    def update_status_registers(self):
        """更新狀態寄存器到主服務器"""
        try:
            if not self.main_server_client or not self.main_server_client.connected:
                return
                
            for gripper_type in ['PGC', 'PGHL', 'PGE']:
                mapping = self.register_mapping[gripper_type]
                status_base = mapping['status_base']
                
                # 讀取夾爪狀態
                status_data = self.read_gripper_status(gripper_type)
                
                # 準備寄存器數據
                registers = [0] * 20  # 20個狀態寄存器
                
                if status_data:
                    # 通用狀態
                    registers[0] = 1  # 模組狀態 - 在線
                    registers[1] = 1  # 連接狀態 - 已連接
                    registers[3] = self.gripper_states[gripper_type]['error_count']  # 錯誤計數
                    registers[14] = int(time.time()) & 0xFFFF  # 時間戳
                    
                    # 型號特定狀態
                    if gripper_type == 'PGC':
                        registers[2] = status_data.get('init_status', 0)      # 初始化狀態
                        registers[4] = status_data.get('grip_status', 0)      # 夾持狀態  
                        registers[5] = status_data.get('position', 0)         # 位置
                    elif gripper_type == 'PGHL':
                        registers[2] = status_data.get('home_status', 0)      # 回零狀態
                        registers[4] = status_data.get('running_status', 0)   # 運行狀態
                        registers[5] = status_data.get('position', 0)         # 位置
                        registers[6] = status_data.get('current', 0)          # 電流
                    elif gripper_type == 'PGE':
                        registers[2] = status_data.get('init_status', 0)      # 初始化狀態
                        registers[4] = status_data.get('grip_status', 0)      # 夾持狀態
                        registers[5] = status_data.get('position', 0)         # 位置
                    
                    #print(f"更新{gripper_type}狀態到地址{status_base}: 連接={registers[1]}, 狀態={registers[2]}, 位置={registers[5]}")
                else:
                    # 設備離線狀態
                    registers[0] = 0  # 模組狀態 - 離線
                    registers[1] = 0  # 連接狀態 - 斷開
                    registers[3] = self.gripper_states[gripper_type]['error_count']
                    #print(f"更新{gripper_type}狀態到地址{status_base}: 離線")
                
                # 寫入主服務器
                result = self.main_server_client.write_registers(
                    address=status_base,
                    values=registers,
                    slave=self.config["tcp_server"]["unit_id"]
                )
                
                if result.isError():
                    print(f"寫入{gripper_type}狀態寄存器失敗: {result}")
                    
        except Exception as e:
            print(f"狀態寄存器更新錯誤: {e}")

    def process_commands(self):
        """處理指令寄存器"""
        try:
            if not self.main_server_client or not self.main_server_client.connected:
                return
                
            for gripper_type in ['PGC', 'PGHL', 'PGE']:
                mapping = self.register_mapping[gripper_type]
                command_base = mapping['command_base']
                
                # 讀取指令寄存器
                result = self.main_server_client.read_holding_registers(
                    address=command_base,
                    count=10,
                    slave=self.config["tcp_server"]["unit_id"]
                )
                
                if result.isError():
                    continue
                    
                command_id = result.registers[3]  # 指令ID
                
                # 檢查新指令
                if command_id != 0 and command_id != self.last_command_ids[gripper_type]:
                    self.last_command_ids[gripper_type] = command_id
                    
                    command = result.registers[0]  # 指令代碼
                    param1 = result.registers[1]   # 參數1
                    param2 = result.registers[2]   # 參數2
                    
                    print(f"收到{gripper_type}指令: 代碼={command}, 參數1={param1}, 參數2={param2}, ID={command_id}")
                    
                    # 執行指令
                    success = self.execute_gripper_command(gripper_type, command, param1, param2)
                    
                    if success:
                        print(f"{gripper_type}指令執行成功")
                    else:
                        print(f"{gripper_type}指令執行失敗")
                    
                    # 清除指令寄存器
                    clear_values = [0] * 10
                    self.main_server_client.write_registers(
                        address=command_base,
                        values=clear_values,
                        slave=self.config["tcp_server"]["unit_id"]
                    )
                    
                    time.sleep(self.config["timing"]["command_delay"])
                    
        except Exception as e:
            print(f"指令處理錯誤: {e}")

    def fast_loop(self):
        """主循環"""
        print("夾爪主循環啟動")
        
        while self.is_running:
            try:
                start_time = time.time()
                
                # 檢查連接
                if not self.connect_main_server():
                    print("主服務器連接失敗，等待重試...")
                    time.sleep(1)
                    continue
                    
                if not self.connect_rtu_devices():
                    print("RTU設備連接失敗，等待重試...")
                    time.sleep(1)
                    continue
                
                # 測試夾爪連接（每10個循環測試一次，減少負載）
                loop_count = getattr(self, 'loop_count', 0)
                if loop_count % 10 == 0:
                    for gripper_type in ['PGC', 'PGHL', 'PGE']:
                        if self.config["grippers"][gripper_type]["enabled"]:
                            self.test_gripper_connection(gripper_type)
                
                self.loop_count = loop_count + 1
                
                # 處理指令
                self.process_commands()
                
                # 更新狀態
                self.update_status_registers()
                
                # 控制循環頻率
                elapsed = time.time() - start_time
                sleep_time = self.config["timing"]["fast_loop_interval"] - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
            except KeyboardInterrupt:
                print("收到中斷信號，停止主循環")
                break
            except Exception as e:
                print(f"主循環錯誤: {e}")
                time.sleep(1)

    def start(self):
        """啟動模組"""
        self.is_running = True
        
        # 啟動主循環線程
        self.main_thread = threading.Thread(target=self.fast_loop, daemon=True)
        self.main_thread.start()
        
        print("夾爪模組已啟動")
        
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            self.stop()

    def stop(self):
        """停止模組"""
        print("正在停止夾爪模組...")
        self.is_running = False
        
        if self.main_server_client:
            self.main_server_client.close()
        if self.rtu_client:
            self.rtu_client.close()
            
        print("夾爪模組已停止")

if __name__ == "__main__":
    gripper_module = GripperModule()
    gripper_module.start()