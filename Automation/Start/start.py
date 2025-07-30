#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
start.py - 主啟動控制器
依序啟動所有執行模組並進行記憶體監控和日誌管理
"""

import time
import signal
import sys
from pathlib import Path
from start_class import StartManager

# ==================== 模組地址映射配置 ====================
MODULE_MODBUS_CONFIG = {
    # 模組名稱: {memory_address: Modbus記憶體地址, log_control_address: 日誌控制地址}
    'VP_main': {'memory_address': 100, 'log_control_address': 120},
    'VP_app': {'memory_address': 101, 'log_control_address': 121},
    'Gripper': {'memory_address': 102, 'log_control_address': 122},
    'Gripper_app': {'memory_address': 103, 'log_control_address': 123},
    'Dobot_main': {'memory_address': 104, 'log_control_address': 124},
    'CCD3_main_app': {'memory_address': 105, 'log_control_address': 125},
    'CCD1VisionCode': {'memory_address': 106, 'log_control_address': 126},
    'AutoProgram_main': {'memory_address': 107, 'log_control_address': 127},
    'AutoFeeding_main': {'memory_address': 108, 'log_control_address': 128},
    'AutoProgram_app': {'memory_address': 109, 'log_control_address': 129},
    'LED_main': {'memory_address': 110, 'log_control_address': 130},
    'LED_app': {'memory_address': 111, 'log_control_address': 131},
}

# ==================== 執行模組路徑配置 ====================
MODULE_PATHS = [
    {
        'path': r'C:\Users\user\Documents\GitHub\DobotDR\Automation\VP\VP_main.py',
        'name': 'VP_main'
    },
    {
        'path': r'C:\Users\user\Documents\GitHub\DobotDR\Automation\VP\VP_app.py',
        'name': 'VP_app'
    },
    {
        'path': r'C:\Users\user\Documents\GitHub\DobotDR\Automation\Gripper\Gripper.py',
        'name': 'Gripper'
    },
    {
        'path': r'C:\Users\user\Documents\GitHub\DobotDR\Automation\Gripper\Gripper_app.py',
        'name': 'Gripper_app'
    },
    {
        'path': r'C:\Users\user\Documents\GitHub\DobotDR\Automation\M1Pro\new_architecture\Dobot_main.py',
        'name': 'Dobot_main'
    },
    {
        'path': r'C:\Users\user\Documents\GitHub\DobotDR\Automation\CCD3\CCD3_main_app.py',
        'name': 'CCD3_main_app'
    },
    {
        'path': r'C:\Users\user\Documents\GitHub\DobotDR\Automation\CCD1\CCD1VisionCodeYOLO.py',
        'name': 'CCD1VisionCode'
    },
    {
        'path': r'C:\Users\user\Documents\GitHub\DobotDR\Automation\AutoProgram\AutoProgram_main.py',
        'name': 'AutoProgram_main'
    },
    {
        'path': r'C:\Users\user\Documents\GitHub\DobotDR\Automation\AutoFeeding\AutoFeeding_main.py',
        'name': 'AutoFeeding_main'
    },
    {
        'path': r'C:\Users\user\Documents\GitHub\DobotDR\Automation\AutoProgram\AutoProgram_app.py',
        'name': 'AutoProgram_app'
    },
    {
        'path': r'C:\Users\user\Documents\GitHub\DobotDR\Automation\light\LED_main.py',
        'name': 'LED_main'
    },
    {
        'path': r'C:\Users\user\Documents\GitHub\DobotDR\Automation\light\LED_app.py',
        'name': 'LED_app'
    },
]

class StartController:
    """主啟動控制器"""
    
    def __init__(self):
        """初始化控制器"""
        self.start_manager = StartManager(modbus_host="127.0.0.1", modbus_port=502)
        self.running = True
        
        # 設置信號處理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        print("[StartController] 主啟動控制器初始化完成")
    
    def signal_handler(self, signum, frame):
        """信號處理器"""
        print(f"\n[StartController] 收到信號 {signum}，正在關閉...")
        self.running = False
    
    def initialize(self) -> bool:
        """初始化系統"""
        print("[StartController] 開始初始化系統...")
        
        # 連接Modbus伺服器
        if not self.start_manager.connect_modbus():
            print("[StartController] Modbus連接失敗，無法繼續")
            return False
        
        # 添加所有執行模組
        for module_config in MODULE_PATHS:
            module_path = module_config['path']
            module_name = module_config['name']
            
            # 檢查模組路徑是否存在
            if not Path(module_path).exists():
                print(f"[StartController] 警告: 模組路徑不存在 {module_path}")
                continue
            
            # 添加模組到管理器
            success = self.start_manager.add_module(
                module_path=module_path,
                module_name=module_name,
                conda_env="ROBOT",
                log_level="ERROR",
                log_retention_minutes=5  # 5分鐘清空一次日誌
            )
            
            if success:
                print(f"[StartController] 成功添加模組: {module_name}")
            else:
                print(f"[StartController] 添加模組失敗: {module_name}")
        
        print("[StartController] 系統初始化完成")
        return True
    
    def start_all_modules(self) -> bool:
        """啟動所有模組"""
        print("[StartController] 開始啟動所有模組...")
        
        success = self.start_manager.start_all_modules()
        
        if success:
            print("[StartController] 所有模組啟動成功")
        else:
            print("[StartController] 部分模組啟動失敗")
        
        return success
    
    def start_monitoring(self):
        """啟動監控系統"""
        print("[StartController] 啟動監控系統...")
        
        # 啟動模組監控
        self.start_manager.start_monitoring()
        
        # 啟動記憶體更新執行緒
        import threading
        memory_thread = threading.Thread(target=self._memory_update_loop, daemon=True)
        memory_thread.start()
        
        print("[StartController] 監控系統已啟動")
    
    def _memory_update_loop(self):
        """記憶體更新迴圈 - 每分鐘更新一次到Modbus"""
        last_update_time = 0
        
        while self.running:
            try:
                current_time = time.time()
                
                # 每60秒更新一次記憶體數據到Modbus
                if current_time - last_update_time >= 60:
                    self._update_memory_to_modbus()
                    last_update_time = current_time
                
                time.sleep(10)  # 每10秒檢查一次
                
            except Exception as e:
                print(f"[StartController] 記憶體更新迴圈異常: {e}")
                time.sleep(10)
    
    def _update_memory_to_modbus(self):
        """更新記憶體使用量到Modbus寄存器"""
        try:
            for module_name, manager in self.start_manager.modules.items():
                # 更新模組記憶體使用量
                manager.update_memory_usage()
                
                # 獲取Modbus地址配置
                if module_name in MODULE_MODBUS_CONFIG:
                    config = MODULE_MODBUS_CONFIG[module_name]
                    memory_address = config['memory_address']
                    memory_mb = manager.memory_usage_mb
                    
                    # 寫入記憶體使用量到Modbus
                    success = self.start_manager.write_modbus_register(memory_address, memory_mb)
                    
                    if success:
                        print(f"[StartController] 更新記憶體: {module_name} = {memory_mb}MB -> 地址{memory_address}")
                    else:
                        print(f"[StartController] 記憶體更新失敗: {module_name}")
        
        except Exception as e:
            print(f"[StartController] 更新記憶體到Modbus失敗: {e}")
    
    def handle_log_control(self):
        """處理日誌控制 - 檢查Modbus地址並調整日誌等級"""
        try:
            if not self.start_manager.connected:
                return
            
            for module_name, manager in self.start_manager.modules.items():
                if module_name in MODULE_MODBUS_CONFIG:
                    config = MODULE_MODBUS_CONFIG[module_name]
                    log_control_address = config['log_control_address']
                    
                    # 讀取日誌控制寄存器
                    try:
                        result = self.start_manager.modbus_client.read_holding_registers(
                            log_control_address, count=1, slave=1
                        )
                        
                        if not result.isError() and len(result.registers) > 0:
                            control_value = result.registers[0]
                            
                            # 根據控制值設置日誌等級
                            # 0=關閉, 1=ERROR, 2=WARNING, 3=INFO, 4=DEBUG
                            log_levels = {0: 'CRITICAL', 1: 'ERROR', 2: 'WARNING', 3: 'INFO', 4: 'DEBUG'}
                            
                            if control_value in log_levels:
                                new_level = log_levels[control_value]
                                
                                if manager.log_level != new_level:
                                    manager.set_log_level(new_level)
                                    print(f"[StartController] 調整日誌等級: {module_name} -> {new_level}")
                    
                    except Exception as e:
                        print(f"[StartController] 讀取日誌控制失敗 {module_name}: {e}")
        
        except Exception as e:
            print(f"[StartController] 處理日誌控制異常: {e}")
    
    def print_status(self):
        """打印系統狀態"""
        status = self.start_manager.get_all_status()
        
        print("\n" + "="*60)
        print("[StartController] 系統狀態報告")
        print("="*60)
        
        manager_info = status['manager_info']
        print(f"總模組數: {manager_info['total_modules']}")
        print(f"運行中模組: {manager_info['running_modules']}")
        print(f"Modbus連接: {'正常' if manager_info['modbus_connected'] else '異常'}")
        print(f"監控狀態: {'啟用' if manager_info['monitoring_active'] else '停用'}")
        
        print("\n模組狀態:")
        print("-" * 60)
        
        for module_name, module_status in status['modules'].items():
            status_str = "運行中" if module_status['is_running'] else "已停止"
            memory_mb = module_status['memory_usage_mb']
            restart_count = module_status['restart_count']
            
            print(f"{module_name:20} | {status_str:6} | {memory_mb:4}MB | 重啟{restart_count}次")
        
        print("="*60)
    
    def run(self):
        """主運行迴圈"""
        print("[StartController] 開始運行主迴圈...")
        
        status_print_interval = 300  # 每5分鐘打印一次狀態
        log_control_interval = 30    # 每30秒檢查一次日誌控制
        last_status_print = 0
        last_log_control = 0
        
        try:
            while self.running:
                current_time = time.time()
                
                # 定期打印狀態
                if current_time - last_status_print >= status_print_interval:
                    self.print_status()
                    last_status_print = current_time
                
                # 定期檢查日誌控制
                if current_time - last_log_control >= log_control_interval:
                    self.handle_log_control()
                    last_log_control = current_time
                
                time.sleep(5)  # 主迴圈間隔
                
        except KeyboardInterrupt:
            print("\n[StartController] 收到中斷信號")
        except Exception as e:
            print(f"[StartController] 主迴圈異常: {e}")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """關閉系統"""
        print("[StartController] 開始關閉系統...")
        
        # 停止監控
        self.start_manager.stop_monitoring()
        
        # 停止所有模組
        for module_name in self.start_manager.modules:
            print(f"[StartController] 停止模組: {module_name}")
            self.start_manager.stop_module(module_name)
        
        # 清理資源
        self.start_manager.cleanup()
        
        print("[StartController] 系統關閉完成")


def main():
    """主函數"""
    print("="*60)
    print("[StartController] DobotDR 自動化系統啟動控制器")
    print("="*60)
    
    # 創建控制器
    controller = StartController()
    
    try:
        # 初始化系統
        if not controller.initialize():
            print("[StartController] 系統初始化失敗，退出")
            return 1
        
        # 啟動所有模組
        controller.start_all_modules()
        
        # 啟動監控系統
        controller.start_monitoring()
        
        # 等待模組啟動穩定
        print("[StartController] 等待模組啟動穩定...")
        time.sleep(10)
        
        # 打印初始狀態
        controller.print_status()
        
        # 進入主運行迴圈
        controller.run()
        
        return 0
        
    except Exception as e:
        print(f"[StartController] 主函數異常: {e}")
        controller.shutdown()
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)