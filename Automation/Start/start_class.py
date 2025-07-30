#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
start_class.py - 執行模組管理類
管理每一個執行模組的開啟、記憶體監控、停止、日誌輸出、關閉等行為
"""

import os
import sys
import subprocess
import threading
import time
import psutil
import logging
import json
from typing import Dict, Optional, Any
from datetime import datetime, timedelta
from pathlib import Path

try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException, ConnectionException
    MODBUS_AVAILABLE = True
except ImportError as e:
    print(f"Modbus Client模組導入失敗: {e}")
    MODBUS_AVAILABLE = False


class ModuleManager:
    """執行模組管理器"""
    
    def __init__(self, module_path: str, module_name: str, conda_env: str = "ROBOT", 
                 log_level: str = "ERROR", log_retention_minutes: int = 5):
        """
        初始化模組管理器
        
        Args:
            module_path: 模組執行檔的完整路徑
            module_name: 模組名稱
            conda_env: Conda環境名稱
            log_level: 日誌等級
            log_retention_minutes: 日誌保存時間(分鐘)
        """
        self.module_path = Path(module_path)
        self.module_name = module_name
        self.conda_env = conda_env
        self.log_level = log_level
        self.log_retention_minutes = log_retention_minutes
        
        # 執行狀態
        self.process: Optional[subprocess.Popen] = None
        self.is_running = False
        self.start_time = None
        self.restart_count = 0
        self.last_restart_time = 0
        
        # 記憶體監控
        self.memory_usage_mb = 0
        self.last_memory_update = 0
        
        # 日誌管理
        self.setup_logging()
        
        # 執行緒鎖
        self.lock = threading.Lock()
        
        print(f"[{self.module_name}] 模組管理器初始化完成")
    
    def setup_logging(self):
        """設置日誌系統"""
        try:
            # 創建日誌目錄
            current_dir = Path(__file__).parent
            log_dir = current_dir / "logs" / self.module_name
            log_dir.mkdir(parents=True, exist_ok=True)
            
            # 設置日誌檔案路徑
            log_file = log_dir / f"{self.module_name}_{datetime.now().strftime('%Y%m%d')}.log"
            
            # 配置日誌格式
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            
            # 使用RotatingFileHandler避免檔案鎖定問題
            from logging.handlers import RotatingFileHandler
            file_handler = RotatingFileHandler(
                log_file, 
                maxBytes=10*1024*1024,  # 10MB
                backupCount=5,
                encoding='utf-8'
            )
            file_handler.setFormatter(formatter)
            file_handler.setLevel(getattr(logging, self.log_level.upper()))
            
            # 創建日誌記錄器
            self.logger = logging.getLogger(f"ModuleManager_{self.module_name}")
            self.logger.setLevel(getattr(logging, self.log_level.upper()))
            
            # 清除現有處理器避免重複
            self.logger.handlers.clear()
            self.logger.addHandler(file_handler)
            
            # 啟動日誌清理執行緒
            self.start_log_cleanup_thread()
            
        except Exception as e:
            print(f"[{self.module_name}] 設置日誌失敗: {e}")
    
    def start_log_cleanup_thread(self):
        """啟動日誌清理執行緒"""
        def cleanup_logs():
            while True:
                try:
                    current_dir = Path(__file__).parent
                    log_dir = current_dir / "logs" / self.module_name
                    
                    if log_dir.exists():
                        cutoff_time = datetime.now() - timedelta(minutes=self.log_retention_minutes)
                        current_log_file = log_dir / f"{self.module_name}_{datetime.now().strftime('%Y%m%d')}.log"
                        
                        for log_file in log_dir.glob("*.log"):
                            # 跳過當天的日誌檔案，避免檔案被鎖定
                            if log_file == current_log_file:
                                continue
                                
                            # 只清理修改時間超過保留期限的檔案
                            if log_file.stat().st_mtime < cutoff_time.timestamp():
                                try:
                                    log_file.unlink()
                                    print(f"[{self.module_name}] 清理過期日誌: {log_file.name}")
                                except (PermissionError, OSError) as file_error:
                                    # 檔案被鎖定時跳過，不報錯
                                    pass
                    
                    time.sleep(300)  # 每5分鐘檢查一次，降低頻率
                    
                except Exception as e:
                    print(f"[{self.module_name}] 日誌清理異常: {e}")
                    time.sleep(300)
        
        cleanup_thread = threading.Thread(target=cleanup_logs, daemon=True)
        cleanup_thread.start()
    
    def _get_conda_python_path(self) -> Optional[str]:
        """獲取conda環境的Python路徑"""
        try:
            # 方法1：使用conda info獲取環境路徑
            cmd = ["conda", "info", "--envs"]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if self.conda_env in line and '*' not in line:  # 排除當前環境標記
                        parts = line.split()
                        if len(parts) >= 2:
                            env_path = parts[-1]  # 環境路徑通常在最後
                            python_path = os.path.join(env_path, 'python.exe')
                            if os.path.exists(python_path):
                                return python_path
            
            # 方法2：使用標準conda路徑
            conda_base = os.environ.get('CONDA_PREFIX_1', os.environ.get('CONDA_PREFIX', ''))
            if not conda_base:
                # 嘗試從conda可執行檔路徑推斷
                conda_exe = subprocess.run(['where', 'conda'], capture_output=True, text=True)
                if conda_exe.returncode == 0:
                    conda_path = conda_exe.stdout.strip().split('\n')[0]
                    conda_base = os.path.dirname(os.path.dirname(conda_path))  # 上兩層目錄
            
            if conda_base:
                python_path = os.path.join(conda_base, 'envs', self.conda_env, 'python.exe')
                if os.path.exists(python_path):
                    return python_path
            
            # 方法3：使用用戶目錄下的anaconda3
            user_conda = os.path.expanduser(f'~/anaconda3/envs/{self.conda_env}/python.exe')
            if os.path.exists(user_conda):
                return user_conda
                
            # 方法4：使用常見的anaconda安裝路徑
            common_paths = [
                f'C:/Users/{os.getenv("USERNAME")}/anaconda3/envs/{self.conda_env}/python.exe',
                f'C:/anaconda3/envs/{self.conda_env}/python.exe',
                f'C:/ProgramData/Anaconda3/envs/{self.conda_env}/python.exe'
            ]
            
            for path in common_paths:
                if os.path.exists(path):
                    return path
            
            return None
            
        except Exception as e:
            self.logger.warning(f"獲取conda Python路徑失敗: {e}")
            return None
    
    def start_module(self) -> bool:
        """啟動模組"""
        with self.lock:
            if self.is_running:
                self.logger.warning("模組已在運行中")
                return True
            
            try:
                # 檢查模組檔案是否存在
                if not self.module_path.exists():
                    self.logger.error(f"模組檔案不存在: {self.module_path}")
                    return False
                
                # 準備執行指令
                working_dir = self.module_path.parent
                
                # 設置環境變數解決編碼問題
                env = os.environ.copy()
                env['PYTHONIOENCODING'] = 'utf-8'
                env['CONDA_VERBOSITY'] = '0'  # 降低conda輸出
                
                # 使用conda環境的Python直接路徑執行
                try:
                    # 獲取conda環境的Python路徑
                    conda_python_path = self._get_conda_python_path()
                    if conda_python_path and Path(conda_python_path).exists():
                        cmd = [conda_python_path, str(self.module_path)]
                        print(f"[{self.module_name}] 使用conda環境Python: {conda_python_path}")
                    else:
                        # 備用方案：使用conda run
                        cmd = ["conda", "run", "-n", self.conda_env, "python", str(self.module_path)]
                        print(f"[{self.module_name}] 使用conda run命令")
                    
                except Exception as conda_error:
                    self.logger.warning(f"Conda環境檢查失敗: {conda_error}，使用系統Python")
                    print(f"[{self.module_name}] Conda環境檢查失敗，使用系統Python")
                    cmd = ["python", str(self.module_path)]
                
                # 啟動進程
                print(f"[{self.module_name}] 執行指令: {' '.join(cmd)}")
                print(f"[{self.module_name}] 工作目錄: {working_dir}")
                
                self.process = subprocess.Popen(
                    cmd,
                    cwd=working_dir,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    bufsize=1,
                    universal_newlines=True,
                    env=env,
                    encoding='utf-8',
                    errors='replace'  # 遇到編碼錯誤時替換為?
                )
                
                # 等待一下確認進程啟動
                time.sleep(3)  # 增加等待時間到3秒
                
                # 檢查進程是否立即退出
                exit_code = self.process.poll()
                if exit_code is not None:
                    # 進程立即退出了，讀取錯誤信息
                    try:
                        stdout, stderr = self.process.communicate(timeout=5)
                        self.logger.error(f"模組立即退出，退出碼: {exit_code}")
                        if stdout.strip():
                            self.logger.error(f"STDOUT: {stdout.strip()}")
                            print(f"[{self.module_name}] STDOUT: {stdout.strip()}")
                        if stderr.strip():
                            self.logger.error(f"STDERR: {stderr.strip()}")
                            print(f"[{self.module_name}] STDERR: {stderr.strip()}")
                        else:
                            print(f"[{self.module_name}] 模組無錯誤信息，可能缺少主迴圈或條件不滿足")
                    except subprocess.TimeoutExpired:
                        self.logger.error("讀取進程輸出超時")
                        
                    print(f"[{self.module_name}] 模組立即退出，退出碼: {exit_code}")
                    self.is_running = False
                    return False
                
                self.is_running = True
                self.start_time = time.time()
                self.restart_count += 1
                
                self.logger.info(f"模組啟動成功 PID: {self.process.pid}")
                print(f"[{self.module_name}] 啟動成功 PID: {self.process.pid}")
                
                # 啟動輸出監控執行緒
                self.start_output_monitor()
                
                return True
                
            except Exception as e:
                self.logger.error(f"啟動模組失敗: {e}")
                print(f"[{self.module_name}] 啟動失敗: {e}")
                self.is_running = False
                return False
    
    def start_output_monitor(self):
        """啟動輸出監控執行緒"""
        def monitor_stdout():
            try:
                while self.process and self.process.poll() is None:
                    try:
                        line = self.process.stdout.readline()
                        if line:
                            # 清理控制字符和非打印字符
                            clean_line = ''.join(char for char in line if char.isprintable() or char.isspace()).strip()
                            if clean_line:
                                self.logger.info(f"STDOUT: {clean_line}")
                        else:
                            time.sleep(0.1)
                    except UnicodeDecodeError as ude:
                        self.logger.warning(f"STDOUT編碼錯誤: {ude}")
                    except Exception as e:
                        self.logger.error(f"STDOUT讀取錯誤: {e}")
                        break
            except Exception as e:
                self.logger.error(f"STDOUT監控異常: {e}")
        
        def monitor_stderr():
            try:
                while self.process and self.process.poll() is None:
                    try:
                        line = self.process.stderr.readline()
                        if line:
                            # 清理控制字符和非打印字符
                            clean_line = ''.join(char for char in line if char.isprintable() or char.isspace()).strip()
                            if clean_line:
                                # 過濾conda的一些無用錯誤信息
                                if not any(ignore in clean_line.lower() for ignore in [
                                    'unicodeencodeerror', 'cp950', 'illegal multibyte'
                                ]):
                                    self.logger.error(f"STDERR: {clean_line}")
                        else:
                            time.sleep(0.1)
                    except UnicodeDecodeError as ude:
                        self.logger.warning(f"STDERR編碼錯誤: {ude}")
                    except Exception as e:
                        self.logger.error(f"STDERR讀取錯誤: {e}")
                        break
            except Exception as e:
                self.logger.error(f"STDERR監控異常: {e}")
        
        stdout_thread = threading.Thread(target=monitor_stdout, daemon=True)
        stderr_thread = threading.Thread(target=monitor_stderr, daemon=True)
        
        stdout_thread.start()
        stderr_thread.start()
    
    def stop_module(self) -> bool:
        """停止模組"""
        with self.lock:
            if not self.is_running or not self.process:
                return True
            
            try:
                # 溫和終止
                self.process.terminate()
                
                # 等待進程結束
                try:
                    self.process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    # 強制終止
                    self.process.kill()
                    self.process.wait()
                
                self.is_running = False
                self.process = None
                self.logger.info("模組已停止")
                print(f"[{self.module_name}] 已停止")
                
                return True
                
            except Exception as e:
                self.logger.error(f"停止模組失敗: {e}")
                print(f"[{self.module_name}] 停止失敗: {e}")
                return False
    
    def is_alive(self) -> bool:
        """檢查模組是否存活"""
        if not self.is_running or not self.process:
            return False
        
        try:
            # 檢查進程是否還在運行
            exit_code = self.process.poll()
            if exit_code is None:
                # 進程還在運行
                return True
            else:
                # 進程已結束，記錄退出碼
                self.logger.error(f"模組進程已結束，退出碼: {exit_code}")
                print(f"[{self.module_name}] 模組進程已結束，退出碼: {exit_code}")
                self.is_running = False
                return False
        except Exception as e:
            self.logger.error(f"檢查進程狀態失敗: {e}")
            return False
    
    def update_memory_usage(self):
        """更新記憶體使用量"""
        try:
            if not self.is_running or not self.process:
                self.memory_usage_mb = 0
                return
            
            # 獲取進程記憶體資訊
            process = psutil.Process(self.process.pid)
            memory_info = process.memory_info()
            
            # 轉換為MB並無條件捨去
            self.memory_usage_mb = int(memory_info.rss / 1024 / 1024)
            self.last_memory_update = time.time()
            
        except (psutil.NoSuchProcess, psutil.AccessDenied, Exception) as e:
            self.memory_usage_mb = 0
            if self.is_running:
                self.logger.warning(f"更新記憶體使用量失敗: {e}")
    
    def get_status(self) -> Dict[str, Any]:
        """獲取模組狀態"""
        return {
            'module_name': self.module_name,
            'module_path': str(self.module_path),
            'is_running': self.is_running,
            'pid': self.process.pid if self.process else None,
            'start_time': self.start_time,
            'restart_count': self.restart_count,
            'memory_usage_mb': self.memory_usage_mb,
            'log_level': self.log_level,
            'conda_env': self.conda_env
        }
    
    def set_log_level(self, new_level: str):
        """設置日誌等級"""
        try:
            self.log_level = new_level.upper()
            self.logger.setLevel(getattr(logging, self.log_level))
            
            # 更新所有處理器的等級
            for handler in self.logger.handlers:
                handler.setLevel(getattr(logging, self.log_level))
            
            self.logger.info(f"日誌等級已更新為: {self.log_level}")
            
        except Exception as e:
            self.logger.error(f"設置日誌等級失敗: {e}")
    
    def cleanup(self):
        """清理資源"""
        self.stop_module()


class StartManager:
    """啟動管理器 - 管理所有執行模組"""
    
    def __init__(self, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        """
        初始化啟動管理器
        
        Args:
            modbus_host: Modbus伺服器IP
            modbus_port: Modbus伺服器端口
        """
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client = None
        self.connected = False
        
        # 模組管理器字典
        self.modules: Dict[str, ModuleManager] = {}
        
        # 監控執行緒控制
        self.monitoring_active = False
        self.monitor_thread = None
        
        # 設置日誌
        self.setup_logging()
        
        print("[StartManager] 啟動管理器初始化完成")
    
    def setup_logging(self):
        """設置日誌系統"""
        try:
            current_dir = Path(__file__).parent
            log_dir = current_dir / "logs" / "StartManager"
            log_dir.mkdir(parents=True, exist_ok=True)
            
            log_file = log_dir / f"StartManager_{datetime.now().strftime('%Y%m%d')}.log"
            
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            
            file_handler = logging.FileHandler(log_file, encoding='utf-8')
            file_handler.setFormatter(formatter)
            file_handler.setLevel(logging.INFO)
            
            self.logger = logging.getLogger("StartManager")
            self.logger.setLevel(logging.INFO)
            self.logger.addHandler(file_handler)
            
        except Exception as e:
            print(f"[StartManager] 設置日誌失敗: {e}")
    
    def connect_modbus(self) -> bool:
        """連接Modbus TCP伺服器"""
        if not MODBUS_AVAILABLE:
            self.logger.error("Modbus Client不可用")
            return False
        
        try:
            if self.modbus_client:
                self.modbus_client.close()
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            if self.modbus_client.connect():
                self.connected = True
                self.logger.info(f"Modbus TCP連接成功: {self.modbus_host}:{self.modbus_port}")
                print(f"[StartManager] Modbus連接成功: {self.modbus_host}:{self.modbus_port}")
                return True
            else:
                self.logger.error(f"Modbus TCP連接失敗: {self.modbus_host}:{self.modbus_port}")
                print(f"[StartManager] Modbus連接失敗: {self.modbus_host}:{self.modbus_port}")
                return False
                
        except Exception as e:
            self.logger.error(f"Modbus TCP連接異常: {e}")
            print(f"[StartManager] Modbus連接異常: {e}")
            return False
    
    def write_modbus_register(self, address: int, value: int) -> bool:
        """寫入Modbus寄存器"""
        if not self.connected or not self.modbus_client:
            return False
        
        try:
            result = self.modbus_client.write_register(address, value, slave=1)
            return not result.isError()
        except Exception as e:
            self.logger.error(f"寫入Modbus寄存器失敗 地址{address}: {e}")
            return False
    
    def add_module(self, module_path: str, module_name: str, 
                   conda_env: str = "ROBOT", log_level: str = "ERROR", 
                   log_retention_minutes: int = 5) -> bool:
        """添加執行模組"""
        try:
            if module_name in self.modules:
                self.logger.warning(f"模組 {module_name} 已存在")
                return False
            
            manager = ModuleManager(
                module_path=module_path,
                module_name=module_name,
                conda_env=conda_env,
                log_level=log_level,
                log_retention_minutes=log_retention_minutes
            )
            
            self.modules[module_name] = manager
            self.logger.info(f"添加模組: {module_name}")
            print(f"[StartManager] 添加模組: {module_name}")
            
            return True
            
        except Exception as e:
            self.logger.error(f"添加模組失敗 {module_name}: {e}")
            print(f"[StartManager] 添加模組失敗 {module_name}: {e}")
            return False
    
    def start_module(self, module_name: str) -> bool:
        """啟動指定模組"""
        if module_name not in self.modules:
            self.logger.error(f"模組不存在: {module_name}")
            return False
        
        return self.modules[module_name].start_module()
    
    def stop_module(self, module_name: str) -> bool:
        """停止指定模組"""
        if module_name not in self.modules:
            self.logger.error(f"模組不存在: {module_name}")
            return False
        
        return self.modules[module_name].stop_module()
    
    def start_all_modules(self) -> bool:
        """啟動所有模組"""
        success_count = 0
        total_count = len(self.modules)
        
        for module_name in self.modules:
            if self.start_module(module_name):
                success_count += 1
                time.sleep(1)  # 間隔啟動避免資源競爭
        
        self.logger.info(f"啟動模組完成: {success_count}/{total_count}")
        print(f"[StartManager] 啟動模組完成: {success_count}/{total_count}")
        
        return success_count == total_count
    
    def start_monitoring(self):
        """啟動監控執行緒"""
        if self.monitoring_active:
            return
        
        self.monitoring_active = True
        self.monitor_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.monitor_thread.start()
        
        self.logger.info("監控執行緒已啟動")
        print("[StartManager] 監控執行緒已啟動")
    
    def stop_monitoring(self):
        """停止監控執行緒"""
        self.monitoring_active = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=5)
        
        self.logger.info("監控執行緒已停止")
        print("[StartManager] 監控執行緒已停止")
    
    def _monitoring_loop(self):
        """監控主迴圈"""
        while self.monitoring_active:
            try:
                # 檢查並重啟關閉的模組
                for module_name, manager in self.modules.items():
                    if manager.start_time is not None:  # 只檢查已經啟動過的模組
                        if not manager.is_alive():
                            # 增加重啟間隔，避免頻繁重啟
                            current_time = time.time()
                            if current_time - manager.last_restart_time < 30:  # 30秒內不重複重啟
                                continue
                                
                            manager.last_restart_time = current_time
                            self.logger.warning(f"檢測到模組關閉，重新啟動: {module_name}")
                            print(f"[StartManager] 檢測到模組關閉，重新啟動: {module_name}")
                            
                            success = manager.start_module()
                            if not success:
                                self.logger.error(f"重啟模組失敗: {module_name}")
                                print(f"[StartManager] 重啟模組失敗: {module_name}")
                
                # 更新記憶體使用量
                self._update_memory_monitoring()
                
                time.sleep(30)  # 增加檢查間隔到30秒，減少頻繁檢查
                
            except Exception as e:
                self.logger.error(f"監控迴圈異常: {e}")
                time.sleep(30)
    
    def _update_memory_monitoring(self):
        """更新記憶體監控到Modbus"""
        try:
            # 每分鐘更新一次記憶體數據到Modbus
            current_time = time.time()
            
            for module_name, manager in self.modules.items():
                manager.update_memory_usage()
                
                # 檢查是否需要更新到Modbus (每60秒一次)
                if current_time - manager.last_memory_update < 60:
                    continue
                
                # 這裡需要根據具體的模組地址映射來寫入
                # 在start.py中會定義具體的地址映射
                
        except Exception as e:
            self.logger.error(f"更新記憶體監控失敗: {e}")
    
    def get_all_status(self) -> Dict[str, Any]:
        """獲取所有模組狀態"""
        status = {
            'manager_info': {
                'total_modules': len(self.modules),
                'running_modules': sum(1 for m in self.modules.values() if m.is_running),
                'modbus_connected': self.connected,
                'monitoring_active': self.monitoring_active
            },
            'modules': {}
        }
        
        for module_name, manager in self.modules.items():
            status['modules'][module_name] = manager.get_status()
        
        return status
    
    def cleanup(self):
        """清理所有資源"""
        self.stop_monitoring()
        
        for manager in self.modules.values():
            manager.cleanup()
        
        if self.modbus_client and self.connected:
            self.modbus_client.close()
        
        self.logger.info("StartManager清理完成")
        print("[StartManager] 清理完成")