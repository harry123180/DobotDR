#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
testSQLite.py - CCD1原始數據檢查工具
用於比對CCD1原始檢測結果與AutoFeeding處理結果
檢查數據是否一致
"""

import os
import sys
import sqlite3
import time
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from datetime import datetime

# Modbus TCP Client
try:
    from pymodbus.client import ModbusTcpClient
    MODBUS_AVAILABLE = True
except ImportError:
    MODBUS_AVAILABLE = False
    print("❌ pymodbus未安裝，無法讀取Modbus寄存器")

# CoordinateSupporter
coordinate_supporter_path = os.path.join(os.path.dirname(__file__), '..', 'API')
if coordinate_supporter_path not in sys.path:
    sys.path.append(coordinate_supporter_path)

try:
    from CoordinateSupporter import CoordinateSupporter, CoordinatePoint
    COORDINATE_SUPPORTER_AVAILABLE = True
    print("✅ CoordinateSupporter模組導入成功")
except ImportError as e:
    print(f"❌ CoordinateSupporter模組導入失敗: {e}")
    COORDINATE_SUPPORTER_AVAILABLE = False


@dataclass
class ModbusData:
    """Modbus寄存器數據"""
    dr_f_count: int = 0
    stack_count: int = 0
    total_detections: int = 0
    detection_success: int = 0
    timestamp: str = ""


@dataclass
class SQLiteData:
    """SQLite數據統計"""
    ccd1_total_records: int = 0
    ccd1_dr_f_count: int = 0
    ccd1_other_count: int = 0
    best_total_records: int = 0
    find_total_records: int = 0
    last_update_time: str = ""


class SQLiteChecker:
    """SQLite數據檢查工具"""
    
    def __init__(self, db_path: str = None, modbus_host: str = "127.0.0.1", modbus_port: int = 502):
        # SQLite數據庫路徑
        if db_path is None:
            self.db_path = r"C:\Users\user\Documents\GitHub\DobotDR\Automation\CCD1\ccd1_coordinate_supporter.db"
        else:
            self.db_path = db_path
        
        # Modbus連接設置
        self.modbus_host = modbus_host
        self.modbus_port = modbus_port
        self.modbus_client: Optional[ModbusTcpClient] = None
        self.modbus_connected = False
        
        # CoordinateSupporter
        self.coordinate_supporter: Optional[CoordinateSupporter] = None
        
        # CCD1 Modbus寄存器地址
        self.CCD1_REGISTERS = {
            'DR_F_COUNT': 240,
            'STACK_COUNT': 242,
            'TOTAL_DETECTIONS': 243,
            'DETECTION_SUCCESS': 244
        }
        
        print(f"SQLite檢查工具初始化")
        print(f"數據庫路徑: {self.db_path}")
        print(f"Modbus地址: {self.modbus_host}:{self.modbus_port}")
    
    def init_coordinate_supporter(self) -> bool:
        """初始化CoordinateSupporter"""
        try:
            if not COORDINATE_SUPPORTER_AVAILABLE:
                print("❌ CoordinateSupporter不可用")
                return False
            
            self.coordinate_supporter = CoordinateSupporter(db_path=self.db_path)
            print("✅ CoordinateSupporter初始化成功")
            return True
        except Exception as e:
            print(f"❌ CoordinateSupporter初始化失敗: {e}")
            return False
    
    def connect_modbus(self) -> bool:
        """連接Modbus"""
        try:
            if not MODBUS_AVAILABLE:
                print("❌ Modbus功能不可用")
                return False
            
            self.modbus_client = ModbusTcpClient(
                host=self.modbus_host,
                port=self.modbus_port,
                timeout=3.0
            )
            
            self.modbus_connected = self.modbus_client.connect()
            
            if self.modbus_connected:
                print("✅ Modbus連接成功")
            else:
                print("❌ Modbus連接失敗")
            
            return self.modbus_connected
        except Exception as e:
            print(f"❌ Modbus連接異常: {e}")
            return False
    
    def read_modbus_register(self, address: int) -> Optional[int]:
        """讀取Modbus寄存器"""
        try:
            if not self.modbus_connected:
                return None
            
            result = self.modbus_client.read_holding_registers(address, count=1, slave=1)
            if not result.isError():
                return result.registers[0]
            return None
        except Exception:
            return None
    
    def read_modbus_data(self) -> Optional[ModbusData]:
        """讀取CCD1 Modbus數據"""
        try:
            if not self.modbus_connected:
                print("❌ Modbus未連接，無法讀取寄存器")
                return None
            
            modbus_data = ModbusData()
            modbus_data.timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            
            # 讀取各個寄存器
            modbus_data.dr_f_count = self.read_modbus_register(self.CCD1_REGISTERS['DR_F_COUNT']) or 0
            modbus_data.stack_count = self.read_modbus_register(self.CCD1_REGISTERS['STACK_COUNT']) or 0
            modbus_data.total_detections = self.read_modbus_register(self.CCD1_REGISTERS['TOTAL_DETECTIONS']) or 0
            modbus_data.detection_success = self.read_modbus_register(self.CCD1_REGISTERS['DETECTION_SUCCESS']) or 0
            
            return modbus_data
        except Exception as e:
            print(f"❌ Modbus數據讀取失敗: {e}")
            return None
    
    def check_sqlite_tables(self) -> bool:
        """檢查SQLite表是否存在"""
        try:
            if not os.path.exists(self.db_path):
                print(f"❌ SQLite數據庫不存在: {self.db_path}")
                return False
            
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # 檢查表是否存在
            cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
            tables = [row[0] for row in cursor.fetchall()]
            
            required_tables = ['ccd1_detection_results', 'best_algorithm_results', 'find_algorithm_results']
            missing_tables = [table for table in required_tables if table not in tables]
            
            if missing_tables:
                print(f"❌ 缺少表格: {missing_tables}")
                conn.close()
                return False
            
            print(f"✅ SQLite表格檢查完成，找到表格: {tables}")
            conn.close()
            return True
            
        except Exception as e:
            print(f"❌ SQLite表格檢查失敗: {e}")
            return False
    
    def check_table_structure(self) -> Dict[str, List[str]]:
        """檢查SQLite表結構"""
        try:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            table_structures = {}
            
            # 獲取所有表名
            cursor.execute("SELECT name FROM sqlite_master WHERE type='table'")
            tables = [row[0] for row in cursor.fetchall()]
            
            # 檢查每個表的結構
            for table in tables:
                cursor.execute(f"PRAGMA table_info({table})")
                columns = [row[1] for row in cursor.fetchall()]  # row[1] 是列名
                table_structures[table] = columns
            
            conn.close()
            return table_structures
            
        except Exception as e:
            print(f"❌ 表結構檢查失敗: {e}")
            return {}

    def read_sqlite_data(self) -> Optional[SQLiteData]:
        """讀取SQLite數據統計"""
        try:
            if not os.path.exists(self.db_path):
                print(f"❌ SQLite數據庫不存在")
                return None
            
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            sqlite_data = SQLiteData()
            
            # 1. CCD1檢測結果統計
            cursor.execute("SELECT COUNT(*) FROM ccd1_detection_results")
            sqlite_data.ccd1_total_records = cursor.fetchone()[0]
            
            # 統計標籤0和其他標籤的數量
            cursor.execute("SELECT COUNT(*) FROM ccd1_detection_results WHERE label_id = 0")
            sqlite_data.ccd1_dr_f_count = cursor.fetchone()[0]
            
            cursor.execute("SELECT COUNT(*) FROM ccd1_detection_results WHERE label_id != 0")
            sqlite_data.ccd1_other_count = cursor.fetchone()[0]
            
            # 2. Best算法結果統計
            cursor.execute("SELECT COUNT(*) FROM best_algorithm_results")
            sqlite_data.best_total_records = cursor.fetchone()[0]
            
            # 3. Find算法結果統計
            cursor.execute("SELECT COUNT(*) FROM find_algorithm_results")
            sqlite_data.find_total_records = cursor.fetchone()[0]
            
            # 4. 最後更新時間 - 嘗試不同的時間字段
            try:
                # 先檢查是否有created_at字段
                cursor.execute("PRAGMA table_info(ccd1_detection_results)")
                columns = [row[1] for row in cursor.fetchall()]
                
                if 'created_at' in columns:
                    cursor.execute("SELECT MAX(created_at) FROM ccd1_detection_results")
                elif 'timestamp' in columns:
                    cursor.execute("SELECT MAX(timestamp) FROM ccd1_detection_results")
                elif 'created_time' in columns:
                    cursor.execute("SELECT MAX(created_time) FROM ccd1_detection_results")
                else:
                    print("⚠️ 未找到時間字段，使用ROWID")
                    cursor.execute("SELECT MAX(ROWID) FROM ccd1_detection_results")
                
                result = cursor.fetchone()[0]
                if result:
                    sqlite_data.last_update_time = str(result)
                else:
                    sqlite_data.last_update_time = "無數據"
                    
            except Exception as time_e:
                print(f"⚠️ 時間字段讀取失敗: {time_e}")
                sqlite_data.last_update_time = "無法讀取"
            
            conn.close()
            return sqlite_data
            
        except Exception as e:
            print(f"❌ SQLite數據讀取失敗: {e}")
            return None
    
    def get_detailed_ccd1_data(self) -> Dict[str, Any]:
        """獲取CCD1詳細數據"""
        try:
            if not self.coordinate_supporter:
                return {}
            
            # 讀取所有CCD1數據
            all_points = self.coordinate_supporter.get_ccd1_detection_results()
            
            # 按標籤分類
            label_stats = {}
            for point in all_points:
                label_id = point.label_id
                if label_id not in label_stats:
                    label_stats[label_id] = []
                label_stats[label_id].append(point)
            
            # 統計信息
            detailed_data = {
                'total_points': len(all_points),
                'label_stats': {},
                'sample_points': all_points[:5] if all_points else []  # 前5個樣本
            }
            
            for label_id, points in label_stats.items():
                detailed_data['label_stats'][label_id] = {
                    'count': len(points),
                    'avg_confidence': sum(p.confidence for p in points) / len(points) if points else 0,
                    'sample': points[0] if points else None
                }
            
            return detailed_data
            
        except Exception as e:
            print(f"❌ CCD1詳細數據獲取失敗: {e}")
            return {}
    
    def get_algorithm_results(self) -> Dict[str, Any]:
        """獲取算法處理結果"""
        try:
            if not self.coordinate_supporter:
                return {}
            
            # Best算法結果
            best_results = self.coordinate_supporter.get_best_algorithm_results()
            
            # Find算法結果
            find_results = self.coordinate_supporter.get_find_algorithm_results()
            
            return {
                'best_algorithm': {
                    'count': len(best_results),
                    'sample_points': best_results[:3] if best_results else []
                },
                'find_algorithm': {
                    'count': len(find_results),
                    'sample_points': find_results[:3] if find_results else []
                }
            }
            
        except Exception as e:
            print(f"❌ 算法結果獲取失敗: {e}")
            return {}
    
    def print_comparison_report(self):
        """打印比較報告"""
        print("\n" + "="*60)
        print("CCD1原始數據 vs SQLite數據 比較報告")
        print("="*60)
        
        # 0. 先檢查表結構
        print("\n📋 0. SQLite表結構檢查:")
        table_structures = self.check_table_structure()
        if table_structures:
            for table_name, columns in table_structures.items():
                print(f"   表格 '{table_name}': {len(columns)}個字段")
                print(f"     字段: {', '.join(columns)}")
        else:
            print("   ❌ 無法讀取表結構")
        
        # 1. 讀取Modbus數據
        print("\n📊 1. CCD1 Modbus寄存器數據:")
        modbus_data = self.read_modbus_data()
        if modbus_data:
            print(f"   DR_F數量 (240): {modbus_data.dr_f_count}")
            print(f"   STACK數量 (242): {modbus_data.stack_count}")
            print(f"   總檢測數量 (243): {modbus_data.total_detections}")
            print(f"   檢測成功標誌 (244): {modbus_data.detection_success}")
            print(f"   讀取時間: {modbus_data.timestamp}")
        else:
            print("   ❌ 無法讀取Modbus數據")
        
        # 2. 讀取SQLite統計
        print("\n📊 2. SQLite數據庫統計:")
        sqlite_data = self.read_sqlite_data()
        if sqlite_data:
            print(f"   CCD1表總記錄數: {sqlite_data.ccd1_total_records}")
            print(f"   標籤0 (DR_F)數量: {sqlite_data.ccd1_dr_f_count}")
            print(f"   其他標籤數量: {sqlite_data.ccd1_other_count}")
            print(f"   Best算法結果數: {sqlite_data.best_total_records}")
            print(f"   Find算法結果數: {sqlite_data.find_total_records}")
            print(f"   最後更新時間: {sqlite_data.last_update_time}")
        else:
            print("   ❌ 無法讀取SQLite數據")
        
        # 3. 詳細CCD1數據分析
        print("\n📊 3. CCD1詳細數據分析:")
        detailed_data = self.get_detailed_ccd1_data()
        if detailed_data:
            print(f"   總點數: {detailed_data['total_points']}")
            print(f"   標籤統計:")
            for label_id, stats in detailed_data['label_stats'].items():
                print(f"     標籤{label_id}: {stats['count']}個 (平均置信度: {stats['avg_confidence']:.3f})")
            
            if detailed_data['sample_points']:
                print(f"   樣本點 (前3個):")
                for i, point in enumerate(detailed_data['sample_points'][:3]):
                    print(f"     點{i+1}: 標籤{point.label_id}, 像素({point.x:.1f},{point.y:.1f}), "
                         f"世界({point.world_x:.2f},{point.world_y:.2f}), 置信度{point.confidence:.3f}")
        else:
            print("   ❌ 無法獲取詳細數據")
        
        # 4. 算法處理結果
        print("\n📊 4. 算法處理結果:")
        algorithm_data = self.get_algorithm_results()
        if algorithm_data:
            print(f"   Best算法結果: {algorithm_data['best_algorithm']['count']}個")
            print(f"   Find算法結果: {algorithm_data['find_algorithm']['count']}個")
            
            if algorithm_data['find_algorithm']['sample_points']:
                print(f"   Find算法樣本點:")
                for i, point in enumerate(algorithm_data['find_algorithm']['sample_points']):
                    print(f"     點{i+1}: 像素({point.x:.1f},{point.y:.1f}), "
                         f"世界({point.world_x:.2f},{point.world_y:.2f})")
        else:
            print("   ❌ 無法獲取算法結果")
        
        # 5. 數據一致性檢查
        print("\n🔍 5. 數據一致性分析:")
        if modbus_data and sqlite_data and detailed_data:
            # 檢查總數是否一致
            modbus_total = modbus_data.total_detections
            sqlite_total = sqlite_data.ccd1_total_records
            
            print(f"   Modbus總檢測數: {modbus_total}")
            print(f"   SQLite總記錄數: {sqlite_total}")
            
            if modbus_total == sqlite_total:
                print("   ✅ 總數量一致")
            else:
                print(f"   ❌ 總數量不一致，差異: {abs(modbus_total - sqlite_total)}")
            
            # 檢查DR_F數量
            modbus_dr_f = modbus_data.dr_f_count
            sqlite_dr_f = sqlite_data.ccd1_dr_f_count
            
            print(f"   Modbus DR_F數: {modbus_dr_f}")
            print(f"   SQLite DR_F數: {sqlite_dr_f}")
            
            if modbus_dr_f == sqlite_dr_f:
                print("   ✅ DR_F數量一致")
            else:
                print(f"   ❌ DR_F數量不一致，差異: {abs(modbus_dr_f - sqlite_dr_f)}")
            
            # 算法處理率
            if sqlite_dr_f > 0:
                best_rate = (sqlite_data.best_total_records / sqlite_dr_f) * 100
                find_rate = (sqlite_data.find_total_records / sqlite_dr_f) * 100
                print(f"   Best算法通過率: {best_rate:.1f}%")
                print(f"   Find算法通過率: {find_rate:.1f}%")
            
            # 🔥 計算AutoFeeding處理的物件總數
            autofeeding_total = sqlite_data.best_total_records + (sqlite_data.ccd1_total_records - sqlite_data.ccd1_dr_f_count)
            print(f"\n🔍 AutoFeeding處理數量分析:")
            print(f"   Best算法結果: {sqlite_data.best_total_records}個")
            print(f"   非DR_F物件: {sqlite_data.ccd1_total_records - sqlite_data.ccd1_dr_f_count}個")
            print(f"   AutoFeeding處理總數: {autofeeding_total}個")
            print(f"   CCD1原始總數: {sqlite_data.ccd1_total_records}個")
            
            if autofeeding_total == sqlite_data.ccd1_total_records:
                print("   ✅ AutoFeeding處理數量與CCD1原始數量一致")
            else:
                missing = sqlite_data.ccd1_total_records - autofeeding_total
                print(f"   ❌ 數量不一致，缺少: {missing}個物件")
                
                # 分析可能的原因
                print(f"\n🔍 可能原因分析:")
                dr_f_not_processed = sqlite_data.ccd1_dr_f_count - sqlite_data.best_total_records
                if dr_f_not_processed > 0:
                    print(f"   - DR_F未通過Best算法: {dr_f_not_processed}個")
                    print(f"   - Best算法通過率: {(sqlite_data.best_total_records/sqlite_data.ccd1_dr_f_count)*100:.1f}%")
                
        else:
            print("   ❌ 數據不完整，無法進行一致性檢查")
        
        print("\n" + "="*60)
    
    def run_continuous_check(self, interval: int = 10):
        """持續檢查模式"""
        print(f"\n開始持續檢查模式，每{interval}秒檢查一次")
        print("按 Ctrl+C 退出")
        
        try:
            while True:
                print(f"\n[{datetime.now().strftime('%H:%M:%S')}] 執行檢查...")
                self.print_comparison_report()
                
                print(f"\n等待{interval}秒後進行下次檢查...")
                time.sleep(interval)
                
        except KeyboardInterrupt:
            print("\n檢查已停止")
    
    def disconnect(self):
        """斷開連接"""
        if self.modbus_client and self.modbus_connected:
            self.modbus_client.close()
            self.modbus_connected = False
            print("✅ Modbus連接已斷開")


def main():
    """主程序"""
    print("=== CCD1原始數據檢查工具 ===")
    print("用途：比對CCD1 Modbus寄存器與SQLite數據庫的一致性")
    print()
    
    # 檢查依賴
    if not MODBUS_AVAILABLE:
        print("❌ pymodbus未安裝，將無法讀取Modbus寄存器")
    
    if not COORDINATE_SUPPORTER_AVAILABLE:
        print("❌ CoordinateSupporter未找到，將無法讀取SQLite數據")
        return
    
    # 創建檢查工具
    checker = SQLiteChecker()
    
    # 初始化CoordinateSupporter
    if not checker.init_coordinate_supporter():
        print("❌ CoordinateSupporter初始化失敗")
        return
    
    # 檢查SQLite表
    if not checker.check_sqlite_tables():
        print("❌ SQLite表格檢查失敗")
        return
    
    # 連接Modbus（可選）
    modbus_ok = checker.connect_modbus()
    if not modbus_ok:
        print("⚠️  Modbus連接失敗，將只檢查SQLite數據")
    
    try:
        # 提供選項
        print("\n選擇操作模式:")
        print("1. 單次檢查")
        print("2. 持續檢查 (每10秒)")
        print("3. 持續檢查 (每30秒)")
        print("4. 退出")
        
        choice = input("\n請選擇 (1-4): ").strip()
        
        if choice == "1":
            checker.print_comparison_report()
        elif choice == "2":
            checker.run_continuous_check(10)
        elif choice == "3":
            checker.run_continuous_check(30)
        elif choice == "4":
            print("退出程序")
        else:
            print("無效選擇，執行單次檢查")
            checker.print_comparison_report()
            
    except KeyboardInterrupt:
        print("\n程序被中斷")
    finally:
        checker.disconnect()
        print("\n程序結束")


if __name__ == "__main__":
    main()