# CoordinateSupporter.py - 座標處理與篩選支援模組
import sqlite3
import json
import math
from datetime import datetime
from typing import List, Tuple, Dict, Any, Optional
from dataclasses import dataclass
import os
import logging

@dataclass
class CoordinatePoint:
    """座標點數據結構"""
    x: float
    y: float
    confidence: float = 0.0
    world_x: Optional[float] = None
    world_y: Optional[float] = None
    label_id: int = 0
    
    def distance_to(self, other: 'CoordinatePoint') -> float:
        """計算到另一個點的距離"""
        dx = self.x - other.x
        dy = self.y - other.y
        return math.sqrt(dx * dx + dy * dy)
    
    def to_tuple(self) -> Tuple[float, float]:
        """轉換為座標元組"""
        return (self.x, self.y)
    
    def to_dict(self) -> Dict[str, Any]:
        """轉換為字典"""
        return {
            'x': self.x,
            'y': self.y,
            'confidence': self.confidence,
            'world_x': self.world_x,
            'world_y': self.world_y,
            'label_id': self.label_id
        }

class CoordinateSupporter:
    """座標處理與篩選支援類 - SQLite管理器"""
    
    def __init__(self, db_path: str = "coordinate_supporter.db"):
        """
        初始化座標支援器
        
        Args:
            db_path: SQLite資料庫路徑
        """
        self.db_path = db_path
        self.logger = logging.getLogger(self.__class__.__name__)
        
        # 初始化資料庫
        self._init_database()
        
        self.logger.info(f"CoordinateSupporter 初始化完成: {self.db_path}")
    
    def _init_database(self):
        """初始化SQLite資料庫表結構"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        try:
            # CCD1檢測結果表 (覆蓋式)
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS ccd1_detection_results (
                    id INTEGER PRIMARY KEY,
                    label_id INTEGER NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    confidence REAL NOT NULL,
                    world_x REAL,
                    world_y REAL,
                    timestamp TEXT NOT NULL
                )
            ''')
            
            # Best算法結果表 (覆蓋式)
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS best_algorithm_results (
                    id INTEGER PRIMARY KEY,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    confidence REAL NOT NULL,
                    world_x REAL,
                    world_y REAL,
                    label_id INTEGER NOT NULL,
                    source_count INTEGER NOT NULL,
                    timestamp TEXT NOT NULL
                )
            ''')
            
            # Find算法結果表 (覆蓋式)
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS find_algorithm_results (
                    id INTEGER PRIMARY KEY,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    confidence REAL NOT NULL,
                    world_x REAL,
                    world_y REAL,
                    label_id INTEGER NOT NULL,
                    safe_distance REAL NOT NULL,
                    timestamp TEXT NOT NULL
                )
            ''')
            
            conn.commit()
            
        finally:
            conn.close()
    
    def save_ccd1_detection_results(self, detection_dict: Dict[int, int], 
                                   objects_detail: Dict[int, List[Dict[str, Any]]]) -> bool:
        """
        保存CCD1檢測結果 (覆蓋式) - 執行者模式
        由CCD1模組透過Modbus旗標控制觸發
        
        Args:
            detection_dict: {label_id: count} - 標籤檢測數量
            objects_detail: {label_id: [objects]} - 物件詳細資訊
        
        Returns:
            bool: 保存成功與否
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        try:
            # 清空舊數據
            cursor.execute('DELETE FROM ccd1_detection_results')
            
            # 插入新數據
            record_count = 0
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            
            for label_id, objects in objects_detail.items():
                for obj in objects:
                    cursor.execute('''
                        INSERT INTO ccd1_detection_results 
                        (label_id, x, y, confidence, world_x, world_y, timestamp)
                        VALUES (?, ?, ?, ?, ?, ?, ?)
                    ''', (
                        label_id,
                        obj.get('x', 0),
                        obj.get('y', 0),
                        obj.get('confidence', 0.0),
                        obj.get('world_x'),
                        obj.get('world_y'),
                        timestamp
                    ))
                    record_count += 1
            
            conn.commit()
            
            self.logger.info(f"CCD1檢測結果已保存: {record_count} 個物件")
            return True
            
        except Exception as e:
            conn.rollback()
            self.logger.error(f"保存CCD1檢測結果失敗: {e}")
            return False
            
        finally:
            conn.close()
    
    def get_ccd1_detection_results(self, label_id: Optional[int] = None) -> List[CoordinatePoint]:
        """
        讀取CCD1檢測結果 - 執行者模式
        由AutoFeeding模組透過Modbus旗標控制觸發
        
        Args:
            label_id: 指定標籤ID，None為全部
        
        Returns:
            List[CoordinatePoint]: 座標點列表
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        try:
            if label_id is not None:
                cursor.execute('''
                    SELECT label_id, x, y, confidence, world_x, world_y
                    FROM ccd1_detection_results
                    WHERE label_id = ?
                    ORDER BY id
                ''', (label_id,))
            else:
                cursor.execute('''
                    SELECT label_id, x, y, confidence, world_x, world_y
                    FROM ccd1_detection_results
                    ORDER BY label_id, id
                ''')
            
            points = []
            for row in cursor.fetchall():
                points.append(CoordinatePoint(
                    x=row[1],
                    y=row[2],
                    confidence=row[3],
                    world_x=row[4],
                    world_y=row[5],
                    label_id=row[0]
                ))
            
            self.logger.debug(f"讀取CCD1檢測結果: {len(points)} 個點")
            return points
            
        finally:
            conn.close()
    
    def best_algorithm(self, capture1: List[CoordinatePoint], 
                      capture2: List[CoordinatePoint], 
                      capture3: List[CoordinatePoint],
                      tolerance_mm: float,
                      circle_diameter: float = 60.0) -> List[CoordinatePoint]:
        """
        Best算法 - 三次拍攝匹配算法
        找出三次拍攝中誤差範圍內都存在的點
        
        Args:
            capture1: 第一次拍攝座標
            capture2: 第二次拍攝座標
            capture3: 第三次拍攝座標
            tolerance_mm: 誤差容忍距離(mm)
            circle_diameter: 圓直徑(mm)，用於重疊檢查
        
        Returns:
            List[CoordinatePoint]: 匹配成功的座標點
        """
        result = []
        
        # 遍歷第一次拍攝的每個點
        for p1 in capture1:
            # 在第二次拍攝中找匹配點
            matches2 = [p2 for p2 in capture2 if p1.distance_to(p2) <= tolerance_mm]
            if not matches2:
                continue
            
            # 在第三次拍攝中找匹配點
            matches3 = [p3 for p3 in capture3 if p1.distance_to(p3) <= tolerance_mm]
            if not matches3:
                continue
            
            # 計算三點平均位置
            avg_x = (p1.x + matches2[0].x + matches3[0].x) / 3.0
            avg_y = (p1.y + matches2[0].y + matches3[0].y) / 3.0
            avg_confidence = (p1.confidence + matches2[0].confidence + matches3[0].confidence) / 3.0
            
            # 世界座標平均
            world_x = None
            world_y = None
            if all(p.world_x is not None for p in [p1, matches2[0], matches3[0]]):
                world_x = (p1.world_x + matches2[0].world_x + matches3[0].world_x) / 3.0
                world_y = (p1.world_y + matches2[0].world_y + matches3[0].world_y) / 3.0
            
            candidate = CoordinatePoint(
                x=avg_x,
                y=avg_y,
                confidence=avg_confidence,
                world_x=world_x,
                world_y=world_y,
                label_id=p1.label_id
            )
            
            # 檢查是否與已加入的結果重疊
            overlap = any(candidate.distance_to(r) < circle_diameter for r in result)
            if not overlap:
                result.append(candidate)
        
        self.logger.info(f"Best算法完成: 輸入{len(capture1)}+{len(capture2)}+{len(capture3)}點, 輸出{len(result)}點")
        return result
    
    def find_algorithm(self, input_points: List[CoordinatePoint], 
                      safe_distance: float) -> List[CoordinatePoint]:
        """
        Find算法 - 安全距離篩選算法
        篩選出彼此距離大於安全距離的點
        
        Args:
            input_points: 輸入座標點列表
            safe_distance: 安全距離要求(mm)
        
        Returns:
            List[CoordinatePoint]: 篩選後的座標點
        """
        if not input_points:
            return []
        
        result = []
        remaining = input_points.copy()
        
        while remaining:
            # 選取第一個點
            current = remaining[0]
            result.append(current)
            
            # 移除所有與當前點距離小於安全距離的點
            remaining = [p for p in remaining if current.distance_to(p) >= safe_distance]
        
        self.logger.info(f"Find算法完成: 輸入{len(input_points)}點, 輸出{len(result)}點")
        return result
    
    def save_best_algorithm_results(self, results: List[CoordinatePoint]) -> bool:
        """
        保存Best算法結果 (覆蓋式) - 執行者模式
        由AutoFeeding模組透過Modbus旗標控制觸發
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        try:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            
            # 清空舊數據
            cursor.execute('DELETE FROM best_algorithm_results')
            
            # 插入新數據
            for i, point in enumerate(results):
                cursor.execute('''
                    INSERT INTO best_algorithm_results 
                    (x, y, confidence, world_x, world_y, label_id, source_count, timestamp)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                ''', (
                    point.x, point.y, point.confidence,
                    point.world_x, point.world_y, point.label_id,
                    3, timestamp  # source_count固定為3(三次拍攝)
                ))
            
            conn.commit()
            
            self.logger.info(f"Best算法結果已保存: {len(results)} 個點")
            return True
            
        except Exception as e:
            conn.rollback()
            self.logger.error(f"保存Best算法結果失敗: {e}")
            return False
            
        finally:
            conn.close()
    
    def get_best_algorithm_results(self, label_id: Optional[int] = None) -> List[CoordinatePoint]:
        """讀取Best算法結果"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        try:
            if label_id is not None:
                cursor.execute('''
                    SELECT x, y, confidence, world_x, world_y, label_id
                    FROM best_algorithm_results
                    WHERE label_id = ?
                    ORDER BY id
                ''', (label_id,))
            else:
                cursor.execute('''
                    SELECT x, y, confidence, world_x, world_y, label_id
                    FROM best_algorithm_results
                    ORDER BY label_id, id
                ''')
            
            points = []
            for row in cursor.fetchall():
                points.append(CoordinatePoint(
                    x=row[0], y=row[1], confidence=row[2],
                    world_x=row[3], world_y=row[4], label_id=row[5]
                ))
            
            return points
            
        finally:
            conn.close()
    
    def save_find_algorithm_results(self, results: List[CoordinatePoint], 
                                   safe_distance: float) -> bool:
        """
        保存Find算法結果 (覆蓋式) - 執行者模式
        由AutoFeeding模組透過Modbus旗標控制觸發
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        try:
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            
            # 清空舊數據
            cursor.execute('DELETE FROM find_algorithm_results')
            
            # 插入新數據
            for point in results:
                cursor.execute('''
                    INSERT INTO find_algorithm_results 
                    (x, y, confidence, world_x, world_y, label_id, safe_distance, timestamp)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?)
                ''', (
                    point.x, point.y, point.confidence,
                    point.world_x, point.world_y, point.label_id,
                    safe_distance, timestamp
                ))
            
            conn.commit()
            
            self.logger.info(f"Find算法結果已保存: {len(results)} 個點")
            return True
            
        except Exception as e:
            conn.rollback()
            self.logger.error(f"保存Find算法結果失敗: {e}")
            return False
            
        finally:
            conn.close()
    
    def get_find_algorithm_results(self, label_id: Optional[int] = None) -> List[CoordinatePoint]:
        """
        讀取Find算法結果 - 執行者模式
        由AutoProgram模組透過Modbus旗標控制觸發
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        try:
            if label_id is not None:
                cursor.execute('''
                    SELECT x, y, confidence, world_x, world_y, label_id
                    FROM find_algorithm_results
                    WHERE label_id = ?
                    ORDER BY id
                ''', (label_id,))
            else:
                cursor.execute('''
                    SELECT x, y, confidence, world_x, world_y, label_id
                    FROM find_algorithm_results
                    ORDER BY label_id, id
                ''')
            
            points = []
            for row in cursor.fetchall():
                points.append(CoordinatePoint(
                    x=row[0], y=row[1], confidence=row[2],
                    world_x=row[3], world_y=row[4], label_id=row[5]
                ))
            
            self.logger.debug(f"讀取Find算法結果: {len(points)} 個點")
            return points
            
        finally:
            conn.close()
    
    def get_processing_status(self, table_name: str) -> Dict[str, Any]:
        """
        獲取記錄數量 (供模組參考使用)
        注意: Modbus旗標控制由各模組自行管理，此處僅提供數據參考
        
        Args:
            table_name: 表名稱 ('ccd1_detection_results', 'best_algorithm_results', 'find_algorithm_results')
        
        Returns:
            Dict: 記錄數量等資訊
        """
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        try:
            cursor.execute(f'SELECT COUNT(*) FROM {table_name}')
            count = cursor.fetchone()[0]
            
            cursor.execute(f'SELECT timestamp FROM {table_name} ORDER BY id DESC LIMIT 1')
            last_row = cursor.fetchone()
            last_update = last_row[0] if last_row else ''
            
            return {
                'record_count': count,
                'last_update': last_update,
                'has_data': count > 0
            }
                
        finally:
            conn.close()
    
    def export_results_to_json(self, output_file: str, result_type: str = 'find') -> bool:
        """
        導出結果為JSON格式
        
        Args:
            output_file: 輸出檔案路徑
            result_type: 結果類型 ('ccd1', 'best', 'find')
        
        Returns:
            bool: 導出成功與否
        """
        try:
            if result_type == 'ccd1':
                points = self.get_ccd1_detection_results()
            elif result_type == 'best':
                points = self.get_best_algorithm_results()
            elif result_type == 'find':
                points = self.get_find_algorithm_results()
            else:
                raise ValueError(f"無效的結果類型: {result_type}")
            
            data = {
                'result_type': result_type,
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'count': len(points),
                'points': [point.to_dict() for point in points]
            }
            
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            
            self.logger.info(f"結果已導出到: {output_file}")
            return True
            
        except Exception as e:
            self.logger.error(f"導出結果失敗: {e}")
            return False
    
    def close(self):
        """關閉資料庫連接"""
        self.logger.info("CoordinateSupporter 已關閉")