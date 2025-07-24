# CoordinateSupporter 使用手冊

## 系統架構概述

### 整體流程設計
```
CCD1模組 → [Modbus旗標控制] → CoordinateSupporter.save_ccd1_detection_results()
    ↓
AutoFeeding模組 → [Modbus旗標控制] → CoordinateSupporter.get_ccd1_detection_results() 
                → Best算法 → Find算法 → CoordinateSupporter.save_find_algorithm_results()
    ↓
AutoProgram模組 → [Modbus旗標控制] → CoordinateSupporter.get_find_algorithm_results()
                → 傳送給Flow1 → 執行動作
```

### 控制方式說明

**CoordinateSupporter = 執行者**
- 只負責實際的SQLite讀寫操作
- 不控制讀寫時機，純粹聽命行事
- 各模組透過 Modbus 旗標自行控制何時觸發讀寫

**各模組 = 控制者**
- CCD1: 檢測完成後設置旗標，觸發 CoordinateSupporter 寫入
- AutoFeeding: 透過旗標確認可讀取，觸發 CoordinateSupporter 讀取
- AutoProgram: 透過旗標確認可讀取，觸發 CoordinateSupporter 讀取

### 核心算法說明

#### Best算法 (三次拍攝匹配)
- **目的**: 解決YOLO檢測的幻覺和飄移問題
- **原理**: 連續拍攝3張圖片，找出3次檢測中誤差範圍內都存在的點
- **輸入**: 3次檢測的座標列表 + 誤差容忍距離
- **輸出**: 確實存在的可信座標點

#### Find算法 (安全距離篩選)
- **目的**: 避免機械臂夾取時互相干擾
- **原理**: 篩選出彼此距離大於安全距離的點
- **輸入**: Best算法結果 + 安全距離要求
- **輸出**: 距離符合要求的座標點

## CoordinateSupporter 類別

### 初始化
```python
from CoordinateSupporter import CoordinateSupporter

# 初始化座標支援器
supporter = CoordinateSupporter(db_path="coordinate_supporter.db")
```

### 核心數據結構

#### CoordinatePoint
```python
@dataclass
class CoordinatePoint:
    x: float                    # X座標
    y: float                    # Y座標  
    confidence: float = 0.0     # 置信度
    world_x: Optional[float] = None    # 世界X座標
    world_y: Optional[float] = None    # 世界Y座標
    label_id: int = 0          # 標籤ID
```

## 使用場景範例

### 場景1: CCD1模組整合使用

```python
# CCD1模組中 - 通過Modbus旗標控制寫入
from CoordinateSupporter import CoordinateSupporter

class CCD1Integration:
    def __init__(self):
        self.coordinate_supporter = CoordinateSupporter()
        self.modbus_client = ModbusClient()  # 假設的Modbus客戶端
    
    def on_detection_complete(self, detection_dict, objects_detail):
        """檢測完成後的處理 - 由檢測流程觸發"""
        
        # 步驟1: 設置寫入旗標 (通知其他模組正在寫入)
        self.modbus_client.write_register('CCD1_RESULT_WRITING', 1)
        
        # 步驟2: 執行實際寫入操作
        success = self.coordinate_supporter.save_ccd1_detection_results(
            detection_dict, objects_detail
        )
        
        # 步驟3: 清除寫入旗標 (通知其他模組寫入完成，可讀取)
        self.modbus_client.write_register('CCD1_RESULT_WRITING', 0)
        self.modbus_client.write_register('CCD1_RESULT_READY', 1)
        
        return success
```

### 場景2: AutoFeeding模組使用

```python
# AutoFeeding模組 - 通過Modbus旗標控制三次拍攝處理
from CoordinateSupporter import CoordinateSupporter, CoordinatePoint

class AutoFeeding:
    def __init__(self):
        self.coordinate_supporter = CoordinateSupporter()
        self.modbus_client = ModbusClient()  # Modbus客戶端
        self.tolerance_mm = 5.0        # 誤差容忍距離
        self.circle_diameter = 60.0    # 圓直徑
        self.safe_distance = 80.0      # 安全距離
    
    def execute_triple_capture_process(self, label_id=0):
        """執行三次拍攝處理流程 - 通過Modbus旗標控制"""
        
        capture_results = []
        
        # 步驟1: 執行三次拍照檢測
        for i in range(3):
            print(f"第{i+1}次拍照檢測...")
            
            # 1.1 觸發CCD1拍照檢測 (透過Modbus指令)
            self.modbus_client.write_register('CCD1_CAPTURE_COMMAND', 16)
            
            # 1.2 等待檢測完成旗標
            while not self.modbus_client.read_register('CCD1_RESULT_READY'):
                time.sleep(0.1)
            
            # 1.3 讀取檢測結果 (CCD1已完成寫入)
            points = self.coordinate_supporter.get_ccd1_detection_results(label_id)
            capture_results.append(points)
            
            # 1.4 清除準備旗標，準備下次檢測
            self.modbus_client.write_register('CCD1_RESULT_READY', 0)
            
            print(f"第{i+1}次檢測到 {len(points)} 個點")
        
        # 步驟2: 執行Best算法
        best_results = self.coordinate_supporter.best_algorithm(
            capture_results[0], capture_results[1], capture_results[2],
            tolerance_mm=self.tolerance_mm, circle_diameter=self.circle_diameter
        )
        
        # 步驟3: 執行Find算法
        find_results = self.coordinate_supporter.find_algorithm(
            best_results, safe_distance=self.safe_distance
        )
        
        # 步驟4: 保存最終結果 (通過旗標控制)
        self.modbus_client.write_register('AUTOFEEDING_RESULT_WRITING', 1)
        
        self.coordinate_supporter.save_find_algorithm_results(
            find_results, self.safe_distance
        )
        
        self.modbus_client.write_register('AUTOFEEDING_RESULT_WRITING', 0)
        self.modbus_client.write_register('AUTOFEEDING_RESULT_READY', 1)
        
        print(f"處理完成: 最終篩選出 {len(find_results)} 個座標點")
        return find_results
```

### 場景3: AutoProgram模組使用

```python
# AutoProgram模組 - 通過Modbus旗標控制讀取篩選結果
from CoordinateSupporter import CoordinateSupporter

class AutoProgram:
    def __init__(self):
        self.coordinate_supporter = CoordinateSupporter()
        self.modbus_client = ModbusClient()  # Modbus客戶端
    
    def get_final_coordinates_for_flow1(self, label_id=0):
        """獲取最終座標供Flow1使用 - 通過Modbus旗標控制"""
        
        # 步驟1: 檢查AutoFeeding結果是否可讀
        if not self.modbus_client.read_register('AUTOFEEDING_RESULT_READY'):
            print("AutoFeeding結果尚未準備好")
            return []
        
        # 步驟2: 讀取最終篩選結果
        final_points = self.coordinate_supporter.get_find_algorithm_results(label_id)
        
        print(f"獲取到 {len(final_points)} 個最終座標點")
        return final_points
    
    def send_coordinates_to_flow1(self, points):
        """發送座標給Flow1執行"""
        
        # 步驟1: 設置Flow1數據寫入旗標
        self.modbus_client.write_register('FLOW1_DATA_WRITING', 1)
        
        # 步驟2: 將座標寫入Flow1讀取的地址
        for i, point in enumerate(points):
            print(f"座標{i+1}: ({point.x:.1f}, {point.y:.1f})")
            if point.world_x is not None:
                print(f"  世界座標: ({point.world_x:.2f}, {point.world_y:.2f})")
            
            # 寫入Flow1的Modbus寄存器
            self._write_point_to_flow1_registers(i, point)
        
        # 步驟3: 清除寫入旗標，設置數據準備旗標
        self.modbus_client.write_register('FLOW1_DATA_WRITING', 0)
        self.modbus_client.write_register('FLOW1_DATA_READY', 1)
        self.modbus_client.write_register('FLOW1_POINT_COUNT', len(points))
        
        # 步驟4: 發送執行指令給Flow1
        self.modbus_client.write_register('FLOW1_EXECUTE_COMMAND', 1)
        
        print(f"已發送 {len(points)} 個座標給Flow1執行")
    
    def _write_point_to_flow1_registers(self, index, point):
        """將座標點寫入Flow1的Modbus寄存器"""
        base_addr = 1000 + index * 10  # 假設每個點佔10個寄存器
        
        # 寫入像素座標
        self.modbus_client.write_register(base_addr, int(point.x))
        self.modbus_client.write_register(base_addr + 1, int(point.y))
        
        # 寫入世界座標 (如果有)
        if point.world_x is not None:
            world_x_int = int(point.world_x * 100)  # 轉為整數保存
            world_y_int = int(point.world_y * 100)
            self.modbus_client.write_register(base_addr + 2, world_x_int)
            self.modbus_client.write_register(base_addr + 3, world_y_int)
        
        # 寫入其他資訊
        self.modbus_client.write_register(base_addr + 4, int(point.confidence * 1000))
        self.modbus_client.write_register(base_addr + 5, point.label_id)
```

## SQLite表結構

### 主要表格

1. **ccd1_detection_results** - CCD1檢測原始結果
2. **best_algorithm_results** - Best算法處理結果  
3. **find_algorithm_results** - Find算法最終結果

### Modbus旗標控制模式

**CoordinateSupporter 不控制旗標，各模組自行管理:**

#### CCD1模組旗標
- `CCD1_RESULT_WRITING`: 正在寫入檢測結果 (1=寫入中, 0=完成)
- `CCD1_RESULT_READY`: 檢測結果可讀取 (1=可讀, 0=不可讀)

#### AutoFeeding模組旗標  
- `AUTOFEEDING_RESULT_WRITING`: 正在寫入篩選結果
- `AUTOFEEDING_RESULT_READY`: 篩選結果可讀取

#### AutoProgram/Flow1旗標
- `FLOW1_DATA_WRITING`: 正在寫入Flow1數據
- `FLOW1_DATA_READY`: Flow1數據可讀取
- `FLOW1_POINT_COUNT`: Flow1座標點數量
- `FLOW1_EXECUTE_COMMAND`: Flow1執行指令

## API 參考

### 基本操作

#### 保存CCD1檢測結果
```python
success = supporter.save_ccd1_detection_results(detection_dict, objects_detail)
```

#### 讀取CCD1檢測結果
```python
points = supporter.get_ccd1_detection_results(label_id=0)  # 指定標籤
all_points = supporter.get_ccd1_detection_results()        # 全部標籤
```

### 算法處理

#### Best算法
```python
best_results = supporter.best_algorithm(
    capture1=points1,
    capture2=points2, 
    capture3=points3,
    tolerance_mm=5.0,
    circle_diameter=60.0
)
```

#### Find算法
```python
find_results = supporter.find_algorithm(
    input_points=best_results,
    safe_distance=80.0
)
```

### 結果保存與讀取

#### 保存結果
```python
supporter.save_best_algorithm_results(best_results)
supporter.save_find_algorithm_results(find_results, safe_distance=80.0)
```

#### 讀取結果
```python
best_points = supporter.get_best_algorithm_results(label_id=0)
find_points = supporter.get_find_algorithm_results(label_id=0)
```

### 狀態檢查

#### 檢查記錄數量 (供參考)
```python
status = supporter.get_processing_status('find_algorithm_results')
print(f"記錄數量: {status['record_count']}")
print(f"最後更新: {status['last_update']}")
print(f"有數據: {status['has_data']}")
```

### 數據導出

#### 導出為JSON
```python
# 導出Find算法結果
supporter.export_results_to_json('final_results.json', 'find')

# 導出Best算法結果  
supporter.export_results_to_json('best_results.json', 'best')

# 導出CCD1原始結果
supporter.export_results_to_json('ccd1_results.json', 'ccd1')
```

## 完整使用範例

```python
from CoordinateSupporter import CoordinateSupporter, CoordinatePoint
import time

# 初始化
supporter = CoordinateSupporter()

# 模擬CCD1檢測結果保存
detection_dict = {0: 4, 1: 2}  # 標籤0有4個，標籤1有2個
objects_detail = {
    0: [
        {'x': 100, 'y': 200, 'confidence': 0.95, 'world_x': 50.5, 'world_y': 80.2},
        {'x': 150, 'y': 220, 'confidence': 0.87, 'world_x': 55.1, 'world_y': 82.1},
        {'x': 120, 'y': 180, 'confidence': 0.92, 'world_x': 52.3, 'world_y': 78.9},
        {'x': 180, 'y': 240, 'confidence': 0.89, 'world_x': 57.8, 'world_y': 84.5}
    ],
    1: [
        {'x': 300, 'y': 400, 'confidence': 0.88, 'world_x': 85.2, 'world_y': 120.3},
        {'x': 320, 'y': 420, 'confidence': 0.91, 'world_x': 87.1, 'world_y': 122.8}
    ]
}

# 保存檢測結果
supporter.save_ccd1_detection_results(detection_dict, objects_detail)

# 模擬三次拍攝處理
label_id = 0  # 處理標籤0
points1 = supporter.get_ccd1_detection_results(label_id)

# 模擬第二次和第三次拍攝 (略有位置偏移)
points2 = [CoordinatePoint(p.x + 2, p.y + 1, p.confidence, p.world_x, p.world_y, p.label_id) for p in points1]
points3 = [CoordinatePoint(p.x - 1, p.y + 2, p.confidence, p.world_x, p.world_y, p.label_id) for p in points1]

# 執行Best算法
best_results = supporter.best_algorithm(points1, points2, points3, tolerance_mm=5.0)
supporter.save_best_algorithm_results(best_results)

# 執行Find算法
find_results = supporter.find_algorithm(best_results, safe_distance=50.0)
supporter.save_find_algorithm_results(find_results, safe_distance=50.0)

# 檢查最終結果
print(f"最終篩選結果: {len(find_results)} 個座標點")
for i, point in enumerate(find_results):
    print(f"點{i+1}: ({point.x:.1f}, {point.y:.1f}) 世界座標: ({point.world_x:.2f}, {point.world_y:.2f})")

# 導出結果
supporter.export_results_to_json('final_coordinates.json', 'find')
```

## 注意事項

1. **執行者模式**: CoordinateSupporter 只執行讀寫，不控制時機
2. **Modbus旗標**: 各模組自行管理旗標，協調讀寫時機  
3. **覆蓋式寫入**: 每次保存都會清空舊數據，無歷史記錄
4. **線程安全**: 透過SQLite事務確保數據一致性
5. **標籤ID**: 支援多標籤處理，可指定特定標籤或處理全部
6. **座標精度**: 支援像素座標和世界座標雙重存儲

## 故障排除

### 常見問題

1. **讀取空數據**: 檢查對應模組的 `READY` 旗標是否設置
2. **算法無結果**: 檢查容忍距離和安全距離設定
3. **座標偏移**: 確認三次拍攝的一致性
4. **旗標衝突**: 確保各模組正確管理旗標狀態

### 除錯方法

1. 檢查記錄數量: `get_processing_status()`
2. 導出中間結果: `export_results_to_json()`  
3. 檢查日誌輸出: 算法會輸出處理統計信息
4. 驗證Modbus旗標: 確認各模組旗標設置正確