# AutoFeeding模組SQLite增強版技術文檔

## 版本資訊
- **版本**: AutoFeeding SQLite增強版 v2.0
- **基地址**: 900-999
- **新增核心功能**: SQLite篩選結果寫入、AutoProgram旗標管理
- **更新日期**: 2025年7月

---

## 📊 系統概述

### 核心增強功能
基於原有AutoFeeding模組，新增以下核心功能：

1. **SQLite篩選結果寫入**: 將Best+Find算法篩選後的座標自動寫入SQLite數據庫
2. **AutoProgram旗標管理**: 透過970寄存器與AutoProgram模組協調
3. **智能檢測執行邏輯**: 只有在條件滿足時才執行新的檢測
4. **向後兼容性**: 保持原有960-964寄存器交握功能

### 系統架構流程
```
AutoFeeding檢測 → 篩選出N個座標 → 寫入SQLite → 設置旗標970=1 
                                                    ↓
AutoProgram讀取SQLite ← 旗標被清0 ← AutoProgram讀取完成
        ↓
重新開始檢測循環
```

---

## 🚀 新增功能詳解

### 1. SQLite篩選結果寫入功能

#### 觸發條件
- 三次拍攝+Best算法+Find算法完成
- 篩選結果數量 > 0 (至少1個座標)

#### 存儲位置
```
數據庫文件: C:\Users\user\Documents\GitHub\DobotDR\Automation\CCD1\ccd1_coordinate_supporter.db
存儲表格: find_algorithm_results
```

#### 存儲內容
- 像素座標 (x, y)
- 世界座標 (world_x, world_y) 
- 置信度 (confidence)
- 標籤ID (label_id = 0, DR_F正面物件)
- 時間戳 (timestamp)

#### 核心方法
```python
def save_screening_results_to_sqlite(self, final_results: List[CoordinatePoint]) -> bool:
    """將篩選結果保存到SQLite數據庫"""
    # 只有當len(final_results) > 0時才寫入
    # 使用coordinate_supporter.save_find_algorithm_results()
```

### 2. AutoProgram旗標管理系統

#### 核心旗標地址
- **970寄存器**: AutoFeeding篩選完成旗標
  - `1` = 有結果可讀取 (AutoFeeding設置)
  - `0` = 已被讀取 (AutoProgram清除)

#### 協調邏輯
```python
def set_screening_complete_flag(self) -> bool:
    """設置篩選完成旗標，通知AutoProgram可以讀取"""
    
def check_screening_flag_cleared(self) -> bool:
    """檢查篩選完成旗標是否被AutoProgram清除"""
    
def should_execute_detection(self) -> bool:
    """判斷是否應該執行檢測
    執行條件：
    1. 外部控制啟用 (920=1)
    2. 篩選完成旗標被清除 (970=0)
    """
```

### 3. 智能檢測執行邏輯

#### 檢測執行條件
AutoFeeding只有在同時滿足以下條件時才執行新的檢測：
1. **外部控制啟用**: 920寄存器 = 1
2. **篩選旗標已清除**: 970寄存器 = 0 (AutoProgram已讀取完成)

#### 執行流程
```
檢測完成 → SQLite寫入 → 設置970=1 → 等待AutoProgram讀取 
                                        ↓
970=0且920=1 → 重新執行檢測 ← AutoProgram清除970=0
```

### 4. 向後兼容性功能

#### 兼容性座標設置
篩選結果的第一個座標會同時設置到原有960-964寄存器：
```python
# 兼容性設置：第一個座標到960-964寄存器
first_point = final_results[0]
target_coords = (first_point.world_x, first_point.world_y)
self.set_dr_f_available(target_coords)
```

---

## 📋 寄存器映射表

### 狀態寄存器 (900-919) - 只讀
| 地址 | 功能 | 類型 | 說明 |
|------|------|------|------|
| 900-905 | 原有狀態統計 | 原有 | 模組狀態、計數器等 |
| 906 | 記憶體使用量(MB) | 原有 | 每10分鐘更新 |
| 907-912 | 原有狀態 | 原有 | 錯誤代碼、操作狀態等 |
| **913** | **篩選旗標狀態監控** | **新增** | **顯示當前970旗標狀態(調試用)** |

### 控制寄存器 (920-929) - 讀寫
| 地址 | 功能 | 說明 |
|------|------|------|
| 920 | 運行控制 | 0=停止, 1=啟動 |
| 921-929 | 保留 | 預留擴展 |

### VP參數寄存器 (930-949) - 讀寫
| 地址 | 功能 | 預設值 |
|------|------|--------|
| 930-933 | 第一組震動參數 | 4,45,43,800 |
| 934-937 | 第二組震動參數 | 11,98,64,800 |
| 938-939 | 停止參數 | 3,100 |

### Flow4參數寄存器 (950-959) - 讀寫
| 地址 | 功能 | 預設值 |
|------|------|--------|
| 950 | 脈衝持續時間(ms) | 100 |
| 951 | 脈衝間隔(ms) | 50 |

### 座標交握寄存器 (960-979) - 混合讀寫
| 地址 | 功能 | 說明 |
|------|------|------|
| 960 | DR_F可用標誌 | 兼容性功能 |
| 961-964 | DR_F座標 | 兼容性功能(第一個座標) |
| 965 | 座標已讀取標誌 | 兼容性功能 |
| **970** | **篩選完成旗標** | **核心新功能** |

### 系統參數寄存器 (980-999) - 讀寫
| 地址 | 功能 | 預設值 |
|------|------|--------|
| 980 | 檢測週期間隔(ms) | 1000 |
| 981 | CCD1超時(ms) | 5000 |
| 982-993 | 其他系統參數 | 各種閾值設定 |

---

## 🛠️ 使用方式

### 外部控制AutoFeeding (SQLite增強版)
```python
from pymodbus.client import ModbusTcpClient

client = ModbusTcpClient('127.0.0.1', port=502)
client.connect()

# 啟動AutoFeeding (具備SQLite寫入功能)
client.write_register(920, 1, slave=1)

# 檢查運行狀態
status = client.read_holding_registers(900, 1, slave=1).registers[0]
print(f"AutoFeeding狀態: {status}")

# 監控篩選完成旗標
screening_flag = client.read_holding_registers(970, 1, slave=1).registers[0]
print(f"篩選完成旗標: {screening_flag} (1=可讀, 0=已讀)")

# 監控旗標狀態 (調試用)
flag_status = client.read_holding_registers(913, 1, slave=1).registers[0]
print(f"旗標狀態監控: {flag_status}")

client.close()
```

### AutoProgram讀取篩選結果
```python
from CoordinateSupporter import CoordinateSupporter

# 1. 檢查篩選完成旗標
screening_flag = modbus_client.read_register(970)
if screening_flag == 1:
    # 2. 讀取SQLite篩選結果
    supporter = CoordinateSupporter(db_path="ccd1_coordinate_supporter.db")
    results = supporter.get_find_algorithm_results(label_id=0)
    
    print(f"讀取到 {len(results)} 個篩選結果:")
    for i, point in enumerate(results):
        print(f"  點{i+1}: 世界座標({point.world_x:.2f}, {point.world_y:.2f})mm")
    
    # 3. 處理完成後清除旗標
    modbus_client.write_register(970, 0)
```

### SQLite數據查看
```python
# 方法1: 使用CoordinateSupporter
from CoordinateSupporter import CoordinateSupporter

supporter = CoordinateSupporter(db_path="C:/Users/user/Documents/GitHub/DobotDR/Automation/CCD1/ccd1_coordinate_supporter.db")
results = supporter.get_find_algorithm_results()

print(f"當前SQLite中有 {len(results)} 個篩選結果")
for point in results:
    print(f"像素({point.x:.1f}, {point.y:.1f}) -> 世界({point.world_x:.2f}, {point.world_y:.2f})mm")
```

```sql
-- 方法2: 直接SQL查詢
SELECT * FROM find_algorithm_results ORDER BY id DESC LIMIT 10;
```

---

## 🔄 運作流程

### 完整工作流程
```
1. 外部控制啟動 (920=1)
   ↓
2. 檢查篩選旗標 (970=0?)
   ↓
3. 執行三次拍攝+Best+Find算法
   ↓
4. 篩選出N個座標 (N>0?)
   ↓
5. 寫入SQLite數據庫
   ↓
6. 設置篩選完成旗標 (970=1)
   ↓
7. 設置兼容性座標 (960-964，第一個座標)
   ↓
8. 等待AutoProgram讀取
   ↓
9. AutoProgram清除旗標 (970=0)
   ↓
10. 重新開始檢測循環
```

### 異常處理流程
```
SQLite寫入失敗 → 記錄錯誤 → 不設置旗標 → 重試檢測
旗標設置失敗 → 記錄錯誤 → 本週期失敗 → 重試檢測
無篩選結果 → 觸發Flow4送料 → VP震動清空 → 重新檢測
```

---

## ⚠️ 重要注意事項

### 部署要求
1. **依賴模組**: CoordinateSupporter.py必須在API目錄
2. **SQLite路徑**: 確保C:\Users\user\Documents\GitHub\DobotDR\Automation\CCD1\目錄存在
3. **pymodbus版本**: 必須使用pymodbus==3.9.2
4. **psutil套件**: 用於記憶體監控

### 操作順序
1. 啟動主Modbus TCP Server (端口502)
2. 啟動CCD1視覺檢測模組 (v5.2以上，支援SQLite)
3. 啟動VP震動盤模組
4. **啟動AutoFeeding模組 (SQLite增強版)**
5. **最後啟動AutoProgram模組**

### 數據特性
- **覆蓋式存儲**: 每次檢測會覆蓋SQLite中舊數據
- **旗標協調**: 970寄存器是AutoFeeding和AutoProgram的核心協調機制
- **向後兼容**: 保持960-964寄存器功能，不影響現有Flow1程序

### 監控建議
- **913寄存器**: 監控當前旗標狀態（調試用）
- **906寄存器**: 監控記憶體使用量
- **900寄存器**: 監控模組整體狀態

---

## 🐛 故障排除

### 常見問題

#### 1. SQLite寫入失敗
```
症狀: 日誌顯示"SQLite保存失敗"
原因: CoordinateSupporter初始化失敗或數據庫路徑問題
解決: 檢查API目錄下是否有CoordinateSupporter.py
```

#### 2. 旗標設置失敗
```
症狀: 日誌顯示"旗標設置失敗"
原因: Modbus寫入970寄存器失敗
解決: 檢查Modbus連接狀態和寄存器範圍
```

#### 3. AutoProgram無法讀取
```
症狀: AutoProgram等待AutoFeeding結果超時
原因: 970旗標未設置或SQLite數據為空
解決: 檢查913寄存器顯示的旗標狀態
```

#### 4. 檢測不執行
```
症狀: 920=1但不執行檢測
原因: 970旗標未被清除(AutoProgram未讀取)
解決: 手動清除970寄存器或重新啟動AutoProgram
```

### 調試指令
```python
# 檢查核心狀態
screening_flag = read_register(970)  # 篩選完成旗標
flag_monitor = read_register(913)    # 旗標狀態監控
external_control = read_register(920) # 外部控制狀態

print(f"外部控制: {external_control}, 篩選旗標: {screening_flag}, 監控: {flag_monitor}")

# 強制清除旗標 (緊急情況)
write_register(970, 0)

# 檢查SQLite記錄數量
from CoordinateSupporter import CoordinateSupporter
supporter = CoordinateSupporter()
results = supporter.get_find_algorithm_results()
print(f"SQLite中有 {len(results)} 個記錄")
```

---

## 📈 性能特性

### 記憶體使用
- **監控頻率**: 每10分鐘更新906寄存器
- **存儲精度**: 整數MB，無條件捨去小數點
- **獨立執行緒**: 不影響主檢測邏輯

### 檢測效率
- **智能觸發**: 只有在條件滿足時才執行檢測
- **SQLite緩存**: 利用CoordinateSupporter的數據庫連接池
- **兼容並行**: 960-964寄存器和970旗標並行工作

### 穩定性保證
- **異常隔離**: SQLite錯誤不影響基本檢測功能
- **向後兼容**: 原有功能完全保留
- **資源管理**: 執行緒自動清理，無記憶體洩漏

---

**版本更新**: SQLite增強版 v2.0  
**適用場景**: DR專案AutoFeeding與AutoProgram協調工作  
**技術支援**: 透過913寄存器監控旗標狀態，透過日誌查看詳細運行信息