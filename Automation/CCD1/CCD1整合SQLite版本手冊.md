# CCD1視覺控制系統 SQLite整合版本使用手冊

## 版本資訊
- **系統版本**: CCD1 YOLOv11 純Modbus版本 v5.2
- **功能特性**: 整合CoordinateSupporter SQLite支援
- **更新日期**: 2025年1月

---

## 📊 系統概述

### 核心功能
- YOLOv11物件檢測 (DR_F/STACK分類)
- Modbus TCP Client架構 (握手式狀態機控制)
- **SQLite數據庫整合** (新增)
- 座標轉換 (像素座標 ↔ 世界座標)
- 自動初始化和自我重載
- 記憶體使用量監控

### 系統架構流程
```
PLC控制指令 → CCD1檢測 → Modbus寄存器 + SQLite數據庫 → AutoFeeding讀取
```

---

## 🚀 啟動方式

### 1. 直接運行
```bash
python CCD1VisionCode_YOLOv11_ModbusOnly.py
```

### 2. 自動初始化組件
系統啟動時會自動：
- 連接Modbus TCP服務器 (127.0.0.1:502)
- 載入標定檔案 (內外參數)
- 初始化相機連接 (192.168.1.8)
- 載入YOLO模型檔案
- **初始化SQLite數據庫** (新增)

---

## 📋 Modbus寄存器映射

### 控制寄存器 (200-206)
| 地址 | 名稱 | 功能 | 說明 |
|------|------|------|------|
| 200 | CONTROL_COMMAND | 控制指令 | 8=拍照, 16=拍照+檢測, 32=初始化 |
| 201 | STATUS_REGISTER | 狀態寄存器 | bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized |
| 202 | MODEL_SELECT | 模型選擇 | 0=未載入, 1-20=模型ID |
| 203 | CAPTURE_COMPLETE | 拍照完成 | 1=完成, 0=未完成 |
| 204 | DETECT_COMPLETE | 檢測完成 | 1=完成, 0=未完成 |
| 205 | OPERATION_SUCCESS | 操作成功 | 1=成功, 0=失敗 |
| 206 | ERROR_CODE | 錯誤代碼 | 錯誤類型編號 |

### 檢測參數 (210-211)
| 地址 | 名稱 | 功能 | 說明 |
|------|------|------|------|
| 210-211 | CONFIDENCE_HIGH/LOW | 置信度閾值 | ×10000存儲，預設8000(0.8) |

### 檢測結果 (240-259)
| 地址 | 名稱 | 功能 | 說明 |
|------|------|------|------|
| 240 | DR_F_COUNT | DR_F數量 | 檢測到的DR_F物件數量 |
| 242 | STACK_COUNT | STACK數量 | 檢測到的STACK物件數量 |
| 243 | TOTAL_DETECTIONS | 總檢測數量 | DR_F + STACK 總數 |
| 244 | DETECTION_SUCCESS | 檢測成功標誌 | 1=成功, 0=失敗 |
| 245-254 | DR_F座標 | DR_F物件座標 | 最多5個DR_F的X,Y座標 |
| 257-258 | STACK座標 | STACK物件座標 | 第1個STACK的X,Y座標 |
| 259 | MODEL_ID_USED | 使用模型ID | 本次檢測使用的模型編號 |

### 世界座標 (260-280)
| 地址範圍 | 功能 | 說明 |
|----------|------|------|
| 260 | WORLD_COORD_VALID | 世界座標有效性 | 1=有效, 0=無效 |
| 261-280 | DR_F世界座標 | DR_F世界座標 | 前5個DR_F的世界座標(×100存儲) |

### 系統狀態 (281-299)
| 地址 | 名稱 | 功能 | 說明 |
|------|------|------|------|
| 281-283 | 時間統計 | 耗時資訊 | 拍照/處理/總耗時(ms) |
| 284-287 | 計數統計 | 計數資訊 | 操作/錯誤/連接/模型切換次數 |
| 290-293 | 版本運行時間 | 系統資訊 | 版本號(5.2)/運行時間 |
| 295 | MEMORY_USAGE_MB | 記憶體使用量 | 當前記憶體使用量(MB) |
| 296 | SYSTEM_RELOAD | 系統重載觸發 | 寫入1觸發重載 |
| 297 | RELOAD_STATUS | 重載狀態 | 1=重載中, 0=完成 |
| **298** | **SQLITE_WRITE_COMPLETE** | **SQLite寫入完成** | **1=完成, 0=已清除** ⭐ |

---

## 🆕 SQLite整合功能

### SQLite檔案位置
- **檔案名稱**: `ccd1_coordinate_supporter.db`
- **檔案位置**: 與CCD1 Python檔案同一目錄
- **自動創建**: 首次運行時自動生成

### 數據表結構

#### 1. ccd1_detection_results (CCD1檢測結果)
| 欄位 | 類型 | 說明 |
|------|------|------|
| id | INTEGER | 主鍵 |
| label_id | INTEGER | 標籤ID (0=DR_F, 1=STACK) |
| x | REAL | X座標 |
| y | REAL | Y座標 |
| confidence | REAL | **YOLO實際置信度** ⭐ |
| world_x | REAL | 世界X座標 (如果有標定) |
| world_y | REAL | 世界Y座標 (如果有標定) |
| timestamp | TEXT | 時間戳記 |

#### 數據示例
```sql
id | label_id | x     | y     | confidence | world_x | world_y | timestamp
1  | 0        | 100.5 | 200.3 | 0.95      | 50.2    | 80.1    | 2025-01-20 10:30:15.123
2  | 0        | 150.2 | 220.8 | 0.87      | 55.1    | 82.3    | 2025-01-20 10:30:15.123
3  | 1        | 300.1 | 400.5 | 0.88      | NULL    | NULL    | 2025-01-20 10:30:15.123
```

---

## 🔄 操作流程

### 1. 正常檢測流程
```
1. PLC寫入寄存器200=16 (拍照+檢測指令)
2. CCD1檢查寄存器298是否為0 (SQLite旗標檢查)
3. 執行YOLO檢測
4. 寫入Modbus寄存器 (原有邏輯)
5. 寫入SQLite數據庫 (新增)
6. 設置寄存器298=1 (通知AutoFeeding可讀取)
```

### 2. 防重複檢測機制
```
1. PLC再次寫入寄存器200=16
2. CCD1檢查寄存器298=1 (旗標未清除)
3. 設置ERROR_CODE=50 + ALARM狀態
4. 拒絕執行檢測，保護數據完整性
```

### 3. AutoFeeding清除流程
```
1. AutoFeeding透過CoordinateSupporter讀取SQLite數據
2. 處理Best/Find算法
3. 完成後寫入寄存器298=0 (清除旗標)
4. CCD1可接受下次檢測指令
```

---

## ⚠️ 錯誤代碼表

| 錯誤代碼 | 說明 | 處理方式 |
|----------|------|----------|
| 1-49 | 原有錯誤代碼 | 參考原有文檔 |
| **50** | **SQLite寫入未完成錯誤** | 等待AutoFeeding清除旗標298 |
| **51** | **SQLite旗標檢查異常** | 檢查Modbus連接狀態 |

---

## 🔧 控制指令

### 基本控制
```python
# 拍照指令
modbus_client.write_register(200, 8)

# 拍照+檢測指令 (會觸發SQLite寫入)
modbus_client.write_register(200, 16)

# 初始化指令
modbus_client.write_register(200, 32)

# 清除指令
modbus_client.write_register(200, 0)
```

### 模型管理
```python
# 切換到模型1
modbus_client.write_register(202, 1)

# 切換到模型5
modbus_client.write_register(202, 5)

# 卸載模型
modbus_client.write_register(202, 0)
```

### 系統管理
```python
# 觸發系統重載
modbus_client.write_register(296, 1)

# 檢查重載狀態
reload_status = modbus_client.read_register(297)  # 1=重載中, 0=完成

# 檢查記憶體使用量
memory_mb = modbus_client.read_register(295)  # 記憶體使用量(MB)
```

### SQLite旗標管理 (主要由AutoFeeding控制)
```python
# 檢查SQLite寫入狀態
sqlite_flag = modbus_client.read_register(298)  # 1=有新數據, 0=已讀取

# AutoFeeding清除旗標 (讀取完成後)
modbus_client.write_register(298, 0)
```

---

## 📈 監控和狀態檢查

### 系統狀態監控
```python
# 檢查基本狀態
status_reg = modbus_client.read_register(201)
ready = bool(status_reg & 0x01)      # bit0
running = bool(status_reg & 0x02)    # bit1  
alarm = bool(status_reg & 0x04)      # bit2
initialized = bool(status_reg & 0x08) # bit3

# 檢查檢測結果
dr_f_count = modbus_client.read_register(240)
stack_count = modbus_client.read_register(242)
total_count = modbus_client.read_register(243)
success = modbus_client.read_register(244)

# 檢查SQLite狀態
sqlite_complete = modbus_client.read_register(298)
print(f"SQLite寫入完成: {'是' if sqlite_complete else '否'}")
```

### 性能監控
```python
# 檢查時間統計
capture_time = modbus_client.read_register(281)   # 拍照耗時(ms)
process_time = modbus_client.read_register(282)   # 處理耗時(ms)
total_time = modbus_client.read_register(283)     # 總耗時(ms)

# 檢查記憶體使用
memory_mb = modbus_client.read_register(295)      # 記憶體使用量(MB)

# 檢查操作統計
operation_count = modbus_client.read_register(284) # 操作次數
error_count = modbus_client.read_register(285)     # 錯誤次數
```

---

## 🔗 與AutoFeeding模組整合

### AutoFeeding讀取流程
```python
# 1. 檢查CCD1是否有新數據
sqlite_flag = modbus_client.read_register(298)
if sqlite_flag == 1:
    # 2. 讀取SQLite數據
    points = coordinate_supporter.get_ccd1_detection_results()
    
    # 3. 執行三次拍攝算法
    # ... Best算法和Find算法處理 ...
    
    # 4. 清除旗標，允許CCD1下次檢測
    modbus_client.write_register(298, 0)
```

### 數據格式
AutoFeeding從SQLite讀取的數據格式：
```python
# CoordinatePoint物件
point.x          # X座標
point.y          # Y座標  
point.confidence # YOLO實際置信度 (0.0-1.0)
point.world_x    # 世界X座標 (如果有)
point.world_y    # 世界Y座標 (如果有)
point.label_id   # 0=DR_F, 1=STACK
```

---

## 📁 檔案結構

```
CCD1/
├── CCD1VisionCode_YOLOv11_ModbusOnly.py  # 主程式
├── ccd1_coordinate_supporter.db          # SQLite數據庫 (自動生成)
├── camera_matrix_DR.npy                  # 內參檔案
├── extrinsic_DR.npy                      # 外參檔案
├── dist_coeffs_DR.npy                    # 畸變係數檔案 (可選)
├── best.pt                               # YOLO模型檔案
├── model_1.pt                            # 其他模型檔案 (可選)
├── model_2.pt                            # 其他模型檔案 (可選)
└── ...
```

---

## 🛠️ 故障排除

### 常見問題

#### 1. SQLite相關錯誤
```
錯誤: SQLite寫入完成旗標未被清除
原因: AutoFeeding未正確清除寄存器298
解決: 手動寫入寄存器298=0，或重啟AutoFeeding模組
```

#### 2. 數據庫檔案問題
```
錯誤: CoordinateSupporter初始化失敗
原因: 資料庫檔案權限或路徑問題
解決: 檢查檔案權限，確保有寫入權限
```

#### 3. 檢測被拒絕
```
錯誤: ERROR_CODE=50
原因: 上次檢測的SQLite數據未被AutoFeeding讀取
解決: 等待AutoFeeding處理，或手動清除旗標
```

### 除錯指令
```python
# 檢查SQLite狀態
sqlite_flag = modbus_client.read_register(298)
print(f"SQLite旗標: {sqlite_flag}")

# 檢查錯誤代碼
error_code = modbus_client.read_register(206)
print(f"錯誤代碼: {error_code}")

# 強制清除SQLite旗標 (緊急情況使用)
modbus_client.write_register(298, 0)

# 觸發系統重載 (重置所有狀態)
modbus_client.write_register(296, 1)
```

---

## 📝 使用注意事項

### 1. SQLite數據管理
- 每次檢測會**覆蓋**SQLite中的舊數據
- 數據不保留歷史記錄，僅保存最新檢測結果
- AutoFeeding必須及時讀取數據，避免數據丟失

### 2. 旗標協調
- 寄存器298是CCD1和AutoFeeding的協調機制
- **CCD1負責設置為1**，**AutoFeeding負責清除為0**
- 絕對不要手動長期設置為1，會阻止後續檢測

### 3. 性能考量
- SQLite寫入增加了少量檢測耗時 (通常<10ms)
- 記憶體使用量會稍微增加
- 每分鐘自動更新記憶體使用量到寄存器295

### 4. 容錯處理
- SQLite寫入失敗不會影響Modbus檢測結果
- 系統優先保證基本檢測功能正常
- 發生異常時會在日誌中詳細記錄

---

## 🔄 版本更新記錄

### v5.2 (CoordinateSupporter整合版)
- ✅ 新增SQLite數據庫支援
- ✅ 新增防重複檢測機制
- ✅ 新增寄存器298 (SQLite寫入完成旗標)
- ✅ 記錄YOLO實際置信度到數據庫
- ✅ 錯誤代碼50/51支援
- ✅ 與AutoFeeding模組整合就緒

### v5.1 (純Modbus版)
- ✅ 移除Web介面
- ✅ 自動初始化功能
- ✅ 記憶體監控
- ✅ 系統自我重載

---

## 📞 技術支援

### 日誌查看
系統運行時會在控制台輸出詳細日誌，包括：
- SQLite寫入狀態
- 旗標檢查結果  
- 檢測結果統計
- 錯誤詳細資訊

### 數據驗證
可以使用SQLite瀏覽器工具查看數據庫內容：
```sql
-- 查看最新檢測結果
SELECT * FROM ccd1_detection_results ORDER BY id DESC;

-- 統計各類物件數量
SELECT label_id, COUNT(*) as count FROM ccd1_detection_results GROUP BY label_id;
```

---

**更新完成日期**: 2025年1月  
**適用版本**: CCD1 YOLOv11 純Modbus版本 v5.2 (CoordinateSupporter整合版)