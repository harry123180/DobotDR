# CCD1視覺檢測模組握手控制說明檔案 v5.2 (更新版)

## 概述

CCD1視覺檢測模組YOLOv11版本採用DR_F/STACK二分類檢測(不含CASE_B)，基於握手式狀態機控制，使用Modbus TCP Client架構連接主服務器(127.0.0.1:502)。

## 基本信息

- **模組名稱**: CCD1視覺檢測模組 YOLOv11版本  
- **基地址範圍**: 200-299 (100個寄存器)
- **檢測類型**: YOLOv11物件檢測
- **檢測對象**: DR_F, STACK (二種分類)
- **輪詢頻率**: 50ms
- **版本號**: v5.2 (更新版)
- **相機IP**: 192.168.1.8
- **Web介面**: http://localhost:5051

## 修正後寄存器映射表

### 1. 核心控制握手寄存器 (200-206)

| 地址 | 名稱 | 功能 | 數值定義 | 讀寫 |
|------|------|------|----------|------|
| 200 | CONTROL_COMMAND | 控制指令 | 0=清空, 8=拍照, 16=拍照+檢測, 32=重新初始化 | R/W |
| 201 | STATUS_REGISTER | 狀態寄存器 | bit0=Ready, bit1=Running, bit2=Alarm, bit3=Initialized | R |
| 202 | MODEL_SELECT | 模型選擇 | 0=未指定, 1-20=模型ID | R/W |
| 203 | CAPTURE_COMPLETE | 拍照完成標誌 | 0=未完成, 1=已完成 | R |
| 204 | DETECT_COMPLETE | 檢測完成標誌 | 0=未完成, 1=已完成 | R |
| 205 | OPERATION_SUCCESS | 操作成功標誌 | 0=失敗, 1=成功 | R |
| 206 | ERROR_CODE | 錯誤代碼 | 0=無錯誤, >0=錯誤代碼 | R |

### 2. YOLOv11檢測參數寄存器 (210-219)

| 地址 | 名稱 | 功能 | 數值定義 | 讀寫 | 預設值 |
|------|------|------|----------|------|--------|
| 210 | CONFIDENCE_HIGH | 置信度閾值高位 | 32位置信度×10000高16位 | R/W | 0 |
| 211 | CONFIDENCE_LOW | 置信度閾值低位 | 32位置信度×10000低16位 | R/W | 8000 |
| 212 | RESERVED_212 | 保留參數1 | 未使用 | R/W | 0 |
| 213 | RESERVED_213 | 保留參數2 | 未使用 | R/W | 0 |
| 214 | RESERVED_214 | 保留參數3 | 未使用 | R/W | 0 |
| 215 | RESERVED_215 | 保留參數4 | 未使用 | R/W | 0 |
| 216-219 | - | 保留區域 | 未使用 | R/W | 0 |

**置信度閾值計算方式**:
- 實際值 = ((CONFIDENCE_HIGH << 16) + CONFIDENCE_LOW) / 10000.0
- 例如: 0.8 → 8000 → HIGH=0, LOW=8000

### 3. YOLOv11檢測結果寄存器 (240-259) 【與程式碼對齊】

| 地址 | 名稱 | 功能 | 數值定義 | 讀寫 |
|------|------|------|----------|------|
| 240 | DR_F_COUNT | DR_F檢測數量 | 0-5個目標 | R |
| 241 | **[預留]** | **預留CASE_B_COUNT** | **程式碼中未使用** | R |
| 242 | STACK_COUNT | STACK檢測數量 | 0-255個目標 | R |
| 243 | TOTAL_DETECTIONS | 總檢測數量 | DR_F + STACK | R |
| 244 | DETECTION_SUCCESS | 檢測成功標誌 | 0=失敗, 1=成功 | R |

### 4. DR_F座標寄存器 (245-254) 【與程式碼對齊】

| 地址 | 名稱 | 功能 | 數值定義 | 讀寫 |
|------|------|------|----------|------|
| 245 | DR_F_1_X | DR_F 1號 X座標 | 像素座標 | R |
| 246 | DR_F_1_Y | DR_F 1號 Y座標 | 像素座標 | R |
| 247 | DR_F_2_X | DR_F 2號 X座標 | 像素座標 | R |
| 248 | DR_F_2_Y | DR_F 2號 Y座標 | 像素座標 | R |
| 249 | DR_F_3_X | DR_F 3號 X座標 | 像素座標 | R |
| 250 | DR_F_3_Y | DR_F 3號 Y座標 | 像素座標 | R |
| 251 | DR_F_4_X | DR_F 4號 X座標 | 像素座標 | R |
| 252 | DR_F_4_Y | DR_F 4號 Y座標 | 像素座標 | R |
| 253 | DR_F_5_X | DR_F 5號 X座標 | 像素座標 | R |
| 254 | DR_F_5_Y | DR_F 5號 Y座標 | 像素座標 | R |

### 5. 擴展檢測結果寄存器 (255-259) 【與程式碼對齊】

| 地址 | 名稱 | 功能 | 數值定義 | 讀寫 |
|------|------|------|----------|------|
| 255 | **[預留]** | **預留CASE_B_1_X** | **程式碼中未使用** | R |
| 256 | **[預留]** | **預留CASE_B_1_Y** | **程式碼中未使用** | R |
| 257 | STACK_1_X | STACK 1號 X座標 | 像素座標 | R |
| 258 | STACK_1_Y | STACK 1號 Y座標 | 像素座標 | R |
| 259 | MODEL_ID_USED | 檢測使用模型ID | 實際使用的模型編號 | R |

### 6. 世界座標寄存器 (260-280) 【與程式碼對齊】

| 地址 | 名稱 | 功能 | 數值定義 | 讀寫 |
|------|------|------|----------|------|
| 260 | WORLD_COORD_VALID | 世界座標有效標誌 | 0=無效, 1=有效 | R |
| 261 | DR_F_1_WORLD_X_HIGH | DR_F 1號世界X座標高位 | 32位世界X座標×100高16位 | R |
| 262 | DR_F_1_WORLD_X_LOW | DR_F 1號世界X座標低位 | 32位世界X座標×100低16位 | R |
| 263 | DR_F_1_WORLD_Y_HIGH | DR_F 1號世界Y座標高位 | 32位世界Y座標×100高16位 | R |
| 264 | DR_F_1_WORLD_Y_LOW | DR_F 1號世界Y座標低位 | 32位世界Y座標×100低16位 | R |
| 265 | DR_F_2_WORLD_X_HIGH | DR_F 2號世界X座標高位 | 同上格式 | R |
| 266 | DR_F_2_WORLD_X_LOW | DR_F 2號世界X座標低位 | 同上格式 | R |
| 267 | DR_F_2_WORLD_Y_HIGH | DR_F 2號世界Y座標高位 | 同上格式 | R |
| 268 | DR_F_2_WORLD_Y_LOW | DR_F 2號世界Y座標低位 | 同上格式 | R |
| 269 | DR_F_3_WORLD_X_HIGH | DR_F 3號世界X座標高位 | 同上格式 | R |
| 270 | DR_F_3_WORLD_X_LOW | DR_F 3號世界X座標低位 | 同上格式 | R |
| 271 | DR_F_3_WORLD_Y_HIGH | DR_F 3號世界Y座標高位 | 同上格式 | R |
| 272 | DR_F_3_WORLD_Y_LOW | DR_F 3號世界Y座標低位 | 同上格式 | R |
| 273 | DR_F_4_WORLD_X_HIGH | DR_F 4號世界X座標高位 | 同上格式 | R |
| 274 | DR_F_4_WORLD_X_LOW | DR_F 4號世界X座標低位 | 同上格式 | R |
| 275 | DR_F_4_WORLD_Y_HIGH | DR_F 4號世界Y座標高位 | 同上格式 | R |
| 276 | DR_F_4_WORLD_Y_LOW | DR_F 4號世界Y座標低位 | 同上格式 | R |
| 277 | DR_F_5_WORLD_X_HIGH | DR_F 5號世界X座標高位 | 同上格式 | R |
| 278 | DR_F_5_WORLD_X_LOW | DR_F 5號世界X座標低位 | 同上格式 | R |
| 279 | DR_F_5_WORLD_Y_HIGH | DR_F 5號世界Y座標高位 | 同上格式 | R |
| 280 | DR_F_5_WORLD_Y_LOW | DR_F 5號世界Y座標低位 | 同上格式 | R |

### 7. 統計資訊寄存器 (281-299) 【與程式碼對齊】

| 地址 | 名稱 | 功能 | 數值定義 | 讀寫 |
|------|------|------|----------|------|
| 281 | LAST_CAPTURE_TIME | 最後拍照耗時 | 毫秒單位 | R |
| 282 | LAST_PROCESS_TIME | 最後處理耗時 | 毫秒單位 | R |
| 283 | LAST_TOTAL_TIME | 最後總耗時 | 毫秒單位 | R |
| 284 | OPERATION_COUNT | 操作計數器 | 累積成功次數 | R |
| 285 | ERROR_COUNT | 錯誤計數器 | 累積錯誤次數 | R |
| 286 | CONNECTION_COUNT | 連接計數器 | 累積連接次數 | R |
| 287 | MODEL_SWITCH_COUNT | 模型切換次數 | 累積切換次數 | R |
| 288-289 | - | 保留區域 | 未使用 | R |
| 290 | VERSION_MAJOR | 軟體版本主號 | 5 (YOLOv11版本) | R |
| 291 | VERSION_MINOR | 軟體版本次號 | 0 | R |
| 292 | UPTIME_HOURS | 運行時間小時 | 系統運行時間 | R |
| 293 | UPTIME_MINUTES | 運行時間分鐘 | 系統運行時間 | R |
| 294-299 | - | 保留區域 | 未使用 | R |

## AutoProgram專案中CCD1使用方式

### 在AutoProgram_main.py中的應用

根據AutoProgram_main.py，CCD1模組主要用於自動入料系統：

```python
class AutoFeedingThread:
    def __init__(self):
        self.CCD1_BASE = 200
        # ... 其他初始化

    def trigger_ccd1_detection(self) -> CCD1DetectionResult:
        """觸發CCD1拍照檢測並獲取結果"""
        # 向200地址寫入16觸發拍照+檢測
        if not self.write_register(200, 16):
            return result
        
        # 等待拍照、檢測、操作完成標誌
        while timeout:
            capture_complete = self.read_register(203)
            detect_complete = self.read_register(204)
            operation_success = self.read_register(205)
            
            if all flags complete:
                break
        
        # 讀取檢測結果
        case_f_count = self.read_register(240)      # DR_F_COUNT
        total_detections = self.read_register(243)  # TOTAL_DETECTIONS
        
        # 讀取DR_F世界座標 (261-264為第一個目標)
        world_x = self.read_32bit_register(261, 262)
        world_y = self.read_32bit_register(263, 264)
```

### AutoProgram中的關鍵應用

1. **料件檢測**: 檢測保護區域內是否有DR_F目標
2. **座標獲取**: 獲取第一個DR_F的世界座標用於機械臂抓取
3. **狀態監控**: 通過握手協議確保檢測完成

## 增強版握手協議操作流程

### 1. 基本握手流程 (與程式碼對齊)

```
1. 系統初始化完成 → 狀態寄存器201=9 (Ready=1, Initialized=1)
2. AutoProgram下達控制指令200=16 → 檢查Ready=1
3. 開始執行 → 狀態寄存器201=10 (Ready=0, Running=1, Initialized=1)
4. 執行期間保持Running=1，外部可監控進度
5. 執行完成 → 狀態寄存器201=8 (Running=0, Initialized=1)
6. 設置完成標誌203/204/205=1
7. AutoProgram讀取結果並清零寄存器
8. 系統檢測到清零 → 狀態寄存器201=9 (Ready=1, Initialized=1)
```

### 2. AutoProgram拍照+檢測操作序列

```python
# AutoProgram操作步驟:
def trigger_ccd1_detection():
    # 1. 檢查狀態寄存器201 bit0=1 (Ready狀態)
    ccd1_status = read_register(201)
    if not (ccd1_status & 0x01):
        return False
    
    # 2. 清零控制寄存器
    clear_ccd1_registers()
    
    # 3. 寫入控制指令200=16 (拍照+檢測)
    write_register(200, 16)
    
    # 4. 等待完成標誌 (超時處理)
    timeout = 10.0  # 10秒超時
    start_time = time.time()
    
    while (time.time() - start_time) < timeout:
        capture_complete = read_register(203)
        detect_complete = read_register(204)
        operation_success = read_register(205)
        
        if capture_complete == 1 and detect_complete == 1 and operation_success == 1:
            # 5. 讀取檢測結果
            case_f_count = read_register(240)        # DR_F數量
            total_detections = read_register(243)    # 總檢測數
            
            # 6. 讀取世界座標 (如需要)
            if case_f_count > 0:
                world_x = read_32bit_register(261, 262)  # 第一個DR_F世界X
                world_y = read_32bit_register(263, 264)  # 第一個DR_F世界Y
            
            # 7. 清空寄存器
            clear_ccd1_registers()
            return True
        
        time.sleep(0.1)  # 100ms檢查間隔
    
    return False  # 超時失敗
```

### 3. 與其他模組的整合使用

```python
# AutoProgram中的典型使用場景
class AutoFeedingThread:
    def auto_feeding_cycle(self):
        # 1. 檢查模組狀態
        if not self.check_modules_status():
            return False
        
        # 2. 觸發CCD1檢測
        detection_result = self.trigger_ccd1_detection()
        
        if detection_result.operation_success:
            # 3. 尋找保護區域內的DR_F
            target_coords = self.find_case_f_in_protection_zone(detection_result)
            
            if target_coords:
                # 4. 更新座標到第一個DR_F位置(261-264)
                self.update_first_case_f_coordinates(target_coords)
                self.feeding_ready = True
                
                # 5. 通知RobotJob執行Flow1
                if self.feeding_ready_callback:
                    self.feeding_ready_callback()
            else:
                # 6. 根據檢測結果決定後續動作
                if detection_result.total_detections < 4:
                    self.trigger_flow4_feeding()  # 補料
                else:
                    self.trigger_vp_vibration_and_redetect()  # 震動散開
```

## 世界座標處理 (與AutoProgram對齊)

### 世界座標讀取方式

```python
def read_32bit_register(self, high_addr: int, low_addr: int) -> float:
    """讀取32位世界座標並轉換為實際值"""
    high_val = self.read_register(high_addr)
    low_val = self.read_register(low_addr)
    
    # 合併32位值
    combined = (high_val << 16) + low_val
    
    # 處理補碼(負數)
    if combined >= 2147483648:  # 2^31
        combined = combined - 4294967296  # 2^32
    
    # 轉換為毫米(除以100)
    return combined / 100.0

# 使用範例:
world_x = read_32bit_register(261, 262)  # DR_F_1世界X座標
world_y = read_32bit_register(263, 264)  # DR_F_1世界Y座標
```

### 世界座標寫入方式 (AutoProgram中的實現)

```python
def update_first_case_f_coordinates(self, target_coords):
    """將目標座標覆蓋到第一個DR_F位置(261-264)"""
    world_x, world_y = target_coords
    
    # 轉換為整數形式(×100)
    world_x_int = int(world_x * 100)
    world_y_int = int(world_y * 100)
    
    # 處理負數(補碼)
    if world_x_int < 0:
        world_x_int = world_x_int + 4294967296  # 2^32
    if world_y_int < 0:
        world_y_int = world_y_int + 4294967296  # 2^32
    
    # 分解為高低位
    x_high = (world_x_int >> 16) & 0xFFFF
    x_low = world_x_int & 0xFFFF
    y_high = (world_y_int >> 16) & 0xFFFF
    y_low = world_y_int & 0xFFFF
    
    # 寫入寄存器261-264
    write_register(261, x_high)  # DR_F_1_WORLD_X_HIGH
    write_register(262, x_low)   # DR_F_1_WORLD_X_LOW
    write_register(263, y_high)  # DR_F_1_WORLD_Y_HIGH
    write_register(264, y_low)   # DR_F_1_WORLD_Y_LOW
```

## 錯誤處理與狀態機

### 狀態寄存器201位定義

| 位 | 名稱 | 含義 | 說明 |
|----|------|------|------|
| 0 | Ready | 就緒狀態 | 1=可接受新指令, 0=忙碌中 |
| 1 | Running | 執行狀態 | 1=正在執行指令, 0=空閒 |
| 2 | Alarm | 警報狀態 | 1=系統異常, 0=正常 |
| 3 | Initialized | 初始化狀態 | 1=已初始化, 0=未初始化 |

### 常見狀態值對照表

| 十進制值 | 二進制 | 狀態描述 | AutoProgram處理建議 |
|----------|--------|----------|---------------------|
| 0 | 0000 | 系統未初始化 | 跳過檢測週期 |
| 1 | 0001 | Ready但未初始化 | 異常狀態，需檢查 |
| 8 | 1000 | 已初始化但未Ready | 等待Ready |
| 9 | 1001 | Ready+Initialized | 正常待機，可下指令 |
| 10 | 1010 | Running+Initialized | 正在執行，請等待 |
| 12 | 1100 | Alarm+Initialized | 系統異常，跳過週期 |

### AutoProgram中的錯誤處理

```python
def check_modules_status(self) -> bool:
    """檢查CCD1模組狀態"""
    ccd1_status = self.read_register(201)  # STATUS_REGISTER
    if ccd1_status is None:
        print("[AutoFeeding] ✗ CCD1模組無回應")
        return False
    
    ccd1_ready = bool(ccd1_status & 0x01)      # bit0=Ready
    ccd1_initialized = bool(ccd1_status & 0x08) # bit3=Initialized
    ccd1_alarm = bool(ccd1_status & 0x04)       # bit2=Alarm
    
    if ccd1_alarm:
        print("[AutoFeeding] ✗ CCD1模組處於Alarm狀態")
        return False
    
    # 如果未Ready但已初始化，嘗試重置
    if not ccd1_ready and ccd1_initialized:
        print("[AutoFeeding] CCD1未Ready但已初始化，嘗試重置...")
        self.clear_ccd1_registers()
        time.sleep(0.5)
        # 重新檢查狀態...
    
    return ccd1_ready and ccd1_initialized
```

## 性能與限制

### 性能指標 (與實際測試對齊)
- **檢測速度**: 通常 < 300ms (拍照+YOLOv11檢測)
- **握手響應**: 50ms輪詢頻率  
- **置信度範圍**: 0.1-1.0 (建議0.6-0.95)
- **最大檢測數量**: DR_F=5個座標輸出, STACK=1個座標輸出
- **模型切換**: 支援1-20個模型動態切換
- **座標精度**: 像素級 + 0.01mm世界座標
- **AutoProgram超時**: 10秒檢測超時

### 系統限制
- 單相機模式 (192.168.1.8)
- 需要YOLOv11模型檔案 (model_1.pt ~ model_20.pt)
- 需要標定檔案才能提供世界座標
- Modbus TCP Client模式，需要外部服務器
- AutoProgram只使用第一個DR_F的世界座標(261-264)

## 故障診斷

### AutoProgram常見問題

1. **檢測超時**
   ```python
   # 症狀: trigger_ccd1_detection 返回超時
   # 檢查: 狀態寄存器201是否Ready
   # 解決: 重置CCD1寄存器或重啟模組
   ```

2. **世界座標異常**
   ```python
   # 症狀: read_32bit_register 返回異常值
   # 檢查: 寄存器260 WORLD_COORD_VALID 是否=1
   # 解決: 確認標定檔案已正確載入
   ```

3. **檢測結果為0**
   ```python
   # 症狀: case_f_count 始終為0
   # 檢查: 寄存器240 DR_F_COUNT
   # 解決: 調整置信度或確認目標物存在
   ```

## 版本更新記錄

### v5.2 (更新版) - 2024-12
- **修正**: 寄存器地址映射與AutoProgram_main.py完全對齊
- **修正**: 移除CASE_B分類，僅保留DR_F/STACK
- **新增**: AutoProgram專案使用方式說明
- **新增**: 世界座標讀寫範例代碼
- **修正**: 241和255-256地址標註為預留(程式碼中未使用)
- **優化**: 握手協議說明與實際實現對齊

### v5.1 (修正版) - 2024-12
- **修正**: 寄存器地址映射與程式碼對齊
- **新增**: STACK分類檢測支援  
- **新增**: 多模型動態切換功能
- **新增**: 增強版握手協議
- **修正**: 完成標誌寄存器203-206
- **新增**: 模型管理寄存器202和287
- **優化**: 錯誤處理和狀態機邏輯

### v5.0 (YOLOv11版本)
- 完全移除圓形檢測功能
- 新增YOLOv11 DR_F/STACK分類檢測  
- 重新設計寄存器映射表
- 僅輸出DR_F座標值
- 置信度閾值可通過寄存器控制
- 保留世界座標轉換功能
- 優化握手協議響應速度