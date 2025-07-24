# -*- coding: utf-8 -*-
"""
CCD1_Simple_Example.py - CCD1視覺系統簡單範例
新人友善版本，展示基本功能使用
"""

import time
from CCD1_Simple_Vison_System import CCD1VisionSystem, CCD1VisionConfig


def main():
    """
    CCD1視覺系統基本使用範例
    展示：初始化、檢測、模型切換、座標獲取
    """
    
    print("=== CCD1視覺系統基本使用範例 ===\n")
    
    # ==================== 步驟1: 系統配置 ====================
    print("步驟1: 設定系統配置")
    
    # 取得當前程式所在目錄
    import os
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    config = CCD1VisionConfig(
        # 相機IP設定
        camera_ip="192.168.1.8",
        
        # 檢測類別設定 (根據你的YOLO模型)
        num_classes=2,
        class_names={
            0: "DR_F",    # 標籤0
            1: "STACK"    # 標籤1
        },
        
        # 置信度閾值 (0.0-1.0，越高越嚴格)
        confidence_threshold=0.8,
        
        # 是否啟用世界座標轉換
        enable_world_coord=True,
        
        # 標定檔案路徑 (使用絕對路徑)
        calibration_dir=current_dir,
        intrinsic_file="camera_matrix_DR.npy",
        extrinsic_file="extrinsic_DR3.npy",
        dist_coeffs_file="dist_coeffs_DR.npy",
        
        # 圖像保存設定
        save_raw_image=True,
        save_result_image=True,
        save_dir=os.path.join(current_dir, "detection_results")
    )
    
    print("✓ 配置完成")
    print(f"  相機IP: {config.camera_ip}")
    print(f"  檢測類別: {config.class_names}")
    print(f"  置信度閾值: {config.confidence_threshold}")
    print(f"  世界座標: {'啟用' if config.enable_world_coord else '關閉'}")
    
    
    # ==================== 步驟2: 初始化系統 ====================
    print("\n步驟2: 初始化視覺系統")
    
    try:
        vision_system = CCD1VisionSystem(config)
        print("✓ 系統初始化成功")
    except Exception as e:
        print(f"✗ 系統初始化失敗: {e}")
        return
    
    # 檢查系統狀態
    status = vision_system.get_status()
    print(f"  相機連接: {'成功' if status['camera_connected'] else '失敗'}")
    print(f"  YOLO模型: {'已載入' if status['yolo_model_loaded'] else '未載入'}")
    print(f"  當前模型ID: {status['current_model_id']}")
    print(f"  可用模型: {status['available_models']}")
    print(f"  標定載入: {'成功' if status['calibration_loaded'] else '失敗'}")
    
    
    # ==================== 步驟3: 執行檢測 ====================
    print("\n步驟3: 執行物件檢測")
    
    # 執行5次檢測作為示範
    for i in range(5):
        print(f"\n--- 第{i+1}次檢測 ---")
        
        # 執行檢測 (自動拍照並分析)
        result = vision_system.detect_objects()
        
        if result.success:
            print("✓ 檢測成功")
            
            # 獲取標籤0的數量 (DR_F)
            label0_count = result.detections_by_class.get(0, 0)
            print(f"  標籤0 (DR_F) 數量: {label0_count}")
            
            # 獲取標籤1的數量 (STACK)
            label1_count = result.detections_by_class.get(1, 0)
            print(f"  標籤1 (STACK) 數量: {label1_count}")
            
            # 獲得全部物件總數
            total_objects = result.total_detections
            print(f"  全部物件總數: {total_objects}")
            
            # 顯示處理時間
            print(f"  處理時間: {result.processing_time:.1f}ms")
            
            # 如果有世界座標，顯示座標資訊
            if result.world_coord_valid:
                print("  世界座標:")
                
                # 顯示標籤0的世界座標
                if 0 in result.world_coordinates_by_class:
                    world_coords_0 = result.world_coordinates_by_class[0]
                    for j, (x, y) in enumerate(world_coords_0):
                        print(f"    DR_F #{j+1}: ({x:.2f}, {y:.2f}) mm")
                
                # 顯示標籤1的世界座標
                if 1 in result.world_coordinates_by_class:
                    world_coords_1 = result.world_coordinates_by_class[1]
                    for j, (x, y) in enumerate(world_coords_1):
                        print(f"    STACK #{j+1}: ({x:.2f}, {y:.2f}) mm")
            else:
                print("  世界座標: 無法計算")
                
        else:
            print(f"✗ 檢測失敗: {result.error_message}")
        
        # 等待1秒再進行下次檢測
        time.sleep(1)
    
    
    # ==================== 步驟4: 模型切換示範 ====================
    print("\n步驟4: YOLO模型切換示範")
    
    # 獲取可用的模型列表
    available_models = status['available_models']
    print(f"可用模型: {available_models}")
    
    if len(available_models) > 1:
        # 嘗試切換到模型2
        target_model = 2 if 2 in available_models else available_models[1]
        
        print(f"嘗試切換到模型{target_model}...")
        
        if vision_system.switch_yolo_model(target_model):
            print(f"✓ 成功切換到模型{target_model}")
            
            # 用新模型執行一次檢測
            print("使用新模型執行檢測...")
            result = vision_system.detect_objects()
            
            if result.success:
                print(f"✓ 新模型檢測成功")
                print(f"  使用模型ID: {result.model_id_used}")
                print(f"  標籤0數量: {result.detections_by_class.get(0, 0)}")
                print(f"  標籤1數量: {result.detections_by_class.get(1, 0)}")
                print(f"  總物件數: {result.total_detections}")
            else:
                print(f"✗ 新模型檢測失敗: {result.error_message}")
        else:
            print(f"✗ 模型切換失敗")
    else:
        print("只有一個模型可用，跳過切換示範")
    
    
    # ==================== 步驟5: 置信度調整示範 ====================
    print("\n步驟5: 置信度調整示範")
    
    original_threshold = config.confidence_threshold
    print(f"原始置信度: {original_threshold}")
    
    # 調整置信度到0.5 (更寬鬆，會檢測到更多物件)
    new_threshold = 0.5
    vision_system.update_confidence_threshold(new_threshold)
    print(f"調整置信度到: {new_threshold}")
    
    # 用新置信度執行檢測
    result = vision_system.detect_objects()
    if result.success:
        print(f"✓ 新置信度檢測結果:")
        print(f"  標籤0數量: {result.detections_by_class.get(0, 0)}")
        print(f"  標籤1數量: {result.detections_by_class.get(1, 0)}")
        print(f"  總物件數: {result.total_detections}")
    
    # 恢復原始置信度
    vision_system.update_confidence_threshold(original_threshold)
    print(f"恢復置信度到: {original_threshold}")
    
    
    # ==================== 步驟6: 清理與關閉 ====================
    print("\n步驟6: 清理系統資源")
    
    try:
        vision_system.disconnect()
        print("✓ 系統已安全關閉")
    except Exception as e:
        print(f"✗ 關閉時發生錯誤: {e}")


def simple_detection_example():
    """
    最簡單的檢測範例 - 只要3行代碼
    """
    print("\n=== 超簡單範例 (只要3行代碼) ===")
    
    # 取得當前目錄
    import os
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 第1行: 創建配置 (使用絕對路徑)
    config = CCD1VisionConfig(
        camera_ip="192.168.1.8",
        calibration_dir=current_dir
    )
    
    # 第2行: 初始化系統
    vision = CCD1VisionSystem(config)
    
    # 第3行: 執行檢測
    result = vision.detect_objects()
    
    # 顯示結果
    if result.success:
        print(f"檢測成功! 找到 {result.total_detections} 個物件")
        print(f"DR_F: {result.detections_by_class.get(0, 0)} 個")
        print(f"STACK: {result.detections_by_class.get(1, 0)} 個")
        
        # 如果有世界座標，也顯示幾個範例
        if result.world_coord_valid and 1 in result.world_coordinates_by_class:
            world_coords = result.world_coordinates_by_class[1][:3]  # 只顯示前3個
            print("前3個STACK的世界座標:")
            for i, (x, y) in enumerate(world_coords):
                print(f"  STACK #{i+1}: ({x:.2f}, {y:.2f}) mm")
    else:
        print(f"檢測失敗: {result.error_message}")
    
    # 清理
    vision.disconnect()


def get_world_coordinates_example():
    """
    獲取世界座標的專用範例
    """
    print("\n=== 世界座標獲取範例 ===")
    
    # 取得當前目錄
    import os
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 配置系統 (確保啟用世界座標，使用絕對路徑)
    config = CCD1VisionConfig(
        camera_ip="192.168.1.8",
        enable_world_coord=True,
        calibration_dir=current_dir,
        intrinsic_file="camera_matrix_DR.npy",
        extrinsic_file="extrinsic_DR3.npy",
        dist_coeffs_file="dist_coeffs_DR.npy"
    )
    
    vision = CCD1VisionSystem(config)
    result = vision.detect_objects()
    
    if result.success and result.world_coord_valid:
        print("✓ 成功獲取世界座標:")
        
        # 專門處理標籤0的世界座標
        if 0 in result.world_coordinates_by_class:
            coords_0 = result.world_coordinates_by_class[0]
            print(f"標籤0 (DR_F) 世界座標:")
            for i, (x, y) in enumerate(coords_0):
                print(f"  物件{i+1}: X={x:.2f}mm, Y={y:.2f}mm")
        
        # 專門處理標籤1的世界座標
        if 1 in result.world_coordinates_by_class:
            coords_1 = result.world_coordinates_by_class[1]
            print(f"標籤1 (STACK) 世界座標 (顯示前10個):")
            for i, (x, y) in enumerate(coords_1[:10]):  # 只顯示前10個
                print(f"  物件{i+1}: X={x:.2f}mm, Y={y:.2f}mm")
            
            if len(coords_1) > 10:
                print(f"  ... 還有 {len(coords_1) - 10} 個物件")
                
    else:
        print("無法獲取世界座標")
        if not result.success:
            print(f"檢測失敗: {result.error_message}")
        else:
            print("世界座標轉換未啟用或標定檔案載入失敗")
            print("請檢查標定檔案是否存在:")
            print(f"  - {config.calibration_dir}/{config.intrinsic_file}")
            print(f"  - {config.calibration_dir}/{config.extrinsic_file}")
            print(f"  - {config.calibration_dir}/{config.dist_coeffs_file}")
    
    vision.disconnect()


if __name__ == "__main__":
    # 執行完整範例
    #main()
    
    # 執行簡單範例
    simple_detection_example()
    
    # 執行座標範例
    #get_world_coordinates_example()
    
    #print("\n=== 所有範例執行完成 ===")