# -*- coding: utf-8 -*-
"""
CCD1記憶體測試程式 - 10000次測試 (配合記憶體優化版本)
排除首次模型加載記憶體增長，只監控實際運行記憶體變化
新增詳細記憶體監控和智能清理機制
"""
import sys
import psutil
import gc
import time
import logging

# 關閉所有不必要的日誌輸出
logging.getLogger('ultralytics').setLevel(logging.CRITICAL)
logging.getLogger('PIL').setLevel(logging.CRITICAL)
logging.getLogger().setLevel(logging.CRITICAL)

sys.path.insert(0, r"C:\Users\user\Documents\GitHub\DobotDR\Automation_Class\CCD1")
from CCD1_Headless_Vision_System import CCD1VisionSystem, CCD1VisionConfig

def get_memory_usage():
    """獲取當前記憶體使用量 (MB)"""
    process = psutil.Process()
    return process.memory_info().rss / 1024 / 1024
def get_detection_summary(result):
    """獲取檢測結果摘要"""
    if not result.success:
        return "失敗", 0, 0, 0
    
    # 獲取各標籤數量
    label_0_count = result.detections_by_class.get(0, 0)  # DR_F
    label_1_count = result.detections_by_class.get(1, 0)  # STACK
    total_count = result.total_detections
    
    return "成功", label_0_count, label_1_count, total_count
def print_memory_checkpoints(vision_system):
    """輸出記憶體檢查點資訊"""
    try:
        memory_stats = vision_system.get_memory_stats()
        if 'system' in memory_stats and 'checkpoints' in memory_stats['system']:
            checkpoints = memory_stats['system']['checkpoints']
            if checkpoints:
                print("記憶體檢查點:")
                for name, info in list(checkpoints.items())[-3:]:  # 只顯示最新3個
                    print(f"  {name}: {info['memory']:.2f}MB (+{info['diff_from_baseline']:+.2f}MB)")
    except Exception as e:
        print(f"獲取記憶體檢查點失敗: {e}")

def main():
    """主測試函數"""
    
    # 配置 - 優化記憶體管理設置
    config = CCD1VisionConfig(
        camera_ip="192.168.1.8",
        save_raw_image=False,
        save_result_image=False,
        enable_world_coord=False,
        
        # 記憶體優化配置
        enable_memory_monitoring=True,      # 啟用記憶體監控
        memory_warning_threshold=30.0,      # 30MB警告閾值
        force_gc_frequency=5,               # 每5次強制垃圾回收
        max_cached_images=2,                # 最大快取2張圖像
        memory_cleanup_frequency=10         # 每10次清理
    )
    
    print("初始化系統並加載模型...")
    
    # 初始化視覺系統
    vision_system = CCD1VisionSystem(config)
    
    # 執行第一次檢測以完成模型加載
    print("執行首次檢測以完成模型加載...")
    first_result = vision_system.detect_objects()
    first_status, first_label0, first_label1, first_total = get_detection_summary(first_result)
    print(f"首次檢測: {first_status} | DR_F: {first_label0}, STACK: {first_label1}, 總計: {first_total}")
    
    # 重置記憶體基準 (使用新的API)
    vision_system.reset_memory_baseline()
    
    # 強制垃圾回收並等待記憶體穩定
    gc.collect()
    time.sleep(2)
    
    # 將首次檢測後的記憶體作為基準
    baseline_memory = get_memory_usage()
    max_memory = baseline_memory
    min_memory = baseline_memory
    
    print(f"模型加載完成，基準記憶體: {baseline_memory:.2f} MB")
    print("開始10000次記憶體測試...\n")
    
    # 測試變數
    success_count = 0
    warning_count = 0  # 記憶體警告次數
    cleanup_count = 0  # 清理次數
    
    try:
        for i in range(10000):
            # 執行檢測
            result = vision_system.detect_objects()
            
            if result.success:
                success_count += 1
            
            # 每100次更新記憶體統計
            if (i + 1) % 100 == 0:
                current_memory = get_memory_usage()
                max_memory = max(max_memory, current_memory)
                min_memory = min(min_memory, current_memory)
                
                change = current_memory - baseline_memory
                growth_rate = (change / baseline_memory) * 100
                
                # 獲取詳細記憶體統計
                try:
                    memory_stats = vision_system.get_memory_stats()
                    system_change = memory_stats['system']['total_change']
                    system_growth = memory_stats['system']['growth_rate']
                    
                    print(f"第{i+1:5d}次 | "
                          f"記憶體: {current_memory:7.2f}MB | "
                          f"變化: {change:+6.2f}MB ({growth_rate:+5.1f}%) | "
                          f"系統追蹤: {system_change:+5.2f}MB ({system_growth:+4.1f}%)")
                    
                    # 檢查是否觸發了記憶體警告
                    if system_change > config.memory_warning_threshold:
                        warning_count += 1
                        print(f"    ⚠️  記憶體警告 #{warning_count}: {system_change:.2f}MB > {config.memory_warning_threshold}MB")
                    
                except Exception as e:
                    print(f"第{i+1:5d}次 | "
                          f"記憶體: {current_memory:7.2f}MB | "
                          f"變化: {change:+6.2f}MB ({growth_rate:+5.1f}%) | "
                          f"統計獲取失敗: {e}")
                
                # 每500次輸出詳細檢查點
                if (i + 1) % 500 == 0:
                    print_memory_checkpoints(vision_system)
                
                # 強制垃圾回收
                collected = gc.collect()
                if collected > 5:  # 只在回收較多物件時顯示
                    print(f"    🧹 垃圾回收: {collected}個物件")
                
                # 檢查記憶體增長異常
                if growth_rate > 50:
                    print(f"    ❌ 記憶體增長異常: {growth_rate:.1f}%")
                    
                    # 使用新的強制清理功能
                    print("    🔧 執行強制清理...")
                    vision_system.force_cleanup()
                    cleanup_count += 1
                    
                    # 更新基準
                    baseline_memory = get_memory_usage()
                    print(f"    🔄 重置基準記憶體: {baseline_memory:.2f}MB")
        
    except KeyboardInterrupt:
        print(f"\n測試被中斷 (已完成 {i+1} 次檢測)")
    except Exception as e:
        print(f"\n測試異常: {e}")
        import traceback
        print(f"詳細錯誤: {traceback.format_exc()}")
    
    # 最終記憶體統計
    final_memory = get_memory_usage()
    final_change = final_memory - baseline_memory
    final_growth_rate = (final_change / baseline_memory) * 100
    
    print(f"\n{'='*60}")
    print(f"測試完成 - {i+1} 次檢測")
    print(f"成功率: {success_count/max(i+1,1)*100:.1f}%")
    print(f"記憶體警告次數: {warning_count}")
    print(f"強制清理次數: {cleanup_count}")
    
    print(f"\n記憶體統計:")
    print(f"  基準記憶體: {baseline_memory:.2f} MB")
    print(f"  最終記憶體: {final_memory:.2f} MB")
    print(f"  記憶體變化: {final_change:+.2f} MB ({final_growth_rate:+.1f}%)")
    print(f"  最大記憶體: {max_memory:.2f} MB")
    print(f"  最小記憶體: {min_memory:.2f} MB")
    
    # 獲取系統詳細記憶體報告
    try:
        final_memory_stats = vision_system.get_memory_stats()
        print(f"\n詳細記憶體報告:")
        if 'system' in final_memory_stats:
            system_stats = final_memory_stats['system']
            print(f"  系統追蹤記憶體: {system_stats['current']:.2f} MB")
            print(f"  系統記憶體變化: {system_stats['total_change']:+.2f} MB ({system_stats['growth_rate']:+.1f}%)")
        
        if 'resource_tracker' in final_memory_stats:
            resource_stats = final_memory_stats['resource_tracker']
            print(f"  活躍物件數: {resource_stats['active_objects']}")
            print(f"  總分配物件數: {resource_stats['total_allocated']}")
        
        if 'yolo_detector' in final_memory_stats:
            yolo_stats = final_memory_stats['yolo_detector']
            print(f"  YOLO檢測器記憶體: {yolo_stats['detector']['current']:.2f} MB")
            print(f"  檢測次數: {yolo_stats['detection_count']}")
            print(f"  圖像快取大小: {yolo_stats['cache_size']}")
        
    except Exception as e:
        print(f"獲取詳細記憶體報告失敗: {e}")
    
    # 記憶體洩漏判斷 (調整閾值)
    if final_growth_rate > 15:
        print(f"\n❌ 可能存在記憶體洩漏 (增長 {final_growth_rate:.1f}%)")
    elif final_growth_rate > 8:
        print(f"\n⚠️  記憶體使用略有增長 (增長 {final_growth_rate:.1f}%)")
    else:
        print(f"\n✅ 記憶體使用穩定 (變化 {final_growth_rate:.1f}%)")
    
    # 效能評估
    if warning_count == 0:
        print("🎯 記憶體管理表現優秀，無警告觸發")
    elif warning_count < i // 1000:  # 少於千分之一的警告率
        print(f"🟡 記憶體管理表現良好，警告率: {warning_count/(i+1)*100:.2f}%")
    else:
        print(f"🔴 記憶體管理需要優化，警告率: {warning_count/(i+1)*100:.2f}%")
    
    # 清理
    print(f"\n開始清理系統...")
    vision_system.disconnect()
    
    # 最終清理
    collected = gc.collect()
    cleanup_memory = get_memory_usage()
    
    print(f"系統清理完成:")
    print(f"  清理前記憶體: {final_memory:.2f} MB")
    print(f"  清理後記憶體: {cleanup_memory:.2f} MB")
    print(f"  清理釋放: {final_memory - cleanup_memory:.2f} MB")
    print(f"  垃圾回收: {collected} 個物件")

def quick_test():
    """快速測試 - 100次檢測"""
    print("快速記憶體測試 (100次)...")
    
    config = CCD1VisionConfig(
        camera_ip="192.168.1.8",
        save_raw_image=False,
        save_result_image=False,
        enable_world_coord=False,
        enable_memory_monitoring=True,
        memory_warning_threshold=20.0,
        force_gc_frequency=3,
        max_cached_images=1
    )
    
    vision_system = CCD1VisionSystem(config)
    
    # 首次檢測
    first_result = vision_system.detect_objects()
    vision_system.reset_memory_baseline()
    
    baseline_memory = get_memory_usage()
    print(f"基準記憶體: {baseline_memory:.2f} MB")
    
    success_count = 0
    
    for i in range(100):
        result = vision_system.detect_objects()
        if result.success:
            success_count += 1
        
        if (i + 1) % 20 == 0:
            current_memory = get_memory_usage()
            change = current_memory - baseline_memory
            growth_rate = (change / baseline_memory) * 100
            
            memory_stats = vision_system.get_memory_stats()
            system_change = memory_stats['system']['total_change']
            
            print(f"第{i+1:3d}次 | 記憶體: {current_memory:.2f}MB | "
                  f"變化: {change:+.2f}MB | 系統: {system_change:+.2f}MB")
    
    final_memory = get_memory_usage()
    final_change = final_memory - baseline_memory
    
    print(f"\n快速測試完成:")
    print(f"成功率: {success_count/100*100:.1f}%")
    print(f"記憶體變化: {final_change:+.2f} MB")
    
    vision_system.disconnect()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "quick":
        quick_test()
    else:
        main()