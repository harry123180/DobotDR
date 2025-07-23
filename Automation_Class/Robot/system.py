#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
system.py - 自動化系統主控制器
- 物件管理器執行緒：初始化所有子系統
- 參數設定事件：設定CCD1模型、夾爪開度、震動盤參數
- 自動進料事件：CCD1檢測物料並控制震動進料
- 機械臂上料事件：執行Flow1流程(直接實作)
- 機械臂組裝事件：執行Flow2流程(直接實作)
"""

import os
import sys
import time
import threading
import logging
import json
import signal
import psutil
from typing import Dict, Any, Optional, List

# 添加專案路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(os.path.dirname(current_dir), 'Robot'))

# 導入機器人系統
from Robot import Robot, RobotConfig

# 全域變數 - 系統控制
system_running = True             # 系統運行狀態
user_interrupt = False            # 用戶中斷標誌

# 全域變數 - 物件和狀態
robot = None

# 全域變數 - 物料狀態
material_available = True          # 原物料狀態
label_0_objects = []              # 標籤0物件座標列表
label_1_objects = []              # 標籤1物件座標列表
label_2_objects = []              # 標籤2物件座標列表
flow1_execution_count = 0         # Flow1執行次數

# 全域變數 - 角度數據
current_angle = 0.0               # CCD3檢測角度
command_angle = 45.0              # 指令角度(檢測角度+補償值)

# 全域變數 - 系統參數（參考AutoFeeding配置）
ccd1_model_id = 1                 # CCD1模型ID
gripper_opening = 360             # 夾爪開度
vp_spread_strength = 107          # VP擴散強度
vp_spread_frequency = 87          # VP擴散頻率
vp_spread_duration = 0.8          # VP擴散持續時間

# 全域變數 - 點位數據
robot_points = {}                 # 機械臂點位數據

# 全域變數 - 記憶體監控
ccd1_memory_critical = False      # CCD1記憶體臨界狀態
last_memory_check = 0             # 上次記憶體檢查時間

# 全域鎖和事件
system_lock = threading.Lock()
object_manager_event = threading.Event()
parameter_setup_event = threading.Event()
auto_feeding_event = threading.Event()
robot_pickup_event = threading.Event()
robot_assembly_event = threading.Event()

# 常數
PICKUP_HEIGHT = 147.52
ANGLE_OFFSET = 45.0
MEMORY_CRITICAL_THRESHOLD = 500.0  # 500MB觸發CCD1重新初始化

def get_memory_usage():
    """獲取當前記憶體使用量 (MB)"""
    process = psutil.Process()
    return process.memory_info().rss / 1024 / 1024

def check_memory_status():
    """檢查記憶體狀態"""
    global ccd1_memory_critical, last_memory_check
    
    current_time = time.time()
    # 每60秒檢查一次記憶體，避免過度頻繁檢查
    if current_time - last_memory_check < 60:
        return
    
    last_memory_check = current_time
    current_memory = get_memory_usage()
    
    if current_memory > MEMORY_CRITICAL_THRESHOLD:
        if not ccd1_memory_critical:
            logging.warning(f"記憶體使用達到臨界值: {current_memory:.2f}MB > {MEMORY_CRITICAL_THRESHOLD}MB")
            ccd1_memory_critical = True
    else:
        ccd1_memory_critical = False

def reinitialize_ccd1():
    """重新初始化CCD1系統 - 完全重建"""
    global robot, ccd1_memory_critical
    
    try:
        logging.info("開始完全重新初始化CCD1系統...")
        
        # 完全斷開和清理CCD1
        if robot and robot.CCD1:
            logging.info("完全斷開CCD1連接...")
            # 斷開連接並清理所有資源
            robot.CCD1.disconnect()
            robot.CCD1 = None
            time.sleep(2)
        
        # 1次強制垃圾回收
        import gc
        collected = gc.collect()
        logging.info(f"垃圾回收: {collected}個物件")
        
        # 等待記憶體穩定
        time.sleep(2)
        
        # 完全重新初始化CCD1 - 這會重新創建所有組件包括YOLO模型
        if robot:
            logging.info("完全重新初始化CCD1...")
            success = robot.init_ccd1()
            if success:
                # 驗證YOLO模型是否正確載入
                if robot.CCD1:
                    # 重新初始化後重設記憶體基準
                    if hasattr(robot.CCD1, 'reset_memory_baseline'):
                        robot.CCD1.reset_memory_baseline()
                        logging.info("CCD1重新初始化後記憶體基準已重置")
                    
                    # 執行一次測試檢測來確保模型載入正常
                    test_result = robot.get_ccd1_counts()
                    if test_result is not None:
                        logging.info("CCD1重新初始化並測試成功")
                        ccd1_memory_critical = False
                        current_memory = get_memory_usage()
                        logging.info(f"重新初始化後記憶體: {current_memory:.2f}MB")
                        return True
                    else:
                        logging.error("CCD1重新初始化後測試失敗")
                        return False
                else:
                    logging.error("CCD1重新初始化後對象為空")
                    return False
            else:
                logging.error("CCD1重新初始化失敗")
                return False
        
    except Exception as e:
        logging.error(f"CCD1重新初始化異常: {e}")
        return False

def signal_handler(signum, frame):
    """信號處理器 - 處理Ctrl+C中斷"""
    global system_running, user_interrupt, robot
    
    print("\n收到用戶中斷信號 (Ctrl+C)")
    logging.info("收到用戶中斷信號，開始安全停止流程")
    
    system_running = False
    user_interrupt = True
    
    # 立即停止機械臂運動並下使能
    if robot and robot.Dobot:
        try:
            logging.info("正在停止機械臂運動...")
            # 等待當前動作完成
            robot.dobot_sync()
            
            # 下使能機械臂
            robot.dobot_disable()
            logging.info("機械臂已下使能")
            
            # 停止震動盤
            if robot.VP:
                robot.vp_stop_vibration()
                logging.info("震動盤已停止")
                
        except Exception as e:
            logging.error(f"安全停止過程中發生錯誤: {e}")
    
    logging.info("安全停止流程完成")

def setup_logging():
    log_dir = os.path.join(current_dir, 'logs')
    os.makedirs(log_dir, exist_ok=True)
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        handlers=[
            logging.FileHandler(os.path.join(log_dir, 'system.log'), encoding='utf-8'),
            logging.StreamHandler()
        ]
    )

def load_robot_points():
    """載入機械臂點位數據"""
    global robot_points
    
    points_file = os.path.join(current_dir, "saved_points", "robot_points.json")
    
    try:
        with open(points_file, "r", encoding="utf-8") as f:
            points_list = json.load(f)
        
        robot_points.clear()
        for point_data in points_list:
            # 支援pose或cartesian格式
            if "pose" in point_data:
                pose_data = point_data["pose"]
            elif "cartesian" in point_data:
                pose_data = point_data["cartesian"]
            else:
                continue
                
            if "joint" not in point_data:
                continue
                
            joint_data = point_data["joint"]
            
            robot_points[point_data["name"]] = {
                'x': float(pose_data["x"]),
                'y': float(pose_data["y"]), 
                'z': float(pose_data["z"]),
                'r': float(pose_data["r"]),
                'j1': float(joint_data["j1"]),
                'j2': float(joint_data["j2"]),
                'j3': float(joint_data["j3"]),
                'j4': float(joint_data["j4"])
            }
        
        logging.info(f"載入機械臂點位成功，共{len(robot_points)}個點位")
        return True
        
    except Exception as e:
        logging.error(f"載入機械臂點位失敗: {e}")
        return False

def object_manager_thread():
    """物件管理器執行緒 - 初始化所有子系統"""
    global robot
    
    try:
        system_lock.acquire()
        logging.info("物件管理器啟動 - 開始初始化子系統")
        
        # 創建機器人配置
        config = RobotConfig(
            dobot_ip="192.168.1.6",
            ccd1_ip="192.168.1.8", 
            ccd3_ip="192.168.1.10",
            vp_ip="192.168.1.7",
            auto_initialize=True,
            enable_logging=True
        )
        
        # 初始化機器人系統
        robot = Robot(config)
        if not robot.initialize_all_systems():
            logging.error("機器人系統初始化失敗")
            return False
        
        # 機械臂連接和使能
        if robot.Dobot and robot.Dobot.is_connected():
            if not robot.dobot_enable():
                logging.error("機械臂使能失敗")
                return False
            logging.info("機械臂連接並使能成功")
        else:
            logging.error("機械臂連接失敗")
            return False
        
        # 夾爪連接和快速關閉
        if robot.Gripper and robot.Gripper.is_connected():
            robot.gripper_quick_close()
            logging.info("夾爪連接並關閉成功")
        else:
            logging.warning("夾爪連接失敗，系統將繼續運作")
        
        # CCD1連接確認（已自動載入模型1）
        if robot.CCD1:
            logging.info("CCD1連接成功（已自動載入模型1）")
            # CCD1初始化完成後重設記憶體基準 - 關鍵修正
            if hasattr(robot.CCD1, 'reset_memory_baseline'):
                robot.CCD1.reset_memory_baseline()
                logging.info("CCD1載入模型後記憶體基準已重置")
        else:
            logging.error("CCD1連接失敗")
            return False
        
        # CCD3連接和初始化
        if robot.CCD3:
            logging.info("CCD3連接並初始化成功")
        else:
            logging.error("CCD3連接失敗")
            return False
        
        # 載入機械臂點位
        if not load_robot_points():
            logging.error("載入機械臂點位失敗")
            return False
        
        logging.info("物件管理器初始化完成")
        return True
        
    except Exception as e:
        logging.error(f"物件管理器初始化異常: {e}")
        return False
    finally:
        system_lock.release()

def parameter_setup_thread():
    """參數設定事件執行緒"""
    global robot, ccd1_model_id, gripper_opening
    
    try:
        logging.info("參數設定事件啟動")
        
        # 等待物件管理器完成
        if not object_manager_event.wait(timeout=30):
            logging.error("等待物件管理器超時")
            return False
        
        system_lock.acquire()
        
        # 確認CCD1模型狀態（只有在需要不同模型時才切換）
        if robot.CCD1 and ccd1_model_id != 1:
            robot.switch_ccd1_model(ccd1_model_id)
            logging.info(f"CCD1模型切換到: {ccd1_model_id}")
        else:
            logging.info(f"CCD1使用預設模型: {ccd1_model_id}")
        
        # 設定夾爪開度
        if robot.Gripper and robot.Gripper.is_connected():
            robot.gripper_smart_release(gripper_opening)
            logging.info(f"夾爪開度設定為: {gripper_opening}")
        
        # 設定震動盤參數（參考AutoFeeding配置）
        if robot.VP:
            robot.vp_set_backlight(True, 50)
            logging.info(f"震動盤參數設定 - 擴散強度:{vp_spread_strength}, 擴散頻率:{vp_spread_frequency}")
        
        logging.info("參數設定完成")
        return True
        
    except Exception as e:
        logging.error(f"參數設定異常: {e}")
        return False
    finally:
        system_lock.release()

def auto_feeding_thread():
    """自動進料事件執行緒"""
    global robot, material_available, label_0_objects, label_1_objects, label_2_objects, flow1_execution_count
    
    try:
        logging.info("自動進料事件啟動")
        
        # 等待參數設定完成
        if not parameter_setup_event.wait(timeout=30):
            logging.error("等待參數設定超時")
            return False
        
        retry_count = 0
        max_retries = 10
        
        while retry_count < max_retries:
            try:
                system_lock.acquire()
                
                # 檢查記憶體狀態並處理
                check_memory_status()
                if ccd1_memory_critical:
                    logging.warning("檢測到記憶體臨界，重新初始化CCD1")
                    if not reinitialize_ccd1():
                        logging.error("CCD1重新初始化失敗，中止進料")
                        break
                
                # CCD1拍照檢測
                detections = robot.get_ccd1_counts()
                if not detections:
                    logging.error("CCD1檢測失敗")
                    # 如果檢測失敗且處於記憶體臨界狀態，嘗試重新初始化
                    if ccd1_memory_critical:
                        logging.warning("檢測失敗可能由記憶體問題引起，嘗試重新初始化CCD1")
                        if reinitialize_ccd1():
                            # 重新初始化成功，重新嘗試檢測
                            detections = robot.get_ccd1_counts()
                            if not detections:
                                retry_count += 1
                                continue
                        else:
                            retry_count += 1
                            continue
                    else:
                        retry_count += 1
                        continue
                
                # 從檢測結果中提取數量信息
                dr_f_count = detections.get(0, 0)     # DR_F是標籤0
                stack_count = detections.get(1, 0)    # STACK是標籤1
                
                # 獲取座標信息
                coords_result = robot.get_ccd1_coordinates(use_last_result=True)
                
                # 提取世界座標
                if 'world_coordinates' in coords_result:
                    world_coords = coords_result['world_coordinates']
                elif 'world' in coords_result:
                    world_coords = coords_result['world']
                else:
                    world_coords = {}
                
                # DR_F對應標籤0，STACK對應標籤1
                label_0_count = dr_f_count
                label_1_count = stack_count  
                label_2_count = 0  # 目前沒有第三類物件
                
                logging.info(f"檢測結果 - DR_F(標籤0):{label_0_count}, STACK(標籤1):{label_1_count}")
                
                # 判斷標籤0數量
                if label_0_count >= 1:
                    # 更新全域變數 - 使用世界座標
                    label_0_objects = world_coords.get(0, [])
                    label_1_objects = world_coords.get(1, [])
                    label_2_objects = world_coords.get(2, [])
                    flow1_execution_count = 0  # 重置Flow1執行次數
                    
                    logging.info(f"物料準備完成 - DR_F數量:{len(label_0_objects)}, 實際座標:{len(label_0_objects)}個")
                    return True
                
                # 標籤0數量為0，檢查標籤1+標籤2數量
                total_other_count = label_1_count + label_2_count
                
                logging.info(f"DR_F不足，其他物件總數: {total_other_count}")
                
                if total_other_count < 5:
                    # 數量少於5，執行震動進料
                    logging.info("其他物件數量<5，執行震動進料控制")
                    robot.vibration_feed()
                else:
                    # 數量大於5，開始震動盤震動直到標籤0出現
                    logging.info("其他物件數量≥5，開始震動盤震動")
                    # 使用AutoFeeding配置的震動參數
                    robot.vp_start_vibration("vertical", vp_spread_strength, vp_spread_frequency, vp_spread_duration)
                    
                    # 持續檢測直到標籤0出現
                    shake_retry = 0
                    while shake_retry < 10:
                        time.sleep(1)
                        # 重新檢測
                        new_detections = robot.get_ccd1_counts()
                        if new_detections:
                            new_dr_f_count = new_detections.get(0, 0)
                            
                            logging.info(f"震動中檢測 - DR_F:{new_dr_f_count}個")
                            
                            if new_dr_f_count >= 1:
                                robot.vp_stop_vibration()
                                # 重新獲取座標
                                new_coords = robot.get_ccd1_coordinates(use_last_result=True)
                                if 'world_coordinates' in new_coords:
                                    new_world_coords = new_coords['world_coordinates']
                                elif 'world' in new_coords:
                                    new_world_coords = new_coords['world']
                                else:
                                    new_world_coords = {}
                                
                                label_0_objects = new_world_coords.get(0, [])
                                label_1_objects = new_world_coords.get(1, [])
                                label_2_objects = new_world_coords.get(2, [])
                                flow1_execution_count = 0
                                logging.info(f"震動成功 - DR_F數量:{len(label_0_objects)}個")
                                return True
                        
                        shake_retry += 1
                    
                    robot.vp_stop_vibration()
                    logging.warning("震動10次後仍無DR_F物件")
                
                # 震動進料後重新檢測
                time.sleep(0.5)  # 等待震動效果
                
                retry_count += 1
                logging.info(f"震動進料完成，重新檢測 - 第{retry_count}次嘗試")
                
            finally:
                system_lock.release()
            
            time.sleep(0.5)
        
        # 連續10次都沒有標籤0，直接結束程序
        material_available = False
        logging.error("連續10次檢測無標籤0物件，程序結束")
        return False
        
    except Exception as e:
        logging.error(f"自動進料異常: {e}")
        return False

def robot_pickup_thread():
    """機械臂上料事件執行緒 - Flow1流程直接實作"""
    global robot, label_0_objects, flow1_execution_count, current_angle, command_angle, system_running, user_interrupt
    
    try:
        logging.info("機械臂上料事件啟動(Flow1流程)")
        
        # 等待自動進料完成
        if not auto_feeding_event.wait(timeout=60):
            logging.error("等待自動進料超時")
            return False
        
        pickup_cycle_count = 0  # 本輪上料次數計數
        
        while pickup_cycle_count < 3 and system_running and not user_interrupt:
            # 檢查是否有可用的標籤0物件
            if flow1_execution_count >= len(label_0_objects):
                logging.info("當前批次標籤0物件已處理完成，需要重新進料")
                break
            
            system_lock.acquire()
            
            try:
                # 檢查用戶中斷
                if user_interrupt:
                    logging.info("檢測到用戶中斷，停止上料流程")
                    return False
                
                # 獲取當前要處理的物件座標
                target_coord = label_0_objects[flow1_execution_count]
                logging.info(f"處理第{flow1_execution_count + 1}個物件，座標:({target_coord[0]:.2f}, {target_coord[1]:.2f})")
                
                # Flow1流程開始
                # 1. 切換到左手手勢
                robot.gripper_quick_close()
                if hasattr(robot.Dobot, 'dashboard') and robot.Dobot.dashboard:
                    try:
                        robot.Dobot.dashboard.SetArmOrientation(1)
                        logging.info("切換到左手手勢")
                    except Exception as e:
                        logging.warning(f"手勢切換失敗: {e}，繼續執行")
                
                # 檢查中斷
                if user_interrupt: return False
                
                # 2. 移動到standby
                if "standby" in robot_points:
                    point = robot_points["standby"]
                    if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                        robot.Dobot.move.JointMovJ(point['j1'], point['j2'], point['j3'], point['j4'])
                        robot.dobot_sync()
                        logging.info("移動到standby完成")
                
                # 檢查中斷
                if user_interrupt: return False
                
                # 3. 移動到VP_TOPSIDE
                if "VP_TOPSIDE" in robot_points:
                    point = robot_points["VP_TOPSIDE"]
                    if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                        robot.Dobot.move.JointMovJ(point['j1'], point['j2'], point['j3'], point['j4'])
                        robot.dobot_sync()
                        logging.info("移動到VP_TOPSIDE完成")
                
                # 檢查中斷
                if user_interrupt: return False
                
                # 4. 移動到標籤0的XY座標(VP_TOPSIDE同高) - 使用MovL
                vp_point = robot_points["VP_TOPSIDE"]
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.MovL(target_coord[0], target_coord[1], vp_point['z'], vp_point['r'])
                    robot.dobot_sync()
                    logging.info("移動到檢測位置上方完成")
                
                # 檢查中斷
                if user_interrupt: return False
                
                # 5. 移動到標籤0的XY座標(PICKUP_HEIGHT高度) - 使用MovL
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.MovL(target_coord[0], target_coord[1], PICKUP_HEIGHT, vp_point['r'])
                    robot.dobot_sync()
                    logging.info("移動到夾取位置完成")
                
                # 6. 撐開夾爪
                robot.gripper_smart_release(gripper_opening)
                time.sleep(0.3)
                logging.info("夾爪撐開完成")
                
                # 檢查中斷
                if user_interrupt: return False
                
                # 7. 返回VP_TOPSIDE
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.MovL(vp_point['x'], vp_point['y'], vp_point['z'], vp_point['r'])
                    robot.dobot_sync()
                    logging.info("返回VP_TOPSIDE完成")
                
                # 檢查中斷
                if user_interrupt: return False
                
                # 8. 返回standby
                standby_point = robot_points["standby"]
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.JointMovJ(standby_point['j1'], standby_point['j2'], standby_point['j3'], standby_point['j4'])
                    robot.dobot_sync()
                    logging.info("返回standby完成")
                
                # 檢查中斷
                if user_interrupt: return False
                
                # 9. 移動到Rotate_top
                if "Rotate_top" in robot_points:
                    point = robot_points["Rotate_top"]
                    if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                        robot.Dobot.move.JointMovJ(point['j1'], point['j2'], point['j3'], point['j4'])
                        robot.dobot_sync()
                        logging.info("移動到Rotate_top完成")
                
                # 檢查中斷
                if user_interrupt: return False
                
                # 10. 移動到Rotate_down
                if "Rotate_down" in robot_points:
                    point = robot_points["Rotate_down"]
                    if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                        robot.Dobot.move.JointMovJ(point['j1'], point['j2'], point['j3'], point['j4'])
                        robot.dobot_sync()
                        logging.info("移動到Rotate_down完成")
                
                # 11. 夾爪關閉
                robot.gripper_quick_close()
                time.sleep(0.3)
                logging.info("夾爪關閉完成")
                
                # 檢查中斷
                if user_interrupt: return False
                
                # 12. 返回Rotate_top
                rotate_top_point = robot_points["Rotate_top"]
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.JointMovJ(rotate_top_point['j1'], rotate_top_point['j2'], rotate_top_point['j3'], rotate_top_point['j4'])
                    robot.dobot_sync()
                    logging.info("返回Rotate_top完成")
                
                # 檢查中斷
                if user_interrupt: return False
                
                # 13. 返回standby
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.JointMovJ(standby_point['j1'], standby_point['j2'], standby_point['j3'], standby_point['j4'])
                    robot.dobot_sync()
                    logging.info("返回standby完成")
                
                # 14. CCD3獲取角度
                try:
                    angle_result = robot.get_angle()
                    if angle_result is not None:
                        current_angle = angle_result
                        command_angle = current_angle + ANGLE_OFFSET
                        logging.info(f"CCD3角度檢測成功 - 檢測角度:{current_angle:.2f}°, 指令角度:{command_angle:.2f}°")
                    else:
                        current_angle = 0.0
                        command_angle = ANGLE_OFFSET
                        logging.warning("CCD3角度檢測失敗，使用預設值")
                except Exception as e:
                    logging.error(f"CCD3角度檢測異常: {e}")
                    current_angle = 0.0
                    command_angle = ANGLE_OFFSET
                
                # 檢查中斷
                if user_interrupt: return False
                
                # 15. 移動到Rotate_top
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.JointMovJ(rotate_top_point['j1'], rotate_top_point['j2'], rotate_top_point['j3'], rotate_top_point['j4'])
                    robot.dobot_sync()
                
                # 16. 移動到Rotate_down
                rotate_down_point = robot_points["Rotate_down"]
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.JointMovJ(rotate_down_point['j1'], rotate_down_point['j2'], rotate_down_point['j3'], rotate_down_point['j4'])
                    robot.dobot_sync()
                
                # 17. 撐開夾爪
                robot.gripper_smart_release(gripper_opening)
                time.sleep(0.3)
                
                # 檢查中斷
                if user_interrupt: return False
                
                # 18. 返回Rotate_top
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.JointMovJ(rotate_top_point['j1'], rotate_top_point['j2'], rotate_top_point['j3'], rotate_top_point['j4'])
                    robot.dobot_sync()
                
                # 19. 移動到put_asm_top
                if "put_asm_top" in robot_points:
                    point = robot_points["put_asm_top"]
                    if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                        robot.Dobot.move.JointMovJ(point['j1'], point['j2'], point['j3'], point['j4'])
                        robot.dobot_sync()
                        logging.info("移動到put_asm_top完成")
                
                # 更新Flow1執行次數
                flow1_execution_count += 1
                pickup_cycle_count += 1
                logging.info(f"Flow1流程完成，執行次數: {flow1_execution_count}，本輪次數: {pickup_cycle_count}")
                
            except Exception as e:
                logging.error(f"Flow1流程執行異常: {e}")
                return False
                
            finally:
                system_lock.release()
        
        # 連續執行3次後，或物件用完時，記錄狀態
        if pickup_cycle_count >= 3:
            logging.info("完成3次連續上料，需要重新進料")
        elif flow1_execution_count >= len(label_0_objects):
            logging.info("當前批次物件已用完，需要重新進料")
        
        return True
        
    except Exception as e:
        logging.error(f"機械臂上料事件異常: {e}")
        return False

def robot_assembly_thread():
    """機械臂組裝事件執行緒 - Flow2流程直接實作"""
    global robot, command_angle, system_running, user_interrupt
    
    try:
        logging.info("機械臂組裝事件啟動(Flow2流程)")
        
        # 等待機械臂上料完成
        if not robot_pickup_event.wait(timeout=60):
            logging.error("等待機械臂上料超時")
            return False
        
        # 檢查用戶中斷
        if user_interrupt:
            logging.info("檢測到用戶中斷，停止組裝流程")
            return False
        
        # 等待5秒
        time.sleep(5)
        
        system_lock.acquire()
        
        try:
            # Flow2流程開始
            logging.info(f"開始組裝流程，使用角度: {command_angle:.2f}°")
            
            # 檢查中斷
            if user_interrupt: return False
            
            # 1. 移動到put_asm_top (帶J4角度) - 使用JointMovJ
            if "put_asm_top" in robot_points:
                point = robot_points["put_asm_top"]
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.JointMovJ(point['j1'], point['j2'], point['j3'], command_angle)
                    robot.dobot_sync()
                    logging.info(f"移動到put_asm_top完成 (J4角度:{command_angle:.2f}°)")
            
            # 檢查中斷
            if user_interrupt: return False
            
            # 2. 移動到put_asm_down (帶J4角度) - 使用JointMovJ
            if "put_asm_down" in robot_points:
                point = robot_points["put_asm_down"]
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.JointMovJ(point['j1'], point['j2'], point['j3'], command_angle)
                    robot.dobot_sync()
                    logging.info(f"移動到put_asm_down完成 (J4角度:{command_angle:.2f}°)")
            
            # 3. 夾爪關閉
            robot.gripper_quick_close()
            time.sleep(0.3)
            logging.info("夾爪關閉完成")
            
            # 檢查中斷
            if user_interrupt: return False
            
            # 4. 返回put_asm_top - 使用JointMovJ
            put_asm_top_point = robot_points["put_asm_top"]
            if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                robot.Dobot.move.JointMovJ(put_asm_top_point['j1'], put_asm_top_point['j2'], put_asm_top_point['j3'], command_angle)
                robot.dobot_sync()
                logging.info("返回put_asm_top完成")
            
            # 檢查中斷
            if user_interrupt: return False
            
            # 5. 返回standby
            if "standby" in robot_points:
                point = robot_points["standby"]
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.JointMovJ(point['j1'], point['j2'], point['j3'], point['j4'])
                    robot.dobot_sync()
                    logging.info("返回standby完成")
            
            logging.info("Flow2組裝流程完成")
            return True
            
        except Exception as e:
            logging.error(f"Flow2流程執行異常: {e}")
            return False
            
    finally:
        system_lock.release()

def main():
    """主程式入口"""
    global system_running, user_interrupt
    
    setup_logging()
    
    # 設置信號處理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    logging.info("自動化系統啟動")
    logging.info("按 Ctrl+C 可安全停止系統")
    
    try:
        # 1. 啟動物件管理器執行緒
        object_manager_thread_handle = threading.Thread(target=object_manager_thread, daemon=True)
        object_manager_thread_handle.start()
        object_manager_thread_handle.join()
        object_manager_event.set()
        
        if user_interrupt:
            return
        
        # 2. 啟動參數設定事件
        parameter_setup_thread_handle = threading.Thread(target=parameter_setup_thread, daemon=True)
        parameter_setup_thread_handle.start()
        parameter_setup_thread_handle.join()
        parameter_setup_event.set()
        
        if user_interrupt:
            return
        
        # 3. 啟動自動進料事件
        auto_feeding_thread_handle = threading.Thread(target=auto_feeding_thread, daemon=True)
        auto_feeding_thread_handle.start()
        auto_feeding_thread_handle.join()
        auto_feeding_event.set()
        
        # 4. 主循環 - 重複執行上料和組裝
        while material_available and system_running and not user_interrupt:
            # 執行3次上料流程或直到物件用完
            for pickup_round in range(3):
                if not system_running or user_interrupt:
                    break
                
                # 檢查是否需要重新進料
                if flow1_execution_count >= len(label_0_objects):
                    logging.info("物件用完，重新觸發自動進料事件")
                    auto_feeding_event.clear()
                    auto_feeding_thread_handle = threading.Thread(target=auto_feeding_thread, daemon=True)
                    auto_feeding_thread_handle.start()
                    auto_feeding_thread_handle.join()
                    auto_feeding_event.set()
                    
                    if not material_available or user_interrupt:
                        break
                
                # 機械臂上料事件 (單次)
                robot_pickup_thread_handle = threading.Thread(target=robot_pickup_thread, daemon=True)
                robot_pickup_thread_handle.start()
                robot_pickup_thread_handle.join()
                robot_pickup_event.set()
                
                if user_interrupt:
                    break
                
                # 機械臂組裝事件 (單次)
                robot_assembly_thread_handle = threading.Thread(target=robot_assembly_thread, daemon=True)
                robot_assembly_thread_handle.start()
                robot_assembly_thread_handle.join()
                robot_assembly_event.set()
                
                if user_interrupt:
                    break
                
                # 重置事件以便下次循環
                robot_pickup_event.clear()
                robot_assembly_event.clear()
                
                logging.info(f"完成第{pickup_round + 1}輪上料組裝循環")
            
            if user_interrupt:
                break
            
            # 連續3次完成後，重新觸發自動進料
            if system_running and not user_interrupt:
                logging.info("完成3輪循環，重新觸發自動進料事件")
                auto_feeding_event.clear()
                auto_feeding_thread_handle = threading.Thread(target=auto_feeding_thread, daemon=True)
                auto_feeding_thread_handle.start()
                auto_feeding_thread_handle.join()
                auto_feeding_event.set()
        
        if not user_interrupt:
            logging.info("系統停止 - 原物料不足")
        
    except KeyboardInterrupt:
        logging.info("收到中斷信號，正在關閉系統")
        user_interrupt = True
    except Exception as e:
        logging.error(f"系統運行異常: {e}")
    finally:
        # 清理資源
        system_running = False
        if robot:
            try:
                robot.emergency_stop()
                robot.disconnect_all()
            except:
                pass
        logging.info("系統已安全關閉")

if __name__ == "__main__":
    main()