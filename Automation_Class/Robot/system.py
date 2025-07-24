# -*- coding: utf-8 -*-
"""
system.py - 自動化系統主控制器 (修正版本)
修正日誌錯誤和相機連接問題
"""

import os
import sys
import time
import threading
import logging
import json
from typing import Dict, Any, Optional, List

# 添加專案路徑
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(os.path.dirname(current_dir), 'Robot'))

# 導入機器人系統
from Robot import Robot, RobotConfig

# 全域變數 - 物件和狀態
robot = None

# 全域變數 - 物料狀態
material_available = True
label_0_objects = []
label_1_objects = []
label_2_objects = []
flow1_execution_count = 0

# 全域變數 - 角度數據
current_angle = 0.0
command_angle = 45.0

# 全域變數 - 系統參數
ccd1_model_id = 1
gripper_opening = 360
vp_spread_action_code = 5
vp_spread_strength = 107
vp_spread_frequency = 87
vp_spread_duration = 0.8
vp_stop_command_code = 3

# 全域變數 - 點位數據
robot_points = {}

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

# 修正日誌設置 - 避免多執行緒檔案關閉問題
def setup_logging():
    """設置日誌系統 - 修正多執行緒問題"""
    log_dir = os.path.join(current_dir, 'logs')
    os.makedirs(log_dir, exist_ok=True)
    
    # 創建根日誌器
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    
    # 清除現有的處理器
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)
    
    # 創建格式器
    formatter = logging.Formatter('%(asctime)s [%(levelname)s] %(message)s')
    
    # 檔案處理器 - 使用追加模式並且每次都重新開啟
    file_handler = logging.FileHandler(
        os.path.join(log_dir, 'system.log'), 
        mode='a', 
        encoding='utf-8',
        delay=False  # 立即開啟檔案
    )
    file_handler.setFormatter(formatter)
    file_handler.setLevel(logging.INFO)
    
    # 控制台處理器
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)
    console_handler.setLevel(logging.INFO)
    
    # 添加處理器
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    
    return logger

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
    """物件管理器執行緒 - 修正CCD3狀態檢查"""
    global robot
    
    try:
        with system_lock:
            logging.info("物件管理器啟動 - 開始初始化子系統")
            
            # 創建機器人配置
            config = RobotConfig(
                dobot_ip="192.168.1.6",
                ccd1_ip="192.168.1.8", 
                ccd3_ip="192.168.1.10",
                vp_ip="192.168.1.7",
                auto_initialize=False,
                enable_logging=True,
                silent_mode=False
            )
            
            # 初始化機器人系統
            robot = Robot(config)
            
            # 手動初始化各子系統，允許部分失敗
            initialized_count = 0
            
            # 1. 初始化CCD1
            if robot.init_ccd1():
                logging.info("CCD1初始化成功")
                initialized_count += 1
            else:
                logging.warning("CCD1初始化失敗，但系統繼續運作")
            
            # 2. 初始化CCD3
            if robot.init_ccd3():
                logging.info("CCD3初始化成功")
                initialized_count += 1
            else:
                logging.warning("CCD3初始化失敗，但系統繼續運作")
            
            # 3. 初始化震動盤
            if robot.init_vp():
                logging.info("震動盤初始化成功")
                initialized_count += 1
            else:
                logging.warning("震動盤初始化失敗，但系統繼續運作")
            
            # 4. 初始化夾爪
            if robot.init_gripper():
                logging.info("夾爪初始化成功")
                initialized_count += 1
            else:
                logging.warning("夾爪初始化失敗，但系統繼續運作")
            
            # 5. 初始化機械臂
            if robot.init_dobot():
                logging.info("機械臂初始化成功")
                initialized_count += 1
            else:
                logging.error("機械臂初始化失敗")
                return False
            
            logging.info(f"子系統初始化完成，成功: {initialized_count}/5")
            
            # 機械臂連接和使能
            if robot.Dobot and hasattr(robot.Dobot, 'is_connected') and robot.Dobot.is_connected():
                if robot.dobot_enable():
                    logging.info("機械臂連接並使能成功")
                else:
                    logging.error("機械臂使能失敗")
                    return False
            else:
                logging.error("機械臂連接失敗")
                return False
            
            # 夾爪連接和快速關閉（如果可用）
            if robot.Gripper and hasattr(robot.Gripper, 'is_connected') and robot.Gripper.is_connected():
                robot.gripper_quick_close()
                logging.info("夾爪連接並關閉成功")
            else:
                logging.warning("夾爪不可用，系統將繼續運作")
            
            # CCD1連接確認（如果可用）
            if robot.subsystem_status.get('CCD1', False):
                logging.info("CCD1系統可用")
            else:
                logging.warning("CCD1系統不可用，檢測功能將受限")
            
            # CCD3連接確認（修正：使用subsystem_status檢查）
            if robot.subsystem_status.get('CCD3', False):
                logging.info("CCD3系統可用")
            else:
                logging.warning("CCD3系統不可用，角度檢測功能將受限")
            
            # 載入機械臂點位
            if not load_robot_points():
                logging.error("載入機械臂點位失敗")
                return False
            
            logging.info("物件管理器初始化完成")
            return True
            
    except Exception as e:
        logging.error(f"物件管理器初始化異常: {e}")
        return False

def parameter_setup_thread():
    """參數設定事件執行緒 - 修正震動盤檢查"""
    global robot, ccd1_model_id, gripper_opening, vp_spread_strength, vp_spread_frequency
    
    try:
        logging.info("參數設定事件啟動")
        
        # 等待物件管理器完成
        if not object_manager_event.wait(timeout=30):
            logging.error("等待物件管理器超時")
            return False
        
        with system_lock:
            # CCD1模型確認（如果可用）
            if robot.CCD1 and hasattr(robot.CCD1, 'initialized') and robot.CCD1.initialized:
                if ccd1_model_id != 1:
                    if robot.switch_ccd1_model(ccd1_model_id):
                        logging.info(f"CCD1模型切換到: {ccd1_model_id}")
                    else:
                        logging.warning(f"CCD1模型切換失敗: {ccd1_model_id}")
                else:
                    logging.info(f"CCD1使用預設模型: {ccd1_model_id}")
            else:
                logging.warning("CCD1不可用，跳過模型設定")
            
            # 夾爪開度設定（如果可用）
            if robot.Gripper and hasattr(robot.Gripper, 'is_connected') and robot.Gripper.is_connected():
                if robot.gripper_smart_release(gripper_opening):
                    logging.info(f"夾爪開度設定為: {gripper_opening}")
                else:
                    logging.warning("夾爪開度設定失敗")
            else:
                logging.warning("夾爪不可用，跳過開度設定")
            
            # 震動盤參數設定（修正檢查邏輯）
            logging.info(f"震動盤狀態檢查:")
            logging.info(f"  robot.VP: {robot.VP}")
            if robot.VP:
                logging.info(f"  hasattr connected: {hasattr(robot.VP, 'connected')}")
                if hasattr(robot.VP, 'connected'):
                    logging.info(f"  VP.connected: {robot.VP.connected}")
                if hasattr(robot.VP, 'is_connected'):
                    logging.info(f"  VP.is_connected(): {robot.VP.is_connected()}")
            
            # 嘗試多種震動盤狀態檢查方式
            vp_available = False
            if robot.VP:
                if hasattr(robot.VP, 'is_connected') and callable(robot.VP.is_connected):
                    vp_available = robot.VP.is_connected()
                elif hasattr(robot.VP, 'connected'):
                    vp_available = robot.VP.connected
                else:
                    # 嘗試調用背光測試連接
                    try:
                        test_result = robot.vp_set_backlight(True, 50)
                        vp_available = test_result
                        logging.info(f"震動盤測試連接結果: {test_result}")
                    except Exception as e:
                        logging.warning(f"震動盤測試連接失敗: {e}")
                        vp_available = False
            
            if vp_available:
                try:
                    if robot.vp_set_backlight(True, 50):
                        logging.info(f"震動盤背光設定成功")
                    else:
                        logging.warning("震動盤背光設定失敗")
                    logging.info(f"震動盤參數 - 強度:{vp_spread_strength}, 頻率:{vp_spread_frequency}")
                except Exception as e:
                    logging.error(f"震動盤操作異常: {e}")
            else:
                logging.warning("震動盤不可用，跳過參數設定")
            
            logging.info("參數設定完成")
            return True
            
    except Exception as e:
        logging.error(f"參數設定異常: {e}")
        return False

def auto_feeding_thread():
    """自動進料事件執行緒 - 修正震動盤操作"""
    global robot, material_available, label_0_objects, label_1_objects, label_2_objects, flow1_execution_count
    
    try:
        logging.info("自動進料事件啟動")
        
        # 等待參數設定完成
        if not parameter_setup_event.wait(timeout=30):
            logging.error("等待參數設定超時")
            return False
        
        # 檢查CCD1是否可用
        if not (robot.CCD1 and hasattr(robot.CCD1, 'initialized') and robot.CCD1.initialized):
            logging.error("CCD1不可用，無法執行自動進料")
            material_available = False
            return False
        
        retry_count = 0
        max_retries = 10
        
        while retry_count < max_retries:
            try:
                with system_lock:
                    # CCD1檢測
                    try:
                        detections = robot.get_ccd1_counts()
                        if not detections:
                            logging.error("CCD1檢測失敗")
                            break
                    except Exception as e:
                        logging.error(f"CCD1檢測異常: {e}")
                        break
                    
                    # 提取數量信息
                    label_0_count = detections.get(0, 0)
                    label_1_count = detections.get(1, 0)
                    label_2_count = detections.get(2, 0)
                    
                    # 獲取座標信息
                    try:
                        coords_result = robot.get_ccd1_coordinates(use_last_result=True)
                        if 'world_coordinates' in coords_result:
                            world_coords = coords_result['world_coordinates']
                        elif 'world' in coords_result:
                            world_coords = coords_result['world']
                        else:
                            world_coords = {}
                    except Exception as e:
                        logging.warning(f"獲取世界座標失敗: {e}，使用像素座標")
                        if 'pixel' in coords_result:
                            world_coords = coords_result['pixel']
                        else:
                            world_coords = {}
                    
                    logging.info(f"檢測結果 - 標籤0:{label_0_count}, 標籤1:{label_1_count}, 標籤2:{label_2_count}")
                    
                    # 判斷標籤0數量
                    if label_0_count >= 1:
                        label_0_objects = world_coords.get(0, [])
                        label_1_objects = world_coords.get(1, [])
                        label_2_objects = world_coords.get(2, [])
                        flow1_execution_count = 0
                        
                        logging.info(f"物料準備完成 - 標籤0數量:{len(label_0_objects)}個")
                        return True
                    
                    # 執行進料邏輯
                    total_other_count = label_1_count + label_2_count
                    logging.info(f"標籤0不足，其他物件總數: {total_other_count}")
                    
                    if total_other_count < 5:
                        # 震動進料
                        logging.info("其他物件數量<5，執行震動進料控制")
                        if hasattr(robot, 'vibration_feed'):
                            try:
                                robot.vibration_feed()
                                logging.info("震動進料執行成功")
                            except Exception as e:
                                logging.warning(f"震動進料執行失敗: {e}")
                        else:
                            logging.warning("震動進料功能不可用")
                    else:
                        # 震動盤震動
                        logging.info("其他物件數量≥5，開始震動盤震動")
                        
                        # 多種方式嘗試震動盤操作
                        vp_success = False
                        if robot.VP:
                            try:
                                # 方法1: 直接調用robot的方法
                                result = robot.vp_start_vibration("vertical", vp_spread_strength, vp_spread_frequency, vp_spread_duration)
                                if result:
                                    logging.info("震動盤啟動成功")
                                    time.sleep(vp_spread_duration + 0.5)
                                    robot.vp_stop_vibration()
                                    vp_success = True
                                else:
                                    logging.warning("震動盤啟動失敗")
                            except Exception as e:
                                logging.warning(f"方法1震動盤操作失敗: {e}")
                                
                                # 方法2: 直接調用VP對象的方法
                                try:
                                    if hasattr(robot.VP, 'start_vibration'):
                                        robot.VP.start_vibration(vp_spread_strength, vp_spread_frequency)
                                        logging.info("震動盤直接啟動成功")
                                        time.sleep(vp_spread_duration)
                                        robot.VP.stop_vibration()
                                        vp_success = True
                                except Exception as e2:
                                    logging.warning(f"方法2震動盤操作失敗: {e2}")
                        
                        if not vp_success:
                            logging.warning("震動盤操作失敗，跳過震動")
                    
                    retry_count += 1
                    logging.info(f"進料完成，重新檢測 - 第{retry_count}次嘗試")
                    
            except Exception as e:
                logging.error(f"自動進料循環異常: {e}")
                break
            
            time.sleep(0.5)
        
        material_available = False
        logging.error("連續10次檢測無標籤0物件，程序結束")
        return False
        
    except Exception as e:
        logging.error(f"自動進料異常: {e}")
        return False

def robot_pickup_thread():
    """機械臂上料事件執行緒 - Flow1流程直接實作"""
    global robot, label_0_objects, flow1_execution_count, current_angle, command_angle
    
    try:
        logging.info("機械臂上料事件啟動(Flow1流程)")
        
        # 等待自動進料完成
        if not auto_feeding_event.wait(timeout=60):
            logging.error("等待自動進料超時")
            return False
        
        # 檢查是否有可用的標籤0物件
        if flow1_execution_count >= len(label_0_objects):
            logging.info("所有標籤0物件已處理完成，需要重新進料")
            return False
        
        with system_lock:
            try:
                # 獲取當前要處理的物件座標
                if flow1_execution_count >= len(label_0_objects):
                    logging.error("沒有可用的標籤0物件座標")
                    return False
                
                target_coord = label_0_objects[flow1_execution_count]
                logging.info(f"處理第{flow1_execution_count + 1}個物件，座標:({target_coord[0]:.2f}, {target_coord[1]:.2f})")
                
                # 檢查機械臂是否可用
                if not (robot.Dobot and hasattr(robot.Dobot, 'is_connected') and robot.Dobot.is_connected()):
                    logging.error("機械臂不可用")
                    return False
                if robot.Gripper and hasattr(robot.Gripper, 'is_connected') and robot.Gripper.is_connected():
                    robot.gripper_quick_close()
                    time.sleep(0.3)
                    logging.info("夾爪關閉完成")
                else:
                    logging.warning("夾爪不可用，跳過夾爪操作")
                # Flow1流程開始
                # 1. 切換到左手手勢
                if hasattr(robot.Dobot, 'dashboard') and robot.Dobot.dashboard:
                    try:
                        robot.Dobot.dashboard.SetArmOrientation(1)
                        logging.info("切換到左手手勢")
                    except Exception as e:
                        logging.warning(f"手勢切換失敗: {e}，繼續執行")
                
                # 2. 移動到standby
                if "standby" in robot_points:
                    point = robot_points["standby"]
                    if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                        robot.Dobot.move.JointMovJ(point['j1'], point['j2'], point['j3'], point['j4'])
                        robot.dobot_sync()
                        logging.info("移動到standby完成")
                
                # 3. 移動到VP_TOPSIDE
                if "VP_TOPSIDE" in robot_points:
                    point = robot_points["VP_TOPSIDE"]
                    if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                        robot.Dobot.move.JointMovJ(point['j1'], point['j2'], point['j3'], point['j4'])
                        robot.dobot_sync()
                        logging.info("移動到VP_TOPSIDE完成")
                
                # 4. 移動到標籤0的XY座標(VP_TOPSIDE同高)
                vp_point = robot_points["VP_TOPSIDE"]
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.MovL(target_coord[0], target_coord[1], vp_point['z'], vp_point['r'])
                    robot.dobot_sync()
                    logging.info("移動到檢測位置上方完成")
                
                # 5. 移動到標籤0的XY座標(PICKUP_HEIGHT高度)
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.MovL(target_coord[0], target_coord[1], PICKUP_HEIGHT, vp_point['r'])
                    robot.dobot_sync()
                    logging.info("移動到夾取位置完成")
                
                # 6. 撐開夾爪
                if robot.Gripper and hasattr(robot.Gripper, 'is_connected') and robot.Gripper.is_connected():
                    robot.gripper_smart_release(gripper_opening)
                    time.sleep(0.3)
                    logging.info("夾爪撐開完成")
                else:
                    logging.warning("夾爪不可用，跳過夾爪操作")
                
                # 7. 返回VP_TOPSIDE
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.MovL(vp_point['x'], vp_point['y'], vp_point['z'], vp_point['r'])
                    robot.dobot_sync()
                    logging.info("返回VP_TOPSIDE完成")
                
                # 8. 返回standby
                standby_point = robot_points["standby"]
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.JointMovJ(standby_point['j1'], standby_point['j2'], standby_point['j3'], standby_point['j4'])
                    robot.dobot_sync()
                    logging.info("返回standby完成")
                
                # 9. 移動到Rotate_top
                if "Rotate_top" in robot_points:
                    point = robot_points["Rotate_top"]
                    if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                        robot.Dobot.move.JointMovJ(point['j1'], point['j2'], point['j3'], point['j4'])
                        robot.dobot_sync()
                        logging.info("移動到Rotate_top完成")
                
                # 10. 移動到Rotate_down
                if "Rotate_down" in robot_points:
                    point = robot_points["Rotate_down"]
                    if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                        robot.Dobot.move.JointMovJ(point['j1'], point['j2'], point['j3'], point['j4'])
                        robot.dobot_sync()
                        logging.info("移動到Rotate_down完成")
                
                # 11. 夾爪關閉
                if robot.Gripper and hasattr(robot.Gripper, 'is_connected') and robot.Gripper.is_connected():
                    robot.gripper_quick_close()
                    time.sleep(0.3)
                    logging.info("夾爪關閉完成")
                else:
                    logging.warning("夾爪不可用，跳過夾爪操作")
                
                # 12. 返回Rotate_top
                rotate_top_point = robot_points["Rotate_top"]
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.JointMovJ(rotate_top_point['j1'], rotate_top_point['j2'], rotate_top_point['j3'], rotate_top_point['j4'])
                    robot.dobot_sync()
                    logging.info("返回Rotate_top完成")
                
                # 13. 返回standby
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.JointMovJ(standby_point['j1'], standby_point['j2'], standby_point['j3'], standby_point['j4'])
                    robot.dobot_sync()
                    logging.info("返回standby完成")
                
                # 14. CCD3獲取角度 - 修正狀態檢查
                try:
                    # 修正：使用subsystem_status檢查CCD3是否可用
                    if robot.subsystem_status.get('CCD3', False):
                        # 嘗試調用Robot類的get_angle方法
                        if hasattr(robot, 'get_angle'):
                            angle_result = robot.get_angle()
                            if angle_result is not None:
                                current_angle = float(angle_result)
                                command_angle = current_angle + ANGLE_OFFSET
                                logging.info(f"CCD3角度檢測成功 - 檢測角度:{current_angle:.2f}°, 指令角度:{command_angle:.2f}°")
                            else:
                                current_angle = 0.0
                                command_angle = ANGLE_OFFSET
                                logging.warning("CCD3角度檢測失敗，使用預設值")
                        else:
                            # 直接調用CCD3的capture_and_detect方法
                            detection_result = robot.CCD3.capture_and_detect()
                            if detection_result and hasattr(detection_result, 'angle') and detection_result.angle is not None:
                                current_angle = float(detection_result.angle)
                                command_angle = current_angle + ANGLE_OFFSET
                                logging.info(f"CCD3角度檢測成功 - 檢測角度:{current_angle:.2f}°, 指令角度:{command_angle:.2f}°")
                            else:
                                current_angle = 0.0
                                command_angle = ANGLE_OFFSET
                                logging.warning("CCD3角度檢測失敗，使用預設值")
                    else:
                        current_angle = 0.0
                        command_angle = ANGLE_OFFSET
                        logging.info(f"CCD3不可用，使用預設角度: {command_angle:.2f}°")
                except Exception as e:
                    logging.error(f"CCD3角度檢測異常: {e}")
                    current_angle = 0.0
                    command_angle = ANGLE_OFFSET
                
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
                if robot.Gripper and hasattr(robot.Gripper, 'is_connected') and robot.Gripper.is_connected():
                    robot.gripper_smart_release(gripper_opening)
                    time.sleep(0.3)
                else:
                    logging.warning("夾爪不可用，跳過夾爪操作")
                
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
                logging.info(f"Flow1流程完成，執行次數: {flow1_execution_count}")
                
                return True
                
            except Exception as e:
                logging.error(f"Flow1流程執行異常: {e}")
                return False
    
    except Exception as e:
        logging.error(f"機械臂上料事件異常: {e}")
        return False

def robot_assembly_thread():
    """機械臂組裝事件執行緒 - Flow2流程直接實作"""
    global robot, command_angle
    
    try:
        logging.info("機械臂組裝事件啟動(Flow2流程)")
        
        # 等待機械臂上料完成
        if not robot_pickup_event.wait(timeout=60):
            logging.error("等待機械臂上料超時")
            return False
        
        # 等待5秒
        time.sleep(5)
        
        with system_lock:
            try:
                # 檢查機械臂是否可用
                if not (robot.Dobot and hasattr(robot.Dobot, 'is_connected') and robot.Dobot.is_connected()):
                    logging.error("機械臂不可用")
                    return False
                
                # Flow2流程開始
                logging.info(f"開始組裝流程，使用角度: {command_angle:.2f}°")
                
                # 1. 移動到put_asm_top (帶J4角度)
                if "put_asm_top" in robot_points:
                    point = robot_points["put_asm_top"]
                    if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                        robot.Dobot.move.JointMovJ(point['j1'], point['j2'], point['j3'], command_angle)
                        robot.dobot_sync()
                        logging.info(f"移動到put_asm_top完成 (J4角度:{command_angle:.2f}°)")
                
                # 2. 移動到put_asm_down (帶J4角度)
                if "put_asm_down" in robot_points:
                    point = robot_points["put_asm_down"]
                    if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                        robot.Dobot.move.JointMovJ(point['j1'], point['j2'], point['j3'], command_angle)
                        robot.dobot_sync()
                        logging.info(f"移動到put_asm_down完成 (J4角度:{command_angle:.2f}°)")
                
                # 3. 夾爪關閉
                if robot.Gripper and hasattr(robot.Gripper, 'is_connected') and robot.Gripper.is_connected():
                    robot.gripper_quick_close()
                    time.sleep(0.3)
                    logging.info("夾爪關閉完成")
                else:
                    logging.warning("夾爪不可用，跳過夾爪操作")
                
                # 4. 返回put_asm_top
                put_asm_top_point = robot_points["put_asm_top"]
                if hasattr(robot.Dobot, 'move') and robot.Dobot.move:
                    robot.Dobot.move.JointMovJ(put_asm_top_point['j1'], put_asm_top_point['j2'], put_asm_top_point['j3'], command_angle)
                    robot.dobot_sync()
                    logging.info("返回put_asm_top完成")
                
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
    
    except Exception as e:
        logging.error(f"機械臂組裝事件異常: {e}")
        return False

def main():
    """主程式入口 - 完整版本"""
    # 設置日誌
    logger = setup_logging()
    logging.info("自動化系統啟動")
    
    try:
        # 1. 啟動物件管理器執行緒
        logging.info("=== 階段1: 初始化物件管理器 ===")
        object_manager_thread_handle = threading.Thread(target=object_manager_thread, daemon=True)
        object_manager_thread_handle.start()
        object_manager_thread_handle.join(timeout=60)
        
        if object_manager_thread_handle.is_alive():
            logging.error("物件管理器初始化超時")
            return
        
        object_manager_event.set()
        logging.info("物件管理器初始化完成")
        
        # 2. 啟動參數設定事件
        logging.info("=== 階段2: 執行參數設定 ===")
        parameter_setup_thread_handle = threading.Thread(target=parameter_setup_thread, daemon=True)
        parameter_setup_thread_handle.start()
        parameter_setup_thread_handle.join(timeout=30)
        
        if parameter_setup_thread_handle.is_alive():
            logging.error("參數設定超時")
            return
        
        parameter_setup_event.set()
        logging.info("參數設定完成")
        
        # 3. 啟動自動進料事件
        logging.info("=== 階段3: 執行自動進料 ===")
        auto_feeding_thread_handle = threading.Thread(target=auto_feeding_thread, daemon=True)
        auto_feeding_thread_handle.start()
        auto_feeding_thread_handle.join(timeout=120)
        
        if auto_feeding_thread_handle.is_alive():
            logging.error("自動進料超時")
            return
        
        auto_feeding_event.set()
        logging.info("自動進料完成")
        
        # 4. 檢查是否可以進入生產循環
        if not material_available:
            logging.error("原物料不足，無法開始生產循環")
            return
        
        if not (robot.Dobot and hasattr(robot.Dobot, 'is_connected') and robot.Dobot.is_connected()):
            logging.error("機械臂不可用，無法開始生產循環")
            return
        
        logging.info("=== 階段4: 開始生產循環 ===")
        
        # 主生產循環
        cycle_count = 0
        max_cycles = 100  # 設定最大循環次數，避免無限循環
        
        while material_available and cycle_count < max_cycles:
            try:
                logging.info(f"--- 開始第{cycle_count + 1}次生產循環 ---")
                
                # 檢查是否需要重新進料
                if flow1_execution_count >= len(label_0_objects):
                    logging.info("需要重新進料")
                    auto_feeding_event.clear()
                    auto_feeding_thread_handle = threading.Thread(target=auto_feeding_thread, daemon=True)
                    auto_feeding_thread_handle.start()
                    auto_feeding_thread_handle.join(timeout=120)
                    
                    if not material_available:
                        logging.info("重新進料失敗，結束生產循環")
                        break
                    
                    auto_feeding_event.set()
                
                # 4a. 機械臂上料事件 (Flow1)
                logging.info(f"執行機械臂上料事件 - 處理第{flow1_execution_count + 1}個物件")
                robot_pickup_event.clear()
                robot_pickup_thread_handle = threading.Thread(target=robot_pickup_thread, daemon=True)
                robot_pickup_thread_handle.start()
                robot_pickup_thread_handle.join(timeout=180)  # 3分鐘超時
                
                if robot_pickup_thread_handle.is_alive():
                    logging.error("機械臂上料超時")
                    break
                
                robot_pickup_event.set()
                logging.info("機械臂上料完成")
                
                # 4b. 機械臂組裝事件 (Flow2)
                logging.info("執行機械臂組裝事件")
                robot_assembly_event.clear()
                robot_assembly_thread_handle = threading.Thread(target=robot_assembly_thread, daemon=True)
                robot_assembly_thread_handle.start()
                robot_assembly_thread_handle.join(timeout=120)  # 2分鐘超時
                
                if robot_assembly_thread_handle.is_alive():
                    logging.error("機械臂組裝超時")
                    break
                
                robot_assembly_event.set()
                logging.info("機械臂組裝完成")
                
                cycle_count += 1
                logging.info(f"第{cycle_count}次生產循環完成")
                
                # 重置事件以便下次循環
                robot_pickup_event.clear()
                robot_assembly_event.clear()
                
                # 短暫休息
                time.sleep(1)
                
            except Exception as e:
                logging.error(f"生產循環{cycle_count + 1}異常: {e}")
                break
        
        if cycle_count >= max_cycles:
            logging.info(f"達到最大循環次數({max_cycles})，正常結束")
        elif not material_available:
            logging.info("原物料不足，生產循環結束")
        else:
            logging.info("生產循環異常結束")
        
        logging.info(f"總共完成 {cycle_count} 次生產循環")
        
    except KeyboardInterrupt:
        logging.info("收到中斷信號，正在關閉系統")
    except Exception as e:
        logging.error(f"系統運行異常: {e}")
    finally:
        # 安全清理資源
        try:
            if robot:
                logging.info("開始安全關閉系統...")
                if hasattr(robot, 'emergency_stop'):
                    robot.emergency_stop()
                if hasattr(robot, 'disconnect_all'):
                    robot.disconnect_all()
                logging.info("系統已安全關閉")
        except Exception as e:
            logging.error(f"系統關閉時發生錯誤: {e}")

if __name__ == "__main__":
    main()