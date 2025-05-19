#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
circle_mission.py

功能：
1. 使用 MAVROS 控制 PX4 飞控
2. 起飞到 1m 高度
3. 起飞前通过串口发送声光警报开启指令 (0x55,0x01,0xFE)，持续 1s，同时持续发布 setpoints；随后发送关闭指令 (0x55,0x02,0xFE)，继续发布 setpoints
4. 在本地坐标系平面 z=1m 上缓慢绘制直径 1m（半径 0.5m）的圆轨迹
5. 圆绘制完成后再次发送 1s 的声光警报，同步持续发布 setpoints；随后发送关闭指令
6. 切换到 AUTO.LAND 并降落，发布 setpoints 直至解锁

使用：
$ chmod +x circle_mission.py
$ rosrun <your_package> circle_mission.py

注意：根据实际情况修改串口、命名空间等。"""

import rospy
import math
import serial
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped

# 全局状态变量
target_state = None
current_pose = None

def state_cb(msg):
    global target_state
    target_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg

if __name__ == '__main__':
    rospy.init_node('circle_mission', anonymous=True)

    # 初始化串口（声光警报）
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    rospy.loginfo("Serial port opened")

    # 命名空间配置
    ns = ''  # 多机可设 'iris_0/'

    # 订阅飞控状态和定位
    rospy.Subscriber(ns + 'mavros/state', State, state_cb)
    rospy.Subscriber(ns + 'mavros/local_position/pose', PoseStamped, pose_cb)
    pose_pub = rospy.Publisher(ns + 'mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # 服务客户端
    rospy.wait_for_service(ns + 'mavros/cmd/arming')
    rospy.wait_for_service(ns + 'mavros/set_mode')
    arm_srv = rospy.ServiceProxy(ns + 'mavros/cmd/arming', CommandBool)
    mode_srv = rospy.ServiceProxy(ns + 'mavros/set_mode', SetMode)

    rate = rospy.Rate(20)  # 20Hz

    # 等待连接\    
    while not rospy.is_shutdown() and (target_state is None or not target_state.connected):
        rate.sleep()
    rospy.loginfo("FCU connected")

    # 定义悬停目标 z=1m\    
    hover = PoseStamped()
    hover.header.frame_id = 'map'
    hover.pose.position.x = 0.0
    hover.pose.position.y = 0.0
    hover.pose.position.z = 1.0
    hover.pose.orientation.x = 0.0
    hover.pose.orientation.y = 0.0
    hover.pose.orientation.z = 0.0
    hover.pose.orientation.w = 1.0

    # 预热 setpoint 发布
    for _ in range(100):
        hover.header.stamp = rospy.Time.now()
        pose_pub.publish(hover)
        rate.sleep()

    # 起飞前报警 ON 1s，同步发布 setpoints
    rospy.loginfo("Alarm ON before takeoff")
    t0 = rospy.Time.now()
    ser.write(bytes([0x0D,0x01,0xFE]))
    while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 1.0:
        hover.header.stamp = rospy.Time.now()
        pose_pub.publish(hover)
        rate.sleep()
    ser.write(bytes([0x0D,0x02,0xFE]))

    # 切换 OFFBOARD 并解锁
    rospy.loginfo("Switch to OFFBOARD and ARM")
    mode_srv(base_mode=0, custom_mode='OFFBOARD')
    arm_srv(True)

    # 发布 setpoints 等待模式和解锁
    for _ in range(100):
        hover.header.stamp = rospy.Time.now()
        pose_pub.publish(hover)
        rate.sleep()
    while not rospy.is_shutdown():
        if target_state.mode == 'OFFBOARD' and target_state.armed:
            rospy.loginfo("OFFBOARD & armed confirmed")
            break
        hover.header.stamp = rospy.Time.now()
        pose_pub.publish(hover)
        rate.sleep()

    # 升到 1m
    rospy.loginfo("Ascending to 1.0m")
    while not rospy.is_shutdown():
        if current_pose and abs(current_pose.pose.position.z - 1.0) < 0.05:
            rospy.loginfo("Reached 1.0m altitude")
            break
        hover.header.stamp = rospy.Time.now()
        pose_pub.publish(hover)
        rate.sleep()

    # 缓慢绘制圆形轨迹
    rospy.loginfo("Drawing circle radius 0.5m")
    radius = 0.5
    num_points = 300
    for i in range(num_points+1):
        theta = 2*math.pi * i / num_points
        hover.pose.position.x = radius * math.cos(theta)
        hover.pose.position.y = radius * math.sin(theta)
        hover.header.stamp = rospy.Time.now()
        pose_pub.publish(hover)
        rate.sleep()

    # 绘制完成报警 ON 1s，同步发布
    rospy.loginfo("Alarm ON after circle")
    t1 = rospy.Time.now()
    ser.write(bytes([0x0D,0x01,0xFE]))
    while not rospy.is_shutdown() and (rospy.Time.now() - t1).to_sec() < 1.0:
        hover.header.stamp = rospy.Time.now()
        pose_pub.publish(hover)
        rate.sleep()
    ser.write(bytes([0x0D,0x02,0xFE]))

    # 自动降落
    rospy.loginfo("Initiating AUTO.LAND")
    mode_srv(base_mode=0, custom_mode='AUTO.LAND')
    # 持续发布直到解锁
    while not rospy.is_shutdown() and target_state.armed:
        hover.header.stamp = rospy.Time.now()
        pose_pub.publish(hover)
        rate.sleep()

    rospy.loginfo("Mission complete: landed and disarmed")
