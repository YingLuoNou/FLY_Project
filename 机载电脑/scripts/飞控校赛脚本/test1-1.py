#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
takeoff_hover_land.py

Python 脚本：
1. 使用 MAVROS 控制 PX4 飞控
2. 自动起飞到高度 1m
3. 悬停 5 秒
4. 下降至地面并解锁（或着陆）

使用方法：
$ chmod +x takeoff_hover_land.py
$ ./takeoff_hover_land.py

注意：请确保已启动 mavros 节点，并根据实际情况设置命名空间和 Frame ID。
"""

import rospy
import math
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped

current_state = None
current_pose = None

# 状态回调
def state_cb(msg):
    global current_state
    current_state = msg

# 位姿回调
def pose_cb(msg):
    global current_pose
    current_pose = msg

# 主程序
if __name__ == '__main__':
    rospy.init_node('takeoff_hover_land', anonymous=True)

    # 订阅飞控状态和本地位姿
    rospy.Subscriber('mavros/state', State, state_cb)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, pose_cb)
    pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # Service 客户端
    rospy.wait_for_service('mavros/cmd/arming')
    rospy.wait_for_service('mavros/set_mode')
    arm_service = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    mode_service = rospy.ServiceProxy('mavros/set_mode', SetMode)

    rate = rospy.Rate(20)  # 20Hz

    # 等待与飞控建立连接
    while not rospy.is_shutdown() and (current_state is None or not current_state.connected):
        rate.sleep()
    rospy.loginfo('FCU connected')

    # 构造目标位姿
    target = PoseStamped()
    target.header.frame_id = 'map'
    target.pose.position.x = 0.0
    target.pose.position.y = 0.0
    target.pose.position.z = 0.0
    # 保持水平姿态
    target.pose.orientation.x = 0.0
    target.pose.orientation.y = 0.0
    target.pose.orientation.z = 0.0
    target.pose.orientation.w = 1.0

    # 发布预热 setpoints
    for _ in range(100):
        pose_pub.publish(target)
        rate.sleep()

    # 切换 OFFBOARD 并解锁
    mode_service(0, 'OFFBOARD')
    arm_service(True)
    rospy.loginfo('OFFBOARD enabled, vehicle armed')

    pose_pub.publish(target)
    #rate.sleep()

    # 起飞到 1m
    rospy.loginfo('Taking off to 1.0 m')
    target.pose.position.z = 1.0
    # 持续发布，直到达到高度
    while not rospy.is_shutdown():
        if current_pose is not None:
            z = current_pose.pose.position.z
            if abs(z - 1.0) < 0.05:  # 5cm 容差
                break
        target.header.stamp = rospy.Time.now()
        pose_pub.publish(target)
        rate.sleep()

    rospy.loginfo('Reached 1.0 m, hovering for 5 seconds')
    # 悬停 5 秒
    hover_start = rospy.Time.now()
    while not rospy.is_shutdown() and (rospy.Time.now() - hover_start).to_sec() < 5.0:
        target.header.stamp = rospy.Time.now()
        pose_pub.publish(target)
        rate.sleep()

    # 下降至地面
    rospy.loginfo('Descending to ground')
    target.pose.position.z = 0.0
    while not rospy.is_shutdown():
        if current_pose is not None:
            z = current_pose.pose.position.z
            if z < 0.1:  # 接近地面
                break
        target.header.stamp = rospy.Time.now()
        pose_pub.publish(target)
        rate.sleep()

    # 解锁或着陆
    rospy.loginfo('Landing complete, disarming')
    arm_service(False)

    rospy.loginfo('Script finished')
    rospy.spin()
