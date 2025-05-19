#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
rotate_and_land.py

Python 脚本：
1. 使用 MAVROS 控制 PX4 飞控
2. 起飞到高度 1m 并悬停
3. 等待飞机实际达到 1m，再开始旋转任务
4. 按顺序执行平滑旋转到 90°, 180°, 270°, 360°：
   - 使用梯度位姿更新控制 yaw，替代速度控制，更精准、平滑
   - 每到一个角度，停 2s 并发送 `0x55 0x01 0xFE`，2s 后发送 `0x55 0x02 0xFE`
5. 最后切换到 `AUTO.LAND` 自动降落

改动：
- 取消角速度接口，改为基于 PoseStamped 发布带小角增量的 yaw 更新
- 通过线性插值或固定步长分段更新 yaw，使旋转更加平滑且受 PX4 姿态环保护

使用方法：
$ chmod +x rotate_and_land.py
$ rosrun <your_package> rotate_and_land.py

注意：根据实际环境，调整命名空间、串口设备等。"""

import rospy
import math
import serial
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# 全局状态
current_state = None
current_pose = None

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg

def get_current_yaw():
    if current_pose is None:
        return None
    q = current_pose.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    return yaw

if __name__ == '__main__':
    rospy.init_node('rotate_and_land', anonymous=True)

    # 初始化串口
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    rospy.loginfo('Opened serial port.')

    ns = ''  # 多机："iris_0/"
    rospy.Subscriber(ns + 'mavros/state', State, state_cb)
    rospy.Subscriber(ns + 'mavros/local_position/pose', PoseStamped, pose_cb)
    pose_pub = rospy.Publisher(ns + 'mavros/setpoint_position/local', PoseStamped, queue_size=10)

    rospy.wait_for_service(ns + 'mavros/cmd/arming')
    rospy.wait_for_service(ns + 'mavros/set_mode')
    arm_srv = rospy.ServiceProxy(ns + 'mavros/cmd/arming', CommandBool)
    mode_srv = rospy.ServiceProxy(ns + 'mavros/set_mode', SetMode)

    rate = rospy.Rate(20)

    # 等待连接
    while not rospy.is_shutdown() and (current_state is None or not current_state.connected):
        rate.sleep()
    rospy.loginfo('FCU connected')

    # 初始 hover 位姿
    hover = PoseStamped()
    hover.header.frame_id = 'map'
    hover.pose.position.x = 0.0
    hover.pose.position.y = 0.0
    hover.pose.position.z = 1.0
    hover.pose.orientation.x = 0.0
    hover.pose.orientation.y = 0.0
    hover.pose.orientation.z = 0.0
    hover.pose.orientation.w = 1.0

    # 预热 setpoints
    for _ in range(100):
        hover.header.stamp = rospy.Time.now()
        pose_pub.publish(hover)
        rate.sleep()

    # 切 OFFBOARD & 解锁
    mode_srv(0, 'OFFBOARD')
    arm_srv(True)
    rospy.loginfo('OFFBOARD & Armed')

    pose_pub.publish(hover)
    rate.sleep()

    # 等待升到 1m
    rospy.loginfo('Climb to 1m')
    while not rospy.is_shutdown():
        if current_pose and abs(current_pose.pose.position.z - 1.0) < 0.05:
            rospy.loginfo('Reached 1m')
            break
        hover.header.stamp = rospy.Time.now()
        pose_pub.publish(hover)
        rate.sleep()

    angles = [90, 180, 270, 360]
    steps = 80  # 每段旋转分成 50 步
    for deg in angles:
        start_yaw = get_current_yaw()
        if start_yaw is None:
            continue
        target_yaw = math.radians(deg % 360)
        # 计算最短旋转增量
        diff = (target_yaw - start_yaw + math.pi) % (2*math.pi) - math.pi
        rospy.loginfo(f'Smooth rotating to {deg}°, total increment {math.degrees(diff):.1f}°')
        # 分段插值
        for i in range(1, steps+1):
            frac = i/steps
            yaw_i = start_yaw + diff*frac
            q = quaternion_from_euler(0, 0, yaw_i)
            hover.pose.orientation.x = q[0]
            hover.pose.orientation.y = q[1]
            hover.pose.orientation.z = q[2]
            hover.pose.orientation.w = q[3]
            hover.header.stamp = rospy.Time.now()
            pose_pub.publish(hover)
            rate.sleep()

        # 最终锁定 target yaw
        q = quaternion_from_euler(0, 0, target_yaw)
        hover.pose.orientation.x = q[0]
        hover.pose.orientation.y = q[1]
        hover.pose.orientation.z = q[2]
        hover.pose.orientation.w = q[3]
        hover.header.stamp = rospy.Time.now()
        pose_pub.publish(hover)

        # 到位后停 2s 并串口
        rospy.loginfo(f'Reached {deg}°, hold 2s with alarm')
        ser.write(bytes([0x0D, 0x01, 0xFE]))
        t0 = rospy.Time.now()
        while (rospy.Time.now()-t0).to_sec() < 2 and not rospy.is_shutdown():
            hover.header.stamp = rospy.Time.now()
            pose_pub.publish(hover)
            rate.sleep()
        ser.write(bytes([0x0D, 0x02, 0xFE]))
        t1 = rospy.Time.now()
        while (rospy.Time.now()-t1).to_sec() < 2 and not rospy.is_shutdown():
            hover.header.stamp = rospy.Time.now()
            pose_pub.publish(hover)
            rate.sleep()

    rospy.loginfo('Rotation done, landing')
    mode_srv(0, 'AUTO.LAND')
    rospy.spin()
