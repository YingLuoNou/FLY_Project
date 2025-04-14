#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import datetime

class OneImageSaver:
    def __init__(self):
        rospy.init_node('one_image_saver', anonymous=True)

        self.image_topic = "/left_camera/image"
        self.bridge = CvBridge()
        self.save_dir = rospy.get_param("~save_dir", "./images")
        self.saved = False

        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        rospy.Subscriber(self.image_topic, Image, self.callback)
        rospy.loginfo("等图片话题数据 %s", self.image_topic)

    def callback(self, msg):
        if self.saved:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("CVBridge error: %s", e)
            return
        
        current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        filename = os.path.join(self.save_dir, f"{current_time}saved_frame.png")
        cv2.imwrite(filename, cv_image)
        rospy.loginfo("✅ 保存图片到: %s", filename)
        self.saved = True

        # 等1秒再退出（保证写入完成）
        rospy.sleep(1.0)
        rospy.signal_shutdown("图片保存完成。节点退出。")

if __name__ == '__main__':
    try:
        saver = OneImageSaver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
