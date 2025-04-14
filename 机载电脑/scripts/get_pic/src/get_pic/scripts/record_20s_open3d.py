#!/home/eh/.venv/bin/python3
import rospy
from livox_ros_driver2.msg import CustomMsg  # 修改为正确的消息类型
import open3d as o3d
import numpy as np
import threading
import datetime

points = []
recording = True
lock = threading.Lock()

def cloud_callback(msg):
    global points
    # 假设msg包含XYZ坐标字段，您需要根据CustomMsg结构调整
    cloud_data = [(point.x, point.y, point.z) for point in msg.points]  # 根据CustomMsg中的数据结构处理
    with lock:
        points.extend(cloud_data)

def stop_recording(seconds):
    global recording
    rospy.sleep(seconds)
    recording = False
    rospy.loginfo("⏱️ 录制结束，正在保存点云...")

def save_pcd_file(filename, points):
    pc_np = np.array(points, dtype=np.float32)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pc_np)
    o3d.io.write_point_cloud(filename, pcd, write_ascii=False)
    rospy.loginfo("✅ 点云保存至: %s", filename)

if __name__ == '__main__':
    rospy.init_node('record_20s_open3d')

    rospy.Subscriber("/livox/lidar", CustomMsg, cloud_callback)  # 修改为CustomMsg类型
    rospy.loginfo("📡 正在录制点云数据，持续 20 秒...")
    threading.Thread(target=stop_recording, args=(20,)).start()
    current_time = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and recording:
        rate.sleep()

    with lock:
        if points:
            save_pcd_file(f"./pcd/merged_20s_cloud_{current_time}.pcd", points)
        else:
            rospy.logwarn("⚠️ 没有收到任何点云数据！")

