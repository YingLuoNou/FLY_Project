source /home/yln/ws_Livox2/devel/setup.sh
nohup roslaunch livox_ros_driver2 msg_MID360.launch & sleep 0.5
source /home/yln/ws_hikrobot_camera/devel/setup.sh
nohup roslaunch hikrobot_camera hikrobot_camera.launch & sleep 0.5
PID1=$!
rosrun topic_tools relay /livox/lidar /sensor_msgs/PointCloud2 & sleep 0.5
rosrun topic_tools relay /left_camera/image /sensor_msgs/Image & sleep 0.5
rosrun topic_tools relay /left_camera/camera_info /sensor_msgs/CameraInfo & sleep 0.5
# 开始录制
echo "开始录制 bag 文件，持续 30 秒..."
rosbag record /sensor_msgs/PointCloud2 /sensor_msgs/Image /sensor_msgs/CameraInfo & sleep 1.5
PID=$!
sleep 35
echo "录制完毕"
kill -2 $PID
kill -2 $PID1
rosnode kill -a
