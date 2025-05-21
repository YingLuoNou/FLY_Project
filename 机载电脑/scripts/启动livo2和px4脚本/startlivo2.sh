source /home/yln/ws_Livox2/devel/setup.sh
nohup roslaunch livox_ros_driver2 msg_MID360.launch & sleep 1
source /home/yln/ws_hikrobot_camera/devel/setup.sh
nohup roslaunch hikrobot_camera hikrobot_camera.launch & sleep 1
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
source /home/yln/ws_livo2/devel/setup.sh
roslaunch fast_livo mapping_mid360.launch rviz:=false
