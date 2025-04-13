## 教程链接
[gitee](https://gitee.com/gwmunan/ros2/wikis/pages?sort_id=10536766&doc_id=4855084)
## 连接wsl与USB设备
[教程](https://learn.microsoft.com/zh-cn/windows/wsl/connect-usb)，需要执行完附加USB设备  
## 安装HIKROBOT-MVS-CAMERA-ROS编译时报错
**注意海康机器人工业相机要安装客户端MVS（不要装成runtime sdk）**  
1.opencv版本报错（cmakelist文件内改版本）  
2.显示找不到文件  
```bash
/usr/bin/ld: 找不到 -lGCBase_gcc421_v3_0
/usr/bin/ld: 找不到 -lGenApi_gcc421_v3_0
/usr/bin/ld: 找不到 -llog4cpp_gcc421_v3_0
/usr/bin/ld: 找不到 -lLog_gcc421_v3_0
/usr/bin/ld: 找不到 -lMathParser_gcc421_v3_0
/usr/bin/ld: 找不到 -lNodeMapData_gcc421_v3_0
/usr/bin/ld: 找不到 -lXmlParser_gcc421_v3_0
```
[解决方法](https://github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS/issues/12):  
对于新版的海康SDK，首先查看你的链接库版本：  
`ls /opt/MVS/lib/64/`  
查看名称，如：libLog_gcc485_v3_0.so，注意其中的485  
然后打开CMakeList  
`vi src/HIKROBOT-MVS-CAMERA-ROS/CMakeLists.txt`  
在第36行：  
```
target_link_libraries(
${PROJECT_NAME}
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
GCBase_gcc485_v3_0
MvCameraControl
#GenApi_gcc485_v3_0
MVGigEVisionSDK
log4cpp_gcc485_v3_0 MVRender
Log_gcc485_v3_0 MvUsb3vTL
#MathParser_gcc485_v3_0
#NodeMapData_gcc485_v3_0
MediaProcess
#XmlParser_gcc485_v3_0 X11
)
```
注释掉Genapi、MathParser、NodeMapData、XmlParser，然后将其他的gcc后面的三位数字改成之前看到的三位数字，我的是485。  
退出，重新make就可以了  
## HIKROBOT-MVS-CAMERA-ROS修改硬件触发
@@修改hikrobot_camera.hpp可以改变ros中的相机触发方式  
```hpp
//软件触发  
// ********** frame **********/
nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);//0-不使用外部触发；1-使用外部触发
```  
然后重新catkin_make  
[来源](https://github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS/issues/7)  
## 安装无目标标定环境Ceres一定要使用2.0.0及以下版本
[2.0.0版本链接]([https://ceres-solver.googlesource.com/ceres-solver/+/refs/tags/1.14.0](http://ceres-solver.org/ceres-solver-2.0.0.tar.gz))  
`wget http://ceres-solver.org/ceres-solver-2.0.0.tar.gz`  
## 安装无目标标定(lidar_camera_calib)环境Ceres Solver时报错
```bash
CMake Error at CMakeLists.txt:173 (find_package):
  By not providing "Findabsl.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "absl", but
  CMake did not find one.

  Could not find a package configuration file provided by "absl" with any of
  the following names:

    abslConfig.cmake
    absl-config.cmake

  Add the installation prefix of "absl" to CMAKE_PREFIX_PATH or set
  "absl_DIR" to a directory containing one of the above files.  If "absl"
  provides a separate development package or SDK, be sure it has been
  installed.


-- Configuring incomplete, errors occurred!
```
解决方法：  
源码安装  
```bash
git clone https://github.com/abseil/abseil-cpp.git
cd abseil-cpp
mkdir build && cd build
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
```
## 无目标标定(lidar_camera_calib)catkin_make出错
```bash
[ 66%] Linking CXX executable /home/yangluonou/livox_camera_calib/devel/lib/livox_camera_calib/bag_to_pcd
/usr/bin/ld: /usr/lib/gcc/x86_64-linux-gnu/9/../../../x86_64-linux-gnu/libpcl_io.so: undefined reference to `libusb_set_option'
collect2: error: ld returned 1 exit status
make[2]: *** [livox_camera_calib/CMakeFiles/bag_to_pcd.dir/build.make:452: /home/yangluonou/livox_camera_calib/devel/lib/livox_camera_calib/bag_to_pcd] Error 1
make[1]: *** [CMakeFiles/Makefile2:2551: livox_camera_calib/CMakeFiles/bag_to_pcd.dir/all] Error 2
make[1]: *** Waiting for unfinished jobs....
```
这个链接错误是由于 libpcl_io.so 在链接时引用了 libusb_set_option 函数，但你当前系统中的 libusb 版本可能较旧（比如 1.0.23），而不包含这个函数。libusb_set_option 是从 libusb 1.0.26 才引入的。  
```bash
sudo apt install libudev-dev
wget https://github.com/libusb/libusb/releases/download/v1.0.26/libusb-1.0.26.tar.bz2
tar -xjf libusb-1.0.26.tar.bz2
cd libusb-1.0.26
./configure --prefix=/usr/local
make -j$(nproc)
sudo make install
```
## libusb_set_option报错
这个链接错误是由于 libpcl_io.so 在链接时引用了 libusb_set_option 函数，但你当前系统中的 libusb 版本可能较旧（比如 1.0.23），而不包含这个函数。libusb_set_option 是从 libusb 1.0.26 才引入的。  
- 检查当前版本：  
  ```bash
  pkg-config --modversion libusb-1.0
  ```
- 下载源码并编译新版本 libusb：  
  ```bash
  sudo apt remove libusb-1.0-0-dev
  
  wget https://github.com/libusb/libusb/releases/download/v1.0.26/libusb-1.0.26.tar.bz2
  tar -xjf libusb-1.0.26.tar.bz2
  cd libusb-1.0.26
  ./configure --prefix=/usr/local
  make -j$(nproc)
  sudo make install
  ```
- 确保新版本被使用：
  ```bash
  export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
  ```
  如果你使用 catkin 编译，建议在 CMakeLists.txt 中指定 libusb 路径：
  ```txt
  link_directories(/usr/local/lib)
  include_directories(/usr/local/include/libusb-1.0)
  ```
## mid360无需连接stm32 PA9进行卫星授时
否则会出现时间对不齐问题
