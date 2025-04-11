## 教程链接
[gitee](https://gitee.com/gwmunan/ros2/wikis/pages?sort_id=10536766&doc_id=4855084)
## 安装HIKROBOT-MVS-CAMERA-ROS编译时报错
**注意要安装客户端MVS**  
1.opencv版本报错（cmake改版本）  
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
ls /opt/MVS/lib/64/  
查看名称，如：libLog_gcc485_v3_0.so，注意其中的485  
然后打开CMakeList  
vi src/HIKROBOT-MVS-CAMERA-ROS/CMakeLists.txt  
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
[来源](https://github.com/luckyluckydadada/HIKROBOT-MVS-CAMERA-ROS/issues/7)  
