## 法一：
launch文件内添加  
```xml
<arg name="enable_infra1" default="true"/>
<arg name="enable_infra2" default="true"/>
```
## 法二；
添加运行参数：  
```bash
roslaunch realsense2_camera rs_camera.launch enable_infra1:=true enable_infra2:=true
```
