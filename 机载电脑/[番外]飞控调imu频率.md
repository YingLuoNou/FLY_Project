尝试过sd卡，地面站控制台修改（只能临时修改）均失败  
最终方法在机载电脑上输入  
```bash
rosrun mavros mavcmd long 511 105 4500 0 0 0 0 0
```
```bash
rosrun mavros mavcmd long 511 31 4500 0 0 0 0 0
```
参数解释：511,31,105都是imu参数对应的数字，无需修改  
4500是imu发送频率，例如：  
- 50Hz → 20000（20ms）
- 100Hz → 10000（10ms）
- 200Hz → 5000（5ms）
为了达到200HZ，经过测试，使用4500这个值
在launch启动时就修改频率可以通过修改px4.launch文件
```xml
<launch>
  <!-- ...其它启动配置代码 -->

  <!-- 添加 -->
  <node pkg="mavros" type="command" name="mavcmd_imu1" output="screen" args="511 105 4500 0 0 0 0 0" />
  <node pkg="mavros" type="command" name="mavcmd_imu2" output="screen" args="511 31 4500 0 0 0 0 0" />
<launch>
```
