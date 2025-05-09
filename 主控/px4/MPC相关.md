## MPC

MPC_* 是一组用于 位置控制器的参数，控制无人机在位置模式（Position）、任务模式（Mission）、Offboard 模式(程控模式)等自动控制状态下的飞行行为
关键MPC参数与程控模式下的建议值

THR
|参数|作用|建议值|
|-|-|-|
MPC_THR_MAX     |  | 60%
MPC_THR_MIN     |  | 15%

Z
|参数|作用|建议值|
|-|-|-|
MPC_Z_VEL_MAX_UP|  上升最大速度 | 0.5m/s
MPC_Z_VEL_MAX_DN|  下降最大速度 | 0.5m/s
MPC_ACC_DOWN_MAX|  垂直最大加速度 | 2m/s<sup>2</sup>
MPC_ACC_UP_MAX  |  |  2m/s<sup>2</sup>
MPC_TKO_SPEED	  |  起飞时上升速度 | 1
MPC_LAND_SPEED  |  降落速度	通常设为 | 0.4
MPC_Z_ERR_MAX	  |  垂直最大偏差限制	米

XY
|参数|作用|建议值|
|-|-|-|
MPC_XY_VEL_MAX  |  最大水平速度	| 1m/s
MPC_XY_CRUISE   |  默认水平巡航速度（Mission/Offboard）| 0.6m/s
MPC_XY_ACC_MAX	|  水平最大加速度	| 1.0m/s<sup>2</sup>
MPC_XY_VEL_ALL	|  所有模式下的最大水平速度（包括自动/手动）
MPC_XY_ERR_MAX  |  水平最大偏差限制
MPC_TILTMAX_AIR |  最大可倾斜角度（°），防止姿态过大失控 | 15°
MPC_MAN_TILT_MAX|  手动最大倾斜角
MPC_YAWRAUTO_MAX|  自动航向转弯最大速度（°/s） | 30°/s
Offboard        |  模式专用参数（例如使用 ROS 控制飞行）

MPC_Z_VEL_P_ACC,该参数 决定了 Z 轴速度误差被放大为多少加速度

例如

目标下降速度为 -1.0 m/s

实际下降速度为 -2.0 m/s

当前参数值：MPC_Z_VEL_P_ACC = 4.0

速度误差 = -1.0 - (-2.0) = 1.0 m/s

纠正加速度 = 4.0 * 1.0 = 4.0 m/s²（上升方向）

高度(Altitude)模式下






