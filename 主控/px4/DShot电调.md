先强调一下Pixhawk 6C有两个电机输出口，一个是FMU（AUX），另外一个是I/O（MAIN）。
io那边不支持dshot协议。要使用dhsot，只能接到fmu那边！！！！！！！！！！！！dshot不用校准，普通PWM电调要校准


参数|设置
|-|-|
SYS_USE_IO|disable
DSHOT_CONFIG|DShot600

电机反转教学参考下面的文章

DShot电调：https://docs.px4.io/v1.15/zh/peripherals/dshot.html#commands

更改顺序

https://docs.px4.io/v1.12/zh/config_mc/racer_setup.html

四合一电调常用电机顺序更改参数，MOT_ORDERING




