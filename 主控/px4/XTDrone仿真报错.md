## mode not suitable for takeoff

<img width="600" alt="image" src="https://github.com/user-attachments/assets/77cb879d-4eea-413f-bff3-e58cf5937931" />

px4: disable RC requirement before arming by setting COM_RCL_EXCEPT=4

在`/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/airframes`路径下的`10016_iris`(对应无人机型号)文件进行配置
```bash
echo "param set COM_RCL_EXCEPT 4" >> PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/10016_iris
make ~/PX4_Firmware/px4_sitl_default
```
