# WSL2 镜像网卡模式
需要在win11的用户文件夹根目录创建该文件
> [!WARNING]
> 注意,该文件默认是不存在的
```
# .wslconfig 文件

[wsl2]
networkingMode=mirrored
```
