# WSL2 镜像网卡模式
需要在win11的用户文件夹根目录创建该文件
> [!WARNING]
> 注意,该文件默认是不存在的
其他配置参考:
[微软官网:WSL 中的高级设置配置](https://learn.microsoft.com/zh-cn/windows/wsl/wsl-config#wslconfig)
[使用 WSL 访问网络应用程序](https://learn.microsoft.com/zh-cn/windows/wsl/networking#mirrored-mode-networking)
```
# .wslconfig 文件

[wsl2]
networkingMode=mirrored
```
