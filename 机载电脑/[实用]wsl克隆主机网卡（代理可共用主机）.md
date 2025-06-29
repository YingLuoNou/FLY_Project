### 配置wsl镜像Windows网卡
1.打开Windows用户文件夹，(win+R输入`%USERPROFILE%`)  
2.新建文件`.wslconfig`,内容:  
```
[experimental]
networkingMode=mirrored
dnsTunneling=true
firewall=true
autoProxy=true
```
3.重启运行wsl:  
```
wsl --shutdown
wsl
```
> 注：1.此配置仅支持Windows11 22h2及以上版本  
> 2.使用管理员权限在 PowerShell 窗口中运行以下命令，以配置 Hyper-V 防火墙设置，从而允许入站连接：  
`NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow`  
[参考](https://learn.microsoft.com/zh-cn/windows/wsl/networking)  
