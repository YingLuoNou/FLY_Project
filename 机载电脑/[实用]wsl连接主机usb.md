参考来源：https://learn.microsoft.com/zh-cn/windows/wsl/connect-usb
# 1.安装 USBIPD-WIN 项目
- 在 WSL 上安装 USBIPD
  转到 [usbipd-win 项目的最新发布页面。](https://github.com/dorssel/usbipd-win/releases)
  选择 .msi 文件，该文件将下载安装程序。 （你可能会收到一条警告，要求你确认你信任此下载）。
  运行下载 usbipd-win_x.msi 安装程序文件。

  > 或者，也可以使用 [Windows 程序包管理器 （winget）](https://learn.microsoft.com/zh-cn/windows/package-manager/winget/)安装 usbipd-win 项目。 如果已安装 winget，只需使用以下命令： winget install --interactive --exact dorssel.usbipd-win 安装 usbipd-win。 如果省略 --interactive，则当需要安装驱动程序时，winget 可能会立即重启计算机。
# 2.连接 USB 设备
**在附加 USB 设备之前，请确保 WSL 命令行处于打开状态。**  
- 通过以 管理员 模式打开 PowerShell 并输入以下命令列出连接到 Windows 的所有 USB 设备。 列出设备后，选择并复制要附加到 WSL 的设备总线 ID。
  ```
  usbipd list
  ```
- 在附加 USB 设备之前，必须使用该命令`usbipd bind`来共享设备，从而允许它附加到 WSL。 这需要管理员权限。 选择要在 WSL 中使用的设备的总线 ID，然后运行以下命令。 运行命令后，请再次使用命令`usbipd list`验证设备是否共享。
  ```
  usbipd bind --busid 4-4
  ```
- 若要附加 USB 设备，请运行以下命令。 （不再需要使用提升的管理员提示。**确保 WSL 命令提示符处于打开状态**。 *请注意，只要 USB 设备连接到 WSL，Windows 将无法使用它*。 一旦连接到 WSL，任何在 WSL 2 上运行的发行版都可以使用该 USB 设备。 请确认设备是否已连接`usbipd list`。 在 WSL 提示符下，运行 lsusb 以验证 USB 设备是否已列出，并且可以使用 Linux 工具与之交互。
  ```
  usbipd attach --wsl --busid <busid>
  ```
- 在 WSL 中使用设备后，可以物理断开 USB 设备的连接，或者从 PowerShell 运行以下命令：
  ```
  usbipd detach --busid <busid>
  ```
