## 新增或替换一个更大的 swap 文件

1. **查看当前 swap 情况**  
   ```bash
   sudo swapon --show
   free -h
   ```
2. **关闭所有 swap**  
   ```bash
   sudo swapoff -a
   ```
3. **创建一个新的 swap 文件**  
   - 使用 `fallocate`（快速）：
     ```bash
     sudo fallocate -l 4G /swapfile
     ```
   - 如果 `fallocate` 不可用，使用 `dd`：
     ```bash
     sudo dd if=/dev/zero of=/swapfile bs=1M count=4096 status=progress
     ```
4. **设置正确权限**  
   ```bash
   sudo chmod 600 /swapfile
   ```
5. **格式化并启用 swap**  
   ```bash
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```
6. **验证生效**  
   ```bash
   swapon --show
   free -h
   ```
7. **开机自动挂载**  
   在 `/etc/fstab` 末尾添加：  
   ```text
   /swapfile none swap sw 0 0
   ```
8. **（可选）调整 swappiness**  
   系统默认 `vm.swappiness=60`，可调小：  
   ```bash
   echo 'vm.swappiness=10' | sudo tee /etc/sysctl.d/99-swappiness.conf
   sudo sysctl --system
   ```
