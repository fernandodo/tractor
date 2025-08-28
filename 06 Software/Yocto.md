
# 📘 从零开始构建标准镜像（不含 ROS）

## 🔹 一次性准备（首次安装主机环境时做）

1. **安装依赖包（Ubuntu 24.04.3）**
    
    ```bash
    sudo apt update
    sudo apt install -y gawk wget git-core diffstat unzip texinfo gcc build-essential \
      chrpath socat cpio python3 python3-pip python3-pexpect xz-utils debianutils \
      iputils-ping python3-git python3-jinja2 libegl1-mesa libsdl1.2-dev \
      pylint3 xterm tar locales rsync file bc curl
    ```
    
2. **处理 AppArmor 限制（24.04 特有，每次系统更新可能需要重做一次）**
    
    ```bash
    sudo apparmor_parser -R /etc/apparmor.d/unprivileged_userns
    aa-status   # 可选：检查 profile 是否卸载
    ```
    
3. **设置 MMC 分区数与 locale**
    
    ```bash
    echo 'options mmc_block perdev_minors=16' | sudo tee /etc/modprobe.d/mmc_block.conf
    sudo update-locale LANG=en_US.UTF-8
    ```
    
4. **添加当前用户到必要组（重启后永久生效）**
    
    ```bash
    sudo adduser $USER dialout
    sudo adduser $USER tty
    sudo adduser $USER plugdev
    sudo adduser $USER disk
    ```
    
    > 退出并重新登录/重启，确保生效。
    
5. **安装 repo 工具**
    
    ```bash
    mkdir -p ~/bin
    curl -o ~/bin/repo https://storage.googleapis.com/git-repo-downloads/repo
    chmod a+x ~/bin/repo
    echo 'export PATH=$HOME/bin:$PATH' >> ~/.bashrc
    export PATH=$HOME/bin:$PATH
    ```
    
6. **初始化工程目录**
    
    ```bash
    mkdir -p ~/Projects/st/openstlinux && cd ~/Projects/st/openstlinux
    
    repo init -u https://github.com/STMicroelectronics/oe-manifest.git \
              -b refs/tags/openstlinux-6.6-yocto-scarthgap-mpu-v25.06.11
    repo sync
    ```
    

---

## 🔹 每次重启后必做

1. **进入 build 目录**
    
    ```bash
    cd ~/Projects/st/openstlinux/build-openstlinuxweston-stm32mp25-disco
    ```
    
2. **重新 source 环境**
    
    ```bash
    source ../layers/meta-st/scripts/envsetup.sh
    ```
    
3. **再次解除 AppArmor 限制（如果系统更新恢复了 profile）**
    
    ```bash
    sudo apparmor_parser -R /etc/apparmor.d/unprivileged_userns
    ```
    

---

## 🔹 第一次构建（配置 + 编译）

1. **确认 MACHINE 与 DISTRO**
    
    ```bash
    export MACHINE=stm32mp25-disco
    export DISTRO=openstlinux-weston
    ```
    
2. **接受 EULA（必须）**
    
    ```bash
    echo 'ACCEPT_EULA_'$MACHINE' = "1"' >> conf/local.conf
    ```
    
3. **可选：指定下载和缓存目录到 HDD**
    
    ```bash
    echo 'DL_DIR = "/mnt/hdd/yocto-cache/downloads"'   >> conf/local.conf
    echo 'SSTATE_DIR = "/mnt/hdd/yocto-cache/sstate-cache"' >> conf/local.conf
    ```
    
4. **开始编译标准 Weston 镜像**
    
    ```bash
    bitbake st-image-weston
    ```
    

---

## 🔹 后续使用

- 如果编译中断，重新执行：
    
    ```bash
    bitbake st-image-weston
    ```
    
    Yocto 会自动跳过已完成任务，从断点继续。
    
- 如果要控制并行度（比如 8 核机器）：  
    在 `conf/local.conf` 里加：
    
    ```conf
    BB_NUMBER_THREADS = "8"
    PARALLEL_MAKE = "-j8"
    ```
    
- 烧录时可用 **STM32CubeProgrammer** (DFU/USB-OTG) 或者写 SD 卡镜像（`wic`/`bmaptool`）。
    

---

👉 这样整理后：

- **一次性准备** = 装依赖、repo、配置 locale 等。
    
- **每次重启都要做** = `source envsetup.sh` + AppArmor 解除（如果更新后恢复了 profile）。
    
- **构建步骤** = 进入 build 目录 → 配置 MACHINE/DISTRO → bitbake。
    
# 