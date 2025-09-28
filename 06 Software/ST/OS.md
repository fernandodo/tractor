

# 📌 安装 & 设置 mDNS（Avahi/Bonjour）+ 永久修改主机名

## 1. 安装 Avahi（mDNS 服务）

大多数 Linux 发行版用 Avahi 实现 Bonjour 协议。

* **Debian/Ubuntu**

```bash
sudo apt-get update
sudo apt-get install avahi-daemon avahi-utils
```

* **Yocto/OpenEmbedded**
  在 `local.conf` 或自定义 layer 里加：

```conf
IMAGE_INSTALL:append = " avahi-daemon avahi-utils "
```

* **其他嵌入式 BSP**
  确认包管理器里有 `avahi-daemon`，或者源码编译。

---

## 2. 启动 & 开机自启

```bash
sudo systemctl enable avahi-daemon
sudo systemctl start avahi-daemon
```

检查状态：

```bash
systemctl status avahi-daemon
```

---

## 3. 永久修改主机名

### 方法 A（推荐，基于 systemd）

```bash
sudo hostnamectl set-hostname myboard
```

同时检查 `/etc/hosts`，确保有：

```text
127.0.1.1   myboard
```

### 方法 B（手动修改文件）

1. 编辑 `/etc/hostname`

   ```text
   myboard
   ```
2. 编辑 `/etc/hosts`

   ```text
   127.0.1.1   myboard
   ```
3. 重启：

   ```bash
   sudo reboot
   ```

---

## 4. 验证

在同一局域网的其他电脑（Linux/macOS/Windows Bonjour 客户端）上运行：

```bash
ping myboard.local
```

---

## 5. 常见注意事项

* 如果网络里已有同名设备（`myboard.local` 被占用），Avahi 会自动加后缀避免冲突，例如：

  ```
  myboard-2.local
  myboard-e3-cb-78.local
  ```
* Windows 默认不支持 Bonjour，需要安装 **Apple Bonjour**（很多软件自带，比如 iTunes）。
* 在部分公司/学校 Wi-Fi，**mDNS 广播可能被禁用**，这种情况下 `.local` 无法解析。

---

✅ 总结：

1. 安装并启用 **avahi-daemon**。
2. 永久修改 `/etc/hostname` 和 `/etc/hosts`。
3. 确保 Avahi 在跑。
4. 就能用 **`<hostname>.local`** 来访问你的嵌入式设备。
