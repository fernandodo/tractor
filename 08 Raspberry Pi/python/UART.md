给你一份在 Raspberry Pi 上做 **UART 通信** 的实战速查。你前面正用 Python，这里也放上用 `pyserial` 的最小可跑示例。

# 1) 硬件&电气要点

- **电平**：树莓派 GPIO 是 **3.3 V TTL**，不可直接接 **5 V** 串口（会烧板）。
    
    - 和 PC 连接：用 **USB-TTL 转串口**（3.3 V 版，常见 CH340/CP2102/FT232）。
        
    - 和 RS-485/RS-232：需外接 **收发器/电平转换**（MAX3485/SP3485 或 MAX3232）。
        
- **默认引脚（40Pin 排针）**
    
    - **GPIO14 = TXD (Pin 8)**
        
    - **GPIO15 = RXD (Pin 10)**
        
    - **GND = Pin 6/9/14/20/25/30/34/39**
        
- 交叉接线：Pi **TXD → 对端 RXD**，Pi **RXD → 对端 TXD**，共地。
    

# 2) 打开并确认 UART 设备

1. 打开 `raspi-config`（或 GUI 的 Raspberry Pi Configuration）：
    
    ```
    sudo raspi-config
    Interface Options → Serial Port
    - Login shell over serial?  选 No （关闭串口登录）
    - Enable serial port hardware? 选 Yes
    ```
    
2. `config.txt`（/boot/firmware/config.txt 或 /boot/config.txt）里通常会有：
    
    ```
    enable_uart=1
    ```
    
    如果要把 **蓝牙占用的 PL011** 让出来（Pi3/4/5 通用做法）：
    
    ```
    dtoverlay=disable-bt
    ```
    
    然后：
    
    ```
    sudo systemctl disable hciuart
    sudo reboot
    ```
    
3. 查看设备名（系统会给出稳定别名）：
    
    ```
    ls -l /dev/serial*
    # /dev/serial0 -> 指向当前主 UART（可能是 /dev/ttyAMA0 或 /dev/ttyS0）
    # /dev/serial1 -> 次 UART（若存在）
    ```
    

> 小结：**优先用 `/dev/serial0`**，不用死记 `ttyAMA0/ttyS0`。
> 
> - `ttyAMA0` = 硬件 **PL011**（时钟独立，波特率稳定）。
>     
> - `ttyS0` = **mini-UART**（受核心频率影响，低负载建议也够用）。
>     

# 3) 权限与占用

- 把当前用户加入串口权限组：
    
    ```
    sudo usermod -aG dialout $USER
    # 重新登录或重启生效
    ```
    
- 确保没有 **serial getty** 或其它进程占用：
    
    ```
    sudo systemctl stop serial-getty@ttyAMA0.service
    sudo systemctl disable serial-getty@ttyAMA0.service
    sudo lsof /dev/serial0
    ```
    

# 4) 快速环回测试（可选）

把 **GPIO14(TX)** 和 **GPIO15(RX)** 暂时短接：

```bash
stty -F /dev/serial0 115200 -echo -icanon
cat /dev/serial0 &
echo "hello" > /dev/serial0
# 看到 cat 打印的 hello 即环通OK，Ctrl+C 结束
```

# 5) Python（pyserial）最小示例

安装：

```bash
sudo apt update
sudo apt install python3-pip
pip3 install pyserial
```

发送/接收（文本行协议举例）：

```python
import serial
import time

# 用稳定别名；若你已让出 PL011，serial0 多半指向 /dev/ttyAMA0
ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=0.2)

# 发送一行
ser.write(b'PING\n')

# 读一行（对端需按行回车）
line = ser.readline()   # 返回 bytes
print("RX:", line)

# 连续收（非阻塞轮询示例）
deadline = time.time() + 2.0
buf = bytearray()
while time.time() < deadline:
    n = ser.in_waiting
    if n:
        buf.extend(ser.read(n))
    time.sleep(0.01)

print("RX raw:", bytes(buf))
ser.close()
```

**二进制帧**（起止字节+长度+CRC）也完全可行，建议自定义一个 `read_exact(n)` 和环形缓冲解析状态机。

# 6) 常见波特率与稳定性

- 常用：9600/19200/38400/57600/115200/921600。
    
- 若使用 **mini-UART (ttyS0)**，高波特在高负载或变频时可能不稳；追求稳定选 **PL011 (ttyAMA0)** + `dtoverlay=disable-bt`。
    

# 7) RS-485 半双工（简要）

- 接 MAX3485/SP3485，**DE/RE**（发送使能）接一根 GPIO，例如 **GPIO18**。
    
- 发送前拉高 DE，发完适当延时后拉低（或用驱动自动 DE）。
    
- Python 伪代码：
    

```python
import serial, time
import RPi.GPIO as GPIO

DE_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(DE_PIN, GPIO.OUT, initial=GPIO.LOW)

ser = serial.Serial('/dev/serial0', 115200, timeout=0.1)

def rs485_send(pkt: bytes):
    GPIO.output(DE_PIN, GPIO.HIGH)
    time.sleep(0.001)       # 驱动建立时间，按芯片手册调
    ser.write(pkt)
    ser.flush()             # 确保已出 FIFO
    time.sleep(0.001)       # 发送尾延时，留给收发器把最后位推出去
    GPIO.output(DE_PIN, GPIO.LOW)

# 接收时保持 DE_PIN 低电平
```

# 8) 调试&故障排查清单

- `dmesg | grep tty` 看系统识别了哪些串口。
    
- 用 `minicom`/`screen` 与对端人肉互发：
    
    ```
    sudo apt install minicom
    minicom -D /dev/serial0 -b 115200
    ```
    
- 若读取全是空：检查 **GND** 是否共地；检查 **交叉接线**；确认对端波特率/校验位一致。
    
- 若乱码：波特率不匹配；mini-UART 受动态频率影响。
    
- 若偶发丢字节：适当增大 `timeout`、降低波特、或切换到 PL011；核对流控（通常关 `rtscts=False`）。
    
- 性能：Python 下建议使用 **批量读写**、**减少 per-byte 循环**，协议层用 **状态机** 而非正则逐字节匹配。
    

# 9) 进阶

- **多串口**：可通过 USB-TTL 扩展为 `/dev/ttyUSB0/1/…`，或使用帽子板（多路 UART）。
    
- **系统服务自启**：用 `systemd` 启动你的 Python 串口服务（记得 `Restart=on-failure`）。
    
- **C/C++**：需要极致性能时，建议迁移到 C/C++（termios）或用 C 扩展模块。
    
