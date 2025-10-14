要让 **LC29H(BS)** 作为基站（Base Station）输出 RTK 改正数据，可以按照《**Quectel LC29H(BS)_GNSS Protocol Specification V1.0**》[[Quectel_LC29H(BS)_GNSS_Protocol_Specification_V1.0.pdf]]中的步骤配置：

---

### 🧭 一、设定基站工作模式

使用 **PQTMCFGSVIN** 命令选择基站工作模式：

#### 1️⃣ 自动测量（Survey-in）模式

模块自动测定并平均位置：

```
$PQTMCFGSVIN,W,1,<MinDur>,<3D_AccLimit>,0,0,0*CS
```

示例（测量最少 12 小时，精度 15 m）：

```
$PQTMCFGSVIN,W,1,43200,15,0,0,0*CS
```

#### 2️⃣ 固定坐标（Fixed）模式

输入已知的基站 ECEF 坐标：

```
$PQTMCFGSVIN,W,2,0,0.0,<ECEF_X>,<ECEF_Y>,<ECEF_Z>*CS
```

示例：

```
$PQTMCFGSVIN,W,2,0,0.0,-2472446.4619,4828304.1363,3343730.2653*34
```

设置完成后保存并重启：

```
$PQTMSAVEPAR*5A
```

---

### 🛰️ 二、启用 RTCM 改正数据输出

LC29H(BS) 支持 RTCM 10403.3 标准。

#### 1️⃣ 选择 RTCM 输出模式

```
$PAIR432,<Mode>*CS
```

|Mode|说明|
|---|---|
|-1|关闭输出|
|0|输出 RTCM-3 MSM4 消息|
|1|输出 RTCM-3 MSM7 消息（推荐）|

示例：

```
$PAIR432,1*22
```

#### 2️⃣ 启用固定天线参考点 (ARP) 消息 1005

```
$PAIR434,1*24
```

#### 3️⃣ 启用星历 (Ephemeris) 消息（1019、1020、1042 等）

```
$PAIR436,1*26
```

设置完成后，模块将连续输出 RTCM 消息到主串口或配置的 UART 接口。

---

### 📡 三、支持的 RTCM 消息类型

模块会输出下列消息用于 RTK 改正：

|消息类型|说明|
|---|---|
|1005|基站参考点 ARP|
|1019 / 1020 / 1042 / 1044 / 1046|GPS、GLONASS、BDS、QZSS、Galileo 星历|
|1074 / 1084 / 1094 / 1114 / 1124|MSM4 观测数据|
|1077 / 1087 / 1097 / 1117 / 1127|MSM7 观测数据|

---

### 🔧 四、查询与验证

- 查询当前 RTCM 输出模式：
    
    ```
    $PAIR433*3E
    ```
    
- 查询天线点输出状态：
    
    ```
    $PAIR435*38
    ```
    
- 查询星历输出状态：
    
    ```
    $PAIR437*3A
    ```
    

---

### ✅ 五、完整配置示例（MSM7 模式）

```text
$PQTMCFGSVIN,W,2,0,0.0,-2472446.4619,4828304.1363,3343730.2653*34
$PQTMSAVEPAR*5A
$PAIR432,1*22
$PAIR434,1*24
$PAIR436,1*26
```

执行完并重启模块后，LC29H(BS) 就会在 UART 输出 RTCM 1005、1019、1077 等改正数据，可供 LC29H-BA 等流动站接收使用，实现 RTK 厘米级定位。


> ([[Quectel_LC29H(BS)_GNSS_Protocol_Specification_V1.0.pdf#page=7&selection=173,0,178,11&color=yellow|Input/output, ASCII, proprietary]])
> Input/output, ASCII, proprietary

