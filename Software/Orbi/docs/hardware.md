# Orbi 硬件事实表（从原理图 / PCB / BOM 解出）

来源：`C:\Users\WilliTourt\Desktop\Orbi-SpaceMouse\Hardware`
- 主板原理图：`PCB/Imgs/MomBoard/Orbi MomBoard Schem.png`
- 底板原理图：`PCB/Imgs/BottomBoard/Orbi BottomBoard Schem.png`
- BOM：`BOMs/BOM_Orbi V1 Motherboard_*.xlsx`、`BOMs/BOM_Orbi V1 Bottom Board_*.xlsx`

## 1. 主控（主板 U4）

ESP32-S3-MINI-1-N8（8 MB flash，无 PSRAM），原生 USB（IO19/IO20）。

## 2. 主板引脚映射

| 网络名 | GPIO | 说明 |
|---|---|---|
| ESP_BOOT | IO0 | BOOT 键 SB1 → GND，C5 100 nF；下载模式用 |
| AXP_PWREN | IO5 | 到底板 AXP202 区，电源控制 |
| TMAG_SDA | IO6 | 传感器 I2C 总线 **A** 的 SDA |
| TMAG_SCL | IO7 | 传感器 I2C 总线 **A** 的 SCL |
| KEY2 | IO8 | 按键 2，10 K 上拉 3V3，按下拉低，1 µF 去抖 |
| TMAG1_INT | IO9 | 传感器 1 中断 |
| LED | IO10 | 板载状态 LED，**低电平点亮**（3V3 → 330 Ω → LED → 该网络） |
| AXP_IRQ | IO11 | AXP202 中断输入 |
| AXP_SCL | IO12 | PMU I2C 总线 **B** 的 SCL |
| AXP_SDA | IO13 | PMU I2C 总线 **B** 的 SDA |
| TMAG3_INT | IO14 | 传感器 3 中断 |
| USB_DN / USB_DP | IO18 / IO19 | 原生 USB（经板对板连接器到底板 USB-C） |
| TMAG2_INT | IO38 | 传感器 2 中断 |
| KEY1 | IO39 | 按键 1，10 K 上拉 3V3，按下拉低 |
| ESP_TX / ESP_RX | IO43 / IO44 | UART0 调试串口 |
| WS2812_DI | IO45 | 10 颗 WS2812B-MINI 数据线（首颗输入） |

## 3. 磁传感器（TMAG3001A2 × 3）

| 位号 | 总线 | SDA/SCL | ADDR 接法 | I2C 7 位地址 | 中断 |
|---|---|---|---|---|---|
| U1 | A | IO6 / IO7 | GND | **0x34** | IO9 |
| U2 | A | IO6 / IO7 | 3V3 | **0x35** | IO38 |
| U3 | B | IO13 / IO12 | 3V3 | **0x35** | IO14 |

- 器件版本 A2：默认量程 ±120 mT（`X_Y_Range=0`），273 LSB/mT ≈ 3.663 µT/LSB；置 1 变 ±240 mT。
- 数据寄存器 0x12~0x17 为 X/Y/Z 各 16 位二进制补码，大端（MSB 在前）。
- 地址规则（datasheet Table 6-3）：ADDR→GND=0x34，→VCC=0x35，→SDA=0x36，→SCL=0x37。
- 三条 INT 分别独占到 MCU，INTB 默认为**低有效**（`INTB_POL=0`）。
- U3 与 AXP202（同为 0x34）**共用总线 B**，靠地址区分，不冲突。

## 4. 电源 / 底板

- PMU：**AXP202**（QFN-48），USB-C 5 V 输入（USBLC6-2SC6 保护），锂电池 `803160 3.7 V`，滑动开关 `MST22D18G2`。
- 电池检测：`0.03 Ω 1%` 采样电阻（R2）+ `200 K`（R5）→ 充电电流/电量检测；NTC 接口（`ZX-SH1.0-2PWT`）。
- 板对板 12 pin 连接器：USB_DP、USB_DN、3V3 ×2、AXP_PWRDOK、AXP_IRQ、AXP_PWREN、AXP_SDA、AXP_SCL、GND ×2。
- 充电指示 LED（CHGLED，可选件）；`PSW`、`AXP_PWRDOK` 一路给 ESP 做电源状态输入。

## 5. 灯效

- 10 × WS2812B-MINI-V6 串成一条链，数据 IO45，末端 R11 2.2 K 到 GND。
- **注意：整条链用 3V3 供电**（BOM/原理图如此）。WS2812B-MINI 的 VDD 规格下限是 3.7 V 级别，
  3.3 V 下会出现亮度偏低、蓝色通道先衰减、个别灯珠抖动。固件侧默认把全局亮度压到 ≤ 25 %，
  并把 3V3 供电列为硬件评审项（见下方待确认清单）。

## 6. 磁铁 / 机械

- 底板 BOM 有 6 × `D6*3mm` 磁铁（无位号，属结构件）；摇杆件 `Orbi_Joystick` + 回中弹簧。
- 原理图引用 TI 应用笔记「X、Y、Z 霍尔元件的位置」（图 6-2），即三颗传感器围绕磁铁呈特定几何布置。

## 7. 需要硬件侧确认的点

1. WS2812 链 3.3 V 供电是否需要改 5 V（AXP202 的 boost/一路 LDO），或软件限亮。
2. `AXP_PWREN` 具体驱动的是 AXP202 的哪一路（DCDC/LDO 使能，还是 PWRON 时序），开关机时序需要定义。
3. 三颗磁铁与传感器的实际几何（间距、离轴量、磁化方向），决定 6DOF 解算矩阵。
4. 摇杆的自由度：只有倾斜两轴，还是含绕 Z 旋转 / 下压。
