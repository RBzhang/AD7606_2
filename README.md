# AD7606 基于 Zynq-7020 的四通道同步采样系统

本工程在 Xilinx Zynq-7020 (xc7z020clg400-2) 上实现 AD7606/AD7606C 并行接口的 4 通道同步采样：
- **PL 端**：50 MHz 时钟驱动 ADC 并行读控制器，以 100 kSPS 速率采样并写入双缓冲 BRAM
- **PS 端**：AXI GPIO 中断驱动，bank 填满时触发 ISR → 读取 BRAM → 释放 bank

---

## 1. 系统架构

```
AD7606/AD7606C ──→ adc_sample4 ──→ bram_sample_writer ──→ BRAM Port B (PL writer)
                                         │           │
                                    pl_status   ps_control
                                         │           │
                                    AXI GPIO (IRQ → PS GIC)
                                    gpio_io_i  gpio2_io_o
                                         │           │
                                    BRAM Port A ← AXI BRAM Controller ← PS (0x40000000)

中断路径: bank_ready 0→1 → AXI GPIO ip2intc_irpt → PS IRQ_F2P[0] → GIC SPI 61 → ISR
```

**时钟域**：全部使用 PS FCLK_CLK0 (50 MHz)。

**复位**：PS `FCLK_RESET0_N` (active low) 同时复位 `adc_sample4` 和 `bram_sample_writer`。

---

## 2. 工程文件说明

### 2.1 PL 逻辑 (Verilog)

| 文件 | 作用 |
|---|---|
| `system_top.v` | 顶层模块，例化 `adc_sample4`、`bram_sample_writer`、`design_1_wrapper`，连接时钟/复位/中断线 |
| `adc_sample4.v` | ADC 并行读控制器 FSM：CONVST/BUSY/CS/RD 时序 + `adc_db` 输入同步 + 4 通道采样输出 + overrun/underrun 脉冲 |
| `bram_sample_writer.v` | BRAM 双缓冲写入器：ping-pong 两 bank 各 4096 样本；`sample_idx_stable` 避免 GPIO 中断风暴；AXI GPIO 状态/控制接口 |
| `sample_tick_gen.v` | 固定周期采样节拍发生器，每 `SYS_CLK_HZ / SAMPLE_RATE_HZ` 周期输出 1 clk 脉冲 |

### 2.2 约束

| 文件 | 作用 |
|---|---|
| `pin.xdc` | ADC 接口的 FPGA 引脚分配和 IOSTANDARD (LVCMOS33) |

### 2.3 Block Design

| IP | 配置 |
|---|---|
| `processing_system7_0` | Zynq-7000 PS, FCLK0=50MHz, UART0 on MIO14/15, DDR3, IRQ_F2P[0] enabled |
| `axi_bram_ctrl_0` | AXI BRAM Controller v4.1, SINGLE_PORT_BRAM=1, 64KB |
| `blk_mem_gen_0` | True Dual Port RAM, 16384×32bit (64KB), WRITE_FIRST |
| `axi_gpio_0` | Dual channel: CH1=32bit input (pl_status), CH2=32bit output (ps_control), **C_INTERRUPT_PRESENT=1** |
| `axi_smc` | AXI SmartConnect (1 slave: PS M_AXI_GP0 → 2 masters) |
| `rst_ps7_0_50M` | Processor System Reset, driven by FCLK_RESET0_N |

**地址映射**：
- BRAM: `0x40000000 ~ 0x4000FFFF` (64KB)
- AXI GPIO: `0x41200000 ~ 0x4120FFFF`

**中断连线**：`axi_gpio_0/ip2intc_irpt` → `processing_system7_0/IRQ_F2P[0]`

### 2.4 PS 软件

| 文件 | 作用 |
|---|---|
| `helloworld.c` | 中断驱动采集（UART 打印模式） |
| `adc_ethernet/src/helloworld.c` | 中断驱动采集（以太网 UDP 流式发送，无限发送，PC 端实时接收） |
| `platform.c` / `platform.h` | Zynq 平台初始化 (cache, UART) |
| `pc_receiver/ad7606_receiver.py` | PC 端 UDP 接收器，按通道分文件存储 |
| `pc_receiver/ad7606_plot.py` | PC 端数据绘图工具，读取分通道文件绘制波形 |

---

## 3. 外部接口信号 (FPGA ↔ ADC)

| FPGA 顶层信号 | 方向 | 必须连接 | 作用 |
|---|---|---|---|
| `adc_convst_a` | 输出 | 是 | 转换启动 A |
| `adc_convst_b` | 输出 | 是 | 转换启动 B (与 A 同步) |
| `adc_busy` | 输入 | 是 | 转换忙 (高=转换中) |
| `adc_cs_n` | 输出 | 是 | 片选 (低有效) |
| `adc_rd_n` | 输出 | 是 | 并行读脉冲 (低有效) |
| `adc_db[15:0]` | 输入 | 是 | 16 位并行数据总线 |
| `adc_reset` | 输出 | 是 | ADC 复位 (配置后自动产生脉冲) |
| `adc_os[2:0]` | 输出 | 否 | 过采样选择 (代码固定为 000=关闭) |

`adc_range`、`adc_stby`、`adc_ref_select` 等配置引脚由 PCB 硬件固定。

---

## 4. BRAM 数据布局

```
64KB (16384 × 32bit):
  Bank 0: 0x0000 ~ 0x7FFF (4096 samples × 8 bytes)
  Bank 1: 0x8000 ~ 0xFFFF (4096 samples × 8 bytes)

每个样本 8 字节:
  byte offset +0: word0 = {ch2[15:0], ch1[15:0]}
  byte offset +4: word1 = {ch4[15:0], ch3[15:0]}

ARM little-endian 读取: int16_t 数组顺序为 ch1, ch2, ch3, ch4
```

PC 端接收后按通道拆分存储为独立文件：
```
<output_dir>/
  ch1.bin   -- 通道 1 样本 (int16 LE)
  ch2.bin   -- 通道 2 样本 (int16 LE)
  ch3.bin   -- 通道 3 样本 (int16 LE)
  ch4.bin   -- 通道 4 样本 (int16 LE)
```

---

## 5. AXI GPIO 寄存器定义

### ps_control (PS → PL, GPIO2 output @ 0x41200008)

| Bit | 名称 | 类型 |
|-----|------|------|
| 0 | clear_bank0_ready | 上升沿 |
| 1 | clear_bank1_ready | 上升沿 |
| 2 | capture_enable | 电平 |
| 3 | clear_overflow | 上升沿 |
| 4 | soft_reset | 上升沿 |
| 5 | clear_adc_overrun | 上升沿 |
| 6 | clear_adc_underrun | 上升沿 |

### pl_status (PL → PS, GPIO input @ 0x41200000)

| Bit | 名称 |
|-----|------|
| 0 | bank0_ready |
| 1 | bank1_ready |
| 2 | overflow |
| 3 | active_bank |
| 4 | writer_busy |
| 5 | capture_enable (回读) |
| 6 | adc_overrun (锁存) |
| 7 | adc_underrun (锁存) |
| [31:16] | sample_idx_stable (仅在 bank 填满时更新，避免中断风暴) |

### 中断寄存器 (@ 0x41200000 + offset)

| 偏移 | 寄存器 | 配置值 | 说明 |
|------|--------|--------|------|
| 0x011C | GIER | `0x80000000` | Global Interrupt Enable |
| 0x0128 | IP IER | `0x00000001` | Channel 1 interrupt enable |
| 0x0120 | IP ISR | 读: 状态, 写: `0x00000001` 清除 | 中断状态 |

### GIC 配置

| 参数 | 值 |
|------|-----|
| 中断 ID | **61** (GIC SPI ID for IRQ_F2P[0]) |
| 触发类型 | Level High (`0x1`) |
| 优先级 | `0xA0` (10) |

---

## 6. 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `SYS_CLK_HZ` | 50_000_000 | 系统时钟 |
| `SAMPLE_RATE_HZ` | 100_000 | 采样率 |
| `ADC_TOTAL_CH` | 8 | 每帧读取通道数 (V5~V8 丢弃) |
| `RESET_CLKS` | 10 | ADC 复位脉宽 |
| `CONVST_HIGH_CLKS` | 2 | CONVST 脉宽 |
| `RD_LOW_CLKS` | 2 | RD 低电平时间 |
| `RD_HIGH_CLKS` | 2 | RD 高电平时间 |
| `BANK_SAMPLE_COUNT` | 4096 | 每 bank 样本数 (~41ms @100kSPS) |

---

## 7. 中断驱动操作流程

```text
PL 侧 (自动):
  Bank 0 填满 → bank0_ready 0→1 → AXI GPIO ip2intc_irpt 置位

PS 侧 ISR:
  读 gpio_data → g_bank_events |= bank_ready → 清 GPIO ISR → 返回

PS 侧 main:
  g_bank_events ≠ 0 → 读取 BRAM → 打印样本 → pulse_control 清除 bank_ready
  → 等待 pl_status 确认清除 → 等待下一个 bank
```

**时序 (100 kSPS)**：
```
t=0      : bank0 开始填充
t≈41ms   : bank0_ready=1 → ISR → main 读取 BRAM (~0.3ms) → 清除
t≈82ms   : bank1_ready=1 → ISR → main 读取 → 清除
...
```

---

## 8. 编译和测试

### PL (Vivado)

1. 打开 `AD7606_2.xpr`
2. BD 参数确认: BRAM Port B MEM_SIZE=65536, AXI GPIO interrupt enabled, FCLK_RESET0_N exported
3. Generate Block Design → 综合 → 实现 → 生成 bitstream

### PS (Vitis)

1. Export Hardware (include bitstream) → `system_top.xsa`
2. 创建/更新 application
3. 编译 `helloworld.c` → Run

### 预期输出 (UART 115200 baud)

```text
========================================
  AD7606 Interrupt-Driven Data Acquisition
========================================

[INIT] GIC + GPIO interrupts configured
[INIT] Soft reset PL writer...
[INIT] Enabling capture...

[BANK0] idx=4095 status=0x0FFF0029
[BANK0] sample#0:  10779  10737  10741  10775 | sample#4095:  10776  10736  10740  10774
[BANK1] idx=4095 status=0x0FFF0022
[BANK1] sample#0:  10777  10736  10742  10774 | sample#4095:  10777  10736  10740  10773
...

[DONE] Capture stopped.
```

---

## 9. 注意事项

### 9.1 BRAM Port B MEM_SIZE
必须为 **65536** (64KB)。如果使用默认值 8192 (8KB)，AXI BRAM Controller 的地址范围会被限制，导致 bank1 (0x8000+) 读取时触发 AXI DECERR → Data Abort。

### 9.2 `pl_status` 中断风暴防护
`bram_sample_writer.v` 中 `pl_status[31:16]` 使用 `sample_idx_stable` 而非 `sample_idx`。因为 `sample_idx` 每 10µs 变化一次，若直接连到 GPIO 输入会导致每秒 100,000 次中断。`sample_idx_stable` 仅在 bank 填满时更新。

### 9.3 GIC 中断 ID
`xparameters.h` 中的 `XPAR_FABRIC_AXI_GPIO_0_INTR = 29` 是 Vivado fabric 中断索引，**不是** GIC ID。Zynq-7000 PL IRQ_F2P[0] 的 GIC SPI ID 是 **61** (`32 + 29`)，`XScuGic_Connect` 需要传入 61。

### 9.4 GPIO 中断触发类型
AXI GPIO 的 `ip2intc_irpt` 是电平触发信号，必须在 GIC 中配置为 Level High (`0x1`)，否则 ISR 不会被调用。

### 9.5 D-Cache
裸机程序中需禁用 D-Cache (`Xil_DCacheDisable()`)，避免缓存 AXI GPIO 和 BRAM 的读取值导致读到 stale 数据。

### 9.6 1 MSPS 升级
如需提升至 1 MSPS：FCLK0→100MHz, `ADC_TOTAL_CH=4`, `SAMPLE_PERIOD_CLKS=100`。此时 bank 填满时间从 41ms 缩短至 4.1ms，PS 仍需约 0.3ms 读取 BRAM，CPU 利用率 <10%。需额外开启 PS ENET0 配合 lwIP UDP 发送。

---

## 10. 以太网 UDP 实时流式传输

数据通过 ENET0 (RGMII) → lwIP UDP → 网线 → PC 实时传输。

### 10.1 Block Design 配置

在 Vivado Block Design 的 Zynq PS7 中配置 ENET0：

| 配置项 | 值 |
|--------|-----|
| ENET0 | Enabled, MIO 16-27, RGMII, 1000 Mbps |
| MDIO | Enabled, MIO 52-53 |
| ENET0 clock | IO PLL, 125 MHz |

### 10.2 BSP 修改

#### 10.2.1 添加 lwIP 库

Vitis → `platform_ad7606` → Board Support Package Settings → 勾选 `lwip220` → 重新生成 BSP。

#### 10.2.2 强制千兆模式

由于 YT8521S PHY 不在 lwIP 已知 PHY 列表中（仅支持 Marvell/TI/Realtek/ADI），需绕过 auto-negotiation：

1. 修改 `lwipopts.h.in`：将 `@linkspeed@` 替换为 `#define CONFIG_LINKSPEED1000 1`
2. 修改 `xemacpsif_physpeed.c`：`phy_setup_emacps()` 函数直接返回 `1000`，跳过所有 MDIO 操作
3. 在 `helloworld.c` 中强制 `netif->flags |= NETIF_FLAG_LINK_UP`

> 修改 `lwipopts.h.in` 和 `xemacpsif_physpeed.c` 后需 Clean 并重新 Build Platform 使改动生效。

### 10.3 Vitis 应用程序

#### 创建步骤

1. 用 "Hello World" 模板创建 Application（如 `adc_ethernet`）
2. 修改 `CMakeLists.txt`：`collect(PROJECT_LIB_DEPS xilstandalone;xiltimer;lwip220)`
3. 修改 `lscript.ld`：`_HEAP_SIZE` 增大到 `0x20000` (128KB)
4. 用 `vitis/ethernet_stream/src/helloworld.c` 替换模板代码

#### 网络参数（可在源码中修改）

```c
#define LOCAL_IP_ADDR  "192.168.1.10"    // Zynq 板 IP
#define DEST_IP_ADDR   "192.168.1.100"   // PC IP
#define UDP_PORT       5001
```

#### Build & Run

1. Build Platform（含 lwIP BSP）
2. Build Application
3. Debug/Run → UART 终端查看初始化日志

#### UART 预期输出

```
========================================
  AD7606 Ethernet UDP Streaming
========================================

[INIT] interrupts configured
[INIT] lwIP starting...
Using default Speed from design
link speed: 1000 (forced)
[INIT] UDP on port 5001 ready
[INIT] setting forced link up...
[INIT] resetting PL...
[INIT] capture ON

[10 banks sent]
[20 banks sent]
...
```

### 10.4 硬件连接

```
Zynq 板 RJ45 ──[网线]── PC 以太网口
     192.168.1.10           192.168.1.100/24
```

### 10.5 PC 端：数据接收

Windows 11 需先设置以太网静态 IP：`192.168.1.100` / `255.255.255.0`。

接收到的数据按通道分别存入独立文件，每个文件为 int16 little-endian 格式：

```
<output_dir>/
  ch1.bin   -- 通道 1 原始采样值 (int16 LE)
  ch2.bin   -- 通道 2 原始采样值 (int16 LE)
  ch3.bin   -- 通道 3 原始采样值 (int16 LE)
  ch4.bin   -- 通道 4 原始采样值 (int16 LE)
```

```powershell
# 收 1 秒数据（约 24 个 bank, 每个通道 ~200KB）
python pc_receiver\ad7606_receiver.py --duration 1

# 收 5 秒数据
python pc_receiver\ad7606_receiver.py --duration 5

# 按 bank 数接收
python pc_receiver\ad7606_receiver.py --max-banks 50

# 指定输出目录
python pc_receiver\ad7606_receiver.py --duration 2 --output-dir capture_01
```

### 10.6 PC 端：数据绘图

依赖：`pip install numpy matplotlib`

```powershell
# 默认：读取 ad7606_data\ 目录下的 ch1.bin ~ ch4.bin，4 通道分列子图
python pc_receiver\ad7606_plot.py

# 指定目录
python pc_receiver\ad7606_plot.py --dir capture_01

# 4 通道叠加显示
python pc_receiver\ad7606_plot.py --dir ad7606_data --layout overlay

# 只看前 512 个样本
python pc_receiver\ad7606_plot.py --dir ad7606_data --samples 512

# 保存为图片（不弹窗）
python pc_receiver\ad7606_plot.py --dir ad7606_data --save plot.png

# 仅打印统计信息
python pc_receiver\ad7606_plot.py --dir ad7606_data --stats-only
```

### 10.7 修改摘要

| 文件 | 改动 |
|------|------|
| `design_1.bd` | ENET0 MDIO → MIO 52/53 |
| `bsp/.../lwipopts.h.in` | `CONFIG_LINKSPEED1000` 替代 AUTO_DETECT |
| `bsp/.../xemacpsif_physpeed.c` | `phy_setup_emacps()` 直接返回 1000 |
| `adc_ethernet/src/helloworld.c` | UDP 流式发送 + 强制 LINK_UP |
| `adc_ethernet/src/CMakeLists.txt` | 添加 `lwip220` 库 |
| `adc_ethernet/src/lscript.ld` | `_HEAP_SIZE = 0x20000` |
| `system_top.v` | `RD_LOW_CLKS` 2→5 (AD7606C t_ACC 余量) |
| `adc_sample4.v` | 两级 adc_db 流水线 + `RD_LOW_CLKS` 5→5 |
| `pc_receiver/ad7606_receiver.py` | UDP 接收，按通道分文件存储 (ch1.bin ~ ch4.bin) |
| `pc_receiver/ad7606_plot.py` | 读取分通道文件，4 通道绘图 + 统计 |
