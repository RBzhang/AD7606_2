# AD7606 基于 Zynq-7020 的四通道同步采样系统

本工程在 Xilinx Zynq-7020 (xc7z020clg400-2) 上实现 AD7606/AD7606C 并行接口的 4 通道同步采样：
- **PL 端**：50 MHz 时钟驱动 ADC 并行读控制器 (`adc_sample4`)，以 100 kSPS 速率采样并写入双缓冲 BRAM
- **PS 端**：通过 AXI BRAM Controller 和 AXI GPIO 控制 ping-pong 缓冲切换，读取采样数据

---

## 1. 系统架构

```
AD7606/AD7606C ──→ adc_sample4 ──→ bram_sample_writer ──→ BRAM Port B
                                         │                      │
                                    AXI GPIO              BRAM Port A (64KB True Dual Port)
                                    (ps_control/           ──→ AXI BRAM Controller
                                     pl_status)                ──→ Zynq PS (0x40000000)

Zynq PS ──→ Ethernet / UART (测试)
```

**时钟域**：全部使用 PS FCLK_CLK0 (50 MHz)，PL 端无额外 PLL/MMCM。

**复位**：PS `FCLK_RESET0_N` (active low) 同时复位 `adc_sample4` 和 `bram_sample_writer` 的寄存器状态。

---

## 2. 工程文件说明

### 2.1 PL 逻辑 (Verilog)

| 文件 | 作用 |
|---|---|
| `system_top.v` | 真正的顶层模块，例化 `adc_sample4`、`bram_sample_writer`、`design_1_wrapper` |
| `adc_sample4.v` | ADC 并行读控制器 FSM：CONVST/BUSY/CS/RD 时序 + 4 通道采样输出 |
| `bram_sample_writer.v` | BRAM 双缓冲写入器：ping-pong 两 bank 各 4096 样本，AXI GPIO 状态/控制 |
| `sample_tick_gen.v` | 固定周期采样节拍发生器，每 `SYS_CLK_HZ / SAMPLE_RATE_HZ` 周期输出 1 clk 脉冲 |

### 2.2 约束

| 文件 | 作用 |
|---|---|
| `pin.xdc` | ADC 接口的 FPGA 引脚分配和 IOSTANDARD (LVCMOS33) |

### 2.3 Block Design

| 文件 | 作用 |
|---|---|
| `design_1.bd` | Vivado Block Design：Zynq PS + AXI SmartConnect + AXI BRAM Controller + BRAM (64KB) + AXI GPIO + Processor System Reset |

| IP | 配置 |
|---|---|
| `processing_system7_0` | Zynq-7000 PS, FCLK0=50MHz, UART0 on MIO14/15, DDR3 |
| `axi_bram_ctrl_0` | AXI BRAM Controller v4.1, SINGLE_PORT_BRAM=1, 64KB |
| `blk_mem_gen_0` | True Dual Port RAM, 16384×32bit (64KB), WRITE_FIRST, byte-write-enable |
| `axi_gpio_0` | Dual channel: GPIO=32bit input (pl_status), GPIO2=32bit output (ps_control) |
| `axi_smc` | AXI SmartConnect (1 slave: PS M_AXI_GP0 → 2 masters: BRAM ctrl + GPIO) |
| `rst_ps7_0_50M` | Processor System Reset, driven by FCLK_RESET0_N |

**地址映射**：
- BRAM: `0x40000000 ~ 0x4000FFFF` (64KB)
- AXI GPIO: `0x41200000 ~ 0x4120FFFF`

### 2.4 PS 软件

| 文件 | 作用 |
|---|---|
| `helloworld.c` | 裸机测试程序：轮询 bank_ready → 读取样本 → 打印 → 释放 bank |
| `platform.c` / `platform.h` | Zynq 平台初始化 (cache, UART) |
| `CMakeLists.txt` | Vitis CMake 构建文件 |

---

## 3. 外部接口信号 (FPGA ↔ ADC)

| FPGA 顶层信号 | FPGA 方向 | 必须连接 | 作用 |
|---|---|---|---|
| `adc_convst_a` | 输出 | 是 | 转换启动 A (上升沿采样保持) |
| `adc_convst_b` | 输出 | 是 | 转换启动 B (与 A 同步，实现全通道同时采样) |
| `adc_busy` | 输入 | 是 | 转换忙信号 (高=转换中, 低=数据可读) |
| `adc_cs_n` | 输出 | 是 | 片选 (低有效) |
| `adc_rd_n` | 输出 | 是 | 并行读脉冲 (低有效) |
| `adc_db[15:0]` | 输入 | 是 | 16 位并行数据总线 |
| `adc_reset` | 输出 | 通常需要 | ADC 复位 (配置后自动产生 RESET_CLKS 周期高脉冲) |
| `adc_os[2:0]` | 输出 | 不一定 | 过采样选择 (代码固定为 000=关闭) |

`adc_range`、`adc_stby`、`adc_ref_select` 等配置引脚未引出 FPGA 端口，由 PCB 硬件固定电平。

---

## 4. BRAM 数据布局

```
BRAM 64KB (16384 × 32bit):
  Bank 0: 0x0000 ~ 0x7FFF (4096 samples × 8 bytes)
  Bank 1: 0x8000 ~ 0xFFFF (4096 samples × 8 bytes)

每个样本 8 字节 (2 × 32bit word):
  byte offset +0: word0 = {ch2[15:0], ch1[15:0]}
  byte offset +4: word1 = {ch4[15:0], ch3[15:0]}
```

ARM little-endian 读取时，`int16_t` 数组顺序为 `ch1, ch2, ch3, ch4`。

---

## 5. AXI GPIO 寄存器定义

### ps_control (PS → PL, GPIO2 output)

| Bit | 名称 | 类型 |
|-----|------|------|
| 0 | clear_bank0_ready | 上升沿有效 |
| 1 | clear_bank1_ready | 上升沿有效 |
| 2 | capture_enable | 电平有效 |
| 3 | clear_overflow | 上升沿有效 |
| 4 | soft_reset | 上升沿有效 |
| 5 | clear_adc_overrun | 上升沿有效 |
| 6 | clear_adc_underrun | 上升沿有效 |

### pl_status (PL → PS, GPIO input)

| Bit | 名称 |
|-----|------|
| 0 | bank0_ready |
| 1 | bank1_ready |
| 2 | overflow (两 bank 都满时丢弃新样本) |
| 3 | active_bank |
| 4 | writer_busy |
| 5 | capture_enable (回读) |
| 6 | adc_overrun (锁存, sticky) |
| 7 | adc_underrun (锁存, sticky) |
| [31:16] | sample_idx (当前 active bank 内索引) |

---

## 6. 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `SYS_CLK_HZ` | 50_000_000 | 系统时钟 (PS FCLK_CLK0) |
| `SAMPLE_RATE_HZ` | 100_000 | 采样率 (100 kSPS) |
| `ADC_TOTAL_CH` | 8 | 每帧读取通道数 (V5~V8 丢弃) |
| `RESET_CLKS` | 10 | ADC 复位脉宽 (200ns @50MHz) |
| `CONVST_HIGH_CLKS` | 2 | CONVST 脉宽 (40ns) |
| `RD_LOW_CLKS` | 2 | RD 低电平时间 (40ns) |
| `RD_HIGH_CLKS` | 2 | RD 高电平时间 (40ns) |
| `BANK_SAMPLE_COUNT` | 4096 | 每 bank 样本数 (约 41ms @100kSPS) |

---

## 7. ps_control 操作流程

```text
1. 上电后 PS 写 CTRL_SOFT_RESET 复位 writer 状态
2. 写 CTRL_CAPTURE_EN = 1 开始采集
3. PL 填满 Bank 0 → bank0_ready = 1, 自动切换至 Bank 1
4. PS 检测 bank0_ready → 读 BRAM (0x40000000) → 写 CTRL_CLEAR_BANK0
5. PL 填满 Bank 1 → bank1_ready = 1, 自动切换回 Bank 0
6. PS 检测 bank1_ready → 读 BRAM (0x40008000) → 写 CTRL_CLEAR_BANK1
7. 重复 3-6
```

---

## 8. 编译和测试

### PL 端

1. Vivado 打开 `AD7606_2.xpr`
2. 如需修改 BD，编辑 `design_1.bd` → Validate → Generate Block Design
3. 综合 + 实现 + 生成 bitstream

### PS 端

1. File → Export → Export Hardware (include bitstream) → `system_top.xsa`
2. Vitis 中创建/更新 platform 和 application
3. 将 `helloworld.c` 复制到 `src/`
4. 编译 → Run (或生成 BOOT.BIN 烧录到 SD/QSPI)

### 预期输出 (UART 115200 baud)

```text
========================================
  AD7606 BRAM Data Acquisition
  4 channels x 4096 samples/bank
========================================

[INIT] Soft reset PL writer...
[INIT] Enabling capture (100 kSPS, 4ch)...

[BANK0] idx=0 | sample#0:  10737  10742  10774  10720 | sample#4095: ...
[BANK1] idx=82 | sample#0:  10781  10740  ... | sample#4095: ...
...
```

ADC 编码范围：-32768 ~ +32767 对应 ±10V (LSB ≈ 0.305mV)。无输入信号时编码应在 0 附近 ±10 LSB。

---

## 9. 注意事项

1. **BRAM Port B MEM_SIZE 必须为 65536**：在 Vivado Block Design 中，BRAM 外部接口 `BRAM_PORTB_0` 的 `MEM_SIZE` 参数应为 65536 (64KB)，否则 AXI BRAM Controller 会被限制为较小的地址范围（默认为 8192 = 8KB），导致 bank1 (0x8000+) 不可读。
2. **FCLK_RESET0_N 必须引出**：BD wrapper 需要导出 `FCLK_RESET0_N` 端口，`system_top.v` 将其连接到 `adc_sample4` 和 `bram_sample_writer` 的 `rst_n`。
3. **AXI GPIO 的 Dual Channel 配置**：GPIO → all inputs (pl_status), GPIO2 → all outputs (ps_control)。
4. **D-Cache**：裸机程序中需禁用 D-Cache 或使用 `Xil_DCacheInvalidateRange()` 避免缓存 AXI 外设读取。
