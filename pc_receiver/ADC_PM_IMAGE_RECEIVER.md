# AD7606 直采 DAC 图像解调脚本说明

本文档说明 `pc_receiver/ad7606_pm_image_receiver.py` 的设计目的、接收流程和使用方法。该脚本用于在没有 USRP 的情况下，直接用 AD7606 采集 DAC 输出电压，验证 Zynq 图像发送通路是否正确。

## 1. 应用场景

原无线相位调制链路为：

```text
Zynq 图像数据
    → PL 插入 Barker-13 帧同步头
    → BRAM LUT 映射为 14-bit DAC code
    → DAC 输出偏压
    → 变容二极管改变 RF 相位
    → USRP 提取相位并恢复图像
```

在没有 USRP 时，可以先绕过 RF 相位调制链路，直接验证 DAC 输出是否符合预期：

```text
Zynq 图像数据
    → PL 插入 Barker-13 帧同步头
    → BRAM LUT 映射为 14-bit DAC code
    → DAC 输出电压
    → AD7606 CH1 采样
    → Zynq/以太网 UDP 输出 ADC 数据
    → PC 端脚本检测 Barker 并恢复图像
```

因此，新的 PC 端接收逻辑不再做复数 IQ 相位解调，而是对 ADC 采集到的电压波形做：

```text
ADC 电压/码值流
    → Barker-13 相关同步
    → 用同步头估计低/高电平
    → 后续图像采样按电压区间归一化
    → 恢复 8-bit 灰度图像
```

## 2. 与原 AD7606 UDP 接收脚本的关系

原脚本 `pc_receiver/ad7606_receiver.py` 的功能是接收 UDP 数据、重组 AD7606 bank，并把 4 路通道分别保存为二进制文件。

新的 `ad7606_pm_image_receiver.py` 沿用相同 UDP 协议：

```text
bytes 0..3   : bank_id   uint32 little-endian
bytes 4..7   : frag_seq  uint32 little-endian
bytes 8..11  : start_idx uint32 little-endian，bank 内采样起点
bytes 12..   : int16 little-endian 交织 ADC 数据
```

每组逻辑采样包含 4 路 AD7606 数据：

```text
ch1, ch2, ch3, ch4, ch1, ch2, ch3, ch4, ...
```

每路为 16-bit 有符号整数。每个 bank 包含 4096 组逻辑采样，即每路 4096 个采样点。

新的脚本在完成 bank 重组后，默认只取第 1 路：

```text
AD7606 CH1 ← DAC 输出电压
```

如果需要改用其他通道，可以通过 `--channel` 参数指定。

## 3. 与 USRP 相位解调逻辑的对应关系

USRP 接收机 `pm_rx_usrp` 的核心流程是：

```text
USRP IQ
    → 数字下变频
    → Complex to MagPhase
    → 取相位
    → Barker-13 同步
    → 相位零点校正
    → 相位归一化到 0~255
```

AD7606 直采 DAC 时没有 IQ，也没有相位包裹问题，因此新脚本改为：

```text
AD7606 CH1 电压/码值
    → Barker-13 同步
    → 估计同步头低电平 sync_low
    → 估计同步头高电平 sync_high
    → 后续图像采样归一化：

      gray = (sample - sync_low) / (sync_high - sync_low) × 255
```

也就是说，USRP 版本恢复的是“相位 → 灰度”，AD7606 版本恢复的是“电压 → 灰度”。

## 4. Barker-13 同步设计

发送端每帧图像前插入 Barker-13 同步头：

```text
[+ + + + + - - + + - + - +]
```

对应二值符号为：

```text
+ → 0xFF
- → 0x00
```

每个 Barker chip 持续 32 个 DAC 符号周期。因此，在 DAC 采样率为 `dac_sample_rate`、AD7606 单通道采样率为 `adc_sample_rate` 时，每个 DAC 符号对应的 ADC 采样点数为：

```text
samples_per_symbol ≈ round(adc_sample_rate / dac_sample_rate)
```

每个 Barker chip 对应的 ADC 采样点数为：

```text
chip_samples = 32 × samples_per_symbol
```

完整同步头长度为：

```text
sync_len = 13 × 32 × samples_per_symbol
```

例如：

```text
AD7606 采样率 = 1 MS/s
DAC 输出采样率 = 100 kS/s
samples_per_symbol = 10
sync_len = 13 × 32 × 10 = 4160 个 ADC 采样点
```

脚本会在 CH1 采样流中滑动搜索这个同步模板。为增强鲁棒性，相关检测前会去除窗口内的直流分量和线性趋势，因此对 ADC 零偏和缓慢漂移不敏感。

## 5. 图像恢复流程

检测到 Barker 同步头后，脚本执行以下步骤。

### 5.1 估计同步头高低电平

同步头中 `+` chip 对应高电平，`-` chip 对应低电平。脚本分别取这些位置上的 ADC 采样，并用中位数估计：

```text
sync_high = median(samples at Barker + positions)
sync_low  = median(samples at Barker - positions)
```

使用中位数是为了减小偶发毛刺的影响。

### 5.2 读取图像 payload

每帧 payload 长度为：

```text
frame_sample_count = frame_width × frame_height × samples_per_symbol
```

对于 320×180 图像：

```text
frame_size = 320 × 180 = 57600 像素
```

如果 `samples_per_symbol = 10`，则每帧图像对应：

```text
57600 × 10 = 576000 个 ADC 采样点
```

### 5.3 每个像素做平均降采样

如果 ADC 采样率高于 DAC 输出采样率，一个 DAC 像素符号会对应多个 ADC 采样。脚本会把同一像素周期内的多个 ADC 点做平均：

```text
pixel_level[k] = mean(samples of symbol k)
```

这里是普通算术平均，因为采集的是电压，不是相位；USRP 相位解调中的圆周平均在这里不需要。

### 5.4 电压归一化为灰度

使用同步头估计出来的高低电平，把图像 payload 映射为 8-bit 灰度：

```text
gray_float = (pixel_level - sync_low) / (sync_high - sync_low)
gray_u8 = clip(round(gray_float × 255), 0, 255)
```

最后 reshape 成：

```text
height × width
```

并保存为 PNG。

## 6. 运行方式

典型命令如下：

```bash
python pc_receiver/ad7606_pm_image_receiver.py \
    --port 5001 \
    --target-ip 192.168.10.200 \
    --frame-width 320 \
    --frame-height 180 \
    --adc-sample-rate 1000000 \
    --dac-sample-rate 100000 \
    --channel 1 \
    --max-frames 10
```

参数含义：

| 参数 | 含义 |
|---|---|
| `--port` | PC 端监听的 UDP 端口，默认 5001 |
| `--target-ip` | 只接收指定 Zynq IP 的 UDP 包；设为空字符串可接收任意来源 |
| `--frame-width` | 图像宽度，当前视频建议 320 |
| `--frame-height` | 图像高度，当前视频建议 180 |
| `--adc-sample-rate` | AD7606 单通道采样率 |
| `--dac-sample-rate` | 发送端 DAC 输出采样率，即每个图像符号输出速率 |
| `--samples-per-symbol` | 手动指定每个 DAC 符号对应几个 ADC 点；默认 0 表示自动由采样率比值计算 |
| `--channel` | 使用哪一路 AD7606 输入，默认 1 |
| `--corr-thresh` | Barker 归一化相关阈值，默认 0.85 |
| `--min-sync-span` | Barker 高低电平最小跨度，单位为 ADC counts，默认 0 表示不限制 |
| `--max-frames` | 解调到指定帧数后停止，0 表示不限制 |
| `--output-dir` | 输出 PNG 图像目录，默认 `ad7606_pm_frames` |

输出文件示例：

```text
ad7606_pm_frames/
    frame_0000.png
    frame_0001.png
    frame_0002.png
    latest.png
```

其中 `latest.png` 会始终复制为最近解调出的一帧，便于实时查看。

## 7. 关键参数设置建议

### 7.1 采样率匹配

最重要的是 `adc_sample_rate / dac_sample_rate`。

如果发送端 DAC 输出采样率为 100 kS/s，AD7606 单通道采样率为 1 MS/s，则：

```text
samples_per_symbol = 10
```

如果 DAC 输出采样率改为 2.5 MS/s，而 AD7606 仍为 1 MS/s，则 ADC 采样率低于 DAC 符号速率，无法逐像素恢复图像。因此做 DAC 直采验证时，建议先把 DAC 输出采样率设置得低一些，使 AD7606 至少满足：

```text
adc_sample_rate >= dac_sample_rate
```

更推荐：

```text
adc_sample_rate = 5~10 × dac_sample_rate
```

这样每个像素周期有多个 ADC 点可以平均，恢复更稳定。

### 7.2 图像尺寸

如果发送端已经改成 320×180，则接收脚本必须使用：

```bash
--frame-width 320 --frame-height 180
```

否则脚本会按错误的 payload 长度截帧，导致图像错位或无法同步。

### 7.3 同步阈值

默认相关阈值为：

```bash
--corr-thresh 0.85
```

如果同步困难，可以临时降低到：

```bash
--corr-thresh 0.75
```

如果误检较多，可以提高到：

```bash
--corr-thresh 0.90
```

也可以设置高低电平最小跨度，例如：

```bash
--min-sync-span 500
```

这表示 Barker 同步头高低电平差至少要有约 500 个 ADC 码值。

## 8. 与 DAC LUT 的关系

该脚本默认使用同步头高低电平做线性归一化，因此适合验证：

```text
灰度值越大 → DAC 输出电压越高
```

如果发送端 BRAM LUT 是非线性校准表，那么 ADC 直采恢复出来的图像可能出现灰度非线性失真。这不一定表示发送通路错误，而是因为：

```text
灰度 → DAC code → DAC 电压
```

本身不是线性关系。

如果后续需要精确验证像素值，可以增加“反 LUT”功能：先根据发送端的 256 项 DAC LUT 建立 `ADC level → pixel value` 的查表关系，再恢复灰度。当前版本先使用同步头估计区间做线性恢复，便于快速验证帧同步和图像内容是否正确。

## 9. 当前验证重点

该接收脚本主要用于确认：

1. Zynq 是否按帧输出了 Barker-13 同步头；
2. DAC 输出是否能被 AD7606 正确采集；
3. UDP 采样链路是否连续；
4. 每帧图像 payload 长度是否正确；
5. 320×180 图像/视频是否能按顺序恢复；
6. DAC 输出电平区间是否足够稳定。

如果该脚本能够稳定保存出连续清晰的 PNG 帧，说明：

```text
PS 图像数据 → AXI DMA → PL 成帧 → LUT → DAC 输出 → AD7606 采集 → UDP 接收
```

这条发送与采样链路基本正确。

## 10. 与后续 USRP 验证的关系

AD7606 直采只能验证 DAC 输出电压是否正确，不能验证：

```text
变容二极管 C-V 曲线
RF 相位调制深度
无线链路相位噪声
USRP IQ 解调链路
```

因此该方法是 USRP 前的中间验证步骤。通过后，再接回完整链路：

```text
DAC 输出偏压 → 变容二极管 → RF 相位调制 → USRP 相位解调
```
