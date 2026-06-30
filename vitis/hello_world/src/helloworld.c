/******************************************************************************
* AD7606 BRAM Acquisition Test — Release Version
*
* Continuously reads 4-channel ADC data from dual-bank BRAM via AXI.
* PL writer (bram_sample_writer) fills alternating 4096-sample banks.
* This program polls bank_ready flags through AXI GPIO, reads each bank,
* prints header + tail samples, and releases the bank.
******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include "platform.h"
#include "xil_printf.h"
#include "xil_cache.h"

#define AXI_GPIO_BASE       0x41200000
#define GPIO_DATA_OFFSET    0x0000
#define GPIO_TRI_OFFSET     0x0004
#define GPIO2_DATA_OFFSET   0x0008
#define GPIO2_TRI_OFFSET    0x000C

#define AXI_BRAM_BASE       0x40000000

#define CTRL_CLEAR_BANK0    (1U << 0)
#define CTRL_CLEAR_BANK1    (1U << 1)
#define CTRL_CAPTURE_EN     (1U << 2)
#define CTRL_CLEAR_OVF      (1U << 3)
#define CTRL_SOFT_RESET     (1U << 4)
#define CTRL_CLEAR_OVERRUN  (1U << 5)
#define CTRL_CLEAR_UNDERRUN (1U << 6)

#define STAT_BANK0_READY    (1U << 0)
#define STAT_BANK1_READY    (1U << 1)
#define STAT_OVERFLOW       (1U << 2)
#define STAT_ACTIVE_BANK    (1U << 3)
#define STAT_WRITER_BUSY    (1U << 4)
#define STAT_CAPTURE_EN     (1U << 5)
#define STAT_ADC_OVERRUN    (1U << 6)
#define STAT_ADC_UNDERRUN   (1U << 7)
#define STAT_IDX_SHIFT      16

#define BANK_SAMPLE_COUNT   4096
#define MAX_BANKS           100

static volatile uint32_t *gpio_data  = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_DATA_OFFSET);
static volatile uint32_t *gpio_tri   = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_TRI_OFFSET);
static volatile uint32_t *gpio2_data = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO2_DATA_OFFSET);
static volatile uint32_t *gpio2_tri  = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO2_TRI_OFFSET);

static void delay_loop(volatile int n) { while (n--); }

static void pulse_control(uint32_t bit)
{
    uint32_t cur = *gpio2_data;
    *gpio2_data = cur | bit;
    delay_loop(5000);
    *gpio2_data = cur & ~bit;
}

int main()
{
    init_platform();

    // Disable D-cache: AXI peripheral space may be cached by default MMU
    Xil_DCacheDisable();

    print("\r\n========================================\r\n");
    print("  AD7606 BRAM Data Acquisition\r\n");
    print("  4 channels x 4096 samples/bank\r\n");
    print("========================================\r\n\r\n");

    *gpio_tri   = 0xFFFFFFFF;
    *gpio2_tri  = 0x00000000;

    // Soft reset the PL writer
    print("[INIT] Soft reset PL writer...\r\n");
    *gpio2_data = CTRL_SOFT_RESET;
    delay_loop(50000);
    *gpio2_data = 0;

    // Enable capture
    print("[INIT] Enabling capture (100 kSPS, 4ch)...\r\n\r\n");
    *gpio2_data = CTRL_CAPTURE_EN;

    uint32_t status;
    int bank_num = 0;

    while (bank_num < MAX_BANKS) {
        // Wait for any bank to become ready
        while (1) {
            status = *gpio_data;
            if ((status & STAT_BANK0_READY) || (status & STAT_BANK1_READY))
                break;
        }

        uint32_t idx = (status >> STAT_IDX_SHIFT) & 0xFFFF;

        // Error flags
        if (status & STAT_ADC_OVERRUN) {
            xil_printf("[!] ADC overrun (idx=%d)\r\n", idx);
            pulse_control(CTRL_CLEAR_OVERRUN);
        }
        if (status & STAT_ADC_UNDERRUN) {
            xil_printf("[!] ADC underrun (idx=%d)\r\n", idx);
            pulse_control(CTRL_CLEAR_UNDERRUN);
        }
        if (status & STAT_OVERFLOW) {
            xil_printf("[!] BRAM overflow — PS too slow\r\n");
            pulse_control(CTRL_CLEAR_OVF);
        }

        int is_bank1 = (status & STAT_BANK1_READY) ? 1 : 0;
        volatile uint32_t *buf = (volatile uint32_t *)(AXI_BRAM_BASE + (is_bank1 ? 0x8000 : 0x0000));

        // Read first 3 and last 1 sample
        uint32_t w0 = buf[0], w1 = buf[1];
        int16_t ch1_0 = (int16_t)(w0 & 0xFFFF);
        int16_t ch2_0 = (int16_t)(w0 >> 16);
        int16_t ch3_0 = (int16_t)(w1 & 0xFFFF);
        int16_t ch4_0 = (int16_t)(w1 >> 16);

        int last = (BANK_SAMPLE_COUNT - 1) * 2;
        uint32_t wl0 = buf[last], wl1 = buf[last + 1];
        int16_t ch1_n = (int16_t)(wl0 & 0xFFFF);
        int16_t ch2_n = (int16_t)(wl0 >> 16);
        int16_t ch3_n = (int16_t)(wl1 & 0xFFFF);
        int16_t ch4_n = (int16_t)(wl1 >> 16);

        xil_printf("[BANK%d] idx=%d | sample#0: %6d %6d %6d %6d | sample#%d: %6d %6d %6d %6d\r\n",
                   is_bank1, idx,
                   ch1_0, ch2_0, ch3_0, ch4_0,
                   BANK_SAMPLE_COUNT - 1,
                   ch1_n, ch2_n, ch3_n, ch4_n);

        // Release bank
        if (is_bank1)
            pulse_control(CTRL_CLEAR_BANK1);
        else
            pulse_control(CTRL_CLEAR_BANK0);

        bank_num++;
    }

    *gpio2_data = 0;
    print("\r\n[DONE] Capture stopped.\r\n");
    cleanup_platform();
    return 0;
}
