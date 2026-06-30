/******************************************************************************
* AD7606 BRAM Acquisition — Interrupt-Driven Version
*
* AXI GPIO interrupt fires when bank0_ready or bank1_ready asserts (0→1 edge).
* ISR sets a volatile flag; main loop reads BRAM, releases bank, prints.
*
* Hardware requirements:
*   AXI GPIO: C_INTERRUPT_PRESENT=1, ip2intc_irpt → PS IRQ_F2P[0]
*   PL: pl_status bits[31:16] (sample_idx) must be stable (see bram_sample_writer.v)
******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include "platform.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xil_exception.h"
#include "xscugic.h"

// --- Address map (from design_1 block design) ---
#define AXI_GPIO_BASE       0x41200000
#define GPIO_DATA_OFFSET    0x0000
#define GPIO_TRI_OFFSET     0x0004
#define GPIO2_DATA_OFFSET   0x0008
#define GPIO2_TRI_OFFSET    0x000C
#define GPIO_GIER_OFFSET    0x011C
#define GPIO_IP_ISR_OFFSET  0x0120
#define GPIO_IP_IER_OFFSET  0x0128

#define AXI_BRAM_BASE       0x40000000

// --- ps_control bits (PS → PL, GPIO2 output) ---
#define CTRL_CLEAR_BANK0    (1U << 0)
#define CTRL_CLEAR_BANK1    (1U << 1)
#define CTRL_CAPTURE_EN     (1U << 2)
#define CTRL_CLEAR_OVF      (1U << 3)
#define CTRL_SOFT_RESET     (1U << 4)
#define CTRL_CLEAR_OVERRUN  (1U << 5)
#define CTRL_CLEAR_UNDERRUN (1U << 6)

// --- pl_status bits (PL → PS, GPIO input) ---
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

// --- Interrupt ID ---
// PL IRQ_F2P[0] → GIC SPI ID 61 on Zynq-7000 (UG585 Table 7-4)
// xparameters.h XPAR_FABRIC_AXI_GPIO_0_INTR = 29 is the fabric index,
// XScuGic_Connect expects the GIC SPI ID.
#ifndef GPIO_INTR_ID
#define GPIO_INTR_ID  61
#endif

// --- Globals ---
static volatile uint32_t *gpio_data    = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_DATA_OFFSET);
static volatile uint32_t *gpio_tri     = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_TRI_OFFSET);
static volatile uint32_t *gpio2_data   = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO2_DATA_OFFSET);
static volatile uint32_t *gpio2_tri    = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO2_TRI_OFFSET);
static volatile uint32_t *gpio_gier    = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_GIER_OFFSET);
static volatile uint32_t *gpio_ip_isr  = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_IP_ISR_OFFSET);
static volatile uint32_t *gpio_ip_ier  = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_IP_IER_OFFSET);

static XScuGic gic_inst;

// ISR → main communication: bit0=bank0_ready, bit1=bank1_ready
static volatile uint32_t g_bank_events = 0;

// --- Utility ---
static void delay_loop(volatile int n) { while (n--); }

static void pulse_control(uint32_t bit)
{
    uint32_t cur = *gpio2_data;
    *gpio2_data = cur | bit;
    delay_loop(5000);
    *gpio2_data = cur & ~bit;
}

// --- GPIO ISR ---
static void gpio_isr(void *callback_ref)
{
    uint32_t status = *gpio_data;

    // Latch bank_ready events for main loop
    g_bank_events |= (status & (STAT_BANK0_READY | STAT_BANK1_READY));

    // Clear AXI GPIO interrupt (write 1 to ISR channel 1 bit)
    *gpio_ip_isr = 0x00000001;
}

// --- Sample printing (bank read from BRAM) ---
static void print_bank_head_tail(int is_bank1)
{
    volatile uint32_t *buf = (volatile uint32_t *)(AXI_BRAM_BASE + (is_bank1 ? 0x8000 : 0x0000));

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

    xil_printf("[BANK%d] sample#0: %6d %6d %6d %6d | sample#%d: %6d %6d %6d %6d\r\n",
               is_bank1,
               ch1_0, ch2_0, ch3_0, ch4_0,
               BANK_SAMPLE_COUNT - 1,
               ch1_n, ch2_n, ch3_n, ch4_n);
}

// --- GIC + GPIO interrupt setup ---
static int setup_interrupts(void)
{
    int status;
    XScuGic_Config *gic_cfg;

    gic_cfg = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
    if (gic_cfg == NULL) return -1;

    status = XScuGic_CfgInitialize(&gic_inst, gic_cfg, gic_cfg->CpuBaseAddress);
    if (status != XST_SUCCESS) return -2;

    // Register exception handlers (GIC + IRQ)
    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                 (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                                 &gic_inst);
    Xil_ExceptionEnable();

    // Connect AXI GPIO interrupt
    status = XScuGic_Connect(&gic_inst, GPIO_INTR_ID,
                             (Xil_ExceptionHandler)gpio_isr, NULL);
    if (status != XST_SUCCESS) return -3;

    // Set trigger type: level-high (AXI GPIO ip2intc_irpt is level-sensitive)
    XScuGic_SetPriorityTriggerType(&gic_inst, GPIO_INTR_ID, 0xA0, 0x1);

    XScuGic_Enable(&gic_inst, GPIO_INTR_ID);

    // Enable GPIO interrupt in IP: GIER bit31=1, IP IER ch1=1
    *gpio_gier   = 0x80000000;
    *gpio_ip_ier = 0x00000001;

    return 0;
}

// --- Main ---
int main()
{
    init_platform();
    Xil_DCacheDisable();

    print("\r\n========================================\r\n");
    print("  AD7606 Interrupt-Driven Data Acquisition\r\n");
    print("========================================\r\n\r\n");

    // GPIO direction
    *gpio_tri   = 0xFFFFFFFF;
    *gpio2_tri  = 0x00000000;

    // Set up GIC + GPIO interrupts
    if (setup_interrupts() != 0) {
        print("[ERROR] Interrupt setup failed\r\n");
        return -1;
    }
    print("[INIT] GIC + GPIO interrupts configured\r\n");

    // Soft reset PL writer
    print("[INIT] Soft reset PL writer...\r\n");
    *gpio2_data = CTRL_SOFT_RESET;
    delay_loop(50000);
    *gpio2_data = 0;

    // Enable capture
    print("[INIT] Enabling capture...\r\n\r\n");
    *gpio2_data = CTRL_CAPTURE_EN;

    int bank_count = 0;
    int max_banks  = 10;


    while (bank_count < max_banks) {
        // Wait for bank_ready (ISR sets g_bank_events, polling as fallback)
        int timeout = 0;
        while (g_bank_events == 0) {
            // Polling fallback: check PL status directly
            uint32_t raw = *gpio_data;
            if (raw & (STAT_BANK0_READY | STAT_BANK1_READY)) {
                g_bank_events |= (raw & (STAT_BANK0_READY | STAT_BANK1_READY));
                break;
            }
            if (++timeout > 20000000) {
                xil_printf("[DIAG] TIMEOUT raw=0x%08X\r\n", raw);
                goto done;
            }
        }

        // Atomically consume events
        uint32_t events;
        __asm__ volatile("cpsid i");
        events = g_bank_events;
        g_bank_events = 0;
        __asm__ volatile("cpsie i");

        // Process bank0
        if (events & STAT_BANK0_READY) {
            uint32_t status = *gpio_data;
            uint32_t idx = (status >> STAT_IDX_SHIFT) & 0xFFFF;
            xil_printf("[BANK0] idx=%d status=0x%08X\r\n", idx, status);

            if (status & STAT_ADC_OVERRUN)  pulse_control(CTRL_CLEAR_OVERRUN);
            if (status & STAT_OVERFLOW)     pulse_control(CTRL_CLEAR_OVF);

            print_bank_head_tail(0);
            pulse_control(CTRL_CLEAR_BANK0);

            // Wait until PL confirms bank0_ready is cleared,
            // then flush any stale ISR flag caused by 1→0 edge
            // racing with GPIO input synchronizer
            while (*gpio_data & STAT_BANK0_READY);
            delay_loop(1000);
            __asm__ volatile("cpsid i");
            g_bank_events &= ~STAT_BANK0_READY;
            __asm__ volatile("cpsie i");

            bank_count++;
        }

        // Process bank1
        if (events & STAT_BANK1_READY) {
            uint32_t status = *gpio_data;
            uint32_t idx = (status >> STAT_IDX_SHIFT) & 0xFFFF;
            xil_printf("[BANK1] idx=%d status=0x%08X\r\n", idx, status);

            if (status & STAT_ADC_OVERRUN)  pulse_control(CTRL_CLEAR_OVERRUN);
            if (status & STAT_OVERFLOW)     pulse_control(CTRL_CLEAR_OVF);

            print_bank_head_tail(1);
            pulse_control(CTRL_CLEAR_BANK1);

            while (*gpio_data & STAT_BANK1_READY);
            delay_loop(1000);
            __asm__ volatile("cpsid i");
            g_bank_events &= ~STAT_BANK1_READY;
            __asm__ volatile("cpsie i");

            bank_count++;
        }
    }

done:
    // Stop capture
    *gpio2_data = 0;
    print("\r\n[DONE] Capture stopped.\r\n");
    cleanup_platform();
    return 0;
}
