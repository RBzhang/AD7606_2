/******************************************************************************
 * AD7606  Ethernet UDP Streaming  (bare-metal, lwIP RAW API, NO_SYS_NO_TIMERS)
 *
 * Replaces main.c in a Vitis "Hello World" template project.
 *
 * BSP requirements: enable lwip220 library in domain config
 * CMakeLists.txt: collect(PROJECT_LIB_DEPS xilstandalone;xiltimer;lwip220)
 * lscript.ld: _HEAP_SIZE = 0x20000 (128KB min)
 *
 * UDP packet format (12-byte header + raw ADC samples):
 *   [0..3]   bank_id   (LE u32)
 *   [4..7]   frag_seq  (LE u32)
 *   [8..11]  sample_idx(LE u32)
 *   [12..N]  interleaved int16 ch1,ch2,ch3,ch4 ...
 ******************************************************************************/

#include <stdint.h>

#include "xil_printf.h"
#include "xil_cache.h"
#include "xil_exception.h"
#include "xscugic.h"

#include "lwip/init.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "netif/xadapter.h"
#include "sleep.h"

/* ========== Network ========== */
#define DEST_IP_ADDR   "192.168.1.100"
#define LOCAL_IP_ADDR  "192.168.1.10"
#define NET_MASK       "255.255.255.0"
#define GW_ADDR        "192.168.1.1"
#define UDP_PORT       5001

/* ========== AXI address map ========== */
#define AXI_GPIO_BASE  0x41200000U
#define AXI_BRAM_BASE  0x40000000U

#define GPIO_DATA       (AXI_GPIO_BASE + 0x0000U)
#define GPIO_TRI        (AXI_GPIO_BASE + 0x0004U)
#define GPIO2_DATA      (AXI_GPIO_BASE + 0x0008U)
#define GPIO2_TRI       (AXI_GPIO_BASE + 0x000CU)
#define GPIO_GIER       (AXI_GPIO_BASE + 0x011CU)
#define GPIO_IP_ISR     (AXI_GPIO_BASE + 0x0120U)
#define GPIO_IP_IER     (AXI_GPIO_BASE + 0x0128U)

/* ========== ps_control bits (PS -> PL) ========== */
#define CTRL_BANK0      (1U << 0)
#define CTRL_BANK1      (1U << 1)
#define CTRL_CAP_EN     (1U << 2)
#define CTRL_CLR_OVF    (1U << 3)
#define CTRL_SOFTRST    (1U << 4)
#define CTRL_CLR_OVR    (1U << 5)
#define CTRL_CLR_UNDR   (1U << 6)

/* ========== pl_status bits (PL -> PS) ========== */
#define STAT_BANK0      (1U << 0)
#define STAT_BANK1      (1U << 1)
#define STAT_OVF        (1U << 2)
#define STAT_ACTIVE     (1U << 3)
#define STAT_BUSY       (1U << 4)
#define STAT_CAP_EN     (1U << 5)
#define STAT_ADC_OVR    (1U << 6)
#define STAT_ADC_UNDR   (1U << 7)
#define STAT_IDX_SHIFT  16

#define BANK_SAMPLES    4096
#define BANK_BYTES      (BANK_SAMPLES * 8)    /* 32768 */
#define MAX_PAYLOAD     1460
#define SAMPS_PER_PKT   (MAX_PAYLOAD / 8)      /* 182 */

#define GPIO_INTR_ID    61

/* ========== Globals ========== */
static volatile uint32_t *gpio_data   = (volatile uint32_t *)GPIO_DATA;
static volatile uint32_t *gpio2_data  = (volatile uint32_t *)GPIO2_DATA;
static volatile uint32_t *gpio_gier   = (volatile uint32_t *)GPIO_GIER;
static volatile uint32_t *gpio_ip_isr = (volatile uint32_t *)GPIO_IP_ISR;
static volatile uint32_t *gpio_ip_ier = (volatile uint32_t *)GPIO_IP_IER;

static XScuGic             gic_inst;
static volatile uint32_t   g_events  = 0;
static struct netif        g_netif;
static struct udp_pcb     *g_udp;
static ip_addr_t           g_dest_ip;
static volatile int        g_link_up = 0;

/* ========== Helpers ========== */
static void busy_dly(volatile int n) { while (n--) __asm__ volatile("nop"); }

static void pulse(uint32_t bit)
{
    uint32_t cur = *gpio2_data;
    *gpio2_data = cur | bit;
    busy_dly(5000);
    *gpio2_data = cur & ~bit;
}

/* ========== GPIO ISR ========== */
static void gpio_isr(void *ref)
{
    g_events |= (*gpio_data & (STAT_BANK0 | STAT_BANK1));
    *gpio_ip_isr = 0x1;
}

static int init_intr(void)
{
    XScuGic_Config *cfg = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
    if (!cfg) return -1;
    if (XST_SUCCESS != XScuGic_CfgInitialize(&gic_inst, cfg, cfg->CpuBaseAddress))
        return -2;

    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
        (Xil_ExceptionHandler)XScuGic_InterruptHandler, &gic_inst);
    Xil_ExceptionEnable();

    if (XST_SUCCESS != XScuGic_Connect(&gic_inst, GPIO_INTR_ID,
        (Xil_ExceptionHandler)gpio_isr, NULL)) return -3;

    XScuGic_SetPriorityTriggerType(&gic_inst, GPIO_INTR_ID, 0xA0, 0x1);
    XScuGic_Enable(&gic_inst, GPIO_INTR_ID);

    *gpio_gier   = 0x80000000U;
    *gpio_ip_ier = 0x00000001U;
    return 0;
}


/* ========== UDP: send one bank ========== */
static void udp_send_bank(int is_bank1)
{
    if (!g_udp) return;

    uint32_t base  = is_bank1 ? 0x8000U : 0x0000U;
    uint32_t bid   = is_bank1 ? 1U : 0U;
    uint32_t fidx  = 0U;
    uint32_t left  = BANK_SAMPLES;

    while (left > 0) {
        uint32_t n     = (left > SAMPS_PER_PKT) ? SAMPS_PER_PKT : left;
        uint32_t dsz   = n * 8;
        uint32_t psz   = 12 + dsz;

        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, psz, PBUF_RAM);
        if (!p) { xil_printf("[ERR] pbuf_alloc fail\r\n"); break; }

        uint8_t *b = (uint8_t *)p->payload;
        *(uint32_t *)(b+0) = bid;
        *(uint32_t *)(b+4) = fidx;
        *(uint32_t *)(b+8) = BANK_SAMPLES - left;

        volatile uint32_t *bram =
            (volatile uint32_t *)(AXI_BRAM_BASE + base + (BANK_SAMPLES - left) * 8);
        uint16_t *dst = (uint16_t *)(b+12);
        for (uint32_t i=0; i<n; i++) {
            uint32_t w0 = bram[i*2];
            uint32_t w1 = bram[i*2+1];
            dst[i*4+0] = (uint16_t)(w0 & 0xFFFF);    /* ch1 */
            dst[i*4+1] = (uint16_t)(w0 >> 16);      /* ch2 */
            dst[i*4+2] = (uint16_t)(w1 & 0xFFFF);    /* ch3 */
            dst[i*4+3] = (uint16_t)(w1 >> 16);        /* ch4 */
        }

        udp_sendto(g_udp, p, &g_dest_ip, UDP_PORT);
        pbuf_free(p);

        left -= n;
        fidx++;
    }
}

/* ========== Main ========== */
int main(void)
{
    Xil_DCacheDisable();

    xil_printf("\r\n========================================\r\n");
    xil_printf("  AD7606 Ethernet UDP Streaming\r\n");
    xil_printf("========================================\r\n\r\n");

    /* GPIO direction */
    *(volatile uint32_t *)GPIO_TRI  = 0xFFFFFFFFU;
    *(volatile uint32_t *)GPIO2_TRI = 0x00000000U;

    /* GIC + GPIO interrupts */
    if (init_intr() != 0) {
        xil_printf("[ERR] interrupt init failed\r\n");
        return -1;
    }
    xil_printf("[INIT] interrupts configured\r\n");

    /* lwIP init */
    xil_printf("[INIT] lwIP starting...\r\n");
    lwip_init();

    ip_addr_t ip, nm, gw;
    IP4_ADDR(&ip, 192,168,1,10);
    IP4_ADDR(&nm, 255,255,255,0);
    IP4_ADDR(&gw, 192,168,1,1);
    IP4_ADDR(&g_dest_ip, 192,168,1,100);

    if (!xemac_add(&g_netif, &ip, &nm, &gw, NULL, XPAR_XEMACPS_0_BASEADDR)) {
        xil_printf("[ERR] xemac_add failed\r\n");
        return -1;
    }
    netif_set_default(&g_netif);
    netif_set_up(&g_netif);

    g_udp = udp_new();
    if (!g_udp) { xil_printf("[ERR] udp_new\r\n"); return -1; }
    udp_bind(g_udp, IP_ADDR_ANY, UDP_PORT);
    xil_printf("[INIT] UDP on port %d ready\r\n", UDP_PORT);

    /* set link up (MDIO not functional, PHY auto-negotiated via hardware) */
    xil_printf("[INIT] setting forced link up...\r\n");
    g_netif.flags |= NETIF_FLAG_LINK_UP;
    g_link_up = 1;

    /* soft reset PL writer */
    xil_printf("[INIT] resetting PL...\r\n");
    *gpio2_data = CTRL_SOFTRST;
    busy_dly(50000);
    *gpio2_data = 0;

    /* enable capture */
    xil_printf("[INIT] capture ON\r\n\r\n");
    *gpio2_data = CTRL_CAP_EN;

    int nbanks = 0;
    int max    = 0;   /* 0 = unlimited streaming */

    while (max == 0 || nbanks < max) {
        /* poll events (ISR + fallback) */
        int to = 0;
        while (!g_events) {
            xemacif_input(&g_netif);  /* feed lwIP rx/ARP */

            uint32_t s = *gpio_data;
            if (s & (STAT_BANK0 | STAT_BANK1)) {
                g_events |= (s & (STAT_BANK0 | STAT_BANK1));
                break;
            }
            if (++to > 20000000) {
                xil_printf("[DIAG] TIMEOUT st=0x%08lX\r\n", (unsigned long)s);
                goto done;
            }
        }

        uint32_t ev;
        __asm__ volatile("cpsid i");
        ev = g_events;
        g_events = 0;
        __asm__ volatile("cpsie i");

        if (ev & STAT_BANK0) {
            uint32_t st = *gpio_data;

            if (st & STAT_ADC_OVR) pulse(CTRL_CLR_OVR);
            if (st & STAT_OVF)     pulse(CTRL_CLR_OVF);

            udp_send_bank(0);

            pulse(CTRL_BANK0);
            while (*gpio_data & STAT_BANK0);
            busy_dly(1000);
            __asm__ volatile("cpsid i");
            g_events &= ~STAT_BANK0;
            __asm__ volatile("cpsie i");
            nbanks++;
        }

        if (ev & STAT_BANK1) {
            uint32_t st = *gpio_data;

            if (st & STAT_ADC_OVR) pulse(CTRL_CLR_OVR);
            if (st & STAT_OVF)     pulse(CTRL_CLR_OVF);

            udp_send_bank(1);

            pulse(CTRL_BANK1);
            while (*gpio_data & STAT_BANK1);
            busy_dly(1000);
            __asm__ volatile("cpsid i");
            g_events &= ~STAT_BANK1;
            __asm__ volatile("cpsie i");
            nbanks++;
        }

        if (nbanks % 10 == 0) {
            xil_printf("[%d banks sent]\r\n", nbanks);
        }
    }

done:
    *gpio2_data = 0;
    xil_printf("\r\n[DONE] %d banks streamed\r\n", nbanks);
    return 0;
}
