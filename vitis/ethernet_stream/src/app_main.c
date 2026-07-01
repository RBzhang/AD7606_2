/******************************************************************************
 * AD7606 Ethernet UDP Streaming
 *
 * Replaces lwIP Echo Server with ADC data streaming over UDP.
 * Use Vitis "lwIP Echo Server" template to create the project first,
 * then replace only the main C file with this code.
 *
 * Hardware: Zynq-7020, ENET0 RGMII (MIO16-27)
 *
 * Packet format (per UDP fragment):
 *   [0..3]   bank_id   (LE u32): 0 = bank0, 1 = bank1
 *   [4..7]   frag_seq  (LE u32): fragment sequence, 0-based
 *   [8..11]  start_idx (LE u32): sample index offset in bank
 *   [12..N]  raw 16-bit ADC data (bank data slice)
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "xil_printf.h"
#include "xil_cache.h"
#include "xil_exception.h"
#include "xscugic.h"
#include "sleep.h"

#include "lwip/init.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "netif/xadapter.h"
#include "lwip/inet.h"

/* ---- Network config ---- */
#define DEST_IP_ADDR   "192.168.1.100"
#define LOCAL_IP_ADDR  "192.168.1.10"
#define NET_MASK       "255.255.255.0"
#define GW_ADDR        "192.168.1.1"
#define UDP_PORT       5001

/* ---- AXI address map ---- */
#define AXI_GPIO_BASE       0x41200000
#define AXI_BRAM_BASE       0x40000000

/* Register offsets for AXI GPIO v2.0 */
#define GPIO_DATA_OFFSET    0x0000
#define GPIO_TRI_OFFSET     0x0004
#define GPIO2_DATA_OFFSET   0x0008
#define GPIO2_TRI_OFFSET    0x000C
#define GPIO_GIER_OFFSET    0x011C
#define GPIO_IP_ISR_OFFSET  0x0120
#define GPIO_IP_IER_OFFSET  0x0128

/* ---- ps_control bits (PS -> PL, GPIO2 output) ---- */
#define CTRL_CLEAR_BANK0    (1U << 0)
#define CTRL_CLEAR_BANK1    (1U << 1)
#define CTRL_CAPTURE_EN     (1U << 2)
#define CTRL_CLEAR_OVF      (1U << 3)
#define CTRL_SOFT_RESET     (1U << 4)
#define CTRL_CLEAR_OVERRUN  (1U << 5)
#define CTRL_CLEAR_UNDERRUN (1U << 6)

/* ---- pl_status bits (PL -> PS, GPIO input) ---- */
#define STAT_BANK0_READY    (1U << 0)
#define STAT_BANK1_READY    (1U << 1)
#define STAT_OVERFLOW       (1U << 2)
#define STAT_ACTIVE_BANK    (1U << 3)
#define STAT_WRITER_BUSY    (1U << 4)
#define STAT_CAPTURE_EN     (1U << 5)
#define STAT_ADC_OVERRUN    (1U << 6)
#define STAT_ADC_UNDERRUN   (1U << 7)
#define STAT_IDX_SHIFT      16

/* ---- Data size ---- */
#define BANK_SAMPLE_COUNT   4096
#define MAX_UDP_PAYLOAD     1460
#define SAMPLES_PER_PACKET  (MAX_UDP_PAYLOAD / 8)       /* 182 samples */
#define FRAGS_PER_BANK      (BANK_SAMPLE_COUNT / SAMPLES_PER_PACKET + \
                             (BANK_SAMPLE_COUNT % SAMPLES_PER_PACKET ? 1 : 0))

/* ---- Interrupt ID ---- */
#define GPIO_INTR_ID  61

/* ---- GPIO registers ---- */
static volatile uint32_t *gpio_data   = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_DATA_OFFSET);
static volatile uint32_t *gpio_tri    = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_TRI_OFFSET);
static volatile uint32_t *gpio2_data  = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO2_DATA_OFFSET);
static volatile uint32_t *gpio2_tri   = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO2_TRI_OFFSET);
static volatile uint32_t *gpio_gier   = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_GIER_OFFSET);
static volatile uint32_t *gpio_ip_isr = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_IP_ISR_OFFSET);
static volatile uint32_t *gpio_ip_ier = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_IP_IER_OFFSET);

static XScuGic gic_inst;
static volatile uint32_t g_bank_events = 0;

/* ---- lwIP objects ---- */
static struct netif server_netif;
static struct udp_pcb *udp_pcb;
static ip_addr_t dest_ip;
static volatile int link_ready = 0;

/* ---- Utilities ---- */
static void short_delay(volatile int n) { while (n--) { __asm__ volatile("nop"); } }

static void pulse_control(uint32_t bit)
{
    uint32_t cur = *gpio2_data;
    *gpio2_data = cur | bit;
    short_delay(5000);
    *gpio2_data = cur & ~bit;
}

/* ---- GPIO ISR ---- */
static void gpio_isr(void *callback_ref)
{
    uint32_t status = *gpio_data;
    g_bank_events |= (status & (STAT_BANK0_READY | STAT_BANK1_READY));
    *gpio_ip_isr = 0x00000001;
}

/* ---- UDP: send one full bank as fragmented packets ---- */
static void send_bank_over_udp(int is_bank1)
{
    if (udp_pcb == NULL) {
        xil_printf("[ERR] UDP PCB is NULL, dropping bank\r\n");
        return;
    }

    uint32_t bank_base = is_bank1 ? 0x8000 : 0x0000;
    uint32_t bank_id   = is_bank1 ? 1 : 0;
    uint32_t remaining  = BANK_SAMPLE_COUNT;
    uint32_t sample_idx = 0;
    uint32_t frag_seq   = 0;

    while (remaining > 0) {
        uint32_t samples_now = (remaining > SAMPLES_PER_PACKET)
                               ? SAMPLES_PER_PACKET : remaining;
        uint32_t data_bytes  = samples_now * 8;
        uint32_t pkt_size    = 12 + data_bytes;

        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, pkt_size, PBUF_RAM);
        if (p == NULL) {
            xil_printf("[ERR] pbuf_alloc failed, frag=%lu\r\n", frag_seq);
            break;
        }

        uint8_t *buf = (uint8_t *)p->payload;

        /* 12-byte header */
        *(uint32_t *)(buf + 0) = bank_id;
        *(uint32_t *)(buf + 4) = frag_seq;
        *(uint32_t *)(buf + 8) = sample_idx;

        /* Copy BRAM data slice */
        volatile uint32_t *bram = (volatile uint32_t *)(AXI_BRAM_BASE + bank_base + sample_idx * 8);
        uint16_t *dst = (uint16_t *)(buf + 12);

        for (uint32_t i = 0; i < samples_now; i++) {
            uint32_t idx = i * 2;
            uint32_t w0  = bram[idx];
            uint32_t w1  = bram[idx + 1];
            dst[idx + 0] = (uint16_t)(w0 & 0xFFFF);   /* ch1 */
            dst[idx + 1] = (uint16_t)(w0 >> 16);       /* ch2 */
            dst[idx + 2] = (uint16_t)(w1 & 0xFFFF);   /* ch3 */
            dst[idx + 3] = (uint16_t)(w1 >> 16);       /* ch4 */
        }

        err_t err = udp_sendto(udp_pcb, p, &dest_ip, UDP_PORT);
        pbuf_free(p);

        if (err != ERR_OK) {
            xil_printf("[ERR] sendto failed err=%d frag=%lu\r\n", err, frag_seq);
        }

        sample_idx += samples_now;
        remaining   -= samples_now;
        frag_seq++;
    }
}

/* ---- GIC interrupt init ---- */
static int setup_interrupts(void)
{
    XScuGic_Config *gic_cfg;
    int status;

    gic_cfg = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
    if (gic_cfg == NULL) return -1;

    status = XScuGic_CfgInitialize(&gic_inst, gic_cfg, gic_cfg->CpuBaseAddress);
    if (status != XST_SUCCESS) return -2;

    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(
        XIL_EXCEPTION_ID_INT,
        (Xil_ExceptionHandler)XScuGic_InterruptHandler,
        &gic_inst);
    Xil_ExceptionEnable();

    status = XScuGic_Connect(&gic_inst, GPIO_INTR_ID,
                             (Xil_ExceptionHandler)gpio_isr, NULL);
    if (status != XST_SUCCESS) return -3;

    XScuGic_SetPriorityTriggerType(&gic_inst, GPIO_INTR_ID, 0xA0, 0x1);
    XScuGic_Enable(&gic_inst, GPIO_INTR_ID);

    *gpio_gier   = 0x80000000;
    *gpio_ip_ier = 0x00000001;

    return 0;
}

/* ---- Network status callback ---- */
static void netif_status_callback(struct netif *netif)
{
    if (netif_is_link_up(netif)) {
        xil_printf("[ETH] Link UP\r\n");
        link_ready = 1;
    } else {
        xil_printf("[ETH] Link DOWN\r\n");
        link_ready = 0;
    }
}

/* ---- Main ---- */
int main(void)
{
    xil_printf("\r\n========================================\r\n");
    xil_printf("  AD7606 Ethernet UDP Streaming\r\n");
    xil_printf("========================================\r\n\r\n");

    Xil_DCacheDisable();

    /* Init GPIO direction */
    *gpio_tri  = 0xFFFFFFFF;
    *gpio2_tri = 0x00000000;

    /* Setup interrupts */
    if (setup_interrupts() != 0) {
        xil_printf("[ERROR] GIC interrupt setup failed\r\n");
        return -1;
    }
    xil_printf("[INIT] GIC + GPIO interrupts configured\r\n");

    /* Init lwIP */
    xil_printf("[INIT] Initializing lwIP...\r\n");
    lwip_init();

    /* Network interface */
    ip_addr_t ipaddr, netmask, gw;
    IP4_ADDR(&ipaddr,  192, 168,   1,  10);
    IP4_ADDR(&netmask, 255, 255, 255,   0);
    IP4_ADDR(&gw,      192, 168,   1,   1);
    IP4_ADDR(&dest_ip, 192, 168,   1, 100);

    if (!xemac_add(&server_netif, NULL, NULL, NULL, &ipaddr, &netmask, &gw)) {
        xil_printf("[ERROR] xemac_add failed\r\n");
        return -1;
    }
    netif_set_default(&server_netif);
    netif_set_up(&server_netif);
    netif_set_status_callback(&server_netif, netif_status_callback);

    /* UDP PCB */
    udp_pcb = udp_new();
    if (udp_pcb == NULL) {
        xil_printf("[ERROR] udp_new failed\r\n");
        return -1;
    }
    udp_bind(udp_pcb, IP_ADDR_ANY, UDP_PORT);
    xil_printf("[INIT] UDP socket ready, port %d\r\n", UDP_PORT);

    /* Wait for Ethernet link (poll up to 3 seconds) */
    xil_printf("[INIT] Waiting for Ethernet link...\r\n");
    int link_attempts = 0;
    while (!link_ready && link_attempts < 300) {
        xemacif_input(&server_netif);
        usleep(10000);
        link_attempts++;
    }
    if (!link_ready) {
        xil_printf("[WARN] No Ethernet link detected, continuing anyway\r\n");
    }

    /* Soft reset PL writer */
    xil_printf("[INIT] Soft reset PL writer...\r\n");
    *gpio2_data = CTRL_SOFT_RESET;
    short_delay(50000);
    *gpio2_data = 0;

    /* Enable capture */
    xil_printf("[INIT] Enabling capture...\r\n\r\n");
    *gpio2_data = CTRL_CAPTURE_EN;

    int bank_count = 0;
    int max_banks  = 1000;  /* 0 = unlimited */

    while (max_banks == 0 || bank_count < max_banks) {
        /* Poll for bank_ready (ISR-driven + polling fallback) */
        int timeout = 0;
        while (g_bank_events == 0) {
            xemacif_input(&server_netif);  /* feed lwIP receive/ARP */

            uint32_t raw = *gpio_data;
            if (raw & (STAT_BANK0_READY | STAT_BANK1_READY)) {
                g_bank_events |= (raw & (STAT_BANK0_READY | STAT_BANK1_READY));
                break;
            }
            if (++timeout > 20000000) {
                xil_printf("[DIAG] TIMEOUT raw=0x%08lX\r\n", (unsigned long)raw);
                goto done;
            }
        }

        /* Atomically consume events */
        uint32_t events;
        __asm__ volatile("cpsid i");
        events = g_bank_events;
        g_bank_events = 0;
        __asm__ volatile("cpsie i");

        /* Bank 0 */
        if (events & STAT_BANK0_READY) {
            uint32_t stat = *gpio_data;
            xil_printf("[BANK0] idx=%lu status=0x%08lX\r\n",
                       (unsigned long)((stat >> STAT_IDX_SHIFT) & 0xFFFF),
                       (unsigned long)stat);

            if (stat & STAT_ADC_OVERRUN) pulse_control(CTRL_CLEAR_OVERRUN);
            if (stat & STAT_OVERFLOW)    pulse_control(CTRL_CLEAR_OVF);

            send_bank_over_udp(0);

            pulse_control(CTRL_CLEAR_BANK0);
            while (*gpio_data & STAT_BANK0_READY);
            short_delay(1000);
            __asm__ volatile("cpsid i");
            g_bank_events &= ~STAT_BANK0_READY;
            __asm__ volatile("cpsie i");

            bank_count++;
        }

        /* Bank 1 */
        if (events & STAT_BANK1_READY) {
            uint32_t stat = *gpio_data;
            xil_printf("[BANK1] idx=%lu status=0x%08lX\r\n",
                       (unsigned long)((stat >> STAT_IDX_SHIFT) & 0xFFFF),
                       (unsigned long)stat);

            if (stat & STAT_ADC_OVERRUN) pulse_control(CTRL_CLEAR_OVERRUN);
            if (stat & STAT_OVERFLOW)    pulse_control(CTRL_CLEAR_OVF);

            send_bank_over_udp(1);

            pulse_control(CTRL_CLEAR_BANK1);
            while (*gpio_data & STAT_BANK1_READY);
            short_delay(1000);
            __asm__ volatile("cpsid i");
            g_bank_events &= ~STAT_BANK1_READY;
            __asm__ volatile("cpsie i");

            bank_count++;
        }
    }

done:
    *gpio2_data = 0;
    xil_printf("\r\n[DONE] Streaming stopped. Total banks: %d\r\n", bank_count);
    return 0;
}
