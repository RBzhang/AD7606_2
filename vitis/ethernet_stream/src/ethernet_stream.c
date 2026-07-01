/******************************************************************************
 * AD7606  Ethernet UDP Streaming
 *
 * Replaces UART print with lwIP UDP to stream BRAM bank data to a PC.
 * Each bank (4096 samples * 8 bytes = 32KB) is fragmented into packets.
 *
 * Hardware: Zynq-7020, ENET0 RGMII (MIO16-27), GEM0 @ 125MHz ref clock
 *
 * Packet format (per fragment):
 *   [0..3]   bank_id   (LE u32): 0 = bank0, 1 = bank1
 *   [4..7]   frag_seq  (LE u32): fragment sequence, 0-based
 *   [8..11]  start_idx (LE u32): sample index offset in bank
 *   [12..N]  raw 16-bit ADC data (bank data slice)
 *
 * Build requirements (Vitis 2024.2):
 *   - BSP: enable lwip211 library
 *   - CMakeLists.txt: add lwip4_lib;lwip6_lib to LIB_DEPS
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "platform.h"
#include "xil_printf.h"
#include "xil_cache.h"
#include "xil_exception.h"
#include "xscugic.h"
#include "xil_mmu.h"

#include "lwip/init.h"
#include "lwip/udp.h"
#include "lwip/ip_addr.h"
#include "lwip/tcpip.h"
#include "lwip/timeouts.h"
#include "netif/xadapter.h"

/* ---- Network config ---- */
#define DEST_IP_ADDR   "192.168.1.100"   /* PC IP address */
#define LOCAL_IP_ADDR  "192.168.1.10"    /* Zynq board static IP */
#define NET_MASK       "255.255.255.0"
#define GW_ADDR        "192.168.1.1"
#define UDP_PORT       5001

/* ---- Hardware address map ---- */
#define AXI_GPIO_BASE       0x41200000
#define GPIO_DATA_OFFSET    0x0000
#define GPIO_TRI_OFFSET     0x0004
#define GPIO2_DATA_OFFSET   0x0008
#define GPIO2_TRI_OFFSET    0x000C
#define GPIO_GIER_OFFSET    0x011C
#define GPIO_IP_ISR_OFFSET  0x0120
#define GPIO_IP_IER_OFFSET  0x0128

#define AXI_BRAM_BASE       0x40000000

/* ---- ps_control bits (PS -> PL) ---- */
#define CTRL_CLEAR_BANK0    (1U << 0)
#define CTRL_CLEAR_BANK1    (1U << 1)
#define CTRL_CAPTURE_EN     (1U << 2)
#define CTRL_CLEAR_OVF      (1U << 3)
#define CTRL_SOFT_RESET     (1U << 4)
#define CTRL_CLEAR_OVERRUN  (1U << 5)
#define CTRL_CLEAR_UNDERRUN (1U << 6)

/* ---- pl_status bits (PL -> PS) ---- */
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
#define BANK_SIZE_BYTES     (BANK_SAMPLE_COUNT * 8)  /* 32768 */
#define MAX_UDP_PAYLOAD     1460
#define SAMPLES_PER_PACKET  (MAX_UDP_PAYLOAD / 8)    /* 182 */

/* ---- Interrupt ID ---- */
#define GPIO_INTR_ID  61

/* ---- GPIO register pointers ---- */
static volatile uint32_t *gpio_data   = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_DATA_OFFSET);
static volatile uint32_t *gpio_tri    = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_TRI_OFFSET);
static volatile uint32_t *gpio2_data  = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO2_DATA_OFFSET);
static volatile uint32_t *gpio2_tri   = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO2_TRI_OFFSET);
static volatile uint32_t *gpio_gier   = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_GIER_OFFSET);
static volatile uint32_t *gpio_ip_isr = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_IP_ISR_OFFSET);
static volatile uint32_t *gpio_ip_ier = (volatile uint32_t *)(AXI_GPIO_BASE + GPIO_IP_IER_OFFSET);

static XScuGic gic_inst;
static volatile uint32_t g_bank_events = 0;

/* ---- lwIP globals ---- */
static struct netif server_netif;
static struct udp_pcb *udp_pcb;
static ip_addr_t dest_ip;
static volatile int link_ready = 0;

/* ---- Utility ---- */
static void delay_loop(volatile int n) { while (n--); }

static void pulse_control(uint32_t bit)
{
    uint32_t cur = *gpio2_data;
    *gpio2_data = cur | bit;
    delay_loop(5000);
    *gpio2_data = cur & ~bit;
}

/* ---- lwIP callback: link status change ---- */
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

/* ---- Send one BRAM bank as fragmented UDP packets ---- */
static void send_bank_over_udp(int is_bank1)
{
    if (!link_ready || udp_pcb == NULL) {
        xil_printf("[ERR] Network not ready, dropping bank%d\r\n", is_bank1);
        return;
    }

    uint32_t bank_base = is_bank1 ? 0x8000 : 0x0000;
    uint32_t bank_id = is_bank1 ? 1 : 0;
    uint32_t total_samples = BANK_SAMPLE_COUNT;
    uint32_t sample_idx = 0;
    uint32_t frag_seq = 0;

    while (sample_idx < total_samples) {
        uint32_t samples_this_pkt = total_samples - sample_idx;
        if (samples_this_pkt > SAMPLES_PER_PACKET) {
            samples_this_pkt = SAMPLES_PER_PACKET;
        }

        uint32_t data_bytes = samples_this_pkt * 8;
        uint32_t pkt_size = 12 + data_bytes; /* 12-byte header + payload */

        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, pkt_size, PBUF_RAM);
        if (p == NULL) {
            xil_printf("[ERR] pbuf_alloc failed at sample %lu\r\n", sample_idx);
            break;
        }

        /* Write header */
        uint8_t *hdr = (uint8_t *)p->payload;
        *(uint32_t *)(hdr + 0) = bank_id;       /* LE u32 */
        *(uint32_t *)(hdr + 4) = frag_seq;
        *(uint32_t *)(hdr + 8) = sample_idx;

        /* Copy BRAM data directly into pbuf payload */
        uint16_t *dst = (uint16_t *)(hdr + 12);
        volatile uint32_t *bram_ptr = (volatile uint32_t *)(AXI_BRAM_BASE + bank_base + sample_idx * 8);
        uint32_t words = samples_this_pkt * 2;

        for (uint32_t i = 0; i < words; i++) {
            uint32_t w = bram_ptr[i];
            dst[i * 2 + 0] = (uint16_t)(w & 0xFFFF);
            dst[i * 2 + 1] = (uint16_t)(w >> 16);
        }

        err_t err = udp_sendto(udp_pcb, p, &dest_ip, UDP_PORT);
        pbuf_free(p);

        if (err != ERR_OK) {
            xil_printf("[ERR] udp_sendto failed: %d at frag %lu\r\n", err, frag_seq);
            break;
        }

        sample_idx += samples_this_pkt;
        frag_seq++;
    }
}

/* ---- GPIO ISR ---- */
static void gpio_isr(void *callback_ref)
{
    uint32_t status = *gpio_data;
    g_bank_events |= (status & (STAT_BANK0_READY | STAT_BANK1_READY));
    *gpio_ip_isr = 0x00000001;
}

/* ---- GIC + GPIO interrupt setup ---- */
static int setup_interrupts(void)
{
    int status;
    XScuGic_Config *gic_cfg;

    gic_cfg = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
    if (gic_cfg == NULL) return -1;

    status = XScuGic_CfgInitialize(&gic_inst, gic_cfg, gic_cfg->CpuBaseAddress);
    if (status != XST_SUCCESS) return -2;

    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
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

/* ---- Ethernet PHY link poll thread ---- */
static void phy_poll(void)
{
    struct netif *netif = &server_netif;
    xemacpsif_speed speed = xemacpsif_get_linkspeed(netif);

    if (speed != 0 && !link_ready) {
        xil_printf("[ETH] Link at %d Mbps\r\n", (speed == 1000) ? 1000 :
                                                (speed == 100) ? 100 : 10);
        link_ready = 1;
    } else if (speed == 0 && link_ready) {
        xil_printf("[ETH] Link DOWN\r\n");
        link_ready = 0;
    }
}

/* ---- Main ---- */
int main()
{
    init_platform();
    Xil_DCacheDisable();

    print("\r\n========================================\r\n");
    print("  AD7606 Ethernet UDP Streaming\r\n");
    print("========================================\r\n\r\n");

    /* GPIO direction */
    *gpio_tri  = 0xFFFFFFFF;
    *gpio2_tri = 0x00000000;

    /* Set up GIC + GPIO interrupts */
    if (setup_interrupts() != 0) {
        print("[ERROR] Interrupt setup failed\r\n");
        return -1;
    }
    print("[INIT] GIC + GPIO interrupts configured\r\n");

    /* Initialize lwIP */
    xil_printf("[INIT] Starting lwIP...\r\n");
    lwip_init();

    /* Set up network interface */
    ip_addr_t ipaddr, netmask, gw;
    IP4_ADDR(&ipaddr,  192, 168,   1,  10);
    IP4_ADDR(&netmask, 255, 255, 255,   0);
    IP4_ADDR(&gw,      192, 168,   1,   1);
    IP4_ADDR(&dest_ip, 192, 168,   1, 100);

    if (!xemac_add(&server_netif, NULL, NULL, NULL, &ipaddr, &netmask, &gw)) {
        print("[ERROR] xemac_add failed\r\n");
        return -1;
    }
    netif_set_default(&server_netif);
    netif_set_up(&server_netif);
    netif_set_status_callback(&server_netif, netif_status_callback);

    /* Create UDP PCB */
    udp_pcb = udp_new();
    if (udp_pcb == NULL) {
        print("[ERROR] udp_new failed\r\n");
        return -1;
    }
    udp_bind(udp_pcb, IP_ADDR_ANY, UDP_PORT);

    xil_printf("[INIT] UDP socket ready, port %d\r\n", UDP_PORT);
    xil_printf("[INIT] Waiting for Ethernet link...\r\n");

    /* Wait for link up (max 5 seconds) */
    int link_timeout = 0;
    while (!link_ready && link_timeout < 5000000) {
        phy_poll();
        xemacif_input(&server_netif);
        link_timeout++;
    }
    if (!link_ready) {
        xil_printf("[WARN] No link detected, streaming anyway...\r\n");
    }

    /* Soft reset PL writer */
    xil_printf("[INIT] Soft reset PL writer...\r\n");
    *gpio2_data = CTRL_SOFT_RESET;
    delay_loop(50000);
    *gpio2_data = 0;

    /* Enable capture */
    xil_printf("[INIT] Enabling capture...\r\n\r\n");
    *gpio2_data = CTRL_CAPTURE_EN;

    int bank_count = 0;
    int max_banks  = 10;

    while (bank_count < max_banks) {
        /* Poll for bank_ready (ISR + polling fallback) */
        int timeout = 0;
        while (g_bank_events == 0) {
            /* feed lwIP receive path (ARP, etc.) */
            xemacif_input(&server_netif);

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

        /* Atomically consume events */
        uint32_t events;
        /* Get the specific idle function */
        __asm__ volatile("cpsid i");
        events = g_bank_events;
        g_bank_events = 0;
        __asm__ volatile("cpsie i");

        /* Process bank0 */
        if (events & STAT_BANK0_READY) {
            uint32_t status = *gpio_data;
            uint32_t idx = (status >> STAT_IDX_SHIFT) & 0xFFFF;
            xil_printf("[BANK0] idx=%lu, streaming %d samples...\r\n", idx, BANK_SAMPLE_COUNT);

            if (status & STAT_ADC_OVERRUN)  pulse_control(CTRL_CLEAR_OVERRUN);
            if (status & STAT_OVERFLOW)     pulse_control(CTRL_CLEAR_OVF);

            send_bank_over_udp(0);

            pulse_control(CTRL_CLEAR_BANK0);
            while (*gpio_data & STAT_BANK0_READY);
            delay_loop(1000);
            __asm__ volatile("cpsid i");
            g_bank_events &= ~STAT_BANK0_READY;
            __asm__ volatile("cpsie i");
            bank_count++;
        }

        /* Process bank1 */
        if (events & STAT_BANK1_READY) {
            uint32_t status = *gpio_data;
            uint32_t idx = (status >> STAT_IDX_SHIFT) & 0xFFFF;
            xil_printf("[BANK1] idx=%lu, streaming %d samples...\r\n", idx, BANK_SAMPLE_COUNT);

            if (status & STAT_ADC_OVERRUN)  pulse_control(CTRL_CLEAR_OVERRUN);
            if (status & STAT_OVERFLOW)     pulse_control(CTRL_CLEAR_OVF);

            send_bank_over_udp(1);

            pulse_control(CTRL_CLEAR_BANK1);
            while (*gpio_data & STAT_BANK1_READY);
            delay_loop(1000);
            __asm__ volatile("cpsid i");
            g_bank_events &= ~STAT_BANK1_READY;
            __asm__ volatile("cpsie i");
            bank_count++;
        }

        /* PHY link poll */
        phy_poll();
    }

done:
    *gpio2_data = 0;
    print("\r\n[DONE] Streaming stopped.\r\n");
    cleanup_platform();
    return 0;
}
