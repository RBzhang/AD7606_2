`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name: system_top
// Description:
//   Real top level for the Zynq-7020 AD7606 acquisition system.
//
//   Do not edit the auto-generated design_1_wrapper.v directly. This module
//   instantiates it and connects the exported BRAM native port and AXI GPIO
//   ports to user RTL.
//
//   Data path:
//       AD7606/AD7606C
//           -> adc_sample4
//           -> sample_valid + sample_data[63:0]
//           -> bram_sample_writer
//           -> design_1_wrapper external BRAM port
//           -> PS reads BRAM through AXI BRAM Controller
//           -> PS sends data to PC through Ethernet in bare-metal software
//
//   AXI GPIO convention from design_1_wrapper:
//       gpio_rtl_0_tri_i : PL -> PS status
//       gpio_rtl_1_tri_o : PS -> PL control
//////////////////////////////////////////////////////////////////////////////////

module system_top #(
    parameter integer SYS_CLK_HZ       = 50_000_000,
    parameter integer SAMPLE_RATE_HZ   = 200_000,
    parameter integer ADC_TOTAL_CH     = 8,
    parameter integer RESET_CLKS       = 10,
    parameter integer CONVST_HIGH_CLKS = 2,
    parameter integer RD_LOW_CLKS      = 5,
    parameter integer RD_HIGH_CLKS     = 2,
    parameter integer BANK_SAMPLE_COUNT = 4096
)(
    // =========================
    // AD7606/AD7606C -> FPGA PL
    // =========================
    input  wire        adc_busy,
    input  wire [15:0] adc_db,

    // =========================
    // FPGA PL -> AD7606/AD7606C
    // =========================
    output wire        adc_convst_a,
    output wire        adc_convst_b,
    output wire        adc_cs_n,
    output wire        adc_rd_n,
    output wire        adc_reset,
    output wire [2:0]  adc_os,

    // =========================
    // Zynq PS DDR and fixed IO
    // =========================
    inout  wire [14:0] DDR_addr,
    inout  wire [2:0]  DDR_ba,
    inout  wire        DDR_cas_n,
    inout  wire        DDR_ck_n,
    inout  wire        DDR_ck_p,
    inout  wire        DDR_cke,
    inout  wire        DDR_cs_n,
    inout  wire [3:0]  DDR_dm,
    inout  wire [31:0] DDR_dq,
    inout  wire [3:0]  DDR_dqs_n,
    inout  wire [3:0]  DDR_dqs_p,
    inout  wire        DDR_odt,
    inout  wire        DDR_ras_n,
    inout  wire        DDR_reset_n,
    inout  wire        DDR_we_n,
    inout  wire        FIXED_IO_ddr_vrn,
    inout  wire        FIXED_IO_ddr_vrp,
    inout  wire [53:0] FIXED_IO_mio,
    inout  wire        FIXED_IO_ps_clk,
    inout  wire        FIXED_IO_ps_porb,
    inout  wire        FIXED_IO_ps_srstb
);

    // ============================================================
    // Clock and reset from Zynq PS
    // ============================================================
    wire        clk;
    wire        ps_fclk_resetn;

    // ============================================================
    // ADC sample stream
    // ============================================================

    wire        sample_valid;
    wire [63:0] sample_data;
    wire        adc_overrun;
    wire        adc_underrun;

    adc_sample4 #(
        .SYS_CLK_HZ       (SYS_CLK_HZ),
        .SAMPLE_RATE_HZ   (SAMPLE_RATE_HZ),
        .ADC_TOTAL_CH     (ADC_TOTAL_CH),
        .RESET_CLKS       (RESET_CLKS),
        .CONVST_HIGH_CLKS (CONVST_HIGH_CLKS),
        .RD_LOW_CLKS      (RD_LOW_CLKS),
        .RD_HIGH_CLKS     (RD_HIGH_CLKS)
    ) u_adc_sample4 (
        .clk            (clk),
        .rst_n          (ps_fclk_resetn),

        .adc_busy       (adc_busy),
        .adc_db         (adc_db),
        .adc_convst_a   (adc_convst_a),
        .adc_convst_b   (adc_convst_b),
        .adc_cs_n       (adc_cs_n),
        .adc_rd_n       (adc_rd_n),
        .adc_reset      (adc_reset),
        .adc_os         (adc_os),
        .data_valid     (sample_valid),
        .data_out       (sample_data),
        .overrun_o      (adc_overrun),
        .underrun_o     (adc_underrun)
    );

    // ============================================================
    // BRAM native port and AXI GPIO signals exported by BD wrapper
    // ============================================================

    wire [31:0] bram_addr;
    wire        bram_clk;
    wire [31:0] bram_din;
    wire [31:0] bram_dout;
    wire        bram_en;
    wire        bram_rst;
    wire [3:0]  bram_we;

    wire [31:0] pl_status;
    wire [31:0] ps_control;

    bram_sample_writer #(
        .BANK_SAMPLE_COUNT(BANK_SAMPLE_COUNT),
        .BANK0_BASE_ADDR  (32'h0000_0000),
        .BANK1_BASE_ADDR  (32'h0000_8000)
    ) u_bram_sample_writer (
        .clk               (clk),
        .rst_n             (ps_fclk_resetn),
        .sample_valid      (sample_valid),
        .sample_data       (sample_data),
        .ps_control        (ps_control),
        .pl_status         (pl_status),
        .sample_overrun_i  (adc_overrun),
        .sample_underrun_i (adc_underrun),
        .bram_addr         (bram_addr),
        .bram_clk          (bram_clk),
        .bram_din          (bram_din),
        .bram_dout         (bram_dout),
        .bram_en           (bram_en),
        .bram_rst          (bram_rst),
        .bram_we           (bram_we)
    );

    // ============================================================
    // Auto-generated Block Design wrapper
    // ============================================================

    design_1_wrapper u_design_1_wrapper (
        .BRAM_PORTB_0_addr (bram_addr),
        .BRAM_PORTB_0_clk  (bram_clk),
        .BRAM_PORTB_0_din  (bram_din),
        .BRAM_PORTB_0_dout (bram_dout),
        .BRAM_PORTB_0_en   (bram_en),
        .BRAM_PORTB_0_rst  (bram_rst),
        .BRAM_PORTB_0_we   (bram_we),

        .DDR_addr          (DDR_addr),
        .DDR_ba            (DDR_ba),
        .DDR_cas_n         (DDR_cas_n),
        .DDR_ck_n          (DDR_ck_n),
        .DDR_ck_p          (DDR_ck_p),
        .DDR_cke           (DDR_cke),
        .DDR_cs_n          (DDR_cs_n),
        .DDR_dm            (DDR_dm),
        .DDR_dq            (DDR_dq),
        .DDR_dqs_n         (DDR_dqs_n),
        .DDR_dqs_p         (DDR_dqs_p),
        .DDR_odt           (DDR_odt),
        .DDR_ras_n         (DDR_ras_n),
        .DDR_reset_n       (DDR_reset_n),
        .DDR_we_n          (DDR_we_n),

        .FCLK_CLK0         (clk),
        .FCLK_RESET0_N     (ps_fclk_resetn),

        .FIXED_IO_ddr_vrn  (FIXED_IO_ddr_vrn),
        .FIXED_IO_ddr_vrp  (FIXED_IO_ddr_vrp),
        .FIXED_IO_mio      (FIXED_IO_mio),
        .FIXED_IO_ps_clk   (FIXED_IO_ps_clk),
        .FIXED_IO_ps_porb  (FIXED_IO_ps_porb),
        .FIXED_IO_ps_srstb (FIXED_IO_ps_srstb),

        .gpio_rtl_0_tri_i  (pl_status),
        .gpio_rtl_1_tri_o  (ps_control)
    );

endmodule
