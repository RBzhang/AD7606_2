`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 2026/06/26
// Module Name: sample_tick_gen
// Description:
//   Fixed-period sample tick generator.
//   The output sample_tick is asserted for one clk cycle every
//   SYS_CLK_HZ / SAMPLE_RATE_HZ clock cycles.
//
//   This module is intentionally independent of the ADC read FSM.
//   It has no external reset input. All registers use FPGA configuration-time
//   initial values, which is suitable for Xilinx FPGA/SoC PL logic.
//////////////////////////////////////////////////////////////////////////////////

module sample_tick_gen #(
    parameter integer SYS_CLK_HZ     = 50_000_000,
    parameter integer SAMPLE_RATE_HZ = 100_000
)(
    input  wire clk,

    output reg  sample_tick = 1'b0
);

    localparam integer SAMPLE_PERIOD_CLKS = SYS_CLK_HZ / SAMPLE_RATE_HZ;

    reg [31:0] tick_cnt = 32'd0;

    always @(posedge clk) begin
        sample_tick <= 1'b0;

        if (tick_cnt >= SAMPLE_PERIOD_CLKS - 1) begin
            tick_cnt    <= 32'd0;
            sample_tick <= 1'b1;
        end else begin
            tick_cnt <= tick_cnt + 1'b1;
        end
    end

endmodule
