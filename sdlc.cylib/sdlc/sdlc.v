`include "cypress.v"

module dpll (
        input  wire clk,
        input  wire sync_in,
        output wire sync_out);

    /* --------- DCO -------------------------------------------------- */

    wire early, late;
    wire [6:0] count;
    reg  clken, clken90, dco_en;

    cy_psoc3_count7 #(
        .cy_period(7'd14),
        .cy_alt_mode(`TRUE))
    dco (
        .clock(clk),
        .reset(1'b0),
        .load(1'b0),
        .enable(dco_en),
        .count(count),
        .tc(sync_out)
    );

    always @(posedge clk) begin
        clken   <= (count[2:0] == 0);
        clken90 <= (count[2:0] == 4);
        if      (count[3:0] == 2)   dco_en <= late;
        else if (count[3:0] == 0)   dco_en <= !early;
        else                        dco_en <= 1'b1;
    end

    /* -------- Phase detection --------------------------------------- */

    reg dff1_q, dff2_q, dff3_q, dff4_q;

    /* Sample data */
    always @(posedge clk) begin
        if (clken) dff3_q <= sync_in;
        if (clken90) begin
            dff1_q <= sync_in;
            dff2_q <= dff1_q;
            dff4_q <= dff3_q;
        end
    end

    /* Edge detection */
    assign late =   (!dff2_q &&  dff1_q &&  dff4_q);
    assign early =  (!dff2_q &&  dff1_q && !dff4_q) ||
                    ( dff2_q && !dff1_q);

endmodule

module oversample (
        input  wire clk,
        input  wire sync_in,
        input  wire data_in,
        output wire data_out);

    parameter dpconfig0 = {
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG0: Idle */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC___D0, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG1: Preload Period (A0 <= D0) */
        `CS_ALU_OP__INC, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG2: Inc A0  ( A0 <= A0 + 1 ) */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC___D0, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG3: Preload Period (A0 <= D0) */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG4: Idle */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_A0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG5: Preload Period (A0 <= 0) */
        `CS_ALU_OP__DEC, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG6: Dec A0  ( A0 <= A0 - 1 ) */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_A0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG7: Preload Period (A0 <= 0) */

         8'hFF, 8'h00, /* SC_REG4: */
         8'hFF, 8'hFF, /* SC_REG5: */
        `SC_CMPB_A0_D1, `SC_CMPA_A0_D1, `SC_CI_B_ARITH,
        `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_DSBL,
        `SC_A_MASK_DSBL, `SC_DEF_SI_0, `SC_SI_B_DEFSI,
        `SC_SI_A_DEFSI, /* SC_REG6: */
        `SC_A0_SRC_ACC, `SC_SHIFT_SL, 1'b0,
         1'b0, `SC_FIFO1_BUS, `SC_FIFO0__A0,
        `SC_MSB_DSBL, `SC_MSB_BIT0, `SC_MSB_NOCHN,
        `SC_FB_NOCHN, `SC_CMP1_NOCHN,
        `SC_CMP0_NOCHN, /* SC_REG7: */
         10'h00, `SC_FIFO_CLK__DP,`SC_FIFO_CAP_FX,
        `SC_FIFO__EDGE,`SC_FIFO__SYNC,`SC_EXTCRC_DSBL,
        `SC_WRK16CAT_DSBL /* SC_REG8: */
    };

    cy_psoc3_dp8 #(.cy_dpconfig_a(dpconfig0))
    counterdp (
        .clk(clk),
        .cs_addr({data_in, 1'b1, sync_in}),
        .route_si(1'b0),
        .route_ci(1'b0),
        .f0_load(1'b0),
        .f1_load(1'b0),
        .d0_load(1'b0),
        .d1_load(1'b0),
        .cmsb(data_out)
    );

endmodule

module rx_ctrl (
        input  wire clk,
        input  wire data_en,
        input  wire data_in,
        output wire sync,
        output wire strip,
        output wire latch);

    reg   [2:0] rx_ones_count;
    reg   [3:0] rx_bit_count;
    reg         sync_d;

    always @(posedge clk) begin
        if (data_en) begin
            if (!data_in)   rx_ones_count <= 0;
            else            rx_ones_count <= rx_ones_count + 1;

            if (sync_d)         rx_bit_count <= 4'd15;
            else if (!strip)    rx_bit_count <= rx_bit_count - 1;

            sync_d <= sync;
        end
    end

    assign strip = rx_ones_count == 5;
    assign sync  = rx_ones_count == 6;
    assign latch = rx_bit_count  == 0 ? data_en : 1'b0;

endmodule

module crc_sdlc (
        input  wire clk,
        input  wire reset,
        input  wire data_en,
        input  wire data_in,
        input  wire data_latch);

    wire        si;
    wire  [1:0] so, cmsb;
    reg  [15:0] crc;
    reg   [2:0] dp_cs_addr;
    reg         data_en_d;
    reg         data_in_d;

    parameter dpconfig0 = {
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG0: Idle */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_A0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG1: Reset CRC, Load A1 */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SR, `CS_A0_SRC_NONE, `CS_A1_SRC__ALU,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGB,
        `CS_CMP_SEL_CFGA, /* CS_REG2: Shift data */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG3: Idle */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG4: Idle */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG5: Idle */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP___SL, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG6: CRC-XOR */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP___SL, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG7: CRC-Shift */

         8'hFF, 8'h00, /* SC_REG4: */
         8'hFF, 8'hFF, /* SC_REG5: */
        `SC_CMPB_A0_D1, `SC_CMPA_A0_D1, `SC_CI_B_ARITH,
        `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_DSBL,
        `SC_A_MASK_DSBL, `SC_DEF_SI_0, `SC_SI_B_CHAIN,
        `SC_SI_A_ROUTE, /* SC_REG6: Chained, A:Left shift, B: Right Shift */
        `SC_A0_SRC_ACC, `SC_SHIFT_SL, 1'b0,
         1'b0, `SC_FIFO1_BUS, `SC_FIFO0__A1,
        `SC_MSB_DSBL, `SC_MSB_BIT0, `SC_MSB_NOCHN,
        `SC_FB_NOCHN, `SC_CMP1_NOCHN,
        `SC_CMP0_NOCHN, /* SC_REG7: */
         10'h00, `SC_FIFO_CLK__DP,`SC_FIFO_CAP_FX,
        `SC_FIFO__EDGE,`SC_FIFO__SYNC,`SC_EXTCRC_ENBL,
        `SC_WRK16CAT_DSBL /* SC_REG8: */
    };

    parameter dpconfig1 = {
        //dpconfig0[207:80],
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG0: Idle */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_A0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG1: Reset CRC, Load A1 */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SR, `CS_A0_SRC_NONE, `CS_A1_SRC__ALU,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGB,
        `CS_CMP_SEL_CFGA, /* CS_REG2: Shift data */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG3: Idle */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG4: Idle */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG5: Idle */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP___SL, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG6: CRC-XOR */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP___SL, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG7: CRC-Shift */

         8'hFF, 8'h00, /* SC_REG4: */
         8'hFF, 8'hFF, /* SC_REG5: */
        `SC_CMPB_A0_D1, `SC_CMPA_A0_D1, `SC_CI_B_ARITH,
        `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_DSBL,
        `SC_A_MASK_DSBL, `SC_DEF_SI_0, `SC_SI_B_ROUTE,
        `SC_SI_A_CHAIN, /* SC_REG6: Chained, A:Left shift, B: Right Shift */
        `SC_A0_SRC_ACC, `SC_SHIFT_SL, 1'b0,
         1'b0, `SC_FIFO1_BUS, `SC_FIFO0__A1,
        `SC_MSB_DSBL, `SC_MSB_BIT0, `SC_MSB_NOCHN,
        `SC_FB_NOCHN, `SC_CMP1_NOCHN,
        `SC_CMP0_NOCHN, /* SC_REG7: */
         10'h00, `SC_FIFO_CLK__DP,`SC_FIFO_CAP_FX,
        `SC_FIFO__EDGE,`SC_FIFO__SYNC,`SC_EXTCRC_ENBL,
        `SC_WRK16CAT_DSBL /* SC_REG8: */
    };

    always @(reset or data_en or data_en_d or si) begin
        if (reset)          dp_cs_addr <= 3'b001;
        else if (data_en)   dp_cs_addr <= {2'b11, si};
        else if (data_en_d) dp_cs_addr <= 3'b010;
        else                dp_cs_addr <= 3'b000;
    end

    assign si = data_en_d ? data_in_d : (data_in ^ cmsb[1]);

    /* Fixme */
    always @(posedge clk) begin
        data_en_d <= data_en;
        data_in_d <= data_in;
    end

    cy_psoc3_dp16 #(.cy_dpconfig_a(dpconfig0), .cy_dpconfig_b(dpconfig1))
    crc_dp (
        .clk(clk),
        .cs_addr(dp_cs_addr),
        .route_si(si),
        .route_ci(1'b0),
        .f0_load(data_latch),
        .f1_load(1'b0),
        .d0_load(1'b0),
        .d1_load(1'b0),
        .so(so),
        .cmsb(cmsb)
    );

endmodule

module sdlc (
        input  wire clk,
        input  wire rx_data,
        input  wire rx_clk,
        output reg  rx_rdy);

    wire rx_clk_sync, rx_os;
    wire rx_sync, rx_strip, rx_latch;

    always @(posedge clk) begin
        if (rx_clk_sync) rx_rdy <= !rx_rdy;
    end

    dpll dpll (
        .clk(clk),
        .sync_in(rx_clk),
        .sync_out(rx_clk_sync)
    );

    oversample oversample (
        .clk(clk),
        .sync_in(rx_clk_sync),
        .data_in(rx_data),
        .data_out(rx_os)
    );

    rx_ctrl rx_ctrl (
        .clk(clk),
        .data_en(rx_clk_sync),
        .data_in(rx_os),
        .sync(rx_sync),
        .strip(rx_strip),
        .latch(rx_latch)
    );

    crc_sdlc crc (
        .clk(clk),
        .reset(rx_sync),
        .data_en(rx_clk_sync),
        .data_in(rx_os),
        .data_latch(rx_latch)
    );

endmodule
