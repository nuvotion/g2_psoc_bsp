`include "cypress.v"

module dpll (
        input  wire clk,
        input  wire sync_in,
        output wire sync_out,
        output reg  sync_out_d);

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
        clken       <= (count[2:0] == 0);
        clken90     <= (count[2:0] == 4);
        sync_out_d  <= sync_out;
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
    dp (
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

module tx_ctrl (
        input  wire         clk,
        input  wire         data_en,
        output wire         tx_crc,
        output wire         tx_crc_rst,
        output wire         tx_sync,
        output wire   [1:0] tx_state);

    localparam ST_TX_IDLE       = 0;
    localparam ST_TX_SYNC_START = 1;
    localparam ST_TX_PACKET     = 2;
    localparam ST_TX_SYNC_END   = 3;

    wire        f1_blk_stat, z0, z1;
    reg   [1:0] state = ST_TX_IDLE;
    reg   [1:0] cs_addr;
    reg         end_of_sync;

    assign tx_state     = state;
    assign tx_crc_rst   = end_of_sync;

    always @(posedge clk) begin
        case (state)
            ST_TX_SYNC_START: begin
                if (end_of_sync)  state <= ST_TX_PACKET;
            end

            ST_TX_PACKET: begin
                if (z0 & data_en) state <= ST_TX_SYNC_END;
            end

            ST_TX_SYNC_END: begin
                if (z1 & data_en) state <= ST_TX_IDLE;
            end

            default: begin /* ST_TX_IDLE */
                if (~f1_blk_stat) state <= ST_TX_SYNC_START;
            end
        endcase
    end

    always @(*) begin
        cs_addr     = 2'b00;
        end_of_sync = 1'b0;
        case (state)
            ST_TX_SYNC_START: begin
                if (z1 & ~f1_blk_stat)  cs_addr = 2'b10;
                else if (data_en)       cs_addr = 2'b11;
                if (z1 & data_en & f1_blk_stat) end_of_sync <= 1'b1;
            end

            ST_TX_PACKET: begin
                if (data_en & ~z0)      cs_addr = 2'b01;
            end

            ST_TX_SYNC_END: begin
                if (data_en)            cs_addr = 2'b11;
            end
        endcase
    end

    parameter dpconfig0 = {
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SR, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG0: Idle */
        `CS_ALU_OP__DEC, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC___D1,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG1: Data en and sync load */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC___F1,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG2: Load preamble */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SR, `CS_A0_SRC_NONE, `CS_A1_SRC__ALU,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG3: Shift sync */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG4: Idle */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG5: Idle */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG6: Idle */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG7: Idle */

         8'hFF, 8'h00, /* SC_REG4: */
         8'hFF, 8'hFF, /* SC_REG5: */
        `SC_CMPB_A0_D1, `SC_CMPA_A0_D1, `SC_CI_B_ARITH,
        `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_DSBL,
        `SC_A_MASK_DSBL, `SC_DEF_SI_0, `SC_SI_B_DEFSI,
        `SC_SI_A_DEFSI, /* SC_REG6: */
        `SC_A0_SRC_ACC, `SC_SHIFT_SL, 1'b0,
         1'b0, `SC_FIFO1_BUS, `SC_FIFO0_BUS,
        `SC_MSB_DSBL, `SC_MSB_BIT0, `SC_MSB_NOCHN,
        `SC_FB_NOCHN, `SC_CMP1_NOCHN,
        `SC_CMP0_NOCHN, /* SC_REG7: */
         10'h00, `SC_FIFO_CLK__DP,`SC_FIFO_CAP_FX,
        `SC_FIFO__EDGE,`SC_FIFO__SYNC,`SC_EXTCRC_DSBL,
        `SC_WRK16CAT_DSBL /* SC_REG8: */
    };

    cy_psoc3_dp8 #(.cy_dpconfig_a(dpconfig0))
    dp (
        .clk(clk),
        .cs_addr({1'b0, cs_addr}),
        .route_si(1'b0),
        .route_ci(1'b0),
        .f0_load(1'b0),
        .f1_load(1'b0),
        .d0_load(1'b0),
        .d1_load(1'b0),
        .f1_blk_stat(f1_blk_stat),
        .z0(z0),
        .z1(z1),
        .cl0(tx_crc),
        .so(tx_sync)
    );

endmodule

module sdlc_dp (
        input  wire         clk,
        input  wire         reset,
        input  wire         data_en,
        input  wire         crc_en,
        input  wire         data_in,
        input  wire         data_latch,
        input  wire   [1:0] tx_state,
        input  wire         tx_crc,
        output wire   [1:0] tx_out,
        output wire         rx_drq,
        output wire         tx_drq);

    localparam ST_TX_IDLE       = 0;
    localparam ST_TX_SYNC_START = 1;
    localparam ST_TX_PACKET     = 2;
    localparam ST_TX_SYNC_END   = 3;

    wire        si, ci;
    wire  [1:0] ce1, cmsb, f0_bus_stat, f1_bus_stat, f1_blk_stat;
    wire        bit_tc;
    reg   [2:0] dp_cs_addr;

    parameter dpconfig0 = {
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG0: Idle */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_A0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG1: Reset CRC */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SR, `CS_A0_SRC_NONE, `CS_A1_SRC__ALU,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGB,
        `CS_CMP_SEL_CFGA, /* CS_REG2: Shift data */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG3: Idle */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SR, `CS_A0_SRC_NONE, `CS_A1_SRC___F1,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG4: Shift data, load A1 */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_A0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC___F1,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG5: Reset CRC and load A1 */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP___SL, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_ENBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
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
        dpconfig0[207:80],

         8'hFF, 8'h00, /* SC_REG4: */
         8'hFF, 8'hFF, /* SC_REG5: */
        `SC_CMPB_A0_D1, `SC_CMPA_A0_D1, `SC_CI_B_ARITH,
        `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_DSBL,
        `SC_A_MASK_DSBL, `SC_DEF_SI_0, `SC_SI_B_ROUTE,
        `SC_SI_A_CHAIN, /* SC_REG6: Chained, A:Left shift, B: Right Shift */
        `SC_A0_SRC_ACC, `SC_SHIFT_SL, 1'b0,
         1'b0, `SC_FIFO1_BUS, `SC_FIFO0__A1,
        `SC_MSB_DSBL, `SC_MSB_BIT0, `SC_MSB_NOCHN,
        `SC_FB_CHNED, `SC_CMP1_CHNED,
        `SC_CMP0_NOCHN, /* SC_REG7: */
         10'h00, `SC_FIFO_CLK__DP,`SC_FIFO_CAP_FX,
        `SC_FIFO__EDGE,`SC_FIFO__SYNC,`SC_EXTCRC_DSBL,
        `SC_WRK16CAT_DSBL /* SC_REG8: */
    };

    always @(*) begin

        dp_cs_addr = 3'b000;
        if (tx_state == ST_TX_IDLE) begin
        end else begin
            if (reset)                      dp_cs_addr = 3'b101;        // Reset CRC, Load A1
            else if (tx_state == ST_TX_PACKET) begin
                if (data_en & bit_tc & ~tx_crc) dp_cs_addr = 3'b100;        // Load A1 
                else if (data_en & tx_crc)      dp_cs_addr = 3'b111;        // Calc CRC
                else if (crc_en & ~tx_crc)      dp_cs_addr = 3'b110;        // Calc CRC
                else if (data_en)               dp_cs_addr = 3'b010;        // Shift data
            end
        end
    end

    assign si = crc_en ? (data_in ^ cmsb[1]) : data_in;
    assign ci = ~(data_in ^ cmsb[1]);

    cy_psoc3_dp16 #(.cy_dpconfig_a(dpconfig0), .cy_dpconfig_b(dpconfig1))
    dp (
        .clk(clk),
        .cs_addr(dp_cs_addr),
        .route_si(si),
        .route_ci(ci),
        .f0_load(data_latch),
        .f1_load(1'b0),
        .d0_load(1'b0),
        .d1_load(1'b0),

        .ce1(ce1),
        .cmsb(cmsb),
        .so(tx_out),
        .f0_bus_stat(f0_bus_stat),
        .f1_bus_stat(f1_bus_stat),
        .f1_blk_stat(f1_blk_stat)
    );

    assign rx_drq = f0_bus_stat[0];
    assign tx_drq = f1_bus_stat[1];

    cy_psoc3_count7 #(
        .cy_period(7'd15),
        .cy_alt_mode(`TRUE))
    bit_count (
        .clock(clk),
        .reset(1'b0),
        .load(reset),
        .enable(data_en),
        .tc(bit_tc)
    );

    /*
    cy_psoc3_count7 #(
        .cy_period(7'd14),
        .cy_alt_mode(`TRUE))
    ones_count (
        .clock(clk),
        .reset(1'b0),
        .load(1'b0),
        .enable(dco_en),
        .count(count),
        .tc(sync_out)
    );
    */

endmodule

module sdlc (
        input  wire clk,
        input  wire rx_data,
        input  wire rx_clk,
        output reg  tx_data,
        output wire rx_drq,
        output reg  tx_drq);

    localparam ST_TX_IDLE       = 0;
    localparam ST_TX_SYNC_START = 1;
    localparam ST_TX_PACKET     = 2;
    localparam ST_TX_SYNC_END   = 3;

    wire        dpll_clk_sync, dpll_clk_sync_d;
    reg         rx_clk_gated;
    reg         sdlc_dp_rst;
    reg         sdlc_dp_din;

    wire  [1:0] tx_state;
    wire        tx_crc;
    wire        tx_dp_drq;
    wire        tx_crc_rst;
    wire        tx_sync;
    wire  [1:0] tx_data_out;

    wire        rx_os;
    wire        rx_sync;
    wire        rx_strip;
    wire        rx_latch;

    always @(*) begin
        if (tx_state == ST_TX_IDLE) begin
            rx_clk_gated    = rx_clk;
            tx_drq          = 1'b0;
            sdlc_dp_rst     = rx_sync;
            sdlc_dp_din     = rx_os;
        end else begin
            rx_clk_gated    = 1'b0;
            tx_drq          = tx_dp_drq;
            sdlc_dp_rst     = tx_crc_rst;
            sdlc_dp_din     = dpll_clk_sync ? tx_data_out[0] : tx_data;
        end
    end

    sdlc_dp sdlc_dp (
        .clk(clk),
        .reset(sdlc_dp_rst),
        .data_en(dpll_clk_sync),
        .crc_en(dpll_clk_sync_d),
        .data_in(sdlc_dp_din),
        .data_latch(rx_latch),
        .tx_state(tx_state),
        .tx_crc(tx_crc),
        .tx_out(tx_data_out),
        .rx_drq(rx_drq),
        .tx_drq(tx_dp_drq)
    );

    dpll dpll (
        .clk(clk),
        .sync_in(rx_clk_gated),
        .sync_out(dpll_clk_sync),
        .sync_out_d(dpll_clk_sync_d)
    );

    /* -------- Transmitter-------------------------------------------- */
    tx_ctrl tx_ctrl (
        .clk(clk),
        .data_en(dpll_clk_sync_d),
        .tx_state(tx_state),
        .tx_crc(tx_crc),
        .tx_crc_rst(tx_crc_rst),
        .tx_sync(tx_sync)
    );

    always @(posedge clk) begin
        if (dpll_clk_sync) begin
            case (tx_state)
                ST_TX_SYNC_START: begin
                    tx_data <= tx_sync;
                end

                ST_TX_PACKET: begin
                    if (tx_crc) tx_data <= tx_data_out[1];
                    else        tx_data <= tx_data_out[0];
                end

                ST_TX_SYNC_END: begin
                    tx_data <= tx_sync;
                end
            endcase
        end
    end

    /* -------- Receiver ---------------------------------------------- */
    oversample oversample (
        .clk(clk),
        .sync_in(dpll_clk_sync),
        .data_in(rx_data),
        .data_out(rx_os)
    );

    rx_ctrl rx_ctrl (
        .clk(clk),
        .data_en(dpll_clk_sync),
        .data_in(rx_os),
        .sync(rx_sync),
        .strip(rx_strip),
        .latch(rx_latch)
    );

endmodule
