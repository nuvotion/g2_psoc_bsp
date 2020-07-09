`include "cypress.v"

module cy_psoc3_udb_clock_enable_v1_0_shim(
        input  wire clock_in,
        input  wire enable,
        output wire clock_out);

    parameter	sync_mode = `FALSE;
    parameter	sim       = `FALSE;

    wire clock_in_shim;

    generate
        if (sim) assign clock_in_shim = ~clock_in;
        else     assign clock_in_shim =  clock_in;
    endgenerate

    cy_psoc3_udb_clock_enable_v1_0 #(.sync_mode(sync_mode))
    clk_enable (
        clock_in_shim,
        enable,
        clock_out
    );

endmodule

module dpll (
        input  wire clk,
        input  wire sync_in,
        output wire sync_out,
        output reg  clk_out);

    parameter sim = `FALSE;

    /* --------- DCO -------------------------------------------------- */

    wire        early, late;
    wire  [6:0] count;
    reg         clken, clken90, dco_en;

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
        if      (count[3:0] == 2)   dco_en  <= late;
        else if (count[3:0] == 0)   dco_en  <= !early;
        else                        dco_en  <= 1'b1;
        if      (count[3:0] == 5)   clk_out <= 1'b1;
        else if (count[3:0] == 13)  clk_out <= 1'b0;
    end

    /* -------- Phase detection --------------------------------------- */

    reg dff1_q, dff2_q, dff3_q, dff4_q;
    wire clk_en, clk_en90;

    cy_psoc3_udb_clock_enable_v1_0_shim #(.sync_mode(`TRUE), .sim(sim))
    clk_sync_1 (
        .clock_in(clk),
        .enable(clken),
        .clock_out(clk_en)
    );

    cy_psoc3_udb_clock_enable_v1_0_shim #(.sync_mode(`TRUE), .sim(sim))
    clk_sync_0 (
        .clock_in(clk),
        .enable(clken90),
        .clock_out(clk_en90)
    );

    /* Sample data */
    always @(posedge clk_en) begin
        dff3_q <= sync_in;
    end

    always @(posedge clk_en90) begin
        dff1_q <= sync_in;
        dff2_q <= dff1_q;
        dff4_q <= dff3_q;
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

module tx_ctrl (
        input  wire clk,
        input  wire data_en,
        input  wire zero_ins,
        output reg  sdlc_dir,
        output wire tx_crc_rst,
        output wire tx_state_crc,
        output wire tx_state_sync,
        output wire tx_sync_data);

    parameter sim = `FALSE;

    localparam ST_TX_IDLE       = 0;
    localparam ST_TX_SYNC_START = 1;
    localparam ST_TX_PACKET     = 2;
    localparam ST_TX_SYNC_END   = 3;

    wire        clk_fsm;
    wire        f1_blk_stat, z0, z1;
    wire        load_preamble;
    reg   [1:0] state;
    reg         zero_ins_gate;

    assign tx_state_sync    = state[0];
    assign tx_crc_rst       = (state == ST_TX_SYNC_START) & z1 & f1_blk_stat;

    cy_psoc3_udb_clock_enable_v1_0_shim #(.sync_mode(`TRUE), .sim(sim))
    clk_sync_0 (
        .clock_in(clk),
        .enable(data_en),
        .clock_out(clk_fsm)
    );

    always @(posedge clk_fsm) begin
        zero_ins_gate <= (state == ST_TX_PACKET);
        case (state)
            ST_TX_SYNC_START: begin
                if (tx_crc_rst)     state <= ST_TX_PACKET;
            end

            ST_TX_PACKET: begin
                if (z0 & ~zero_ins) state <= ST_TX_SYNC_END;
            end

            ST_TX_SYNC_END: begin
                if (z1)             state <= ST_TX_IDLE;
            end

            default: begin /* ST_TX_IDLE */
                sdlc_dir <= 1'b0;
                if (~f1_blk_stat) begin
                    state       <= ST_TX_SYNC_START;
                    sdlc_dir    <= 1'b1;
                end
            end
        endcase
    end

    assign load_preamble =  (z1 & ~f1_blk_stat) |
                            (data_en & ((zero_ins & zero_ins_gate) | ~sdlc_dir));

    parameter dpconfig0 = {
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SL, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG0: Idle */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SL, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG1: Idle */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC___F1,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG2: Load preamble */
        `CS_ALU_OP_PASS, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC_NONE, `CS_A1_SRC___F1,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG3: Load preamble */

        `CS_ALU_OP__DEC, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC___D1,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG4: Data en and sync load */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SL, `CS_A0_SRC_NONE, `CS_A1_SRC__ALU,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG5: Shift sync */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SL, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG6: Idle */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SL, `CS_A0_SRC_NONE, `CS_A1_SRC_NONE,
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

    cy_psoc3_dp8 #(.f1_blk_sync(`FALSE), .cy_dpconfig_a(dpconfig0))
    dp (
        .clk(clk),
        .cs_addr({data_en, load_preamble, state[0]}),
        .route_si(1'b0),
        .route_ci(1'b0),
        .f0_load(1'b0),
        .f1_load(1'b0),
        .d0_load(1'b0),
        .d1_load(1'b0),
        .f1_blk_stat(f1_blk_stat),
        .z0(z0),
        .z1(z1),
        .cl0(tx_state_crc),
        .so(tx_sync_data)
    );

endmodule

module sdlc_dp (
        input  wire         clk,
        input  wire   [3:0] clken,
        input  wire         sdlc_dir,
        input  wire         tx_start,
        input  wire         tx_state_crc,
        input  wire         tx_state_sync,
        input  wire         tx_sync_data,
        output wire         tx_zero_ins,
        output reg          tx_out,
        output wire         tx_drq,
        input  wire         rx_data,
        output wire         rx_valid,
        output wire         rx_drq,
        output wire   [7:0] dbg);

    parameter sim = `FALSE;

    wire        dp_clk, out_clk;
    reg         dp_clk_en;
    wire        si, ci, f0_load;
    wire  [1:0] ce1, cmsb, so, f0_bus_stat, f1_bus_stat;
    reg         dp_din;
    wire        so_lsb;
    wire        bit_tc;
    wire        zero_ins;
    wire        pkt_sync;
    reg   [1:0] dp_cs_addr;

    assign dbg[0] = clk; 
    assign dbg[1] = tx_out; 
    assign dbg[2] = zero_ins; 
    assign dbg[3] = sdlc_dir; 
    assign dbg[4] = dp_cs_addr[0]; 
    assign dbg[5] = dp_cs_addr[1]; 
    assign dbg[6] = f1_bus_stat[1]; 
    assign dbg[7] = pkt_sync; 

    assign tx_zero_ins  = zero_ins;
    assign tx_drq       = sdlc_dir & f1_bus_stat[1];
    assign rx_valid     = ce1[1];
    assign rx_drq       = f0_bus_stat[1];

    cy_psoc3_udb_clock_enable_v1_0_shim #(.sync_mode(`TRUE), .sim(sim))
    clk_sync_1 (
        .clock_in(clk),
        .enable(clken[2]),
        .clock_out(out_clk)
    );

    cy_psoc3_udb_clock_enable_v1_0_shim #(.sync_mode(`TRUE), .sim(sim))
    clk_sync_0 (
        .clock_in(clk),
        .enable(dp_clk_en),
        .clock_out(dp_clk)
    );

    always @(posedge out_clk) begin
        if (tx_state_sync) begin
            tx_out <= tx_sync_data;
        end else begin
            if (zero_ins)           tx_out <= 1'b0;
            else if (tx_state_crc)  tx_out <= cmsb[1];
            else                    tx_out <= dp_din;
        end
    end

    parameter dpconfig0 = {
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SR, `CS_A0_SRC_NONE, `CS_A1_SRC__ALU,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGB,
        `CS_CMP_SEL_CFGA, /* CS_REG0: Shift data */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SR, `CS_A0_SRC_NONE, `CS_A1_SRC__ALU,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGB,
        `CS_CMP_SEL_CFGA, /* CS_REG1: Shift data */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP___SL, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_ENBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG2: CRC-XOR */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_A0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG3: Reset CRC */

        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SR, `CS_A0_SRC_NONE, `CS_A1_SRC__ALU,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGB,
        `CS_CMP_SEL_CFGA, /* CS_REG4: Shift data */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SR, `CS_A0_SRC_NONE, `CS_A1_SRC___F1,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG5: Shift data, load A1 */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP___SL, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_ENBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG6: CRC-XOR */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_A0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG7: Reset CRC */

         8'hFF, 8'h00, /* SC_REG4: */
         8'hFF, 8'hFF, /* SC_REG5: */
        `SC_CMPB_A0_D1, `SC_CMPA_A0_D1, `SC_CI_B_ARITH,
        `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_DSBL,
        `SC_A_MASK_DSBL, `SC_DEF_SI_0, `SC_SI_B_CHAIN,
        `SC_SI_A_ROUTE, /* SC_REG6: Chained, A:Left shift, B: Right Shift */
        `SC_A0_SRC_ACC, `SC_SHIFT_SR, 1'b0,
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
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SR, `CS_A0_SRC_NONE, `CS_A1_SRC__ALU,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGB,
        `CS_CMP_SEL_CFGA, /* CS_REG0: Shift data */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SR, `CS_A0_SRC_NONE, `CS_A1_SRC__ALU,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGB,
        `CS_CMP_SEL_CFGA, /* CS_REG1: Shift data */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP___SL, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_ENBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG2: CRC-XOR */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_A0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG3: Reset CRC */

        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SR, `CS_A0_SRC_NONE, `CS_A1_SRC__ALU,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGB,
        `CS_CMP_SEL_CFGA, /* CS_REG4: Shift data */
        `CS_ALU_OP_PASS, `CS_SRCA_A1, `CS_SRCB_D0,
        `CS_SHFT_OP___SR, `CS_A0_SRC_NONE, `CS_A1_SRC___F1,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG5: Shift data, load A1 */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_D0,
        `CS_SHFT_OP___SL, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_ENBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG6: CRC-XOR */
        `CS_ALU_OP__XOR, `CS_SRCA_A0, `CS_SRCB_A0,
        `CS_SHFT_OP_PASS, `CS_A0_SRC__ALU, `CS_A1_SRC_NONE,
        `CS_FEEDBACK_DSBL, `CS_CI_SEL_CFGA, `CS_SI_SEL_CFGA,
        `CS_CMP_SEL_CFGA, /* CS_REG7: Reset CRC */

         8'hFF, 8'h00, /* SC_REG4: */
         8'hFF, 8'hFF, /* SC_REG5: */
        `SC_CMPB_A0_D1, `SC_CMPA_A0_D1, `SC_CI_B_ARITH,
        `SC_CI_A_ARITH, `SC_C1_MASK_DSBL, `SC_C0_MASK_DSBL,
        `SC_A_MASK_DSBL, `SC_DEF_SI_0, `SC_SI_B_ROUTE,
        `SC_SI_A_CHAIN, /* SC_REG6: Chained, A:Left shift, B: Right Shift */
        `SC_A0_SRC_ACC, `SC_SHIFT_SR, 1'b0,
         1'b0, `SC_FIFO1_BUS, `SC_FIFO0__A1,
        `SC_MSB_DSBL, `SC_MSB_BIT0, `SC_MSB_NOCHN,
        `SC_FB_CHNED, `SC_CMP1_CHNED,
        `SC_CMP0_NOCHN, /* SC_REG7: */
         10'h00, `SC_FIFO_CLK__DP,`SC_FIFO_CAP_FX,
        `SC_FIFO__EDGE,`SC_FIFO__SYNC,`SC_EXTCRC_DSBL,
        `SC_WRK16CAT_DSBL /* SC_REG8: */
    };

    always @(posedge clk) begin
        dp_clk_en     <= (clken[0] | clken[1]) & ~zero_ins;
        dp_cs_addr[1] <=  clken[1];
        dp_cs_addr[0] <= (tx_start & clken[0]) | (tx_start & clken[1]) |
                         (~tx_state_sync & ~tx_state_crc & bit_tc & clken[0]) | 
                         (~sdlc_dir & pkt_sync);
    end

    always @(posedge clk) dp_din  <= sdlc_dir ? so_lsb : rx_data;

    assign si       = (cmsb[1] & ~clken[1]) ^ dp_din;
    assign ci       = ~(tx_state_crc & sdlc_dir) & (~cmsb[1] ^ (dp_din));
    assign f0_load  = bit_tc & clken[2];

    cy_psoc3_dp16 #(
        .cy_dpconfig_a(dpconfig0), .cy_dpconfig_b(dpconfig1))
    dp (
        .clk(dp_clk),
        .cs_addr({sdlc_dir, dp_cs_addr}),
        .route_si(si),
        .route_ci(ci),
        .f0_load(f0_load),
        .f1_load(1'b0),
        .d0_load(1'b0),
        .d1_load(1'b0),

        .ce1(ce1),
        .cmsb(cmsb),
        .so(so),
        .f0_bus_stat(f0_bus_stat),
        .f1_bus_stat(f1_bus_stat)
    );

    /* Work around clock enable for simulation */
    generate
        if (sim) begin
            reg so_lsb_d;
            always @(negedge clk) so_lsb_d  <= so[0];
            assign so_lsb   = so_lsb_d;
        end else begin
            assign so_lsb   = so[0];
        end
    endgenerate

    /* -------- Byte marker ------------------------------------------- */
    wire bit_cnt_en = ~zero_ins & clken[2];

    cy_psoc3_count7 #(
        .cy_period(7'd15),
        .cy_alt_mode(`TRUE))
    bit_counter (
        .clock(clk),
        .reset(1'b0),
        .load(pkt_sync),
        .enable(bit_cnt_en),
        .tc(bit_tc)
    );


    /* -------- Zero insertion ---------------------------------------- */
    wire        zero_ins_cnt_rst = sdlc_dir ? ~tx_out : ~dp_din;
    wire        zero_ins_cnt_en  = sdlc_dir ? clken[3] : clken[2];
    wire  [6:0] zero_ins_cnt;

    cy_psoc3_count7 #(
        .cy_period(7'd6),
        .cy_alt_mode(`TRUE))
    zero_ins_counter (
        .clock(clk),
        .reset(1'b0),
        .load(zero_ins_cnt_rst),
        .enable(zero_ins_cnt_en),
        .count(zero_ins_cnt),
        .tc(pkt_sync)
    );

    assign zero_ins = (zero_ins_cnt[2:0] == 1);

endmodule

module sdlc (
        input  wire clk,
        input  wire rx_clk,
        input  wire rx_data,
        output wire sdlc_dir,
        output wire tx_clk,
        output wire tx_data,
        output wire tx_drq,
        output wire rx_valid,
        output wire rx_drq,
        output wire [7:0] dbg);

    parameter   sim = `FALSE;

    wire        dpll_sync_out;
    reg   [3:1] dpll_sync_out_d;
    wire  [3:0] dpll_sync;
    wire        rx_clk_gated;

    wire        tx_state_sync;
    wire        tx_zero_ins;
    wire        tx_state_crc;
    wire        tx_crc_rst;
    wire        tx_sync_data;

    wire        rx_os;

    wire  [7:0] dbg_int;

    assign dbg = {dbg_int[7:1], tx_clk};

    assign rx_clk_gated = rx_clk & ~sdlc_dir;
    assign dpll_sync    = {dpll_sync_out_d, dpll_sync_out};

    dpll #(.sim(sim)) dpll (
        .clk(clk),
        .sync_in(rx_clk_gated),
        .sync_out(dpll_sync_out),
        .clk_out(tx_clk)
    );

    always @(posedge clk) begin
        dpll_sync_out_d[1] <= dpll_sync_out;
        dpll_sync_out_d[2] <= dpll_sync_out_d[1];
        dpll_sync_out_d[3] <= dpll_sync_out_d[2];
    end

    sdlc_dp #(.sim(sim)) sdlc_dp (
        .clk(clk),
        .clken(dpll_sync),
        .sdlc_dir(sdlc_dir),
        .tx_start(tx_crc_rst),
        .tx_state_sync(tx_state_sync),
        .tx_sync_data(tx_sync_data),
        .tx_state_crc(tx_state_crc),
        .tx_zero_ins(tx_zero_ins),
        .tx_out(tx_data),
        .tx_drq(tx_drq),
        .rx_data(rx_os),
        .rx_valid(rx_valid),
        .rx_drq(rx_drq),
        .dbg(dbg_int)
    );

    tx_ctrl #(.sim(sim)) tx_ctrl (
        .clk(clk),
        .data_en(dpll_sync[2]),
        .zero_ins(tx_zero_ins),
        .sdlc_dir(sdlc_dir),
        .tx_crc_rst(tx_crc_rst),
        .tx_state_crc(tx_state_crc),
        .tx_state_sync(tx_state_sync),
        .tx_sync_data(tx_sync_data)
    );

    oversample oversample (
        .clk(clk),
        .sync_in(dpll_sync[1]),
        .data_in(rx_data),
        .data_out(rx_os)
    );

endmodule
