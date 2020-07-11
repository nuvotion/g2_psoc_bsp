`timescale 1ns / 1ns

`include "sdlc.v"

module sdlc_tb();

    localparam CLK_PERIOD = 20.833;

    reg         clk;
    reg  [15:0] cpu_rd_data;
    reg         dummy;
    wire        tx_clk;
    wire        tx_data;
    wire        rx_drq;
    wire        tx_drq;

    initial begin
        $dumpfile(``WAVE_FILE);
        $dumpvars;
        clk = 0;
        #(12_000 * CLK_PERIOD) $finish;
    end

    always #CLK_PERIOD clk <= !clk;

    always @(clk) sdlc_tx.sdlc_dp.dp.U1.cpu_clock <= ~clk;
    always @(clk) sdlc_tx.sdlc_dp.dp.U0.cpu_clock <= ~clk;
    always @(clk) sdlc_tx.tx_ctrl.dp.U0.cpu_clock <= ~clk;
    always @(clk) sdlc_rx.sdlc_dp.dp.U1.cpu_clock <= ~clk;
    always @(clk) sdlc_rx.sdlc_dp.dp.U0.cpu_clock <= ~clk;
    always @(clk) sdlc_rx.tx_ctrl.dp.U0.cpu_clock <= ~clk;

    initial begin
        /* Force init values for sim */
        sdlc_tx.sdlc_dp.tx_out = 1'b0;
        sdlc_tx.tx_ctrl.state = 0;
        sdlc_rx.tx_ctrl.state = 0;

        sdlc_tx.dpll.dco.actl <= 1'b1;
        sdlc_rx.dpll.dco.actl <= 1'b0;
        #1000;
        sdlc_rx.dpll.dco.actl <= 1'b1;

        /* Write polynomial */
        sdlc_tx.sdlc_dp.dp.U1.d0_write(8'h08);
        sdlc_tx.sdlc_dp.dp.U0.d0_write(8'h10);
        sdlc_rx.sdlc_dp.dp.U1.d0_write(8'h08);
        sdlc_rx.sdlc_dp.dp.U0.d0_write(8'h10);

        /* CRC match */
        sdlc_tx.sdlc_dp.dp.U1.d1_write(8'hE2);
        sdlc_tx.sdlc_dp.dp.U0.d1_write(8'hF0);
        sdlc_rx.sdlc_dp.dp.U1.d1_write(8'hE2);
        sdlc_rx.sdlc_dp.dp.U0.d1_write(8'hF0);

        /* Setup TX state machine */
        sdlc_tx.tx_ctrl.dp.U0.d0_write(8'd16);
        sdlc_tx.tx_ctrl.dp.U0.d1_write(8'h7E);
        sdlc_rx.tx_ctrl.dp.U0.d0_write(8'd16);
        sdlc_rx.tx_ctrl.dp.U0.d1_write(8'h7E);
    end

    sdlc #(.sim(`TRUE)) sdlc_tx (
        .clk(clk),
        .rx_clk(1'b0),
        .rx_data(1'b0),
        .tx_clk(tx_clk),
        .tx_data(tx_data),
        .tx_drq(tx_drq)
    );

    sdlc #(.sim(`TRUE)) sdlc_rx (
        .clk(clk),
        .rx_clk(tx_clk),
        .rx_data(tx_data),
        .rx_drq(rx_drq)
    );

    /* ------------ TX driver -------------------------------------------- */
    reg   [7:0] tx_msg[13:0];
    reg   [3:0] tx_ptr = 0;

    initial begin
        #5500;
        tx_msg[ 0] = 8'h01;
        tx_msg[ 1] = 8'h23;
        tx_msg[ 2] = 8'h45;
        tx_msg[ 3] = 8'h67;
        tx_msg[ 4] = 8'h89;
        tx_msg[ 5] = 8'hAB;
        tx_msg[ 6] = 8'hCD;
        tx_msg[ 7] = 8'hEF;
        tx_msg[ 8] = 8'hCD;
        tx_msg[ 9] = 8'hEF;
        tx_msg[10] = 8'hF4;
        tx_msg[11] = 8'h32;
        tx_msg[12] = 8'hF4;
        tx_msg[13] = 8'h32;

        //sdlc_tx.tx_ctrl.dp.U0.a0_write(8'd87);
        sdlc_tx.tx_ctrl.dp.U0.a0_write(8'd71);
        sdlc_tx.tx_ctrl.dp.U0.fifo1_write(8'hFF);
        sdlc_tx.tx_ctrl.dp.U0.fifo1_write(8'hFF);
        sdlc_tx.tx_ctrl.dp.U0.fifo1_write(8'hFF);
        sdlc_tx.tx_ctrl.dp.U0.fifo1_write(8'h7E);

        sdlc_tx.sdlc_dp.dp.U1.fifo1_write(8'h00);
        sdlc_tx.sdlc_dp.dp.U0.fifo1_write(8'h15);
        sdlc_tx.sdlc_dp.dp.U1.fifo1_write(8'h1B);
        sdlc_tx.sdlc_dp.dp.U0.fifo1_write(8'h00);
        sdlc_tx.sdlc_dp.dp.U1.fifo1_write(8'h00);
        sdlc_tx.sdlc_dp.dp.U0.fifo1_write(8'h85);
        sdlc_tx.sdlc_dp.dp.U1.fifo1_write(8'hFF);
        sdlc_tx.sdlc_dp.dp.U0.fifo1_write(8'h00);

        #100_000
        sdlc_tx.tx_ctrl.dp.U0.a0_write(8'd71);

        sdlc_tx.sdlc_dp.dp.U1.fifo1_write(8'h00);
        sdlc_tx.sdlc_dp.dp.U0.fifo1_write(8'h15);
        sdlc_tx.sdlc_dp.dp.U1.fifo1_write(8'h00);
        sdlc_tx.sdlc_dp.dp.U0.fifo1_write(8'h00);
        sdlc_tx.sdlc_dp.dp.U1.fifo1_write(8'h00);
        sdlc_tx.sdlc_dp.dp.U0.fifo1_write(8'h00);
        sdlc_tx.sdlc_dp.dp.U1.fifo1_write(8'hFF);
        sdlc_tx.sdlc_dp.dp.U0.fifo1_write(8'h00);

        sdlc_tx.tx_ctrl.dp.U0.fifo1_write(8'hFF);
        sdlc_tx.tx_ctrl.dp.U0.fifo1_write(8'hFF);
        sdlc_tx.tx_ctrl.dp.U0.fifo1_write(8'hFF);
        sdlc_tx.tx_ctrl.dp.U0.fifo1_write(8'h7E);


        /*
        forever wait (tx_drq) begin 
            sdlc_tx.sdlc_dp.dp.U1.fifo1_write(tx_msg[tx_ptr + 1]);
            sdlc_tx.sdlc_dp.dp.U0.fifo1_write(tx_msg[tx_ptr]);
            if (tx_ptr == 12)   tx_ptr = 0;
            else                tx_ptr = tx_ptr + 2;
        end
        */
    end

`define RD_FIFO0 \
    @(posedge rx_drq);                                          \
    cpu_rd_data[15:8] <= sdlc_rx.sdlc_dp.dp.U1.cpu_data_out0;   \
    cpu_rd_data[ 7:0] <= sdlc_rx.sdlc_dp.dp.U0.cpu_data_out0;   \
    sdlc_rx.sdlc_dp.dp.U1.fifo0_read(dummy);                    \
    sdlc_rx.sdlc_dp.dp.U0.fifo0_read(dummy)

    /* ------------ RX driver -------------------------------------------- */
    initial begin
        `RD_FIFO0;
        `RD_FIFO0;
        `RD_FIFO0;
        `RD_FIFO0;
        `RD_FIFO0;
        `RD_FIFO0;
    end

endmodule
