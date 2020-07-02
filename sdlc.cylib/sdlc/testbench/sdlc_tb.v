`timescale 1ns / 1ns

`include "sdlc.v"

module sdlc_tb();

    localparam CLK_PERIOD = 20.833;

    initial begin
        $dumpfile(``WAVE_FILE);
        $dumpvars;
        clk = 0;
        #(50_000 * CLK_PERIOD) $finish;
    end

    always #CLK_PERIOD clk <= !clk;

//    always @(clk) sdlc.sdlc_dp.dp.U1.cpu_clock <= ~clk;
//    always @(clk) sdlc.sdlc_dp.dp.U0.cpu_clock <= ~clk;
//    always @(clk) sdlc.tx_ctrl.dp.U0.cpu_clock <= ~clk;
    always @(clk) sdlc_tx.sdlc_dp.dp.U1.cpu_clock <= ~clk;
    always @(clk) sdlc_tx.sdlc_dp.dp.U0.cpu_clock <= ~clk;
    always @(clk) sdlc_tx.tx_ctrl.dp.U0.cpu_clock <= ~clk;

    initial begin
//        sdlc.dpll.dco.actl <= 1'b1;
        sdlc_tx.dpll.dco.actl <= 1'b1;

        /* Write polynomial */
//        sdlc.sdlc_dp.dp.U1.d0_write(8'h08);
//        sdlc.sdlc_dp.dp.U0.d0_write(8'h10);
        sdlc_tx.sdlc_dp.dp.U1.d0_write(8'h08);
        sdlc_tx.sdlc_dp.dp.U0.d0_write(8'h10);

        /* CRC match */
//        sdlc.sdlc_dp.dp.U1.d1_write(8'hE2);
//        sdlc.sdlc_dp.dp.U0.d1_write(8'hF0);
        sdlc_tx.sdlc_dp.dp.U1.d1_write(8'hE2);
        sdlc_tx.sdlc_dp.dp.U0.d1_write(8'hF0);

        /* Setup TX state machine */
//        sdlc.tx_ctrl.dp.U0.d0_write(8'd16);
//        sdlc.tx_ctrl.dp.U0.d1_write(8'h7E);
        sdlc_tx.tx_ctrl.dp.U0.d0_write(8'd16);
        sdlc_tx.tx_ctrl.dp.U0.d1_write(8'h7E);
    end

    reg         clk;
    reg         rx_data;
    reg         rx_clk;
    reg  [15:0] cpu_rd_data;
    reg         dummy;
    wire        rx_drq;
    wire        tx_drq;

    /*
    sdlc sdlc (
        .clk(clk),
        .rx_clk(rx_clk),
        .rx_data(rx_data),
        .rx_drq(rx_drq)
    );
    */

    sdlc sdlc_tx (
        .clk(clk),
        .rx_clk(rx_clk),
        .rx_data(rx_data),
        .tx_drq(tx_drq)
    );

    /* ------------ TX driver -------------------------------------------- */
    reg   [7:0] tx_data[9:0];
    reg   [3:0] tx_ptr = 0;

    initial begin
        #5500;
        tx_data[0] = 8'h01;
        tx_data[1] = 8'h23;
        tx_data[2] = 8'h45;
        tx_data[3] = 8'h67;
        tx_data[4] = 8'h89;
        tx_data[5] = 8'hAB;
        tx_data[6] = 8'hCD;
        tx_data[7] = 8'hEF;
        tx_data[8] = 8'hF4;
        tx_data[9] = 8'h32;

        sdlc_tx.tx_ctrl.dp.U0.a0_write(8'd87);
        sdlc_tx.tx_ctrl.dp.U0.fifo1_write(8'hFF);
        sdlc_tx.tx_ctrl.dp.U0.fifo1_write(8'hFF);
        sdlc_tx.tx_ctrl.dp.U0.fifo1_write(8'hFF);
        sdlc_tx.tx_ctrl.dp.U0.fifo1_write(8'h7E);

        forever wait (tx_drq) begin 
            sdlc_tx.sdlc_dp.dp.U1.fifo1_write(tx_data[tx_ptr + 1]);
            sdlc_tx.sdlc_dp.dp.U0.fifo1_write(tx_data[tx_ptr]);
            if (tx_ptr == 8)    tx_ptr = 0;
            else                tx_ptr = tx_ptr + 2;
        end
    end

`define RD_FIFO0 \
    @(posedge rx_drq);                                      \
    cpu_rd_data[15:8] <= sdlc.sdlc_dp.dp.U1.cpu_data_out0;  \
    cpu_rd_data[ 7:0] <= sdlc.sdlc_dp.dp.U0.cpu_data_out0;  \
    sdlc.sdlc_dp.dp.U1.fifo0_read(dummy);                   \
    sdlc.sdlc_dp.dp.U0.fifo0_read(dummy)

    /* ------------ RX driver -------------------------------------------- */
    /*
    initial begin
        `RD_FIFO0;
        `RD_FIFO0;
        `RD_FIFO0;
        `RD_FIFO0;
        `RD_FIFO0;
    end
    */

    /* ------------ Stimulus --------------------------------------------- */
    reg     stim_clk;
    integer data_file;
    integer scan_file;

    initial begin
        stim_clk <= 0;
        data_file = $fopen("../testbench/sdlc_dump.txt", "r");
        if (data_file == 0) begin
            $display("data_file handle was NULL");
            $finish;
        end
    end

    always #5 stim_clk <= !stim_clk;

    always @(posedge stim_clk) begin
        scan_file = $fscanf(data_file, "'%b''%b'\n", rx_data, rx_clk);
    end
 
endmodule
