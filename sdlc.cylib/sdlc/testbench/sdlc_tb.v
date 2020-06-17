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

    initial begin
        sdlc.dpll.clk_div.actl <= 1'b0;
        #7800
        sdlc.dpll.clk_div.actl <= 1'b1;
    end

    reg clk;
    reg rx_data;
    reg rx_clk;

    sdlc sdlc (
        .clk(clk),
        .rx_clk(rx_clk),
        .rx_data(rx_data)
    );

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
