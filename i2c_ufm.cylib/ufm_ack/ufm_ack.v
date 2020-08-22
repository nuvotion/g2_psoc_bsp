`include "cypress.v"

module ufm_ack (
        input  wire clk,
        input  wire scl_in,
        input  wire sda_in,
        output wire sda_out);

    reg         scl_d;
    reg         sda_d;
    wire        count_en;
    wire        count_clk;
    reg   [2:0] count;
    reg         ack;
    reg         sda_ufm;

    assign count_en = (~scl_in & scl_d) | (~sda_in & sda_d & scl_d);

    always @(posedge clk) begin
        scl_d   <= scl_in;
        sda_d   <= sda_in;
        sda_ufm <= ack & sda_in;
    end

    cy_psoc3_udb_clock_enable_v1_0 #(.sync_mode(`TRUE))
    clk_enable (
        .clock_in(clk),
        .enable(count_en),
        .clock_out(count_clk)
    );

    always @(posedge count_clk) begin
        if (scl_in)     count <= 7;
        else if (ack)   count <= count - 1;
        
        if (scl_in)             ack <= 1'b0;
        else if (count == 0)    ack <= 1'b0;
        else                    ack <= 1'b1;
    end

    assign sda_out = sda_ufm;

endmodule
