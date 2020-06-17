`include "cypress.v"

module sdlc (
        input  wire clk,
        input  wire rx_data,
        input  wire rx_clk,
        output wire rx_rdy);

    dpll dpll (
        .clk(clk),
        .sync_in(rx_clk),
        .clken_out(rx_rdy)
    );

endmodule

module dpll (
        input  wire clk,
        input  wire sync_in,
        output wire clken_out);

    /* --------- VCO -------------------------------------------------- */

    wire early, late;
    wire [6:0] count;
    reg  clken, clken90, faster, slower;

    cy_psoc3_count7 #(
        .cy_period(7'd15),
        .cy_init_value(7'd13),
        .cy_alt_mode(`TRUE))
    clk_div (
        .clock(clk),
        .reset(faster),
        .load(slower),
        .enable(1'b1),
        .count(count),
        .tc(clken_out)
    );

    always @(posedge clk) begin
        clken   <= (count[2:0] == 0) ? 1'b1 : 1'b0;
        clken90 <= (count[2:0] == 4) ? 1'b1 : 1'b0;
        faster  <= (count[3:0] == 0) ? late : 1'b0;
        slower  <= (count[3:0] == 0) ? early : 1'b0;
    end

    /* -------- Phase detection --------------------------------------- */

    reg dff1_q, dff2_q, dff3_q, dff4_q;

    /* Sample data */
    always @(posedge clk) begin
        if (clken == 1'b1) dff3_q <= sync_in;
        if (clken90 == 1'b1) begin
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
