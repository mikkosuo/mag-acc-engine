// mae_fifo.v -- FIFO used by mag_acc_engine

`default_nettype none

// Synchronous FIFO, any DEPTH >= 1, first-word fall-through (dout valid while !empty).
// Simultaneous push and pop are allowed, also when full.
module mae_fifo #(
    parameter integer WIDTH = 32,
    parameter integer DEPTH = 32
) (
    input  wire                         clock,
    input  wire                         reset,
    input  wire                         push,
    input  wire [WIDTH-1:0]             din,
    input  wire                         pop,
    output wire [WIDTH-1:0]             dout,
    output wire                         empty,
    output reg  [$clog2(DEPTH+1)-1:0]   count
);
    localparam integer PW = (DEPTH > 1) ? $clog2(DEPTH) : 1;

    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [PW-1:0]    rptr, wptr;

    assign dout  = mem[rptr];
    assign empty = (count == 0);

    wire do_pop  = pop && !empty;
    localparam integer               LAST_I = DEPTH - 1;
    localparam [$clog2(DEPTH+1)-1:0] FULL   = DEPTH[$clog2(DEPTH+1)-1:0];
    localparam [PW-1:0]              LAST   = LAST_I[PW-1:0];

    wire do_push = push && ((count != FULL) || do_pop);

    always @(posedge clock) begin
        if (reset) begin
            rptr  <= {PW{1'b0}};
            wptr  <= {PW{1'b0}};
            count <= 0;
        end else begin
            if (do_push) begin
                mem[wptr] <= din;
                wptr <= (wptr == LAST) ? {PW{1'b0}} : wptr + 1'b1;
            end
            if (do_pop)
                rptr <= (rptr == LAST) ? {PW{1'b0}} : rptr + 1'b1;
            case ({do_push, do_pop})
                2'b10:   count <= count + 1'b1;
                2'b01:   count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end
endmodule


`default_nettype wire
