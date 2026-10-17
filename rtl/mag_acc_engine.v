// mag_acc_engine.v -- magnitude-accumulate engine for non-coherent integration
//
// One job processes LEN consecutive 32-bit words:
//
//     X2[i] := (MODE ? X2[i] : 0) + (cordic_mag(X1[i]) >> SHIFT)        (mod 2^32)
//
// X1[i] packs I = bits[15:0], Q = bits[31:16] (int16 each). cordic_mag is a
// vectoring CORDIC, N_ITER iterations with GUARD extra LSBs, whose output lies
// within 0.6 LSB of K_N * |I + jQ| at the defaults; K_N ~= 1.64676 (not compensated).
// MODE = 0 (overwrite) initialises the accumulator for the first pulse of a CPI,
// MODE = 1 (accumulate) is used for the remaining pulses.
//
// Interfaces (all AXI4-Lite, single clock, active-high synchronous reset):
//   s_axil_*  configuration slave, register map below
//   m_src_*   read-only master that fetches X1
//   m_dst_*   read/write master that fetches X2 (MODE = 1 only) and writes it back
// Each master keeps up to FIFO_DEPTH reads outstanding; responses must return in
// order (AXI4-Lite has no IDs). The datapath sustains one element per cycle when
// both masters see single-cycle memory.
//
// Register map (byte offsets, 32-bit registers):
//   0x00 CTRL    [0] START (W, self-clearing; ignored while BUSY)
//                [1] MODE  (RW) 0 = overwrite, 1 = accumulate
//                [2] IRQ_EN(RW) irq = DONE & IRQ_EN
//   0x04 STATUS  [0] BUSY (RO)  [1] DONE (W1C)  [2] ERR (W1C, any non-OKAY master response)
//                DONE and ERR are also cleared when a job starts.
//   0x08 SRC     X1 base address (bits [1:0] ignored, word aligned)
//   0x0C DST     X2 base address (bits [1:0] ignored, word aligned)
//   0x10 LEN     number of elements; LEN = 0 completes immediately
//   0x14 SHIFT   [4:0] right shift applied to the magnitude
//   0x18 INFO    (RO) [7:0] N_ITER, [15:8] GUARD, [31:16] version
//   0x1C CYCLES  (RO) clock cycles the last job was BUSY
// SRC, DST, LEN, SHIFT and MODE are latched at START; writing them while BUSY
// only affects the next job. The X1 and X2 ranges must not overlap.

`default_nettype none

module mag_acc_engine #(
    parameter integer N_ITER     = 11,
    parameter integer GUARD      = 6,
    parameter integer FIFO_DEPTH = 32,
    parameter integer S_ADDR_W   = 8
) (
    input  wire                 clock,
    input  wire                 reset,

    // ---- AXI4-Lite configuration slave
    input  wire [S_ADDR_W-1:0]  s_axil_awaddr,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [2:0]           s_axil_awprot,   // unused: no protection levels
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire                 s_axil_awvalid,
    output wire                 s_axil_awready,
    input  wire [31:0]          s_axil_wdata,
    input  wire [3:0]           s_axil_wstrb,
    input  wire                 s_axil_wvalid,
    output wire                 s_axil_wready,
    output wire [1:0]           s_axil_bresp,
    output reg                  s_axil_bvalid,
    input  wire                 s_axil_bready,
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [S_ADDR_W-1:0]  s_axil_araddr,   // [1:0] ignored: word registers
    /* verilator lint_on UNUSEDSIGNAL */
    /* verilator lint_off UNUSEDSIGNAL */
    input  wire [2:0]           s_axil_arprot,   // unused: no protection levels
    /* verilator lint_on UNUSEDSIGNAL */
    input  wire                 s_axil_arvalid,
    output wire                 s_axil_arready,
    output reg  [31:0]          s_axil_rdata,
    output wire [1:0]           s_axil_rresp,
    output reg                  s_axil_rvalid,
    input  wire                 s_axil_rready,

    // ---- AXI4-Lite source master (read only)
    output wire [31:0]          m_src_araddr,
    output wire [2:0]           m_src_arprot,
    output wire                 m_src_arvalid,
    input  wire                 m_src_arready,
    input  wire [31:0]          m_src_rdata,
    input  wire [1:0]           m_src_rresp,
    input  wire                 m_src_rvalid,
    output wire                 m_src_rready,

    // ---- AXI4-Lite destination master (read + write)
    output wire [31:0]          m_dst_araddr,
    output wire [2:0]           m_dst_arprot,
    output wire                 m_dst_arvalid,
    input  wire                 m_dst_arready,
    input  wire [31:0]          m_dst_rdata,
    input  wire [1:0]           m_dst_rresp,
    input  wire                 m_dst_rvalid,
    output wire                 m_dst_rready,
    output wire [31:0]          m_dst_awaddr,
    output wire [2:0]           m_dst_awprot,
    output wire                 m_dst_awvalid,
    input  wire                 m_dst_awready,
    output wire [31:0]          m_dst_wdata,
    output wire [3:0]           m_dst_wstrb,
    output wire                 m_dst_wvalid,
    input  wire                 m_dst_wready,
    input  wire [1:0]           m_dst_bresp,
    input  wire                 m_dst_bvalid,
    output wire                 m_dst_bready,

    output wire                 irq
);

    localparam [15:0] VERSION  = 16'h0001;
    localparam integer W        = 19 + GUARD;  // signed CORDIC datapath width (see model check_widths)
    localparam integer MAG_BITS = 17;          // cordic_mag output < 2^17 for any int16 pair
    localparam integer CNT_W    = $clog2(FIFO_DEPTH + 1);
    localparam [7:0]   N_ITER_B = N_ITER[7:0];
    localparam [7:0]   GUARD_B  = GUARD[7:0];
    localparam integer RND      = (GUARD > 0) ? (1 << (GUARD - 1)) : 0;  // round half up

    // =====================================================================
    // Configuration registers
    // =====================================================================
    reg        mode_r, irq_en_r;
    reg        busy, done_r, err_r;
    reg [31:0] src_r, dst_r, len_r;
    reg [4:0]  shift_r;
    reg [31:0] cycles_r;

    // job registers, latched at START
    reg [31:0] job_src, job_dst, job_len;
    reg [4:0]  job_shift;
    reg        job_mode;

    function [31:0] apply_strb(input [31:0] old_v, input [31:0] new_v, input [3:0] strb);
        integer b;
        begin
            for (b = 0; b < 4; b = b + 1)
                apply_strb[8*b +: 8] = strb[b] ? new_v[8*b +: 8] : old_v[8*b +: 8];
        end
    endfunction

    // ---- write channel: AW and W are accepted independently, then committed together
    reg                aw_got, w_got;
    /* verilator lint_off UNUSEDSIGNAL */
    reg [S_ADDR_W-1:0] aw_addr;
    /* verilator lint_on UNUSEDSIGNAL */
    reg [31:0]         w_data;
    reg [3:0]          w_strb;

    assign s_axil_awready = !aw_got;
    assign s_axil_wready  = !w_got;
    assign s_axil_bresp   = 2'b00;

    wire        wr_commit = aw_got && w_got && !s_axil_bvalid;
    wire [S_ADDR_W-3:0] wr_word = aw_addr[S_ADDR_W-1:2];
    wire [S_ADDR_W-3:0] rd_word = s_axil_araddr[S_ADDR_W-1:2];
    wire        start_req = wr_commit && (wr_word == 0) && w_strb[0] && w_data[0];
    wire        launch    = start_req && !busy;
    wire        mode_new  = w_strb[0] ? w_data[1] : mode_r;

    // ---- read channel
    assign s_axil_arready = !s_axil_rvalid;
    assign s_axil_rresp   = 2'b00;

    reg [31:0] rd_mux;
    always @(*) begin
        case (rd_word)
            0:    rd_mux = {29'd0, irq_en_r, mode_r, 1'b0};
            1:    rd_mux = {29'd0, err_r, done_r, busy};
            2:    rd_mux = src_r;
            3:    rd_mux = dst_r;
            4:    rd_mux = len_r;
            5:    rd_mux = {27'd0, shift_r};
            6:    rd_mux = {VERSION, GUARD_B, N_ITER_B};
            7:    rd_mux = cycles_r;
            default: rd_mux = 32'd0;
        endcase
    end

    always @(posedge clock) begin
        if (reset) begin
            s_axil_rvalid <= 1'b0;
            s_axil_rdata  <= 32'd0;
        end else begin
            if (s_axil_arvalid && s_axil_arready) begin
                s_axil_rvalid <= 1'b1;
                s_axil_rdata  <= rd_mux;
            end else if (s_axil_rvalid && s_axil_rready) begin
                s_axil_rvalid <= 1'b0;
            end
        end
    end

    // =====================================================================
    // Engine: counters and credits
    // =====================================================================
    reg [31:0] src_issued;   // X1 read addresses accepted
    reg [31:0] dst_issued;   // X2 read addresses accepted (MODE = 1)
    reg [31:0] pushed;       // elements that entered the CORDIC pipeline
    reg [31:0] written;      // elements whose AW and W have both been accepted
    reg [31:0] bcnt;         // write responses received
    reg        aw_done, w_done;

    // FIFO interfaces
    wire               src_empty, dst_empty, res_empty;
    wire [31:0]        src_head, dst_head;
    wire [MAG_BITS-1:0] res_head;
    /* verilator lint_off UNUSEDSIGNAL */
    wire [CNT_W-1:0]   src_count, dst_count, res_count;   // observed by the testbench only
    /* verilator lint_on UNUSEDSIGNAL */

    // A read is issued only while (issued - consumed) < FIFO_DEPTH, so the response
    // FIFO can always take the data and rready may be held high.
    wire src_ar_ok = busy && (src_issued != job_len) && ((src_issued - pushed)  < FIFO_DEPTH);
    wire dst_ar_ok = busy && job_mode && (dst_issued != job_len) && ((dst_issued - written) < FIFO_DEPTH);

    assign m_src_arvalid = src_ar_ok;
    assign m_src_araddr  = job_src + {src_issued[29:0], 2'b00};
    assign m_src_arprot  = 3'b000;
    assign m_src_rready  = 1'b1;

    assign m_dst_arvalid = dst_ar_ok;
    assign m_dst_araddr  = job_dst + {dst_issued[29:0], 2'b00};
    assign m_dst_arprot  = 3'b000;
    assign m_dst_rready  = 1'b1;

    wire src_ar_hs = m_src_arvalid && m_src_arready;
    wire dst_ar_hs = m_dst_arvalid && m_dst_arready;
    wire src_r_hs  = m_src_rvalid;           // rready is constant 1
    wire dst_r_hs  = m_dst_rvalid;

    // An element enters the pipeline when its X1 word is available and the
    // pipeline + result FIFO hold fewer than FIFO_DEPTH elements in flight.
    wire push_fire = busy && !src_empty && (pushed != job_len) && ((pushed - written) < FIFO_DEPTH);

    // Write stage: result and (MODE = 1) the old X2 value pair up in element order.
    wire               wr_ready  = busy && !res_empty && (!job_mode || !dst_empty);
    wire [31:0]        acc_old   = job_mode ? dst_head : 32'd0;
    wire [31:0]        acc_new   = acc_old + {{(32-MAG_BITS){1'b0}}, res_head};

    assign m_dst_awvalid = wr_ready && !aw_done;
    assign m_dst_awaddr  = job_dst + {written[29:0], 2'b00};
    assign m_dst_awprot  = 3'b000;
    assign m_dst_wvalid  = wr_ready && !w_done;
    assign m_dst_wdata   = acc_new;
    assign m_dst_wstrb   = 4'b1111;
    assign m_dst_bready  = 1'b1;

    wire aw_hs   = m_dst_awvalid && m_dst_awready;
    wire w_hs    = m_dst_wvalid  && m_dst_wready;
    wire wr_fire = wr_ready && (aw_done || aw_hs) && (w_done || w_hs);
    wire b_hs    = m_dst_bvalid;

    // =====================================================================
    // FIFOs
    // =====================================================================
    mae_fifo #(.WIDTH(32), .DEPTH(FIFO_DEPTH)) u_src_fifo (
        .clock(clock), .reset(reset),
        .push(src_r_hs), .din(m_src_rdata),
        .pop(push_fire), .dout(src_head),
        .empty(src_empty), .count(src_count));

    mae_fifo #(.WIDTH(32), .DEPTH(FIFO_DEPTH)) u_dst_fifo (
        .clock(clock), .reset(reset),
        .push(dst_r_hs), .din(m_dst_rdata),
        .pop(wr_fire && job_mode), .dout(dst_head),
        .empty(dst_empty), .count(dst_count));

    reg                 out_v;
    reg [MAG_BITS-1:0]  out_d;

    mae_fifo #(.WIDTH(MAG_BITS), .DEPTH(FIFO_DEPTH)) u_res_fifo (
        .clock(clock), .reset(reset),
        .push(out_v), .din(out_d),
        .pop(wr_fire), .dout(res_head),
        .empty(res_empty), .count(res_count));

    // =====================================================================
    // CORDIC pipeline: stage 0 = fold, stages 1..N_ITER = iterations, then output
    // =====================================================================
    reg signed [W-1:0] px [0:N_ITER];
    reg signed [W-1:0] py [0:N_ITER];
    reg                pv [0:N_ITER];

    wire signed [W-1:0] i_ext = {{(W-16){src_head[15]}}, src_head[15:0]};
    wire signed [W-1:0] q_ext = {{(W-16){src_head[31]}}, src_head[31:16]};
    wire signed [W-1:0] i_abs = i_ext[W-1] ? -i_ext : i_ext;

    always @(posedge clock) begin
        if (reset) begin
            pv[0] <= 1'b0;
        end else begin
            pv[0] <= push_fire;
            if (push_fire) begin
                px[0] <= i_abs <<< GUARD;
                py[0] <= q_ext <<< GUARD;
            end
        end
    end

    genvar k;
    generate
        for (k = 0; k < N_ITER; k = k + 1) begin : g_iter
            wire               y_pos = !py[k][W-1];
            wire signed [W-1:0] xs   = px[k] >>> k;
            wire signed [W-1:0] ys   = py[k] >>> k;
            always @(posedge clock) begin
                if (reset) begin
                    pv[k+1] <= 1'b0;
                end else begin
                    pv[k+1] <= pv[k];
                    if (pv[k]) begin
                        px[k+1] <= y_pos ? (px[k] + ys) : (px[k] - ys);
                        py[k+1] <= y_pos ? (py[k] - xs) : (py[k] + xs);
                    end
                end
            end
        end
    endgenerate

    wire signed [W-1:0]  x_rnd   = px[N_ITER] + RND[W-1:0];
    /* verilator lint_off UNUSEDSIGNAL */
    wire        [W-1:0]  x_scl   = x_rnd >>> GUARD;          // x >= 0, so this equals a logical shift
    /* verilator lint_on UNUSEDSIGNAL */   // upper bits are provably zero (model check_widths)
    wire [MAG_BITS-1:0]  mag     = x_scl[MAG_BITS-1:0];
    wire [MAG_BITS-1:0]  mag_shr = mag >> job_shift;

    always @(posedge clock) begin
        if (reset) begin
            out_v <= 1'b0;
            out_d <= {MAG_BITS{1'b0}};
        end else begin
            out_v <= pv[N_ITER];
            if (pv[N_ITER]) out_d <= mag_shr;
        end
    end

    // =====================================================================
    // Control state
    // =====================================================================
    always @(posedge clock) begin
        if (reset) begin
            aw_got <= 1'b0; w_got <= 1'b0; s_axil_bvalid <= 1'b0;
            aw_addr <= {S_ADDR_W{1'b0}}; w_data <= 32'd0; w_strb <= 4'd0;
            mode_r <= 1'b0; irq_en_r <= 1'b0;
            busy <= 1'b0; done_r <= 1'b0; err_r <= 1'b0;
            src_r <= 32'd0; dst_r <= 32'd0; len_r <= 32'd0; shift_r <= 5'd0;
            cycles_r <= 32'd0;
            job_src <= 32'd0; job_dst <= 32'd0; job_len <= 32'd0; job_shift <= 5'd0; job_mode <= 1'b0;
            src_issued <= 32'd0; dst_issued <= 32'd0; pushed <= 32'd0; written <= 32'd0; bcnt <= 32'd0;
            aw_done <= 1'b0; w_done <= 1'b0;
        end else begin
            // ---- slave write path
            if (s_axil_awvalid && s_axil_awready) begin aw_got <= 1'b1; aw_addr <= s_axil_awaddr; end
            if (s_axil_wvalid && s_axil_wready)   begin w_got <= 1'b1; w_data <= s_axil_wdata; w_strb <= s_axil_wstrb; end
            if (s_axil_bvalid && s_axil_bready)   s_axil_bvalid <= 1'b0;

            if (wr_commit) begin
                aw_got <= 1'b0; w_got <= 1'b0; s_axil_bvalid <= 1'b1;
                case (wr_word)
                    0: if (w_strb[0]) begin mode_r <= w_data[1]; irq_en_r <= w_data[2]; end
                    1: if (w_strb[0]) begin
                              if (w_data[1]) done_r <= 1'b0;
                              if (w_data[2]) err_r  <= 1'b0;
                          end
                    2: src_r   <= apply_strb(src_r, w_data, w_strb);
                    3: dst_r   <= apply_strb(dst_r, w_data, w_strb);
                    4: len_r   <= apply_strb(len_r, w_data, w_strb);
                    5: if (w_strb[0]) shift_r <= w_data[4:0];
                    default: ;
                endcase
            end

            // ---- engine progress
            if (src_ar_hs) src_issued <= src_issued + 32'd1;
            if (dst_ar_hs) dst_issued <= dst_issued + 32'd1;
            if (push_fire) pushed     <= pushed + 32'd1;
            if (b_hs)      bcnt       <= bcnt + 32'd1;

            if (wr_fire) begin
                written <= written + 32'd1;
                aw_done <= 1'b0;
                w_done  <= 1'b0;
            end else begin
                if (aw_hs) aw_done <= 1'b1;
                if (w_hs)  w_done  <= 1'b1;
            end

            if ((src_r_hs && m_src_rresp != 2'b00) ||
                (dst_r_hs && m_dst_rresp != 2'b00) ||
                (b_hs     && m_dst_bresp != 2'b00))
                err_r <= 1'b1;

            if (busy) cycles_r <= cycles_r + 32'd1;

            if (busy && bcnt == job_len) begin
                busy   <= 1'b0;
                done_r <= 1'b1;
            end

            // ---- job launch (only reachable while !busy, so it cannot race the above)
            if (launch) begin
                job_src    <= {src_r[31:2], 2'b00};
                job_dst    <= {dst_r[31:2], 2'b00};
                job_len    <= len_r;
                job_shift  <= shift_r;
                job_mode   <= mode_new;
                src_issued <= 32'd0; dst_issued <= 32'd0; pushed <= 32'd0;
                written    <= 32'd0; bcnt <= 32'd0;
                aw_done    <= 1'b0;  w_done <= 1'b0;
                cycles_r   <= 32'd0;
                err_r      <= 1'b0;
                busy       <= (len_r != 32'd0);
                done_r     <= (len_r == 32'd0);
            end
        end
    end

    assign irq = done_r && irq_en_r;

endmodule


`default_nettype wire
