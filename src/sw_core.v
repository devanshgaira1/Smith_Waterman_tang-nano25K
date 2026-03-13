`timescale 1ns / 1ps

module sw_core #(
    parameter MAX_D = 1024
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        seq_we,
    input  wire [11:0] seq_addr,
    input  wire [3:0]  seq_din,
    input  wire [10:0] lenT,
    input  wire [10:0] lenD,
    input  wire        start,
    output reg         done,
    output reg  [15:0] score_out
);

localparam signed [15:0] MATCH    =  16'sd2;
localparam signed [15:0] MISMATCH = -16'sd1;
localparam signed [15:0] GAP_OPEN = -16'sd4;
localparam signed [15:0] GAP_EXT  = -16'sd1;
localparam signed [15:0] NEG_INF  = -16'sd16383;

localparam BDEPTH = 3 * MAX_D;

/* ================================
   Sequence Memories
================================ */

reg [3:0] target_mem [0:MAX_D-1];
reg [3:0] db_mem     [0:MAX_D-1];

always @(posedge clk) begin
    if (seq_we) begin
        if (seq_addr < MAX_D)
            target_mem[seq_addr] <= seq_din;
        else if (seq_addr < 2*MAX_D)
            db_mem[seq_addr - MAX_D] <= seq_din;
    end
end

/* ================================
   Score RAM (3 rotating diagonals)
================================ */

(* ram_style = "block" *) reg signed [15:0] P_ram  [0:BDEPTH-1];
(* ram_style = "block" *) reg signed [15:0] Q_ram  [0:BDEPTH-1];
(* ram_style = "block" *) reg signed [15:0] Da_ram [0:BDEPTH-1];
(* ram_style = "block" *) reg signed [15:0] Db_ram [0:BDEPTH-1];

reg signed [15:0] P_q, Q_q, Da_q, Db_q;
reg [11:0] P_ra, Q_ra, Da_ra, Db_ra;

reg        ram_we;
reg [11:0] ram_wa;
reg signed [15:0] P_wd, Q_wd, D_wd;

always @(posedge clk) begin
    P_q  <= P_ram [P_ra];
    Q_q  <= Q_ram [Q_ra];
    Da_q <= Da_ram[Da_ra];
    Db_q <= Db_ram[Db_ra];

    if (ram_we) begin
        P_ram [ram_wa] <= P_wd;
        Q_ram [ram_wa] <= Q_wd;
        Da_ram[ram_wa] <= D_wd;
        Db_ram[ram_wa] <= D_wd;
    end
end

/* ================================
   FSM
================================ */

localparam IDLE=3'd0, INIT=3'd1, NEWDIAG=3'd2,
           PREFETCH=3'd3, LOAD=3'd4, COMPUTE=3'd5,
           ENDDIAG=3'd6, FINISH=3'd7;

reg [2:0] state;

/* ================================
   Control Registers
================================ */

reg signed [15:0] num_diag, t_diag, diag_len;
reg signed [15:0] database_cursor, target_cursor;
reg [1:0]         diag_idx;
reg signed [15:0] j_cnt;
reg [11:0]        init_cnt;
reg signed [15:0] max_score, local_max;

/* ================================
   Length Extension (SAFE)
================================ */

wire signed [15:0] lenT_ext = {5'd0, lenT};
wire signed [15:0] lenD_ext = {5'd0, lenD};

/* ================================
   Next Diagonal Math (WIDTH SAFE)
================================ */

wire signed [15:0] next_db =
    (num_diag > (lenT_ext - 16'sd1)) ?
    (num_diag - lenT_ext + 16'sd1) :
    16'sd0;

wire signed [15:0] next_tc = num_diag - next_db;

wire signed [15:0] next_je =
    (num_diag < lenD_ext) ?
    num_diag :
    (lenD_ext - 16'sd1);

wire signed [15:0] next_dl =
    next_je - next_db + 16'sd1;

/* ================================
   Rotating Bases (WIDTH SAFE)
================================ */

wire [1:0] one_diag =
    (diag_idx==2'd0) ? 2'd2 :
    (diag_idx==2'd1) ? 2'd0 : 2'd1;

wire [1:0] two_diag =
    (diag_idx==2'd0) ? 2'd1 :
    (diag_idx==2'd1) ? 2'd2 : 2'd0;

wire [11:0] one_base =
    (one_diag==2'd0) ? 12'd0 :
    (one_diag==2'd1) ? 12'd1024 :
                       12'd2048;

wire [11:0] two_base =
    (two_diag==2'd0) ? 12'd0 :
    (two_diag==2'd1) ? 12'd1024 :
                       12'd2048;

wire [11:0] cur_base =
    (diag_idx==2'd0) ? 12'd0 :
    (diag_idx==2'd1) ? 12'd1024 :
                       12'd2048;

/* ================================
   Address Logic
================================ */

wire signed [15:0] cur_idx_full =
    database_cursor + j_cnt;

wire [11:0] cur_pos =
    cur_idx_full[11:0];

wire [11:0] left_pos =
    (cur_idx_full > 16'sd0) ?
    (cur_idx_full[11:0] - 12'd1) :
    12'd0;

wire [11:0] addr_up     = one_base + cur_pos;
wire [11:0] addr_left   = one_base + left_pos;
wire [11:0] addr_upleft = two_base + left_pos;
wire [11:0] addr_cur    = cur_base + cur_pos;

/* ================================
   Nucleotide Fetch
================================ */

wire [10:0] t_idx = target_cursor[10:0] - j_cnt[10:0];

wire [3:0] t_nuc = target_mem[t_idx];
wire [3:0] d_nuc = db_mem[cur_idx_full[10:0]];

wire        is_pad = (t_nuc==4'd4) || (d_nuc==4'd4);
wire signed [15:0] mval =
    (t_nuc == d_nuc) ? MATCH : MISMATCH;

/* ================================
   Pipeline Registers
================================ */

reg        is_pad_p;
reg signed [15:0] mval_p;
reg [11:0] waddr_p;

/* ================================
   Recurrence (WIDTH SAFE)
================================ */

wire signed [15:0] new_P =
    (Da_q + GAP_OPEN > P_q + GAP_EXT) ?
    (Da_q + GAP_OPEN) :
    (P_q + GAP_EXT);

wire signed [15:0] new_Q =
    (Da_q + GAP_OPEN > Q_q + GAP_EXT) ?
    (Da_q + GAP_OPEN) :
    (Q_q + GAP_EXT);

wire signed [15:0] ds1 = Db_q + mval_p;
wire signed [15:0] ds2 = (ds1 > 16'sd0) ? ds1 : 16'sd0;
wire signed [15:0] ds3 = (ds2 > new_P) ? ds2 : new_P;

wire signed [15:0] new_D =
    (ds3 > new_Q) ? ds3 : new_Q;

/* ================================
   FSM Logic
================================ */

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        done <= 1'b0;
        score_out <= 16'd0;
        ram_we <= 1'b0;
        num_diag <= 16'sd0;
        diag_idx <= 2'd0;
        max_score <= 16'sd0;
        local_max <= 16'sd0;
        init_cnt <= 12'd0;
    end
    else begin
        done <= 1'b0;
        ram_we <= 1'b0;

        case (state)

        IDLE:
            if (start) begin
                t_diag <= lenT_ext + lenD_ext - 16'sd1;
                num_diag <= 16'sd0;
                diag_idx <= 2'd0;
                max_score <= 16'sd0;
                init_cnt <= 12'd0;
                state <= INIT;
            end

        INIT:
            begin
                ram_we <= 1'b1;
                ram_wa <= init_cnt;
                P_wd <= NEG_INF;
                Q_wd <= NEG_INF;
                D_wd <= 16'sd0;

                if (init_cnt == BDEPTH-1)
                    state <= NEWDIAG;
                else
                    init_cnt <= init_cnt + 12'd1;
            end

        NEWDIAG:
            begin
                j_cnt <= 16'sd0;
                local_max <= 16'sd0;

                if (num_diag >= t_diag) begin
                    score_out <= max_score;
                    done <= 1'b1;
                    state <= FINISH;
                end
                else begin
                    database_cursor <= next_db;
                    target_cursor   <= next_tc;
                    diag_len        <= next_dl;
                    state <= PREFETCH;
                end
            end

        PREFETCH:
            if (j_cnt < diag_len) begin
                P_ra <= addr_up;
                Q_ra <= addr_left;
                Da_ra <= addr_left;
                Db_ra <= addr_upleft;
                is_pad_p <= is_pad;
                mval_p <= mval;
                waddr_p <= addr_cur;
                state <= LOAD;
            end
            else
                state <= ENDDIAG;

        LOAD:
            state <= COMPUTE;

        COMPUTE:
            begin
                ram_we <= 1'b1;
                ram_wa <= waddr_p;

                if (is_pad_p) begin
                    P_wd <= NEG_INF;
                    Q_wd <= NEG_INF;
                    D_wd <= 16'sd0;
                end
                else begin
                    P_wd <= new_P;
                    Q_wd <= new_Q;
                    D_wd <= new_D;
                    if (new_D > local_max)
                        local_max <= new_D;
                end

                j_cnt <= j_cnt + 16'sd1;
                state <= PREFETCH;
            end

        ENDDIAG:
            begin
                if (local_max > max_score)
                    max_score <= local_max;

                diag_idx <= (diag_idx==2'd2) ?
                            2'd0 :
                            diag_idx + 2'd1;

                num_diag <= num_diag + 16'sd1;
                state <= NEWDIAG;
            end

        FINISH:
            state <= IDLE;

        endcase
    end
end

endmodule