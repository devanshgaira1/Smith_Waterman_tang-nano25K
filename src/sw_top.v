`timescale 1ns / 1ps

module sw_top (
    input  wire clk,
    input  wire rst_n,
    input  wire uart_rx,
    output wire uart_tx
);

localparam CLK_MHZ = 32'd50;
localparam BAUD    = 32'd921600;

/* ================================
   UART RX
================================ */

wire [7:0] rx_data;
wire       rx_valid;
wire       rx_ready = 1'b1;

uart_rx #(
    .CLK_FRE(CLK_MHZ),
    .BAUD_RATE(BAUD)
) u_rx (
    .clk(clk),
    .rst_n(rst_n),
    .rx_data(rx_data),
    .rx_data_valid(rx_valid),
    .rx_data_ready(rx_ready),
    .rx_pin(uart_rx)
);

/* ================================
   UART TX
================================ */

reg  [7:0] tx_data;
reg        tx_valid;
wire       tx_ready;

uart_tx #(
    .CLK_FRE(CLK_MHZ),
    .BAUD_RATE(BAUD)
) u_tx (
    .clk(clk),
    .rst_n(rst_n),
    .tx_data(tx_data),
    .tx_data_valid(tx_valid),
    .tx_data_ready(tx_ready),
    .tx_pin(uart_tx)
);

/* ================================
   SW Core
================================ */

reg         sw_we;
reg  [11:0] sw_addr;
reg  [3:0]  sw_din;
reg         sw_start;
wire        sw_done;
wire [15:0] sw_score;

reg [10:0] lenT_r, lenD_r;
reg [11:0] rx_seq_cnt;

sw_core u_core (
    .clk(clk),
    .rst_n(rst_n),
    .seq_we(sw_we),
    .seq_addr(sw_addr),
    .seq_din(sw_din),
    .lenT(lenT_r),
    .lenD(lenD_r),
    .start(sw_start),
    .done(sw_done),
    .score_out(sw_score)
);

/* ================================
   FSM
================================ */

localparam FSM_SYNC      = 4'd0;
localparam FSM_LENT_MSB  = 4'd1;
localparam FSM_LENT_LSB  = 4'd2;
localparam FSM_LEND_MSB  = 4'd3;
localparam FSM_LEND_LSB  = 4'd4;
localparam FSM_TARGET    = 4'd5;
localparam FSM_DB        = 4'd6;
localparam FSM_WAIT      = 4'd7;
localparam FSM_TX_MARK   = 4'd8;
localparam FSM_TX_MSB    = 4'd9;
localparam FSM_TX_LSB    = 4'd10;

reg [3:0]  fsm;
reg [15:0] score_latch;

/* ================================
   Main FSM
================================ */

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fsm         <= FSM_SYNC;
        rx_seq_cnt  <= 12'd0;
        lenT_r      <= 11'd0;
        lenD_r      <= 11'd0;
        score_latch <= 16'd0;
        sw_we       <= 1'b0;
        sw_start    <= 1'b0;
        sw_addr     <= 12'd0;
        sw_din      <= 4'd0;
        tx_data     <= 8'd0;
        tx_valid    <= 1'b0;
    end
    else begin
        sw_we    <= 1'b0;
        sw_start <= 1'b0;

        if (tx_valid && tx_ready)
            tx_valid <= 1'b0;

        case (fsm)

        /* ================================
           WAIT FOR SYNC BYTE 0xAA
        ================================= */
        FSM_SYNC:
            if (rx_valid && rx_data == 8'hAA)
                fsm <= FSM_LENT_MSB;

        /* ================================
           RECEIVE TARGET LENGTH
        ================================= */
        FSM_LENT_MSB:
            if (rx_valid) begin
                lenT_r[10:8] <= rx_data[2:0];
                fsm <= FSM_LENT_LSB;
            end

        FSM_LENT_LSB:
            if (rx_valid) begin
                lenT_r[7:0] <= rx_data;
                rx_seq_cnt  <= 12'd0;
                fsm <= FSM_LEND_MSB;
            end

        /* ================================
           RECEIVE DB LENGTH
        ================================= */
        FSM_LEND_MSB:
            if (rx_valid) begin
                lenD_r[10:8] <= rx_data[2:0];
                fsm <= FSM_LEND_LSB;
            end

        FSM_LEND_LSB:
            if (rx_valid) begin
                lenD_r[7:0] <= rx_data;
                rx_seq_cnt  <= 12'd0;
                fsm <= FSM_TARGET;
            end

        /* ================================
           LOAD TARGET SEQUENCE
        ================================= */
        FSM_TARGET:
            if (rx_valid) begin
                sw_we   <= 1'b1;
                sw_addr <= rx_seq_cnt;   // already 12-bit safe
                sw_din  <= rx_data[3:0];

                if (rx_seq_cnt == (lenT_r - 11'd1)) begin
                    rx_seq_cnt <= 12'd0;
                    fsm <= FSM_DB;
                end
                else
                    rx_seq_cnt <= rx_seq_cnt + 12'd1;
            end

        /* ================================
           LOAD DATABASE SEQUENCE
        ================================= */
        FSM_DB:
            if (rx_valid) begin
                sw_we <= 1'b1;

                // SAFE 12-bit addition
                sw_addr <= 12'd1024 + rx_seq_cnt;

                sw_din <= rx_data[3:0];

                if (rx_seq_cnt == (lenD_r - 11'd1)) begin
                    sw_start <= 1'b1;
                    fsm <= FSM_WAIT;
                end
                else
                    rx_seq_cnt <= rx_seq_cnt + 12'd1;
            end

        /* ================================
           WAIT FOR CORE DONE
        ================================= */
        FSM_WAIT:
            if (sw_done) begin
                score_latch <= sw_score;
                tx_data  <= 8'hBB;
                tx_valid <= 1'b1;
                fsm <= FSM_TX_MARK;
            end

        /* ================================
           TRANSMIT SCORE
        ================================= */
        FSM_TX_MARK:
            if (tx_ready && !tx_valid) begin
                tx_data  <= score_latch[15:8];
                tx_valid <= 1'b1;
                fsm <= FSM_TX_MSB;
            end

        FSM_TX_MSB:
            if (tx_ready && !tx_valid) begin
                tx_data  <= score_latch[7:0];
                tx_valid <= 1'b1;
                fsm <= FSM_TX_LSB;
            end

        FSM_TX_LSB:
            if (tx_ready && !tx_valid)
                fsm <= FSM_SYNC;

        default:
            fsm <= FSM_SYNC;

        endcase
    end
end

endmodule