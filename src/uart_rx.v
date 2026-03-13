// ============================================================
// uart_rx.v  -  Gowin official UART RX module
//               From Tang 25K example project (GitHub)
//               DO NOT MODIFY - proven working on GW5A-25
// ============================================================
module uart_rx
#(
    parameter CLK_FRE  = 50,       // clock frequency in MHz
    parameter BAUD_RATE = 115200   // baud rate
)
(
    input            clk,
    input            rst_n,
    output reg [7:0] rx_data,
    output reg       rx_data_valid,
    input            rx_data_ready,
    input            rx_pin
);

localparam CYCLE = CLK_FRE * 1000000 / BAUD_RATE;

localparam S_IDLE     = 1;
localparam S_START    = 2;
localparam S_REC_BYTE = 3;
localparam S_STOP     = 4;
localparam S_DATA     = 5;

reg [2:0]  state;
reg [2:0]  next_state;
reg        rx_d0;
reg        rx_d1;
wire       rx_negedge;
reg [7:0]  rx_bits;
reg [15:0] cycle_cnt;
reg [2:0]  bit_cnt;

assign rx_negedge = rx_d1 && ~rx_d0;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin rx_d0 <= 0; rx_d1 <= 0; end
    else        begin rx_d0 <= rx_pin; rx_d1 <= rx_d0; end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) state <= S_IDLE;
    else        state <= next_state;
end

always @(*) begin
    case (state)
        S_IDLE:     next_state = rx_negedge              ? S_START    : S_IDLE;
        S_START:    next_state = (cycle_cnt == CYCLE-1)  ? S_REC_BYTE : S_START;
        S_REC_BYTE: next_state = (cycle_cnt == CYCLE-1 && bit_cnt == 7) ? S_STOP : S_REC_BYTE;
        S_STOP:     next_state = (cycle_cnt == CYCLE/2-1) ? S_DATA    : S_STOP;
        S_DATA:     next_state = rx_data_ready            ? S_IDLE    : S_DATA;
        default:    next_state = S_IDLE;
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rx_data_valid <= 0;
    else if (state == S_STOP && next_state != state) rx_data_valid <= 1;
    else if (state == S_DATA && rx_data_ready)       rx_data_valid <= 0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rx_data <= 0;
    else if (state == S_STOP && next_state != state) rx_data <= rx_bits;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) bit_cnt <= 0;
    else if (state == S_REC_BYTE) begin
        if (cycle_cnt == CYCLE-1) bit_cnt <= bit_cnt + 3'd1;
    end else bit_cnt <= 0;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) cycle_cnt <= 0;
    else if ((state == S_REC_BYTE && cycle_cnt == CYCLE-1) || next_state != state)
        cycle_cnt <= 0;
    else cycle_cnt <= cycle_cnt + 16'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rx_bits <= 0;
    else if (state == S_REC_BYTE && cycle_cnt == CYCLE/2-1)
        rx_bits[bit_cnt] <= rx_pin;
end

endmodule