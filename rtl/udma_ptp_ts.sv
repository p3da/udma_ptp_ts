/**
 * Author: David Pernerstorfer <es20m012@technikum-wien.at>
 * Date: 2022-01-03
 * Description: connects the AXIS RX an TX channel from ethernet mac to pulpissimo udma
 */

module udma_ptp_ts #(
    parameter L2_AWIDTH_NOAL = 12,
    parameter TRANS_SIZE     = 16,

    parameter RX_FIFO_BUFFER_DEPTH = 32,
    parameter RX_FIFO_BUFFER_DEPTH_LOG = $clog2(RX_FIFO_BUFFER_DEPTH)
) (
    input  logic                      sys_clk_i,
    input  logic                      clk_eth,
    input  logic                      clk_eth90,
    input  logic                      rst_eth,
    input  logic   	                  rstn_i,

    input  logic               [31:0] cfg_data_i,
    input  logic                [4:0] cfg_addr_i,
    input  logic                      cfg_valid_i,
    input  logic                      cfg_rwn_i,
    output logic                      cfg_ready_o,
    output logic               [31:0] cfg_data_o,

    output logic [L2_AWIDTH_NOAL-1:0] cfg_rx_startaddr_o,
    output logic     [TRANS_SIZE-1:0] cfg_rx_size_o,
    output logic                      cfg_rx_continuous_o,
    output logic                      cfg_rx_en_o,
    output logic                      cfg_rx_clr_o,
    input  logic                      cfg_rx_en_i,
    input  logic                      cfg_rx_pending_i,
    input  logic [L2_AWIDTH_NOAL-1:0] cfg_rx_curr_addr_i,
    input  logic     [TRANS_SIZE-1:0] cfg_rx_bytes_left_i,

    output logic                [1:0] data_rx_datasize_o,
    output logic               [31:0] data_rx_o,
    output logic                      data_rx_valid_o,
    input  logic                      data_rx_ready_i,

    input  logic               [95:0] eth_rx_axis_tdata,
    input  logic                      eth_rx_axis_tvalid,
    output logic                      eth_rx_axis_tready,
    input  logic                      eth_rx_axis_tlast,
    input  logic                      eth_rx_axis_tuser
);

/* udma peripheral uses 32bit data words */
assign data_rx_datasize_o = 2'b10;

/* signals between rx dc fifo and generic fifo */
logic            s_data_rx_valid_out;
logic            s_data_rx_ready_out;
logic     [95:0] s_data_rx_out;

logic            s_data_rx_valid_in;
logic            s_data_rx_ready_in;
logic     [31:0] s_data_rx_in;

typedef enum {
    IDLE = 2'b00,
    WORD2 = 2'b01,
    WORD3 = 2'b10
} udma_ptp_ts_state_t;

udma_ptp_ts_state_t curr_state, next_state;

logic [95:0] data_rx_tmp;
logic [95:0] data_rx_tmp_next;


/* tells register file the number of 32-bit elements in the */
logic [RX_FIFO_BUFFER_DEPTH_LOG:0] cfg_rx_fifo_elements;

/* register interface */
udma_eth_frame_reg #(
    .L2_AWIDTH_NOAL(L2_AWIDTH_NOAL),
    .TRANS_SIZE(TRANS_SIZE),
    .RX_FIFO_BUFFER_DEPTH(RX_FIFO_BUFFER_DEPTH)
) u_reg_if (
    .clk_i              ( sys_clk_i           ),
    .rstn_i             ( rstn_i              ),

    .cfg_data_i         ( cfg_data_i          ),
    .cfg_addr_i         ( cfg_addr_i          ),
    .cfg_valid_i        ( cfg_valid_i         ),
    .cfg_rwn_i          ( cfg_rwn_i           ),
    .cfg_ready_o        ( cfg_ready_o         ),
    .cfg_data_o         ( cfg_data_o          ),

    .cfg_rx_startaddr_o ( cfg_rx_startaddr_o  ),
    .cfg_rx_size_o      ( cfg_rx_size_o       ),
    .cfg_rx_continuous_o( cfg_rx_continuous_o ),
    .cfg_rx_en_o        ( cfg_rx_en_o         ),
    .cfg_rx_clr_o       ( cfg_rx_clr_o        ),
    .cfg_rx_en_i        ( cfg_rx_en_i         ),
    .cfg_rx_pending_i   ( cfg_rx_pending_i    ),
    .cfg_rx_curr_addr_i ( cfg_rx_curr_addr_i  ),
    .cfg_rx_bytes_left_i( cfg_rx_bytes_left_i ),

    .cfg_rx_fifo_elements (cfg_rx_fifo_elements)

);

/* rx dc fifo */
udma_dc_fifo #(
    .DATA_WIDTH(32),
    .BUFFER_DEPTH(RX_FIFO_BUFFER_DEPTH)
) u_dc_fifo_rx (
    .src_clk_i    ( clk_eth            ),
    .src_rstn_i   ( rst_eth            ),
    .src_data_i   ( eth_rx_axis_tdata  ),
    .src_valid_i  ( eth_rx_axis_tvalid ),
    .src_ready_o  ( eth_rx_axis_tready ),
    .dst_clk_i    ( sys_clk_i          ),
    .dst_rstn_i   ( rstn_i             ),
    .dst_data_o   ( s_data_rx_out        ),
    .dst_valid_o  ( s_data_rx_valid_out    ),
    .dst_ready_i  ( s_data_rx_ready_out    )
);

/* rx fifos; this fifo is just necessary to count the elements */
io_generic_fifo #(
    .DATA_WIDTH(32),
    .BUFFER_DEPTH(RX_FIFO_BUFFER_DEPTH)
) u_fifo_rx (
    .clk_i   ( sys_clk_i       ),
    .rstn_i  ( rstn_i          ),
    .clr_i   ( 1'b0            ),

    .elements_o (cfg_rx_fifo_elements),

    .data_o  (s_data_rx_o),
    .valid_o (data_rx_valid_o),
    .ready_i (data_rx_ready_i),

    .valid_i (s_data_rx_valid_in),
    .data_i  (s_data_rx_in),
    .ready_o (s_data_rx_ready_in)
);


/* the following state machine acts as a bridge between the u_dc_fifo_rx with
 * data width 96 and the u_fifo_rx with data width 32.
 * if the dc fifo sends a new valid 96 bit word, the transmission is stalled
 * for the next 2 cycles. the 96 bit is buffered and sequentially transmitted
 * to the 32 bit fifo. the lower 32 bit are transmitted to the receiving fio FIRST.
 */
always_ff @(posedge sys_clk_i, negedge rstn_i) begin
    if (~rstn_i) begin
      curr_state <= IDLE;
      data_rx_tmp <= 'h0;
    end else begin
      curr_state <= next_state;
      data_rx_tmp <= data_rx_tmp_next;
    end
end

always_comb begin
  next_state = state;
  case (curr_state)
    IDLE : begin
      if ((s_data_rx_valid_out & s_data_rx_ready_in) == 1'b1) begin
        next_state = WORD2;
      end
    end
    WORD2: begin
      if (s_data_rx_ready_in) begin
        next_state = WORD3;
      end
    end
    WORD3: begin
      if (s_data_rx_ready_in) begin
        next_state = IDLE;
      end
    end
  endcase
end

always_comb begin
    case (curr_state)
        IDLE : begin
          s_data_rx_ready_out = s_data_rx_ready_in;
          s_data_rx_valid_in = s_data_rx_valid_out;
          s_data_rx_in = s_data_rx_out[95:64];
          data_rx_tmp_next = s_data_rx_out;
        end
        WORD2: begin
            s_data_rx_ready_out = 1'b0;
            s_data_rx_valid_in = 1'b1;
            s_data_rx_in = data_rx_tmp[63:32];
            data_rx_tmp_next = data_rx_tmp;
        end
        WORD3: begin
            s_data_rx_ready_out = 1'b0;
            s_data_rx_valid_in = 1'b1;
            s_data_rx_in = data_rx_tmp[31:0];
            data_rx_tmp_next = data_rx_tmp;
        end
        default: begin
            s_data_rx_ready_out = 1'b0;
            s_data_rx_valid_in = 1'b1;
            s_data_rx_in = 'h0;
            data_rx_tmp_next = IDLE;
        end
    endcase
end

endmodule
