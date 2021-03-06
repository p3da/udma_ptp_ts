/**
 * Author: David Pernerstorfer <es20m012@technikum-wien.at>
 * Date: 2022-01-06
 * Description: configuration interface for the udma peripheral "ptp ts"
 */

// general control registers for uDMA core - pulpissimo/doc/datasheet/datasheet.pdf
`define REG_RX_SADDR     5'b00000 //BASEADDR+0x00
`define REG_RX_SIZE      5'b00001 //BASEADDR+0x04
`define REG_RX_CFG       5'b00010 //BASEADDR+0x08
`define REG_WHOAMI       5'b00011 //dummy register

`define REG_TX_SADDR     5'b00100 //BASEADDR+0x10
`define REG_TX_SIZE      5'b00101 //BASEADDR+0x14
`define REG_TX_CFG       5'b00110 //BASEADDR+0x18

// eth_frame specific registers
// bit 2 (R) - fifo full (is set to 1 if the RX fifo is full)
`define REG_RX_FIFO_CFG  5'b00111 //BASEADDR+0x1C
// holds the current number of elements in the RX fifo
`define REG_RX_FIFO_N    5'b01000 //BASEADDR+0x20


module udma_ptp_ts_reg #(
    parameter L2_AWIDTH_NOAL = 12,
    parameter TRANS_SIZE     = 16,
    parameter RX_FIFO_BUFFER_DEPTH = 1024,
    parameter RX_FIFO_BUFFER_DEPTH_LOG = $clog2(RX_FIFO_BUFFER_DEPTH)
) (
    input  logic 	                    clk_i,
    input  logic   	                  rstn_i,

    /* RW register signals */
    input  logic               [31:0] cfg_data_i,
    input  logic                [4:0] cfg_addr_i,
    input  logic                      cfg_valid_i,
    input  logic                      cfg_rwn_i,
    output logic               [31:0] cfg_data_o,
    output logic                      cfg_ready_o,

    /* control signals connected to uDMA core */
    output logic [L2_AWIDTH_NOAL-1:0] cfg_rx_startaddr_o,
    output logic     [TRANS_SIZE-1:0] cfg_rx_size_o,
    output logic                      cfg_rx_continuous_o,
    output logic                      cfg_rx_en_o,
    output logic                      cfg_rx_clr_o,
    input  logic                      cfg_rx_en_i,
    input  logic                      cfg_rx_pending_i,
    input  logic [L2_AWIDTH_NOAL-1:0] cfg_rx_curr_addr_i,
    input  logic     [TRANS_SIZE-1:0] cfg_rx_bytes_left_i,

    /* state of RX fifo */
    input  logic    [RX_FIFO_BUFFER_DEPTH_LOG:0]    cfg_rx_fifo_elements
);

logic [L2_AWIDTH_NOAL-1:0] r_rx_startaddr;
logic   [TRANS_SIZE-1 : 0] r_rx_size;
logic                      r_rx_continuous;
logic                      r_rx_en;
logic                      r_rx_clr;

logic                [4:0] s_wr_addr;
logic                [4:0] s_rd_addr;

assign s_wr_addr = (cfg_valid_i & ~cfg_rwn_i) ? cfg_addr_i : 5'h0;
assign s_rd_addr = (cfg_valid_i &  cfg_rwn_i) ? cfg_addr_i : 5'h0;

/* the following signals are directly driven by the corresponding registers */
assign cfg_rx_startaddr_o  = r_rx_startaddr;
assign cfg_rx_size_o       = r_rx_size;
assign cfg_rx_continuous_o = r_rx_continuous;
assign cfg_rx_en_o         = r_rx_en;
assign cfg_rx_clr_o        = r_rx_clr;

always_ff @(posedge clk_i, negedge rstn_i) begin
    if (~rstn_i) begin
        r_rx_startaddr               <= 'h0;
        r_rx_size                    <= 'h0;
        r_rx_continuous              <= 'h0;
        r_rx_en                      <= 'h0;
        r_rx_clr                     <= 'h0;

    end else begin
        r_rx_en   <=  'h0;
        r_rx_clr  <=  'h0;

        /* write the "read configuration registers (cfg_rwn_i == 0 => write into a register )*/
        if (cfg_valid_i & ~cfg_rwn_i) begin
            case (s_wr_addr)
              `REG_RX_SADDR:
                  r_rx_startaddr    <= cfg_data_i[L2_AWIDTH_NOAL-1:0];
              `REG_RX_SIZE:
                  r_rx_size         <= cfg_data_i[TRANS_SIZE-1:0];
              `REG_RX_CFG:
              begin
                  r_rx_clr            <= cfg_data_i[6];
                  r_rx_en             <= cfg_data_i[4];
                  r_rx_continuous     <= cfg_data_i[0];
              end
            endcase
        end // if (cfg_valid_i ...)
   end // end if (~rstn_i)
end //always

always_comb begin
    cfg_data_o <= 32'h0;

    /* read register */
    case (s_rd_addr)
        `REG_RX_SADDR:
            cfg_data_o <= cfg_rx_curr_addr_i;
        `REG_RX_SIZE:
            cfg_data_o[TRANS_SIZE-1:0] <= cfg_rx_bytes_left_i;
        `REG_RX_CFG:
            cfg_data_o <= {26'h0,cfg_rx_pending_i,cfg_rx_en_i,3'h0,r_rx_continuous};
        `REG_WHOAMI:
            cfg_data_o <= 32'hDEADBEEF;
        `REG_RX_FIFO_CFG:
            cfg_data_o <= {
                29'h0,
                cfg_rx_fifo_elements == RX_FIFO_BUFFER_DEPTH,
                1'b0,
                1'b0
            };
        `REG_RX_FIFO_N:
            cfg_data_o[RX_FIFO_BUFFER_DEPTH_LOG:0] <= cfg_rx_fifo_elements;
        default:
            cfg_data_o <= 'h0;
     endcase
end

assign cfg_ready_o  = 1'b1;

endmodule
