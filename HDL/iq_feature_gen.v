`timescale 1ns / 1ps

// Generate six Q3.8 feature channels from RMS-normalized I/Q samples.
// Feature order: I, Q, magnitude, delta-magnitude, differential-real,
// differential-imaginary.  The CORDIC result-valid signal pops the matching
// side information from a small ordering FIFO, so no fixed IP latency is
// assumed in this RTL.
module iq_feature_gen #(
    parameter integer FRAME_LEN = 1024,
    parameter integer FIFO_AW   = 6
)(
    input  wire               clk,
    input  wire               rst,
    input  wire signed [11:0] norm_i,
    input  wire signed [11:0] norm_q,
    input  wire               norm_valid,

    output reg  signed [11:0] feat_i,
    output reg  signed [11:0] feat_q,
    output reg  signed [11:0] feat_r,
    output reg  signed [11:0] feat_dr,
    output reg  signed [11:0] feat_dre,
    output reg  signed [11:0] feat_dim,
    output reg                feat_valid
);

localparam integer FIFO_DEPTH = (1 << FIFO_AW);
localparam integer MUL_LAT    = 3;  // mult_gen_2 pipeline stages

reg signed [11:0] i_prev;
reg signed [11:0] q_prev;
reg signed [11:0] r_prev;
reg        [9:0]  sample_cnt;
reg        [MUL_LAT-1:0] mul_valid_pipe;
reg signed [11:0] i_pipe [0:MUL_LAT-1];
reg signed [11:0] q_pipe [0:MUL_LAT-1];
reg               first_pipe [0:MUL_LAT-1];
integer           pi;

reg signed [11:0] fifo_i   [0:FIFO_DEPTH-1];
reg signed [11:0] fifo_q   [0:FIFO_DEPTH-1];
reg signed [11:0] fifo_dre [0:FIFO_DEPTH-1];
reg signed [11:0] fifo_dim [0:FIFO_DEPTH-1];
reg               fifo_first[0:FIFO_DEPTH-1];
reg [FIFO_AW-1:0] wr_ptr;
reg [FIFO_AW-1:0] rd_ptr;

wire signed [23:0] i_sq_s;
wire signed [23:0] q_sq_s;
wire        [24:0] mag_sq = $unsigned(i_sq_s) + $unsigned(q_sq_s);

wire signed [23:0] ii_prev;
wire signed [23:0] qq_prev;
wire signed [23:0] qi_prev;
wire signed [23:0] iq_prev;
wire signed [24:0] dre_full = ii_prev + qq_prev;
wire signed [24:0] dim_full = qi_prev - iq_prev;
wire               mul_out_valid = mul_valid_pipe[MUL_LAT-1];
wire signed [11:0] mul_i = i_pipe[MUL_LAT-1];
wire signed [11:0] mul_q = q_pipe[MUL_LAT-1];
wire               mul_first = first_pipe[MUL_LAT-1];

wire        [31:0] sqrt_in_data = {7'd0, mag_sq};
wire        [15:0] sqrt_out_data;
wire               sqrt_out_valid;

function signed [11:0] sat12;
    input signed [31:0] x;
    begin
        if(x > 32'sd2047)
            sat12 = 12'sd2047;
        else if(x < -32'sd2048)
            sat12 = -12'sd2048;
        else
            sat12 = x[11:0];
    end
endfunction

wire signed [11:0] dre_q38 = sat12($signed(dre_full) >>> 8);
wire signed [11:0] dim_q38 = sat12($signed(dim_full) >>> 8);
wire signed [11:0] r_q38   = (sqrt_out_data > 16'd2047) ?
                              12'sd2047 : $signed(sqrt_out_data[11:0]);
wire signed [12:0] dr_ext  = $signed({1'b0, r_q38}) -
                              $signed({r_prev[11], r_prev});

always @(posedge clk or posedge rst) begin
    if(rst) begin
        i_prev    <= 12'sd0;
        q_prev    <= 12'sd0;
        r_prev    <= 12'sd0;
        sample_cnt<= 10'd0;
        mul_valid_pipe <= {MUL_LAT{1'b0}};
        for(pi=0; pi<MUL_LAT; pi=pi+1) begin
            i_pipe[pi]     <= 12'sd0;
            q_pipe[pi]     <= 12'sd0;
            first_pipe[pi] <= 1'b0;
        end
        wr_ptr    <= {FIFO_AW{1'b0}};
        rd_ptr    <= {FIFO_AW{1'b0}};
        feat_i    <= 12'sd0;
        feat_q    <= 12'sd0;
        feat_r    <= 12'sd0;
        feat_dr   <= 12'sd0;
        feat_dre  <= 12'sd0;
        feat_dim  <= 12'sd0;
        feat_valid<= 1'b0;
    end else begin
        feat_valid <= 1'b0;
        mul_valid_pipe <= {mul_valid_pipe[MUL_LAT-2:0], norm_valid};
        i_pipe[0]      <= norm_i;
        q_pipe[0]      <= norm_q;
        first_pipe[0]  <= norm_valid && (sample_cnt == 0);
        for(pi=1; pi<MUL_LAT; pi=pi+1) begin
            i_pipe[pi]     <= i_pipe[pi-1];
            q_pipe[pi]     <= q_pipe[pi-1];
            first_pipe[pi] <= first_pipe[pi-1];
        end

        if(norm_valid) begin
            i_prev             <= norm_i;
            q_prev             <= norm_q;
            if(sample_cnt == FRAME_LEN-1)
                sample_cnt <= 10'd0;
            else
                sample_cnt <= sample_cnt + 1'b1;
        end

        if(mul_out_valid) begin
            fifo_i[wr_ptr]     <= mul_i;
            fifo_q[wr_ptr]     <= mul_q;
            // Python uses i_prev[0]=i[0], q_prev[0]=q[0]: therefore
            // d_re[0]=I[0]^2+Q[0]^2 and d_im[0]=0.
            fifo_dre[wr_ptr]   <= mul_first ? sat12($signed(mag_sq) >>> 8) : dre_q38;
            fifo_dim[wr_ptr]   <= mul_first ? 12'sd0 : dim_q38;
            fifo_first[wr_ptr] <= mul_first;
            wr_ptr             <= wr_ptr + 1'b1;
        end

        if(sqrt_out_valid) begin
            feat_i     <= fifo_i[rd_ptr];
            feat_q     <= fifo_q[rd_ptr];
            feat_r     <= r_q38;
            feat_dr    <= fifo_first[rd_ptr] ? 12'sd0 : sat12($signed(dr_ext));
            feat_dre   <= fifo_dre[rd_ptr];
            feat_dim   <= fifo_dim[rd_ptr];
            feat_valid <= 1'b1;
            rd_ptr     <= rd_ptr + 1'b1;
            r_prev     <= r_q38;
        end
    end
end

// New Xilinx CORDIC IP: square root, unsigned integer, input width 25,
// output width 13, parallel architecture, AXI4-Stream blocking mode.
cordic_feat6 u_cordic_feat6 (
    .aclk                    (clk),
    .s_axis_cartesian_tvalid (mul_out_valid),
    .s_axis_cartesian_tdata  (sqrt_in_data),
    .m_axis_dout_tvalid      (sqrt_out_valid),
    .m_axis_dout_tdata       (sqrt_out_data)
);

// Existing Xilinx Multiplier IP: signed 12 x signed 12 -> signed 24,
// three pipeline stages. Six instances infer/use six DSP48 multipliers.
mult_gen_2 u_mul_i_sq (
    .CLK(clk), .A(norm_i), .B(norm_i), .P(i_sq_s)
);

mult_gen_2 u_mul_q_sq (
    .CLK(clk), .A(norm_q), .B(norm_q), .P(q_sq_s)
);

mult_gen_2 u_mul_ii_prev (
    .CLK(clk), .A(norm_i), .B(i_prev), .P(ii_prev)
);

mult_gen_2 u_mul_qq_prev (
    .CLK(clk), .A(norm_q), .B(q_prev), .P(qq_prev)
);

mult_gen_2 u_mul_qi_prev (
    .CLK(clk), .A(norm_q), .B(i_prev), .P(qi_prev)
);

mult_gen_2 u_mul_iq_prev (
    .CLK(clk), .A(norm_i), .B(q_prev), .P(iq_prev)
);

endmodule
