`timescale 1ns / 1ps

// Synthesizable streaming top for complete LFRPNet inference.
// Stage 1: input_norm + iq_feature_gen (preprocessing)
// Stage 2: feat_extract                (embedding network)
// Stage 3: proto_metric_class          (classification)
// Each stage owns its input storage, permitting adjacent samples to overlap.
module AMR_pipeline_top(
    input  wire               clk_250m,
    input  wire               clk_data,
    input  wire               rst,
    input  wire               amr_en,
    input  wire [9:0]         amr_times,
    input  wire signed [15:0] input_i,
    input  wire signed [15:0] input_q,
    input  wire               input_valid,
    output wire               input_ready,
    output wire               amr_done,
    output wire [2:0]         predicted_class,
    output wire [11:0]        embedding_probe
);

wire [11:0] norm_i, norm_q;
wire norm_valid;
wire signed [11:0] feat_i, feat_q, feat_r, feat_dr, feat_dre, feat_dim;
wire feat_valid;
wire feat_extract_ready;
wire class_ready;
wire emb_valid;
wire [383:0] emb_flat;
assign predicted_class = u_classification.min_idx;
assign embedding_probe = emb_flat[11:0]^emb_flat[23:12]^emb_flat[35:24]^emb_flat[47:36]^
 emb_flat[59:48]^emb_flat[71:60]^emb_flat[83:72]^emb_flat[95:84]^
 emb_flat[107:96]^emb_flat[119:108]^emb_flat[131:120]^emb_flat[143:132]^
 emb_flat[155:144]^emb_flat[167:156]^emb_flat[179:168]^emb_flat[191:180]^
 emb_flat[203:192]^emb_flat[215:204]^emb_flat[227:216]^emb_flat[239:228]^
 emb_flat[251:240]^emb_flat[263:252]^emb_flat[275:264]^emb_flat[287:276]^
 emb_flat[299:288]^emb_flat[311:300]^emb_flat[323:312]^emb_flat[335:324]^
 emb_flat[347:336]^emb_flat[359:348]^emb_flat[371:360]^emb_flat[383:372];
// Keep detailed counters on chip for ILA/debug. Exporting all eight counters
// would exceed the available package IO on the XC7Z020 target.
wire [9:0] cnt_4ASK, cnt_8ASK, cnt_BPSK, cnt_QPSK;
wire [9:0] cnt_8PSK, cnt_16QAM, cnt_64QAM, cnt_GMSK;

input_norm u_preprocess (
    .wr_clk(clk_data), .rd_clk(clk_250m), .rst(rst),
    .input_i(input_i), .input_q(input_q), .input_valid(input_valid),
    .input_ready(input_ready), .param_est_ready(feat_extract_ready),
    .amr_en(amr_en), .amr_times(amr_times),
    .norm_i(norm_i), .norm_q(norm_q), .norm_valid(norm_valid)
);

iq_feature_gen u_six_feature (
    .clk(clk_250m), .rst(rst),
    .norm_i(norm_i), .norm_q(norm_q), .norm_valid(norm_valid),
    .feat_i(feat_i), .feat_q(feat_q), .feat_r(feat_r),
    .feat_dr(feat_dr), .feat_dre(feat_dre), .feat_dim(feat_dim),
    .feat_valid(feat_valid)
);

feat_extract u_feature_extract (
    .clk(clk_250m), .rst(rst),
    .feat_i(feat_i), .feat_q(feat_q), .feat_r(feat_r),
    .feat_dr(feat_dr), .feat_dre(feat_dre), .feat_dim(feat_dim),
    .feat_valid(feat_valid), .feat_extract_ready(feat_extract_ready),
    .class_ready(class_ready), .emb_valid(emb_valid), .emb_flat(emb_flat)
);

proto_metric_class u_classification (
    .clk(clk_250m), .rst(rst), .amr_en(amr_en), .amr_times(amr_times),
    .emb_valid(emb_valid), .emb_flat(emb_flat), .class_ready(class_ready),
    .amr_done(amr_done),
    .cnt_4ASK(cnt_4ASK), .cnt_8ASK(cnt_8ASK),
    .cnt_BPSK(cnt_BPSK), .cnt_QPSK(cnt_QPSK),
    .cnt_8PSK(cnt_8PSK), .cnt_16QAM(cnt_16QAM),
    .cnt_64QAM(cnt_64QAM), .cnt_GMSK(cnt_GMSK)
);

endmodule
