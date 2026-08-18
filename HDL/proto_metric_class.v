`timescale 1ns / 1ps
/********************************************************************************************************************\
Copyright(c) 2010, UESTC Technology Inc, All right reserved Department

Department      : 电子科技大学测控技术与仪器研究所
Author          : DingWei
Project Name    : Automatic_Modulation_Recognition
Product Ver     : V1.0
Module Name     : proto_metric_class.v
Module Called   : none
Called By       : 
Target Device   : xc7z020iclg400
Tools Version   : vivado 2018.3

Description     : 模块五：原型分类模块
 
Modification History:
Date          By         Review         Rev.       Res        Ncp        PR-NO.STAGE        Change Description
-----------------------------------------------------------------------------------------------------------------------
2026/2/13	DingWei		   无		    v1.0		无		   无			 无			     创建模块，实现基本功能
\*********************************************************************************************************************/

module proto_metric_class(
    input  wire             clk,                    // 系统时钟
    input  wire             rst,                    // 高有效复位

    // AMR参数
    input  wire             amr_en,                 // 调制识别使能，上升沿有效
    input  wire [9:0]       amr_times,              // 本轮需识别的样本数

    // 与前级特征提取模块对接
    input  wire             emb_valid,              // embedding有效标志，高有效
    input  wire [383:0]     emb_flat,               // 32维embedding
    output reg              class_ready,            // 本模块ready标志，高有效

    output reg              amr_done,               // 识别结束标志，高有效
    output reg  [9:0]       cnt_4ASK,               // 4ASK分类统计
    output reg  [9:0]       cnt_8ASK,               // 8ASK分类统计
    output reg  [9:0]       cnt_BPSK,               // BPSK分类统计
    output reg  [9:0]       cnt_QPSK,               // QPSK分类统计
    output reg  [9:0]       cnt_8PSK,               // 8PSK分类统计
    output reg  [9:0]       cnt_16QAM,              // 16QAM分类统计
    output reg  [9:0]       cnt_64QAM,              // 64QAM分类统计
    output reg  [9:0]       cnt_GMSK                // GMSK分类统计
);


/*************************************************************************************\
    parameter_declaration
\*************************************************************************************/
// 类别/维度参数
localparam      [3:0]       NUM_CLASS   = 4'd8;     
localparam      [5:0]       NUM_DIM     = 6'd32;    

localparam      [3:0]       SQ_SHIFT    = 4'd8;

// 状态定义
localparam      [2:0]       S_IDLE      = 3'd0;     // 空闲状态
localparam      [2:0]       S_EMB_LOAD  = 3'd1;     // 加载特征向量
localparam      [2:0]       S_RUN       = 3'd2;     // 距离度量
localparam      [2:0]       S_WAIT      = 3'd3;     // 距离度量等待
localparam      [2:0]       S_CLASS     = 3'd4;     // 更新分类结果


/*************************************************************************************\
    reg_declaration
\*************************************************************************************/
// 状态寄存器
reg         [2:0]                   state;          // 当前状态

// 存储寄存器
reg                                 amr_en_r;       // amr_en锁存
reg signed  [11:0]                  emb_reg [0:31]; // 32维特征向量锁存

// 索引与计数器
reg         [4:0]                   dim_idx;        // 特征向量维度索引
reg         [4:0]                   dim_idx_r;      // 特征向量维度索引寄存
reg         [2:0]                   class_idx;      // 信号类别索引
reg         [2:0]                   min_idx;        // 最小距离类别索引
reg         [9:0]                   cnt_times;      // 已分类样本计数
// Measurement registers retained for behavioural/post-route observation.
// cnt_cycles counts clk cycles from the amr_en rising edge through S_CLASS;
// last_test_cycles holds the completed single-sample latency.
reg         [31:0]                  cnt_cycles;
reg         [31:0]                  last_test_cycles;

// 地址寄存器
reg         [7:0]                   rom_addr;       // ROM地址

// 数据寄存器
reg signed  [20:0]                  dist2_temp;     // 距离平方
reg signed  [20:0]                  min_dist2;      // 最小距离

// 标志寄存器
reg                                 mul_valid;      // 乘法计算有效标志，高有效
reg         [2:0]                   valid_pipe;     // 有效标志延迟


/*************************************************************************************\
    integer_declaration
\*************************************************************************************/
integer                             ii;             // for循环计数


/*************************************************************************************\
    wire_declaration
\*************************************************************************************/
// 标志信号
wire                                amr_en_p;       // 调制识别使能信号上升沿

// 数据信号
wire signed [11:0]                  rom_dout;       // ROM输出数据
wire signed [11:0]                  diff;           // 特征向量与类原型差分
wire signed [23:0]                  diff2;          // 特征向量与类原型差分平方


/*************************************************************************************\
    assign_logic
\*************************************************************************************/
// 上升沿检测
assign  amr_en_p    =   amr_en & !amr_en_r;

// diff计算
assign diff         =   emb_reg[dim_idx_r] - rom_dout;


/*************************************************************************************\
    function_declaration
\*************************************************************************************/
function signed [23:0] arshift_round_sym_24;
    input signed [23:0] x;
    input integer       shift;
    begin
        arshift_round_sym_24 = x  >>> shift;
    end
endfunction


/*************************************************************************************\
    状态机
\*************************************************************************************/
always @(posedge clk or posedge rst) begin
    if(rst) begin
        class_ready     <=  1'd0;
        amr_done        <=  1'd0;
        cnt_4ASK        <=  10'd0; 
        cnt_8ASK        <=  10'd0; 
        cnt_BPSK        <=  10'd0; 
        cnt_QPSK        <=  10'd0;
        cnt_8PSK        <=  10'd0; 
        cnt_16QAM       <=  10'd0; 
        cnt_64QAM       <=  10'd0; 
        cnt_GMSK        <=  10'd0;
        state           <=  S_IDLE;
        amr_en_r        <=  1'd0;
        dim_idx         <=  5'd0;
        dim_idx_r       <=  5'd0;
        class_idx       <=  3'd0;
        min_idx         <=  3'd0;
        cnt_times       <=  10'd0;
        cnt_cycles      <=  32'd0;
        last_test_cycles<=  32'd0;
        rom_addr        <=  8'd0;
        dist2_temp      <=  21'sd0;
        min_dist2       <=  21'sd0;
        mul_valid       <=  1'd0;
        valid_pipe      <=  3'd0;
        for(ii=0; ii<32; ii=ii+1) begin
            emb_reg[ii] <=  12'sd0;
        end
    end else begin
        amr_en_r    <=  amr_en;
        dim_idx_r   <=  dim_idx;
        valid_pipe  <=  {valid_pipe[1:0] , mul_valid};
        if(state != S_IDLE)
            cnt_cycles <= cnt_cycles + 1'd1;
        case(state)
            S_IDLE: begin
                if(amr_en_p) begin
                    amr_done    <=  1'd0;
                    cnt_4ASK    <=  10'd0; 
                    cnt_8ASK    <=  10'd0; 
                    cnt_BPSK    <=  10'd0; 
                    cnt_QPSK    <=  10'd0;
                    cnt_8PSK    <=  10'd0; 
                    cnt_16QAM   <=  10'd0; 
                    cnt_64QAM   <=  10'd0; 
                    cnt_GMSK    <=  10'd0;
                    cnt_times   <=  10'd0;
                    cnt_cycles  <=  32'd0;
                    last_test_cycles <= 32'd0;
                    state       <=  S_EMB_LOAD;
                end else begin
                    state   <= S_IDLE;
                end
            end

            S_EMB_LOAD: begin
                dist2_temp  <=  21'sd0;
                min_dist2   <=  21'sd0;
                class_idx   <=  3'd0;
                min_idx     <=  3'd0;
                dim_idx     <=  5'd0;
                mul_valid   <=  1'd0;
                rom_addr    <=  8'd0;
                if(emb_valid) begin
                    class_ready <=  1'd0;
                    state       <=  S_RUN;
                    for(ii=0; ii<32; ii=ii+1) begin
                        emb_reg[ii] <= emb_flat[ii*12 +: 12];
                    end
                end else begin
                    class_ready  <=  1'd1;
                end
            end

            S_RUN: begin
                mul_valid   <=  1'd1;
                rom_addr    <=  rom_addr + 1'd1;
                if(dim_idx == (NUM_DIM - 1)) begin
                    dim_idx     <=  5'd0;
                    state       <=  S_WAIT;
                end else begin
                    dim_idx <=  dim_idx + 1'd1;
                end

                if(valid_pipe[2]) begin
                    dist2_temp   <=  dist2_temp + arshift_round_sym_24(diff2, SQ_SHIFT);
                end else begin
                    dist2_temp   <=  21'sd0;
                end
            end

            S_WAIT: begin
                mul_valid       <=  1'd0;
                if(valid_pipe[2]) begin
                    dist2_temp   <=  dist2_temp + arshift_round_sym_24(diff2, SQ_SHIFT);
                end else begin
                    if(class_idx == 3'd0) begin
                        min_idx     <=  class_idx;
                        min_dist2   <=  dist2_temp;
                        class_idx   <=  class_idx + 1'd1;
                        state       <=  S_RUN;
                    end else begin
                        if(dist2_temp < min_dist2) begin
                            min_dist2   <=  dist2_temp;
                            min_idx     <=  class_idx;
                        end else begin
                            min_idx     <=  min_idx;
                        end

                        if(class_idx == (NUM_CLASS -1)) begin
                            class_idx   <=  3'd0;
                            state       <=  S_CLASS;
                        end else begin
                            class_idx   <=  class_idx + 1'd1;
                            state       <=  S_RUN;
                        end
                    end
                end
            end

            S_CLASS: begin
                // Keep the completed count visible until the next amr_en.
                // With amr_times=1, cnt_times therefore changes 0 -> 1 and
                // its rising edge is the end marker in the waveform.
                cnt_times <= cnt_times + 1'd1;
                last_test_cycles <= cnt_cycles + 1'd1;
                case(min_idx)
                    3'd0: cnt_4ASK  <=  cnt_4ASK + 1'd1;
                    3'd1: cnt_8ASK  <=  cnt_8ASK + 1'd1;
                    3'd2: cnt_BPSK  <=  cnt_BPSK + 1'd1;
                    3'd3: cnt_QPSK  <=  cnt_QPSK + 1'd1;
                    3'd4: cnt_8PSK  <=  cnt_8PSK + 1'd1;
                    3'd5: cnt_16QAM <=  cnt_16QAM + 1'd1;
                    3'd6: cnt_64QAM <=  cnt_64QAM + 1'd1;
                    3'd7: cnt_GMSK  <=  cnt_GMSK + 1'd1;
                    default: ;
                endcase

                if((cnt_times + 1'd1) >= amr_times) begin
                    amr_done <=  1'd1;
                    state           <=  S_IDLE;
                end else begin
                    state           <=  S_EMB_LOAD;
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

/*************************************************************************************\
    例化部分
\*************************************************************************************/
blk_mem_gen_13 u_proto_rom (
    .clka   (clk        ),      // input wire clka
    .addra  (rom_addr   ),      // input wire [7 : 0] addra
    .douta  (rom_dout   )       // output wire [11 : 0] douta
);

mult_gen_2 u_diff2 (
    .CLK (clk           ),      // input wire CLK
    .A   (diff          ),      // input wire [11 : 0] A
    .B   (diff          ),      // input wire [11 : 0] B
    .P   (diff2         )       // output wire [23 : 0] P
);

endmodule
