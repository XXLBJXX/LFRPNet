`timescale 1ns / 1ps
/*********************************************************************************************************************\
Copyright(c) 2010, UESTC Technology Inc, All right reserved Department

Department      : 电子科技大学测控技术与仪器研究所
Author          : DingWei
Project Name    : Automatic_Modulation_Recognition
Product Ver     : V1.0
Module Name     : feat_extract.v
Module Called   : none
Called By       : 
Target Device   : xc7z020iclg400
Tools Version   : vivado 2018.3

Description     : 模块四：特征提取模块

Modification History:
Date          By         Review         Rev.       Res        Ncp        PR-NO.STAGE        Change Description
------------------------------------------------------------------------------------------------------------------------
2026/2/10	DingWei		   无		    v1.0		无		   无			 无			     创建模块，实现基本功能
\**********************************************************************************************************************/

module feat_extract(
    input  wire             clk,                                    // 系统时钟
    input  wire             rst,                                    // 高有效复位

    // 与相位重整变换模块对接
    input  wire signed [11:0] feat_i,
    input  wire signed [11:0] feat_q,
    input  wire signed [11:0] feat_r,
    input  wire signed [11:0] feat_dr,
    input  wire signed [11:0] feat_dre,
    input  wire signed [11:0] feat_dim,
    input  wire               feat_valid,
    output reg              feat_extract_ready,                     // 本模块ready标志，高有效

    // 与原型分类模块对接
    input  wire             class_ready,                            // 原型分类模块ready标志，高有效
    output reg              emb_valid,                              // embedding输出有效，高有效
    output wire [383:0]     emb_flat                                // 32维embedding
);


/*************************************************************************************\
    parameter_declaration
\*************************************************************************************/
// 卷积各层长度
localparam          [10:0]  L0              = 11'd1024;             
localparam          [9:0]   L1              = 10'd512;              
localparam          [9:0]   L2              = 10'd512;              
localparam          [8:0]   L3              = 9'd256;               
localparam          [7:0]   L4              = 8'd128;               
localparam          [6:0]   L5              = 7'd64;                

`include "gn_layer_config.vh"

// 卷积核尺寸
localparam          [1:0]   K3              = 2'd3;                 
localparam          [2:0]   K5              = 3'd5;                 
localparam          [2:0]   K7              = 3'd7;                 

// stride
localparam                  STRIDE1         = 1'd1;
localparam          [1:0]   STRIDE2         = 2'd2;

// padding
localparam                  PAD1            = 1'd1;
localparam          [1:0]   PAD2            = 2'd2;
localparam          [1:0]   PAD3            = 2'd3;

// 权重ROM基址
localparam          [7:0]   WBASE_B1        = 8'd0;                 
localparam          [7:0]   WBASE_B2_DW     = 8'd14;
localparam          [7:0]   WBASE_B2_PW     = 8'd19;
localparam          [7:0]   WBASE_R3_SK     = 8'd27;
localparam          [7:0]   WBASE_R3_C1     = 8'd43;
localparam          [7:0]   WBASE_R3_DW     = 8'd123;
localparam          [7:0]   WBASE_R3_PW     = 8'd126;
localparam          [7:0]   WBASE_B4        = 8'd158;

// 偏置ROM基址
localparam          [3:0]   BBASE_B1        = 4'd0;                 
localparam          [3:0]   BBASE_B2_DW     = 4'd1;                 
localparam          [3:0]   BBASE_B2_PW     = 4'd2;                 
localparam          [3:0]   BBASE_R3_SK     = 4'd3;                 
localparam          [3:0]   BBASE_R3_C1     = 4'd5;                 
localparam          [3:0]   BBASE_R3_DW     = 4'd7;                 
localparam          [3:0]   BBASE_R3_PW     = 4'd9;                 
localparam          [3:0]   BBASE_B4        = 4'd11;                

// 状态定义
localparam          [4:0]   S_IDLE          = 5'd0;                 // 空闲态
// Block1
localparam          [4:0]   S_B1_INIT       = 5'd1;                 // block1初始化
localparam          [4:0]   S_B1_RUN        = 5'd2;                 // block1卷积计算
localparam          [4:0]   S_B1_DONE       = 5'd3;                 // block1卷积完成
// Block2
localparam          [4:0]   S_B2_DW_INIT    = 5'd4;                 // block2 DW初始化
localparam          [4:0]   S_B2_DW_RUN     = 5'd5;                 // block2 DW卷积计算
localparam          [4:0]   S_B2_DW_DONE    = 5'd6;                 // block2 DW卷积完成
localparam          [4:0]   S_B2_PW_INIT    = 5'd7;                 // block2 PW初始化
localparam          [4:0]   S_B2_PW_RUN     = 5'd8;                 // block2 PW卷积计算
localparam          [4:0]   S_B2_PW_DONE    = 5'd9;                 // block2 PW卷积完成
// Pool2
localparam          [4:0]   S_P2_INIT       = 5'd10;                // pool2初始化
localparam          [4:0]   S_P2_RUN        = 5'd11;                // pool2运行
// Res3
localparam          [4:0]   S_R3_SK_INIT    = 5'd12;                // res3 skip初始化
localparam          [4:0]   S_R3_SK_RUN     = 5'd13;                // res3 skip卷积计算
localparam          [4:0]   S_R3_SK_DONE    = 5'd14;                // res3 skip卷积完成
localparam          [4:0]   S_R3_C1_INIT    = 5'd15;                // res3 conv1初始化
localparam          [4:0]   S_R3_C1_RUN     = 5'd16;                // res3 conv1卷积计算
localparam          [4:0]   S_R3_C1_DONE    = 5'd17;                // res3 conv1卷积完成
localparam          [4:0]   S_R3_DW_INIT    = 5'd18;                // res3 DW初始化
localparam          [4:0]   S_R3_DW_RUN     = 5'd19;                // res3 DW卷积计算
localparam          [4:0]   S_R3_DW_DONE    = 5'd20;                // res3 DW卷积完成
localparam          [4:0]   S_R3_PW_INIT    = 5'd21;                // res3 PW初始化
localparam          [4:0]   S_R3_PW_RUN     = 5'd22;                // res3 PW卷积计算
localparam          [4:0]   S_R3_PW_DONE    = 5'd23;                // res3 PW卷积完成
// Block4
localparam          [4:0]   S_B4_INIT       = 5'd24;                // block4初始化
localparam          [4:0]   S_B4_RUN        = 5'd25;                // block4卷积计算
localparam          [4:0]   S_B4_DONE       = 5'd26;                // block4卷积完成
// GAP
localparam          [4:0]   S_GAP           = 5'd27;                
// 输出
localparam          [5:0]   S_OUT           = 6'd29;                // 特征向量输出
localparam          [5:0]   S_GN_WAIT       = 6'd30;
localparam          [5:0]   S_GN_LOAD_HI    = 6'd31;


/*************************************************************************************\
    reg_declaration
\*************************************************************************************/
// 状态寄存器
reg                 [5:0]                   state;
reg                 [5:0]                   gn_next_state;
reg                 [2:0]                   gn_target;
localparam [2:0] GN_T_DST=3'd0, GN_T_SKIP=3'd1, GN_T_RES=3'd2, GN_T_GAP=3'd3;

reg gn_start, gn_load_valid, gn_load_hi, gn_load_last;
reg [9:0] gn_cfg_length;
reg gn_cfg_channels64;
reg [8:0] gn_cfg_param_base;
reg [3:0] gn_cfg_frac_in, gn_cfg_frac_out;
reg gn_cfg_relu6;
reg signed [927:0] gn_load_data;
wire gn_load_ready, gn_out_valid, gn_busy, gn_done;
wire [9:0] gn_out_time;
wire gn_out_hi;
wire signed [639:0] gn_out_data;
reg signed [28:0] gn_hi_hold[0:31];
reg gn_hi_last;
                                                                
// 双缓冲选择
reg                                         src_sel;                // 0=RAMA；1=RAMB

// embedding输出寄存器
reg signed          [11:0]                  emb [0:31];             

// 标志寄存器
reg                                         cur_is_dw;              // 1=DW，0=普通卷积/PW
reg                                         cout_lo_flag;           // 输出低32通道标志，高有效
reg                                         in_time_oob;            // 时间索引越界标志，高有效
reg                 [2:0]                   tap_idx;                // tap索引
reg                                         b1_ch_group;
reg                                         b1_ch_group_r;

// 累加器
reg signed          [28:0]                  acc [0:31];             // 卷积计算累加器
reg signed          [19:0]                  acc_ass [0:31];         // 辅助累加器

// MAC相关
reg                                         mac_issue_vld;          // MAC发起有效标志，高有效
reg                 [3:0]                   vld_pipe;               // MAC有效延迟
reg                 [5:0]                   base_ch_issue;          // MAC输入通道的索引
reg                 [5:0]                   base_ch_issue_r;        // MAC输入通道的索引寄存

// 卷积计算输入数据寄存
reg signed          [19:0]                  x_conv0 [0:31];         
reg signed          [19:0]                  x_conv1 [0:31];         
reg signed          [19:0]                  x_conv2;                
reg signed          [19:0]                  x_conv3;                

// ROM地址
reg                 [7:0]                   wrom_addr;              

// ROM数据寄存
reg signed          [11:0]                  wrom0_dout_r [0:31];      
reg signed          [11:0]                  wrom1_dout_r [0:31];    
reg signed          [11:0]                  wrom2_dout_r [0:31];    
reg signed          [11:0]                  wrom3_dout_r [0:31];    

// ram_iq端口
reg                                         ram_iq_wea;             
reg                 [9:0]                   ram_iq_addra;           
reg                 [71:0]                  ram_iq_dina;
reg signed          [11:0]                  ram_iq_addrb;           

// 双缓冲源端RAM端口
reg signed          [10:0]                  src_lo_addrb;           
reg signed          [8:0]                   src_hi_addrb;           

// 双缓冲终端RAM端口
reg                                         dst_lo_wea;                   
reg                                         dst_hi_wea;             
reg                 [8:0]                   dst_lo_addra;           
reg                 [6:0]                   dst_hi_addra;           
reg                 [639:0]                 dst_lo_dina;            
reg                 [639:0]                 dst_hi_dina;            

// skip RAM端口
reg                                         sk_lo_wea;              
reg                                         sk_hi_wea;              
reg                 [6:0]                   sk_lo_addra;            
reg                 [6:0]                   sk_hi_addra;            
reg                 [6:0]                   sk_lo_addrb;            
reg                 [6:0]                   sk_hi_addrb;            
reg                 [639:0]                 sk_lo_dina;             
reg                 [639:0]                 sk_hi_dina;             


/*************************************************************************************\
    integer_declaration
\*************************************************************************************/
integer                                     ii;                     // for循环计数


/*************************************************************************************\
    wire_declaration
\*************************************************************************************/
// ram_iq/skip_ram输出
wire                [71:0]                  ram_iq_doutb;
wire                [639:0]                 sk_lo_doutb;            
wire                [639:0]                 sk_hi_doutb;            

// 双缓冲rama/b_lo端口
wire                                        rama_lo_wea;                  
wire                                        ramb_lo_wea;            
wire                [8:0]                   rama_lo_addra;          
wire                [8:0]                   ramb_lo_addra;          
wire                [8:0]                   rama_lo_addrb;          
wire                [8:0]                   ramb_lo_addrb;          
wire                [639:0]                 rama_lo_dina;           
wire                [639:0]                 ramb_lo_dina;           
wire                [639:0]                 rama_lo_doutb;          
wire                [639:0]                 ramb_lo_doutb;          

// 双缓冲rama/b_hi端口
wire                                        rama_hi_wea;                  
wire                                        ramb_hi_wea;            
wire                [6:0]                   rama_hi_addra;          
wire                [6:0]                   ramb_hi_addra;          
wire                [6:0]                   rama_hi_addrb;          
wire                [6:0]                   ramb_hi_addrb;          
wire                [639:0]                 rama_hi_dina;           
wire                [639:0]                 ramb_hi_dina;           
wire                [639:0]                 rama_hi_doutb;          
wire                [639:0]                 ramb_hi_doutb;          

// 卷积运算RAM输出
wire                [639:0]                 src_lo_doutb;           
wire                [639:0]                 src_hi_doutb;           

// ROM输出
wire                [383:0]                 wrom0_dout;             
wire                [383:0]                 wrom1_dout;             
wire                [383:0]                 wrom2_dout;             
wire                [383:0]                 wrom3_dout;             

//乘法器输出
wire signed         [31:0]                  p0     [0:31];
wire signed         [31:0]                  p1     [0:31];
wire signed         [31:0]                  p2     [0:31];
wire signed         [31:0]                  p3     [0:31];


/*************************************************************************************\
    assign_logic
\*************************************************************************************/
assign  rama_lo_wea     =    src_sel ? dst_lo_wea : 1'd0;
assign  rama_hi_wea     =    src_sel ? dst_hi_wea : 1'd0;
assign  ramb_lo_wea     =    src_sel ? 1'd0 : dst_lo_wea;
assign  ramb_hi_wea     =    src_sel ? 1'd0 : dst_hi_wea;

assign  rama_lo_addra   =    src_sel ? dst_lo_addra : 9'd0;
assign  rama_hi_addra   =    src_sel ? dst_hi_addra : 7'd0;
assign  ramb_lo_addra   =    src_sel ? 9'd0 : dst_lo_addra;
assign  ramb_hi_addra   =    src_sel ? 7'd0 : dst_hi_addra;

assign  rama_lo_dina    =    src_sel ? dst_lo_dina : 640'd0;
assign  rama_hi_dina    =    src_sel ? dst_hi_dina : 640'd0;
assign  ramb_lo_dina    =    src_sel ? 640'd0 : dst_lo_dina;
assign  ramb_hi_dina    =    src_sel ? 640'd0 : dst_hi_dina;

assign  rama_lo_addrb   =    src_sel ? 9'd0 : src_lo_addrb[8:0];
assign  rama_hi_addrb   =    src_sel ? 7'd0 : src_hi_addrb[6:0];
assign  ramb_lo_addrb   =    src_sel ? src_lo_addrb[8:0] : 9'd0;
assign  ramb_hi_addrb   =    src_sel ? src_hi_addrb[6:0] : 7'd0;

assign  src_lo_doutb    =   src_sel ? ramb_lo_doutb : rama_lo_doutb;
assign  src_hi_doutb    =   src_sel ? ramb_hi_doutb : rama_hi_doutb;


/*************************************************************************************\
    genvar
\*************************************************************************************/
genvar gi;
generate
    for (gi=0; gi<32; gi=gi+1) begin : GEN_EMB_PACK
        assign emb_flat[gi*12 +: 12] = emb[gi];
    end
endgenerate


/*************************************************************************************\
    function_declaration
\*************************************************************************************/
function signed [11:0] relu6_q38;
    input signed [28:0] x;
    begin
        if(x < 12'sd0)        
            relu6_q38 = 12'sd0;
        else if(x > 12'sd1536) 
            relu6_q38 = 12'sd1536;
        else
            relu6_q38 = x[11:0];
    end
endfunction

function signed [19:0] relu6_q5_20;
    input signed [20:0] x;
    begin
        if(x < 0) relu6_q5_20 = 20'sd0;
        else if(x > 21'sd192) relu6_q5_20 = 20'sd192;
        else relu6_q5_20 = x[19:0];
    end
endfunction

function signed [31:0] arshift_round_sym_32;
    input signed [31:0] x;
    input integer       shift;
    begin
        arshift_round_sym_32 = x >>> shift;
    end
endfunction

function signed [19:0] ch32;
    input [639:0] bus;
    input [4:0] ch;
    begin
        case(ch)
            5'd0  : ch32 = bus[19:0];     
            5'd1  : ch32 = bus[39:20];    
            5'd2  : ch32 = bus[59:40];    
            5'd3  : ch32 = bus[79:60];    
            5'd4  : ch32 = bus[99:80];    
            5'd5  : ch32 = bus[119:100];  
            5'd6  : ch32 = bus[139:120];  
            5'd7  : ch32 = bus[159:140];  
            5'd8  : ch32 = bus[179:160];  
            5'd9  : ch32 = bus[199:180];  
            5'd10 : ch32 = bus[219:200];  
            5'd11 : ch32 = bus[239:220];  
            5'd12 : ch32 = bus[259:240];  
            5'd13 : ch32 = bus[279:260];  
            5'd14 : ch32 = bus[299:280];  
            5'd15 : ch32 = bus[319:300];  
            5'd16 : ch32 = bus[339:320];  
            5'd17 : ch32 = bus[359:340];  
            5'd18 : ch32 = bus[379:360];  
            5'd19 : ch32 = bus[399:380];  
            5'd20 : ch32 = bus[419:400];  
            5'd21 : ch32 = bus[439:420];  
            5'd22 : ch32 = bus[459:440];  
            5'd23 : ch32 = bus[479:460];  
            5'd24 : ch32 = bus[499:480];  
            5'd25 : ch32 = bus[519:500];  
            5'd26 : ch32 = bus[539:520];  
            5'd27 : ch32 = bus[559:540];  
            5'd28 : ch32 = bus[579:560];  
            5'd29 : ch32 = bus[599:580];  
            5'd30 : ch32 = bus[619:600];  
            5'd31 : ch32 = bus[639:620];  
            default: ch32 = 20'sd0;
        endcase
    end
endfunction


/**************************************************************************************************\
    ROM数据寄存
\**************************************************************************************************/
always @(posedge clk or posedge rst) begin
    if(rst) begin
        for(ii=0; ii<32; ii=ii+1) begin
            wrom0_dout_r[ii]  <=  12'sd0;
            wrom1_dout_r[ii]  <=  12'sd0;
            wrom2_dout_r[ii]  <=  12'sd0;
            wrom3_dout_r[ii]  <=  12'sd0;
        end
    end else begin
        for(ii=0; ii<32; ii=ii+1) begin
            wrom0_dout_r[ii]  <=  wrom0_dout[ii*12 +: 12];
            wrom1_dout_r[ii]  <=  wrom1_dout[ii*12 +: 12];
            wrom2_dout_r[ii]  <=  wrom2_dout[ii*12 +: 12];
            wrom3_dout_r[ii]  <=  wrom3_dout[ii*12 +: 12];
        end
    end
end

/**************************************************************************************************\
    卷积乘法器输入
\**************************************************************************************************/
always @(posedge clk or posedge rst) begin
    if(rst) begin
        for(ii=0; ii<32; ii=ii+1) begin
            x_conv0[ii] <=  20'sd0;
            x_conv1[ii] <=  20'sd0;
        end
        x_conv2 <=  20'sd0;
        x_conv3 <=  20'sd0;
    end else begin
        if(in_time_oob) begin
            for(ii=0; ii<32; ii=ii+1) begin
                x_conv0[ii] <=  20'sd0;
                x_conv1[ii] <=  20'sd0;
            end
            x_conv2 <=  20'sd0;
            x_conv3 <=  20'sd0;
        end else if((state == S_B1_RUN) || (state == S_B1_DONE)) begin
            if(!b1_ch_group_r) begin
                for(ii=0; ii<32; ii=ii+1) begin
                    x_conv0[ii] <= {{8{ram_iq_doutb[11]}}, ram_iq_doutb[11:0]};
                    x_conv1[ii] <= {{8{ram_iq_doutb[23]}}, ram_iq_doutb[23:12]};
                end
                x_conv2 <= {{8{ram_iq_doutb[35]}}, ram_iq_doutb[35:24]};
                x_conv3 <= {{8{ram_iq_doutb[47]}}, ram_iq_doutb[47:36]};
            end else begin
                for(ii=0; ii<32; ii=ii+1) begin
                    x_conv0[ii] <= {{8{ram_iq_doutb[59]}}, ram_iq_doutb[59:48]};
                    x_conv1[ii] <= {{8{ram_iq_doutb[71]}}, ram_iq_doutb[71:60]};
                end
                x_conv2 <= 20'sd0;
                x_conv3 <= 20'sd0;
            end
        end else if(cur_is_dw) begin
            for(ii=0; ii<32; ii=ii+1) begin
                x_conv0[ii] <=  src_lo_doutb[ii*20 +: 20];
                x_conv1[ii] <=  src_hi_doutb[ii*20 +: 20];
            end
            x_conv2 <=  20'sd0;
            x_conv3 <=  20'sd0;
        end else begin
            if(base_ch_issue_r < 6'd32) begin
                for(ii=0; ii<32; ii=ii+1) begin
                    x_conv0[ii] <=  ch32(src_lo_doutb, (base_ch_issue_r + 6'd0));
                    x_conv1[ii] <=  ch32(src_lo_doutb, (base_ch_issue_r + 6'd1));
                end
                x_conv2   <=  ch32(src_lo_doutb, (base_ch_issue_r + 6'd2));
                x_conv3   <=  ch32(src_lo_doutb, (base_ch_issue_r + 6'd3));
            end else begin
                for(ii=0; ii<32; ii=ii+1) begin
                    x_conv0[ii] <=  ch32(src_hi_doutb, (base_ch_issue_r + 6'd0) - 6'd32);
                    x_conv1[ii] <=  ch32(src_hi_doutb, (base_ch_issue_r + 6'd1) - 6'd32);
                end
                x_conv2   <=  ch32(src_hi_doutb, (base_ch_issue_r + 6'd2) - 6'd32);
                x_conv3   <=  ch32(src_hi_doutb, (base_ch_issue_r + 6'd3) - 6'd32);
            end
        end
    end
end


/**************************************************************************************************\
    状态机
\**************************************************************************************************/
always @(posedge clk or posedge rst) begin
    if(rst) begin
        feat_extract_ready  <=  1'd0;
        emb_valid           <=  1'd0;
        state               <=  S_IDLE;             
        src_sel             <=  1'd0;
        cur_is_dw           <=  1'd0;   
        cout_lo_flag        <=  1'd1;
        in_time_oob         <=  1'd0;  
        tap_idx             <=  3'd0; 
        b1_ch_group         <=  1'b0;
        b1_ch_group_r       <=  1'b0;
        mac_issue_vld       <=  1'd0;
        vld_pipe            <=  4'd0; 
        base_ch_issue       <=  6'd0;  
        base_ch_issue_r     <=  6'd0;
        wrom_addr           <=  8'd0;
        ram_iq_wea          <=  1'd0;
        ram_iq_addra        <=  10'h3FF;
        ram_iq_dina         <=  72'd0;
        ram_iq_addrb        <=  12'sd0;
        src_lo_addrb        <=  11'sd0;
        src_hi_addrb        <=  9'sd0;
        dst_lo_wea          <=  1'd0;  
        dst_hi_wea          <=  1'd0;  
        dst_lo_addra        <=  9'd0;
        dst_hi_addra        <=  7'd0;
        dst_lo_dina         <=  640'd0; 
        dst_hi_dina         <=  640'd0; 
        sk_lo_wea           <=  1'd0;  
        sk_hi_wea           <=  1'd0;
        sk_lo_addra         <=  7'd0; 
        sk_hi_addra         <=  7'd0;   
        sk_lo_addrb         <=  7'd0;  
        sk_hi_addrb         <=  7'd0;   
        sk_lo_dina          <=  640'd0;    
        sk_hi_dina          <=  640'd0;
        gn_start            <= 1'b0;
        gn_load_valid       <= 1'b0;
        gn_load_hi          <= 1'b0;
        gn_load_last        <= 1'b0;
        gn_cfg_length       <= 0;
        gn_cfg_channels64   <= 0;
        gn_cfg_param_base   <= 0;
        gn_cfg_frac_in      <= 0;
        gn_cfg_frac_out     <= 0;
        gn_cfg_relu6        <= 0;
        gn_load_data        <= 0;
        gn_hi_last          <= 0;
        gn_next_state       <= S_IDLE;
        gn_target           <= GN_T_DST;
        for(ii=0; ii<32; ii=ii+1) begin
            emb[ii]         <=  12'sd0;
            acc[ii]         <=  29'sd0;
            acc_ass[ii]     <=  20'sd0;
        end
    end else begin
        gn_start        <= 1'b0;
        gn_load_valid   <= 1'b0;
        gn_load_last    <= 1'b0;
        vld_pipe        <=  {vld_pipe[2:0], mac_issue_vld};
        base_ch_issue_r <=  base_ch_issue;
        b1_ch_group_r   <=  b1_ch_group;
        case (state)
            S_IDLE: begin
                emb_valid   <=  1'd0;
                if(feat_valid) begin
                    feat_extract_ready  <=  1'd0;
                    ram_iq_wea          <=  1'd1;
                    ram_iq_dina         <=  {feat_dim, feat_dre, feat_dr, feat_r, feat_q, feat_i};
                    if(ram_iq_addra == (L0 - 2)) begin
                        ram_iq_addra    <=  ram_iq_addra + 1'd1;
                        state           <=  S_B1_INIT;
                    end else begin
                        ram_iq_addra    <=  ram_iq_addra + 1'd1;
                    end
                end else begin
                    feat_extract_ready  <=  1'd1;
                end
            end

            S_B1_INIT: begin
                gn_start <= 1'b1; gn_cfg_length <= L1; gn_cfg_channels64 <= 1'b0;
                gn_cfg_param_base <= GNBASE_B1; gn_cfg_frac_in <= GNFIN_B1;
                gn_cfg_frac_out <= GNFOUT_B1; gn_cfg_relu6 <= 1'b1;
                ram_iq_wea      <=  1'd0;
                src_sel         <=  1'd0;
                wrom_addr       <=  WBASE_B1;
                cur_is_dw       <=  1'd0;
                tap_idx         <=  3'd0;
                b1_ch_group     <=  1'b0;
                ram_iq_addrb    <=  12'sd0 - PAD3;
                dst_lo_wea      <=  1'd0;
                dst_lo_addra    <=  9'd511;
                in_time_oob     <=  1'd0;
                state           <=  S_B1_RUN;
                for(ii=0; ii<32; ii=ii+1) begin
                    acc[ii]     <=  29'sd0;
                end
            end

            S_B1_RUN: begin
                dst_lo_wea      <=  1'd0;
                mac_issue_vld   <=  1'd1; 
                if((ram_iq_addrb < 0) || (ram_iq_addrb >= L0)) begin
                    in_time_oob     <=  1'd1;
                end else begin
                    in_time_oob     <=  1'd0;
                end

                if(!b1_ch_group) begin
                    b1_ch_group     <=  1'b1;
                    wrom_addr       <=  wrom_addr + 1'b1;
                end else begin
                    b1_ch_group     <=  1'b0;
                    if(tap_idx == (K7 - 1)) begin
                        ram_iq_addrb <= ram_iq_addrb - (K7 - 1) + STRIDE2;
                        wrom_addr    <= WBASE_B1;
                        tap_idx      <= 3'd0;
                        state        <= S_B1_DONE;
                    end else begin
                        ram_iq_addrb <= ram_iq_addrb + 1'd1;
                        wrom_addr    <= wrom_addr + 1'b1;
                        tap_idx      <= tap_idx + 1'd1;
                    end
                end

                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 9) + arshift_round_sym_32(p1[ii], 9) + arshift_round_sym_32(p2[ii], 9) + arshift_round_sym_32(p3[ii], 9);
                    end
                end else begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <=  29'sd0;
                    end
                end
            end

            S_B1_DONE: begin
                mac_issue_vld   <=  1'd0;
                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 9) + arshift_round_sym_32(p1[ii], 9) + arshift_round_sym_32(p2[ii], 9) + arshift_round_sym_32(p3[ii], 9);
                    end
                end else begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        gn_load_data[ii*29 +: 29] <= acc[ii];
                    end
                    gn_load_valid <= 1'b1; gn_load_hi <= 1'b0;
                    if(dst_lo_addra == (L1 - 2)) begin
                        dst_lo_addra    <=  dst_lo_addra + 1'd1;
                        gn_load_last <= 1'b1; gn_next_state <= S_B2_DW_INIT;
                        gn_target <= GN_T_DST; state <= S_GN_WAIT;
                    end else begin
                        dst_lo_addra    <=  dst_lo_addra + 1'd1;
                        state           <=  S_B1_RUN;
                    end
                end
            end

            S_B2_DW_INIT: begin
                gn_start <= 1'b1; gn_cfg_length <= L2; gn_cfg_channels64 <= 1'b0;
                gn_cfg_param_base <= GNBASE_B2_DW; gn_cfg_frac_in <= GNFIN_B2_DW;
                gn_cfg_frac_out <= GNFOUT_B2_DW; gn_cfg_relu6 <= 1'b1;
                src_sel         <=  1'd1;
                wrom_addr       <=  WBASE_B2_DW;
                cur_is_dw       <=  1'd1;
                tap_idx         <=  3'd0;
                src_lo_addrb    <=  11'sd0 - PAD2; 
                dst_lo_wea      <=  1'd0;
                dst_lo_addra    <=  9'd511;
                in_time_oob     <=  1'd0;
                state           <=  S_B2_DW_RUN;
                for(ii=0; ii<32; ii=ii+1) begin
                    acc[ii]     <=  29'sd0;
                end
            end

            S_B2_DW_RUN: begin
                dst_lo_wea      <=  1'd0;
                mac_issue_vld   <=  1'd1; 
                if((src_lo_addrb < 0) || (src_lo_addrb >= L1)) begin
                    in_time_oob     <=  1'd1;
                end else begin
                    in_time_oob     <=  1'd0;
                end

                if(tap_idx == (K5 - 1)) begin
                    src_lo_addrb    <=  src_lo_addrb - (K5 - 1) + STRIDE1;
                    wrom_addr       <=  WBASE_B2_DW; 
                    tap_idx         <=  3'd0;
                    state           <=  S_B2_DW_DONE;
                end else begin
                    wrom_addr       <=  wrom_addr + 1'd1;
                    src_lo_addrb    <=  src_lo_addrb + 1'd1;
                    tap_idx         <=  tap_idx + 1'd1;
                end

                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 9);
                    end
                end else begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= 29'sd0;
                    end
                end
            end

            S_B2_DW_DONE: begin
                mac_issue_vld   <=  1'd0;
                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 9);
                    end
                end else begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        gn_load_data[ii*29 +: 29] <= acc[ii];
                    end
                    gn_load_valid <= 1'b1; gn_load_hi <= 1'b0;
                    if(dst_lo_addra == (L2 - 2)) begin
                        dst_lo_addra    <=  dst_lo_addra + 1'd1;
                        gn_load_last <= 1'b1; gn_next_state <= S_B2_PW_INIT;
                        gn_target <= GN_T_DST; state <= S_GN_WAIT;
                    end else begin
                        dst_lo_addra    <=  dst_lo_addra + 1'd1;
                        state           <=  S_B2_DW_RUN;
                    end
                end
            end

            S_B2_PW_INIT: begin
                gn_start <= 1'b1; gn_cfg_length <= L2; gn_cfg_channels64 <= 1'b0;
                gn_cfg_param_base <= GNBASE_B2_PW; gn_cfg_frac_in <= GNFIN_B2_PW;
                gn_cfg_frac_out <= GNFOUT_B2_PW; gn_cfg_relu6 <= 1'b1;
                src_sel         <=  1'd0;
                wrom_addr       <=  WBASE_B2_PW;
                cur_is_dw       <=  1'd0;
                tap_idx         <=  3'd0;
                src_lo_addrb    <=  11'sd0;
                base_ch_issue   <=  6'd0; 
                dst_lo_wea      <=  1'd0;
                dst_lo_addra    <=  9'd511;
                in_time_oob     <=  1'd0;
                state           <=  S_B2_PW_RUN;
                for(ii=0; ii<32; ii=ii+1) begin
                    acc[ii]     <=  29'sd0;
                end
            end

            S_B2_PW_RUN: begin
                dst_lo_wea      <=  1'd0;
                mac_issue_vld   <=  1'd1; 
                if(base_ch_issue == 6'd28) begin
                    base_ch_issue   <=  6'd0;
                    src_lo_addrb    <=  src_lo_addrb + STRIDE1;
                    wrom_addr       <=  WBASE_B2_PW; 
                    state           <=  S_B2_PW_DONE;
                end else begin
                    wrom_addr       <=  wrom_addr + 1'd1;
                    base_ch_issue   <=  base_ch_issue + 3'd4;
                end

                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 9) + arshift_round_sym_32(p1[ii], 9) + arshift_round_sym_32(p2[ii], 9) + arshift_round_sym_32(p3[ii], 9);
                    end
                end else begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= 29'sd0;
                    end
                end
            end

            S_B2_PW_DONE: begin
                mac_issue_vld   <=  1'd0;
                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 9) + arshift_round_sym_32(p1[ii], 9) + arshift_round_sym_32(p2[ii], 9) + arshift_round_sym_32(p3[ii], 9);
                    end
                end else begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        gn_load_data[ii*29 +: 29] <= acc[ii];
                    end
                    gn_load_valid <= 1'b1; gn_load_hi <= 1'b0;
                    if(dst_lo_addra == (L2 - 2)) begin
                        dst_lo_addra    <=  dst_lo_addra + 1'd1;
                        gn_load_last <= 1'b1; gn_next_state <= S_P2_INIT;
                        gn_target <= GN_T_DST; state <= S_GN_WAIT;
                    end else begin
                        dst_lo_addra    <=  dst_lo_addra + 1'd1;
                        state           <=  S_B2_PW_RUN;
                    end
                end
            end

            S_P2_INIT: begin
                src_sel         <=  1'd1;
                tap_idx         <=  3'd0;           
                src_lo_addrb    <=  11'sd0;
                dst_lo_wea      <=  1'd0;
                dst_lo_addra    <=  9'd511;
                state           <=  S_P2_RUN;
            end

            S_P2_RUN: begin
                if(tap_idx == 3'd0) begin
                    dst_lo_wea  <=  1'd0;
                    if(dst_lo_addra == 9'd255) begin
                        state   <=  S_R3_SK_INIT;
                    end else begin
                        src_lo_addrb    <=  src_lo_addrb + 1'd1;
                        tap_idx         <=  tap_idx + 1'd1;
                    end
                end else if(tap_idx == 3'd1) begin
                    tap_idx <=  tap_idx + 1'd1;
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= {9'd0 , src_lo_doutb[ii*20 +: 20]};
                    end
                end else if(tap_idx == 3'd2) begin
                    tap_idx         <=  3'd0;
                    src_lo_addrb    <=  src_lo_addrb + 1'd1;
                    dst_lo_wea      <=  1'd1;
                    dst_lo_addra    <=  dst_lo_addra + 1'd1;
                    for(ii=0; ii<32; ii=ii+1) begin
                        if($signed(acc[ii][19:0]) >= $signed(src_lo_doutb[ii*20 +: 20])) begin
                            dst_lo_dina[ii*20 +: 20]    <=  acc[ii][19:0];
                        end else begin
                            dst_lo_dina[ii*20 +: 20]    <=  src_lo_doutb[ii*20 +: 20];
                        end
                    end
                end 
                else begin
                    state   <=  S_P2_RUN;
                end
            end

            S_R3_SK_INIT: begin
                gn_start <= 1'b1; gn_cfg_length <= L4; gn_cfg_channels64 <= 1'b1;
                gn_cfg_param_base <= GNBASE_R3_SK; gn_cfg_frac_in <= GNFIN_R3_SK;
                gn_cfg_frac_out <= GNFOUT_R3_SK; gn_cfg_relu6 <= 1'b0;
                src_sel         <=  1'd0;
                wrom_addr       <=  WBASE_R3_SK;
                cur_is_dw       <=  1'd0;
                tap_idx         <=  3'd0;
                src_lo_addrb    <=  11'sd0;
                base_ch_issue   <=  6'd0; 
                sk_lo_wea       <=  1'd0;
                sk_hi_wea       <=  1'd0;
                sk_lo_addra     <=  7'd127;
                sk_hi_addra     <=  7'd127;
                in_time_oob     <=  1'd0;
                cout_lo_flag    <=  1'd1;
                state           <=  S_R3_SK_RUN;
                for(ii=0; ii<32; ii=ii+1) begin
                    acc[ii]     <=  29'sd0;
                end
            end

            S_R3_SK_RUN: begin
                sk_lo_wea       <=  1'd0;
                sk_hi_wea       <=  1'd0;
                mac_issue_vld   <=  1'd1;
                if(cout_lo_flag) begin
                    wrom_addr       <=  wrom_addr + 1'd1;
                    if(base_ch_issue == 6'd28) begin
                        base_ch_issue   <=  6'd0;
                        state           <=  S_R3_SK_DONE;
                    end else begin
                        base_ch_issue   <=  base_ch_issue + 3'd4;
                    end
                end else begin
                    if(base_ch_issue == 6'd28) begin
                        base_ch_issue   <=  6'd0;
                        src_lo_addrb    <=  src_lo_addrb + STRIDE2;
                        wrom_addr       <=  WBASE_R3_SK; 
                        state           <=  S_R3_SK_DONE;
                    end else begin
                        wrom_addr       <=  wrom_addr + 1'd1;
                        base_ch_issue   <=  base_ch_issue + 3'd4;
                    end
                end

                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 9) + arshift_round_sym_32(p1[ii], 9) + arshift_round_sym_32(p2[ii], 9) + arshift_round_sym_32(p3[ii], 9);
                    end
                end else begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= 29'sd0;
                    end
                end
            end

            S_R3_SK_DONE: begin
                mac_issue_vld   <=  1'd0;
                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 9) + arshift_round_sym_32(p1[ii], 9) + arshift_round_sym_32(p2[ii], 9) + arshift_round_sym_32(p3[ii], 9);
                    end
                end else begin
                    if(cout_lo_flag) begin
                        sk_lo_addra     <=  sk_lo_addra + 1'd1;
                        cout_lo_flag    <=  1'd0;
                        state           <=  S_R3_SK_RUN;
                        for(ii=0; ii<32; ii=ii+1) begin
                            gn_load_data[ii*29 +: 29] <= acc[ii];
                        end
                        gn_load_valid <= 1'b1; gn_load_hi <= 1'b0;
                    end else begin
                        cout_lo_flag    <=  1'd1;
                        for(ii=0; ii<32; ii=ii+1) begin
                            gn_load_data[ii*29 +: 29] <= acc[ii];
                        end
                        gn_load_valid <= 1'b1; gn_load_hi <= 1'b1;
                        if(sk_hi_addra == (L4 - 2)) begin
                            sk_hi_addra     <=  sk_hi_addra + 1'd1;
                            gn_load_last <= 1'b1; gn_next_state <= S_R3_C1_INIT;
                            gn_target <= GN_T_SKIP; state <= S_GN_WAIT;
                        end else begin
                            sk_hi_addra     <=  sk_hi_addra + 1'd1;
                            state           <=  S_R3_SK_RUN;
                        end
                    end
                end
            end

            S_R3_C1_INIT: begin
                gn_start <= 1'b1; gn_cfg_length <= L4; gn_cfg_channels64 <= 1'b1;
                gn_cfg_param_base <= GNBASE_R3_C1; gn_cfg_frac_in <= GNFIN_R3_C1;
                gn_cfg_frac_out <= GNFOUT_R3_C1; gn_cfg_relu6 <= 1'b1;
                src_sel         <=  1'd0;
                sk_lo_wea       <=  1'd0;
                sk_hi_wea       <=  1'd0;
                wrom_addr       <=  WBASE_R3_C1;
                cur_is_dw       <=  1'd0;
                tap_idx         <=  3'd0;
                src_lo_addrb    <=  11'sd0 - PAD2;
                base_ch_issue   <=  6'd0; 
                dst_lo_wea      <=  1'd0;
                dst_hi_wea      <=  1'd0;
                dst_lo_addra    <=  9'd511;
                dst_hi_addra    <=  7'd127;
                in_time_oob     <=  1'd0;
                cout_lo_flag    <=  1'd1;
                state           <=  S_R3_C1_RUN;
                for(ii=0; ii<32; ii=ii+1) begin
                    acc[ii]     <=  29'sd0;
                end
            end

            S_R3_C1_RUN: begin
                dst_lo_wea      <=  1'd0;
                dst_hi_wea      <=  1'd0;
                mac_issue_vld   <=  1'd1;
                if((src_lo_addrb < 0) || (src_lo_addrb >= L3)) begin
                    in_time_oob     <=  1'd1;
                end else begin
                    in_time_oob     <=  1'd0;
                end

                if(cout_lo_flag) begin
                    if((tap_idx == (K5 - 1)) && (base_ch_issue == 6'd28)) begin
                        tap_idx         <=  3'd0;
                        base_ch_issue   <=  6'd0;
                        src_lo_addrb    <=  src_lo_addrb - (K5 - 1);
                        wrom_addr       <=  wrom_addr + 1'd1; 
                        state           <=  S_R3_C1_DONE;
                    end else begin
                        wrom_addr       <=  wrom_addr + 1'd1;
                        if(base_ch_issue == 6'd28) begin
                            base_ch_issue   <=  6'd0;
                            tap_idx         <=  tap_idx + 1'd1;
                            src_lo_addrb    <=  src_lo_addrb + 1'd1;
                        end else begin
                            base_ch_issue   <=  base_ch_issue + 3'd4;
                        end
                    end
                end else begin
                    if((tap_idx == (K5 - 1)) && (base_ch_issue == 6'd28)) begin
                        tap_idx         <=  3'd0;
                        base_ch_issue   <=  6'd0;
                        src_lo_addrb    <=  src_lo_addrb - (K5 - 1) + STRIDE2;
                        wrom_addr       <=  WBASE_R3_C1; 
                        state           <=  S_R3_C1_DONE;
                    end else begin
                        wrom_addr       <=  wrom_addr + 1'd1;
                        if(base_ch_issue == 6'd28) begin
                            base_ch_issue   <=  6'd0;
                            tap_idx         <=  tap_idx + 1'd1;
                            src_lo_addrb    <=  src_lo_addrb + 1'd1;
                        end else begin
                            base_ch_issue   <=  base_ch_issue + 3'd4;
                        end
                    end
                end

                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 9) + arshift_round_sym_32(p1[ii], 9) + arshift_round_sym_32(p2[ii], 9) + arshift_round_sym_32(p3[ii], 9);
                    end
                end else begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= 29'sd0;
                    end
                end
            end

            S_R3_C1_DONE: begin
                mac_issue_vld   <=  1'd0;
                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 9) + arshift_round_sym_32(p1[ii], 9) + arshift_round_sym_32(p2[ii], 9) + arshift_round_sym_32(p3[ii], 9);
                    end
                end else begin
                    if(cout_lo_flag) begin
                        dst_lo_addra    <=  dst_lo_addra + 1'd1;
                        cout_lo_flag    <=  1'd0;
                        state           <=  S_R3_C1_RUN;
                        for(ii=0; ii<32; ii=ii+1) begin
                            gn_load_data[ii*29 +: 29] <= acc[ii];
                        end
                        gn_load_valid <= 1'b1; gn_load_hi <= 1'b0;
                    end else begin
                        cout_lo_flag    <=  1'd1;
                        for(ii=0; ii<32; ii=ii+1) begin
                            gn_load_data[ii*29 +: 29] <= acc[ii];
                        end
                        gn_load_valid <= 1'b1; gn_load_hi <= 1'b1;
                        if(dst_hi_addra == (L4 - 2)) begin
                            dst_hi_addra    <=  dst_hi_addra + 1'd1;
                            gn_load_last <= 1'b1; gn_next_state <= S_R3_DW_INIT;
                            gn_target <= GN_T_DST; state <= S_GN_WAIT;
                        end else begin
                            dst_hi_addra    <=  dst_hi_addra + 1'd1;
                            state           <=  S_R3_C1_RUN;
                        end
                    end
                end
            end

            S_R3_DW_INIT: begin
                gn_start <= 1'b1; gn_cfg_length <= L4; gn_cfg_channels64 <= 1'b1;
                gn_cfg_param_base <= GNBASE_R3_DW; gn_cfg_frac_in <= GNFIN_R3_DW;
                gn_cfg_frac_out <= GNFOUT_R3_DW; gn_cfg_relu6 <= 1'b0;
                src_sel         <=  1'd1;
                wrom_addr       <=  WBASE_R3_DW;
                cur_is_dw       <=  1'd1;
                tap_idx         <=  3'd0;
                src_lo_addrb    <=  11'sd0 - PAD1;
                src_hi_addrb    <=  9'sd0 - PAD1;
                base_ch_issue   <=  6'd0; 
                dst_lo_wea      <=  1'd0;
                dst_hi_wea      <=  1'd0;
                dst_lo_addra    <=  9'd511;
                dst_hi_addra    <=  7'd127;
                in_time_oob     <=  1'd0;
                state           <=  S_R3_DW_RUN;
                for(ii=0; ii<32; ii=ii+1) begin
                    acc[ii]     <=  29'sd0;
                    acc_ass[ii] <=  20'sd0;
                end
            end

            S_R3_DW_RUN: begin
                dst_lo_wea      <=  1'd0;
                dst_hi_wea      <=  1'd0;
                mac_issue_vld   <=  1'd1; 
                if((src_lo_addrb < 0) || (src_lo_addrb >= L4)) begin
                    in_time_oob     <=  1'd1;
                end else begin
                    in_time_oob     <=  1'd0;
                end

                if(tap_idx == (K3 - 1)) begin
                    src_lo_addrb    <=  src_lo_addrb - (K3 - 1) + STRIDE1;
                    src_hi_addrb    <=  src_hi_addrb - (K3 - 1) + STRIDE1;
                    wrom_addr       <=  WBASE_R3_DW; 
                    tap_idx         <=  3'd0;
                    state           <=  S_R3_DW_DONE;
                end else begin
                    wrom_addr       <=  wrom_addr + 1'd1;
                    src_lo_addrb    <=  src_lo_addrb + 1'd1;
                    src_hi_addrb    <=  src_hi_addrb + 1'd1;
                    tap_idx         <=  tap_idx + 1'd1;
                end

                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii]     <= acc[ii] + arshift_round_sym_32(p0[ii], 6);
                        acc_ass[ii] <= acc_ass[ii] + arshift_round_sym_32(p1[ii], 6);
                    end
                end else begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii]     <= 29'sd0;
                        acc_ass[ii] <= 20'sd0;
                    end
                end
            end

            S_R3_DW_DONE: begin
                mac_issue_vld   <=  1'd0;
                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 6);
                        acc_ass[ii] <= acc_ass[ii] + arshift_round_sym_32(p1[ii], 6);
                    end
                end else begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        gn_load_data[ii*29 +: 29] <= acc[ii];
                        gn_hi_hold[ii] <= {{9{acc_ass[ii][19]}},acc_ass[ii]};
                    end
                    gn_load_valid <= 1'b1; gn_load_hi <= 1'b0;
                    gn_hi_last <= (dst_lo_addra == (L4 - 2));
                    state <= S_GN_LOAD_HI;
                    if(dst_lo_addra == (L4 - 2)) begin
                        dst_lo_addra    <=  dst_lo_addra + 1'd1;
                    end else begin
                        dst_lo_addra    <=  dst_lo_addra + 1'd1;
                    end
                    dst_hi_addra <= dst_hi_addra + 1'd1;
                end
            end

            S_R3_PW_INIT: begin
                gn_start <= 1'b1; gn_cfg_length <= L4; gn_cfg_channels64 <= 1'b1;
                gn_cfg_param_base <= GNBASE_R3_PW; gn_cfg_frac_in <= GNFIN_R3_PW;
                gn_cfg_frac_out <= GNFOUT_R3_PW; gn_cfg_relu6 <= 1'b0;
                src_sel         <=  1'd0;
                wrom_addr       <=  WBASE_R3_PW;
                cur_is_dw       <=  1'd0;
                tap_idx         <=  3'd0;
                src_lo_addrb    <=  11'sd0;
                src_hi_addrb    <=  9'sd0;
                sk_lo_addrb     <=  7'd0;
                sk_hi_addrb     <=  7'd0;
                base_ch_issue   <=  6'd0; 
                dst_lo_wea      <=  1'd0;
                dst_hi_wea      <=  1'd0;
                dst_lo_addra    <=  9'd511;
                dst_hi_addra    <=  7'd127;
                in_time_oob     <=  1'd0;
                cout_lo_flag    <=  1'd1;
                state           <=  S_R3_PW_RUN;
                for(ii=0; ii<32; ii=ii+1) begin
                    acc[ii]     <=  29'sd0;
                end
            end

            S_R3_PW_RUN: begin
                dst_lo_wea      <=  1'd0;
                dst_hi_wea      <=  1'd0;
                mac_issue_vld   <=  1'd1; 
                if(cout_lo_flag) begin
                    wrom_addr   <=  wrom_addr + 1'd1;
                    if(base_ch_issue == 6'd60) begin
                        base_ch_issue   <=  6'd0;
                        state           <=  S_R3_PW_DONE;
                    end else begin
                        base_ch_issue   <=  base_ch_issue + 3'd4;
                    end
                end else begin
                    if(base_ch_issue == 6'd60) begin
                        base_ch_issue   <=  6'd0;
                        src_lo_addrb    <=  src_lo_addrb + STRIDE1;
                        src_hi_addrb    <=  src_hi_addrb + STRIDE1;
                        wrom_addr       <=  WBASE_R3_PW; 
                        state           <=  S_R3_PW_DONE;
                    end else begin
                        wrom_addr       <=  wrom_addr + 1'd1;
                        base_ch_issue   <=  base_ch_issue + 3'd4;
                    end
                end

                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 9) + arshift_round_sym_32(p1[ii], 9) + arshift_round_sym_32(p2[ii], 9) + arshift_round_sym_32(p3[ii], 9);
                    end
                end else begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= 29'sd0;
                    end
                end
            end

            S_R3_PW_DONE: begin
                mac_issue_vld   <=  1'd0;
                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 9) + arshift_round_sym_32(p1[ii], 9) + arshift_round_sym_32(p2[ii], 9) + arshift_round_sym_32(p3[ii], 9);
                    end
                end else begin
                    if(cout_lo_flag) begin
                        dst_lo_addra    <=  dst_lo_addra + 1'd1;
                        cout_lo_flag    <=  1'd0;
                        sk_lo_addrb     <=  sk_lo_addrb + 1'd1;
                        state           <=  S_R3_PW_RUN;
                        for(ii=0; ii<32; ii=ii+1) begin
                            gn_load_data[ii*29 +: 29] <= acc[ii];
                        end
                        gn_load_valid <= 1'b1; gn_load_hi <= 1'b0;
                    end else begin
                        cout_lo_flag    <=  1'd1;
                        sk_hi_addrb     <=  sk_hi_addrb + 1'd1;
                        for(ii=0; ii<32; ii=ii+1) begin
                            gn_load_data[ii*29 +: 29] <= acc[ii];
                        end
                        gn_load_valid <= 1'b1; gn_load_hi <= 1'b1;
                        if(dst_hi_addra == (L4 - 2)) begin
                            dst_hi_addra    <=  dst_hi_addra + 1'd1;
                            gn_load_last <= 1'b1; gn_next_state <= S_B4_INIT;
                            gn_target <= GN_T_RES; state <= S_GN_WAIT;
                        end else begin
                            dst_hi_addra    <=  dst_hi_addra + 1'd1;
                            state           <=  S_R3_PW_RUN;
                        end
                    end
                end
            end

            S_B4_INIT: begin
                gn_start <= 1'b1; gn_cfg_length <= L5; gn_cfg_channels64 <= 1'b0;
                gn_cfg_param_base <= GNBASE_B4; gn_cfg_frac_in <= GNFIN_B4;
                gn_cfg_frac_out <= GNFOUT_B4; gn_cfg_relu6 <= 1'b1;
                src_sel         <=  1'd1;
                wrom_addr       <=  WBASE_B4;
                cur_is_dw       <=  1'd0;
                tap_idx         <=  3'd0;
                src_lo_addrb    <=  11'sd0 - PAD1;
                src_hi_addrb    <=  9'sd0 - PAD1;
                base_ch_issue   <=  6'd0; 
                dst_lo_wea      <=  1'd0;
                dst_hi_wea      <=  1'd0;
                in_time_oob     <=  1'd0;
                state           <=  S_B4_RUN;
                for(ii=0; ii<32; ii=ii+1) begin
                    acc[ii]     <=  29'sd0;
                    acc_ass[ii] <=  20'sd0; 
                end
            end

            S_B4_RUN: begin
                mac_issue_vld   <=  1'd1;
                if((src_lo_addrb < 0) || (src_lo_addrb >= L4)) begin
                    in_time_oob     <=  1'd1;
                end else begin
                    in_time_oob     <=  1'd0;
                end

                if((tap_idx == (K3 - 1)) && (base_ch_issue == 6'd60)) begin
                    tap_idx         <=  3'd0;
                    base_ch_issue   <=  6'd0;
                    src_lo_addrb    <=  src_lo_addrb - (K3 - 1) + STRIDE2;
                    src_hi_addrb    <=  src_hi_addrb - (K3 - 1) + STRIDE2;
                    wrom_addr       <=  WBASE_B4; 
                    state           <=  S_B4_DONE;
                end else begin
                    wrom_addr       <=  wrom_addr + 1'd1;
                    if(base_ch_issue == 6'd60) begin
                        base_ch_issue   <=  6'd0;
                        tap_idx         <=  tap_idx + 1'd1;
                        src_lo_addrb    <=  src_lo_addrb + 1'd1;
                        src_hi_addrb    <=  src_hi_addrb + 1'd1;
                    end else begin
                        base_ch_issue   <=  base_ch_issue + 3'd4;
                    end
                end

                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 9) + arshift_round_sym_32(p1[ii], 9) + arshift_round_sym_32(p2[ii], 9) + arshift_round_sym_32(p3[ii], 9);
                    end
                end else begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= 29'sd0;
                    end
                end
            end

            S_B4_DONE: begin
                mac_issue_vld   <=  1'd0;
                if(vld_pipe[3]) begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        acc[ii] <= acc[ii] + arshift_round_sym_32(p0[ii], 9) + arshift_round_sym_32(p1[ii], 9) + arshift_round_sym_32(p2[ii], 9) + arshift_round_sym_32(p3[ii], 9);
                    end
                end else begin
                    for(ii=0; ii<32; ii=ii+1) begin
                        gn_load_data[ii*29 +: 29] <= acc[ii];
                    end
                    gn_load_valid <= 1'b1; gn_load_hi <= 1'b0;
                    if(src_lo_addrb == ((L5 * STRIDE2) - PAD1)) begin
                        gn_load_last <= 1'b1; gn_next_state <= S_GAP;
                        gn_target <= GN_T_GAP; state <= S_GN_WAIT;
                    end else begin
                        state           <=  S_B4_RUN;
                    end
                end
            end

            // R3 depthwise produces the lower and upper 32 channels together.
            // The GN loader accepts one 32-channel beat per clock, so the upper
            // half is sent in this extra state.
            S_GN_LOAD_HI: begin
                gn_load_valid <= 1'b1;
                gn_load_hi <= 1'b1;
                for(ii=0;ii<32;ii=ii+1)
                    gn_load_data[ii*29 +: 29] <= gn_hi_hold[ii];
                if(gn_hi_last) begin
                    gn_load_last <= 1'b1;
                    gn_next_state <= S_R3_PW_INIT;
                    gn_target <= GN_T_DST;
                    state <= S_GN_WAIT;
                end else begin
                    state <= S_R3_DW_RUN;
                end
            end

            // Common second-pass writeback for every GN layer.
            S_GN_WAIT: begin
                dst_lo_wea <= 1'b0; dst_hi_wea <= 1'b0;
                sk_lo_wea <= 1'b0; sk_hi_wea <= 1'b0;
                if(gn_out_valid) begin
                    case(gn_target)
                        GN_T_DST: begin
                            if(gn_out_hi) begin
                                dst_hi_wea <= 1'b1;
                                dst_hi_addra <= gn_out_time[6:0];
                                dst_hi_dina <= gn_out_data;
                            end else begin
                                dst_lo_wea <= 1'b1;
                                dst_lo_addra <= gn_out_time[8:0];
                                dst_lo_dina <= gn_out_data;
                            end
                        end
                        GN_T_SKIP: begin
                            if(gn_out_hi) begin
                                sk_hi_wea <= 1'b1;
                                sk_hi_addra <= gn_out_time[6:0];
                                sk_hi_dina <= gn_out_data;
                            end else begin
                                sk_lo_wea <= 1'b1;
                                sk_lo_addra <= gn_out_time[6:0];
                                sk_lo_dina <= gn_out_data;
                            end
                        end
                        GN_T_RES: begin
                            sk_lo_addrb <= gn_out_time[6:0];
                            sk_hi_addrb <= gn_out_time[6:0];
                            if(gn_out_hi) begin
                                dst_hi_wea <= 1'b1;
                                dst_hi_addra <= gn_out_time[6:0];
                                for(ii=0;ii<32;ii=ii+1)
                                    dst_hi_dina[ii*20 +: 20] <= relu6_q5_20(
                                        $signed({gn_out_data[ii*20+19],gn_out_data[ii*20 +: 20]})
                                      + ($signed({sk_hi_doutb[ii*20+19],sk_hi_doutb[ii*20 +: 20]}) <<< 1));
                            end else begin
                                dst_lo_wea <= 1'b1;
                                dst_lo_addra <= gn_out_time[8:0];
                                for(ii=0;ii<32;ii=ii+1)
                                    dst_lo_dina[ii*20 +: 20] <= relu6_q5_20(
                                        $signed({gn_out_data[ii*20+19],gn_out_data[ii*20 +: 20]})
                                      + ($signed({sk_lo_doutb[ii*20+19],sk_lo_doutb[ii*20 +: 20]}) <<< 1));
                            end
                        end
                        GN_T_GAP: begin
                            for(ii=0;ii<32;ii=ii+1)
                                acc_ass[ii] <= acc_ass[ii] + $signed(gn_out_data[ii*20 +: 20]);
                        end
                    endcase
                end
                if(gn_done)
                    state <= gn_next_state;
            end

            S_GAP: begin
                state   <=  S_OUT;
                for(ii=0; ii<32; ii=ii+1) begin
                    // B4 GN output is Q*.5. GAP divides the 64 samples by 64
                    // and converts the embedding interface to Q2.7: >>6 then <<2.
                    emb[ii] <= arshift_round_sym_32({12'd0, acc_ass[ii]} , 4);
                end
            end

            S_OUT: begin
                if(class_ready) begin
                    emb_valid   <=  1'd1;
                    state       <=  S_IDLE;
                end else begin
                    state   <=  S_OUT;
                end
            end

            default: state <= S_IDLE;

        endcase
    end
end


/*************************************************************************************\
    例化部分
\*************************************************************************************/
group_norm_engine u_group_norm (
    .clk(clk), .rst(rst), .start(gn_start),
    .cfg_length(gn_cfg_length), .cfg_channels64(gn_cfg_channels64),
    .cfg_param_base(gn_cfg_param_base),
    .cfg_frac_in(gn_cfg_frac_in), .cfg_frac_out(gn_cfg_frac_out),
    .cfg_relu6(gn_cfg_relu6), .load_ready(gn_load_ready),
    .load_valid(gn_load_valid), .load_hi(gn_load_hi),
    .load_data(gn_load_data), .load_last(gn_load_last),
    .out_valid(gn_out_valid), .out_time(gn_out_time),
    .out_hi(gn_out_hi), .out_data(gn_out_data),
    .busy(gn_busy), .done(gn_done)
);

blk_mem_gen_feat6 u_ram_iq (
    .clka   (clk                ),      // input wire clka
    .ena    (1'b1               ),      // keep write port enabled
    .wea    (ram_iq_wea         ),      // input wire [0 : 0] wea
    .addra  (ram_iq_addra       ),      // input wire [9 : 0] addra
    .dina   (ram_iq_dina        ),      // input wire [71 : 0] dina
    .clkb   (clk                ),      // input wire clkb
    .enb    (1'b1               ),      // keep read port enabled
    .addrb  (ram_iq_addrb[9:0]  ),      // input wire [9 : 0] addrb
    .doutb  (ram_iq_doutb       )       // output wire [71 : 0] doutb
);

blk_mem_gen_6 u_rama_lo (
    .clka   (clk                ),      // input wire clka
    .wea    (rama_lo_wea        ),      // input wire [0 : 0] wea
    .addra  (rama_lo_addra      ),      // input wire [8 : 0] addra
    .dina   (rama_lo_dina       ),      // input wire [639 : 0] dina
    .clkb   (clk                ),      // input wire clkb
    .addrb  (rama_lo_addrb      ),      // input wire [8 : 0] addrb
    .doutb  (rama_lo_doutb      )       // output wire [639 : 0] doutb
);

blk_mem_gen_6 u_ramb_lo (
    .clka   (clk                ),      // input wire clka
    .wea    (ramb_lo_wea        ),      // input wire [0 : 0] wea
    .addra  (ramb_lo_addra      ),      // input wire [8 : 0] addra
    .dina   (ramb_lo_dina       ),      // input wire [639 : 0] dina
    .clkb   (clk                ),      // input wire clkb
    .addrb  (ramb_lo_addrb      ),      // input wire [8 : 0] addrb
    .doutb  (ramb_lo_doutb      )       // output wire [639 : 0] doutb
);

blk_mem_gen_7 u_rama_hi (
    .clka   (clk                ),      // input wire clka
    .wea    (rama_hi_wea        ),      // input wire [0 : 0] wea
    .addra  (rama_hi_addra      ),      // input wire [6 : 0] addra
    .dina   (rama_hi_dina       ),      // input wire [639 : 0] dina
    .clkb   (clk                ),      // input wire clkb
    .addrb  (rama_hi_addrb      ),      // input wire [6 : 0] addrb
    .doutb  (rama_hi_doutb      )       // output wire [639 : 0] doutb
);

blk_mem_gen_7 u_ramb_hi (
    .clka   (clk                ),      // input wire clka
    .wea    (ramb_hi_wea        ),      // input wire [0 : 0] wea
    .addra  (ramb_hi_addra      ),      // input wire [6 : 0] addra
    .dina   (ramb_hi_dina       ),      // input wire [639 : 0] dina
    .clkb   (clk                ),      // input wire clkb
    .addrb  (ramb_hi_addrb      ),      // input wire [6 : 0] addrb
    .doutb  (ramb_hi_doutb      )       // output wire [639 : 0] doutb
);

blk_mem_gen_7 u_sk_lo (
    .clka   (clk                ),      // input wire clka
    .wea    (sk_lo_wea          ),      // input wire [0 : 0] wea
    .addra  (sk_lo_addra        ),      // input wire [6 : 0] addra
    .dina   (sk_lo_dina         ),      // input wire [639 : 0] dina
    .clkb   (clk                ),      // input wire clkb
    .addrb  (sk_lo_addrb        ),      // input wire [6 : 0] addrb
    .doutb  (sk_lo_doutb        )       // output wire [639 : 0] doutb
);

blk_mem_gen_7 u_sk_hi (
    .clka   (clk                ),      // input wire clka
    .wea    (sk_hi_wea          ),      // input wire [0 : 0] wea
    .addra  (sk_hi_addra        ),      // input wire [6 : 0] addra
    .dina   (sk_hi_dina         ),      // input wire [639 : 0] dina
    .clkb   (clk                ),      // input wire clkb
    .addrb  (sk_hi_addrb        ),      // input wire [6 : 0] addrb
    .doutb  (sk_hi_doutb        )       // output wire [639 : 0] doutb
);

blk_mem_gen_8 u_wrom0 (
    .clka   (clk                ),      // input wire clka 
    .addra  (wrom_addr          ),      // input wire [7 : 0] addra 
    .douta  (wrom0_dout         )       // output wire [383 : 0] douta
);

blk_mem_gen_9 u_wrom1 (
    .clka   (clk                ),      // input wire clka 
    .addra  (wrom_addr          ),      // input wire [7 : 0] addra 
    .douta  (wrom1_dout         )       // output wire [383 : 0] douta
);

blk_mem_gen_10 u_wrom2 (
    .clka   (clk                ),      // input wire clka 
    .addra  (wrom_addr          ),      // input wire [7 : 0] addra 
    .douta  (wrom2_dout         )       // output wire [383 : 0] douta
);

blk_mem_gen_11 u_wrom3 (
    .clka   (clk                ),      // input wire clka 
    .addra  (wrom_addr          ),      // input wire [7 : 0] addra 
    .douta  (wrom3_dout         )       // output wire [383 : 0] douta
);

genvar oc;
generate
    for(oc=0; oc<32; oc=oc+1) begin : GEN_MUL
        mult_gen_3 u_m0 (
            .CLK  (clk              ),  // input wire CLK 
            .A    (x_conv0[oc]      ),  // input wire [19 : 0] A 
            .B    (wrom0_dout_r[oc] ),  // input wire [11 : 0] B 
            .P    (p0[oc]           )   // output wire [31 : 0] P
        );

        mult_gen_3 u_m1 (
            .CLK  (clk              ),  // input wire CLK 
            .A    (x_conv1[oc]      ),  // input wire [19 : 0] A 
            .B    (wrom1_dout_r[oc] ),  // input wire [11 : 0] B 
            .P    (p1[oc]           )   // output wire [31 : 0] P
        );

        mult_gen_3 u_m2 (
            .CLK  (clk              ),  // input wire CLK 
            .A    (x_conv2          ),  // input wire [19 : 0] A 
            .B    (wrom2_dout_r[oc] ),  // input wire [11 : 0] B 
            .P    (p2[oc]           )   // output wire [31 : 0] P
        );

        mult_gen_3 u_m3 (
            .CLK  (clk              ),  // input wire CLK 
            .A    (x_conv3          ),  // input wire [19 : 0] A 
            .B    (wrom3_dout_r[oc] ),  // input wire [11 : 0] B 
            .P    (p3[oc]           )   // output wire [31 : 0] P
        );
    end
endgenerate

endmodule
