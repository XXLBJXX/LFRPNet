`timescale 1ns / 1ps
/**********************************************************************************************************************\
Copyright(c) 2010, UESTC Technology Inc, All right reserved Department

Department      : 电子科技大学测控技术与仪器研究所
Author          : DingWei
Project Name    : Automatic_Modulation_Recognition
Product Ver     : V1.0
Module Name     : input_norm.v
Module Called   : none
Called By       : 
Target Device   : xc7z020iclg400
Tools Version   : vivado 2018.3

Description     : 输入归一化模块模块

Modification History:
Date          By         Review         Rev.       Res        Ncp        PR-NO.STAGE        Change Description
-------------------------------------------------------------------------------------------------------------------------------
2026/2/4	DingWei		   无		    v1.0		无		   无			 无			     创建模块，实现基本功能
\*****************************************************************************************************************************/


module input_norm(
    output wire          input_ready,
    input  wire          wr_clk,                           // 写时钟
    input  wire          rd_clk,                           // 读时钟
    input  wire          rst,                              // 系统复位，高有效    
    input  wire  [15:0]  input_i,                          // I路原始数据
    input  wire  [15:0]  input_q,                          // Q路原始数据
    input  wire          input_valid,                      // 输入数据有效指示，高有效    
    input  wire          param_est_ready,                  // 相位重整参数估计模块ready信号，高有效
    input  wire          amr_en,                           // 调制识别使能，上升沿有效
    input  wire  [9:0]   amr_times,                        // 一轮测试需识别样本数
        
    output reg   [11:0]  norm_i,                           // 归一化后 I路数据
    output reg   [11:0]  norm_q,                           // 归一化后 Q路数据
    output reg           norm_valid                        // 输出数据有效指示，高有效   
    );


/*************************************************************************************\
    parameter_declaration
\*************************************************************************************/
// 常数定义
localparam        [10:0] FRAME_LEN  =   11'd1024;          // 单帧数据长度
// localparam        [7:0]  AMR_TIMES  =   8'd200;            // 单次测试识别次数

// Q3.8饱和截断范围
localparam signed [11:0] SAT_MAX    =   12'sd2047;        
localparam signed [11:0] SAT_MIN    =   -12'sd2048;
reg                       ram_we     =   1'b0;

// 写状态机定义
localparam        [1:0]  W_IDLE     =   2'd0;              // 空闲状态
localparam        [1:0]  W_RAM_IN   =   2'd1;              // RAM写入状态
localparam        [1:0]  W_DONE     =   2'd2;              // 完成状态

// 读状态机定义
localparam        [1:0]  R_IDLE     =   2'd0;              // 空闲状态
localparam        [1:0]  R_SQRT     =   2'd1;              // 开方状态
localparam        [1:0]  R_DIV      =   2'd2;              // 除法状态
localparam        [1:0]  R_STREAM   =   2'd3;              // 流式输出状态


/*************************************************************************************\
    reg_declaration
\*************************************************************************************/
// 状态寄存器
reg        [1:0]   wr_state         =   W_IDLE;            // 写时钟域当前状态
reg        [1:0]   rd_state         =   R_IDLE;            // 读时钟域当前状态

// 写时钟域数据寄存器
reg        [31:0]  pwr_sum          =   32'd0;             // 功率累加和中间变量
reg        [41:0]  total_energy     =   42'd0;             // 一帧总功率累加结果
reg        [31:0]  ram_data_in      =   32'd0;             // RAM输入数据
reg        [4:0]   valid_pipe       =   5'd0;              // 输入流水线valid打拍

// 读时钟域数据寄存器
reg        [41:0]  energy_latched   =   42'd0;             // 总功率锁存
reg        [5:0]   out_valid_pipe   =   6'd0;              // 输出流水线Valid打拍
reg        [36:0]  round_i          =   37'd0;             // I路补位
reg        [36:0]  round_q          =   37'd0;             // Q路补位
reg        [23:0]  shifted_i        =   24'd0;             // I路移位
reg        [23:0]  shifted_q        =   24'd0;             // Q路移位

// 控制寄存器
reg                cordic_start     =   1'd0;              // Cordic使能
reg                div_start        =   1'd0;              // Divider使能

// 标志寄存器
reg                wr_done          =   1'd0;              // 数据写完成标志
reg                rd_done          =   1'd0;              // 数据读完成标志

// CDC寄存器
reg        [2:0]   amr_en_r         =   3'd0;              // 调制识别使能寄存
reg        [2:0]   wr_done_r        =   3'd0;              // 数据写完成标志寄存
reg        [2:0]   rd_done_r        =   3'd0;              // 数据读完成标志寄存

// 地址与计数寄存器
reg        [9:0]   wr_addr          =   10'h3FF;           // RAM写地址
reg        [9:0]   rd_addr          =   10'd0;             // RAM读地址
reg        [9:0]   cnt_accum        =   10'd0;             // 累加点数计数器
reg        [9:0]   cnt_frame        =   10'd0;             // 调制识别次数计数器


/*************************************************************************************\
    wire_declaration
\*************************************************************************************/
//标志信号
wire               amr_en_p;                               // 调制识别使能上升沿
wire               wr_done_p;                              // 数据写完成标志上升沿
wire               rd_done_p;                              // 数据读完成标志上升沿

// IP核互联信号
wire        [31:0] ram_dout_b;                             // RAM B口读出数据
wire        [31:0] pwr_i_sq;                               // I路平方
wire        [31:0] pwr_q_sq;                               // Q路平方

// Cordic相关信号
wire               cordic_done;                            // Cordic计算完成
wire        [47:0] cordic_in_padded;                       // Cordic输入
wire        [23:0] cordic_out;                             // Cordic输出

// Divider相关信号
wire               div_done;                               // Divider计算完成
wire        [23:0] div_divisor;                            // 除数
wire        [7:0]  div_dividend;                           // 被除数
wire        [31:0] div_out;                                // 除法器输出

// 输出计算信号
wire        [20:0] norm_coef;                              // 归一化系数
wire        [36:0] mult_out_i;                             // 归一化结果 I路
wire        [36:0] mult_out_q;                             // 归一化结果 Q路


/*************************************************************************************\
    assign_logic
\*************************************************************************************/
// Cordic
assign  cordic_in_padded    =   {6'd0, energy_latched};

// Divider
assign  div_divisor         =   cordic_out;                // Cordic输出模值
assign  div_dividend        =   8'd32;                     
assign  norm_coef           =   div_out[23:3];
assign  input_ready         =   (wr_state == W_RAM_IN);

// 上升沿检测
assign  amr_en_p            =   amr_en_r[1] & !amr_en_r[2];
assign  wr_done_p           =   wr_done_r[1] & !wr_done_r[2];
assign  rd_done_p           =   rd_done_r[1] & !rd_done_r[2];


/*************************************************************************************\
--------------------------跨时钟域处理---------------------------
\*************************************************************************************/
always @(posedge wr_clk or posedge rst) begin
    if(rst) begin
        amr_en_r    <= 3'd0;
        rd_done_r   <=  3'd0;
    end else begin
        amr_en_r    <=  {amr_en_r[1:0], amr_en};
        rd_done_r   <=  {rd_done_r[1:0], rd_done};
    end
end

always @(posedge rd_clk or posedge rst) begin
    if(rst) begin
        wr_done_r   <=  3'd0;
    end else begin
        wr_done_r   <=  {wr_done_r[1:0], wr_done};
    end
end


/*************************************************************************************\
----------------------写时钟域状态机-------------------------
\*************************************************************************************/
always @(posedge wr_clk or posedge rst) begin
    if(rst) begin
        ram_data_in     <=  32'd0;
        ram_we          <=  1'b0;
        wr_state        <=  W_IDLE;
        wr_addr         <=  10'd1023;
        cnt_frame       <=  10'd0;
    end else begin
        ram_we <= (wr_state == W_RAM_IN) && input_valid;
        case(wr_state)
            W_IDLE: begin
                if(amr_en_p) begin
                    wr_state    <=  W_RAM_IN;
                    cnt_frame   <=  10'd0;
                end else
                    wr_state    <=  W_IDLE;
            end

            W_RAM_IN: begin
                if(input_valid) begin
                    ram_data_in <=  {input_i, input_q};
                    if(wr_addr == FRAME_LEN - 2) begin
                        wr_addr     <=  wr_addr + 1'd1;
                        wr_state    <=  W_DONE;
                    end else begin
                        wr_addr     <=  wr_addr + 1'd1;
                        wr_state    <=  W_RAM_IN;
                    end
                end else begin
                    wr_state    <=  W_RAM_IN;
                end
            end
            
            W_DONE: begin
                if(rd_done_p) begin
                    if(cnt_frame >= (amr_times - 1)) begin
                        wr_state    <=  W_IDLE;
                        cnt_frame   <=  10'd0;
                    end else begin
                        wr_state    <=  W_RAM_IN;
                        cnt_frame   <=  cnt_frame + 1'd1;
                    end
                end else begin
                    wr_state    <=  W_DONE;
                end
            end
        endcase
    end
end


/*************************************************************************************\
-------------------------input_valid打拍-----------------------
\*************************************************************************************/
always @(posedge wr_clk or posedge rst) begin
    if(rst) begin 
        valid_pipe  <=  5'd0;
    end else begin
        if(wr_state == W_RAM_IN)
            valid_pipe <= {valid_pipe[3:0], input_valid};
        else
            valid_pipe <= {valid_pipe[3:0], 1'd0};
    end
end

always @(posedge wr_clk or posedge rst) begin
    if(rst) begin
        pwr_sum         <=  32'd0;
        total_energy    <=  42'd0;
        cnt_accum       <=  10'd0;
        wr_done         <=  1'd0;
    end else begin
        pwr_sum <= pwr_i_sq[30:0] + pwr_q_sq[30:0];
        if(valid_pipe[4]) begin
            if (cnt_accum == 0) begin
                cnt_accum       <=  cnt_accum + 1'd1;
                total_energy    <=  pwr_sum;
            end else begin
                total_energy <= total_energy + pwr_sum;
                if(cnt_accum == FRAME_LEN - 1) begin
                    wr_done     <=  1'd1;
                    cnt_accum   <=  cnt_accum + 1'd1;
                end else begin
                    wr_done     <=  1'd0;
                    cnt_accum   <=  cnt_accum + 1'd1;
                end
            end
        end else begin
            total_energy    <=  total_energy;
        end
    end
end


/*************************************************************************************\
----------------------读时钟域状态机-------------------------
\*************************************************************************************/
always @(posedge rd_clk or posedge rst) begin
    if(rst) begin
        rd_state        <=  R_IDLE;
        energy_latched  <=  42'd0;
        cordic_start    <=  1'd0;
        div_start       <=  1'd0;
        rd_addr         <=  10'd0;
        rd_done         <=  1'd0;
    end else begin
        case(rd_state)
            R_IDLE: begin
                if(wr_done_p) begin
                    rd_done         <=  1'd0;
                    energy_latched  <=  total_energy;   
                    rd_state        <=  R_SQRT;
                    cordic_start    <=  1'd1;           
                end else begin
                    cordic_start    <=  1'd0;
                end
            end
            
            R_SQRT: begin
                if(cordic_done) begin
                    rd_state        <=  R_DIV;
                    cordic_start    <=  1'd0;
                    div_start       <=  1'd1;           
                end else begin
                    div_start       <=  1'd0; 
                end
            end
            
            R_DIV: begin
                if(div_done) begin
                    if(param_est_ready) begin            
                        div_start   <=  1'd0;
                        rd_addr     <=  10'd0;
                        rd_state    <=  R_STREAM;
                    end else begin
                        rd_state    <=  R_DIV;
                    end
                end else begin
                    rd_state    <=  R_DIV;
                end
            end
            
            R_STREAM: begin
                if(rd_addr == FRAME_LEN - 1) begin
                    rd_done     <=  1'd1;
                    rd_state    <=  R_IDLE;
                end else begin
                    rd_addr         <=  rd_addr + 1'd1;
                end
            end
        endcase
    end
end


/*************************************************************************************\
-------------------norm_valid打拍--------------------
\*************************************************************************************/
always @(posedge rd_clk or posedge rst) begin
    if(rst) begin 
        out_valid_pipe  <=  6'd0;
        norm_valid    <=  1'd0;
    end else begin
        norm_valid    <=  out_valid_pipe[5];
        if(rd_state == R_STREAM) begin
            out_valid_pipe <= {out_valid_pipe[4:0], 1'b1};
        end else begin
            out_valid_pipe <= {out_valid_pipe[4:0], 1'b0};
        end
    end
end

always @(posedge rd_clk) begin
    if(mult_out_i[36]) begin
        round_i <=  mult_out_i - 37'd4096;
    end else begin
        round_i <=  mult_out_i + 37'd4096;
    end

    if(mult_out_q[36]) begin
        round_q <=  mult_out_q - 37'd4096;
    end else begin
        round_q <=  mult_out_q + 37'd4096;
    end
    
    if(round_i[36]) begin
        shifted_i   <=  ~((~round_i + 1'd1) >> 13) + 1'd1;
    end
    else begin
        shifted_i   <=  round_i >> 13;
    end
    
    if(round_q[36]) begin
        shifted_q   <=  ~((~round_q + 1'd1) >> 13) + 1'd1;
    end
    else begin
        shifted_q   <=  round_q >> 13;
    end
    
    if($signed(shifted_i) > SAT_MAX) 
        norm_i   <=  SAT_MAX;
    else if($signed(shifted_i) < SAT_MIN) 
        norm_i   <=  SAT_MIN;
    else 
        norm_i   <=  shifted_i[11:0];
    
    if($signed(shifted_q) > SAT_MAX) 
        norm_q   <= SAT_MAX;
    else if($signed(shifted_q) < SAT_MIN) 
        norm_q   <= SAT_MIN;
    else 
        norm_q   <= shifted_q[11:0];
end


/*************************************************************************************\
    例化部分
\*************************************************************************************/
blk_mem_gen_0 u_ram_norm (
    .clka   (wr_clk                          ),  // input wire clka
    .wea    (ram_we                          ),  // input wire [0 : 0] wea
    .addra  (wr_addr                         ),  // input wire [9 : 0] addra
    .dina   (ram_data_in                     ),  // input wire [31 : 0] dina    
    .clkb   (rd_clk                          ),  // input wire clkb
    .addrb  (rd_addr                         ),  // input wire [9 : 0] addrb
    .doutb  (ram_dout_b                      )   // output wire [31 : 0] doutb
);

mult_gen_0 u_mult_i_sq (
    .CLK    (wr_clk                          ),  // input wire CLK
    .A      (input_i                         ),  // input wire [15 : 0] A
    .B      (input_i                         ),  // input wire [15 : 0] B
    .P      (pwr_i_sq                        )   // output wire [31 : 0] P
);

mult_gen_0 u_mult_q_sq (
    .CLK    (wr_clk                          ),  // input wire CLK
    .A      (input_q                         ),  // input wire [15 : 0] A
    .B      (input_q                         ),  // input wire [15 : 0] B
    .P      (pwr_q_sq                        )   // output wire [31 : 0] P
);

cordic_0 u_cordic (
    .aclk                   (rd_clk          ),  // input wire aclk
    .aresetn                (~rst           ),   // input wire aresetn
    .s_axis_cartesian_tvalid(cordic_start    ),  // input wire s_axis_cartesian_tvalid
    .s_axis_cartesian_tdata (cordic_in_padded),  // input wire [47 : 0] s_axis_cartesian_tdata
    .m_axis_dout_tvalid     (cordic_done     ),  // output wire m_axis_dout_tvalid
    .m_axis_dout_tdata      (cordic_out      )   // output wire [23 : 0] m_axis_dout_tdata
);

div_gen_0 u_div (
    .aclk                   (rd_clk          ),  // input wire aclk
    .aresetn                (~rst           ),   // input wire aresetn
    .s_axis_divisor_tvalid  (div_start       ),  // input wire s_axis_divisor_tvalid
    .s_axis_divisor_tdata   (div_divisor     ),  // input wire [23 : 0] s_axis_divisor_tdata
    .s_axis_dividend_tvalid (div_start       ),  // input wire s_axis_dividend_tvalid
    .s_axis_dividend_tdata  (div_dividend    ),  // input wire [7 : 0] s_axis_dividend_tdata
    .m_axis_dout_tvalid     (div_done        ),  // output wire m_axis_dout_tvalid
    .m_axis_dout_tdata      (div_out         )   // output wire [31 : 0] m_axis_dout_tdata
);

mult_gen_1 u_mult_i_out (
    .CLK    (rd_clk                          ),  // input wire CLK
    .A      (norm_coef                       ),  // input wire [20 : 0] A
    .B      (ram_dout_b[31:16]               ),  // input wire [15 : 0] B (I路)
    .P      (mult_out_i                      )   // output wire [36 : 0] P
);

mult_gen_1 u_mult_q_out (
    .CLK    (rd_clk                          ),  // input wire CLK
    .A      (norm_coef                       ),  // input wire [20 : 0] A
    .B      (ram_dout_b[15:0]                ),  // input wire [15 : 0] B (Q路)
    .P      (mult_out_q                      )   // output wire [36 : 0] P
);

endmodule
