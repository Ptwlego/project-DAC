`timescale 1ns/1ps
module top_d_e_display #(
    parameter integer CLK_HZ = 100_000_000,
    parameter         SEG_COMMON_ANODE = 1'b1,
    parameter integer LCD_REFRESH_MS   = 50,
    parameter integer DEBOUNCE_MS      = 3
)(
    input  wire        clk,
    input  wire        rstn,
    input  wire        btn_p,
    input  wire        btn_m,
    input  wire        btn_p2,
    input  wire        btn_m2,
    input  wire        btn_p8,
    input  wire        btn_m8,
    output wire [7:0]  dac_d,
    output wire        dac_wr_n,
    output wire        dac_csn,
    output wire        dac_ldac_n,
    output wire [3:0]  seg_an,
    output wire [7:0]  seg,
    output wire        lcd_rs,
    output wire        lcd_en,
    output wire [3:0]  lcd_d
);
    localparam integer DB_CYCLES = (CLK_HZ/1000)*DEBOUNCE_MS;
    wire db_p, db_m, db_p2, db_m2, db_p8, db_m8;
    sync_debounce #(.CYCLES(DB_CYCLES), .ACTIVE_LOW(1'b0)) u_db_p  (.clk(clk), .rstn(rstn), .din(btn_p),  .dout(db_p));
    sync_debounce #(.CYCLES(DB_CYCLES), .ACTIVE_LOW(1'b0)) u_db_m  (.clk(clk), .rstn(rstn), .din(btn_m),  .dout(db_m));
    sync_debounce #(.CYCLES(DB_CYCLES), .ACTIVE_LOW(1'b0)) u_db_p2 (.clk(clk), .rstn(rstn), .din(btn_p2), .dout(db_p2));
    sync_debounce #(.CYCLES(DB_CYCLES), .ACTIVE_LOW(1'b0)) u_db_m2 (.clk(clk), .rstn(rstn), .din(btn_m2), .dout(db_m2));
    sync_debounce #(.CYCLES(DB_CYCLES), .ACTIVE_LOW(1'b0)) u_db_p8 (.clk(clk), .rstn(rstn), .din(btn_p8), .dout(db_p8));
    sync_debounce #(.CYCLES(DB_CYCLES), .ACTIVE_LOW(1'b0)) u_db_m8 (.clk(clk), .rstn(rstn), .din(btn_m8), .dout(db_m8));

    wire [7:0]  w_dac_d;
    wire        w_dac_wr_n, w_dac_csn, w_dac_ldac_n;

    dac_ctrl_step u_dac (
        .clk(clk), .rstn(rstn),
        .btn_p(db_p), .btn_m(db_m),
        .btn_p2(db_p2), .btn_m2(db_m2),
        .btn_p8(db_p8), .btn_m8(db_m8),
        .dac_d(w_dac_d), .dac_wr_n(w_dac_wr_n),
        .dac_csn(w_dac_csn), .dac_ldac_n(w_dac_ldac_n)
    );

    assign dac_d      = w_dac_d;
    assign dac_wr_n   = w_dac_wr_n;
    assign dac_csn    = w_dac_csn;
    assign dac_ldac_n = w_dac_ldac_n;

    wire [3:0] H,T,O;
    bin8_to_bcd3 u_bcd(.bin(w_dac_d), .hundreds(H), .tens(T), .ones(O));

    sevenseg_4dig #(
        .CLK_HZ(CLK_HZ),
        .COMMON_ANODE(SEG_COMMON_ANODE),
        .SHOW_WR_DOT(1'b1)
    ) u_seg (
        .clk(clk), .rstn(rstn),
        .H(H), .T(T), .O(O),
        .wr_pulse(~w_dac_wr_n),
        .an(seg_an), .seg(seg)
    );

    wire [127:0] line0;
    lcd_text_gen_line1_only u_txt(.dac_d(w_dac_d), .line0_ascii(line0));

    lcd_hd44780_4bit_simple #(
        .CLK_HZ(CLK_HZ),
        .REFRESH_MS(LCD_REFRESH_MS)
    ) u_lcd (
        .clk(clk), .rstn(rstn),
        .line0(line0),
        .rs(lcd_rs), .en(lcd_en), .d(lcd_d)
    );
endmodule

module dac_ctrl_step (
    input  wire       clk,
    input  wire       rstn,
    input  wire       btn_p,
    input  wire       btn_m,
    input  wire       btn_p2,
    input  wire       btn_m2,
    input  wire       btn_p8,
    input  wire       btn_m8,
    output reg [7:0]  dac_d,
    output reg        dac_wr_n,
    output wire       dac_csn,
    output wire       dac_ldac_n
);
    assign dac_csn    = 1'b0;
    assign dac_ldac_n = 1'b0;
    reg signed [8:0] delta;
    always @* begin
        casex ({btn_p8, btn_p2, btn_p, btn_m8, btn_m2, btn_m})
            6'b1xxxxx: delta =  9'sd8;
            6'b01xxxx: delta =  9'sd2;
            6'b001xxx: delta =  9'sd1;
            6'b0001xx: delta = -9'sd8;
            6'b00001x: delta = -9'sd2;
            6'b000001: delta = -9'sd1;
            default  : delta =  9'sd0;
        endcase
    end
    wire trig = (delta != 0);
    function [7:0] sat_add;
        input [7:0] base;
        input signed [8:0] d;
        reg signed [9:0] s;
        begin
            s = $signed({1'b0,base}) + d;
            if (s < 0)       sat_add = 8'd0;
            else if (s >255) sat_add = 8'd255;
            else             sat_add = s[7:0];
        end
    endfunction
    reg [7:0] dac_next;
    localparam S_IDLE  = 2'd0;
    localparam S_WR_LO = 2'd1;
    localparam S_WR_HI = 2'd2;
    reg [1:0]  state, nstate;
    reg [2:0]  cnt;
    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            state    <= S_IDLE;
            dac_wr_n <= 1'b1;
            dac_d    <= 8'd128;
            dac_next <= 8'd128;
            cnt      <= 3'd0;
        end else begin
            state <= nstate;
            case(state)
                S_IDLE:  begin dac_wr_n <= 1'b1; cnt <= 3'd0; if(trig) dac_next <= sat_add(dac_d, delta); end
                S_WR_LO: begin dac_wr_n <= 1'b0; cnt <= cnt + 1'b1; end
                S_WR_HI: begin dac_wr_n <= 1'b1; cnt <= cnt + 1'b1; end
            endcase
            if(state==S_WR_HI && nstate==S_IDLE) dac_d <= dac_next;
        end
    end
    always @* begin
        nstate = state;
        case(state)
            S_IDLE  : nstate = (trig ? S_WR_LO : S_IDLE);
            S_WR_LO : nstate = (cnt==3'd3 ? S_WR_HI : S_WR_LO);
            S_WR_HI : nstate = (cnt==3'd7 ? S_IDLE  : S_WR_HI);
            default : nstate = S_IDLE;
        endcase
    end
endmodule

module bin8_to_bcd3(
  input  [7:0] bin,
  output [3:0] hundreds, tens, ones
);
  wire [9:0] h9 = bin / 100;
  wire [6:0] t7 = (bin % 100) / 10;
  wire [3:0] o4 = bin % 10;
  assign hundreds = h9[3:0];
  assign tens     = t7[3:0];
  assign ones     = o4[3:0];
endmodule

module sevenseg_4dig #(
    parameter integer CLK_HZ = 100_000_000,
    parameter         COMMON_ANODE = 1'b1,
    parameter         SHOW_WR_DOT  = 1'b1
)(
    input  wire       clk, rstn,
    input  wire [3:0] H, T, O,
    input  wire       wr_pulse,
    output reg  [3:0] an,
    output reg  [7:0] seg
);
    localparam integer SCAN_HZ = 4000;
    localparam integer DIV = CLK_HZ / SCAN_HZ;
    reg [$clog2(DIV)-1:0] divc;
    reg [1:0] sel;
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin divc<=0; sel<=0; end
        else begin
            if(divc==DIV-1) begin divc<=0; sel<=sel+1'b1; end
            else divc<=divc+1'b1;
        end
    end
    function [6:0] enc; input [3:0] d;
    begin
        case(d)
        4'd0: enc=7'b1000000; 4'd1: enc=7'b1111001;
        4'd2: enc=7'b0100100; 4'd3: enc=7'b0110000;
        4'd4: enc=7'b0011001; 4'd5: enc=7'b0010010;
        4'd6: enc=7'b0000010; 4'd7: enc=7'b1111000;
        4'd8: enc=7'b0000000; 4'd9: enc=7'b0010000;
        default: enc=7'b1111111; endcase
    end endfunction
    wire blankH = (H==0);
    wire blankT = blankH && (T==0);
    reg [6:0] sev; reg dp;
    always @* begin
        dp = 1'b1;
        case(sel)
            2'd0: begin sev=7'b1111111; dp=1'b1; an = COMMON_ANODE ? 4'b1110 : 4'b0001; end
            2'd1: begin sev= blankH ? 7'b1111111 : enc(H); an = COMMON_ANODE ? 4'b1101 : 4'b0010; end
            2'd2: begin sev= blankT ? 7'b1111111 : enc(T); an = COMMON_ANODE ? 4'b1011 : 4'b0100; end
            2'd3: begin sev= enc(O); dp = (SHOW_WR_DOT && wr_pulse) ? 1'b0 : 1'b1; an = COMMON_ANODE ? 4'b0111 : 4'b1000; end
        endcase
        seg = {dp, sev};
        if(!COMMON_ANODE) seg = ~seg;
    end
endmodule

module lcd_text_gen_line1_only (
    input  wire [7:0]   dac_d,
    output wire [127:0] line0_ascii
);
    wire [3:0] H = dac_d / 100;
    wire [3:0] T = (dac_d % 100) / 10;
    wire [3:0] O = dac_d % 10;
    wire [7:0] chH = "0" + H, chT = "0" + T, chO = "0" + O;
    assign line0_ascii = {
        8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,8'h20,
        8'h20,8'h20,8'h20,8'h20,
        chO, chT, chH,
        "=", "C", "A", "D"
    };
endmodule

module lcd_hd44780_4bit_simple #(
    parameter integer CLK_HZ = 100_000_000,
    parameter integer REFRESH_MS = 50
)(
    input  wire        clk, rstn,
    input  wire [127:0] line0,
    output reg         rs, en,
    output reg  [3:0]  d
);
    localparam integer US = CLK_HZ/1_000_000;
    localparam integer MS = CLK_HZ/1_000;
    reg [31:0] timer;
    task delay_us(input integer us); begin timer <= us*US; end endtask
    task delay_ms(input integer ms); begin timer <= ms*MS; end endtask
    reg [7:0]  tx_byte;
    reg        tx_cmd;
    reg [1:0]  tx_phase;
    reg        tx_busy, tx_start;
    localparam [3:0] F_SET=4'd0, D_ON=4'd1, CLEAR=4'd2, ENTRY=4'd3, SET_DDR=4'd4, PRINT=4'd5, REFRESH=4'd6;
    reg [3:0]  st;
    reg [4:0]  idx;
    reg [127:0] line_l;
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin
            en<=0; rs<=0; d<=0; tx_phase<=0; tx_busy<=0; tx_start<=0;
        end else begin
            if(tx_start && !tx_busy) begin
                tx_busy  <= 1'b1; tx_phase <= 2'd0; tx_start <= 1'b0;
            end
            if(tx_busy) begin
                case(tx_phase)
                    2'd0: begin rs<=~tx_cmd; d<=tx_byte[7:4]; en<=1'b1; tx_phase<=2'd1; delay_us(1); end
                    2'd1: begin en<=1'b0; tx_phase<=2'd2; delay_us(40); end
                    2'd2: begin rs<=~tx_cmd; d<=tx_byte[3:0]; en<=1'b1; tx_phase<=2'd3; delay_us(1); end
                    2'd3: begin en<=1'b0; tx_busy<=1'b0; delay_us(40); end
                endcase
            end else if (timer!=0) begin
                timer <= timer - 1;
            end
        end
    end
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin
            st<=F_SET; idx<=0; line_l<=0; timer<=0; tx_start<=0; tx_busy<=0;
            rs<=0; en<=0; d<=0; delay_ms(20);
        end else begin
            if(timer!=0 || tx_busy) begin
            end else begin
                case(st)
                F_SET:   begin tx_cmd<=1'b1; tx_byte<=8'h28; tx_start<=1'b1; st<=D_ON;   end
                D_ON:    begin tx_cmd<=1'b1; tx_byte<=8'h0C; tx_start<=1'b1; st<=CLEAR; end
                CLEAR:   begin tx_cmd<=1'b1; tx_byte<=8'h01; tx_start<=1'b1; delay_ms(2); st<=ENTRY; end
                ENTRY:   begin tx_cmd<=1'b1; tx_byte<=8'h06; tx_start<=1'b1; st<=SET_DDR; end
                SET_DDR: begin tx_cmd<=1'b1; tx_byte<=8'h80; tx_start<=1'b1; idx<=0; line_l<=line0; st<=PRINT; end
                PRINT:   begin tx_cmd<=1'b0; tx_byte<=line_l[127 - idx*8 -: 8]; tx_start<=1'b1;
                                  idx<=idx+1'b1; if(idx==5'd15) begin st<=REFRESH; delay_ms(REFRESH_MS); end end
                REFRESH: begin st<=SET_DDR; end
                endcase
            end
        end
    end
endmodule

module sync_debounce #(
  parameter integer CYCLES = 100_000,
  parameter         ACTIVE_LOW = 1'b0
)(
  input  wire clk,
  input  wire rstn,
  input  wire din,
  output reg  dout
);
  wire din_n = ACTIVE_LOW ? ~din : din;
  reg s0, s1;
  always @(posedge clk or negedge rstn) begin
    if(!rstn) {s0,s1} <= 2'b00; else {s0,s1} <= {din_n, s0};
  end
  localparam W = (CYCLES<=1) ? 1 : $clog2(CYCLES);
  reg [W-1:0] acc;
  always @(posedge clk or negedge rstn) begin
    if(!rstn) begin acc<=0; dout<=1'b0; end
    else if (s1==dout) acc<=0;
    else if (acc==CYCLES-1) begin dout<=s1; acc<=0; end
    else acc<=acc+1'b1;
  end
endmodule
