`timescale 1ns / 1ps

`define state_push_initial 3'd0
`define state_push_read0 3'd1
`define state_push_read1 3'd2
`define state_push_write 3'd3
`define state_first 3'd4
`define state_count0 3'd5
`define state_count1 3'd6
`define state_done 3'd7

module fir
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  wire                      awready,// FIR transmit to TESTBENCH
    output  wire                      wready,//write data ,FIR transmit to TESTBENCH
    input   wire                     awvalid,//testbench(master) can transmit to FIR(write address)
    input   wire [(pADDR_WIDTH-1):0] awaddr,//testbench(master) transmit to FIR
    input   wire                     wvalid,//testbench(master) can transmit to FIR
    input   wire [(pDATA_WIDTH-1):0] wdata,//testbench(master) transmit to FIR
    output  wire                      arready,//address read ,FIR transmit to TESTBENCH
    input   wire                     rready,//TESTBENCH ready to receive data from FIR
    input   wire                     arvalid,//testbench transmit to FIR
    input   wire [(pADDR_WIDTH-1):0] araddr,//testbench transmit to FIR
    output  wire                      rvalid,//FIR to testbench
    output  wire [(pDATA_WIDTH-1):0]  rdata, // FIR to testbench
    input   wire                     ss_tvalid,//axis slave
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, //x[i]
    input   wire                     ss_tlast,
    output  wire                      ss_tready,//ready to receive data from testbench ,axis slave
    input   wire                     sm_tready,//ready to output data  ,axis master
    output  wire                      sm_tvalid, //axis master
    output  wire [(pDATA_WIDTH-1):0]  sm_tdata,//y[i]
    output  wire                      sm_tlast,
   
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,//FIR output
    output  wire [(pADDR_WIDTH-1):0] tap_A,//address
    input   wire [(pDATA_WIDTH-1):0]tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,//address
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);
//reg for fir output
reg awready;
reg wready;
reg arready;
reg rvalid;
reg [31:0]rdata;
reg ss_tready;
reg sm_tvalid;
reg [31:0]sm_tdata;
reg sm_tlast;
//reg for tap Ram output
reg tap_WE;
reg tap_EN;
reg [31:0]tap_Di;
reg [11:0]tap_A;
//reg for data Ram
reg data_WE;
reg data_EN;
reg [31:0]data_Di;
reg [11:0]data_A;
//don't use output as control signal
reg state;//to determine read out need 2clk (tap ram)
reg next_state;//to determine read out need 2clk (tap ram)
reg [31:0]data_length;//save 600 from tsetbench
reg [11:0]config_reg;//determine ap_start or ap_idle or ap_done 
reg [11:0]next_config_reg;
reg [31:0]temp;
reg [2:0]count_state;
reg [2:0]next_count_state;
reg [31:0]write_stack;
reg [31:0]read_stack;
reg convolution_en;//convolution enable
reg first;
reg [5:0]n;
reg [5:0]next_n;
reg [11:0]next_tap_A;
reg done;
reg [9:0]i;//determine when output data number == datalength ,done==1 *
reg [9:0]next_i;
//tap RAM ----first write in . second read out.  
//FSM for determine read out from tap RAM need 2 clk
//Read need 2clk , first clk to A-->r_a ,second clk to read data
always@(posedge axis_clk)
begin
    if(~axis_rst_n)
    begin
        state<=0;
        config_reg <= 12'd4;//ap_idle==1 
        temp <= 32'd0;//output value=0
        n <= 6'd0;
        count_state <= 3'd0;
        next_i<=10'd0;
    end
    else
    begin
        state<=next_state;
        config_reg <= next_config_reg;
        temp <= sm_tdata;
        n <= next_n;
        count_state <= next_count_state;
        i<=next_i;
    end
end
always@(*)
begin
    if(awaddr>=12'h20 && awaddr<=12'hFF && awvalid)
    begin //write address
        tap_EN=1'b1;//enable
        tap_A[3:0] = awaddr[3:0];
        tap_A[11:8] = 4'd0;
        case(awaddr[7:4]) //TESTBENCH give 12'h20,,,,12'h24----->translate to  12'h00  , 12'h24---->12'h04     write in
        4'h2: tap_A[7:4]=4'h0;
        4'h3: tap_A[7:4]=4'h1;
        4'h4: tap_A[7:4]=4'h2;
        4'h5: tap_A[7:4]=4'h3;
        4'h6: tap_A[7:4]=4'h4;
        4'h7: tap_A[7:4]=4'h5;
        4'h8: tap_A[7:4]=4'h6;
        4'h9: tap_A[7:4]=4'h7;
        4'hA: tap_A[7:4]=4'h8;
        4'hB: tap_A[7:4]=4'h9;
        4'hC: tap_A[7:4]=4'hA;
        4'hD: tap_A[7:4]=4'hB;
        4'hE: tap_A[7:4]=4'hC;
        4'hF: tap_A[7:4]=4'hD;
        default:tap_A[7:4]=4'h0;
        endcase
        awready = 1'b1;
        wready = 1'b1;
        tap_WE = 4'b1111;
        tap_Di = wdata;//write data into tap ram        
    end
    //testbench ready to receive data from FIR   read out to compare with exp data(right answer)     
    else if(araddr>=12'h20 && araddr<=12'hFF && arvalid)
    begin //read address
        tap_EN=1'b1;//enable
        //FSM for determine read out from tapRAM need 2 clk
        if(~state)
        begin //A--->r_A
            tap_A[3:0] = araddr[3:0];
            tap_A[11:8] = 4'd0;
            tap_WE = 4'b0000;
            case(araddr[7:4])
            4'h2: tap_A[7:4]=4'h0;
            4'h3: tap_A[7:4]=4'h1;
            4'h4: tap_A[7:4]=4'h2;
            4'h5: tap_A[7:4]=4'h3;
            4'h6: tap_A[7:4]=4'h4;
            4'h7: tap_A[7:4]=4'h5;
            4'h8: tap_A[7:4]=4'h6;
            4'h9: tap_A[7:4]=4'h7;
            4'hA: tap_A[7:4]=4'h8;
            4'hB: tap_A[7:4]=4'h9;
            4'hC: tap_A[7:4]=4'hA;
            4'hD: tap_A[7:4]=4'hB;
            4'hE: tap_A[7:4]=4'hC;
            4'hF: tap_A[7:4]=4'hD;
            default:tap_A[7:4]=4'h0;
            endcase
            arready = 1;
            rdata = 32'd0;
            rvalid = 0;
            next_state = 1;          
        end            
        else 
        begin //read data
            arready = 0;
            tap_Di = 32'd0;
            tap_WE = 4'b0000;
            tap_A = tap_A;
            rdata = tap_Do;
            rvalid = 1;
            next_state = 0;//back to read address state
        end//above is tap RAM code
    end
    else if(awaddr == 12'h10 && awvalid) 
    begin//save datalength
        arready = 0;
        rvalid = 0;
        awready = 1;
        data_length = wdata;//write into data_length register
        wready = 1;
        tap_A = 12'd0;
        rdata = 32'd0;
        next_state = 0;
        next_config_reg = 12'd4;// ap_idle
    end
    else if(awaddr == 12'h00 && awvalid) 
    begin//ap_start        
        next_config_reg = wdata[11:0];
        wready = 1;
        awready = 1;
        tap_WE = 4'b0000;  
        tap_EN = 1;
    end 
    else if(ss_tvalid && config_reg[0])//can input ss_tdata
    begin
        next_config_reg = 12'd0;//ap_start back to 0 (1 clk),after data transfer starts 
        convolution_en = 1;
        first = 1;//
    end
    else if(~config_reg[2] && config_reg[1])//ap_done==1,ap_idle==0 
    begin
        next_config_reg = 12'd4;//ap_done==1
        convolution_en = 0;//stop process 
        rvalid=1;
        rdata=32'h0000_0002;
    end
    else if(config_reg[2] && ~config_reg[1])//ap_idle==1 ,ap_done==0
    begin
        next_config_reg = 12'd4;//ap_idle==1
        convolution_en = 0;
        rvalid=1;
        rdata=32'h0000_0004;
    end
    else//config_reg==12'h00
    begin
        next_config_reg = config_reg;
        convolution_en = convolution_en;
        tap_A = next_tap_A;
        first = 0;
        rdata=32'h0000_0000;
    end    
end
// in data ram readout need 2 clk ,and write in need 1 clk 
always@(*)
begin
    if(convolution_en)
    begin
        case(count_state)  
        `state_push_initial:
        begin
            if(first)//start input data (at beginning 12'h00 address <data ram>)(first 100000...)
            begin
                next_tap_A = {6'd0,n};
                data_A = {6'd0,n};
                data_EN = 1;
                data_WE = 4'b1111;
                data_Di = ss_tdata;
                read_stack = 32'd0;
                ss_tready = 1;
                sm_tvalid = 0;
                next_n = n + 6'd4;//wait 1 clk to +4
                sm_tdata = 32'd0;
                done = 0;
                next_count_state = `state_first;
            end
            else
            begin//first read address at 12'h00
                next_tap_A = {6'd0,n};
                data_A = {6'd0,n};
                data_EN = 1;
                data_WE = 4'b0000;
                write_stack = ss_tdata;
                read_stack = 32'd0;
                ss_tready = 1;
                sm_tvalid = 0;
                next_n = n + 6'd4;
                sm_tdata = 32'd0;
                done = 0;
                next_count_state = `state_push_read1;
            end
        end
        `state_push_read0://first read address at 12'h04--->12'h40
        begin
            next_tap_A = {6'd0,n};
            data_A = {6'd0,n};
            data_EN = 1;
            data_WE = 4'b0000;
            write_stack = read_stack;
            read_stack = read_stack;
            ss_tready = 0;
            next_n = n + 6'd4;
            sm_tdata = 32'd0;
            done = 0;
            next_count_state = `state_push_read1;
        end
        `state_push_read1://read data out
        begin
            next_tap_A = next_tap_A;
            data_A = data_A;
            data_EN = 1;
            data_WE = 4'b0000;
            write_stack = write_stack;
            read_stack = data_Do;
            ss_tready = 0;
            next_n = n;
            sm_tdata = 32'd0;
            done = 0;
            next_count_state = `state_push_write;
        end
        `state_push_write://write data in
        begin
            if(tap_A == 6'd40)
            begin
                next_tap_A = next_tap_A;
                data_A = data_A;
                data_EN = 1;
                data_WE = 4'b1111;
                data_Di = write_stack;
                ss_tready = 0;
                next_n = 6'd0;//back to 12'h00
                sm_tdata = 32'd0;
                done = 0;
                next_count_state = `state_count0;
            end
            else
            begin
                next_tap_A = next_tap_A;
                data_A = data_A;
                data_EN = 1;
                data_WE = 4'b1111;
                data_Di = write_stack;
                ss_tready = 0;
                next_n = n;
                sm_tdata = 32'd0;
                done = 0;
                next_count_state = `state_push_read0;
            end         
        end        
        `state_first://data ram first input
        begin
            if(n == 6'd40)
            begin
                next_tap_A = {6'd0,n};
                data_A = {6'd0,n};
                data_EN = 1;
                data_WE = 4'b1111;
                data_Di = 32'd0;
                ss_tready = 0;
                next_n = 6'd0;//back to 12'h00
                sm_tdata = 32'd0;
                done = 0;
                next_count_state = `state_count0;
            end
            else
            begin
                next_tap_A = {6'd0,n};//12'h04---12'h36 give 0
                data_A = {6'd0,n};
                data_EN = 1;
                data_WE = 4'b1111;
                data_Di = 32'd0;
                ss_tready = 0;
                next_n = n + 6'd4;
                sm_tdata = 32'd0;
                done = 0;
                next_count_state = `state_first;
            end
        end
        `state_count0://read address (read out need 2 clk)
        begin
            next_tap_A = {6'd0,n};//back to 12'h00
            data_A = {6'd0,n};//back to 12'h00
            data_EN = 1;
            data_WE = 4'b0000;
            ss_tready = 0;
            sm_tvalid = 0;
            next_n = n + 6'd4;
            sm_tdata = temp;//0
            done = 0;
            next_count_state = `state_count1;
        end
        `state_count1://read data
        begin
            if(n == 6'd40)
            begin
                next_tap_A = next_tap_A;
                data_A = data_A;
                data_EN = 1;
                data_WE = 4'b0000;
                ss_tready = 0;
                sm_tvalid = 1;//after 55 clk can output y[i]
                sm_tdata = temp + data_Do * tap_Do;
                next_n = 6'd0;
                done = 0;
                next_i=i+10'd1;//i add to 600 ,done==1
                if(i==10'd599)
                begin
                    //sm_tlast=1;
                    next_count_state = `state_done;
                end
                else next_count_state = `state_push_initial;// back to input ss_tdata
            end
            else
            begin
                next_tap_A = next_tap_A;
                data_A = data_A;
                data_EN = 1;
                data_WE = 4'b0000;
                ss_tready = 0;
                sm_tvalid = 0;
                sm_tdata = temp + data_Do * tap_Do;
                next_n = n;
                done = 0;
                next_count_state = `state_count0;
            end
        end
        `state_done:
        begin
            next_tap_A = next_tap_A;
            data_A = data_A;
            data_EN = 1;
            data_WE = 4'b0000;
            ss_tready = 0;
            sm_tvalid = 0;
            next_n = 6'd0;
            sm_tdata = 32'd0;
            done = 1;        
            sm_tlast=1;
            next_config_reg = 12'h002;//ap_done
            next_count_state = `state_done;                
        end
        default:
        begin
            next_tap_A = next_tap_A;
            data_A = data_A;
            data_EN = 1;
            data_WE = 4'b0000;
            ss_tready = 0;
            sm_tvalid = 0;
            next_n = 6'd0;
            sm_tdata = 32'd0;
            done = 0;
            next_count_state = `state_push_initial;
        end
        endcase
    end
    else//no convolution
    begin
        next_tap_A = next_tap_A;
        data_A = data_A;
        data_EN = 1;
        data_WE = 4'b0000;
        ss_tready = 0;
        sm_tvalid = 0;
        next_n = 6'd0;
        sm_tdata = 32'd0;
        done = 0;
        next_count_state = `state_push_initial;
    end    
end
endmodule
