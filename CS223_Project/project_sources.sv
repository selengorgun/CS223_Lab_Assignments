`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 20.12.2020 23:30:42
// Design Name: 
// Module Name: project_sources
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module debounce(input logic clk, input logic button,output logic pulse ); 

  

logic [24:0] timer; 

typedef enum logic [1:0]{S0,S1,S2,S3} states; 

states state, nextState; 

logic gotInput; 

  

always_ff@(posedge clk) 

    begin     

        state <= nextState; 

        if(gotInput) 

            timer <= 25000000; 

        else 

            timer <= timer - 1; 

    end 
always_comb 
    case(state) 
        S0: if(button)  
            begin //startTimer 
                nextState = S1;     
                gotInput = 1; 
            end 
            else begin nextState = S0; gotInput = 0; end 
        S1: begin nextState = S2; gotInput = 0; end 
        S2: begin nextState = S3; gotInput = 0; end 
        S3: begin if(timer == 0) nextState = S0; else nextState = S3; gotInput = 0; end 
        default: begin nextState = S0; gotInput = 0; end 
        endcase 

assign pulse = ( state == S1 ); 

endmodule 

  
module regFile(input logic clk, we, reset, input logic [3:0] wAdr, rAdr1, rAdr2, 
               input logic [7:0] wData, 
               output logic [7:0] rData1, rData2); 

                
    logic [7:0] mem[15:0]; 
     
    always_ff @ (posedge clk) 
        if(we) begin
            mem[wAdr] <= wData; 
        end
        else if(reset) begin
            for(integer i = 0; i < 16; i++)
                mem[i] = 8'b00000000;
        end
             
    assign rData1 = mem[rAdr1]; 
    assign rData2 = mem[rAdr2]; 
endmodule 
  


module PC(input logic [4:0] brAdr, input logic dB, reset, clk, branch, output logic [4:0] adr);
    
    initial begin adr = 5'b11111; end
    always@(posedge clk)
    begin
        if (reset) 
        begin
        adr = 5'b11111;
        end
        else if(dB) begin
            if(adr < 31)begin
                if(branch)
                adr = brAdr;
                else 
                begin
                adr = adr + 1;	
                end
            end
            else begin
            adr = 5'b00000;
            end
        end
        else if(branch)begin
        adr = brAdr;
        end
    end
endmodule

module branch(input logic jEn, equals, output logic brControl);
    always@(*)begin
        if(equals & jEn)
            brControl = 1;
        else
            brControl = 0;
   end
endmodule
  
module top( input logic clk, lB, rB, uB, dB, mB,  
            input logic [15:0] inst_sw,  
            output logic [15:0] nextInstruction, 
            output logic [6:0] seg, output logic dp, output logic [3:0] an); 
     
     logic wEnReg, jEn, wEnData, brControl, aluControl;
     logic  equals;
     logic [7:0] result;
     logic [15:0] instMem, instruction, nextInstMem;
     logic [3:0] first4 = instruction[15:12];
     logic [3:0] wAdrReg, rAdr1, rAdr2, wAdr, disp_adr, nextAdr;
     logic [3:0] rAdr, rAdrr, adrb;
     logic [7:0] wDataReg, wData, rData1, rData2, rData, disp_data, rDataa; 
     logic pulse1, pulse2, pulse3, pulse4, pulse5;
     logic immbit = instruction[12];
     logic [4:0] adr, adrpc;
         
    debounce d1(clk, lB, pulse1); 
    debounce d2(clk, rB, pulse2); 
    debounce d3(clk, uB, pulse3); 
    debounce d4(clk, dB, pulse4); 
    debounce d5(clk, mB, pulse5); 
    
    
    
    PC pcc(instruction[12:8], pulse4, pulse3, clk, brControl, adr);  
    instMem im(adr,  pulse3, instMem, nextInstruction);  
    controller cont(instruction[15:13], pulse3, wEnData, wEnReg, jEn, aluControl); 
    regFile rf(clk, wEnReg, pulse3, wAdrReg, instruction[7:4], instruction[3:0], wDataReg, rData1, rData2); 
    alu a(rData1, rData2, aluControl, result, equals); 
    dataMem dm(clk, wEnData, pulse3, rAdr, rAdrr, wAdr, wData, rData, rDataa); 
    branch b(jEn, equals, brControl);
    SevSeg_4digit segsev(clk, disp_adr, 17, disp_data [7:4], disp_data [3:0],seg, dp, an );   
    
    assign disp_data = rDataa;
    assign disp_adr = rAdrr;
       
    always_ff@(posedge clk)
    if(pulse1) begin
        if(disp_adr > 0)
            disp_adr = disp_adr - 1;
        else
            disp_adr <= 15;
    end
    else if(pulse2) begin
        if(disp_adr < 16)
            disp_adr <= disp_adr + 1;
        else
            disp_adr <= 0;
    end
     
    always_ff@(posedge clk) 
    if(pulse5)
    instruction = inst_sw;
    else 
    instruction = instMem;

    
    initial begin
    wAdr = 0;
    wData = 0;
    wAdrReg = 0;
    wDataReg = 0;
    rAdr = 0;
    rAdr1 = 0;
    rAdr2 = 0;
    disp_adr = 0;
    disp_data = 0;
    rData = 0;
    rData1 = 0;
    rData2 = 0; 
    end
     
    always @(*)   
    begin                 
            if(wEnData)
            begin
                    if(immbit == 1)
                    begin
                    //Take value and address from the instruction and write it to datamem 
                     wAdr = instruction[11:8]; 
                     wData = instruction[7:0];  
                    end 
                    
                    else
                    begin
                    //Read from reg, write to datamem 
                     wAdr = instruction[7:4];
                     wData = rData2;
                    end
            end 
            if(wEnReg)
            begin   
                    if(immbit == 0)
                    begin
                    //Read from datamem, write to reg  
                    wAdrReg = instruction[7:4];
                    wDataReg = rData;  
                    rAdr = instruction[3:0]; 
                    end
                    
                    else
                    begin
                    //Take value and address from the instruction and write to reg 
                     wDataReg = instruction[7:0]; 
                     wAdrReg = instruction[11:8]; 
                    end 
            end 
            if(aluControl & wEnReg)
                begin
                        //Read values from reg  
                        //After addition, write the result to reg     
                         wAdrReg = instruction[11:8]; 
                         wDataReg = result; 
                end
            
    end 

endmodule 
  
module alu(input logic [7:0] src1, src2, input logic control,  
           output logic [7:0] result, output logic equals); 
            
    always @(src1, src2, control, result, equals) 
    begin 
        if(control)
            begin 
                   if(src1 == src2)
                    equals = 1;
                   else
                    equals = 0; 
                    result = src1 + src2; 
            end 
    end 

    
endmodule 
  
module instMem(input logic [4:0] adr, input logic reset, output logic [15:0] instruction, nextInst); 
     logic [15:0] im[31:0]; 

    always @(*) 
    begin 
            if(adr == 31) begin             
                if(reset)begin
                instruction = 16'b1000000000000000; end
                else begin
                instruction = im[adr]; end
            nextInst = im[0]; 
            end
            else begin 
            instruction = im[adr];
            nextInst = im[adr+1];
            end
    end                                                        
    
    initial im[0] = 16'b000_1_0110_0000_0101; // store 5 in dm 6
    initial im[1] = 16'b000_1_0001_0000_0101; // store 5 in dm 1
    initial im[2] = 16'b101_00100_0001_0110; // jump to 4 if [6] == [1]
    initial im[3] = 16'b000_1_0110_0000_0111; // store 7 in dm 6
    initial im[4] = 16'b000_1_0110_0000_0001; // store 1 in dm 6
    initial im[5] = 16'b001_0_0000_0100_0111; 
    initial im[6] = 16'b000_0_0000_0010_0100; 
    initial im[7] = 16'b0100001000100100; 
    initial im[8] = 16'b1010010100000000; 
    initial im[9] = 16'b0000000000110011; 
    initial im[10] = 16'b1110000000000000;   
    initial im[31] = 16'b1110000000000000;     
endmodule 

  
module controller(input logic [2:0] opcode, input logic reset, 
                  output logic wEnData, wEnReg, jEn, output logic aluControl); 
     
    always @(*)   
    begin                 
       case(opcode) 
            3'b000:begin //Store
                if(reset)
                begin
                 wEnData = 0; 
                 wEnReg = 0; 
                 jEn = 0; 
                 aluControl = 0; 
                end
                else
                begin 
                 wEnData = 1; 
                 wEnReg = 0; 
                 jEn = 0; 
                 aluControl = 0; 
                 end
                end 
            3'b001:begin //Load 
                   if(reset)
                   begin
                   wEnData = 0; 
                   wEnReg = 0; 
                   jEn = 0; 
                   aluControl = 0; 
                   end
                   else
                   begin
                   wEnData = 0; 
                   wEnReg = 1; 
                   jEn = 0; 
                   aluControl = 0;
                   end 
                end 
            3'b010:begin //Addition 
                   if(reset)
                begin
                wEnData = 0; 
                 wEnReg = 0; 
                 jEn = 0; 
                 aluControl = 0; 
                end 
                else
                begin
                 wEnData = 0; 
                 wEnReg = 1; 
                 jEn = 0; 
                 aluControl = 1; 
                 end
                end 
            3'b101:begin //Branch if equals 
                   if(reset)
                begin
                wEnData = 0; 
                 wEnReg = 0; 
                 jEn = 0; 
                 aluControl = 0; 
                end
                else
                begin
                 wEnData = 0; 
                 wEnReg = 0; 
                 jEn = 1; 
                 aluControl = 1;
                 end 
                end 
             default:begin 
                 wEnData = 0; 
                 wEnReg = 0; 
                 jEn = 0; 
                 aluControl = 0; 
                end
       endcase 
   end 
     
endmodule 
  

module dataMem(input logic clk, we, reset, input logic [3:0] rAdr1, rAdr2, wAdr, input logic [7:0] wData, 
                output logic [7:0] rData1, rData2); 

    logic [7:0] mem[15:0]; 
    
    
    always_ff @ (posedge clk) 
        if(reset) begin
            for(integer i = 0; i < 16; i++)
                mem[i] <= 8'b00000000;
        end 
        else if(we) begin 
            mem[wAdr] <= wData;
        end

        
    assign rData1 = mem[rAdr1]; 
    assign rData2 = mem[rAdr2];
endmodule 

module SevSeg_4digit( 

input clk,  

input [3:0] in3, in2, in1, in0, //user inputs for each digit (hexadecimal value) 

output [6:0]seg, logic dp, // just connect them to FPGA pins (individual LEDs). 

output [3:0] an   // just connect them to FPGA pins (enable vector for 4 digits active low) 

); 

  

// divide system clock (100Mhz for Basys3) by 2^N using a counter, which allows us to multiplex at lower speed 

localparam N = 18; 

logic [N-1:0] count = {N{1'b0}}; //initial value 

always@ (posedge clk) 

	count <= count + 1; 

  

  

logic [4:0]digit_val; // 7-bit register to hold the current data on output 

logic [3:0]digit_en;  //register for the 4 bit enable 

  

always@ (*) 

begin 

digit_en = 4'b1111; //default 

digit_val = in0; //default 

  

  case(count[N-1:N-2]) //using only the 2 MSB's of the counter  

     

   2'b00 :  //select first 7Seg. 

    begin 

     digit_val = {1'b0, in0}; 

     digit_en = 4'b1110; 

    end 

     

   2'b01:  //select second 7Seg. 

    begin 

     digit_val = {1'b0, in1}; 

     digit_en = 4'b1101; 

    end 

     

   2'b10:  //select third 7Seg. 

    begin 

     digit_val = {1'b1, in2}; 

     digit_en = 4'b1011; 

    end 

      

   2'b11:  //select forth 7Seg. 

    begin 

     digit_val = {1'b0, in3}; 

     digit_en = 4'b0111; 

    end 

  endcase 

end 

  

  

//Convert digit number to LED vector. LEDs are active low. 

logic [6:0] sseg_LEDs;  

always @(*) 

begin  

  sseg_LEDs = 7'b1111111; //default 

  case( digit_val) 

   5'd0 : sseg_LEDs = 7'b1000000; //to display 0 

   5'd1 : sseg_LEDs = 7'b1111001; //to display 1 

   5'd2 : sseg_LEDs = 7'b0100100; //to display 2 

   5'd3 : sseg_LEDs = 7'b0110000; //to display 3 

   5'd4 : sseg_LEDs = 7'b0011001; //to display 4 

   5'd5 : sseg_LEDs = 7'b0010010; //to display 5 

   5'd6 : sseg_LEDs = 7'b0000010; //to display 6 

   5'd7 : sseg_LEDs = 7'b1111000; //to display 7 

   5'd8 : sseg_LEDs = 7'b0000000; //to display 8 

   5'd9 : sseg_LEDs = 7'b0010000; //to display 9 

   5'd10: sseg_LEDs = 7'b0001000; //to display a 

   5'd11: sseg_LEDs = 7'b0000011; //to display b 

   5'd12: sseg_LEDs = 7'b1000110; //to display c 

   5'd13: sseg_LEDs = 7'b0100001; //to display d 

   5'd14: sseg_LEDs = 7'b0000110; //to display e 

   5'd15: sseg_LEDs = 7'b0001110; //to display f    

   5'd16: sseg_LEDs = 7'b0110111; //to display "=" 

   default : sseg_LEDs = 7'b0111111; //dash 

  endcase 

end 

  

assign an = digit_en; 

assign seg = sseg_LEDs;  

assign dp = 1'b1; //turn dp off 

endmodule 
