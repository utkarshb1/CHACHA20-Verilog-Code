`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 23.06.2019 13:41:44
// Design Name: 
// Module Name: add_xor
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


module add_xor(input wire [31:0] a,
               input wire [31:0] b,
               input wire [31:0] c,
               output wire [31:0] d,
               output wire [31:0] e
    );

    //Intermediate Registers
    reg [31:0] p;
    reg [31:0] q;

    //Concurrent assignment of ports
    assign d = p;
    assign e = q;
    initial
        begin
            p = a + b;  //32 bit addition
            q = c ^ p;  //32 bit XOR         
        end
endmodule
