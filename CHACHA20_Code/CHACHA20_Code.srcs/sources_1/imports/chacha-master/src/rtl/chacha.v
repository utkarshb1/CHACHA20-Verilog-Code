`timescale 1ns / 1ps 
//======================================================================
//
// chacha.v
// --------
// Top level wrapper for the ChaCha stream, cipher core providing
// a simple memory like interface with 32 bit data access.
//
//
// Copyright (c) 2013  Secworks Sweden AB
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the following
// conditions are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

module chacha(
              input wire           clk,     //Defining inputs and Outputs
              input wire           reset_n,
              input wire           cs,
              input wire           we,
              input wire [7 : 0]   addr,
              input wire [31 : 0]  write_data,
              output wire [31 : 0] read_data,
              input wire N
             );

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  localparam ADDR_CTRL        = 8'h08;      //Defining local parameter as STATE for Control 
  localparam CTRL_INIT_BIT    = 0;          //Defining INITIAL Control flag
  localparam CTRL_NEXT_BIT    = 1;          //Defining NEXT Control flaf

  localparam ADDR_STATUS      = 8'h09;      //Defining local parameter ADDR_STATUS as state
  localparam STATUS_READY_BIT = 0;          //Defining flag 

  localparam ADDR_ROUNDS      = 8'h0b;      //Defining State for Rounds
  localparam ROUNDS_HIGH_BIT  = 4;          //Defining local parameter required to tell number of bits required to represent the rounds
  localparam ROUNDS_LOW_BIT   = 0;          //Defining Lowest bit value

  localparam ADDR_KEY0        = 8'h10;      //Defining State for representing First 32 bit of Key
  localparam ADDR_KEY7        = 8'h17;      //Defining State for representing Last 32 bit of Key

  localparam ADDR_NONCE0      = 8'h20;      //Defining State for the highest 32 bit of NONCE
  localparam ADDR_NONCE1      = 8'h21;      //Defining State for the middle 32 bit of NONCE
  localparam ADDR_NONCE2      = 8'h22;      //Defining State for the lowest 32 bit of NONCE

  localparam ADDR_DATA_IN0    = 8'h40;      //Defining State for highest 32 bit input data          
  localparam ADDR_DATA_IN15   = 8'h4f;      //Defining State for lowest 32 bit input data
  
  localparam ADDR_DATA_OUT0   = 8'h80;      //Defining State for highest 32 bit output data
  localparam ADDR_DATA_OUT15  = 8'h8f;      //Defining State for lowest 32 bit output data

  localparam DEFAULT_CTR_INIT = 64'h0;      //Defining State for Default Control Initialization

//  localparam ADDR_DATA_IN0   = 12'h100;
//  localparam ADDR_DATA_IN374 = 12'h277;
//  localparam ADDR_DATA_OUT0  = 12'h300;      //Defining State for highest 32 bit output data
//  localparam ADDR_DATA_OUT374 = 12'h477;  
    
  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg          init_reg;                    //Initial reg
  reg          init_new;                    //Initial updated reg
  reg          next_reg;                    //Next reg
  reg          next_new;                    //Next updated Reg

  reg [4 : 0]  rounds_reg;                  //Rounds reg for storing round values ini bits  
  reg          rounds_we;                   //Rounds write flag

  reg [31 : 0] key_reg [0 : 7];             //Register for storing the 256 bit key in chunks of 32 bit
  reg          key_we;                      //Key write flag

  reg [31 : 0] nonce_reg[0 : 2];            //Register for Storing the 96 bit nonce in chunks of 32 bit
  reg          nonce_we;                    //Nonce write flag

  reg [31 : 0] data_in_reg [0 : 15];        //Register for Storing the 512 bit Input data in chunks of 32 bit
  reg          data_in_we;                  //Input data write flag
  

  //----------------------------------------------------------------
  // Registers and Wires for output ports.
  //----------------------------------------------------------------
  wire [255 : 0] core_key;
  wire [95 : 0]  core_nonce;
  wire           core_ready;
  wire [511 : 0] core_data_in;
  wire [511 : 0] core_data_out;
  wire           core_data_out_valid;

  reg [31 : 0]   tmp_read_data;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign core_key     = {key_reg[0], key_reg[1], key_reg[2], key_reg[3],                            //Assigning the 256 bit key to core_key
                         key_reg[4], key_reg[5], key_reg[6], key_reg[7]};

  assign core_nonce      = {nonce_reg[0], nonce_reg[1],nonce_reg[2]};                               //Assigning the 96 bit nonce to core_nonce

  assign core_data_in = {data_in_reg[00], data_in_reg[01], data_in_reg[02], data_in_reg[03],        //Assigning the 512 bit Input data to core_data_in
                         data_in_reg[04], data_in_reg[05], data_in_reg[06], data_in_reg[07],
                         data_in_reg[08], data_in_reg[09], data_in_reg[10], data_in_reg[11],
                         data_in_reg[12], data_in_reg[13], data_in_reg[14], data_in_reg[15]};

  assign read_data     = tmp_read_data;                                                             //Assigning the intermediate reg of output_data to read_data


  //----------------------------------------------------------------
  // core instantiation.
  //----------------------------------------------------------------
  chacha_core core (
                    .clk(clk),
                    .reset_n(reset_n),
                    .init(init_reg),
                    .next(next_reg),
                    .key(core_key),
                    .nonce(core_nonce),
                    .ctr(DEFAULT_CTR_INIT),
                    .rounds(rounds_reg),
                    .data_in(core_data_in),
                    .ready(core_ready),
                    .data_out(core_data_out),
                    .data_out_valid(core_data_out_valid)
                   );


  //----------------------------------------------------------------
  // reg_update
  //
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with asynchronous
  // active low reset. All registers have write enable.
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin : reg_update
     integer i;
      if (!reset_n)                                 //At start of execution if reset then:
        begin
          init_reg   <= 0;                          //Initialize the Initial and Next reg with Zero
          next_reg   <= 0;
          rounds_reg <= 5'h0;                       //Initialize the round bits with zero
          nonce_reg[0]  <= 32'h0;                   //Initialize the Nonce 1st chunk with zero
          nonce_reg[1]  <= 32'h0;                   //Initialize the Nonce 2nd chunk with zero
          nonce_reg[2]  <= 32'h0;                   //Initialize the Nonce 3rd chunk with zero

          for (i = 0 ; i < 8 ; i = i + 1)           //Intializing the key values with zero
            key_reg[i] <= 32'h0;

          for (i = 0 ; i < 16 ; i = i + 1)          //Initialize the input data as zero
            data_in_reg[i] <= 32'h0;
        end
      else                                          //If not reset
        begin       
          init_reg <= init_new;                     //Assign the value of updated Initial flag to previous Initial Flag
          next_reg <= next_new;                     //Assign the value of updated next flag to previous Next flag

          if (rounds_we)                            //If the value of Rounds write flag becomes 1 then:
            rounds_reg <= write_data[ROUNDS_HIGH_BIT : ROUNDS_LOW_BIT];     //Assign the current round count to Rounds register

          if (key_we)                               //If the value of Key write flag becomes 1 then:
            key_reg[addr[2 : 0]] <= write_data;     //Assign the chunks of input key to Key register

          if (nonce_we)                             //If the value of Rounds write flag becomes 1 then:                    
            nonce_reg[addr[1:0]] <= write_data;     //Assign the chunks of Nonce of input nonce to Nonce register

          if (data_in_we)                           //If the value of Rounds write flag becomes 1 then:

            data_in_reg[addr[3 : 0]] <= write_data; //Assign the chunks of Input data to data_in_reg register         
        end
        
    //addr[N:0] is used for indexing of the respective State according to the different chunks
    end // reg_update


  //----------------------------------------------------------------
  // Address decoder logic.
  //----------------------------------------------------------------
  always @*                                         //Initialization of always block
    begin : addr_decoder
//      keylen_we     = 1'h0;
      rounds_we     = 1'h0;                         //Intializing all the flags as zero
      key_we        = 1'h0;
      nonce_we      = 1'h0;
      data_in_we    = 1'h0;
      init_new      = 1'h0;
      next_new      = 1'h0;
      tmp_read_data = 32'h0;                                    //Initializing the temporary reg for storing the input values to zero

      if (cs)                                                   //If Control State flag is 1 then:
        begin
          if (we)                                               //If write enable flag is 1 then:   
            begin
              if (addr == ADDR_CTRL)                            //If the input address from test bench is equal to the address of ADDR_CTRL then:
                begin
                  init_new = write_data[CTRL_INIT_BIT];         //Assign the flag value from the input obtained ot initial flag and new flag
                  next_new = write_data[CTRL_NEXT_BIT];
                end

              if (addr == ADDR_ROUNDS)                          //If the ainput address in equal to State value of ADDR_ROUNDS then:
                rounds_we = 1;                                  //Set the Rounds flag to 1

              if ((addr >= ADDR_KEY0) && (addr <= ADDR_KEY7))   //If the input address is between the starting address and ending address of the key State then
                key_we = 1;                                     //Set the Key write flag to 1

              if ((addr >= ADDR_NONCE0) && (addr <= ADDR_NONCE2))//If the input address is between the starting address and endian address of the Nonce State then 
                nonce_we = 1;                                   //Set Nonce write flag to 1

              if ((addr >= ADDR_DATA_IN0) && (addr <= ADDR_DATA_IN15))//If the input address is between the starting address and endian address of the Data input State then 
                data_in_we = 1;                                 //Set Input data write flag to 1
            end // if (we)

          else                                                                          //If write enable flag is 0 then: 
            begin
              if ((addr >= ADDR_KEY0) && (addr <= ADDR_KEY7))                           //If the input address is between the starting address and ending address of the key State then:
                tmp_read_data = key_reg[addr[2 : 0]];                                   //Assign the key values in the form of 32 bit chunks

              if ((addr >= ADDR_DATA_OUT0) && (addr <= ADDR_DATA_OUT15))                //If the input address is between the starting address and endian address of the Output data State then:
                tmp_read_data = core_data_out[(15 - (addr - ADDR_DATA_OUT0)) * 32 +: 32];// Assign the output data in the form of 32 bit chunks

              case (addr)                                                               //Select different Case States according to the input Adddress
                ADDR_CTRL:    tmp_read_data = {30'h0, next_reg, init_reg};              //If addr is same as ADDR_CTRL, Assign the flag values in the form of 32 bit to tmp_read_data
                ADDR_STATUS:  tmp_read_data = {30'h0, core_data_out_valid, core_ready}; //If addr is same as ADDR_STATUS, Assign the flag values in the form of 32 bit to tmp_read_data
                ADDR_ROUNDS:  tmp_read_data = {27'h0, rounds_reg};                      //If addr is same as ADDR_ROUNDS, Assign the round counter values to tmp_read_data
                ADDR_NONCE0:  tmp_read_data = nonce_reg[0];                             //If addr is same as ADDR_NONCE0, Assign the 32 bit first  nonce value to tmp_read_data
                ADDR_NONCE1:  tmp_read_data = nonce_reg[1];                             //If addr is same as ADDR_NONCE1, Assign the 32 bit middle nonce value to tmp_read_data
                ADDR_NONCE2:  tmp_read_data = nonce_reg[2];                             //If addr is same as ADDR_NONCE2, Assign the 32 bit last nonce value to tmp_read_data 

                default:
                  begin
                  end
              endcase                               //End of case (address)
            end                                     
        end
    end // End of always block addr_decoder
endmodule // chacha

//======================================================================
// EOF chacha.v
//======================================================================
