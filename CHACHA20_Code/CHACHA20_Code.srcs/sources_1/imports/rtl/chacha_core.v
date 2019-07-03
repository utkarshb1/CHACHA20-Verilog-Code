`timescale 1ns / 1ps 

//======================================================================
//
// chacha_core.v
// --------------
// Verilog 2001 implementation of the stream cipher ChaCha.
// This is the internal core with wide interfaces.
//
//
// Copyright (c) 2013 Secworks Sweden AB
// All rights reserved.
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

module chacha_core(
                   input wire            clk,
                   input wire            reset_n,

                   input wire            init,
                   input wire            next,

                   input wire [255 : 0]  key,
                   input wire [95 : 0]   nonce,
                   input wire [31 : 0]   ctr,
                   input wire [4 : 0]    rounds,

                   input wire [511 : 0]  data_in,

                   output wire           ready,

                   output wire [511 : 0] data_out,
                   output wire           data_out_valid
                  );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  // Datapath quartterround states names.
  localparam QR0 = 0;
  localparam QR1 = 1;

  localparam NUM_ROUNDS = 4'ha; // Declaring localparameters for assignning constant value
  
  localparam cons0 = 32'h61707865;	// Declaring localparameters for assignning constant value
  localparam cons1 = 32'h3320646e;
  localparam cons2 = 32'h79622d32;
  localparam cons3 = 32'h6b206574;
  
  localparam CTRL_IDLE     = 3'h0; 	// Declaring localparameters for assignning constant value
  localparam CTRL_INIT     = 3'h1;
  localparam CTRL_ROUNDS   = 3'h2;
  localparam CTRL_FINALIZE = 3'h3;
  localparam CTRL_DONE     = 3'h4;


  //----------------------------------------------------------------
  // l2b()
  //
  // Swap bytes from little to big endian byte order.
  //----------------------------------------------------------------
  function [31 : 0] l2b(input [31 : 0] op);
    begin
      l2b = {op[7 : 0], op[15 : 8], op[23 : 16], op[31 : 24]};
    end
  endfunction // b2l


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------
  reg [31 : 0]  state_reg [0 : 15]; 	// Declaring reg for assignning value
  reg [31 : 0]  state_new [0 : 15];		// Declaring reg named new to act as intermediate while writing value to final reg
  reg           state_we;					// Declaring reg as flag for write_enable to keep check when to write

  reg [511 : 0] data_out_reg;				// Declaring reg for assignning value
  reg [511 : 0] data_out_new;				// Declaring reg named new to act as intermediate while writing value to final reg

  reg           data_out_valid_reg;		// Declaring reg for assignning value
  reg           data_out_valid_new;		// Declaring reg named new to act as intermediate while writing value to final reg
  reg           data_out_valid_we;		// Declaring reg as flag for write_enable to keep check when to write


  reg           qr_ctr_reg;				// Declaring reg for assignning value
  reg           qr_ctr_new;				// Declaring reg named new to act as intermediate while writing value to final reg
  reg           qr_ctr_we;			    // Declaring reg as flag for write_enable to keep check when to write
  reg           qr_ctr_inc;				// Declaring reg named as increase to increase count for quarterround
  reg           qr_ctr_rst;				// Declaring reg named as reset to reset count for quarterround

  reg [3 : 0]   dr_ctr_reg;				// Declaring reg for assignning value
  reg [3 : 0]   dr_ctr_new;				// Declaring reg named new to act as intermediate while writing value to final reg
  reg           dr_ctr_we;				// Declaring reg as flag for write_enable to keep check when to write
  reg           dr_ctr_inc;				// Declaring reg named as increase to increase count for rounds in algorithm
  reg           dr_ctr_rst;				// Declaring reg named as reset to reset count for rounds in algorithm

  reg [31 : 0]  block0_ctr_reg;			// Declaring reg for assignning value
  reg [31 : 0]  block0_ctr_new;			// Declaring reg named new to act as intermediate while writing value to final reg
  reg           block0_ctr_we;			// Declaring reg as flag for write_enable to keep check when to write
  reg           block_ctr_inc;			// Declaring reg named as increase to increase count for block_count in state matrix
  reg           block_ctr_set;			// Declaring reg named as set to set count for block_count in state matrix

  reg           ready_reg;
  reg           ready_new;
  reg           ready_we;

  reg [2 : 0]   chacha_ctrl_reg;		// Declaring reg for assignning diffrent state for fsm
  reg [2 : 0]   chacha_ctrl_new;		// Declaring reg named new to act as intermediate while writing value to final reg
  reg           chacha_ctrl_we;			// Declaring reg as flag for write_enable to keep check when to write

  // Test
  reg [31 : 0] msb_block_state [0 : 15];	// Declaring 16 reg of size 32bit for 16 indexes of state matrix (with data in msb form)
  reg [31 : 0] lsb_block_state [0 : 15];	// Declaring 16 reg of size 32bit for 16 indexes of state matrix (with data in lsb form)
  reg [511 : 0] block_state;				// Declaring 512 bit reg for whole state matrix

  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  reg [31 : 0] init_state_word [0 : 15];	// Declaring 16 reg of size 32 bit for 16 indexes of initial state matrix 

  reg init_state;							// Declaring reg and using it as flag while assigning initial values in state matrix
  reg update_state;							// Declaring reg and using it as flag while updateing values in state matrix through quarterround
  reg update_output;						// Declaring reg and using it as flag while assigning output values in state matrix after quarterround

  reg [31 : 0]  qr0_a;						// Declaring reg for assignning input value of 1st qaurterround 
  reg [31 : 0]  qr0_b;						// Declaring reg for assignning input value of 1st qaurterround
  reg [31 : 0]  qr0_c;						// Declaring reg for assignning input value of 1st qaurterround
  reg [31 : 0]  qr0_d;						// Declaring reg for assignning input value of 1st qaurterround
  wire [31 : 0] qr0_a_prim;				    // Declaring reg for assignning output value of 1st qaurterround 
  wire [31 : 0] qr0_b_prim;				    // Declaring reg for assignning output value of 1st qaurterround
  wire [31 : 0] qr0_c_prim;			        // Declaring reg for assignning output value of 1st qaurterround
  wire [31 : 0] qr0_d_prim;				    // Declaring reg for assignning output value of 1st qaurterround

  reg [31 : 0]  qr1_a;						// Declaring reg for assignning input value of 2nd qaurterround
  reg [31 : 0]  qr1_b;						// Declaring reg for assignning input value of 2nd qaurterround
  reg [31 : 0]  qr1_c;						// Declaring reg for assignning input value of 2nd qaurterround
  reg [31 : 0]  qr1_d;						// Declaring reg for assignning input value of 2nd qaurterround
  wire [31 : 0] qr1_a_prim;				    // Declaring reg for assignning output value of 2nd qaurterround
  wire [31 : 0] qr1_b_prim;				    // Declaring reg for assignning output value of 2nd qaurterround
  wire [31 : 0] qr1_c_prim;				    // Declaring reg for assignning output value of 2nd qaurterround
  wire [31 : 0] qr1_d_prim;				    // Declaring reg for assignning output value of 2nd qaurterround
 
  reg [31 : 0]  qr2_a;						// Declaring reg for assignning input value of 3rd qaurterround
  reg [31 : 0]  qr2_b;						// Declaring reg for assignning input value of 3rd qaurterround
  reg [31 : 0]  qr2_c;						// Declaring reg for assignning input value of 3rd qaurterround
  reg [31 : 0]  qr2_d;						// Declaring reg for assignning input value of 3rd qaurterround
  wire [31 : 0] qr2_a_prim;				// Declaring reg for assignning output value of 3rd qaurterround
  wire [31 : 0] qr2_b_prim;				// Declaring reg for assignning output value of 3rd qaurterround
  wire [31 : 0] qr2_c_prim;				// Declaring reg for assignning output value of 3rd qaurterround
  wire [31 : 0] qr2_d_prim;				// Declaring reg for assignning output value of 3rd qaurterround

  reg [31 : 0]  qr3_a;						// Declaring reg for assignning input value of 4th qaurterround
  reg [31 : 0]  qr3_b;						// Declaring reg for assignning input value of 4th qaurterround
  reg [31 : 0]  qr3_c;						// Declaring reg for assignning input value of 4th qaurterround
  reg [31 : 0]  qr3_d;						// Declaring reg for assignning input value of 4th qaurterround
  wire [31 : 0] qr3_a_prim;				// Declaring reg for assignning output value of 4th qaurterround
  wire [31 : 0] qr3_b_prim;				// Declaring reg for assignning output value of 4th qaurterround
  wire [31 : 0] qr3_c_prim;				// Declaring reg for assignning output value of 4th qaurterround
  wire [31 : 0] qr3_d_prim;				// Declaring reg for assignning output value of 4th qaurterround


  //----------------------------------------------------------------
  // Instantiation of the qr modules.
  //----------------------------------------------------------------
  chacha_qr qr0(
                .a(qr0_a),
                .b(qr0_b),
                .c(qr0_c),
                .d(qr0_d),

                .a_prim(qr0_a_prim),
                .b_prim(qr0_b_prim),
                .c_prim(qr0_c_prim),
                .d_prim(qr0_d_prim)
               );

  chacha_qr qr1(
                .a(qr1_a),
                .b(qr1_b),
                .c(qr1_c),
                .d(qr1_d),

                .a_prim(qr1_a_prim),
                .b_prim(qr1_b_prim),
                .c_prim(qr1_c_prim),
                .d_prim(qr1_d_prim)
               );

  chacha_qr qr2(
                .a(qr2_a),
                .b(qr2_b),
                .c(qr2_c),
                .d(qr2_d),

                .a_prim(qr2_a_prim),
                .b_prim(qr2_b_prim),
                .c_prim(qr2_c_prim),
                .d_prim(qr2_d_prim)
               );

  chacha_qr qr3(
                .a(qr3_a),
                .b(qr3_b),
                .c(qr3_c),
                .d(qr3_d),

                .a_prim(qr3_a_prim),
                .b_prim(qr3_b_prim),
                .c_prim(qr3_c_prim),
                .d_prim(qr3_d_prim)
               );


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign data_out = data_out_reg;
  assign data_out_valid = data_out_valid_reg;
  assign ready = ready_reg;


  //----------------------------------------------------------------
  // reg_update
  //
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with synchronous
  // active low reset. All registers have write enable.
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin : reg_update
     integer i;						// Temperory variable to be used in loop 

      if (!reset_n)					// Condition to reset all reg
        begin
          for (i = 0 ; i < 16 ; i = i + 1) 	//Loop to assign 0 value in state matrix
            state_reg[i] <= 32'h0;

          data_out_reg       <= 512'h0; 		//Initalizng to inital or zero value
          data_out_valid_reg <= 0;
          qr_ctr_reg         <= QR0;
          dr_ctr_reg         <= 0;
          block0_ctr_reg     <= 32'h0;
          chacha_ctrl_reg    <= CTRL_IDLE;
          ready_reg          <= 1;
        end
      else
        begin
          if (state_we)								// Condition Checking write_enable of state to write new output in state reg
            begin
              for (i = 0 ; i < 16 ; i = i + 1)	//Loop to assign new value to state reg
                state_reg[i] <= state_new[i];
            end

          if (update_output)						// Condition Checking updated output of state to write new output in output data reg
            data_out_reg <= data_out_new;

          if (data_out_valid_we)					// Condition Checking data output valid- To let output data to assign
            data_out_valid_reg <= data_out_valid_new;

          if (qr_ctr_we)							// Condition Checking write_enable for writing new value of count to reg 
            qr_ctr_reg <= qr_ctr_new;

          if (dr_ctr_we)							// Condition Checking write_enable for writing new value of count to reg
            dr_ctr_reg <= dr_ctr_new;			

          if (block0_ctr_we)						// Condition Checking write_enable for writing new value of count to reg
            block0_ctr_reg <= block0_ctr_new;

          if (ready_we)								// Condition Checking write_enable of state to write new output in state reg
            ready_reg <= ready_new;

          if (chacha_ctrl_we)						// Condition checking write_enable for writing new value of fsm state to reg
            chacha_ctrl_reg <= chacha_ctrl_new;
        end
    end // reg_update


  //----------------------------------------------------------------
  // init_state_logic
  //
  // Calculates the initial state for a given block.
  //----------------------------------------------------------------
  always @*
    begin : init_state_logic
      reg [31 : 0] key0;							// Declaring 8 reg of size 32 bit to assign value of 256 bit key 					
      reg [31 : 0] key1;
      reg [31 : 0] key2;
      reg [31 : 0] key3;
      reg [31 : 0] key4;
      reg [31 : 0] key5;
      reg [31 : 0] key6;
      reg [31 : 0] key7;

      key0 = l2b(key[255 : 224]);				// Changing value of key from little to big endian
      key1 = l2b(key[223 : 192]);				// Changing value of key from little to big endian
      key2 = l2b(key[191 : 160]);				// Changing value of key from little to big endian
      key3 = l2b(key[159 : 128]);				// Changing value of key from little to big endian
      key4 = l2b(key[127 :  96]);				// Changing value of key from little to big endian
      key5 = l2b(key[95  :  64]);				// Changing value of key from little to big endian
      key6 = l2b(key[63  :  32]);				// Changing value of key from little to big endian
      key7 = l2b(key[31  :   0]);				// Changing value of key from little to big endian
      
        init_state_word[00] = cons0; 				// Assigning values of constant to initial state matrix
        init_state_word[01] = cons1; 				// Assigning values of constant to initial state matrix
        init_state_word[02] = cons2; 				// Assigning values of constant to initial state matrix
		init_state_word[03] = cons3; 				// Assigning values of constant to initial state matrix
		init_state_word[04] = key0; 				// Assigning values of key to initial state matrix
		init_state_word[05] = key1; 				// Assigning values of key to initial state matrix
		init_state_word[06] = key2; 				// Assigning values of key to initial state matrix
		init_state_word[07] = key3; 				// Assigning values of key to initial state matrix
		init_state_word[08] = key4; 				// Assigning values of key to initial state matrix
		init_state_word[09] = key5; 				// Assigning values of key to initial state matrix
		init_state_word[10] = key6; 				// Assigning values of key to initial state matrix
		init_state_word[11] = key7; 				// Assigning values of key to initial state matrix
		init_state_word[12] = block0_ctr_reg; 				// Assigning values of block_counter to initial state matrix
		init_state_word[13] = l2b(nonce[95 : 64]); 				// Assigning values of nonce to initial state matrix
		init_state_word[14] = l2b(nonce[63 : 32]); 				// Assigning values of nonce to initial state matrix
		init_state_word[15] = l2b(nonce[31 :  0]); 				// Assigning values of nonce to initial state matrix
		end

  //----------------------------------------------------------------
  // state_logic
  // Logic to init and update the internal state.
  //----------------------------------------------------------------
  always @*
    begin : state_logic
      integer i;										// Temperory variable to be used in loop

      for (i = 0 ; i < 16 ; i = i + 1)			//Loop to assign 0 value to state new
        state_new[i] = 32'h0;
      state_we = 0;									// Assigning write_enable flag to zero

      qr0_a = 32'h0;									// Assigning all quarterround reg to zero
      qr0_b = 32'h0;
      qr0_c = 32'h0;
      qr0_d = 32'h0;
      qr1_a = 32'h0;
      qr1_b = 32'h0;
      qr1_c = 32'h0;
      qr1_d = 32'h0;
      qr2_a = 32'h0;
      qr2_b = 32'h0;
      qr2_c = 32'h0;
      qr2_d = 32'h0;
      qr3_a = 32'h0;
      qr3_b = 32'h0;
      qr3_c = 32'h0;
      qr3_d = 32'h0;

      if (init_state)								// Condition when inital flag enable
        begin
          for (i = 0 ; i < 16 ; i = i + 1)	// Loop for assigning intial state values to new state which would futher be transfered to reg 
            state_new[i] = init_state_word[i];
          state_we   = 1;							// Enabling write flag to transfer to reg
        end // if (init_state)

      if (update_state)								// Condition for update flag
        begin
          state_we = 1;								// Enabling write flag to transfer to reg
          case (qr_ctr_reg)						// Using Case Statement for controling which type of quarterround to run
            QR0:										// QR0 flag indicate column type quarterround
              begin
                qr0_a = state_reg[00];			// Assigning state values to quarterround input
                qr0_b = state_reg[04];			// Assigning state values to quarterround input
                qr0_c = state_reg[08];			// Assigning state values to quarterround input
                qr0_d = state_reg[12];			// Assigning state values to quarterround input
                qr1_a = state_reg[01];			// Assigning state values to quarterround input
                qr1_b = state_reg[05];			// Assigning state values to quarterround input
                qr1_c = state_reg[09];			// Assigning state values to quarterround input
                qr1_d = state_reg[13];			// Assigning state values to quarterround input
                qr2_a = state_reg[02];			// Assigning state values to quarterround input
                qr2_b = state_reg[06];			// Assigning state values to quarterround input
                qr2_c = state_reg[10];			// Assigning state values to quarterround input
                qr2_d = state_reg[14];			// Assigning state values to quarterround input
                qr3_a = state_reg[03];			// Assigning state values to quarterround input
                qr3_b = state_reg[07];			// Assigning state values to quarterround input
                qr3_c = state_reg[11];			// Assigning state values to quarterround input
                qr3_d = state_reg[15];			// Assigning state values to quarterround input
                state_new[00] = qr0_a_prim;			// Assigning quarterround output to new state
                state_new[04] = qr0_b_prim;			// Assigning quarterround output to new state			
                state_new[08] = qr0_c_prim;			// Assigning quarterround output to new state			
                state_new[12] = qr0_d_prim;			// Assigning quarterround output to new state			
                state_new[01] = qr1_a_prim;			// Assigning quarterround output to new state			
                state_new[05] = qr1_b_prim;			// Assigning quarterround output to new state			
                state_new[09] = qr1_c_prim;			// Assigning quarterround output to new state
                state_new[13] = qr1_d_prim;			// Assigning quarterround output to new state		
                state_new[02] = qr2_a_prim;			// Assigning quarterround output to new state			
                state_new[06] = qr2_b_prim;			// Assigning quarterround output to new state			
                state_new[10] = qr2_c_prim;			// Assigning quarterround output to new state			
                state_new[14] = qr2_d_prim;			// Assigning quarterround output to new state			
                state_new[03] = qr3_a_prim;			// Assigning quarterround output to new state			
                state_new[07] = qr3_b_prim;			// Assigning quarterround output to new state		
                state_new[11] = qr3_c_prim;			// Assigning quarterround output to new state			
              end

            QR1:											// QR1 flag indicate diagonal type quarterround
              begin
                qr0_a = state_reg[00];			// Assigning state values to quarterround input
                qr0_b = state_reg[05];			// Assigning state values to quarterround input
                qr0_c = state_reg[10];			// Assigning state values to quarterround input
                qr0_d = state_reg[15];			// Assigning state values to quarterround input
                qr1_a = state_reg[01];			// Assigning state values to quarterround input
                qr1_b = state_reg[06];			// Assigning state values to quarterround input
                qr1_c = state_reg[11];			// Assigning state values to quarterround input
                qr1_d = state_reg[12];			// Assigning state values to quarterround input
                qr2_a = state_reg[02];			// Assigning state values to quarterround input
                qr2_b = state_reg[07];			// Assigning state values to quarterround input
                qr2_c = state_reg[08];			// Assigning state values to quarterround input
                qr2_d = state_reg[13];			// Assigning state values to quarterround input
                qr3_a = state_reg[03];			// Assigning state values to quarterround input
                qr3_b = state_reg[04];			// Assigning state values to quarterround input
                qr3_c = state_reg[09];			// Assigning state values to quarterround input
                qr3_d = state_reg[14];			// Assigning state values to quarterround input
                state_new[00] = qr0_a_prim;			// Assigning quarterround output to new state
                state_new[05] = qr0_b_prim;			// Assigning quarterround output to new state
                state_new[10] = qr0_c_prim;			// Assigning quarterround output to new state
                state_new[15] = qr0_d_prim;			// Assigning quarterround output to new state
                state_new[01] = qr1_a_prim;			// Assigning quarterround output to new state
                state_new[06] = qr1_b_prim;			// Assigning quarterround output to new state
                state_new[11] = qr1_c_prim;			// Assigning quarterround output to new state
                state_new[12] = qr1_d_prim;			// Assigning quarterround output to new state
                state_new[02] = qr2_a_prim;			// Assigning quarterround output to new state
                state_new[07] = qr2_b_prim;			// Assigning quarterround output to new state
                state_new[08] = qr2_c_prim;			// Assigning quarterround output to new state
                state_new[13] = qr2_d_prim;			// Assigning quarterround output to new state
                state_new[03] = qr3_a_prim;			// Assigning quarterround output to new state
                state_new[04] = qr3_b_prim;			// Assigning quarterround output to new state
                state_new[09] = qr3_c_prim;			// Assigning quarterround output to new state
                state_new[14] = qr3_d_prim;			// Assigning quarterround output to new state
              end
          endcase // case (quarterround_select)
        end // if (update_state)
    end // state_logic


  //----------------------------------------------------------------
  // data_out_logic
  // Final output logic that combines the result from state
  // update with the input block. This adds a 16 rounds and
  // a final layer of XOR gates.
  //
  // Note that we also remap all the words into LSB format.
  //----------------------------------------------------------------
  always @*
    begin : data_out_logic
      integer i;												// Temp variable used in loop
      reg [31 : 0] msb_block_state [0 : 15];			// Declaring 16 reg of size 32bit to store output in Msb form
      reg [31 : 0] lsb_block_state [0 : 15];			// Declaring 16 reg of size 32bit to store output in Lsb form
      reg [511 : 0] block_state;
      reg [31:0] block_state_temp;							// Declaring 512 bit reg to store final state

      for (i = 0 ; i < 16 ; i = i + 1)
        begin
          msb_block_state[i] = init_state_word[i] + state_reg[i];			// Loop for Adding new state with intial state matrix 
          lsb_block_state[i] = l2b(msb_block_state[i][31 : 0]);			// Converting From msb to lsb
        end

      block_state = {lsb_block_state[00], lsb_block_state[01],				// Combining all lsb state to final state matrix of size 512 bit
                     lsb_block_state[02], lsb_block_state[03],
                     lsb_block_state[04], lsb_block_state[05],
                     lsb_block_state[06], lsb_block_state[07],
                     lsb_block_state[08], lsb_block_state[09],
                     lsb_block_state[10], lsb_block_state[11],
                     lsb_block_state[12], lsb_block_state[13],
                     lsb_block_state[14], lsb_block_state[15]};
                     
      // XORing the state matrix with the input data
      //Uncomment the block as per requirement
      
      //32-bit XOR with input data
      block_state_temp = {lsb_block_state[02]};
      for (i=0;i<16;i = i+1)
        begin
            data_out_new[i*32 +:32] = data_in[i*32 +:32] ^ block_state_temp;
        end
       
//       //64-bit XOR with input data
//       block_state_temp = {lsb_block_state[02],lsb_block_state[03]};
//      for (i=0;i<16;i = i+1)
//        begin
//            data_out_new[i*64 +:64] = data_in[i*64 +:64] ^ block_state_temp;
//        end

//        //128-bit XOR with input data
//       block_state_temp = {lsb_block_state[02],lsb_block_state[03],
//                          lsb_block_state[04], lsb_block_state[05]};
//      for (i=0;i<4;i = i+1)
//        begin
//            data_out_new[i*128 +:128] = data_in[i*128 +:128] ^ block_state_temp;
//        end
        
//        //256-bit XOR with input data
//        block_state_temp = {lsb_block_state[02],lsb_block_state[03],
//                          lsb_block_state[04], lsb_block_state[05],
//                          lsb_block_state[06], lsb_block_state[07],
//                          lsb_block_state[08], lsb_block_state[09]};
//      for (i=0;i<2;i = i+1)
//        begin
//            data_out_new[i*256 +:256] = data_in[i*256 +:256] ^ block_state_temp;
//        end
        
        //512-bit XOR with input data
//      data_out_new = data_in ^ block_state;				// XORing the final output with 512bit input and assigning it to final data output
    
    end // data_out_logic


  //----------------------------------------------------------------
  // qr_ctr
  // Update logic for the quarterround counter, a monotonically
  // increasing counter with reset.
  //----------------------------------------------------------------
  always @*
    begin : qr_ctr
      qr_ctr_new = 0;										// Intializing counter for quarterround to zero
      qr_ctr_we  = 0;										// Intializing write_enable of quarterround to zero

      if (qr_ctr_rst)										// Condition to reset the counter
        begin
          qr_ctr_new = 0;									// Intializing counter for quarterround to zero
          qr_ctr_we  = 1;									// Intializing write_enable of quarterround to zero to write new in reg
        end

      if (qr_ctr_inc)										// Condition to increase the counter
        begin
          qr_ctr_new = qr_ctr_reg + 1'b1;				// Adding 1 bit to reg value and assigning it to new 
          qr_ctr_we  = 1;									// Assigning 1 to write_enable
        end
    end // qr_ctr


  //----------------------------------------------------------------
  // dr_ctr
  // Update logic for the round counter, a monotonically
  // increasing counter with reset.
  //----------------------------------------------------------------
  always @*
    begin : dr_ctr
      dr_ctr_new = 0;										// Intializing counter for round to zero
      dr_ctr_we  = 0;										// Intializing write_enable of round counter to zero

      if (dr_ctr_rst)										// Condition to reset the counter
        begin
          dr_ctr_new = 0;									// Intializing counter for round to zero
          dr_ctr_we  = 1;									// Intializing write_enable of round to zero to write new in reg
        end

      if (dr_ctr_inc)										// Condition to increase the counter
        begin
          dr_ctr_new = dr_ctr_reg + 1'b1;				// Adding 1 bit to reg value and assigning it to new 
          dr_ctr_we  = 1;									// Assigning 1 to write_enable
        end
    end // dr_ctr


  //----------------------------------------------------------------
  // block_ctr
  // Update logic for the 64-bit block counter, a monotonically
  // increasing counter with reset.
  //----------------------------------------------------------------
  always @*
    begin : block_ctr
      block0_ctr_new = 32'h0;								// Intializing counter for block to zero
      block0_ctr_we = 0;									// Intializing write_enable of block counter to zero

      if (block_ctr_set)									// Condition to set the counter
        begin
          block0_ctr_new = ctr[31 : 00];				// Intializing counter for block to input counter
          block0_ctr_we = 1;								// Intializing write_enable of round to zero to write new in reg
        end

      if (block_ctr_inc)									// Condition to increase the counter
        begin
          block0_ctr_new = block0_ctr_reg + 1;		// Adding 1 bit to reg value and assigning it to new 
          block0_ctr_we = 1;								// Assigning 1 to write_enable
        end
    end // block_ctr


  //----------------------------------------------------------------
  // chacha_ctrl_fsm
  // Logic for the state machine controlling the core behaviour.
  //----------------------------------------------------------------
  always @*
    begin : chacha_ctrl_fsm
      init_state         = 0;								// Intializing all the flag to zero or inital value
      update_state       = 0;
      update_output      = 0;
      qr_ctr_inc         = 0;
      qr_ctr_rst         = 0;
      dr_ctr_inc         = 0;
      dr_ctr_rst         = 0;
      block_ctr_inc      = 0;
      block_ctr_set      = 0;
      ready_new          = 0;
      ready_we           = 0;
      data_out_valid_new = 0;
      data_out_valid_we  = 0;
      chacha_ctrl_new    = CTRL_IDLE;
      chacha_ctrl_we     = 0;

      case (chacha_ctrl_reg)							// Using Case statement for controling FSM 
        CTRL_IDLE:										// When idle State
          begin
            if (init)									//If intial flag one
              begin
                block_ctr_set   = 1;				// Put block counter to set
                ready_new       = 0;				// Put ready to 0 
                ready_we        = 1;				// Write_enable flag set  
                chacha_ctrl_new = CTRL_INIT;		// Changing control to inital state and writing it in reg by enabling we
                chacha_ctrl_we  = 1;
              end
          end

        CTRL_INIT:										// When inital State
          begin
            init_state      = 1;						// Put intial state flag to 1
            qr_ctr_rst      = 1;						// Put quarterround counter flag to 1
            dr_ctr_rst      = 1;						// Put round counter flag to 1
            chacha_ctrl_new = CTRL_ROUNDS;		// Changing control to round state and writing it in reg by enabling we
            chacha_ctrl_we  = 1;
          end

        CTRL_ROUNDS:										// When Round state
          begin
            update_state = 1;							// Put update state flag to 1
            qr_ctr_inc   = 1;							// Put Inc quarterround counter flag to 1
            if (qr_ctr_reg == QR1)					// Condition For diagonal round
              begin
                dr_ctr_inc = 1;						//Put inc round counter  to 1
                if (dr_ctr_reg == (rounds[4 : 1] - 1))	// Condition checking number of rounds to perform
                  begin
                    chacha_ctrl_new = CTRL_FINALIZE;		// Change to Final State
                    chacha_ctrl_we  = 1;			// Write_enable flag to 1
                  end
              end
          end

        CTRL_FINALIZE:									// When Final State
          begin
            ready_new          = 1;					// Put raedy flag to 1
            ready_we           = 1;					// Write_enable flag to 1
            update_output      = 1;					// Put update output flag to 1
            data_out_valid_new = 1;					// Assigning valid out to 1 to transfer output
            data_out_valid_we  = 1;					// Write_enable flag to 1
            chacha_ctrl_new    = CTRL_DONE;		// Change state to done
            chacha_ctrl_we     = 1;					// Write_enable flag to 1
          end

        CTRL_DONE:										// when Done State
          begin
            if (init)									// If inital flag set
              begin
                ready_new          = 0;			// Put ready flag to 0
                ready_we           = 1;			// Write_enable flag to 0
                data_out_valid_new = 0;			// Assigning valid out to 1 to not transfer output
                data_out_valid_we  = 1;			// Write_enable flag to 0
                block_ctr_set      = 1;			// Put block counter to set
                chacha_ctrl_new    = CTRL_INIT;	// Changing State to inital
                chacha_ctrl_we     = 1;			// Write_enable flag to 1
              end
            else if (next)								// If next flag is set
              begin
                ready_new          = 0;			// Put ready flag to 0
                ready_we           = 1;			// Write_enable flag to 1
                data_out_valid_new = 0;			// Assigning valid out to 1 to not transfer output
                data_out_valid_we  = 1;			// Write_enable flag to 1
                block_ctr_inc      = 1;			// Put inc block counter to 1
                chacha_ctrl_new    = CTRL_INIT;	// Changing State to inital
                chacha_ctrl_we     = 1;			// Write_enable flag to 1
              end
          end

        default:
          begin

          end
      endcase // case (chacha_ctrl_reg)
    end // chacha_ctrl_fsm
endmodule // chacha_core

//======================================================================
// EOF chacha_core.v
//======================================================================
