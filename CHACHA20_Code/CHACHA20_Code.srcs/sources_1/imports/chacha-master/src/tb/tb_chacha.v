//======================================================================
//
// tb_chacha.v
// -----------
// Testbench for the Chacha top level wrapper.
//
//
// Copyright (c) 2013, Secworks Sweden AB
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

module tb_chacha();

  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  parameter DEBUG = 1;

  localparam CLK_HALF_PERIOD = 1;                  //Defining local parameter clock period
  localparam CLK_PERIOD = 2 * CLK_HALF_PERIOD;		//Defining local parameter for convenience

  localparam EIGHT_ROUNDS  = 8;							//Defining local parameter for convenience
  localparam TEN_ROUNDS = 10;								
  localparam TWELWE_ROUNDS = 12;
  localparam TWENTY_ROUNDS = 20;

  localparam ADDR_CTRL        = 8'h08;					//Defining local parameter to link address
  localparam CTRL_INIT_BIT    = 0;
  localparam CTRL_NEXT_BIT    = 1;						//Defining local parameter for convenience

  localparam ADDR_STATUS      = 8'h09;					//Defining local parameter to link address
  localparam STATUS_READY_BIT = 0;

  localparam ADDR_ROUNDS      = 8'h0b;
  localparam ROUNDS_HIGH_BIT  = 4;
  localparam ROUNDS_LOW_BIT   = 0;

  localparam ADDR_KEY0        = 8'h10;					//Defining local parameter to link address
  localparam ADDR_KEY1        = 8'h11;
  localparam ADDR_KEY2        = 8'h12;
  localparam ADDR_KEY3        = 8'h13;
  localparam ADDR_KEY4        = 8'h14;
  localparam ADDR_KEY5        = 8'h15;
  localparam ADDR_KEY6        = 8'h16;
  localparam ADDR_KEY7        = 8'h17;

  localparam ADDR_IV0         = 8'h20;					//Defining local parameter to link address
  localparam ADDR_IV1         = 8'h21;
  localparam ADDR_IV2         = 8'h22;

  localparam ADDR_DATA_IN0    = 8'h40;					//Defining local parameter to link address
  localparam ADDR_DATA_IN1   = 8'h41;
  localparam ADDR_DATA_IN2   = 8'h42;
  localparam ADDR_DATA_IN3    = 8'h43;
  localparam ADDR_DATA_IN4   = 8'h44;
  localparam ADDR_DATA_IN5   = 8'h45;
  localparam ADDR_DATA_IN6    = 8'h46;
  localparam ADDR_DATA_IN7   = 8'h47;
  localparam ADDR_DATA_IN8   = 8'h48;
  localparam ADDR_DATA_IN9    = 8'h49;
  localparam ADDR_DATA_IN10   = 8'h4a;
  localparam ADDR_DATA_IN11   = 8'h4b;
  localparam ADDR_DATA_IN12    = 8'h4c;
  localparam ADDR_DATA_IN13   = 8'h4d;
  localparam ADDR_DATA_IN14   = 8'h4e;
  localparam ADDR_DATA_IN15   = 8'h4f;


  localparam ADDR_DATA_OUT0   = 8'h80;					//Defining local parameter to link address
  localparam ADDR_DATA_OUT1   = 8'h81;
  localparam ADDR_DATA_OUT2   = 8'h82;
  localparam ADDR_DATA_OUT3   = 8'h83;
  localparam ADDR_DATA_OUT4   = 8'h84;
  localparam ADDR_DATA_OUT5   = 8'h85;
  localparam ADDR_DATA_OUT6   = 8'h86;
  localparam ADDR_DATA_OUT7   = 8'h87;
  localparam ADDR_DATA_OUT8   = 8'h88;
  localparam ADDR_DATA_OUT9   = 8'h89;
  localparam ADDR_DATA_OUT10  = 8'h8a;
  localparam ADDR_DATA_OUT11  = 8'h8b;
  localparam ADDR_DATA_OUT12  = 8'h8c;
  localparam ADDR_DATA_OUT13  = 8'h8d;
  localparam ADDR_DATA_OUT14  = 8'h8e;
  localparam ADDR_DATA_OUT15  = 8'h8f;
  localparam N = 141;										//Defining local parameter for input file 
  localparam M = (N*512) - 1;								//Defining local parameter for input file
  
  //----------------------------------------------------------------
  // Register and Wire declarations.
  //----------------------------------------------------------------
  reg tb_clk;													// Declaring reg as clock
  reg tb_reset_n;												// Declaring reg for reset flag

  reg           tb_cs;										// Declaring reg as control state			
  reg           tb_write_read;							// Declaring reg for write and read enable

  reg  [7 : 0]  tb_address;								// Declaring reg for address in input
  reg  [31 : 0] tb_data_in;								// Declaring reg for data as input
  wire [31 : 0] tb_data_out;								// Declaring reg for data as output
 
  reg [63 : 0] cycle_ctr;							 		// Declaring reg for cycle count
  reg [31 : 0] error_ctr;									// Declaring reg for error count
  reg [31 : 0] tc_ctr;										// Declaring reg for test case count
  
  reg [31 : 0] read_data;									// Declaring reg for output data									

  reg [511 : 0] extracted_data;							// Declaring reg to combine all output data

  reg display_cycle_ctr;									// Declaring reg as flag of display cycle counter
  reg display_read_write;									// Declaring reg as flag of write read 
  reg display_core_state;									// Declaring reg as flag of display of core state

  reg [11:0] RAM [0:5999];									// Declaring reg to extract data from input text file
  reg [0:M] RAM1;												// Declaring reg to extract data from input text file
  reg [511:0] RAM2 [140:0];								// Declaring reg to extract data from input text file
  integer outfile1;											// Declaring temp integer to extract data from input text file
  //----------------------------------------------------------------
  // Chacha device under test.
  //----------------------------------------------------------------
  chacha dut(													// chacha module instantaion 
             .clk(tb_clk),
             .reset_n(tb_reset_n),
             .cs(tb_cs),
             .we(tb_write_read),
             .addr(tb_address),
             .write_data(tb_data_in),
             .read_data(tb_data_out),
             .N(N)
            );


  //----------------------------------------------------------------
  // clk_gen
  //
  // Clock generator process.
  //----------------------------------------------------------------
  always
    begin : clk_gen
      #CLK_HALF_PERIOD tb_clk = !tb_clk;			//Delay of CLK_HALF_PERIOD to generate clock
    end // clk_gen


  //--------------------------------------------------------------------
  // dut_monitor
  //
  // Monitor displaying information every cycle.
  // Includes the cycle counter.
  //--------------------------------------------------------------------
  always @ (posedge tb_clk)							// To display information of each cycle
    begin : dut_monitor
      cycle_ctr = cycle_ctr + 1;						// Incrementing cycle count

      if (display_cycle_ctr)							// if display counter flag in enabled
        begin
          $display("cycle = %016x:", cycle_ctr);
        end

      if (display_core_state)							// if display core state flag in enabled then display the states of matrix
        begin
          $display("core ctrl: 0x%02x, core_qr_ctr: 0x%02x, core_dr_ctr: 0x%02x, init: 0x%01x, next: 0x%01x, core_ready: 0x%02x",
                   dut.core.chacha_ctrl_reg, dut.core.qr_ctr_reg,
                   dut.core.dr_ctr_reg, dut.core.init,
                   dut.core.next, dut.core.ready_reg);

          $display("state0_reg  = 0x%08x, state1_reg  = 0x%08x, state2_reg  = 0x%08x, state3_reg  = 0x%08x",
                   dut.core.state_reg[00], dut.core.state_reg[01], dut.core.state_reg[02], dut.core.state_reg[03]);
          $display("state4_reg  = 0x%08x, state5_reg  = 0x%08x, state6_reg  = 0x%08x, state7_reg  = 0x%08x",
                   dut.core.state_reg[04], dut.core.state_reg[05], dut.core.state_reg[06], dut.core.state_reg[07]);
          $display("state8_reg  = 0x%08x, state9_reg  = 0x%08x, state10_reg = 0x%08x, state11_reg = 0x%08x",
                   dut.core.state_reg[08], dut.core.state_reg[09], dut.core.state_reg[10], dut.core.state_reg[11]);
          $display("state12_reg = 0x%08x, state13_reg = 0x%08x, state14_reg = 0x%08x, state15_reg = 0x%08x",
                   dut.core.state_reg[12], dut.core.state_reg[13], dut.core.state_reg[14], dut.core.state_reg[15]);
          $display("");
        end

      if (display_read_write)							// if display read write flag in enabled
        begin

          if (dut.cs)
            begin
              if (dut.we)
                begin
                  $display("*** Write acess: addr 0x%02x = 0x%08x", dut.addr, dut.write_data);
                end
              else
                begin
                  $display("*** Read acess: addr 0x%02x = 0x%08x", dut.addr, dut.read_data);
                end
            end
        end

    end // dut_monitor


  //----------------------------------------------------------------
  // reset_dut
  //----------------------------------------------------------------
  task reset_dut;									// task to reset the reset flag after 2 clock peroid
    begin
      tb_reset_n = 0;
      #(2 * CLK_PERIOD);
      tb_reset_n = 1;
    end
  endtask // reset_dut


  //----------------------------------------------------------------
  // init_sim()
  //
  // Set the input to the DUT to defined values.
  //----------------------------------------------------------------
  task init_sim;									// Task to initalize all flags to zero
    begin
      cycle_ctr     = 0;
      error_ctr     = 0;
      tc_ctr        = 0;
      tb_clk        = 0;
      tb_reset_n    = 0;
      tb_cs         = 0;
      tb_write_read = 0;
      tb_address    = 8'h0;
      tb_data_in    = 32'h0;

      display_cycle_ctr  = 0;
      display_read_write = 0;
      display_core_state = 0;
    end
  endtask // init_sim


  //----------------------------------------------------------------
  // read_reg
  //
  // Task that reads and display the value of
  // a register in the dut.
  //----------------------------------------------------------------
  task read_reg(input [7 : 0] addr);			// Task to read register value
    begin
      tb_cs         = 1;
      tb_write_read = 0;
      tb_address    = addr;						// giving value of addr in input of module
      #(CLK_PERIOD);									// After one clock peroid delay again initalizing to zero
      tb_cs         = 0;
      tb_write_read = 0;
      tb_address    = 8'h0;
      tb_data_in    = 32'h0;
    end
  endtask // read_reg


  //----------------------------------------------------------------
  // write_reg
  //
  // Task that writes to a register in the dut.
  //----------------------------------------------------------------
  task write_reg(input [7 : 0] addr, input [31 : 0] data);
    begin												// Task to write value in register 
      tb_cs         = 1;
      tb_write_read = 1;
      tb_address    = addr;						// giving value of addr in input of module
      tb_data_in    = data;						// giving value of data in input of module
      #(CLK_PERIOD);									// After one clock peroid delay again initalizing to zero
      tb_cs         = 0;
      tb_write_read = 0;
      tb_address    = 8'h0;
      tb_data_in    = 32'h0;
    end
  endtask // write_reg


  //----------------------------------------------------------------
  // dump_top_state
  //
  // Dump the internal state of the top to std out.
  //----------------------------------------------------------------
  task dump_top_state;								// Task to display all the contents of initial state
    begin
      $display("");
      $display("Top internal state");
      $display("------------------");
      $display("init_reg   = %01x", dut.init_reg);
      $display("next_reg   = %01x", dut.next_reg);
//      $display("keylen_reg = %01x", dut.keylen_reg);
      $display("rounds_reg = %01x", dut.rounds_reg);
      $display("");

      $display("key0_reg = %08x, key1_reg  = %08x, key2_reg = %08x, key3_reg  = %08x",
               dut.key_reg[0], dut.key_reg[1], dut.key_reg[2], dut.key_reg[3]);
      $display("key4_reg = %08x, key5_reg  = %08x, key6_reg = %08x, key7_reg  = %08x",
               dut.key_reg[4], dut.key_reg[5], dut.key_reg[6], dut.key_reg[7]);
      $display("");

      $display("iv0_reg = %08x, iv1_reg = %08x,iv2_reg = %08x", dut.nonce_reg[0], dut.nonce_reg[1],dut.nonce_reg[2]);
      $display("");

      $display("data_in0_reg  = %08x, data_in1_reg   = %08x, data_in2_reg  = %08x, data_in3_reg   = %08x",
               dut.data_in_reg[00], dut.data_in_reg[01], dut.data_in_reg[02], dut.data_in_reg[03]);
      $display("data_in4_reg  = %08x, data_in5_reg   = %08x, data_in6_reg  = %08x, data_in7_reg   = %08x",
               dut.data_in_reg[04], dut.data_in_reg[05], dut.data_in_reg[06], dut.data_in_reg[07]);
      $display("data_in8_reg  = %08x, data_in9_reg   = %08x, data_in10_reg = %08x, data_in11_reg  = %08x",
               dut.data_in_reg[08], dut.data_in_reg[09], dut.data_in_reg[10], dut.data_in_reg[11]);
      $display("data_in12_reg = %08x, data_in13_reg  = %08x, data_in14_reg = %08x, data_in15_reg  = %08x",
               dut.data_in_reg[12], dut.data_in_reg[13], dut.data_in_reg[14], dut.data_in_reg[15]);
      $display("");

      $display("ready = 0x%01x, data_out_valid = %01x", dut.core_ready, dut.core_data_out_valid);
      $display("data_out00 = %08x, data_out01 = %08x, data_out02 = %08x, data_out03 = %08x",
               dut.core_data_out[511 : 480], dut.core_data_out[479 : 448],
               dut.core_data_out[447 : 416], dut.core_data_out[415 : 384]);
      $display("data_out04 = %08x, data_out05 = %08x, data_out06 = %08x, data_out07 = %08x",
               dut.core_data_out[383 : 352], dut.core_data_out[351 : 320],
               dut.core_data_out[319 : 288], dut.core_data_out[287 : 256]);
      $display("data_out08 = %08x, data_out09 = %08x, data_out10 = %08x, data_out11 = %08x",
               dut.core_data_out[255 : 224], dut.core_data_out[223 : 192],
               dut.core_data_out[191 : 160], dut.core_data_out[159 : 128]);
      $display("data_out12 = %08x, data_out13 = %08x, data_out14 = %08x, data_out15 = %08x",
               dut.core_data_out[127 :  96], dut.core_data_out[95  :  64],
               dut.core_data_out[63  :  32], dut.core_data_out[31  :   0]);
      $display("");
    end
  endtask // dump_top_state


  //----------------------------------------------------------------
  // dump_core_state
  //
  // Dump the internal state of the core to std out.
  //----------------------------------------------------------------
  task dump_core_state;								// Task to display all the contents of updated state
    begin
      $display("");
      $display("Core internal state");
      $display("-------------------");
      $display("Round state:");
      $display("state0_reg  = 0x%08x, state1_reg  = 0x%08x, state2_reg  = 0x%08x, state3_reg  = 0x%08x",
               dut.core.state_reg[00], dut.core.state_reg[01], dut.core.state_reg[02], dut.core.state_reg[03]);
      $display("state4_reg  = 0x%08x, state5_reg  = 0x%08x, state6_reg  = 0x%08x, state7_reg  = 0x%08x",
               dut.core.state_reg[04], dut.core.state_reg[05], dut.core.state_reg[06], dut.core.state_reg[07]);
      $display("state8_reg  = 0x%08x, state9_reg  = 0x%08x, state10_reg = 0x%08x, state11_reg = 0x%08x",
               dut.core.state_reg[08], dut.core.state_reg[09], dut.core.state_reg[10], dut.core.state_reg[11]);
      $display("state12_reg = 0x%08x, state13_reg = 0x%08x, state14_reg = 0x%08x, state15_reg = 0x%08x",
               dut.core.state_reg[12], dut.core.state_reg[13], dut.core.state_reg[14], dut.core.state_reg[15]);
      $display("");

      $display("rounds = %01x", dut.core.rounds);
      $display("qr_ctr_reg = %01x, dr_ctr_reg  = %01x", dut.core.qr_ctr_reg, dut.core.dr_ctr_reg);
      $display("");

      $display("chacha_ctrl_reg = %02x", dut.core.chacha_ctrl_reg);
      $display("");

      $display("data_in = %064x", dut.core.data_in);
      $display("data_out_valid_reg = %01x", dut.core.data_out_valid_reg);
      $display("");

      $display("qr0_a_prim = %08x, qr0_b_prim = %08x", dut.core.qr0_a_prim, dut.core.qr0_b_prim);
      $display("qr0_c_prim = %08x, qr0_d_prim = %08x", dut.core.qr0_c_prim, dut.core.qr0_d_prim);
      $display("");
    end
  endtask // dump_core_state


  //----------------------------------------------------------------
  // display_test_result()
  //
  // Display the accumulated test results.
  //----------------------------------------------------------------
  task display_test_result;									// Task to display if any error comes
    begin
      if (error_ctr == 0)
        begin
          $display("*** All %02d test cases completed successfully", tc_ctr);
        end
      else
        begin
          $display("*** %02d test cases did not complete successfully.", error_ctr);
        end
    end
  endtask // display_test_result

  //----------------------------------------------------------------
  // write_localparams()
  //
  // Write key, iv and other parameters to the dut.
  //----------------------------------------------------------------
  task write_parameters(input [256 : 0] key,						// Task to write value in registers
                            input [95 : 0]  nonce,
                            input [4 : 0]   rounds,input[511:0] data_inpt);
    begin : xyz
    integer k;

      write_reg(ADDR_KEY0, key[255 : 224]);						// Calling write task to assign value in diffrent address
      write_reg(ADDR_KEY1, key[223 : 192]);
      write_reg(ADDR_KEY2, key[191 : 160]);
      write_reg(ADDR_KEY3, key[159 : 128]);
      write_reg(ADDR_KEY4, key[127 :  96]);
      write_reg(ADDR_KEY5, key[95  :  64]);
      write_reg(ADDR_KEY6, key[63  :  32]);
      write_reg(ADDR_KEY7, key[31 :    0]);
      write_reg(ADDR_IV0, nonce[95 : 64]);
      write_reg(ADDR_IV1, nonce[63 : 32]);
      write_reg(ADDR_IV2, nonce[31 : 0]);
      write_reg(ADDR_ROUNDS, {{27'h0}, rounds});
      write_reg(ADDR_DATA_IN0, data_inpt[511:480]);
      write_reg(ADDR_DATA_IN1, data_inpt[479:448]);
      write_reg(ADDR_DATA_IN2, data_inpt[447:416]);
      write_reg(ADDR_DATA_IN3, data_inpt[415:384]);
      write_reg(ADDR_DATA_IN4, data_inpt[383:352]);
      write_reg(ADDR_DATA_IN5, data_inpt[351:320]);
      write_reg(ADDR_DATA_IN6, data_inpt[319:288]);
      write_reg(ADDR_DATA_IN7, data_inpt[287:256]);
      write_reg(ADDR_DATA_IN8, data_inpt[255:224]);
      write_reg(ADDR_DATA_IN9, data_inpt[223:192]);
      write_reg(ADDR_DATA_IN10, data_inpt[191:160]);
      write_reg(ADDR_DATA_IN11, data_inpt[159:128]);
      write_reg(ADDR_DATA_IN12, data_inpt[127:96]);
      write_reg(ADDR_DATA_IN13, data_inpt[95:64]);
      write_reg(ADDR_DATA_IN14, data_inpt[63:32]);
      write_reg(ADDR_DATA_IN15, data_inpt[31:0]);
    end
  endtask // write_parameters


  //----------------------------------------------------------------
  // start_init_block()
  //
  // Toggle the init signal in the dut to make it start processing
  // the first block available in the data in registers.
  //
  // Note: It is the callers responsibility to call the function
  // when the dut is ready to react on the init signal.
  //----------------------------------------------------------------
  task start_init_block;												// Task to start intial block by toggling address of control 
    begin
      write_reg(ADDR_CTRL, 32'h00000001);
      #(2 * CLK_PERIOD);
      write_reg(ADDR_CTRL, 32'h00000000);
    end
  endtask // start_init_block


  //----------------------------------------------------------------
  // start_next_block()
  //
  // Toggle the next signal in the dut to make it start processing
  // the next block available in the data in registers.
  //
  // Note: It is the callers responsibility to call the function
  // when the dut is ready to react on the next signal.
  //----------------------------------------------------------------
  task start_next_block;												// Task to start next block by toggling address of control
    begin
      write_reg(ADDR_CTRL, 32'h00000002);
      #(2 * CLK_PERIOD);
      write_reg(ADDR_CTRL, 32'h00000000);

      if (DEBUG)
        begin
          $display("Debug of next state.");
          dump_core_state();
          #(2 * CLK_PERIOD);
          dump_core_state();
        end
    end
  endtask // start_next_block


  //----------------------------------------------------------------
  // wait_ready()
  //
  // Wait for the ready flag in the dut to be set.
  //
  // Note: It is the callers responsibility to call the function
  // when the dut is actively processing and will in fact at some
  // point set the flag.
  //----------------------------------------------------------------
  task wait_ready;														// Task to wait until Ready signal comes
    begin
      while (!tb_data_out[STATUS_READY_BIT])
        begin
          read_reg(ADDR_STATUS);
        end
    end
  endtask // wait_ready


  //----------------------------------------------------------------
  // extract_data()
  //
  // Extracts all 16 data out words and combine them into the
  // global extracted_data.
  //----------------------------------------------------------------
  task extract_data;												// Task that takes all output and combines to 512-bit  
    begin:Extract_data
    integer k;
    
          read_reg(ADDR_DATA_OUT0);							//reading address and then assigning it to final extracted data
          extracted_data [511 : 480] = tb_data_out;
          read_reg(ADDR_DATA_OUT1);
          extracted_data [479 : 448] = tb_data_out;
          read_reg(ADDR_DATA_OUT2);
          extracted_data [447 : 416] = tb_data_out;
          read_reg(ADDR_DATA_OUT3);
          extracted_data [415 : 384] = tb_data_out;
          read_reg(ADDR_DATA_OUT4);
          extracted_data [383 : 352] = tb_data_out;
          read_reg(ADDR_DATA_OUT5);
          extracted_data [351 : 320] = tb_data_out;
          read_reg(ADDR_DATA_OUT6);
          extracted_data [319 : 288] = tb_data_out;
          read_reg(ADDR_DATA_OUT7);
          extracted_data [287 : 256] = tb_data_out;
          read_reg(ADDR_DATA_OUT8);
          extracted_data [255 : 224] = tb_data_out;
          read_reg(ADDR_DATA_OUT9);
          extracted_data [223 : 192] = tb_data_out;
          read_reg(ADDR_DATA_OUT10);
          extracted_data [191 : 160] = tb_data_out;
          read_reg(ADDR_DATA_OUT11);
          extracted_data [159 : 128] = tb_data_out;
          read_reg(ADDR_DATA_OUT12);
          extracted_data [127 :  96] = tb_data_out;
          read_reg(ADDR_DATA_OUT13);
          extracted_data [95  :  64] = tb_data_out;
          read_reg(ADDR_DATA_OUT14);
          extracted_data [63  :  32] = tb_data_out;
          read_reg(ADDR_DATA_OUT15);
          extracted_data [31  :   0] = tb_data_out;
          $fdisplay(outfile1,"%h",extracted_data);		//Displying final output

      
    end
  endtask // extract_data

  //----------------------------------------------------------------
  // run_test_vector()
  //
  // Runs a test case based on the given test vector.
  //----------------------------------------------------------------
  task run_test_vector(											// Task to take input and start the algorithm
                       input [256 : 0] key,
                       input [95 : 0]  nonce,
                       input [4 : 0]   rounds,
                       input [511:0] data_inpt,
                       input [7:0] k);
    begin
      tc_ctr = tc_ctr + 1;										// incrementing test case counter
      $display("***----------***");
      $display("Count = %01x",k);
      $display("***----------***");
      write_parameters(key, nonce, rounds,data_inpt);	// writing value in Reg

      start_init_block();							 			//Intialize to start inital block
      $display("*** Started.");
      wait_ready();												//Wait for ready signal
      $display("*** Ready seen.");
      dump_top_state();								 			//Intialize to next block
      extract_data();
      
      $display("Final Output:");
      $display("0x%064x", extracted_data);				//Displaying final output
      $display("");
    end
  endtask // run_test_vector


  //----------------------------------------------------------------
  // chacha_test
  // The main test functionality.
  //----------------------------------------------------------------
  initial
    begin : chacha_test
    integer k;
      $display("   -- Testbench for chacha started --");
      init_sim();														// intialzing all flags to zero
      reset_dut();													// To change reset flag

      $display("State at init after reset:");
      dump_top_state();												//Displaying intial state 
      $readmemh("test4.txt", RAM);								//reading input data from text file and storing in reg
      RAM1 = RAM[0];
      for(k = 1;k<6000;k = k+1)									// Dividing them from 6000 row to other reg
        begin
            RAM1 = {RAM1,RAM[k]};
        end
      for(k=0;k<141; k = k+1)									  	// Dividing them in 512 bits each
        begin
            RAM2[k] = RAM1[k*512 +:512];
        end
      $display(" ");
      outfile1 = $fopen("out_32bit.txt","w");                       //Creating output file to store the data
      $display("TC1-: Given key 256 bits and Nonce. 96 bits, 20 rounds.");
      for (k = 0;k<141;k = k+1)									//For loop to run algorithm for all input
        begin                                                   // Intializing to run the algorithm with key,nonce,rounds and input data
              run_test_vector(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f,
                            96'h000000090000004a00000000,   
                            TWENTY_ROUNDS,
                            RAM2[k],k);
              #(100 * CLK_PERIOD);
              
        end
        $fclose(outfile1);

      display_test_result();										//Display final 
      $display("*** Chacha simulation done.");
    end // chacha_test
endmodule // tb_chacha

//======================================================================
// EOF tb_chacha.v
//======================================================================
