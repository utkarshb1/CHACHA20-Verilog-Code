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

  localparam CLK_HALF_PERIOD = 1;                   //Defining local parameter clock period
  localparam CLK_PERIOD = 2 * CLK_HALF_PERIOD;

  localparam KEY_256_BITS = 1;                      //Defining local paramter for representing a 256-bit Key


  localparam EIGHT_ROUNDS  = 8;
  localparam TEN_ROUNDS = 10;
  localparam TWELWE_ROUNDS = 12;
  localparam TWENTY_ROUNDS = 20;

  localparam DISABLE = 0;
  localparam ENABLE  = 1;

  localparam ADDR_CTRL        = 8'h08;
  localparam CTRL_INIT_BIT    = 0;
  localparam CTRL_NEXT_BIT    = 1;

  localparam ADDR_STATUS      = 8'h09;
  localparam STATUS_READY_BIT = 0;

  localparam ADDR_ROUNDS      = 8'h0b;
  localparam ROUNDS_HIGH_BIT  = 4;
  localparam ROUNDS_LOW_BIT   = 0;

  localparam ADDR_KEY0        = 8'h10;
  localparam ADDR_KEY1        = 8'h11;
  localparam ADDR_KEY2        = 8'h12;
  localparam ADDR_KEY3        = 8'h13;
  localparam ADDR_KEY4        = 8'h14;
  localparam ADDR_KEY5        = 8'h15;
  localparam ADDR_KEY6        = 8'h16;
  localparam ADDR_KEY7        = 8'h17;

  localparam ADDR_IV0         = 8'h20;
  localparam ADDR_IV1         = 8'h21;
  localparam ADDR_IV2         = 8'h22;

  localparam ADDR_DATA_IN0    = 8'h40;
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


  localparam ADDR_DATA_OUT0   = 8'h80;
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
  localparam N = 141;
  localparam M = (N*512) - 1;
//  reg [N-1:0] addr_data_in;
//  reg [N-1:0] addr_data_out;

  
  //----------------------------------------------------------------
  // Register and Wire declarations.
  //----------------------------------------------------------------
  reg tb_clk;
  reg tb_reset_n;

  reg           tb_cs;
  reg           tb_write_read;

  reg  [7 : 0]  tb_address;
  reg  [31 : 0] tb_data_in;
  wire [31 : 0] tb_data_out;
  wire          tb_error;

  reg [63 : 0] cycle_ctr;
  reg [31 : 0] error_ctr;
  reg [31 : 0] tc_ctr;

  reg          error_found;
  reg [31 : 0] read_data;

  reg [511 : 0] extracted_data;
//  reg [M : 0] extracted_data;

  reg display_cycle_ctr;
  reg display_read_write;
  reg display_core_state;

  reg [11:0] RAM [0:5999];
  reg [0:M] RAM1;
  reg [511:0] RAM2 [140:0];
  integer outfile1;
  //----------------------------------------------------------------
  // Chacha device under test.
  //----------------------------------------------------------------
  chacha dut(
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
      #CLK_HALF_PERIOD tb_clk = !tb_clk;
    end // clk_gen


  //--------------------------------------------------------------------
  // dut_monitor
  //
  // Monitor displaying information every cycle.
  // Includes the cycle counter.
  //--------------------------------------------------------------------
  always @ (posedge tb_clk)
    begin : dut_monitor
      cycle_ctr = cycle_ctr + 1;

      if (display_cycle_ctr)
        begin
          $display("cycle = %016x:", cycle_ctr);
        end

      if (display_core_state)
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

      if (display_read_write)
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
  task reset_dut;
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
  task init_sim;
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
  task read_reg(input [7 : 0] addr);
    begin
      tb_cs         = 1;
      tb_write_read = 0;
      tb_address    = addr;
      #(CLK_PERIOD);
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
    begin
      tb_cs         = 1;
      tb_write_read = 1;
      tb_address    = addr;
      tb_data_in    = data;
      #(CLK_PERIOD);
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
  task dump_top_state;
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
  task dump_core_state;
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
//      $display("block0_ctr_reg = %08x, block1_ctr_reg = %08x", dut.core.block0_ctr_reg, dut.core.block1_ctr_reg);

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
  task display_test_result;
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
  // read_write_test()
  //
  // Simple test case that tries to read and write to the
  // registers in the dut.
  //
  // Note: Currently not self testing. No expected values.
  //----------------------------------------------------------------
  task read_write_test;
    begin
      tc_ctr = tc_ctr + 1;

      write_reg(ADDR_KEY0, 32'h55555555);
      read_reg(ADDR_KEY0);
      write_reg(ADDR_KEY1, 32'haaaaaaaa);
      read_reg(ADDR_KEY1);
      read_reg(ADDR_CTRL);
      read_reg(ADDR_STATUS);
      read_reg(ADDR_ROUNDS);

      read_reg(ADDR_KEY0);
      read_reg(ADDR_KEY1);
      read_reg(ADDR_KEY2);
      read_reg(ADDR_KEY3);
      read_reg(ADDR_KEY4);
      read_reg(ADDR_KEY5);
      read_reg(ADDR_KEY6);
      read_reg(ADDR_KEY7);
    end
  endtask // read_write_test


  //----------------------------------------------------------------
  // write_localparams()
  //
  // Write key, iv and other parameters to the dut.
  //----------------------------------------------------------------
  task write_parameters(input [256 : 0] key,
                            input [95 : 0]  nonce,
                            input [4 : 0]   rounds,input[511:0] data_inpt);
    begin : xyz
    integer k;
//    addr_data_in[0] = 12'h100;
//    addr_data_out[0] = 12'h300;
      write_reg(ADDR_KEY0, key[255 : 224]);
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
//      write_reg(addr_data_in[0],data_inpt[31:0]);
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
      
      
      
//      for(k=1;k<N;k = k+1)
//        begin
//            addr_data_in[k] = addr_data_in[k-1] + 12'h1;
//            write_reg(addr_data_in[k],data_inpt[k*32 +:32]);
//            addr_data_out[k] = addr_data_out[k-1] + 12'h1;                   
//        end
      
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
  task start_init_block;
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
  task start_next_block;
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
  task wait_ready;
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
  task extract_data;
    begin:Extract_data
    integer k;
    
    
          read_reg(ADDR_DATA_OUT0);
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
          
//          $display ("RAM[%b]=%d",i,RAM[i]);
          $fdisplay(outfile1,"%h",extracted_data);
//       for(k=N;k>=0;k = k-1)
//        begin
//            read_reg(addr_data_out[N-k]);
//            extracted_data[(k*32-1) -:32] = tb_data_out;          
//        end
      
    end
  endtask // extract_data

  //----------------------------------------------------------------
  // run_two_blocks_test_vector()
  //
  // Runs a test case with two blocks based on the given
  // test vector. Only the final block is compared.
  //----------------------------------------------------------------
//  task run_two_blocks_test_vector(
//                                  input [256 : 0] key,
//                                  input [95 : 0]  iv,
//                                  input [4 : 0]   rounds,
//                                  input[M:0]data_inpt);
//    begin
//      tc_ctr = tc_ctr + 1;

//      write_parameters(key, iv, rounds,data_inpt);
//      start_init_block();
//      wait_ready();
//      extract_data();

//      if (DEBUG)
//        begin
//          $display("State after first block:");
//          dump_core_state();

//          $display("First block:");
//          $display("0x%064x", extracted_data);
//        end

//      start_next_block();

//      if (DEBUG)
//        begin
//          $display("State after init of second block:");
//          dump_core_state();
//        end

//      wait_ready();
//      extract_data();

//      if (DEBUG)
//        begin
//          $display("State after init of second block:");
//          dump_core_state();

//          $display("Second block:");
//          $display("0x%064x", extracted_data);
//        end
//      $display("Final Output:");
//      $display("0x%064x", extracted_data);
//      $display("");
//    end
//  endtask // run_two_blocks_test_vector


  //----------------------------------------------------------------
  // run_test_vector()
  //
  // Runs a test case based on the given test vector.
  //----------------------------------------------------------------
  task run_test_vector(
                       input [256 : 0] key,
                       input [95 : 0]  nonce,
                       input [4 : 0]   rounds,
                       input [511:0] data_inpt,
                       input [7:0] k);
    begin
      tc_ctr = tc_ctr + 1;
      $display("***----------***");
      $display("Count = %01x",k);
      $display("***----------***");
      write_parameters(key, nonce, rounds,data_inpt);

      start_init_block();
      $display("*** Started.");
      wait_ready();
      $display("*** Ready seen.");
      dump_top_state();
      extract_data();
      
      $display("Final Output:");
      $display("0x%064x", extracted_data);
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
      init_sim();
      reset_dut();

      $display("State at init after reset:");
      dump_top_state();
      $readmemh("test5.txt", RAM);
      RAM1 = RAM[0];
      for(k = 1;k<6000;k = k+1)
        begin
            RAM1 = {RAM1,RAM[k]};
        end
      for(k=0;k<141; k = k+1)
        begin
            RAM2[k] = RAM1[k*512 +:512];
        end
//      $display("0x%064x",RAM2[140]);
      $display(" ");
      outfile1 = $fopen("out5.txt","w");
      $display("TC1-: Given key 256 bits and Nonce. 96 bits, 20 rounds.");
      for (k = 0;k<141;k = k+1)
        begin
//              $display("Count = %01x",k);
              run_test_vector(/*TC7, TWO,*/
                            256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f,
                            96'h000000090000004a00000000,
                            TWENTY_ROUNDS,
                            RAM2[k],k);
//                            512'h4c616469657320616e642047656e746c656d656e206f662074686520636c617373206f66202739393a204966204920636f756c64206f6666657220796f75206f);
              #(100 * CLK_PERIOD);
              
        end
        $fclose(outfile1);
//      run_two_blocks_test_vector(
//                                 256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f,
//                                 KEY_256_BITS,
//                                 96'h000000090000004a00000000,
//                                 TWENTY_ROUNDS,
//                                 512'hfe882395601ce8aded444867fe62ed8741420002e5d28bb573113a418c1f4008e954c188f38ec4f26bb8555e2b7c92bf4380e2ea9e553187fdd42821794416de,
//                                 512'h4c616469657320616e642047656e746c656d656e206f662074686520636c617373206f66202739393a204966204920636f756c64206f6666657220796f75206f);

      display_test_result();
      $display("*** chacha simulation done.");
//      $finish;
    end // chacha_test
endmodule // tb_chacha

//======================================================================
// EOF tb_chacha.v
//======================================================================
