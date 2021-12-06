// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

`include "apb/asssign.svh"
`include "apb/typedef.svh"

// Author: Luca Valente <luca.valente@unibo.it>
  
/// Testbench for APB FLL Interface
module apb_fll_tb #(
    parameter int unsigned APB_ADDR_WIDTH = 12,
    parameter int unsigned NR_FLLS        = 3
);

   localparam int unsigned RTC_CLOCK_PERIOD = 30.517us;
   localparam time SYS_TA   = 1ns;
   localparam time SYS_TT   = 2ns;

   localparam int unsigned ApbAddrWidth      = 32;
   localparam int unsigned ApbDataWidth      = 32;
   localparam int unsigned ApbStrbWidth      = 4;
   localparam int unsigned RegDataWidth      = 32;
   localparam int unsigned NoApbRegs         = 12;   
   localparam logic [NoApbRegs-1:0] ReadOnly = 12'h001; 
   
   typedef logic [ApbAddrWidth-1:0] apb_addr_t;
   typedef logic [ApbDataWidth-1:0] apb_data_t;
   typedef logic [ApbStrbWidth-1:0] apb_strb_t;
   typedef logic [RegDataWidth-1:0] reg_data_t;

   localparam apb_addr_t BaseAddr      = 32'h1A10_0000;
   localparam apb_addr_t TestStartAddr = 32'h1A10_0000;
   localparam apb_addr_t TestEndAddr   = 32'h1A10_0030;

   logic [3:0]  clk;
   logic rst_n;
   logic ref_clk;
   
    APB_BUS #(.ADDR_WIDTH(32), .DATA_WIDTH(32)) apb(
                .clk_i(clk[0])
                );
    FLL_BUS fll_intf(
                     .clk_i(clk[0])
                     );
    APB_DV apb_dv(
                  .clk_i(clk[0])
                  );
   
    typedef apb_test::apb_driver #(.ADDR_WIDTH(32), .DATA_WIDTH(32), .TA(SYS_TA), .TT(SYS_TT)) apb_drv_t;
    apb_drv_t apb_master_drv = new(apb_dv);
    `APB_ASSIGN(apb,apb_dv)

    apb_fll_if #(
        .APB_ADDR_WIDTH ( APB_ADDR_WIDTH )
    ) i_apb_fll_if (
        .clk_i     ( clk[0]   ),
        .rst_ni    ( rst_n    ),
        .apb       ( apb      ),
        .fll_intf  ( fll_intf )
    );

    gf22_FLL #(
        // Clock & reset
        .OUTCLK(clk), // FLL clock outputs
        .REFCLK(ref_clk), // Reference clock input
        .RSTB(rst_n),   // Asynchronous reset (active low)
        .CFGREQ(fll_intf.req), // CFG I/F access request (active high)
        .CFGACK(fll_inft.ack), // CFG I/F access granted (active high)
        .CFGAD(fll_intf.addr),  // CFG I/F address bus
        .CFGD(fll_intf.wdata),   // CFG I/F input data bus (write)
        .CFGQ(fll_intf.rdata),   // CFG I/F output data bus (read)
        .CFGWEB(fll_intf.web), // CFG I/F write enable (active low)
        .PWD(1'b0),    // Asynchronous power down (active high)
        .RET(1'b0),    // Asynchronous retention/isolation control (active high)
        .TM(),     // Test mode (active high)
        .TE(),     // Scan enable (active high)
        .TD(),     // Scan data input for chain 1:4
        .TQ(),     // Scan data output for chain 1:4
        .JTD(),    // Scan data in 5
        .JTQ()     // Scan data out 5
        );

   
    initial begin
        ref_clk = 1'b0;
        rst_n = 1'b0;
        repeat (8)
            #(RTC_CLOCK_PERIOD/2) ref_clk = 1'b0;

        rst_n = 1'b1;
        forever
            #(RTC_CLOCK_PERIOD/2) ref_clk = ~ref_clk;
    end

  initial begin : proc_apb_master
    automatic apb_addr_t addr;
    automatic apb_data_t data;
    automatic apb_strb_t strb;
    automatic logic      resp;
    automatic bit        w;

    done <= 1'b0;

    // reset dut
    @(posedge rst_n);
    apb_master.reset_master();
    repeat (10) @(posedge clk[0]);

    // Step 0 : set clk[0]
    data = 'h10030A73;
    addr = 'hC;
    apb_master.read(addr, data, resp);
    $display("Read to addr: %0h. Data: %0h. Resp: %0h", addr,  data, resp);
    repeat ($urandom_range(0,5)) @(posedge clk[0]);
    apb_master.write(addr, data, resp);
    $display("Write to addr: %0h. Data: %0h. Resp: %0h", addr,  data, resp);
    repeat ($urandom_range(0,5)) @(posedge clk[0]);

    // Step 1
    for (int unsigned i = TestStartAddr; i < TestEndAddr; i=i+4) begin
      addr = apb_addr_t'(i);
      apb_master.read(addr, data, resp);
      $display("Read to addr: %0h. Data: %0h. Resp: %0h", addr,  data, resp);
      repeat ($urandom_range(0,5)) @(posedge clk[0]);
    end

    for (int unsigned i = TestStartAddr+'hB; i < TestEndAddr; i=i+4) begin
      addr = apb_addr_t'(i);
      data = 'h10030A73;
      apb_master.write(addr, data, resp);
      $display("Write to addr: %0h. Data: %0h. Resp: %0h", addr,  data, resp);
      repeat ($urandom_range(0,5)) @(posedge clk[0]);
      apb_master.read(addr, data, resp);
      $display("Read to addr: %0h. Data: %0h. Resp: %0h", addr,  data, resp);
      repeat ($urandom_range(0,5)) @(posedge clk[0]);
    end


    done <= 1'b1;
  end

endmodule
