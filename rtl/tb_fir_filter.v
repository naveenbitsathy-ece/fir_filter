
`timescale 1ns/1ps

module tb_fir_filter;

    parameter N             = 4;
    parameter DATA_WIDTH    = 16;
    parameter COEFF_WIDTH   = 16;
    parameter ACC_WIDTH     = 40;
    parameter COEFF_FRAC_BITS = 15;
    parameter ADDR_WIDTH    = 2;

    reg clk;
    reg rst_n;

    reg signed [DATA_WIDTH-1:0] in_data;
    reg in_valid;
    wire in_ready;

    reg coeff_we;
    reg [ADDR_WIDTH-1:0] coeff_addr;
    reg signed [COEFF_WIDTH-1:0] coeff_data;

    wire signed [DATA_WIDTH-1:0] out_data;
    wire out_valid;

    integer errors;
    integer sample_count;

    // Expected outputs for impulse response
    reg signed [DATA_WIDTH-1:0] expected [0:5];

    // DUT instantiation
    fir_filter #(
        .N(N),
        .DATA_WIDTH(DATA_WIDTH),
        .COEFF_WIDTH(COEFF_WIDTH),
        .ACC_WIDTH(ACC_WIDTH),
        .COEFF_FRAC_BITS(COEFF_FRAC_BITS),
        .ADDR_WIDTH(ADDR_WIDTH)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),

        .in_data(in_data),
        .in_valid(in_valid),
        .in_ready(in_ready),

        .coeff_we(coeff_we),
        .coeff_addr(coeff_addr),
        .coeff_data(coeff_data),

        .out_data(out_data),
        .out_valid(out_valid)
    );

    // Clock generation: 100 MHz
    always #5 clk = ~clk;

    // Load FIR coefficient
    task load_coefficient;
        input [ADDR_WIDTH-1:0] address;
        input signed [COEFF_WIDTH-1:0] value;
        begin
            @(negedge clk);
            coeff_we   = 1'b1;
            coeff_addr = address;
            coeff_data = value;

            @(negedge clk);
            coeff_we = 1'b0;
        end
    endtask

    // Apply input sample and check output
    task send_sample;
        input signed [DATA_WIDTH-1:0] sample;
        input signed [DATA_WIDTH-1:0] reference;
        begin
            @(negedge clk);
            in_data  = sample;
            in_valid = 1'b1;

            @(posedge clk);
            #1;

            if (out_valid) begin
                if (out_data === reference) begin
                    $display("PASS | Sample %0d | Input = %0d | Output = %0d",
                             sample_count, sample, out_data);
                end
                else begin
                    $display("FAIL | Sample %0d | Expected = %0d | Got = %0d",
                             sample_count, reference, out_data);
                    errors = errors + 1;
                end
            end
            else begin
                $display("FAIL | Sample %0d | out_valid not asserted",
                         sample_count);
                errors = errors + 1;
            end

            sample_count = sample_count + 1;

            @(negedge clk);
            in_valid = 1'b0;
        end
    endtask

    initial begin

        $dumpfile("fir_waveform.vcd");
        $dumpvars(0, tb_fir_filter);

        clk          = 0;
        rst_n        = 0;
        in_data      = 0;
        in_valid     = 0;
        coeff_we     = 0;
        coeff_addr   = 0;
        coeff_data   = 0;
        errors       = 0;
        sample_count = 0;

        // Expected impulse response:
        // Coefficients: 1362, 15022, 15022, 1362
        // Input impulse: 16384
        expected[0] = 681;
        expected[1] = 7511;
        expected[2] = 7511;
        expected[3] = 681;
        expected[4] = 0;
        expected[5] = 0;

        // Reset
        repeat(3) @(negedge clk);
        rst_n = 1'b1;

        // Load Q15 FIR coefficients
        $display("\nLoading FIR coefficients...");

        load_coefficient(0, 16'sd1362);
        load_coefficient(1, 16'sd15022);
        load_coefficient(2, 16'sd15022);
        load_coefficient(3, 16'sd1362);

        $display("\nStarting FIR impulse test...\n");

        // Apply impulse followed by zeros
        send_sample(16'sd16384, expected[0]);
        send_sample(16'sd0,     expected[1]);
        send_sample(16'sd0,     expected[2]);
        send_sample(16'sd0,     expected[3]);
        send_sample(16'sd0,     expected[4]);
        send_sample(16'sd0,     expected[5]);

        // Simulation summary
        $display("\n================================");
        $display("       FIR SIMULATION REPORT");
        $display("================================");
        $display("Total samples tested : %0d", sample_count);
        $display("Total errors         : %0d", errors);

        if (errors == 0)
            $display("RESULT: ALL TESTS PASSED");
        else
            $display("RESULT: TEST FAILED");

        $display("================================\n");

        $finish;
    end

endmodule
