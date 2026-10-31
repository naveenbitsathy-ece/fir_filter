
`timescale 1ns / 1ps

module fir_filter #(
    parameter N = 4,
    parameter DATA_WIDTH = 16,
    parameter COEFF_WIDTH = 16,
    parameter ACC_WIDTH = 40,
    parameter COEFF_FRAC_BITS = 15,
    parameter ADDR_WIDTH = (N <= 1) ? 1 : $clog2(N)
)(
    input wire clk,
    input wire rst_n,

    // Input sample interface
    input wire signed [DATA_WIDTH-1:0] in_data,
    input wire in_valid,
    output wire in_ready,

    // Coefficient loading interface
    input wire coeff_we,
    input wire [ADDR_WIDTH-1:0] coeff_addr,
    input wire signed [COEFF_WIDTH-1:0] coeff_data,

    // Output interface
    output reg signed [DATA_WIDTH-1:0] out_data,
    output reg out_valid
);

    // -----------------------------------------------------
    // LOCAL PARAMETERS
    // -----------------------------------------------------

    localparam PRODUCT_WIDTH = DATA_WIDTH + COEFF_WIDTH;

    localparam IDLE = 2'b00;
    localparam CALC = 2'b01;
    localparam RESP = 2'b10;

    // -----------------------------------------------------
    // INTERNAL REGISTERS
    // -----------------------------------------------------

    reg [1:0] state;

    reg signed [DATA_WIDTH-1:0] sample_reg;

    // Previous input samples
    reg signed [DATA_WIDTH-1:0] delay_line [0:N-1];

    // FIR coefficients
    reg signed [COEFF_WIDTH-1:0] coeff_mem [0:N-1];

    // MAC signals
    reg signed [DATA_WIDTH-1:0] tap_sample;
    reg signed [PRODUCT_WIDTH-1:0] product;
    reg signed [ACC_WIDTH-1:0] acc_next;

    integer i;
    integer j;

    // -----------------------------------------------------
    // INPUT READY
    // -----------------------------------------------------

    // Accept samples only when idle and not loading coefficients
    assign in_ready = (state == IDLE) && !coeff_we;

    // -----------------------------------------------------
    // COMBINATIONAL FIR MULTIPLY-ACCUMULATE
    // -----------------------------------------------------

    always @(*) begin

        acc_next   = {ACC_WIDTH{1'b0}};
        tap_sample = {DATA_WIDTH{1'b0}};
        product    = {PRODUCT_WIDTH{1'b0}};

        for (i = 0; i < N; i = i + 1) begin

            if (i == 0)
                tap_sample = sample_reg;
            else
                tap_sample = delay_line[i-1];

            // Signed multiplication
            product = tap_sample * coeff_mem[i];

            // Sign extend product before accumulation
            acc_next = acc_next +
                {{(ACC_WIDTH-PRODUCT_WIDTH)
                    {product[PRODUCT_WIDTH-1]}},
                    product};

        end
    end

    // -----------------------------------------------------
    // SEQUENTIAL FSM
    // -----------------------------------------------------

    always @(posedge clk or negedge rst_n) begin

        if (!rst_n) begin

            state      <= IDLE;
            sample_reg <= 0;
            out_data   <= 0;
            out_valid  <= 0;

            for (j = 0; j < N; j = j + 1) begin
                delay_line[j] <= 0;
                coeff_mem[j] <= 0;
            end

        end
        else begin

            case (state)

                // =========================================
                // IDLE: COEFFICIENT LOADING OR SAMPLE INPUT
                // =========================================

                IDLE: begin

                    out_valid <= 1'b0;

                    // Load one coefficient
                    if (coeff_we && (coeff_addr < N)) begin

                        coeff_mem[coeff_addr] <= coeff_data;

                    end

                    // Accept exactly one input sample
                    else if (in_valid && in_ready) begin

                        sample_reg <= in_data;

                        state <= CALC;

                    end

                end


                // =========================================
                // CALC: COMPUTE ONE FIR OUTPUT
                // =========================================

                CALC: begin

                    // Sum all N products and scale Q15
                    out_data <= acc_next >>> COEFF_FRAC_BITS;

                    // Shift sample history exactly once
                    for (j = N-1; j > 0; j = j-1) begin
                        delay_line[j] <= delay_line[j-1];
                    end

                    delay_line[0] <= sample_reg;

                    // Notify PS that result is ready
                    out_valid <= 1'b1;

                    state <= RESP;

                end


                // =========================================
                // RESP: WAIT FOR PS ACKNOWLEDGEMENT
                // =========================================

                RESP: begin

                    // Keep out_valid HIGH until PS lowers in_valid
                    out_valid <= 1'b1;

                    if (!in_valid) begin

                        out_valid <= 1'b0;

                        state <= IDLE;

                    end

                end


                // =========================================
                // DEFAULT RECOVERY
                // =========================================

                default: begin

                    state <= IDLE;
                    out_valid <= 1'b0;

                end

            endcase

        end

    end

endmodule
