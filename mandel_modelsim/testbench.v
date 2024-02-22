`timescale 1ps / 1ps

module mandelbrot_iterate_tb;

// Inputs
reg signed [26:0] ci, cr;
reg [15:0] max_iterations;
reg clk;
reg reset;

// Outputs
wire [15:0] iterations;

// Instantiate the Unit Under Test (UUT)
mandelbrot_iterate uut (
    .ci(ci), 
    .cr(cr), 
    .max_iterations(max_iterations), 
    .iterations(iterations), 
    .clk(clk), 
    .reset(reset)
);

task perform_test;
    input signed [26:0] test_ci, test_cr;
    input [9:0] expected_iterations;
    begin
        reset = 1; 
        #10;
        ci = test_ci;
        cr = test_cr;
        max_iterations = 10'd1000;
        reset = 0; 
        #10;
        #4000;

        if (iterations == expected_iterations) begin
            $display("Test passed for ci=%d, cr=%d. Iterations: %d", ci, cr, iterations);
        end else begin
            $display("Test failed for ci=%d, cr=%d. Expected iterations: %d, got: %d", ci, cr, expected_iterations, iterations);
        end
    end
endtask

initial begin
    // Initialize Inputs
    ci = 0;
    cr = 0;
    max_iterations = 15'd1000;
    clk = 0;
    reset=0;
    #100
    reset = 1;

    // Wait 100 ns for global reset to finish
    #100;
    reset = 0; // Release reset
    #10;
    
    perform_test(27'd126164665, 27'd133462754, 10'd20);
    perform_test(27'd127087412, 27'd132120576, 10'd40);
    perform_test(27'd128681247, 27'd1006632, 10'd60);
    perform_test(27'd127506842, 27'd134049956, 10'd80);
    perform_test(27'd128681247, 27'd83886, 10'd100);

    perform_test(27'd127372624, 27'd132355458, 10'd150);
    perform_test(27'd129452999, 27'd1392508, 10'd500);
    perform_test(27'd130199585, 27'd2264924, 10'd800);    

    perform_test(27'd838860, 27'd677721, 10'd100);
    perform_test(27'd838860, 27'd133378868, 10'd100);
end

// Clock generation
always #1 clk = ~clk; // Generate a clock with period 10ns

endmodule

