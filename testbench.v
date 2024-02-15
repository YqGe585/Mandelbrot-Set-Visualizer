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
    
    // Test Case 1: Simple scenario
    ci = 27'd5452595; // 4.23 fixed point representation
    cr = 27'd167772; // 4.23 fixed point representation
    max_iterations = 15'd1000;
    
    // Wait for the test case to complete
    #10;
    
    // Add more test cases as needed
    // Test Case 2, Test Case 3, etc.
    
end

// Clock generation
always #5 clk = ~clk; // Generate a clock with period 10ns

endmodule

