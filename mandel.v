module mandelbrot_iterate(
    input signed [26:0] ci, cr,
    input [15:0] max_iterations,
    output reg [15:0] iterations,
    input clk,
    input reset
);

reg signed [26:0] zi, zr;
reg signed [26:0] zi_temp, zr_temp;
wire signed [26:0] zi_squared, zr_squared, zrmulzi;
reg signed [26:0] z_sum;

signed_mult inst1(zi_squared, zi, zi);
signed_mult inst2(zr_squared, zr, zr);
signed_mult inst3(zrmulzi, zr, zi);

always @(posedge clk) begin
    if (reset) begin
        iterations <= 0;
        zi <= 0;
        zr <= 0;
    end 
    else begin
        // zi_squared <= zi * zi;
        // zr_squared <= zr * zr;

        //z_sum <= zr_squared + zi_squared;

        if (iterations < max_iterations && (zr_squared + zi_squared) <= (4 << 23)) begin
            zr <= zr_squared - zi_squared + cr;
            zi <= (zrmulzi << 1) + ci;

            // zi <= zi_temp;
            // zr <= zr_temp;

            iterations <= iterations + 1;
        end
    end
end

endmodule


module signed_mult (out, a, b);
	output 	signed  [26:0]	out;
	input 	signed	[26:0] 	a;
	input 	signed	[26:0] 	b;
	// intermediate full bit length
	wire 	signed	[53:0]	mult_out;
	assign mult_out = a * b;
	// select bits for 7.20 fixed point
	assign out = {mult_out[53], mult_out[48:23]};
endmodule