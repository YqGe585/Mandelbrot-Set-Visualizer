// (C) 2001-2015 Altera Corporation. All rights reserved.
// Your use of Altera Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Altera Program License Subscription 
// Agreement, Altera MegaCore Function License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Altera and sold by 
// Altera or its authorized distributors.  Please refer to the applicable 
// agreement for further details.


// Copyright (C) 1991-2010 Altera Corporation
//  Your use of Altera Corporation's design tools, logic functions 
//  and other software and tools, and its AMPP partner logic 
//  functions, and any output files from any of the foregoing 
//  (including device programming or simulation files), and any 
//  associated documentation or information are expressly subject 
//  to the terms and conditions of the Altera Program License 
//  Subscription Agreement, Altera MegaCore Function License 
//  Agreement, or other applicable license agreement, including, 
//  without limitation, that your use is for the sole purpose of 
//  programming logic devices manufactured by Altera and sold by 
//  Altera or its authorized distributors.  Please refer to the 
//  applicable agreement for further details.



//synthesis_resources = lpm_add_sub 4 lpm_mult 1 reg 254 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_mult_single
	( 
	aclr,
	clk_en,
	clock,
	dataa,
	datab,
	result) /* synthesis synthesis_clearbox=1 */;
	input   aclr;
	input   clk_en;
	input   clock;
	input   [31:0]  dataa;
	input   [31:0]  datab;
	output   [31:0]  result;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_off
`endif
	tri0   aclr;
	tri1   clk_en;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_on
`endif

	reg	dataa_exp_all_one_ff_p1;
	reg	dataa_exp_not_zero_ff_p1;
	reg	dataa_man_not_zero_ff_p1;
	reg	dataa_man_not_zero_ff_p2;
	reg	datab_exp_all_one_ff_p1;
	reg	datab_exp_not_zero_ff_p1;
	reg	datab_man_not_zero_ff_p1;
	reg	datab_man_not_zero_ff_p2;
	reg	[9:0]	delay_exp2_bias;
	reg	[9:0]	delay_exp3_bias;
	reg	[9:0]	delay_exp_bias;
	reg	delay_man_product_msb;
	reg	delay_man_product_msb2;
	reg	delay_man_product_msb_p0;
	reg	[23:0]	delay_round;
	reg	[8:0]	exp_add_p1;
	reg	[9:0]	exp_adj_p1;
	reg	[9:0]	exp_adj_p2;
	reg	[8:0]	exp_bias_p1;
	reg	[8:0]	exp_bias_p2;
	reg	[7:0]	exp_result_ff;
	reg	input_is_infinity_dffe_0;
	reg	input_is_infinity_dffe_1;
	reg	input_is_infinity_dffe_2;
	reg	input_is_infinity_dffe_3;
	reg	input_is_infinity_ff1;
	reg	input_is_infinity_ff2;
	reg	input_is_infinity_ff3;
	reg	input_is_infinity_ff4;
	reg	input_is_nan_dffe_0;
	reg	input_is_nan_dffe_1;
	reg	input_is_nan_dffe_2;
	reg	input_is_nan_dffe_3;
	reg	input_is_nan_ff1;
	reg	input_is_nan_ff2;
	reg	input_is_nan_ff3;
	reg	input_is_nan_ff4;
	reg	input_not_zero_dffe_0;
	reg	input_not_zero_dffe_1;
	reg	input_not_zero_dffe_2;
	reg	input_not_zero_dffe_3;
	reg	input_not_zero_ff1;
	reg	input_not_zero_ff2;
	reg	input_not_zero_ff3;
	reg	input_not_zero_ff4;
	reg	lsb_dffe;
	reg	[22:0]	man_result_ff;
	reg	man_round_carry_p0;
	reg	[23:0]	man_round_p;
	reg	[23:0]	man_round_p0;
	reg	[24:0]	man_round_p2;
	reg	round_dffe;
	reg	[0:0]	sign_node_ff0;
	reg	[0:0]	sign_node_ff1;
	reg	[0:0]	sign_node_ff2;
	reg	[0:0]	sign_node_ff3;
	reg	[0:0]	sign_node_ff4;
	reg	[0:0]	sign_node_ff5;
	reg	[0:0]	sign_node_ff6;
	reg	[0:0]	sign_node_ff7;
	reg	[0:0]	sign_node_ff8;
	reg	[0:0]	sign_node_ff9;
	reg	sticky_dffe;
	wire  [8:0]   wire_exp_add_adder_result;
	wire  [9:0]   wire_exp_adj_adder_result;
	wire  [9:0]   wire_exp_bias_subtr_result;
	wire  [24:0]   wire_man_round_adder_result;
	wire  [47:0]   wire_man_product2_mult_result;
	wire  [9:0]  bias;
	wire  [7:0]  dataa_exp_all_one;
	wire  [7:0]  dataa_exp_not_zero;
	wire  [22:0]  dataa_man_not_zero;
	wire  [7:0]  datab_exp_all_one;
	wire  [7:0]  datab_exp_not_zero;
	wire  [22:0]  datab_man_not_zero;
	wire  exp_is_inf;
	wire  exp_is_zero;
	wire  [9:0]  expmod;
	wire  [7:0]  inf_num;
	wire  lsb_bit;
	wire  [24:0]  man_shift_full;
	wire  [7:0]  result_exp_all_one;
	wire  [8:0]  result_exp_not_zero;
	wire  round_bit;
	wire  round_carry;
	wire  [22:0]  sticky_bit;

	// synopsys translate_off
	initial
		dataa_exp_all_one_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_exp_all_one_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_exp_all_one_ff_p1 <= dataa_exp_all_one[7];
	// synopsys translate_off
	initial
		dataa_exp_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_exp_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_exp_not_zero_ff_p1 <= dataa_exp_not_zero[7];
	// synopsys translate_off
	initial
		dataa_man_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_man_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_man_not_zero_ff_p1 <= dataa_man_not_zero[10];
	// synopsys translate_off
	initial
		dataa_man_not_zero_ff_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_man_not_zero_ff_p2 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_man_not_zero_ff_p2 <= dataa_man_not_zero[22];
	// synopsys translate_off
	initial
		datab_exp_all_one_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_exp_all_one_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_exp_all_one_ff_p1 <= datab_exp_all_one[7];
	// synopsys translate_off
	initial
		datab_exp_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_exp_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_exp_not_zero_ff_p1 <= datab_exp_not_zero[7];
	// synopsys translate_off
	initial
		datab_man_not_zero_ff_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_man_not_zero_ff_p1 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_man_not_zero_ff_p1 <= datab_man_not_zero[10];
	// synopsys translate_off
	initial
		datab_man_not_zero_ff_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_man_not_zero_ff_p2 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_man_not_zero_ff_p2 <= datab_man_not_zero[22];
	// synopsys translate_off
	initial
		delay_exp2_bias = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_exp2_bias <= 10'b0;
		else if  (clk_en == 1'b1)   delay_exp2_bias <= delay_exp_bias;
	// synopsys translate_off
	initial
		delay_exp3_bias = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_exp3_bias <= 10'b0;
		else if  (clk_en == 1'b1)   delay_exp3_bias <= delay_exp2_bias;
	// synopsys translate_off
	initial
		delay_exp_bias = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_exp_bias <= 10'b0;
		else if  (clk_en == 1'b1)   delay_exp_bias <= wire_exp_bias_subtr_result;
	// synopsys translate_off
	initial
		delay_man_product_msb = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb <= delay_man_product_msb_p0;
	// synopsys translate_off
	initial
		delay_man_product_msb2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb2 <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb2 <= delay_man_product_msb;
	// synopsys translate_off
	initial
		delay_man_product_msb_p0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_man_product_msb_p0 <= 1'b0;
		else if  (clk_en == 1'b1)   delay_man_product_msb_p0 <= wire_man_product2_mult_result[47];
	// synopsys translate_off
	initial
		delay_round = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) delay_round <= 24'b0;
		else if  (clk_en == 1'b1)   delay_round <= ((man_round_p2[23:0] & {24{(~ man_round_p2[24])}}) | (man_round_p2[24:1] & {24{man_round_p2[24]}}));
	// synopsys translate_off
	initial
		exp_add_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_add_p1 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_add_p1 <= wire_exp_add_adder_result;
	// synopsys translate_off
	initial
		exp_adj_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_adj_p1 <= 10'b0;
		else if  (clk_en == 1'b1)   exp_adj_p1 <= delay_exp3_bias;
	// synopsys translate_off
	initial
		exp_adj_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_adj_p2 <= 10'b0;
		else if  (clk_en == 1'b1)   exp_adj_p2 <= wire_exp_adj_adder_result;
	// synopsys translate_off
	initial
		exp_bias_p1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_bias_p1 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_bias_p1 <= exp_add_p1[8:0];
	// synopsys translate_off
	initial
		exp_bias_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_bias_p2 <= 9'b0;
		else if  (clk_en == 1'b1)   exp_bias_p2 <= exp_bias_p1;
	// synopsys translate_off
	initial
		exp_result_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_result_ff <= 8'b0;
		else if  (clk_en == 1'b1)   exp_result_ff <= ((inf_num & {8{((exp_is_inf | input_is_infinity_ff4) | input_is_nan_ff4)}}) | ((exp_adj_p2[7:0] & {8{(~ exp_is_zero)}}) & {8{input_not_zero_ff4}}));
	// synopsys translate_off
	initial
		input_is_infinity_dffe_0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_0 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_0 <= ((dataa_exp_all_one_ff_p1 & (~ (dataa_man_not_zero_ff_p1 | dataa_man_not_zero_ff_p2))) | (datab_exp_all_one_ff_p1 & (~ (datab_man_not_zero_ff_p1 | datab_man_not_zero_ff_p2))));
	// synopsys translate_off
	initial
		input_is_infinity_dffe_1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_1 <= input_is_infinity_dffe_0;
	// synopsys translate_off
	initial
		input_is_infinity_dffe_2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_2 <= input_is_infinity_dffe_1;
	// synopsys translate_off
	initial
		input_is_infinity_dffe_3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_dffe_3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_dffe_3 <= input_is_infinity_dffe_2;
	// synopsys translate_off
	initial
		input_is_infinity_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff1 <= input_is_infinity_dffe_3;
	// synopsys translate_off
	initial
		input_is_infinity_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff2 <= input_is_infinity_ff1;
	// synopsys translate_off
	initial
		input_is_infinity_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff3 <= input_is_infinity_ff2;
	// synopsys translate_off
	initial
		input_is_infinity_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinity_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinity_ff4 <= input_is_infinity_ff3;
	// synopsys translate_off
	initial
		input_is_nan_dffe_0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_0 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_0 <= ((dataa_exp_all_one_ff_p1 & (dataa_man_not_zero_ff_p1 | dataa_man_not_zero_ff_p2)) | (datab_exp_all_one_ff_p1 & (datab_man_not_zero_ff_p1 | datab_man_not_zero_ff_p2)));
	// synopsys translate_off
	initial
		input_is_nan_dffe_1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_1 <= input_is_nan_dffe_0;
	// synopsys translate_off
	initial
		input_is_nan_dffe_2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_2 <= input_is_nan_dffe_1;
	// synopsys translate_off
	initial
		input_is_nan_dffe_3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe_3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe_3 <= input_is_nan_dffe_2;
	// synopsys translate_off
	initial
		input_is_nan_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff1 <= input_is_nan_dffe_3;
	// synopsys translate_off
	initial
		input_is_nan_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff2 <= input_is_nan_ff1;
	// synopsys translate_off
	initial
		input_is_nan_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff3 <= input_is_nan_ff2;
	// synopsys translate_off
	initial
		input_is_nan_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_ff4 <= input_is_nan_ff3;
	// synopsys translate_off
	initial
		input_not_zero_dffe_0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_0 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_0 <= (dataa_exp_not_zero_ff_p1 & datab_exp_not_zero_ff_p1);
	// synopsys translate_off
	initial
		input_not_zero_dffe_1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_1 <= input_not_zero_dffe_0;
	// synopsys translate_off
	initial
		input_not_zero_dffe_2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_2 <= input_not_zero_dffe_1;
	// synopsys translate_off
	initial
		input_not_zero_dffe_3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_dffe_3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_dffe_3 <= input_not_zero_dffe_2;
	// synopsys translate_off
	initial
		input_not_zero_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff1 <= input_not_zero_dffe_3;
	// synopsys translate_off
	initial
		input_not_zero_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff2 <= input_not_zero_ff1;
	// synopsys translate_off
	initial
		input_not_zero_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff3 <= input_not_zero_ff2;
	// synopsys translate_off
	initial
		input_not_zero_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_not_zero_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   input_not_zero_ff4 <= input_not_zero_ff3;
	// synopsys translate_off
	initial
		lsb_dffe = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) lsb_dffe <= 1'b0;
		else if  (clk_en == 1'b1)   lsb_dffe <= lsb_bit;
	// synopsys translate_off
	initial
		man_result_ff = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_result_ff <= 23'b0;
		else if  (clk_en == 1'b1)   man_result_ff <= {((((((delay_round[22] & input_not_zero_ff4) & (~ input_is_infinity_ff4)) & (~ exp_is_inf)) & (~ exp_is_zero)) | (input_is_infinity_ff4 & (~ input_not_zero_ff4))) | input_is_nan_ff4), (((((delay_round[21:0] & {22{input_not_zero_ff4}}) & {22{(~ input_is_infinity_ff4)}}) & {22{(~ exp_is_inf)}}) & {22{(~ exp_is_zero)}}) & {22{(~ input_is_nan_ff4)}})};
	// synopsys translate_off
	initial
		man_round_carry_p0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_carry_p0 <= 1'b0;
		else if  (clk_en == 1'b1)   man_round_carry_p0 <= round_carry;
	// synopsys translate_off
	initial
		man_round_p = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p <= 24'b0;
		else if  (clk_en == 1'b1)   man_round_p <= man_shift_full[24:1];
	// synopsys translate_off
	initial
		man_round_p0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p0 <= 24'b0;
		else if  (clk_en == 1'b1)   man_round_p0 <= man_round_p;
	// synopsys translate_off
	initial
		man_round_p2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_round_p2 <= 25'b0;
		else if  (clk_en == 1'b1)   man_round_p2 <= wire_man_round_adder_result;
	// synopsys translate_off
	initial
		round_dffe = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) round_dffe <= 1'b0;
		else if  (clk_en == 1'b1)   round_dffe <= round_bit;
	// synopsys translate_off
	initial
		sign_node_ff0 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff0 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff0 <= (dataa[31] ^ datab[31]);
	// synopsys translate_off
	initial
		sign_node_ff1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff1 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff1 <= sign_node_ff0[0:0];
	// synopsys translate_off
	initial
		sign_node_ff2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff2 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff2 <= sign_node_ff1[0:0];
	// synopsys translate_off
	initial
		sign_node_ff3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff3 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff3 <= sign_node_ff2[0:0];
	// synopsys translate_off
	initial
		sign_node_ff4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff4 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff4 <= sign_node_ff3[0:0];
	// synopsys translate_off
	initial
		sign_node_ff5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff5 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff5 <= sign_node_ff4[0:0];
	// synopsys translate_off
	initial
		sign_node_ff6 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff6 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff6 <= sign_node_ff5[0:0];
	// synopsys translate_off
	initial
		sign_node_ff7 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff7 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff7 <= sign_node_ff6[0:0];
	// synopsys translate_off
	initial
		sign_node_ff8 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff8 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff8 <= sign_node_ff7[0:0];
	// synopsys translate_off
	initial
		sign_node_ff9 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_node_ff9 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_node_ff9 <= sign_node_ff8[0:0];
	// synopsys translate_off
	initial
		sticky_dffe = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sticky_dffe <= 1'b0;
		else if  (clk_en == 1'b1)   sticky_dffe <= sticky_bit[22];
	lpm_add_sub   exp_add_adder
	( 
	.aclr(aclr),
	.cin(1'b0),
	.clken(clk_en),
	.clock(clock),
	.cout(),
	.dataa({1'b0, dataa[30:23]}),
	.datab({1'b0, datab[30:23]}),
	.overflow(),
	.result(wire_exp_add_adder_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.add_sub(1'b1)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		exp_add_adder.lpm_pipeline = 1,
		exp_add_adder.lpm_width = 9,
		exp_add_adder.lpm_type = "lpm_add_sub";
	lpm_add_sub   exp_adj_adder
	( 
	.cin(1'b0),
	.cout(),
	.dataa(exp_adj_p1),
	.datab({expmod[9:0]}),
	.overflow(),
	.result(wire_exp_adj_adder_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		exp_adj_adder.lpm_pipeline = 0,
		exp_adj_adder.lpm_width = 10,
		exp_adj_adder.lpm_type = "lpm_add_sub";
	lpm_add_sub   exp_bias_subtr
	( 
	.cout(),
	.dataa({1'b0, exp_bias_p2}),
	.datab({bias[9:0]}),
	.overflow(),
	.result(wire_exp_bias_subtr_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.cin(),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		exp_bias_subtr.lpm_direction = "SUB",
		exp_bias_subtr.lpm_pipeline = 0,
		exp_bias_subtr.lpm_representation = "UNSIGNED",
		exp_bias_subtr.lpm_width = 10,
		exp_bias_subtr.lpm_type = "lpm_add_sub";
	lpm_add_sub   man_round_adder
	( 
	.cout(),
	.dataa({1'b0, man_round_p0}),
	.datab({{24{1'b0}}, man_round_carry_p0}),
	.overflow(),
	.result(wire_man_round_adder_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.cin(),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		man_round_adder.lpm_pipeline = 0,
		man_round_adder.lpm_width = 25,
		man_round_adder.lpm_type = "lpm_add_sub";
	lpm_mult   man_product2_mult
	( 
	.aclr(aclr),
	.clken(clk_en),
	.clock(clock),
	.dataa({1'b1, dataa[22:0]}),
	.datab({1'b1, datab[22:0]}),
	.result(wire_man_product2_mult_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.sum({1{1'b0}})
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		man_product2_mult.lpm_pipeline = 5,
		man_product2_mult.lpm_representation = "UNSIGNED",
		man_product2_mult.lpm_widtha = 24,
		man_product2_mult.lpm_widthb = 24,
		man_product2_mult.lpm_widthp = 48,
		man_product2_mult.lpm_widths = 1,
		man_product2_mult.lpm_type = "lpm_mult",
		man_product2_mult.lpm_hint = "DEDICATED_MULTIPLIER_CIRCUITRY=YES";
	assign
		bias = {{3{1'b0}}, {7{1'b1}}},
		dataa_exp_all_one = {(dataa[30] & dataa_exp_all_one[6]), (dataa[29] & dataa_exp_all_one[5]), (dataa[28] & dataa_exp_all_one[4]), (dataa[27] & dataa_exp_all_one[3]), (dataa[26] & dataa_exp_all_one[2]), (dataa[25] & dataa_exp_all_one[1]), (dataa[24] & dataa_exp_all_one[0]), dataa[23]},
		dataa_exp_not_zero = {(dataa[30] | dataa_exp_not_zero[6]), (dataa[29] | dataa_exp_not_zero[5]), (dataa[28] | dataa_exp_not_zero[4]), (dataa[27] | dataa_exp_not_zero[3]), (dataa[26] | dataa_exp_not_zero[2]), (dataa[25] | dataa_exp_not_zero[1]), (dataa[24] | dataa_exp_not_zero[0]), dataa[23]},
		dataa_man_not_zero = {(dataa[22] | dataa_man_not_zero[21]), (dataa[21] | dataa_man_not_zero[20]), (dataa[20] | dataa_man_not_zero[19]), (dataa[19] | dataa_man_not_zero[18]), (dataa[18] | dataa_man_not_zero[17]), (dataa[17] | dataa_man_not_zero[16]), (dataa[16] | dataa_man_not_zero[15]), (dataa[15] | dataa_man_not_zero[14]), (dataa[14] | dataa_man_not_zero[13]), (dataa[13] | dataa_man_not_zero[12]), (dataa[12] | dataa_man_not_zero[11]), dataa[11], (dataa[10] | dataa_man_not_zero[9]), (dataa[9] | dataa_man_not_zero[8]), (dataa[8] | dataa_man_not_zero[7]), (dataa[7] | dataa_man_not_zero[6]), (dataa[6] | dataa_man_not_zero[5]), (dataa[5] | dataa_man_not_zero[4]), (dataa[4] | dataa_man_not_zero[3]), (dataa[3] | dataa_man_not_zero[2]), (dataa[2] | dataa_man_not_zero[1]), (dataa[1] | dataa_man_not_zero[0]), dataa[0]},
		datab_exp_all_one = {(datab[30] & datab_exp_all_one[6]), (datab[29] & datab_exp_all_one[5]), (datab[28] & datab_exp_all_one[4]), (datab[27] & datab_exp_all_one[3]), (datab[26] & datab_exp_all_one[2]), (datab[25] & datab_exp_all_one[1]), (datab[24] & datab_exp_all_one[0]), datab[23]},
		datab_exp_not_zero = {(datab[30] | datab_exp_not_zero[6]), (datab[29] | datab_exp_not_zero[5]), (datab[28] | datab_exp_not_zero[4]), (datab[27] | datab_exp_not_zero[3]), (datab[26] | datab_exp_not_zero[2]), (datab[25] | datab_exp_not_zero[1]), (datab[24] | datab_exp_not_zero[0]), datab[23]},
		datab_man_not_zero = {(datab[22] | datab_man_not_zero[21]), (datab[21] | datab_man_not_zero[20]), (datab[20] | datab_man_not_zero[19]), (datab[19] | datab_man_not_zero[18]), (datab[18] | datab_man_not_zero[17]), (datab[17] | datab_man_not_zero[16]), (datab[16] | datab_man_not_zero[15]), (datab[15] | datab_man_not_zero[14]), (datab[14] | datab_man_not_zero[13]), (datab[13] | datab_man_not_zero[12]), (datab[12] | datab_man_not_zero[11]), datab[11], (datab[10] | datab_man_not_zero[9]), (datab[9] | datab_man_not_zero[8]), (datab[8] | datab_man_not_zero[7]), (datab[7] | datab_man_not_zero[6]), (datab[6] | datab_man_not_zero[5]), (datab[5] | datab_man_not_zero[4]), (datab[4] | datab_man_not_zero[3]), (datab[3] | datab_man_not_zero[2]), (datab[2] | datab_man_not_zero[1]), (datab[1] | datab_man_not_zero[0]), datab[0]},
		exp_is_inf = (((~ exp_adj_p2[9]) & exp_adj_p2[8]) | ((~ exp_adj_p2[8]) & result_exp_all_one[7])),
		exp_is_zero = (exp_adj_p2[9] | (~ result_exp_not_zero[8])),
		expmod = {{8{1'b0}}, (delay_man_product_msb2 & man_round_p2[24]), (delay_man_product_msb2 ^ man_round_p2[24])},
		inf_num = {8{1'b1}},
		lsb_bit = man_shift_full[1],
		man_shift_full = ((wire_man_product2_mult_result[46:22] & {25{(~ wire_man_product2_mult_result[47])}}) | (wire_man_product2_mult_result[47:23] & {25{wire_man_product2_mult_result[47]}})),
		result = {sign_node_ff9[0:0], exp_result_ff[7:0], man_result_ff[22:0]},
		result_exp_all_one = {(result_exp_all_one[6] & exp_adj_p2[7]), (result_exp_all_one[5] & exp_adj_p2[6]), (result_exp_all_one[4] & exp_adj_p2[5]), (result_exp_all_one[3] & exp_adj_p2[4]), (result_exp_all_one[2] & exp_adj_p2[3]), (result_exp_all_one[1] & exp_adj_p2[2]), (result_exp_all_one[0] & exp_adj_p2[1]), exp_adj_p2[0]},
		result_exp_not_zero = {(result_exp_not_zero[7] | exp_adj_p2[8]), (result_exp_not_zero[6] | exp_adj_p2[7]), (result_exp_not_zero[5] | exp_adj_p2[6]), (result_exp_not_zero[4] | exp_adj_p2[5]), (result_exp_not_zero[3] | exp_adj_p2[4]), (result_exp_not_zero[2] | exp_adj_p2[3]), (result_exp_not_zero[1] | exp_adj_p2[2]), (result_exp_not_zero[0] | exp_adj_p2[1]), exp_adj_p2[0]},
		round_bit = man_shift_full[0],
		round_carry = (round_dffe & (lsb_dffe | sticky_dffe)),
		sticky_bit = {(sticky_bit[21] | (wire_man_product2_mult_result[47] & wire_man_product2_mult_result[22])), (sticky_bit[20] | wire_man_product2_mult_result[21]), (sticky_bit[19] | wire_man_product2_mult_result[20]), (sticky_bit[18] | wire_man_product2_mult_result[19]), (sticky_bit[17] | wire_man_product2_mult_result[18]), (sticky_bit[16] | wire_man_product2_mult_result[17]), (sticky_bit[15] | wire_man_product2_mult_result[16]), (sticky_bit[14] | wire_man_product2_mult_result[15]), (sticky_bit[13] | wire_man_product2_mult_result[14]), (sticky_bit[12] | wire_man_product2_mult_result[13]), (sticky_bit[11] | wire_man_product2_mult_result[12]), (sticky_bit[10] | wire_man_product2_mult_result[11]), (sticky_bit[9] | wire_man_product2_mult_result[10]), (sticky_bit[8] | wire_man_product2_mult_result[9]), (sticky_bit[7] | wire_man_product2_mult_result[8]), (sticky_bit[6] | wire_man_product2_mult_result[7]), (sticky_bit[5] | wire_man_product2_mult_result[6]), (sticky_bit[4] | wire_man_product2_mult_result[5]), (sticky_bit[3] | wire_man_product2_mult_result[4]), (sticky_bit[2] | wire_man_product2_mult_result[3]), (sticky_bit[1] | wire_man_product2_mult_result[2]), (sticky_bit[0] | wire_man_product2_mult_result[1]), wire_man_product2_mult_result[0]};
endmodule //fpoint_qsys_mult_single
//VALID FILE

//altfp_add_sub CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" DENORMAL_SUPPORT="NO" DEVICE_FAMILY="STRATIXIV" DIRECTION="VARIABLE" EXCEPTION_HANDLING="NO" PIPELINE=8 REDUCED_FUNCTIONALITY="NO" SPEED_OPTIMIZED="YES" WIDTH_EXP=8 WIDTH_MAN=23 aclr add_sub clk_en clock dataa datab result
//VERSION_BEGIN 10.1 cbx_altbarrel_shift 2010:09:06:21:07:24:PN cbx_altfp_add_sub 2010:09:06:21:07:24:PN cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_cycloneii 2010:09:06:21:07:25:PN cbx_lpm_add_sub 2010:09:06:21:07:25:PN cbx_lpm_compare 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN cbx_stratix 2010:09:06:21:07:25:PN cbx_stratixii 2010:09:06:21:07:25:PN  VERSION_END
// synthesis VERILOG_INPUT_VERSION VERILOG_2001
// altera message_off 10463



// Copyright (C) 1991-2010 Altera Corporation
//  Your use of Altera Corporation's design tools, logic functions 
//  and other software and tools, and its AMPP partner logic 
//  functions, and any output files from any of the foregoing 
//  (including device programming or simulation files), and any 
//  associated documentation or information are expressly subject 
//  to the terms and conditions of the Altera Program License 
//  Subscription Agreement, Altera MegaCore Function License 
//  Agreement, or other applicable license agreement, including, 
//  without limitation, that your use is for the sole purpose of 
//  programming logic devices manufactured by Altera and sold by 
//  Altera or its authorized distributors.  Please refer to the 
//  applicable agreement for further details.




//altbarrel_shift CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" DEVICE_FAMILY="STRATIXIV" PIPELINE=1 SHIFTDIR="LEFT" WIDTH=26 WIDTHDIST=5 aclr clk_en clock data distance result
//VERSION_BEGIN 10.1 cbx_altbarrel_shift 2010:09:06:21:07:24:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END

//synthesis_resources = reg 27 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altbarrel_shift_fjg
	( 
	aclr,
	clk_en,
	clock,
	data,
	distance,
	result) /* synthesis synthesis_clearbox=1 */;
	input   aclr;
	input   clk_en;
	input   clock;
	input   [25:0]  data;
	input   [4:0]  distance;
	output   [25:0]  result;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_off
`endif
	tri0   aclr;
	tri1   clk_en;
	tri0   clock;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_on
`endif

	reg	[0:0]	dir_pipe;
	reg	[25:0]	sbit_piper1d;
	wire  [5:0]  dir_w;
	wire  direction_w;
	wire  [15:0]  pad_w;
	wire  [155:0]  sbit_w;
	wire  [4:0]  sel_w;
	wire  [129:0]  smux_w;

	// synopsys translate_off
	initial
		dir_pipe = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dir_pipe <= 1'b0;
		else if  (clk_en == 1'b1)   dir_pipe <= {dir_w[4]};
	// synopsys translate_off
	initial
		sbit_piper1d = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sbit_piper1d <= 26'b0;
		else if  (clk_en == 1'b1)   sbit_piper1d <= smux_w[129:104];
	assign
		dir_w = {dir_pipe[0], dir_w[3:0], direction_w},
		direction_w = 1'b0,
		pad_w = {16{1'b0}},
		result = sbit_w[155:130],
		sbit_w = {sbit_piper1d, smux_w[103:0], data},
		sel_w = {distance[4:0]},
		smux_w = {((({26{(sel_w[4] & (~ dir_w[4]))}} & {sbit_w[113:104], pad_w[15:0]}) | ({26{(sel_w[4] & dir_w[4])}} & {pad_w[15:0], sbit_w[129:120]})) | ({26{(~ sel_w[4])}} & sbit_w[129:104])), ((({26{(sel_w[3] & (~ dir_w[3]))}} & {sbit_w[95:78], pad_w[7:0]}) | ({26{(sel_w[3] & dir_w[3])}} & {pad_w[7:0], sbit_w[103:86]})) | ({26{(~ sel_w[3])}} & sbit_w[103:78])), ((({26{(sel_w[2] & (~ dir_w[2]))}} & {sbit_w[73:52], pad_w[3:0]}) | ({26{(sel_w[2] & dir_w[2])}} & {pad_w[3:0], sbit_w[77:56]})) | ({26{(~ sel_w[2])}} & sbit_w[77:52])), ((({26{(sel_w[1] & (~ dir_w[1]))}} & {sbit_w[49:26], pad_w[1:0]}) | ({26{(sel_w[1] & dir_w[1])}} & {pad_w[1:0], sbit_w[51:28]})) | ({26{(~ sel_w[1])}} & sbit_w[51:26])), ((({26{(sel_w[0] & (~ dir_w[0]))}} & {sbit_w[24:0], pad_w[0]}) | ({26{(sel_w[0] & dir_w[0])}} & {pad_w[0], sbit_w[25:1]})) | ({26{(~ sel_w[0])}} & sbit_w[25:0]))};
endmodule //fpoint_qsys_addsub_single_altbarrel_shift_fjg


//altbarrel_shift CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" DEVICE_FAMILY="STRATIXIV" SHIFTDIR="RIGHT" WIDTH=26 WIDTHDIST=5 data distance result
//VERSION_BEGIN 10.1 cbx_altbarrel_shift 2010:09:06:21:07:24:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altbarrel_shift_44e
	( 
	data,
	distance,
	result) /* synthesis synthesis_clearbox=1 */;
	input   [25:0]  data;
	input   [4:0]  distance;
	output   [25:0]  result;

	wire  [5:0]  dir_w;
	wire  direction_w;
	wire  [15:0]  pad_w;
	wire  [155:0]  sbit_w;
	wire  [4:0]  sel_w;
	wire  [129:0]  smux_w;

	assign
		dir_w = {dir_w[4:0], direction_w},
		direction_w = 1'b1,
		pad_w = {16{1'b0}},
		result = sbit_w[155:130],
		sbit_w = {smux_w[129:0], data},
		sel_w = {distance[4:0]},
		smux_w = {((({26{(sel_w[4] & (~ dir_w[4]))}} & {sbit_w[113:104], pad_w[15:0]}) | ({26{(sel_w[4] & dir_w[4])}} & {pad_w[15:0], sbit_w[129:120]})) | ({26{(~ sel_w[4])}} & sbit_w[129:104])), ((({26{(sel_w[3] & (~ dir_w[3]))}} & {sbit_w[95:78], pad_w[7:0]}) | ({26{(sel_w[3] & dir_w[3])}} & {pad_w[7:0], sbit_w[103:86]})) | ({26{(~ sel_w[3])}} & sbit_w[103:78])), ((({26{(sel_w[2] & (~ dir_w[2]))}} & {sbit_w[73:52], pad_w[3:0]}) | ({26{(sel_w[2] & dir_w[2])}} & {pad_w[3:0], sbit_w[77:56]})) | ({26{(~ sel_w[2])}} & sbit_w[77:52])), ((({26{(sel_w[1] & (~ dir_w[1]))}} & {sbit_w[49:26], pad_w[1:0]}) | ({26{(sel_w[1] & dir_w[1])}} & {pad_w[1:0], sbit_w[51:28]})) | ({26{(~ sel_w[1])}} & sbit_w[51:26])), ((({26{(sel_w[0] & (~ dir_w[0]))}} & {sbit_w[24:0], pad_w[0]}) | ({26{(sel_w[0] & dir_w[0])}} & {pad_w[0], sbit_w[25:1]})) | ({26{(~ sel_w[0])}} & sbit_w[25:0]))};
endmodule //fpoint_qsys_addsub_single_altbarrel_shift_44e


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" WIDTH=32 WIDTHAD=5 data q
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="NO" WIDTH=16 WIDTHAD=4 data q
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="NO" WIDTH=8 WIDTHAD=3 data q zero
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="NO" WIDTH=4 WIDTHAD=2 data q zero
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="NO" WIDTH=2 WIDTHAD=1 data q zero
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_i0b
	( 
	data,
	q,
	zero) /* synthesis synthesis_clearbox=1 */;
	input   [1:0]  data;
	output   [0:0]  q;
	output   zero;


	assign
		q = {data[1]},
		zero = (~ (data[0] | data[1]));
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_i0b

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_l0b
	( 
	data,
	q,
	zero) /* synthesis synthesis_clearbox=1 */;
	input   [3:0]  data;
	output   [1:0]  q;
	output   zero;

	wire  [0:0]   wire_altpriority_encoder13_q;
	wire  wire_altpriority_encoder13_zero;
	wire  [0:0]   wire_altpriority_encoder14_q;
	wire  wire_altpriority_encoder14_zero;

	fpoint_qsys_addsub_single_altpriority_encoder_i0b   altpriority_encoder13
	( 
	.data(data[1:0]),
	.q(wire_altpriority_encoder13_q),
	.zero(wire_altpriority_encoder13_zero));
	fpoint_qsys_addsub_single_altpriority_encoder_i0b   altpriority_encoder14
	( 
	.data(data[3:2]),
	.q(wire_altpriority_encoder14_q),
	.zero(wire_altpriority_encoder14_zero));
	assign
		q = {(~ wire_altpriority_encoder14_zero), ((wire_altpriority_encoder14_zero & wire_altpriority_encoder13_q) | ((~ wire_altpriority_encoder14_zero) & wire_altpriority_encoder14_q))},
		zero = (wire_altpriority_encoder13_zero & wire_altpriority_encoder14_zero);
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_l0b

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_q0b
	( 
	data,
	q,
	zero) /* synthesis synthesis_clearbox=1 */;
	input   [7:0]  data;
	output   [2:0]  q;
	output   zero;

	wire  [1:0]   wire_altpriority_encoder11_q;
	wire  wire_altpriority_encoder11_zero;
	wire  [1:0]   wire_altpriority_encoder12_q;
	wire  wire_altpriority_encoder12_zero;

	fpoint_qsys_addsub_single_altpriority_encoder_l0b   altpriority_encoder11
	( 
	.data(data[3:0]),
	.q(wire_altpriority_encoder11_q),
	.zero(wire_altpriority_encoder11_zero));
	fpoint_qsys_addsub_single_altpriority_encoder_l0b   altpriority_encoder12
	( 
	.data(data[7:4]),
	.q(wire_altpriority_encoder12_q),
	.zero(wire_altpriority_encoder12_zero));
	assign
		q = {(~ wire_altpriority_encoder12_zero), (({2{wire_altpriority_encoder12_zero}} & wire_altpriority_encoder11_q) | ({2{(~ wire_altpriority_encoder12_zero)}} & wire_altpriority_encoder12_q))},
		zero = (wire_altpriority_encoder11_zero & wire_altpriority_encoder12_zero);
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_q0b


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="NO" WIDTH=8 WIDTHAD=3 data q
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="NO" WIDTH=4 WIDTHAD=2 data q
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="NO" WIDTH=2 WIDTHAD=1 data q
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_iha
	( 
	data,
	q) /* synthesis synthesis_clearbox=1 */;
	input   [1:0]  data;
	output   [0:0]  q;


	assign
		q = {data[1]};
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_iha

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_lha
	( 
	data,
	q) /* synthesis synthesis_clearbox=1 */;
	input   [3:0]  data;
	output   [1:0]  q;

	wire  [0:0]   wire_altpriority_encoder17_q;
	wire  [0:0]   wire_altpriority_encoder18_q;
	wire  wire_altpriority_encoder18_zero;

	fpoint_qsys_addsub_single_altpriority_encoder_iha   altpriority_encoder17
	( 
	.data(data[1:0]),
	.q(wire_altpriority_encoder17_q));
	fpoint_qsys_addsub_single_altpriority_encoder_i0b   altpriority_encoder18
	( 
	.data(data[3:2]),
	.q(wire_altpriority_encoder18_q),
	.zero(wire_altpriority_encoder18_zero));
	assign
		q = {(~ wire_altpriority_encoder18_zero), ((wire_altpriority_encoder18_zero & wire_altpriority_encoder17_q) | ((~ wire_altpriority_encoder18_zero) & wire_altpriority_encoder18_q))};
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_lha

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_qha
	( 
	data,
	q) /* synthesis synthesis_clearbox=1 */;
	input   [7:0]  data;
	output   [2:0]  q;

	wire  [1:0]   wire_altpriority_encoder15_q;
	wire  [1:0]   wire_altpriority_encoder16_q;
	wire  wire_altpriority_encoder16_zero;

	fpoint_qsys_addsub_single_altpriority_encoder_lha   altpriority_encoder15
	( 
	.data(data[3:0]),
	.q(wire_altpriority_encoder15_q));
	fpoint_qsys_addsub_single_altpriority_encoder_l0b   altpriority_encoder16
	( 
	.data(data[7:4]),
	.q(wire_altpriority_encoder16_q),
	.zero(wire_altpriority_encoder16_zero));
	assign
		q = {(~ wire_altpriority_encoder16_zero), (({2{wire_altpriority_encoder16_zero}} & wire_altpriority_encoder15_q) | ({2{(~ wire_altpriority_encoder16_zero)}} & wire_altpriority_encoder16_q))};
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_qha

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_aja
	( 
	data,
	q) /* synthesis synthesis_clearbox=1 */;
	input   [15:0]  data;
	output   [3:0]  q;

	wire  [2:0]   wire_altpriority_encoder10_q;
	wire  wire_altpriority_encoder10_zero;
	wire  [2:0]   wire_altpriority_encoder9_q;

	fpoint_qsys_addsub_single_altpriority_encoder_q0b   altpriority_encoder10
	( 
	.data(data[15:8]),
	.q(wire_altpriority_encoder10_q),
	.zero(wire_altpriority_encoder10_zero));
	fpoint_qsys_addsub_single_altpriority_encoder_qha   altpriority_encoder9
	( 
	.data(data[7:0]),
	.q(wire_altpriority_encoder9_q));
	assign
		q = {(~ wire_altpriority_encoder10_zero), (({3{wire_altpriority_encoder10_zero}} & wire_altpriority_encoder9_q) | ({3{(~ wire_altpriority_encoder10_zero)}} & wire_altpriority_encoder10_q))};
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_aja


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="NO" WIDTH=16 WIDTHAD=4 data q zero
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_a2b
	( 
	data,
	q,
	zero) /* synthesis synthesis_clearbox=1 */;
	input   [15:0]  data;
	output   [3:0]  q;
	output   zero;

	wire  [2:0]   wire_altpriority_encoder19_q;
	wire  wire_altpriority_encoder19_zero;
	wire  [2:0]   wire_altpriority_encoder20_q;
	wire  wire_altpriority_encoder20_zero;

	fpoint_qsys_addsub_single_altpriority_encoder_q0b   altpriority_encoder19
	( 
	.data(data[7:0]),
	.q(wire_altpriority_encoder19_q),
	.zero(wire_altpriority_encoder19_zero));
	fpoint_qsys_addsub_single_altpriority_encoder_q0b   altpriority_encoder20
	( 
	.data(data[15:8]),
	.q(wire_altpriority_encoder20_q),
	.zero(wire_altpriority_encoder20_zero));
	assign
		q = {(~ wire_altpriority_encoder20_zero), (({3{wire_altpriority_encoder20_zero}} & wire_altpriority_encoder19_q) | ({3{(~ wire_altpriority_encoder20_zero)}} & wire_altpriority_encoder20_q))},
		zero = (wire_altpriority_encoder19_zero & wire_altpriority_encoder20_zero);
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_a2b

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_9u8
	( 
	data,
	q) /* synthesis synthesis_clearbox=1 */;
	input   [31:0]  data;
	output   [4:0]  q;

	wire  [3:0]   wire_altpriority_encoder7_q;
	wire  [3:0]   wire_altpriority_encoder8_q;
	wire  wire_altpriority_encoder8_zero;

	fpoint_qsys_addsub_single_altpriority_encoder_aja   altpriority_encoder7
	( 
	.data(data[15:0]),
	.q(wire_altpriority_encoder7_q));
	fpoint_qsys_addsub_single_altpriority_encoder_a2b   altpriority_encoder8
	( 
	.data(data[31:16]),
	.q(wire_altpriority_encoder8_q),
	.zero(wire_altpriority_encoder8_zero));
	assign
		q = {(~ wire_altpriority_encoder8_zero), (({4{wire_altpriority_encoder8_zero}} & wire_altpriority_encoder7_q) | ({4{(~ wire_altpriority_encoder8_zero)}} & wire_altpriority_encoder8_q))};
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_9u8


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="YES" WIDTH=32 WIDTHAD=5 data q
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="YES" WIDTH=16 WIDTHAD=4 data q zero
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="YES" WIDTH=8 WIDTHAD=3 data q zero
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="YES" WIDTH=4 WIDTHAD=2 data q zero
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="YES" WIDTH=2 WIDTHAD=1 data q zero
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_64b
	( 
	data,
	q,
	zero) /* synthesis synthesis_clearbox=1 */;
	input   [1:0]  data;
	output   [0:0]  q;
	output   zero;


	assign
		q = {(~ data[0])},
		zero = (~ (data[0] | data[1]));
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_64b

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_94b
	( 
	data,
	q,
	zero) /* synthesis synthesis_clearbox=1 */;
	input   [3:0]  data;
	output   [1:0]  q;
	output   zero;

	wire  [0:0]   wire_altpriority_encoder27_q;
	wire  wire_altpriority_encoder27_zero;
	wire  [0:0]   wire_altpriority_encoder28_q;
	wire  wire_altpriority_encoder28_zero;

	fpoint_qsys_addsub_single_altpriority_encoder_64b   altpriority_encoder27
	( 
	.data(data[1:0]),
	.q(wire_altpriority_encoder27_q),
	.zero(wire_altpriority_encoder27_zero));
	fpoint_qsys_addsub_single_altpriority_encoder_64b   altpriority_encoder28
	( 
	.data(data[3:2]),
	.q(wire_altpriority_encoder28_q),
	.zero(wire_altpriority_encoder28_zero));
	assign
		q = {wire_altpriority_encoder27_zero, ((wire_altpriority_encoder27_zero & wire_altpriority_encoder28_q) | ((~ wire_altpriority_encoder27_zero) & wire_altpriority_encoder27_q))},
		zero = (wire_altpriority_encoder27_zero & wire_altpriority_encoder28_zero);
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_94b

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_e4b
	( 
	data,
	q,
	zero) /* synthesis synthesis_clearbox=1 */;
	input   [7:0]  data;
	output   [2:0]  q;
	output   zero;

	wire  [1:0]   wire_altpriority_encoder25_q;
	wire  wire_altpriority_encoder25_zero;
	wire  [1:0]   wire_altpriority_encoder26_q;
	wire  wire_altpriority_encoder26_zero;

	fpoint_qsys_addsub_single_altpriority_encoder_94b   altpriority_encoder25
	( 
	.data(data[3:0]),
	.q(wire_altpriority_encoder25_q),
	.zero(wire_altpriority_encoder25_zero));
	fpoint_qsys_addsub_single_altpriority_encoder_94b   altpriority_encoder26
	( 
	.data(data[7:4]),
	.q(wire_altpriority_encoder26_q),
	.zero(wire_altpriority_encoder26_zero));
	assign
		q = {wire_altpriority_encoder25_zero, (({2{wire_altpriority_encoder25_zero}} & wire_altpriority_encoder26_q) | ({2{(~ wire_altpriority_encoder25_zero)}} & wire_altpriority_encoder25_q))},
		zero = (wire_altpriority_encoder25_zero & wire_altpriority_encoder26_zero);
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_e4b

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_u5b
	( 
	data,
	q,
	zero) /* synthesis synthesis_clearbox=1 */;
	input   [15:0]  data;
	output   [3:0]  q;
	output   zero;

	wire  [2:0]   wire_altpriority_encoder23_q;
	wire  wire_altpriority_encoder23_zero;
	wire  [2:0]   wire_altpriority_encoder24_q;
	wire  wire_altpriority_encoder24_zero;

	fpoint_qsys_addsub_single_altpriority_encoder_e4b   altpriority_encoder23
	( 
	.data(data[7:0]),
	.q(wire_altpriority_encoder23_q),
	.zero(wire_altpriority_encoder23_zero));
	fpoint_qsys_addsub_single_altpriority_encoder_e4b   altpriority_encoder24
	( 
	.data(data[15:8]),
	.q(wire_altpriority_encoder24_q),
	.zero(wire_altpriority_encoder24_zero));
	assign
		q = {wire_altpriority_encoder23_zero, (({3{wire_altpriority_encoder23_zero}} & wire_altpriority_encoder24_q) | ({3{(~ wire_altpriority_encoder23_zero)}} & wire_altpriority_encoder23_q))},
		zero = (wire_altpriority_encoder23_zero & wire_altpriority_encoder24_zero);
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_u5b


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="YES" WIDTH=16 WIDTHAD=4 data q
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="YES" WIDTH=8 WIDTHAD=3 data q
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="YES" WIDTH=4 WIDTHAD=2 data q
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END


//altpriority_encoder CBX_AUTO_BLACKBOX="ON" CBX_SINGLE_OUTPUT_FILE="ON" LSB_PRIORITY="YES" WIDTH=2 WIDTHAD=1 data q
//VERSION_BEGIN 10.1 cbx_altpriority_encoder 2010:09:06:21:07:25:PN cbx_mgl 2010:09:06:21:22:18:PN  VERSION_END

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_6la
	( 
	data,
	q) /* synthesis synthesis_clearbox=1 */;
	input   [1:0]  data;
	output   [0:0]  q;


	assign
		q = {(~ data[0])};
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_6la

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_9la
	( 
	data,
	q) /* synthesis synthesis_clearbox=1 */;
	input   [3:0]  data;
	output   [1:0]  q;

	wire  [0:0]   wire_altpriority_encoder33_q;
	wire  wire_altpriority_encoder33_zero;
	wire  [0:0]   wire_altpriority_encoder34_q;

	fpoint_qsys_addsub_single_altpriority_encoder_64b   altpriority_encoder33
	( 
	.data(data[1:0]),
	.q(wire_altpriority_encoder33_q),
	.zero(wire_altpriority_encoder33_zero));
	fpoint_qsys_addsub_single_altpriority_encoder_6la   altpriority_encoder34
	( 
	.data(data[3:2]),
	.q(wire_altpriority_encoder34_q));
	assign
		q = {wire_altpriority_encoder33_zero, ((wire_altpriority_encoder33_zero & wire_altpriority_encoder34_q) | ((~ wire_altpriority_encoder33_zero) & wire_altpriority_encoder33_q))};
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_9la

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_ela
	( 
	data,
	q) /* synthesis synthesis_clearbox=1 */;
	input   [7:0]  data;
	output   [2:0]  q;

	wire  [1:0]   wire_altpriority_encoder31_q;
	wire  wire_altpriority_encoder31_zero;
	wire  [1:0]   wire_altpriority_encoder32_q;

	fpoint_qsys_addsub_single_altpriority_encoder_94b   altpriority_encoder31
	( 
	.data(data[3:0]),
	.q(wire_altpriority_encoder31_q),
	.zero(wire_altpriority_encoder31_zero));
	fpoint_qsys_addsub_single_altpriority_encoder_9la   altpriority_encoder32
	( 
	.data(data[7:4]),
	.q(wire_altpriority_encoder32_q));
	assign
		q = {wire_altpriority_encoder31_zero, (({2{wire_altpriority_encoder31_zero}} & wire_altpriority_encoder32_q) | ({2{(~ wire_altpriority_encoder31_zero)}} & wire_altpriority_encoder31_q))};
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_ela

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_uma
	( 
	data,
	q) /* synthesis synthesis_clearbox=1 */;
	input   [15:0]  data;
	output   [3:0]  q;

	wire  [2:0]   wire_altpriority_encoder29_q;
	wire  wire_altpriority_encoder29_zero;
	wire  [2:0]   wire_altpriority_encoder30_q;

	fpoint_qsys_addsub_single_altpriority_encoder_e4b   altpriority_encoder29
	( 
	.data(data[7:0]),
	.q(wire_altpriority_encoder29_q),
	.zero(wire_altpriority_encoder29_zero));
	fpoint_qsys_addsub_single_altpriority_encoder_ela   altpriority_encoder30
	( 
	.data(data[15:8]),
	.q(wire_altpriority_encoder30_q));
	assign
		q = {wire_altpriority_encoder29_zero, (({3{wire_altpriority_encoder29_zero}} & wire_altpriority_encoder30_q) | ({3{(~ wire_altpriority_encoder29_zero)}} & wire_altpriority_encoder29_q))};
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_uma

//synthesis_resources = 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single_altpriority_encoder_tma
	( 
	data,
	q) /* synthesis synthesis_clearbox=1 */;
	input   [31:0]  data;
	output   [4:0]  q;

	wire  [3:0]   wire_altpriority_encoder21_q;
	wire  wire_altpriority_encoder21_zero;
	wire  [3:0]   wire_altpriority_encoder22_q;

	fpoint_qsys_addsub_single_altpriority_encoder_u5b   altpriority_encoder21
	( 
	.data(data[15:0]),
	.q(wire_altpriority_encoder21_q),
	.zero(wire_altpriority_encoder21_zero));
	fpoint_qsys_addsub_single_altpriority_encoder_uma   altpriority_encoder22
	( 
	.data(data[31:16]),
	.q(wire_altpriority_encoder22_q));
	assign
		q = {wire_altpriority_encoder21_zero, (({4{wire_altpriority_encoder21_zero}} & wire_altpriority_encoder22_q) | ({4{(~ wire_altpriority_encoder21_zero)}} & wire_altpriority_encoder21_q))};
endmodule //fpoint_qsys_addsub_single_altpriority_encoder_tma

//synthesis_resources = lpm_add_sub 14 lpm_compare 1 reg 356 
//synopsys translate_off
`timescale 1 ps / 1 ps
//synopsys translate_on
module  fpoint_qsys_addsub_single
	( 
	aclr,
	add_sub,
	clk_en,
	clock,
	dataa,
	datab,
	result) /* synthesis synthesis_clearbox=1 */;
	input   aclr;
	input   add_sub;
	input   clk_en;
	input   clock;
	input   [31:0]  dataa;
	input   [31:0]  datab;
	output   [31:0]  result;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_off
`endif
	tri0   aclr;
	tri1   add_sub;
	tri1   clk_en;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_on
`endif

	wire  [25:0]   wire_lbarrel_shift_result;
	wire  [25:0]   wire_rbarrel_shift_result;
	wire  [4:0]   wire_leading_zeroes_cnt_q;
	wire  [4:0]   wire_trailing_zeros_cnt_q;
	reg	add_sub_dffe1;
	reg	add_sub_dffe12;
	reg	[8:0]	aligned_dataa_exp_dffe12;
	reg	[23:0]	aligned_dataa_man_dffe12;
	reg	aligned_dataa_sign_dffe12;
	reg	[8:0]	aligned_datab_exp_dffe12;
	reg	[23:0]	aligned_datab_man_dffe12;
	reg	aligned_datab_sign_dffe12;
	reg	both_inputs_are_infinite_dffe1;
	reg	[7:0]	data_exp_dffe1;
	reg	[25:0]	dataa_man_dffe1;
	reg	dataa_sign_dffe1;
	reg	[25:0]	datab_man_dffe1;
	reg	datab_sign_dffe1;
	reg	denormal_res_dffe3;
	reg	denormal_res_dffe4;
	reg	[1:0]	exp_adj_dffe21;
	reg	[7:0]	exp_out_dffe5;
	reg	[7:0]	exp_res_dffe2;
	reg	[7:0]	exp_res_dffe21;
	reg	[7:0]	exp_res_dffe3;
	reg	[7:0]	exp_res_dffe4;
	reg	infinite_output_sign_dffe1;
	reg	infinite_output_sign_dffe2;
	reg	infinite_output_sign_dffe21;
	reg	infinite_output_sign_dffe3;
	reg	infinite_output_sign_dffe31;
	reg	infinite_output_sign_dffe4;
	reg	infinite_res_dffe3;
	reg	infinite_res_dffe4;
	reg	infinity_magnitude_sub_dffe2;
	reg	infinity_magnitude_sub_dffe21;
	reg	infinity_magnitude_sub_dffe3;
	reg	infinity_magnitude_sub_dffe31;
	reg	infinity_magnitude_sub_dffe4;
	reg	input_dataa_infinite_dffe12;
	reg	input_dataa_nan_dffe12;
	reg	input_datab_infinite_dffe12;
	reg	input_datab_nan_dffe12;
	reg	input_is_infinite_dffe1;
	reg	input_is_infinite_dffe2;
	reg	input_is_infinite_dffe21;
	reg	input_is_infinite_dffe3;
	reg	input_is_infinite_dffe31;
	reg	input_is_infinite_dffe4;
	reg	input_is_nan_dffe1;
	reg	input_is_nan_dffe2;
	reg	input_is_nan_dffe21;
	reg	input_is_nan_dffe3;
	reg	input_is_nan_dffe31;
	reg	input_is_nan_dffe4;
	reg	[25:0]	man_add_sub_res_mag_dffe21;
	reg	man_add_sub_res_sign_dffe21;
	reg	[25:0]	man_dffe31;
	reg	[4:0]	man_leading_zeros_dffe31;
	reg	[22:0]	man_out_dffe5;
	reg	[22:0]	man_res_dffe4;
	reg	man_res_is_not_zero_dffe3;
	reg	man_res_is_not_zero_dffe31;
	reg	man_res_is_not_zero_dffe4;
	reg	need_complement_dffe2;
	reg	round_bit_dffe21;
	reg	round_bit_dffe3;
	reg	round_bit_dffe31;
	reg	rounded_res_infinity_dffe4;
	reg	sign_dffe31;
	reg	sign_out_dffe5;
	reg	sign_res_dffe3;
	reg	sign_res_dffe4;
	reg	sticky_bit_dffe1;
	reg	sticky_bit_dffe2;
	reg	sticky_bit_dffe21;
	reg	sticky_bit_dffe3;
	reg	sticky_bit_dffe31;
	reg	zero_man_sign_dffe2;
	reg	zero_man_sign_dffe21;
	wire  [8:0]   wire_add_sub1_result;
	wire  [8:0]   wire_add_sub2_result;
	wire  [5:0]   wire_add_sub3_result;
	wire  [8:0]   wire_add_sub4_result;
	wire  [8:0]   wire_add_sub5_result;
	wire  [8:0]   wire_add_sub6_result;
	wire  wire_man_2comp_res_lower_cout;
	wire  [13:0]   wire_man_2comp_res_lower_result;
	wire  [13:0]   wire_man_2comp_res_upper0_result;
	wire  [13:0]   wire_man_2comp_res_upper1_result;
	wire  wire_man_add_sub_lower_cout;
	wire  [13:0]   wire_man_add_sub_lower_result;
	wire  [13:0]   wire_man_add_sub_upper0_result;
	wire  [13:0]   wire_man_add_sub_upper1_result;
	wire  wire_man_res_rounding_add_sub_lower_cout;
	wire  [12:0]   wire_man_res_rounding_add_sub_lower_result;
	wire  [12:0]   wire_man_res_rounding_add_sub_upper1_result;
	wire  wire_trailing_zeros_limit_comparator_agb;
	wire  add_sub_dffe11_wi;
	wire  add_sub_dffe11_wo;
	wire  add_sub_dffe12_wi;
	wire  add_sub_dffe12_wo;
	wire  add_sub_dffe13_wi;
	wire  add_sub_dffe13_wo;
	wire  add_sub_dffe14_wi;
	wire  add_sub_dffe14_wo;
	wire  add_sub_dffe15_wi;
	wire  add_sub_dffe15_wo;
	wire  add_sub_dffe1_wi;
	wire  add_sub_dffe1_wo;
	wire  add_sub_dffe25_wi;
	wire  add_sub_dffe25_wo;
	wire  add_sub_w2;
	wire  [12:0]  adder_upper_w;
	wire  [8:0]  aligned_dataa_exp_dffe12_wi;
	wire  [8:0]  aligned_dataa_exp_dffe12_wo;
	wire  [8:0]  aligned_dataa_exp_dffe13_wi;
	wire  [8:0]  aligned_dataa_exp_dffe13_wo;
	wire  [8:0]  aligned_dataa_exp_dffe14_wi;
	wire  [8:0]  aligned_dataa_exp_dffe14_wo;
	wire  [8:0]  aligned_dataa_exp_dffe15_wi;
	wire  [8:0]  aligned_dataa_exp_dffe15_wo;
	wire  [8:0]  aligned_dataa_exp_w;
	wire  [23:0]  aligned_dataa_man_dffe12_wi;
	wire  [23:0]  aligned_dataa_man_dffe12_wo;
	wire  [23:0]  aligned_dataa_man_dffe13_wi;
	wire  [23:0]  aligned_dataa_man_dffe13_wo;
	wire  [23:0]  aligned_dataa_man_dffe14_wi;
	wire  [23:0]  aligned_dataa_man_dffe14_wo;
	wire  [25:0]  aligned_dataa_man_dffe15_w;
	wire  [23:0]  aligned_dataa_man_dffe15_wi;
	wire  [23:0]  aligned_dataa_man_dffe15_wo;
	wire  [25:0]  aligned_dataa_man_w;
	wire  aligned_dataa_sign_dffe12_wi;
	wire  aligned_dataa_sign_dffe12_wo;
	wire  aligned_dataa_sign_dffe13_wi;
	wire  aligned_dataa_sign_dffe13_wo;
	wire  aligned_dataa_sign_dffe14_wi;
	wire  aligned_dataa_sign_dffe14_wo;
	wire  aligned_dataa_sign_dffe15_wi;
	wire  aligned_dataa_sign_dffe15_wo;
	wire  aligned_dataa_sign_w;
	wire  [8:0]  aligned_datab_exp_dffe12_wi;
	wire  [8:0]  aligned_datab_exp_dffe12_wo;
	wire  [8:0]  aligned_datab_exp_dffe13_wi;
	wire  [8:0]  aligned_datab_exp_dffe13_wo;
	wire  [8:0]  aligned_datab_exp_dffe14_wi;
	wire  [8:0]  aligned_datab_exp_dffe14_wo;
	wire  [8:0]  aligned_datab_exp_dffe15_wi;
	wire  [8:0]  aligned_datab_exp_dffe15_wo;
	wire  [8:0]  aligned_datab_exp_w;
	wire  [23:0]  aligned_datab_man_dffe12_wi;
	wire  [23:0]  aligned_datab_man_dffe12_wo;
	wire  [23:0]  aligned_datab_man_dffe13_wi;
	wire  [23:0]  aligned_datab_man_dffe13_wo;
	wire  [23:0]  aligned_datab_man_dffe14_wi;
	wire  [23:0]  aligned_datab_man_dffe14_wo;
	wire  [25:0]  aligned_datab_man_dffe15_w;
	wire  [23:0]  aligned_datab_man_dffe15_wi;
	wire  [23:0]  aligned_datab_man_dffe15_wo;
	wire  [25:0]  aligned_datab_man_w;
	wire  aligned_datab_sign_dffe12_wi;
	wire  aligned_datab_sign_dffe12_wo;
	wire  aligned_datab_sign_dffe13_wi;
	wire  aligned_datab_sign_dffe13_wo;
	wire  aligned_datab_sign_dffe14_wi;
	wire  aligned_datab_sign_dffe14_wo;
	wire  aligned_datab_sign_dffe15_wi;
	wire  aligned_datab_sign_dffe15_wo;
	wire  aligned_datab_sign_w;
	wire  borrow_w;
	wire  both_inputs_are_infinite_dffe1_wi;
	wire  both_inputs_are_infinite_dffe1_wo;
	wire  both_inputs_are_infinite_dffe25_wi;
	wire  both_inputs_are_infinite_dffe25_wo;
	wire  [7:0]  data_exp_dffe1_wi;
	wire  [7:0]  data_exp_dffe1_wo;
	wire  [31:0]  dataa_dffe11_wi;
	wire  [31:0]  dataa_dffe11_wo;
	wire  [25:0]  dataa_man_dffe1_wi;
	wire  [25:0]  dataa_man_dffe1_wo;
	wire  dataa_sign_dffe1_wi;
	wire  dataa_sign_dffe1_wo;
	wire  dataa_sign_dffe25_wi;
	wire  dataa_sign_dffe25_wo;
	wire  [31:0]  datab_dffe11_wi;
	wire  [31:0]  datab_dffe11_wo;
	wire  [25:0]  datab_man_dffe1_wi;
	wire  [25:0]  datab_man_dffe1_wo;
	wire  datab_sign_dffe1_wi;
	wire  datab_sign_dffe1_wo;
	wire  denormal_flag_w;
	wire  denormal_res_dffe32_wi;
	wire  denormal_res_dffe32_wo;
	wire  denormal_res_dffe33_wi;
	wire  denormal_res_dffe33_wo;
	wire  denormal_res_dffe3_wi;
	wire  denormal_res_dffe3_wo;
	wire  denormal_res_dffe41_wi;
	wire  denormal_res_dffe41_wo;
	wire  denormal_res_dffe42_wi;
	wire  denormal_res_dffe42_wo;
	wire  denormal_res_dffe4_wi;
	wire  denormal_res_dffe4_wo;
	wire  denormal_result_w;
	wire  [7:0]  exp_a_all_one_w;
	wire  [7:0]  exp_a_not_zero_w;
	wire  [6:0]  exp_adj_0pads;
	wire  [1:0]  exp_adj_dffe21_wi;
	wire  [1:0]  exp_adj_dffe21_wo;
	wire  [1:0]  exp_adj_dffe23_wi;
	wire  [1:0]  exp_adj_dffe23_wo;
	wire  [1:0]  exp_adj_dffe26_wi;
	wire  [1:0]  exp_adj_dffe26_wo;
	wire  [1:0]  exp_adjust_by_add1;
	wire  [1:0]  exp_adjust_by_add2;
	wire  [8:0]  exp_adjustment2_add_sub_dataa_w;
	wire  [8:0]  exp_adjustment2_add_sub_datab_w;
	wire  [8:0]  exp_adjustment2_add_sub_w;
	wire  [8:0]  exp_adjustment_add_sub_dataa_w;
	wire  [8:0]  exp_adjustment_add_sub_datab_w;
	wire  [8:0]  exp_adjustment_add_sub_w;
	wire  [7:0]  exp_all_ones_w;
	wire  [7:0]  exp_all_zeros_w;
	wire  exp_amb_mux_dffe13_wi;
	wire  exp_amb_mux_dffe13_wo;
	wire  exp_amb_mux_dffe14_wi;
	wire  exp_amb_mux_dffe14_wo;
	wire  exp_amb_mux_dffe15_wi;
	wire  exp_amb_mux_dffe15_wo;
	wire  exp_amb_mux_w;
	wire  [8:0]  exp_amb_w;
	wire  [7:0]  exp_b_all_one_w;
	wire  [7:0]  exp_b_not_zero_w;
	wire  [8:0]  exp_bma_w;
	wire  [2:0]  exp_diff_abs_exceed_max_w;
	wire  [4:0]  exp_diff_abs_max_w;
	wire  [7:0]  exp_diff_abs_w;
	wire  [7:0]  exp_intermediate_res_dffe41_wi;
	wire  [7:0]  exp_intermediate_res_dffe41_wo;
	wire  [7:0]  exp_intermediate_res_dffe42_wi;
	wire  [7:0]  exp_intermediate_res_dffe42_wo;
	wire  [7:0]  exp_intermediate_res_w;
	wire  [7:0]  exp_out_dffe5_wi;
	wire  [7:0]  exp_out_dffe5_wo;
	wire  [7:0]  exp_res_dffe21_wi;
	wire  [7:0]  exp_res_dffe21_wo;
	wire  [7:0]  exp_res_dffe22_wi;
	wire  [7:0]  exp_res_dffe22_wo;
	wire  [7:0]  exp_res_dffe23_wi;
	wire  [7:0]  exp_res_dffe23_wo;
	wire  [7:0]  exp_res_dffe25_wi;
	wire  [7:0]  exp_res_dffe25_wo;
	wire  [7:0]  exp_res_dffe26_wi;
	wire  [7:0]  exp_res_dffe26_wo;
	wire  [7:0]  exp_res_dffe27_wi;
	wire  [7:0]  exp_res_dffe27_wo;
	wire  [7:0]  exp_res_dffe2_wi;
	wire  [7:0]  exp_res_dffe2_wo;
	wire  [7:0]  exp_res_dffe32_wi;
	wire  [7:0]  exp_res_dffe32_wo;
	wire  [7:0]  exp_res_dffe33_wi;
	wire  [7:0]  exp_res_dffe33_wo;
	wire  [7:0]  exp_res_dffe3_wi;
	wire  [7:0]  exp_res_dffe3_wo;
	wire  [7:0]  exp_res_dffe4_wi;
	wire  [7:0]  exp_res_dffe4_wo;
	wire  [7:0]  exp_res_max_w;
	wire  [8:0]  exp_res_not_zero_w;
	wire  [8:0]  exp_res_rounding_adder_dataa_w;
	wire  [8:0]  exp_res_rounding_adder_w;
	wire  exp_rounded_res_infinity_w;
	wire  [7:0]  exp_rounded_res_max_w;
	wire  [7:0]  exp_rounded_res_w;
	wire  [8:0]  exp_rounding_adjustment_w;
	wire  [8:0]  exp_value;
	wire  force_infinity_w;
	wire  force_nan_w;
	wire  force_zero_w;
	wire  guard_bit_dffe3_wo;
	wire  infinite_output_sign_dffe1_wi;
	wire  infinite_output_sign_dffe1_wo;
	wire  infinite_output_sign_dffe21_wi;
	wire  infinite_output_sign_dffe21_wo;
	wire  infinite_output_sign_dffe22_wi;
	wire  infinite_output_sign_dffe22_wo;
	wire  infinite_output_sign_dffe23_wi;
	wire  infinite_output_sign_dffe23_wo;
	wire  infinite_output_sign_dffe25_wi;
	wire  infinite_output_sign_dffe25_wo;
	wire  infinite_output_sign_dffe26_wi;
	wire  infinite_output_sign_dffe26_wo;
	wire  infinite_output_sign_dffe27_wi;
	wire  infinite_output_sign_dffe27_wo;
	wire  infinite_output_sign_dffe2_wi;
	wire  infinite_output_sign_dffe2_wo;
	wire  infinite_output_sign_dffe31_wi;
	wire  infinite_output_sign_dffe31_wo;
	wire  infinite_output_sign_dffe32_wi;
	wire  infinite_output_sign_dffe32_wo;
	wire  infinite_output_sign_dffe33_wi;
	wire  infinite_output_sign_dffe33_wo;
	wire  infinite_output_sign_dffe3_wi;
	wire  infinite_output_sign_dffe3_wo;
	wire  infinite_output_sign_dffe41_wi;
	wire  infinite_output_sign_dffe41_wo;
	wire  infinite_output_sign_dffe42_wi;
	wire  infinite_output_sign_dffe42_wo;
	wire  infinite_output_sign_dffe4_wi;
	wire  infinite_output_sign_dffe4_wo;
	wire  infinite_res_dff32_wi;
	wire  infinite_res_dff32_wo;
	wire  infinite_res_dff33_wi;
	wire  infinite_res_dff33_wo;
	wire  infinite_res_dffe3_wi;
	wire  infinite_res_dffe3_wo;
	wire  infinite_res_dffe41_wi;
	wire  infinite_res_dffe41_wo;
	wire  infinite_res_dffe42_wi;
	wire  infinite_res_dffe42_wo;
	wire  infinite_res_dffe4_wi;
	wire  infinite_res_dffe4_wo;
	wire  infinity_magnitude_sub_dffe21_wi;
	wire  infinity_magnitude_sub_dffe21_wo;
	wire  infinity_magnitude_sub_dffe22_wi;
	wire  infinity_magnitude_sub_dffe22_wo;
	wire  infinity_magnitude_sub_dffe23_wi;
	wire  infinity_magnitude_sub_dffe23_wo;
	wire  infinity_magnitude_sub_dffe26_wi;
	wire  infinity_magnitude_sub_dffe26_wo;
	wire  infinity_magnitude_sub_dffe27_wi;
	wire  infinity_magnitude_sub_dffe27_wo;
	wire  infinity_magnitude_sub_dffe2_wi;
	wire  infinity_magnitude_sub_dffe2_wo;
	wire  infinity_magnitude_sub_dffe31_wi;
	wire  infinity_magnitude_sub_dffe31_wo;
	wire  infinity_magnitude_sub_dffe32_wi;
	wire  infinity_magnitude_sub_dffe32_wo;
	wire  infinity_magnitude_sub_dffe33_wi;
	wire  infinity_magnitude_sub_dffe33_wo;
	wire  infinity_magnitude_sub_dffe3_wi;
	wire  infinity_magnitude_sub_dffe3_wo;
	wire  infinity_magnitude_sub_dffe41_wi;
	wire  infinity_magnitude_sub_dffe41_wo;
	wire  infinity_magnitude_sub_dffe42_wi;
	wire  infinity_magnitude_sub_dffe42_wo;
	wire  infinity_magnitude_sub_dffe4_wi;
	wire  infinity_magnitude_sub_dffe4_wo;
	wire  input_dataa_denormal_dffe11_wi;
	wire  input_dataa_denormal_dffe11_wo;
	wire  input_dataa_denormal_w;
	wire  input_dataa_infinite_dffe11_wi;
	wire  input_dataa_infinite_dffe11_wo;
	wire  input_dataa_infinite_dffe12_wi;
	wire  input_dataa_infinite_dffe12_wo;
	wire  input_dataa_infinite_dffe13_wi;
	wire  input_dataa_infinite_dffe13_wo;
	wire  input_dataa_infinite_dffe14_wi;
	wire  input_dataa_infinite_dffe14_wo;
	wire  input_dataa_infinite_dffe15_wi;
	wire  input_dataa_infinite_dffe15_wo;
	wire  input_dataa_infinite_w;
	wire  input_dataa_nan_dffe11_wi;
	wire  input_dataa_nan_dffe11_wo;
	wire  input_dataa_nan_dffe12_wi;
	wire  input_dataa_nan_dffe12_wo;
	wire  input_dataa_nan_w;
	wire  input_dataa_zero_dffe11_wi;
	wire  input_dataa_zero_dffe11_wo;
	wire  input_dataa_zero_w;
	wire  input_datab_denormal_dffe11_wi;
	wire  input_datab_denormal_dffe11_wo;
	wire  input_datab_denormal_w;
	wire  input_datab_infinite_dffe11_wi;
	wire  input_datab_infinite_dffe11_wo;
	wire  input_datab_infinite_dffe12_wi;
	wire  input_datab_infinite_dffe12_wo;
	wire  input_datab_infinite_dffe13_wi;
	wire  input_datab_infinite_dffe13_wo;
	wire  input_datab_infinite_dffe14_wi;
	wire  input_datab_infinite_dffe14_wo;
	wire  input_datab_infinite_dffe15_wi;
	wire  input_datab_infinite_dffe15_wo;
	wire  input_datab_infinite_w;
	wire  input_datab_nan_dffe11_wi;
	wire  input_datab_nan_dffe11_wo;
	wire  input_datab_nan_dffe12_wi;
	wire  input_datab_nan_dffe12_wo;
	wire  input_datab_nan_w;
	wire  input_datab_zero_dffe11_wi;
	wire  input_datab_zero_dffe11_wo;
	wire  input_datab_zero_w;
	wire  input_is_infinite_dffe1_wi;
	wire  input_is_infinite_dffe1_wo;
	wire  input_is_infinite_dffe21_wi;
	wire  input_is_infinite_dffe21_wo;
	wire  input_is_infinite_dffe22_wi;
	wire  input_is_infinite_dffe22_wo;
	wire  input_is_infinite_dffe23_wi;
	wire  input_is_infinite_dffe23_wo;
	wire  input_is_infinite_dffe25_wi;
	wire  input_is_infinite_dffe25_wo;
	wire  input_is_infinite_dffe26_wi;
	wire  input_is_infinite_dffe26_wo;
	wire  input_is_infinite_dffe27_wi;
	wire  input_is_infinite_dffe27_wo;
	wire  input_is_infinite_dffe2_wi;
	wire  input_is_infinite_dffe2_wo;
	wire  input_is_infinite_dffe31_wi;
	wire  input_is_infinite_dffe31_wo;
	wire  input_is_infinite_dffe32_wi;
	wire  input_is_infinite_dffe32_wo;
	wire  input_is_infinite_dffe33_wi;
	wire  input_is_infinite_dffe33_wo;
	wire  input_is_infinite_dffe3_wi;
	wire  input_is_infinite_dffe3_wo;
	wire  input_is_infinite_dffe41_wi;
	wire  input_is_infinite_dffe41_wo;
	wire  input_is_infinite_dffe42_wi;
	wire  input_is_infinite_dffe42_wo;
	wire  input_is_infinite_dffe4_wi;
	wire  input_is_infinite_dffe4_wo;
	wire  input_is_nan_dffe13_wi;
	wire  input_is_nan_dffe13_wo;
	wire  input_is_nan_dffe14_wi;
	wire  input_is_nan_dffe14_wo;
	wire  input_is_nan_dffe15_wi;
	wire  input_is_nan_dffe15_wo;
	wire  input_is_nan_dffe1_wi;
	wire  input_is_nan_dffe1_wo;
	wire  input_is_nan_dffe21_wi;
	wire  input_is_nan_dffe21_wo;
	wire  input_is_nan_dffe22_wi;
	wire  input_is_nan_dffe22_wo;
	wire  input_is_nan_dffe23_wi;
	wire  input_is_nan_dffe23_wo;
	wire  input_is_nan_dffe25_wi;
	wire  input_is_nan_dffe25_wo;
	wire  input_is_nan_dffe26_wi;
	wire  input_is_nan_dffe26_wo;
	wire  input_is_nan_dffe27_wi;
	wire  input_is_nan_dffe27_wo;
	wire  input_is_nan_dffe2_wi;
	wire  input_is_nan_dffe2_wo;
	wire  input_is_nan_dffe31_wi;
	wire  input_is_nan_dffe31_wo;
	wire  input_is_nan_dffe32_wi;
	wire  input_is_nan_dffe32_wo;
	wire  input_is_nan_dffe33_wi;
	wire  input_is_nan_dffe33_wo;
	wire  input_is_nan_dffe3_wi;
	wire  input_is_nan_dffe3_wo;
	wire  input_is_nan_dffe41_wi;
	wire  input_is_nan_dffe41_wo;
	wire  input_is_nan_dffe42_wi;
	wire  input_is_nan_dffe42_wo;
	wire  input_is_nan_dffe4_wi;
	wire  input_is_nan_dffe4_wo;
	wire  [27:0]  man_2comp_res_dataa_w;
	wire  [27:0]  man_2comp_res_datab_w;
	wire  [27:0]  man_2comp_res_w;
	wire  [22:0]  man_a_not_zero_w;
	wire  [27:0]  man_add_sub_dataa_w;
	wire  [27:0]  man_add_sub_datab_w;
	wire  [25:0]  man_add_sub_res_mag_dffe21_wi;
	wire  [25:0]  man_add_sub_res_mag_dffe21_wo;
	wire  [25:0]  man_add_sub_res_mag_dffe23_wi;
	wire  [25:0]  man_add_sub_res_mag_dffe23_wo;
	wire  [25:0]  man_add_sub_res_mag_dffe26_wi;
	wire  [25:0]  man_add_sub_res_mag_dffe26_wo;
	wire  [27:0]  man_add_sub_res_mag_dffe27_wi;
	wire  [27:0]  man_add_sub_res_mag_dffe27_wo;
	wire  [27:0]  man_add_sub_res_mag_w2;
	wire  man_add_sub_res_sign_dffe21_wo;
	wire  man_add_sub_res_sign_dffe23_wi;
	wire  man_add_sub_res_sign_dffe23_wo;
	wire  man_add_sub_res_sign_dffe26_wi;
	wire  man_add_sub_res_sign_dffe26_wo;
	wire  man_add_sub_res_sign_dffe27_wi;
	wire  man_add_sub_res_sign_dffe27_wo;
	wire  man_add_sub_res_sign_w2;
	wire  [27:0]  man_add_sub_w;
	wire  [22:0]  man_all_zeros_w;
	wire  [22:0]  man_b_not_zero_w;
	wire  [25:0]  man_dffe31_wo;
	wire  [25:0]  man_intermediate_res_w;
	wire  [4:0]  man_leading_zeros_cnt_w;
	wire  [4:0]  man_leading_zeros_dffe31_wi;
	wire  [4:0]  man_leading_zeros_dffe31_wo;
	wire  [22:0]  man_nan_w;
	wire  [22:0]  man_out_dffe5_wi;
	wire  [22:0]  man_out_dffe5_wo;
	wire  [22:0]  man_res_dffe4_wi;
	wire  [22:0]  man_res_dffe4_wo;
	wire  man_res_is_not_zero_dffe31_wi;
	wire  man_res_is_not_zero_dffe31_wo;
	wire  man_res_is_not_zero_dffe32_wi;
	wire  man_res_is_not_zero_dffe32_wo;
	wire  man_res_is_not_zero_dffe33_wi;
	wire  man_res_is_not_zero_dffe33_wo;
	wire  man_res_is_not_zero_dffe3_wi;
	wire  man_res_is_not_zero_dffe3_wo;
	wire  man_res_is_not_zero_dffe41_wi;
	wire  man_res_is_not_zero_dffe41_wo;
	wire  man_res_is_not_zero_dffe42_wi;
	wire  man_res_is_not_zero_dffe42_wo;
	wire  man_res_is_not_zero_dffe4_wi;
	wire  man_res_is_not_zero_dffe4_wo;
	wire  [25:0]  man_res_mag_w2;
	wire  man_res_not_zero_dffe23_wi;
	wire  man_res_not_zero_dffe23_wo;
	wire  man_res_not_zero_dffe26_wi;
	wire  man_res_not_zero_dffe26_wo;
	wire  [24:0]  man_res_not_zero_w2;
	wire  [25:0]  man_res_rounding_add_sub_datab_w;
	wire  [25:0]  man_res_rounding_add_sub_w;
	wire  [23:0]  man_res_w3;
	wire  [22:0]  man_rounded_res_w;
	wire  man_rounding_add_value_w;
	wire  [23:0]  man_smaller_dffe13_wi;
	wire  [23:0]  man_smaller_dffe13_wo;
	wire  [23:0]  man_smaller_w;
	wire  need_complement_dffe22_wi;
	wire  need_complement_dffe22_wo;
	wire  need_complement_dffe2_wi;
	wire  need_complement_dffe2_wo;
	wire  [1:0]  pos_sign_bit_ext;
	wire  [3:0]  priority_encoder_1pads_w;
	wire  round_bit_dffe21_wi;
	wire  round_bit_dffe21_wo;
	wire  round_bit_dffe23_wi;
	wire  round_bit_dffe23_wo;
	wire  round_bit_dffe26_wi;
	wire  round_bit_dffe26_wo;
	wire  round_bit_dffe31_wi;
	wire  round_bit_dffe31_wo;
	wire  round_bit_dffe32_wi;
	wire  round_bit_dffe32_wo;
	wire  round_bit_dffe33_wi;
	wire  round_bit_dffe33_wo;
	wire  round_bit_dffe3_wi;
	wire  round_bit_dffe3_wo;
	wire  round_bit_w;
	wire  rounded_res_infinity_dffe4_wi;
	wire  rounded_res_infinity_dffe4_wo;
	wire  [4:0]  rshift_distance_dffe13_wi;
	wire  [4:0]  rshift_distance_dffe13_wo;
	wire  [4:0]  rshift_distance_dffe14_wi;
	wire  [4:0]  rshift_distance_dffe14_wo;
	wire  [4:0]  rshift_distance_dffe15_wi;
	wire  [4:0]  rshift_distance_dffe15_wo;
	wire  [4:0]  rshift_distance_w;
	wire  sign_dffe31_wi;
	wire  sign_dffe31_wo;
	wire  sign_dffe32_wi;
	wire  sign_dffe32_wo;
	wire  sign_dffe33_wi;
	wire  sign_dffe33_wo;
	wire  sign_out_dffe5_wi;
	wire  sign_out_dffe5_wo;
	wire  sign_res_dffe3_wi;
	wire  sign_res_dffe3_wo;
	wire  sign_res_dffe41_wi;
	wire  sign_res_dffe41_wo;
	wire  sign_res_dffe42_wi;
	wire  sign_res_dffe42_wo;
	wire  sign_res_dffe4_wi;
	wire  sign_res_dffe4_wo;
	wire  [5:0]  sticky_bit_cnt_dataa_w;
	wire  [5:0]  sticky_bit_cnt_datab_w;
	wire  [5:0]  sticky_bit_cnt_res_w;
	wire  sticky_bit_dffe1_wi;
	wire  sticky_bit_dffe1_wo;
	wire  sticky_bit_dffe21_wi;
	wire  sticky_bit_dffe21_wo;
	wire  sticky_bit_dffe22_wi;
	wire  sticky_bit_dffe22_wo;
	wire  sticky_bit_dffe23_wi;
	wire  sticky_bit_dffe23_wo;
	wire  sticky_bit_dffe25_wi;
	wire  sticky_bit_dffe25_wo;
	wire  sticky_bit_dffe26_wi;
	wire  sticky_bit_dffe26_wo;
	wire  sticky_bit_dffe27_wi;
	wire  sticky_bit_dffe27_wo;
	wire  sticky_bit_dffe2_wi;
	wire  sticky_bit_dffe2_wo;
	wire  sticky_bit_dffe31_wi;
	wire  sticky_bit_dffe31_wo;
	wire  sticky_bit_dffe32_wi;
	wire  sticky_bit_dffe32_wo;
	wire  sticky_bit_dffe33_wi;
	wire  sticky_bit_dffe33_wo;
	wire  sticky_bit_dffe3_wi;
	wire  sticky_bit_dffe3_wo;
	wire  sticky_bit_w;
	wire  [5:0]  trailing_zeros_limit_w;
	wire  zero_man_sign_dffe21_wi;
	wire  zero_man_sign_dffe21_wo;
	wire  zero_man_sign_dffe22_wi;
	wire  zero_man_sign_dffe22_wo;
	wire  zero_man_sign_dffe23_wi;
	wire  zero_man_sign_dffe23_wo;
	wire  zero_man_sign_dffe26_wi;
	wire  zero_man_sign_dffe26_wo;
	wire  zero_man_sign_dffe27_wi;
	wire  zero_man_sign_dffe27_wo;
	wire  zero_man_sign_dffe2_wi;
	wire  zero_man_sign_dffe2_wo;

	fpoint_qsys_addsub_single_altbarrel_shift_fjg   lbarrel_shift
	( 
	.aclr(aclr),
	.clk_en(clk_en),
	.clock(clock),
	.data(man_dffe31_wo),
	.distance(man_leading_zeros_cnt_w),
	.result(wire_lbarrel_shift_result));
	fpoint_qsys_addsub_single_altbarrel_shift_44e   rbarrel_shift
	( 
	.data({man_smaller_dffe13_wo, {2{1'b0}}}),
	.distance(rshift_distance_dffe13_wo),
	.result(wire_rbarrel_shift_result));
	fpoint_qsys_addsub_single_altpriority_encoder_9u8   leading_zeroes_cnt
	( 
	.data({man_add_sub_res_mag_dffe21_wo[25:1], 1'b1, {6{1'b0}}}),
	.q(wire_leading_zeroes_cnt_q));
	fpoint_qsys_addsub_single_altpriority_encoder_tma   trailing_zeros_cnt
	( 
	.data({{9{1'b1}}, man_smaller_dffe13_wo[22:0]}),
	.q(wire_trailing_zeros_cnt_q));
	// synopsys translate_off
	initial
		add_sub_dffe1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) add_sub_dffe1 <= 1'b0;
		else if  (clk_en == 1'b1)   add_sub_dffe1 <= add_sub_dffe1_wi;
	// synopsys translate_off
	initial
		add_sub_dffe12 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) add_sub_dffe12 <= 1'b0;
		else if  (clk_en == 1'b1)   add_sub_dffe12 <= add_sub_dffe12_wi;
	// synopsys translate_off
	initial
		aligned_dataa_exp_dffe12 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) aligned_dataa_exp_dffe12 <= 9'b0;
		else if  (clk_en == 1'b1)   aligned_dataa_exp_dffe12 <= aligned_dataa_exp_dffe12_wi;
	// synopsys translate_off
	initial
		aligned_dataa_man_dffe12 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) aligned_dataa_man_dffe12 <= 24'b0;
		else if  (clk_en == 1'b1)   aligned_dataa_man_dffe12 <= aligned_dataa_man_dffe12_wi;
	// synopsys translate_off
	initial
		aligned_dataa_sign_dffe12 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) aligned_dataa_sign_dffe12 <= 1'b0;
		else if  (clk_en == 1'b1)   aligned_dataa_sign_dffe12 <= aligned_dataa_sign_dffe12_wi;
	// synopsys translate_off
	initial
		aligned_datab_exp_dffe12 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) aligned_datab_exp_dffe12 <= 9'b0;
		else if  (clk_en == 1'b1)   aligned_datab_exp_dffe12 <= aligned_datab_exp_dffe12_wi;
	// synopsys translate_off
	initial
		aligned_datab_man_dffe12 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) aligned_datab_man_dffe12 <= 24'b0;
		else if  (clk_en == 1'b1)   aligned_datab_man_dffe12 <= aligned_datab_man_dffe12_wi;
	// synopsys translate_off
	initial
		aligned_datab_sign_dffe12 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) aligned_datab_sign_dffe12 <= 1'b0;
		else if  (clk_en == 1'b1)   aligned_datab_sign_dffe12 <= aligned_datab_sign_dffe12_wi;
	// synopsys translate_off
	initial
		both_inputs_are_infinite_dffe1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) both_inputs_are_infinite_dffe1 <= 1'b0;
		else if  (clk_en == 1'b1)   both_inputs_are_infinite_dffe1 <= both_inputs_are_infinite_dffe1_wi;
	// synopsys translate_off
	initial
		data_exp_dffe1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) data_exp_dffe1 <= 8'b0;
		else if  (clk_en == 1'b1)   data_exp_dffe1 <= data_exp_dffe1_wi;
	// synopsys translate_off
	initial
		dataa_man_dffe1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_man_dffe1 <= 26'b0;
		else if  (clk_en == 1'b1)   dataa_man_dffe1 <= dataa_man_dffe1_wi;
	// synopsys translate_off
	initial
		dataa_sign_dffe1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) dataa_sign_dffe1 <= 1'b0;
		else if  (clk_en == 1'b1)   dataa_sign_dffe1 <= dataa_sign_dffe1_wi;
	// synopsys translate_off
	initial
		datab_man_dffe1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_man_dffe1 <= 26'b0;
		else if  (clk_en == 1'b1)   datab_man_dffe1 <= datab_man_dffe1_wi;
	// synopsys translate_off
	initial
		datab_sign_dffe1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) datab_sign_dffe1 <= 1'b0;
		else if  (clk_en == 1'b1)   datab_sign_dffe1 <= datab_sign_dffe1_wi;
	// synopsys translate_off
	initial
		denormal_res_dffe3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) denormal_res_dffe3 <= 1'b0;
		else if  (clk_en == 1'b1)   denormal_res_dffe3 <= denormal_res_dffe3_wi;
	// synopsys translate_off
	initial
		denormal_res_dffe4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) denormal_res_dffe4 <= 1'b0;
		else if  (clk_en == 1'b1)   denormal_res_dffe4 <= denormal_res_dffe4_wi;
	// synopsys translate_off
	initial
		exp_adj_dffe21 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_adj_dffe21 <= 2'b0;
		else if  (clk_en == 1'b1)   exp_adj_dffe21 <= exp_adj_dffe21_wi;
	// synopsys translate_off
	initial
		exp_out_dffe5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_out_dffe5 <= 8'b0;
		else if  (clk_en == 1'b1)   exp_out_dffe5 <= exp_out_dffe5_wi;
	// synopsys translate_off
	initial
		exp_res_dffe2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_res_dffe2 <= 8'b0;
		else if  (clk_en == 1'b1)   exp_res_dffe2 <= exp_res_dffe2_wi;
	// synopsys translate_off
	initial
		exp_res_dffe21 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_res_dffe21 <= 8'b0;
		else if  (clk_en == 1'b1)   exp_res_dffe21 <= exp_res_dffe21_wi;
	// synopsys translate_off
	initial
		exp_res_dffe3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_res_dffe3 <= 8'b0;
		else if  (clk_en == 1'b1)   exp_res_dffe3 <= exp_res_dffe3_wi;
	// synopsys translate_off
	initial
		exp_res_dffe4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) exp_res_dffe4 <= 8'b0;
		else if  (clk_en == 1'b1)   exp_res_dffe4 <= exp_res_dffe4_wi;
	// synopsys translate_off
	initial
		infinite_output_sign_dffe1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) infinite_output_sign_dffe1 <= 1'b0;
		else if  (clk_en == 1'b1)   infinite_output_sign_dffe1 <= infinite_output_sign_dffe1_wi;
	// synopsys translate_off
	initial
		infinite_output_sign_dffe2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) infinite_output_sign_dffe2 <= 1'b0;
		else if  (clk_en == 1'b1)   infinite_output_sign_dffe2 <= infinite_output_sign_dffe2_wi;
	// synopsys translate_off
	initial
		infinite_output_sign_dffe21 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) infinite_output_sign_dffe21 <= 1'b0;
		else if  (clk_en == 1'b1)   infinite_output_sign_dffe21 <= infinite_output_sign_dffe21_wi;
	// synopsys translate_off
	initial
		infinite_output_sign_dffe3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) infinite_output_sign_dffe3 <= 1'b0;
		else if  (clk_en == 1'b1)   infinite_output_sign_dffe3 <= infinite_output_sign_dffe3_wi;
	// synopsys translate_off
	initial
		infinite_output_sign_dffe31 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) infinite_output_sign_dffe31 <= 1'b0;
		else if  (clk_en == 1'b1)   infinite_output_sign_dffe31 <= infinite_output_sign_dffe31_wi;
	// synopsys translate_off
	initial
		infinite_output_sign_dffe4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) infinite_output_sign_dffe4 <= 1'b0;
		else if  (clk_en == 1'b1)   infinite_output_sign_dffe4 <= infinite_output_sign_dffe4_wi;
	// synopsys translate_off
	initial
		infinite_res_dffe3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) infinite_res_dffe3 <= 1'b0;
		else if  (clk_en == 1'b1)   infinite_res_dffe3 <= infinite_res_dffe3_wi;
	// synopsys translate_off
	initial
		infinite_res_dffe4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) infinite_res_dffe4 <= 1'b0;
		else if  (clk_en == 1'b1)   infinite_res_dffe4 <= infinite_res_dffe4_wi;
	// synopsys translate_off
	initial
		infinity_magnitude_sub_dffe2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) infinity_magnitude_sub_dffe2 <= 1'b0;
		else if  (clk_en == 1'b1)   infinity_magnitude_sub_dffe2 <= infinity_magnitude_sub_dffe2_wi;
	// synopsys translate_off
	initial
		infinity_magnitude_sub_dffe21 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) infinity_magnitude_sub_dffe21 <= 1'b0;
		else if  (clk_en == 1'b1)   infinity_magnitude_sub_dffe21 <= infinity_magnitude_sub_dffe21_wi;
	// synopsys translate_off
	initial
		infinity_magnitude_sub_dffe3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) infinity_magnitude_sub_dffe3 <= 1'b0;
		else if  (clk_en == 1'b1)   infinity_magnitude_sub_dffe3 <= infinity_magnitude_sub_dffe3_wi;
	// synopsys translate_off
	initial
		infinity_magnitude_sub_dffe31 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) infinity_magnitude_sub_dffe31 <= 1'b0;
		else if  (clk_en == 1'b1)   infinity_magnitude_sub_dffe31 <= infinity_magnitude_sub_dffe31_wi;
	// synopsys translate_off
	initial
		infinity_magnitude_sub_dffe4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) infinity_magnitude_sub_dffe4 <= 1'b0;
		else if  (clk_en == 1'b1)   infinity_magnitude_sub_dffe4 <= infinity_magnitude_sub_dffe4_wi;
	// synopsys translate_off
	initial
		input_dataa_infinite_dffe12 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_dataa_infinite_dffe12 <= 1'b0;
		else if  (clk_en == 1'b1)   input_dataa_infinite_dffe12 <= input_dataa_infinite_dffe12_wi;
	// synopsys translate_off
	initial
		input_dataa_nan_dffe12 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_dataa_nan_dffe12 <= 1'b0;
		else if  (clk_en == 1'b1)   input_dataa_nan_dffe12 <= input_dataa_nan_dffe12_wi;
	// synopsys translate_off
	initial
		input_datab_infinite_dffe12 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_datab_infinite_dffe12 <= 1'b0;
		else if  (clk_en == 1'b1)   input_datab_infinite_dffe12 <= input_datab_infinite_dffe12_wi;
	// synopsys translate_off
	initial
		input_datab_nan_dffe12 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_datab_nan_dffe12 <= 1'b0;
		else if  (clk_en == 1'b1)   input_datab_nan_dffe12 <= input_datab_nan_dffe12_wi;
	// synopsys translate_off
	initial
		input_is_infinite_dffe1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinite_dffe1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinite_dffe1 <= input_is_infinite_dffe1_wi;
	// synopsys translate_off
	initial
		input_is_infinite_dffe2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinite_dffe2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinite_dffe2 <= input_is_infinite_dffe2_wi;
	// synopsys translate_off
	initial
		input_is_infinite_dffe21 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinite_dffe21 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinite_dffe21 <= input_is_infinite_dffe21_wi;
	// synopsys translate_off
	initial
		input_is_infinite_dffe3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinite_dffe3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinite_dffe3 <= input_is_infinite_dffe3_wi;
	// synopsys translate_off
	initial
		input_is_infinite_dffe31 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinite_dffe31 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinite_dffe31 <= input_is_infinite_dffe31_wi;
	// synopsys translate_off
	initial
		input_is_infinite_dffe4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_infinite_dffe4 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_infinite_dffe4 <= input_is_infinite_dffe4_wi;
	// synopsys translate_off
	initial
		input_is_nan_dffe1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe1 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe1 <= input_is_nan_dffe1_wi;
	// synopsys translate_off
	initial
		input_is_nan_dffe2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe2 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe2 <= input_is_nan_dffe2_wi;
	// synopsys translate_off
	initial
		input_is_nan_dffe21 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe21 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe21 <= input_is_nan_dffe21_wi;
	// synopsys translate_off
	initial
		input_is_nan_dffe3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe3 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe3 <= input_is_nan_dffe3_wi;
	// synopsys translate_off
	initial
		input_is_nan_dffe31 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe31 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe31 <= input_is_nan_dffe31_wi;
	// synopsys translate_off
	initial
		input_is_nan_dffe4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) input_is_nan_dffe4 <= 1'b0;
		else if  (clk_en == 1'b1)   input_is_nan_dffe4 <= input_is_nan_dffe4_wi;
	// synopsys translate_off
	initial
		man_add_sub_res_mag_dffe21 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_add_sub_res_mag_dffe21 <= 26'b0;
		else if  (clk_en == 1'b1)   man_add_sub_res_mag_dffe21 <= man_add_sub_res_mag_dffe21_wi;
	// synopsys translate_off
	initial
		man_add_sub_res_sign_dffe21 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_add_sub_res_sign_dffe21 <= 1'b0;
		else if  (clk_en == 1'b1)   man_add_sub_res_sign_dffe21 <= man_add_sub_res_sign_dffe27_wo;
	// synopsys translate_off
	initial
		man_dffe31 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_dffe31 <= 26'b0;
		else if  (clk_en == 1'b1)   man_dffe31 <= man_add_sub_res_mag_dffe26_wo;
	// synopsys translate_off
	initial
		man_leading_zeros_dffe31 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_leading_zeros_dffe31 <= 5'b0;
		else if  (clk_en == 1'b1)   man_leading_zeros_dffe31 <= man_leading_zeros_dffe31_wi;
	// synopsys translate_off
	initial
		man_out_dffe5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_out_dffe5 <= 23'b0;
		else if  (clk_en == 1'b1)   man_out_dffe5 <= man_out_dffe5_wi;
	// synopsys translate_off
	initial
		man_res_dffe4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_res_dffe4 <= 23'b0;
		else if  (clk_en == 1'b1)   man_res_dffe4 <= man_res_dffe4_wi;
	// synopsys translate_off
	initial
		man_res_is_not_zero_dffe3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_res_is_not_zero_dffe3 <= 1'b0;
		else if  (clk_en == 1'b1)   man_res_is_not_zero_dffe3 <= man_res_is_not_zero_dffe3_wi;
	// synopsys translate_off
	initial
		man_res_is_not_zero_dffe31 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_res_is_not_zero_dffe31 <= 1'b0;
		else if  (clk_en == 1'b1)   man_res_is_not_zero_dffe31 <= man_res_is_not_zero_dffe31_wi;
	// synopsys translate_off
	initial
		man_res_is_not_zero_dffe4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) man_res_is_not_zero_dffe4 <= 1'b0;
		else if  (clk_en == 1'b1)   man_res_is_not_zero_dffe4 <= man_res_is_not_zero_dffe4_wi;
	// synopsys translate_off
	initial
		need_complement_dffe2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) need_complement_dffe2 <= 1'b0;
		else if  (clk_en == 1'b1)   need_complement_dffe2 <= need_complement_dffe2_wi;
	// synopsys translate_off
	initial
		round_bit_dffe21 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) round_bit_dffe21 <= 1'b0;
		else if  (clk_en == 1'b1)   round_bit_dffe21 <= round_bit_dffe21_wi;
	// synopsys translate_off
	initial
		round_bit_dffe3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) round_bit_dffe3 <= 1'b0;
		else if  (clk_en == 1'b1)   round_bit_dffe3 <= round_bit_dffe3_wi;
	// synopsys translate_off
	initial
		round_bit_dffe31 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) round_bit_dffe31 <= 1'b0;
		else if  (clk_en == 1'b1)   round_bit_dffe31 <= round_bit_dffe31_wi;
	// synopsys translate_off
	initial
		rounded_res_infinity_dffe4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) rounded_res_infinity_dffe4 <= 1'b0;
		else if  (clk_en == 1'b1)   rounded_res_infinity_dffe4 <= rounded_res_infinity_dffe4_wi;
	// synopsys translate_off
	initial
		sign_dffe31 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_dffe31 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_dffe31 <= sign_dffe31_wi;
	// synopsys translate_off
	initial
		sign_out_dffe5 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_out_dffe5 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_out_dffe5 <= sign_out_dffe5_wi;
	// synopsys translate_off
	initial
		sign_res_dffe3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_res_dffe3 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_res_dffe3 <= sign_res_dffe3_wi;
	// synopsys translate_off
	initial
		sign_res_dffe4 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sign_res_dffe4 <= 1'b0;
		else if  (clk_en == 1'b1)   sign_res_dffe4 <= sign_res_dffe4_wi;
	// synopsys translate_off
	initial
		sticky_bit_dffe1 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sticky_bit_dffe1 <= 1'b0;
		else if  (clk_en == 1'b1)   sticky_bit_dffe1 <= sticky_bit_dffe1_wi;
	// synopsys translate_off
	initial
		sticky_bit_dffe2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sticky_bit_dffe2 <= 1'b0;
		else if  (clk_en == 1'b1)   sticky_bit_dffe2 <= sticky_bit_dffe2_wi;
	// synopsys translate_off
	initial
		sticky_bit_dffe21 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sticky_bit_dffe21 <= 1'b0;
		else if  (clk_en == 1'b1)   sticky_bit_dffe21 <= sticky_bit_dffe21_wi;
	// synopsys translate_off
	initial
		sticky_bit_dffe3 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sticky_bit_dffe3 <= 1'b0;
		else if  (clk_en == 1'b1)   sticky_bit_dffe3 <= sticky_bit_dffe3_wi;
	// synopsys translate_off
	initial
		sticky_bit_dffe31 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) sticky_bit_dffe31 <= 1'b0;
		else if  (clk_en == 1'b1)   sticky_bit_dffe31 <= sticky_bit_dffe31_wi;
	// synopsys translate_off
	initial
		zero_man_sign_dffe2 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) zero_man_sign_dffe2 <= 1'b0;
		else if  (clk_en == 1'b1)   zero_man_sign_dffe2 <= zero_man_sign_dffe2_wi;
	// synopsys translate_off
	initial
		zero_man_sign_dffe21 = 0;
	// synopsys translate_on
	always @ ( posedge clock or  posedge aclr)
		if (aclr == 1'b1) zero_man_sign_dffe21 <= 1'b0;
		else if  (clk_en == 1'b1)   zero_man_sign_dffe21 <= zero_man_sign_dffe21_wi;
	lpm_add_sub   add_sub1
	( 
	.aclr(aclr),
	.clken(clk_en),
	.clock(clock),
	.cout(),
	.dataa(aligned_dataa_exp_w),
	.datab(aligned_datab_exp_w),
	.overflow(),
	.result(wire_add_sub1_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.add_sub(1'b1),
	.cin()
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		add_sub1.lpm_direction = "SUB",
		add_sub1.lpm_pipeline = 1,
		add_sub1.lpm_representation = "SIGNED",
		add_sub1.lpm_width = 9,
		add_sub1.lpm_type = "lpm_add_sub";
	lpm_add_sub   add_sub2
	( 
	.aclr(aclr),
	.clken(clk_en),
	.clock(clock),
	.cout(),
	.dataa(aligned_datab_exp_w),
	.datab(aligned_dataa_exp_w),
	.overflow(),
	.result(wire_add_sub2_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.add_sub(1'b1),
	.cin()
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		add_sub2.lpm_direction = "SUB",
		add_sub2.lpm_pipeline = 1,
		add_sub2.lpm_representation = "SIGNED",
		add_sub2.lpm_width = 9,
		add_sub2.lpm_type = "lpm_add_sub";
	lpm_add_sub   add_sub3
	( 
	.cout(),
	.dataa(sticky_bit_cnt_dataa_w),
	.datab(sticky_bit_cnt_datab_w),
	.overflow(),
	.result(wire_add_sub3_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.cin(),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		add_sub3.lpm_direction = "SUB",
		add_sub3.lpm_representation = "SIGNED",
		add_sub3.lpm_width = 6,
		add_sub3.lpm_type = "lpm_add_sub";
	lpm_add_sub   add_sub4
	( 
	.cout(),
	.dataa(exp_adjustment_add_sub_dataa_w),
	.datab(exp_adjustment_add_sub_datab_w),
	.overflow(),
	.result(wire_add_sub4_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.cin(),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		add_sub4.lpm_direction = "ADD",
		add_sub4.lpm_representation = "SIGNED",
		add_sub4.lpm_width = 9,
		add_sub4.lpm_type = "lpm_add_sub";
	lpm_add_sub   add_sub5
	( 
	.aclr(aclr),
	.clken(clk_en),
	.clock(clock),
	.cout(),
	.dataa(exp_adjustment2_add_sub_dataa_w),
	.datab(exp_adjustment2_add_sub_datab_w),
	.overflow(),
	.result(wire_add_sub5_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.add_sub(1'b1),
	.cin()
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		add_sub5.lpm_direction = "ADD",
		add_sub5.lpm_pipeline = 1,
		add_sub5.lpm_representation = "SIGNED",
		add_sub5.lpm_width = 9,
		add_sub5.lpm_type = "lpm_add_sub";
	lpm_add_sub   add_sub6
	( 
	.cout(),
	.dataa(exp_res_rounding_adder_dataa_w),
	.datab(exp_rounding_adjustment_w),
	.overflow(),
	.result(wire_add_sub6_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.cin(),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		add_sub6.lpm_direction = "ADD",
		add_sub6.lpm_representation = "SIGNED",
		add_sub6.lpm_width = 9,
		add_sub6.lpm_type = "lpm_add_sub";
	lpm_add_sub   man_2comp_res_lower
	( 
	.aclr(aclr),
	.add_sub(add_sub_w2),
	.cin(borrow_w),
	.clken(clk_en),
	.clock(clock),
	.cout(wire_man_2comp_res_lower_cout),
	.dataa(man_2comp_res_dataa_w[13:0]),
	.datab(man_2comp_res_datab_w[13:0]),
	.overflow(),
	.result(wire_man_2comp_res_lower_result));
	defparam
		man_2comp_res_lower.lpm_pipeline = 1,
		man_2comp_res_lower.lpm_representation = "SIGNED",
		man_2comp_res_lower.lpm_width = 14,
		man_2comp_res_lower.lpm_type = "lpm_add_sub";
	lpm_add_sub   man_2comp_res_upper0
	( 
	.aclr(aclr),
	.add_sub(add_sub_w2),
	.cin(1'b0),
	.clken(clk_en),
	.clock(clock),
	.cout(),
	.dataa(man_2comp_res_dataa_w[27:14]),
	.datab(man_2comp_res_datab_w[27:14]),
	.overflow(),
	.result(wire_man_2comp_res_upper0_result));
	defparam
		man_2comp_res_upper0.lpm_pipeline = 1,
		man_2comp_res_upper0.lpm_representation = "SIGNED",
		man_2comp_res_upper0.lpm_width = 14,
		man_2comp_res_upper0.lpm_type = "lpm_add_sub";
	lpm_add_sub   man_2comp_res_upper1
	( 
	.aclr(aclr),
	.add_sub(add_sub_w2),
	.cin(1'b1),
	.clken(clk_en),
	.clock(clock),
	.cout(),
	.dataa(man_2comp_res_dataa_w[27:14]),
	.datab(man_2comp_res_datab_w[27:14]),
	.overflow(),
	.result(wire_man_2comp_res_upper1_result));
	defparam
		man_2comp_res_upper1.lpm_pipeline = 1,
		man_2comp_res_upper1.lpm_representation = "SIGNED",
		man_2comp_res_upper1.lpm_width = 14,
		man_2comp_res_upper1.lpm_type = "lpm_add_sub";
	lpm_add_sub   man_add_sub_lower
	( 
	.aclr(aclr),
	.add_sub(add_sub_w2),
	.cin(borrow_w),
	.clken(clk_en),
	.clock(clock),
	.cout(wire_man_add_sub_lower_cout),
	.dataa(man_add_sub_dataa_w[13:0]),
	.datab(man_add_sub_datab_w[13:0]),
	.overflow(),
	.result(wire_man_add_sub_lower_result));
	defparam
		man_add_sub_lower.lpm_pipeline = 1,
		man_add_sub_lower.lpm_representation = "SIGNED",
		man_add_sub_lower.lpm_width = 14,
		man_add_sub_lower.lpm_type = "lpm_add_sub";
	lpm_add_sub   man_add_sub_upper0
	( 
	.aclr(aclr),
	.add_sub(add_sub_w2),
	.cin(1'b0),
	.clken(clk_en),
	.clock(clock),
	.cout(),
	.dataa(man_add_sub_dataa_w[27:14]),
	.datab(man_add_sub_datab_w[27:14]),
	.overflow(),
	.result(wire_man_add_sub_upper0_result));
	defparam
		man_add_sub_upper0.lpm_pipeline = 1,
		man_add_sub_upper0.lpm_representation = "SIGNED",
		man_add_sub_upper0.lpm_width = 14,
		man_add_sub_upper0.lpm_type = "lpm_add_sub";
	lpm_add_sub   man_add_sub_upper1
	( 
	.aclr(aclr),
	.add_sub(add_sub_w2),
	.cin(1'b1),
	.clken(clk_en),
	.clock(clock),
	.cout(),
	.dataa(man_add_sub_dataa_w[27:14]),
	.datab(man_add_sub_datab_w[27:14]),
	.overflow(),
	.result(wire_man_add_sub_upper1_result));
	defparam
		man_add_sub_upper1.lpm_pipeline = 1,
		man_add_sub_upper1.lpm_representation = "SIGNED",
		man_add_sub_upper1.lpm_width = 14,
		man_add_sub_upper1.lpm_type = "lpm_add_sub";
	lpm_add_sub   man_res_rounding_add_sub_lower
	( 
	.cout(wire_man_res_rounding_add_sub_lower_cout),
	.dataa(man_intermediate_res_w[12:0]),
	.datab(man_res_rounding_add_sub_datab_w[12:0]),
	.overflow(),
	.result(wire_man_res_rounding_add_sub_lower_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.cin(),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		man_res_rounding_add_sub_lower.lpm_direction = "ADD",
		man_res_rounding_add_sub_lower.lpm_representation = "SIGNED",
		man_res_rounding_add_sub_lower.lpm_width = 13,
		man_res_rounding_add_sub_lower.lpm_type = "lpm_add_sub";
	lpm_add_sub   man_res_rounding_add_sub_upper1
	( 
	.cin(1'b1),
	.cout(),
	.dataa(man_intermediate_res_w[25:13]),
	.datab(man_res_rounding_add_sub_datab_w[25:13]),
	.overflow(),
	.result(wire_man_res_rounding_add_sub_upper1_result)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.add_sub(1'b1),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		man_res_rounding_add_sub_upper1.lpm_direction = "ADD",
		man_res_rounding_add_sub_upper1.lpm_representation = "SIGNED",
		man_res_rounding_add_sub_upper1.lpm_width = 13,
		man_res_rounding_add_sub_upper1.lpm_type = "lpm_add_sub";
	lpm_compare   trailing_zeros_limit_comparator
	( 
	.aeb(),
	.agb(wire_trailing_zeros_limit_comparator_agb),
	.ageb(),
	.alb(),
	.aleb(),
	.aneb(),
	.dataa(sticky_bit_cnt_res_w),
	.datab(trailing_zeros_limit_w)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_off
	`endif
	,
	.aclr(1'b0),
	.clken(1'b1),
	.clock(1'b0)
	`ifndef FORMAL_VERIFICATION
	// synopsys translate_on
	`endif
	);
	defparam
		trailing_zeros_limit_comparator.lpm_representation = "SIGNED",
		trailing_zeros_limit_comparator.lpm_width = 6,
		trailing_zeros_limit_comparator.lpm_type = "lpm_compare";
	assign
		add_sub_dffe11_wi = add_sub,
		add_sub_dffe11_wo = add_sub_dffe11_wi,
		add_sub_dffe12_wi = add_sub_dffe11_wo,
		add_sub_dffe12_wo = add_sub_dffe12,
		add_sub_dffe13_wi = add_sub_dffe12_wo,
		add_sub_dffe13_wo = add_sub_dffe13_wi,
		add_sub_dffe14_wi = add_sub_dffe13_wo,
		add_sub_dffe14_wo = add_sub_dffe14_wi,
		add_sub_dffe15_wi = add_sub_dffe14_wo,
		add_sub_dffe15_wo = add_sub_dffe15_wi,
		add_sub_dffe1_wi = add_sub_dffe15_wo,
		add_sub_dffe1_wo = add_sub_dffe1,
		add_sub_dffe25_wi = add_sub_w2,
		add_sub_dffe25_wo = add_sub_dffe25_wi,
		add_sub_w2 = (((((dataa_sign_dffe1_wo & (~ datab_sign_dffe1_wo)) & (~ add_sub_dffe1_wo)) | (((~ dataa_sign_dffe1_wo) & (~ datab_sign_dffe1_wo)) & add_sub_dffe1_wo)) | (((~ dataa_sign_dffe1_wo) & datab_sign_dffe1_wo) & (~ add_sub_dffe1_wo))) | ((dataa_sign_dffe1_wo & datab_sign_dffe1_wo) & add_sub_dffe1_wo)),
		adder_upper_w = man_intermediate_res_w[25:13],
		aligned_dataa_exp_dffe12_wi = aligned_dataa_exp_w,
		aligned_dataa_exp_dffe12_wo = aligned_dataa_exp_dffe12,
		aligned_dataa_exp_dffe13_wi = aligned_dataa_exp_dffe12_wo,
		aligned_dataa_exp_dffe13_wo = aligned_dataa_exp_dffe13_wi,
		aligned_dataa_exp_dffe14_wi = aligned_dataa_exp_dffe13_wo,
		aligned_dataa_exp_dffe14_wo = aligned_dataa_exp_dffe14_wi,
		aligned_dataa_exp_dffe15_wi = aligned_dataa_exp_dffe14_wo,
		aligned_dataa_exp_dffe15_wo = aligned_dataa_exp_dffe15_wi,
		aligned_dataa_exp_w = {1'b0, ({8{(~ input_dataa_denormal_dffe11_wo)}} & dataa_dffe11_wo[30:23])},
		aligned_dataa_man_dffe12_wi = aligned_dataa_man_w[25:2],
		aligned_dataa_man_dffe12_wo = aligned_dataa_man_dffe12,
		aligned_dataa_man_dffe13_wi = aligned_dataa_man_dffe12_wo,
		aligned_dataa_man_dffe13_wo = aligned_dataa_man_dffe13_wi,
		aligned_dataa_man_dffe14_wi = aligned_dataa_man_dffe13_wo,
		aligned_dataa_man_dffe14_wo = aligned_dataa_man_dffe14_wi,
		aligned_dataa_man_dffe15_w = {aligned_dataa_man_dffe15_wo, {2{1'b0}}},
		aligned_dataa_man_dffe15_wi = aligned_dataa_man_dffe14_wo,
		aligned_dataa_man_dffe15_wo = aligned_dataa_man_dffe15_wi,
		aligned_dataa_man_w = {(((~ input_dataa_infinite_dffe11_wo) & (~ input_dataa_denormal_dffe11_wo)) & (~ input_dataa_zero_dffe11_wo)), ({23{(~ input_dataa_denormal_dffe11_wo)}} & dataa_dffe11_wo[22:0]), {2{1'b0}}},
		aligned_dataa_sign_dffe12_wi = aligned_dataa_sign_w,
		aligned_dataa_sign_dffe12_wo = aligned_dataa_sign_dffe12,
		aligned_dataa_sign_dffe13_wi = aligned_dataa_sign_dffe12_wo,
		aligned_dataa_sign_dffe13_wo = aligned_dataa_sign_dffe13_wi,
		aligned_dataa_sign_dffe14_wi = aligned_dataa_sign_dffe13_wo,
		aligned_dataa_sign_dffe14_wo = aligned_dataa_sign_dffe14_wi,
		aligned_dataa_sign_dffe15_wi = aligned_dataa_sign_dffe14_wo,
		aligned_dataa_sign_dffe15_wo = aligned_dataa_sign_dffe15_wi,
		aligned_dataa_sign_w = dataa_dffe11_wo[31],
		aligned_datab_exp_dffe12_wi = aligned_datab_exp_w,
		aligned_datab_exp_dffe12_wo = aligned_datab_exp_dffe12,
		aligned_datab_exp_dffe13_wi = aligned_datab_exp_dffe12_wo,
		aligned_datab_exp_dffe13_wo = aligned_datab_exp_dffe13_wi,
		aligned_datab_exp_dffe14_wi = aligned_datab_exp_dffe13_wo,
		aligned_datab_exp_dffe14_wo = aligned_datab_exp_dffe14_wi,
		aligned_datab_exp_dffe15_wi = aligned_datab_exp_dffe14_wo,
		aligned_datab_exp_dffe15_wo = aligned_datab_exp_dffe15_wi,
		aligned_datab_exp_w = {1'b0, ({8{(~ input_datab_denormal_dffe11_wo)}} & datab_dffe11_wo[30:23])},
		aligned_datab_man_dffe12_wi = aligned_datab_man_w[25:2],
		aligned_datab_man_dffe12_wo = aligned_datab_man_dffe12,
		aligned_datab_man_dffe13_wi = aligned_datab_man_dffe12_wo,
		aligned_datab_man_dffe13_wo = aligned_datab_man_dffe13_wi,
		aligned_datab_man_dffe14_wi = aligned_datab_man_dffe13_wo,
		aligned_datab_man_dffe14_wo = aligned_datab_man_dffe14_wi,
		aligned_datab_man_dffe15_w = {aligned_datab_man_dffe15_wo, {2{1'b0}}},
		aligned_datab_man_dffe15_wi = aligned_datab_man_dffe14_wo,
		aligned_datab_man_dffe15_wo = aligned_datab_man_dffe15_wi,
		aligned_datab_man_w = {(((~ input_datab_infinite_dffe11_wo) & (~ input_datab_denormal_dffe11_wo)) & (~ input_datab_zero_dffe11_wo)), ({23{(~ input_datab_denormal_dffe11_wo)}} & datab_dffe11_wo[22:0]), {2{1'b0}}},
		aligned_datab_sign_dffe12_wi = aligned_datab_sign_w,
		aligned_datab_sign_dffe12_wo = aligned_datab_sign_dffe12,
		aligned_datab_sign_dffe13_wi = aligned_datab_sign_dffe12_wo,
		aligned_datab_sign_dffe13_wo = aligned_datab_sign_dffe13_wi,
		aligned_datab_sign_dffe14_wi = aligned_datab_sign_dffe13_wo,
		aligned_datab_sign_dffe14_wo = aligned_datab_sign_dffe14_wi,
		aligned_datab_sign_dffe15_wi = aligned_datab_sign_dffe14_wo,
		aligned_datab_sign_dffe15_wo = aligned_datab_sign_dffe15_wi,
		aligned_datab_sign_w = datab_dffe11_wo[31],
		borrow_w = ((~ sticky_bit_dffe1_wo) & (~ add_sub_w2)),
		both_inputs_are_infinite_dffe1_wi = (input_dataa_infinite_dffe15_wo & input_datab_infinite_dffe15_wo),
		both_inputs_are_infinite_dffe1_wo = both_inputs_are_infinite_dffe1,
		both_inputs_are_infinite_dffe25_wi = both_inputs_are_infinite_dffe1_wo,
		both_inputs_are_infinite_dffe25_wo = both_inputs_are_infinite_dffe25_wi,
		data_exp_dffe1_wi = (({8{(~ exp_amb_mux_dffe15_wo)}} & aligned_dataa_exp_dffe15_wo[7:0]) | ({8{exp_amb_mux_dffe15_wo}} & aligned_datab_exp_dffe15_wo[7:0])),
		data_exp_dffe1_wo = data_exp_dffe1,
		dataa_dffe11_wi = dataa,
		dataa_dffe11_wo = dataa_dffe11_wi,
		dataa_man_dffe1_wi = (({26{(~ exp_amb_mux_dffe15_wo)}} & aligned_dataa_man_dffe15_w) | ({26{exp_amb_mux_dffe15_wo}} & wire_rbarrel_shift_result)),
		dataa_man_dffe1_wo = dataa_man_dffe1,
		dataa_sign_dffe1_wi = aligned_dataa_sign_dffe15_wo,
		dataa_sign_dffe1_wo = dataa_sign_dffe1,
		dataa_sign_dffe25_wi = dataa_sign_dffe1_wo,
		dataa_sign_dffe25_wo = dataa_sign_dffe25_wi,
		datab_dffe11_wi = datab,
		datab_dffe11_wo = datab_dffe11_wi,
		datab_man_dffe1_wi = (({26{(~ exp_amb_mux_dffe15_wo)}} & wire_rbarrel_shift_result) | ({26{exp_amb_mux_dffe15_wo}} & aligned_datab_man_dffe15_w)),
		datab_man_dffe1_wo = datab_man_dffe1,
		datab_sign_dffe1_wi = aligned_datab_sign_dffe15_wo,
		datab_sign_dffe1_wo = datab_sign_dffe1,
		denormal_flag_w = ((((~ force_nan_w) & (~ force_infinity_w)) & (~ force_zero_w)) & denormal_res_dffe4_wo),
		denormal_res_dffe32_wi = denormal_result_w,
		denormal_res_dffe32_wo = denormal_res_dffe32_wi,
		denormal_res_dffe33_wi = denormal_res_dffe32_wo,
		denormal_res_dffe33_wo = denormal_res_dffe33_wi,
		denormal_res_dffe3_wi = denormal_res_dffe33_wo,
		denormal_res_dffe3_wo = denormal_res_dffe3,
		denormal_res_dffe41_wi = denormal_res_dffe42_wo,
		denormal_res_dffe41_wo = denormal_res_dffe41_wi,
		denormal_res_dffe42_wi = denormal_res_dffe3_wo,
		denormal_res_dffe42_wo = denormal_res_dffe42_wi,
		denormal_res_dffe4_wi = denormal_res_dffe41_wo,
		denormal_res_dffe4_wo = denormal_res_dffe4,
		denormal_result_w = ((~ exp_res_not_zero_w[8]) | exp_adjustment2_add_sub_w[8]),
		exp_a_all_one_w = {(dataa[30] & exp_a_all_one_w[6]), (dataa[29] & exp_a_all_one_w[5]), (dataa[28] & exp_a_all_one_w[4]), (dataa[27] & exp_a_all_one_w[3]), (dataa[26] & exp_a_all_one_w[2]), (dataa[25] & exp_a_all_one_w[1]), (dataa[24] & exp_a_all_one_w[0]), dataa[23]},
		exp_a_not_zero_w = {(dataa[30] | exp_a_not_zero_w[6]), (dataa[29] | exp_a_not_zero_w[5]), (dataa[28] | exp_a_not_zero_w[4]), (dataa[27] | exp_a_not_zero_w[3]), (dataa[26] | exp_a_not_zero_w[2]), (dataa[25] | exp_a_not_zero_w[1]), (dataa[24] | exp_a_not_zero_w[0]), dataa[23]},
		exp_adj_0pads = {7{1'b0}},
		exp_adj_dffe21_wi = (({2{man_add_sub_res_mag_dffe27_wo[26]}} & exp_adjust_by_add2) | ({2{(~ man_add_sub_res_mag_dffe27_wo[26])}} & exp_adjust_by_add1)),
		exp_adj_dffe21_wo = exp_adj_dffe21,
		exp_adj_dffe23_wi = exp_adj_dffe21_wo,
		exp_adj_dffe23_wo = exp_adj_dffe23_wi,
		exp_adj_dffe26_wi = exp_adj_dffe23_wo,
		exp_adj_dffe26_wo = exp_adj_dffe26_wi,
		exp_adjust_by_add1 = 2'b01,
		exp_adjust_by_add2 = 2'b10,
		exp_adjustment2_add_sub_dataa_w = exp_value,
		exp_adjustment2_add_sub_datab_w = exp_adjustment_add_sub_w,
		exp_adjustment2_add_sub_w = wire_add_sub5_result,
		exp_adjustment_add_sub_dataa_w = {priority_encoder_1pads_w, wire_leading_zeroes_cnt_q},
		exp_adjustment_add_sub_datab_w = {exp_adj_0pads, exp_adj_dffe26_wo},
		exp_adjustment_add_sub_w = wire_add_sub4_result,
		exp_all_ones_w = {8{1'b1}},
		exp_all_zeros_w = {8{1'b0}},
		exp_amb_mux_dffe13_wi = exp_amb_mux_w,
		exp_amb_mux_dffe13_wo = exp_amb_mux_dffe13_wi,
		exp_amb_mux_dffe14_wi = exp_amb_mux_dffe13_wo,
		exp_amb_mux_dffe14_wo = exp_amb_mux_dffe14_wi,
		exp_amb_mux_dffe15_wi = exp_amb_mux_dffe14_wo,
		exp_amb_mux_dffe15_wo = exp_amb_mux_dffe15_wi,
		exp_amb_mux_w = exp_amb_w[8],
		exp_amb_w = wire_add_sub1_result,
		exp_b_all_one_w = {(datab[30] & exp_b_all_one_w[6]), (datab[29] & exp_b_all_one_w[5]), (datab[28] & exp_b_all_one_w[4]), (datab[27] & exp_b_all_one_w[3]), (datab[26] & exp_b_all_one_w[2]), (datab[25] & exp_b_all_one_w[1]), (datab[24] & exp_b_all_one_w[0]), datab[23]},
		exp_b_not_zero_w = {(datab[30] | exp_b_not_zero_w[6]), (datab[29] | exp_b_not_zero_w[5]), (datab[28] | exp_b_not_zero_w[4]), (datab[27] | exp_b_not_zero_w[3]), (datab[26] | exp_b_not_zero_w[2]), (datab[25] | exp_b_not_zero_w[1]), (datab[24] | exp_b_not_zero_w[0]), datab[23]},
		exp_bma_w = wire_add_sub2_result,
		exp_diff_abs_exceed_max_w = {(exp_diff_abs_exceed_max_w[1] | exp_diff_abs_w[7]), (exp_diff_abs_exceed_max_w[0] | exp_diff_abs_w[6]), exp_diff_abs_w[5]},
		exp_diff_abs_max_w = {5{1'b1}},
		exp_diff_abs_w = (({8{(~ exp_amb_mux_w)}} & exp_amb_w[7:0]) | ({8{exp_amb_mux_w}} & exp_bma_w[7:0])),
		exp_intermediate_res_dffe41_wi = exp_intermediate_res_dffe42_wo,
		exp_intermediate_res_dffe41_wo = exp_intermediate_res_dffe41_wi,
		exp_intermediate_res_dffe42_wi = exp_intermediate_res_w,
		exp_intermediate_res_dffe42_wo = exp_intermediate_res_dffe42_wi,
		exp_intermediate_res_w = exp_res_dffe3_wo,
		exp_out_dffe5_wi = (({8{force_nan_w}} & exp_all_ones_w) | ({8{(~ force_nan_w)}} & (({8{force_infinity_w}} & exp_all_ones_w) | ({8{(~ force_infinity_w)}} & (({8{(force_zero_w | denormal_flag_w)}} & exp_all_zeros_w) | ({8{(~ (force_zero_w | denormal_flag_w))}} & exp_res_dffe4_wo)))))),
		exp_out_dffe5_wo = exp_out_dffe5,
		exp_res_dffe21_wi = exp_res_dffe27_wo,
		exp_res_dffe21_wo = exp_res_dffe21,
		exp_res_dffe22_wi = exp_res_dffe2_wo,
		exp_res_dffe22_wo = exp_res_dffe22_wi,
		exp_res_dffe23_wi = exp_res_dffe21_wo,
		exp_res_dffe23_wo = exp_res_dffe23_wi,
		exp_res_dffe25_wi = data_exp_dffe1_wo,
		exp_res_dffe25_wo = exp_res_dffe25_wi,
		exp_res_dffe26_wi = exp_res_dffe23_wo,
		exp_res_dffe26_wo = exp_res_dffe26_wi,
		exp_res_dffe27_wi = exp_res_dffe22_wo,
		exp_res_dffe27_wo = exp_res_dffe27_wi,
		exp_res_dffe2_wi = exp_res_dffe25_wo,
		exp_res_dffe2_wo = exp_res_dffe2,
		exp_res_dffe32_wi = ({8{(~ denormal_result_w)}} & exp_adjustment2_add_sub_w[7:0]),
		exp_res_dffe32_wo = exp_res_dffe32_wi,
		exp_res_dffe33_wi = exp_res_dffe32_wo,
		exp_res_dffe33_wo = exp_res_dffe33_wi,
		exp_res_dffe3_wi = exp_res_dffe33_wo,
		exp_res_dffe3_wo = exp_res_dffe3,
		exp_res_dffe4_wi = exp_rounded_res_w,
		exp_res_dffe4_wo = exp_res_dffe4,
		exp_res_max_w = {(exp_res_max_w[6] & exp_adjustment2_add_sub_w[7]), (exp_res_max_w[5] & exp_adjustment2_add_sub_w[6]), (exp_res_max_w[4] & exp_adjustment2_add_sub_w[5]), (exp_res_max_w[3] & exp_adjustment2_add_sub_w[4]), (exp_res_max_w[2] & exp_adjustment2_add_sub_w[3]), (exp_res_max_w[1] & exp_adjustment2_add_sub_w[2]), (exp_res_max_w[0] & exp_adjustment2_add_sub_w[1]), exp_adjustment2_add_sub_w[0]},
		exp_res_not_zero_w = {(exp_res_not_zero_w[7] | exp_adjustment2_add_sub_w[8]), (exp_res_not_zero_w[6] | exp_adjustment2_add_sub_w[7]), (exp_res_not_zero_w[5] | exp_adjustment2_add_sub_w[6]), (exp_res_not_zero_w[4] | exp_adjustment2_add_sub_w[5]), (exp_res_not_zero_w[3] | exp_adjustment2_add_sub_w[4]), (exp_res_not_zero_w[2] | exp_adjustment2_add_sub_w[3]), (exp_res_not_zero_w[1] | exp_adjustment2_add_sub_w[2]), (exp_res_not_zero_w[0] | exp_adjustment2_add_sub_w[1]), exp_adjustment2_add_sub_w[0]},
		exp_res_rounding_adder_dataa_w = {1'b0, exp_intermediate_res_dffe41_wo},
		exp_res_rounding_adder_w = wire_add_sub6_result,
		exp_rounded_res_infinity_w = exp_rounded_res_max_w[7],
		exp_rounded_res_max_w = {(exp_rounded_res_max_w[6] & exp_rounded_res_w[7]), (exp_rounded_res_max_w[5] & exp_rounded_res_w[6]), (exp_rounded_res_max_w[4] & exp_rounded_res_w[5]), (exp_rounded_res_max_w[3] & exp_rounded_res_w[4]), (exp_rounded_res_max_w[2] & exp_rounded_res_w[3]), (exp_rounded_res_max_w[1] & exp_rounded_res_w[2]), (exp_rounded_res_max_w[0] & exp_rounded_res_w[1]), exp_rounded_res_w[0]},
		exp_rounded_res_w = exp_res_rounding_adder_w[7:0],
		exp_rounding_adjustment_w = {{8{1'b0}}, man_res_rounding_add_sub_w[24]},
		exp_value = {1'b0, exp_res_dffe26_wo},
		force_infinity_w = ((input_is_infinite_dffe4_wo | rounded_res_infinity_dffe4_wo) | infinite_res_dffe4_wo),
		force_nan_w = (infinity_magnitude_sub_dffe4_wo | input_is_nan_dffe4_wo),
		force_zero_w = (~ man_res_is_not_zero_dffe4_wo),
		guard_bit_dffe3_wo = man_res_w3[0],
		infinite_output_sign_dffe1_wi = (((~ input_datab_infinite_dffe15_wo) & aligned_dataa_sign_dffe15_wo) | (input_datab_infinite_dffe15_wo & (~ (aligned_datab_sign_dffe15_wo ^ add_sub_dffe15_wo)))),
		infinite_output_sign_dffe1_wo = infinite_output_sign_dffe1,
		infinite_output_sign_dffe21_wi = infinite_output_sign_dffe27_wo,
		infinite_output_sign_dffe21_wo = infinite_output_sign_dffe21,
		infinite_output_sign_dffe22_wi = infinite_output_sign_dffe2_wo,
		infinite_output_sign_dffe22_wo = infinite_output_sign_dffe22_wi,
		infinite_output_sign_dffe23_wi = infinite_output_sign_dffe21_wo,
		infinite_output_sign_dffe23_wo = infinite_output_sign_dffe23_wi,
		infinite_output_sign_dffe25_wi = infinite_output_sign_dffe1_wo,
		infinite_output_sign_dffe25_wo = infinite_output_sign_dffe25_wi,
		infinite_output_sign_dffe26_wi = infinite_output_sign_dffe23_wo,
		infinite_output_sign_dffe26_wo = infinite_output_sign_dffe26_wi,
		infinite_output_sign_dffe27_wi = infinite_output_sign_dffe22_wo,
		infinite_output_sign_dffe27_wo = infinite_output_sign_dffe27_wi,
		infinite_output_sign_dffe2_wi = infinite_output_sign_dffe25_wo,
		infinite_output_sign_dffe2_wo = infinite_output_sign_dffe2,
		infinite_output_sign_dffe31_wi = infinite_output_sign_dffe26_wo,
		infinite_output_sign_dffe31_wo = infinite_output_sign_dffe31,
		infinite_output_sign_dffe32_wi = infinite_output_sign_dffe31_wo,
		infinite_output_sign_dffe32_wo = infinite_output_sign_dffe32_wi,
		infinite_output_sign_dffe33_wi = infinite_output_sign_dffe32_wo,
		infinite_output_sign_dffe33_wo = infinite_output_sign_dffe33_wi,
		infinite_output_sign_dffe3_wi = infinite_output_sign_dffe33_wo,
		infinite_output_sign_dffe3_wo = infinite_output_sign_dffe3,
		infinite_output_sign_dffe41_wi = infinite_output_sign_dffe42_wo,
		infinite_output_sign_dffe41_wo = infinite_output_sign_dffe41_wi,
		infinite_output_sign_dffe42_wi = infinite_output_sign_dffe3_wo,
		infinite_output_sign_dffe42_wo = infinite_output_sign_dffe42_wi,
		infinite_output_sign_dffe4_wi = infinite_output_sign_dffe41_wo,
		infinite_output_sign_dffe4_wo = infinite_output_sign_dffe4,
		infinite_res_dff32_wi = (exp_res_max_w[7] & (~ exp_adjustment2_add_sub_w[8])),
		infinite_res_dff32_wo = infinite_res_dff32_wi,
		infinite_res_dff33_wi = infinite_res_dff32_wo,
		infinite_res_dff33_wo = infinite_res_dff33_wi,
		infinite_res_dffe3_wi = infinite_res_dff33_wo,
		infinite_res_dffe3_wo = infinite_res_dffe3,
		infinite_res_dffe41_wi = infinite_res_dffe42_wo,
		infinite_res_dffe41_wo = infinite_res_dffe41_wi,
		infinite_res_dffe42_wi = infinite_res_dffe3_wo,
		infinite_res_dffe42_wo = infinite_res_dffe42_wi,
		infinite_res_dffe4_wi = infinite_res_dffe41_wo,
		infinite_res_dffe4_wo = infinite_res_dffe4,
		infinity_magnitude_sub_dffe21_wi = infinity_magnitude_sub_dffe27_wo,
		infinity_magnitude_sub_dffe21_wo = infinity_magnitude_sub_dffe21,
		infinity_magnitude_sub_dffe22_wi = infinity_magnitude_sub_dffe2_wo,
		infinity_magnitude_sub_dffe22_wo = infinity_magnitude_sub_dffe22_wi,
		infinity_magnitude_sub_dffe23_wi = infinity_magnitude_sub_dffe21_wo,
		infinity_magnitude_sub_dffe23_wo = infinity_magnitude_sub_dffe23_wi,
		infinity_magnitude_sub_dffe26_wi = infinity_magnitude_sub_dffe23_wo,
		infinity_magnitude_sub_dffe26_wo = infinity_magnitude_sub_dffe26_wi,
		infinity_magnitude_sub_dffe27_wi = infinity_magnitude_sub_dffe22_wo,
		infinity_magnitude_sub_dffe27_wo = infinity_magnitude_sub_dffe27_wi,
		infinity_magnitude_sub_dffe2_wi = ((~ add_sub_dffe25_wo) & both_inputs_are_infinite_dffe25_wo),
		infinity_magnitude_sub_dffe2_wo = infinity_magnitude_sub_dffe2,
		infinity_magnitude_sub_dffe31_wi = infinity_magnitude_sub_dffe26_wo,
		infinity_magnitude_sub_dffe31_wo = infinity_magnitude_sub_dffe31,
		infinity_magnitude_sub_dffe32_wi = infinity_magnitude_sub_dffe31_wo,
		infinity_magnitude_sub_dffe32_wo = infinity_magnitude_sub_dffe32_wi,
		infinity_magnitude_sub_dffe33_wi = infinity_magnitude_sub_dffe32_wo,
		infinity_magnitude_sub_dffe33_wo = infinity_magnitude_sub_dffe33_wi,
		infinity_magnitude_sub_dffe3_wi = infinity_magnitude_sub_dffe33_wo,
		infinity_magnitude_sub_dffe3_wo = infinity_magnitude_sub_dffe3,
		infinity_magnitude_sub_dffe41_wi = infinity_magnitude_sub_dffe42_wo,
		infinity_magnitude_sub_dffe41_wo = infinity_magnitude_sub_dffe41_wi,
		infinity_magnitude_sub_dffe42_wi = infinity_magnitude_sub_dffe3_wo,
		infinity_magnitude_sub_dffe42_wo = infinity_magnitude_sub_dffe42_wi,
		infinity_magnitude_sub_dffe4_wi = infinity_magnitude_sub_dffe41_wo,
		infinity_magnitude_sub_dffe4_wo = infinity_magnitude_sub_dffe4,
		input_dataa_denormal_dffe11_wi = input_dataa_denormal_w,
		input_dataa_denormal_dffe11_wo = input_dataa_denormal_dffe11_wi,
		input_dataa_denormal_w = ((~ exp_a_not_zero_w[7]) & man_a_not_zero_w[22]),
		input_dataa_infinite_dffe11_wi = input_dataa_infinite_w,
		input_dataa_infinite_dffe11_wo = input_dataa_infinite_dffe11_wi,
		input_dataa_infinite_dffe12_wi = input_dataa_infinite_dffe11_wo,
		input_dataa_infinite_dffe12_wo = input_dataa_infinite_dffe12,
		input_dataa_infinite_dffe13_wi = input_dataa_infinite_dffe12_wo,
		input_dataa_infinite_dffe13_wo = input_dataa_infinite_dffe13_wi,
		input_dataa_infinite_dffe14_wi = input_dataa_infinite_dffe13_wo,
		input_dataa_infinite_dffe14_wo = input_dataa_infinite_dffe14_wi,
		input_dataa_infinite_dffe15_wi = input_dataa_infinite_dffe14_wo,
		input_dataa_infinite_dffe15_wo = input_dataa_infinite_dffe15_wi,
		input_dataa_infinite_w = (exp_a_all_one_w[7] & (~ man_a_not_zero_w[22])),
		input_dataa_nan_dffe11_wi = input_dataa_nan_w,
		input_dataa_nan_dffe11_wo = input_dataa_nan_dffe11_wi,
		input_dataa_nan_dffe12_wi = input_dataa_nan_dffe11_wo,
		input_dataa_nan_dffe12_wo = input_dataa_nan_dffe12,
		input_dataa_nan_w = (exp_a_all_one_w[7] & man_a_not_zero_w[22]),
		input_dataa_zero_dffe11_wi = input_dataa_zero_w,
		input_dataa_zero_dffe11_wo = input_dataa_zero_dffe11_wi,
		input_dataa_zero_w = ((~ exp_a_not_zero_w[7]) & (~ man_a_not_zero_w[22])),
		input_datab_denormal_dffe11_wi = input_datab_denormal_w,
		input_datab_denormal_dffe11_wo = input_datab_denormal_dffe11_wi,
		input_datab_denormal_w = ((~ exp_b_not_zero_w[7]) & man_b_not_zero_w[22]),
		input_datab_infinite_dffe11_wi = input_datab_infinite_w,
		input_datab_infinite_dffe11_wo = input_datab_infinite_dffe11_wi,
		input_datab_infinite_dffe12_wi = input_datab_infinite_dffe11_wo,
		input_datab_infinite_dffe12_wo = input_datab_infinite_dffe12,
		input_datab_infinite_dffe13_wi = input_datab_infinite_dffe12_wo,
		input_datab_infinite_dffe13_wo = input_datab_infinite_dffe13_wi,
		input_datab_infinite_dffe14_wi = input_datab_infinite_dffe13_wo,
		input_datab_infinite_dffe14_wo = input_datab_infinite_dffe14_wi,
		input_datab_infinite_dffe15_wi = input_datab_infinite_dffe14_wo,
		input_datab_infinite_dffe15_wo = input_datab_infinite_dffe15_wi,
		input_datab_infinite_w = (exp_b_all_one_w[7] & (~ man_b_not_zero_w[22])),
		input_datab_nan_dffe11_wi = input_datab_nan_w,
		input_datab_nan_dffe11_wo = input_datab_nan_dffe11_wi,
		input_datab_nan_dffe12_wi = input_datab_nan_dffe11_wo,
		input_datab_nan_dffe12_wo = input_datab_nan_dffe12,
		input_datab_nan_w = (exp_b_all_one_w[7] & man_b_not_zero_w[22]),
		input_datab_zero_dffe11_wi = input_datab_zero_w,
		input_datab_zero_dffe11_wo = input_datab_zero_dffe11_wi,
		input_datab_zero_w = ((~ exp_b_not_zero_w[7]) & (~ man_b_not_zero_w[22])),
		input_is_infinite_dffe1_wi = (input_dataa_infinite_dffe15_wo | input_datab_infinite_dffe15_wo),
		input_is_infinite_dffe1_wo = input_is_infinite_dffe1,
		input_is_infinite_dffe21_wi = input_is_infinite_dffe27_wo,
		input_is_infinite_dffe21_wo = input_is_infinite_dffe21,
		input_is_infinite_dffe22_wi = input_is_infinite_dffe2_wo,
		input_is_infinite_dffe22_wo = input_is_infinite_dffe22_wi,
		input_is_infinite_dffe23_wi = input_is_infinite_dffe21_wo,
		input_is_infinite_dffe23_wo = input_is_infinite_dffe23_wi,
		input_is_infinite_dffe25_wi = input_is_infinite_dffe1_wo,
		input_is_infinite_dffe25_wo = input_is_infinite_dffe25_wi,
		input_is_infinite_dffe26_wi = input_is_infinite_dffe23_wo,
		input_is_infinite_dffe26_wo = input_is_infinite_dffe26_wi,
		input_is_infinite_dffe27_wi = input_is_infinite_dffe22_wo,
		input_is_infinite_dffe27_wo = input_is_infinite_dffe27_wi,
		input_is_infinite_dffe2_wi = input_is_infinite_dffe25_wo,
		input_is_infinite_dffe2_wo = input_is_infinite_dffe2,
		input_is_infinite_dffe31_wi = input_is_infinite_dffe26_wo,
		input_is_infinite_dffe31_wo = input_is_infinite_dffe31,
		input_is_infinite_dffe32_wi = input_is_infinite_dffe31_wo,
		input_is_infinite_dffe32_wo = input_is_infinite_dffe32_wi,
		input_is_infinite_dffe33_wi = input_is_infinite_dffe32_wo,
		input_is_infinite_dffe33_wo = input_is_infinite_dffe33_wi,
		input_is_infinite_dffe3_wi = input_is_infinite_dffe33_wo,
		input_is_infinite_dffe3_wo = input_is_infinite_dffe3,
		input_is_infinite_dffe41_wi = input_is_infinite_dffe42_wo,
		input_is_infinite_dffe41_wo = input_is_infinite_dffe41_wi,
		input_is_infinite_dffe42_wi = input_is_infinite_dffe3_wo,
		input_is_infinite_dffe42_wo = input_is_infinite_dffe42_wi,
		input_is_infinite_dffe4_wi = input_is_infinite_dffe41_wo,
		input_is_infinite_dffe4_wo = input_is_infinite_dffe4,
		input_is_nan_dffe13_wi = (input_dataa_nan_dffe12_wo | input_datab_nan_dffe12_wo),
		input_is_nan_dffe13_wo = input_is_nan_dffe13_wi,
		input_is_nan_dffe14_wi = input_is_nan_dffe13_wo,
		input_is_nan_dffe14_wo = input_is_nan_dffe14_wi,
		input_is_nan_dffe15_wi = input_is_nan_dffe14_wo,
		input_is_nan_dffe15_wo = input_is_nan_dffe15_wi,
		input_is_nan_dffe1_wi = input_is_nan_dffe15_wo,
		input_is_nan_dffe1_wo = input_is_nan_dffe1,
		input_is_nan_dffe21_wi = input_is_nan_dffe27_wo,
		input_is_nan_dffe21_wo = input_is_nan_dffe21,
		input_is_nan_dffe22_wi = input_is_nan_dffe2_wo,
		input_is_nan_dffe22_wo = input_is_nan_dffe22_wi,
		input_is_nan_dffe23_wi = input_is_nan_dffe21_wo,
		input_is_nan_dffe23_wo = input_is_nan_dffe23_wi,
		input_is_nan_dffe25_wi = input_is_nan_dffe1_wo,
		input_is_nan_dffe25_wo = input_is_nan_dffe25_wi,
		input_is_nan_dffe26_wi = input_is_nan_dffe23_wo,
		input_is_nan_dffe26_wo = input_is_nan_dffe26_wi,
		input_is_nan_dffe27_wi = input_is_nan_dffe22_wo,
		input_is_nan_dffe27_wo = input_is_nan_dffe27_wi,
		input_is_nan_dffe2_wi = input_is_nan_dffe25_wo,
		input_is_nan_dffe2_wo = input_is_nan_dffe2,
		input_is_nan_dffe31_wi = input_is_nan_dffe26_wo,
		input_is_nan_dffe31_wo = input_is_nan_dffe31,
		input_is_nan_dffe32_wi = input_is_nan_dffe31_wo,
		input_is_nan_dffe32_wo = input_is_nan_dffe32_wi,
		input_is_nan_dffe33_wi = input_is_nan_dffe32_wo,
		input_is_nan_dffe33_wo = input_is_nan_dffe33_wi,
		input_is_nan_dffe3_wi = input_is_nan_dffe33_wo,
		input_is_nan_dffe3_wo = input_is_nan_dffe3,
		input_is_nan_dffe41_wi = input_is_nan_dffe42_wo,
		input_is_nan_dffe41_wo = input_is_nan_dffe41_wi,
		input_is_nan_dffe42_wi = input_is_nan_dffe3_wo,
		input_is_nan_dffe42_wo = input_is_nan_dffe42_wi,
		input_is_nan_dffe4_wi = input_is_nan_dffe41_wo,
		input_is_nan_dffe4_wo = input_is_nan_dffe4,
		man_2comp_res_dataa_w = {pos_sign_bit_ext, datab_man_dffe1_wo},
		man_2comp_res_datab_w = {pos_sign_bit_ext, dataa_man_dffe1_wo},
		man_2comp_res_w = {(({14{(~ wire_man_2comp_res_lower_cout)}} & wire_man_2comp_res_upper0_result) | ({14{wire_man_2comp_res_lower_cout}} & wire_man_2comp_res_upper1_result)), wire_man_2comp_res_lower_result},
		man_a_not_zero_w = {(dataa[22] | man_a_not_zero_w[21]), (dataa[21] | man_a_not_zero_w[20]), (dataa[20] | man_a_not_zero_w[19]), (dataa[19] | man_a_not_zero_w[18]), (dataa[18] | man_a_not_zero_w[17]), (dataa[17] | man_a_not_zero_w[16]), (dataa[16] | man_a_not_zero_w[15]), (dataa[15] | man_a_not_zero_w[14]), (dataa[14] | man_a_not_zero_w[13]), (dataa[13] | man_a_not_zero_w[12]), (dataa[12] | man_a_not_zero_w[11]), (dataa[11] | man_a_not_zero_w[10]), (dataa[10] | man_a_not_zero_w[9]), (dataa[9] | man_a_not_zero_w[8]), (dataa[8] | man_a_not_zero_w[7]), (dataa[7] | man_a_not_zero_w[6]), (dataa[6] | man_a_not_zero_w[5]), (dataa[5] | man_a_not_zero_w[4]), (dataa[4] | man_a_not_zero_w[3]), (dataa[3] | man_a_not_zero_w[2]), (dataa[2] | man_a_not_zero_w[1]), (dataa[1] | man_a_not_zero_w[0]), dataa[0]},
		man_add_sub_dataa_w = {pos_sign_bit_ext, dataa_man_dffe1_wo},
		man_add_sub_datab_w = {pos_sign_bit_ext, datab_man_dffe1_wo},
		man_add_sub_res_mag_dffe21_wi = man_res_mag_w2,
		man_add_sub_res_mag_dffe21_wo = man_add_sub_res_mag_dffe21,
		man_add_sub_res_mag_dffe23_wi = man_add_sub_res_mag_dffe21_wo,
		man_add_sub_res_mag_dffe23_wo = man_add_sub_res_mag_dffe23_wi,
		man_add_sub_res_mag_dffe26_wi = man_add_sub_res_mag_dffe23_wo,
		man_add_sub_res_mag_dffe26_wo = man_add_sub_res_mag_dffe26_wi,
		man_add_sub_res_mag_dffe27_wi = man_add_sub_res_mag_w2,
		man_add_sub_res_mag_dffe27_wo = man_add_sub_res_mag_dffe27_wi,
		man_add_sub_res_mag_w2 = (({28{man_add_sub_w[27]}} & man_2comp_res_w) | ({28{(~ man_add_sub_w[27])}} & man_add_sub_w)),
		man_add_sub_res_sign_dffe21_wo = man_add_sub_res_sign_dffe21,
		man_add_sub_res_sign_dffe23_wi = man_add_sub_res_sign_dffe21_wo,
		man_add_sub_res_sign_dffe23_wo = man_add_sub_res_sign_dffe23_wi,
		man_add_sub_res_sign_dffe26_wi = man_add_sub_res_sign_dffe23_wo,
		man_add_sub_res_sign_dffe26_wo = man_add_sub_res_sign_dffe26_wi,
		man_add_sub_res_sign_dffe27_wi = man_add_sub_res_sign_w2,
		man_add_sub_res_sign_dffe27_wo = man_add_sub_res_sign_dffe27_wi,
		man_add_sub_res_sign_w2 = ((need_complement_dffe22_wo & (~ man_add_sub_w[27])) | ((~ need_complement_dffe22_wo) & man_add_sub_w[27])),
		man_add_sub_w = {(({14{(~ wire_man_add_sub_lower_cout)}} & wire_man_add_sub_upper0_result) | ({14{wire_man_add_sub_lower_cout}} & wire_man_add_sub_upper1_result)), wire_man_add_sub_lower_result},
		man_all_zeros_w = {23{1'b0}},
		man_b_not_zero_w = {(datab[22] | man_b_not_zero_w[21]), (datab[21] | man_b_not_zero_w[20]), (datab[20] | man_b_not_zero_w[19]), (datab[19] | man_b_not_zero_w[18]), (datab[18] | man_b_not_zero_w[17]), (datab[17] | man_b_not_zero_w[16]), (datab[16] | man_b_not_zero_w[15]), (datab[15] | man_b_not_zero_w[14]), (datab[14] | man_b_not_zero_w[13]), (datab[13] | man_b_not_zero_w[12]), (datab[12] | man_b_not_zero_w[11]), (datab[11] | man_b_not_zero_w[10]), (datab[10] | man_b_not_zero_w[9]), (datab[9] | man_b_not_zero_w[8]), (datab[8] | man_b_not_zero_w[7]), (datab[7] | man_b_not_zero_w[6]), (datab[6] | man_b_not_zero_w[5]), (datab[5] | man_b_not_zero_w[4]), (datab[4] | man_b_not_zero_w[3]), (datab[3] | man_b_not_zero_w[2]), (datab[2] | man_b_not_zero_w[1]), (datab[1] | man_b_not_zero_w[0]), datab[0]},
		man_dffe31_wo = man_dffe31,
		man_intermediate_res_w = {{2{1'b0}}, man_res_w3},
		man_leading_zeros_cnt_w = man_leading_zeros_dffe31_wo,
		man_leading_zeros_dffe31_wi = (~ wire_leading_zeroes_cnt_q),
		man_leading_zeros_dffe31_wo = man_leading_zeros_dffe31,
		man_nan_w = 23'b10000000000000000000000,
		man_out_dffe5_wi = (({23{force_nan_w}} & man_nan_w) | ({23{(~ force_nan_w)}} & (({23{force_infinity_w}} & man_all_zeros_w) | ({23{(~ force_infinity_w)}} & (({23{(force_zero_w | denormal_flag_w)}} & man_all_zeros_w) | ({23{(~ (force_zero_w | denormal_flag_w))}} & man_res_dffe4_wo)))))),
		man_out_dffe5_wo = man_out_dffe5,
		man_res_dffe4_wi = man_rounded_res_w,
		man_res_dffe4_wo = man_res_dffe4,
		man_res_is_not_zero_dffe31_wi = man_res_not_zero_dffe26_wo,
		man_res_is_not_zero_dffe31_wo = man_res_is_not_zero_dffe31,
		man_res_is_not_zero_dffe32_wi = man_res_is_not_zero_dffe31_wo,
		man_res_is_not_zero_dffe32_wo = man_res_is_not_zero_dffe32_wi,
		man_res_is_not_zero_dffe33_wi = man_res_is_not_zero_dffe32_wo,
		man_res_is_not_zero_dffe33_wo = man_res_is_not_zero_dffe33_wi,
		man_res_is_not_zero_dffe3_wi = man_res_is_not_zero_dffe33_wo,
		man_res_is_not_zero_dffe3_wo = man_res_is_not_zero_dffe3,
		man_res_is_not_zero_dffe41_wi = man_res_is_not_zero_dffe42_wo,
		man_res_is_not_zero_dffe41_wo = man_res_is_not_zero_dffe41_wi,
		man_res_is_not_zero_dffe42_wi = man_res_is_not_zero_dffe3_wo,
		man_res_is_not_zero_dffe42_wo = man_res_is_not_zero_dffe42_wi,
		man_res_is_not_zero_dffe4_wi = man_res_is_not_zero_dffe41_wo,
		man_res_is_not_zero_dffe4_wo = man_res_is_not_zero_dffe4,
		man_res_mag_w2 = (({26{man_add_sub_res_mag_dffe27_wo[26]}} & man_add_sub_res_mag_dffe27_wo[26:1]) | ({26{(~ man_add_sub_res_mag_dffe27_wo[26])}} & man_add_sub_res_mag_dffe27_wo[25:0])),
		man_res_not_zero_dffe23_wi = man_res_not_zero_w2[24],
		man_res_not_zero_dffe23_wo = man_res_not_zero_dffe23_wi,
		man_res_not_zero_dffe26_wi = man_res_not_zero_dffe23_wo,
		man_res_not_zero_dffe26_wo = man_res_not_zero_dffe26_wi,
		man_res_not_zero_w2 = {(man_res_not_zero_w2[23] | man_add_sub_res_mag_dffe21_wo[25]), (man_res_not_zero_w2[22] | man_add_sub_res_mag_dffe21_wo[24]), (man_res_not_zero_w2[21] | man_add_sub_res_mag_dffe21_wo[23]), (man_res_not_zero_w2[20] | man_add_sub_res_mag_dffe21_wo[22]), (man_res_not_zero_w2[19] | man_add_sub_res_mag_dffe21_wo[21]), (man_res_not_zero_w2[18] | man_add_sub_res_mag_dffe21_wo[20]), (man_res_not_zero_w2[17] | man_add_sub_res_mag_dffe21_wo[19]), (man_res_not_zero_w2[16] | man_add_sub_res_mag_dffe21_wo[18]), (man_res_not_zero_w2[15] | man_add_sub_res_mag_dffe21_wo[17]), (man_res_not_zero_w2[14] | man_add_sub_res_mag_dffe21_wo[16]), (man_res_not_zero_w2[13] | man_add_sub_res_mag_dffe21_wo[15]), (man_res_not_zero_w2[12] | man_add_sub_res_mag_dffe21_wo[14]), (man_res_not_zero_w2[11] | man_add_sub_res_mag_dffe21_wo[13]), (man_res_not_zero_w2[10] | man_add_sub_res_mag_dffe21_wo[12]), (man_res_not_zero_w2[9] | man_add_sub_res_mag_dffe21_wo[11]), (man_res_not_zero_w2[8] | man_add_sub_res_mag_dffe21_wo[10]), (man_res_not_zero_w2[7] | man_add_sub_res_mag_dffe21_wo[9]), (man_res_not_zero_w2[6] | man_add_sub_res_mag_dffe21_wo[8]), (man_res_not_zero_w2[5] | man_add_sub_res_mag_dffe21_wo[7]), (man_res_not_zero_w2[4] | man_add_sub_res_mag_dffe21_wo[6]), (man_res_not_zero_w2[3] | man_add_sub_res_mag_dffe21_wo[5]), (man_res_not_zero_w2[2] | man_add_sub_res_mag_dffe21_wo[4]), (man_res_not_zero_w2[1] | man_add_sub_res_mag_dffe21_wo[3]), (man_res_not_zero_w2[0] | man_add_sub_res_mag_dffe21_wo[2]), man_add_sub_res_mag_dffe21_wo[1]},
		man_res_rounding_add_sub_datab_w = {{25{1'b0}}, man_rounding_add_value_w},
		man_res_rounding_add_sub_w = {(({13{(~ wire_man_res_rounding_add_sub_lower_cout)}} & adder_upper_w) | ({13{wire_man_res_rounding_add_sub_lower_cout}} & wire_man_res_rounding_add_sub_upper1_result)), wire_man_res_rounding_add_sub_lower_result},
		man_res_w3 = wire_lbarrel_shift_result[25:2],
		man_rounded_res_w = (({23{man_res_rounding_add_sub_w[24]}} & man_res_rounding_add_sub_w[23:1]) | ({23{(~ man_res_rounding_add_sub_w[24])}} & man_res_rounding_add_sub_w[22:0])),
		man_rounding_add_value_w = (round_bit_dffe3_wo & (sticky_bit_dffe3_wo | guard_bit_dffe3_wo)),
		man_smaller_dffe13_wi = man_smaller_w,
		man_smaller_dffe13_wo = man_smaller_dffe13_wi,
		man_smaller_w = (({24{exp_amb_mux_w}} & aligned_dataa_man_dffe12_wo) | ({24{(~ exp_amb_mux_w)}} & aligned_datab_man_dffe12_wo)),
		need_complement_dffe22_wi = need_complement_dffe2_wo,
		need_complement_dffe22_wo = need_complement_dffe22_wi,
		need_complement_dffe2_wi = dataa_sign_dffe25_wo,
		need_complement_dffe2_wo = need_complement_dffe2,
		pos_sign_bit_ext = {2{1'b0}},
		priority_encoder_1pads_w = {4{1'b1}},
		result = {sign_out_dffe5_wo, exp_out_dffe5_wo, man_out_dffe5_wo},
		round_bit_dffe21_wi = round_bit_w,
		round_bit_dffe21_wo = round_bit_dffe21,
		round_bit_dffe23_wi = round_bit_dffe21_wo,
		round_bit_dffe23_wo = round_bit_dffe23_wi,
		round_bit_dffe26_wi = round_bit_dffe23_wo,
		round_bit_dffe26_wo = round_bit_dffe26_wi,
		round_bit_dffe31_wi = round_bit_dffe26_wo,
		round_bit_dffe31_wo = round_bit_dffe31,
		round_bit_dffe32_wi = round_bit_dffe31_wo,
		round_bit_dffe32_wo = round_bit_dffe32_wi,
		round_bit_dffe33_wi = round_bit_dffe32_wo,
		round_bit_dffe33_wo = round_bit_dffe33_wi,
		round_bit_dffe3_wi = round_bit_dffe33_wo,
		round_bit_dffe3_wo = round_bit_dffe3,
		round_bit_w = ((((((~ man_add_sub_res_mag_dffe27_wo[26]) & (~ man_add_sub_res_mag_dffe27_wo[25])) & man_add_sub_res_mag_dffe27_wo[0]) | (((~ man_add_sub_res_mag_dffe27_wo[26]) & man_add_sub_res_mag_dffe27_wo[25]) & man_add_sub_res_mag_dffe27_wo[1])) | ((man_add_sub_res_mag_dffe27_wo[26] & (~ man_add_sub_res_mag_dffe27_wo[25])) & man_add_sub_res_mag_dffe27_wo[2])) | ((man_add_sub_res_mag_dffe27_wo[26] & man_add_sub_res_mag_dffe27_wo[25]) & man_add_sub_res_mag_dffe27_wo[2])),
		rounded_res_infinity_dffe4_wi = exp_rounded_res_infinity_w,
		rounded_res_infinity_dffe4_wo = rounded_res_infinity_dffe4,
		rshift_distance_dffe13_wi = rshift_distance_w,
		rshift_distance_dffe13_wo = rshift_distance_dffe13_wi,
		rshift_distance_dffe14_wi = rshift_distance_dffe13_wo,
		rshift_distance_dffe14_wo = rshift_distance_dffe14_wi,
		rshift_distance_dffe15_wi = rshift_distance_dffe14_wo,
		rshift_distance_dffe15_wo = rshift_distance_dffe15_wi,
		rshift_distance_w = (({5{exp_diff_abs_exceed_max_w[2]}} & exp_diff_abs_max_w) | ({5{(~ exp_diff_abs_exceed_max_w[2])}} & exp_diff_abs_w[4:0])),
		sign_dffe31_wi = ((man_res_not_zero_dffe26_wo & man_add_sub_res_sign_dffe26_wo) | ((~ man_res_not_zero_dffe26_wo) & zero_man_sign_dffe26_wo)),
		sign_dffe31_wo = sign_dffe31,
		sign_dffe32_wi = sign_dffe31_wo,
		sign_dffe32_wo = sign_dffe32_wi,
		sign_dffe33_wi = sign_dffe32_wo,
		sign_dffe33_wo = sign_dffe33_wi,
		sign_out_dffe5_wi = ((~ force_nan_w) & ((force_infinity_w & infinite_output_sign_dffe4_wo) | ((~ force_infinity_w) & sign_res_dffe4_wo))),
		sign_out_dffe5_wo = sign_out_dffe5,
		sign_res_dffe3_wi = sign_dffe33_wo,
		sign_res_dffe3_wo = sign_res_dffe3,
		sign_res_dffe41_wi = sign_res_dffe42_wo,
		sign_res_dffe41_wo = sign_res_dffe41_wi,
		sign_res_dffe42_wi = sign_res_dffe3_wo,
		sign_res_dffe42_wo = sign_res_dffe42_wi,
		sign_res_dffe4_wi = sign_res_dffe41_wo,
		sign_res_dffe4_wo = sign_res_dffe4,
		sticky_bit_cnt_dataa_w = {1'b0, rshift_distance_dffe15_wo},
		sticky_bit_cnt_datab_w = {1'b0, wire_trailing_zeros_cnt_q},
		sticky_bit_cnt_res_w = wire_add_sub3_result,
		sticky_bit_dffe1_wi = wire_trailing_zeros_limit_comparator_agb,
		sticky_bit_dffe1_wo = sticky_bit_dffe1,
		sticky_bit_dffe21_wi = sticky_bit_w,
		sticky_bit_dffe21_wo = sticky_bit_dffe21,
		sticky_bit_dffe22_wi = sticky_bit_dffe2_wo,
		sticky_bit_dffe22_wo = sticky_bit_dffe22_wi,
		sticky_bit_dffe23_wi = sticky_bit_dffe21_wo,
		sticky_bit_dffe23_wo = sticky_bit_dffe23_wi,
		sticky_bit_dffe25_wi = sticky_bit_dffe1_wo,
		sticky_bit_dffe25_wo = sticky_bit_dffe25_wi,
		sticky_bit_dffe26_wi = sticky_bit_dffe23_wo,
		sticky_bit_dffe26_wo = sticky_bit_dffe26_wi,
		sticky_bit_dffe27_wi = sticky_bit_dffe22_wo,
		sticky_bit_dffe27_wo = sticky_bit_dffe27_wi,
		sticky_bit_dffe2_wi = sticky_bit_dffe25_wo,
		sticky_bit_dffe2_wo = sticky_bit_dffe2,
		sticky_bit_dffe31_wi = sticky_bit_dffe26_wo,
		sticky_bit_dffe31_wo = sticky_bit_dffe31,
		sticky_bit_dffe32_wi = sticky_bit_dffe31_wo,
		sticky_bit_dffe32_wo = sticky_bit_dffe32_wi,
		sticky_bit_dffe33_wi = sticky_bit_dffe32_wo,
		sticky_bit_dffe33_wo = sticky_bit_dffe33_wi,
		sticky_bit_dffe3_wi = sticky_bit_dffe33_wo,
		sticky_bit_dffe3_wo = sticky_bit_dffe3,
		sticky_bit_w = ((((((~ man_add_sub_res_mag_dffe27_wo[26]) & (~ man_add_sub_res_mag_dffe27_wo[25])) & sticky_bit_dffe27_wo) | (((~ man_add_sub_res_mag_dffe27_wo[26]) & man_add_sub_res_mag_dffe27_wo[25]) & (sticky_bit_dffe27_wo | man_add_sub_res_mag_dffe27_wo[0]))) | ((man_add_sub_res_mag_dffe27_wo[26] & (~ man_add_sub_res_mag_dffe27_wo[25])) & ((sticky_bit_dffe27_wo | man_add_sub_res_mag_dffe27_wo[0]) | man_add_sub_res_mag_dffe27_wo[1]))) | ((man_add_sub_res_mag_dffe27_wo[26] & man_add_sub_res_mag_dffe27_wo[25]) & ((sticky_bit_dffe27_wo | man_add_sub_res_mag_dffe27_wo[0]) | man_add_sub_res_mag_dffe27_wo[1]))),
		trailing_zeros_limit_w = 6'b000010,
		zero_man_sign_dffe21_wi = zero_man_sign_dffe27_wo,
		zero_man_sign_dffe21_wo = zero_man_sign_dffe21,
		zero_man_sign_dffe22_wi = zero_man_sign_dffe2_wo,
		zero_man_sign_dffe22_wo = zero_man_sign_dffe22_wi,
		zero_man_sign_dffe23_wi = zero_man_sign_dffe21_wo,
		zero_man_sign_dffe23_wo = zero_man_sign_dffe23_wi,
		zero_man_sign_dffe26_wi = zero_man_sign_dffe23_wo,
		zero_man_sign_dffe26_wo = zero_man_sign_dffe26_wi,
		zero_man_sign_dffe27_wi = zero_man_sign_dffe22_wo,
		zero_man_sign_dffe27_wo = zero_man_sign_dffe27_wi,
		zero_man_sign_dffe2_wi = (dataa_sign_dffe25_wo & add_sub_dffe25_wo),
		zero_man_sign_dffe2_wo = zero_man_sign_dffe2;
endmodule //fpoint_qsys_addsub_single
//VALID FILE

//Legal Notice: (C)2010 Altera Corporation. All rights reserved.  Your
//use of Altera Corporation's design tools, logic functions and other
//software and tools, and its AMPP partner logic functions, and any
//output files any of the foregoing (including device programming or
//simulation files), and any associated documentation or information are
//expressly subject to the terms and conditions of the Altera Program
//License Subscription Agreement or other applicable license agreement,
//including, without limitation, that your use is for the sole purpose
//of programming logic devices manufactured by Altera and sold by Altera
//or its authorized distributors.  Please refer to the applicable
//agreement for further details.

// synthesis translate_off
`timescale 1ns / 1ps
// synthesis translate_on

// turn off superfluous verilog processor warnings 
// altera message_level Level1 
// altera message_off 10034 10035 10036 10037 10230 10240 10030 

module fpoint_qsys (
                     // inputs:
                      clk,
                      clk_en,
                      dataa,
                      datab,
                      n,
                      reset,
                      start,

                     // outputs:
                      done,
                      result
                   )
;

  output           done;
  output  [ 31: 0] result;
  input            clk;
  input            clk_en;
  input   [ 31: 0] dataa;
  input   [ 31: 0] datab;
  input   [  1: 0] n;
  input            reset;
  input            start;

  wire             add_sub;
  wire    [  3: 0] counter_in;
  reg     [  3: 0] counter_out;
  reg     [ 31: 0] dataa_regout;
  reg     [ 31: 0] datab_regout;
  wire             done;
  wire    [  3: 0] load_data;
  wire             local_reset_n;
  wire    [ 31: 0] result;
  wire    [ 31: 0] result_addsub;
  wire    [ 31: 0] result_mult;
  //register the input for dataa
  always @(posedge clk or negedge local_reset_n)
    begin
      if (local_reset_n == 0)
          dataa_regout <= 0;
      else if (clk_en)
          dataa_regout <= dataa;
    end


  //register the input for datab
  always @(posedge clk or negedge local_reset_n)
    begin
      if (local_reset_n == 0)
          datab_regout <= 0;
      else if (clk_en)
          datab_regout <= datab;
    end


  fpoint_qsys_mult_single the_fp_mult
    (
      .aclr (reset),
      .clk_en (clk_en),
      .clock (clk),
      .dataa (dataa_regout),
      .datab (datab_regout),
      .result (result_mult)
    );


  fpoint_qsys_addsub_single the_fp_addsub
    (
      .aclr (reset),
      .add_sub (add_sub),
      .clk_en (clk_en),
      .clock (clk),
      .dataa (dataa_regout),
      .datab (datab_regout),
      .result (result_addsub)
    );


  //s1, which is an e_custom_instruction_slave
  //down_counter to signal done
  always @(posedge clk or negedge local_reset_n)
    begin
      if (local_reset_n == 0)
          counter_out <= 4'd10;
      else if (clk_en)
          counter_out <= counter_in;
    end


  //decrement or load the counter based on start
  assign counter_in = (start == 0)? counter_out - 1'b1 :
    load_data;

  assign add_sub = n[0];
  assign local_reset_n = ~reset;
  assign done = clk_en & ~|counter_out & ~start;
  //select load value of counter based on n
  assign load_data = (n == 0)? 10 :
    (n == 1)? 8 :
    8;

  //multiplex output based on n
  assign result = (n == 0)? result_mult :
    (n == 1)? result_addsub :
    result_addsub;


endmodule

