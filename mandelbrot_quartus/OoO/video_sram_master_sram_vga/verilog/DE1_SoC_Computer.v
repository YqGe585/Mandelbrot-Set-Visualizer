

module DE1_SoC_Computer (
	////////////////////////////////////
	// FPGA Pins
	////////////////////////////////////

	// Clock pins
	CLOCK_50,
	CLOCK2_50,
	CLOCK3_50,
	CLOCK4_50,

	// ADC
	ADC_CS_N,
	ADC_DIN,
	ADC_DOUT,
	ADC_SCLK,

	// Audio
	AUD_ADCDAT,
	AUD_ADCLRCK,
	AUD_BCLK,
	AUD_DACDAT,
	AUD_DACLRCK,
	AUD_XCK,

	// SDRAM
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_LDQM,
	DRAM_RAS_N,
	DRAM_UDQM,
	DRAM_WE_N,

	// I2C Bus for Configuration of the Audio and Video-In Chips
	FPGA_I2C_SCLK,
	FPGA_I2C_SDAT,

	// 40-Pin Headers
	GPIO_0,
	GPIO_1,
	
	// Seven Segment Displays
	HEX0,
	HEX1,
	HEX2,
	HEX3,
	HEX4,
	HEX5,

	// IR
	IRDA_RXD,
	IRDA_TXD,

	// Pushbuttons
	KEY,

	// LEDs
	LEDR,

	// PS2 Ports
	PS2_CLK,
	PS2_DAT,
	
	PS2_CLK2,
	PS2_DAT2,

	// Slider Switches
	SW,

	// Video-In
	TD_CLK27,
	TD_DATA,
	TD_HS,
	TD_RESET_N,
	TD_VS,

	// VGA
	VGA_B,
	VGA_BLANK_N,
	VGA_CLK,
	VGA_G,
	VGA_HS,
	VGA_R,
	VGA_SYNC_N,
	VGA_VS,

	////////////////////////////////////
	// HPS Pins
	////////////////////////////////////
	
	// DDR3 SDRAM
	HPS_DDR3_ADDR,
	HPS_DDR3_BA,
	HPS_DDR3_CAS_N,
	HPS_DDR3_CKE,
	HPS_DDR3_CK_N,
	HPS_DDR3_CK_P,
	HPS_DDR3_CS_N,
	HPS_DDR3_DM,
	HPS_DDR3_DQ,
	HPS_DDR3_DQS_N,
	HPS_DDR3_DQS_P,
	HPS_DDR3_ODT,
	HPS_DDR3_RAS_N,
	HPS_DDR3_RESET_N,
	HPS_DDR3_RZQ,
	HPS_DDR3_WE_N,

	// Ethernet
	HPS_ENET_GTX_CLK,
	HPS_ENET_INT_N,
	HPS_ENET_MDC,
	HPS_ENET_MDIO,
	HPS_ENET_RX_CLK,
	HPS_ENET_RX_DATA,
	HPS_ENET_RX_DV,
	HPS_ENET_TX_DATA,
	HPS_ENET_TX_EN,

	// Flash
	HPS_FLASH_DATA,
	HPS_FLASH_DCLK,
	HPS_FLASH_NCSO,

	// Accelerometer
	HPS_GSENSOR_INT,
		
	// General Purpose I/O
	HPS_GPIO,
		
	// I2C
	HPS_I2C_CONTROL,
	HPS_I2C1_SCLK,
	HPS_I2C1_SDAT,
	HPS_I2C2_SCLK,
	HPS_I2C2_SDAT,

	// Pushbutton
	HPS_KEY,

	// LED
	HPS_LED,
		
	// SD Card
	HPS_SD_CLK,
	HPS_SD_CMD,
	HPS_SD_DATA,

	// SPI
	HPS_SPIM_CLK,
	HPS_SPIM_MISO,
	HPS_SPIM_MOSI,
	HPS_SPIM_SS,

	// UART
	HPS_UART_RX,
	HPS_UART_TX,

	// USB
	HPS_CONV_USB_N,
	HPS_USB_CLKOUT,
	HPS_USB_DATA,
	HPS_USB_DIR,
	HPS_USB_NXT,
	HPS_USB_STP
);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

////////////////////////////////////
// FPGA Pins
////////////////////////////////////

// Clock pins
input						CLOCK_50;
input						CLOCK2_50;
input						CLOCK3_50;
input						CLOCK4_50;

// ADC
inout						ADC_CS_N;
output					ADC_DIN;
input						ADC_DOUT;
output					ADC_SCLK;

// Audio
input						AUD_ADCDAT;
inout						AUD_ADCLRCK;
inout						AUD_BCLK;
output					AUD_DACDAT;
inout						AUD_DACLRCK;
output					AUD_XCK;

// SDRAM
output 		[12: 0]	DRAM_ADDR;
output		[ 1: 0]	DRAM_BA;
output					DRAM_CAS_N;
output					DRAM_CKE;
output					DRAM_CLK;
output					DRAM_CS_N;
inout			[15: 0]	DRAM_DQ;
output					DRAM_LDQM;
output					DRAM_RAS_N;
output					DRAM_UDQM;
output					DRAM_WE_N;

// I2C Bus for Configuration of the Audio and Video-In Chips
output					FPGA_I2C_SCLK;
inout						FPGA_I2C_SDAT;

// 40-pin headers
inout			[35: 0]	GPIO_0;
inout			[35: 0]	GPIO_1;

// Seven Segment Displays
output		[ 6: 0]	HEX0;
output		[ 6: 0]	HEX1;
output		[ 6: 0]	HEX2;
output		[ 6: 0]	HEX3;
output		[ 6: 0]	HEX4;
output		[ 6: 0]	HEX5;

// IR
input						IRDA_RXD;
output					IRDA_TXD;

// Pushbuttons
input			[ 3: 0]	KEY;

// LEDs
output		[ 9: 0]	LEDR;

// PS2 Ports
inout						PS2_CLK;
inout						PS2_DAT;

inout						PS2_CLK2;
inout						PS2_DAT2;

// Slider Switches
input			[ 9: 0]	SW;

// Video-In
input						TD_CLK27;
input			[ 7: 0]	TD_DATA;
input						TD_HS;
output					TD_RESET_N;
input						TD_VS;

// VGA
output		[ 7: 0]	VGA_B;
output					VGA_BLANK_N;
output					VGA_CLK;
output		[ 7: 0]	VGA_G;
output					VGA_HS;
output		[ 7: 0]	VGA_R;
output					VGA_SYNC_N;
output					VGA_VS;



////////////////////////////////////
// HPS Pins
////////////////////////////////////
	
// DDR3 SDRAM
output		[14: 0]	HPS_DDR3_ADDR;
output		[ 2: 0]  HPS_DDR3_BA;
output					HPS_DDR3_CAS_N;
output					HPS_DDR3_CKE;
output					HPS_DDR3_CK_N;
output					HPS_DDR3_CK_P;
output					HPS_DDR3_CS_N;
output		[ 3: 0]	HPS_DDR3_DM;
inout			[31: 0]	HPS_DDR3_DQ;
inout			[ 3: 0]	HPS_DDR3_DQS_N;
inout			[ 3: 0]	HPS_DDR3_DQS_P;
output					HPS_DDR3_ODT;
output					HPS_DDR3_RAS_N;
output					HPS_DDR3_RESET_N;
input						HPS_DDR3_RZQ;
output					HPS_DDR3_WE_N;

// Ethernet
output					HPS_ENET_GTX_CLK;
inout						HPS_ENET_INT_N;
output					HPS_ENET_MDC;
inout						HPS_ENET_MDIO;
input						HPS_ENET_RX_CLK;
input			[ 3: 0]	HPS_ENET_RX_DATA;
input						HPS_ENET_RX_DV;
output		[ 3: 0]	HPS_ENET_TX_DATA;
output					HPS_ENET_TX_EN;

// Flash
inout			[ 3: 0]	HPS_FLASH_DATA;
output					HPS_FLASH_DCLK;
output					HPS_FLASH_NCSO;

// Accelerometer
inout						HPS_GSENSOR_INT;

// General Purpose I/O
inout			[ 1: 0]	HPS_GPIO;

// I2C
inout						HPS_I2C_CONTROL;
inout						HPS_I2C1_SCLK;
inout						HPS_I2C1_SDAT;
inout						HPS_I2C2_SCLK;
inout						HPS_I2C2_SDAT;

// Pushbutton
inout						HPS_KEY;

// LED
inout						HPS_LED;

// SD Card
output					HPS_SD_CLK;
inout						HPS_SD_CMD;
inout			[ 3: 0]	HPS_SD_DATA;

// SPI
output					HPS_SPIM_CLK;
input						HPS_SPIM_MISO;
output					HPS_SPIM_MOSI;
inout						HPS_SPIM_SS;

// UART
input						HPS_UART_RX;
output					HPS_UART_TX;

// USB
inout						HPS_CONV_USB_N;
input						HPS_USB_CLKOUT;
inout			[ 7: 0]	HPS_USB_DATA;
input						HPS_USB_DIR;
input						HPS_USB_NXT;
output					HPS_USB_STP;

//=======================================================
//  REG/WIRE declarations
//=======================================================

wire			[15: 0]	hex3_hex0;
//wire			[15: 0]	hex5_hex4;

//assign HEX0 = ~hex3_hex0[ 6: 0]; // hex3_hex0[ 6: 0]; 
//assign HEX1 = ~hex3_hex0[14: 8];
//assign HEX2 = ~hex3_hex0[22:16];
//assign HEX3 = ~hex3_hex0[30:24];
//assign HEX4 = 7'b1111111;
//assign HEX5 = 7'b1111111;

//HexDigit Digit0(HEX0, hex3_hex0[3:0]);
//HexDigit Digit1(HEX1, hex3_hex0[7:4]);
//HexDigit Digit2(HEX2, hex3_hex0[11:8]);
//HexDigit Digit3(HEX3, hex3_hex0[15:12]);

//=======================================================
// SRAM/VGA state machine
//=======================================================
// --Check for sram address=0 nonzero, which means that
//   HPS wrote some new data.
//
// --Read sram address 1 and 2 to get x1, y1 
//   left-most x, upper-most y
// --Read sram address 3 and 4 to get x2, y2
//   right-most x, lower-most y
// --Read sram address 5 to get color
// --write a rectangle to VGA
//
// --clear sram address=0 to signal HPS
//=======================================================
// Controls for Qsys sram slave exported in system module
//=======================================================
wire [31:0] sram_readdata ;
reg [31:0] data_buffer [NUM_MODULES-1:0];
reg [31:0] sram_writedata;
reg [7:0] sram_address; 
reg sram_write;
wire sram_clken = 1'b1;
wire sram_chipselect = 1'b1;
reg [7:0] state ;

// rectangle corners
reg [9:0] x1, y1, x2, y2 ;
reg [31:0] timer ; // may need to throttle write-rate
//=======================================================
// Controls for VGA memory
//=======================================================
//wire [31:0] vga_out_base_address = 32'h0000_0000 ;  // vga base addr
//reg [7:0] vga_sram_writedata ;
//reg [31:0] vga_sram_address; 
//reg vga_sram_write ;
//wire vga_sram_clken = 1'b1;
//wire vga_sram_chipselect = 1'b1;

//=======================================================
// pixel address is
reg [9:0] vga_x_cood [NUM_MODULES-1:0] ; 
reg [9:0] vga_y_cood [NUM_MODULES-1:0];
wire [7:0] pixel_color [NUM_MODULES-1:0];
wire ite_flag [NUM_MODULES-1:0] ;
reg ite_flag_prev [NUM_MODULES-1:0] ;
reg skip_flag_prev [NUM_MODULES-1:0] ;
reg signed [26:0] real_part [NUM_MODULES-1:0]; 
reg signed [26:0] imag_part [NUM_MODULES-1:0];
reg reset_solver[NUM_MODULES-1:0];
reg [NUM_MODULES-1:0] done ;
reg max_done;

reg [1:0] current_state [NUM_MODULES-1:0]; 
reg [1:0] next_state [NUM_MODULES-1:0]; 
reg [31:0] time_counter [NUM_MODULES-1:0] ;
reg [31:0] max_counter;
reg [31:0] max_counter_hex;
wire CLOCK_25;
wire CLOCK_150;
wire CLOCK_250;
wire [9:0] next_x;
wire [9:0] next_y;
reg [31:0] write_addr [NUM_MODULES-1:0]; 
reg [31:0] read_addr [NUM_MODULES-1:0];
reg [7:0] write_color [NUM_MODULES-1:0];
wire [7:0] read_color [NUM_MODULES-1:0];
reg we [NUM_MODULES-1:0];
wire [31:0] global_addr;
wire [7:0] data_out;

//parameter REAL_MIN = 27'd117440512;
//parameter REAL_MAX = 27'd125829120;
//parameter IMAG_MIN = 27'd125829120;
//parameter IMAG_MAX = 27'd8388608;
parameter SCREEN_WIDTH = 640;
parameter SCREEN_HEIGHT = 480;
parameter NUM_MODULES = 28;
parameter MEM_SIZE = 12000;  //11040
parameter ADDR_WIDTH = 15;
parameter BLOCK_ROW = 92;
parameter BLOCK_COL = 120;
parameter SOLVER_ROW = 27'd7;
parameter SOLVER_COL = 27'd4;

parameter LEFT_X_MIN = 27'd124403057;
parameter LEFT_X_MAX = 27'd127255184;
parameter LEFT_Y_MIN = 27'd132791665;
parameter LEFT_Y_MAX = 27'd1426063;

parameter RIGHT_X_MIN = 27'd130023424;
parameter RIGHT_X_MAX = 27'd2097152;
parameter RIGHT_Y_MIN = 27'd130023424;
parameter RIGHT_Y_MAX = 27'd4194304;



//parameter x_step = 27'd39321;
//parameter y_step = 27'd34952;
//parameter ROW_X = 27'd275247;
//parameter COL_Y = 27'd139808;

wire [31:0] xmod;
wire [31:0] ymod;
wire [31:0] xdiv;
wire [31:0] ydiv;
wire [31:0] module_select;
wire [31:0] local_addr;
reg [7:0] temp_color [NUM_MODULES-1:0];
wire [31:0] x_module_select;
wire [31:0] y_module_select;
wire [31:0] x_local_addr;
wire [31:0] y_local_addr;
reg [9:0] offset_addr_x[NUM_MODULES-1:0];
reg [9:0] offset_addr_y[NUM_MODULES-1:0];
reg [9:0] offset_real[NUM_MODULES-1:0];
reg [9:0] offset_imag[NUM_MODULES-1:0];
reg [4:0] index;

reg signed [26:0] ROW_X [NUM_MODULES-1:0];
reg signed [26:0] COL_Y [NUM_MODULES-1:0];
reg signed [26:0] rst_value_real [NUM_MODULES-1:0];
reg signed [26:0] rst_value_imag [NUM_MODULES-1:0];
reg signed [26:0] x_step = 27'd39321;
reg signed [26:0] y_step = 27'd34952;
reg signed [80:0] x_step_81 = {27'd0,27'd39321,27'd0};
reg signed [80:0] y_step_81 = {27'd0,27'd34952,27'd0};

reg signed [26:0] REAL_MIN = 27'd117440512;
reg signed [26:0] REAL_MAX = 27'd125829120;
reg signed [26:0] IMAG_MIN = 27'd125829120;
reg signed [26:0] IMAG_MAX = 27'd8388608;

reg signed [26:0] REAL_MIN_temp = 27'd117440512;
reg signed [26:0] REAL_MAX_temp = 27'd125829120;
reg signed [26:0] IMAG_MIN_temp = 27'd125829120;
reg signed [26:0] IMAG_MAX_temp = 27'd8388608;

reg [3:0] KEY_CURRENT;  

wire signed [26:0] zoom_level_Positive;
wire signed [26:0] zoom_level_negetive;
wire signed [3:0] zoom_level;

wire skip_flag [NUM_MODULES-1:0];

reg [15:0] max_iterations [NUM_MODULES-1:0];
reg [15:0] max_iterations_temp = 16'd1000;
wire [18:0] reorder_out;
reg [18:0] reorder_in;

//assign zoom_level_Positive = 27'd39321/x_step;
//assign zoom_level_negetive = x_step/27'd39321;

HexDigit Digit0(HEX0, (max_counter_hex/150000)%10 );
HexDigit Digit1(HEX1, max_counter_hex/1500000 );
HexDigit Digit2(HEX2, reorder_out[3:0]);
HexDigit Digit3(HEX3, {skip_flag[3], skip_flag[2], skip_flag[1], skip_flag[0]});
//assign HEX3 = 7'b1111111;
HexDigit Digit4(HEX4, NUM_MODULES%10 );
HexDigit Digit5(HEX5, NUM_MODULES/10 );


// use next_x and next_y to index into M10K memory
assign x_module_select = next_x % SOLVER_ROW;
assign y_module_select = next_y % SOLVER_COL;
assign x_local_addr = next_x / SOLVER_ROW;
assign y_local_addr = next_y / SOLVER_COL;
assign module_select = x_module_select + y_module_select * SOLVER_ROW;
assign local_addr = x_local_addr + y_local_addr * BLOCK_ROW;

genvar j;
generate
	for(j=0;j<NUM_MODULES;j=j+1) begin: VGA_mux
		always @(*) begin
			if(j==module_select) begin
				read_addr[j] = local_addr;
				temp_color[j] = read_color[j];
			end
			else begin
				read_addr[j] = 32'b0;
				temp_color[j] = 8'b0;
			end
		end
	end
endgenerate

assign data_out = temp_color[module_select[4:0]];



// memory state machine, each iterator has its own M10K memory.
genvar i;
generate
	for (i=0;i<NUM_MODULES;i=i+1) begin : gen_solvers
		mandelbrot_iterate insts(
			 .ci(imag_part[i]), 
			 .cr(real_part[i]), 
			 .max_iterations(max_iterations[i]), 
			 .ite_flag(ite_flag[i]), 
			 .color_reg(pixel_color[i]),
			 .clk(CLOCK_150), 
			 .reset(reset_solver[i] || (~KEY[0])),
			 .skip_flag(skip_flag[i]),
			 .i(i)
		);
		M10K M1( 
			 .q(read_color[i]),
			 .d(write_color[i]),
			 .write_address(write_addr[i]),
			 .read_address(read_addr[i]),
			 .we(we[i]), 
			 .clk(CLOCK_150)
		);
		

		// state machine
		// next state
		always @(posedge CLOCK_150) begin
			 if (~KEY[0]) begin
				  current_state[i] <= 2'd0;
			 end else begin
				  current_state[i] <= next_state[i];
			 end
		end

		// state changing logic 
		always @(*) begin
			 case (current_state[i])
				  2'd0: next_state[i] = 2'd2;
				  2'd2: next_state[i] = (offset_addr_y[i] > 10'd476) ? 2'd3 : 2'd2;
				  2'd3: next_state[i] = max_done? 2'd0: 2'd3;
				  default: next_state[i] = 2'd0;
			 endcase
		end

		// output logic
		always @(posedge CLOCK_150) begin
			 if (current_state[i] == 2'd0) begin  // reset
				  vga_x_cood[i] <= 10'b0 ;
				  vga_y_cood[i] <= 10'b0 ;
				  offset_addr_x[i] <= 10'b0;
				  offset_addr_y[i] <= 10'b0;
				  done[i] <= 1'b0;
				  REAL_MIN <= REAL_MIN_temp;
				  IMAG_MAX <= IMAG_MAX_temp;
				  rst_value_real[i] <= REAL_MIN_temp + (i[26:0]%SOLVER_ROW)*x_step;
				  rst_value_imag[i] <= IMAG_MAX_temp + (i[26:0]/SOLVER_ROW)*y_step;
				  real_part[i] <= REAL_MIN_temp+(i[26:0]%SOLVER_ROW)*x_step;
				  imag_part[i] <= IMAG_MAX_temp-(i[26:0]/SOLVER_ROW)*y_step;
				  ROW_X[i] <= x_step * SOLVER_ROW;
				  COL_Y[i] <= y_step * SOLVER_COL;
				  offset_real[i] <= REAL_MIN_temp;
				  offset_imag[i] <= IMAG_MAX_temp;

				  time_counter[i] <= 32'b0;
				  we[i]<=1'b0;
				  write_addr[i] <= 32'hFFFFFFFF; // FFFFFFFF
				  reset_solver[i] <= 0;
				  max_iterations[i] <= max_iterations_temp;
//				  write_color[i] <= pixel_color[i];
				  //index <= 0;
			 end
			 else if (current_state[i] == 2'd2) begin  // waiting for calculation result and write memory
				  time_counter[i] <= time_counter[i]+32'b1;
				  ite_flag_prev[i] <= ite_flag[i];
				  skip_flag_prev[i] <= skip_flag[i];

				  if( (ite_flag[i] && ~ite_flag_prev[i]) || (skip_flag[i] && ~skip_flag_prev[i]) )begin
						if(offset_addr_x[i] < 10'd637) begin
							offset_addr_x[i] <= offset_addr_x[i] + SOLVER_ROW;
							vga_x_cood[i] <= offset_addr_x[i] + i%SOLVER_ROW;
							
							offset_real[i] <= offset_real[i] + ROW_X[i];
							real_part[i] <= real_part[i] + ROW_X[i];
							
							offset_imag[i] <= offset_imag[i];
							
						end
						else if (offset_addr_x[i] == 10'd637) begin
							offset_addr_x[i] <= 10'b0;
							vga_x_cood[i] <= offset_addr_x[i] + i%SOLVER_ROW;
							
							offset_addr_y[i] <= offset_addr_y[i] + SOLVER_COL;
							vga_y_cood[i] <= offset_addr_y[i] + i/SOLVER_ROW;
							
							offset_real[i] <= REAL_MIN;
							real_part[i] <= rst_value_real[i];
							
							offset_imag[i] <= offset_imag[i] - COL_Y[i];
							imag_part[i] <= imag_part[i] - COL_Y[i];
						end
						
						write_addr[i] <= write_addr[i] + 1 ;
						we[i]<=1'b1;
						reset_solver[i] <= 1;
						
						if(skip_flag[i]) begin
							write_color[i] <= 8'b0;
						end
						else begin
							write_color[i] <= pixel_color[i];
						end
				  end
				  else begin
						
				  		reset_solver[i] <=0;
						we[i]<=1'b0;
				  end
			 end 
			 else if (current_state[i] == 2'd3) begin // done
				  time_counter[i]<=time_counter[i];
				  done[i] <= 1;
				  we[i]<=0;
				  write_addr[i] <= 32'hFFFFFFFF;
				  reset_solver[i] <= 1;
				  max_iterations[i] <= max_iterations_temp;
			 end
		end
	end
endgenerate

  
// HPS read state machine

reg [4:0] current_state2;
reg [4:0] next_state2;

// next state
always @(posedge CLOCK_50) begin
	if(~KEY[0]) begin
		current_state2 <= 5'd17;
	end
	else begin
		current_state2 <= next_state2;
	end
end


// state transition logic

always @(*) begin
	case(current_state2)
	5'd0: next_state2 = 5'd1;
	5'd1: next_state2 = 5'd2;
	5'd2: next_state2 = 5'd3;
	5'd3: next_state2 = 5'd4;
	5'd4: next_state2 = 5'd5;
	5'd5: next_state2 = 5'd6;
	5'd6: next_state2 = 5'd7;
	5'd7: next_state2 = 5'd8;
	5'd8: next_state2 = 5'd9;
	5'd9: next_state2 = 5'd10;
	5'd10: next_state2 = 5'd11;
	5'd11: next_state2 = 5'd12;
	5'd12: next_state2 = 5'd13;
	5'd13: next_state2 = 5'd14;
	5'd14: next_state2 = 5'd15;
	5'd15: next_state2 = 5'd16;
	5'd16: next_state2 = 5'd0;
	
	//reset
	5'd17: next_state2 = 5'd18;
	5'd18: next_state2 = 5'd19;
	5'd19: next_state2 = 5'd20;
	5'd20: next_state2 = 5'd21;
	5'd21: next_state2 = 5'd22;
	5'd22: next_state2 = 5'd23;
	5'd23: next_state2 = 5'd24;
	5'd24: next_state2 = 5'd0;
	default: next_state2 = 5'd0;
	endcase
end

// state machine output

always @(posedge CLOCK_50) begin
	// reading REAL_MIN
	if(current_state2 == 5'd0) begin
		sram_address <= 8'd3 ;
		sram_write <= 1'b0 ;
	end
	else if (current_state2 == 5'd1) begin
	end
	else if (current_state2 == 5'd2) begin
		REAL_MIN_temp <= sram_readdata;
      sram_write <= 1'b0;
	end 
	
	// reading IMAG_MAX
	else if(current_state2 == 5'd3) begin
		sram_address <= 8'd4 ;
		sram_write <= 1'b0 ;
	end
	else if (current_state2 == 5'd4) begin
	end
	else if (current_state2 == 5'd5) begin
		IMAG_MAX_temp <= sram_readdata;
      sram_write <= 1'b0;
	end 
	
	// reading x_step
	else if(current_state2 ==5'd6) begin
		sram_address <= 8'd5 ;
		sram_write <= 1'b0 ;
	end
	else if (current_state2 == 5'd7) begin
	end
	else if (current_state2 == 5'd8) begin
		x_step <= sram_readdata;
      sram_write <= 1'b0;
	end 
	
	// reading y_step
	else if(current_state2 == 5'd9) begin
		sram_address <= 8'd6 ;
		sram_write <= 1'b0 ;
	end
	else if (current_state2 == 5'd10) begin
	end
	else if (current_state2 == 5'd11) begin
		y_step <= sram_readdata;
      sram_write <= 1'b0;
	end 
	
	// reading max_iterations
	else if(current_state2 == 5'd12) begin
		sram_address <= 8'd7 ;
		sram_write <= 1'b0 ;
	end
	else if (current_state2 == 5'd13) begin
	end
	else if (current_state2 == 5'd14) begin
		max_iterations_temp <= sram_readdata[15:0];
      sram_write <= 1'b0;
	end 
	
	//writing max_counter_hex
	else if (current_state2 == 5'd15) begin
		sram_address <= 8'd1 ;
		sram_write<= 1'b1 ;
		sram_writedata <= max_counter_hex ;
	end
	else if (current_state2 == 5'd16) begin
	
	end
	
	//writing real_min
	else if (current_state2 == 5'd17) begin
		sram_address <= 8'd3 ;
		sram_write<= 1'b1 ;
		sram_writedata <= 27'd117440512 ;
	end
	else if (current_state2 == 5'd18) begin
	
	end
	
	//writing imag_max
	else if (current_state2 == 5'd19) begin
		sram_address <= 8'd4 ;
		sram_write<= 1'b1 ;
		sram_writedata <= 27'd8388608 ;
	end
	else if (current_state2 == 5'd20) begin
	
	end
	
	//writing x_step
	else if (current_state2 == 5'd21) begin
		sram_address <= 8'd5 ;
		sram_write<= 1'b1 ;
		sram_writedata <= 27'd39321 ;
	end
	else if (current_state2 == 5'd22) begin
	
	end
	
	//writing y_step
	else if (current_state2 == 5'd23) begin
		sram_address <= 8'd6 ;
		sram_write<= 1'b1 ;
		sram_writedata <= 27'd34952 ;
	end
	else if (current_state2 == 5'd24) begin
	
	end
	
//	//writing HPS_flag
//	else if (current_state2 == 5'd25) begin
//		sram_address <= 8'd8 ;
//		sram_write<= 1'b1 ;
//		sram_writedata <= 27'd34952 ;
//	end
//	else if (current_state2 == 5'd26) begin
//	
//	end
end



// calculate the max_counter for rendering time
always @(posedge CLOCK_150) begin
	if(~KEY[0]) begin
		index <= 0;
		max_counter <= 0;
		max_done <= 0 ;
	end
	else begin
		if(done == 28'hFFFFFFF) begin
			if(index < NUM_MODULES) begin
				max_done <= 0 ;
				if(time_counter[index] > max_counter) begin
					max_counter <= time_counter[index];
				end
				else begin
					max_counter <= max_counter;
				end
				index <= index + 1;
			end
			else begin
				max_done <= 1;
				max_counter <= 0;
				max_counter_hex <= max_counter;
//				sram_address <= 8'd1 ;
//				sram_write<= 1'b1 ;
//				sram_writedata <= SW[8] ? (SW[7] ? x_step : y_step) : max_counter ;
				index <= 0;
			end
		end
		else begin
			max_counter <= 0;
//			sram_address <= 8'd1 ;
//			sram_write<= 1'b0 ;
//			sram_writedata <= max_counter ;
			index <= 0;
		end
	end
end

video_pll pll1 (
		.refclk(CLOCK_50),   //  refclk.clk
		.rst(~KEY[0]),      //   reset.reset
		.outclk_0(CLOCK_25), // outclk0.clk
		.locked()    //  locked.export
	);
	
pll2 pll2 (
		.refclk(CLOCK_50),   //  refclk.clk
		.rst(~KEY[0]),      //   reset.reset
		.outclk_0(CLOCK_150), // outclk0.clk
		.locked()    //  locked.export
	);
	
//ite_pll ite_pll (
//		.refclk(CLOCK2_50),   //  refclk.clk
//		.rst(~KEY[0]),      //   reset.reset
//		.outclk_0(CLOCK_250), // outclk0.clk
//		.locked()    //  locked.export
//	);

vga_driver VGA1 (
    .clock(CLOCK_25),     // 25 MHz
    .reset(~KEY[0]),     // Active high
    .color_in(data_out), // Pixel color data (RRRGGGBB)
    .next_x(next_x),  // x-coordinate of NEXT pixel that will be drawn
    .next_y(next_y),  // y-coordinate of NEXT pixel that will be drawn
    .hsync(VGA_HS),    // HSYNC (to VGA connector)
    .vsync(VGA_VS),    // VSYNC (to VGA connctor)
    .red(VGA_R),     // RED (to resistor DAC VGA connector)
    .green(VGA_G),   // GREEN (to resistor DAC to VGA connector)
    .blue(VGA_B),    // BLUE (to resistor DAC to VGA connector)
    .sync(VGA_SYNC_N),          // SYNC to VGA connector
    .clk(VGA_CLK),           // CLK to VGA connector
    .blank(VGA_BLANK_N)          // BLANK to VGA connector
);

M10K_reorder M_reorder( 
	 .q(reorder_out),
	 .d({SW[9:0],SW[8:0]}),
	 .write_address({SW[9:0],SW[9:0],SW[9:0]}),
	 .read_address({SW[9:0],SW[9:0],SW[9:0]}),
	 .we(~KEY[1]), 
	 .clk(CLOCK_150)
);

//=======================================================
//  Structural coding
//=======================================================
// From Qsys

Computer_System The_System (
	////////////////////////////////////
	// FPGA Side
	////////////////////////////////////

	// Global signals
	.system_pll_ref_clk_clk					(CLOCK_50),
	.system_pll_ref_reset_reset			(1'b0),
	
	// SRAM shared block with HPS
	.onchip_sram_s1_address               (sram_address),               
	.onchip_sram_s1_clken                 (sram_clken),                 
	.onchip_sram_s1_chipselect            (sram_chipselect),            
	.onchip_sram_s1_write                 (sram_write),                 
	.onchip_sram_s1_readdata              (sram_readdata),              
	.onchip_sram_s1_writedata             (sram_writedata),             
	.onchip_sram_s1_byteenable            (4'b1111), 
	
//	//  sram to video
//	.onchip_vga_buffer_s1_address    (vga_sram_address),    
//	.onchip_vga_buffer_s1_clken      (vga_sram_clken),      
//	.onchip_vga_buffer_s1_chipselect (vga_sram_chipselect), 
//	.onchip_vga_buffer_s1_write      (vga_sram_write),      
//	.onchip_vga_buffer_s1_readdata   (),   // never read from vga here
//	.onchip_vga_buffer_s1_writedata  (vga_sram_writedata),   
//
//	// AV Config
//	.av_config_SCLK							(FPGA_I2C_SCLK),
//	.av_config_SDAT							(FPGA_I2C_SDAT),
//
	// 50 MHz clock bridge
	.clock_bridge_0_in_clk_clk            (CLOCK_50), //(CLOCK_50), 
	
//	// VGA Subsystem
//	.vga_pll_ref_clk_clk 					(CLOCK2_50),
//	.vga_pll_ref_reset_reset				(1'b0),
//	.vga_CLK										(VGA_CLK),
//	.vga_BLANK									(VGA_BLANK_N),
//	.vga_SYNC									(VGA_SYNC_N),
//	.vga_HS										(VGA_HS),
//	.vga_VS										(VGA_VS),
//	.vga_R										(VGA_R),
//	.vga_G										(VGA_G),
//	.vga_B										(VGA_B),
		
//	// SDRAM
//	.sdram_clk_clk								(DRAM_CLK),
//   .sdram_addr									(DRAM_ADDR),
//	.sdram_ba									(DRAM_BA),
//	.sdram_cas_n								(DRAM_CAS_N),
//	.sdram_cke									(DRAM_CKE),
//	.sdram_cs_n									(DRAM_CS_N),
//	.sdram_dq									(DRAM_DQ),
//	.sdram_dqm									({DRAM_UDQM,DRAM_LDQM}),
//	.sdram_ras_n								(DRAM_RAS_N),
//	.sdram_we_n									(DRAM_WE_N),
	
	////////////////////////////////////
	// HPS Side
	////////////////////////////////////
	// DDR3 SDRAM
	.memory_mem_a			(HPS_DDR3_ADDR),
	.memory_mem_ba			(HPS_DDR3_BA),
	.memory_mem_ck			(HPS_DDR3_CK_P),
	.memory_mem_ck_n		(HPS_DDR3_CK_N),
	.memory_mem_cke		(HPS_DDR3_CKE),
	.memory_mem_cs_n		(HPS_DDR3_CS_N),
	.memory_mem_ras_n		(HPS_DDR3_RAS_N),
	.memory_mem_cas_n		(HPS_DDR3_CAS_N),
	.memory_mem_we_n		(HPS_DDR3_WE_N),
	.memory_mem_reset_n	(HPS_DDR3_RESET_N),
	.memory_mem_dq			(HPS_DDR3_DQ),
	.memory_mem_dqs		(HPS_DDR3_DQS_P),
	.memory_mem_dqs_n		(HPS_DDR3_DQS_N),
	.memory_mem_odt		(HPS_DDR3_ODT),
	.memory_mem_dm			(HPS_DDR3_DM),
	.memory_oct_rzqin		(HPS_DDR3_RZQ),
		  
	// Ethernet
	.hps_io_hps_io_gpio_inst_GPIO35	(HPS_ENET_INT_N),
	.hps_io_hps_io_emac1_inst_TX_CLK	(HPS_ENET_GTX_CLK),
	.hps_io_hps_io_emac1_inst_TXD0	(HPS_ENET_TX_DATA[0]),
	.hps_io_hps_io_emac1_inst_TXD1	(HPS_ENET_TX_DATA[1]),
	.hps_io_hps_io_emac1_inst_TXD2	(HPS_ENET_TX_DATA[2]),
	.hps_io_hps_io_emac1_inst_TXD3	(HPS_ENET_TX_DATA[3]),
	.hps_io_hps_io_emac1_inst_RXD0	(HPS_ENET_RX_DATA[0]),
	.hps_io_hps_io_emac1_inst_MDIO	(HPS_ENET_MDIO),
	.hps_io_hps_io_emac1_inst_MDC		(HPS_ENET_MDC),
	.hps_io_hps_io_emac1_inst_RX_CTL	(HPS_ENET_RX_DV),
	.hps_io_hps_io_emac1_inst_TX_CTL	(HPS_ENET_TX_EN),
	.hps_io_hps_io_emac1_inst_RX_CLK	(HPS_ENET_RX_CLK),
	.hps_io_hps_io_emac1_inst_RXD1	(HPS_ENET_RX_DATA[1]),
	.hps_io_hps_io_emac1_inst_RXD2	(HPS_ENET_RX_DATA[2]),
	.hps_io_hps_io_emac1_inst_RXD3	(HPS_ENET_RX_DATA[3]),

	// Flash
	.hps_io_hps_io_qspi_inst_IO0	(HPS_FLASH_DATA[0]),
	.hps_io_hps_io_qspi_inst_IO1	(HPS_FLASH_DATA[1]),
	.hps_io_hps_io_qspi_inst_IO2	(HPS_FLASH_DATA[2]),
	.hps_io_hps_io_qspi_inst_IO3	(HPS_FLASH_DATA[3]),
	.hps_io_hps_io_qspi_inst_SS0	(HPS_FLASH_NCSO),
	.hps_io_hps_io_qspi_inst_CLK	(HPS_FLASH_DCLK),

	// Accelerometer
	.hps_io_hps_io_gpio_inst_GPIO61	(HPS_GSENSOR_INT),

	//.adc_sclk                        (ADC_SCLK),
	//.adc_cs_n                        (ADC_CS_N),
	//.adc_dout                        (ADC_DOUT),
	//.adc_din                         (ADC_DIN),

	// General Purpose I/O
	.hps_io_hps_io_gpio_inst_GPIO40	(HPS_GPIO[0]),
	.hps_io_hps_io_gpio_inst_GPIO41	(HPS_GPIO[1]),

	// I2C
	.hps_io_hps_io_gpio_inst_GPIO48	(HPS_I2C_CONTROL),
	.hps_io_hps_io_i2c0_inst_SDA		(HPS_I2C1_SDAT),
	.hps_io_hps_io_i2c0_inst_SCL		(HPS_I2C1_SCLK),
	.hps_io_hps_io_i2c1_inst_SDA		(HPS_I2C2_SDAT),
	.hps_io_hps_io_i2c1_inst_SCL		(HPS_I2C2_SCLK),

	// Pushbutton
	.hps_io_hps_io_gpio_inst_GPIO54	(HPS_KEY),

	// LED
	.hps_io_hps_io_gpio_inst_GPIO53	(HPS_LED),

	// SD Card
	.hps_io_hps_io_sdio_inst_CMD	(HPS_SD_CMD),
	.hps_io_hps_io_sdio_inst_D0	(HPS_SD_DATA[0]),
	.hps_io_hps_io_sdio_inst_D1	(HPS_SD_DATA[1]),
	.hps_io_hps_io_sdio_inst_CLK	(HPS_SD_CLK),
	.hps_io_hps_io_sdio_inst_D2	(HPS_SD_DATA[2]),
	.hps_io_hps_io_sdio_inst_D3	(HPS_SD_DATA[3]),

	// SPI
	.hps_io_hps_io_spim1_inst_CLK		(HPS_SPIM_CLK),
	.hps_io_hps_io_spim1_inst_MOSI	(HPS_SPIM_MOSI),
	.hps_io_hps_io_spim1_inst_MISO	(HPS_SPIM_MISO),
	.hps_io_hps_io_spim1_inst_SS0		(HPS_SPIM_SS),

	// UART
	.hps_io_hps_io_uart0_inst_RX	(HPS_UART_RX),
	.hps_io_hps_io_uart0_inst_TX	(HPS_UART_TX),

	// USB
	.hps_io_hps_io_gpio_inst_GPIO09	(HPS_CONV_USB_N),
	.hps_io_hps_io_usb1_inst_D0		(HPS_USB_DATA[0]),
	.hps_io_hps_io_usb1_inst_D1		(HPS_USB_DATA[1]),
	.hps_io_hps_io_usb1_inst_D2		(HPS_USB_DATA[2]),
	.hps_io_hps_io_usb1_inst_D3		(HPS_USB_DATA[3]),
	.hps_io_hps_io_usb1_inst_D4		(HPS_USB_DATA[4]),
	.hps_io_hps_io_usb1_inst_D5		(HPS_USB_DATA[5]),
	.hps_io_hps_io_usb1_inst_D6		(HPS_USB_DATA[6]),
	.hps_io_hps_io_usb1_inst_D7		(HPS_USB_DATA[7]),
	.hps_io_hps_io_usb1_inst_CLK		(HPS_USB_CLKOUT),
	.hps_io_hps_io_usb1_inst_STP		(HPS_USB_STP),
	.hps_io_hps_io_usb1_inst_DIR		(HPS_USB_DIR),
	.hps_io_hps_io_usb1_inst_NXT		(HPS_USB_NXT)
);
endmodule // end top level

//============================================================
// M10K module for testing
//============================================================
// See example 12-16 in 
// http://people.ece.cornell.edu/land/courses/ece5760/DE1_SOC/HDL_style_qts_qii51007.pdf
//============================================================

module M10K( 
    output reg [7:0] q,
    input [7:0] d,
    input [31:0] write_address, read_address,
    input we, clk
);
	 // force M10K ram style
	 // 256 words of 8 bits
    reg [7:0] mem [12000:0]  /* synthesis ramstyle = "no_rw_check, M10K" */;
	 reg [31:0] read_addr;
	 
    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= d;
		  end
		  q <= mem[read_address];
    end
endmodule

module M10K_reorder( 
    output reg [18:0] q,
    input [18:0] d,
    input [31:0] write_address, read_address,
    input we, clk
);
	 // force M10K ram style
	 // 256 words of 8 bits
    reg [18:0] mem [307200:0]  /* synthesis ramstyle = "no_rw_check, M10K" */;
	 reg [31:0] read_addr;
	 
    always @ (posedge clk) begin
        if (we) begin
            mem[write_address] <= d;
		  end
		  q <= mem[read_address];
    end
endmodule



module mandelbrot_iterate(
    input signed [26:0] ci, cr,
    input [15:0] max_iterations,
    input clk,
    output reg ite_flag,
	output reg [7:0] color_reg,
    input reset,
	 output reg skip_flag,
	 input [31:0] i
);
// black region
parameter LEFT_X_MIN = 27'd124403057;
parameter LEFT_X_MAX = 27'd127255184;
parameter LEFT_Y_MIN = 27'd132791665;
parameter LEFT_Y_MAX = 27'd1426063;

parameter RIGHT_X_MIN = 27'd130023424;
parameter RIGHT_X_MAX = 27'd2097152;
parameter RIGHT_Y_MIN = 27'd130023424;
parameter RIGHT_Y_MAX = 27'd4194304;

reg signed [26:0] zi, zr;
reg signed [26:0] zi_temp, zr_temp;
wire signed [26:0] zi_squared, zr_squared, zrmulzi;
reg [15:0] iterations;


signed_mult inst1(zi_squared, zi, zi);
signed_mult inst2(zr_squared, zr, zr);
signed_mult inst3(zrmulzi, zr, zi);

always @(posedge clk) begin
    if (reset == 1'b1) begin
        iterations <= 0;
        zi <= 0;
        zr <= 0;
		  ite_flag <= 1'b0;
		  skip_flag <= 1'b0;
    end 
    else begin
		  if (iterations < max_iterations && (zr_squared + zi_squared) <= (4 << 23)) begin
            zr <= zr_squared - zi_squared + cr;
            zi <= (zrmulzi << 1) + ci;
            iterations <= iterations + 1;
				ite_flag <= 1'b0;
        end
		  else begin
		      ite_flag <= 1'b1;
		  end
		  
		  if ((($signed(cr)<=$signed(LEFT_X_MAX)) && ($signed(cr)>=$signed(LEFT_X_MIN)) && ($signed(ci)<=$signed(LEFT_Y_MAX)) && ($signed(ci)>=$signed(LEFT_Y_MIN)) ) ||
		  (($signed(cr)<=$signed(RIGHT_X_MAX)) && ($signed(cr)>=$signed(RIGHT_X_MIN)) && ($signed(ci)<=$signed(RIGHT_Y_MAX)) && ($signed(ci)>=$signed(RIGHT_Y_MIN)) )) begin
				skip_flag <= 1'b1;
		  end
		  else begin
				skip_flag <= 1'b0;
		  end
//		  ite_flag <= ((zr_squared + zi_squared) > (4 << 23)) | (iterations >= max_iterations);
    end
	if (iterations >= max_iterations) begin
	color_reg <= 8'b_000_000_00 ; // black
	end
	else if (iterations >= (max_iterations >>> 1)) begin
	color_reg <= 8'b_011_001_00 ;
	end
	else if (iterations >= (max_iterations >>> 2)) begin
	color_reg <= 8'b_011_001_00 ;
	end
	else if (iterations >= (max_iterations >>> 3)) begin
	color_reg <= 8'b_101_010_01 ;
	end
	else if (iterations >= (max_iterations >>> 4)) begin
	color_reg <= 8'b_011_001_01 ;
	end
	else if (iterations >= (max_iterations >>> 5)) begin
	color_reg <= 8'b_001_001_01 ;
	end
	else if (iterations >= (max_iterations >>> 6)) begin
	color_reg <= 8'b_011_010_10 ;
	end
	else if (iterations >= (max_iterations >>> 7)) begin
	color_reg <= 8'b_010_100_10 ;
	end
	else if (iterations >= (max_iterations >>> 8)) begin
	color_reg <= 8'b_010_100_10 ;
	end
	else begin
	color_reg <= 8'b_010_100_10 ;
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

//VGA Driver Written by Hunter
module vga_driver (
    input wire clock,     // 25 MHz
    input wire reset,     // Active high
    input [7:0] color_in, // Pixel color data (RRRGGGBB)
    output [9:0] next_x,  // x-coordinate of NEXT pixel that will be drawn
    output [9:0] next_y,  // y-coordinate of NEXT pixel that will be drawn
    output wire hsync,    // HSYNC (to VGA connector)
    output wire vsync,    // VSYNC (to VGA connctor)
    output [7:0] red,     // RED (to resistor DAC VGA connector)
    output [7:0] green,   // GREEN (to resistor DAC to VGA connector)
    output [7:0] blue,    // BLUE (to resistor DAC to VGA connector)
    output sync,          // SYNC to VGA connector
    output clk,           // CLK to VGA connector
    output blank          // BLANK to VGA connector
);

    // Horizontal parameters (measured in clock cycles)
    parameter [9:0] H_ACTIVE  =  10'd_639 ;
    parameter [9:0] H_FRONT   =  10'd_15 ;
    parameter [9:0] H_PULSE   =  10'd_95 ;
    parameter [9:0] H_BACK    =  10'd_47 ;

    // Vertical parameters (measured in lines)
    parameter [9:0] V_ACTIVE   =  10'd_479 ;
    parameter [9:0] V_FRONT    =  10'd_9 ;
    parameter [9:0] V_PULSE =  10'd_1 ;
    parameter [9:0] V_BACK  =  10'd_32 ;

    // Parameters for readability
    parameter   LOW     = 1'b_0 ;
    parameter   HIGH    = 1'b_1 ;

    // States (more readable)
    parameter   [7:0]    H_ACTIVE_STATE    = 8'd_0 ;
    parameter   [7:0]   H_FRONT_STATE     = 8'd_1 ;
    parameter   [7:0]   H_PULSE_STATE   = 8'd_2 ;
    parameter   [7:0]   H_BACK_STATE     = 8'd_3 ;

    parameter   [7:0]    V_ACTIVE_STATE    = 8'd_0 ;
    parameter   [7:0]   V_FRONT_STATE    = 8'd_1 ;
    parameter   [7:0]   V_PULSE_STATE   = 8'd_2 ;
    parameter   [7:0]   V_BACK_STATE     = 8'd_3 ;

    // Clocked registers
    reg              hysnc_reg ;
    reg              vsync_reg ;
    reg     [7:0]   red_reg ;
    reg     [7:0]   green_reg ;
    reg     [7:0]   blue_reg ;
    reg              line_done ;

    // Control registers
    reg     [9:0]   h_counter ;
    reg     [9:0]   v_counter ;

    reg     [7:0]    h_state ;
    reg     [7:0]    v_state ;

    // State machine
    always@(posedge clock) begin
        // At reset . . .
        if (reset) begin
            // Zero the counters
            h_counter   <= 10'd_0 ;
            v_counter   <= 10'd_0 ;
            // States to ACTIVE
            h_state     <= H_ACTIVE_STATE  ;
            v_state     <= V_ACTIVE_STATE  ;
            // Deassert line done
            line_done   <= LOW ;
        end
        else begin
            //////////////////////////////////////////////////////////////////////////
            ///////////////////////// HORIZONTAL /////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////
            if (h_state == H_ACTIVE_STATE) begin
                // Iterate horizontal counter, zero at end of ACTIVE mode
                h_counter <= (h_counter==H_ACTIVE)?10'd_0:(h_counter + 10'd_1) ;
                // Set hsync
                hysnc_reg <= HIGH ;
                // Deassert line done
                line_done <= LOW ;
                // State transition
                h_state <= (h_counter == H_ACTIVE)?H_FRONT_STATE:H_ACTIVE_STATE ;
            end
            if (h_state == H_FRONT_STATE) begin
                // Iterate horizontal counter, zero at end of H_FRONT mode
                h_counter <= (h_counter==H_FRONT)?10'd_0:(h_counter + 10'd_1) ;
                // Set hsync
                hysnc_reg <= HIGH ;
                // State transition
                h_state <= (h_counter == H_FRONT)?H_PULSE_STATE:H_FRONT_STATE ;
            end
            if (h_state == H_PULSE_STATE) begin
                // Iterate horizontal counter, zero at end of H_PULSE mode
                h_counter <= (h_counter==H_PULSE)?10'd_0:(h_counter + 10'd_1) ;
                // Clear hsync
                hysnc_reg <= LOW ;
                // State transition
                h_state <= (h_counter == H_PULSE)?H_BACK_STATE:H_PULSE_STATE ;
            end
            if (h_state == H_BACK_STATE) begin
                // Iterate horizontal counter, zero at end of H_BACK mode
                h_counter <= (h_counter==H_BACK)?10'd_0:(h_counter + 10'd_1) ;
                // Set hsync
                hysnc_reg <= HIGH ;
                // State transition
                h_state <= (h_counter == H_BACK)?H_ACTIVE_STATE:H_BACK_STATE ;
                // Signal line complete at state transition (offset by 1 for synchronous state transition)
                line_done <= (h_counter == (H_BACK-1))?HIGH:LOW ;
            end
            //////////////////////////////////////////////////////////////////////////
            ///////////////////////// VERTICAL ///////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////
            if (v_state == V_ACTIVE_STATE) begin
                // increment vertical counter at end of line, zero on state transition
                v_counter<=(line_done==HIGH)?((v_counter==V_ACTIVE)?10'd_0:(v_counter+10'd_1)):v_counter ;
                // set vsync in active mode
                vsync_reg <= HIGH ;
                // state transition - only on end of lines
                v_state<=(line_done==HIGH)?((v_counter==V_ACTIVE)?V_FRONT_STATE:V_ACTIVE_STATE):V_ACTIVE_STATE ;
            end
            if (v_state == V_FRONT_STATE) begin
                // increment vertical counter at end of line, zero on state transition
                v_counter<=(line_done==HIGH)?((v_counter==V_FRONT)?10'd_0:(v_counter + 10'd_1)):v_counter ;
                // set vsync in front porch
                vsync_reg <= HIGH ;
                // state transition
                v_state<=(line_done==HIGH)?((v_counter==V_FRONT)?V_PULSE_STATE:V_FRONT_STATE):V_FRONT_STATE;
            end
            if (v_state == V_PULSE_STATE) begin
                // increment vertical counter at end of line, zero on state transition
                v_counter<=(line_done==HIGH)?((v_counter==V_PULSE)?10'd_0:(v_counter + 10'd_1)):v_counter ;
                // clear vsync in pulse
                vsync_reg <= LOW ;
                // state transition
                v_state<=(line_done==HIGH)?((v_counter==V_PULSE)?V_BACK_STATE:V_PULSE_STATE):V_PULSE_STATE;
            end
            if (v_state == V_BACK_STATE) begin
                // increment vertical counter at end of line, zero on state transition
                v_counter<=(line_done==HIGH)?((v_counter==V_BACK)?10'd_0:(v_counter + 10'd_1)):v_counter ;
                // set vsync in back porch
                vsync_reg <= HIGH ;
                // state transition
                v_state<=(line_done==HIGH)?((v_counter==V_BACK)?V_ACTIVE_STATE:V_BACK_STATE):V_BACK_STATE ;
            end

            //////////////////////////////////////////////////////////////////////////
            //////////////////////////////// COLOR OUT ///////////////////////////////
            //////////////////////////////////////////////////////////////////////////
            // Assign colors if in active mode
            red_reg<=(h_state==H_ACTIVE_STATE)?((v_state==V_ACTIVE_STATE)?{color_in[7:5],5'd_0}:8'd_0):8'd_0 ;
            green_reg<=(h_state==H_ACTIVE_STATE)?((v_state==V_ACTIVE_STATE)?{color_in[4:2],5'd_0}:8'd_0):8'd_0 ;
            blue_reg<=(h_state==H_ACTIVE_STATE)?((v_state==V_ACTIVE_STATE)?{color_in[1:0],6'd_0}:8'd_0):8'd_0 ;

        end
    end
    // Assign output values - to VGA connector
    assign hsync = hysnc_reg ;
    assign vsync = vsync_reg ;
    assign red = red_reg ;
    assign green = green_reg ;
    assign blue = blue_reg ;
    assign clk = clock ;
    assign sync = 1'b_0 ;
    assign blank = hysnc_reg & vsync_reg ;
    // The x/y coordinates that should be available on the NEXT cycle
    assign next_x = (h_state==H_ACTIVE_STATE)?h_counter:10'd_0 ;
    assign next_y = (v_state==V_ACTIVE_STATE)?v_counter:10'd_0 ;

endmodule