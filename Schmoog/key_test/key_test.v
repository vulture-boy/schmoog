
module key_test(
	// Onboard Devices
	CLOCK_50, CLOCK2_50, KEY, LEDR,
	
	// Keyboard
	PS2_DAT, PS2_CLK,
	
	// Audio
	FPGA_I2C_SCLK, FPGA_I2C_SDAT, AUD_XCK, 
	AUD_DACLRCK, AUD_ADCLRCK, AUD_BCLK, 
	AUD_ADCDAT, AUD_DACDAT,
	
	// VGA
	VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N,
	VGA_SYNC_N, VGA_R, VGA_G, VGA_B
	);

	/* INITIALIZATION */
	
	input CLOCK_50, CLOCK2_50;
	input [0:0] KEY;
	output [9:0] LEDR;
	
	// Keyboard
	input PS2_DAT;
	input PS2_CLK;
	
	// Audio: I2C Audio/Video config interface
	output FPGA_I2C_SCLK;
	inout FPGA_I2C_SDAT;
	// Audio CODEC
	input AUD_DACLRCK, AUD_ADCLRCK, AUD_BCLK;
	input AUD_ADCDAT;
	output AUD_XCK;
	output AUD_DACDAT;
	
	// VGA
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	
	// Reset_n; 1 is active (audio, keyboard)
	wire reset_n;
	assign reset_n = ~KEY[0];
	// Reset; 0 is active (VGA)
	wire reset; 
	assign reset = KEY[0];
	
	
	/* KEYBOARD MODULE */
	
	// Valid high when press / release detected
	// Indicates whether a press (1) or release (0) occured
	wire valid, makeBreak;
	// Byte code indicating a given key's data (use hex)
	wire [7:0] outCode;
	// Interpreted key signals
	wire [15:0] keyboard_key;
	
	keyboard_press_driver keys(
	  .CLOCK_50(CLOCK_50), 
	  .valid(valid), 
	  .makeBreak(makeBreak),
	  .outCode(outCode), 
	  .PS2_DAT(PS2_DAT), // PS2 data line
	  .PS2_CLK(PS2_CLK), // PS2 clock line
	  .reset(reset_n)
	);
	
	key_interpreter key_int(
		.clock(CLOCK_50),
		.reset(reset_n),
		.makeBreak(makeBreak),
		.valid(valid),
		.outCode(outCode),
		.keyboard_key(keyboard_key)
	); 
	
	assign LEDR[0] = keyboard_key[0];
	assign LEDR[1] = keyboard_key[1];
	assign LEDR[2] = keyboard_key[2];
	assign LEDR[3] = keyboard_key[3];
	assign LEDR[4] = keyboard_key[4];
	
	/* AUDIO MODULE */

	wire read_ready, write_ready; // CODEC ready for read/write operation
	wire read, write; // send data from / to the CODEC (both channels)
	wire [23:0] readdata_left, readdata_right; // left and right channel data from the CODEC
	wire [23:0] writedata_left, writedata_right; // left and right channel data to the CODEC
	// AUD_* - should connect to top-level entity I/O of the same name.
	//         These signals go directly to the Audio CODEC
	// I2C_* - should connect to top-level entity I/O of the same name.
	//         These signals go directly to the Audio/Video Config module
	
	// Output Only 
	assign read = 1'b0;
	assign write = 1'b1;
	
	// Schmoog-specific wirings (send to datapath?)
	wire [2:0] current_octave; // Current Octave selected
	wire [7:0] wave_out1; // Create additional for multiple keys
	
	//TEMP DEFAULTS
	assign current_octave = 3'b100;
	
	// Produces one wave (make more for more keys)
	wave_creator wav1(
		.clk(CLOCK_50),
		.enable(keyboard_key[4]), // Replace with 'sound is playing'
		.reset_n(reset_n),
		.note(4'b0000), // Replace with 'note_wave[0]'
		.octave(current_octave), // TODO: Default to 4, consider making array?
		.wave(2'b00),
		.pulse(wave_out1) 
	);
	
	// USE THESE TO FEED INFORMATION TO THE CODEC
	assign writedata_left = 24'd62500 * wave_out1;
	assign writedata_right = 24'd62500 * wave_out1;
	
	clock_generator my_clock_gen(
		// inputs
		.CLOCK2_50(CLOCK2_50),
		.reset(reset_n),

		// outputs
		.AUD_XCK(AUD_XCK)
	);

	audio_and_video_config cfg(
		// Inputs
		.clk(CLOCK_50),
		.reset(reset_n),

		// Bidirectionals
		.I2C_SDAT(FPGA_I2C_SDAT),
		.I2C_SCLK(FPGA_I2C_SCLK)
	);

	audio_codec codec(
		// Inputs
		.clk(CLOCK_50),
		.reset(reset_n),

		.read(read),	.write(write),
		.writedata_left(writedata_left), .writedata_right(writedata_right),

		.AUD_ADCDAT(AUD_ADCDAT),

		// Bidirectionals
		.AUD_BCLK(AUD_BCLK),
		.AUD_ADCLRCK(AUD_ADCLRCK),
		.AUD_DACLRCK(AUD_DACLRCK),

		// Outputs
		.read_ready(read_ready), .write_ready(write_ready),
		.readdata_left(readdata_left), .readdata_right(readdata_right),
		.AUD_DACDAT(AUD_DACDAT)
	);
	
	/* VGA MODULE */
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;
	
	// TEMP values
	assign colour = 3'b000;
	assign x = 8'b0000_0000;
	assign y = 7'b0000_000;
	assign writeEn = 1'b0;

	// VGA Adapter Module
	vga_adapter VGA(
			.resetn(reset),
			.clock(CLOCK_50),
			.colour(colour),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
		defparam VGA.BACKGROUND_IMAGE = "logo.colour.mif"; 
	
endmodule

/* Interprets keyboard input from the PS2 driver and outputs
 *		keyboard_key signals that are used by the Schmoog
 *	
 */
module key_interpreter(clock, reset, makeBreak, outCode, valid, keyboard_key);
	input makeBreak, valid, reset, clock;
	input [7:0] outCode;
	output reg [15:0] keyboard_key;
	
	always @(posedge clock) 
		begin
		if (reset)
			begin
				keyboard_key <= 16'd0;
			end
		else
			begin
			if (valid)
				begin
				if (makeBreak) // Make (press)
					begin
					case (outCode) // BASED ON SET 2
						// ROW 1
						8'h15: keyboard_key <= {keyboard_key[15:1], 1'b1}; // Q
						8'h1d: keyboard_key <= {keyboard_key[15:2], 1'b1, keyboard_key[0:0]}; // W
						8'h24: keyboard_key <= {keyboard_key[15:3], 1'b1, keyboard_key[1:0]}; // E
						8'h2d: keyboard_key <= {keyboard_key[15:4], 1'b1, keyboard_key[2:0]}; // R
						8'h2c: keyboard_key <= {keyboard_key[15:5], 1'b1, keyboard_key[3:0]}; // T
						8'h35: keyboard_key <= {keyboard_key[15:6], 1'b1, keyboard_key[4:0]}; // Y
						8'h3c: keyboard_key <= {keyboard_key[15:7], 1'b1, keyboard_key[5:0]}; // U
						8'h43: keyboard_key <= {keyboard_key[15:8], 1'b1, keyboard_key[6:0]}; // I
						8'h44: keyboard_key <= {keyboard_key[15:9], 1'b1, keyboard_key[7:0]}; // O
						8'h4d: keyboard_key <= {keyboard_key[15:10], 1'b1, keyboard_key[8:0]}; // P
						8'h54: keyboard_key <= {keyboard_key[15:11], 1'b1, keyboard_key[9:0]}; // [
						8'h5b: keyboard_key <= {keyboard_key[15:12], 1'b1, keyboard_key[10:0]}; // ]
						// ARROW
						8'h75: keyboard_key <= {keyboard_key[15:13], 1'b1, keyboard_key[11:0]}; // Up
						8'h72: keyboard_key <= {keyboard_key[15:14], 1'b1, keyboard_key[12:0]}; //; // Down
						8'h6b: keyboard_key <= {keyboard_key[15:15], 1'b1, keyboard_key[13:0]}; //; // Left
						8'h74: keyboard_key <= {1'b1, keyboard_key[14:0]}; //; // Right
						default: keyboard_key <= keyboard_key;
					endcase
					end
				else				// Break (release)
					begin
					case (outCode) // BASED ON SET 2
						// ROW 1
						8'h15: keyboard_key <= {keyboard_key[15:1], 1'b0}; // Q
						8'h1d: keyboard_key <= {keyboard_key[15:2], 1'b0, keyboard_key[0:0]}; // W
						8'h24: keyboard_key <= {keyboard_key[15:3], 1'b0, keyboard_key[1:0]}; // E
						8'h2d: keyboard_key <= {keyboard_key[15:4], 1'b0, keyboard_key[2:0]}; // R
						8'h2c: keyboard_key <= {keyboard_key[15:5], 1'b0, keyboard_key[3:0]}; // T
						8'h35: keyboard_key <= {keyboard_key[15:6], 1'b0, keyboard_key[4:0]}; // Y
						8'h3c: keyboard_key <= {keyboard_key[15:7], 1'b0, keyboard_key[5:0]}; // U
						8'h43: keyboard_key <= {keyboard_key[15:8], 1'b0, keyboard_key[6:0]}; // I
						8'h44: keyboard_key <= {keyboard_key[15:9], 1'b0, keyboard_key[7:0]}; // O
						8'h4d: keyboard_key <= {keyboard_key[15:10], 1'b0, keyboard_key[8:0]}; // P
						8'h54: keyboard_key <= {keyboard_key[15:11], 1'b0, keyboard_key[9:0]}; // [
						8'h5b: keyboard_key <= {keyboard_key[15:12], 1'b0, keyboard_key[10:0]}; // ]
						// ARROW
						8'h75: keyboard_key <= {keyboard_key[15:13], 1'b0, keyboard_key[11:0]}; // Up
						8'h72: keyboard_key <= {keyboard_key[15:14], 1'b0, keyboard_key[12:0]}; //; // Down
						8'h6b: keyboard_key <= {keyboard_key[15:15], 1'b0, keyboard_key[13:0]}; //; // Left
						8'h74: keyboard_key <= {1'b0, keyboard_key[14:0]}; //; // Right
						default: keyboard_key <= keyboard_key;
					endcase
					end
				end // Make/Break
			else // Valid
				keyboard_key <= keyboard_key;
			end
		end
endmodule

/* Outputs a representation of a specified wavetype for the
 * 	designated frequency (based on note and octave)
 *
 * The wave is divided into PULSE_SIZE parts for each wave, which 
 * 	represents its accuracy.
 */
module wave_creator(clk, enable, reset_n, note, octave, wave, pulse);
	input clk, enable, reset_n;
	input [3:0] note; // Determines which note to play
	input [2:0] octave; // Determines which octave to play in
	input [1:0] wave; // Determines type of wave to output
	output reg [7:0] pulse; // Accuracy determined by size of output

	reg [21:0] count; // For counting down from >=50MHZ
	
	wire [21:0] hertz_count; // counter val. for specified note
	
	// Determines counter val. for specifed note
	soundMux freq(
		.MuxSelect(note),
		.Out(hertz_count)
		);
	
	// Pulse wave size manipulation
	always @(*)
		begin
		case (wave)
			2'b00: // Square
				begin
				if (count > (hertz_count / 2))
					pulse = 8'd0;
				else
					pulse = 8'd255;
				end
			2'b01: // Saw
				begin
				if (count > (hertz_count / 8) * 7)
					pulse = 8'd0;
				else if (count > (hertz_count / 4) * 3)
					pulse = 8'd37;
				else if (count > (hertz_count / 8) * 5)
					pulse = 8'd73;
				else if (count > (hertz_count / 2))
					pulse = 8'd109;
				else if (count > (hertz_count / 8) * 3)
					pulse = 8'd146;
				else if (count >  (hertz_count / 4))
					pulse = 8'd182;
				else if (count > (hertz_count / 8))
					pulse = 8'd218;
				else
					pulse = 8'd255;
				end
			2'b10: // Triangle
				begin
					// STUB
					pulse = 1'b0;
				end
			2'b11: // Sine
				begin
					// STUB
					pulse = 1'b0;
				end
		endcase
		end

	// Countdown
	always @(posedge clk) 
		begin
		if (reset_n == 1'b1) // On reset
			begin
				count <= hertz_count / (8'd2 << octave) - 1'b1;
			end
		else if (enable == 1'b1)  // Decrement count if enabled
			begin
			if (count == 0) // When count is the minimum value
				begin
				count <= hertz_count / (8'd2 << octave) - 1'b1;
				end
			else // When q is not the maximum value
				begin
				count <= count - 1'b1; // Decrement
				end
			end
		end
	
endmodule

/* Outputs a counter value for each note on a keyboard
 * 	at octave 0.
 *
 * Range: 12 values, excess are set to 0.
 *
 * Each value is c0 * 2^(MuxSelect / 12), but we can't use float values.
 */
module soundMux(
	input [3:0] MuxSelect,
	output reg [21:0] Out
	);

	always @(*)
	begin
		case (MuxSelect [3:0]) 
		4'b0000 : Out = 22'd3057753; // C0
		4'b0001 : Out = 22'd2886169; // C#0
		4'b0010 : Out = 22'd2724202; // D0
		4'b0011 : Out = 22'd2571355; // D#0
		4'b0100 : Out = 22'd2426949; // E0
		4'b0101 : Out = 22'd2290741; // F0
		4'b0110 : Out = 22'd2162162; // F#
		4'b0111 : Out = 22'd2040816; // D0
		4'b1000 : Out = 22'd1926263; // D#0
		4'b1001 : Out = 22'd1818182; // E0
		4'b1010 : Out = 22'd1716149; // F0
		4'b1011 : Out = 22'd1619800; // F#0
		default : Out = 22'd0; // TEST Value / error
		endcase
	end
endmodule