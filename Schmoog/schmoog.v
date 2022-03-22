// Macros
`define KEYB_KEYS 12 // Note: dependent on number of key inputs
`define PULSE_SIZE 8 // Note: wave dependency in wave_maker module


module schmoog(
	// Onboard Devices
	CLOCK_50, CLOCK2_50, KEY, 
	
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
	
	/* WIRING */
	
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
	wire [KEYB_KEYS - 1:0] keyboard_key;
	wire [3:0] keyboard_arrow;
	
	keyboard_press_driver keys(
	  .CLOCK_50(CLOCK_50), 
	  .valid(valid), 
	  .makeBreak(makeBreak),
	  .outCode(outCode), 
	  .PS2_DAT(PS2_DAT), // PS2 data line
	  .PS2_CLK(PS2_CLK), // PS2 clock line
	  .reset(reset_n)
	);
	
	key_interpreter(
		.reset(reset_n),
		.makeBreak(makeBreak)
		.valid(valid),
		.outCode(outCode),
		.keyboard_key(keyboard_key),
		.keyboard_arrow(keyboard_arrow)
	);

	/* AUDIO MODULE */

	wire read_ready, write_ready; // CODEC ready for read/write operation
	wire read, write; // send data from / to the CODEC (both channels)
	wire [23:0] readdata_left, readdata_right; // left and right channel data from the CODEC
	wire [23:0] writedata_left, writedata_right; // left and right channel data to the CODEC
	// AUD_* - should connect to top-level entity I/O of the same name.
	//         These signals go directly to the Audio CODEC
	// I2C_* - should connect to top-level entity I/O of the same name.
	//         These signals go directly to the Audio/Video Config module
	
	// USE THESE TO FEED INFORMATION TO / FROM THE CODEC
	assign writedata_left = 24'd0;
	assign writedata_right = 24'd0;
	assign read = 1'b0;
	assign write = 1'b0;
	
	// Schmoog-specific wirings (send to datapath?)
	wire [2:0] current_octave; // Current Octave selected
	wire sound_playing; // Is a key pressed for sound?
	wire [PULSE_SIZE - 1:0] wave_out1; // Create additional for multiple keys
	
	//TEMP DEFAULTS
	assign sound_playing = 1'b1;
	assign current_octave = 3'b100;
	
	clock_generator my_clock_gen(
		// inputs
		CLOCK2_50,
		reset_n,

		// outputs
		AUD_XCK
	);

	audio_and_video_config cfg(
		// Inputs
		CLOCK_50,
		reset_n,

		// Bidirectionals
		FPGA_I2C_SDAT,
		FPGA_I2C_SCLK
	);

	audio_codec codec(
		// Inputs
		CLOCK_50,
		reset_n,

		read,	write,
		writedata_left, writedata_right,

		AUD_ADCDAT,

		// Bidirectionals
		AUD_BCLK,
		AUD_ADCLRCK,
		AUD_DACLRCK,

		// Outputs
		read_ready, write_ready,
		readdata_left, readdata_right,
		AUD_DACDAT
	);
	
	// Produces one wave (make more for more keys)
	wave_creator wav1(
		.clk(CLOCK_50),
		.enable(sound_playing), // Replace with 'sound is playing'
		.reset_n(reset_n),
		.note(4'b0000), // Replace with 'note_wave[0]'
		.octave(current_octave), // TODO: Default to 4, consider making array?
		.pulse(wave_out1[0]) 
	);
	
	/* VGA MODULE */
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	wire [2:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;

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

/* 
 * 
 * 
 */
module datapath();
	
	
endmodule

/* Visual feedback for keys pressed
 * 
 * 
 */
module key_display(keyboard_key, clock, x_out, y_out, c_out);
	/* Wave: 64w, 48h
	0x, 0y - Wave 1
	64x, 0y - Wave 2
	0x, 48y - Wave 3
	64x, 48y - Wave 4
	
	Destination: 160x 120y
	
	Record / Play: 24w 8h
	2, 98 - Record ON
	+24x Record OFF
	Play ON
	Play OFF
	
	Destination:
	
	Keys: 6w 20h / Dest: 74y
	2 107 key1 ON (C @54, F @78)
	+ 6x key1 OFF
	key2 ON (D @62, G @86, A @94)
	key2 OFF 
	key3 ON (E @70, B @102)
	key3 OFF
	
	blackkeys: 4w 20h / Dest: 74y (x: 59, 67, 83, 91, 99)
	50 107 key4 ON 
	58 107 key4 OFF
	
	numbers: 4w 5h / Dest: 39x (y: 78, 85)
	67x 107y (add 4x to increment)
	
	*/
	input [KEYB_KEYS - 1:0] keyboard_key;
	input clock;
	output reg [7:0] x_out;
	output reg [6:0] y_out;
	output [2:0] c_out;
	
	reg [7:0] x_posit; // initial spritesheet position
	reg [6:0] y_posit; //
	reg [7:0] x_max;
	reg [6:0] y_max;
	reg [7:0] x_counter; // position from offset
	reg [7:0] y_counter; // 
	reg [4:0] count_active; // Designates which draw event has priority
	/*
	Free: 0
	Number: 1
	Keys: 2-13
	Play: 14
	Record: 15
	Wave: 16
	*/
	
	always@(posedge clock)
		begin
		if (count_active == 4'b0000) // Not currently assigned a draw task
			begin
			if (keyboard_key[0]) // C
				x_posit = 
				y_posit = 
				x_max = 8'd6;
				y_max = 7'd20;
				x_out = 8'd54;
				y_out = 7'd74;
				count_active == 4'b0010;
				
				// Use predefined address x, y, to load c value from RAM
				// output values, increment counter
			else
				//Set all values to zero
				
			if (keyboard_key[1]) // C#
				// STUB
			end
		end
	
	/* Graphic Memory */
	
	spritesheet sprite(
		.address({(y_posit + y_counter), (x_posit + x_counter)}), // 14 bit address (seven bits x, seven bits y)
		.clock(clock),
		.data(3'b000), // For writing to memory (unneeded)
		.wren(1'b0), // Read-Only
		.q(c_out)
	);

	
endmodule

/* Outputs a representation of a specified wavetype for the
 * 	designated frequency (based on note and octave)
 *
 * The wave is divided into PULSE_SIZE parts for each wave, which 
 * 	represents its accuracy.
 */
module wave_creator(clk, enable, reset_n, octave, wave, pulse);
	input clk, enable, reset_n;
	input [3:0] note; // Determines which note to play
	input [2:0] octave; // Determines which octave to play in
	input [1:0] wave; // Determines type of wave to output
	output reg [PULSE_SIZE - 1:0] pulse; // Accuracy determined by size of output

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
				else if (count > (hertz_count / 2)
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
					pulse = 1'b0
				end
			2'b11: // Sine
				begin
					// STUB
					pulse = 1'b0
				end
		endcase
		end

	// Countdown
	always @(posedge clk) 
		begin
		if (reset_n == 1'b1) // On reset
			begin
				count <= hertz_count / (8'd2 ** octave) - 1'b1;
			end
		else if (enable == 1'b1)  // Decrement count if enabled
			begin
			if (count == 0) // When count is the minimum value
				begin
				count <= hertz_count / (8'd2 ** octave) - 1'b1;
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

/* Interprets keyboard input from the PS2 driver and outputs
 *		keyboard_key signals that are used by the Schmoog
 *	
 */
module key_interpreter(reset, makeBreak, outCode, valid, keyboard_key, keyboard_arrow);
	input makeBreak, valid;
	input [7:0] outCode;
	output reg [KEYB_KEYS - 1:0] keyboard_key;
	output reg [3:0] keyboard_arrow;
	
	always @(*) 
		begin
		if (reset)
			begin
				keyboard_key = 10'd0;
				keyboard_arrow = 4'd0;
			end
		else
			begin
			if (valid)
				begin
				if (makeBreak) // Make (press)
					begin
					case (outCode) // BASED ON SET 2
						// ROW 1
						8'h15: keyboard_key[0] = 1; // Q
						8'h1d: keyboard_key[1] = 1; // W
						8'h24: keyboard_key[2] = 1; // E
						8'h2d: keyboard_key[3] = 1; // R
						8'h2c: keyboard_key[4] = 1; // T
						8'h35: keyboard_key[5] = 1; // Y
						8'h3c: keyboard_key[6] = 1; // U
						8'h43: keyboard_key[7] = 1; // I
						8'h44: keyboard_key[8] = 1; // O
						8'h4d: keyboard_key[9] = 1; // P
						8'h54: keyboard_key[10] = 1; // [
						8'h5b: keyboard_key[11] = 1; // ]
						/*// ROW 2
						8'h1C: keyboard_key[0] = 1; // A
						8'h1b: keyboard_key[1] = 1; // S
						8'h23: keyboard_key[2] = 1; // D
						8'h2b: keyboard_key[3] = 1; // F
						8'h34: keyboard_key[4] = 1; // G
						8'h33: keyboard_key[5] = 1; // H
						8'h3b: keyboard_key[6] = 1; // J
						8'h42: keyboard_key[7] = 1; // K
						8'h4b: keyboard_key[8] = 1; // L
						8'h4c: keyboard_key[9] = 1; // ; */
						// ARROW
						8'h75: keyboard_arrow[0] = 1; // Up
						8'h72: keyboard_arrow[1] = 1; // Down
						8'h6b: keyboard_arrow[2] = 1; // Left
						8'h74: keyboard_arrow[3] = 1; // Right
						default:
					endcase
					end
				else				// Break (release)
					begin
					case (outCode) // BASED ON SET 2
						// ROW 1
						8'h15: keyboard_key[0] = 0; // Q
						8'h1d: keyboard_key[1] = 0; // W
						8'h24: keyboard_key[2] = 0; // E
						8'h2d: keyboard_key[3] = 0; // R
						8'h2c: keyboard_key[4] = 0; // T
						8'h35: keyboard_key[5] = 0; // Y
						8'h3c: keyboard_key[6] = 0; // U
						8'h43: keyboard_key[7] = 0; // I
						8'h44: keyboard_key[8] = 0; // O
						8'h4d: keyboard_key[9] = 0; // P
						8'h54: keyboard_key[10] = 0; // [
						8'h5b: keyboard_key[11] = 0; // ]
						/*// ROW 2
						8'h1C: keyboard_key[0] = 0; // A
						8'h1b: keyboard_key[1] = 0; // S
						8'h23: keyboard_key[2] = 0; // D
						8'h2b: keyboard_key[3] = 0; // F
						8'h34: keyboard_key[4] = 0; // G
						8'h33: keyboard_key[5] = 0; // H
						8'h3b: keyboard_key[6] = 0; // J
						8'h42: keyboard_key[7] = 0; // K
						8'h4b: keyboard_key[8] = 0; // L
						8'h4c: keyboard_key[9] = 0; // ; */
						// ARROW
						8'h75: keyboard_arrow[0] = 0; // Up
						8'h72: keyboard_arrow[1] = 0; // Down
						8'h6b: keyboard_arrow[2] = 0; // Left
						8'h74: keyboard_arrow[3] = 0; // Right
						default:
					endcase
					end
				end
			end
		end
	

endmodule
