
module Audio_Demo (
	// Inputs
	CLOCK_50,
	KEY,

	AUD_ADCDAT,

	// Bidirectionals
	AUD_BCLK,
	AUD_ADCLRCK,
	AUD_DACLRCK,

	FPGA_I2C_SDAT,

	// Outputs
	AUD_XCK,
	AUD_DACDAT,

	FPGA_I2C_SCLK,
	//SW,
	enableSound,
	
	RAMAddress
);

/*****************************************************************************
 *                           Parameter Declarations                          *
 *****************************************************************************/


/*****************************************************************************
 *                             Port Declarations                             *
 *****************************************************************************/
// Inputs
input				CLOCK_50;
input		[3:0]	KEY;
//input		[3:0]	SW;
input		enableSound; 


input				AUD_ADCDAT;

// Bidirectionals
inout				AUD_BCLK;
inout				AUD_ADCLRCK;
inout				AUD_DACLRCK;

inout				FPGA_I2C_SDAT;

// Outputs
output				AUD_XCK;
output				AUD_DACDAT;
output				FPGA_I2C_SCLK;


/*****************************************************************************
 *                 Internal Wires and Registers Declarations                 *
 *****************************************************************************/
// Internal Wires
wire					audio_in_available;
wire		[31:0]	left_channel_audio_in;
wire		[31:0]	right_channel_audio_in;
wire					read_audio_in;

wire					audio_out_allowed;
wire		[31:0]	left_channel_audio_out;
wire		[31:0]	right_channel_audio_out;
wire					write_audio_out;

// Internal Registers

reg [13:0] count;	//triggers every 12500 clock cycles to trigger the read
output reg [15:0] RAMAddress; //stores the ram address needed to be read 
reg readRAM; //allows us to read the ram
wire [13:0]snd; //stores the sound - reg
wire [31:0] sound; //input to audio controller 

//counter wires

/*****************************************************************************
 *                             Sequential Logic                              *
 *****************************************************************************/
initial count = 14'd12500;

always @(posedge CLOCK_50)
begin

if (enableSound) begin
	if (count == 14'b0) begin
		readRAM <= 1'b1;
		RAMAddress <= RAMAddress + 1'b1;
		count <= 14'd12500;
	end
	else begin
		count <= count - 1;
		//readRAM <= 1'b0; 
		RAMAddress <= RAMAddress;
	end 
end 

end

assign sound = (count == 14'b0)? {snd, 18'b0} : sound;
/*****************************************************************************
 *                        	RAM					                           *
 *****************************************************************************/

//instantiating the ram: this is where the values are read from
songRAM SRAM(.address(RAMAddress), .clock(readRAM), .data(14'b0), .wren(1'b0), .q(snd));

/*****************************************************************************
 *                            Combinational Logic                            *
 *****************************************************************************/
 


assign read_audio_in					= 1'b0;
assign left_channel_audio_out		= sound;
assign right_channel_audio_out	= sound;
assign write_audio_out				= 1'b1;

/*****************************************************************************
 *                              Internal Modules                             *
 *****************************************************************************/

Audio_Controller Audio_Controller (
	// Inputs
	.CLOCK_50						(CLOCK_50),
	.reset						(~KEY[0]),

	.clear_audio_in_memory		(),
	.read_audio_in				(read_audio_in),
	
	.clear_audio_out_memory		(),
	.left_channel_audio_out		(),
	.right_channel_audio_out	(),
	.write_audio_out			(write_audio_out),

	.AUD_ADCDAT					(AUD_ADCDAT),

	// Bidirectionals
	.AUD_BCLK					(AUD_BCLK),
	.AUD_ADCLRCK				(AUD_ADCLRCK),
	.AUD_DACLRCK				(AUD_DACLRCK),


	// Outputs
	.audio_in_available			(audio_in_available),
	.left_channel_audio_in		(left_channel_audio_out	),
	.right_channel_audio_in		(right_channel_audio_out),

	.audio_out_allowed			(audio_out_allowed),

	.AUD_XCK						(AUD_XCK),
	.AUD_DACDAT					(AUD_DACDAT)

);

avconf #(.USE_MIC_INPUT(1)) avc (
	.FPGA_I2C_SCLK					(FPGA_I2C_SCLK),
	.FPGA_I2C_SDAT					(FPGA_I2C_SDAT),
	.CLOCK_50					(CLOCK_50),
	.reset						(~KEY[0])
);

endmodule

