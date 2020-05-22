`timescale 10ms/10ms

module running_game(

  CLOCK_50,   // On Board 50 MHz
  // Your inputs and outputs here
  SW,
  KEY,
  HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6,
  LEDR

  // The ports below are for the VGA output.  Do not change.
  ,VGA_CLK,      // VGA Clock
  VGA_HS,    // VGA H_SYNC
  VGA_VS,    // VGA V_SYNC
  VGA_BLANK_N,   // VGA BLAN
  VGA_SYNC_N,   // VGA SYNC
  VGA_R,      // VGA Red[9:0]
  VGA_G,     // VGA Green[9:0]
  VGA_B,      // VGA Blue[9:0]
  
  //the ports below are for the audio module. Do not change
  
  AUD_ADCDAT,
  AUD_BCLK,
  AUD_ADCLRCK,
  AUD_DACLRCK,
  FPGA_I2C_SDAT,
  AUD_XCK,
  AUD_DACDAT,
  FPGA_I2C_SCLK
 );

 input   CLOCK_50; // 50 MHz
 // Declare your inputs and outputs here
 input   [9:0] SW;
 input   [3:0] KEY;
 output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, HEX6;
 output [9:0] LEDR;

 // Do not change the following outputs
 output   VGA_CLK;       // VGA Clock
 output   VGA_HS;     // VGA H_SYNC
 output   VGA_VS;     // VGA V_SYNC
 output   VGA_BLANK_N;    // VGA BLANK
 output   VGA_SYNC_N;    // VGA SYNC
 output [9:0] VGA_R;       // VGA Red[9:0]
 output [9:0] VGA_G;      // VGA Green[9:0]
 output [9:0] VGA_B;       // VGA Blue[9:0]
 
 
 //Audio inouts 
  input AUD_ADCDAT;
// Bidirectionals
	inout				AUD_BCLK;
	inout				AUD_ADCLRCK;
	inout				AUD_DACLRCK;
	inout				FPGA_I2C_SDAT;
// Outputs
	output				AUD_XCK;
	output				AUD_DACDAT;
	output				FPGA_I2C_SCLK;


// Put your code here. Your code should produce signals x,y,colour and writeEn
// for the VGA controller, in addition to any other functionality your design may require.
// Create an Instance of a VGA controller - there can be only one!
// Define the number of colours as well as the initial background
// image file (.MIF) for the controller.

 vga_adapter VGA(
   .resetn(resetn),
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
  defparam VGA.BITS_PER_COLOUR_CHANNEL = 3;
  defparam VGA.BACKGROUND_IMAGE = "newstartscreen.mif";

  
  
  /** AUDIO PORT **/
  wire enableSound; 
  wire [7:0] result;
  wire [19:0] ramAddr;
  
  Audio_Demo AD1(
	// Inputs
	.CLOCK_50(CLOCK_50),
	.KEY(KEY[3:0]),
	.AUD_ADCDAT(AUD_ADCDAT),
	.AUD_BCLK(AUD_BCLK),
	.AUD_ADCLRCK(AUD_ADCLRCK),
	.AUD_DACLRCK(AUD_DACLRCK),
	.FPGA_I2C_SDAT(FPGA_I2C_SDAT),
	.AUD_XCK(AUD_XCK),
	.AUD_DACDAT(AUD_DACDAT),
	.FPGA_I2C_SCLK(FPGA_I2C_SCLK),
	.enableSound(enableSound),
	.ramOut(result)
);
  

 // Put your code here. Your code should produce signals x,y,colour and writeEn
 // for the VGA controller, in addition to any other functionality your design may require.
 wire resetn;
 assign resetn = KEY[0];

 // Create the colour, x, y and writeEn wires that are inputs to the controller.
 wire [8:0] colour;
 wire [8:0] colorErase;
 wire [7:0] x;
 wire [7:0] xerase;
 wire [7:0] y;
 wire [7:0] yerase;
 wire writeEn;
 wire ld_x;
 wire ld_y;
 wire ld_r;
 wire ld_player;
 wire erase;
 wire [4:0] count;
 wire [14:0] screenCount;
 wire [7:0] x_count;
 wire [7:0]y_count;

 //GAME VARIABLES

 wire jump, dash;
 assign jump = KEY[2];
 assign dash = KEY[1];
 wire noCollision;
 wire [16:0] score;
 wire [7:0]x_pl;
 wire [7:0]y_pl;

//

////instantiation of timing wires.

wire gameClock, move, makeObstacle;

 //Pseudo-Random Generator variables

wire enable;
wire [6:0]dataOut;
wire [1:0] modulus;
assign modulus = dataOut % 3;

random u1(.clk(CLOCK_50), .resetn(resetn), .enable(enable), .data(dataOut));

assign LEDR[2] = modulus[1];
assign LEDR[1] = modulus[0];

 control c(
 
  .clk(CLOCK_50),
  .game_start(~KEY[3]),
  .scoreON (SW[1]),
  .resetn(KEY[0]),
  .ld_x(ld_x),
  .ld_y(ld_y),
  .ld_r(ld_r),
  .ld_player (ld_player),
  .erase(erase),
  .plot(writeEn),
  .count(count),
  .screenCount(screenCount),
  .frame2 (gameClock),
  .makeObs (makeObstacle),
  .move (move),
  .colCheck (noCollision),
  .data(modulus),
  .done_erase(doneErase),
  
  .sound (enableSound)
 );


 datapath d(

  .x(x_count),
  .y(y_count),
  .ld_x(ld_x),
  .ld_y(ld_y),
  .ld_r(ld_r),
  .ld_player(ld_player),
  .erase(erase),
  .color(SW[9:7]),
  .resetn(KEY[0]),
  .clk(CLOCK_50),
  .count(count),
  .screenCount(screenCount),
  .x_to_plot(x),
  .y_to_plot(y),
  .color_to_plot(colour),
  .x_pl(x_pl),
  .y_pl(y_pl),
  .enableJump (jump),
  .move(move),
  .data(modulus),
  .color_erase(colorErase),
  .x_erase(xerase),
  .y_erase(yerase)


 );

 x_counter xc (

  .enable(move),
 .resetn(KEY[0]),
  .x(x_count),
  .dash(SW[2]),
  .assignEnable(enable)

 );

 y_counter yc(

  .enable(move),
  .resetn(KEY[0]),
  .y(y_count),
  .data (modulus)

 );
 


 //Draw the background
 wire doneErase;
 drawBackground b(.erase_on(erase), .clock(CLOCK_50), .x_final(xerase), .y_final(yerase), .count_done(doneErase), .color_out(colorErase));
 
 
  //game clock and object generation counter
 //change the clocking times 26'b00000011001011011100110101
 wire [25:0]pulseWidth; 
 gameLevel gL (.choice(SW[9:8], .pulseWidth(pulseWidth)); 

 rateDivideCount c3(.pulseWidth(pulseWidth), .pulse(gameClock), .clk(CLOCK_50)); //60 Hz THIS IS THE GAME CLOCK   00000000000000000000001100
 frameCounter F1 (.Clock (CLOCK_50), .En(gameClock), .Q(move));  //counts till 15 frames, which is when a pixel is moved.
 makeobstacle mo1 (.Clock (CLOCK_50), .En (move), .Q(makeObstacle)); //counts 20 frames, and then sends a signal that the obs should be made

 

 ScoreKeeper SK1 (.enable(move), .obst_x(x_count), .erase(erase), .pl_x(x_pl), .obst_y(y_count), .pl_y(y_pl), .collision(noCollision), .resetn(KEY[0]), .score(score), .clock(CLOCK_50), .game_start(~KEY[3]));
 playerReg PR1(.enable(move), .resetn(resetn), .y(y_pl), .x(x_pl), .clock(CLOCK_50), .jump(jump));

 assign LEDR[0] = noCollision;
 assign LEDR[4] = enableSound; 

 //Hex instantiations for score down here
// Display D1(.I(ramAddr[3:0]), .O(HEX0[6:0]));
// Display D2(.I(ramAddr[7:4]), .O(HEX1[6:0]));
// Display D3(.I(ramAddr[11:8]), .O(HEX2[6:0]));
// Display D4(.I(ramAddr[15:12]), .O(HEX2[6:0]));

//
// Display D5(.I(result[3:0]), .O(HEX3[6:0]));
// Display D6(.I(result[7:4]), .O(HEX5[6:0]));
 
endmodule

module gameLevel (input [1:0] choice, output [25:0] pulseWidth);

always @ (*) begin
case (choice) score
3'b00: pulseWidth = 26'b00000011001011011100110101;
3'b10: pulseWidth = 26'b00000001100101101110011011;
3'b11: pulseWidth = 26'b00000000110010110111001101;
default: pulseWidth = 26'b00000011001011011100110101;

endcase 
end 
endmodule


module control(

 input clk,
 input game_start,
 input scoreON,
 input resetn,
 output reg ld_x,
 output reg ld_y,
 output reg ld_r, //Load register - small object.
 output reg ld_player,
 output reg erase,
 output reg plot,
 output reg [5:0] count,
 output reg [14:0] screenCount,
 input frame2, //triggers every time a frame is changed
 input makeObs,
 input move,
 input colCheck,
 input [2:0] data, 
 input done_erase,
 
 output reg sound

 );

 

 reg [2:0] current_state, next_state;
 reg [5:0] countTill;

 

 //states

 localparam

 S_START  = 4'd0,
 S_LOAD 	  = 4'd1,
 S_PLOT_P  = 4'd2,
 S_HOLD_P  = 4'd3,
 S_PLOT_O  = 4'd4,
 S_HOLD_O  = 4'd5,
 S_Collide = 4'd6,
 S_ERASE   = 4'd7;


 //next state logic

 always@(*)begin

  case(current_state)

 S_START: next_state <= (game_start & colCheck!= 1'b1)? S_LOAD : S_START; //((go == 1'b0) & (colCheck == 1'b0) == 1'b1) 

 S_LOAD : next_state <= S_PLOT_P;

 S_PLOT_P : next_state <= (count == 6'b010000) ? S_HOLD_P : S_PLOT_P;  //count == 5'b10000

 S_HOLD_P : next_state <= S_PLOT_O; //(data == 2'b00)? S_PLOT_O : S_PLOT_O1; //O = WHITE, O1 = PINK

 S_PLOT_O: next_state <= (count == countTill)? S_HOLD_O : S_PLOT_O;   //count == 5'b10000

 S_HOLD_O : next_state <= ((move) == 1) ? S_Collide : S_HOLD_O; //holds the screen for 15 frames and then erases


 S_Collide: next_state <= ((colCheck) == 1'b1)? S_START : S_ERASE; //just changes state to start in case a collision is detected


 S_ERASE: next_state <= ((done_erase) == 1'b1)? S_LOAD : S_ERASE; //parses the entire screen and erases it 15'b100101100000000

 
 default: next_state <= S_LOAD;

  endcase

 end

 //enable signals

 always@(posedge clk)begin

  ld_x <= 1'b0;
  ld_y <= 1'b0;
  ld_r <= 1'b0;
  ld_player <= 1'b0;
  erase <= 1'b0;
  plot <= 1'b0;
  sound <= 1'b0;

  case(current_state)
  
  S_START : begin

  ld_x <= 1'b0;
  ld_y <= 1'b0;
  ld_r <= 1'b0;
  ld_player <= 1'b0;
  erase <= 1'b0;
  plot <= 1'b0;
  countTill <= 6'b0;
  
  end

 

  S_LOAD : begin

   count <= 6'b0;
   erase <= 1'b0;
   screenCount <= 15'b0;
	sound <= 1'b1; 

  //always want this to be the case, unless S_START.

     ld_x <= 1'b1;
     ld_y <= 1'b1;
     ld_r <= 1'b1;
     plot <= 1'b0;


   if (data == 2'b10)
		countTill <= 6'd31;

	else
		countTill <= 6'd15;

   end

   S_PLOT_P : begin
	
     ld_player <= 1'b1;
     plot <= 1'b1;
     count <= count + 1'b1; //this draws 16 pixels of the player
	  sound <= 1'b1; 

   end

   S_HOLD_P : begin

     count <= 6'b0;
	  plot <= 1'b0;
     ld_player <= 1'b0;
     ld_r <= 1'b1;
	  sound <= 1'b1; 

 end

//object 1

 S_PLOT_O : begin

     ld_r <= 1'b1;
     plot <= 1'b1;
     count <= count + 1'b1; //this draws 16 pixels of the obstacle
	  sound <= 1'b1; 
		
 end


 S_HOLD_O : begin

     ld_r <= 1'b0;
     plot <= 1'b0;
     count <= 6'b0;
	 sound <= 1'b1; 
		
 end

 

 S_ERASE : begin

     erase <= 1'b1;
     plot <= 1'b1;
	  	sound <= 1'b1;
   end
	


  endcase

 end

 //current state register

 always@(posedge clk) begin

  if(!resetn)

   current_state <= S_START;

  else

   current_state <= next_state;

 end

endmodule

module datapath(

 input [7:0] x,
 input [7:0] y,
 input ld_x,
 input ld_y,
 input ld_r,
 input ld_player,
 input erase,
 input [8:0] color,
 input resetn,
 input clk,
 input [5:0] count,
 input [14:0] screenCount,
 output reg [7:0] x_to_plot,
 output reg [7:0] y_to_plot,
 output reg [8:0] color_to_plot,
 input [7:0] x_pl,
 input [7:0] y_pl,
 input enableJump,
 input move,
 input [1:0] data,
 input [7:0] x_erase,
 input[7:0] y_erase,
 input [8:0] color_erase

 );

//obstacle coordinates and color

 reg [7:0] x_;
 reg [7:0] y_;
 reg [8:0] color_;

//loads obstacle registers x_, y_ with values in the specified direction

//values from the registers update only when move is triggered.

 always @(posedge clk) begin

  if (!resetn) begin
   x_ <= 8'b0;
   y_ <= 8'b0;
   color_ <= 9'b0;
  end

  else begin
	if (erase) begin
		color_ <= color_erase;
		x_ <= x_erase;
		y_ <= y_erase;

	end
	


	else begin
		if (data == 2'b00)  begin
			color_ <= 9'b111000000; //plot the normal box

		end

		else if(data == 2'b01) begin
			color_ <= 9'b000111000; //plot a box twice the height of the one we already hav

		end

		else if(data == 2'b10) begin //long boi
			color_ <= 3'b000000111; //plot a box twice the height of the one we already have

		end

		if(ld_x)
			x_ <= x;

		if(ld_y)
			y_ <= y;
	end

  end


 end


//sending to the VGA

 always @(posedge clk) begin

  if (!resetn) begin
     x_to_plot <= 8'b0;
     y_to_plot <= 8'b0;
     color_to_plot <= 9'b000;

  end

  else begin

     if (ld_r) begin  //plotting the obstacle box

         x_to_plot <= (x_+ count[1:0]); // (x_ + screenCount [7:0]) : (x_+ count[1:0])
         y_to_plot <= (y_ + count[4:2]); // (y_ + screenCount [13:8]) : (y_ + count[3:2])
         color_to_plot <= color_;

     end

     if (ld_player) begin //plotting the player box

         x_to_plot <=  (x_pl+ count[1:0]);     //(x_pl+ count[1:0]);
         y_to_plot <= (y_pl + count[3:2]);    //(y_pl + count[3:2]);
         color_to_plot <= 9'b111111000;

     end
	  
	  if(erase) begin
	  
			x_to_plot <= x_;
			y_to_plot <= y_;
			color_to_plot <= color_;
	  end
	  
 end
 end
endmodule


//X counter

module x_counter(

 input enable,

 input resetn,

 output reg [7:0] x,

 input dash,

 

 output reg assignEnable

 );

 reg right;

 always @ (posedge enable, negedge resetn)

 begin

  if (!resetn) begin

   x <= 8'd160; //8'd159;

 assignEnable <= 1'b0;

  end

  else begin

   if (dash) //if dashing conditions found

  x <= x - 8'd2;

 

 else

//under normal conditions

   x <= x - 8'b1;

 assignEnable <= 1'b0;

 

   if (x == 8'd0) begin//left edge

  x <= 8'd160;

  assignEnable <= 1'b1;

 end

 

  end

 end


endmodule

//Y Counter - just loading a change in values.

module y_counter(input enable, input resetn, output reg [6:0] y, input [1:0]data);

 always @ (posedge enable, negedge resetn)

 begin

  if (!resetn)

   y <= 8'd60; //7'd60;

 else begin

 

 //positioning the objects based on thier attributes

 

 if (data == 2'b0) //if it's a hole

  y <= 7'd64;

 else if (data == 2'b10)

  y <= 7'd56;

  else

  y<= 8'd60;  //7'd60 

 end

 end

endmodule

module playerReg(input enable, input resetn, output reg [7:0] y, output reg [7:0] x,

input clock, input jump);

 

 //giving values

 always @ (negedge resetn, posedge clock)

 begin

 if (!resetn) begin

   x <= 8'b0;

   y <= 8'd60;

 end

 else if (x === 8'bx || y === 7'bx)begin

   x <= 8'b0;

   y <= 8'd60;

 end

 else if (jump & enable) begin //start jumping

   x<= 8'b0;

   if (y > 8'd48) //moving upwards

   y <= y - 8'd16;

 end

 else if(enable & (y == 8'd48 | y< 8'd60 )) //moves downwards if there is no jump - resets after jump.

 y <= y + 8'd16;

 else begin //if there is no need to jump, just holds the previous values.

   x <= 8'b0;

   y <= y;

 end

 end

endmodule


/** ==  ==  ==  ==  ==  ==  == This module keeps check on the score ==  ==  ==  ==  ==  ==  ==  == **/

module ScoreKeeper (input enable, input [7:0] obst_x, pl_x, input [6:0] obst_y, pl_y, input erase, output reg collision, input resetn, output reg [16:0] score, input clock, input game_start);

always @ (posedge clock, negedge resetn)
begin

if (!resetn)
 begin
     collision <=1'b0;
     score <= 7'b0;
 end
 
else
 begin
 
	 if (game_start)
	 begin
		  collision <=1'b0;
		  score <= 7'b0;
	 end

     if (collision === 1'bx || score === 7'bx )
         begin
             collision <= 1'b0;
             score <= 7'b0;
         end

     else if (enable & !erase)
         begin
             if ((pl_x + 8'd16) == (obst_x+8'd16) & (obst_y == pl_y))
                 collision <= 1'b1;
				

             else if (collision != 1'b1)
                 begin
                     score <= score + 1'b1;

                     //scoring mechanism.
                 if (score[3:0] == 4'b1001) //score[3:0] == 9)
                     begin
                         score [7:4] <= score[7:4]+ 1'b1;
                         score [3:0] <= 4'b0;

                             if (score [7:4] == 4'b1001)
                                 begin
                                     score[11:8] <= score [11:8] + 1'b1;
                                     score [7:4] <= 4'b0;
												 
                                         if (score[11:8] == 4'b1001)
                                         begin
                                             score [16:12] <= score [16:12] + 1'b1;
                                             score [11:8] <= 4'b0;
                                         end
                                 end
                     end
                 end
         end
 end
end
endmodule

/** This module generates the signal for when an obstacle should be created **/

module makeobstacle (Clock, En, Q);

input Clock;
input En;

output reg Q;

reg [4:0]count;

always @ (posedge Clock)

 begin

 if (count === 5'bx) begin

  count <= 0;
  Q<=0;

 end

 else if (En) begin   //when 15 frames have elapsed, en = 1

  count <= count + 1;
  Q <= 0;

 end

 else if (count == 5'b10100) begin //resets the counter after 20 is reached

  count<= 5'b0;
  Q <= 1;

 end

 else begin

  count <= count;
  Q <= 0;

 end
 end

 

endmodule

/** This is generates a pulse when 15 frames have elapsed **/

module frameCounter (Clock, En, Q);

input Clock;
input En;
output reg Q;

reg [3:0]count;

always @ (posedge Clock)
 begin

 if (count === 4'bx) begin
  count <= 0;
  Q <=0;

 end

 else if (En) begin   //when gameClock == 1, en is high
  count <= count + 1;
  Q <= 0;

 end

 else if (count == 4'b1111) begin //resets the counter after 15 is reached

  count<= 4'b0;
  Q <= 1;

 end

 else begin

  count <= count;
  Q <= 0;

  end

 end
endmodule

module rateDivideCount (pulseWidth, pulse, clk);

 input [25:0] pulseWidth;
 reg [25:0] counter;
 input clk;
 output reg pulse;

 always @ (posedge clk)
 begin
 if (counter === 26'bx) begin
  counter <= 0;
 end

 else if (counter == pulseWidth)
 begin
  counter<= 0;
  pulse <= 1;
 end

 else
  begin
  counter <= counter + 1;
  pulse <= 0;
  end

 end

endmodule

/** ===================================================== MODULES: HEX DECODER ====================================================== **/

module Display(input[3:0] I, output [6:0] O);

 HEX H1( .c0(I[0]), .c1(I[1]), .c2(I[2]), .c3(I[3]), .l0(O[0]), .l1(O[1]), .l2(O[2]), .l3(O[3]), .l4(O[4]), .l5(O[5]), .l6(O[6]));

endmodule


module HEX(input c0, c1, c2, c3, output l0, l1, l2, l3, l4, l5, l6);

 assign l0 = (~c3&~c2&~c1&c0) | (~c3&c2&~c1&~c0) | (c3&~c2&c1&c0) | (c3&c2&~c1&c0);
 assign l1 = (~c3&c2&~c1&c0) | (~c3&c2&c1&~c0) | (c3&~c2&c1&c0) | (c3&c2&~c1&~c0) | (c3&c2&c1&~c0) | (c3&c2&c1&c0);
 assign l2 = (~c3&~c2&c1&~c0) | (c3&c2&~c1&~c0) | (c3&c2&c1&~c0) | (c3&c2&c1&c0);
 assign l3 = (~c3&~c2&~c1&c0) | (~c3&c2&~c1&~c0) | (~c3&c2&c1&c0) | (c3&~c2&~c1&c0) | (c3&~c2&c1&~c0) | (c3&c2&c1&c0);
 assign l4 = (~c3&~c2&~c1&c0) | (~c3&~c2&c1&c0) | (~c3&c2&~c1&~c0) | (~c3&c2&~c1&c0) | (~c3&c2&c1&c0) | (c3&~c2&~c1&c0);
 assign l5 = (~c3&~c2&~c1&c0) | (~c3&~c2&c1&~c0) | (~c3&~c2&c1&c0) | (~c3&c2&c1&c0) | (c3&c2&~c1&c0);
 assign l6 = (~c3&~c2&~c1&~c0) | (~c3&~c2&~c1&c0) | (~c3&c2&c1&c0) | (c3&c2&~c1&~c0);

endmodule

//============================LFSR Module=================================

module random(input clk, input resetn, input enable, output [6:0] data);

wire feedback;
reg [12:0] random;

initial begin
random <= 13'b1010101110100;
end

assign feedback = {random[10]~^random[7]^random[0]};

always@(posedge clk) begin
 if(!resetn)
  random <= 13'b1111111111111;

 else if(!enable)
  random <= random;

 else
  random <= {random[11], random[10], random[9], random[8], random[7], random[6], random[5], random[4], random[3], random[2], random[1], random[0], feedback};
end

assign data = {random[12],random[9],random[8], random[5], random[4], random[2], random[0]};

endmodule


//==================================RAM module to draw the background - reads from the ram file (background.v)=====================
module drawBackground(
input erase_on, clock,
output [7:0] x_final,
output [7:0] y_final,
output reg count_done,
output reg [8:0] color_out

);

wire [8:0] color;
reg [7:0] count_x;
reg [7:0] count_y;
reg [14:0] memory;

background gameBg(.address(memory), .clock(clock), .data(9'b0), .wren(1'b0), .q(color));

always @(posedge clock)
begin
	if(!erase_on)
	begin
		count_x <= 8'b0;
		count_y <= 8'b0;
		count_done = 1'b0;
		color_out <= 8'b0;
		memory <= 15'b0;
	end
	
	else if(erase_on && !count_done)
	begin
		color_out <= color;
		memory <= memory +1;
		if(count_x < 8'd159)
			count_x <= count_x +1;
			
		if(count_x == 8'd159 && count_y< 8'd119)
		begin
			count_x <= 8'b0;
			count_y <= count_y +1;
		end

		if(count_x == 8'd159 && count_y == 8'd119)
			count_done <= 1'b1;
	end
end

assign x_final = count_x;
assign y_final = count_y;

endmodule
