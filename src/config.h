// A simple TCO for XpressNet / XBus 
// based on the XpressNetMaster library (Copyright (c) 08/2016 - 2020 Philipp Gahtow)
// The code in this main program is published under the GPL
// All definitions should go here

// MCP23017 portexpander used or not - work in progress
// choice not fully implemented in code
#define MCP				

// Adresses are shifted by 1 for XPressNet at z21PG (6 sent from this code here is 7 in real life).
// Loconet at z21PG no shift 
// By defining SHIFTADDR all later definitions can be real life addresses
#define SHIFTADDR	0

// If a MCP23017 portexpander is used, only the follwowing Atmega pins are available as I/Os. 
// Pin 8 is used for the RS485 communication
// 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17
// If no MPC is present, the I2C pins can additionally be used:
// 18, 19

// Atmega inputs (5 arrays in total, all must have same length!)
// Define which pins of the Atmega are used as inputs (contact to GND for active)
// D2-D8 (2-8); D10-D13(10-13); A0-A5 (14-19) (A4, A5 not if MCP)
#define INATMEL			{14, 16}	//all inputs
// Define for each of the pins an adress and a position (left or right, "red" or "green" etc.)
#define INATMELADD		{3,  3}	//Addr 1 for inputs
#define INATMELPOS		{1,  0}	//Pos each Addr 1
// Define for each of the pins a second adress and a position (left or right, "red" or "green" etc.)
// With this setting, one switch can set two positions on different turnouts
#define INATMELADD2	    {4,  4}	//Addr 2 for inputs, "0" if not in use
#define INATMELPOS2	    {1,  0}	//Pos each Addr 2, "0" if not in use

// Atmega outputs (3 arrays in total, all must have same length!)
// Define which pins of the Atmega are used as outputs (the logic in the code sets HIGH as active
// i.e. LEDs are conntected Output > Resistor > LED > GND)
// D2-D8 (2-8); D10-D13(10-13); A0-A5 (14-19) (A4, A5 not if MCP)
#define OUTATMEL		{2, 3, 15, 17}			//all outputs
// Define for each of the pins an adress and a position (left or right, "red" or "green" etc.)
#define OUTATMELADD	    {2, 2,  3,  3} 			//Addr for outputs
#define OUTATMELPOS	    {1, 0,  1,  0}	    	//Pos each Addr

// MCP23017 outputs (code supports only outputs on MCP23017) (3 arrays in total, all must have same length!)
// Define which pins of the MCP23017 are used as outputs (the logic in the code sets HIGH as active
// i.e. LEDs are conntected Output > Resistor > LED > GND)
//0-7 port A; 8-15 port B
#define OUTMCP			{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}	//all outputs 
// Define for each of the pins an adress and a position (left or right, "red" or "green" etc.)
#define OUTMCPADD		{6, 6, 5, 5, 7, 7, 8, 8, 9, 9,  4,  4,  3,  3,  2,  2}	//Addr for outputs
#define OUTMCPPOS		{0, 1, 1, 0, 1, 0, 1, 0, 1, 0,  0,  1,  0,  1,  0,  1}	//Pos each Addr