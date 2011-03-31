/** Arduino library to interact with SonMicro SM125 RFID Reader
 *
 * SM125-IC supports Manchester modulation. The supported tags are Atmel / Temic T55xx(Q5, T5551, T5552, 
 * T5554,T5555,T5557) and the industries’ most popular 125 KHz read-only EM4102 tag from the EM Micro.
 *
 * This library is free software; you can redistribute it and/or modify it under the terms of the GNU 
 * Lesser General Public License as published by the Free Software Foundation; either version 2.1 of 
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without 
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along with this library; 
 * if not, write to the Free Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  
 * 02110-1301  USA
 *
 * @copyright 	(C) 2011 HarvestGardener, All rights reserved.
 * @license 	GNU Lesser General Public License v2.1 or later
 * @version 	0.4b
 * @date		30 March 2011
 * @contact		hrvstgardener [at] gmail [dot] com
 * @link 		https://github.com/HarvestGardener/SM125-125khz-RFID-Arduino-Library
 * @ref			SM125 Datasheet        	- http://www.sonmicro.com/en/downloads/125/ds_SM125_V30.pdf 
 *				SM125 Manual	       	- http://www.sonmicro.com/en/downloads/125/um_SM125.pdf
 *				EM4102 Datasheet		- http://www.sonmicro.com/en/downloads/125/H4102_C.pdf
 *				T5555/Q5 Datasheet		- http://www.sonmicro.com/en/downloads/125/Q5B.pdf
 *      		T55x7 Datasheet  		- http://www.sonmicro.com/en/downloads/125/t5557.pdf
 **/
//Header File Includes
#include "WProgram.h"
#include "SM125.h"

//SM125 Functionality Includes
#include "..\NewSoftSerial\NewSoftSerial.h"

//**************************************Basic Command Definitions************************************//
#define CMD_READ				0x10 				//Starts the “Read Operation” and looks for tag continuously
#define CMD_READ_WITH_PASS 		0x13				//Starts “Read Operation” for password protected Tag
#define CMD_WRITE 				0x20				//Write data to the Tag
#define CMD_WRITE_WITH_PASS 	0x23				//Write data to the password protected Tag
#define CMD_WRITE_OUTPUT_PINS 	0x62				//Sets Output1 state as Logic 1 or 0
#define CMD_READ_INPUT_PIN 		0x63				//Read INPUT1 state as logic 1 or 0
#define CMD_SET_AUTO_MODE 		0x87				//Determine Auto Mode behavior

//*********************************Configuration Command Definitions**********************************//
#define CMD_SET_PROG_PARAM 		0x40				//Sets parameters for Tag programming
#define CMD_SET_RADF 			0x41				//Sets Antenna Drive frequency
#define CMD_SET_BAUD 			0x52				//Sets UART Baud Rate
#define CMD_SET_BYTE_TRACK 		0x55				//Sets Byte(s) to be tracked while reading Tag
#define CMD_SET_OUTPUT_TYPE 	0x88				//Determines Output Types
#define CMD_SET_I2C 			0x90				//Sets I2C slave address or enable/disable I2C

//****************************************Read Mode Definitions**************************************//
#define READ_MODE_BT_M64 		0x01				//Byte Track Mode – Manchester RF/64
#define READ_MODE_RAW_EM4102 	0x02				//EM4102 Mode not-decoded(raw) – Manchester RF/64
#define READ_MODE_EM4102		0x03				//EM4102 Mode – Parity decoded – Manchester RF/64
#define READ_MODE_BT_M32		0x04				//Byte Track Mode – Manchester RF/32

//**********************************************Static Bytes*****************************************//
#define HEADER                  0xFF               	//Fixed header byte
#define RESERVED                0x01               	//Fixed byte reserved for future use
#define AUTO_READ_DISABLE		0x00				//Disables Auto Read Mode
#define	AUTO_READ_ENABLE		0x01				//Enables Auto Read Mode
#define AUTO_PASSWORD_DISABLE	0x00			    //Password is not sent during auto read operations
#define AUTO_PASSWORD_ENABLE	0x01				//Password is sent approximately each second during auto read

//**************************************Static Command Definitions***********************************//
//Command Format: [Header: 0xFF] [Reserved: 0x01] [Length in bytes of cmd and data] [Command] [Data] [Checksum: sum(all bytes except header)]
byte CMD_STOP_READ[5]		        =	{0xFF, 0x01, 0x01, 0x12, 0x14};	//Stops Reading
byte CMD_RESET[5]		       		=	{0xFF, 0x01, 0x01, 0x51, 0x53};	//Resets SM125
byte CMD_SLEEP[5]		        	=	{0xFF, 0x01, 0x01, 0x60, 0x62};	//Puts SM125 in Sleep mode to save power
byte CMD_FIRMWARE[5]		        =	{0xFF, 0x01, 0x01, 0x50, 0x52};	//Gets firmware version
byte CMD_READ_CONFIG[5]		        =	{0xFF, 0x01, 0x01, 0x70, 0x72};	//Get configuration parameters of SM125
byte CMD_SET_Q5_PROG_PARAM[10]	    =	{0xFF, 0x01, 0x06, 0x40, 0x64, 0xC8, 0xAA, 0x32, 0x60, 0x00}; //Set programming parameters for Q5 cards
byte CMD_SET_T5557_PROG_PARAM[10]	=	{0xFF, 0x01, 0x06, 0x40, 0x64, 0xC8, 0xB9, 0x3C, 0x60, 0x00}; //Set Programming parameters for T5557 cards

/** SM125 Class Constructor 
 *
 * Create a new soft serial port to handle connections to the RFID IC
 *
 * @param	int		rxPin	Arduino pin connected to SM125 TX
 *
 * @param	int		txPin	Arduino pin connected to SM125 RX
 *
 * @return  None
 *
 * @status	Untested
 **/
SM125::SM125(int rxPin, int txPin) : serialRFID(rxPin, txPin)
{
	// set the data rate for the NewSoftSerial port
	serialRFID.begin(19200);
  
	//DEBUG: Start serial @ 9600
	Serial.begin(9600);
}

//********************************************Basic Commands*****************************************//
/** Execute a static command 
 *
 * @param	byte*	cmdToExecute	The static definition of the command to be executed
 *
 * @param	int		len				Length of the command to be executed
 *
 * @return      0 if successfull, 1 if not
 *
 * @status	Tested, Works.
 **/
int SM125::cmdStaticExec(byte* cmdToExecute, int len) {
	// send the request bytes
	for (int m=0; m<len; m++) {
		serialRFID.write(cmdToExecute[m]);
	}
  
	//flush the RFID serial port
	serialRFID.flush();
	return 0;
}

/** Start standard read operation
 *
 * SM125 will send tag data as soon as the tag enters into field.
 *
 * @param	byte	hexMode		1 Byte – Mode or Type of Read - Mode will be set to 0x03 automatically 
 *					in the SM125 device if it is different than the acceptable values.
 *
 * @param	byte	hexBlock	1 Byte – Number of total blocks to be read - For EM4102 Mode block number 
 *					will be adjusted to 0x02 automatically in the SM125 device.
 *
 *					For Byte Track Mode, this should be between 1 and 7. Values greater than 7 
 *					blocks will be adjusted to 7 automatically in the SM125 device.
 *
 * @return	0 if successfull, 1 if not
 *
 * @status	Tested, Works
 **/
int SM125::cmdRead (byte hexMode, byte hexBlock) {
	
	//Calculate checksum of all bytes except header
	byte hexLength = 0x03;
	byte hexChksum = RESERVED + hexLength + CMD_READ + hexMode + hexBlock;
	
	//Build and execute command
	byte cmdToExecute[7] = {HEADER, RESERVED, hexLength, CMD_READ, hexMode, hexBlock, hexChksum};
        
        // send the request bytes
        for (int m=0; m<7; m++) {
			serialRFID.write(cmdToExecute[m]);
        }
        //flush buffer *critical*
        serialRFID.flush();
        
        delay(500);
        
	//verify successful completion of command
	if (verifySuccess() > 0) return 1; 
	
	return 0;
}

/** Start read operation for a password protected tag
 *
 * The password will be sent approximately each second to the tag automatically. A tag that is not password 
 * protected will also be read. Response indicating successful operation is expected immediately after 
 * command is sent.
 *
 * @param	byte	hexMode		1 Byte – Mode or Type of Read - Mode will be set to 0x03 automatically 
 *					in the SM125 device if it is different than the acceptable values.
 *
 * @param	byte	hexBlock	1 Byte – Number of total blocks to be read - For EM4102 Mode block number 
 *					will be adjusted to 0x02 automatically in the SM125 device.
 *
 *					For Byte Track Mode, this should be between 1 and 7. Values greater than 7 
 *					blocks will be adjusted to 7 automatically in the SM125 device.
 *
 * @param	byte	hexPByte1	1st password byte		Note that password is the Block7 of the tag data.
 *
 * @param	byte	hexPByte2	2nd password byte
 *
 * @param	byte	hexPByte3	3rd password byte
 *
 * @param	byte	hexPByte4	4th password byte
 *
 * @return  0 if successfull, 1 if not
 *
 * @status	Untested
 **/
int SM125::cmdReadwPass (byte hexMode, byte hexBlock, byte hexPByte1, byte hexPByte2, byte hexPByte3, byte hexPByte4) {
	
	//Calculate checksum of all bytes except header
	byte hexLength = 0x07;
	byte hexChksum = RESERVED + hexLength + CMD_READ_WITH_PASS + hexMode + hexBlock + hexPByte1 + hexPByte2 + hexPByte3 + hexPByte4;
	
	//Build and execute command
	byte cmdToExecute[11] = {HEADER, RESERVED, hexLength, CMD_READ_WITH_PASS, hexMode, hexBlock, hexPByte1, hexPByte2, hexPByte3, hexPByte4, hexChksum};
        
        // send the request bytes
        for (int m=0; m<11; m++) {
			serialRFID.write(cmdToExecute[m]);
        }
        //flush buffer *critical*
        serialRFID.flush();
        
        delay(500);
        
	//verify successful completion of command
	if (verifySuccess() > 0) return 1; 
	
	return 0;
}

/** Start standard write operation in SM125
 * 
 * Response may arrive with around 500ms delay and does not indicate a successful write operation, only that 
 * the mode was correctly set and data transmitted. Verify using a follow-on read operation.
 *
 * @param	byte	hexBlock        1 Byte – Block number to be programmed 
 *                                  ********************************************************************
 *									**Please make sure you go over Tag Programming section in         **
 *                                  **SM125 User Manual document before you program any block of      **
 *                                  **the tag. Incorrect writing may lock the tag and make it useless.**
 *                                  ********************************************************************
 *                                  Block Numbers:
 *                                  0x00       Configuration Block. **WARNING: Invalid data here may brick card!**  
 *                                  0x01       Block 1. Used for Byte Track  
 *                                  0x02       Block 2. Free Block   
 *                                  0x03       Block 3. Free Block   
 *                                  0x04       Block 4. Free Block   
 *                                  0x05       Block 5. Free Block   
 *                                  0x06       Block 6. Free Block   
 *                                  0x07       Block 7. Password Block or Free Block
 *
 * @param	byte	hexByte1        1st byte in hex to be written to card at specificed block
 *
 * @param	byte	hexByte2        2nd byte in hex to be written to card at specificed block
 * 
 * @param	byte	hexByte3        3rd byte in hex to be written to card at specificed block
 * 
 * @param	byte	hexByte4        4th byte in hex to be written to card at specificed block
 *
 * @return  0 if successfull, 1 if not
 *
 * @status	Tested, works except for return value
 **/
int SM125::cmdWrite (byte hexBlock, byte hexByte1, byte hexByte2, byte hexByte3, byte hexByte4) {
  //Calculate checksum of all bytes except header
  byte hexLength = 0x06;
  byte hexChksum = RESERVED + hexLength + CMD_WRITE + hexBlock + hexByte1 + hexByte2 + hexByte3 + hexByte4;
  
  //Build and execute command
  byte cmdToExecute[10] = {HEADER, RESERVED, hexLength, CMD_WRITE, hexBlock, hexByte1, hexByte2, hexByte3, hexByte4, hexChksum};
        
		// send the request bytes
        for (int m=0; m<10; m++) {
			serialRFID.write(cmdToExecute[m]);
        }
        //flush buffer *critical*
        serialRFID.flush();
        
        delay(500);
        
	//verify successful completion of command
	if (verifySuccess() > 0) return 1; 
	
	return 0;
}

/** Write data to a tag with a password
 *
 * First password is sent then programming is performed. Response may arrive with around 500ms delay and does 
 * not indicate a successful write operation, only that the mode was correctly set and data transmitted. 
 * Verify using a follow-on read operation.
 *
 * @param	byte	hexBlock        1 Byte – Block number to be programmed 
 *                                 	********************************************************************
 *									**Please make sure you go over Tag Programming section in         **
 *                                  **SM125 User Manual document before you program any block of      **
 *                                  **the tag. Incorrect writing may lock the tag and make it useless.**
 *                                  ********************************************************************
 *                                  Block Numbers:
 *                                  0x00       Configuration Block. **WARNING: Invalid data here may brick card!**  
 *                                  0x01       Block 1. Used for Byte Track  
 *                                  0x02       Block 2. Free Block   
 *                                  0x03       Block 3. Free Block   
 *                                  0x04       Block 4. Free Block   
 *                                  0x05       Block 5. Free Block   
 *                                  0x06       Block 6. Free Block   
 *                                  0x07       Block 7. Password Block or Free Block
 *
 * @param	byte	hexByte1        1st byte in hex to be written to card at specificed block
 *
 * @param	byte	hexByte2        2nd byte in hex to be written to card at specificed block
 * 
 * @param	byte	hexByte3        3rd byte in hex to be written to card at specificed block
 * 
 * @param	byte	hexByte4        4th byte in hex to be written to card at specificed block
 *
 * @param	byte	hexPByte1		1st password byte		Note that password is the Block7 of the tag data.
 *
 * @param	byte	hexPByte2		2nd password byte
 *
 * @param	byte	hexPByte3		3rd password byte
 *
 * @param	byte	hexPByte4		4th password byte
 *
 * @return  0 if successfull, 1 if not
 *
 * @status	Untested
 **/
 int SM125::cmdWritewPass (byte hexMode, byte hexBlock, byte hexByte1, byte hexByte2, byte hexByte3, byte hexByte4, byte hexPByte1, byte hexPByte2, byte hexPByte3, byte hexPByte4) {
	
	//Calculate checksum of all bytes except header
	byte hexLength = 0x0A;
	byte hexChksum = RESERVED + hexLength + CMD_WRITE_WITH_PASS + hexMode + hexBlock + hexByte1 + hexByte2 + hexByte3 + hexByte4 + hexPByte1 + hexPByte2 + hexPByte3 + hexPByte4;
	
	//Build and execute command
	byte cmdToExecute[15] = {HEADER, RESERVED, hexLength, CMD_WRITE_WITH_PASS, hexMode, hexBlock, hexByte1, hexByte2, hexByte3, hexByte4, hexPByte1, hexPByte2, hexPByte3, hexPByte4, hexChksum};
        
        // send the request bytes
        for (int m=0; m<11; m++) {
          serialRFID.write(cmdToExecute[m]);
        }
        //flush buffer *critical*
        serialRFID.flush();
        
        delay(500);
        
	//verify successful completion of command
	if (verifySuccess() > 0) return 1; 
	
	return 0;
}

/** Enable or disable Auto Read Mode
 *
 * The parameters define the Mode(Read/Modulation type) to be used, number of blocks to be read and if it is 
 * necessary or not to send a password. Once Auto Read mode command is executed, the parameters are stored in 
 * non-volatile memory of the SM125, negating the need for an external microcontroller to initiate read operations. 
 *
 * @param	byte	hexAutoMode	1 Byte - Auto Mode Enable / Disable
 *
 * @param	byte	hexMode		1 Byte – Mode or Type of Read - Mode will be set to 0x03 automatically 
 *					in the SM125 device if it is different than the acceptable values.
 *
 * @param	byte	hexBlock	1 Byte – Number of total blocks to be read - For EM4102 Mode block number 
 *					will be adjusted to 0x02 automatically in the SM125 device.
 *
 *					For Byte Track Mode, this should be between 1 and 7. Values greater than 7 
 *					blocks will be adjusted to 7 automatically in the SM125 device.
 *
 * @param	byte	hexPassMode	Auto Mode Enable / Disable
 *
 * @param	byte	hexPByte1	1st password byte		Note that password is the Block7 of the tag data.
 *
 * @param	byte	hexPByte2	2nd password byte		If the password is not going to be used, random numbers 
 *														can be given.
 * @param	byte	hexPByte3	3rd password byte
 *
 * @param	byte	hexPByte4	4th password byte
 *
 * @return  0 if successfull, 1 if not
 *
 * @status	Untested
 **/
 int SM125::cmdSetAutoMode (byte hexAutoMode, byte hexMode, byte hexBlock, byte hexPassMode, byte hexPByte1, byte hexPByte2, byte hexPByte3, byte hexPByte4) {
	
	//Calculate checksum of all bytes except header
	byte hexLength = 0x09;
	byte hexChksum = RESERVED + hexLength + CMD_SET_AUTO_MODE + hexAutoMode + hexMode + hexBlock + hexPassMode + hexPByte1 + hexPByte2 + hexPByte3 + hexPByte4;
 
 	//Build and execute command
	byte cmdToExecute[13] = {HEADER, RESERVED, hexLength, CMD_SET_AUTO_MODE, hexAutoMode, hexMode, hexBlock, hexPassMode, hexPByte1, hexPByte2, hexPByte3, hexPByte4, hexChksum};
        
        // send the request bytes
        for (int m=0; m<13; m++) {
			serialRFID.write(cmdToExecute[m]);
        }
        //flush buffer *critical*
        serialRFID.flush();
        
        delay(500);
        
	//verify successful completion of command
	if (verifySuccess() > 0) return 1; 
	
	return 0;
}

//****************************************Configuration Commands*************************************//
/** Set parameters for Tag programming
 *
 * The use of this command is undocumented in SM125 literature. See command aliases for predefined usages for Q5 
 * and T5557 cards.
 *
 * @param	byte	hexWG		Unknown
 *
 * @param	byte	hexSG      	Unknown
 *
 * @param	byte	hexONE      Unknown
 * 
 * @param	byte	hexZERO     Unknown  
 * 
 * @param	byte	hexPADF     Unknown
 *
 * @return  0 if successfull, 1 if not
 *
 * @status	Untested
 **/
int SM125::cmdSetProgParam (byte hexWG, byte hexSG, byte hexONE, byte hexZERO, byte hexPADF) {
  //Calculate checksum of all bytes except header
  byte hexLength = 0x06;
  byte hexChksum = RESERVED + hexLength + CMD_SET_PROG_PARAM + hexWG + hexSG + hexONE + hexZERO + hexPADF;
  
  //Build and execute command
  byte cmdToExecute[10] = {HEADER, RESERVED, hexLength, CMD_SET_PROG_PARAM, hexWG, hexSG, hexONE, hexZERO, hexPADF, hexChksum};
        
        // send the request bytes
        for (int m=0; m<10; m++) {
          serialRFID.write(cmdToExecute[m]);
        }
        //flush buffer *critical*
        serialRFID.flush();
        
        delay(500);
        
	//verify successful completion of command
	if (verifySuccess() > 0) return 1; 
	
	return 0;
}

/** Return the version of the RFID IC firmware 
 *
 * @return  Prints version information to Serial
 *
 * @status  Tested, works
 **/
void SM125::cmdReadFirmwareVersion(){
  //send the read command via static command execution
  cmdStaticExec(CMD_FIRMWARE, 5);
  delay(500);
  
  //read and parse the response
  if(serialRFID.available()) {
    //discard header
    serialRFID.read();
    serialRFID.read();
    //pull off the length of the version response
    int length = (int)serialRFID.read();
    //discard command byte
    serialRFID.read();
    Serial.print("Firmware ");
    //get firmware bytes
    for(int i = 0; i<length - 1; i++){              //response is length - 1 because the command is included
      Serial.print(serialRFID.read(), BYTE);
    }
    Serial.print('\n');
    //discard checksum
    serialRFID.read();
  }
}

//*******************************************Command Aliases*****************************************//
/** Stop Read Operation
 *
 * @return      0 if successfull, 1 if not
 *
 * @status	Tested, works
 **/
int SM125::cmdStopRead() {
	//execute command
	cmdStaticExec(CMD_STOP_READ, 5);
	
	//verify successful completion of command
	if (verifySuccess() > 0) return 1; 
	
	return 0;
}

/** Reset SM125. 
 *
 * Note: If Auto Read Mode is enabled, automated read operation will resume after reset.
 *
 * @return      0 if successfull, 1 if not
 *
 * @status	Untested
 **/
int SM125::cmdReset() {
	//execute command
	cmdStaticExec(CMD_RESET, 5);
	
	//verify successful completion of command
	if (verifySuccess() > 0) return 1; 
	
	return 0;
}

/** Put SM125 into sleep mode
 *
 * Once module enters into sleep mode, only external hardware reset or Power-On-Reset will wake up the SM125 device.
 *
 * @return      0 if successfull, 1 if not
 *
 * @status	Untested
 **/
int SM125::cmdSleep() {
	//execute command
	cmdStaticExec(CMD_SLEEP, 5);
	
	//verify successful completion of command
	if (verifySuccess() > 0) return 1; 
	
	return 0;
}

/** Set SM125 to Q5 programming parameters
 *
 * @return      0 if successfull, 1 if not
 *
 * @status	Untested
 **/
int SM125::cmdSetQ5ProgParam() {
	//execute command
	cmdStaticExec(CMD_SET_Q5_PROG_PARAM, 10);
	
	//verify successful completion of command
	if (verifySuccess() > 0) return 1; 
	
	return 0;
}

/** Set SM125 to T5557 programming parameters
 *
 * @return      0 if successfull, 1 if not
 *
 * @status	Tested, works
 **/
int SM125::cmdSetT5557ProgParam() {
	//execute command
	cmdStaticExec(CMD_SET_T5557_PROG_PARAM, 10);
	
	//verify successful completion of command
	if (verifySuccess() > 0) return 1; 
	
	return 0;
}

/** Reset Q5 card to default settings
 * 
 * Config and Byte Track blocks are reset to default values, blocks 2-7 are zeroed out.
 *
 * WARNING: It is very important not to attempt to reset a T55xx card with this command. This will likely
 * result in bricking the card as they use different block 0 config values. 
 *
 * @return  0 if successfull, 1 if not
 *
 * @status	Untested *CRITICAL* need to verify the config block and byte track settings before testing
 **/
int SM125::cmdResetQ5Tag() {
	//stop any current read operations
	cmdStopRead();
	
	//Set Q5 Programming Parameters
	cmdSetQ5ProgParam();
	
	//Reset Config Block
	int resetConfig = cmdWrite(0x00, 0x60, 0x01, 0xF0, 0x0E);
	
	//Reset Byte Track config
	int resetByteTrack = cmdWrite(0x01, 0x52, 0x58, 0x8B, 0x45);
	
	//Reset data blocks
	int resetBlock2 = cmdWrite(0x02, 0x00, 0x00, 0x00, 0x00);
	int resetBlock3 = cmdWrite(0x03, 0x00, 0x00, 0x00, 0x00);
	int resetBlock4 = cmdWrite(0x04, 0x00, 0x00, 0x00, 0x00);
	int resetBlock5 = cmdWrite(0x05, 0x00, 0x00, 0x00, 0x00);
	int resetBlock6 = cmdWrite(0x06, 0x00, 0x00, 0x00, 0x00);
	int resetBlock7 = cmdWrite(0x07, 0x00, 0x00, 0x00, 0x00);
	
	//verify successful operation of all reset commands
	int returnVal = resetConfig + resetByteTrack + resetBlock2 + resetBlock3 + resetBlock4 + resetBlock5 + resetBlock6 + resetBlock7;
	
	if (returnVal > 0) {
		return 1; //one or more resets failed
	}
	else {return 0;}
}

/** Reset T5557 card to default settings
 * 
 * Config and Byte Track blocks are reset to default values, blocks 2-7 are zeroed out.
 *
 * WARNING: It is very important not to attempt to reset a Q5 card with this command. This will likely
 * result in bricking the card as they use different block 0 config values. 
 *
 * @return  0 if successfull, 1 if not
 *
 * @status	Tested, works.
 **/
int SM125::cmdResetT5557Tag() {
	//stop any current read operations
	cmdStopRead();
	
	//Set T5557 Programming Parameters
	cmdSetT5557ProgParam();
	
	//Reset Config Block SM125 settings: 0x00, 0x14, 0x80, 0xE0
	int resetConfig = cmdWrite(0x00, 0x00, 0x14, 0x80, 0xE0); //note this is different than my recorded value of 0x00, 0x08, 0x80, 0xE8
        delay(500);
	
	//Reset Byte Track config
	int resetByteTrack = cmdWrite(0x01, 0x52, 0x58, 0x8B, 0x45);
        delay(500);
 
	//Reset data blocks
	int resetBlock2 = cmdWrite(0x02, 0x00, 0x00, 0x00, 0x00);
        delay(500);
	int resetBlock3 = cmdWrite(0x03, 0x00, 0x00, 0x00, 0x00);
        delay(500);
	int resetBlock4 = cmdWrite(0x04, 0x00, 0x00, 0x00, 0x00);
        delay(500);
	int resetBlock5 = cmdWrite(0x05, 0x00, 0x00, 0x00, 0x00);
        delay(500);
	int resetBlock6 = cmdWrite(0x06, 0x00, 0x00, 0x00, 0x00);
        delay(500);
	int resetBlock7 = cmdWrite(0x07, 0x00, 0x00, 0x00, 0x00);
        delay(500);
	
	//verify successful operation of all reset commands
	int returnVal = resetConfig + resetByteTrack + resetBlock2 + resetBlock3 + resetBlock4 + resetBlock5 + resetBlock6 + resetBlock7;
	
	if (returnVal > 0) {
		return 1; //one or more resets failed
	}
	else {return 0;}
}

//***************************************Housekeeping Functions**************************************//
/** Verify successful command completion
 *
 * @return  0 if successful, 1 if not
 *
 * @status  Tested, works
 **/
int SM125::verifySuccess() {	
	if (serialRFID.read() == 0xFF) {
		if (serialRFID.read() == 0x01) {
			if(serialRFID.read() == 0x01) {
				if (serialRFID.read() == 0x99) {
					if (serialRFID.read() == 0x9B) {
						return 0;
					}
					else return 1;
				}
				else return 1;
			}
			else return 1;
		}
		else return 1;
	}
	else return 1;
}
//*******************************************Debug Commands******************************************//
/** DEBUG: Read the response of the RFID reader
 *
 * @return  Nothing, prints response to Serial port
 *
 * @status  Tested, Works
 **/
void SM125::printCmdResponse() {
  if(serialRFID.available()) {
    Serial.println("");
    Serial.print("Header: ");
    Serial.print(serialRFID.read(), HEX);
    Serial.print('\n');
    Serial.print("Reserved: ");
    Serial.print(serialRFID.read(), HEX);
    Serial.print('\n');
    Serial.print("Length: ");
    int length = (int)serialRFID.read();
    Serial.print(length);
    Serial.print('\n');
    Serial.print("Command: ");
    Serial.print(serialRFID.read(), HEX);
    Serial.print('\n');
    for(int i = 0; i<length - 1; i++){              //response is length - 1 because the command is included
      Serial.print("Response: ");
      Serial.print(serialRFID.read(), HEX);
      Serial.print('\n');
    }
    Serial.print("Checksum: ");
    Serial.print(serialRFID.read(), HEX);
    Serial.print('\n');
  }
}

//******************************************Response Commands****************************************//
/** Return Entire SM125 Response
 *
 * Note: Byte has to be statically defined in order to return correctly. This is memory inefficient and should probably
 * be implemented in a better fashion. 
 *
 * @return  byte*		SM125Response	Byte array containing entire SM125 response. First byte indicates array length
 *
 * @status  Tested, Works.
 **/
//definitely need to check the array math for this function
byte* SM125::getSM125Response() {
	//Create null byte array to return if command exits abnormally or no data is received
	static byte nullReturn[1] = {0x00};
	
	//Get reponse data from SM125
	if(serialRFID.available()) {
		byte responseHeader = serialRFID.read();
		byte responseReserved = serialRFID.read();
		byte responseHLength = serialRFID.read();
		
		//Convert hex length to integer for use in for loop and byte array definition
		int responseILength = (int)responseHLength;

		//Create byte array, size = payload length + header(3) + checksum(1) +  first length byte(1)
		int size = responseILength + 5;
		static byte SM125Response[50];
		
		//drop in existing data
		SM125Response[0] = responseHLength + 0x05;
		SM125Response[1] = responseHeader;
		SM125Response[2] = responseReserved;
		SM125Response[3] = responseHLength;
		SM125Response[4] = serialRFID.read(); //response command
			
		//Get SM125 response payload
		for(int i = 0; i<responseILength - 1; i++){              //response is length - 1 because the command is included
			SM125Response[i + 5] = serialRFID.read();
		}
		
		//Add in the checksum
		SM125Response[responseILength + 4] = serialRFID.read();
		
		//Return received data
		return SM125Response;
	}
	return nullReturn;
}

/** Return SM125 Response Payload
 *
 * Note: Byte has to be statically defined in order to return correctly. This is memory inefficient and should probably
 * be implemented in a better fashion. 
 *
 * @return  byte*		SM125Response	Byte array containing SM125 response payload. First byte indicates array length
 *
 * @status  Untested
 **/
byte* SM125::getSM125ResponsePayload() {
	//Create null byte array to return if command exits abnormally or no data is received
	static byte nullReturn[1] = {0x00};
	
	//Get reponse data from SM125
	if(serialRFID.available()) {
		serialRFID.read(); //Drop header byte
		serialRFID.read(); //Drop reserved byte
		byte responseHLength = serialRFID.read(); 
		
		//Convert hex length to integer for use in for loop and byte array definition
		int responseILength = (int)responseHLength;

		//Statically sized array [required in this implementation to return correctly]
		static byte SM125Response[50];
		
		//define length of response
		SM125Response[0] = responseHLength - 0x01;
		serialRFID.read(); //Drop response command byte
			
		//Get SM125 response payload
		for(int i = 0; i<responseILength - 1; i++){              //response is length - 1 because the command is included
			SM125Response[i + 1] = serialRFID.read();
		}
		
		serialRFID.read(); //drop checksum byte
		
		//Return received data
		return SM125Response;
	}
	return nullReturn;
}

