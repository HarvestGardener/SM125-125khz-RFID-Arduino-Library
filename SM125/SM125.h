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
#ifndef SM125_h
#define SM125_h

#include "WProgram.h"
#include "..\NewSoftSerial\NewSoftSerial.h"

class SM125 
{
  //Public functions
  public:
	//Constructor
    SM125(int rxPin, int txPin);
	
	//Basic Commmands
    int cmdStaticExec(byte* cmdToExecute, int len);
    int cmdRead (byte hexMode, byte hexBlock);
	int cmdReadwPass (byte hexMode, byte hexBlock, byte hexPByte1, byte hexPByte2, byte hexPByte3, byte hexPByte4);
	int cmdWrite (byte hexBlock, byte hexByte1, byte hexByte2, byte hexByte3, byte hexByte4);
	int cmdWritewPass (byte hexMode, byte hexBlock, byte hexByte1, byte hexByte2, byte hexByte3, byte hexByte4, byte hexPByte1, byte hexPByte2, byte hexPByte3, byte hexPByte4);
	int cmdSetAutoMode (byte hexAutoMode, byte hexMode, byte hexBlock, byte hexPassMode, byte hexPByte1, byte hexPByte2, byte hexPByte3, byte hexPByte4);
	
	//Configuration Commands
	int cmdSetProgParam (byte hexWG, byte hexSG, byte hexONE, byte hexZERO, byte hexPADF);
	void cmdReadFirmwareVersion();
	
	//Command Aliases
	int cmdStopRead();
	int cmdReset();
	int cmdSleep();
	int cmdSetQ5ProgParam();
	int cmdSetT5557ProgParam();
	int cmdResetQ5Tag();
	int cmdResetT5557Tag();
	
	//Reponse Commands
	byte* getSM125Response();
	byte* getSM125ResponsePayload();
	
	//Debug Commands
	void printCmdResponse();
	
  //Private functions & variables
  private:
    int verifySuccess();
	NewSoftSerial serialRFID;
};

#endif