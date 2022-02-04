//-----------------------------------------------------------------------------
//   File:      bulkloop.c
//   Contents:  Hooks required to implement USB peripheral function.
//
// $Archive: /USB/Examples/FX2LP/vend_ax/Vend_Ax.c $
// $Date: 3/23/05 3:01p $
// $Revision: 3 $
//
//
//-----------------------------------------------------------------------------
// Copyright 2003, Cypress Semiconductor Corporation
//-----------------------------------------------------------------------------
#pragma NOIV					// Do not generate interrupt vectors

#include "fx2.h"
#include "fx2regs.h"
#include <intrins.h> //Jay Edit 6/06/2021

extern BOOL	GotSUD;			// Received setup data flag
extern BOOL	Sleep;
extern BOOL	Rwuen;
extern BOOL	Selfpwr;

BYTE	Configuration;		// Current configuration
BYTE	AlternateSetting;	// Alternate settings

//-----------------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------------
#define	VR_UPLOAD		0xc0
#define VR_DOWNLOAD		0x40

#define VR_ANCHOR_DLD   0xa0 // handled by core
#define VR_EEPROM		0xa2 // loads (uploads) EEPROM
#define	VR_RAM			0xa3 // loads (uploads) external ram
#define VR_SETI2CADDR	0xa4
#define VR_GETI2C_TYPE  0xa5 // 8 or 16 byte address
#define VR_GET_CHIP_REV 0xa6 // Rev A, B = 0, Rev C = 2 // NOTE: New TNG Rev
#define VR_TEST_MEM     0xa7 // runs mem test and returns result
#define VR_RENUM	    0xa8 // renum
#define VR_DB_FX	    0xa9 // Force use of double byte address EEPROM (for FX)
#define VR_I2C_100    0xaa // put the i2c bus in 100Khz mode
#define VR_I2C_400    0xab // put the i2c bus in 400Khz mode
#define VR_NOSDPAUTO  0xac // test code. does uploads using SUDPTR with manual length override

#define SERIAL_ADDR		0x50
#define EP0BUFF_SIZE	0x40

#define GET_CHIP_REV()		((CPUCS >> 4) & 0x00FF) // EzUSB Chip Rev Field

#define UNUSED(x)  if(x){}

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
BYTE	DB_Addr;					//TPM Dual Byte Address stat
BYTE	I2C_Addr;					//TPM I2C address

BYTE	ACTVCM_IDAddr;		 // Actuator device ID Address
WORD  ACTVCM_val;        // Actuator position DAC value
BYTE	ACTVCM_ADIDAddr;   // ADC device ID Address
BYTE	ACTVCM_ADRegAddr;  // Read back from ADC value
BYTE  IAN209_IDAddr;			// A/D Convert device ID Address
BYTE	IAN209_RegAddr;		 // A/D Convert Register Address
WORD	IAN209_val;				// A/D Convert Value
BYTE  SelectI2C;       // Select I2C Port
BYTE  SelectRelay;    // Select Relay Port
BYTE  SelectLED;     // Select LED Port
BYTE  TimerCounter=0x00;


//-----------------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------------
void EEPROMWrite(WORD addr, BYTE length, BYTE xdata *buf); //TPM EEPROM Write
void EEPROMRead(WORD addr, BYTE length, BYTE xdata *buf);  //TPM EEPROM Read


//Vic Edit
void Init_GPIOB(void);
void Init_GPIOC(void);
void Init_ActuatorVal(void);
void Init_IAN209Val(void);

void INA209_I2CWrite(BYTE device_ID , BYTE reg_addr, WORD value);
void INA209_I2CRead(BYTE device_ID , BYTE reg_addr, WORD xdata *buf);

void Simulate_I2C_Start(BYTE SelI2C);
void Simulate_I2C_Stop(BYTE SelI2C);
void Simulate_I2C_Data(BYTE value,BYTE SelI2C);
void Init_AK7375(BYTE device_ID, BYTE value); //Jay Edit 7/23/2021
void AK7375_Simulate_I2C(BYTE device_ID, WORD value); //Jay Edit 7/23/2021
void AK7375_I2CRead(BYTE device_ID, BYTE reg_addr, BYTE xdata *buf);

void RelayOn(BYTE SelectItem);
void RelayOff(BYTE SelectItem);
void LEDOn(BYTE SelectItem);
void LEDOff(BYTE SelectItem);

//void DelayUS(unsigned short int us); //Jay Edit 6/06/2021

//-----------------------------------------------------------------------------
// Task Dispatcher hooks
//	The following hooks are called by the task dispatcher.
//-----------------------------------------------------------------------------

void TD_Init(void) 				// Called once at startup
{
        EZUSB_InitI2C();			// Initialize I2C Bus
	   
	    //Init_LEDReg();	//Vic Edit
        Init_GPIOB();	  //Vic Edit
        Init_GPIOC();   //Vic Edit
        Init_ActuatorVal(); //Vic Edit
        Init_IAN209Val(); // Vic Edit
}

void TD_Poll(void) 				// Called repeatedly while the device is idle
{
}

BOOL TD_Suspend(void) 			// Called before the device goes into suspend mode
{
	return(TRUE);
}

BOOL TD_Resume(void) 			// Called after the device resumes
{
	return(TRUE);
}

//-----------------------------------------------------------------------------
// Device Request hooks
//	The following hooks are called by the end point 0 device request parser.
//-----------------------------------------------------------------------------


BOOL DR_GetDescriptor(void)
{
   return(TRUE);
}

BOOL DR_SetConfiguration(void)   // Called when a Set Configuration command is received
{
   Configuration = SETUPDAT[2];
   return(TRUE);            // Handled by user code
}

BOOL DR_GetConfiguration(void)   // Called when a Get Configuration command is received
{
   EP0BUF[0] = Configuration;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);            // Handled by user code
}

BOOL DR_SetInterface(void)       // Called when a Set Interface command is received
{
   AlternateSetting = SETUPDAT[2];
   return(TRUE);            // Handled by user code
}

BOOL DR_GetInterface(void)       // Called when a Set Interface command is received
{
   EP0BUF[0] = AlternateSetting;
   EP0BCH = 0;
   EP0BCL = 1;
   return(TRUE);            // Handled by user code
}

BOOL DR_GetStatus(void)
{
   return(TRUE);
}

BOOL DR_ClearFeature(void)
{
   return(TRUE);
}

BOOL DR_SetFeature(void)
{
   return(TRUE);
}


// Device request parser
// The 8 bytes in a USB SETUPDAT Packet
// Byte[0] : bmRequestType --> Request type,Direction and Recipicent.
// Byte[1] : bRequest --> The actual request.
// Byte[2] : wValueL
// Byte[3] : wValueH
// Byte[4] : wIndexL
// Byte[5] : wIndexH
// Byte[6] : wLengthL
// Byte[7] : wLengthH
BOOL DR_VendorCmnd(void)
{
	WORD		addr, len, bc;
	WORD		ChipRev;
	WORD    i;
	BYTE    xdata test[2];


	// Determine I2C boot eeprom device address; addr = 0x0 for 8 bit addr eeproms (24LC00)
	I2C_Addr = SERIAL_ADDR | ((I2CS & 0x10) >> 4); // addr=0x01 for 16 bit addr eeprom (LC65)
	// Indicate if it is a dual byte address part
	DB_Addr = (BOOL)(I2C_Addr & 0x01); //TPM: ID1 is 16 bit addr bit - set by rocker sw or jumper
	
	switch(SETUPDAT[1])
	{ //TPM handle new commands

			case 0xEF:	//EEPROM Read
				addr = SETUPDAT[2];		// Get address and length
				addr |= SETUPDAT[3] << 8;
				len = SETUPDAT[6];
				len |= SETUPDAT[7] << 8;

					while(len)					// Move requested data through EP0IN 
					{							// one packet at a time.	
	          while(EP0CS & bmEPBUSY);	
						if(len < EP0BUFF_SIZE)
							bc = len;
						else
							bc = EP0BUFF_SIZE;
	
							for(i=0; i<bc; i++)
								*(EP0BUF+i) = 0xcd;
						
						EEPROMRead(addr,(WORD)bc,(WORD)EP0BUF);
						EP0BCH = 0;
						EP0BCL = (BYTE)bc; // Arm endpoint with # bytes to transfer	
						addr += bc;
						len -= bc;
	
					}
				break;

			case 0xEE:	// EEPROM Write
				addr = SETUPDAT[2];		// Get address and length
				addr |= SETUPDAT[3] << 8;
				len = SETUPDAT[6];
				len |= SETUPDAT[7] << 8;
	
				while(EP0CS & bmEPBUSY);
	
				test[0] = SETUPDAT[4];
				test[1] = 0xCD;
				EEPROMWrite(addr,0x01,&test[0]);
				break;

			case 0xE0: //Run Mode
				CPUCS = 0x0A;
				OED = 0x18;
				IOD = 0x00;
				TimerCounter = 0;
				EZUSB_Delay(500);
				break;
				
			case 0xE1: //Stop Run
				OED = 0x00;
				EZUSB_Delay(500);
				CPUCS = 0x08;
				IFCONFIG |= 20;
				PCON |= 0x01;
				EZUSB_Delay(500); //ms
				break;

//----------------------- For Actuator Test Command (Start)-----------------------//

			case 0xB0: //Set Device ID for Actuator
				addr = SETUPDAT[2];		// Get address and length
				addr |= SETUPDAT[3] << 8;
				len = SETUPDAT[6];
				len |= SETUPDAT[7] << 8;
	
				while(EP0CS & bmEPBUSY);

				ACTVCM_IDAddr = (addr & 0xff);
				break;

			case 0xB1: //Move Actuator
				addr = SETUPDAT[2];		// Get address and length
				addr |= SETUPDAT[3] << 8;
				len = SETUPDAT[6];
				len |= SETUPDAT[7] << 8;
				
				SelectI2C = SETUPDAT[4];
					
				while(EP0CS & bmEPBUSY);

				if(addr > 0x0fff)
					addr = 0x0fff;
				if(addr < 0)
					addr = 0x0000;     

				//RelayOff(SelectRelay); // switch to the driver loop
        AK7375_Simulate_I2C(ACTVCM_IDAddr,addr);				
				break;
				
			case 0xB2: //Initial Actuator to Stand-by Mode
				addr = SETUPDAT[2];		// Get address and length
				//addr |= SETUPDAT[3] << 8;
				//len = SETUPDAT[6];
				//len |= SETUPDAT[7] << 8;
				
				SelectI2C = SETUPDAT[4];
					
				while(EP0CS & bmEPBUSY);

				if(addr > 0xff)
					addr = 0x0fff;
				if(addr < 0)
					addr = 0x00;
				
				Init_AK7375(ACTVCM_IDAddr, addr); 
				break;
				
			case 0xB3: //Initial Actuator to Active Mode
				addr = SETUPDAT[2];		// Get address and length
				//addr |= SETUPDAT[3] << 8;
				//len = SETUPDAT[6];
				//len |= SETUPDAT[7] << 8;
				
				SelectI2C = SETUPDAT[4];
					
				while(EP0CS & bmEPBUSY);

				if(addr > 0x0fff)
					addr = 0xff;
				if(addr < 0)
					addr = 0x00;
				
  			Init_AK7375(ACTVCM_IDAddr, addr); 
				break;
				
			case 0xB4: //Initial Actuator to Sleep Mode
				addr = SETUPDAT[2];		// Get address and length
				//addr |= SETUPDAT[3] << 8;
				//len = SETUPDAT[6];
				//len |= SETUPDAT[7] << 8;
				
				SelectI2C = SETUPDAT[4];
					
				while(EP0CS & bmEPBUSY);

				if(addr > 0x0fff)
					addr = 0xff;
				if(addr < 0)
					addr = 0x00;
				
    	  Init_AK7375(ACTVCM_IDAddr, addr); 
				break;
				
			case 0xB5: // Read-out the regdata
				ACTVCM_ADRegAddr = SETUPDAT[4];		// Get c++ Reg address
				len = SETUPDAT[6];
				len |= SETUPDAT[7] << 8;
				while(EP0CS & bmEPBUSY);
			  AK7375_I2CRead(ACTVCM_ADIDAddr,ACTVCM_ADRegAddr,EP0BUF);
				EP0BCH = 0;
			  EP0BCL = 1; // byte counter 1 for read
				break;

				
//----------------------- For Actuator Test Command ( End )-----------------------//


//----------------------- For ADC Command (Start) -----------------------//

			case 0xC0:  //Set Device ID for IAN209
				addr = SETUPDAT[2];		// Get address and length
				addr |= SETUPDAT[3] << 8;
				len = SETUPDAT[6];
				len |= SETUPDAT[7] << 8;
	
				while(EP0CS & bmEPBUSY);

				IAN209_IDAddr = (addr & 0xff);
				break;

			case 0xC1:	//Read IAN209
				IAN209_RegAddr = SETUPDAT[4];		// Get c++ Reg address

				while(EP0CS & bmEPBUSY);
				INA209_I2CRead(IAN209_IDAddr,IAN209_RegAddr,(BYTE)EP0BUF); // call fx2 function call

				EP0BCH = 0;
			  EP0BCL = 2; // byte counter 1 for read
				break;

		  case 0xC2:	//Write  IAN209
				IAN209_val = SETUPDAT[2];		// Get c++ value
				IAN209_val |= SETUPDAT[3] << 8;				
				IAN209_RegAddr = SETUPDAT[4]; // Get c++ RegADDR
				INA209_I2CWrite(IAN209_IDAddr,IAN209_RegAddr,IAN209_val); // call fx2 function call
				EP0BCH = 0;
			  EP0BCL = 1;
				break;

//----------------------- For ADC Command ( End )-----------------------//

//----------------------- For Relay and LED Command (Start) -----------------------//

			case 0xD0:  //Set Relay to High	
				SelectRelay = SETUPDAT[4];
				while(EP0CS & bmEPBUSY);
				RelayOn(SelectRelay);
				break;

			case 0xD1:	//Set Relay to Low
				SelectRelay = SETUPDAT[4];
				while(EP0CS & bmEPBUSY);
				RelayOff(SelectRelay);
				break;

			case 0xD2:	//Set LED on
				SelectLED = SETUPDAT[4];
				while(EP0CS & bmEPBUSY);
				LEDOn(SelectLED);
				break;

			case 0xD3:	// Set LED off
				SelectLED = SETUPDAT[4];
				while(EP0CS & bmEPBUSY);
				LEDOff(SelectLED);
			  break;

	  case VR_GETI2C_TYPE:
				*EP0BUF = DB_Addr;
				EP0BCH = 0;
				EP0BCL = 1; // Arm endpoint with # bytes to transfer
				EP0CS |= bmHSNAK; // Acknowledge handshake phase of device request
		    break;
		
		case VR_GET_CHIP_REV:
					ChipRev = GET_CHIP_REV();
					*EP0BUF = ChipRev;
					EP0BCH = 0;
					EP0BCL = 1; // Arm endpoint with # bytes to transfer
					EP0CS |= bmHSNAK; // Acknowledge handshake phase of device request
			    break;
		
		case VR_TEST_MEM:
					*EP0BUF = 0x0F; // Fail
					EP0BCH = 0;
					EP0BCL = 1; // Arm endpoint with # bytes to transfer
					EP0CS |= bmHSNAK; // Acknowledge handshake phase of device request
		    	break;
		
		case VR_SETI2CADDR:
			   I2C_Addr = SETUPDAT[2];
		  	 break;
		
		case VR_I2C_100:
			I2CTL &= ~bm400KHZ;
			EP0BCH = 0;
			EP0BCL = 0;
			break;
		
		case VR_I2C_400:
			I2CTL |= bm400KHZ;
			EP0BCH = 0;
			EP0BCL = 0;
			break;
		
		case VR_RENUM:
					*EP0BUF = 7;
					EP0BCH = 0;
					EP0BCL = 1; // Arm endpoint with # bytes to transfer
					EP0CS |= bmHSNAK; // Acknowledge handshake phase of device request
					EZUSB_Delay(1000);
					EZUSB_Discon(TRUE);		// renumerate until setup received
			break;
		
		case VR_NOSDPAUTO:
         // we want specify our own length for the transfer so
         // disable the automatic length feature of the Setup Data Autopointer
         SUDPTRCTL &= ~bmSDPAUTO;
         EP0BCH = SETUPDAT[7];
         EP0BCL = SETUPDAT[6];
         SUDPTRH = SETUPDAT[3];
         SUDPTRL = SETUPDAT[2];
         break;
			
		case VR_DB_FX:
			DB_Addr = 0x01;		//TPM: need to assert double byte
			I2C_Addr |= 0x01;	//TPM: need to assert double byte
        // NOTE: This case falls through !
		case VR_RAM:
		case VR_EEPROM:
			addr = SETUPDAT[2];		// Get address and length
			addr |= SETUPDAT[3] << 8;
			len = SETUPDAT[6];
			len |= SETUPDAT[7] << 8;
			// Is this an upload command ?
			if(SETUPDAT[0] == VR_UPLOAD)
			{
				while(len)					// Move requested data through EP0IN 
				{							// one packet at a time.

          while(EP0CS & bmEPBUSY);

					if(len < EP0BUFF_SIZE)
						bc = len;
					else
						bc = EP0BUFF_SIZE;

					// Is this a RAM upload ?
					if(SETUPDAT[1] == VR_RAM)
					{
						for(i=0; i<bc; i++)
							*(EP0BUF+i) = *((BYTE xdata *)addr+i);
					}
					else
					{
						for(i=0; i<bc; i++)
							*(EP0BUF+i) = 0xcd;
						EEPROMRead(addr,(WORD)bc,(WORD)EP0BUF);
					}

					EP0BCH = 0;
					EP0BCL = (BYTE)bc; // Arm endpoint with # bytes to transfer

					addr += bc;
					len -= bc;

				}
			}
			// Is this a download command ?
			else if(SETUPDAT[0] == VR_DOWNLOAD)
			{
				while(len)					// Move new data through EP0OUT 
				{							// one packet at a time.
					// Arm endpoint - do it here to clear (after sud avail)
					EP0BCH = 0;
					EP0BCL = 0; // Clear bytecount to allow new data in; also stops NAKing

					while(EP0CS & bmEPBUSY);

					bc = EP0BCL; // Get the new bytecount

					// Is this a RAM download ?
					if(SETUPDAT[1] == VR_RAM)
					{
						for(i=0; i<bc; i++)
							 *((BYTE xdata *)addr+i) = *(EP0BUF+i);
					}
					else
						EEPROMWrite(addr,bc,(WORD)EP0BUF);

					addr += bc;
					len -= bc;
				}
			}
			break;
	}
	return(FALSE); // no error; command handled OK
}

//-----------------------------------------------------------------------------
// USB Interrupt Handlers
//	The following functions are called by the USB interrupt jump table.
//-----------------------------------------------------------------------------

// Setup Data Available Interrupt Handler
void ISR_Sudav(void) interrupt 0
{
   // enable the automatic length feature of the Setup Data Autopointer
   // in case a previous transfer disbaled it
   SUDPTRCTL |= bmSDPAUTO;

   GotSUD = TRUE;            // Set flag
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUDAV;         // Clear SUDAV IRQ
}

// Setup Token Interrupt Handler
void ISR_Sutok(void) interrupt 0
{
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUTOK;         // Clear SUTOK IRQ
}

void ISR_Sof(void) interrupt 0
{
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSOF;            // Clear SOF IRQ
}

void ISR_Ures(void) interrupt 0
{
   if (EZUSB_HIGHSPEED())
   {
      pConfigDscr = pHighSpeedConfigDscr;
      pOtherConfigDscr = pFullSpeedConfigDscr;
   }
   else
   {
      pConfigDscr = pFullSpeedConfigDscr;
      pOtherConfigDscr = pHighSpeedConfigDscr;
   }
   
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmURES;         // Clear URES IRQ
}

void ISR_Susp(void) interrupt 0
{
   Sleep = TRUE;
   EZUSB_IRQ_CLEAR();
   USBIRQ = bmSUSP;
}

void ISR_Highspeed(void) interrupt 0
{
   if (EZUSB_HIGHSPEED())
   {
      pConfigDscr = pHighSpeedConfigDscr;
      pOtherConfigDscr = pFullSpeedConfigDscr;
   }
   else
   {
      pConfigDscr = pFullSpeedConfigDscr;
      pOtherConfigDscr = pHighSpeedConfigDscr;
   }

   EZUSB_IRQ_CLEAR();
   USBIRQ = bmHSGRANT;
}

void ISR_Ep0ack(void) interrupt 0
{
}
void ISR_Stub(void) interrupt 0
{
}
void ISR_Ep0in(void) interrupt 0
{
}
void ISR_Ep0out(void) interrupt 0
{
}
void ISR_Ep1in(void) interrupt 0
{
}
void ISR_Ep1out(void) interrupt 0
{
}
void ISR_Ep2inout(void) interrupt 0
{
}
void ISR_Ep4inout(void) interrupt 0
{
}
void ISR_Ep6inout(void) interrupt 0
{
}
void ISR_Ep8inout(void) interrupt 0
{
}
void ISR_Ibn(void) interrupt 0
{
}
void ISR_Ep0pingnak(void) interrupt 0
{
}
void ISR_Ep1pingnak(void) interrupt 0
{
}
void ISR_Ep2pingnak(void) interrupt 0
{
}
void ISR_Ep4pingnak(void) interrupt 0
{
}
void ISR_Ep6pingnak(void) interrupt 0
{
}
void ISR_Ep8pingnak(void) interrupt 0
{
}
void ISR_Errorlimit(void) interrupt 0
{
}
void ISR_Ep2piderror(void) interrupt 0
{
}
void ISR_Ep4piderror(void) interrupt 0
{
}
void ISR_Ep6piderror(void) interrupt 0
{
}
void ISR_Ep8piderror(void) interrupt 0
{
}
void ISR_Ep2pflag(void) interrupt 0
{
}
void ISR_Ep4pflag(void) interrupt 0
{
}
void ISR_Ep6pflag(void) interrupt 0
{
}
void ISR_Ep8pflag(void) interrupt 0
{
}
void ISR_Ep2eflag(void) interrupt 0
{
}
void ISR_Ep4eflag(void) interrupt 0
{
}
void ISR_Ep6eflag(void) interrupt 0
{
}
void ISR_Ep8eflag(void) interrupt 0
{
}
void ISR_Ep2fflag(void) interrupt 0
{
}
void ISR_Ep4fflag(void) interrupt 0
{
}
void ISR_Ep6fflag(void) interrupt 0
{
}
void ISR_Ep8fflag(void) interrupt 0
{
}
void ISR_GpifComplete(void) interrupt 0
{
}
void ISR_GpifWaveform(void) interrupt 0
{
}

void EEPROMWriteByte(WORD addr, BYTE value)
{
	BYTE		i = 0;
	BYTE xdata 	ee_str[3];

	if(DB_Addr)
		ee_str[i++] = MSB(addr);

	ee_str[i++] = LSB(addr);
	ee_str[i++] = value;

	EZUSB_WriteI2C(I2C_Addr, i, ee_str);
   EZUSB_WaitForEEPROMWrite(I2C_Addr);
}


void EEPROMWrite(WORD addr, BYTE length, BYTE xdata *buf)
{
	BYTE	i;
	for(i=0;i<length;++i)
		EEPROMWriteByte(addr++,buf[i]);
}

void EEPROMRead(WORD addr, BYTE length, BYTE xdata *buf)
{
	BYTE		i = 0;
	//BYTE		j = 0;
	BYTE xdata 	ee_str[2];

	if(DB_Addr)
		ee_str[i++] = MSB(addr);

	ee_str[i++] = LSB(addr);

	EZUSB_WriteI2C(I2C_Addr, i, ee_str);

//	for(j=0; j < length; j++)
//		*(buf+j) = 0xcd;

	EZUSB_ReadI2C(I2C_Addr, length, buf);
}

////////////////////////////////////////////
 void Init_GPIOB(void)
{
	OEB = 0xFF;	//Set Port B Output Enable ; 0: Disable , 1: Set Enable
	EZUSB_Delay(500);
	IFCONFIG = IFCONFIG & 0xFC; // Set IFCFG0=0 and IFCFG1=0 for "port mode", See TRM Document (Table 13-10)

	//IOB = 0xBB; //bit0 and bit1 are 1
	IOB = 0x33; //bit0 and bit1 are 1
}

void Init_GPIOC(void)
{
	OEC = 0xFF;	//Set Port C Output Enable ; 0: Disable , 1: Set Enable
	EZUSB_Delay(500);
	IFCONFIG = IFCONFIG & 0xFC; // Set IFCFG0=0 and IFCFG1=0 for "port mode", See TRM Document (Table 13-10)

	//IOC = 0xBB;
	IOC = 0x13;
}

void Init_ActuatorVal(void)
{
	 ACTVCM_IDAddr = 0x18; // W:0x18, R:0x19 for AK7375
	 ACTVCM_ADIDAddr = 0x19; // R:0x19 for AK7375
}

void Init_IAN209Val(void)
{
	 IAN209_IDAddr = 0x80; //0x80 for IAN209 A0:GND , A1:GND
}

/* //GPIO I2C
 void Simulate_I2C_Start(BYTE SelI2C)
{
	int j=0;
	//Start condition
	switch(SelI2C)
	{
		case 0:  // PB0 , PB1
			IOB = IOB & 0xFD;
			j=0; //For delay
			IOB = IOB & 0xFC;
		break;

		case 1:  // PB4 , PB5
			IOB = IOB & 0xDF;
			j=0; //For delay
			IOB = IOB & 0xCF;
		break;

		case 2:  // PC0 , PC1
			IOC = IOC & 0xFD;
			j=0; //For delay
			IOC = IOC & 0xFC;
		break;
	}
}

void Simulate_I2C_Stop(BYTE SelI2C)
{
	BYTE j=0;
	//End condition
	switch(SelI2C)
	{
		case 0:  // PB0 , PB1
			//IOB = IOB | 0x02; // original
		  IOB = IOB | 0x01; // Jay Edit 6/06/2021
		  j=0; //For delay
			IOB = IOB | 0x03;
		break;

					//case 1:  // PB2 , PB3
					//	IOB = IOB | 0x08;
					//	IOB = IOB | 0x0C;
					//break;

		case 1:  // PB4 , PB5
			//IOB = IOB | 0x20; // original
		  IOB = IOB | 0x10; // Jay Edit 6/06/2021
			IOB = IOB | 0x30;
		break;

					//case 3:  // PB6 , PB7
					//	IOB = IOB | 0x80;
					//	IOB = IOB | 0xC0;
					//break;

	   case 2:  // PC0 , PC1
	  	 //IOC = IOC | 0x02; // original
		  IOC = IOC | 0x01;// Jay Edit 6/06/2021
			IOC = IOC | 0x03;
		break;
    }
}

void Simulate_I2C_Data(BYTE value,BYTE SelI2C)
{
	BYTE		i = 0,j=0;
	BYTE		Value_Array[8]={0};

	for(i=0;i<8;++i)
	{
		Value_Array[i] = (value >> (7 - i)) & 0x01;
	}

	switch(SelI2C)
	{
		case 0: // I2C via PB0-->SCK , PB1-->SDA
			for(i=0;i<8;++i)
			{
				PB1 = Value_Array[i];
				PB0 = 0x01;
				j=0; //For delay
				PB0 = 0x00;
				PB1 = 0x00;
			}

			//wait for ack
			OEB = 0xFD;
			while(1)
			{
				PB0 = 0x01;
				if( (PB1 & 0x01) == 0x00)
				 break;
		
			    PB0 = 0x00;
		    }
			PB0 = 0x00;
			OEB = 0xFF;
		break;


		case 1: // I2C via PB4-->SCK , PB5-->SDA
			for(i=0;i<8;++i)
			{
				PB5 = Value_Array[i];
				PB4 = 0x01;
				j=0; //For delay
				PB4 = 0x00;
				PB5 = 0x00;
			}

			//wait for ack
			OEB = 0xDF;
			while(1)
			{
				PB4 = 0x01;
				if( (PB5 & 0x01) == 0x00)
				 break;
		
			    PB4 = 0x00;
		    }
			PB4 = 0x00;
			OEB = 0xFF;
		break;


	   case 2: // I2C via PC0-->SCK , PC1-->SDA
			for(i=0;i<8;++i)
			{
				PC1 = Value_Array[i];
				PC0 = 0x01;
				j=0; //For delay
				PC0 = 0x00;
				PC1 = 0x00;
			}

			//wait for ack
			OEC = 0xFD;
			while(1)
			{
				PC0 = 0x01;
				if( (PC1 & 0x01) == 0x00)
				 break;
		
			    PC0 = 0x00;
		    }
			PC0 = 0x00;
			OEC = 0xFF;
		break;
	}	
}
*/

void Init_AK7375(BYTE device_ID, BYTE value) //Jay Edit
{
/* //GPIO I2C
	Simulate_I2C_Start(SelI2C);
	Simulate_I2C_Data(device_ID,SelI2C);
	Simulate_I2C_Data(0x02,SelI2C); // set to active mode, start to move
	Simulate_I2C_Data(value&0xFF,SelI2C);
  Simulate_I2C_Stop(SelI2C);	
*/
	BYTE		i = 0;
	BYTE xdata 	ee_str[2];

	ee_str[i++] = 0x02;
	ee_str[i++] = value&0xFF;

	EZUSB_WriteI2C(device_ID, i, ee_str);
}

void AK7375_Simulate_I2C(BYTE device_ID, WORD value) //Jay Edit 
{
/* //GPIO I2C
//[3]write one by one byte
	Simulate_I2C_Start(SelI2C);
	Simulate_I2C_Data(device_ID,SelI2C);
	Simulate_I2C_Data(0x00,SelI2C);
	Simulate_I2C_Data((value >> 4)&0xFF,SelI2C);
	Simulate_I2C_Stop(SelI2C);
	//
	Simulate_I2C_Start(SelI2C);
	Simulate_I2C_Data(device_ID,SelI2C);
	Simulate_I2C_Data(0x01,SelI2C);
	Simulate_I2C_Data((value&0x0F)<<4,SelI2C); 
	Simulate_I2C_Stop(SelI2C);
//
*/ 
//[4]use Cypress I2C& connect to EEPROM I2C PIN
	BYTE		i = 0;
	BYTE xdata 	ee_str[2];

  ee_str[i++] = 0x00;
	ee_str[i++] = (value >> 4)&0xFF;
	EZUSB_WriteI2C(device_ID, i, ee_str);
//
	i = 0;

  ee_str[i++] = 0x01;
	ee_str[i++] = (value&0x0F)<<4;
	EZUSB_WriteI2C(device_ID, i, ee_str);
//
}

/* //GPIO I2C
BYTE GetByte(void)
{
	BYTE i, a;
	BYTE temp = 0, j=0;
	PB1 = 0x01;
	//OEB = 0xFD; // Set I/O OEB b0:1(output) b1:0(input)
    for(i = 0; i < 8; i++)
    {
        PB0 = 0x00;
        j=0; //For delay;
        PB0 = 0x01;
        if(PB1)
        {
            a = 0x01;
        }
        else
        {
            a = 0;
        }
        
        temp |= (a << (7 - i));
        j=0; //For delay
    }
    
    PB0 = 0x00;

	//
		OEB = 0xFF; // Reset OEB for later ACK: b0:1(output) b1:1(output)
	return(temp);
}
*/

void AK7375_I2CRead(BYTE device_ID, BYTE reg_addr, BYTE xdata *buf)
{
	BYTE		i = 0;
	BYTE xdata 	ee_str[1];
	ee_str[i++] = reg_addr;
	
	EZUSB_WriteI2C(0x18, i, ee_str);
	EZUSB_ReadI2C(device_ID, i, buf);
	
/* //GPIO I2C	
//ADC Data Readout: how to receieve?
	BYTE		i = 0,j=0;
  BYTE		r1Value_Array;
//BYTE		r2Value_Array[8]={0};
//WORD    rVCMDAC[16]={0};

//Write Dummy Command
	Simulate_I2C_Start(SelI2C);
	Simulate_I2C_Data(ACTVCM_IDAddr,SelI2C); // 0x18
	Simulate_I2C_Data(reg_addr,SelI2C); // 0x84
  j=0; //delay

//Read Command
	Simulate_I2C_Stop(SelI2C); // Pull-up SDA line
  Simulate_I2C_Start(SelI2C); 

	Simulate_I2C_Data(device_ID,SelI2C); // 0x19

// 1st Data Transfer
  r1Value_Array = GetByte();

// 2nd Data Transfer

// No ACK
//	PB1 = 0x01;

	Simulate_I2C_Stop(SelI2C);

	return r1Value_Array;
*/
}


void RelayOn(BYTE SelectItem)
{
	switch(SelectItem)
    {
		case 0:
			PB2 = 0x00;	//Vpd1 = Off
			PB3 = 0x01;	//Vre1 = high
			break;
		case 1:
			PB6 = 0x00;	//Vpd2 = Off
			PB7 = 0x01;  //Vre2 = high
			break;
		case 2:
			PC2 = 0x00;	//Vpd3 = Off
			PC3 = 0x01;  //Vre3 = high
			break;
    }
}


void RelayOff(BYTE SelectItem)
{
	switch(SelectItem)
    {
		case 0:
			PB2 = 0x00;	//Vpd1 = off
			PB3 = 0x00;  //Vre1 = low
			break;
		case 1:
			PB6 = 0x00;	//Vpd2 = off
			PB7 = 0x00;  //Vre2 = low
			break;
		case 2:
			PC2 = 0x00;	//Vpd3 = off
			PC3 = 0x00;  //Vre3 = low
			break;
    }
}


void LEDOn(BYTE SelectItem)
{
	switch(SelectItem)
    {
		case 0:
			PC4 = 0x01;	//LED1 = On
			break;
		case 1:
			PC5 = 0x01;	//LED2 = On
			break;
		case 2:
			PC6 = 0x01;	//LED3 = On
			break;
    }
}


void LEDOff(BYTE SelectItem)
{
	switch(SelectItem)
    {
		case 0:
			PC4 = 0x00;	//LED1 = Off
			break;
		case 1:
			PC5 = 0x00;	//LED2 = Off
			break;
		case 2:
			PC6 = 0x00;	//LED3 = Off
			break;
    }
}

void INA209_I2CWrite(BYTE device_ID , BYTE reg_addr, WORD value)
{
	BYTE		i = 0;
	BYTE xdata 	ee_str[3];

	ee_str[i++] = reg_addr;
	ee_str[i++] = MSB(value);
	ee_str[i++] = LSB(value);

	EZUSB_WriteI2C(device_ID, i, ee_str);
}

void INA209_I2CRead(BYTE device_ID , BYTE reg_addr, WORD xdata *buf)
{
	BYTE		i = 0;
	BYTE xdata 	ee_str[2];
	ee_str[i++] = reg_addr;

	EZUSB_WriteI2C(device_ID, i, ee_str);
	EZUSB_ReadI2C(device_ID, 0x02, (WORD)buf);
//EZUSB_ReadI2C(device_ID, reg_addr, buf);
}


//Jay Edit 6/06/2021
/*
void DelayUS(unsigned short int us) // micro-second delay
{
	while (us--)
	{
		_nop_();
	}
}
*/

