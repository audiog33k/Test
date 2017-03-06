/*

Files modified in this program:
main.c
startup.c uVectorEntry




Rev. 0 top/bottom connectors are reversed.  Connectors are yellow on bottom and
white on top.  With Rev. 0 board, video is on top.

Looking at the back of the unit:

OUT3    OUT2    OUT1    OUT0    IN3     IN2     IN1     IN0     White

OUT7    OUT6    OUT5    OUT4    IN7     IN6     IN5     IN4     Yellow

Input 4, 5, 6,  7  have programmable 75 ohm termination.  Rev 0 board has these to yellow (should have been white)           

The idea is that the unit can be a 4:4 composite video matrix switcher with mono audio or can be an 8:8 
composite video matrix switcher without audio.



======================
Items to fix on schematic/PCB hello
======================
(1)  Reverse connections between top and bottom connectors.
(2)  Switch data bus to LCD bit 7.  Was on PB.7, change to PC.4
(3)  Switched cap voltage inverter output caps are not grounded.
(4)  Mounting holes for light pipes
(5)  Add a second switch into reset switch position

======================
To Do
======================
Done    Test 75 ohm termination mux
        Test sync detect
Done    Test 485


======================
Code To Do
======================





=====================
Serial EEPROM Mapping
=====================
0x00    Flag set to 0xA5 Address has been set
0x01    Network address Tens
0x02    Network address Ones
0x03    Termination.  '1' turn on termination.  '0' turn off termination
0x04
0x05
0x06
0x07
0x08
0x09
0x0A
0x0B
0x0C
0x0D
0x0E
0x0F
0x10    Flag set to 0xA5 if mapping for mux has be set
0x11    Input 1 destination routing, arrayMapping [8] is loaded from these locations
0x12    Input 2
0x13    Input 3
0x14    Input 4
0x15    Input 5
0x16    Input 6
0x17    Input 7
0x18    Input 8





*/

//#include "driverlib\inc\lm3s6911.h"
#include "driverlib\inc\hw_types.h"
#include "driverlib\inc\hw_memmap.h"
#include "driverlib\inc\hw_sysctl.h"
#include "driverlib\inc\hw_ints.h"
#include "driverlib\gpio.h"
#include "driverlib\sysctl.h"
#include "driverlib\i2c.h"
#include "driverlib\interrupt.h"
#include "driverlib\uart.h"
#include "driverlib\systick.h"


#define VERSION_MAJOR 		'0'
#define VERSION_MINOR 		'0'
#define VERSION_ALPHA 		'B'

#define MODEL1			'V'
#define MODEL2			'S'
#define MODEL3			'E'
#define MODEL4			'4'
#define MODEL5			'4'
#define MODEL6			'M'

#define NULL    0x00

#define EEPROM_ADDRESS_NETWORK_ADDRESS_FLAG     0x00
#define EEPROM_ADDRESS_NETWORK_ADDRESS_TENS     0x01
#define EEPROM_ADDRESS_NETWORK_ADDRESS_ONES     0x02
#define EEPROM_ADDRESS_TERMINATION              0x03
#define EEPROM_ADDRESS_CROSSPOINT_FLAG          0x10                        // Crosspoint flag, contains 0xA5 if values have been stored
#define EEPROM_ADDRESS_CROSSPOINT_DATA          0x11                        // This location (and next 7) contain crosspoint settings



#define PORTC	        GPIO_PORTC_BASE
#define PIN_LED		GPIO_PIN_5
#define PIN_SWITCH1      GPIO_PIN_6

#define PORT_DATA_BUS	GPIO_PORTB_BASE                                     // data bus, used for LCD interface and parallel interface to video mux
#define DATA_BUS_0	GPIO_PIN_0
#define DATA_BUS_1	GPIO_PIN_1
#define DATA_BUS_2	GPIO_PIN_2
#define DATA_BUS_3	GPIO_PIN_3
#define DATA_BUS_4	GPIO_PIN_4
#define DATA_BUS_5	GPIO_PIN_5
#define DATA_BUS_6	GPIO_PIN_6
#define DATA_BUS_7	GPIO_PIN_4                                          // moved to PC4  Pin 25

#define PORT_LCD_CONTROL    GPIO_PORTA_BASE
#define LCD_WR              GPIO_PIN_4         
#define LCD_A0              GPIO_PIN_5
#define RS485_TX_EN         GPIO_PIN_3                                      // DE "Data Enable", High assertion, Take high only while transmitting   

#define PORT_MUX_CONTROL  GPIO_PORTD_BASE
#define MUX_A0	          GPIO_PIN_0
#define MUX_A1	          GPIO_PIN_1
#define MUX_A2	          GPIO_PIN_2
#define MUX_RESET_NEG	  GPIO_PIN_3
#define MUX_CE_NEG        GPIO_PIN_4                                        // chip enable, low to allow clock and reset to work
#define MUX_CLK_NEG       GPIO_PIN_5                                        // falling edge triggered
#define MUX_UPDATE_NEG    GPIO_PIN_6
#define VIDEO_LOAD        GPIO_PIN_7                                        // enables analog switch to turn on/off 75 ohm load on audio inputs

#define PORT_E          GPIO_PORTE_BASE
#define PIN_HSYNC1      GPIO_PIN_0
#define PIN_HSYNC2      GPIO_PIN_1
#define PIN_HSYNC3      GPIO_PIN_2
#define PIN_HSYNC4      GPIO_PIN_3


#define I2C_SLAVE_ADDRESS_LEFT_EEPROM   0x41                                // 0100 0001, PCA9501 U3 I/O expander, left column, internal EEPROM 
#define I2C_SLAVE_ADDRESS_LEFT_BANK     0x01                                // PCA9501 U3 I/O expander, left column of LEDs
#define I2C_SLAVE_ADRESS_RIGHT_EEPROM   0x42                                // 0100 0001, PCA9501 U3 I/O expander, right column, internal EEPROM 
#define I2C_SLAVE_ADDRESS_RIGHT_BANK    0x02                                // PCA9501 U4 I/O expander, right column of LEDs
#define I2C_SLAVE_ADDRESS_EEPROM        0x50                                // 24LC01 EEPROM

#define LINE_ONE	                0x80
#define LINE_TWO	                0xC0
#define UARTFR                          0x18                                // UART Flag Offset

#define ON  0x01
#define OFF 0x00
#define DEFAULT_NETWORK_ADDRESS_TENS '0'
#define DEFAULT_NETWORK_ADDRESS_ONES '1'

void I2CSendDataLED(unsigned char recI2CData, unsigned char recI2CSlaveAddress);
void I2CSendDataEEPROM(unsigned char recI2CAddress, unsigned char recI2CData, unsigned char recI2CSlaveAddress );
unsigned char I2CReadDataEEPROM(unsigned char recI2CAddress, unsigned char recI2CSlaveAddress );
unsigned char I2CReadDataEEPROMCurrentAddress(unsigned char recI2CSlaveAddress );
unsigned char I2CReadDataEEPROMPCA9501(unsigned char recI2CAddress, unsigned char recI2CSlaveAddress );
void I2CSendDataEEPROMPCA9501(unsigned char recI2CAddress, unsigned char recI2CData, unsigned char recI2CSlaveAddress );
void LcdWritePulse(void);              
void InitDebugLCD(void);
void DebugPutChar(unsigned char c);
void ClearDebugLCDLine(unsigned char line);                                 // clears a single line on the debug LCD
void ClearDebugLCD(void);
void DebugLCDWrite(unsigned char lineNum, unsigned char pos,  char *str);
void DelayLCD100us(unsigned int recTime);
void DataBusOut (unsigned char recChar);
void SetupVideoMux (void);
void ControlVideoMux(unsigned char input, unsigned char output);
void Setup (void);
void UpdateLED (unsigned char inputVar, unsigned char outputVar);
void ProcessSerialCommand(void);
void SendString(unsigned char *rec);                                        // sends a string out uart, this is a blocking function
void ControlVideoInputLoad(unsigned char onOff);                            // 1=enable 75 ohm load, 0=turn off 75 ohm load
void PulseVideoMuxUpdate(void);                                             // pulses update low
void SysTickIntHandler(void);
unsigned int CheckTimerDifference (unsigned int recCompare, unsigned int recPrevious);
void ProcessSerialInput(void);
void CheckForSerialTimeOut(void);
void CheckForCrossPointMapping(void);
void SerialSplash(void);
void ProcessSerialSourceDestinationCommand(unsigned char recSource, unsigned char recDestination);
void ReadEepromNetworkAddress(void);
void Configuration(void);
void SendSerialString(void);                                                // sends an array of characters contained in txBuffer if serialTransmitFlag is set
void StringCopy(unsigned char *rec);
void SerialTransmitEnable(unsigned char copy, unsigned char *arrayPointer); // copy = 1 use array pointer, copy = 0 use existing txbuffer contents 
void Rs485FlowControl(void);                                                // Asserts a processor pin when ever UART is busy
void CheckForVideoPresence(void);                                           // checks video inputs 1-4 for the presence of video, loads a aserial string
void ResetDefaults(void);
void LookForSwitch(void);
void InitMapping(void);
void InitAddress (void);

//
// Top     Red      1011 1111   0xBF
//         Red      1110 1111   0xEF
//         Red      1111 1011   0xFB
// Bottom  Red      1111 1110   0xFE
//
// Top     Green    0111 1111   0x7F
//         Green    1101 1111   0xDF
//         Green    1111 0111   0xF7
// Bottom  Green    1111 1101   0xFD
//
const char ledArray[] = {0xFE, 0xFB, 0xEF, 0xBF, 0xFD, 0xF7, 0xDF, 0x7F};
const char defaultArrayMapping[8] = {0,1,2,3,4,5,6,7};              // map output 0 to 0, 1 to 1, etc...
unsigned char arrayMapping [8];
unsigned char displayIndex = 0;                                     // 0-7 shows current connection mapping on LEDs

unsigned char rxBuffer[20];
unsigned char rxPointer = 0;                                        // index into array rxBuffer

unsigned char txBuffer[20];
unsigned char txPointer = 0;                                        // index into an array txBuffer

unsigned char networkAddressTens;
unsigned char networkAddressOnes;
unsigned int mSecondCounter = 0;

unsigned int mSecondCounterSerialTimeOut = 0;
unsigned char serialTransmitFlag = 0;                               // when set to 1, a string is ready to send


int main(void)
{
    unsigned int mSecondCounterDebug = 0;
    unsigned char ledDisplayIndex = 0;    
  
    Setup();                                                        // processor setup
    Configuration();                                                // setup things attached to processor
    SerialSplash();    

    
//    I2CSendDataEEPROMPCA9501(0x00, 0x30, I2C_SLAVE_ADDRESS_LEFT_EEPROM );

    
    txBuffer[0] = ' ';
    txBuffer[1] = ' ';
    txBuffer[2] = ' ';
    txBuffer[3] = ' ';
    txBuffer[4] = NULL;

    txBuffer[0] = I2CReadDataEEPROMPCA9501(0x00, I2C_SLAVE_ADDRESS_LEFT_EEPROM);
    txBuffer[1] = I2CReadDataEEPROMPCA9501(0x01, I2C_SLAVE_ADDRESS_LEFT_EEPROM);
    txBuffer[2] = I2CReadDataEEPROMPCA9501(0x02, I2C_SLAVE_ADDRESS_LEFT_EEPROM);
    txBuffer[3] = I2CReadDataEEPROMPCA9501(0x03, I2C_SLAVE_ADDRESS_LEFT_EEPROM);
    txBuffer[4] = NULL;
    SendString(txBuffer);

    txBuffer[0] = 'D';
    txBuffer[1] = 'O';
    txBuffer[2] = 'N';
    txBuffer[3] = 'E';
    txBuffer[4] = NULL;
    SendString(txBuffer);
    
    while(1)
    {
        Rs485FlowControl();
        SendSerialString();                                             // checks to see if there are characters to send out uart
        LookForSwitch();                                                // check for switch 1 press

    
        
        
        if (CheckTimerDifference(750, mSecondCounterDebug))             // look for a difference of 250mS from last stored value
        {
            mSecondCounterDebug  = mSecondCounter;
            
            GPIOPinWrite(PORTC, PIN_LED, ~GPIOPinRead(PORTC, PIN_LED)); 

            UpdateLED (ledDisplayIndex, arrayMapping[ledDisplayIndex]);

            ledDisplayIndex++;
            if (ledDisplayIndex > 7) ledDisplayIndex = 0;               // all eight inputs displayed, start over
        }
     
       
        if (UARTCharsAvail(UART0_BASE))
        {
            rxBuffer[rxPointer] = UARTCharGet(UART0_BASE);
            ProcessSerialInput();
            mSecondCounterSerialTimeOut = mSecondCounter;               // reset the timeout count
        }
       
        CheckForSerialTimeOut();
    }
}
// ****************************************************************************
// ***************************** ResetDefaults ********************************
// ****************************************************************************
//
void ResetDefaults(void)
{
        SerialTransmitEnable(1,"Reset Defaults");
        InitMapping();
        InitAddress ();        
  
}
// ****************************************************************************
// ***************************** ResetDefaults ********************************
// ****************************************************************************
//
void LookForSwitch(void)
{
    unsigned int switchCounter;
    unsigned char secondCount = 0;
    
    if (GPIOPinRead(PORTC, PIN_SWITCH1))
    {   
        while (GPIOPinRead(PORTC, PIN_SWITCH1))                     // hold here while switch is pressed
        {
            if (CheckTimerDifference(999, switchCounter))           // look for a difference of 1 second from last stored value
            {
                switchCounter  = mSecondCounter;
                secondCount++;
            }          
          
        }
        // check to see how long the switch was held
        if (secondCount > 5)
        {
                ResetDefaults();
        }
        else
        {
            SerialTransmitEnable(1,"Debug Switch");
/*
            displayIndex++;
            if (displayIndex > 7)
            {
                displayIndex  = 0;
            }
            UpdateLED(displayIndex, arrayMapping[displayIndex]);
*/          
          
        }
    }
}
// ****************************************************************************
// ************************** Rs485FlowControl ********************************
// ****************************************************************************
//
// This function check for busy bit on UART transmitter.  If BIT3 is high
// the UART is busy transmitting.  If so send enable RS485 output, set bit high.
//
void Rs485FlowControl(void)
{
    if ((0x0008 & HWREG(UART0_BASE + UARTFR)))                             //  00000000 00000000 00000000 0000B000, B = Busy
    {
        GPIOPinWrite( PORT_LCD_CONTROL, RS485_TX_EN, RS485_TX_EN);          // RS485 direction control, TX_EN High Transmitting
    }
    else
    {
        GPIOPinWrite( PORT_LCD_CONTROL, RS485_TX_EN, 0);                    // TX_EN Low Not Transmitting  
    }
}
// ****************************************************************************
// ************************ SerialTransmitEnable ******************************
// ****************************************************************************
//
// This function triggers a write of an array to the serial port.  
// If "copy" is non zero, the arrayPointer is used to copy string to tx buffer.
// If "copy" is zero, exisiting tx buffer contents are used.
//
void SerialTransmitEnable(unsigned char copy, unsigned char *arrayPointer)
{
    if (copy)                               // if set copy contents pointed to by received pointer to tx buffer
    {
        StringCopy(arrayPointer); 
    }
    serialTransmitFlag = 1;                 // set a flag to tell main loop there is characters to send
}
// ****************************************************************************
// ************************** SendSerialString ********************************
// ****************************************************************************
// 
// This function will send an array of characters to the serial port.  The array of characters
// must be NULL terminated.  This function is triggered when the flag "serialTransmitFlag"
// is set. If the FIFO is full the function will terminate and must be called again.  This
// function uses the following global variables to track the how many characters have been sent
// and the status.
// 
// serialTransmitFlag       This variable set to one triggers the writting of the data
// txPointer            Index through the array
//
void SendSerialString(void)
{
    unsigned char exitFlag = 0;

    if (serialTransmitFlag == 1)                                        // check to see if we should be sending data
    {
        while (exitFlag == 0)                                           // stuff bytes out till told to stop
        {
            if (txBuffer[txPointer] != 0x00)                        // check for null termination (end of string)
            {
                if(UARTSpaceAvail(UART0_BASE))
                {
                    UARTCharPutNonBlocking(UART0_BASE,txBuffer[txPointer]); 
                    txPointer++;
                }
                else
                {
                    exitFlag = 1;                                       // FIFO is full exit out of this function
                }
            }
            else                                                        // all data has been transmitted
            {
                GPIOPinWrite( PORT_LCD_CONTROL, RS485_TX_EN, 0);        // RS485 direction control, TX_EN High Transmitting
                txPointer = 0;                                      // reset pointer that works through the array
                serialTransmitFlag = 0;                                 // set flag that the array has been sent
                exitFlag = 1;                                           // all bytes sent, exit out of this function
            }
        }
    }
}
void StringCopy(unsigned char *rec)
{
    unsigned char x = 0;
    while(*rec != NULL)
    {
        txBuffer[x] = *rec++; 
        x++;
    }
    txBuffer[x] = NULL;
}
// ****************************************************************************
// **************************** UARTIntHandler ********************************
// ****************************************************************************

void UARTIntHandler(void)
{
    unsigned long ulStatus;  

    ulStatus = UARTIntStatus(UART0_BASE, true);         // Get the interrrupt status
    UARTIntClear(UART0_BASE, ulStatus);                 // Clear the asserted interrupts.
    
           
/*    
  unsigned long ulStatus;

    ulStatus = UARTIntStatus(UART0_BASE, true);         // Get the interrrupt status.
    UARTIntClear(UART0_BASE, ulStatus);                 // Clear the asserted interrupts.


    while(UARTCharsAvail(UART0_BASE))
    {
        // Read the next character from the UART and write it back to the UART.
        UARTCharPutNonBlocking(UART0_BASE, (UARTCharGetNonBlocking(UART0_BASE))+1);
//        rxBuffer[rxPointer] = UARTCharGetNonBlocking(UART0_BASE);
    }
    ProcessSerialInput();
*/
}

//********************************************************
//********************** SendString **********************
//********************************************************
void SendString(unsigned char *rec)
{
	while(*rec != '\0')
	{
            UARTCharPut(UART0_BASE,*rec++);
	}
	UARTCharPut(UART0_BASE,0x0d);
	UARTCharPut(UART0_BASE,0x0a);
}
// ****************************************************************************
// *********************** CheckForCrossPointMapping **************************
// ****************************************************************************
//
void CheckForCrossPointMapping(void)
{
unsigned localVar;

    localVar = I2CReadDataEEPROMPCA9501(EEPROM_ADDRESS_CROSSPOINT_FLAG, I2C_SLAVE_ADDRESS_EEPROM);   // read first location 0x10 of EEPROM, 0xA5 if crosspoint initialized   
    if (localVar == 0xA5)                                           // initialized
    {
        for(localVar=0; localVar<8; localVar++)
        {
            arrayMapping[localVar] = I2CReadDataEEPROMPCA9501(localVar + EEPROM_ADDRESS_CROSSPOINT_DATA, I2C_SLAVE_ADDRESS_EEPROM); // EEPROM locations 0x11,12,13,14,15,16,17,18 contain mappings    
        }
    }
    else                                                            // not initialized
    {
        InitMapping();
        SendString("DEBUG - No Map");
    }
    for(localVar=0; localVar<8;localVar++)
    {
        ControlVideoMux(localVar, arrayMapping[localVar]);
    }
}
void InitMapping(void)
{
    unsigned char localVar;
    
         // load EEPROM with default crosspoint mapping
        for(localVar=0; localVar<8; localVar++)
        {
            I2CSendDataEEPROMPCA9501(localVar + EEPROM_ADDRESS_CROSSPOINT_DATA, defaultArrayMapping[localVar], I2C_SLAVE_ADDRESS_EEPROM);  //address, data, I2C address
            arrayMapping[localVar] = defaultArrayMapping[localVar];     
        }
        I2CSendDataEEPROMPCA9501(EEPROM_ADDRESS_CROSSPOINT_FLAG, 0xA5, I2C_SLAVE_ADDRESS_EEPROM);   // mark eeprom as initialized
}
//********************************************************
//******************* VersionViaSerial *******************
//********************************************************
void SerialSplash(void)
{
	SendString("AV Widgets");

 	// send model
	
	txBuffer[0] = 'M';
	txBuffer[1] = 'o';
	txBuffer[2] = 'd';
	txBuffer[3] = 'e';
	txBuffer[4] = 'l';
	txBuffer[5] = ':';
	txBuffer[6] = MODEL1;
	txBuffer[7] = MODEL2;
	txBuffer[8] = MODEL3;
	txBuffer[9] = MODEL4;
	txBuffer[10] = MODEL5;
	txBuffer[11] = 0x00;
	SendString(txBuffer);

	// send version

 	txBuffer[0] = 'V';
	txBuffer[1] = 'e';
	txBuffer[2] = 'r';
	txBuffer[3] = ':';
	txBuffer[4] = VERSION_MAJOR;
	txBuffer[5] = '.';
	txBuffer[6] = VERSION_MINOR;
	txBuffer[7] = VERSION_ALPHA;
	txBuffer[8] = 0x00;
	SendString(txBuffer);
}
// ****************************************************************************
// ******************* I2CReadDataEEPROMCurrentAddress ************************
// ****************************************************************************
//
unsigned char I2CReadDataEEPROMCurrentAddress(unsigned char recI2CSlaveAddress )
{	
    unsigned char returnVar;
    

        I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, recI2CSlaveAddress, true);
	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

	while (I2CMasterBusy(I2C1_MASTER_BASE));

        returnVar = I2CMasterDataGet(I2C1_MASTER_BASE);
        
        return returnVar;        
}
// ****************************************************************************
// ************************ I2CReadDataEEPROM *********************************
// ****************************************************************************
//
//#define I2C_MASTER_CMD_SINGLE_SEND              0x00000007
//#define I2C_MASTER_CMD_SINGLE_RECEIVE           0x00000007
//#define I2C_MASTER_CMD_BURST_SEND_START         0x00000003
//#define I2C_MASTER_CMD_BURST_SEND_CONT          0x00000001
//#define I2C_MASTER_CMD_BURST_SEND_FINISH        0x00000005
//#define I2C_MASTER_CMD_BURST_SEND_ERROR_STOP    0x00000004
//#define I2C_MASTER_CMD_BURST_RECEIVE_START      0x0000000b
//#define I2C_MASTER_CMD_BURST_RECEIVE_CONT       0x00000009
//#define I2C_MASTER_CMD_BURST_RECEIVE_FINISH     0x00000005
//#define I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP 0x00000005
//
unsigned char I2CReadDataEEPROM(unsigned char recI2CAddress, unsigned char recI2CSlaveAddress )
{	
    unsigned char returnVar;
    
	while (I2CMasterBusy(I2C1_MASTER_BASE));

        // send control byte and address
        I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, recI2CSlaveAddress, false);
	I2CMasterDataPut(I2C1_MASTER_BASE, recI2CAddress);
	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);

	while (I2CMasterBusy(I2C1_MASTER_BASE));
        
	//send control byte again with read bit set
        I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, recI2CSlaveAddress, true);

	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

	while (I2CMasterBusy(I2C1_MASTER_BASE));

        returnVar = I2CMasterDataGet(I2C1_MASTER_BASE);
        
        return returnVar;        
}
// ****************************************************************************
// ********************* I2CReadDataEEPROMPCA9501 *****************************
// ****************************************************************************
// 
// This function is a I2C routine to read the EEPROM of the NXP PCA9501.  The PCA9501 is 
// also an I/O expander.  It has 128 bytes of EEPROM.  The EEPROM read is goofy in that it requires
// another start condition.  It also does not want a 9 bit ack read at the end of the read.  If 
// 9th read occurs, it hangs the I2C bus.  Therefor a second dummy read is required.
// 
// [Start] [Slave Address(W)] [Ack] [Memory Address] [Ack] [Start] [Slave Address(R)][Ack] [Data][?][Data] [Stop]
// [?] Do not do an ack read.
//
//#define I2C_MASTER_CMD_SINGLE_SEND              0x00000007
//#define I2C_MASTER_CMD_SINGLE_RECEIVE           0x00000007
//#define I2C_MASTER_CMD_BURST_SEND_START         0x00000003
//#define I2C_MASTER_CMD_BURST_SEND_CONT          0x00000001
//#define I2C_MASTER_CMD_BURST_SEND_FINISH        0x00000005
//#define I2C_MASTER_CMD_BURST_SEND_ERROR_STOP    0x00000004
//#define I2C_MASTER_CMD_BURST_RECEIVE_START      0x0000000b        Nope, repeat but no stop condition
//#define I2C_MASTER_CMD_BURST_RECEIVE_CONT       0x00000009
//#define I2C_MASTER_CMD_BURST_RECEIVE_FINISH     0x00000005        Nope, does not do the repeat
//#define I2C_MASTER_CMD_BURST_RECEIVE_ERROR_STOP 0x00000005
//
unsigned char I2CReadDataEEPROMPCA9501(unsigned char recI2CAddress, unsigned char recI2CSlaveAddress )
{	
    unsigned char returnVar;
    
	while (I2CMasterBusy(I2C1_MASTER_BASE));

        // send control byte and address
        I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, recI2CSlaveAddress, false);     // load physical address of PCA9501, false=write
	I2CMasterDataPut(I2C1_MASTER_BASE, recI2CAddress);                      // load address of EEPROM to read
	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);    // Send data with a "start"

	while (I2CMasterBusy(I2C1_MASTER_BASE));
        
	//send control byte again with read bit set
        I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, recI2CSlaveAddress, true);      // load physical address of PCA9501, true=read
       	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); // send data with another start without a stop

	while (I2CMasterBusy(I2C1_MASTER_BASE));

        returnVar = I2CMasterDataGet(I2C1_MASTER_BASE);                         // read the data

	while (I2CMasterBusy(I2C1_MASTER_BASE));

       	I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);   // setup for another read and setup for a "end" condition
        I2CMasterDataGet(I2C1_MASTER_BASE);                                     // read data and don't store it.

        return returnVar;        
}
// ****************************************************************************
// ************************ I2CSendDataEEPROM *********************************
// ****************************************************************************
//
// This function sends a single byte to an I2C EEPROM location.
// When write is finished, the location will be read till the data is valid.
// A timeout "postTimeOutCount" increments to let the read bailout if an error.
//
void I2CSendDataEEPROM(unsigned char recI2CAddress, unsigned char recI2CData, unsigned char recI2CSlaveAddress )
{	
    unsigned char readVar;
    unsigned char flagReadOk = 0;
    unsigned int postTimeOutCount = 0;     
    
        while (I2CMasterBusy(I2C1_MASTER_BASE));
    
        I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, recI2CSlaveAddress, false);
        I2CMasterDataPut(I2C1_MASTER_BASE, recI2CAddress);
        I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    
        while (I2CMasterBusy(I2C1_MASTER_BASE));
    
        I2CMasterDataPut(I2C1_MASTER_BASE, recI2CData);
        I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
 
        while (flagReadOk == 0)
        {   
            readVar = I2CReadDataEEPROMPCA9501(recI2CAddress, recI2CSlaveAddress); 
            if (readVar == recI2CData)
            {
              flagReadOk = 1;  
            } 
            postTimeOutCount++;
            if(postTimeOutCount>10000)
            {
                flagReadOk = 2;  
            }
        }
}
// ****************************************************************************
// ********************* I2CSendDataEEPROMPCA9501 *****************************
// ****************************************************************************
//
// This function sends a single byte to an I2C EEPROM location.
// When write is finished, the location will be read till the data is valid.
// A timeout "postTimeOutCount" increments to let the read bailout if an error.
//
void I2CSendDataEEPROMPCA9501(unsigned char recI2CAddress, unsigned char recI2CData, unsigned char recI2CSlaveAddress )
{	
    unsigned char readVar;
    unsigned char flagReadOk = 0;
    unsigned int postTimeOutCount = 0;     
    
        while (I2CMasterBusy(I2C1_MASTER_BASE));
    
        I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, recI2CSlaveAddress, false);     // load physical address of PCA9501, false=write
        I2CMasterDataPut(I2C1_MASTER_BASE, recI2CAddress);                      // write the physical address
        I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    
        while (I2CMasterBusy(I2C1_MASTER_BASE));

        I2CMasterDataPut(I2C1_MASTER_BASE, recI2CData);
        I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

        while (flagReadOk == 0)
        {   
            readVar = I2CReadDataEEPROMPCA9501(recI2CAddress, recI2CSlaveAddress); 
            if (readVar == recI2CData)
            {
              flagReadOk = 1;  
            } 
            postTimeOutCount++;
            if(postTimeOutCount>10000)
            {
                flagReadOk = 2;  
            }
        }
}
//*******************************************************************
//********************** CheckForSerialTimeOut **********************
//*******************************************************************
//
// This function checks to see if a serial input string has partially been received.
// If it is partial and a timeout has been reached, the buffer is cleared and string count reset.
//
void CheckForSerialTimeOut(void)
{
    unsigned char localFlushCnt;

    if(rxPointer != 0)				                    // check to see if we are in middle of command 										
    {
        if (CheckTimerDifference(250, mSecondCounterSerialTimeOut))         // look for a difference of 250mS from last stored value
        {
            SendString("CMD ERROR 7 TIMEOUT");                              //debug
            mSecondCounterSerialTimeOut = mSecondCounter;                   // reset the timeout count
            rxPointer = 0;					    // start string index back to zero
            for(localFlushCnt=0; localFlushCnt<9; localFlushCnt++)
            {
                    rxBuffer[localFlushCnt] = 0x00;
            }	
        }
    }
 }

//*******************************************************************
//*********************** ProcessSerialInput ************************
//*******************************************************************
//
// Serial command is 10 bytes long.
// Each command string has a preface of AVW followed by two bytes of address
// 
// AVW01xxxxE
// AVW01SxDxE		Local, Select input number 1
// -----Sx---           Source, x denotes source 1 though 8
// -------Dx-           Destination, x denotes destination 1 through 8
// AVW01S1D1E           Example.  Connect Source 1 with Destination 1
//
// AVW00A##xE		Global, set the address
// AVW00RxxxE		Global, read the address
// AVW01VxxxE		Local, Read firmware version
// AVW01T#xxE		Local, Termination, 1=turn on 75 ohm termination on inputs 5,6,7,8. 0=turns it off
// AVW01PxxxE           Local, Presence detection, returns which inputs have active video attached
//
// Not implemented
//
// AVW01T##xE		Local, Scan switch time, Minimum 01 seconds, Maximum 99 seconds 
// AVW01MMxxE		Local, Scan Mode Manual
// AVW01MAxxE		Local, Scan Mode Auto
//
void ProcessSerialInput(void)
{

    if (rxPointer == 0)
    {
        if (rxBuffer[rxPointer] == 'A')
        {
                rxPointer++;					// string format ok so far, increment string counter
        }
        else
        {
                rxPointer = 0; 					// bad string format
        }
    }
    else if (rxPointer == 1)
    {
        if (rxBuffer[rxPointer] == 'V')
        {
                rxPointer++;					// string format ok so far, increment string counter
        }
        else
        {
                rxPointer = 0;  					// bad string format
        }
    }
    else if (rxPointer == 2)
    {
        if (rxBuffer[rxPointer] == 'W')
        {
                rxPointer++;					// string format ok so far, increment string counter
        }
        else
        {
                rxPointer = 0;  					// bad string format
        }
    }
    else if (rxPointer == 3)					// address tens	  
    {
        if (rxBuffer[rxPointer] < '0' || rxBuffer[rxPointer] > '9') 		// check for valid range, must be ascii 0-9
        {
                rxPointer = 0;  					// bad string format
        }
        else
        {
                rxPointer++;					// string format ok so far, increment string counter
        }
    }
    else if (rxPointer == 4)					// address ones
    {
        if (rxBuffer[rxPointer] < '0' || rxBuffer[rxPointer] > '9') 		// check for valid range, must be ascii 0-9
        {
                rxPointer = 0;  					// bad string format
        }
        else
        {
                rxPointer++;					// string format ok so far, increment string counter
        }
    }
    else if (rxPointer == 5)
    {
            rxPointer++;
    }
    else if (rxPointer == 6)
    {
            rxPointer++;
    }
    else if (rxPointer == 7)
    {
            rxPointer++;
    }
    else if (rxPointer == 8)
    {
            rxPointer++;
    }
    else if (rxPointer == 9)
    {
            if (rxBuffer[9] == 'E')
            {
                    // *** Valid Command ***
                    ProcessSerialCommand();
                    rxPointer = 0;
            }
            else
            {
                    rxPointer = 0;
//                    SendString("CMD ERROR 6 ETX");
                    SerialTransmitEnable(1,"CMD ERROR 6 ETX");
            }
    }
}

//*******************************************************************
//********************* ProcessSerialCommand ************************
//*******************************************************************
//
// List of serial command errors
// CMD ERROR 1 FORMAT           Unregonized command or bad format
// CMD ERROR 2 ADDRESS          Not a valid address or this device not at this address
// CMD ERROR 3 GLOBAL           Invalid command with a global address
// CMD ERROR 4 SRC DEST RANGE   Source or destination is out of range (or not correct)
// CMD ERROR 5 TERM             Invalid command issued with a termination command, must be '0' or '1'
// CMD ERROR 6 ETX              Last byte of 10 byte command should be "E" ETX End of Transmission
// CMD ERROR 7 TIMEOUT          Too much time between bytes on incoming command string
//

void ProcessSerialCommand(void)
{
	if(rxBuffer[3] == '0' && rxBuffer[4] == '0')				// check for global command
	{
            if(rxBuffer[5] == 'A')						// address change command
            {
                if(rxBuffer[6] >= '0' && rxBuffer[6] <= '9' )			// check for valid range, ascii 0 - 9
                {
                    if(rxBuffer[7] >= '0' && rxBuffer[7] <= '9' )		// check for valid range, ascii 0 - 9
                    {
                        networkAddressTens = rxBuffer[6];
                        I2CSendDataEEPROMPCA9501(EEPROM_ADDRESS_NETWORK_ADDRESS_TENS, networkAddressTens, I2C_SLAVE_ADDRESS_EEPROM);  //address, data, I2C address    
                        networkAddressOnes = rxBuffer[7];
                        I2CSendDataEEPROMPCA9501(EEPROM_ADDRESS_NETWORK_ADDRESS_ONES, networkAddressOnes, I2C_SLAVE_ADDRESS_EEPROM);  //address, data, I2C address    
                        I2CSendDataEEPROMPCA9501(EEPROM_ADDRESS_NETWORK_ADDRESS_FLAG, 0xA5, I2C_SLAVE_ADDRESS_EEPROM);                 // flag address as good    
//                        SendString("CMD OK ADDRESS CHANGED");
                        SerialTransmitEnable(1,"CMD OK ADDRESS CHANGED");

                    }
                }
            }
            else if(rxBuffer[5] == 'R')						// read the address command
            {
                txBuffer[0] = 'C';
                txBuffer[1] = 'M';
                txBuffer[2] = 'D';
                txBuffer[3] = ' ';
                txBuffer[4] = 'O';
                txBuffer[5] = 'K';
                txBuffer[6] = ' ';
                txBuffer[7] = 'A';
                txBuffer[8] = 'D';
                txBuffer[9] = 'D';
                txBuffer[10] = ':';
                txBuffer[11] = ' ';
                txBuffer[12] = networkAddressTens;
                txBuffer[13] = networkAddressOnes;
                txBuffer[14] = 0x00;
                SerialTransmitEnable(0,NULL);
            }
            else
            {
                    SerialTransmitEnable(1,"CMD ERROR 3 GLOBAL");
            }
	}
	else if(rxBuffer[3] == networkAddressTens && rxBuffer[4] == networkAddressOnes)
	{
		if(rxBuffer[5] == 'S' && rxBuffer[7] == 'D' )			// check for proper source/destination format
		{  
                    if (rxBuffer[6] > '0' &&  rxBuffer[6] < '9' && rxBuffer[8] > '0' &&  rxBuffer[8] < '9') // must be 1 through 8
                    {
                        ProcessSerialSourceDestinationCommand(rxBuffer[6], rxBuffer[8]);
                        SerialTransmitEnable(1,"CMD OK Source Destination");
                    }
                    else
                    {
                        SerialTransmitEnable(1,"CMD ERROR 4 SRC DEST RANGE");
                    }
		}
		else if(rxBuffer[5] == 'T')					// Termination command
		{
			if(rxBuffer[6] == '1') 					// turn on the 75 ohm termination
			{
                            ControlVideoInputLoad(ON);                          // Turn on 75 ohm termination.
                            I2CSendDataEEPROMPCA9501(EEPROM_ADDRESS_TERMINATION, '1', I2C_SLAVE_ADDRESS_EEPROM);                    
                            SerialTransmitEnable(1,"CMD OK TERMINATION ON");
			}
			else if(rxBuffer[6] == '0')				// turn off termination
			{
                            ControlVideoInputLoad(OFF);                          // Turn off 75 ohm termination.
                            I2CSendDataEEPROMPCA9501(EEPROM_ADDRESS_TERMINATION, '0', I2C_SLAVE_ADDRESS_EEPROM);                    
                            SerialTransmitEnable(1,"CMD OK TERMINATION OFF");
			}
                        else
                        {
                            SerialTransmitEnable(1,"CMD ERROR 5 TERM");
                        }
		}
		else if(rxBuffer[5] == 'V')	                                // report version of firmware
		{
                    txBuffer[0] = 'C';
                    txBuffer[1] = 'M';
                    txBuffer[2] = 'D';
                    txBuffer[3] = ' ';
                    txBuffer[4] = 'O';
                    txBuffer[5] = 'K';
                    txBuffer[6] = ' ';
                    txBuffer[7] = 'V';
                    txBuffer[8] = 'E';
                    txBuffer[9] = 'R';
                    txBuffer[10] = ':';
                    txBuffer[11] = ' ';
                    txBuffer[12] = VERSION_MAJOR;
                    txBuffer[13] = '.';
                    txBuffer[14] = VERSION_MINOR;
                    txBuffer[15] = VERSION_ALPHA;
                    txBuffer[16] = 0x00;
                    SerialTransmitEnable(0,NULL);
		}
                else if (rxBuffer[5] == 'P')
                {
                    CheckForVideoPresence(); 
                }
		else
		{
                    SerialTransmitEnable(1,"CMD ERROR 1 FORMAT");
		}
	}
	else
	{
            SerialTransmitEnable(1,"CMD ERROR 2 ADDRESS");
	}
}
// ****************************************************************************
// ************************ CheckForVideoPresence *****************************
// ****************************************************************************
//
// This function checks for presence of video on channels 1 - 4.  Will report
// back via serial port what channels have video attached.  Scan the port several
// times because the sync line does pulse low, we don't want to accidentally 
// do a scan when sync pulse is low.
//
void CheckForVideoPresence(void)
{
    unsigned char n;
    
        txBuffer[0] = 'C';
        txBuffer[1] = 'M';
        txBuffer[2] = 'D';
        txBuffer[3] = ' ';
        txBuffer[4] = 'O';
        txBuffer[5] = 'K';
        txBuffer[6] = ' ';
        txBuffer[7] = 'V';
        txBuffer[8] = 'I';
        txBuffer[9] = 'D';
        txBuffer[10] = ':';
        txBuffer[11] = ' ';
        txBuffer[12] = ' ';                                         // clear it now.  Will load with a '1' later if signal is present.
        txBuffer[13] = ' ';                                         // ditto
        txBuffer[14] = ' ';                                         // ditto again
        txBuffer[15] = ' ';                                         // ditto again again
        txBuffer[16] = 0x00;
        SerialTransmitEnable(0,NULL);

        for (n=0; n<10; n++)
        {
            if (GPIOPinRead(PORT_E, PIN_HSYNC1)) txBuffer[12] = '1';
            if (GPIOPinRead(PORT_E, PIN_HSYNC2)) txBuffer[13] = '2';
            if (GPIOPinRead(PORT_E, PIN_HSYNC3)) txBuffer[14] = '3';
            if (GPIOPinRead(PORT_E, PIN_HSYNC4)) txBuffer[15] = '4';
        }
}
// ****************************************************************************
// ********************* ReadEepromNetworkAddress *****************************
// ****************************************************************************
//
// This function checks the serial EEPROM for a network address.  If no valid address is
// found a default address is stored.
//
void ReadEepromNetworkAddress(void)
{
unsigned localVar;

    localVar = I2CReadDataEEPROMPCA9501(EEPROM_ADDRESS_NETWORK_ADDRESS_FLAG, I2C_SLAVE_ADDRESS_EEPROM);    // read flag location, network address stored in EEPROM?  
    if (localVar == 0xA5)                                                                           // initialized
    {
        networkAddressTens = I2CReadDataEEPROMPCA9501(EEPROM_ADDRESS_NETWORK_ADDRESS_TENS, I2C_SLAVE_ADDRESS_EEPROM);        
        networkAddressOnes = I2CReadDataEEPROMPCA9501(EEPROM_ADDRESS_NETWORK_ADDRESS_ONES, I2C_SLAVE_ADDRESS_EEPROM);        
    } 
    else        // EEPROM does not have network address stored
    {
        SendString("DEBUG - No Address");
        InitAddress ();    
    }
}
void InitAddress (void)
{
        I2CSendDataEEPROMPCA9501(EEPROM_ADDRESS_NETWORK_ADDRESS_TENS, DEFAULT_NETWORK_ADDRESS_TENS, I2C_SLAVE_ADDRESS_EEPROM);  //address, data, I2C address    
        I2CSendDataEEPROMPCA9501(EEPROM_ADDRESS_NETWORK_ADDRESS_ONES, DEFAULT_NETWORK_ADDRESS_ONES, I2C_SLAVE_ADDRESS_EEPROM);  //address, data, I2C address    
        I2CSendDataEEPROMPCA9501(EEPROM_ADDRESS_NETWORK_ADDRESS_FLAG, 0xA5, I2C_SLAVE_ADDRESS_EEPROM);                          // flag address as good      
}

// ****************************************************************************
// ****************** ProcessSerialSourceDestinationCommand *******************
// ****************************************************************************
// This function will receive a Source and Destination value in ASCII:
//      Source value of 1 - 8
//      Destination value of 1- 8
//
void ProcessSerialSourceDestinationCommand(unsigned char recSource, unsigned char recDestination)
{
    recSource = recSource - 0x30;               // convert from ascii
    recSource = recSource - 1;                  // convert to zero based

    recDestination = recDestination - 0x30;     // convert from ascii
    recDestination = recDestination - 1;        // convert to zero based

    ControlVideoMux(recSource, recDestination); 
  
    arrayMapping[recSource] =  recDestination; 
    I2CSendDataEEPROMPCA9501(recSource + 0x11, recDestination, I2C_SLAVE_ADDRESS_EEPROM);  //address, data, I2C address    
    
}
// ****************************************************************************
// ************************* CheckTimerDifference *****************************
// ****************************************************************************
//
// This function checks the difference between the current system timer which increments
// 1000x a second and a value of the counter at a previous moment in time.  If the
// difference has been exceeded a "1" is returned.
//
// recPrevious = system timer value stored at an earlier time
// recCompare = difference value that is being compared
//
unsigned int CheckTimerDifference (unsigned int recCompare, unsigned int recPrevious)
{
    unsigned char returnVar=0;
    
    if(mSecondCounter < recPrevious)        // a wrap has occured
    {
        if ((mSecondCounter + (1000-recPrevious)) >= recCompare)
        {
          returnVar = 1;
        }
        else
        {
          returnVar = 0;
        }
    }
    else
    {
        if((mSecondCounter - recPrevious) >= recCompare)
        {
          returnVar = 1;
        }
        else
        {
          returnVar = 0;
        }
    }
    return returnVar;
}
// ****************************************************************************
// *************************** SysTickIntHandler ******************************
// ****************************************************************************
//
// This is an interrupt that fires every 1000x second.  This function name is
// added to the vector table in "startup.c"
//
void SysTickIntHandler(void)
{
    mSecondCounter++;
    
    if ( mSecondCounter > 1000)
    {
        mSecondCounter = 0;
    }

}
// ****************************************************************************
// ***************************** SetupVideoMux ********************************
// ****************************************************************************
// 
// MUX_A0	              // outputs are A0, A1, A2     
// MUX_A1	          
// MUX_A2	          
// MUX_RESET_NEG	  
// MUX_CE_NEG                // chip enable, low to allow clock and reset to work
// MUX_CLK_NEG               // falling edge triggered
// MUX_UPDATE_NEG            // control registers update switch array, low updates, not a clocked signal
// DATA_BUS_0	            // inputs are D0, D1, D2
// DATA_BUS_1	
// DATA_BUS_2	
// DATA_BUS_3	            // D3=1 Outputs are enabled
//
void SetupVideoMux (void)
{  
    GPIOPinWrite( PORT_MUX_CONTROL, MUX_RESET_NEG, MUX_RESET_NEG );     // reset is low asserted, high for right now 
    GPIOPinWrite( PORT_MUX_CONTROL, MUX_CE_NEG, MUX_CE_NEG );           // CE is low asserted, high for right now
    GPIOPinWrite( PORT_MUX_CONTROL, MUX_UPDATE_NEG, MUX_UPDATE_NEG );         
    GPIOPinWrite( PORT_MUX_CONTROL, MUX_CLK_NEG, MUX_CLK_NEG );         // clock is high to low pulse, start high 
    GPIOPinWrite( PORT_MUX_CONTROL, MUX_A0, 0);                 // start with A0 low
    GPIOPinWrite( PORT_MUX_CONTROL, MUX_A1, 0);                 // start with A1 low
    GPIOPinWrite( PORT_MUX_CONTROL, MUX_A2, 0);                 // start with A2 low 
    GPIOPinWrite(PORT_DATA_BUS,DATA_BUS_0, 0);                // start with data bus low
    GPIOPinWrite(PORT_DATA_BUS,DATA_BUS_1, 0);
    GPIOPinWrite(PORT_DATA_BUS,DATA_BUS_2, 0);
    GPIOPinWrite(PORT_DATA_BUS,DATA_BUS_3, 0);
}
// ****************************************************************************
// **************************** ControlVideoMux *******************************
// ****************************************************************************
//
// Output:        A0, A1 ,A2    on port PD0..PD2 (PORT_MUX_CONTROL)
// Input:         D0, D1 ,D2    on port PB0..PB2 (PORT_DATA_BUS)
// Enable Output  D3            1=Output Enabled
//
void ControlVideoMux(unsigned char input, unsigned char output)
{

    GPIOPinWrite( PORT_MUX_CONTROL, MUX_UPDATE_NEG, MUX_UPDATE_NEG );       // make sure update high 
    GPIOPinWrite( PORT_MUX_CONTROL, MUX_CE_NEG, 0 );                    // CE = 0, allow clocking 

    GPIOPinWrite( PORT_MUX_CONTROL, MUX_UPDATE_NEG, 0 );                // update low
  
    GPIOPinWrite(PORT_DATA_BUS,0x0F, input | 0x08);                     // allow writes to D3..D0, 0000 1000, make sure D3 is high
    GPIOPinWrite(PORT_MUX_CONTROL,0x07, output);                        // allow writes to A2..A0,

    // clock in the data, this was timed as a 240nS low pulse, video mux specs >100nS
    GPIOPinWrite( PORT_MUX_CONTROL, MUX_CLK_NEG, MUX_CLK_NEG );          // clock high 
    GPIOPinWrite( PORT_MUX_CONTROL, MUX_CLK_NEG, 0 );                   // clock low 
    GPIOPinWrite( PORT_MUX_CONTROL, MUX_CLK_NEG, MUX_CLK_NEG );          // clock high
 
    GPIOPinWrite( PORT_MUX_CONTROL, MUX_UPDATE_NEG, MUX_UPDATE_NEG );   // update back high, this should latch data

    GPIOPinWrite( PORT_MUX_CONTROL, MUX_CE_NEG, MUX_CE_NEG );           // CE = 1, block further clocking 
}
// ****************************************************************************
// ************************* ControlVideoInputLoad ****************************
// ****************************************************************************
//
// Passing a 1 (ON or TRUE) to this function enables an analog switch
// that terminates input with a 75 ohm load.  This is used to turn an 
// audio input into a video input.
//

void ControlVideoInputLoad(unsigned char onOff)
{
    if (onOff == ON)
    {
        GPIOPinWrite(PORT_MUX_CONTROL, VIDEO_LOAD, VIDEO_LOAD);           // 1=switch on, enables 75 ohm load
    }
    else if (onOff == OFF)
    {
        GPIOPinWrite(PORT_MUX_CONTROL, VIDEO_LOAD, 0);           // 0=switch off, disables 75 ohm load
    }
}
// ****************************************************************************
// ****************************** UpdateLED ***********************************
// ****************************************************************************
//
//
//
void UpdateLED (unsigned char inputVar, unsigned char outputVar)
{
    // Show inputs, LEDs on left hand side, red=video. green=audio
    I2CSendDataLED(ledArray[inputVar],I2C_SLAVE_ADDRESS_LEFT_BANK);          

    // Show outputs, LEDs on right hand side, red=video. green=audio
    I2CSendDataLED(ledArray[outputVar],I2C_SLAVE_ADDRESS_RIGHT_BANK);          
}
// ****************************************************************************
// ************************************* Setup ********************************
// ****************************************************************************
//
// Setup ports and peripherals
//
void Setup (void)
{
      // If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.  
      // This is a workaround to allow the PLL to operate reliably.
      if( DEVICE_IS_REVA2 )
      {
              SysCtlLDOSet( SYSCTL_LDO_2_75V );
      }
      
      /* Set the clocking to run from the PLL at 50 MHz */
      SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ );
 
      // Port C setup
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);  
      GPIOPinTypeGPIOOutput(PORTC, PIN_LED | DATA_BUS_7);
      GPIOPinTypeGPIOInput(PORTC, PIN_SWITCH1);             // make the switch input an input
      
      // I2C Setup
      SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
//      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);     
      GPIOPinConfigure(GPIO_PA6_I2C1SCL);                           // it is at PA6
      GPIOPinConfigure(GPIO_PA7_I2C1SDA);                           // it is at PA7
      GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7 | GPIO_PIN_6);     // enable direction and pullups

      I2CMasterInitExpClk(I2C1_MASTER_BASE, SysCtlClockGet(), false);
      I2CMasterEnable(I2C1_MASTER_BASE);

//      I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, I2C_SLAVE_ADDRESS_LEFT_BANK, false);
//      IntEnable(INT_I2C1);
//      I2CMasterIntEnable(I2C1_MASTER_BASE);

      // UART Setup
      // UART0 is on Port0 PA0=RX PA1=TX
      SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);  // enable the UART
      GPIOPinConfigure(GPIO_PA0_U0RX);              
      GPIOPinConfigure(GPIO_PA1_U0TX);
      GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
      UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 19200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
      UARTFIFOEnable(UART0_BASE);
        
      // Enable the UART interrupt.
//        IntEnable(INT_UART0);
//        UARTIntEnable(UART0_BASE, UART_INT_TX);
//        UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
//        UARTTxIntModeSet(UART0_BASE, UART_TXINT_MODE_FIFO);
      
      // Setup PortB
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);  
      GPIOPinTypeGPIOOutput(PORT_DATA_BUS, DATA_BUS_0 | DATA_BUS_1 | DATA_BUS_2 | DATA_BUS_3 | DATA_BUS_4 | DATA_BUS_5 | DATA_BUS_6 );
      
      // Setup for LCD
      GPIOPinTypeGPIOOutput(PORT_LCD_CONTROL, LCD_WR | LCD_A0 | RS485_TX_EN);
      InitDebugLCD();
      ClearDebugLCD(); 
      
      // setup Port D, video mux control
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  
      GPIOPinTypeGPIOOutput(PORT_MUX_CONTROL, 0xFF);                                        // make all of the outputs on this port outputs

      // setup Port E, Video presense detection
      SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);  
      GPIOPinTypeGPIOInput(PORT_E, PIN_HSYNC1 | PIN_HSYNC2 | PIN_HSYNC3 | PIN_HSYNC4);    // make HSYNC connections to processor inputs

      // ********* Setup system tick interrupt ***************  
      
    // 1/50MHz = 20nS, 16777216 * 20nS = 336mS, 16777216 is max value passed to this function
    // set interrupt to fire 1000x second (or 1mS), 1ms/20nS = 50,000
    SysTickPeriodSet(50000);
    

    // Enable the SysTick Interrupt.
    SysTickIntEnable();

    // Enable SysTick.
    SysTickEnable();

      
    // Enable interrupts to the processor.
    IntMasterEnable();
}
// ****************************************************************************
// **************************** Configuration *********************************
// ****************************************************************************
void Configuration(void)
{
    unsigned localVar;
    
    GPIOPinWrite( PORT_LCD_CONTROL, RS485_TX_EN, 0);                    // TX_EN low (not transmitting)

    ReadEepromNetworkAddress();
    SetupVideoMux();                                                    // low level setup of crosspoint IC
    CheckForCrossPointMapping();                                        // check to see if crosspoint values stored in EEPROM
 
    localVar = I2CReadDataEEPROMPCA9501(EEPROM_ADDRESS_TERMINATION, I2C_SLAVE_ADDRESS_EEPROM);        
    if (localVar == '1')
    {
        ControlVideoInputLoad(ON);                                     // Turn on 75 ohm termination.  It is audio
    }
    else if(localVar == '0')
    {
        ControlVideoInputLoad(OFF);  
    }
}
// ****************************************************************************
// ************************ I2CSendDataLED *********************************
// ****************************************************************************

void I2CSendDataLED(unsigned char recI2CData, unsigned char recI2CSlaveAddress )
{	
    while (I2CMasterBusy(I2C1_MASTER_BASE));
    I2CMasterSlaveAddrSet(I2C1_MASTER_BASE, recI2CSlaveAddress, false);
    I2CMasterDataPut(I2C1_MASTER_BASE, recI2CData);
    I2CMasterControl(I2C1_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
}
// ****************************************************************************
// ************************ DelayLCD100us *************************************
// ****************************************************************************
//
// A delay of 100uS
// Pass in:
// 10     = 1mS
// 100    = 10mS
// 1,000  = 100mS
// 10,000 = 1S
//
void DelayLCD100us(unsigned int recTime)
{
  unsigned int localCnt;
  unsigned int innerLoop; 
  for(localCnt = 0; localCnt < recTime; localCnt++)
  {
      for(innerLoop=0; innerLoop<1000; innerLoop++);  
  }
}

// ****************************************************************************
// ************************ LcdWritePulse *************************************
// ****************************************************************************
//
// Data clocked into LCD module on low to high transition.
//
void LcdWritePulse(void)              
{
    GPIOPinWrite( PORT_LCD_CONTROL, LCD_WR, 0);          // LCD WR pin low
    DelayLCD100us(1);    
    GPIOPinWrite( PORT_LCD_CONTROL, LCD_WR, LCD_WR );          // LCD WR pin high
    DelayLCD100us(1);    
    GPIOPinWrite( PORT_LCD_CONTROL, LCD_WR, 0);          // LCD WR pin low
}
// ****************************************************************************
// ************************* InitDebugLCD *************************************
// ****************************************************************************
//
// Setup for the character LCD module.  
// A0 = 0 = commands
// A0 = 1 = display data
//
void InitDebugLCD(void)
{
   unsigned int waitTime = 2500;
   GPIOPinWrite( PORT_LCD_CONTROL, LCD_WR, 0 );            // make sure LCD WR pin starts low

    DataBusOut(0x30);                                           // initialization, must be done 3x
   GPIOPinWrite(PORT_LCD_CONTROL, LCD_A0, 0 );            // A0 = 0 = command select
   LcdWritePulse();
   DelayLCD100us(waitTime);

    DataBusOut(0x30);                                           // 2x
   GPIOPinWrite(PORT_LCD_CONTROL, LCD_A0, 0 );            // A0 = 0 = command select
   LcdWritePulse();
   DelayLCD100us(waitTime);

    DataBusOut(0x30);                                           // 3x
   GPIOPinWrite(PORT_LCD_CONTROL, LCD_A0, 0 );            // A0 = 0 = command select
   LcdWritePulse();
   DelayLCD100us(waitTime);

    DataBusOut(0x3C);                                           // 8 bit interface, 2 lines
   GPIOPinWrite(PORT_LCD_CONTROL, LCD_A0, 0 );            // A0 = 0 = command select
   LcdWritePulse();
   DelayLCD100us(waitTime);

    DataBusOut(0x01);                                           // clear display, cursor home
   GPIOPinWrite(PORT_LCD_CONTROL, LCD_A0, 0 );            // A0 = 0 = command select
   LcdWritePulse();
   DelayLCD100us(waitTime);

    DataBusOut(0x06);                                           // set cursor increment mode
   GPIOPinWrite(PORT_LCD_CONTROL, LCD_A0, 0 );            // A0 = 0 = command select
   LcdWritePulse();
   DelayLCD100us(waitTime);

    DataBusOut(0x0C);                                           // display ON, cursor OFF, blink OFF
   GPIOPinWrite(PORT_LCD_CONTROL, LCD_A0, 0 );            // A0 = 0 = command select
   LcdWritePulse();
   DelayLCD100us(waitTime);
}
// ****************************************************************************
// ************************** DataBusOut **************************************
// ****************************************************************************
//
// This function puts spreads a byte across two different ports. D0-D6 are put on
// PORT_DATA_BUS which happens to be port B.  Put D7 on PC.4.  This was done because
// PB.7 is a JTAG line.
//
void DataBusOut (unsigned char recChar)
{
    GPIOPinWrite(PORT_DATA_BUS,0x7F, recChar);                                      // 0111 1111, write to whole port (except pin 7)
    GPIOPinWrite(PORTC,DATA_BUS_7, ((recChar>>3) & DATA_BUS_7));           // D7 is connected to PC.4 7 to 4 
}
//-------------------------------------------------------------------------
void DebugPutChar(unsigned char recCharacter)
{
   if( recCharacter > 0x1F )                                          // only allow characters to the LCD (no control codes)
   {
        DataBusOut(recCharacter);
      GPIOPinWrite(PORT_LCD_CONTROL, LCD_A0, LCD_A0 );          // A0 = 1 = data write
      LcdWritePulse();
      DelayLCD100us(1);
   }
}

//-------------------------------------------------------------------------
void ClearDebugLCDLine(unsigned char line)                 // clears a single line on the debug LCD
{
   DebugLCDWrite(line, 0, "                    ");
}
void ClearDebugLCD(void)                                   // clears both lines on the debug LCD
{
   ClearDebugLCDLine(LINE_ONE);
   ClearDebugLCDLine(LINE_TWO);
}

// ****************************************************************************
// ************************* DebugLCDWrite ************************************
// ****************************************************************************
// 
// Writes a string to the character display.  A string is written till a 0x00 is found.
// lineNum = LINE_ONE 0x80 or LINE_TWO 0xC0 which is start of the line position in memory
// pos = an offset into the position on the LCD
// str = pointer to start of string
//
void DebugLCDWrite(unsigned char lineNum, unsigned char pos,  char *str)
{
   DataBusOut(lineNum + pos);                                   // form and put on bus starting poing in memory
   GPIOPinWrite(PORT_LCD_CONTROL, LCD_A0, 0 );                  // A0 = 0 = command select
   LcdWritePulse();                                             // clock it in

   while( *str != '\0' )
   {
      DebugPutChar(*str++);
   }
   DebugPutChar(0x0a);                                     // send out a line feed for serial port use     
   DebugPutChar(0x0d);                                     // send out a carriage return for serial port use     
}

// *********************************************************************************
