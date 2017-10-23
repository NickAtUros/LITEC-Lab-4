/*	Lab 4, car control
	speed gain around 25
	steer gain around 25
	PCA - 16 bit, sysclk/12, CEX0/2
	P0.6  pin14; SDA
	P0.7  pin15; SCL
	P1.0  pin12; CEX0 STEER
	P1.2  pin10; CEX2 SPEED
	P1.7  pin05; ADC battery
	P3.6  pin31; switch
*/
#include <c8051_SDCC.h>
#include <stdio.h>
#include <stdlib.h>
#include <i2c.h>
//====================Function Prototypes====================
void XBR0_Init(void);
void Port_Init(void);
void PCA_Init(void);
void PCA_ISR(void) __interrupt 9;
void SMB_Init(void);
void ADC_Init(void);
void Servo_Init(void);
void Speed_gain(void);
void Speed_Servo(void);
void Steer_gain(void);
void Steer_Servo(void);
signed int error1(void);
unsigned int ReadCompass(void);
unsigned int ReadRanger(void);
void PingRanger(void);
unsigned char battcheck(void);
void ask_user(void);
//====================Global Variables====================
__sbit __at 0xB6 SS1;			//SlideSwitch   Port 3 Pin 6; Pin 31
__sbit __at 0x97 BATT;			//Battery Volt  Port 1 Pin 7; Pin 5
//---------------parameters---------------
/////////////////
#define PW_PERIOD 28671			//20ms
/////////////////
#define SD_MIN    2028			//1.1ms
#define SD_CENTER 2765 			//1.5ms * 22118400/12
#define SD_MAX    3502			//1.9ms		diff 737
#define SD_LINLOW 30			//lower limit of deadzone - becomes linear
#define SD_LINUPP 50			//upper limit of deadzone - becomes linear
/////////////////
#define HE_MIN    2265			//Min PW
#define HE_CENTER 2765			//Center PW
#define HE_MAX    3165			//max PW	diff 1108
#define HE_LINLOW 3590			//0-10	deadzone
#define HE_LINUPP 10			//0+10	deadzone
/////////////////
//------------------timing-------------------
unsigned char count20 = 0;		//count per 20ms
signed int time = 1;
unsigned char update = 0;		//batt, disp, range, heading/keypad
								//b  3,    2,    1,    0
//------------------ranger-------------------
unsigned int range = 0;			//range reading

unsigned int SD_GAIN = -60;		//gain of range to speed (737 [pw] / 30 [cm])
  signed int SD_LOWER = 0;		//lower bound of input value
  signed int SD_UPPER = 0;		//upper bound of input value
unsigned int SD = 0;			//actual PW value
//------------------heading------------------
unsigned int heading = 0;		//heading reading
unsigned int desired_heading = 1800;
unsigned char HE_GAIN = 25;		//gain of range to steering (1108 [pw] / 900-10 [.1deg]) *20 for resolution
  signed int HE_LOWER = 0;		//lower bound of input value
  signed int HE_UPPER = 0;		//upper bound of input value
unsigned int HE = 0;			//actual PW value
  signed int error;
//-------------miscellaneous---------------
  signed char keypad = 0;		//keypad input
//====================Main Function====================
void main(void) {
	unsigned int voltage = 0;		//voltage in millivolts
	Sys_Init();
	putchar(' '); //the quotes in this line may not format correctly
	Port_Init();
	XBR0_Init();
	SMB_Init();
	PCA_Init();
	ADC_Init();
	Servo_Init();	//Speed servo, 1.5ms pw for 1 second
	Speed_gain();
//	Steer_gain();
	printf("\r\nheading\terror\trange\theading pulsewidth\ttime", heading, error1(), range, HE, time);
	while(1) {
		//=================Routine Timed Jobs===================
		if (update & 0x08) {	//battery
			update &= ~0x08;	//set back to 0
			voltage = 58*battcheck();	// 15000/256 = 58.59375
		}
		if (update & 0x04) {	//display
			update &= ~0x04;	//set back to 0
			lcd_clear();
			lcd_print("Battery %dmV\n", voltage);
			lcd_print("Range %dcm\n", range);
			lcd_print("Heading %d\n", heading);
			lcd_print("Desired heading %d\n", desired_heading);
		}
		if (update & 0x02) {	//range
			update &= ~0x02;	//set back to 0
			range = ReadRanger();
			PingRanger();
		}
		if (update & 0x01) {	//compass, keypad
			update &= ~0x01;	//set back to 0
			heading = ReadCompass();
			keypad = read_keypad();
			error = error1();
		}
		// printf("update is %x\r\n", update);
		//===================Main Code=====================
		if (SS1) {	//slideswitch is on
			error = desired_heading - heading;
				Steer_Servo();
				Speed_Servo();
		}
		else {		//slideswitch is off
			if (keypad != -1)	//something pressed - update stuffs
				ask_user();
			HE = HE_CENTER;
			SD = SD_CENTER;
		}
		if ((count20 & 0x03) == 0)
		{
			printf("\r\n%d\t%d\t%d\t%d\t%d", heading, error1(), range, HE, time);
			time++;
		}
		//------steering------
		PCA0CPL0 = 0xFFFF - HE;
		PCA0CPH0 = (0xFFFF - HE) >> 8;
		//------speeding------
		PCA0CPL2 = 0xFFFF - SD;
		PCA0CPH2 = (0xFFFF - SD) >> 8;
	} //end infinite loooop
}
//====================Subroutines====================
//-----------------Servo Functions----------------
void Speed_gain() {	//recalculate lower and upper.
	SD_LOWER = SD_LINLOW - (SD_CENTER - SD_MIN)/SD_GAIN;
	SD_UPPER = SD_LINUPP + (SD_MAX - SD_CENTER)/SD_GAIN;
}
void Speed_Servo() {	//input range, output SD

	if (range >= 25 && range <= 30)	//between 25cm and 30cm - neutral
		{
			SD = SD_CENTER;		//set PW to center value
			//HE = HE - ((30-SD ) / 10) * SD_GAIN * 900;
			HE = HE_MIN;
		}
	else if (range <= 60)		//between 0 and 25cm and 30 and 60cm turn left
		{
		SD = SD_MIN;
		HE = HE_MIN;
		//if(HE < HE_MIN)
			//HE = HE_MIN;
		}
	else							//greater than 60cm - full forward
		SD = SD_MIN;
		HE = HE;		//set PW to min value
}

void Steer_Servo() {	//input error, output HE

	HE = ((error * HE_GAIN)*.1) + HE_CENTER;
		if (HE > HE_MAX)
			HE = HE_MAX;

		if (HE < HE_MIN)
			HE = HE_MIN;
}
signed int error1(void) {	//calculate the error
	signed int error = 0;

	error = desired_heading - heading;
	if (error <= -1800)
		error = error + 3600;
	if (error >=  1800)
		error = error - 3600;
	return error;
}
//------------------i2c interface-------------------
unsigned int ReadCompass(void) {
	unsigned char Data[2]; // Data is an array with a length of 2
	i2c_read_data(0xC0, 2, Data, 2); // read two byte, starting at reg 2

	return (((unsigned int)Data[0] << 8) | Data[1]);	//degrees between 0 and 3599
}
unsigned int ReadRanger(void) {
	unsigned char Data[2];		//create array size 2
	i2c_read_data(0xE0, 2, Data, 2);	//ranger addr, start reg 2, read 2 bytes
	return (((unsigned int)Data[0] << 8) | Data[1]);	//extract 16 bit value
}
void PingRanger(void) {
	unsigned char Data[1];	//write 0x51 to reg 0 of ranger
	Data[0] = 0x51;			//ping command
	i2c_write_data(0xE0, 0, Data, 1);//ranger addr, start reg 0, write 1 byte
}
//----------------adc init--------------------
void ADC_Init(void) {
	REF0CN = 0x03; 		//Set Vref to use interal reference voltage (2.4 V)
	ADC1CN = 0x80; 		//Enable A/D converter A/D
	ADC1CF |= 0x01; 	//Set A/D converter gain to 1
}

//----------------i2c inits--------------------
void SMB_Init(void) {
	SMB0CR = 0x93;	//set scl to 100kHz
	ENSMB = 1;		//enable SMBus
}
//---------------Port_Init-----------------------------------
void Port_Init(){		// Set up ports for input and output
	P0MDOUT &=~0xC0;	//1100 0000 P0.6,7 for I2C
	P0 	    |= 0xC0;	//open drain

	P1MDOUT |= 0x04;	//0000 0100 P1.2; CEX2 in push-pull mode
	P1MDOUT |= 0x01;	//0000 0001 P1.0; CEX0 in push-pull mode

	P3MDOUT &=~0x40;	//0100 0000 P3.6; switch
	P3		|= 0x40;	//open drain
}
//----------------XBR0_Init----------------------------------
void XBR0_Init() {	// Set up the crossbar
	XBR0 = 0x27;	//URART0, SPI, SMB, CEX 0-3
}
//---------------PCA_Init--------------------------------------------------
void PCA_Init(void){// Set up Programmable Counter Array
	PCA0MD = 0x81;	//sysclk/12, enable cf interrupt, suspend while idle
	PCA0CN = 0x40;	//enable pca counter
	PCA0CPM0 = 0xC2;//CEX0; 16 bit, enable comparator, enable pwm
	PCA0CPM2 = 0xC2;//CEX2; 16 bit, enable comparator, enable pwm

	EIE1 |= 0x08;	//enable pca interrupts
	EA = 1;			//enable all interrupts
}
//-----------PCA_ISR----------------------------------------------------------
// Interrupt Service Routine for Programmable Counter Array Overflow Interrupt
void PCA_ISR (void) __interrupt 9 {	//set period
	if (CF) {	//if pca counter overflow
		CF = 0;
		PCA0L = PW_PERIOD & 0xFF;	//low byte of start count
		PCA0H = PW_PERIOD>>8;		//high byte of start count
		//-------every 20ms--------(servo controllers)
		count20++;
		//-------every 40ms--------(compass, keypad)
		if ((count20 & 0x01) == 0)	//____ ___0 every 2 20ms
			update |= 0x01;		//0001
		//-------every 80ms--------(ranger)
		if ((count20 & 0x03) == 0)	//____ __00 every 4 20ms
			update |= 0x02;		//0010
		//-------every 320ms--------(LCD)
		if ((count20 & 0x37) == 0)	//____ 0000 every 55 20ms
			update |= 0x04;		//0100

		//-------every 1280ms--------(battery voltage)
		if ((count20 & 0x3F) == 0)	//__00 0000 every 64 20ms
			update |= 0x08;		//1000
	}
	PCA0CN &= 0xC0;	//reset cex0-4 flags
}
void Servo_Init (void) {
	PCA0CPL2 = 0xFFFF - SD_CENTER;
	PCA0CPH2 = (0xFFFF - SD_CENTER) >> 8;
	while ((update & 0x08) == 0);	//until ~1sec passed
	update &=~0x08;	//reset flag
}
unsigned char battcheck(void) {
	AMX1SL = 7;		//set pin 7 as input
	ADC1CN = ADC1CN & ~0x20;	//clear completed flag
	ADC1CN = ADC1CN | 0x10;		//initiate a/d conversion
	while ((ADC1CN & 0x20) == 0x00);	//wait for completion
	return ADC1;
}
void ask_user(void) {	//update steering gain and heading
	lcd_clear();
	lcd_print("Curr heading %d", heading);
	lcd_print("\nSteering gain:");
	HE_GAIN = kpd_input(1);
	lcd_print("\nDesired heading:");
	desired_heading = kpd_input(1);
	Speed_gain();
	//Steer_gain();
}
