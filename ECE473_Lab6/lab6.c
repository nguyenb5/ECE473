// Lab4
// Expected Connections:
// Bargraph board           Mega128 board 
// --------------      ----------------------    
//     reglck            PORTD bit 2                     
//     srclk             PORTB bit 1 (sclk)
//     sdin              PORTB bit 2 (mosi)
//     oe_n                   ground
//     gnd2                   ground
//     vdd2                     vcc
//     sd_out               no connect
 
/* Encoder board */
// 	SH/LD		PORTE bit 6
// 	CLK_INH	PORTE bit 4
//	SCK		PORTB bit 1 (sclk)
//	SER_OUT	PORTB bit 3 (miso)

/* ADC CdS cell */
// 	Photocell 	PORTF bit 7

/* AudioBoard */
// 	Audio_from_atmega		PORTD bit 4
// 	Atmega_vol_control		PORTE bit 3

/* Radio Board */
// 	GIO2/INT	PORTE bit 7
//	Reset		PORTE bit 2
//	SCLK		PORTD bit 0 (I2C SCL)
//	SDIO		PORTD bit 1 (I2C SDA)

/* Temperature Sensor LM73 */
// 	CLK		PORTD bit 0 (I2C SCL)
//	DAT		PORTD bit 1 (I2C SDA)
 

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "hd44780.h"
#include "twi_master.h"
#include "lm73_functions.h"
#include "si4734.h"

#define TRUE 1
#define FALSE 0

#define RINGING_TIME 60
#define SNOOZE_TIME 10 

#define JUST_TONE 	0
#define JUST_RADIO 1

uint8_t indexToLed[11] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80,
			0x98, 0xFF};
uint8_t brightness[10] = {0xD0, 0xD0, 0xB5, 0xA1, 0x8D, 0x79, 0x65, 0x51, 0x3D, 0x15};

/** Global Vars **/
volatile uint8_t newEncodeData;
volatile uint8_t toBarGraph = 1;
volatile uint32_t current_second; 
volatile uint32_t alarm_display_second;
volatile uint32_t alarm_time;
volatile bool clk_mode = 1;		//default 12h style
volatile uint8_t stateEncoder1;
volatile uint8_t stateEncoder2;
volatile uint8_t dot;
volatile uint16_t lastADCread = 217; 
bool button_flag[2] = {0};
uint8_t state;
bool isAlarmSet;
bool alarm_mode = 0;					//default tone for alarm
bool isAM;
bool isRinging = 0;
bool isSnoozing = 0;

char lcd_final[32];

/* Radio Global Vars*/
bool isRadioChange = 0;			//default to no change
bool applyRadioChange = 0;		//default to do nothing 
bool isRadioOn;
extern enum radio_band{FM, AM, SW};
extern volatile uint8_t STC_interrupt;

volatile enum radio_band current_radio_band = FM;

uint16_t eeprom_fm_freq;
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t  eeprom_volume;

uint16_t desired_fm_freq = 10630;
uint16_t current_fm_freq = 10630;
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t  current_volume = 50;		//Default 50% volume

/* Temperature Sensor TWI variable*/
extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];
uint16_t lm73_temp;  //a place to assemble the temperature from the lm73
char temperature_string[4];

volatile uint8_t inc_dec;
int i;	//dummy var

/* LCD variables*/
char     lcd_str_h[16];  //holds string to send to lcd  
char     lcd_str_l[16];  //holds string to send to lcd 
div_t    fp_adc_result, fp_low_result;  //double fp_adc_result; 
/** End Global Vars **/

/* Initialization */
void spi_init(void);
void clock_config_init(void);
void IO_config_init(void);

/* ADC stuff */
void ADC_init(void);
uint16_t CdS_read(void);
void setBrightness(uint8_t value);

/* Encoder stuff */
int dirOfEncoder(void);
void updateSPI(void);
void bargraphLatch(void);

/* LED stuff */
void processLEDbrightness(uint16_t ADC_value);
void update7Segment(uint16_t number, uint8_t dot);
uint16_t second_to_min_hour(volatile uint32_t *second, bool mode);

/* Button stuff */
uint8_t isButtonPressed(uint8_t button);
void processButton(void);

/* Audio stuff */
void checkAlarm(void);
void SET_VOLUME(uint8_t percent);
void inline TONE_ON(void)	{TIMSK |= (1<<OCIE1A);}				//Allow Atmega to enable tone generator
void inline TONE_OFF(void)	{TIMSK &= ~(1UL << OCIE1A);}	//Disable Atmega tone generator TIMSK  |= (1<<OCIE1A)
void snoozing(void);

/* Temperature Sensor stuff */
void getTemperature(void);
void processTemperature(void);

/* Radio stuff */
void radio_init(void);
void radio_reset(void);
void inline RADIO_ON(void){set_property(0x4000, 0x003F); isRadioOn = 1;}	//Max radio volume output is 3F, min 0
void inline RADIO_OFF(void){set_property(0x4000, 0x0000); isRadioOn = 0;}	//This shut of the output of the radio within the IC
void processRadioTune(void);


void spi_init(void){
	/* Run this code before attempting to write to the LCD.*/
 DDRF  |= 0x08;  //port F bit 3 is enabling for LCD
 PORTF &= 0xF7;  //port F bit 3 is initially low

 DDRB  |= 0x07;  //Turn on SS_n, MOSI, SCLK. SS_n must be out for MSTR mode  

 //Master mode, Clock=clk/4, Cycle half phase, MSB first
 SPCR=(1<<SPE) | (1<<MSTR); //enable SPI, clk low initially, rising edge sample
 SPSR=(1<<SPI2X);           //SPI at 2x speed (8 MHz)  
}

/***********************************************************************/
//			clock_config_init
//	Set up Timer0, Timer1, Timer2, Timer3
/***********************************************************************/
void clock_config_init(void){
	
	/* Config timer 0 to keep track of seconds*/
	TCCR0 |= (1<<CS00);  	//normal mode, no prescaling
	ASSR  |= (1<<AS0);   	//use ext oscillator
	TIMSK |= (1<<TOIE0); 	//allow interrupts on overflow
	
	/* Config timer 1 for sounds generation*/
	TCCR1A |= 0x00;	//CTC mode, discontected ports
	TCCR1B |= (1<<WGM12) | (1<<CS10); //CTC, no prescaler
	TCCR1C |= 0x00;	//No forced compare
	OCR1A   = 20000;	//Frequency setting
	// Tried CompareOutputA but it will break the LED display
	TIMSK  |= (1<<OCIE1A);	//Enable interrupt
	
	/* Timer2 OC2 PWM for LED display PB7 */
	//Enable fast PWM, non-inverting output mode
  	//64 prescaler (goal is 967Hz)
  	TCCR2 = (1<<WGM21) | (1<<WGM20) | (1<<COM21) | (1<<CS21) | (1<<CS20);
  	//Default PWM value of half brightness
  	//OCR2 set the BOTTOM. Output flip when pass BOTTOM and TOP
  	OCR2 = 0xE1; //Half brightness at start up
	
	/* Config timer 3 for volume control */
	// 9 bits, fast-PWM mode, non-inverting on OC3A (PE3)
	//8 prescaler, frequency is 3.90KHz
  	TCCR3A |= (1<<COM3A1) | (1<<WGM31);
  	TCCR3B |= (1<<WGM32) | (1<<CS31);
  	TCCR3C |= 0x00;	//No forced compare	
  	OCR3A   = 1;		//Audio shutup at the begining
	//Let's go to Thai Chili
	
}

void IO_config_init(void){
	DDRE |= (1<<PE3);	//Volume control pin for audio
	DDRD |= (1<<PD4);	//Audio out for audio
	
	DDRA = 0xFF;		//PortA to all output for 4 digits 7 seg LED
	DDRB |= 0xF0;		//PortB bit4->7 to control 4 digits 7 seg LED
	DDRE |= (1<<PE6) | (1<<PE4);  	//PortE bit6, bit 4 to control Encoder SH/LD. CLK INH!
	DDRD |= 0x04;		//PortD bit2 for barGraph REG_CLK
	
	DDRE |=(1<<PE2) |(1<< PE7);		//PortE bit 2 -  Radio Reset, PortE bit 7 - Radio GIO2/INT
}

/***********************************************************************/
//			ADC_init
//	Set up portF bit 7 and ADC reg for Cd_S light meter
/***********************************************************************/
void ADC_init(void){
	DDRF  &= ~(_BV(DDF7)); 	//make port F bit 7 the ADC input  
	PORTF &= ~(_BV(PF7));  		//port F bit 7 pullups must be off
	
	ADMUX = (1<<MUX0) | (1<<MUX1) | (1<<MUX2) | (1<<REFS0);    //single-ended input, PORTF bit 7, right adjusted, 
	//10 bits, reference is AVCC
	ADCSRA = (1 << ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADIE); //ADC enabled, don't start yet, enable interrupt on conversion
}

void radio_init(void){
	DDRE 	|=(1<<PE2) |(1<< PE7);		//PortE bit 2 -  Radio Reset, PortE bit 7 - Radio GIO2/INT
	PORTE 	|= 0x04;									//Radio reset at powerup
	
	//Enable interrupt for Radio
	EICRB |= (1<<ISC71) | (1<<ISC70);
	EIMSK |= (1<<INT7);
}

void radio_reset(void){
	PORTE &= ~(1<<PE7); //int2 initially low to sense TWI mode
	DDRE  |= 0x80;      //turn on Port E bit 7 to drive it low
	PORTE |=  (1<<PE2); //hardware reset Si4734 
	_delay_us(200);     //hold for 200us, 100us by spec         
	PORTE &= ~(1<<PE2); //release reset 
	_delay_us(30);      //5us required because of my slow I2C translators I suspect
                    //Si code in "low" has 30us delay...no explaination given
	DDRE  &= ~(0x80);   //now Port E bit 7 becomes input from the radio interrupt
}

void processRadioTune(void){
	if((desired_fm_freq != current_fm_freq) && applyRadioChange == TRUE){
		current_fm_freq = desired_fm_freq; 
		fm_tune_freq();
		SET_VOLUME(current_volume);
		applyRadioChange = FALSE;
	}
}

/***********************************************************************/
//			CdSReadStart
//			Start ADC conversion
/***********************************************************************/
void CdSReadStart(void){
	ADCSRA |= (1 << ADSC); 			//poke the ADSC bit and start conversion
}

/***********************************************************************/
//			setBrightness
//	Adjust brightness changing PWM duty ratio
/***********************************************************************/
void setBrightness(uint8_t value){OCR2 = value;}	//Dimmest: OCR2 = 0xE1; Brightest OCR2 = 0x15;

/***********************************************************************/
//			processLEDbrightness
//	Adjust brightness index, then update the brightness for the LED display
/***********************************************************************/
void processLEDbrightness(uint16_t ADC_value){
	static uint8_t brightnessIndex;
	if(brightnessIndex < (ADC_value/100)) //higher index, brighter
		brightnessIndex++;
	else if(brightnessIndex > (ADC_value/100 + 1))
		brightnessIndex--;
	
	setBrightness(brightness[brightnessIndex]);
}

/* BEGIN INTERRUPT ROUTINE LIST*/
//Interrupt for ADC conversion
ISR(ADC_vect){
  lastADCread = ADC; 	//Range from 50 to 1020 tested
}

// This timer update:
//	Colon in LED display
//	Bargraph AM/PM motion
//	Start ADC read
//	Change LED display brightness(changing duty ratio)
ISR(TIMER0_OVF_vect){
	static uint8_t count_7ms=0;  //hold value of count between interrupts
	count_7ms++;                 //extend counter
	//toggle PB0 each time this happens
	if((count_7ms % 128) == 0) {// If one second doing something
		current_second++;
		// blinking the mid colon everyone sec
		if(bit_is_set(dot,0)) { //if dot is 0bx11
			dot ^= (1UL << 0); // blinking the mid colon
			dot ^= (1UL << 1);
		}
		else dot |= (1<<0)|(1<<1);
		
		//Request data from the Temperature sensor
		getTemperature();
		
		if(!isAM) toBarGraph = (toBarGraph << 1); 	//PM going down
		else toBarGraph = (toBarGraph >> 1);		//AM going up
	}
	if(count_7ms ==1){		//If 1792ms do something
		CdSReadStart(); 
	} 
	if((count_7ms % 32) ==0){	//if 0.25 sec doing something
		processLEDbrightness(lastADCread);
		if(isRadioChange == TRUE) {
			isRadioChange = FALSE;
			applyRadioChange = TRUE;
		}
	}
	
}//TIMER0_OVF_vect

ISR(TIMER1_COMPA_vect){
   //Toggle audio output bit
   PORTD ^= (1UL << 4); 
}

//******************************************************************************
//                          External Interrupt 7 ISR                     
// Handles the interrupts from the radio that tells us when a command is done.
// The interrupt can come from either a "clear to send" (CTS) following most
// commands or a "seek tune complete" interrupt (STC) when a scan or tune command
// like fm_tune_freq is issued. The GPIO2/INT pin on the Si4734 emits a low
// pulse to indicate the interrupt. I have measured but the datasheet does not
// confirm a width of 3uS for CTS and 1.5uS for STC interrupts.
//
// I am presently using the Si4734 so that its only interrupting when the 
// scan_tune_complete is pulsing. Seems to work fine. (12.2014)
//
// External interrupt 7 is on Port E bit 7. The interrupt is triggered on the
// rising edge of Port E bit 7.  The i/o clock must be running to detect the
// edge (not asynchronouslly triggered)
//******************************************************************************
ISR(INT7_vect){STC_interrupt = TRUE;}
//******************************************************************************
/* END INTERRUPT ROUTINE LIST*/

/***********************************************************************/
//			dirOfEncoder
//	Return: clockwise 1, counter-clockwise -1 for left encoder
//	Return: clockwise 2, counter-clockwise -2 for right encoder
/***********************************************************************/
int dirOfEncoder(void){
	//extract k bits from p position: https://www.geeksforgeeks.org/extract-k-bits-given-position-number/
	//	((1 << k) - 1) & (number >> (p - 1))
	//static variable remains in memory while the program is running
	//Almost like global except it cannot be access out of scope
	static uint8_t encoderA= 	0;
	static uint8_t encoderA_prev=	0;
	static uint8_t encoderB= 	0;
	static uint8_t encoderB_prev=	0;
	
	//save previous data
	encoderA_prev = encoderA;
	encoderB_prev = encoderB;
	
	//Update newer data
	encoderA = ((1 << 2) - 1) & (newEncodeData >> (0)); //extract bit 0 and 1 of newEncodeData
	encoderB = ((1 << 2) - 1) & (newEncodeData >> (2)); //extract bit 2 and 3 of newEncodeData
	
	//also can do
	//encoderA = (newEncodeData & 0x03);
	//encoderB = (newEncodeData & 0x0C)>>2;
	if(encoderA != encoderA_prev){
		if((encoderA_prev == 1) && (encoderA == 3))	//transistion from 3->1 is clockwise
			return 1;
		if((encoderA_prev == 2) && (encoderA == 3))
			return -1;
	}
	if(encoderB != encoderB_prev){
		if((encoderB_prev == 1) && (encoderB == 3))	//transistion from 3->1 is clockwise
			return 2;
		if((encoderB_prev == 2) && (encoderB == 3))
			return -2;
	}
	

	return 0;
}

/***********************************************************************/
//			updateSPI
//	Return: void
//	Send new data to bargraph and retrieve data from encoder
/***********************************************************************/
void updateSPI(void){
	/* Encoder code */
	//Flipping bits for CLK_INH and SH/LD for encoder
	PORTE &= ~(0x40);
	PORTE |=   0x10 ;
	
	SPDR = toBarGraph;              	 //send to bar graph display 
    	while(bit_is_clear(SPSR, SPIF)){};     //wait till data sent out (a while loop)
	newEncodeData = SPDR;			//Load in new encoder data
	
	//Flipping bits for CLK_INH and SH/LD for encoder
	PORTE |=   0x40;
	PORTE &= ~(0x10);
	
	if(toBarGraph == 0x80 && !isAM){toBarGraph = 1;}	//only for PM
	if(toBarGraph == 0x00 && isAM){toBarGraph = 0x80;}	//only for AM
	bargraphLatch();
}

/***********************************************************************/
//			bargraphLatch
// 	latching the data to the output of the bargraph
/***********************************************************************/
void bargraphLatch(void){
	PORTD |= 0x04;         	//HC595 output reg - rising edge...
	PORTD &= ~0x04;          	//and falling edge	
}

void processEncoder(volatile uint32_t *time_to_mod, int encoderFlag){
	if(state == 0 | state ==1){
		if(encoderFlag == 2) 		*time_to_mod = *time_to_mod + 60;
		else if(encoderFlag == -2)	*time_to_mod = *time_to_mod - 60;
		else if(encoderFlag == 1)	*time_to_mod = *time_to_mod + 3600;
		else if(encoderFlag == -1)	*time_to_mod = *time_to_mod - 3600;
	}
	if(state == 3){	//Allow freq selection and volume adjustment
		uint16_t temp = *time_to_mod;		//cant seem to compare *time_to_mod directly
		if(encoderFlag == 2 && temp <10800)			{	*time_to_mod = *time_to_mod + 20;	isRadioChange = TRUE;}
		else if(encoderFlag == -2 && temp > 8800)		{	*time_to_mod = *time_to_mod - 20;	isRadioChange = TRUE;}
		else if(encoderFlag == 1 && current_volume < 100)	{	current_volume += 5;			isRadioChange = TRUE;}
		else if(encoderFlag == -1 && current_volume > 0)	{	current_volume -= 5;			isRadioChange = TRUE;}
	}
}

/***********************************************************************/
//update7Segment
//Convert a number to oneth, tenth ... for each digit of the 4LED - 7segment display
//No leading 0 suppression
//Parameter: Number - to print out, Dot for special character L1 L2 L3
//for Dot, pass in b001 to enable L1, b011 to enable L2+L1, b100 to enable L3
//Return: void
/***********************************************************************/
void update7Segment(uint16_t number, uint8_t dot){
				//(PORT bit = 1 in B mean 5V in electrical)
	DDRA = 0xFF;    	//Set port A to all output
	
	uint8_t oneth		= number%10;		//Extract the oneth
	uint8_t tenth		= (number%100)/10;	//Extract the tenth
	uint8_t hundredth	= (number%1000)/100;	// ...
	uint8_t thousandth	= (number%10000)/1000;
	
	//Changing output of PORTB here does not impact SPI
	//SPI need PORTB bit 1->3
	// Cycle through 4 digits of the LED display
		//PORTB = 0x00 ; //<- Issue here
		PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB6));	//Clear bit 4,5,6
		PORTA = indexToLed[oneth];
	_delay_ms(1);

		//PORTB = 0x10;
		PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB6));	//Clear bit 4,5,6
		PORTB |=  (1<<PB4);
		PORTA = indexToLed[tenth];
	_delay_ms(1);
	
	if(dot){
		//PORTB = 0x20;
		PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB6));	//Clear bit 4,5,6
		PORTB |=  (1<<PB5);
		// Decode for portA: 0x04 = L1+L2, 0x03 = L3||| Write a 0 to port A to turn on the dot, sinking current.
		PORTA = ~(dot);
		_delay_ms(1);
	}

		//PORTB = 0x30;
		PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB6));	//Clear bit 4,5,6
		PORTB |=  (1<<PB4) | (1<<PB5);
		PORTA = indexToLed[hundredth];
	_delay_ms(1);
	
		//PORTB = 0x40;
		PORTB &= ~((1<<PB4) | (1<<PB5) | (1<<PB6));	//Clear bit 4,5,6
		PORTB |=  (1<<PB6);
		PORTA = indexToLed[thousandth];
	_delay_ms(1);
	
}


/***********************************************************************/
//			second_to_min_hour
//	Return: hour and minute in decimal format for LED display
//	MAJOR disadvantage: computationally intensive
/***********************************************************************/
uint16_t second_to_min_hour(volatile uint32_t *second, bool mode){
	uint8_t hour_tenth;
	uint8_t hour_oneth;
	uint8_t min_tenth;
	uint8_t min_oneth;
	uint16_t result;
	
	min_oneth = ((*second/60) % 60) % 10;
	min_tenth = ((*second/60) % 60) / 10;
	// if mode 0 = 24 hours type, 1 = 12 hours type
	hour_oneth = (*second/3600)%10;
	hour_tenth = (*second/3600)/10;
	
	// if mode 0 = 24 hours type, 1 = 12 hours type
	if(hour_tenth == 2 && hour_oneth == 4) *second = 0;	//reset second count if time reach 24 hours
	
	//Converting 24 hours to 12 hours type
	result = hour_tenth*1000 + hour_oneth*100 + min_tenth*10 + min_oneth;
	if(mode == 1){
		result  = result%1200;
	}		
	
	return result;
}

/*
IsButtonPressed
Parameter: Button - Take in an number as a button and check the button state if pressed for more than 12 cycle
Return:	 1 if the button is pressed for 12 cycle
	 0 if the button is not pressed, or not long enough
*/
uint8_t isButtonPressed(uint8_t button){
	DDRA = 0x00;		//Set port A to all input
	PORTA = 0xFF;		//Set port A to all pull-ups;

	PORTB |=  (1<<PB4) | (1<<PB5) | (1<<PB6); // Turn on tristate buffer for the button board
	_delay_us(20);
	static uint16_t state [8] = {0};	//Fill the array with all 0s
	state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;	//Debounce 12 cycle
	if(state[button] == 0xF000) return 1;	//Button is pressed
	return 0;				//Button is not pressed
}

/***********************************************************************/
//                                processButton 
//	Take in button flag and set the mode of the system
//	Button 0 switch from normal to alarm setting
//	Button 1 turn on or turn off the alarm at that specific time 
//	Button 2 switch between 12 hour mode or 24 hour mode                             
/***********************************************************************/
void processButtomNumber(int i){
	if(state == 0){ 			//Normal State
		switch(i){
			case 0: 
				state = 1;
				break;
			case 2:
				clk_mode = !clk_mode;	// L3 dot on = 24hour type
				dot ^= 1UL << 2;				//Toggle the L3 dot on the LED display
				break; 
			case 4:		//Radio tune button
				state = 3;
		}
	}
	else if(state == 1){	//Setting alarm time
		switch(i){
			case 0:
				state = 0;
				break;
			case 1:
				isAlarmSet = !isAlarmSet;
				if(isAlarmSet) alarm_time = alarm_display_second;
				break;
			case 2:
				clk_mode = !clk_mode;	// L3 dot on = 24hour type
				dot ^= 1UL << 2;				//Toggle the L3 dot on the LED display	
				break;
			case 3:
				alarm_mode = !alarm_mode;
		}
	}
	else if(state == 2){	//Alarm is firing
		switch(i){
			case 7:
				isRinging = 0;
				state = 0;
				break;
			default:
				snoozing();
		}	
	}
	else if(state == 3){ //Radio tune button
		switch(i){
			case 4:
				state = 0;
				break;
		}
		//Turn on radio and start tuning ?
	}
	
}

void checkAlarm(void){
	if(isAlarmSet && (current_second == alarm_time)) { 	// turn volume up if Ringing
		isRinging = 1;
			if(alarm_mode == JUST_TONE) TONE_ON();
			if(alarm_mode == JUST_RADIO & isRadioOn == 0) {	//over sending I2C line will kill the system
				RADIO_ON();
				isRadioOn = 1;
			}
		SET_VOLUME(current_volume);
	}
	else if(isRinging == 1 && (current_second < alarm_time + RINGING_TIME)) //Keep ringing
		SET_VOLUME(current_volume);
	else {								// turn volume down if not ringing
		SET_VOLUME(0);
		TONE_OFF();
			if(isRadioOn){		//over sending I2C will kill the system
				RADIO_OFF();
				isRadioOn = 0;
				SET_VOLUME(0);
			}
		isRinging = 0;	
	}
}

void SET_VOLUME(uint8_t percent){
	OCR3A = percent * 4 + 1;			//OCR3A doesn't like to be 0
}

void snoozing(void){
	isRinging = 0;
	alarm_time = current_second + SNOOZE_TIME;
	state = 0;
}

void updateDisplay(void){
	//Check multiple flag to generate the LCD display

	char appendAlarm[] = "Alarm ";
	char appendRadio[] = "Radio Mode";
	char appendTone[]	= "Tone Mode ";
	char temperaturePrint[] = "Temp: ";
	char appendNoAlarm[] = "Alarm is off    ";
	
	//To clean the string before appending;
	for(i = 0; i< 32; i++) {lcd_final[i] = ' ';}
	lcd_final[0]= '\0';	//Need this char or the system will get segmentation fault
	
	if(isAlarmSet){
		strcat(lcd_final, appendAlarm);
		if(alarm_mode == JUST_TONE)
			strcat(lcd_final, appendTone);
		else if (alarm_mode == JUST_RADIO)
			strcat(lcd_final, appendRadio);
	}
	else {
		strcat(lcd_final, appendNoAlarm);
	}
	//Display temperature
	processTemperature();	
	strcat(lcd_final, temperaturePrint);
	strcat(lcd_final, temperature_string);	
	lcd_final[24] = 'F';
	
	//Display AM/PM
	if(isAM){
		lcd_final[30] = 'A';
		lcd_final[31] = 'M';
	}
	else{
		lcd_final[30] = 'P';
		lcd_final[31] = 'M';
	}
		
	refresh_lcd(lcd_final);
}

void getTemperature(void){
	twi_start_rd(LM73_ADDRESS, lm73_rd_buf,2); //read temperature data from LM73 (2 bytes) 
}

void processTemperature(void){
	lm73_temp = lm73_rd_buf[0]; 	//save high temperature byte into lm73_temp
	lm73_temp = lm73_temp << 8; 	//shift it into upper byte 
	lm73_temp |= lm73_rd_buf[1]; 	//"OR" in the low temp byte to lm73_temp 
	
	//conversion to F
	lm73_temp = (lm73_temp*0.0078)*9/5 + 32;
	//convert to string in array with itoa() from avr-libc 
	itoa(lm73_temp, temperature_string, 10); 
}

int main() {

	//Setup
	spi_init();
	IO_config_init();
	clock_config_init();
	ADC_init();
	IO_config_init();
	radio_init();
	init_twi();	 //initalize TWI (twi_master.h)  
	lcd_init(); 
	radio_reset();
	clear_display();
	sei();			//Enable all interrupts
	
	/* Default value for radio*/
	fm_pwr_up();
	desired_fm_freq = 10630;
	current_fm_freq = 10630; 
	fm_tune_freq(); //tune radio freq to current_fm_freq
	
	RADIO_OFF();
	/*Default value when start up for alarm*/
	alarm_display_second = 43200;	//Alarm time setting will display noon
	current_second = 0;


	while(1){
		
		updateSPI();	//Read encoder value + update bar graph to indicate am or pm
		
		updateDisplay();
			
		switch(state){	
			case 0: //State 0 Normal time with/without Alarm
				update7Segment(second_to_min_hour(&current_second, clk_mode), dot); // turn on L1 and L2 also
				processEncoder(&current_second, dirOfEncoder());
				if(current_second < 43200) isAM = 1;
				else isAM = 0;
				checkAlarm();	//Take care of turning off the RADIO
				if(isRinging) state = 2;	//If alarm trigged, move to state 2
				break;
			
			case 1: //State 1 setting time for alarm		
				update7Segment(second_to_min_hour(&alarm_display_second, clk_mode), dot); // turn on L1 and L2 also
				processEncoder(&alarm_display_second, dirOfEncoder());
				if(alarm_display_second < 43200) isAM = 1;
				else isAM = 0;
				break;
			case 2: //State 2 Alarm is firing
				update7Segment(second_to_min_hour(&current_second, clk_mode), dot); // turn on L1 and L2 also
				break;
			case 3: //Adjusting radio
				if(isRadioOn = 0);{
					RADIO_ON();
					SET_VOLUME(current_volume);
				}
				update7Segment(desired_fm_freq/10, 0);
				//update7Segment(current_volume,0);
				processEncoder(&desired_fm_freq, dirOfEncoder());
				processRadioTune();
				break;
		}
		
		for(i=0; i<8; i++){
			if(isButtonPressed(i)) processButtomNumber(i);
		}
		//Radio stuff here
		
		
  } //while
	} //main
