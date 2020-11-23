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

// 
/* Encoder board */
// 	SH/LD		PORTE bit 6
// 	CLK_INH	PORTE bit 7
//	SCK		PORTB bit 1 (sclk)
//	SER_OUT	PORTB bit 3 (miso)

/* ADC CdS cell */
// Photocell 	PORTF bit 7

/* AudioBoard */
// Audio_from_atmega		PORTD bit 4
// Atmega_vol_control		PORTE bit 3

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "hd44780.h"

#define RINGING_TIME 60
#define SNOOZE_TIME 10 

uint8_t indexToLed[11] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80,
			0x98, 0xFF};
uint8_t brightness[10] = {0xD0, 0xD0, 0xB5, 0xA1, 0x8D, 0x79, 0x65, 0x51, 0x3D, 0x15, 0x15};

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
bool isAM;
bool isRinging = 0;
bool isSnoozing = 0;



volatile uint8_t inc_dec;
int i;	//dummy var

char     lcd_str_h[16];  //holds string to send to lcd  
char     lcd_str_l[16];  //holds string to send to lcd 
div_t    fp_adc_result, fp_low_result;  //double fp_adc_result; 
/** End Global Vars **/

//Initialization
void spi_init(void);
void clock_config_init(void);
void IO_config_init(void);
//ADC stuff
void ADC_init(void);
uint16_t CdS_read(void);
void setBrightness(uint8_t value);
//Encoder stuff
int dirOfEncoder(void);
void updateSPI(void);
void bargraphLatch(void);
//LED stuff
void processLEDbrightness(uint16_t ADC_value);
void update7Segment(uint16_t number, uint8_t dot);
uint16_t second_to_min_hour(uint32_t second, bool mode);
//Button stuff
uint8_t isButtonPressed(uint8_t button);
void processButton(void);
//Audio stuff
void checkAlarm(void);

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
	DDRE |= 0xC0;  	//PortE bit6,7 to control Encoder CLK Inhibit, SH/LD!
	DDRD |= 0x04;		//PortD bit2 for barGraph REG_CLK
	
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
		
		if(!isAM) toBarGraph = (toBarGraph << 1); 	//PM going down
		else toBarGraph = (toBarGraph >> 1);		//AM going up
	}
	if(count_7ms ==1){		//If 1792ms do something
		CdSReadStart(); 
	} 
	if((count_7ms % 32) ==0){	//if 0.25 sec doing something
		processLEDbrightness(lastADCread);
	}
	
}//TIMER0_OVF_vect

ISR(TIMER1_COMPA_vect){
   //Toggle audio output bit
   PORTD ^= (1UL << 4); 
}

/* END INTERRUPT ROUTINE LIST*/

/***********************************************************************/
//			dirOfEncoder
//	Return: clockwise 1, counter-clockwise -1 for left encoder
//	Return: clockwise 2, counter-clockwise -2 for right encoder
/***********************************************************************/
int dirOfEncoder(void){
	//extract k bits from p position: https://www.geeksforgeeks.org/extract-k-bits-given-position-number/
	//	((1 << k) - 1) & (number >> (p - 1))
	//State machine
	
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
	PORTE |=   0x80 ;
	
	SPDR = toBarGraph;              	 //send to bar graph display 
    	while(bit_is_clear(SPSR, SPIF)){};     //wait till data sent out (a while loop)
	newEncodeData = SPDR;			//Load in new encoder data
	
	//Flipping bits for CLK_INH and SH/LD for encoder
	PORTE |=   0x40;
	PORTE &= ~(0x80);
	
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
	if(encoderFlag == 2) 		*time_to_mod = *time_to_mod + 60;
	else if(encoderFlag == -2)	*time_to_mod = *time_to_mod - 60;
	else if(encoderFlag == 1)	*time_to_mod = *time_to_mod + 3600;
	else if(encoderFlag == -1)	*time_to_mod = *time_to_mod - 3600;
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
uint16_t second_to_min_hour(uint32_t second, bool mode){
	uint8_t hour_tenth;
	uint8_t hour_oneth;
	uint8_t min_tenth;
	uint8_t min_oneth;
	uint16_t result;
	
	min_oneth = ((second/60) % 60) % 10;
	min_tenth = ((second/60) % 60) / 10;
	// if mode 0 = 24 hours type, 1 = 12 hours type
	hour_oneth = (second/3600)%10;
	hour_tenth = (second/3600)/10;
	
	// if mode 0 = 24 hours type, 1 = 12 hours type
	if(hour_tenth == 2 && hour_oneth == 4) current_second = 0;	//reset second count if time reach 24 hours
	
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
	switch(i){
		case 0: //If state button is pressed
			if(state == 0) 	state = 1;
			else if (state == 1) 	state = 0;
			else if (state == 2) 	snoozing(); 	//while Alarm is firing - snoozing
			break;
		case 1: //If alarm button is pressed
			if(state == 1){ 	//While in alarm setting state, turn on/off the alarm
				isAlarmSet = !isAlarmSet;
				if(isAlarmSet) alarm_time = alarm_display_second;
			}
			else if(state == 2)	snoozing(); 	//while Alarm is firing - snoozing
			
			break;
		case 2: //If clk_mode buttom is pressed
			if(state == 0 | state == 1) {	
				clk_mode = !clk_mode;		// L3 dot on = 24hour type
				dot ^= 1UL <<2;		//Toggle the L3 dot on the LED display
			}
			if(state == 2)		snoozing();	//while Alarm is firing - snoozing
			break;
			
		case 7: //Stop button when ringing
			if(state == 2){isRinging = 0; state = 0;}	//while Alarm is firing - stop alarm, return to normal
			break;
		default: 
			if(state == 2)		snoozing();
			break;
	}
}

void checkAlarm(void){
	if(isAlarmSet && (current_second == alarm_time)) { 	// turn volume up if Ringing
		OCR3A = 200;
		isRinging = 1;	
	}
	else if(isRinging == 1 && (current_second < alarm_time + RINGING_TIME)) //Keep ringing
		OCR3A = 200;
	else {								// turn volume down if not ringing
		OCR3A = 1;
		isRinging = 0;	
	}
}

void snoozing(void){
	isRinging = 0;
	alarm_time = alarm_time + SNOOZE_TIME;
	state = 0;
}

int main() {
	int flag=0;
	char lcd_final0[32] = "Alarm is armed                  ";
	
	//Setup
	spi_init();
	IO_config_init();
	clock_config_init();
	ADC_init();
	IO_config_init();
	lcd_init(); 
	clear_display();
	sei();			//Enable all interrupts
	
	/*Default value when start up*/
	alarm_display_second = 43200;	//Alarm time setting will display noon
	current_second = 0;

	while(1){
		
		updateSPI();	//Read encoder value + update bar graph to indicate am or pm
		if(isAlarmSet) refresh_lcd(lcd_final0);
		else		clear_display();
		
		switch(state){	
			case 0: //State 0 Normal time with/without Alarm
				update7Segment(second_to_min_hour(current_second, clk_mode), dot); // turn on L1 and L2 also
				processEncoder(&current_second, dirOfEncoder());
				if(current_second < 43200) isAM = 1;
				else isAM = 0;
				checkAlarm();
				if(isRinging) state = 2;	//If alarm trigged, move to state 2
				break;
			
			case 1: //State 1 setting time for alarm		
				update7Segment(second_to_min_hour(alarm_display_second, clk_mode), dot); // turn on L1 and L2 also
				processEncoder(&alarm_display_second, dirOfEncoder());
				if(alarm_display_second < 43200) isAM = 1;
				else isAM = 0;
				break;
			case 2: //State 2 Alarm is firing
				update7Segment(second_to_min_hour(current_second, clk_mode), dot); // turn on L1 and L2 also
				break;		
		}
		
		for(i=0; i<8; i++){
			if(isButtonPressed(i)) processButtomNumber(i);
		}
		
  } //while
	} //main
