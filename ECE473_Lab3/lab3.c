// Expected Connections:
// Bargraph board           Mega128 board 
// --------------      ----------------------    
//     reglck            PORTB bit 0 (ss_n)                      
//     srclk             PORTB bit 1 (sclk)
//     sdin              PORTB bit 2 (mosi)
//     oe_n                   ground
//     gnd2                   ground
//     vdd2                     vcc
//     sd_out               no connect

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>

uint8_t indexToLed[11] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80,
			0x98, 0xFF};

/** Global Vars **/
volatile uint8_t toBarGraph;
volatile uint8_t newEncodeData;
volatile uint16_t display_number; 
volatile uint8_t stateEncoder1;
volatile uint8_t stateEncoder2;
volatile bool button0_flag = 0;
volatile bool button1_flag = 0;
volatile uint8_t inc_dec;
int i;	//dummy var
/***********************************************************************/
//                            spi_init                               
//Initalizes the SPI port on the mega128. 
//SPI settings: master mode, clock=clk/2, cycle half phase, low polarity, 
//MSB first, SPI interrupts are disabled. 
//We will poll SPIF bit in SPSR to check SPI send completion
/***********************************************************************/

void spi_init(void){

  SPCR  |=  (1<<SPE) | (1<<MSTR);  //enable SPI, master mode 

  SPSR  |=  (1<<SPI2X);	 	// double speed operation

 }//spi_init

/***********************************************************************/
//                              tcnt0_init                             
//Initalizes timer/counter0 (TCNT0). TCNT0 is running in async mode
//with external 32khz crystal. Set to run in normal mode with no prescaling.
//Interrupt should occur at overflow, 0xFF.
/***********************************************************************/
void tcnt0_init(void){

  ASSR   |=  (1<<AS0);			 //ext osc TOSC

  TIMSK  |=  (1<<TOIE0);	 	//enable TCNT0 overflow interrupt

  TCCR0  |=  (1<<CS00); 		//normal mode, no prescale

}

/***********************************************************************/
//			config_IO
/***********************************************************************/
void config_IO(void){
  DDRB |= 0x07;		 //Turn on SS, MOSI, SCLK
  
  DDRA = 0xFF;		//PortA to all output for 4 digits 7 seg LED
  DDRB |= 0xF0;	//PortB bit4->7 to control 4 digits 7 seg LED
  DDRE |= 0xC0;  	//PortE bit6,7 for encoder CLK Inhibit, SH/LD!
  DDRD |= 0x04;	//PortD bit2 for barGraph REG_CLK
}

/***********************************************************************/
//			bargraphLatch
/***********************************************************************/
void bargraphLatch(void){
	PORTD |= 0x04;         	//HC595 output reg - rising edge...
	PORTD &= ~0x04;          	//and falling edge	
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
	PORTB = 0xF0; 	//Turn on TriStateBuffer for Button Board

	static uint16_t state [8] = {0};	//Fill the array with all 0s
	state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;	//Debounce 12 cycle
	if(state[button] == 0xF000) return 1;	//Button is pressed
	return 0;				//Button is not pressed
}

/*
update7Segment
Convert a number to oneth, tenth ... for each digit of the 4LED - 7segment display
Parameter: Button - Take in an 16 bit unsign number to print out
Return: void
*/
void update7Segment(uint16_t number){
				//(PORT bit = 1 in B mean 5V in electrical)
	DDRA = 0xFF;    	//Set port A to all output
	
	uint8_t oneth		= number%10;		//Extract the oneth
	uint8_t tenth		= (number%100)/10;	//Extract the tenth
	uint8_t hundredth	= (number%1000)/100;	// ...
	uint8_t thousandth	= (number%10000)/1000;
	
	//Changing output of PORTB here does not impact SPI
	//SPI need PORTB bit 1->3
	// Cycle through 4 digits of the LED display
		PORTB = 0x00 ;
		PORTA = indexToLed[oneth];
	_delay_ms(1);
	if(tenth | hundredth | thousandth){	//Leading zero suppression
		PORTB = 0x10 ;
		PORTA = indexToLed[tenth];
	_delay_ms(1);
	}
	if(hundredth | thousandth){		//Leading zero suppression
		PORTB = 0x30;
		PORTA = indexToLed[hundredth];
	_delay_ms(1);
	}
	if(thousandth){				//Leading zero suppression
		PORTB = 0x40;
		PORTA = indexToLed[thousandth];
	_delay_ms(1);
	}
}


/*************************************************************************/
//                           timer/counter0 ISR                          
//When the TCNT0 overflow interrupt occurs, the count_7ms variable is    
//incremented. TCNT0 interrupts should come at 7.8125ms internals.
// 1/32768         = 30.517578uS
//(1/32768)*256    = 7.8125ms
//(1/32768)*256*64 = 500mS
/*************************************************************************/
ISR(TIMER0_OVF_vect){
 	
 	//Flipping bits for CLK_INH and SH/LD for encoder
	PORTE &= ~(0x40);
	PORTE |=   0x80 ;
	
    	SPDR = toBarGraph;              	//send to bar graph display 
    	//SPDR = newEncodeData;              	//send to bar graph display 
    	while(bit_is_clear(SPSR, SPIF)){};               //wait till data sent out (a while loop)
	newEncodeData = SPDR;			//Load in new encoder data
	
	//if(bit_is_set(newEncodeData,0)) toBarGraph = 0x20;
	//else toBarGraph = 0x02;
	
	//Flipping bits for CLK_INH and SH/LD for encoder
	PORTE |=   0x40;
	PORTE &= ~(0x80);
	
	//latch data out to bargraph
	bargraphLatch();
}

/***********************************************************************/
//			dirOfEncoder
//	Return: clockwise 1, counter-clockwise -1, no turn 0
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
			return 1;
		if((encoderB_prev == 2) && (encoderB == 3))
			return -1;
	}
	

	return 0;
}

/***********************************************************************/
//                                processIncDec                                 
/***********************************************************************/
void processIncDec(){

	if(button0_flag == 1)
		inc_dec = 2;
	else if(button1_flag == 1)
		inc_dec = 4;
	else if(button0_flag == 1 & button1_flag == 1)
		inc_dec = 0;
	else 	inc_dec = 1;
}

/***********************************************************************/
//                                main                                 
/***********************************************************************/
int main(){     
tcnt0_init();  //initalize counter timer zero
config_IO();
spi_init();    //initalize SPI port
sei();         //enable interrupts before entering loop

display_number = 0;	//Zero the display number
int flag=0;		//Flag for checking which button is pressed

while(1){
	//Update display
	
	update7Segment(display_number);
		
	//Check Button

	if(isButtonPressed(0) == 1) {button0_flag = !button0_flag;}
	
	if(isButtonPressed(1) == 1) {button1_flag = !button1_flag;}
	
	processIncDec();
	toBarGraph =  button0_flag | button1_flag << 1;

	
		flag =  dirOfEncoder();
		if(flag == 1) 		{
			display_number += inc_dec;
			}
		else if(flag == -1)	{
			display_number -= inc_dec;
			}
	
		if(display_number >= 1024)	display_number = (display_number % 1024);		//Max 1023, wrap around
	
}     //empty main while loop

} //main
