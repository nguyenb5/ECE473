#include <avr/io.h>
#include <util/delay.h>

uint8_t indexToLed[11] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80,
			0x98, 0xFF};
int i;	//Dummy var;
volatile int display_number; 

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
	PORTB = 0x00;	//Turn off TriStateBuffer for Button Board
				//(PORT bit = 1 in B mean 5V in electrical)
	DDRA = 0xFF;    	//Set port A to all output
	
	uint8_t oneth		= number%10;		//Extract the oneth
	uint8_t tenth		= (number%100)/10;	//Extract the tenth
	uint8_t hundredth	= (number%1000)/100;	// ...
	uint8_t thousandth	= (number%10000)/1000;
	
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

int main(){
	DDRB = 0xF8;		//Set portB bits 3-7 as output
				//Bit 3 is TristateBuffer En. Bit 4-7 is for LEDs
	
	display_number = 0;	//Zero the display number
	int flag=0;		//Flag for checking which button is pressed
	
	while(1){
		//Update display
		update7Segment(display_number);
		
		//Check Button
		for(i=0; i<8; i++){
			flag =  isButtonPressed(i);
			if(flag == 1)
				display_number += (1<<i);
		}
			if(display_number >= 1024)	display_number = 0;		//Max 1023, roll to 0				
	}
}
