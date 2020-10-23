#include <avr/io.h>
#include <util/delay.h>

uint8_t indexToLed[11] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80,
			0x98, 0xFF};
int i;

uint8_t checkButton(){
	uint8_t value = 0;
	DDRA = 0x00;		//Set port A to all input
	PORTA = 0xFF;		//Set port A to all pull-ups;
	PORTB |= (0<<3);	//Turn on tristate buffer (tristateBuffer ENABLE is active low)
	_delay_ms(1000);
	switch(PINA){
		case 0xFE: value = 0; break;
		case 0xFD : value = 1; break;
		case 0xFB : value = 2; break;
		case 0xF7 : value = 3; break;
		case 0xEF : value = 4; break;
		case 0xDF : value = 5; break;
		case 0xBF : value = 6; break;
		case 0x7F : value = 7; break;
		default : value = 5;
	}
	// if(bit_is_clear(PINA, 0) ) { value = 3;} // Check for bit 0 of PORTA
	// else value = 5;
	return value;
}

void updateDisplay(uint8_t input){
	PORTB |= (1<<3);	//Turn off tristate buffer on Button Board (PORT bit = 1 in B mean 5V in electrical)
	DDRA = 0xFF;    	//Set port A to all output
	PORTA = indexToLed[input];
	_delay_ms(2);
}

int main(){
	uint8_t value;
	DDRB = 0xF8; 		//Set port B bits 3-7 as output, bit 3 is TristateBuffer Enable, 
				// bit 4-7 is for 4-Numbers LED board
	PORTB = 0x30;		//digit 2 on
	//while(1){
	//	updateDisplay(checkButton());
	//}
	_delay_ms(2000);
	value = checkButton();
	
	PORTB |= (1<<3);	//Turn off tristate buffer on Button Board (PORT bit = 1 in B mean 5V in electrical)
	_delay_ms(2000);
	DDRA = 0xFF;    	//Set port A to all output
	PORTA = indexToLed[value];
	
}




