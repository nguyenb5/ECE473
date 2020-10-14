// lab1_code.c 
// R. Traylor - Modified by Bao Nguyen
// 7.13.20

//This program increments a binary display of the number of button pushes on switch 
//S0 on the mega128 board.

#include <avr/io.h>
#include <util/delay.h>

//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S0,S1 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time. Expects active low pushbutton on 
// Port D bit zero. Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int8_t debounce_switchS0() {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PIND, 0)) | 0xE000;
  if (state == 0xF000) return 1;
  return 0;
}

int8_t debounce_switchS1() {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PIND, 1)) | 0xE000;
  if (state == 0xF000) return 1;
  return 0;
}

//************
// Global var to store debounce value, may be bad ...
//************
_Bool button0_pressed=0;
_Bool button1_pressed=0;

//*******************************************************************************
// Check switch S0, S1.  When found low for 12 passes of "debounce_switchSx(), 
// S1 push button increments the count by one and S2 decrements the count by one. 
// if both button is pressed, no thing will happen.
// This will make an incrementing/decrenemt count on the port B LEDS. 
//*******************************************************************************
int main()
{
DDRB = 0xFF;  //set port B to all outputs, for LEDs
while(1){     //do forever

button0_pressed = debounce_switchS0();	//turn 1 if switch true for 12 passes, S0
button1_pressed = debounce_switchS1(); 	//turn 1 if switch true for 12 passes, S1

if(button0_pressed & bit_is_clear(PIND, 1)) {} //both button pressed, do nothing, 1 of PIND = S1
else if(button0_pressed) {PORTB++;};		//only S0 is pressed, increment port B
if(button1_pressed & bit_is_clear(PIND, 0)) {} //both button pressed, do nothing, 0 of PIND = S0
else if(button1_pressed) {PORTB--;}		//only S2 is pressed, decrement port B
_delay_ms(2);					//keep in loop to debounce 24ms
  	} 
  //while 
} //main

