//Inclass Exercise (switch bounce)
//R. Traylor 7.15.2020

//The code below can reveal how 2 debouncing methods differ and how their
//timing can effect the results of sampling mechanical switches. Interrupts 
//from Timer Counter 0 provide a way to vary the sampling period of a 
//pushbutton switch on the AVR board. The number of pushbutton pushes
//are displayed on the LCD display.

//When the sampling interval is correct, you should see the display count
//increment by one for each button push with little delay. With too long a
//delay, you may be able to hit the switch quickly and not have it detected.
//With too little delay, it could count multiple hits when you have only 
//pushed the button once. 

//Try different pushbuttons and see which creates the most bouncing. They 
//can vary greatly or not at all. Then, for each sampling method, vary the 
//sampling time and and note any difference. Use the #ifdef statements to 
//enable or disable debouncing.

//Keep a note of your findings so in later labs you'll have the approximate
//debouncing timing to use. Note that everybodies answers will probably 
//differ somewhat. There are few "right" or "wrong" answers.

//Fill in the blanks below and submit your switch_example.c file via Canvas by end 
//of day.

//Note: "git" must be installed first. If necessary, "sudo apt install git"

//First, save this file as switch_bounce.c
//Then, download the required files from my area and from github:
//wget http://www.ece.orst.edu/~traylor/ece473/inclass_exercises/io_ports/switch_bounce/Makefile
//git clone https://github.com/rltraylor/hd44780_driver.git

//Finally, try the different debouncing methods and different delays between samples.
//
//The PORTD pin that exhibits the "most" bounce (0-7) : ___
//
//With switch debouncer enabled:
//  too fast, bounces counted            :   _______ (ns or mS)
//
//  too slow, misses fast button pushes  :   _______ (ns or mS)
//
//Without switch debouncer:
//  too fast, bounces counted            :   _______ (ns or mS)
//
//  too slow, misses fast button pushes  :   _______ (ns or mS)

                            
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780_driver/hd44780.h"

//If the next line is commented out, debouncing is disabled
#define DEBOUNCE

char lcd_str[16];                    //holds string to send to lcd
volatile uint16_t switch_count = 0;  //count switch closures
uint8_t i;                           //just a dumb counter

//******************************************************************************
//                            spi_init                               
//Initalizes the SPI port on the mega128. Does no further device specific inits
//******************************************************************************
void spi_init(void){
    DDRB |=  0x07;  //Turn on SS, MOSI, SCLK
    //mstr mode, sck=clk/2, cycle 1/2 phase, low polarity, MSB 1st, no interrupts 
    SPCR=(1<<SPE) | (1<<MSTR); //enable SPI, clk low initially, rising edge sample
    SPSR=(1<<SPI2X); //SPI at 2x FCPU 
}//spi_init

//******************************************************************************
//                            ISR for Timer 0                        
//
//Runs once at each TCNT0 interrupt and notes the polarity of the pushbutton
//switch. When pressed, the pushbutton state is '0'.

//******************************************************************************

ISR(TIMER0_COMP_vect){ 
#ifdef DEBOUNCE 
  //simple, but effective 12-cycle debouncer
  static uint64_t state = 0;     //"state" holds present state
  state = (state << 1) | (! bit_is_clear(PIND, 0)) | 0xE000;
  if (state == 0xF000) {switch_count++;}  //increment if pushed for 12 cycles 
#else 
  //3-state state machine makes pushbutton give one pulse for each push and release
  //It provides no rejection switch bounce however.
  static enum button_state_type{NOTSET, SET, WAIT} button_state;
  switch(button_state){
    case(NOTSET): 
      if(bit_is_clear(PIND,0)){ //stay here till button is pushed
        button_state=SET;
      } break;
    case(SET): 
      switch_count++;           //button was pushed, increment count 
      button_state=WAIT; 
      break;
    case(WAIT): 
      if(bit_is_set(PIND,0)){   //stay here till button is released
        button_state=NOTSET;
      } 
      break;
    default: break;
  } //switch                  
#endif
} //ISR

//******************************************************************************
// main() body
//******************************************************************************
int main(void){
  spi_init();      //initalize SPI 
  lcd_init();      //initalize LCD 
  clear_display(); //manually clear LCD display 
  cursor_off();    //keep LCD cursor off
 
  //Setup timer/counter TCNT0 to run in CTC mode
  TIMSK |= (1<<OCIE0);  //enable TCNT0 compare interrupt
  TCCR0 |= (1<<WGM01);  //CTC mode, no prescaling

//fine tuning of interrupt interval
//OCR0  = 0x01;        //compare register small, interrupts come close together
//OCR0  = 0xFF;        //compare register big, interrupts far apart 
//coarse tuning of interrupt interval
//TCCR0 |= (1<<CS00);                     //no prescaling of clock to counter
//TCCR0 |= (1<<CS01)|(1<<CS00);           //prescale TCNT0 clock by 32 
//TCCR0 |= (1<<CS02)|(1<<CS00);           //prescale TCNT0 clock by 128
//TCCR0 |= (1<<CS02)|(1<<CS01)|(1<<CS00); //prescale TCNT0 clock by 1024

//try some of these values with and without debouncing
//interrupt period = 62.5ns * (OCR0+1) * prescale
//  OCR0=0xFF; TCCR0 |= (1<<CS02)|(1<<CS01)|(1<<CS00); //switch sample period=32.7ms
//  OCR0=0x3F; TCCR0 |= (1<<CS02)|(1<<CS00);           //switch sample period=1ms 
    OCR0=0x01; TCCR0 |= (1<<CS00);                     //switch sample period=125ns

  sei();                             //enable global interrupts
  while(1){                          //main while loop 
    _delay_ms(10);                   //10mS wait to prevent display flicker 
    itoa(switch_count, lcd_str, 10); //convert integer to ascii code
    string2lcd(lcd_str);             //send string to LCD
    cursor_home();
  } //while
} //main
