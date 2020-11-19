// siren.c  - R. Traylor  - 10.31.2013
// 
// Setup TCNT1 to be a varaible frequency tone generator. Its made variable by dynamically 
// changing its compare register.
//
// Set OC1A (PB5) as the audio output signal 
// TCNT2 is used to generate the PWM signal for altering the volume.
// Its output for controlling the volume is on PORTB bit 7.
//
// Set TCNT3 to interrupt the processor at a rate of 1000 times a second. 
// When the interrupt occurs, the ISR for TCNTR3 changes the frequency of 
// timer TCNT1 to affect a continually changing audio at frequency at 
// PORTB bit 5.


#include <avr/io.h>
#include <avr/interrupt.h>

ISR(TIMER3_COMPA_vect) {
//The OCR1A values that work well are from 10000 to 65000
//the values should increment and decrement by about 64
//
  static uint16_t count=0;
  static uint8_t up=1;
    if(up==1){ 
        if(count < 62000){ count = count + 64;}
        else {count = count - 64; up=0;} 
    }  
    if(up==0){
        if(count > 10096){ count = count - 64 ;}
        else{count = count + 64; up= 1;}
    }
  OCR1A = count;
}
                                   

int main(){

  DDRB   |= (1<<PB5) | (1<<PB7);      //set port B bit five and seven as outputs

//setup TCNT1

  TCCR1A |= (1<<COM1A0);   			//CTC mode with output pin on 

  TCCR1B |= (1<<CS10) | (1<<WGM12);        //use OCR1A as source for TOP, use clk/1

  TCCR1C = 0x00;                          //no forced compare 

//setup TCNT3
// siren update frequency = (16,000,000)/(OCR3A) ~ set to about 1000 cycles/sec

  TCCR3A |= (1<<COM3A1)|(1<<COM3A0);      //CTC mode, output pin does not toggle 

  TCCR3B |= (1<<WGM32)|(1<<CS30);         //no prescaling      

  TCCR3C = 0x00;                     //no forced compare 

  OCR3A = 0x4055;                    //pick a speed from 0x1000 to 0xF000

  ETIMSK = (1<<OCIE3A);                          //enable timer 3 interrupt on OCIE3A
  //ETIFR =    (1<<OCF3A);
 //TCNT2 setup for providing the volume control
 //fast PWM mode, TOP=0xFF, clear on match, clk/128
 //output is on PORTB bit 7 
 TCCR2 =  (1<<WGM21) | (1<<WGM20) | (1<<COM21) | (1<<COM20) | (1<<CS20) | (1<<CS21);
 OCR2  = 0xE0;  //set to adjust volume control 

  sei();     //set GIE to enable interrupts
  while(1){} //do forever
 
}  // main
