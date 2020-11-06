//Inclass Exercise (TCNT0 + Interrupts + SPI)
//R. Traylor 9.2.2020

//This exercise builds on our SPI skills and adds using interrupts triggered
//by timer counter zero to time the movement of the LED on the bar graph.  

//Placing different functions within an ISR provides a pseudo partitioning
//mechanism somewhat like a function call.  As such, you will see that the
//while(1){}; portion of the code is empty! This is not necessarily unusual 
//in embedded programming.

//Once again, edit an existing makefile accordingly for this exercise. Fill in 
//the missing code where the blank lines are.

//The bar graph display will work the same way as before with movement at 
//half-second intervals. 

//Make sure the top or bottom LEDs stay on for exactly one period. To make
//sure there are no interactions with your display, unplug your PORTA
//connector from the mega128 board. 

//Submit just this completed file to TEACH by the end of day (5pm).

// bar_graph_demo_skel.c 
// R. Traylor
// 9.2.20
// demos interrupts, counter timer zero and SPI, whoohoo!

// This code implements a timer interrupt to update the bar graph display
// at 0.5 second intervals. Every half second, a new data value is sent to 
// the bargraph via SPI. The value is displayed as single led in a climbing
// pattern.

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

uint8_t previousData;
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
//
void tcnt0_init(void){

  ASSR   |=  (1<<AS0);			 //ext osc TOSC

  TIMSK  |=  (1<<TOIE0);	 	//enable TCNT0 overflow interrupt

  TCCR0  |=  (1<<CS00); 		//normal mode, no prescale

}

void config_IO(void){
  DDRB |= 0x07;		 //Turn on SS, MOSI, SCLK
  
  DDRA = 0xFF;		//PortA to all output for 4 digits LED
  DDRB |= 0x80;	//PortB bit4->7 to control 4 digits LED
  DDRE |= 0xC0;  	//PortE bit6,7 for encoder CLK Inhibit, SH/LD!
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

  static uint8_t count_7ms = 0;        //holds 7ms tick count 

  static uint8_t display_count = 0x01; //holds count for display 

  count_7ms++;                //increment count every 7.8125 ms
 
	PORTE &= ~(0x40);
	PORTE |=   0x80 ;
    SPDR = previousData;              	//send to bar graph display 
    while(bit_is_clear(SPSR, SPIF)){};               //wait till data sent out (a while loop)
    previousData = SPDR;

	PORTE |=   0x40;
	PORTE &= ~(0x80);
    PORTB |= 0x01;         	//HC595 output reg - rising edge...

    PORTB &= ~0x01;          	//and falling edge

}

/***********************************************************************/
//                                main                                 
/***********************************************************************/
int main(){     
tcnt0_init();  //initalize counter timer zero
config_IO();
spi_init();    //initalize SPI port
sei();         //enable interrupts before entering loop
while(1){}     //empty main while loop

} //main
