//Inclass Exercise (SPI)
//R. Traylor 9.2.2020
//
//This exercise works out our understanding of SPI and the peripherials
//that are used with it. It would be good to refresh yourself with the
//operation of the 74HC595 IC that drives the LEDs in the bar graph display.
//
//If you have already built the bar graph display onto the front panel,
//you are ready to start.  You will need to copy a makefile you have
//used before and change the file name accordingly. Fill in the missing
//code where the blank lines are.

//The way the bar graph is to work is that at every half-second interval, 
//a new LED is lit on the bargraph display via SPI. Only one LED is on at 
//a time and it "walks" from the bottom of the display to the top of the display,
//then back to the bottom and repeats. For fun you could make it "walk"
//bottom to top, then top to bottom and repeat. Its up to you.

//Make sure the top or bottom LEDs stay on for exectly one period. To make
//sure there are no interactions with your display, unplug your PORTA
//connector from the mega128 board. You will need the PORTB connector
//for the SPI connections.

//Submit just this completed file to Canvas by the end of day (5pm).

// bar_graph_noints_skel.c 
// R. Traylor
// 9.2.20

// Expected Connections:
// Bargraph board            mega128
// --------------       --------------------     
//     reglck           PORTB bit 0 (SS_n)                       
//     srclk            PORTB bit 1 (sclk)
//     sdin             PORTB bit 2 (mosi)
//     oe_n             ground 
//     gnd              ground 
//     vdd              vcc   
//     sd_out           no connect

#include <avr/io.h>
#include <util/delay.h>

//***********************************************************************
//                            spi_init                               
//**********************************************************************
void spi_init(void){

  DDRB   = 0x07;		 //output mode for SS, MOSI, SCLK

  SPCR   = (1<<SPE) | (1<<MSTR); //master mode, clk low on idle, leading edge sample

  SPSR   = (1<<SPI2X);		 //choose double speed operation

 }//spi_init

//**********************************************************************
//                                main                                 
//**********************************************************************
int main(){     

uint8_t display_count = 1; //holds count for display; initalize it

uint8_t i;  //dummy counter

spi_init(); //initalize SPI port

while(1){                             //main while loop

    SPDR = display_count;  //send display_count to the display 

    while (bit_is_clear(SPSR,SPIF)){} //spin till SPI data has been sent 

    PORTB = PORTB | 0x01;              //send rising edge to regclk on 74HC595 
       
    PORTB = PORTB ^ 0x07;              //send falling edge to regclk on 74HC59

    display_count = (display_count<<1);  //shift display_count for next time 

    if(display_count == 0x80){display_count = 1;} //put bit back into the "1st" positon

    for(i=0; i<=4; i++){_delay_ms(100);}      //0.5 sec delay
  } //while(1)
} //main
