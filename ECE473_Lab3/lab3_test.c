// ECE 473 Lab 3
// R. Traylor
// 9.12.08
// Modified in October 2016 by Nick McComb | www.nickmccomb.net

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

//  BARGRAPH SETUP:
//  regclk   PORTB bit 0 (SS_n)
//  srclk    PORTB bit 1 (SCLK)
//  sdin     PORTB bit 2 (MOSI)
//  oe_n     GND

//  ENCODER SETUP
//  sck         PORTB bit 1
//  ENC_CLK_INH PORTE bit 6
//  ENC_SH/LD   PORTE bit 7

//TODO: CHECK WITH STEVEN ABOUT OVERFLOW

//#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stddef.h>

//Program controls
//#define LEADING_0  //Whether or not you want leading zeros

//Segment pin definitions
#define SEG_A  0x01
#define SEG_B  0x02
#define SEG_C  0x04
#define SEG_D  0x08
#define SEG_E  0x10
#define SEG_F  0x20
#define SEG_G  0x40
#define SEG_DP 0x80

//PORTB Definitions
#define DIG_SEL_1 0x10
#define DIG_SEL_2 0x20
#define DIG_SEL_3 0x40
#define PWM_CTRL 0x80

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12];

//Globals
uint16_t counter = 0;
//TODO: Add rest of globals


//Function Prototypes
void inline incrementCounter( void );
void inline decrementCounter( void );
void setDigit( uint8_t targetDigit );
void configureIO( void );
void configureTimers( void );
void configureSPI( void );
void inline setSegment( uint16_t targetOutput );
void inline clearSegment( void );
void processButtonPress( void );
void processCounterOutput( void );
void inline checkButtons( void );
void inline updateSPI( void );
void processEncoders( void );

//TODO: Move rest of function decs

//TODO: Remove all of this shit
uint8_t randoTest = 0;
uint8_t inc2Bool = 0x00;
uint8_t inc4Bool = 0x00;

//Digit control low-level code
void inline SET_DIGIT_ONE(void)   {PORTB |= DIG_SEL_3; PORTB = PORTB & ~(DIG_SEL_1 | DIG_SEL_2);}
void inline SET_DIGIT_TWO(void)   {PORTB |= DIG_SEL_1 | DIG_SEL_2; PORTB = PORTB & ~(DIG_SEL_3);}
void inline SET_DIGIT_THREE(void) {PORTB |= DIG_SEL_1; PORTB = PORTB & ~(DIG_SEL_2 | DIG_SEL_3);}
void inline SET_DIGIT_FOUR(void)  {PORTB = PORTB & ~(DIG_SEL_1 | DIG_SEL_2 | DIG_SEL_3);}

//Tri-State Buffer Enable
void inline ENABLE_BUFFER(void)   {PORTB |= DIG_SEL_1 | DIG_SEL_2 | DIG_SEL_3;}

//Port A Control
void inline ENABLE_LED_CONTROL(void) {DDRA = 0xFF; SET_DIGIT_THREE(); PORTB |= DIG_SEL_3;} //Enables PORTA as an output, while also ensuring the Tri-state buffer is disabled by selecting digit one
void inline ENABLE_BUTTON_READ(void) {DDRA = 0x00; PORTA = 0xFF;}  //Enable inputs/pullups on PORTA

void inline ENC_CLK_ENABLE(void)  {PORTE &= ~(0x40);}
void inline ENC_CLK_DISABLE(void) {PORTE |=   0x40 ;}

void inline ENC_PARALLEL_ENABLE(void)  {PORTE &= ~(0x80);}
void inline ENC_PARALLEL_DISABLE(void) {PORTE |=   0x80 ;}

//Parsed commands from the encoders (parsed to one call per detent)
void inline ENC_L_COUNTUP(void)   {incrementCounter();}
void inline ENC_L_COUNTDOWN(void) {decrementCounter();}
void inline ENC_R_COUNTUP(void)   {incrementCounter();}
void inline ENC_R_COUNTDOWN(void) {decrementCounter();}

//NOP delay
#define NOP() do { __asm__ __volatile__ ("nop"); } while (0)

//Global Variables
uint32_t output[5]; //Note, this is zero indexed for digits!!! The 0 index is for the colon
int16_t  lastEntered = 0;
int16_t  debounceCounter = 0;
uint8_t  unpressed = 1;
uint8_t  lastEncoderValue = 0x13;
uint8_t  upToDateEncoderValue = 0;  //Holds whether the encoder value is a newly measured value
uint8_t  bargraphOutput = 0;

//Configures the device IO (port directions and intializes some outputs)
void configureIO( void ){

  ENABLE_LED_CONTROL(); 

  //DDRA = 0xFF; //Initialize DDRA as if we want to control the LEDs
  DDRB = 0xF0; //Upper nibble of the B register is for controlling the decoder / PWM Transistor

  DDRB |= 0x07;  //Setup the SPI pins as outputs

  //For this lab, we are just driving the PWM_CTRL line low always
  //PORTB |= PWM_CTRL;  
  //(it defaults to low)
  uint8_t i;

  //Init output to 0
  for(i = 0; i < 5; ++i){
    output[i] = 0;
  }

  DDRE |= 0xC0;  //Enable Clk inhibit pin and async pin as outputs
  ENC_CLK_DISABLE();
  ENC_PARALLEL_ENABLE();

}

//Configures all timer/counters on the device
void configureTimers( void ){
  //Timer 0 configure: Polling buttons
  TIMSK |= (1<<TOIE0); //Enable overflow interrupts
  TCCR0 |= (1<<CS02) | (1<<CS01) | (0<<CS00);  //Normal mode, prescale 

  //OCR0 Output Compare Register

}

//Timer 0 overflow vector
//Polls the buttons / interfaces with SPI
ISR(TIMER0_OVF_vect){

  checkButtons();

  updateSPI();
  
  processEncoders();
}

//Setup SPI on the interface
void configureSPI( void ){

  //Configure SPI
  //Master mode, clk low on idle, leading edge sample
  SPCR = (1 << SPE) | (1 << MSTR) | (0 << CPOL) | (0 << CPHA);   

  //Chose double speed operation
  //SPSR = (1 << SPI2X);


}

//Outputs the proper segment based on the input number
//Note: This function only currently supports 0-9 (as alphas were not needed for the assignment)
void inline setSegment( uint16_t targetOutput ){
  switch(targetOutput){
     case 0:
       PORTA = ~(SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F);
       break;
     case 1:
       PORTA = ~(SEG_B | SEG_C);
       break;
     case 2:
       PORTA = ~(SEG_A | SEG_B | SEG_D | SEG_E | SEG_G);
       break;
     case 3:
       PORTA = ~(SEG_A | SEG_B | SEG_C | SEG_D | SEG_G);  //Changed G to E
       break;
     case 4:
       PORTA = ~(SEG_B | SEG_C | SEG_F | SEG_G);
       break;
     case 5:
       PORTA = ~(SEG_A | SEG_C | SEG_D | SEG_F | SEG_G);
       break;
     case 6:
       PORTA = ~(SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G);
       break;
     case 7:
       PORTA = ~(SEG_A | SEG_B | SEG_C);
       break;
     case 8:
       PORTA = ~(SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G);
       break;
     case 9:
       PORTA = ~(SEG_A | SEG_B | SEG_C | SEG_F | SEG_G);
       break;
     case 10: //A
       break; 
     case 11: //B
       break;
  }
}

//Clears the segments so nothing is being outputted on the port
void inline clearSegment( void ){
  PORTA = 0xFF;
}

//Sets the decoder to choose the appropriate transistor for the appropriate digit. 
//It also sets the appropriate segment outputs.
//NOTE: There is an inherient 100uS delay with any call of this function
void setDigit( uint8_t targetDigit ){ 
  switch(targetDigit){

    clearSegment();

    case 1:
      SET_DIGIT_ONE();
      _delay_us(100);
      if(counter < 1000)
        clearSegment();
      else
        setSegment(output[1]);
      break;
    case 2:
      SET_DIGIT_TWO();
      _delay_us(100);
      if(counter < 100)
        clearSegment();
      else
        setSegment(output[2]);
      break;
    case 3:
      SET_DIGIT_THREE();
      _delay_us(100);
      if(counter < 10)
        clearSegment();
      else
        setSegment(output[3]);
      break;
    case 4:
      SET_DIGIT_FOUR();
      _delay_us(100);
      setSegment(output[4]);
      break;
  }

  //clearSegment();

}

//This function is called when a button is pressed, and handles processing the press, as well as
//changing tne numbers that are to be outputted.
void processButtonPress( void ){
  //counter += 0xFF - PINA;

  //if(counter >= 1024){
  //  counter -= 1023;
  //}

  uint8_t temp = 0xFF - PINA;

  switch(temp){
    case 0x01:
      inc2Bool ^= 0x01;
      bargraphOutput ^= (1 << 0);
      break;
    case 0x02:
      inc4Bool ^= 0x01;
      bargraphOutput ^= (1 << 1);
      break;

  }

}  
 
//This function processess the output of the counter variable.
//It checks for overflow conditions, and then calculates the numbers to be outputted on each 7 segment digit
void processCounterOutput( void ){
  //We want to check for overflow/underflow here
  if(counter < 10000 && counter > 1023) //Check for simple overflow
    counter = (counter % 1024) + 1;
 
  if(counter > 10000) //Check for overflow, because variable is a uint, it will wrap around
    counter = 1023;

  //We want to calculate the presses here, and not every time, as they can take some time,
  //and the user will be more tolerable of a slight sutter at a button press, but not every
  //execution cycle. (In theory. In practice, this math will be unnoticable). 
  uint16_t tempCounter = counter;
  //calculate new output values
  output[4] = tempCounter % 10;
  tempCounter /= 10;
  output[3] = tempCounter % 10;
  tempCounter /= 10;
  output[2] = tempCounter % 10;
  tempCounter /= 10;
  output[1] = tempCounter % 10;

}

//Checks the buttons when called, and calls a seperate processing function once the buttons have been debounced
//It will call it only once per button press, and resets upon button release.
void inline checkButtons( void ){
  ENABLE_BUTTON_READ();
  ENABLE_BUFFER();
  _delay_us(5); //Essentially a nop? No way. Not a nop. Dear god not at all. Same principle, though. Wait for voltages to settle.

  //Latching button debounce
  //The delay from the for loop at the beginning of this while(1) block will handle
  //most of the important debouncing delay, so we can just use a latch here.
  if(PINA != 0xFF){ //If the buttons read anything
    if(unpressed){
      processButtonPress();
      unpressed = 0; //Latches the button press
    }
    else if(PINA == lastEntered){ //Don't preform any action
      ++debounceCounter;
    }
    else if(PINA != lastEntered){
      processButtonPress();
      debounceCounter = 1;
    }

    lastEntered = PINA;
  }
  else {
    unpressed = 1;  //Release the latch
  }

  ENABLE_LED_CONTROL();
  //_delay_us(20);  //Delay to allow voltages to settle
  
}

//Writes out to the bar graph AND reads in from the encoder!
void inline updateSPI( void ){
  
  ENC_CLK_ENABLE();        //Allow us to read in serial data
  ENC_PARALLEL_DISABLE();  //Allow us to read in serial data

  NOP();
  NOP();

  SPDR = bargraphOutput;
  lastEncoderValue = SPDR;

  //Wait for SPI operation
  while (bit_is_clear(SPSR, SPIF)){};

  upToDateEncoderValue = 1;

  ENC_CLK_DISABLE();
  ENC_PARALLEL_ENABLE();

  //Output the bar graph info
  PORTB |=  0x01;
  PORTB &= ~0x01;

}

//Processes commands from the encoders
void processEncoders( void ){
  uint8_t static lEncoderPrev = 0;
  uint8_t static rEncoderPrev = 0;
  uint8_t static lEncoder = 0;
  uint8_t static rEncoder = 0;
  
  lEncoderPrev = lEncoder;
  rEncoderPrev = rEncoder;

  //Save previous values

  lEncoder =  (lastEncoderValue & 0x03);
  rEncoder = ((lastEncoderValue & 0x0C) >> 2);

  //bargraphOutput = rEncoder;

  //Check if the values have changed, if so process them
  if(lEncoder != lEncoderPrev){
    if((lEncoderPrev == 0x01) && (lEncoder == 0x03))
      ENC_L_COUNTUP();
    if((lEncoderPrev == 0x02) && (lEncoder == 0x03))
      ENC_L_COUNTDOWN();
  }

  if(rEncoder != rEncoderPrev){
    if((rEncoderPrev == 0x01) && (rEncoder == 0x03))
      ENC_R_COUNTUP();
    if((rEncoderPrev == 0x02) && (rEncoder == 0x03))
      ENC_R_COUNTDOWN();
  }

  
}

//Called to increment the counter variable
void inline incrementCounter( void ){
  if(inc2Bool & inc4Bool)
    NOP();
  else if (inc2Bool)
    counter += 2;
  else if (inc4Bool)
    counter += 4;
  else
    counter += 1;
    
}

//Called to decrement the counter variable
void inline decrementCounter( void ){
  if(inc2Bool & inc4Bool)
    NOP();
  else if (inc2Bool)
    counter -= 2;
  else if (inc4Bool)
    counter -= 4;
  else
    counter -= 1;
}


//Main function call
int main()
{
//set port bits 4-7 B as outputs
while(1){
  configureIO();
  configureTimers();
  configureSPI();
  sei();


  uint8_t temp_counter = 1;
  uint8_t tempBool = 0x01;

  int j, k;

  //uint8_t counter = 0;

  ENABLE_LED_CONTROL();

  while(1){  //Main control loop
    for(k = 0; k < 15; ++k){
      for(j = 1; j < 5; ++j){
        //clearSegment();
        _delay_us(50);
	
	setDigit(j);  //Contains 100uS delay
	
        _delay_us(130); //Lowest tested to be 750uS because of light bleed, can recomfirm

        clearSegment();

      }
    }
	
    processCounterOutput();  //Doesn't have to happen all of the time, so it's called here.

  }
  
  }
}
