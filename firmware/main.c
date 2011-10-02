/**
 * @file    main.c
 * @author  Nick Lott <nick.lott@gmail.com>
 * @version 1.0
 *
 * @section LICENSE
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details at
 * http://www.gnu.org/copyleft/gpl.html
 *
 * @section DESCRIPTION
 *
 * The Main operating loop for the LED_MATRIX MODULE
 */

/* Source Control Keywords
$Id$
$Header$
$Date$
$DateTime$
$Change$
$File$
$Revision$
$Author$
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>

#include "fontset0.h"


// PORT A 
#define SCLCK_OUT 0 ///< Serial clock output to next module
#define SD_OUT    1 ///< Serial data output to next module
#define SD_IN     2 ///< Serial data input from previous module 
#define OE4       3 ///< Output enable 4th shift register
#define OE3       4 ///< Output enable 3rd shift register
#define OE2       5 ///< Output enable 2nd shift register
#define RCK       6 ///< Register Clock for the shift registers
#define SCL       7 ///< Master Clear for all SRs

// PORT B
#define SERIAL      0  ///< Serial out to Shift Registers (USI)
#define MISO        1  ///< MISO for SPI Bus
#define SCK         2  ///< SCK for shift registers (USI)
#define OE1         3  ///< Output enable 1st shift register
#define DEBUG       4  ///< Debug trigger
#define UNUSED_PB5  5  ///< Not used
#define SCLCK_IN    6  ///< clk from previous module (INT0)
#define UNUSED_PB7  7  ///<  used by the Reset pin

// For FAST USI
#define USICLK_L  (1<<USIWM0)|(0<<USICS0)|(1<<USITC)
#define USICLK_H  (1<<USIWM0)|(0<<USICS0)|(1<<USICLK)|(1<<USITC)
#define HW_SPI 0  ///< set to 1 to use the fast inbuilt USI interface 150x faster.

#define SOUT_DELAY_US 10 ///< us to wait in the clock cycle

#define FRAME_RESET_TIMEOUT 50 ///< number of screen refresh frames to wait before reseting serial interface
							   // This calculates to 20mS if refresh timer is running at 2.5kHz

//debug macros
/// Toggle the DEBUGPIN on port B
/// From page 55 of datasheet for ATTiny261 (2588E–AVR–08/10)
/// "writing a logic one to a bit in the PINx Register, will result in a toggle in the corresponding
/// bit in the Data Register"
#define DEBUG_TRIGGER PINB = (1<<DEBUG)


#define BUFFERSIZE 32  				 		///< Size of ring buffer should be a power of 2
volatile uint8_t buffer[BUFFERSIZE]; 		///< Buffer to contain the 256 bits on the screen.
volatile uint8_t buffer_index;       		///< pointer to current start of buffer.
volatile uint16_t serial_in_buffer;  		///< buffer for next word currently being clocked in
volatile uint16_t serial_out_buffer; 		///< buffer for last word currently being clocked out
volatile uint8_t serial_in_index;    		///< bit of word currently being clocked in or out of the module
// volatile uint8_t sin_flag;
//uint8_t screen_buffer[256];		 		 ///< Screen buffer one byte per pixel. // whoops only 128Bytes on a ATtiny261 need a Attiny861
volatile uint8_t timer_flag; 		 	 	 ///< flag to indicate the regular refresh timer has elapsed
volatile uint16_t frame_timeout_counter=0; 	 ///< counter how many timer periods have elapsed since last edge
volatile uint8_t serial_in_buffer_full_flag; ///< flag to indicate a word has been received on input


/**
 * Initialise the I/O pins
 *
 */
void init_pins (void)
{
    PORTA = 0x00; // Set Port A to off / High Z
    PORTB = 0x00; // Set port B to off / High Z
    
    // set shiftregister output-enable and clear pins High (active low)
    PORTA = (0<<SCLCK_OUT)|(0<<SD_OUT)|(0<<SD_IN)|(1<<OE4)|(1<<OE3)|(1<<OE2)|(0<<RCK)|(1<<SCL);
    PORTB = (0<<SCLCK_IN)|(1<<OE1)|(0<<SCK)|(0<<MISO)|(0<<SERIAL)|(1<<DEBUG);
    
    // Set the direction of pins on Port A 
    DDRA = (1<<SCLCK_OUT)|(1<<SD_OUT)|(0<<SD_IN)|(1<<OE4)|(1<<OE3)|(1<<OE2)|(1<<RCK)|(1<<SCL);
    
    // Set the direction of pins on Port B
    DDRB = (0<<SCLCK_IN)|(1<<OE1)|(1<<SCK)|(0<<MISO)|(1<<SERIAL)|(1<<DEBUG);
}


/** 
 * Initialise the Timer
 *
 * The timer is timer0 and uses the 8Mhz system clock to provide a 50 Hz update 
 * rate for the screen.
 */
void init_timer0(void)
{
    
    // Timer Counter 0 control register A
    // 16 bit mode
    TCCR0A = (1<<TCW0)|(0<<ICEN0)|(0<<ICNC0)|(0<<ICES0)| (0<<ACIC0)|(0<<WGM00);
    
    // Timer Conter 0 Interrupt Mask Register
    // enable ouput compare A interrupt.
    TIMSK  = (1<<OCIE0A); 
    
    // TimerCounter0 control register B
    // reset the counter and set source to system clock with no prescaling.
    TCCR0B  = (0<<TSM)|(1<<PSR0)|(0<<CS02)|(0<<CS01)|(1<<CS00);
    
    // set counter to 0x1f40 or 8000 counts this should give a 1ms interval 
    // set counter to 0x0320 for 800 counts -> 100uS interval
    // set counter to 0x0640 for 1600 counts -> 200uS interval
    // set counter to 0x0C80 for 3200 counts -> 400uS interval (2.5Khz /16  => 156H.25Hz Refresh
    OCR0B =0x0c;
    OCR0A =0x80 ;
    
    timer_flag = 0 ;
    
}

/**
 * Initialise the Serial bus
 *
 * The bus is used to talk to the shift registers. Once the 4 bytes have been
 * clocked out with SCK they must be clocked through with a +ve edge on RCK
 * if the OE is low then they will appear on the outputs. The USI can be used to 
 * clock the data out quickly.
 */
void init_usi(void)
{
    USICR = (0<<USISIE)|(0<<USIOIE)|(0<<USIWM1)|(1<<USIWM0)|(0<<USICS1)|(0<<USICS0)|(0<<USICLK)|(0<<USITC);
    
}


/** 
 * Test the basic operation of the pins
 *
 * This will send some predetermined signals on the pins to check the PCB is in
 * designed as expected by the software.
 *
 */
void test_pins(void)
{
    uint8_t i;
    //Toggle the SD_OUT PIN
    for(i=0;i<10;i++){ 
          PINA = (1<<SD_OUT); // toggle these bits
    }
    
    //Toggle the SD_OUT pin and SCLCK_OUT pin 
    for(i=0;i<10;i++){
          PINA = (1<<SD_OUT)|(1<<SCLCK_OUT);// toggle these bits
          PINB = (1<<SERIAL);
    }
 
}


/** 
 * setup the incoming Serial interface ISR
 */
void init_serial_input(void)
{
    serial_in_index = 16;
    serial_in_buffer = 0;
    buffer_index =0;
    
    // rising edge generates an interrupt
  //  MCUCR |= (1<<ISC01)|(1<<ISC00);
    
    // any edge generates an interrupt
    MCUCR |= (0<<ISC01)|(1<<ISC00);
    
    
    // INT0 only
    GIMSK = (0<<INT1)|(1<<INT0)|(0<<PCIE1)|(0<<PCIE0);
    
    // enable global interrupts
    sei();
}


/** 
 * Serial input ISR
 *
 * This is triggered when a new edge has been detected on the input serial clock 
 * line. The bit will be clocked into the current buffer once 16 bits have been 
 * Received they will be move into the main buffer and the old bits clocked
 * out the serial out interface.
 * This simultaneously clocks out old data 
 * MSB is first.
 *
 * This ISR takes 12-15us on a rising edge while serial_in_index>2
 * On last rising edge the ISR takes 24us
 * Falling edges take 1.4us to complete
 * There is a delay of 3.6us to enter the ISR
 */
ISR(INT0_vect){
	//cli();
	//DEBUG_TRIGGER;
    // look for a rising edge
    if (PINB & (1<<SCLCK_IN)){ 
    	if(serial_in_buffer_full_flag == 0){
			serial_in_index--;

			// set bit in buffer
			if (PINA & (1<< SD_IN) ){
				serial_in_buffer |= (1<<serial_in_index);
			}

			PORTA &= ~( (1<<SCLCK_OUT));  // clear clock

			// set/clear bit on outgoing serial
			if (serial_out_buffer & (1<<serial_in_index)){
				PORTA |= (1<<SD_OUT);                  // set data bit
			}else{
				PORTA &= ~(1<<SD_OUT);                 // clear data bit
			}
			// set the flag if the buffer is full.
			if (serial_in_index == 0){
				serial_in_buffer_full_flag = 1;
			}
    	}
    }else{ // A falling edge
        PORTA |= (1<<SCLCK_OUT);                   // set outgoing clock line
        frame_timeout_counter=0;
    }        
    //DEBUG_TRIGGER;
}


/**
 * Timer interrupt 
 *
 * This is use to update the screen periodically.
 * This ISR takes 768ns between debug triggers
 * Frequency is ~ 4.8Khz (208us period)
 */
ISR(TIMER0_COMPA_vect)
{
//    DEBUG_TRIGGER;
    TCNT0H=0;
    TCNT0L=0;
    timer_flag=1;
//    DEBUG_TRIGGER;
}




/**
 * Send a byte out the serial out port to next module
 *
 * This will clock out the data byte 1 bit at a time. The data is clocked out 
 * Most significant bit first (MSB) and on the rising edge of the clock.
 * The clock is active high and left low after last bit has been sent.
 */
void Send_byte_next_module( uint8_t data ///< 8 bit data to be sent
                           )
{
    uint8_t temp;
    uint8_t i ;
    
    temp = data;
    
    for (i=0;i<8;i++){
        PORTA &= ~(1<<SCLCK_OUT);                 // clear clock line
        _delay_us(SOUT_DELAY_US);
        
        if (temp & 0x80){
            PORTA |= (1<<SD_OUT);                  // set data bit
        }else{
            PORTA &= ~(1<<SD_OUT);                 // clear data bit
        }
        PORTA |= (1<<SCLCK_OUT);                   // set clock line
        temp <<= 1;                                // shift along by one
        _delay_us(SOUT_DELAY_US); 
        
    }
     PORTA &= ~( (1<<SCLCK_OUT)|(1<<SD_OUT) );     // clear clock and data line
}



/**
 * Send a single byte on the USI bus to the shift registers
 * 
 * This sends the 8 bit data byte out the SERIAL line and clocks the SCK with 
 * each byte. This routine has been optimised for speed.
 */
void usi_send_byte(uint8_t data ///< 8 bit data to be sent
                   )
{

    
#if (HW_SPI==1) 
    USIDR = data;
    USICR = USICLK_L; // bit 7 , MSB
    USICR = USICLK_H;
    USICR = USICLK_L; // bit 6
    USICR = USICLK_H;
    USICR = USICLK_L; // bit 5
    USICR = USICLK_H;
    USICR = USICLK_L; // bit 4
    USICR = USICLK_H;
    USICR = USICLK_L; // bit 3
    USICR = USICLK_H;
    USICR = USICLK_L; // bit 2
    USICR = USICLK_H;
    USICR = USICLK_L; // bit 1
    USICR = USICLK_H;
    USICR = USICLK_L; // bit 0 , LSB
    USICR = USICLK_H;
#else
    uint8_t temp;
    uint8_t i ;
    
    temp = data;
    
    for (i=0;i<8;i++){
        PORTB &= ~(1<<SCK);                  // clear clock line
        
        if (temp & 0x80){
            PORTB |= (1<<SERIAL);            // set data bit
        }else{
            PORTB &= ~(1<<SERIAL);           // clear data bit
        }
        
        PORTB |= (1<<SCK);                   // set clock line
        temp <<= 1;                          // shift along by one
    }
    PORTB &= ~( (1<<SCK)|(1<<SERIAL) );      // clear clock and data line
    
#endif
}


/**
 * Enable the Shift registers output
 */
void enable_output( void )
{
    PORTA &=~((1<<OE4)|(1<<OE3)|(1<<OE2));
    PORTB &=~(1<<OE1);
    
}

/**
 * Disable the output from the shift registers
 */
void disable_ouput(void)
{
    PORTA |=~((1<<OE4)|(1<<OE3)|(1<<OE2));
    PORTB |=~(1<<OE1);
}

/** 
 * Shift the data to the ouputs of the shift registers
 *
 * Cycle the RCK (register clock) on the shift register to latch the data 
 * from the data register to the actual outputs
 */
void clock_new_ouput(void)
{
    PINA = (1<<RCK); // Toggle this pin 0 ->1
    PINA = (1<<RCK); // Toggle this pin 1->0
}

/**
 * Display a test pattern on the screen.
 *
 * This will display a test pattern on the screen to ensure that all of the LEDs
 * are operating normally.
 */
void test_all_LEDs(void)
{
    _delay_ms(100);    
    usi_send_byte(0xff);
    usi_send_byte(0x00);
    usi_send_byte(0xff);
    usi_send_byte(0x00);
    clock_new_ouput();
    _delay_ms(100);    
    usi_send_byte(0xff);
    usi_send_byte(0x00);
    usi_send_byte(0x00);
    usi_send_byte(0xff);
    clock_new_ouput();
    _delay_ms(100);    
    usi_send_byte(0x00);
    usi_send_byte(0xff);
    usi_send_byte(0x00);
    usi_send_byte(0xff);
    clock_new_ouput();
    _delay_ms(100);    
    usi_send_byte(0x00);
    usi_send_byte(0xff);
    usi_send_byte(0xff);
    usi_send_byte(0x00);
    clock_new_ouput();
}

/**
 * Reverse a single byte 
 *
 * this was adapted from the example at 
 * http://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious
 */
uint8_t reverse_byte (uint8_t byte ///< byte to reverse
                   )
{
    uint8_t v=byte;
    uint8_t r=v ; // local storage for building the byte
    uint8_t s = sizeof(v) * 8 - 1; // extra shift needed at end

    for (v >>= 1; v; v >>= 1) {   
        r <<= 1;
        r |= v & 1;
        s--;
    }
    r <<= s; // shift when v's highest bits are zero

    return r;
}

/**
 * Draw a single column of the screen
 *
 * sends 4 bytes out to the SPI port to the display. takes approx 80uS for slow 
 * speed operation if HW USI is used it is closer to 23uS
 */
void draw_col(uint8_t column, ///< column to draw 0..15
              uint8_t upper_byte, ///< byte for upper half of column
              uint8_t lower_byte  ///< byte for lower half of column
              )
{
    uint16_t cola; 
    uint16_t colb;
    
    colb = ~(1<<( column ) >> 8);
    cola = ~(1<<(15-column)>>8);
    usi_send_byte(cola);
    usi_send_byte(colb);
    usi_send_byte(reverse_byte( lower_byte));
    usi_send_byte( upper_byte );
    clock_new_ouput();
}


/**
 * Draw the buffer to the screen
 * 
 * This will draw a single column of the current buffer to the screen. The 
 * column count will be incremented. 
 */
void update_screen(void)
{
    static uint8_t i=0;
    static uint8_t cnt =0;
    uint8_t j;
    uint8_t k;
    
    if (cnt++){
        cnt=0;
       draw_col(i,0,0);   
    }else{
        if (i==(BUFFERSIZE>>1)){
            i =0;
        }else{
            i++;
        }
        
        j = buffer_index + (i<<1) ;
        j &= BUFFERSIZE-1 ; // limit to  size of buffer and roll any overflow
        k  = j+1 &(BUFFERSIZE-1);// limit to  size of buffer and roll any overflow
        draw_col(i, buffer[j], buffer[k] );
    }
}


/**
 * Draw the buffer to the screen  - Test brightness
 *
 * This will draw the current buffer and apply a brightness gradient from left 
 * to right for testing purposes
 */
void draw_screen_test(void)
{
    uint8_t i;
    uint8_t j;
    
    for (i=0;i<16;i++){
        for (j=0;j<16;j++){
             _delay_ms(10);
            if (i<=j){
                draw_col(i, buffer[(i<<1)], buffer[(i<<1)+1] );
            }else{
                draw_col(i,0,0);
            }

        }
    }
}


///**
// * Simple scrolling alphabet to test the screen
// */
//
//void test_fontSet0 (void)
//{
//    static uint16_t character=200;
//    static uint8_t count;
//
//    uint8_t i ;
//    // uint8_t i;
//
//    if (count++==50) {
//        character--;
//        count=0;
//    }
//
//    for (i=0;i<16;i++){
//        draw_col(i,pgm_read_byte(fontSet0+character+i),pgm_read_byte(fontSet0-character+i));
//	       _delay_ms(10);
//    }
//}



/**
 * Insert a word into the screen buffer
 *
 * Takes the last word received (serial_in_buffer) and puts it into the buffer
 * the index pointer is moved along by two bytes to account for the two new 
 * bytes in the the buffer.
 */
void insert_word_into_buffer(void)
{


    // send out second to last items in buffer
    serial_out_buffer = buffer[((buffer_index+2)&(BUFFERSIZE-1))]<<8;
    serial_out_buffer += buffer[((buffer_index+3)&(BUFFERSIZE-1))];
    
    // replace last items in buffer with new bytes
    buffer[((buffer_index+0)&(BUFFERSIZE-1))] = serial_in_buffer >>8;
    buffer[((buffer_index+1)&(BUFFERSIZE-1))] = serial_in_buffer & 0xff;
    
    //shift screen buffer index
    buffer_index++;
    buffer_index++;
    
    //limit screen buffer index to 31
    buffer_index &= BUFFERSIZE-1;
    
    // reset the input buffer for next word.
    serial_in_buffer =0;
    serial_in_index =16;
    serial_in_buffer_full_flag =0;
}



/**
 * Main operating loop for the software
 */
int main(void)
{
	uint8_t last_time_serialIndex=0;


	//last_time_serialIndex
	// Hardware Initialisation
	//
	init_pins();
	init_usi();
	init_serial_input();
	init_timer0();
	enable_output();

	//
	// Main Loop
	//
	for(;;){

		// Is it time to update the screen?
		if (timer_flag){
			timer_flag = 0;   // reset the timer flag
			update_screen();  // draw a single column of the screen buffer

			if (frame_timeout_counter++ >FRAME_RESET_TIMEOUT){
				serial_in_buffer_full_flag =0; 	// open the buffer again
				serial_in_index = 16;      		// reset buffer word bit pointer
				serial_in_buffer = 0x0000; 		// clear buffer word
				frame_timeout_counter=0;

//				serial_in_index--; /// Simulate accidental clock signal

//				serial_in_buffer_full_flag =1;
//				serial_in_index = 16; // reset buffer word bit pointer
//				serial_in_buffer = 0x5555; //clear buffer word
			}

		}
		//serial_in_index = 16; // reset buffer word bit pointer
		//serial_in_buffer = 0; //clear buffer word
		// have we got a new word?
		if (serial_in_buffer_full_flag == 1){
			insert_word_into_buffer(); //save the word that just came in.
		}



		last_time_serialIndex = serial_in_index;
	}

	return 0;   /* never reached */
}
