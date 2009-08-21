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

//#include "fontset0.h"


// PORT A 
#define SCLCK_OUT 0 ///< Serial clock ouput to next module
#define SD_OUT    1 ///< Serial data ouput to next module
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
#define HW_SPI 0  ///< set to 1 to use the fast inbuild USI interface 150x faster.

#define SOUT_DELAY_US 15 ///< us to wait in the clock cycle

//debug macros
#define DEBUG_TRIGGER PINB = (1<<DEBUG)

#define BUFFERSIZE 32  ///< Size of ring buffer should be a power of 2
volatile uint8_t buffer[BUFFERSIZE]; ///< Buffer to contain the 256 bits on the screen.
volatile uint8_t buffer_index; ///< pointer to current start of buffer.
volatile uint16_t serial_in_buffer;
volatile uint16_t serial_out_buffer;
volatile uint8_t serial_in_index;
volatile uint8_t sin_flag;
//uint8_t screen_buffer[256]; ///< Screen buffer one byte per pixel.


/**
 * Initalize the I/O pins 
 *
 */
void init_pins (void)
{
    PORTA = 0x00; // Set Port A to off / High Z
    PORTB = 0x00; // Set port B to off / High Z
    
    // set Ouptut enables and clear pins High (active low)
    PORTA = (0<<SCLCK_OUT)|(0<<SD_OUT)|(0<<SD_IN)|(1<<OE4)|(1<<OE3)|(1<<OE2)|(0<<RCK)|(1<<SCL);
    PORTB = (0<<SCLCK_IN)|(1<<OE1)|(0<<SCK)|(0<<MISO)|(0<<SERIAL)|(1<<DEBUG);
    
    // Set the direction of pins on Port A 
    DDRA = (1<<SCLCK_OUT)|(1<<SD_OUT)|(0<<SD_IN)|(1<<OE4)|(1<<OE3)|(1<<OE2)|(1<<RCK)|(1<<SCL);
    
    // Set the direction of pins on Port B
    DDRB = (0<<SCLCK_IN)|(1<<OE1)|(1<<SCK)|(0<<MISO)|(1<<SERIAL)|(1<<DEBUG);
    
    
}



/**
 * Initalize the Serial bus
 *
 * The bus is used to talk to the shiftregisters. Once the 4 bytes have been 
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
 * This will send some pre arranged signals onthe pins to check the PCB is in 
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
    
    // rising edge generates an interupt 
  //  MCUCR |= (1<<ISC01)|(1<<ISC00);
    
    // any edge generates an interupt 
    MCUCR |= (0<<ISC01)|(1<<ISC00);
    
    
    // INT0 only
    GIMSK = (0<<INT1)|(1<<INT0)|(0<<PCIE1)|(0<<PCIE0);
    
    // enable global interuoptrs
    sei();
}

/** 
 * Serial input ISR
 *
 * This is triggered when a new edge has been detected on the input serial clock 
 * line. The bit will be clocked into the current buffer once 16 bits have been 
 * recieved they will be move into the main buffer and the old bits clocked 
 * out the serial out interface.
 * This simultaneously clocks out old data 
 * MSB is first.
 */
ISR(INT0_vect){

    // look for a rising edge
    if (PINB & (1<<SCLCK_IN)){ 
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
    }else{ // A falling edge
        PORTA |= (1<<SCLCK_OUT);                   // set outgoing clock line

    }        
    
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
        temp <<= 1;                                // shift allong by one
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
        temp <<= 1;                          // shift allong by one
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
 * Disable the output from the shiftregisters
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
 * from the data register to the actual ouputs
 */
void clock_new_ouput(void)
{
    PINA = (1<<RCK); // Toge this pin 0 ->1  
    PINA = (1<<RCK); // Toge this pin 1->0  
}

/**
 * Display a test pattern on the screen.
 *
 * This will display a test pattern on the screen to ensure that all of the LEDs
 * are operating nirmally.
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
 * This will draw the current buffer to the screen.
 */
void draw_screen(void)
{
    uint8_t i;
    uint8_t j;
    
    for (i=0;i<(BUFFERSIZE>>1);i++){
        j = buffer_index + (i<<1) ;
        j &= BUFFERSIZE-1 ; // limit to  size of buffer
        draw_col(i, buffer[j+0], buffer[j+1] );
    }
     draw_col(i,0,0);
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
           
            //      _delay_us(1);
        }
    }
}


/**
 * Simple scroling alphabet to test the screen
 */

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
////        _delay_ms(10);
//    }
//}
//



/**
 * Main operating loop for the software
 */
int main(void)
{
     uint8_t i;
    //
    // Hardware Initialization 
    //
    init_pins();
    init_usi();
    init_serial_input();
    for (i=0;i<32;i++){
        buffer[i]=i+1;
    }
    enable_output();
   // disable_ouput();
    //
    // Main Loop
    //

    for(;;){
        i++;
   
      //  test_pins();
      //  Send_byte_next_module(0);
      //  Send_byte_next_module(0); 

        
    //    _delay_ms(50);
        draw_screen();

 
            // have we got a new word?
            if (serial_in_index==0 ){
                
                //      Send_byte_next_module(serial_in_buffer>>8);
                //      Send_byte_next_module(serial_in_buffer & 0xff);
                
                
                // buffer[1] = serial_in_buffer>>8;
                // buffer[0] =  serial_in_buffer & 0xff;
                
                // send out last items in buffer
               serial_out_buffer = buffer[((buffer_index+0)&(BUFFERSIZE-1))]<<8;
               serial_out_buffer += buffer[((buffer_index+1)&(BUFFERSIZE-1))];
                
                // replace with new bytes
                buffer[((buffer_index+0)&(BUFFERSIZE-1))] = serial_in_buffer>>8;
                buffer[((buffer_index+1)&(BUFFERSIZE-1))] = serial_in_buffer & 0xff;
                
                //shift buffer index
                buffer_index++;
                buffer_index++;
                
                //limit buffer index to 31
                buffer_index &= BUFFERSIZE-1;
                
                serial_in_buffer =0;
                serial_in_index =16;    
                
      
                
            }
            
        
    }
    
    return 0;   /* never reached */
}
