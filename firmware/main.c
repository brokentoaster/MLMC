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


#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>
#include <avr/sleep.h>

#include "fontset0.h"


// PORT A  
#define SCLCK_OUT   0  ///< Serial clock output to next module
#define SD_OUT      1  ///< Serial data output to next module
#define SD_IN       2  ///< Serial data input from previous module 
#define OE4         3  ///< Output enable 4th shift register
#define OE3         4  ///< Output enable 3rd shift register
#define OE2         5  ///< Output enable 2nd shift register
#define RCK         6  ///< Register Clock for the shift registers
#define SCL         7  ///< Master Clear for all SRs

// PORT B
#define SERIAL      0  ///< Serial out to Shift Registers (USI)
#define MISO        1  ///< MISO for SPI Bus
#define SCK         2  ///< SCK for shift registers (USI)
#define OE1         3  ///< Output enable 1st shift register
#define DEBUG       4  ///< Debug trigger
#define UNUSED_PB5  5  ///< Not used
#define SCLCK_IN    6  ///< clk from previous module (INT0)
#define UNUSED_PB7  7  ///<  used by the Reset pin


/// Debug macros:
// Toggle the DEBUGPIN on port B
// From page 55 of datasheet for ATTiny261 (2588E–AVR–08/10)
// "writing a logic one to a bit in the PINx Register, will result in a toggle in the corresponding
// bit in the Data Register"
#define DEBUG_TRIGGER PINB = (1<<DEBUG)


// For FAST USI
// Cannot use the HW USI as PB0 is DI not DO as expected.
// with a mod it may work but would break programming. (works)
// TODO: Correct this on V2 of the PCB
#define USICLK_L  (1<<USIWM0)|(0<<USICS0)|(1<<USITC)
#define USICLK_H  (1<<USIWM0)|(0<<USICS0)|(1<<USICLK)|(1<<USITC)
#define HW_SPI 0                ///< set to 1 to use the fast inbuilt USI interface 5x faster. 7uS vs 32us
#define SW_SPI_FAST 0			///< use a nasty looking but as fast as possible SPI routine 20uS vs 32us
#define REVERSE_BYTE_USE_LOOKUP 0	///< use a 256 byte lookup tabel instead of calculating the reversed bit ordered byte

#define SOUT_DELAY_US 10        ///< us to wait in the clock cycle for Send_byte_next_module routine

#define FRAME_RESET_TIMEOUT 4 	///< number of screen refresh frames to wait before reseting serial interface
							    ///< This calculates to 20mS if refresh timer is running at 2.5kHz

#define BUFFERSIZE 32  				 		///< Size of ring buffer should be a power of 2
volatile uint8_t buffer[BUFFERSIZE]; 		///< Buffer to contain the 256 bits on the screen.
volatile uint8_t buffer_index;       		///< pointer to current start of buffer.
volatile uint16_t serial_in_buffer;  		///< buffer for next word currently being clocked in
volatile uint16_t serial_out_buffer; 		///< buffer for last word currently being clocked out
volatile uint8_t serial_in_index;    		///< bit of word currently being clocked in or out of the module

volatile uint16_t time_since_last_bit=0; 	///< counter how many timer periods have elapsed since last edge
volatile uint8_t last_clock_width=0;		///< timed value for last clock width

volatile uint8_t global_flags;				///< global variable for flags
#define FLAG_TIMER 			0 				///< flag to indicate the regular refresh timer has elapsed
#define FLAG_BUFFER_FULL 	1 				///< flag to indicate a word has been received on input
#define FLAG_EDGE			2				///< an edge was detected onthe serial input clock

#define CLEAR_FLAG(b) 		(global_flags &= ~(1<<b))
#define SET_FLAG(b)			(global_flags |= (1<<b))
#define FLAG_IS_SET(b)		(global_flags & (1<<b))
#define FLAG_IS_CLEAR(b) 	((global_flags & (1<<b)) == 0)


#define SET_OR_CLEAR_BIT_IF_BIT_IS_SET(bit,port,pin,data) \
	((1<<(bit)) & (data) ) ? ((port) |= (1<<(pin))) : ((port) &= ~(1<<(pin)));


// Hardware Initialisation functions
void init_pins (void);
void init_timer0(void);
void init_timer1(void);
void init_usi(void);
void init_serial_input(void);

//  Shift register functions
void usi_send_byte(uint8_t data);
void usi_send_byte_LSB_FIRST(uint8_t data);
void enable_output( void );
void disable_ouput(void);

// screen updating functions
void update_screen(void);
void draw_col(uint8_t column, uint8_t upper_byte, uint8_t lower_byte);
uint8_t reverse_byte (uint8_t byte);
void clock_new_ouput(void);

// Serial and buffer functions
void insert_word_into_buffer(void);



/**
 * Main operating loop for the software
 */
int main(void)
{
	//
	// Hardware Initialisation
	//
	init_pins();
	init_usi();
	init_serial_input();
	init_timer0();
	init_timer1();
	enable_output();

	//reduce power consumption in unused peripherals
	// (minor change to current consumption)
	 PRR=(0<<PRTIM1)|(0<<PRTIM0)|(0<<PRUSI)|(1<<PRADC);

	//
	// Main Loop
    //
	for(;;){
        // have we got a new word?
		if (FLAG_IS_SET(FLAG_BUFFER_FULL)){
			insert_word_into_buffer();                                          //save the word that just came in.
		}

		// Is it time to update the screen?
		if (FLAG_IS_SET(FLAG_TIMER)){
			CLEAR_FLAG(FLAG_TIMER);                                             // reset the timer flag
			update_screen();                                                    // draw a single column of the screen buffer

			// Do some other house keeping after updating the screen.
			time_since_last_bit++;                                              // increment our refresh timer/counter (2.5khz)

            // this is to abort the buffer if we have received a partial word but
            // no clocks for some time. This should mean that the screens will
            // synchronise if no data is received for some time.
            // It should also make the system more robust should spurious clocks
            // be received due to noise.
			if ((time_since_last_bit == FRAME_RESET_TIMEOUT)&& FLAG_IS_CLEAR(FLAG_BUFFER_FULL)){
				serial_in_index = 16;                                           // reset buffer word bit pointer
				serial_in_buffer = 0x0000;                                      // clear buffer word
			}
		}

        // Go to sleep (timers still running)
		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_mode();

	}

	return 0;   /* never reached */
}



/**
 * Initialise the I/O pins
 *
 */
void init_pins (void)
{

    // set shift register output-enable and clear pins High (active low)
    PORTA = (0<<SCLCK_OUT)|(0<<SD_OUT)|(0<<SD_IN)|(1<<OE4)|(1<<OE3)|(1<<OE2)|(0<<RCK)|(1<<SCL);
    PORTB = (0<<SCLCK_IN)|(1<<OE1)|(0<<SCK)|(0<<MISO)|(0<<SERIAL)|(1<<DEBUG);
    
    // Set the direction of pins on Port A 
    DDRA = (1<<SCLCK_OUT)|(1<<SD_OUT)|(0<<SD_IN)|(1<<OE4)|(1<<OE3)|(1<<OE2)|(1<<RCK)|(1<<SCL);
    
    // Set the direction of pins on Port B
    DDRB = (0<<SCLCK_IN)|(1<<OE1)|(1<<SCK)|(1<<MISO)|(1<<SERIAL)|(1<<DEBUG);
}



/** 
 * Initialise the Timer
 *
 * The timer is timer0 and uses the 8Mhz system clock to provide a 150 Hz update
 * rate for the screen.
 */
void init_timer0(void)
{
    // Timer Counter 0 control register A
    // 16 bit modelast_clock_width
    TCCR0A = (1<<TCW0)|(0<<ICEN0)|(0<<ICNC0)|(0<<ICES0)| (0<<ACIC0)|(0<<WGM00);
    
    // Timer Conter 0 Interrupt Mask Register
    // enable ouput compare A interrupt.
    TIMSK  |= (1<<OCIE0A);
    
    // TimerCounter0 control register B
    // reset the counter and set source to system clock with no prescaling.
    TCCR0B  = (0<<TSM)|(1<<PSR0)|(0<<CS02)|(0<<CS01)|(1<<CS00);
    
    // set counter to 0x1f40 or 8000 counts this should give a 1ms interval 
    // set counter to 0x0320 for 800 counts -> 100uS interval
    // set counter to 0x0640 for 1600 counts -> 200uS interval
    // set counter to 0x0C80 for 3200 counts -> 400uS interval (2.5Khz /16 cols  => 156H.25Hz Refresh
    OCR0B =0x0c;
    OCR0A =0x80 ;
    
    CLEAR_FLAG(FLAG_TIMER);


}



/**
 * Initialise the secondary Timer
 *
 * The timer is timer1 and uses the 8Mhz system clock to provide a 1MHZ counter
 * Used for timing pulse width of incoming clock signal.
 */
void init_timer1(void)
{
	// TCCR1A – Timer Counter 1 control register A
	TCCR1A = (0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<FOC1A)|(0<<FOC1B)|(0<<PWM1A)|(0<<PWM1B);

	// TCCR1B – Timer/Counter1 Control Register B
	// clocked at clk/8
	TCCR1B = (0<<PWM1X)|(0<<PSR1)|(0<<DTPS11)|(0<<DTPS10)|(0<<CS13)|(1<<CS12)|(0<<CS11)|(0<<CS10);

	// TCCR1C – Timer/Counter1 Control Register C
	TCCR1C = (0<<COM1A1S)|(0<<COM1A0S)|(0<<COM1B1S)|(0<<COM1B0S)|(0<<COM1D1)|(0<<COM1D0)|(0<<FOC1D)|(0<<PWM1D);

	// TCCR1D – Timer/Counter1 Control Register D
	// Normal counter mode
	TCCR1D = (0<<FPIE1)|(0<<FPEN1)|(0<<FPNC1)|(0<<FPES1)|(0<<FPAC1)|(0<<FPF1)|(0<<WGM11)|(0<<WGM10);

	// TCCR1E – Timer/Counter1 Control Register E
	TCCR1E = (0<<OC1OE5)|(0<<OC1OE4)|(0<<OC1OE3)|(0<<OC1OE2)|(0<<OC1OE1)|(0<<OC1OE0);

	// PLLCSR – PLL Control and Status Register
	// synchonous system clock is used.
	PLLCSR = (0<<LSM)|(0<<PCKE)|(0<<PLLE);

	// set top value
	OCR1C =0xff;


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
 * setup the incoming Serial interface ISR
 */
void init_serial_input(void)
{
    // reset buffesr 
    serial_in_index = 16;
    serial_in_buffer = 0;
    buffer_index =0;
    serial_out_buffer = 0;
    
    // rising edge gOCR1A = 0x00;enerates an interrupt
    // MCUCR |= (1<<ISC01)|(1<<ISC00);
    
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
 * On last rising edge the ISlast_clock_widthR takes 24us
 * Falling edges take 1.4us to complete
 * There is a delay of 3.6us to enter the ISR
 */
ISR(INT0_vect){
	//DEBUG_TRIGGER;

	// disable the last bit clock as we have just seen an edge
	TIMSK &= ~(1<<OCIE1A);

	// re-enable interrupts so the screen update can happen
	sei();

    // look for a rising edge
    if (PINB & (1<<SCLCK_IN)){
    	if( FLAG_IS_CLEAR(FLAG_BUFFER_FULL) ){
    		TCNT1 = 0x00;														// Reset Timer1
    		TIFR |= (1<<TOV1); // reset the overflow flag


			serial_in_index--;                                                  // move pointer to next bit

			// set bit in buffer
			if (PINA & (1<< SD_IN) ){
				serial_in_buffer |= (1<<serial_in_index);
			}
			PORTA &= ~( (1<<SCLCK_OUT));                                        // clear outgoing clock

			// set/clear bit on outgoing serial
			if (serial_out_buffer & (1<<serial_in_index)){
				PORTA |= (1<<SD_OUT);                                           // set data bit
			}else{
				PORTA &= ~(1<<SD_OUT);                                          // clear data bit
			}

			// set the flag if the buffer is full.
			if (serial_in_index == 0){
				SET_FLAG(FLAG_BUFFER_FULL);
			}
    	}
    }else{ // A falling edge
        PORTA |= (1<<SCLCK_OUT);                                                // set outgoing clock line
        time_since_last_bit=0;                                                  // reset our inter frame timer

		// save the clock period for use later.
		last_clock_width = TCNT1;

		// Reset Timer1
		TCNT1 = 0x00;

		// setup the Output compare value
		if (TIFR & (1<<TOV1)){
			OCR1A = 0xff;
		}else{
			OCR1A =last_clock_width ;
		}

		// enable the interrupt to clock out the last bit
		TIFR |= (1<<TOV1)|(1<<OCF1A ); 											// reset the overflow and interrupt flag
		asm volatile("nop\n\t"::);
		TIMSK |= (1<<OCIE1A );
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
  //  DEBUG_TRIGGER;
    TCNT0H=0;
    TCNT0L=0;
    SET_FLAG(FLAG_TIMER);
   // DEBUG_TRIGGER;
}



/**
 * Timer1 interrupt
 *
 * This ISR is used to clock out the final bit of a word if no new bit has been
 * received. It then disables it self to give a "one-shot" action.
 */
ISR(TIMER1_COMPA_vect)
{
    PORTA &= ~(1<<SCLCK_OUT);			                            			// clear clock
    TIMSK &= ~(1<<OCIE1A);														// disable the interrupt
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
  #if (SW_SPI_FAST==0) // this gives 940ns and 870ns per fist and following bits
    uint8_t temp;
    uint8_t i ;
    temp = data;
    for (i=0;i<8;i++){
         PORTB &= ~(1<<SCK);                                                     // clear clock line

         if (temp & 0x80){
             PORTB |= (1<<SERIAL);                                               // set data bit
         }else{
             PORTB &= ~(1<<SERIAL);                                              // clear data bit
         }

         PORTB |= (1<<SCK);                                                      // set clock line
         temp <<= 1;                                                             // shift along by one
     }
  #else // this gives 610ns for all bits
    // assume the clock line is low to begin with (set in init routine)
    ((1<<7) & data) ? (PORTB |= (1<<SERIAL)) : (PORTB &= ~(1<<SERIAL));      	// clear or set data bit
    PINB = (1<<SCK);                                                       		// set clock line
    PINB = (1<<SCK);                                                       		// clear clock line
    ((1<<6) & data) ? (PORTB |= (1<<SERIAL)) : (PORTB &= ~(1<<SERIAL));      	// clear or set data bit
    PINB = (1<<SCK);                                                       		// set clock line
    PINB = (1<<SCK);                                                       		// clear clock line
    ((1<<5) & data) ? (PORTB |= (1<<SERIAL)) : (PORTB &= ~(1<<SERIAL));      	// clear or set data bit
    PINB = (1<<SCK);                                                      	 	// set clock line
    PINB = (1<<SCK);                                                       		// clear clock line
    ((1<<4) & data) ? (PORTB |= (1<<SERIAL)) : (PORTB &= ~(1<<SERIAL));      	// clear or set data bit
    PINB = (1<<SCK);                                                       		// set clock line
    PINB = (1<<SCK);                                                       		// clear clock line
    ((1<<3) & data) ? (PORTB |= (1<<SERIAL)) : (PORTB &= ~(1<<SERIAL));      	// clear or set data bit
    PINB = (1<<SCK);                                                       		// set clock line
    PINB = (1<<SCK);                                                       		// clear clock line
    ((1<<2) & data) ? (PORTB |= (1<<SERIAL)) : (PORTB &= ~(1<<SERIAL));      	// clear or set data bit
    PINB = (1<<SCK);                                                       		// set clock line
    PINB = (1<<SCK);                                                       		// clear clock line
    ((1<<1) & data) ? (PORTB |= (1<<SERIAL)) : (PORTB &= ~(1<<SERIAL));      	// clear or set data bit
    PINB = (1<<SCK);                                                       		// set clock line
    PINB = (1<<SCK);                                                       		// clear clock line
    ((1<<0) & data) ? (PORTB |= (1<<SERIAL)) : (PORTB &= ~(1<<SERIAL));      	// clear or set data bit
    PINB = (1<<SCK);                                                       		// set clock line
  #endif
    PORTB &= ~( (1<<SCK)|(1<<SERIAL) );                                         // clear clock and data line
#endif
}



/**
 * Send a single byte LSB first on the USI bus to the shift registers
 *
 * This sends the 8 bit data byte out the SERIAL line and clocks the SCK with
 * each byte. This routine has been optimised for speed.
 */
  void usi_send_byte_LSB_FIRST(uint8_t data ///< 8 bit data to be sent
                   )
{
#if (HW_SPI==1)
	  uint8_t reversed_byte;

#if (REVERSE_BYTE_USE_LOOKUP==1)
	  reversed_byte = pgm_read_byte(reverse_lookup+data);
#else
	  reversed_byte = reverse_byte( data);
#endif

    USIDR = reversed_byte;
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

#if (SW_SPI_FAST==0)
    uint8_t temp;
    uint8_t i ;

    temp = data;

    for (i=0;i<8;i++){
        PORTB &= ~(1<<SCK);                                                     // clear clock line

        if (temp & 0x01){
            PORTB |= (1<<SERIAL);                                               // set data bit
        }else{
            PORTB &= ~(1<<SERIAL);                                              // clear data bit
        }

        PORTB |= (1<<SCK);                                                      // set clock line
        temp >>= 1;                                                             // shift along by one
    }
#else // this gives 620ns for all bits

   // PINB = (1<<SCK);                                                       	// toggle clock line
    SET_OR_CLEAR_BIT_IF_BIT_IS_SET(0,PORTB,SERIAL,data);						// clear or set data bit
    PINB = (1<<SCK);                                                       		// set clock line
    PINB = (1<<SCK);                                                       		// toggle clock line
    SET_OR_CLEAR_BIT_IF_BIT_IS_SET(1,PORTB,SERIAL,data);						// clear or set data bit
    PINB = (1<<SCK);                                                       		// set clock line
    PINB = (1<<SCK);                                                       		// toggle clock line
    SET_OR_CLEAR_BIT_IF_BIT_IS_SET(2,PORTB,SERIAL,data);						// clear or set data bit
    PINB = (1<<SCK);                                                       		// set clock line
    PINB = (1<<SCK);                                                       		// toggle clock line
    SET_OR_CLEAR_BIT_IF_BIT_IS_SET(3,PORTB,SERIAL,data);						// clear or set data bit
    PINB = (1<<SCK);                                                       		// set clock line

    PINB = (1<<SCK);                                                       // toggle clock line
    SET_OR_CLEAR_BIT_IF_BIT_IS_SET(4,PORTB,SERIAL,data);						// clear or set data bit
    PINB = (1<<SCK);                                                       // set clock line
    PINB = (1<<SCK);                                                       // toggle clock line
    SET_OR_CLEAR_BIT_IF_BIT_IS_SET(5,PORTB,SERIAL,data);						// clear or set data bit
    PINB = (1<<SCK);                                                       // set clock line
    PINB = (1<<SCK);                                                       // toggle clock line
    SET_OR_CLEAR_BIT_IF_BIT_IS_SET(6,PORTB,SERIAL,data);						// clear or set data bit
    PINB = (1<<SCK);                                                       // set clock line
    PINB = (1<<SCK);                                                       // toggle clock line
    SET_OR_CLEAR_BIT_IF_BIT_IS_SET(7,PORTB,SERIAL,data);						// clear or set data bit
    PINB = (1<<SCK);                                                       // set clock line
#endif

    PORTB &= ~( (1<<SCK)|(1<<SERIAL) );                                         // clear clock and data line

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
    PORTA |= (1<<OE4)|(1<<OE3)|(1<<OE2);
    PORTB |= (1<<OE1);
}



/** 
 * Shift the data to the ouputs of the shift registers
 *
 * Cycle the RCK (register clock) on the shift register to latch the data 
 * from the data register to the actual outputs
 */
void clock_new_ouput(void)
{
    PINA = (1<<RCK);                                                            // Toggle this pin 0->1
    PINA = (1<<RCK);                                                            // Toggle this pin 1->0
}


#if (REVERSE_BYTE_USE_LOOKUP ==0)
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
#else
static uint8_t PROGMEM reverse_lookup[] = {
	0x00,0x80,0x40,0xc0,0x20,0xa0,0x60,0xe0,0x10,0x90,0x50,0xd0,0x30,0xb0,0x70,0xf0,
	0x08,0x88,0x48,0xc8,0x28,0xa8,0x68,0xe8,0x18,0x98,0x58,0xd8,0x38,0xb8,0x78,0xf8,
	0x04,0x84,0x44,0xc4,0x24,0xa4,0x64,0xe4,0x14,0x94,0x54,0xd4,0x34,0xb4,0x74,0xf4,
	0x0c,0x8c,0x4c,0xcc,0x2c,0xac,0x6c,0xec,0x1c,0x9c,0x5c,0xdc,0x3c,0xbc,0x7c,0xfc,
	0x02,0x82,0x42,0xc2,0x22,0xa2,0x62,0xe2,0x12,0x92,0x52,0xd2,0x32,0xb2,0x72,0xf2,
	0x0a,0x8a,0x4a,0xca,0x2a,0xaa,0x6a,0xea,0x1a,0x9a,0x5a,0xda,0x3a,0xba,0x7a,0xfa,
	0x06,0x86,0x46,0xc6,0x26,0xa6,0x66,0xe6,0x16,0x96,0x56,0xd6,0x36,0xb6,0x76,0xf6,
	0x0e,0x8e,0x4e,0xce,0x2e,0xae,0x6e,0xee,0x1e,0x9e,0x5e,0xde,0x3e,0xbe,0x7e,0xfe,
	0x01,0x81,0x41,0xc1,0x21,0xa1,0x61,0xe1,0x11,0x91,0x51,0xd1,0x31,0xb1,0x71,0xf1,
	0x09,0x89,0x49,0xc9,0x29,0xa9,0x69,0xe9,0x19,0x99,0x59,0xd9,0x39,0xb9,0x79,0xf9,
	0x05,0x85,0x45,0xc5,0x25,0xa5,0x65,0xe5,0x15,0x95,0x55,0xd5,0x35,0xb5,0x75,0xf5,
	0x0d,0x8d,0x4d,0xcd,0x2d,0xad,0x6d,0xed,0x1d,0x9d,0x5d,0xdd,0x3d,0xbd,0x7d,0xfd,
	0x03,0x83,0x43,0xc3,0x23,0xa3,0x63,0xe3,0x13,0x93,0x53,0xd3,0x33,0xb3,0x73,0xf3,
	0x0b,0x8b,0x4b,0xcb,0x2b,0xab,0x6b,0xeb,0x1b,0x9b,0x5b,0xdb,0x3b,0xbb,0x7b,0xfb,
	0x07,0x87,0x47,0xc7,0x27,0xa7,0x67,0xe7,0x17,0x97,0x57,0xd7,0x37,0xb7,0x77,0xf7,
	0x0f,0x8f,0x4f,0xcf,0x2f,0xaf,0x6f,0xef,0x1f,0x9f,0x5f,0xdf,0x3f,0xbf,0x7f,0xff,
};
#endif

static uint8_t columna_lookup[] = {
	127,191,223,239,247,251,253,254,255,255,255,255,255,255,255,255,
};

static uint8_t columnb_lookup[] = {
	255,255,255,255,255,255,255,255,254,253,251,247,239,223,191,127,
};

/**
 * Draw a single column of the screen
 *
 * sends 4 bytes out to the SPI port to the display. takes approx 80uS for slow 
 * speed operation if HW USI is used it is closer to 23uS
 */
void draw_col(uint8_t column,                                                   ///< column to draw 0..15
              uint8_t upper_byte,                                               ///< byte for upper half of column
              uint8_t lower_byte                                                ///< byte for lower half of column
              )
{
    usi_send_byte( columna_lookup[column]);
    usi_send_byte( columnb_lookup[column]);
    usi_send_byte_LSB_FIRST(lower_byte);
    usi_send_byte( upper_byte );
    clock_new_ouput();
}



/**
 * Draw the buffer to the screen
 * 
 * This will draw a single column of the currenable_outputent buffer to the screen. The
 * column count will be incremented. This routine will alternately draw a blank column
 * without incrementing the column count. By controlling the ration of blank to 
 * actual columns we can affect a brightness of the LED screen.
 */
void update_screen(void)
{
    static uint8_t current_column=0;											// this is the column 0-15 we should update next
    static uint8_t cnt =0;														// this is the count value for the brightness
    uint8_t upper_addr;															// byte address in buffer for upper half of column
    uint8_t lower_addr;															// byte address in buffer for lower half of column

    if (cnt++){
    	cnt=0;																	// next time draw the actual column
       draw_col(current_column,0,0);
    }else{
        if (current_column==(BUFFERSIZE>>1)){
        	current_column =0;
        }else{
        	current_column++;
        }
        
        upper_addr = buffer_index + (current_column<<1) ;
        upper_addr &= BUFFERSIZE-1 ;                                            // limit to  size of buffer and roll any overflow
        lower_addr  = upper_addr+1 &(BUFFERSIZE-1);                             // limit to  size of buffer and roll any overflow
        draw_col(current_column, buffer[upper_addr], buffer[lower_addr] );
    }
}



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
    CLEAR_FLAG(FLAG_BUFFER_FULL);
}






#ifdef USE_TEST_FUNCTIONS
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
    for(i=0;i<10;i++){/* Set TCNT1 to 0x01FF */
    	TC1H = 0x01;
    	TCNT1 = 0xFF;
    	/* Read TCNT1 into i */
    	i = TCNT1;
    	i |= ((unsigned int)TC1H << 8);
          PINA = (1<<SD_OUT)|(1<<SCLCK_OUT);// toggle these bits
          PINB = (1<<SERIAL);
    }

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



/**
 * Simple scrolling alphabet to test the screen
 */

void test_fontSet0 (void)
{
    static uint16_t character=200;
    static uint8_t count;

    uint8_t i ;
    // uint8_t i;

    if (count++==50) {
        character--;
        count=0;
    }

    for (i=0;i<16;i++){
        draw_col(i,pgm_read_byte(fontSet0+character+i),pgm_read_byte(fontSet0-character+i));
	       _delay_ms(10);
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
        PORTA &= ~(1<<SCLCK_OUT);                                               // clear clock line
        _delay_us(SOUT_DELAY_US);

        if (temp & 0x80){
            PORTA |= (1<<SD_OUT);                                               // set data bit
        }else{
            PORTA &= ~(1<<SD_OUT);                                              // clear data bit
        }
        PORTA |= (1<<SCLCK_OUT);                                                // set clock line
        temp <<= 1;                                                             // shift along by one
        _delay_us(SOUT_DELAY_US);

    }
     PORTA &= ~( (1<<SCLCK_OUT)|(1<<SD_OUT) );                                  // clear clock and data line
}
#endif
// EOF main.c

