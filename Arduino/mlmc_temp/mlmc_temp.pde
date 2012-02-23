
/* ----------------------------------------------------------------------
 * MAX6675 stuff
 * This section takes care of reading a Temperature from the thermocouple
 */
#include <MAX6675.h>
int LED1 = 13;              // Status LED Pin

int power = 12;
int SCK = 11;                // SCK pin of MAX6675
int CS = 10;                // CS pin on MAX6675
int SO = 9;                // SO pin of MAX6675
int gnd =8;                // IF

int units = 1;              // Units to readout temp (0 = ˚F, 1 = ˚C)
float error = -15.0;        // Temperature compensation error (measured)
float temperature = 0.0;    // Temperature output variable
/*------------------------------------------------------------------------
*/



/* ----------------------------------------------------------------------
 * MLMC stuff
 * Select the fonts and hardware details for the MLMC dis[play here
 */
//#include "font_ttuf1.h"
#include "fontset0.h"
//#include "fontset2.h"
#define FONT_OFFSET 33
int dataPin = 2;
int clockPin = 3;
int spidelay =75;
//const int slaveSelectPin = 10;
int gInterColDelay=0 ;
int gInterCharDelay;

// Initialize the MAX6675 Library for our chip
MAX6675 temp0(CS,SO,SCK,units,error);
/*------------------------------------------------------------------------
*/


/* ----------------------------------------------------------------------
*/
void Send_byte( char data ///< 8 bit dataPin to be sent
              ) {
    uint8_t temp;
    uint8_t i,j ;

    temp = data;

    for (i=0; i<8; i++) {
        digitalWrite(clockPin, LOW);

        if (temp & (1<<7)) {
            digitalWrite(dataPin, HIGH);
        } else {
            digitalWrite(dataPin, LOW);
        }

        delayMicroseconds(spidelay);
        digitalWrite(clockPin,HIGH);  // clockPin the dataPin out
        delayMicroseconds(spidelay);

        temp <<= 1;                                // shift allong by one
    }

    digitalWrite(clockPin, LOW);
    digitalWrite(dataPin, LOW);
}

char reverse(char b) {
    return  (b * 0x0202020202ULL & 0x010884422010ULL) % 1023;
}

// print char to the mlmc display one column at a time
// any leading blank columns of the characters are skipped
// if the char is outside the fontset then
void mlmc_print_char_double(char c, char c2) {
    char i,j,k,l;
    char tdata;
    char data;
    char not_leading;
    char data_array[32];

    for (i=0; i<32; i++) data_array[i]=0;

    k=0;

    // do the top row
    if (c >= FONT_OFFSET) {

        not_leading =0;

        for (i=0; i < fontSet1[1]; i++) {

            data = pgm_read_byte( fontSet1 + (( c - FONT_OFFSET ) * fontSet1[1] ) + i + 3 );

            if (data !=00 || not_leading == 1) {
                // if not a leading blank column then send the column to the display
                not_leading = 1;
                k++;
                data_array[k++]= data;
            }
        }
    } else {

        // send a blank space
        for (i=0; i<fontSet1[1]; i++) {
            k+=2;
        }
    }

//    do the bottom row
    l=0;
    if (c2 >= FONT_OFFSET) {

        not_leading =0;

        for (i=0; i < fontSet1[1]; i++) {

            data = pgm_read_byte( fontSet1 + (( c2 - FONT_OFFSET ) * fontSet1[1] ) + i + 3 );

            if (data !=00 || not_leading == 1) {
                // if not a leading blank column then send the column to the display
                not_leading = 1;
                data_array[l++]= data;   // invert bottom half of screen
                l++;

            }
        }
    } else {

        // send a blank space
        for (i=0; i<fontSet1[1]; i++) {
            l+=2;
        }
    }

    // send out the bytes
    if (l>k) k=l;
    l=0;


    for (l=0; l<k;) {
        Send_byte(data_array[l++]); // invert bottom half of screen
        Send_byte(data_array[l++]);
        delay (gInterColDelay);
    }

    // blank column between characters
    Send_byte(0);
    Send_byte(0);
}

// print char to the mlmc display one column at a time
// any leading blank columns of the characters are skipped
// if the char is outside the fontset then
void mlmc_print_char(char c) {
    char i,j;
    char tdata;
    char data;
    char not_leading;

    if (c >= FONT_OFFSET) {

        not_leading =0;

        for (i=0; i < fontSet1[1]; i++) {

            data = pgm_read_byte( fontSet1 + (( c - FONT_OFFSET ) * fontSet1[1] ) + i + 3 );

            if (data !=00 || not_leading == 1) {
                // if not a leading blank column then send the column to the display
                not_leading = 1;
                Send_byte( ~data); // invert bottom half of screen
                Send_byte( data);
                delay (gInterColDelay);
            }
        }
    } else {

        // send a blank space
        for (i=0; i<fontSet1[1]; i++) {
            Send_byte(0xFF); // invert bottom half of screen
            Send_byte(0);
            delay (gInterColDelay);
        }
    }

    // blank column between characters
    Send_byte(0xFF);
    Send_byte(0);
}


void mlmc_clearscreen() {
    char i;
    for (i=0; i<14; i++) {
        mlmc_print_char(32);
    }
}

void mlmc_print(char *c) {
    int i = 0;

    while (c[i]) {
        mlmc_print_char(c[i]);
        i++;
    }
    delay(gInterCharDelay);
}

/** Step through two strings and print a char from each to the screen

If one string is longer than the other then blank spaces will be printed
*/
void mlmc_print_double(char *c,char *c2) {
    int i = 0;
    char flag = 0;

    while (flag<3) {
        if (c[i]==0 or (flag & (1<<0)))
            flag  |= (1<<0);
        if (c2[i]==0 or (flag & (1<<1)))
            flag |= (1<<1);

        switch (flag) {
            case 0:
                mlmc_print_char_double(c[i],c2[i]);
                break;
            case 1:
                mlmc_print_char_double(0x00,c2[i]);
                break;
            case 2:
                mlmc_print_char_double(c[i],0x00);
                break;
        }
        i++;
    }
    delay(gInterCharDelay);
}

void do_temp_reading(void)
{
   char buffer[255];
   char buffer2[255];
   
   char temp_decimal;
            int temp;
            temperature = temp0.read_temp(1);         // Read the temp

            temp = (int) temperature;
            temp_decimal = (char) ((temperature - temp) *100);

            if (temperature == -1) {                   // If there is an error with the TC, temperature will be -1
                mlmc_print("Error!!");                 // Temperature is -1 and there is a thermocouple error

            } else {
                int tempspeed;
                tempspeed = gInterColDelay;
                gInterColDelay = 40;
//                sprintf(buffer,"Current Temperature:   %d.%02d%cC", temp,temp_decimal,94+FONT_OFFSET);
                sprintf(buffer,"  %d.%02d%cC", temp,temp_decimal,pgm_read_byte(fontSet1)+FONT_OFFSET);
                mlmc_print_double("  Temperature",buffer);
                sprintf(buffer,"%d.%02d", temp,temp_decimal);
                Serial.println(buffer);
                gInterColDelay = tempspeed;
                //    mlmc_print( temperature );          // Print the temperature to Serial

            }
}



void setup(void) {
    int i;

    Serial.begin(57600);
    Serial.println("MLMC Start");

    pinMode(dataPin,OUTPUT);
    pinMode(clockPin,OUTPUT);
    digitalWrite(dataPin, HIGH);
    digitalWrite(dataPin, LOW);


    pinMode(LED1, OUTPUT);
    pinMode(power, OUTPUT);
    digitalWrite(power, HIGH);
    pinMode(gnd, OUTPUT);
    digitalWrite(gnd, LOW);

    /* set the speeds */
    gInterColDelay=25;
    gInterCharDelay = 250;

    /*  Print a short intro message */
    mlmc_clearscreen();
    mlmc_print("Hello");
    delay(500);
    mlmc_print(" World!");
    delay(500);


    mlmc_clearscreen();

    mlmc_print_double("Hello","  World!");
    delay(500);

    /* Demo the selected current */
    for (i=0; i<fontSet1[0]; i++) {
        mlmc_print_char(i+FONT_OFFSET);
    }

}



void loop() {
    char inByte;

    static long unsigned int timer;
    static long unsigned int uptime=0;
    static char cleanme=0;

    if (Serial.available() > 0) {
        // get incoming byte:
        inByte = Serial.read();
        if (inByte<30) {
            gInterColDelay = inByte*5;
        }
        if (inByte=='}') {
            mlmc_clearscreen();
        } else {
            mlmc_print_char(inByte);
            if (inByte==' ') {
                delay(250);
            }
        }
        timer = millis();
        cleanme=1;
    }



    if ((millis()-timer)>1000) {
        if (cleanme) {
            mlmc_clearscreen();
            cleanme=0;
        }

        timer = millis();
        uptime++;
        if (uptime == 60) {
            // mlmc_print(" Another minute ticks by...    ;)         :)");
           // mlmc_print_double("     Out to","      Lunch ");
            uptime = 0;
        } else if (uptime%5==0) {
             do_temp_reading();
        }
      
    }



}
