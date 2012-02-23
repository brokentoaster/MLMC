//#include <SPI.h>
// SPI Master enabled
// MSB of the dataPin byte transmitted first
// SPI mode 0 (CPOL = 0, CPHA = 0)
// SPI clockPin frequency = system clockPin / 4


// MAX6675 stuff
#include <MAX6675.h>
int LED1 = 13;             // Status LED Pin
int gnd =12;
int CS = 10;              // CS pin on MAX6675
int SO = 11;              // SO pin of MAX6675
int SCK = 9;             // SCK pin of MAX6675
int power = 8;

int units = 1;              // Units to readout temp (0 = ˚F, 1 = ˚C)
float error = -9.0;         // Temperature compensation error
float temperature = 0.0;    // Temperature output variable


// MLMC stuff
#include "fontset1.h"
#define FONT_OFFSET 33
int dataPin = 2;
int clockPin = 3;
int spidelay =75;
//const int slaveSelectPin = 10;
int gInterColDelay=0 ;
int gInterCharDelay;




// Initialize the MAX6675 Library for our chip
MAX6675 temp0(CS,SO,SCK,units,error);




void Send_byte_next_module( char data ///< 8 bit dataPin to be sent
                           )
{
    uint8_t temp;
    uint8_t i,j ;
    
    // for (j=0;j<2;j++){
    temp = data;
    //   /Serial.print(temp,DEC);
    // Serial.print(" :");
    for (i=0;i<8;i++){
        digitalWrite(clockPin, LOW);
        
        if (temp & (1<<7)){
            digitalWrite(dataPin, HIGH);
            //       Serial.print("1");
            
        }else{
            digitalWrite(dataPin, LOW);
            // Serial.print("0");
        }
        
        delayMicroseconds(spidelay);
        digitalWrite(clockPin,HIGH);  // clockPin the dataPin out
        delayMicroseconds(spidelay);
        
        //clock it out twice to stretch to 16 bits [HACK][DEBUG]  
        //        digitalWrite(clockPin, LOW);
        //       delayMicroseconds(spidelay);
        //       digitalWrite(clockPin,HIGH);  // clockPin the dataPin out
        //       delayMicroseconds(spidelay); 
        // ------------------------------------------------------
        
        temp <<= 1;                                // shift allong by one
    }
    //  }
    //  Serial.println();
    
    digitalWrite(clockPin, LOW);
    digitalWrite(dataPin, LOW);
    
}

char reverse(char b){
    return  (b * 0x0202020202ULL & 0x010884422010ULL) % 1023;
}

void mlmc_print_char(char c)
{
    char i,j,tdata,data,not_leading;
    
    if (c>=FONT_OFFSET){
        not_leading =0;
        for (i=0;i<fontSet1[1];i++){
            data =pgm_read_byte(fontSet1+((c-FONT_OFFSET)*fontSet1[1])+i+3); 
            if (data !=00 || not_leading == 1){
                not_leading = 1;
                Send_byte_next_module( data);
                Send_byte_next_module( 0);
                delay (gInterColDelay);
            }
        }
    }else{
        for (i=0;i<fontSet1[1];i++){
            Send_byte_next_module(0);
            Send_byte_next_module(0);
            delay (gInterColDelay);
        } 
    }
    Send_byte_next_module(0);
    Send_byte_next_module(0);
}


void mlmc_clearscreen(){
    char i;
    for (i=0;i<14;i++){
        mlmc_print_char(32);
    }
}

void mlmc_print(char *c)
{
    int i = 0;
    
    while (c[i]){
        mlmc_print_char(c[i]);
        i++;
    }
    delay(gInterCharDelay);
}



void setup(void){
    int i;
    //font = fontSet1;
    
    
    // set the slaveSelectPin as an output:
    //pinMode (slaveSelectPin, OUTPUT);
    // initialize SPI:
    // SPI.begin(); 
    // SPI.mode ((1<<SPR1)|(1<<SPR0));                // set SPI clockPin to system clockPin / 16
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
  
    gInterColDelay=25;
    gInterCharDelay = 250;
    
    mlmc_clearscreen();
    mlmc_print("Hello");
    delay(500);  
    mlmc_print(" World!");
    delay(500);
    mlmc_clearscreen();
    
    for (i=0;i<fontSet1[0];i++){
        mlmc_print_char(i+FONT_OFFSET);
    }
    
}


void loop(){
    char inByte;
    static long unsigned int timer;
    static long unsigned int uptime=0;
    static char cleanme=0;
    
    //mlmc_print_char(c);
    //c = c ++  ;
    //if (c>'z') c='!';
    
    
    if (Serial.available() > 0) {
        // get incoming byte:
        inByte = Serial.read();
        if (inByte<30){
            gInterColDelay = inByte*5;
        }
        if (inByte=='}'){
            mlmc_clearscreen();
        }else{
            mlmc_print_char(inByte);
            if (inByte==' '){
                delay(250);
            }
        }
        timer = millis();
        cleanme=1;
    }
    
    
    
    if ((millis()-timer)>5000) {
        if (cleanme){
            mlmc_clearscreen();
            cleanme=0;
        }
      
        timer = millis();
        uptime++;
        if (uptime == 12){
            mlmc_print(" Another minute ticks by...    ;)         :)");
            uptime = 0;
        }else{
            char buffer[255];
            char temp_decimal;
            char temp;
            temperature = temp0.read_temp(1);         // Read the temp 5 times and return the average value to the var
            
            temp = (char) temperature;
            temp_decimal = (char) ((temperature - temp) *100);
            
            if(temperature == -1) {                   // If there is an error with the TC, temperature will be -1
                mlmc_print("Error!!"); // Temperature is -1 and there is a thermocouple error        
                
            } else {
                int tempspeed;
                tempspeed = gInterColDelay;
                gInterColDelay = 0;
//                sprintf(buffer,"Current Temperature:   %d.%02d%cC", temp,temp_decimal,94+FONT_OFFSET);
                sprintf(buffer,"   %d.%02d%cC", temp,temp_decimal,94+FONT_OFFSET);
                mlmc_print(buffer); 
                gInterColDelay = tempspeed;
                //    mlmc_print( temperature );          // Print the temperature to Serial 
                
            }
        }
    }
    
    
    
}
