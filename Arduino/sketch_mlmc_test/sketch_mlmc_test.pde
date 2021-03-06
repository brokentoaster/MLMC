//#include <SPI.h>
// SPI Master enabled
// MSB of the dataPin byte transmitted first
// SPI mode 0 (CPOL = 0, CPHA = 0)
// SPI clockPin frequency = system clockPin / 4

#include "fontset0.h"
#define FONT_OFFSET 33

//#include "fontSet1.h"
int dataPin = 11;
int clockPin = 13;
long spidelay =75;
//const int slaveSelectPin = 10;
int gInterColDelay=0 ;
int gInterCharDelay;


void setup(void){
 

          
             // set the slaveSelectPin as an output:
            //pinMode (slaveSelectPin, OUTPUT);
            // initialize SPI:
           // SPI.begin(); 
            // SPI.mode ((1<<SPR1)|(1<<SPR0));                // set SPI clockPin to system clockPin / 16
           Serial.begin(9600); 
           Serial.println("MLMC Start");
           pinMode(dataPin,OUTPUT);
           pinMode(clockPin,OUTPUT);
          digitalWrite(dataPin, HIGH);
            digitalWrite(dataPin, LOW);
            mlmc_clearscreen();
            gInterColDelay=0;
              mlmc_print("Hello World.");
            delay(500);  
             gInterColDelay=0;
            mlmc_print(" This is Nicks MLMC Project. Use serial port 9600 8N1.");
            delay(500);
           mlmc_clearscreen();
  
}


void Send_byte_next_module( char data ///< 8 bit dataPin to be sent
                           )
{
    uint8_t temp;
    uint8_t i ;
    
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
                
        temp <<= 1;                                // shift allong by one
    }
  //  Serial.println();
    
      digitalWrite(clockPin, LOW);
      digitalWrite(dataPin, LOW);
    
}

char reverse(char b){
  return  (b * 0x0202020202ULL & 0x010884422010ULL) % 1023;
}

void mlmc_print_char(char c)
{
  char i,data;


  char *font;
  
  if (c>=FONT_OFFSET){
   for (i=0;i<fontSet0[1];i++){
      data =pgm_read_byte(fontSet0+((c-FONT_OFFSET)*fontSet0[1])+i+3);  
      
      // reversre byte
    //  data=reverse(data);
      Send_byte_next_module( data);
      Send_byte_next_module(0);
      delay (gInterColDelay);
   }
  }else{
      for (i=0;i<fontSet0[1];i++){
          // reversre byte
          Send_byte_next_module( 0);
          Send_byte_next_module(0);
          delay (gInterColDelay);
       } 
  }
  Send_byte_next_module(0);
  Send_byte_next_module(0);
}
void mlmc_clearscreen(){
  char i;
  for (i=0;i<10;i++){
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

void loop(){
  char inByte;
  static long unsigned int timer;
  static long unsigned int uptime=0;
  
  //mlmc_print_char(c);
  //c = c ++  ;
  //if (c>'z') c='!';


 if (Serial.available() > 0) {
    // get incoming byte:
    inByte = Serial.read();
    mlmc_print_char(inByte);
    timer = millis();
 }
 
 if ((millis()-timer)>2000) {
   mlmc_clearscreen();
   timer = millis();
     uptime++;
     if (uptime == 30){
       mlmc_print("Another minute ticks by...          (^o^)");
       Serial.print("... ");
       uptime = 0;
     }
 }
 if ((millis()%1000)==0){
   
 }
}
