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
int spidelay =100;
//const int slaveSelectPin = 10;
int gInterColDelay=0 ;
int gInterCharDelay;


void setup(void){
 

          
             // set the slaveSelectPin as an output:
            //pinMode (slaveSelectPin, OUTPUT);
            // initialize SPI:
           // SPI.begin(); 
            // SPI.mode ((1<<SPR1)|(1<<SPR0));                // set SPI clockPin to system clockPin / 16
           Serial.begin(19200); 
           Serial.println("MLMC Start");
           pinMode(dataPin,OUTPUT);
           pinMode(clockPin,OUTPUT);
          digitalWrite(dataPin, HIGH);
            digitalWrite(dataPin, LOW);
      
            gInterColDelay=40;
            gInterCharDelay = 250;
            
             mlmc_clearscreen();
            mlmc_print("Hello");
            delay(500);  
            mlmc_print(" World!");
            delay(500);
            mlmc_clearscreen();
     //        gInterColDelay=70;
     //         mlmc_print(" This is Nicks MLMC Project. Use serial port 57600 8N1.");
     //       delay(500);
     //        mlmc_clearscreen();
  
}


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
  char i,data;


  char *font;
  
  if (c>=FONT_OFFSET){
   for (i=0;i<fontSet0[1];i++){
      data =pgm_read_byte(fontSet0+((c-FONT_OFFSET)*fontSet0[1])+i+3);  

      Send_byte_next_module( data);
      Send_byte_next_module( 0);
      delay (gInterColDelay);
   }
  }else{
      for (i=0;i<fontSet0[1];i++){
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
char serial_getch()
{
  char in;
  in = Serial.read();
  return  in;
}

void loop(){
  char rxbyte;
  char temp;
  static long unsigned int timer;
  static long unsigned int uptime=0;
  static char cleanme=0;
  
  //mlmc_print_char(c);
  //c = c ++  ;
  //if (c>'z') c='!';


 if (Serial.available() > 0) {
    // get incoming byte:
    rxbyte = Serial.read();
    
    
  if (rxbyte == 254) //Matrix Orbital uses 254 prefix for commands
	{
		switch (Serial.read())
		{
			case 66: //backlight on (at previously set brightness)
                                // not implemented				

				break;
			case 70: //backlight off
				// not implemented				
                                break;
			case 71:  //set cursor position
				temp = (serial_getch() - 1);  //get column byte
				switch (serial_getch())  //get row byte
				{
					//line 1 is already set up
					case 2:
						temp += 0x40;
						break;
					case 3:
						temp += 0x14;
						break;
					case 4:
						temp += 0x54;
						break;
					default:
						break;
				}
				//lcd.commandWrite(0b10000000 + temp);
				break;
			case 72:  //cursor home (reset display position)
				//lcd.commandWrite(2);
				break;
			case 74:  //show underline cursor
				//lcd.commandWrite(0b00001110);
				break;
			case 75:  //underline cursor off
			case 84:  //block cursor off
				//lcd.commandWrite(0b00001100);
				break;
			case 76:  //move cursor left
				//lcd.commandWrite(16);
				break;
			case 77:  //move cursor right
				//lcd.commandWrite(20);
				break;
			case 78:  //define custom char
				//lcd.commandWrite(64 + (serial_getch() * 8));  //get+set char address
				//for (temp = 7; temp != 0; temp--)
				//{
				//	lcd.print(serial_getch()); //get each pattern byte
				//}
				break;
			case 83:  //show blinking block cursor
				//lcd.commandWrite(0b00001111);
				break;
			case 86:  //GPO OFF
				//implement later
				break;
			case 87:  //GPO ON
				/*temp = serial_getch();
				if (temp == 1)
				{
					GPO1 = GPO_ON;
				}*/
				break;
			case 88:  //clear display, cursor home
				//lcd.commandWrite(1);
				break;
			case 152: //set and remember (doesn't save value, though)
			case 153: //set backlight brightness
				//not implemented
				break;

			//these commands ignored (no parameters)
			case 35: //read serial number
			case 36: //read version number
			case 55: //read module type
			case 59: //exit flow-control mode
			case 65: //auto transmit keypresses
			case 96: //auto-repeat mode off (keypad)
			case 67: //auto line-wrap on
			case 68: //auto line-wrap off
			case 81: //auto scroll on
			case 82: //auto scroll off
			case 104: //init horiz bar graph
			case 109: //init med size digits
			case 115: //init narrow vert bar graph
			case 118: //init wide vert bar graph
				break;
			default:
				//all other commands ignored and parameter byte discarded
				temp = serial_getch();  //dump the command code
				break;
		}
		return;
	} //END OF COMMAND HANDLER

	//change accented char to plain, detect and change descenders
	//NB descenders only work on 5x10 displays. This lookup table works
	//  with my DEM-20845 (Display Elektronik GmbH) LCD using KS0066 chip.
	switch (rxbyte)
	{
		//chars that have direct equivalent in LCD charmap
/*		case 0x67: //g
			rxbyte = 0xE7;
			break;
		case 0x6A: //j
			rxbyte = 0xEA;
			break;
		case 0x70: //p
			rxbyte = 0xF0;
			break;
		case 0x71: //q
			rxbyte = 0xF1;
			break;
		case 0x79: //y
			rxbyte = 0xF9;
			break;
*/		case 0xE4: //ASCII "a" umlaut
			rxbyte = 0xE1;
			break;
		case 0xF1: //ASCII "n" tilde
			rxbyte = 0xEE;
			break;
		case 0xF6: //ASCII "o" umlaut
			rxbyte = 0xEF; //was wrong in v0.86
			break;
		case 0xFC: //ASCII "u" umlaut
			rxbyte = 0xF5;
			break;

		//accented -> plain equivalent
		//and misc symbol translation
		case 0xA3: //sterling (pounds)
			rxbyte = 0xED;
			break;
/*		case 0xB0: //degrees symbol
			rxbyte = 0xDF;
			break;
*/		case 0xB5: //mu
			rxbyte = 0xE4;
			break;
		case 0xC0: //"A" variants
		case 0xC1:
		case 0xC2:
		case 0xC3:
		case 0xC4:
		case 0xC5:
			rxbyte = 0x41;
			break;
		case 0xC8: //"E" variants
		case 0xC9:
		case 0xCA:
		case 0xCB:
			rxbyte = 0x45;
			break;
		case 0xCC: //"I" variants
		case 0xCD:
		case 0xCE:
		case 0xCF:
			rxbyte = 0x49;
			break;
		case 0xD1: //"N" tilde -> plain "N"
			rxbyte = 0x43;
			break;
		case 0xD2: //"O" variants
		case 0xD3:
		case 0xD4:
		case 0xD5:
		case 0xD6:
		case 0xD8:
			rxbyte = 0x4F;
			break;
		case 0xD9: //"U" variants
		case 0xDA:
		case 0xDB:
		case 0xDC:
			rxbyte = 0x55;
			break;
		case 0xDD: //"Y" acute -> "Y"
			rxbyte = 0x59;
			break;
/*		case 0xDF: //beta  //mucks up LCDSmartie's degree symbol??
			rxbyte = 0xE2;
			break;
*/		case 0xE0: //"a" variants except umlaut
		case 0xE1:
		case 0xE2:
		case 0xE3:
		case 0xE5:
			rxbyte = 0x61;
			break;
		case 0xE7: //"c" cedilla -> "c"
			rxbyte = 0x63;
			break;
		case 0xE8: //"e" variants
		case 0xE9:
		case 0xEA:
		case 0xEB:
			rxbyte = 0x65;
			break;
		case 0xEC: //"i" variants
		case 0xED:
		case 0xEE:
		case 0xEF:
			rxbyte = 0x69;
			break;
		case 0xF2: //"o" variants except umlaut
		case 0xF3:
		case 0xF4:
		case 0xF5:
		case 0xF8:
			rxbyte = 0x6F;
			break;
		case 0xF7: //division symbol
			rxbyte = 0xFD;
			break;
		case 0xF9: //"u" variants except umlaut
		case 0xFA:
		case 0xFB:
			rxbyte = 0x75;
			break;
		default:
			break;
	}

   mlmc_print_char(rxbyte); 
    
  
 
 }

}
