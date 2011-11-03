#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>


/**
 *  * Reverse a single byte 
 *   *
 *    * this was adapted from the example at 
 *     * http://graphics.stanford.edu/~seander/bithacks.html#BitReverseObvious
 *      */
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



int main(void){
	int i ;
	uint8_t cola,colb,rev;
 
	printf("static unsigned char __attribute__ ((progmem)) columna_lookup[] = {\n\t");
	for(i=0;i<16;i++){ 	
		cola = ~(1<<(15-i)>>8);
		printf("0x%2x,",cola);
	}
	printf("\n};\n\n");
	
	printf("static unsigned char __attribute__ ((progmem)) columnb_lookup[] = {\n\t");
	for(i=0;i<16;i++){ 	
    		colb = ~(1<<(i) >> 8);
		printf("0x%x,",colb);
	}
	printf("\n};\n");

	printf("static unsigned char __attribute__ ((progmem)) reverse_lookup[] = {");
	for(i=0;i<256;i++){ 	
		if (i%16==0){
			printf("\n\t");
		}
    		rev = reverse_byte(i);
		printf("0x%02x,",rev);
	}
	printf("\n};\n");


	return 0;
}
