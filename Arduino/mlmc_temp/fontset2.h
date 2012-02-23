
//#include <avr/io.h>
//#include <avr/pgmspace.h>
//#include <avr/interrupt.h>

/*** Font Set 1 ***************************************************************
 *	ASCII chars 33-122, 8x8pix 
 * taken from http://mbed.org/users/simonb/programs/OLEDSeps525f/gpdz5j/docs/OLEDSeps525f_8cpp_source.html
 *
 *****************************************************************************/
static unsigned char __attribute__ ((progmem)) fontSet1[] = { 97, 8, 8,
	0x00,0x00,0x00,0x00,0x06,0x5f,0x5f,0x06, // ! 0        33
	0x00,0x00,0x00,0x07,0x07,0x00,0x07,0x07, // " 1
	0x00,0x14,0x7f,0x7f,0x14,0x7f,0x7f,0x14, // # 2
	0x00,0x00,0x24,0x2e,0x6b,0x6b,0x3a,0x12, // $ 3
	0x00,0x46,0x66,0x30,0x18,0x0c,0x66,0x62, // % 4
	0x00,0x30,0x7a,0x4f,0x5d,0x37,0x7a,0x48, // & 5
	0x00,0x00,0x00,0x00,0x00,0x04,0x07,0x03, // ' 6
	0x00,0x00,0x00,0x00,0x1c,0x3e,0x63,0x41, // ( 7
	0x00,0x00,0x00,0x00,0x41,0x63,0x3e,0x1c, // ) 8
	0x08,0x2a,0x3e,0x1c,0x1c,0x3e,0x2a,0x08, // * 9
	0x00,0x00,0x08,0x08,0x3e,0x3e,0x08,0x08, // + 10
	0x00,0x00,0x00,0x00,0x00,0x80,0xe0,0x60, // , 11
	0x00,0x00,0x08,0x08,0x08,0x08,0x08,0x08, // - 12
	0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x60, // . 13
	0x00,0x60,0x30,0x18,0x0c,0x06,0x03,0x01, // / 14
	0x00,0x3e,0x7f,0x41,0x49,0x41,0x7f,0x3e, // 0 15
	0x00,0x44,0x42,0x7f,0x7f,0x40,0x40,0x00, // 1 16
	0x00,0x62,0x73,0x59,0x49,0x6f,0x66,0x00, // 2 17
	0x00,0x22,0x63,0x49,0x49,0x7f,0x36,0x00, // 3 18
	0x00,0x18,0x1c,0x16,0x53,0x7f,0x7f,0x50, // 4 19
	0x00,0x27,0x67,0x45,0x45,0x7d,0x39,0x00, // 5 20
	0x00,0x3c,0x7e,0x4b,0x49,0x79,0x30,0x00, // 6 21
	0x00,0x03,0x03,0x71,0x79,0x0f,0x07,0x00, // 7 22
	0x00,0x36,0x7f,0x49,0x49,0x7f,0x36,0x00, // 8 23
	0x00,0x06,0x4f,0x49,0x69,0x3f,0x1e,0x00, // 9 24
	0x00,0x00,0x00,0x00,0x00,0x00,0x66,0x66, // : 25
	0x00,0x00,0x00,0x00,0x00,0x80,0xe6,0x66, // ; 26
	0x00,0x08,0x1c,0x36,0x63,0x41,0x00,0x00, // < 27
	0x00,0x24,0x24,0x24,0x24,0x24,0x24,0x00, // = 28
	0x00,0x00,0x41,0x63,0x36,0x1c,0x08,0x00, // > 29
	0x00,0x02,0x03,0x51,0x59,0x0f,0x06,0x00, // ? 30
	0x00,0x3e,0x7f,0x41,0x5d,0x55,0x57,0x1e, // @ 31
	0x00,0x7c,0x7e,0x13,0x13,0x7e,0x7c,0x00, // A 32
	0x00,0x41,0x7f,0x7f,0x49,0x49,0x7f,0x36, // B 33
	0x00,0x1c,0x3e,0x63,0x41,0x41,0x63,0x22, // C 34
	0x00,0x41,0x7f,0x7f,0x41,0x63,0x3e,0x1c, // D 35
	0x00,0x41,0x7f,0x7f,0x49,0x5d,0x41,0x63, // E 36
	0x00,0x41,0x7f,0x7f,0x49,0x1d,0x01,0x03, // F 37
	0x00,0x1c,0x3e,0x63,0x41,0x51,0x73,0x72, // G 38
	0x00,0x7f,0x7f,0x08,0x08,0x7f,0x7f,0x00, // H 39
	0x00,0x00,0x41,0x7f,0x7f,0x41,0x00,0x00, // I 40
	0x00,0x30,0x70,0x40,0x41,0x7f,0x3f,0x01, // J 41
	0x00,0x41,0x7f,0x7f,0x08,0x1c,0x77,0x63, // K 42
	0x00,0x41,0x7f,0x7f,0x41,0x40,0x60,0x70, // L 43
	0x00,0x7f,0x7f,0x0e,0x1c,0x0e,0x7f,0x7f, // M 44
	0x00,0x7f,0x7f,0x06,0x0c,0x18,0x7f,0x7f, // N 45
	0x00,0x3e,0x7f,0x41,0x41,0x41,0x7f,0x3e, // O 46
	0x00,0x41,0x7f,0x7f,0x49,0x09,0x0f,0x06, // P 47
	0x00,0x1e,0x3f,0x21,0x71,0x7f,0x5e,0x00, // Q 48
	0x00,0x41,0x7f,0x7f,0x09,0x19,0x7f,0x66, // R 49
	0x00,0x22,0x67,0x4d,0x59,0x73,0x22,0x00, // S 50
	0x00,0x03,0x41,0x7f,0x7f,0x41,0x03,0x00, // T 51
	0x00,0x7f,0x7f,0x40,0x40,0x7f,0x7f,0x00, // U 52
	0x00,0x1f,0x3f,0x60,0x60,0x3f,0x1f,0x00, // V 53
	0x00,0x7f,0x7f,0x30,0x18,0x30,0x7f,0x7f, // W 54
	0x00,0x43,0x67,0x3c,0x18,0x3c,0x67,0x43, // X 55
	0x00,0x07,0x4f,0x78,0x78,0x4f,0x07,0x00, // Y 56
	0x00,0x47,0x63,0x71,0x59,0x4d,0x67,0x73, // Z 57
	0x00,0x00,0x7f,0x7f,0x41,0x41,0x00,0x00, // [ 58
	0x00,0x01,0x03,0x06,0x0c,0x18,0x30,0x60, // \ 59
	0x00,0x00,0x41,0x41,0x7f,0x7f,0x00,0x00, // ] 60
	0x00,0x08,0x0c,0x06,0x03,0x06,0x0c,0x08, // ^ 61
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80, // _ 62
	0x00,0x00,0x00,0x03,0x07,0x04,0x00,0x00, // ` 63
	0x00,0x20,0x74,0x54,0x54,0x3c,0x78,0x40, // a 64
	0x00,0x41,0x7f,0x3f,0x44,0x44,0x7c,0x38, // b 65
	0x00,0x38,0x7c,0x44,0x44,0x6c,0x28,0x00, // c 66
	0x00,0x38,0x7c,0x44,0x45,0x3f,0x7f,0x40, // d 67
	0x00,0x38,0x7c,0x54,0x54,0x5c,0x18,0x00, // e 68
	0x00,0x48,0x7e,0x7f,0x49,0x03,0x02,0x00, // f 69
	0x00,0x98,0xbc,0xa4,0xa4,0xf8,0x7c,0x04, // g 70
	0x00,0x41,0x7f,0x7f,0x08,0x04,0x7c,0x78, // h 71
	0x00,0x00,0x44,0x7d,0x7d,0x40,0x00,0x00, // i 72
	0x00,0x60,0xe0,0x80,0x80,0xfd,0x7d,0x00, // j 73
	0x00,0x41,0x7f,0x7f,0x10,0x38,0x6c,0x44, // k 74
	0x00,0x00,0x41,0x7f,0x7f,0x40,0x00,0x00, // l 75
	0x00,0x7c,0x7c,0x18,0x38,0x1c,0x7c,0x78, // m 76
	0x00,0x7c,0x7c,0x04,0x04,0x7c,0x78,0x00, // n 77
	0x00,0x38,0x7c,0x44,0x44,0x7c,0x38,0x00, // o 78
	0x00,0x84,0xfc,0xf8,0xa4,0x24,0x3c,0x18, // p 79
	0x00,0x18,0x3c,0x24,0xa4,0xf8,0xfc,0x84, // q 80
	0x00,0x44,0x7c,0x78,0x4c,0x04,0x1c,0x18, // r 81
	0x00,0x48,0x5c,0x54,0x54,0x74,0x24,0x00, // s 82
	0x00,0x00,0x04,0x3e,0x7f,0x44,0x24,0x00, // t 83
	0x00,0x3c,0x7c,0x40,0x40,0x3c,0x7c,0x40, // u 84
	0x00,0x1c,0x3c,0x60,0x60,0x3c,0x1c,0x00, // v 85
	0x00,0x3c,0x7c,0x70,0x38,0x70,0x7c,0x3c, // w 86
	0x00,0x44,0x6c,0x38,0x10,0x38,0x6c,0x44, // x 87
	0x00,0x9c,0xbc,0xa0,0xa0,0xfc,0x7c,0x00, // y 88
	0x00,0x4c,0x64,0x74,0x5c,0x4c,0x64,0x00, // z 89
	0x00,0x08,0x08,0x3e,0x77,0x41,0x41,0x00, // { 90
	0x00,0x00,0x00,0x00,0x77,0x77,0x00,0x00, // | 91
	0x00,0x41,0x41,0x77,0x3e,0x08,0x08,0x00, // } 92
	0x00,0x02,0x03,0x01,0x03,0x02,0x03,0x01, // ~ 93
	0x00,0x00,0x06,0x0f,0x09,0x0f,0x06,0x00, //  94
	0x00,0x20,0x00,0x20,0x20,0xc0,0x80,0xe0, // Ä 95
	0x00,0x00,0x04,0x00,0x04,0x44,0x44,0x44, // Å 96
        0x00,0x00,0x00,0x00,0x06,0x09,0x09,0x06,  // degree sign 97 133
        0x00,0x00,0x00,0x00,0x06,0x09,0x09,0x06,  // degree sign 97 133
	};
