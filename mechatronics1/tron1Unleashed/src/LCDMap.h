
//I2C Byte Breakdown
//Bit7=D7
//Bit6=D6
//Bit5=D5
//Bit4=D4
//Bit3=BT(backlight)
//Bit2=E
//Bit1=RW
//Bit0=RS

// LCD Commands
// ---------------------------------------------------------------------------
#define LCD_CLEARDISPLAY        0x10
#define LCD_RETURNHOME          0x20
#define LCD_ENTRYMODESET        0x40
#define LCD_DISPLAYCONTROL      0x80
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80

// flags for display entry mode
// ---------------------------------------------------------------------------
#define LCD_ENTRYSHIFTDISPLAY   0x10
#define LCD_ENTRYSHIFTINCREMENT 0x10
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off and cursor control
// ---------------------------------------------------------------------------
#define LCD_DISPLAYON           0x40
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x20
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x10
#define LCD_BLINKOFF            0x00

// flags for display/cursor shift
// ---------------------------------------------------------------------------
#define LCD_DISPLAYMOVE         0x80
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x40
#define LCD_MOVELEFT            0x00

// flags for function set
// ---------------------------------------------------------------------------
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x80
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x40
#define LCD_5x8DOTS             0x00

// flags for Backpack pins
// ---------------------------------------------------------------------------
#define EN                      0x04
#define READ_FLAG               0x02
#define WRITE_FLAG              0x00
#define RS                      0x01
#define BACKLIGHT               0x08


// ASCII Lettering for bits 7 to 4
#define A7to4                   0x40
#define B7to4                   0x40
#define C7to4                   0x40
#define D7to4                   0x40
#define E7to4                   0x40
#define F7to4                   0x40
#define G7to4                   0x40
#define H7to4                   0x40
#define I7to4                   0x40
#define J7to4                   0x40
#define K7to4                   0x40
#define L7to4                   0x40
#define M7to4                   0x40
#define N7to4                   0x40
#define O7to4                   0x40
#define P7to4                   0x50
#define Q7to4                   0x50
#define R7to4                   0x50
#define S7to4                   0x50
#define T7to4                   0x50
#define U7to4                   0x50
#define V7to4                   0x50
#define W7to4                   0x50
#define X7to4                   0x50
#define Y7to4                   0x50
#define Z7to4                   0x50

//ASCII Lettering for bits 3 to 0
#define A3to0                   0x10
#define B3to0                   0x20
#define C3to0                   0x30
#define D3to0                   0x40
#define E3to0                   0x50
#define F3to0                   0x60
#define G3to0                   0x70
#define H3to0                   0x80
#define I3to0                   0x90
#define J3to0                   0xA0
#define K3to0                   0xB0
#define L3to0                   0xC0
#define M3to0                   0xD0
#define N3to0                   0xE0
#define O3to0                   0xF0
#define P3to0                   0x00
#define Q3to0                   0x10
#define R3to0                   0x20
#define S3to0                   0x30
#define T3to0                   0x40
#define U3to0                   0x50
#define V3to0                   0x60
#define W3to0                   0x70
#define X3to0                   0x80
#define Y3to0                   0x90
#define Z3to0                   0xA0
