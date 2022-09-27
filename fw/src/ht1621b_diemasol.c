#include "engine.h"
#include "pinmapping.h"
#include "string.h"

/***
DIEMASOL 16 segments order
            bcdemnprkutsahgf

DIEMASOL 7 segments order
            cbadegf

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///                                                                                                                     
///     ▄▄    ▄▄  ▄▄▄▄▄▄▄▄    ▄▄▄       ▄▄▄▄     ▄▄▄▄▄      ▄▄▄     ▄▄▄▄▄▄              ▄▄▄  ▄▄▄     ▄▄     ▄▄▄▄▄▄      
///     ██    ██  ▀▀▀██▀▀▀   █▀██      ██▀▀▀█   █▀▀▀▀██▄   █▀██     ██▀▀▀▀██            ███  ███    ████    ██▀▀▀▀█▄    
///     ██    ██     ██        ██     ██ ▄▄▄          ██     ██     ██    ██            ████████    ████    ██    ██    
///     ████████     ██        ██     ███▀▀██▄      ▄█▀      ██     ███████             ██ ██ ██   ██  ██   ██████▀     
///     ██    ██     ██        ██     ██    ██    ▄█▀        ██     ██    ██            ██ ▀▀ ██   ██████   ██          
///     ██    ██     ██     ▄▄▄██▄▄▄  ▀██▄▄██▀  ▄██▄▄▄▄▄  ▄▄▄██▄▄▄  ██▄▄▄▄██            ██    ██  ▄██  ██▄  ██          
///     ▀▀    ▀▀     ▀▀     ▀▀▀▀▀▀▀▀    ▀▀▀▀    ▀▀▀▀▀▀▀▀  ▀▀▀▀▀▀▀▀  ▀▀▀▀▀▀▀             ▀▀    ▀▀  ▀▀    ▀▀  ▀▀          
///                                                                                                                     
///                                                                                                                     
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


HT1621b memory mapping on segments
==================================
@0
     warning run pause hand
     downarrowleft pumpleft3 pumpleft2 pumpleft
@2
     heaterpipes topbaloonexchtosecond secondbaloonpipes temptopleft
     secondbaloontank bidule tempbotright tempbotleft
@4
     pipe secondpanelpipes secondballonpipetofirstpanel firsttankandpanel 
     leftrightarrow exchtopfirsttanktosecondpanel tempfirstpanel tempsecondpanel
@6
     arrowleftheater frost arrowheater1 tempheaterpipes 
     kelvin celsius set crank
@8
 alpha1
     a h g f
     k u t s
     m n p r
     b c d e
@12
 alpha2
     a h g f
     k u t s
     m n p r
     b c d e
@16
 alpha3
     a h g f
     k u t s
     m n p r
     b c d e
@20
 alpha4
     a h g f
     k u t s
     m n p r
     b c d e
@24
 digit1
     d e g f
     x c b a  x=<sun>
@26
 digit2
     d e g f
     x c b a  x=<hour|min delim>
@28
 digit3
     d e g f
     x c b a  x=<2digit.2digit delim>
@30
 digit4
     d e g f
     x c b a  x=<3digit.1digit delim>

***/

#define font16_first 0x20
// not a lookup table for performances
uint16_t const font16[] = 
#include "font_16segs.h"

#define font7_first 0x20
// not a lookup table for performances
uint16_t const font7[] = 
#include "font_7segs.h"


void screen_blank(void) {
  memset(G_state.ht1621b_ram, 0, sizeof(G_state.ht1621b_ram));
}

void screen_string(const char* str) {
  if (!str) {
    return;
  }
  uint8_t len = strlen(str);
  if (len > 4) {
    len=4;
  }
  uint8_t l;
  for (l=0; l<len;l++) {
    uint16_t charsegs = font16[str[l]-font16_first];
    G_state.ht1621b_ram[4+l*2] = charsegs&0xFF;
    G_state.ht1621b_ram[4+l*2+1] = charsegs>>8;
  }
}

void screen_number1decimal(int16_t val) {
  // avoid digit overflow
  if (val < -999) {
    val = -999;
  }
  if (val > 9999) {
    val = 9999;
  }
  // prepare prepend -
  uint8_t neg=0;
  if (val < 0) {
    neg=1;
    val = -val; // display correctly
  }
  uint8_t digit=4;
  // display the base 0.0
  G_state.ht1621b_ram[12+2] = font7['0'-font7_first];
  // set the 1 decimal dot
  G_state.ht1621b_ram[12+3] = font7['0'-font7_first];
  // override digits depending on the value to display
  while(digit-- && val) {
    char num = (val%10) + '0';
    val/=10;
    G_state.ht1621b_ram[12+digit] = font7[num-font7_first];
  }
  if (neg) {
    // we checked the boundary
    G_state.ht1621b_ram[12+digit] = font7['-'-font7_first];
  }
  G_state.ht1621b_ram[15] |= 1<<7;
}

void screen_flag(uint32_t flag) {
  if (flag < 32) {
    G_state.ht1621b_ram[flag/8] |= 1<<(flag%8);
  }
  else if (flag < 36) {
    flag-=32;
    G_state.ht1621b_ram[12+flag] |= 0x80;
  }
}

void ht1621b_write_BE(uint32_t val, uint32_t bitcount) {
  gpio_out(DISP_WR_PORT, DISP_WR_PIN, ON);
  while(bitcount--) {
    gpio_out(DISP_DATA_PORT, DISP_DATA_PIN, (val & 1<<(bitcount)) ? ON:OFF);
    gpio_out(DISP_WR_PORT, DISP_WR_PIN, OFF);
    gpio_out(DISP_WR_PORT, DISP_WR_PIN, ON);
  }
}

void ht1621b_write_LE(uint32_t val, uint32_t bitcount) {
  uint32_t b=0;
  gpio_out(DISP_WR_PORT, DISP_WR_PIN, ON);
  while(b<bitcount) {
    gpio_out(DISP_DATA_PORT, DISP_DATA_PIN, (val & 1<<(b)) ? ON:OFF);
    gpio_out(DISP_WR_PORT, DISP_WR_PIN, OFF);
    gpio_out(DISP_WR_PORT, DISP_WR_PIN, ON);
    b++;
  }
}

void screen_init(void) {
  gpio_out(DISP_RD_PORT, DISP_RD_PIN, ON);
  gpio_out(DISP_WR_PORT, DISP_WR_PIN, ON);
  gpio_out(DISP_CS_PORT, DISP_CS_PIN, 1);
  gpio_out(DISP_CS_PORT, DISP_CS_PIN, 0);
  // Prefix command
  ht1621b_write_BE(4, 3);
  // SYS EN
  ht1621b_write_BE(2, 9);
  // LCD ON
  ht1621b_write_BE(6, 9);
  // BIAS 1/3 + 4 commons
  ht1621b_write_BE(0x52, 9);
  gpio_out(DISP_CS_PORT, DISP_CS_PIN, 1);
}

void screen_update(void) {
#if 1
  uint8_t wordidx;
  gpio_out(DISP_CS_PORT, DISP_CS_PIN, 0);
  // Write DATA
  ht1621b_write_BE(5, 3);
  // Address = 0
  ht1621b_write_BE(0, 6);
  // copy buffer into ht1621b chip
  for (wordidx = 0; wordidx < 32 ; wordidx ++) {
    // fetch nibble corresponding to the word
    uint8_t nibble = (G_state.ht1621b_ram[wordidx/2]>>((wordidx%2)*4)) & 0xF;
    // TODO reverse here ? is bits are not in the right endianness
    ht1621b_write_LE(nibble, 4);
  }
  gpio_out(DISP_CS_PORT, DISP_CS_PIN, 1);
#else
  gpio_out(DISP_CS_PORT, DISP_CS_PIN, 0);
  // Write DATA
  ht1621b_write_BE(5, 3);
  // Address = 0
  ht1621b_write_BE(0, 6);
  for (uint32_t i = 0; i < 128 ; i++) {
    if (G_state.fixed_flags == i) {
      ht1621b_write_BE(1, 1);
    }
    else {
      ht1621b_write_BE(0, 1);
    }
  }
  gpio_out(DISP_CS_PORT, DISP_CS_PIN, 1);
#endif
}
