#pragma once
#include "stdint.h"
#include "stddef.h"

#include "lpc213x.h"

//#define DEBUG_SEGMENTS

#define OFF 0
#define ON 1

typedef enum {
  BUTTON_UP=0,
  BUTTON_SET,
  BUTTON_DOWN,
  BUTTON_COUNT
} pins_e;

typedef enum {
  TEMP_PANEL,
  TEMP_TOP,
  TEMP_BOTTOM,
  TEMP_DELTASTARTTOP,
  TEMP_DELTASTOPTOP,
  TEMP_DELTASTARTBOT,
  TEMP_DELTASTOPBOT,
  TEMP_MAX,
  TEMP_MIN,
  TEMP_VERSION,
  // last entry, for auto propagation in algorithms
  TEMP_COUNT,
} temp_e;

typedef struct {
  #define TIMEOUT_BLOFF_MS 10000
  uint32_t ts_bloff;
  uint8_t last_buttons[BUTTON_COUNT];
  #define TEMP_LAST_MEASURED 3
  #define TEMP_UNDEF ((int16_t)-998)
  int16_t temps[TEMP_COUNT];
  uint8_t temps_change; // mark changes in the temps buffer
  uint8_t temp_state;

  #define DISPLAY_FLAG_EDIT 0x80
  uint8_t display_state; // 0xYZ (Z is the text and temp ref, Y is the editing mode)
  uint32_t fixed_flags; // requested flags to display
  uint32_t blink_flags; // requested flags to blink
  uint32_t blink_flags_state; // current blink state
  #define TIMEOUT_DISPLAY_ANIMATE_MS 250
  uint32_t ts_display; // timeout for next screen animation
  uint8_t ht1621b_ram[(32*4)/8];

  #define TIMEOUT_OUTPUT 1000
  uint32_t ts_output_next;

  // no button interp below a debounce period
  #define TIMEOUT_NEXT_BUTTON 100
  uint32_t ts_next_button;

  uint8_t pump_state;
  uint8_t pump_disabled;
} state_t;

extern state_t G_state;

void screen_blank(void);
void screen_string(const char* str);
void screen_number1decimal(int16_t val);
void screen_flag(uint32_t flag);
void screen_init(void);
void screen_update(void);
void screen_repaint();

typedef enum {
  FLAG_HAND=0,
  FLAG_PAUSE,
  FLAG_RUN,
  FLAG_WARNING,
  FLAG_PUMP,
  FLAG_PUMP2,
  FLAG_PUMP3,
  FLAG_DOWNARROW1,
  FLAG_TEMPTOP,
  FLAG_TANK2PIPE,
  FLAG_TOPTOLEFTPIPE,
  FLAG_HEATERPIPE,
  FLAG_TEMPBOT,
  FLAG_TEMPBOTTANK2,
  FLAG_BIDULE,
  FLAG_TANK2,
  FLAG_TANKANDPANEL,
  FLAG_PIPETANK2TOPANEL,
  FLAG_PANEL2PIPE,
  FLAG_PIPE,
  FLAG_TEMPPANEL2,
  FLAG_TEMPPANEL,
  FLAG_XCHGTANKTOPANEL2,
  FLAG_LEFTARROW,
  FLAG_TEMPHEATERPIPE,
  FLAG_HEATERARROW,
  FLAG_FROST,
  FLAG_HEATERARROW2,
  FLAG_CRANK,
  FLAG_SET,
  FLAG_CELSIUS,
  FLAG_KELVIN,
} screen_flag_e;

void gpio_out(uint8_t port, uint8_t pin, uint32_t state);
uint32_t gpio_in(uint8_t port, uint8_t pin);
