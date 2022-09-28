#include "engine.h"
#include "pinmapping.h"
#include "string.h"

state_t G_state;

////////////////////////////////////////////////////////////////////////////////*/
///                                                                             */
///     ▄▄    ▄▄ ▄▄      ▄▄            ▄▄▄▄▄▄   ▄▄▄   ▄▄   ▄▄▄▄▄▄   ▄▄▄▄▄▄▄▄    */
///     ██    ██ ██      ██            ▀▀██▀▀   ███   ██   ▀▀██▀▀   ▀▀▀██▀▀▀    */
///     ██    ██ ▀█▄ ██ ▄█▀              ██     ██▀█  ██     ██        ██       */
///     ████████  ██ ██ ██               ██     ██ ██ ██     ██        ██       */
///     ██    ██  ███▀▀███               ██     ██  █▄██     ██        ██       */
///     ██    ██  ███  ███             ▄▄██▄▄   ██   ███   ▄▄██▄▄      ██       */
///     ▀▀    ▀▀  ▀▀▀  ▀▀▀             ▀▀▀▀▀▀   ▀▀   ▀▀▀   ▀▀▀▀▀▀      ▀▀       */
///                                                                             */
///                                                                             */
////////////////////////////////////////////////////////////////////////////////*/
void init_hw (void) {
  MAMTIM = 1;
  MAMCR = 1;
  VPBDIV = 0; 
  SCS = 3; 
  PINSEL0 = 0x0;
  PINSEL1 = 0x0;
  PINSEL2 = 0x4;
  //             x  x  
  // 3 12 13 16 28 29
  FIO0SET = 0x30013008;
  //          x           x        x
  // 2 10 11 15 17 18 19 21 25 26 30
  FIO0CLR = 0x462E8C04;
  //              x
  //              RD is not cfg as OUT => unused :)
  FIO0DIR = 0x762F9C0C;
  FIO1PIN = 0;
  FIO1DIR = 0x03FF0000;
  PCONP = 2; // enable PCTIM0
  T0CTCR = 0; // PCLK counter
  T0TCR = 1; // start TIM0
}

//////////////////////////////////////////////////*/
///                                               */
///        ▄▄▄▄   ▄▄▄▄▄▄     ▄▄▄▄▄▄     ▄▄▄▄      */
///      ██▀▀▀▀█  ██▀▀▀▀█▄   ▀▀██▀▀    ██▀▀██     */
///     ██        ██    ██     ██     ██    ██    */
///     ██  ▄▄▄▄  ██████▀      ██     ██    ██    */
///     ██  ▀▀██  ██           ██     ██    ██    */
///      ██▄▄▄██  ██         ▄▄██▄▄    ██▄▄██     */
///        ▀▀▀▀   ▀▀         ▀▀▀▀▀▀     ▀▀▀▀      */
///                                               */
///                                               */
//////////////////////////////////////////////////*/

void gpio_out(uint8_t port, uint8_t pin, uint32_t state) {
  switch(port) {
    case 0:
      if (state) {
        FIO0SET = 1<<pin;
      } else {
        FIO0CLR = 1<<pin;
      }
      break;
    case 1:
      if (state) {
        FIO1SET = 1<<pin;
      } else {
        FIO1CLR = 1<<pin;
      }
      break;
  }
}
uint32_t gpio_in(uint8_t port, uint8_t pin) {
  switch(port) {
    case 0:
      return (FIO0PIN & (1<<pin)) ? ON:OFF;
    case 1:
      return (FIO1PIN & (1<<pin)) ? ON:OFF;
  }
  return 0;
}

void debug_uart(uint8_t b) {
  uint8_t bits=2+8+2;
  uint32_t _in;
  FIO0DIR |= (1<<14);
  //     idle     start    stop      stop
  _in = (1<<0) | (0<<1) | (b<<2) | (1<<10) | (1<<11); // idle before start + 2 stop bit

  while(bits--) {
    gpio_out(0, 14, _in&1); 
    _in>>=1;
    {volatile uint32_t i = 0x100 ; while(i--);}
  }
}

//////////////////////////////////////////////////*/
///                                               */
///     ▄▄▄▄▄▄▄▄   ▄▄▄▄▄▄   ▄▄▄  ▄▄▄  ▄▄▄▄▄▄▄▄    */
///     ▀▀▀██▀▀▀   ▀▀██▀▀   ███  ███  ██▀▀▀▀▀▀    */
///        ██        ██     ████████  ██          */
///        ██        ██     ██ ██ ██  ███████     */
///        ██        ██     ██ ▀▀ ██  ██          */
///        ██      ▄▄██▄▄   ██    ██  ██▄▄▄▄▄▄    */
///        ▀▀      ▀▀▀▀▀▀   ▀▀    ▀▀  ▀▀▀▀▀▀▀▀    */
///                                               */
///                                               */
//////////////////////////////////////////////////*/

uint32_t get_ts(void) {
  uint32_t v = T0TC * 10UL;
  return (v / (147456UL / 4UL)); // time is returned within a 65536 seconds window
}
#define MAX_TS ((0xFFFFFFFFUL*10UL)/(147456UL / 4UL))

uint32_t ts_expired(uint32_t target_ts) {
  uint32_t ts = get_ts();
  debug_uart(ts>>24);
  debug_uart(ts>>16);
  debug_uart(ts>>8);
  debug_uart(ts>>0);
  debug_uart(target_ts>>24);
  debug_uart(target_ts>>16);
  debug_uart(target_ts>>8);
  debug_uart(target_ts>>0);
  {volatile uint32_t i = 0x100 ; while(i--);}
  // THIS IS OK, BUT get_ts has a max value :(
  return (ts - target_ts) < 0x8000000UL;
}

//////////////////////////////////////////////////*/
///                                               */
///     ▄▄▄▄▄▄    ▄▄    ▄▄  ▄▄▄  ▄▄▄  ▄▄▄▄▄▄      */
///     ██▀▀▀▀█▄  ██    ██  ███  ███  ██▀▀▀▀█▄    */
///     ██    ██  ██    ██  ████████  ██    ██    */
///     ██████▀   ██    ██  ██ ██ ██  ██████▀     */
///     ██        ██    ██  ██ ▀▀ ██  ██          */
///     ██        ▀██▄▄██▀  ██    ██  ██          */
///     ▀▀          ▀▀▀▀    ▀▀    ▀▀  ▀▀          */
///                                               */
///                                               */
//////////////////////////////////////////////////*/

void pump(uint8_t state) {
  G_state.pump_state = state;
  // the drive is inverted on the line
  gpio_out(PUMP_PORT, PUMP_PIN, !state);
  if (G_state.pump_state) {
    G_state.blink_flags |= 1<<FLAG_PUMP;
  }
  else {
    G_state.blink_flags &= ~(1<<FLAG_PUMP);
  }
}

//////////////////////////////*/
///                           */
///     ▄▄    ▄▄  ▄▄▄  ▄▄▄    */
///     ██    ██   ██▄▄██     */
///     ██    ██    ████      */
///     ██    ██     ██       */
///     ██    ██    ████      */
///     ▀██▄▄██▀   ██  ██     */
///       ▀▀▀▀    ▀▀▀  ▀▀▀    */
///                           */
///                           */
//////////////////////////////*/

struct {
  uint8_t port;
  uint8_t pin;
} const button_pins[] = {
  {BUTTON_UP_PORT, BUTTON_UP_PIN},
  {BUTTON_SET_PORT, BUTTON_SET_PIN},
  {BUTTON_DOWN_PORT, BUTTON_DOWN_PIN},
};

uint8_t read_button(uint32_t button_idx) {
  return gpio_in(button_pins[button_idx].port, button_pins[button_idx].pin) ? OFF : ON;
}

void button_action(uint32_t button_idx) {
#ifdef DEBUG_SEGMENTS
  switch(button_idx) {
    case BUTTON_UP:
      G_state.fixed_flags--;
      break;
    case BUTTON_SET:
      G_state.fixed_flags = 0;
      break;
    case BUTTON_DOWN:
      G_state.fixed_flags++;
      break;
  } 
  G_state.fixed_flags %= 128;
#else // DEBUG_SEGMENTS
  switch(button_idx) {
    case BUTTON_UP:
      if (G_state.display_state & DISPLAY_FLAG_EDIT) {
        G_state.temps[G_state.display_state&(~DISPLAY_FLAG_EDIT)]+=5;
      }
      else if (G_state.display_state > 0) {
        G_state.display_state--;
      }
      break;
    case BUTTON_SET:
      if (G_state.display_state & DISPLAY_FLAG_EDIT) {
        G_state.display_state &= ~DISPLAY_FLAG_EDIT;
        // TODO save into into long term storage
        G_state.blink_flags &= ~(1<<FLAG_SET);
      }
      else {
        switch(G_state.display_state) {
          case TEMP_DELTASTART:
          case TEMP_DELTASTOP:
          case TEMP_MAX:
          case TEMP_MIN:
            G_state.display_state |= DISPLAY_FLAG_EDIT;
            G_state.blink_flags |= 1<<FLAG_SET;
            break;
        }
      }
      break;
    case BUTTON_DOWN:
      if (G_state.display_state & DISPLAY_FLAG_EDIT) {
        G_state.temps[G_state.display_state&(~DISPLAY_FLAG_EDIT)]-=5;
      }
      else if (G_state.display_state < TEMP_COUNT-1) {
        G_state.display_state++;
      }
      break;
  } 
  screen_repaint();
#endif // DEBUG_SEGMENTS
}

void backlight(uint8_t state) {
  gpio_out(BACKLIGHT_PORT, BACKLIGHT_PIN, state);
}

/* power on backlight and program its shutdown */
void backlight_on(void) {
  backlight(ON);
  G_state.ts_bloff = (get_ts()+TIMEOUT_BLOFF_MS)%MAX_TS;
}

/*
DISPLAY handling:

Temperature panneau
  tPAN xxx°c ('!' panneau)
    UP: nothing
    SET: nothing
    DOWN: v
Temperature haut de ballon
  tTOP  xxx°c ('!' ballhaut)
    UP: ^
    SET: nothing
    DOWN: v
Temperature bas de ballon
  tBOT  xxx°c ('!' ballbas)
    UP: ^
    SET: nothing
    DOWN: v
Delta temperature start (panneau - bas) >= xxx
  dGO xxx°c (default 10°c) 
    UP: ^
    SET: edit
      UP: + 1°c
      SET: end edit
      DOWN: -1°c
    DOWN: v
Delta temperature stop (panneau - bas) <= xxx
  dEND xxx°c (default 4°c)
    UP: ^
    SET: edit
      UP: + 1°c
      SET: end edit
      DOWN: -1°c
    DOWN: v
Delta temperature max ballon <= xxx
  tMAX xxx°c (default 60°c)
    UP: ^
    SET: edit
      UP: + 1°c
      SET: end edit
      DOWN: -1°c
    DOWN: v
Delta temperature min panneau to pump <= xxx
  tMIN xxx°c (default 20°c)
    UP: ^
    SET: edit
      UP: + 1°c
      SET: end edit
      DOWN: -1°c
    DOWN: v
Test screen segments
  test <rand>
    UP: ^
    SET: edit
      UP: + 1
      SET: end edit
      DOWN: -1
    DOWN: v
Version
  VER xxx
    UP: ^
    DOWN: v
*/

char* const U_display_titles[TEMP_COUNT] = {
  "tPAN",
  "tTOP",
  "tBOT",
  "dGO",
  "dEND",
  "tMAX",
  "tMIN",
  "VER",
};

void screen_repaint(void) {
  uint32_t flag;
  uint8_t temp_idx = (G_state.display_state & ~DISPLAY_FLAG_EDIT);
  screen_blank();
#ifdef DEBUG_SEGMENTS
  // set a single segment
  G_state.ht1621b_ram[G_state.fixed_flags/8] = 1<<(G_state.fixed_flags%8);
#else // DEBUG_SEGMENTS
  // display screen segments
  screen_flag(FLAG_TANKANDPANEL);
  G_state.blink_flags &= ~ ((1<<FLAG_TEMPPANEL)|(1<<FLAG_TEMPTOP)|(1<<FLAG_TEMPBOT)|(1<<FLAG_WARNING));
  if (G_state.temps[TEMP_PANEL] == TEMP_UNDEF
    || G_state.temps[TEMP_BOTTOM] == TEMP_UNDEF) {
    G_state.blink_flags |= (1<<FLAG_WARNING);
  }
  if (G_state.temps[TEMP_PANEL] < 0) {
    screen_flag(FLAG_FROST);
  }
  switch(temp_idx) {
    case TEMP_PANEL:
      G_state.blink_flags |= (1<<FLAG_TEMPPANEL);
      screen_flag(FLAG_CELSIUS);
      break;
    case TEMP_TOP:
      G_state.blink_flags |= (1<<FLAG_TEMPTOP);
      screen_flag(FLAG_CELSIUS);
      break;
    case TEMP_BOTTOM:
      G_state.blink_flags |= (1<<FLAG_TEMPBOT);
      screen_flag(FLAG_CELSIUS);
      break;
    case TEMP_DELTASTART:
    case TEMP_DELTASTOP:
      screen_flag(FLAG_TEMPPANEL);
      screen_flag(FLAG_TEMPBOT);
      screen_flag(FLAG_CELSIUS);
      break;
    case TEMP_MAX:
      screen_flag(FLAG_TEMPBOT);
      screen_flag(FLAG_CELSIUS);
      break;
    case TEMP_MIN:
      screen_flag(FLAG_TEMPBOT);
      screen_flag(FLAG_CELSIUS);
      break;
  }
  if (temp_idx <= TEMP_COUNT - 1) {
    screen_string(U_display_titles[temp_idx]);
    screen_number1decimal(G_state.temps[temp_idx]);
  }
  for (flag=0;flag<32;flag++) {
    if (G_state.blink_flags & (1<<flag)) {
      if (G_state.blink_flags_state & (1<<flag)) {
        G_state.blink_flags_state &= ~(1<<flag);
      }
      else {
        G_state.blink_flags_state |= (1<<flag);
        screen_flag(flag);
      }
    }
    if (G_state.fixed_flags & (1<<flag)) {
      screen_flag(flag);
    }
  }
#endif // DEBUG_SEGMENTS
  screen_update();
}

////////////////////////////////////////////////////////////////////////////////*/
///                                                                             */
///     ▄▄▄▄▄▄▄▄            ▄▄▄▄▄▄    ▄▄▄▄▄▄      ▄▄▄▄    ▄▄▄▄▄▄    ▄▄▄▄▄▄▄▄    */
///     ▀▀▀██▀▀▀            ██▀▀▀▀█▄  ██▀▀▀▀██   ██▀▀██   ██▀▀▀▀██  ██▀▀▀▀▀▀    */
///        ██               ██    ██  ██    ██  ██    ██  ██    ██  ██          */
///        ██               ██████▀   ███████   ██    ██  ███████   ███████     */
///        ██               ██        ██  ▀██▄  ██    ██  ██    ██  ██          */
///        ██               ██        ██    ██   ██▄▄██   ██▄▄▄▄██  ██▄▄▄▄▄▄    */
///        ▀▀               ▀▀        ▀▀    ▀▀▀   ▀▀▀▀    ▀▀▀▀▀▀▀   ▀▀▀▀▀▀▀▀    */
///                                                                             */
///                                                                             */
////////////////////////////////////////////////////////////////////////////////*/
void temp_set_source(uint8_t src) {
  switch(src) {
    case 0:
      gpio_out(TEMP_MUX0_PORT, TEMP_MUX0_PIN, OFF);
      gpio_out(TEMP_MUX1_PORT, TEMP_MUX1_PIN, OFF);
      break;
    case 1:
      gpio_out(TEMP_MUX0_PORT, TEMP_MUX0_PIN, ON);
      gpio_out(TEMP_MUX1_PORT, TEMP_MUX1_PIN, OFF);
      break;
    case 2:
      gpio_out(TEMP_MUX0_PORT, TEMP_MUX0_PIN, OFF);
      gpio_out(TEMP_MUX1_PORT, TEMP_MUX1_PIN, ON);
      break;
  }
}

uint8_t temp_exchange_byte(uint8_t _in) {
  uint8_t bits=8;
  uint8_t _out=0;
  gpio_out(TEMP_DIN_PORT, TEMP_DIN_PIN, ON);
  while(bits--) {
    gpio_out(TEMP_SCLK_PORT, TEMP_SCLK_PIN, OFF);
    gpio_out(TEMP_DIN_PORT, TEMP_DIN_PIN, (_in&0x80)?ON:OFF);
    _in<<=1;
    gpio_out(TEMP_SCLK_PORT, TEMP_SCLK_PIN, ON);
    _out = (_out << 1) | (gpio_in(TEMP_DOUT_PORT, TEMP_DOUT_PIN)?1:0);
  }
  return _out;
}

void temp_init(void) {
  temp_set_source(0);
  gpio_out(TEMP_CS_PORT, TEMP_CS_PIN, ON);
  gpio_out(TEMP_SCLK_PORT, TEMP_SCLK_PIN, ON);
  gpio_out(TEMP_DIN_PORT, TEMP_DIN_PIN, OFF);
  gpio_in(TEMP_DOUT_PORT, TEMP_DOUT_PIN);

  gpio_out(TEMP_CS_PORT, TEMP_CS_PIN, OFF);

  // configure AD7790
  temp_exchange_byte(0b00010000);
  temp_exchange_byte(0b10000010);

}

uint8_t temp_is_ready(void) {
  return gpio_in(TEMP_DOUT_PORT, TEMP_DOUT_PIN) == OFF;
}

struct {
  uint16_t R_x10;
  int16_t  T_x10;
} const pt1000_interpolation[] = {
  { (uint32_t)8031, -500 },
  { (uint32_t)8427, -400 },
  { (uint32_t)8822, -300 },
  { (uint32_t)9216, -200 },
  { (uint32_t)9609, -100 },
  { (uint32_t)10000, 0 },
  { (uint32_t)10390, 100 },
  { (uint32_t)10779, 200 },
  { (uint32_t)11167, 300 },
  { (uint32_t)11554, 400 },
  { (uint32_t)11940, 500 },
  { (uint32_t)12324, 600 },
  { (uint32_t)12708, 700 },
  { (uint32_t)13090, 800 },
  { (uint32_t)13471, 900 },
  { (uint32_t)13851, 1000 },
  { (uint32_t)14229, 1100 },
  { (uint32_t)14607, 1200 },
  { (uint32_t)14983, 1300 },
  { (uint32_t)15358, 1400 },
  { (uint32_t)15733, 1500 },
  { (uint32_t)16105, 1600 },
  { (uint32_t)16477, 1700 },
  { (uint32_t)16848, 1800 },
  { (uint32_t)17217, 1900 },
  { (uint32_t)17586, 2000 },
  { (uint32_t)17953, 2100 },
  { (uint32_t)18319, 2200 },
  { (uint32_t)18684, 2300 },
  { (uint32_t)19047, 2400 },
  { (uint32_t)19410, 2500 },
  { (uint32_t)19771, 2600 },
};

/*
On mains (vcc=3.340v):
1276 = 1253/(1963/2000) => measured as 59.5°c (instead of 71°c)
1209 = 1212/(2004/2000) => measured as 44°c (instead of 54°c)
vcc=3.178v
1266 = 1232/(1946/2000) => measured as 59.6°c (instead of 69°c)
*/

int16_t temp_read(void) {
  // if(G_state.fixed_flags & FLAG_RUN) {
  //   G_state.fixed_flags &= ~FLAG_RUN;
  // }
  // else {
  //   G_state.fixed_flags |= FLAG_RUN;
  // }

  // read from ADC LSB is 76uV
  temp_exchange_byte(0x38);
  // substract half the span of the ADC ([-Vref;+Vref])
  uint16_t Upt1k = ((((temp_exchange_byte(0xFF)) << 8 ) | (temp_exchange_byte(0xFF)&0xFF)) & 0xFFFF);
  gpio_out(TEMP_CS_PORT, TEMP_CS_PIN, ON);
  {volatile uint32_t i = 0x10 ; while(i--);}
  gpio_out(TEMP_CS_PORT, TEMP_CS_PIN, OFF);

  // configure AD7790
  temp_exchange_byte(0b00010000);
  temp_exchange_byte(0b10000010);

  // temperature cannot exceed 2kohm of PT1K resistance
  // Upt1k cannot be > Vref, and ADC value is -Vref;+Vref
  if (Upt1k > 0x8000+0x8000/2) {
    return TEMP_UNDEF;
  }

  // Upt1k = Uref * Rpt1k / (Rpt1k + Rref)
  // Rpt1k = Rref * Upt1k / (Uref - Upt1k)
  // Upt1k = AdcStep*Uref
  // Simplification:
  // 2000*((0xB036-0x8000)/0x8000)/(Uref-Uref*((0xB036-0x8000)/0x8000))
  // 2000*((0xB036-0x8000)/0x8000)/(1-((0xB036-0x8000)/0x8000))
  // 2000*0x8000*((0xB036-0x8000)/0x8000)/(0x8000-0x8000*((0xB036-0x8000)/0x8000))
  // 2000*((0xB036-0x8000))/(0x8000-((0xB036-0x8000)))
  // 2000*(0xB036-0x8000)/(0x10000-0xB036)
  uint16_t Rpt1k = 2000UL * ((uint32_t)(Upt1k - 0x8000UL)) / ((uint32_t)( 0x10000UL - Upt1k )) * 10;

  // interpolation of Rpt1k in the pt1000 table
  uint32_t idx = 0;
  // don't compute past the last value
  while (idx+1 < sizeof(pt1000_interpolation) / sizeof(pt1000_interpolation[0])) {
    if (pt1000_interpolation[idx].R_x10 <= Rpt1k && pt1000_interpolation[idx+1].R_x10 > Rpt1k) {
      return (pt1000_interpolation[idx+1].T_x10 - pt1000_interpolation[idx].T_x10) 
             * (Rpt1k - pt1000_interpolation[idx].R_x10) 
             / (pt1000_interpolation[idx+1].R_x10 - pt1000_interpolation[idx].R_x10) 
             + pt1000_interpolation[idx].T_x10;
    }
    idx++;
  }
  return TEMP_UNDEF;
}

void temp_action(void) {
  // invalid temp? safety first, disable and notify
  if (G_state.temps[TEMP_PANEL] == TEMP_UNDEF
    || G_state.temps[TEMP_BOTTOM] == TEMP_UNDEF) {
    pump(OFF);
  }
  // don't pump if panel temp is too low, whatever the storage temperature
  else if (G_state.temps[TEMP_PANEL] <= G_state.temps[TEMP_MIN]) {
    pump(OFF);
  }
  // when temperature reached the maximum (safety) 
  else if (G_state.temps[TEMP_BOTTOM] >= G_state.temps[TEMP_MAX]) {
    pump(OFF);
  }
  // panel - low <= deltastop
  else if (G_state.temps[TEMP_PANEL] - G_state.temps[TEMP_BOTTOM] <= G_state.temps[TEMP_DELTASTOP]) {
    pump(OFF);
  }
  // panel - low >= deltastart
  else if (G_state.temps[TEMP_PANEL] - G_state.temps[TEMP_BOTTOM] >= G_state.temps[TEMP_DELTASTART]) {
    pump(ON);
  }

  // if (G_state.temps_change) {
  //   screen_repaint();
  // }
}

////////////////////////////////////////////////////////////////////////////////*/
///                                                                             */
///     ▄▄▄▄▄▄    ▄▄▄▄▄▄      ▄▄▄▄       ▄▄▄▄   ▄▄▄▄▄▄▄▄    ▄▄▄▄      ▄▄▄▄      */
///     ██▀▀▀▀█▄  ██▀▀▀▀██   ██▀▀██    ██▀▀▀▀█  ██▀▀▀▀▀▀  ▄█▀▀▀▀█   ▄█▀▀▀▀█     */
///     ██    ██  ██    ██  ██    ██  ██▀       ██        ██▄       ██▄         */
///     ██████▀   ███████   ██    ██  ██        ███████    ▀████▄    ▀████▄     */
///     ██        ██  ▀██▄  ██    ██  ██▄       ██             ▀██       ▀██    */
///     ██        ██    ██   ██▄▄██    ██▄▄▄▄█  ██▄▄▄▄▄▄  █▄▄▄▄▄█▀  █▄▄▄▄▄█▀    */
///     ▀▀        ▀▀    ▀▀▀   ▀▀▀▀       ▀▀▀▀   ▀▀▀▀▀▀▀▀   ▀▀▀▀▀     ▀▀▀▀▀      */
///                                                                             */
///                                                                             */
////////////////////////////////////////////////////////////////////////////////*/

void init_state(void) {
  memset(&G_state, 0, sizeof(G_state));
  // temps are in decicelsius
  G_state.temps[TEMP_DELTASTART] = 100;
  G_state.temps[TEMP_DELTASTOP] = 40;
  G_state.temps[TEMP_MAX] = 700;
  G_state.temps[TEMP_MIN] = 200;
  G_state.temps[TEMP_VERSION] = VERSION;

  // start with pump OFF!
  pump(OFF);
}

void process(void) {
  uint32_t idx;

  // handle button press
  for (idx=0; idx < BUTTON_COUNT; idx++) {
    uint8_t buttonstate = read_button(idx);
    if (buttonstate == ON) {
      if (G_state.last_buttons[idx] == OFF) {
        backlight_on();
        button_action(idx);
      }
    }
    G_state.last_buttons[idx] = buttonstate;
  }

  switch(G_state.temp_state) {
    default:
      if (G_state.temp_state >= TEMP_LAST_MEASURED) {
        temp_action();
      }
      temp_set_source(0);
      G_state.temp_state = 0;
      __attribute__((fallthrough));
    case TEMP_PANEL:
    case TEMP_TOP: 
    case TEMP_BOTTOM:
      if (temp_is_ready()) {
        int16_t oldtemp = G_state.temps[G_state.temp_state];
        int16_t temp = temp_read();
        G_state.temps_change &= ~ (1<<G_state.temp_state);
        if (oldtemp != temp) {
          G_state.temps_change |= (1<<G_state.temp_state);
        }
        G_state.temps[G_state.temp_state] = temp;
        G_state.temp_state++;
        temp_set_source(G_state.temp_state);
      }
      break;
  }

  // backlight off
  if (ts_expired(G_state.ts_bloff)) {
    backlight(OFF);
  }

  if (ts_expired(G_state.ts_display)) {
    screen_repaint();
    G_state.ts_display = (get_ts() + TIMEOUT_DISPLAY_ANIMATE_MS)%MAX_TS;
  }
}

int main(void) {
  init_hw();

  temp_init();
  screen_init();
  init_state();

  for (;;) {
    process();
  }
}