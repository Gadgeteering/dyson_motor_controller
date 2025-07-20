#define HALL_EFFECT 2

// H-Bridge Arduino Shield with BTN8982TA
#define R_INH 12
#define L_INH 13
#define R_IN   3
#define L_IN  11
#define R_IS  A0
#define L_IS  A1


// CURRENT_LIMIT_ANALOG_COUNTER / 1023 * 5 * 19.5A = I_LIMIT
#define CURRENT_LIMIT_ANALOG_COUNTER      1000 
#define ENABLE_CURRENT_LIMIT_CHECK        false

#define REF_CYCLE_FROM_DISABLE_TO_ENABLE  8

#define DYSON_V2
// #define DYSON_V6
//#define DYSON_V10


#define ARDUINO_AVR_MEGA256
//#define ARDUINO_AVR_UNO

#if defined(ARDUINO_AVR_UNO)
  // Uno pin assignments
#elif defined(ARDUINO_AVR_MEGA256)
  // Pro Mini assignments
#else
  #error Unsupported board selection.
#endif

#define FIRST_STATE 0



