
#include "variables.h"
void disable_driver()
{
  //digitalWrite(R_INH, 0);
  //digitalWrite(L_INH, 0);
  #if defined(ARDUINO_AVR_UNO)
  PORTB &= 0b11001111;
  #endif
  #if defined(ARDUINO_AVR_MEGA256)
  PORTB &=0b00111111; // Change to Mega 2560
  #endif

}

void enable_driver()
{
  //digitalWrite(R_INH, 1);
  //digitalWrite(L_INH, 1);
  #if defined(ARDUINO_AVR_UNO)
  PORTB |=0b00110000;
  #endif
   #if defined(ARDUINO_AVR_MEGA256)
  PORTB |=0b11000000; // Change to Mega 2560
  #endif
}

void forward()
{
  //digitalWrite(R_IN, 1);
  //digitalWrite(L_IN, 0);
   #if defined(ARDUINO_AVR_MEGA256)
  PORTE |= 0b00100000;// Change to Mega 2560
  PORTB &= 0b11011111;
  #endif
  #if defined(ARDUINO_AVR_UNO)
  PORTD |= 0b00001000;
  PORTB &= 0b11110111;
  #endif
}

void reverse()
{
   //digitalWrite(R_IN, 0);
   //digitalWrite(L_IN, 1);
  #if defined(ARDUINO_AVR_MEGA256)
  PORTE &= 0b11011111;// Change to Mega 2560
  PORTB |= 0b00100000;
  #endif
  #if defined(ARDUINO_AVR_UNO)
  PORTD &= 0b11110111;
  PORTB |= 0b00001000;
  #endif
}

void head_start()
{
  // Poking
  cli();
  enable_driver();
  reverse();
  delayMicroseconds(4000);
  forward();
  delayMicroseconds(4000);
  for (int i = 0; i < 10; i++)
  {
    if (digitalRead(HALL_EFFECT))
    {
      reverse();
    }
    else
    {
      forward();
    }
    delayMicroseconds(1500);
  }
  disable_driver();
  sei();
}
