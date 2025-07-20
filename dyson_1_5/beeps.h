#include "variables.h"
#include "motor_cmds.h"



void beeps_xKHz(int mark, int space, int milliseconds)
{

  cli();
  int x = 0;
  while (x < milliseconds)
  { 
    enable_driver();
    reverse();
    delayMicroseconds(mark);
    digitalWrite(R_INH, 0); // Enable RH
    delayMicroseconds(space);
    
    digitalWrite(R_INH, 1); // Enable RH
    delayMicroseconds(mark);
    digitalWrite(R_INH, 0); // Enable RH
    delayMicroseconds(space);
    x = x + 1;
    }

    digitalWrite(L_INH, 0); //Enable LH
    digitalWrite(R_INH, 0); // Enable RH
    sei();
}

beeps_1KHz()
{
  beeps_xKHz( 50,450,100);
}
beeps_2KHz()
{
  beeps_xKHz( 50,200,100);
}
beeps_3KHz()
{
  beeps_xKHz( 50,150,100);
}
