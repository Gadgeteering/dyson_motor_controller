/* PWM in pin - D8
 * High A - D9
 * LOW A - D4
 * HIGH B - D10
 * LOW B - D3
 * HIGH C - D11
 * LOW C - D2
 * Comparator - D6
*/
#include <Arduino.h>  // for type definitions

#define PWM_max_value      255
#define PWM_min_value      35
#define PWM_value    35
#define speed_min 1
#define speed_max 10000


#define HALL_EFFECT 8 //PB0

// H-Bridge Arduino Shield with Electronoobs


// CURRENT_LIMIT_ANALOG_COUNTER / 1023 * 5 * 19.5A = I_LIMIT
#define CURRENT_LIMIT_ANALOG_COUNTER      1000 
#define ENABLE_CURRENT_LIMIT_CHECK        false

#define REF_CYCLE_FROM_DISABLE_TO_ENABLE  8

 #define DYSON_V2
// #define DYSON_V6
//#define DYSON_V10

void test_seq()
{
test_AL();
test_BL();
test_AH();
test_BH();
}

void test_AL()
{
  cli();
  all_off();
  PORTD = B0010000;
  Serial.println("AL");
  delay(20000);
  sei(); //reset interrupt
}

void test_AH()
{
  cli();
  all_off();
  PORTB |= B0000010;
  Serial.println("AH");
  delay(20000);
  sei(); //reset interrupt
}

void test_BH()
{
  cli();
  all_off();
  PORTB |= B0000100;
  Serial.println("BH");
  delay(20000);
  sei(); //reset interrupt
}

void test_BL()
{
  cli();
  all_off();
  PORTD = B0001000;
  Serial.println("BL");
  delay(20000);
  sei(); //reset interrupt
}

void forward()
//D10 PWM and D2 HIGH
//BH_CL
{
  all_off();
  PORTD = B0010000;      //Set D2 (AL) to HIGH and the rest to LOW
  PORTB |= B0000100;    //Set B2 (BH) to HIGH and the rest to LOW

}
void reverse()
//D11 PWM and D3 HIGH
//CH_BL()
{
  all_off();
  PORTD = B0001000;      //Set D3 (BL) to HIGH and the rest to LOW
  PORTB |= B0000010;    //Set B3 (AH) to HIGH and the rest to LOW

  
}

void all_off()

//All MOSFET off
{
  TCCR1A =  0;            // OC1A and OC1B normal port
  TCCR2A =  0x0;         // OC2A - D11 (CH) compare match noninverting mode, downcounting ,PWM 8-bit
  PORTD = B0000000;      //Set D3 (BL) to HIGH and the rest to LOW
  PORTB  &= B0110001;     //Set B3 (CH) to HIGH and the rest to LOW
  
}


/*This function will only change the PWM values according to the received width_value
that is given by the PWM read on pin D8*/

void SET_PWM(byte width_value){
  //We keep the range of PWM between min and max (8 bit value)
  if(width_value < PWM_min_value)    width_value  = PWM_min_value;
  if(width_value > PWM_max_value)    width_value  = PWM_max_value;
  OCR1A  = width_value;                   // Set pin 9  PWM duty cycle
  OCR1B  = width_value;                   // Set pin 10 PWM duty cycle
  OCR2A  = width_value;                   // Set pin 11 PWM duty cycle
}


void head_start()
{
  // Poking
  reverse();
  delayMicroseconds(4000);
  forward();
  int i =digitalRead(HALL_EFFECT);
  Serial.print(i);
  delayMicroseconds(4000);
  for (int i = 0; i < 10; i++)
  {
    if (digitalRead(HALL_EFFECT))
    {
      reverse();
      int i =digitalRead(HALL_EFFECT);
      Serial.print("R:");
      Serial.println(i);
    }
    else
    {
      forward();
      int i =digitalRead(HALL_EFFECT);
      Serial.print("F:");
      Serial.println(i);
    }
    delayMicroseconds(500);
  }
}

#define FIRST_STATE 0

const uint8_t early_pulse_cycles[]
{
#ifdef DYSON_V10
  0,
  0,
  0,
  0,
  0,
  0,
  REF_CYCLE_FROM_DISABLE_TO_ENABLE,
  REF_CYCLE_FROM_DISABLE_TO_ENABLE,
  REF_CYCLE_FROM_DISABLE_TO_ENABLE - 2,
  REF_CYCLE_FROM_DISABLE_TO_ENABLE - 2,
  REF_CYCLE_FROM_DISABLE_TO_ENABLE - 2,
  REF_CYCLE_FROM_DISABLE_TO_ENABLE - 3,
  REF_CYCLE_FROM_DISABLE_TO_ENABLE - 4,
#elif defined(DYSON_V6)
  0,
  0,
  0,
  REF_CYCLE_FROM_DISABLE_TO_ENABLE - 3,
  REF_CYCLE_FROM_DISABLE_TO_ENABLE - 4,
  REF_CYCLE_FROM_DISABLE_TO_ENABLE - 5,
#elif defined(DYSON_V2)
  0,
  0,
  0,
  REF_CYCLE_FROM_DISABLE_TO_ENABLE - 4,
  REF_CYCLE_FROM_DISABLE_TO_ENABLE - 4,
#endif
};

const uint8_t pulse_cycle_arr[]
{
#ifdef DYSON_V10
  170,
  130,
   60,
   42,
   24,
   15,
   13,
   10,
   6,
   3,
   1,
   1,
   1,
#elif defined(DYSON_V6)
  150,
   42,
   24,
   19,
   13,
   9,
#elif defined(DYSON_V2)
  100,
   45,
   34,
   25,
   25,
#endif
};

const float rpm_thresholds[][2]
{
#ifdef DYSON_V10
  {   0.f,     300.f},
  { 700.f,    1200.f},
  { 2000.f,   5000.f},
  { 3750.f,   7500.f},
  { 5000.f,  14000.f},
  {12000.f,  20000.f},
  {22000.f,  30000.f},
  {25000.f,  40000.f},
  {35000.f,  50000.f},
  {45000.f,  70000.f},
  {60000.f,  85000.f},
  {70000.f,  90000.f},
  {70000.f, 140000.f},
#elif defined(DYSON_V6)
  {0.f,       10000.f},
  {7500.f,    15000.f},
  {10000.f,   35000.f},
  {30000.f,   65000.f},
  {60000.f,   80000.f},
  {70000.f,  140000.f},
#elif defined(DYSON_V2)
  {0.f,      20000.f},
  {15000.f,  30000.f},
  {20000.f,  70000.f},
  {60000.f, 100000.f},
  {80000.f, 150000.f}
#endif
};

volatile uint8_t state = FIRST_STATE;
volatile float speed = 0.f; // in RPM
volatile bool hall_value = false;
volatile bool early_pulse_enable = false;

volatile uint8_t forward_cnt = 0;
volatile uint8_t reverse_cnt = 0;
volatile uint8_t forward_cnt_happenned = 0;
volatile uint8_t reverse_cnt_happenned = 0;

volatile uint8_t inhibit_cnt_forward = 0;
volatile uint8_t inhibit_cnt_reverse = 0;
volatile uint8_t pulse_duration;
volatile uint8_t early_pulse_cnt_from_inhibit;
volatile uint8_t disable_to_hall_effect;

volatile unsigned long time_rising = 0;
volatile unsigned long time_falling = 0;
volatile unsigned long delta_time_high = 0;
volatile unsigned long delta_time_low = 0;
 
void hall_effect_int()
{
  if (!early_pulse_enable)
  {
    //enable_driver();
  }

  // hall_value = digitalRead(HALL_EFFECT);
  hall_value = (PINB & B00000001);// Pin 8 

  if (hall_value)
  {
    time_rising = micros();
    delta_time_high = time_rising - time_falling;
    forward_cnt_happenned = forward_cnt;
    forward_cnt = 0;
    inhibit_cnt_forward = 0;
  }
  else
  {
    time_falling = micros();
    delta_time_low = time_falling - time_rising;
    reverse_cnt_happenned = reverse_cnt;
    reverse_cnt = 0;
    inhibit_cnt_reverse = 0;

    // Measures number of cycles between driver-off and hall effect sensor change
    disable_to_hall_effect = inhibit_cnt_forward;
  }
}

ISR(TIMER1_COMPA_vect){
  TCNT1  = 0; // Clear for every interrupt

  if (hall_value)
  {
    if (forward_cnt >= pulse_duration)
    {
      if (inhibit_cnt_forward == 0)
      {
        //disable_driver();
        all_off();
      }
      inhibit_cnt_forward++;
    }
    else
    {
      forward_cnt++;
    }

    if (early_pulse_enable)
    {
      if (inhibit_cnt_forward == early_pulse_cnt_from_inhibit)
      {
        //enable_driver();
        reverse();
      }
    }
  }
  else
  {
    if (reverse_cnt >= pulse_duration)
    {
      if (inhibit_cnt_reverse == 0)
      {
        //disable_driver();
        all_off();
        
      }
      inhibit_cnt_reverse++;
    }
    else
    {
      reverse_cnt++;
    }

    if (early_pulse_enable)
    {
      if (inhibit_cnt_reverse == early_pulse_cnt_from_inhibit)
      {
        //enable_driver();
        forward();
      }
    }
  }
}

volatile uint8_t cnt = 0;

ISR(TIMER2_COMPA_vect){
  TCNT2  = 0; // Clear for every interrupt

  if (++cnt >= 25) // 25 * 10ms = 250ms
  {
    cnt = 0;
  
    if (speed > rpm_thresholds[state][1])
    {
      state++;
    }
  
    if (speed < rpm_thresholds[state][0])
    {
      state--;
    }

    // Update pulse control parameters
    pulse_duration = pulse_cycle_arr[state];
    early_pulse_cnt_from_inhibit =  early_pulse_cycles[state];
    early_pulse_enable = early_pulse_cycles[state] != 0;
  }
}


int display_int;
int head_count;

void setup()
{
    Serial.begin(115200);
    //pinMode(HALL_EFFECT, INPUT_PULLUP);
    //Our pins for the MNSFET drivers are 2,3,4 and 9,10,11
    DDRD   = B0011100;           //Configure pins 2, 3 and 4 as outputs CL, BL and AL
    PORTD  = B0000000;           //Pins 0 to 7 set to LOW
    DDRB   = B0001110;           //Configure pins 9, 10 and 11 as outputs
    PORTB  = B0000000;          //D9, D10 and D11 to LOW
    pinMode(HALL_EFFECT, INPUT);


    cli();

    attachInterrupt(digitalPinToInterrupt(HALL_EFFECT), hall_effect_int, CHANGE);
  
        // Timer 1
    TCCR1A = 0; // Reset entire TCCR1A to 0 
    TCCR1B = 0; // Reset entire TCCR1B to 0
    TCNT1  = 0; // Clear timer1 counter
  
    // Compare A
    TCCR1B |= B0000010; // CS12 CS11 CS10 -> 010 -> CLK / 8
    TIMSK1 |= B0000010; // Set OCIE1A 1 -> compare match for A
    OCR1A = 12; // 16 MHz / 8 / 12 -> 6us
  
    // Timer 2
    TCCR2A = 0; // Reset entire TCCR2A to 0 
    TCCR2B = 0; // Reset entire TCCR2B to 0


    // Compare A
    TCCR2B |= B0000101; // CS12 CS11 CS10 -> 101 -> CLK / 1024 
    TIMSK2 |= B0000010; //Set OCIE1A to 1 -> compare match for A
    OCR2A = 157; // 16 MHz / 1024 / 157 -> ~10 ms

    Serial.println("start");
    //enable_driver();
    head_count = 0;
    while(head_count<5)
    {
    head_start();
    head_count ++;
    }
    Serial.println("head_start");
    // Initial condition setup
    state = FIRST_STATE;
    pulse_duration = pulse_cycle_arr[FIRST_STATE];
    early_pulse_cnt_from_inhibit =  early_pulse_cycles[FIRST_STATE];
    early_pulse_enable = early_pulse_cycles[FIRST_STATE] != 0;
    display_int = 0;
    //while(1)
    //{
    //  test_AL();
    //  test_BL();
    //  test_AH();
    //  test_BH();
    //}
    sei(); // Enable back the interrupts

}

void loop()
{
  float period_time = (float)(delta_time_high + delta_time_low);

#ifdef DYSON_V10 // 8 poles, single phase
  speed = (period_time != 0.f) ? (15.f * 1000000.f / period_time) : 0.f;
#elif defined(DYSON_V6) // 4 poles, single phase
  speed = (period_time != 0.f) ? (30.f * 1000000.f / period_time) : 0.f;
#elif defined(DYSON_V2) // 2 poles, single phase
  speed = (period_time != 0.f) ? (60.f * 1000000.f / period_time) : 0.f;
#endif

  // These are custom inputs for debugging
  if (Serial.available())
  {
    char data = Serial.read();

    if (data == 'x')
    {
      pulse_duration++;
    }
    if (data == 'z')
    {
      pulse_duration--;
    }
    if (data == 'w')
    {
      early_pulse_cnt_from_inhibit++;
    }
    if (data == 'q')
    {
      early_pulse_cnt_from_inhibit--;
    }
    if (data == 'e')
    {
      early_pulse_enable = true;
    }
    if (data == 's')
    {
      cli();
      all_off();
    }
    if (data == 'S')
    {
      head_start();
    }
    if (data == 'F')
    {
      forward();
    }
    if (data == 'R')
    {
      reverse();
    }
    if (data == 'P')
    {
      head_start();
    }

    if (data == 'a')
    {
      test_AL();
    }
    if (data == 'A')
    {
      test_AH();
    }
    if (data == 'b')
    {
      test_BL();
    }
    if (data == 'B')
    {
      test_BH();
    }
    if (data == 'T')
    {
    head_count = 0;
      while(head_count<10)
      {
        test_seq();
        head_count ++;
      }
    }
    

    
  }
  display_int ++;
  if (display_int > 20000){
  hall_value = (PINB & B00000001);// Pin 8 
  display_int = 0;
  Serial.print(" Hall: ");
  Serial.print(hall_value);
  Serial.print(" ");

  int motor_speed = map(speed,rpm_thresholds[state][0],rpm_thresholds[state][1],PWM_min_value,PWM_max_value);

  SET_PWM(motor_speed);
  Serial.print(" PWM: ");
  Serial.print(motor_speed);
  Serial.print("% ");

  Serial.print(" speed: ");
  Serial.print(speed);
  Serial.print(" RPM");
  
  Serial.print(" state: ");
  Serial.println(state);

  // Serial.print("high: ");
  // Serial.print(delta_time_high);
  // Serial.print(" low: ");
  // Serial.print(delta_time_low);
  // Serial.print(" forward_cnt: ");
  // Serial.print(forward_cnt_happenned);
  // Serial.print(" reverse_cnt: ");
  // Serial.print(reverse_cnt_happenned);

  // Serial.print(" disable_to_hall_effect: ");
  // Serial.print(disable_to_hall_effect);
  // Serial.print(" is_r: ");
  // Serial.print(is_r);
  // Serial.print(" is_l: ");
  // Serial.print(is_l);
  // Serial.print(" pulse_duration: ");
  // Serial.print(pulse_duration);
  // Serial.print(" early_pulse_enable: ");
  // Serial.print(early_pulse_enable);
  // Serial.print(" early_pulse_cnt_from_inhibit: ");
  // Serial.println(early_pulse_cnt_from_inhibit);
  }
}
