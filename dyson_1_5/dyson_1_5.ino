#include "beeps.h"
#include "variables.h"


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
 
#include "interrups.h"


void setup()
{
    Serial.begin(115200);
    pinMode(HALL_EFFECT, INPUT);
    pinMode(HALL_EFFECT, INPUT_PULLUP);
    pinMode(R_IS, INPUT);
    pinMode(L_IS, INPUT);
    pinMode(R_INH, OUTPUT);
    pinMode(L_INH, OUTPUT);
    pinMode(R_IN, OUTPUT);
    pinMode(L_IN, OUTPUT);
    
    disable_driver();
    cli();
    attachInterrupt(digitalPinToInterrupt(HALL_EFFECT), hall_effect_int, CHANGE);

   
      
    // Timer 1
    TCCR1A = 0; // Reset entire TCCR1A to 0 
    TCCR1B = 0; // Reset entire TCCR1B to 0
    TCNT1  = 0; // Clear timer1 counter
  
    // Compare A
    TCCR1B |= B00000010; // CS12 CS11 CS10 -> 010 -> CLK / 8
    TIMSK1 |= B00000010; // Set OCIE1A 1 -> compare match for A
    OCR1A = 12; // 16 MHz / 8 / 12 -> 6us
  
    // Timer 2
    TCCR2A = 0; // Reset entire TCCR2A to 0 
    TCCR2B = 0; // Reset entire TCCR2B to 0

    // Compare A
    TCCR2B |= B00000101; // CS12 CS11 CS10 -> 101 -> CLK / 1024 
    TIMSK2 |= B00000010; //Set OCIE1A to 1 -> compare match for A
    OCR2A = 157; // 16 MHz / 1024 / 157 -> ~10 ms


    TCCR2A =  0;
    TCCR1A =  0;


    // Initial condition setup
    state = FIRST_STATE;
    pulse_duration = pulse_cycle_arr[FIRST_STATE];
    early_pulse_cnt_from_inhibit =  early_pulse_cycles[FIRST_STATE];
    early_pulse_enable = early_pulse_cycles[FIRST_STATE] != 0;

    head_start();
    enable_driver();
    int x = 0;
  while (x < 20)
  {
    beeps_1KHz();
    beeps_2KHz();
    beeps_3KHz();
    x++;
  }
    sei(); // Enable back the interrupts
}
int display_int = 0;

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

    if (data == 'a')
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
      disable_driver();
    }
    if (data == 'b'){
  beeps_1KHz();
  }
  
    if (data == 't')
    { //Test Driver
  
      digitalWrite(L_INH, 0); // Disable RH
      digitalWrite(L_IN, 0);
      digitalWrite(R_INH, 1); // Enable RH
      digitalWrite(R_IN, 1);
      Serial.println(" RH High ");
      delay(10000);
      digitalWrite(R_IN, 0);
      Serial.println(" RH Low ");
      delay(10000);
      digitalWrite(R_INH, 0); // Disable RH
      digitalWrite(L_IN, 1);
      digitalWrite(L_INH, 1); //Enable LH

      Serial.println(" LH High ");
      delay(10000);
      digitalWrite(L_IN, 0);
      Serial.println(" LH Low ");
      delay(10000);
      disable_driver();

    }
  if (data == 'n')
  { 
    head_start();
  }
  }

  int is_r = analogRead(R_IS);
  int is_l = analogRead(L_IS);

  if (ENABLE_CURRENT_LIMIT_CHECK && ((is_r > CURRENT_LIMIT_ANALOG_COUNTER) || (is_l > CURRENT_LIMIT_ANALOG_COUNTER)))
  {
    cli();
    disable_driver();
    Serial.print(" Overcurrent ");
    delay(10000);

  }
  is_r = is_r/ 1023 * 5;
  display_int ++;
  if (display_int > 1000){ 
  display_int = 0;
  //hall_value = digitalRead(HALL_EFFECT);
  Serial.print("Current R: ");
  Serial.print( is_r);
  Serial.print(" L: ");
  Serial.print( is_l);
  Serial.print(" Sensor: ");
  Serial.print( hall_value);
  Serial.print(" speed: ");
  Serial.print(speed);
  Serial.print(" RPM");
  
  Serial.print(" state: ");
  Serial.print(state);

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
  Serial.print(" pulse_duration: ");
  Serial.println(pulse_duration);
  // Serial.print(" early_pulse_enable: ");
  // Serial.print(early_pulse_enable);
  // Serial.print(" early_pulse_cnt_from_inhibit: ");
  // Serial.println(early_pulse_cnt_from_inhibit);
  }

}



