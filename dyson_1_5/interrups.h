void hall_effect_int()
{
  if (!early_pulse_enable)
  {
    enable_driver();
  }

  //hall_value = digitalRead(HALL_EFFECT);
  #if defined(ARDUINO_AVR_UNO)
  hall_value = (PIND & 0b00000100);
  #endif
  //hall_value = (PIND & 0b00000100);
  #if defined(ARDUINO_AVR_MEGA256)
  hall_value = (PINE & 0b00010000); // Changes for MEGA2560
  #endif
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
        disable_driver();
        reverse();
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
        enable_driver();
      }
    }
  }
  else
  {
    if (reverse_cnt >= pulse_duration)
    {
      if (inhibit_cnt_reverse == 0)
      {
        disable_driver();
        forward();
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
        enable_driver();
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