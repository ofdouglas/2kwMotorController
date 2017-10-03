

void hbridge_shutdown(void)
{
  // disable PWM drivers, so the motor is freewheeled

  // disconnect H-bridge from battery ?
}



void hbridge_set_duty(float duty)
{
  // clamp duty cycle to +/- 98%
  // set PWM outputs
  
  // check if system state is STATE_FAULTED?
}


// Periodic 'task' to manage regenerative current.
void hbridge_monitor_ISR(void)
{
  if (current_is_regenerative())
    hbridge_disconnect();
  else
    hrbdige_connect();

  if (vbus_is_overcharged())
    vbus_dump_enable();
  else
    vbus_dump_disable();
}
