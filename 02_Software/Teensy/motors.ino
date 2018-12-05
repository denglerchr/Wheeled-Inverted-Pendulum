void set_motors()
{
  // Convert control signal (must be between -1 and 1), to a PWM signal
  // Max voltage applied is 10.5V

  //stop if battery is low or robot is at ground
  if (v_battery<10.5 || abs(ypr[1]-mpu_bias[1])>1.3)
  {
    speed_left = 0;
    speed_right = 0;
    digitalWrite(PIN_RED_LED, HIGH);
    action[0].floatingPoint = 0.0;
    action[1].floatingPoint = 0.0;
  }
  else
  {

    // Compute low level variables for the motors
    digitalWrite(PIN_RED_LED, LOW);
    speed_left = static_cast<uint8_t>(abs(action[0].floatingPoint)*10.5/v_battery*255);
    speed_right = static_cast<uint8_t>(abs(action[1].floatingPoint)*10.5/v_battery*255);
    (sign(action[0].floatingPoint)>0) ? dir_left = true : dir_left = false;
    (sign(action[1].floatingPoint)>0) ? dir_right = true : dir_right = false;
  }

  //Unocmment to overwrite to test motors
  /*speed_left = 0;
  speed_right = 50;
  dir_left = false;
  dir_right = true;*/

  //Write signals to GPIO
  analogWrite(PIN_PWM_LEFT, speed_left);
  digitalWrite(PIN_DIR_LEFT, dir_left);

  analogWrite(PIN_PWM_RIGHT, speed_right);
  digitalWrite(PIN_DIR_RIGHT, dir_right);
}
