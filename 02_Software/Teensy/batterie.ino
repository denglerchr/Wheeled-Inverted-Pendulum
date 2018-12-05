void get_batterie_voltage()
{
  const float alpha = 0.9;

  //soften by taking moving average
  v_battery = alpha*v_battery + (1-alpha)*3.3*4*(float)(analogRead(PIN_AKKU_ANALOG))/1023; //read value is 4 times the voltage at the input
}
