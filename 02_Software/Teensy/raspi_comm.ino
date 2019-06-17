void RASPBERRY_SERIALFlush()
{
  while(RASPBERRY_SERIAL.available() > 0)
  {
    RASPBERRY_SERIAL.read();
  }
}


void raspi_communication()
{
  // Send state after receiving something from raspi and having new data
  if (RASPBERRY_SERIAL.available() && newdata)
  {
    // Send datan_bytes_to_come
    for (i=0; i<7; i++)
    {
      RASPBERRY_SERIAL.write(state[i].binary, 4);
    }
    //raspi_latency = sinceSendtoRasp;
    newdata = false;
    sinceSendtoRasp = 0;

    // Read status from raspberry pi
    n_bytes_to_come = RASPBERRY_SERIAL.read();

    if (n_bytes_to_come == 1)
    {
      Serial.print("Only 1 byte: ");
      Serial.println((uint8_t)RASPBERRY_SERIAL.read());
      n_bytes_to_come -= 1;
    }
    else if (n_bytes_to_come == 4*2) //read out the bytes of "u"
    {
      i = 0;
      j = 0;
      while (n_bytes_to_come>0)
      {
        if (RASPBERRY_SERIAL.available())
        {
            action[i].binary[j] = RASPBERRY_SERIAL.read();
            n_bytes_to_come -= 1;
            j += 1;
            if (j>=4)
            {
              j = 0;
              i +=1;
            }
        }
      }
    }
    else //Something wrong
    {
      Serial.print(n_bytes_to_come);
      Serial.println(" bytes to come...");
      RASPBERRY_SERIALFlush();
      n_bytes_to_come = 0;
    }

    // Limit u between -1 an 1
    if (action[0].floatingPoint > 1.0)
    {
      action[0].floatingPoint = 1.0;
    }
    else if (action[0].floatingPoint < -1.0)
    {
      action[0].floatingPoint = -1.0;
    }

    if (action[1].floatingPoint > 1.0)
    {
      action[1].floatingPoint = 1.0;
    }
    else if (action[1].floatingPoint < -1.0)
    {
      action[1].floatingPoint = -1.0;
    }

  }
  else if ((sinceSendtoRasp>=500000) && newdata) //long time nothing from raspi, try sending state anyway
  {
    for (i=0; i<7; i++)
    {
      RASPBERRY_SERIAL.write(state[i].binary, 4);
    }
    Serial.println("No answer from raspi, trying to send anyway");
    n_bytes_to_come = 0;
    action[0].floatingPoint = 0.0;
    action[1].floatingPoint = 0.0;
    //Set zero order states to the origin
    state[0].floatingPoint = 0.0;
    state[1].floatingPoint = 0.0;
    state[2].floatingPoint = 0.0;
    state_sensor[0] = 0.0;
    state_sensor[1] = 0.0;
    state_sensor[2] = 0.0;
    mpu_bias[0] = -ypr[0]; // force gear angle to be 0
    newdata = false;
    sinceSendtoRasp = 0;
  }

}
