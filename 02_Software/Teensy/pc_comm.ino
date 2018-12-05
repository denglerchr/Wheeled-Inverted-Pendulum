void pc_communication()
{
  //Print every 0.5 seconds
  if (sinceSendtoPC>500)
  {
    //MPU data
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);

    //Akku Voltage
    Serial.print("Vakku\t");
    Serial.println(v_battery);

    //Encoder
    Serial.print("Encoder L/R ");
    Serial.print(ds_left);
    Serial.print("\t");
    Serial.println(ds_right);

    //State
    Serial.println("--State--");
    Serial.print("x:\t");
    Serial.println(state[0].floatingPoint);
    Serial.print("y:\t");
    Serial.println(state[1].floatingPoint);
    Serial.print("phi:\t");
    Serial.println(state[2].floatingPoint);
    Serial.print("alpha:\t");
    Serial.println(state[3].floatingPoint);
    Serial.print("dalpha:\t");
    Serial.println(state[4].floatingPoint);
    Serial.print("v:\t");
    Serial.println(state[5].floatingPoint);
    Serial.print("dphi:\t");
    Serial.println(state[6].floatingPoint);

    //Action
    /*Serial.print("Action:\t");
    Serial.print(action[0].floatingPoint);
    Serial.print("\t");
    Serial.print(action[1].floatingPoint);
    Serial.println();*/

    //Gyro
    /*Serial.print("Gyro:\t");
    Serial.print(gyr[0]);
    Serial.print("\t");
    Serial.print(gyr[1]*0.06103515625);
    Serial.print("\t");
    Serial.print(gyr[2]);
    Serial.println();*/

    //Debugging
    //Serial.print("Temp1:\t");
    //Serial.println(temp1);
    //Serial.print("Temp2:\t");
    //Serial.println(temp2);


    Serial.println();
    sinceSendtoPC = 0;
  }
}
