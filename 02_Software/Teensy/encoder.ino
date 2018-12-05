void read_encoder()
{
  const float alpha = 0.0;

  //negative sign because encoder forward is robot backward
  motorrot = encoder_left.read();
  ds_left = alpha*ds_left - (1-alpha)*(float) motorrot/ENCODER_CPR/MOTOR_GEAR*DISTANCE_PER_ROTATION;
  motorrot = encoder_right.read();
  ds_right = alpha*ds_right - (1-alpha)*(float) motorrot/ENCODER_CPR/MOTOR_GEAR*DISTANCE_PER_ROTATION;

  //reset to 0 because we want only the movement of the last DT
  encoder_left.write(0);
  encoder_right.write(0);
}
