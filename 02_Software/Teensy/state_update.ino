// Function to catch a large difference in an angle when going fro -pi to pi
float angle_diff(float angle2, float angle1)
{
  float diff;
  diff = angle2 - angle1;

  if (diff>M_PI)
    diff -= 2*M_PI;
  else if (diff<-M_PI)
    diff += 2*M_PI;

  return diff;
}

// Function that updates the state, state vector is [x0, y0, phi, alpha, dalpha, v, dphi]
void state_update()
{
  const float wheel_radius = 0.07; // Radius of the wheels, used to compensate angle (encoders also count the relative motion between body and wheel, which does not account for a velocity in "O")
  float alpha, dphi, dphi2, dalpha; //phi

  // Update stuff depending on the mpu
  //phi = (-ypr[0]-mpu_bias[0]);
  alpha = (ypr[1]-mpu_bias[1]);
  dalpha = -0.0244*gyr[1];//(alpha - state[3].floatingPoint)*(1000000/DT);
  dphi = 0.0244*gyr[2];//angle_diff(phi, state[2].floatingPoint)*(1000000/DT);
  dphi2 = (ds_right-ds_left)/(0.1985)*(1000000/DT);

  // limit accelerations to catch/smoothen sensor faults
  /*if (abs(dalpha-state[4].floatingPoint) > 3.0)
  {
   0.0244*gyr[2] dalpha = 0.99*state[4].floatingPoint+0.01*dalpha;
  }*/

  if (abs(float(0.0244*gyr[2]) - dphi2) > 1.0)
  {
    dphi = dphi2;
    //phi = state[2].floatingPoint + dphi*(DT/1000000.0);
  }

  if (abs(float(-0.0244*gyr[1]) - state[4].floatingPoint)>3.0)
  {
    dalpha = 0.995*state[4].floatingPoint + 0.005*dalpha;
    alpha = 0.995*state[3].floatingPoint + 0.005*alpha;
  }

  //state[2].floatingPoint = phi; // gear angle, invert sign because yaw is defined in a strange way
  state[2].floatingPoint += dphi*(DT/1000000.0);
  state[3].floatingPoint = alpha; // tilt angle
  state[4].floatingPoint = dalpha;
  state[6].floatingPoint = dphi;

  // Update states that do not depent on Mpu data
  state[5].floatingPoint = (ds_left + ds_right)/2*(1000000/DT) + state[4].floatingPoint*wheel_radius; // velocity in m/s
  state[0].floatingPoint += cos(state[2].floatingPoint)*state[5].floatingPoint*(DT/1000000.0); // x
  state[1].floatingPoint += sin(state[2].floatingPoint)*state[5].floatingPoint*(DT/1000000.0); // y

  newdata = true; //set flag to send the data to the raspberry pi
}
