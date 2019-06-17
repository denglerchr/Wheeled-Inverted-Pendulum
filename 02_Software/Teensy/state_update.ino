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
  // Update the sensor data
  state_sensor_update();

  // Outlier detection by comparing sensor value with model
  if (check_sensor_values())
  {
    // Use sensor data
    state[0].floatingPoint = state_sensor[0];
    state[1].floatingPoint = state_sensor[1];
    state[2].floatingPoint = state_sensor[2];
    state[3].floatingPoint = state_sensor[3];
    state[4].floatingPoint = state_sensor[4];
    state[5].floatingPoint = state_sensor[5];
    state[6].floatingPoint = state_sensor[6];
  }
  else
  {
    // Overwrite state_sensor values that were wrongly integrated
    state_sensor[0] = state_model[0];
    state_sensor[1] = state_model[1];
    state_sensor[2] = state_model[2];
    // Use model prediction
    state[0].floatingPoint = state_model[0];
    state[1].floatingPoint = state_model[1];
    state[2].floatingPoint = state_model[2];
    state[3].floatingPoint = state_model[3];
    state[4].floatingPoint = state_model[4];
    state[5].floatingPoint = state_model[5];
    state[6].floatingPoint = state_model[6];
  }

  newdata = true; //set flag to send the data to the raspberry pi
  newprediction = true; //set flag to make a new prediction based on the new state
}

void state_sensor_update()
{
  const float wheel_radius = 0.07; // Radius of the wheels, used to compensate angle (encoders also count the relative motion between body and wheel, which does not account for a velocity in "O")
  float dphi;

  // Get state from sensors
  dphi = 0.0244*gyr[2];//angle_diff(phi, state[2].floatingPoint)*(1000000/DT);
  //dphi2 = (ds_right-ds_left)/(0.1985)*(1000000/DT);

  //Update states depending on the mpu
  state_sensor[2] += dphi*(DT/1000000.0);
  state_sensor[3] = (ypr[1]-mpu_bias[1]); // tilt angle
  state_sensor[4] = -0.0244*gyr[1];
  state_sensor[6] = dphi;

  // Update states that do not depent on Mpu data
  state_sensor[5] = (ds_left + ds_right)/2*(1000000/DT) + state_sensor[4]*wheel_radius; // velocity in m/s
  state_sensor[0] += cos(state_sensor[2])*state_sensor[5]*(DT/1000000.0); // x
  state_sensor[1] += sin(state_sensor[2])*state_sensor[5]*(DT/1000000.0); // y
}

//returns true if sensors data seems ok, else return false
// The threshold is increase if the state was not ok, as the next
// prediction will be made on the model state, thus
// increasing the threshold to circumvent accumulating errors
bool check_sensor_values()
{
  const float alpha_th_min = 1.0;
  const float dalpha_th_min = 4.0; //actually the model is bad at predicting this in some cases
  const float dphi_th_min = 3.0;

  // Ignore model and use sensor if the segway has fallen down
  if (abs(state[3].floatingPoint) > 50.0*M_PI/180.0)
    return true;

  // Check if deviation between sensor and model are inside the tolerance
  if ((abs(state_sensor[3]-state_model[3]) > alpha_th) || (abs(state_sensor[4]-state_model[4]) > dalpha_th) || (abs(state_sensor[6]-state_model[6]) > dphi_th))
  {
    alpha_th *= 1.2;
    dalpha_th *= 1.2;
    dphi_th *= 1.2;
    return false;
  }

  // this should be the most common case, sensor ok and not fallen down
  alpha_th = max(alpha_th/1.2, alpha_th_min);
  dalpha_th = max(dalpha_th/1.2, dalpha_th_min);
  dphi_th = max(dphi_th/1.2, dphi_th_min);
  return true;
}
