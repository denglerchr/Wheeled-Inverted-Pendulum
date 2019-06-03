void dxdt_segway(float* xdot, binaryFloat* x, float* u);

void statepredict(float* x2, binaryFloat* x, float* u)
{
  //use RungeKutta4 and a segway model to estimate next state
  const float dt2 = DT/2000000.0;
  float temp[7];
  float dy1[7];
  float dy2[7];
  float dy3[7];
  float dy4[7];
  int i;

  dxdt_segway(dy1, x, ut);
  for (i = 0; i<7; i++)
      temp[i] = dy1[i]*dt2 + x[i];

  dxdt_segway(dy2, temp, ut);
  for (i = 0; i<7; i++)
      temp[i] = dy2[i]*dt2 + x[i];

  dxdt_segway(dy3, temp, ut);
  for (i = 0; i<7; i++)
      temp[i] = dy3[i]*dt2*2 + x[i];

  dxdt_segway(dy4, temp, ut);
  for (i = 0; i<7; i++)
      x2[i] = dt2*2*(dy1[i] + 2*(dy2[i]+dy3[i]) + dy4[i])/6.0;
}

void dxdt_body(float* xdot, binaryFloat* x, float* u)
{
    const float Mb = 1.76;
  	const float Mw = 0.147;
  	const float R = 0.07;
  	const float cz = 0.077;
  	const float b = 0.1985/2;
  	const float Ixx = ( pow(0.166, 2) + pow(0.21, 2) )*Mb/12 + Mb*pow(cz-0.07, 2);
  	const float Iyy = ( pow(0.072, 2) + pow(0.21, 2) )*Mb/12 + Mb*pow(cz-0.07, 2);
  	const float Izz = ( pow(0.072, 2) + pow(0.166, 2) )*Mb/12;
  	const float Iwa = Mw*pow(R, 2)/2;
    const float Iwd = Mw*pow(b, 2);
    const float g = 9.81;

    const float phi = x[2].floatingPoint;
    const float alpha = x[3].floatingPoint;
    const float dalpha = x[4].floatingPoint;
    const float v = x[5].floatingPoint;
    const float dphi = x[6].floatingPoint;

    float Dalpha, Galpha, H, Kalpha, f21, f22, f23, g21, g22, g23;

    Dalpha = pow(Mb*cos(alpha)*cz*R, 2);
    Dalpha += ((-pow(Mb, 2) - 2*Mw*Mb)*pow(cz, 2) - 2*Iyy*Mw - Iyy*Mb)*pow(R, 2);
    Dalpha += -2*Mb*pow(cz, 2)*Iwa-2*Iyy*Iwa;

    Galpha = (-Mb*pow(cz, 2)+Izz-Ixx)*pow(R*cos(alpha), 2);
    Galpha += (Mb*pow(cz, 2)+Ixx+2*Iwd+2*pow(b, 2)*Mw)*pow(R, 2)+2*pow(b, 2)*Iwa;

    H = 0.5*Mb*pow(R, 2)*Izz + Iwa*Izz-Mw*pow(R, 2)*Ixx;
    H += -Iwa*Ixx - Mb*Mw*pow(cz*R, 2) - Mb*pow(cz, 2)*Iwa;
    H += -0.5*Mb*pow(R, 2)*Ixx + Mw*pow(R, 2)*Izz;

    Kalpha = (-4*Iyy*Mb*pow(R, 2)*cz - 3*pow(R*Mb*cz, 2)*cz + Mb*pow(R, 2)*cz*(Ixx-Izz))*sin(alpha);
    Kalpha += (Mb*pow(R, 2)*cz*(Ixx-Izz) + pow(R*Mb*cz, 2)*cz)*sin(3*alpha);

    f21 = sin(2*alpha)*pow(dphi, 2)*H/Dalpha + pow(Mb*cz*R*dalpha, 2)*sin(2*alpha)/(2*Dalpha);
    f21 += (-2*pow(Mb*R, 2)*cz-4*Iwa*Mb*cz-4*Mw*pow(R, 2)*Mb*cz)*g*sin(alpha)/(2*Dalpha);

    f22 = Kalpha*pow(dphi, 2)+(pow(Mb*cz*R, 2)*g*sin(2*alpha))/(2*Dalpha);
    f22 += (-4*Iyy*Mb*pow(R, 2)*cz - 4*pow(R*Mb*cz, 2)*cz)*sin(alpha)*pow(dalpha, 2)/(4*Dalpha);

    f23 = (-(Ixx-Izz)*pow(R, 2)- pow(Mb*cz*R, 2))*sin(2*alpha)*dalpha*dphi/Galpha;
    f23 += -sin(alpha)*pow(R, 2)*Mb*cz*v*dphi/Galpha;

    g21 = (u[1] + u[0])*(Mb*pow(R, 2) + 2*Mw*pow(R, 2) + 2*Iwa + Mb*cos(alpha)*cz*R)/Dalpha;
    g22 = -(u[1] + u[0]) * R * ( Mb*cos(alpha)*cz*R + Iyy + Mb*pow(cz, 2))/Dalpha;
    g23 = (u[1] - u[0]) * R * b / Galpha;

    xdot[0] = cos(phi)*v;
    xdot[1] = sin(phi)*v;
    xdot[2] = x[6].floatingPoint;
    xdot[3] = x[4].floatingPoint;
    xdot[4] = f21 + g21;
    xdot[5] = f22 + g22;
    xdot[6] = f23 + g23;
}

float drive(float u, float omega)
{
    const float kl = 0.206;
    const float kh = 0.3;
    const float bend = 0.1;
    const float cfric = 0.0065;
    float out = 0.0;

    if (abs(u) < bend)
        out += kl*u;
    else
        out += sign(u)*( kl*bend + kh*( abs(u) - bend ) );

    out -= cfric*omega;
    return out;
}

void dxdt_segway(float* xdot, binaryFloat* x, float* u)
{
    const float R = 0.07;
    const float b = 0.1985/2;
    float v_left, v_right, omega_left, omega_right;
    float tau[2];

    //TODO should I limit u between -1 and 1?
    v_left = x[5] - x[6] * b;
    v_right = x[5] + x[6] * b;
    omega_right = v_right / R - x[4];
    omega_left = v_left / R - x[4];

    tau[0] = drive(u[0], omega_left);
    tau[1] = drive(u[1], omega_right);

    dxdt_body(xdot, x, tau);
}
