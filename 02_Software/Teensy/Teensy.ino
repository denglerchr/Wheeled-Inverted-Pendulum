// This is the main programm for the small inverted pendulum on wheels
// The programm reads the sensors, uses the data to create a state vector
// and send it to the raspberry pi.
//
// TODO: Handle NaN, which can appear when interrupting the rapsberry pi somehow
//
// Include Libraries
//#include "I2Cdev.h"
#include <math.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include <Encoder.h>

#include "pindef.h" // Contains all the Pin definitions
#include "vars_and_types.h"

//Some helper functions
void dmpDataReady() {
  mpuInterrupt = true;
}

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

// This is running once at the beginning
void setup() {
  // Serial connections
  Serial.begin(9600); //to communicate with the PC during setup
  RASPBERRY_SERIAL.begin(1000000); //communication with the raspberry pi

  //I2C communication
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  //mpu setup
  Serial.println(F("Testing device connections..."));
  mpu.initialize();
  pinMode(PIN_MPU_INTERRUPT, INPUT);
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(PIN_MPU_INTERRUPT), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // variable initialization
  for (i=0; i<7; i++)
  {
    state[i].floatingPoint = 0.0;
  }
  for (i=0; i<2; i++)
  {
    action[i].floatingPoint = 0.0;
  }
  v_battery = 11.0;

  //LED setup
  pinMode(PIN_RED_LED, OUTPUT);

  //Motor setup
  pinMode(PIN_DIR_LEFT, OUTPUT);
  pinMode(PIN_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIR_RIGHT, OUTPUT);
  pinMode(PIN_PWM_RIGHT, OUTPUT);
  digitalWrite(PIN_DIR_LEFT, LOW);
  digitalWrite(PIN_PWM_LEFT, LOW);
  digitalWrite(PIN_DIR_RIGHT, LOW);
  digitalWrite(PIN_PWM_RIGHT, LOW);

  // Start with 5s led blink
  for (i=0; i<5; i++)
  {
    digitalWrite(PIN_RED_LED, HIGH);
    delay(500);
    digitalWrite(PIN_RED_LED, LOW);
    delay(500);
  }

}

// Main loop
void loop()
{
  if (sinceStateUpdate>=DT)
  {
    sinceStateUpdate = 0;

    // Get batterie voltage (overwrites "v_battery")
    get_batterie_voltage();

    //Change encoder state if new "u" arrived
    set_motors();

    // Readout MPU if new data is available (overwrite "ypr")
    mpu_readout();

    // Read out encoders (overwrite "ds_left", "ds_right")
    read_encoder();

    // State estimation (overwrite "state" and "newdata")
    state_update();
  }

  // Raspberry pi communication if it is ready (overwrite "u")
  raspi_communication();

  //Uncomment to send data to PC
  pc_communication();
}
