//Contains the structs, defines and global variables
#define ENCODER_CPR 64
#define MOTOR_GEAR 18.8
#define DISTANCE_PER_ROTATION M_PI*0.14 //pi*diameter, in meter
#define DT 10000 //discrete time step size in microseconds

//this type is nice for sending floating units over serial, found here: https://forum.arduino.cc/index.php?topic=112597.0
typedef union {
 float floatingPoint;
 byte binary[4];
} binaryFloat;

//Timing and printing
elapsedMicros sinceSendtoRasp;
elapsedMicros sinceStateUpdate;
elapsedMillis sinceSendtoPC;

//Encoder
Encoder encoder_left(PIN_ENCODER1A, PIN_ENCODER1B); //left wheel
Encoder encoder_right(PIN_ENCODER2A, PIN_ENCODER2B); //right wheel
long motorrot; // Count rotation variable
float ds_left; //estimated distance of left wheel over past discrete time
float ds_right; //estimated distance of right wheel over past discrete time

//Motor related
bool dir_left; //true means forward, false backwards
bool dir_right; //true means forward, false backwards
uint8_t speed_left; //PWM for the motor speed. 255 is max value
uint8_t speed_right; //PWM for the motor speed. 255 is max value

//Raspberry pi communication
uint8_t i, j;
uint8_t n_bytes_to_come = 0;
binaryFloat state[7];
binaryFloat action[2];
bool newdata; //true if a new state vector was computed and not yet send

//MPU6050 Sensor
MPU6050 mpu;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
bool dmpReady = false;  // set true if DMP init was successful
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
int16_t gyr[3] = {0, 0, 0};
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float mpu_bias[3] = {0, 0.007, 0};

//Batterie voltage
float v_battery;

//Debugging
//float temp1;
//float temp2;
