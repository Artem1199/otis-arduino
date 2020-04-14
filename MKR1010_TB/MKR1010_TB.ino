           /*
   OTIS Two Wheeled Self Balancing Robot
   @author EThan Lew
*/


#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "SAMD21turboPWM.h"
#include "PID_v1.h" //cpp library for testing
#include "Secrets.h"

#include <WiFiNINA.h>                                 
#include <SPI.h>
#include <utility/wifi_drv.h>
#include <pid_control.h>


#define DEBUG 
/* MACROS */
#define SERIAL_BAUD 115200
#define I2C_FAST_MODE 400000
#define MPU_INT 8
/* Dual H-bridge macros */
#define DR1 5
#define PWM1 4
#define NEN1 3

#define DR0 1
#define PWM0 2
#define NEN0 0

//WiFiClient client;
WiFiServer server(80);
bool data_sent = false;
/* System Tuning */
uint16_t accel_bias[] = {-752, -2120, 1234};
uint16_t gyro_bias[] = {18, 2, -51};

/* MPU6050 Device Context */
MPU6050 mpu;

/* MPU control/status vars */
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

uint8_t prevDuty = 0;

/* Processing variables */
float angle, angular_rate;

/* orientation/motion vars */
Quaternion q;
VectorFloat gravity;
int16_t gyro[3];
float ypr[3];

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
 // Serial.println("IT");

}

/* Setting PWM properties */
const int freq = 30000;
const int pwmChannel0 = 0;
const int pwmChannel1 = 1;
const int resolution = 8;
uint8_t dutyCycle0 = 200;
uint8_t dutyCycle1 = 200;

/* PID properties */
double originalSetpoint = -0.05;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
double input_old, output_old;

double errSum, lastTime, lastInput= 0, outputSum = 0;

//double Kpt = 90;
//double Kit = 100;
//double Kdt = 7;
double Kpt = 120;
double Kdt = 5;
double Kit = 150;
double Kpy = 15;
double Kdy = 2;
double Kiy = 0;


double setpointy = 100.0;
double inputy, outputy;
double inputy_old, outputy_old;

PID pidTilt(&input, &output, &setpoint, Kpt, Kit, Kdt, DIRECT);
PID pidYaw(&inputy, &outputy, &setpointy, Kpy, Kiy, Kdy, DIRECT);

double errSumy, lastTimey, lastInputy = 0, outputSumy = 0;
//PID pidTilt(&input, &output_old, &setpoint, Kpt, Kit, Kdt, DIRECT);
//PID pidYaw(&inputy, &outputy_old, &setpointy, Kpy, Kiy, Kdy, DIRECT);

float curr =  0x7FFFFFFF;
float prev =  0x7FFFFFFF;
float diff = 0.0;


/* Motor output */
double out0, out1;

uint8_t remote_buff[4];
const uint16_t* remote_16 = (const uint16_t*)remote_buff;

/* Serial In */
String serBuff = "";

/* Websockets Control */

//#define SSID "OTIS-bot"
//#define PASSWORD "japery2019"
#define SERVER_PORT 4141

#define PACKET_SIZE 4

#define IS_SERVER



int status = WL_IDLE_STATUS; 

/*
  
  Server server(SERVER_PORT);
  WiFiClient client;
  size_t len;
*/
uint16_t tiltNumber, yawNumber;

//BluetoothSerial SerialBT;

TurboPWM pwm;

unsigned long postDelay = millis();

/*_____________________________________________________ SETUP _______________________________________________________________*/

void setup() {
  WiFiDrv::pinMode(25, OUTPUT); //GREEN
  WiFiDrv::pinMode(26, OUTPUT); //RED
  WiFiDrv::pinMode(27, OUTPUT); //BLUE
  WiFiDrv::digitalWrite(26, HIGH); // for full brightness

  /* Setup PID */
  pidTilt.SetMode(AUTOMATIC);
  pidTilt.SetSampleTime(10);
  pidTilt.SetOutputLimits(-255, 255); 

  pidYaw.SetMode(AUTOMATIC);
  pidYaw.SetSampleTime(10);
  pidYaw.SetOutputLimits(-255, 255);

  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  
  Serial.begin(SERIAL_BAUD);
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}
  delay(1000);
  
  pinMode(LED_BUILTIN, OUTPUT);
  /* Setup the I2C bus */

 Wire.begin();
 Wire.setClock(400000);

  /* Setup the motors */
  initialize_pwm();
  /* Setup wifi */
  //initialize_wifi();
  server.begin();
  WiFiDrv::digitalWrite(26, LOW); // for full brightness
  WiFiDrv::digitalWrite(27, HIGH); // for full brightness
  /* Setup the IMU and relevant buffers */
  initialize_ypr();

 /* init PID values */
  lastTime = millis() - 10;
  lastTimey = millis() - 10;
  fetch_ypr();
  input = ypr[1];
  inputy = ypr[0];

  WiFiDrv::digitalWrite(27, LOW); // for full brightness
  WiFiDrv::digitalWrite(25, HIGH); // for full brightness

}


/*_____________________________________________________ LOOP _______________________________________________________________*/


unsigned long looptime = micros();

void loop() {

    looptime = micros();

  
  
  fetch_ypr();
  
    if (input != ypr[1]){
      WiFiDrv::digitalWrite(25, HIGH);
    }
  input = ypr[1];
  inputy = ypr[0];


  if (setpointy > 4.0) {
    setpointy = ypr[0];
  }

  pidTilt.Compute();
  pidYaw.Compute();

  Serial.print(" p: ");
  Serial.print(ypr[1]);
  Serial.print(" o0: ");
  Serial.print(out0);
  
  
  out0 = output;
  out1 = output + outputy;


  if (out0 > 0.0) {
    digitalWrite(DR0, false);
  } else {
    digitalWrite(DR0, true);
  }
  //Serial.print(" BP F ");
  if (out1 > 0.0) {
    digitalWrite(DR1, true);
  } else {
    digitalWrite(DR1, false);
  }

  double duty_mag0 = abs(255.0 / 50.0 * min((double)50, abs(out0)));
  double duty_mag1 = abs(1000.0 / 50.0 * min((double)50, abs(out1)));

  duty_mag0 = map(duty_mag0, 0, 255, 0, 1000);
  Serial.print(" DC: ");
  Serial.print((duty_mag0/1000.0)*100);
  
   Serial.print(" Time: ");


  pwm.analogWrite(PWM0, duty_mag0);


    WiFiDrv::digitalWrite(25, LOW);

 Serial.println(micros() - looptime);
}

/*_____________________________________________________ PWM _______________________________________________________________*/

void initialize_pwm() {
  // Backwards: DR1 false, DR2 true
  // Forwards: DR1 true, DR2 false
  bool dir = true;

  pinMode(PWM1, OUTPUT);
  pinMode(DR1, OUTPUT);
  pinMode(NEN1, OUTPUT);

  pinMode(PWM0, OUTPUT);
  pinMode(DR0, OUTPUT);
  pinMode(NEN0, OUTPUT);

  digitalWrite(NEN1, 1);
  digitalWrite(NEN0, 1);
  digitalWrite(DR1, dir);
  digitalWrite(DR0, !dir);


  pwm.setClockDivider(1, false); //
  pwm.timer(1, 1, 800, false); //timer x, prescaler, steps resolution
  
  pwm.setClockDivider(0, false); //
  pwm.timer(0, 1, 800, false); //timer 
}

/*_____________________________________________________ IMU _______________________________________________________________*/

void initialize_ypr() {
  /* Initialize the MPU */
  mpu.initialize();
  /* Verify the MPU */
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  /* Initialize the DMP */
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.setRate(9); //Set sampling rate to 1000/(1+9)
  //uint8_t a = 1;
  //mpu.dmpSetFIFORate(a);
  /* Set the device biases appropiately */
  mpu.setXAccelOffset(accel_bias[0]);
  mpu.setYAccelOffset(accel_bias[1]);
  mpu.setZAccelOffset(accel_bias[2]);
  mpu.setXGyroOffset(gyro_bias[0]);
  mpu.setYGyroOffset(gyro_bias[1]);
  mpu.setZGyroOffset(gyro_bias[2]);


  if (devStatus == 0)
  {
    /* turn on the DMP, now that it's ready */
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    /* Enable Interrupt Detection */
    Serial.println(F("Enabling interrupt detection (MKR1010 PIN 8)..."));
    pinMode(MPU_INT, INPUT_PULLUP);
    attachInterrupt(MPU_INT, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    /* set our DMP Ready flag so the main loop() function knows it's okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

/*_____________________________________________________ YPR _______________________________________________________________*/
void fetch_ypr(){
  /* if programming failed, don't try to do anything */
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) {
  }
  /* reset interrupt flag and get INT_STATUS byte */
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  /* get current FIFO count */
  fifoCount = mpu.getFIFOCount();
  /* check for overflow (this should never happen unless our code is too inefficient) */
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    /* reset so we can continue cleanly */
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  else if (mpuIntStatus & 0x02)
  {
    /* wait for correct available data length, should be a VERY short wait */
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    /* read a packet from FIFO */
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    /* Get sensor data */
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGyro(gyro, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    /* Note that 1.47 is zero tilt error */
    //angle = ypr[1] + 0.06;                   // 0.02 is center of gravity offset
    //angular_rate = -((double)gyro[1]/131.0); // converted to radian
    //Serial.println(ypr[0]);
  }
  
}
