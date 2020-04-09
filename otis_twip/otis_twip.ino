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

#define HOST "192.168.50.158"

#include <WiFiNINA.h>                                 
#include <SPI.h>
#include <WiFiUdp.h>
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
uint16_t gyro_bias[] = {10, 7, 14};
uint16_t accel_z_bias = 900;

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

double Kpt = 80;
double Kit = 91;
double Kdt = 0;


double setpointy = 100.0;
double inputy, outputy;
double inputy_old, outputy_old;

double Kpy = 50;
double Kiy = 40;
double Kdy = 0;

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

WiFiClient client;
enum Protocol {
  TILT_SET,
  YAW_SET
};

/*
  
  Server server(SERVER_PORT);
  WiFiClient client;
  size_t len;
*/
uint16_t tiltNumber, yawNumber;

//BluetoothSerial SerialBT;

TurboPWM pwm;

unsigned long postDelay = millis();

String apiKeyValue = "1";

/*_____________________________________________________ SETUP _______________________________________________________________*/

void setup() {
  WiFiDrv::pinMode(25, OUTPUT); //GREEN
  WiFiDrv::pinMode(26, OUTPUT); //RED
  WiFiDrv::pinMode(27, OUTPUT); //BLUE
  WiFiDrv::digitalWrite(26, HIGH); // for full brightness



  digitalWrite(12, LOW);
  digitalWrite(13, LOW);

  /* Create Bluetooth Serial */
  //SerialBT.begin("OTIS-BOT");
  /* Set the serial baud rate*/
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
  initialize_wifi();
  server.begin();
  WiFiDrv::digitalWrite(26, LOW); // for full brightness
  WiFiDrv::digitalWrite(27, HIGH); // for full brightness
  /* Setup the IMU and relevant buffers */
  initialize_ypr();
  

client.stop();


  initialize_ypr();

 /* init PID values */
  lastTime = millis() - 10;
  lastTimey = millis() - 10;
  fetch_ypr();
  input = ypr[1];
  inputy = ypr[0];

  while(!client){
     client = server.available();
     if (client.connected()) {
      while (client.available()){
        client.read(); 
      }
      client.write("Ack\n"); 
           //client.println("hello"); 
     break;}
     delay(10);
     }

  WiFiDrv::digitalWrite(27, LOW); // for full brightness
  WiFiDrv::digitalWrite(25, HIGH); // for full brightness

}

unsigned long hold = millis();
int count = 0;
int countmax = 0;

unsigned long evalDelay = millis();

unsigned long timediff = 0;
unsigned long sendcounter = 0;
unsigned long notcounter = 0;
bool client_avi = false;

/*_____________________________________________________ LOOP _______________________________________________________________*/

int send_c = 0;

const size_t httpBodySize = 128;
char httpBody[httpBodySize];

unsigned long loopdiff;

void loop() {

  
  loopdiff = micros();
  fetch_ypr();
  
    if (input != ypr[1]){
      WiFiDrv::digitalWrite(25, HIGH);
    }
  input = ypr[1];
  inputy = ypr[0];

  

  
  
  
  //Serial.print("Eqv Values:");
  //Serial.print(send_p);
 // Serial.print(" ");
 // Serial.println(ypr[1]);



  if (setpointy > 4.0) {
    setpointy = ypr[0];
  }

  if (Serial.available() > 0)
  {
    Serial.setTimeout(90);
    serBuff = Serial.readString();

    if (serBuff.substring(0, 4) == "KILL") {
      Serial.println("Killing Motors");
      // TODO: Kill Motors
    } else if (serBuff.substring(0, 7) == "SETTILT") {
      double tiltAngle;
      char __serBuff[sizeof(serBuff)];
      serBuff.toCharArray(__serBuff, sizeof(__serBuff));
      int result = sscanf(__serBuff, "SETTILT %lf", &tiltAngle);
      Serial.print("Setting Tilt: ");
      Serial.println(tiltAngle / 100);
      setpoint = tiltAngle / 100.0;
    } else if (serBuff.substring(0, 6) == "SETYAW") {
      double yawAngle;
      char __serBuff[sizeof(serBuff)];
      serBuff.toCharArray(__serBuff, sizeof(__serBuff));
      int result = sscanf(__serBuff, "SETYAW %lf", &yawAngle);
      Serial.print("Setting yaw: ");
      Serial.println(yawAngle / 100);
      setpointy = yawAngle / 100.0;

    } else if (serBuff.substring(0, 6) == "SETPID") {
      double kpt, kit, kdt;
      char __serBuff[sizeof(serBuff)];
      serBuff.toCharArray(__serBuff, sizeof(__serBuff));
      int result = sscanf(__serBuff, "SETPID %lf %lf %lf", &kpt, &kit, &kdt);

      //      pidTilt.SetTunings((double)kpt, (double)kit, (double)kdt);

      Serial.print("PID Gains Changed. P:");
      //      Serial.print(pidTilt.GetKp());
      Serial.print(" I:");
      //      Serial.print(pidTilt.GetKi());
      Serial.print(" D:");
      //      Serial.println(pidTilt.GetKd());
    }
  }


  //Serial.print(" input: ");
 // Serial.print(input);
  //Serial.print(" inputy: ");
  //Serial.print(inputy);
#ifdef DEBUG
  Serial.print(" lI: ");
  Serial.print(lastInput);
#endif

  unsigned long now = millis();
  
  compute_pid(input, &output, setpoint, Kpt, Kit, Kdt, now, &lastTime, 10, &lastInput, &outputSum);
  compute_pid(inputy, &outputy, setpointy, Kpy, Kiy, Kdy, now, &lastTimey, 10, &lastInputy, &outputSumy);



  uint16_t send_p = ((ypr[1] + 3.14)*10436);
  uint16_t send_y = ((ypr[1] + 3.14)*10436);
  uint16_t send_o = (output+1000) * 33;
  uint16_t send_g = (outputy+1000) * 33;
  
  uint8_t sendarray[]= {send_p & 0xff, send_p >> 8, send_y & 0xff, send_y >> 8,send_o & 0xff, send_o >> 8,send_g & 0xff, send_g >> 8, };
  
  
#ifdef DEBUG
  Serial.print(" input: ");
  Serial.print(input);
  Serial.print(" lT: ");
  Serial.print(lastTime);
  Serial.print(" oS: ");
  Serial.print(outputSum);
  Serial.print(" O: ");
  Serial.println(output);
#endif

  out0 = output - outputy;
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

  double duty_mag0 = abs(1000.0 / 50.0 * min((double)50, abs(out0)));
  double duty_mag1 = abs(1000.0 / 50.0 * min((double)50, abs(out1)));
  // dutyCycle0 = (uint8_t)duty_mag0;
  // dutyCycle1 = (uint8_t)duty_mag1;

  if (fabs(input) < 0.6) {
    pwm.analogWrite(PWM0, duty_mag0);
    pwm.analogWrite(PWM1, duty_mag1);
//    Serial.print(" duty_mag0: ");
//    Serial.print(duty_mag0);
//    Serial.print(" duty_mag1: ");
//    Serial.print(duty_mag1);

  } else {
    pwm.analogWrite(PWM0, 0);
    pwm.analogWrite(PWM1, 0);
  }
  

  if (millis() - postDelay > 10){
          // client.println(httpBody);
          client.write(sendarray, 8);
           postDelay = millis();
        //   value_ptr = 0;          
  }

 

    WiFiDrv::digitalWrite(25, LOW);

   Serial.println(micros() - loopdiff);
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
  mpu.setXGyroOffset(gyro_bias[0]);
  mpu.setYGyroOffset(gyro_bias[1]);
  mpu.setZGyroOffset(gyro_bias[2]);
  mpu.setZAccelOffset(accel_z_bias);

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

/*_____________________________________________________ WIFI _______________________________________________________________*/

void initialize_wifi(){

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

    String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection: 
    delay(3000);
  }
  printWiFiStatus();

  // you're connected now, so print out the data:
  Serial.println("You're connected to the network");
  
  return;
}

/*_____________________________________________________ WiFi _______________________________________________________________*/

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
