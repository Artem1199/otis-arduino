/*
   OTIS Two Wheeled Self Balancing Robot
   @author EThan Lew, modified by AK
*/

#include "Secrets.h"
#include <Wire.h>
#include "I2Cdev.h"
                                 
#include <SPI.h>
#include <WiFiNINA.h>  
#include <WiFiUdp.h>

#include "Encoder.h"
#include <utility/wifi_drv.h>

#include "PID_v1.h"
#include <pid_control.h>

#include "Adafruit_MotorShield.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define WIFI
/* MACROS */
#define SERIAL_BAUD 115200
#define I2C_FAST_MODE 400000

#define MPU_INT 0
#define HOST "192.168.50.158"

#ifdef WIFI
WiFiServer server(80);
WiFiClient client;
#endif


Encoder myEnc(6, 7);
/* System Tuning */
uint16_t gyro_bias[] = {30, -10, 62};
uint16_t accel_bias[] = {-1167, -2147, 1251};

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
}

/* Setting Motor Shield properties */
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);   // Boost/Buck converter side motor
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);   // Adafruit shield side motor
uint8_t dutyCycle0 = 0;
uint8_t dutyCycle1 = 0;

/* PID properties */
double originalSetpoint = -0.1455;  //positive = falls toward fuses
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

uint8_t sampleRate = 5;
double Kpt = 800;//800;
double Kit = 24000;//24000; 
double Kdt = -24;//-24;  //maybe 10 - 15?

double Kpy = 50;
double Kiy = 40;
double Kdy = 0;

double Kpe = 3;
double Kie = 2;
double Kde = -.01;

double Kpf = 3;//zzy logic constants
double Kif = 25;
double Kdf = -.04;

double Kpfy = 1;
double Kify = 1;
double Kdfy = -0.005;



bool fuzzy_c = false;  //select which PID to use


double setpointy = 100.0;
double inputy, outputy;

/* Init PID controller */
PID *pid_ptr = new PID;
PID *pid_ptry = new PID;

PID *pid_ptre = new PID;

FUZ *fuz_ptr = new FUZ;
FUZ *fuz_ptry = new FUZ;

/* Motor output */
double out0, out1;

int status = WL_IDLE_STATUS; 

unsigned long postDelay = millis();
/*_____________________________________________________ SETUP _______________________________________________________________*/

void setup() {
  WiFiDrv::pinMode(25, OUTPUT); //GREEN
  WiFiDrv::pinMode(26, OUTPUT); //RED
  WiFiDrv::pinMode(27, OUTPUT); //BLUE
  WiFiDrv::digitalWrite(26, HIGH); // for full brightness

  digitalWrite(12, LOW);
  digitalWrite(13, LOW);
  
  /* Set the serial baud rate*/
  Serial.begin(SERIAL_BAUD);
  //while (!Serial) {
   // ; // wait for serial port to connect. Needed for native USB port only
  //}
  
  AFMS.begin(1600); //AFMS max frequency is 1.6k
  pinMode(LED_BUILTIN, OUTPUT);
  
  /* Setup the I2C bus */
  Wire.begin();
  Wire.setClock(400000);

  WiFiDrv::digitalWrite(26, LOW); // for full brightness
  WiFiDrv::digitalWrite(27, HIGH); // for full brightness

#ifdef WIFI
  /* Setup wifi */
  initialize_wifi();

  /* Setup communication w/ otis-TCP rust program */
  server.begin();
  client.stop();
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
#endif


  WiFiDrv::digitalWrite(27, LOW); // for full brightness
  WiFiDrv::digitalWrite(25, HIGH); // for full brightness



  /* Setup the IMU and relevant buffers */
  initialize_ypr();
  
 /* init PID values */
  fetch_ypr();
  input = ypr[1];
  inputy = ypr[0];
  

  while (fabs(input) < .6) {
    client.write("Ack\n");
    fetch_ypr();
    input = ypr[1];
  }

  setpointy = ypr[0];

  InitLustreFUZ(fuz_ptr,input,millis(),setpoint,Kpf,Kif,Kdf,sampleRate);
  InitLustreFUZ(fuz_ptry,inputy,millis(),setpointy,Kpfy,Kify,Kdfy,sampleRate);
  
  InitLustrePID(pid_ptr,input,millis(),setpoint,Kpt,Kit,Kdt,sampleRate);
 // InitLustrePID(pid_ptre,0,millis(),0,Kpe,Kie,Kde,sampleRate);
  
  InitLustrePID(pid_ptry,inputy,millis(),setpointy,Kpy,Kiy,Kdy,sampleRate);

}
/*_____________________________________________________ LOOP _______________________________________________________________*/

uint8_t error_cnt = 99;

float lastEnc = 0;
float currentEnc = 0;
float currentTime = 0;
float lastTime = 0;
float encoder_adjust = 0;
bool setpointFound = false;
float encoderSetpoint = 0;
bool encoder_ctrl = true;



float turn_r = 0;
float turn_l = 0;

void loop() {
  
  fetch_ypr();
  
/* detect is MPU is no longer providing safe data */  
    if (input != ypr[1]){
      WiFiDrv::digitalWrite(25, HIGH);
      error_cnt = 99;
       } else {
        error_cnt--;
          if (error_cnt == 0){
          WiFiDrv::digitalWrite(25, LOW);

          while(1){
            myMotor1->setSpeed(0);
            myMotor2->setSpeed(0);
            WiFiDrv::digitalWrite(26, HIGH);
            delay(2000);
            WiFiDrv::digitalWrite(26, LOW);
            delay(2000);}
          } 
       }

  input = ypr[1];
  inputy = ypr[0];
  
  
  currentTime = millis();
  currentEnc = myEnc.read();
  
  if (currentTime - lastTime > 10){
    encoder_adjust = (currentEnc - lastEnc);
     lastTime = currentTime;
     lastEnc = currentEnc;
    // Serial.println(encoderSetpoint - myEnc.read());
  } else {
    encoder_adjust = 0;
  }


  if ((fabs(input) + originalSetpoint < .03) && encoder_ctrl){
        setpoint += encoder_adjust * .00045; } else {
    
    setpoint = originalSetpoint;
  }

  
  

  if (fuzzy_c){
    output = ComputeLustreFUZ(fuz_ptr, input, millis(),setpoint,Kpf,Kif,Kdf,sampleRate) * 255.0;
    WiFiDrv::digitalWrite(27, HIGH);
  } else {
    WiFiDrv::digitalWrite(27, LOW);
   output = ComputeLustrePID(pid_ptr,input,millis(),setpoint,Kpt,Kit,Kdt,sampleRate) ;
          //  + encoder_adjust * 6;
  }


  
 //pidTilt.Compute(millis());
  out0 = output - outputy + turn_l;
  out1 = output + outputy + turn_r;

  
  if (out0 > 0.0) {
     myMotor1->run(BACKWARD);
  } else {
    myMotor1->run(FORWARD);
  }
  if (out1 > 0.0) {
     myMotor2->run(BACKWARD);
  } else {
    myMotor2->run(FORWARD);
  }
  
  if (fabs(input) < 0.5) {
    myMotor1->setSpeed(round(fabs(out0)));
    myMotor2->setSpeed(round(fabs(out1)));

  } else {
  myMotor1->setSpeed(0);
  myMotor2->setSpeed(0);
  }

#ifdef WIFI
  wifi_communication(ypr[1] - setpoint, ypr[0] - setpointy, output, outputy);
#endif
    WiFiDrv::digitalWrite(25, LOW);
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
  mpu.setRate(sampleRate - 1); //Set sampling rate to 1000/(1+9)
  //uint8_t a = 1;
  //mpu.dmpSetFIFORate(a);
  /* Set the device biases appropiately */
  mpu.setXGyroOffset(gyro_bias[0]);
  mpu.setYGyroOffset(gyro_bias[1]);
  mpu.setZGyroOffset(gyro_bias[2]);
  mpu.setXAccelOffset(accel_bias[0]);
  mpu.setYAccelOffset(accel_bias[1]);
  mpu.setZAccelOffset(accel_bias[2]);


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
    Serial.println(F("F of!"));
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

#ifdef WIFI
double f_setpoint = originalSetpoint - .03;
double b_setpoint = originalSetpoint + .05;

double turn_timer = 0;
float setpoint_hold = 0;

void wifi_communication(double pitch_err, double yaw_err, double output, double outputy) {
  
   uint16_t send_p = ((pitch_err + 3.141592)*10436.381);
   uint16_t send_y = ((yaw_err + 3.141592)*10436.381);
   uint16_t send_o = (output+1000) * 33;
   uint16_t send_g = (outputy+1000) * 33;
  
    uint8_t sendarray[]= {send_p & 0xff, send_p >> 8, send_y & 0xff, send_y >> 8,send_o & 0xff, send_o >> 8,send_g & 0xff, send_g >> 8, };
      
        client.write(sendarray, 8);

     if (client.available()){
        uint8_t c =  client.read(); 
        if (c == 1){
           setpoint = f_setpoint;
           encoder_ctrl = false;
        }
        if (c == 2){                
           setpoint = b_setpoint;
           encoder_ctrl = false;
        }
        if (c == 3){
          setpoint = originalSetpoint;
          encoderSetpoint = myEnc.read();
        }
        if (c == 4){
          turn_r = -30;
          turn_l = 30;
          turn_timer = millis();
          encoder_ctrl = false;
        }
        if (c == 5){
          turn_r = 30;
          turn_l = -30;
          turn_timer = millis();
          encoder_ctrl = false;

        } 
        if (c == 6){
          fuzzy_c = false;
        }
        if (c == 7){
          fuzzy_c = true;
        }
         // Serial.print(c);
        //  Serial.println(setpoint, 4);
      }

      
    if (millis()-turn_timer > 100)
    {
      
      turn_r = 0;
      turn_l = 0;
      encoder_ctrl = true;
    };
  
};
#endif
