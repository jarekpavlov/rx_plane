//  8 Channel Receiver
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>

unsigned long loopTime = 0;

////MSU variables:
float rateCalibrationRoll, rateCalibrationPitch, rateCalibrationYaw;
int rateCalibrationNumber;

float rateRoll, ratePitch, rateYaw;
float accX, accY, accZ;
float angleRoll, anglePitch;

float kalmanAngleRoll = 0, kalmanUncertaintyAngleRoll = 2*2;
float kalmanAnglePitch = 0, kalmanUncertaintyAnglePitch = 2*2;
float kalman1DOOutput[]= {0,0};
////

////MSU functions:
void kalman1d(float kalmanState, float kalmanUncertainty, float kalmanInput, float kalmanMesurment) {
  float delta = (millis() - loopTime)/1000.;
  kalmanState = kalmanState + delta * kalmanInput;
  kalmanUncertainty = kalmanUncertainty + delta * delta * 4 * 4;
  float kalmanGain = kalmanUncertainty*1/(1*kalmanUncertainty +3*3);
  kalmanState = kalmanState + kalmanGain * (kalmanMesurment-kalmanState);
  kalmanUncertainty = (1-kalmanGain) * kalmanUncertainty;
  kalman1DOOutput[0] = kalmanState;
  kalman1DOOutput[1] = kalmanUncertainty;
}

void gyroAccGet() {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();

  Wire.requestFrom(0x68,6);
  int16_t gyroX = Wire.read() << 8 | Wire.read();
  int16_t gyroY = Wire.read() << 8 | Wire.read();
  int16_t gyroZ = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  
  Wire.requestFrom(0x68,6);
  int16_t accXLSB = Wire.read() << 8 | Wire.read();
  int16_t accYLSB = Wire.read() << 8 | Wire.read();
  int16_t accZLSB = Wire.read() << 8 | Wire.read();

  rateRoll = (float)gyroX/65.5;
  ratePitch = (float)gyroY/65.5;
  rateYaw = (float)gyroZ/65.5;

  accX = (float)accXLSB/4096;
  accY = (float)accYLSB/4096 + 0.01;
  accZ = (float)accZLSB/4096 - 0.11;

  angleRoll = atan(accY/sqrt(accX*accX + accZ*accZ))*1/(3.142/180);
  anglePitch = atan(accX/sqrt(accY*accY + accZ*accZ))*1/(3.142/180);
}
///
int ch_width_9 = 0;
int ch_width_10 = 0;
int ch_width_5 = 0;
int ch_width_6 = 0;

Servo ch6;
Servo ch5;
Servo ch9;
Servo ch10;

struct Signal {
  byte throttle;
  byte pitch;  
  byte roll; 
  byte yaw; 
  bool autopilot;  
};

Signal data;

const byte pipe[5] = "ch1"; 
RF24 radio(7,8); 

void ResetData()
{
  data.throttle = 0;
  data.roll = 127;
  data.pitch = 127;    
  data.yaw = 127; 
  data.autopilot = false;                                               
}

void setup()
{
  Serial.begin(9600);
  // Set the pins for each PWM signal                                                
  ch5.attach(5);
  ch6.attach(6); 
  ch9.attach(9); 
  ch10.attach(10);
  ch9.writeMicroseconds(1000);
  ch10.writeMicroseconds(1000);
  delay(1000);// needed for some motor drivers
  ///Setting up MSU:
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(2000);
  for (rateCalibrationNumber = 0; rateCalibrationNumber < 2000; rateCalibrationNumber++)
  {
    gyroAccGet();
    rateCalibrationRoll += rateRoll;
    rateCalibrationPitch += ratePitch;
    rateCalibrationYaw += rateYaw;
    delay(1);
  }
    rateCalibrationRoll /= 2000;
    rateCalibrationPitch /= 2000;
    rateCalibrationYaw /= 2000;
///
                             
  ResetData();      
                                  // Configure the NRF24 module                         
  radio.begin();
  radio.setChannel(100);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS); // The lowest data rate value for more stable communication 
  radio.setPALevel(RF24_PA_MAX);   // Output power is set for maximum                          
  radio.openReadingPipe(1,pipe);
  radio.startListening();        // Start the radio comunication for receiver                      
  //delay(120);
  loopTime = millis();
}

unsigned long lastRecvTime = 0;

void loop()
{
  ///MSU section
  gyroAccGet();
  
  rateRoll -= rateCalibrationRoll;
  ratePitch -= rateCalibrationPitch;
  rateYaw -= rateCalibrationYaw;

  kalman1d(kalmanAnglePitch, kalmanUncertaintyAnglePitch, -ratePitch, anglePitch);
  kalmanAnglePitch=kalman1DOOutput[0];
  kalmanUncertaintyAnglePitch = kalman1DOOutput[1];

  kalman1d(kalmanAngleRoll, kalmanUncertaintyAngleRoll, rateRoll, angleRoll);
  kalmanAngleRoll=kalman1DOOutput[0]; 
  kalmanUncertaintyAngleRoll = kalman1DOOutput[1];

  loopTime = millis();
  ///

  while ( radio.available() ) {
    radio.read(&data, sizeof(Signal));
    lastRecvTime = millis();                                     // Receive the data
  }

  if ( millis() - lastRecvTime > 1000 ) {
    ResetData();                                                // Signal lost.. Reset data 
  }
  
  if (data.autopilot) {
    int effectRoll  = limitAngle(-kalmanAngleRoll + map(data.roll, 0, 255, -30, 30));  
    int effectPitch = limitAngle(kalmanAnglePitch + map(data.pitch, 0, 255, -30, 30));

    ch_width_6 = map(effectRoll, -30, 30, 1000, 2000);  
    ch_width_5 = map(effectPitch, -30, 30, 1000, 2000); 

  } else {
    ch_width_6 = map(data.roll, 0, 255, 1000, 2000);
    ch_width_5 = map(data.pitch, 0, 255, 1000, 2000); 
  } 

  int throttleCorrection = 0;

  if (data.throttle > 80) {
    throttleCorrection = map(data.yaw, 0, 255, -80, 80);
  }

  int throttleLeft = data.throttle + throttleCorrection;
  if (throttleLeft > 255) {
    throttleLeft = 255;
  }
  int throttleRight = data.throttle -  throttleCorrection;
    if (throttleRight > 255) {
    throttleRight = 255;
  }

  ch_width_9 = map(throttleLeft, 0, 255, 1000, 2000); 
  ch_width_10 = map(throttleRight, 0, 255, 1000, 2000);
  ch6.writeMicroseconds(ch_width_6);                          // Write the PWM signal
  ch5.writeMicroseconds(ch_width_5);
  ch9.writeMicroseconds(ch_width_9);
  ch10.writeMicroseconds(ch_width_10);
}

int limitAngle (int angle) {
    if (angle > 30) {
      angle = 30;
    } 
    if (angle < -30) {
      angle = -30;
    }
    return angle;
}