//  8 Channel Receiver
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <avr/wdt.h>

int ch_width_3 = 0;
int ch_width_5 = 0;
int ch_width_6 = 0;

Servo ch6;
Servo ch5;
Servo ch3;
struct Signal {
byte throttle;
byte pitch;  
byte roll;   
};
Signal data;
const uint64_t pipeIn = 000322;
RF24 radio(9, 10); 

void ResetData()
{
data.throttle = 0;
data.roll = 127;
data.pitch = 127;                                                   
}
void setup()
{
  Serial.begin(9600);
                                                           // Set the pins for each PWM signal
  ch3.attach(3);
  ch5.attach(5);
  ch6.attach(6);

                                                           
  ResetData();                                             // Configure the NRF24 module
  radio.begin();
  radio.openReadingPipe(1,pipeIn);
  radio.setChannel(100);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);                          // The lowest data rate value for more stable communication  
  radio.setPALevel(RF24_PA_MAX);                            // Output power is set for maximum
  radio.startListening();                                   // Start the radio comunication for receiver
  wdt_enable(WDTO_120MS);
  delay(120);
}
unsigned long lastRecvTime = 0;
void recvData()
{
  while ( radio.available() ) {
  radio.read(&data, sizeof(Signal));
  lastRecvTime = millis();                                    // Receive the data
  wdt_reset();
  }
}
void loop()
{
  recvData();
  unsigned long now = millis();
  if ( now - lastRecvTime > 1000 ) {
    ResetData();                                                // Signal lost.. Reset data 
  }
  ch_width_6 = map(data.roll, 0, 255, 1000, 2000);  
  ch_width_5 = map(data.pitch, 0, 255, 1000, 2000); 
  ch_width_3 = map(data.throttle, 0, 255, 1000, 2000);  
  ch6.writeMicroseconds(ch_width_6);                          // Write the PWM signal
  ch5.writeMicroseconds(ch_width_5);
  ch3.writeMicroseconds(ch_width_3);
  wdt_reset();
}