// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>

const int MPU=0x68; 
int x, y, z;
int xAve, yAve, zAve;

#define FILTER_DEPTH 10
#define ACC_THRESHOLD 1000

#define PKT_SIZE 64

// Blinky on Tx 
#define LED 13
 
// for feather m0  
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);
 
void setup() 
{
  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
    
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
 
  delay(1000); //Wait for serial to come up
 
  Serial.println("Feather LoRa TX Test!");
 
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
 
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  
  Serial.println("Configuring Accelerometer");
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
}
 
int16_t packetnum = 0;  // packet counter, we increment per xmission
 
void loop()
{
  int xdiff = 0, ydiff = 0, zdiff = 0;
  char radiopacket[PKT_SIZE];
  static unsigned int  pktNum = 0, loopcnt = 0;
  loopcnt++;
  
  PollMPUAccel();

  UpdateRunningAverage();

  if (loopcnt < FILTER_DEPTH) 
    return; 

  if (abs(x - xAve) > ACC_THRESHOLD)
  {
     Serial.print("XEvent: "); Serial.print(x);  Serial.print("Ave: ");  Serial.println(xAve); 
     xdiff = abs(x - xAve);
  }

  if (abs(y - yAve) > ACC_THRESHOLD)
  {
     Serial.print("YEvent: "); Serial.print(y);  Serial.print("Ave: ");  Serial.println(yAve); 
     ydiff = abs(y - yAve);
  }

  if (abs(z - zAve) > ACC_THRESHOLD)
  {
     Serial.print("ZEvent: "); Serial.print(z);  Serial.print("Ave: ");  Serial.println(zAve); 
     zdiff = abs(z - zAve);
  }

  if ((xdiff || ydiff || zdiff) == 0)
    return;

  digitalWrite(LED, HIGH);
    
  Serial.println("Transmitting..."); // Send a message to rf95_server
  pktNum++;

  sprintf(radiopacket, "SQUIRLY X: %d Y: %d Z: %d PKT: %d", xdiff, ydiff, zdiff, pktNum); 
  Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[PKT_SIZE - 1] = 0;
  
  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
 
  Serial.println("Waiting for packet to complete..."); 
  delay(100); //used to be 10
  rf95.waitPacketSent();
  Serial.println("Complete !"); 

  digitalWrite(LED, LOW);
}

void PollMPUAccel()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);  
  x=abs((Wire.read()<<8|Wire.read()) - 32768);
  y=abs((Wire.read()<<8|Wire.read()) - 32768);
  z=abs((Wire.read()<<8|Wire.read()) - 32768);

#ifdef DEBUG_ACC  
  Serial.print("Accelerometer: ");
  Serial.print("X = "); Serial.print(x);
  Serial.print(" | Y = "); Serial.print(y);
  Serial.print(" | Z = "); Serial.println(z); 
#endif

  delay(100);
}

void UpdateRunningAverage()
{
  static int i; 
  static int xArr[FILTER_DEPTH], xSum;
  static int yArr[FILTER_DEPTH], ySum;
  static int zArr[FILTER_DEPTH], zSum;
    
  xSum = xSum - xArr[i];            // Remove the oldest entry from the sum
  xArr[i] = x; 
  xSum = xSum + x;
  xAve = xSum / FILTER_DEPTH;

  ySum = ySum - yArr[i];            // Remove the oldest entry from the sum
  yArr[i] = y; 
  ySum = ySum + y;
  yAve = ySum / FILTER_DEPTH;

  zSum = zSum - zArr[i];            // Remove the oldest entry from the sum
  zArr[i] = z; 
  zSum = zSum + z;
  zAve = zSum / FILTER_DEPTH;

  i = (i+1) % FILTER_DEPTH;
}
