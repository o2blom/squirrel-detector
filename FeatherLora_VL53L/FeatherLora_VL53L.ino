// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <Wire.h>
#include <RH_RF95.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

const int MPU=0x68; 

// Blinky on Tx 
#define LED 13
 
// for feather m0  
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define BATT_SENSOR_PIN A9

int sensorPin = A0;   

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
 
// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

SFEVL53L1X distanceSensor;

//Store distance readings to get rolling average
#define HISTORY_SIZE 10
int history[HISTORY_SIZE];
byte historySpot;

enum {
  EVENT_MOTION = 1 << 0,
  EVENT_MOTION_2 = 1 << 1,
} event_e;

typedef struct 
{
    uint32_t dst;
    uint8_t ver;
    uint8_t src;
    uint16_t voltage;
    uint32_t event;
} pkt_t;

pkt_t pkt;

void setup() 
{
  Wire.begin();
  
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

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");

  for (int x = 0; x < HISTORY_SIZE; x++)
    history[x] = 0;

  //Fill in static items 
  pkt.dst = 0xFEEDBABE;
  pkt.ver = 1;
  pkt.src = 2;
}
 
int16_t packetnum = 0;  // packet counter, we increment per xmission
 
void loop()
{
  static unsigned int statusBits = 0;
  static unsigned int statusOld = 0;
  int statusChange = 0;
  int led_enable = 0;
  static unsigned int  pktNum = 0, loopcnt = 0;
  loopcnt++;

  distanceSensor.startRanging(); //Write configuration block of 135 bytes to setup a measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }

  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  if (distance == 0)
    return;

  Serial.print("Distance(mm): ");
  Serial.print(distance);

  //Add new sample to rotating buffer
  history[historySpot] = distance;
  if (++historySpot == HISTORY_SIZE)
    historySpot = 0;

  //Add all values in history together 
  long avgDistance = 0;
  for (int x = 0; x < HISTORY_SIZE; x++)
    avgDistance += history[x];

  //And divide to find the running average 
  avgDistance /= HISTORY_SIZE;

  byte rangeStatus = distanceSensor.getRangeStatus();
  Serial.print("\tRange Status: ");

  //Make it human readable
  switch (rangeStatus)
  {
  case 0:
    Serial.print("Good");
    break;
  case 1:
    Serial.print("Sigma fail");
    break;
  case 2:
    Serial.print("Signal fail");
    break;
  case 7:
    Serial.print("Wrapped target fail");
    break;
  default:
    Serial.print("Unknown: ");
    Serial.print(rangeStatus);
    break;
  }

  //Sample in status bits. All zeroes = all OK
  if ((statusBits < 10) && (rangeStatus != 0))
    statusBits++;
  else if ((statusBits > 0) && (rangeStatus == 0))
    statusBits--;

  if ((statusBits < 3) && (statusOld == 1)) //Going from Error to OK ? 
  {
    statusOld = 0;
    statusChange = 1;
    Serial.print("Status Switching to OK");
  }
  else if ((statusBits > 7) && (statusOld == 0))
  {
    statusOld = 1;
    statusChange = 1;
    Serial.print("Status Switching to Error");
  }
  
//  statusBits = (statusBits << 1) | (!!rangeStatus);  //16 samples

  Serial.print("\tStatusBits: ");
  Serial.print(statusBits);
  

  Serial.println();

  //(abs(distance - avgDistance) > 100)
  float percentDiff = (100 * ((float)distance / (float) avgDistance));

  
  
  if ((((percentDiff < 80) || (percentDiff > 120)) && (led_enable == 0) && (rangeStatus == 0)) ||
      ((led_enable == 0) && statusChange))
  {
    led_enable = 500;
    Serial.print("Motion Detection\n");
  }
  else return;
  
  if (loopcnt < HISTORY_SIZE) 
    return; 

  digitalWrite(LED, HIGH);
    
  Serial.println("Transmitting..."); // Send a message to rf95_server
  pktNum++;

  pkt.event = EVENT_MOTION;
  pkt.voltage = analogRead(BATT_SENSOR_PIN) * ((4.35 * 1000)/1024.0);  //1024 = 6.6V

  Serial.print("Battery Voltage: "); Serial.println(pkt.voltage);

  Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)&pkt, sizeof(pkt_t));
 
  Serial.println("Waiting for packet to complete..."); 
  delay(100); //used to be 10
// rf95.waitPacketSent();  //D
  Serial.println("Complete !"); 

  digitalWrite(LED, LOW);
}

int GetBattVoltage()
{
  float measuredvbat = analogRead(BATT_SENSOR_PIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return (measuredvbat * 1000);
}
