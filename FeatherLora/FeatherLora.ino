
//Enable sensor here 
#define CONFIG_MPU6050
//#define CONFIG_VL53L1X
#define DEVICE_ID 3

//#define DEBUG

#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "SparkFun_VL53L1X.h"

//******** Pin definitions *************
#define LED 13 //Flashes on activity 

#ifdef __AVR_ATmega32U4__
#define BATT_SENSOR_PIN A9
#else //M0 
#define BATT_SENSOR_PIN A7 
#endif

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

//**************************************


#define RF95_FREQ 915.0


#ifdef __AVR_ATmega32U4__
#define EEPROM_ADDRESS_FOR_OFFSET (E2END - 2) // last 2 bytes of EEPROM
#endif

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

enum {
  EVENT_DISTANCE = 1 << 0,
  EVENT_VIBRATION = 1 << 1,
} event_e;

typedef struct 
{
    uint32_t dst;
    uint8_t ver;
    uint8_t src;
    uint16_t voltage;
    uint32_t event;
} pkt_t;

#ifdef DEBUG
#define dbg(...) Serial.print(__VA_ARGS__)
#define dbgln(...) Serial.println(__VA_ARGS__)
#else
#define dbg(...) 
#define dbgln(...) 
#endif

pkt_t pkt;
unsigned long next_callin; 
 
void setup() 
{
  pinMode(LED, OUTPUT);

  digitalWrite(LED, HIGH); //Flash LED to indicate unit is initializing 
  delay(500);
  digitalWrite(LED, LOW);
    
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
 
  delay(1000); //Wait for serial to come up
 
  digitalWrite(RFM95_RST, LOW); //Reset LoRA 
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  randomSeed(analogRead(0));  //This is used to randomize the call-in periods 
  next_callin = millis() + random(50000, 80000);
 
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

  rf95.setTxPower(23, false); //Full Power

  setupSensor();

  dbgln("Testing!");
  Serial.println(RF95_FREQ);

  //Fill in static items 
  pkt.dst = 0xFEEDBABE;
  pkt.ver = 1;
  pkt.src = DEVICE_ID; //eeprom_read_word((uint16_t *)EEPROM_ADDRESS_FOR_OFFSET)
#ifdef __AVR_ATmega32U4__ 
  eeprom_read_word((uint16_t *)EEPROM_ADDRESS_FOR_OFFSET);
#endif   
}
 
void loop()
{
  static unsigned int  pktNum = 0;
  int ev;

  ev = sensorProcessing();
 
  if (ev == 0)
  {
    if (millis() > next_callin)  //Time for call-in ?
      next_callin = millis() + random(50000, 80000);
    else
      return;
  }

  //if we get here there is activity 
  if (ev)
    digitalWrite(LED, HIGH);
    
  pktNum++;

  pkt.event = ev;
#ifdef __AVR_ATmega32U4__  
  pkt.voltage = analogRead(BATT_SENSOR_PIN) * ((4.35 * 1000)/1024.0);  //1024 = 6.6V //Lora32U4 MK2 does not have the same resistor divider :(
#else
  pkt.voltage = analogRead(BATT_SENSOR_PIN) * ((6.6 * 1000)/1024.0);  //1024 = 6.6V
#endif

  if (ev) 
    Serial.println("Activity Detected - Sending pkt");
  else 
    Serial.println("Sending Call-in");
  Serial.print("Battery Voltage: "); Serial.println(pkt.voltage);

  rf95.send((uint8_t *)&pkt, sizeof(pkt_t)); 
  
  Serial.println("Waiting for packet to complete..."); 
  delay(100); //used to be 10
//  rf95.waitPacketSent(); //This does not work for some reason
  Serial.println("Complete !"); 

  if (ev)
    digitalWrite(LED, LOW);
}

//**********************************************
//*************     PMPU6050    ****************
//**********************************************

#ifdef CONFIG_MPU6050
const int MPU_I2C_ADDR=0x68; 
int x, y, z;
int xAve, yAve, zAve;

#define FILTER_DEPTH 10
#define ACC_THRESHOLD 1000

int setupSensor()
{
  Serial.println("Configuring Accelerometer");
  Wire.begin();
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
}

int sensorProcessing() //Returns 1 if activity is detected 
{
  int xdiff = 0, ydiff = 0, zdiff = 0;
  static unsigned int  pktNum = 0, loopcnt = 0;
  loopcnt++;
  
  PollMPUAccel();

  UpdateRunningAverage();

  if (loopcnt < FILTER_DEPTH) 
    return 0; 

  if (abs(x - xAve) > ACC_THRESHOLD)
  {
     Serial.print("XEvent: "); Serial.print(x);  Serial.print(" Ave: ");  Serial.println(xAve); 
     xdiff = abs(x - xAve);
  }

  if (abs(y - yAve) > ACC_THRESHOLD)
  {
     Serial.print("YEvent: "); Serial.print(y);  Serial.print(" Ave: ");  Serial.println(yAve); 
     ydiff = abs(y - yAve);
  }

  if (abs(z - zAve) > ACC_THRESHOLD)
  {
     Serial.print("ZEvent: "); Serial.print(z);  Serial.print(" Ave: ");  Serial.println(zAve); 
     zdiff = abs(z - zAve);
  }

  if ((xdiff || ydiff || zdiff) == 0)
    return 0;

  return EVENT_VIBRATION;
}

void PollMPUAccel()
{
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_I2C_ADDR,12,true);  
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
#endif



//**********************************************
//*************     VL53L1X     ****************
//**********************************************
#ifdef CONFIG_VL53L1X
SFEVL53L1X distanceSensor;

//Store distance readings to get rolling average
#define HISTORY_SIZE 10
int history[HISTORY_SIZE];
byte historySpot;

void setupSensor()
{
  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");

  for (int x = 0; x < HISTORY_SIZE; x++)
    history[x] = 0;
}

int sensorProcessing()
{
  static unsigned int statusBits = 0;
  static unsigned int statusOld = 0;
  static int initCnt = HISTORY_SIZE;
  int statusChange = 0;
  int led_enable = 0;
  static unsigned int  pktNum = 0, loopcnt = 0;

  distanceSensor.startRanging(); //Write configuration block of 135 bytes to setup a measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }

  int distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();
  
  if (distance == 0) //Some form of error
    return 0;

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

  if (initCnt)
  {
    initCnt--; //To avoid trigger when first powered on 
    return 0;
  }

  byte rangeStatus = distanceSensor.getRangeStatus();
  dbg("Range Status: ");

  //Make it human readable
  switch (rangeStatus)
  {
  case 0:
    dbgln("Good");
    break;
  case 1:
    dbgln("Sigma fail");
    break;
  case 2:
    dbgln("Signal fail");
    break;
  case 7:
    dbgln("Wrapped target fail");
    break;
  default:
    dbg("Unknown: ");
    dbgln(rangeStatus);
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
    dbg("Status Switching to OK");
  }
  else if ((statusBits > 7) && (statusOld == 0))
  {
    statusOld = 1;
    statusChange = 1;
    dbg("Status Switching to Error");
  }
  
  dbg("StatusBits: ");
  dbg(statusBits);
  dbgln();

  float percentDiff = (100 * ((float)distance / (float) avgDistance));
  
  if ((((percentDiff < 80) || (percentDiff > 120)) && (led_enable == 0) && (rangeStatus == 0)) ||
      ((led_enable == 0) && statusChange))
  {
    led_enable = 500;
    Serial.print("Motion Detection\n");
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" AveDistance: ");
    Serial.println(avgDistance);
    Serial.print("Percent Diff: ");
    Serial.println(percentDiff);
    Serial.print("History Spot: ");
    Serial.println(historySpot);

    for (int i = 0; i < HISTORY_SIZE; i++)
    {
      Serial.print(" ");
      Serial.print(history[i]);
    }

    Serial.print("\nStatusBits: ");
    Serial.println(statusBits, HEX);  
    Serial.print("StatusOld: ");
    Serial.println(statusOld, HEX);  
    return EVENT_DISTANCE;
  }
  return 0;   
}
#endif
