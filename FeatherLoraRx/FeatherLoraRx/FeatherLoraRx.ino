// Feather9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_TX

#include <SPI.h>
#include <RH_RF95.h>
#include "pitches.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "uptime.h"

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

typedef struct 
{
    uint32_t timestamp;
    uint32_t event;
} event_t;

#define MAX_EVENT_LOG 2
#define MAX_DEVICES 4
typedef struct 
{
    uint16_t voltage;
    int16_t rssi;
    uint32_t timestamp; //This records the time of the last incoming message (used to keep track of units going offline) 
    event_t events[MAX_EVENT_LOG];
} log_t;

log_t logs[MAX_DEVICES];

//#define ENABLE_SERIAL_PORT  //Need to disable this for stand-alone use 


// notes in the melody:
const int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
const int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

// for feather m0 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

#define BUZZER 14  //For M0 Rx 
//#define BUZZER 16

//***********************************************************
//                  OLED Display Defines 
//***********************************************************

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Define the to left-right cordinates of each Sensor Quadrant 
#define N1_X 0
#define N1_Y 0

#define N2_X (display.width() / 2) + 2
#define N2_Y 0

#define N3_X 0
#define N3_Y (display.height() / 2) + 1

#define N4_X (display.width() / 2) + 2
#define N4_Y (display.height() / 2) + 1

#define Y_ROW 10 //10 pixels per row 

#define BATT_W 15
#define BATT_H 8

#define RSSI_WIDTH 14
#define RSSI_HEIGHT 7


void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  memset(logs, 0, sizeof(logs));
  
#ifdef ENABLE_SERIAL_PORT
  while (!Serial) {
    delay(1);
  }
#endif

  delay(100);

  Serial.println("Feather LoRa RX Test!");

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

  digitalWrite(BUZZER, LOW);
  digitalWrite(LED , LOW);
}

void loop()
{
  int gotPkt = 0;
  if (rf95.waitAvailableTimeout(1000, 10)) //Time out every second so we can redraw the UI
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      memcpy(&pkt, &buf, sizeof(pkt));
      Serial.print("Got Pkt Len: ");
      Serial.println(len);

      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

      if ((len == sizeof(pkt)) && (pkt.dst = 0xFEEDBABE))
      {
          Serial.print("Got Packet From Unit: ");
          Serial.println(pkt.src);
          Serial.print("Voltage: ");
          Serial.println(pkt.voltage);
          Serial.print("Event: ");
          Serial.println(pkt.event, HEX);

          if (pkt.src < MAX_DEVICES)
          {
            logs[pkt.src].timestamp = millis();
            logs[pkt.src].rssi = rf95.lastRssi();
            logs[pkt.src].voltage = pkt.voltage;
            if ((logs[pkt.src].timestamp - logs[pkt.src].events[0].timestamp) > 5000)  //Only record new event if its more than 5 sec old
            {
              logs[pkt.src].events[1].timestamp = logs[pkt.src].events[0].timestamp; 
              logs[pkt.src].events[1].event = logs[pkt.src].events[0].event;
              logs[pkt.src].events[0].timestamp = millis();
              logs[pkt.src].events[0].event = pkt.event;
            }
          }
                    
          gotPkt = 1;
       }
       else 
       {
          Serial.println("Packet not for us");
       }
    }
    else
    {
      Serial.println("Rx Failed");
    }
    if (gotPkt)
    {
      digitalWrite(LED, HIGH);
      digitalWrite(BUZZER, HIGH);
//      PlayTones();  //Cant call any functions here w/o crashing 
//      PlayMusic();
      delay(500);
      digitalWrite(LED, LOW);
      digitalWrite(BUZZER, LOW);
    }
  }
  drawUI();
}

int PlayTones()  
{

}
 

int PlayMusic()  //This causes some form of stack corruption 
{
 for (int thisNote = 0; thisNote < 8; thisNote++) 
 {
   // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZER, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    Serial.println(pauseBetweenNotes);
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(BUZZER);
  }
}

void drawUI()
{
  display.clearDisplay();

  drawFrame();
  drawBattery(N1_X + 43, N1_Y + 0, logs[0].voltage);
  drawBattery(N2_X + 43, N2_Y + 0, logs[1].voltage);
  drawBattery(N3_X + 43, N3_Y + 0, logs[2].voltage);
  drawBattery(N4_X + 43, N4_Y + 0, logs[3].voltage);

  drawRSSI(N1_X + 23, N1_Y + 0, logs[0].rssi); 
  drawRSSI(N2_X + 23, N2_Y + 0, logs[1].rssi); 
  drawRSSI(N3_X + 23, N3_Y + 0, logs[2].rssi); 
  drawRSSI(N4_X + 23, N4_Y + 0, logs[3].rssi); 
  
  drawText();    

  display.display();
}

void drawFrame()
{
  display.drawLine(0, (display.height() / 2) - 1 , display.width() - 1, (display.height() / 2) - 1, SSD1306_WHITE); // Vertical Line
  display.drawLine(display.width() / 2, 0, display.width() / 2, display.height() -1, SSD1306_WHITE);   //Horizontal Line
}

void drawBattery(int x, int y, int mV) 
{
  display.drawRect(x, y, BATT_W, BATT_H, SSD1306_WHITE);
  display.drawRect(x + BATT_W, y + (BATT_H / 2) - 2 , 2, 4, SSD1306_WHITE);

  //Figure out conversion from Voltage to pixels 
  if (mV < 3200)
    mV = 3200;
  
  display.fillRect(x, y, BATT_W * ((mV - 3200.0) / (4200.0 - 3200.0)), BATT_H, SSD1306_WHITE);
}

void drawRSSI(int x, int y, int db) 
{
  int i = 10;
  int xt, yt;
  int rfMargin;
  
  rfMargin = 110 - abs(db);

  display.drawTriangle(
      x, y + 7,
      x+14, y, 
      x+14, y + 7, SSD1306_WHITE);

    if (rfMargin < 0)  
      return;        
    else if (rfMargin < 10) {
        xt = 4;
        yt = 2;
    }
    else if (rfMargin < 20) { 
        xt = 6;
        yt = 3;
    }
    else if (rfMargin < 30) { 
        xt = 8;
        yt = 4;
    }
    else if (rfMargin < 40) {
        xt = 10;
        yt = 5;      
    }
    else if (rfMargin < 60) {
        xt = 12;
        yt = 6;      
    }
    else //Full Signal 
    {
        xt = 14;
        yt = 7;      
    }

    display.fillTriangle(
      x, y + RSSI_HEIGHT, //Lower left corner, always same
      x + xt, y + RSSI_HEIGHT - yt, //Upper right corner. X & Y Varies 
      x + xt, y + RSSI_HEIGHT, SSD1306_WHITE); //Lower right corner, X varies 
}

void drawText(void) {
  char str[32];
  unsigned long tc = millis();
  unsigned long td;
  
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(N1_X, N1_Y);     // Start at top-left corner
  display.write("#1");

  if (logs[0].events[0].timestamp)
  {  
    display.setCursor(N1_X, N1_Y + Y_ROW);
    display.write(convTime((tc - logs[0].events[0].timestamp) / 1000));
  }
  
  if (logs[0].events[1].timestamp)
  {  
    display.setCursor(N1_X, N1_Y + Y_ROW * 2);
    display.write(convTime((tc - logs[0].events[1].timestamp) / 1000));
  }

  display.setCursor(N2_X, N2_Y);     // Start at top-left corner
  display.write("#2");

  if (logs[1].events[0].timestamp)
  {    
    display.setCursor(N2_X, N2_Y + Y_ROW);
    display.write(convTime((tc - logs[1].events[0].timestamp) / 1000));
  }
  
  if (logs[1].events[1].timestamp)
  {    
    display.setCursor(N2_X, N2_Y + Y_ROW * 2);
    display.write(convTime((tc - logs[1].events[1].timestamp) / 1000));
  }

  display.setCursor(N3_X, N3_Y);     // Start at top-left corner
  display.write("#3");

  if (logs[2].events[0].timestamp)
  {   
    display.setCursor(N3_X, N3_Y + Y_ROW);
    display.write(convTime((tc - logs[2].events[0].timestamp) / 1000));
  }
  
  if (logs[2].events[1].timestamp)
  {   
    display.setCursor(N3_X, N3_Y + Y_ROW * 2);    
    display.write(convTime((tc - logs[2].events[1].timestamp) / 1000));
  }

  display.setCursor(N4_X, N4_Y);     // Start at top-left corner
  display.write("#4");

  if (logs[3].events[0].timestamp)
  {   
    display.setCursor(N4_X, N4_Y + Y_ROW);
    display.write(convTime((tc - logs[3].events[0].timestamp) / 1000));
  }
    
  if (logs[3].events[1].timestamp)
  {   
    display.setCursor(N4_X, N4_Y + Y_ROW * 2);
    display.write(convTime((tc - logs[3].events[1].timestamp) / 1000));
  }
}

//Converts t in seconds to hh:mm::ss string 
char *convTime(unsigned long t)
{
  static char str[32];
  unsigned long seconds = t;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  unsigned long days = hours / 24;
  seconds %= 60;
  minutes %= 60;
  hours %= 24;

  sprintf(str, "%02d:%02d:%02d", hours, minutes, seconds);
  return str;
}
