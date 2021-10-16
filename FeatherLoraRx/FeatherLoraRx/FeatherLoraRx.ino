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

void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  
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
  if (rf95.available())
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
