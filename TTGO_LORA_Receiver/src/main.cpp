/**
 * NGM Team CanSat Greece 2023 - 2024
 * TTGO ESP32 LoRa v1 (Correct Pinout https://primalcortex.files.wordpress.com/2017/11/ttgolorapinout_v2.jpg)
   https://primalcortex.wordpress.com/2017/11/24/the-esp32-oled-lora-ttgo-lora32-board-and-connecting-it-to-ttn/
 * GPS NEO 6M 3.3v
 * SD Card module 5v 
 * BMP280 sensor 3.3v
 * For SD Card module follow 
 * https://www.youtube.com/watch?v=hLCFNR3jp8Y
 * For Two I2C devices follow
 * https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide/
 * Libraries
 * Adafruit SSD1306@^2.5.9
	 Adafruit GFX Library@^1.11.9
	 LoRa@^0.8.0
	 esp32-micro-sdcard@^0.1.1
	 TinyGPSPlus@^1.0.3
	 EspSoftwareSerial@^8.1.0
	 Adafruit BMP280 Library@^2.6.8

//////////////////////////// MIT License /////////////////////////////////////////
  Copyright (c) <2023> <NextGenMinds.org>

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////
*/
// PlatFormIO need this
#include <Arduino.h>

#include <string>
#include <cstring>

// Libraries for LoRa
#include <SPI.h>
#include <LoRa.h>

//Libraries for OLED Display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 866E6
#define SpreadingFactor 8
#define SyncWord 0xF8


//OLED pins
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define OLED_ADDR 0x3C   //  OLED display TWI address
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

// OLED screen text rows:
#define  row1     0     //  y=0 is top row of size 1 text
#define  row2    10
#define  row3    20
#define  row4    30
#define  row5    40
#define  row6    50
#define  row7    60     //  row7 at 60 is too low

String LoRaData;

// Library for error correction Reed Solomon
#include <RS-FEC.h>

const uint8_t ECC_LENGTH = 32;
const int messageSize = 96;
char repaired[messageSize];
char encodedMessage[messageSize + ECC_LENGTH];
RS::ReedSolomon<messageSize, ECC_LENGTH> rs;

// Number for comparing messages
int messageNumber = 0;

void setup() { 
  //initialize Serial Monitor
  Serial.begin(115200);
  
  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  
  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA RECEIVER ");
  display.display();

  Serial.println("LoRa Receiver Test");
  
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);

  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(SpreadingFactor);           // ranges from 6-12,default 7 see API docs
  LoRa.setSyncWord(SyncWord);           // ranges from 0-0xFF, default 0x34, see API docs
  Serial.println("LoRa Initializing OK!");
  display.setCursor(0,10);
  display.println("LoRa Initializing OK!");
  display.display();  
}

void loop() {

  //try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    //received a packet
    // Serial.print("Received packet ");

    //read packet
    while (LoRa.available()) {
      LoRaData = LoRa.readString();
      Serial.print(messageNumber);
      Serial.print(',');
      Serial.println(String(LoRaData));
      
      // Decode dataString for safe tranfer
      int dataStringLength = LoRaData.length(); 
      if(dataStringLength == (messageSize + ECC_LENGTH + 2))
      {
        for (int i = 0; i < messageSize + ECC_LENGTH; i++) { 
          encodedMessage[i] = LoRaData[i]; 
        } 


        rs.Decode(encodedMessage, repaired);

        Serial.print(messageNumber);
        Serial.print(',');

        for (int i = 0; i < sizeof(repaired); i++)
        {
          Serial.print(repaired[i]);
        }
        Serial.println();
        


        //print RSSI of packet
        int rssi = LoRa.packetRssi();
        // Serial.print(" with RSSI ");    
        // Serial.println(rssi); // received signal strength indicator
        
        // Dsiplay information
        display.clearDisplay();
        display.setCursor(0,row1);
        display.print("LORA RECEIVER");
        display.setCursor(0,row2);
        display.print("RSSI:");
        display.setCursor(30,row2);
        display.print(rssi);
        display.setCursor(0,row3);
        display.print("Received packet:");
        display.setCursor(0,row4);
        display.print(String(repaired));
        display.display();
      }

      messageNumber++;
      
    }
  }
}