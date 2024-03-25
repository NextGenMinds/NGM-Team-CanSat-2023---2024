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

// Libraries for GPS
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// Libraries for SD
#include <mySD.h>

// Libraries for OLED Display
#include <Wire.h> //we need this for BMP280 sensor too
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Libraries for BMP280
#include <Adafruit_BMP280.h>

// Library for error correction Reed Solomon
#include <RS-FEC.h>

const uint8_t ECC_LENGTH = 32;
const int messageSize = 96;
char message[messageSize];
char encoded[messageSize + ECC_LENGTH];
RS::ReedSolomon<messageSize, ECC_LENGTH> rs;

// Data File setup
File cansatData;
File sessionFile;   //SD card filenames are restricted to 8 characters + extension

// Define the pins used by the LoRa transceiver module
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 14
#define LORA_DIO0 26

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define LORA_BAND 866E6
#define SpreadingFactor 8
#define SyncWord 0xF8

int messageNumber = 0;

// SPI port #2:  SD Card Adapter
#define  SD_CLK     17
#define  SD_MISO    13
#define  SD_MOSI    12
#define  SD_CS      23

// GPS pins
static const int GPS_TX = 33;
static const int GPS_RX = 34;
static const uint32_t GPS_BAUD = 9600; //Default baun rate for GPS NEO 6M

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// OLED pins for I2C
#define OLED_SDA 4
#define OLED_SCL 15 
#define OLED_RST 16
#define OLED_ADDR 0x3C   //  OLED I2C address
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

// BMP280 I2C
#define BMP_ADDR 0x76 // I2C module address (SDA and SCL same as OLED)
Adafruit_BMP280 bmpSensor;
Adafruit_Sensor *bmp_temp = bmpSensor.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmpSensor.getPressureSensor();

// Select DeSelect for devices on SPI
#define  Select    LOW   //  Low CS means that SPI device Selected
#define  DeSelect  HIGH  //  High CS means that SPI device Deselected

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      
      gps.encode(gpsSerial.read());
  } 
  while (millis() - start < ms);

}

//Begin Setup
void setup() {
  //initialize Serial Monitor
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Hardware Serial Began");

  //initialize GPS SoftwareSerial
  gpsSerial.begin(GPS_BAUD);
  while (!gpsSerial);
  Serial.println("Software Serial Began");

  //reset OLED display via software
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  digitalWrite(OLED_RST, HIGH);

  //initialize OLED
  Wire.begin(OLED_SDA, OLED_SCL); //Wire.begin() initialize BMP280 sensor too (same I2C)
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR, false, false)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0,0);
  display.print("LORA SENDER ");
  display.display();
  
  Serial.println("LoRa Sender Test");

  //SPI LoRa pins
  pinMode(SD_CS, OUTPUT);
  pinMode(LORA_CS, OUTPUT);
  // Deselect LORA module
  digitalWrite(LORA_CS, DeSelect);

  // Select SD card module
  Serial.print("Initializing SD card...");
  digitalWrite(SD_CS, Select);    //  SELECT (Low) SD Card SPI
  delay(10);

  if (!SD.begin(SD_CS, SD_MOSI, SD_MISO, SD_CLK)) {
    Serial.println("initialization failed!");
    //  now what?
  } else {
    Serial.println("initialization done.");
    display.setCursor(5,row1);
    display.println("SD Card OK!");
    display.display();
    delay(1000);
  }
  // Open "cansat.txt" for writing 
  cansatData = SD.open("cansat1.txt", FILE_WRITE);
  if (cansatData) {
    //cansatData.println("GPS Data");
    cansatData.flush();
    cansatData.close();
  } else {    // File open error
    Serial.println("error opening cansat.txt");
  }
  display.setCursor(5,row2);
  display.println("Wrote in cansat.txt" );
  display.display();
  delay(1000);
  // After writing, then reopen the file and read it 
  cansatData = SD.open("/cansat1.txt");
  // if (cansatData) {    // Read from the file to the end of it 
  //   while (cansatData.available()) {  // Read the file
  //     Serial.write(cansatData.read());
  //   }
  //   cansatData.close();
  // } else {
  //   Serial.println("error opening cansat.txt");
  // }
  // display.setCursor(5,row3);
  // display.println("Read from cansat.txt" );
  // display.display();
  //delay(100);
  // Done testing the SD Card
  digitalWrite(SD_CS, DeSelect); 

  // Now test the LoRa
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS );
  LoRa.setPins( LORA_CS, LORA_RST, LORA_DIO0 );
  digitalWrite(LORA_CS, Select);   //  SELECT (low) LoRa SPI 
  Serial.println("LoRa Sender");
  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("Starting LoRa failed!");
    display.setCursor(5, row4);
    display.println("LoRa Init Failed!");
    display.display();
    // now what?
  } else {
    LoRa.setSpreadingFactor(SpreadingFactor);           // ranges from 6-12,default 7 see API docs
    LoRa.setSyncWord(SyncWord);           // ranges from 0-0xFF, default 0x34, see API docs
    Serial.println("LoRa Initial OK!");
    display.setCursor(5, row4);
    display.println("LoRa Initialized OK!");
    display.display();
    delay(1000);
  }
  digitalWrite(LORA_CS, DeSelect);  
  delay(10);
  Serial.println("Setup done!");

  // Initialize BMP280
  bool bmpStatus = bmpSensor.begin(BMP_ADDR, BMP280_CHIPID);
  if (!bmpStatus) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

}

void loop() {
  // Read BMP280 sensor data
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  // Serial.print(F("Temperature = "));
  // Serial.print(temp_event.temperature);
  // Serial.println(" *C");

  // Serial.print(F("Pressure = "));
  // Serial.print(pressure_event.pressure);
  // Serial.println(" hPa");

  // Read GPS Data
  smartDelay(1000);

  // Data from GPS and BMP280 sensor
  double latitude = gps.location.lat();
  double longitude = gps.location.lng();
  int day = gps.date.day();
  int month = gps.date.month();
  int year = gps.date.year();
  int hour = gps.time.hour();
  int minute = gps.time.minute();
  int second = gps.time.second();
  double speed = gps.speed.mps();
  double speedKPH = gps.speed.kmph();
  double course = gps.course.deg();
  double altitude = gps.altitude.meters();
  uint32_t satellites = gps.satellites.value();
  int32_t hdop = gps.hdop.value();
  float pressure = pressure_event.pressure;
  float temperature = temp_event.temperature;

  String dataString = String(messageNumber) + "," +
                          String(latitude, 6) + "," + 
                          String(longitude, 6) + "," + 
                          String(day) + "/" + String(month) + "/" + String(year) + "," + 
                          String(hour) + ":" + String(minute) + ":" + String(second) + "," + 
                          String(speed) + "," + 
                          String(speedKPH) + "," + 
                          String(course) + "," + 
                          String(altitude) + "," + 
                          String(satellites) + "," + 
                          String(hdop) + "," + 
                          String(pressure) + "," + 
                          String(temperature);

  Serial.println(dataString);

  // Encode dataString for safe tranfer
  int dataStringLength = dataString.length(); 
  //Serial.println(dataStringLength);

  // Clear message array
  memset(message, 0, sizeof(message));
  // Populate message char array
  for (int i = 0; i < dataStringLength; i++) { 
        message[i] = dataString[i]; 
  } 
  for (int i = dataStringLength; i < messageSize; i++) { 
        message[i] = '0'; 
  } 

  // make sure that the new string is null terminated 
  message[dataStringLength] = ',';
  //message[messageSize - 1] = '\0'; 
  Serial.println(message);

  //Encode message with Arduino-FEC
  rs.Encode(message, encoded);
  //Serial.println(sizeof(encoded));
  Serial.print("Encoded message: ");
  for(int i = 0; i < sizeof(encoded); i++) 
  {
    Serial.print(encoded[i]);    
  }    
  Serial.println("");

  //Serial.println(sizeof(encoded));

  // Send LoRa packet to receiver
  // Deselect SD module - Select LoRa module
  digitalWrite(LORA_CS, Select); //
  delay(10);
  digitalWrite(SD_CS, DeSelect);
  delay(10);

  //Send encoded packet
  LoRa.beginPacket();
  //LoRa.println(encoded);
  for(int i = 0; i < sizeof(encoded); i++) 
  {
    LoRa.print(encoded[i]);    
  }    
  LoRa.println("");
  LoRa.endPacket();


  // Select SD module - Deselect LoRa module
  digitalWrite(LORA_CS, DeSelect);
  delay(10);
  digitalWrite(SD_CS, Select);
  delay(10);

  //Save Data to SD
  cansatData = SD.open("/cansat1.txt", FILE_WRITE);
  if (cansatData) {
    cansatData.println(dataString);
    cansatData.flush();
    cansatData.close();
  } else {    //  file open error
    Serial.println("error opening cansat1.txt");
  }

  // Write on display
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, row1);
  display.println("LORA SENDER");
  display.setCursor(0, row2);
  display.print("New GPS Data --> ");
  display.setCursor(0, row3);
  display.print("Latitude = ");
  display.setCursor(65, row3);
  display.print(gps.location.lat());
  display.setCursor(0, row4);
  display.print("Longitude = ");
  display.setCursor(70, row4);
  display.print(gps.location.lng());  
  display.setCursor(0, row5); 
  display.print("Satellites --> ");
  display.setCursor(90, row5); 
  display.println(gps.satellites.value());
  display.setCursor(0, row6); 
  display.print(temp_event.temperature);
  display.setCursor(50, row6); 
  display.println(pressure_event.pressure);
  display.display();

  //Increase packet number
  messageNumber ++;
  //delay(5000);
}