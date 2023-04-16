#include <Arduino.h>
#include <TinyGPS++.h>
#include <SPI.h>
#include <LoRa.h>

TinyGPSPlus gps;
//HardwareSerial Serial1(1);

const uint8_t vbatPin = 34;
float battery_voltage = 0;
unsigned int _conter=0;

//Lora constant
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISnO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define BAND    433E6

static void smartDelay(unsigned long ms);

void voltage(){
  digitalWrite(14, HIGH);
  delay(1);
  float measurement = (float) analogRead(34);
  battery_voltage = (measurement / 4095.0) * 7.26;
  digitalWrite(14, LOW);
  //Serial.print("BAT Volt: ");
  //Serial.println(battery_voltage);
  if (battery_voltage < 3.75){    
    Serial.println("Bateria Baixa");
    //delay(20000);
    //esp_deep_sleep_start(); // to save the lipo - no wakeup.
    // RETURN;
  }  
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 12, 15);   //17-TX 18-RX
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND)) 
  {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  delay(1500);
}

void loop()
{
  Serial.print("Latitude  : ");
  Serial.println(gps.location.lat(), 5);
  Serial.print("Longitude : ");
  Serial.println(gps.location.lng(), 4);
  Serial.print("Satellites: ");
  Serial.println(gps.satellites.value());
  Serial.print("Altitude  : ");
  Serial.print(gps.altitude.feet() / 3.2808);
  Serial.println("M");
  Serial.print("Time      : ");
  Serial.print(gps.time.hour());
  Serial.print(":");
  Serial.print(gps.time.minute());
  Serial.print(":");
  Serial.println(gps.time.second());
  //voltage();
  Serial.print("Bateria    :");
  Serial.println(battery_voltage);
  Serial.println("**********************");
  String msg;
  //String _tmp; // LAT:LONG:SAT:ALT:BAT:CONTER
  char _tmp[24];
  dtostrf((gps.location.lat()),10,7,_tmp);
  msg = "" + String(_tmp) + ":";
  char _tmp1[24];
  dtostrf((gps.location.lng()),10,7,_tmp1);
  msg = msg + _tmp1 + ":";
  msg = msg + gps.satellites.value() + ":";
  msg = msg + gps.altitude.meters() + ":";
  //msg = msg + battery_voltage + ":";
  msg = msg + "0:";
  msg = msg + _conter + ":0";
  Serial.println(msg);
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
  if (_conter >= 32000)
    _conter = 0;
  else
    _conter = _conter + 1;
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (Serial1.available())
      gps.encode(Serial1.read());
  } while (millis() - start < ms);
}