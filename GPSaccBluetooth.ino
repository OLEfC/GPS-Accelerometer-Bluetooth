#include <stdio.h>
#include "MPU9250.h"
#include "Math.h"
#include "BluetoothSerial.h"
#include <TinyGPS++.h>
#include "BluetoothSerial.h"

#define GPS_BAUDRATE 9600  // The default baudrate of NEO-6M is 9600
#define time_offset 7200  // define a clock offset of 7200 seconds (2 hour) ==> UTC + 2
TinyGPSPlus gps;  // the TinyGPS++ object



MPU9250 IMU(Wire,0x68);
int status;
const int ledPin = 32;
//#define USE_PIN // Uncomment this to use PIN during pairing. The pin is specified on the line below
const char *pin = "1234"; // Change this to more secure PIN.

String device_name = "ESP32-BT";

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;





void setup() {
  pinMode (ledPin, OUTPUT);
  Serial.begin(9600);
  Serial2.begin(GPS_BAUDRATE);

  Serial.println(F("ESP32 - GPS module"));

  while(!Serial) {}
  SerialBT.begin(device_name); //Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
  #ifdef USE_PIN
    SerialBT.setPin(pin);
    Serial.println("Using PIN");
  #endif
  status = IMU.begin();
  if (status < 0) {
    
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.println("Status: ");
    Serial.println(status);
    while(1) {}
  }

}
int get_acc() {
  float accX = IMU.getAccelX_mss();
  float accY = IMU.getAccelY_mss();
  float accZ = IMU.getAccelZ_mss();
  // Serial.print("accX:");
  // Serial.println(accX);
  // Serial.print("accY:");
  // Serial.println(accY);
  // Serial.print("accZ:");
  // Serial.println(accZ);Serial
  int acc = (int)abs(9.9-sqrt(accX*accX + accY*accY + accZ*accZ));
  return acc;
}
void get_gps(){
    if (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if (gps.location.isValid()) {
        SerialBT.print(F("- latitude: "));
        SerialBT.println(gps.location.lat(), 7);

        SerialBT.print(F("- longitude: "));
        SerialBT.println(gps.location.lng(), 7);

        // SerialBT.print(F("- altitude: "));
        // if (gps.altitude.isValid())
        //   SerialBT.println(gps.altitude.meters(), 7);
        // else
        //   SerialBT.println(F("INVALID"));
      } else {
        SerialBT.println(F("- location: INVALID"));
      }

      // SerialBT.print(F("- speed: "));
      // if (gps.speed.isValid()) {
      //   SerialBT.print(gps.speed.kmph());
      //   SerialBT.println(F(" km/h"));
      // } else {
      //   SerialBT.println(F("INVALID"));
      // }

      SerialBT.print(F("- GPS date&time: "));
      if (gps.date.isValid() && gps.time.isValid()) {
        SerialBT.print(gps.date.year());
        SerialBT.print(F("-"));
        SerialBT.print(gps.date.month());
        SerialBT.print(F("-"));
        SerialBT.print(gps.date.day());
        SerialBT.print(F(" "));
        SerialBT.print(gps.time.hour()+3);
        SerialBT.print(F(":"));
        SerialBT.print(gps.time.minute());
        SerialBT.print(F(":"));
        SerialBT.println(gps.time.second());
      } else {
        SerialBT.println(F("INVALID"));
      }

      SerialBT.println();
    }
  }

  if (millis() > 5000 && gps.charsProcessed() < 10)
    SerialBT.println(F("No GPS data received: check wiring"));
}


void loop() {
IMU.readSensor();
  
  int acc = get_acc();

  // Serial.print("Variable_2:");
  // Serial.println(acc);
  get_gps();

  if(acc>=20){
    digitalWrite (ledPin, HIGH);  // turn on the LED
    SerialBT.println("FALL DETECTED");
    get_gps();
    SerialBT.println("delay(20000);");


    // Serial.print('/n');
     delay(20000);      
    digitalWrite (ledPin, LOW);  // turn on the LED

  }
  else{
    // Serial.print('0');
    // Serial.print('\n');
  }
  delay(10);
}
