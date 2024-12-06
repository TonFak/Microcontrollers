#include <Wire.h>
#include <RTClib.h>
#include <LiquidCrystal_I2C.h> 
#include <DHT.h>
#include <Adafruit_BMP085.h>

LiquidCrystal_I2C displayLCD(0x27, 16, 2);
DHT temperatureSensor(2, DHT22);
RTC_DS1307 rtcModule;
Adafruit_BMP085 pressureSensor;

void setup() {
  displayLCD.init();
  displayLCD.backlight();
  rtcModule.begin();
  temperatureSensor.begin();
  pressureSensor.begin();
}

void loop() {
  DateTime currentTime = rtcModule.now();

  displayLCD.setCursor(0, 0);
  int hours = currentTime.hour();
  int minutes = currentTime.minute();
  int seconds = currentTime.second();

  if (hours < 10) {
    displayLCD.print("0");
  }
  displayLCD.print(hours);
  displayLCD.print(":");
  if (minutes < 10) {
    displayLCD.print("0");
  }
  displayLCD.print(minutes);
  displayLCD.print(":");
  if (seconds < 10) {
    displayLCD.print("0");
  }
  displayLCD.print(seconds);

  displayLCD.print(" ");

  float temperature = temperatureSensor.readTemperature();
  displayLCD.print(temperature);
  displayLCD.print("°C");

  displayLCD.setCursor(0, 1);
  long pressure = pressureSensor.readPressure();
  displayLCD.print("P");
  displayLCD.print(pressure);

  displayLCD.print(" ");

  float humidity = temperatureSensor.readHumidity();
  displayLCD.print("H");
  displayLCD.print(humidity);

  delay(1000);
  displayLCD.clear();
}
