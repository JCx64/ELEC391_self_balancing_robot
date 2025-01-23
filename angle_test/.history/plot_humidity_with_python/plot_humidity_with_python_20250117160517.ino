/*
  HS300x - Read Sensors

  This example reads data from the on-board HS300x sensor of the
  Nano 33 BLE Sense and prints the temperature and humidity sensor
  values to the Serial Monitor once a second.

  The circuit:
  - Arduino Nano 33 BLE Sense R2

  This example code is in the public domain.

  @note: This example code is been modified by Jiayi Chen, in order to fit plot_humidity_with_python.py
  @author: Jiayi Chen
  @date: 2025/1/17
*/

#include <Arduino_HS300x.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!HS300x.begin()) {
    while (1);
  }
}

void loop() {
  // read all the sensor values
  float humidity    = HS300x.readHumidity();

  // print each of the sensor values
  Serial.print(temperature);

  Serial.print(humidity);

  delay(50);
}
