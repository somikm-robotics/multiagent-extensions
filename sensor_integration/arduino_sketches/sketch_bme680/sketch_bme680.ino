#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

Adafruit_BME680 bme;

void setup() {
  Serial.begin(115200);
  if (!bme.begin(0x77)) {
    Serial.println("BME680 not found!");
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setGasHeater(320, 150); // temp °C, ms
}

void loop() {
  if (!bme.performReading()) return;
  Serial.print("Temp: "); Serial.print(bme.temperature); Serial.print(" °C  ");
  Serial.print("Hum: "); Serial.print(bme.humidity); Serial.print("%  ");
  Serial.print("VOC: "); Serial.print(bme.gas_resistance/1000.0);
  Serial.println(" kΩ");
  delay(1000);
}
