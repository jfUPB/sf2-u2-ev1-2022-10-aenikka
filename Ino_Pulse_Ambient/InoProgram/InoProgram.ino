//Librerias pulso
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

//Librerias ambient
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//Objeto pulso
MAX30105 particleSensor;

//Definicion ambiente
#include <SPI.h>
#define BME_SCK 18
#define BME_MISO 19
#define BME_MOSI 23
#define BME_CS 5
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme(BME_CS); // hardware SPI

//Valores pulso
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

//Valores ambiente
unsigned long delayTime;

void setup()
{
  Serial.begin(115200);
  //Pulso
  //Serial.println("Initializing...");
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("Conecta bien la alimentacion JoJo");
    //Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  //Serial.println("Place your index finger on the sensor with steady pressure.");
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
  //Ambiente
  bool status;

  status = bme.begin();
  delayTime = 1000;
}

void loop()
{
  long irValue = particleSensor.getIR();
  uint8_t dataFrame[1];

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  //Serial.print("IR=");
  //Serial.print(irValue);
  //Serial.print(", BPM=");
  //Serial.println(beatsPerMinute);
  //Serial.print(", Avg BPM=");
  if (Serial.available() > 0)
  {
    dataFrame[0] = Serial.read();
    if (dataFrame[0] == 0x01)
    {
      Serial.write((int)beatAvg);
      Serial.write((int)bme.readTemperature());
      Serial.write((int)bme.readHumidity());
    }
  }

}



//if (irValue < 50000)
//Serial.print(" No dedo");
//Serial.print(" No finger?");

//Serial.println();
