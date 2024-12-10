#define GAIN_PIN D1 // Pin connected to the GAIN pin of MAX9814
#include <math.h>

const float referenceVoltage = 3.3;   // ESP8266 ADC reference voltage (3.3V)
const float referenceSPL = 45.0;     // SPL of calibration sound source in dB
const int referenceADC = 512;        // ADC value recorded at reference SPL
const int numReadings = 10;          // Number of samples for averaging

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <BH1750.h>     

#include "DHT.h"

#define DHTPIN 4     // what digital pin we're connected to

#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321


// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.

DHT dht(DHTPIN, DHTTYPE);

#define DETECT_PIN D0

String LoRaMessage = "";

void setup() {
 
  Serial.begin(115200);
  
  pinMode(A0, INPUT);

  dht.begin();

  pinMode(D0, INPUT); 

}

void loop() {

  //read UV

  int sensor_value = analogRead(A0); 
  float volts = sensor_value * 5.0 / 1024.0;
  float UV_index = volts * 10;
  Serial.print ("Raw ADC data: ");
  Serial.print (sensor_value);
  Serial.print ("  UV Index: ");
  Serial.println (UV_index);

  
  //read if there is obstacle
  int val = digitalRead(D0);
  Serial.println(val);

  // Read the ADC value in real-time
  int adcValue = analogRead(A0);

  // Convert ADC value to voltage
  float Vout = (adcValue * referenceVoltage) / 1023.0;

  // Calculate SPL in dB
  float Vref = (referenceADC * referenceVoltage) / 1023.0;
  float SPL = 20 * log10(Vout / Vref) + referenceSPL;

  // Send data to the Serial Plotter
  // Serial.print(adcValue);     // First value (ADC)
  // Serial.print("\t");         // Tab separates the next value
  // Serial.print(Vout, 3);      // Second value (Voltage)
  // Serial.print("\t");         // Tab separates the next value
  Serial.println(SPL, 2);     // Third value (SPL), ends line for plotting

  
  Serial.print("Temperature: ");
  
  Serial.print(getTemp("c"));
  
  Serial.print(" *C ");
   
   
  Serial.print("Humidity: ");
  Serial.print(getTemp("h"));
  Serial.println(" % ");
  Serial.println("===========================");

  //  LoRaMessage = String(getTemp("f")) + "&" + String(getTemp("hic"))
  //               + "#" + String(getTemp("hic")) + "@" + String(getTemp("h"));
  // LoRa.beginPacket();
  // LoRa.print(LoRaMessage);
  // LoRa.endPacket();
  delay(5000);

}

float getTemp(String req)
{

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  
  
 

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return 0;
  }
  if(req =="c"){
    return t;//return Celsus
  }else if(req =="h"){
    return h;// return humidity
  }else{
    return 0.000;// if there is nothing,  0.000
  }
 
}
