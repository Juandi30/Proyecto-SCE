/*
   ****************************** TSC-Lab *******************************
   ***************************** PRACTICE 3 *****************************
   This practice is about USB DATA ACQUISITION and have 4 different cases:
    • Case 1: Ambient temperature reading using sensor 1 and 2
    • Case 2: Activation of Transistor 1 and Reading of temperature sensor 1 and 2
    
   By: Kevin E. Chica O
   More information: https://tsc-lab.blogspot.com/
*/

#include <OneWire.h>
#include <DallasTemperature.h>

#define T1 0
#define T2 4
#define vent 16

//GPIO pin16 is set as OneWire bus
OneWire ourWire1(T1);
OneWire ourWire2(T2);

//A variable or object is declared for our sensor 1
DallasTemperature sensors1(&ourWire1);
DallasTemperature sensors2(&ourWire2);
//set parameters
int period1 = 3; //medium period in minutes
int period2 = 5;
int freq_sampling = 100; // sampling time
int ciclos = 21; // sampling time

int dutyCycle1 = 0;

// Setting PWM properties
const int freq = 30000;
const int pwmChannell = 0;
const int resolution = 8;

void setup() {
  delay(1000);
  Serial.begin(115200);
  
  sensors1.begin();   //Sensor 1 starts
  sensors2.begin();

  //vENTILADOR 1
  pinMode(vent, OUTPUT);

  // configure LED PWM functionalitites
  ledcSetup(pwmChannell, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(vent, pwmChannell);
  Serial.println("Choose any case: ");
}

void loop() {

  if (Serial.available())
  {
    String string = Serial.readStringUntil('\n');

    if (string == "case_2") {
      Serial.println("Case 2 started");

      for (int i = 1; i <= ciclos; i++) {
        //transistor 0 desactivado
        dutyCycle1 = 0;
        ledcWrite(pwmChannell, dutyCycle1);
        readData1();
        //transistor 1 activate
        dutyCycle1 = 255;
        ledcWrite(pwmChannell, dutyCycle1);
        readData2();
      }
      Serial.println("Case 2 finished");
      Serial.println("Choose any case: ");
    }
    
    
  }

}


//method to read data for 15 minute
void readData1() {

  uint32_t timer = period1 * 60000L;

  for ( uint32_t tStart = millis();  (millis() - tStart) < timer;  ) {
    //The command is sent to read the temperature
    sensors1.requestTemperatures();
    sensors2.requestTemperatures();
    //Obtain the temperature in ºC of sensor 1
    float temp1 = sensors1.getTempCByIndex(0);
    float temp2 = sensors2.getTempCByIndex(0);
    float PromTem = (temp1+temp2)/2;

    //print to display the temperature change
    Serial.print(PromTem);
    Serial.print(",");
    Serial.print(dutyCycle1);
    Serial.println("\n");
    delay(freq_sampling);
  }



}



void readData2() {

  uint32_t timer = period2 * 60000L;

  for ( uint32_t tStart = millis();  (millis() - tStart) < timer;  ) {
    //The command is sent to read the temperature
    sensors1.requestTemperatures();
    sensors2.requestTemperatures();
    //Obtain the temperature in ºC of sensor 1
    float temp1 = sensors1.getTempCByIndex(0);
    float temp2 = sensors2.getTempCByIndex(0);
    float PromTem = (temp1+temp2)/2;

    //print to display the temperature change
    Serial.print(PromTem);
    Serial.print(",");
    Serial.print(dutyCycle1);
    Serial.println("\n");
    delay(freq_sampling);
  }
}
