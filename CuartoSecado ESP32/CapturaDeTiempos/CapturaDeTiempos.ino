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

#define DS18B20PIN1 0
#define DS18B20PIN2 4

float PromTem = 0;


//GPIO pin16 is set as OneWire bus
OneWire ourWire1(DS18B20PIN1);
OneWire ourWire2(DS18B20PIN2);

//A variable or object is declared for our sensor 1
DallasTemperature sensors1(&ourWire1);
DallasTemperature sensors2(&ourWire2);

//set parameters
int period = 15; //medium period in minutes
int freq_sampling = 100; // sampling time
int ciclos = 20; // sampling time

int dutyCycle1 = 0;

// Setting PWM properties
const int freq = 30000;
const int pwmChannell = 0;
const int resolution = 8;

void setup() {
  delay(1000);
  Serial.begin(115200);
  pinMode(16,OUTPUT);
  //digitalWrite(18,1);
  sensors1.begin();   //Sensor 1 starts
  sensors2.begin();
  //transistor 1
 

  // configure LED PWM functionalitites
  ledcSetup(pwmChannell, freq, resolution);
  // attach the channel to the GPIO to be controlled
  Serial.println("Choose any case: ");
}

void loop() {
  readData();
  if(PromTem > 80){
    digitalWrite(16,HIGH);
    Serial.println("LEGOOOOOOOO AL MAXIMOOOOOOOOO");
  }

}


//method to read data for 15 minute
void readData() {

 // uint32_t timer = period * 60000L;

  //for ( uint32_t tStart = millis();  (millis() - tStart) < timer;  ) {
    //The command is sent to read the temperature
    sensors1.requestTemperatures();
    sensors2.requestTemperatures();
    //Obtain the temperature in ºC of sensor 1
    float temp1 = sensors1.getTempCByIndex(0);
    float temp2 = sensors2.getTempCByIndex(0);
    PromTem = (temp1+temp2)/2;

    //print to display the temperature change
    Serial.print(temp1);
    Serial.print(",");
    Serial.print(temp2);
    Serial.print(",");
    Serial.println(PromTem);
    //delay(freq_sampling);
  //}

}
