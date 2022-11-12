#include <WiFi.h>
#include <DallasTemperature.h>
#include <Separador.h>
#include <OneWire.h>
#include <PubSubClient.h>
/*
   ****************************** TSC-Lab *******************************
   ***************************** PRACTICE 36 *****************************
   This practice is about data acquisition with square velocity input
   By: vasanza
   More information: https://tsc-lab.blogspot.com/
*/

//separador library


Separador s;
//GPIO pin 0 is set as OneWire bus
OneWire ourWire1(4);
//GPIO pin 4 is set as OneWire bus
OneWire ourWire2(16);
String temperatura1 = "";
String temperatura2 = "";
String TemperaturaP = "";
float temp1;
float temp2;
float TemPromedio;


// WiFi
const char* ssid = "Claro_ORELLANA0010810810";
const char* password = "MARGOT52521180389789";
WiFiClient espClient;
PubSubClient client(espClient);
String msg;

//Your Domain name with URL path or IP address with path
const char* mqtt_server = "test.mosquitto.org"; 
const char *topic = "ProyectoSCE/TermoCuna/Temp"; //cambiar "your_name" por su nombre o usuario

//A variable or object is declared for our sensor 1
DallasTemperature sensors1(&ourWire1);
//A variable or object is declared for our sensor 2
DallasTemperature sensors2(&ourWire2);

//initial setting for data acquisition
int dutyCycleInitial = 0;
int dutyCycleFinish = 255;
int period = 13000;
int cycles = 21;
int dutyCycle = 0;
int SetPoint = 0;

//motor
int Vent = 17;


// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;

//move
//int encoder = 27;

void motor( void *pvParameters );
void enviar( void *pvParameters );
void Temperatura( void *pvParameters );
void SetPointTem( void *pvParameters );
void dato( void *pvParameters );
//void publicMQTT(void *pvParameters);


  //Temperature of Heater 1
volatile float u = 0.0, u_1 = 0.0; //control action
float Ts = 0.999; //Sampling period (seconds)
//PID parameters----------------------------------
float  kp = 1;
float  ki = 0.0255727088073595;
float  kd = 1;
//--------------------------------------------------
float q0, q1, q2;
volatile float e = 0.0, e_1 = 0.0, e_2 = 0.0;


//-------------------------------------------

// void interruption()    // Function that runs during each interrupt
// {
//   counter=temp1 ;
// }

void setup() {
  Serial.begin(115200);
  sensors1.begin();   //Sensor 1 starts
  sensors2.begin();   //Sensor 2 starts
  // sets the pins as outputs:
  pinMode(Vent, OUTPUT);


  //wifi
  WiFi.mode(WIFI_STA); 

  connect_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  
  //Digital PID
  // Calculo do controle PID digital
  q0 = kp * (1 + Ts / (2.0 * ki) + kd / Ts);
  q1 = -kp * (1 - Ts / (2.0 * ki) + (2.0 * kd) / Ts);
  q2 = (kp * kd) / Ts;

  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(Vent, pwmChannel);
 // attachInterrupt(encoder, interruption, RISING);

 // core 1


  xTaskCreatePinnedToCore(
    motor
    ,  "MotorDC"         // Descriptive name of the function (MAX 8 characters)
    ,  2048              // Size required in STACK memory
    ,  NULL              // INITIAL parameter to receive (void *)
    ,  1                 // Priority, priority = 3 (configMAX_PRIORITIES - 1) is the highest, priority = 0 is the lowest.
    ,  NULL              // Variable that points to the task (optional)
    , 1);                // core 1

 xTaskCreatePinnedToCore(
   Temperatura
   ,  "Temp"              // Descriptive name of the function (MAX 8 characters)
   ,  2048              // Size required in STACK memory
   ,  NULL              // INITIAL parameter to receive (void *)
   ,  1                 // Priority, priority = 3 (configMAX_PRIORITIES - 1) is the highest, priority = 0 is the lowest.
   ,  NULL              // Variable that points to the task (optional)
   , 1);                // core 0

  xTaskCreatePinnedToCore(
    SetPointTem
    ,  "Stp"              // Descriptive name of the function (MAX 8 characters)
    ,  2048              // Size required in STACK memory
    ,  NULL              // INITIAL parameter to receive (void *)
    ,  1                 // Priority, priority = 3 (configMAX_PRIORITIES - 1) is the highest, priority = 0 is the lowest.
    ,  NULL              // Variable that points to the task (optional)
    , 1);                // core 0
  xTaskCreatePinnedToCore(
    dato
    ,  "dato"         // Descriptive name of the function (MAX 8 characters)
    ,  2048              // Size required in STACK memory
    ,  NULL              // INITIAL parameter to receive (void *)
    ,  1                 // Priority, priority = 3 (configMAX_PRIORITIES - 1) is the highest, priority = 0 is the lowest.
    ,  NULL              // Variable that points to the task (optional)
    , 1);
  // xTaskCreatePinnedToCore(
  //   publicMQTT
  //   ,  "MQTT"         // Descriptive name of the function (MAX 8 characters)
  //   ,  2048              // Size required in STACK memory
  //   ,  NULL              // INITIAL parameter to receive (void *)
  //   ,  1                 // Priority, priority = 3 (configMAX_PRIORITIES - 1) is the highest, priority = 0 is the lowest.
  //   ,  NULL              // Variable that points to the task (optional)
  //   , 1);    
}

void loop() {
  //connect_wifi();  
  publicMQTT();
  //delay(1000);
}

void motor( void *pvParameters ) {
  while (1) {    
      digitalWrite(Vent, HIGH);
      //digitalWrite(motor1Pin2, LOW);
      //ledcWrite(pwmChannel, dutyCycle);
      // The calculated action is transformed into PWM 
      ledcWrite(pwmChannel, map(u, 0, 100, 255, 0));
  }
}

// Calcula las RPM quye tiene el motor
void Temperatura( void *pvParameters ) {
  while (1) {
    vTaskDelay(999);
    //Serial.println(TemperaturaP);//255 -> 0x32 0x35 0x35 
    //Serial.write(counter);//0-255
    //-------------------PID-------------------
    e = (SetPoint - TemPromedio);
    // Controle PID
    u = u_1 + q0 * e + q1 * e_1 + q2 * e_2; //Ley del controlador PID discreto

    if (u >= 100.0)        // control saturated action 'uT' at a maximum and minimum limit
    u = 100.0;

    if (u <= 0.0 || SetPoint == 0)
    u = 0.0;

    // Return to real values
    e_2 = e_1;
    e_1 = e;
    u_1 = u;
    //------------------------------------------
    TemPromedio = 0;
  }
}

// Read del SetPoint que viene desde Matlab
void SetPointTem( void *pvParameters ) {
  while (1) {
    //Serial.println("hola");/// EN ESTE APARTADO VA MQTT
    if (Serial.available())
    {
      String string = Serial.readStringUntil('\n');
      SetPoint = string.toInt();
      //Serial.print(SetPoint);
    }
  }
}

void connect_wifi(){
  // Connect or reconnect to WiFi
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, password);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);     
    } 
    Serial.println("\nConnected.");
  }
}

void dato(void *pvParameters) {

  while (1) {
  
  
  //The command is sent to read the temperature
    sensors1.requestTemperatures();
    sensors2.requestTemperatures();
    //Obtain the temperature in ºC of sensor 1
    float temp1 = sensors1.getTempCByIndex(0);
    float temp2 = sensors2.getTempCByIndex(0);
    TemPromedio = (temp1+temp2)/2;
    temperatura1 = String(temp1);
    temperatura2 = String(temp2);
    TemperaturaP = String(TemPromedio);
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("TSCLABClient")) {
      Serial.println("connected");
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  
}

void publicMQTT() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  
  msg = temperatura1+","+temperatura2+","+TemperaturaP+","+SetPoint;

  client.publish(topic, msg.c_str());
  
}
