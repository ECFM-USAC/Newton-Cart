#include "HX711.h"
#include "Wire.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEAdvertisedDevice.h>
#include <BLEScan.h>

//:::::::::::::::::::: DEFINITIONS ::::::::::::::::::::::::
float f;
int d;
//----------- RESERVE IO'S IN PCB --------------
#define RESERVA_1 32
#define RESERVA_2 33
#define RESERVA_3 34

//----------- RTOS TASK DEFINITION --------------
TaskHandle_t Sens;
TaskHandle_t Comm;

SemaphoreHandle_t syncro;

//----------- BLE --------------
BLECharacteristic* BLEDistance;
BLECharacteristic* BLEForce;
bool deviceConnected = false;  //Connection active or not.
float distanceValue = 0;
float forceValue = 0;

#define SERVICE_UUID            "0000180D-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_DISTANCE_UUID  "00002713-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_FORCE_UUID  "00002714-0000-1000-8000-00805F9B34FB"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer){
    deviceConnected = true;
    };

  void onDisconnect(BLEServer* pServer){
    deviceConnected = false;
    }                                     //Returns a true or false in "deviceConnected" variable
};

//----------- SENSORS --------------
//LOADCELL
#define DT 18
#define SCK 19
HX711 scale;
float calibration = +227700; // Calibration value calculated for specific used loadcell.

//HC-SR04
#define TRIGGER 2
#define ECHO 4

//MPU-6050
#define MPU_SDA 15
#define MPU_SCL 13
#define MPU 0x68
#define A_R 16384.0 // To obtain acceleration in terms of 1G, 32768/2
#define G_R 131.0 // 32768/250
#define RadtoDeg = 57.295779 //MPU Conversion values
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float Acc[3];
float Gy[3];
float Angle[3];
String values;
long prev_time;
float dt;

//OPTICAL ENCODER
#define PULSE 34
#define CHANGES 39 //Qty of changes in the used encoder.
#define DIAMETER 70 //Diameter of wheel in wich is located the encoder, in mm.
long prev_time2 = 0;
long dt2 = 0;
int pulsecount = 0;
float velo;



//::::::::::::::::::::: FUNCTIONS ::::::::::::::::::::::::::
//----------- BLE --------------
void ConfigBLE(){
  BLEDevice::init("ESP32PET"); //Create the BLE Device

  BLEServer *pServer = BLEDevice::createServer(); 
  pServer->setCallbacks(new MyServerCallbacks()); //Creating the BLE Server

  BLEService *pService = pServer->createService(SERVICE_UUID); //Create the BLE Service

  BLEDistance = pService->createCharacteristic(            
                      CHARACTERISTIC_DISTANCE_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY   //Create a Distance Characteristic
                      );
  
  BLEForce = pService->createCharacteristic(
                      CHARACTERISTIC_FORCE_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY   //Create a Force Characteristic
                      );
                                           
  BLEDistance->addDescriptor(new BLE2902());
  BLEForce->addDescriptor(new BLE2902());    //BLE2902 needed to notify
  
  pService->start();   //Start the service

  pServer->getAdvertising()->start();  //Start advertising
  Serial.println("Waiting for a client connection to notify...");  
  }

//----------- LOADCELL READ --------------
float GetForce(){
  scale.set_scale(calibration);
  float force = (scale.get_units());
  return force;
}

//------------ HC-SR04 READ ------------
int GetDistance(){
  long tempo, distance;
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds (4);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);

  tempo = pulseIn(ECHO, HIGH, 60); //tempo takes the duration (in ms) of the echo pin, timeout 60ms
  distance = tempo*0.345/2; // distance equal to sound velocity * time /2
  return distance;
}
//----------- IMU READ --------------
String GetIMU(){
  //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
 
   //A partir de los valores del acelerometro, se calculan los angulos Y, X
   //respectivamente, con la formula de la tangente.
   Acc[1] = atan(-1*(AcX/A_R)/sqrt(pow((AcY/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY/A_R)/sqrt(pow((AcX/A_R),2) + pow((AcZ/A_R),2)))*RAD_TO_DEG;
 
   //Leer los valores del Giroscopio
   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x43, se piden 6 registros
   GyX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   GyY=Wire.read()<<8|Wire.read();
   GyZ=Wire.read()<<8|Wire.read();
 
   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
   Gy[2] = GyZ/G_R;

   dt = (millis() - prev_time) / 1000.0;
   prev_time = millis();
 
   //Aplicar el Filtro Complementario
   Angle[0] = 0.98 *(Angle[0]+Gy[0]*dt) + 0.02*Acc[0];
   Angle[1] = 0.98 *(Angle[1]+Gy[1]*dt) + 0.02*Acc[1];

   //Integración respecto del tiempo paras calcular el YAW
   Angle[2] = Angle[2]+Gy[2]*dt;
 
   //Mostrar los valores por consola
   values = "90, " +String(Angle[0]) + "," + String(Angle[1]) + "," + String(Angle[2]) + ", -90";
   return values;
}   
//----------- OPTICAL ENCODER READ -----------
float GetVelocity(){
  dt2 = millis()-prev_time2;
  if(dt2>1000){
    velo = ((2*PI)/CHANGES)*pulsecount; //Arroja velocidad estimada en m/s
    pulsecount=0;
    prev_time2 = millis();
    }
  return velo;
}
//----------- INTERRUPTIONS -----------  
void IRAM_ATTR EncoderCounter(){
  pulsecount++;
  }

//-----------********* RTOS **********--------------
void Sensing(void* parameters){
  for(;;){
      //xSemaphoreTake(syncro, portMAX_DELAY);
      f = GetForce();  
      d = GetDistance();
      //String euler = GetIMU();
      //float v = GetVelocity();
      Serial.print("Fuerza :");
      Serial.println(f);
      Serial.print("Distancia :");
      Serial.println(d);
      //Serial.println("Euler :");
      //Serial.println(euler);
      //Serial.println("Velocity:");
      //Serial.println(v);
      //Serial.println(pulsecount);
      Serial.println("Running at core -> ");
      Serial.println(xPortGetCoreID());
      delay(1000);
      //xSemaphoreGive(syncro); 
      delay(50);   
    } 
  }

void Communicate(void* parameters){
  for(;;){
      if (deviceConnected){
        //Conversion of txValue
        char distanceString[8];
        char forceString[8];
        dtostrf(d,1,2,distanceString);
        dtostrf(f,1,2,forceString);
        
        //Setting the value to the characteristic
        BLEDistance->setValue(distanceString);
        BLEForce->setValue(forceString);
        
        //Notifying the connected client
        BLEDistance->notify();
        BLEForce->notify();
        Serial.println("Sent value: " + String(distanceString));
        Serial.println("Sent value2: " + String(forceString));
        Serial.println();
      }
     delay(500);
    }
  }

  
//:::::::::::::::::: MAIN LOOPS ::::::::::::::::::::
void setup() {
  delay(3000);
  Serial.begin(115200);
  //----------- LOADCELL -----------
  scale.begin(DT,SCK);
  scale.set_scale();
  scale.tare(); //Reset scale to 0.
  long zero_factor = scale.read_average();
  Serial.print("Zero factor: ");
  Serial.println(zero_factor);
  //----------- HC-SR04 -----------
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  //----------- MPU6050 -----------
  Wire.begin(15,13); //GPIO15-->SDA, GPIO13-->SCL
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  //----------- OPTICAL ENCODER -----------
  pinMode(PULSE, INPUT);
  attachInterrupt(PULSE, EncoderCounter, CHANGE);
  //----------- COMMUNICATION CONFIG -----------
  ConfigBLE();
  
  //----------- RTOS TASK SETUP -----------
  xTaskCreatePinnedToCore(
    Sensing,            //Call to function.
    "Communication Task",
    6000,
    NULL,
    1,
    &Sens,      //Task definition in TaskHandler_t function
    0);         //Core

  xTaskCreatePinnedToCore(
    Communicate,            //Call to function.
    "Communication Task",
    6000,
    NULL,
    1,
    &Comm,      //Task definition in TaskHandler_t function
    1);         //Core
}

void loop() {
}
