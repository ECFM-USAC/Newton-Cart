/*
Universidad de San Carlos de Guatemala, Facultad de Ingenieria 
Escuela de Ingeniería Mecanica Electrica, Peter Alexander Salán Aparicio
Programación de plataforma "Newton-Cart" 
Microcontrolador: Chip ESP-WROOM-32, montado en PCB NewtonCart v1.0
Sensores y modulos: HC-SR04; TAL220 con XFW-HX011; HC-020K, ; GY-571, BH1750; TP4056.
Alimentacion: 5V y 3.3V 
*/
//Librerías
#include "HX711.h"
#include "Wire.h"
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLEAdvertisedDevice.h>
#include <BLEScan.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

//:::::::::::::::::::: DEFINICIONES ::::::::::::::::::::::::
float f;
float d;
float v;
float MPUValues[6];
float accx, accy, accz, pitch, corr_pitch, roll, corr_roll, yaw, corr_yaw;
float distance;
float distance_a;
String valores;
long tempo;
int16_t sampling_freq = 200;

//----------- I/O's RESERVADOS EN MICROCONTROLADOR Y PCB --------------
#define RESERVA_1 32
#define RESERVA_2 33
#define RESERVA_3 34

//----------- DEFINICIÓN DE TAREA RTOS --------------
TaskHandle_t Sens, Comm;
SemaphoreHandle_t syncro;

//----------- Definición BLE --------------
BLECharacteristic* BLEDistance;
BLECharacteristic* BLEForce;
BLECharacteristic* BLEVelocity;
BLECharacteristic* BLEAcceleration;
BLECharacteristic* BLEPitch;
bool deviceConnected = false;  //Compara si conexión activa.
float distanceValue = 0;
float forceValue = 0;

#define SERVICE_UUID            "00001819-0000-1000-8000-00805F9B34FB" //Location and navigation service 1819 (Utilizado por similitud).
#define CHARACTERISTIC_DISTANCE_UUID  "00002713-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_FORCE_UUID  "00002714-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_VELOCITY_UUID  "00002715-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_ACCEL_UUID  "00002716-0000-1000-8000-00805F9B34FB"
#define CHARACTERISTIC_PITCH_UUID  "00002717-0000-1000-8000-00805F9B34FB"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer){
    deviceConnected = true;
    };

  void onDisconnect(BLEServer* pServer){
    deviceConnected = false;
    }                                     //Retorna un valor de falso o verdadero en "deviceConnected" 
};

//----------- Definiciones SENSORES --------------
//LOADCELL
#define DT 18
#define SCK 19
HX711 scale;
float calibration = 256500/1000; // Valor de calibración calculado para la celda específica en gramos. 227700 previo calibración de prueba de peso.

//HC-SR04
#define TRIGGER 2
#define ECHO 4

//MPU-6050
#define MPU_SDA 15
#define MPU_SCL 13
#define MPU 0x68
#define A_R 16384.0 // Obtener valor de aceleración en términos de 1G, 32768/2
#define G_R 131.0 // 32768/250
#define RadtoDeg = 57.295779 //Valor de conversión para MPU.
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float Acc[3];
float Gy[3];
float Angle[3];
float values[6];
long prev_time;
float dt;

//ENCODER OPTOELECTRÓNICO
#define PULSE 34
#define CHANGES 40 //# Cambios de estado en el encoder óptico.
#define DIAMETER 64 // Diámetro del neumático colocado, en mm.
long prev_time2 = 0;
long dt2 = 0;
int pulsecount = 0;
float velo;



//::::::::::::::::::::: FUNCIONES ::::::::::::::::::::::::::
//----------- BLE --------------
void ConfigBLE(){
  BLEDevice::init("ESP32PET"); // Se crea un servicio de BLE.
  
  BLEServer *pServer = BLEDevice::createServer(); 
  pServer->setCallbacks(new MyServerCallbacks()); //Se genera el servidor de BLE.

  BLEService *pService = pServer->createService(SERVICE_UUID); //Se crea el servicio BLE

  BLEDistance = pService->createCharacteristic(            
                      CHARACTERISTIC_DISTANCE_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY   //Se define la característica Distancia.
                      );
  
  BLEForce = pService->createCharacteristic(
                      CHARACTERISTIC_FORCE_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY   //Se define la característica Fuerza.
                      );

  BLEVelocity = pService->createCharacteristic(
                      CHARACTERISTIC_VELOCITY_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY   //Se define la característica Velocidad.
                      );

  BLEAcceleration = pService->createCharacteristic(
                      CHARACTERISTIC_ACCEL_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY   //Create a Force Characteristic
                      );

  BLEPitch = pService->createCharacteristic(
                      CHARACTERISTIC_PITCH_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY   //Se define la característica Pitch
                      );
                                           
  BLEDistance->addDescriptor(new BLE2902());
  BLEForce->addDescriptor(new BLE2902());
  BLEVelocity->addDescriptor(new BLE2902());
  BLEAcceleration->addDescriptor(new BLE2902());
  BLEPitch->addDescriptor(new BLE2902());    //Notificaciones parte de  BLE2902
  
  pService->start();   //Se da inicio al servicio descrito 

  pServer->getAdvertising()->start();  //Se comienza a notificar.
  Serial.println("Waiting for a client connection to notify...");  
  }

//----------- LOADCELL READ --------------
float GetForce(){
  //scale.set_scale(calibration);
  float force = (scale.get_units());
  return force;
}

//------------ HC-SR04 READ ------------
float GetDistance(){
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  tempo = pulseIn(ECHO, HIGH); //Tempo toma la duración hacia Echo pin, con timeout de 60s.
  distance_a = distance;
  if(abs(distance - distance_a) >= (1.0*sampling_freq))
    distance = distance_a;
  else distance = tempo*0.345/2; // Distancia equivale a velocidad por tiempo /2
  
  return distance;
}
//----------- IMU MPU6050 READ --------------
float GetIMU(){
  //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
   AcX=(Wire.read()<<8|Wire.read()); //Cada valor ocupa 2 registros
   AcY=(Wire.read()<<8|Wire.read()) - 24000;
   AcZ=(Wire.read()<<8|Wire.read()) - 4200;
 
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
   //String valuesString = "90, " +String(Angle[0]) + "," + String(Angle[1]) + "," + String(Angle[2]) + ", -90";
   //Serial.println(valuesString);
   //Serial.println(dt);
   MPUValues[0]= AcX/A_R;
   MPUValues[1]= AcY/A_R;
   MPUValues[2]= AcZ/A_R; //Valores de aceleración obtenidos por IMU
   MPUValues[3]= Angle[0] - (corr_roll); //ROLL
   MPUValues[4]= Angle[1] - (corr_pitch); //PITCH
   MPUValues[5]= Angle[2] - (corr_yaw); //YAW
   //MPUValues[2]= dt;
   
   return 0;
}   
//----------- OPTICAL ENCODER READ -----------
float GetVelocity(){
  dt2 = millis()-prev_time2;
  if(dt2>=sampling_freq){
    velo = ((2*PI)*(pulsecount/40.0) * DIAMETER/2) * (1000/sampling_freq) ; //Arroja velocidad estimada en m/s
    //Serial.print("Velocity: ");
    //Serial.print(velo);
    //Serial.print(" --pulsos: ");
    //Serial.println(pulsecount);
    pulsecount=0;
    prev_time2 = millis();
    }

  return velo;
}
//----------- INTERRUPCIONES -----------  
void IRAM_ATTR EncoderCounter(){
  pulsecount++;
  }

//-----------********* DEFINICIÓN RTOS **********--------------
void Sensing(void* parameters){
  for(;;){
      //Serial.println("Entering Sensing");
      xSemaphoreTake(syncro, portMAX_DELAY);
      f = GetForce();  
      d = GetDistance();
      v = GetVelocity();
      GetIMU();
      accx = MPUValues[0];
      //accy = MPUValues[1];
      //accy = MPUValues[2];
      //roll = MPUValues[3];
      pitch = MPUValues[4];
      //yaw = MPUValues[5];
      Serial.print("Fuerza: ");
      Serial.println(f);
      //Serial.print("Distancia: ");
      //Serial.println(d);
      //Serial.println("Euler :");

      /*Serial.print(90.0);
      Serial.print(" ");
      Serial.print(accx);
      Serial.print(" ");
      Serial.print(accy);
      Serial.print(" ");
      Serial.print(accz);
      Serial.print(" ");
      Serial.print(-90.0);
      Serial.println();*/

      valores = "90, " + String(roll) + "," + String(pitch) + "," + String(yaw) + ", -90";
      //Serial.println(valores);
      
      //delay(500);
      //Serial.println("Sensing completed");
      xSemaphoreGive(syncro); 
      delay(sampling_freq);   
    } 
  }

void Communicate(void* parameters){
  for(;;){
      xSemaphoreTake(syncro, portMAX_DELAY);
      if (deviceConnected){
        //Conversion of txValue
        char distanceString[8];
        char forceString[8];
        char velocityString[8];
        char accelString[8];
        char pitchString[8];
        dtostrf(d,1,2,distanceString);
        dtostrf(f,1,2,forceString);
        dtostrf(v,1,2,velocityString);
        dtostrf(accx,1,2,accelString);
        dtostrf(pitch,1,2,pitchString);
        
        //Se asigna un valor a la característica
        BLEDistance->setValue(distanceString);
        BLEForce->setValue(forceString);
        BLEVelocity->setValue(velocityString);
        BLEAcceleration->setValue(pitchString);
        BLEPitch->setValue(accelString);
        
        //Se notifica sobre la conexión
        BLEDistance->notify();
        BLEForce->notify();
        BLEVelocity->notify();
        BLEAcceleration->notify();
        BLEPitch->notify();
        //Serial.println("Sent distance: " + String(distanceString));
        Serial.println("Sent force: " + String(forceString));
        //Serial.println("Sent velocity: " + String(velocityString));
        //Serial.println("Sent acceleration: " + String(accelString));;
        //Serial.println("Sent pitch angle: " + String(pitchString));
        Serial.println("enviado");
      }
      xSemaphoreGive(syncro);
      delay(sampling_freq);
    }
  }

  
//:::::::::::::::::: MAIN ::::::::::::::::::::
void setup() {
  //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //Deshabilitar brownout detector
  delay(3000);
  Serial.begin(115200);
  //----------- LOADCELL -----------
  scale.begin(DT,SCK);
  scale.set_scale(calibration);
  scale.tare(); //Se da reset a la báscula
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
  GetIMU(); // Llamado a la rutina de lectura de MPU para correccion de ceros iniciales.
  corr_roll = MPUValues[3];
  corr_pitch = MPUValues[4];
  corr_yaw = MPUValues[5];
  
  //----------- OPTICAL ENCODER -----------
  pinMode(PULSE, INPUT);
  attachInterrupt(PULSE, EncoderCounter, CHANGE);
  //----------- COMM CONFIG -----------
  ConfigBLE();
  delay(1000);
  //----------- SETUP TAREAS RTOS -----------
  syncro = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
    Sensing,            //Llamada a la función
    "Sensing Task",
    8000,
    NULL,
    1,
    &Sens,      //Como se define la tarea en función TaskHandler_t
    1);         //Selección core
  delay(500);
  xTaskCreatePinnedToCore(
    Communicate,            //Llamada a función
    "Communication Task",
    8000,
    NULL,
    2,
    &Comm,      //Como se define la tarea en función TaskHandler_t
    0);         //Selección core
}

void loop() {
//No se ejecuta.
}
