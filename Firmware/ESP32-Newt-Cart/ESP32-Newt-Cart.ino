#include "HX711.h"
#include "Wire.h"
//#include "includes/gpios.h"

//:::::::::::::::::::: DEFINITIONS ::::::::::::::::::::::::
//LOADCELL
#define DT 19
#define SCK 21
HX711 scale;
float calibration = +227700; // Valor calculado para celda de carga.

//HC-SR04
#define TRIGGER 13
#define ECHO 12

//MPU-6050
#define MPU 0x68
#define A_R 16384.0 // Para obtener en función de 1G, 32768/2
#define G_R 131.0 // 32768/250
#define RadtoDeg = 57.295779 //MPU values
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float Acc[3];
float Gy[3];
float Angle[3];
String values;
long prev_time;
float dt;


//::::::::::::::::::::: FUNCTIONS ::::::::::::::::::::::::::
//----------- LOADCELL --------------
float Force(){
  scale.set_scale(calibration);
  float force = (scale.get_units());
  return force;
}

//------------ HC-SR04 ------------
int Distance(){
  long tempo, distance;
  digitalWrite(TRIGGER, LOW);
  delayMicroseconds (4);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);

  tempo = pulseIn(ECHO, HIGH);
  distance = (tempo*10) /(292*2);
  return distance;
}
//----------- IMU --------------
//void Accxyz(){
  //}
//void RPY(){
  //}
//----------- ENCODER -----------


//:::::::::::::::::: MAIN LOOPS ::::::::::::::::::::
void setup() {
  Serial.begin(115200);
  //LOADCELL
  scale.begin(DT,SCK);
  scale.set_scale();
  scale.tare(); //Reset scale to 0.
  long zero_factor = scale.read_average();
  Serial.print("Zero factor: ");
  Serial.println(zero_factor);
  //HC-SR04
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  //MPU-6050
  Wire.begin(22,23); //GPIO22-->SDA, GPIO23-->SCL
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop() {
  float f = Force();
  int d = Distance();
  Serial.print("Fuerza :");
  Serial.println(f);
  Serial.print("Distancia :");
  Serial.println(d);
  Serial.println();
  delay(500);
  

}
