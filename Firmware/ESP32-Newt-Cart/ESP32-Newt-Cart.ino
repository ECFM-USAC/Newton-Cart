#include "HX711.h"
#include "Wire.h"
//#include "includes/gpios.h"

//:::::::::::::::::::: DEFINITIONS ::::::::::::::::::::::::
//LOADCELL
#define DT 18
#define SCK 19
HX711 scale;
float calibration = +227700; // Valor calculado para celda de carga.

//HC-SR04
#define TRIGGER 2
#define ECHO 4

//MPU-6050
#define MPU_SDA 15
#define MPU_SCL 13
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

//OPTICAL ENCODER
#define PULSE 34
#define CHANGES 39 //# de cambios de estado en encoder
#define DIAMETER 70 //Diametro en mm 
long prev_time2 = 0;
long dt2 = 0;
int pulsecount = 0;
float velo;

//RESERVA
#define RESERVA_1 32
#define RESERVA_2 33
#define RESERVA_3 34

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
String IMU(){
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
//----------- OPTICAL ENCODER -----------
float vel(){
  dt2 = millis()-prev_time2;
  if(dt2>1000){
    velo = ((2*PI)/CHANGES)*pulsecount; //Arroja velocidad estimada en m/s
    pulsecount=0;
    prev_time2 = millis();
    }
  return velo;
}
  
void IRAM_ATTR EncoderCounter(){
  pulsecount++;
  }
//:::::::::::::::::: MAIN LOOPS ::::::::::::::::::::
void setup() {
  delay(500);
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
  Wire.begin(15,13); //GPIO15-->SDA, GPIO13-->SCL
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  //OPTICAL ENCODER
  pinMode(PULSE, INPUT);
  attachInterrupt(PULSE, EncoderCounter, CHANGE);
}
//aaaa
void loop() {
  float f = Force();  
  int d = Distance();
  String euler = IMU();
  float v = vel();
  Serial.print("Fuerza :");
  Serial.println(f);
  Serial.print("Distancia :");
  Serial.println(d);
  Serial.println("Euler :");
  Serial.println(euler);
  Serial.println("Velocity:");
  Serial.println(v);
  Serial.println(pulsecount);
  Serial.println("GG FUNCIONO");
  Serial.println();
  
  delay(200);
  

}
