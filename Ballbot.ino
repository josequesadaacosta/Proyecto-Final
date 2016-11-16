#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
 float theta_x;
 int Rw= 0.03;
 int Rb=0.11;
 int Rc=0.11;
 
 
 //Declaracion de los pines
 //Encoders de los motores (mezclados en una sola señal)
 
 int PinEncoderMotor1 = 2;  //interrupt0
int PinEncoderMotor2 = 3;   //interrupt1
int PinEncoderMotor3=18;   //interrupt5....Los pines 20 y 21 estan siendo utilizados para los interrupts del acelerómetro y giroscopio

//Señales de control de PWM para los motores
int Pin_PWM_Motor1=13;   
int Pin_PWM_Motor2=12;
int Pin_PWM_Motor3=11;

//Señales de control para sentido de giro de los motores
int Pin_Dir_Motor1=22;
int Pin_Dir_Motor2=24;
int Pin_Dir_Motor3=26;

//Sensado de corriente para operacion segura (deben ser  menores a 4A)
int Corriente_Motor1=A0;
int Corriente_Motor2=A1;
int Corriente_Motor3=A2;


long timer=0;
long timer_old;


void setup(void) 
{
  
  pinMode(PinEncoderMotor1, INPUT);
  digitalWrite(PinEncoderMotor1, HIGH);
  
   pinMode(PinEncoderMotor2, INPUT);
  digitalWrite(PinEncoderMotor2, HIGH);
  
   pinMode(PinEncoderMotor3, INPUT);
  digitalWrite(PinEncoderMotor3, HIGH);
  
  pinMode(Pin_PWM_Motor1,OUTPUT);
  pinMode(Pin_PWM_Motor2,OUTPUT);
   pinMode(Pin_PWM_Motor3,OUTPUT);
  
  pinMode (Pin_Dir_Motor1,OUTPUT);
  pinMode (Pin_Dir_Motor2,OUTPUT);
  pinMode (Pin_Dir_Motor3,OUTPUT);
  
  pinMode (Corriente_Motor1,INPUT);
  pinMode (Corriente_Motor2,INPUT);
  pinMode (Corriente_Motor3,INPUT);
  
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  //Ganancias del controlador LQR
float K11=-210.0060,  K12=-52.4385, K13=0.0000,   K14=,0.0000  K15= -0.2205,  K16= -0.3190, K17= 1.9596,  K18=  2.2787 ,K19=  -0.0000,K110=-0.0000;
float K21=105.0033,   K22=26.2193,K23=-181.8708,  K24=-45.4131, K25=-0.2205,  K26=-0.3190,   K27=-0.9798,   K28=-1.1394 ,   K29=1.6971 ,  K210= 1.9734;
float K31=105.0033,   K32=26.2193, K33=181.8708,  K34=45.4131 ,  K35=-0.2205,   K36-0.3190 ,  K37=-0.9798 ,  K38=-1.1394,   K39=-1.6971 ,  K310=-1.9734;
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}
 
void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  

//Definicion de estados

float theta_x= event.orientation.z; //Estan al reves en el sensor !
float theta_y= event.orientation.y;
float theta_z= event.orientation.x; //Están al reves en el sensor!

float dtheta_x= gyro.x;
float dtheta_y=gyro.y;
float dtheta_z=gyro.z

float dphi_x= cos(theta_y)*(dtheta_x - dtheta_z*sin(theta_y) + (2^(1/2)*(Rw*dpsi2 - 2*Rw*dpsi1 + Rw*dpsi3))/(3*Rb)) + sin(theta_x)*sin(theta_y)*(dtheta_y*cos(theta_x) + dtheta_z*cos(theta_y)*sin(theta_x) - (2^(1/2)*3^(1/2)*(Rw*dpsi2 - Rw*dpsi3))/(3*Rb)) + cos(theta_x)*sin(theta_y)*((2^(1/2)*(Rw*dpsi1 + Rw*dpsi2 + Rw*dpsi3))/(3*Rb) - dtheta_y*sin(theta_x) + dtheta_z*cos(theta_x)*cos(theta_y))
float dphi_y=  cos(theta_x)*(dtheta_y*cos(theta_x) + dtheta_z*cos(theta_y)*sin(theta_x) - (2^(1/2)*3^(1/2)*(Rw*dpsi2 - Rw*dpsi3))/(3*Rb)) - sin(theta_x)*((2^(1/2)*(Rw*dpsi1 + Rw*dpsi2 + Rw*dpsi3))/(3*Rb) - dtheta_y*sin(theta_x) + dtheta_z*cos(theta_x)*cos(theta_y))
float dphi_z=  cos(theta_x)*cos(theta_y)*((2^(1/2)*(Rw*dpsi1 + Rw*dpsi2 + Rw*dpsi3))/(3*Rb) - dtheta_y*sin(theta_x) + dtheta_z*cos(theta_x)*cos(theta_y)) - sin(theta_y)*(dtheta_x - dtheta_z*sin(theta_y) + (2^(1/2)*(Rw*dpsi2 - 2*Rw*dpsi1 + Rw*dpsi3))/(3*Rb)) + cos(theta_y)*sin(theta_x)*(dtheta_y*cos(theta_x) + dtheta_z*cos(theta_y)*sin(theta_x) - (2^(1/2)*3^(1/2)*(Rw*dpsi2 - Rw*dpsi3))/(3*Rb))

Serial.print("X: ");
Serial.print(gyro.x());
Serial.print(" Y: ");
Serial.print(gyro.y());
Serial.print(" Z: ");
Serial.print(gyro.z());
Serial.println("");
//Serial.print(" Z: ");
//Serial.print(euler.z());
//Serial.println("");
//  /* Display the floating point data */
//  Serial.print("Z: ");
//  Serial.print(event.orientation.x, 4);
//  Serial.print("\tY: ");
//  Serial.print(event.orientation.y, 4);
//  Serial.print("\tX: ");
//  Serial.print(event.orientation.z, 4);
//  Serial.println("");
  
  delay(100);
}
