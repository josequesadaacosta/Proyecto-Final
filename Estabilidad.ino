#include <Dagu4Motor.h>
#include <Encoder.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);



#define fullturnTicks 6533 // Numero de ticks por revolucion incuido el gearbox
#define DegToRad(x) ((x)*0.01745329252) //Conversion grados a radianes
#define RadToDeg(x) ((x)*57.2957795131) //Conversion radianes a grados 
 //2pi/6533
//inicializacion de variables
float psi_1=0.0;
float psi_2=0.0;
float psi_3=0.0;

float dpsi_1=0.0;
float dpsi_2=0.0;
float dpsi_3=0.0;

float phi_x=0.0;
float phi_y=0.0;
float phi_z=0.0;
//
//float theta_x=0;
//float theta_y=0;
//float theta_z=0;

float dphi_x=0.0;
float dphi_y=0.0;
float dphi_z=0.0;

double raiz2= sqrt(2);
double raiz3=sqrt(3);
 double Rw= 0.03; //radio de las ruedas
 double Rb=0.11; // radio de la bola
 double Rc=0.11; // radio del cuerpo
 float Rmotor= 2.4; //Ohm Resistencia interna del motor
 float KMotor= 0.3306;     //Nm/A Constante de flujo del motor
 int pin1;
 int pin2;
 int pin3;
 
 int CURRENT_LIMIT = (1024 / 5) * 3.75; 
 double corrienteReal_1;
 double corrienteReal_2;
 double corrienteReal_3;
 
 float phi_xdeseado=0.000;
 float phi_ydeseado=0.000;
 
 
 //float  theta_z=0;
 
  double T1=3.0;//inicializar valores de los torques
  double T2=0.0;
  double T3=0.0;
  int PWM1=0;
  int PWM2=0;
  int PWM3=0;
  
 
 
 
 
 //Declaracion de los pines
 //Encoders de los motores (mezclados en una sola señal)
 
const  int PinAEncoderMotor1 = 2;  //interrupt0
const int PinAEncoderMotor2 = 3;   //interrupt1
const int PinAEncoderMotor3=18;   //interrupt5....Los pines 20 y 21 estan siendo utilizados para los interrupts del acelerómetro y giroscopio

const int PinBEncoderMotor1= 28;
const int PinBEncoderMotor2= 29;
const int PinBEncoderMotor3= 30;

//Señales de control de PWM para los motores
const int Pin_PWM_Motor1=13;   
const int Pin_PWM_Motor2=12;
const int Pin_PWM_Motor3=11;

//Señales de control para sentido de giro de los motores
const int Pin_Dir_Motor1=22;
const int Pin_Dir_Motor2=24;
const  int Pin_Dir_Motor3=26;

//Sensado de corriente para operacion segura (deben ser  menores a 4A)
const int Corriente_Motor1=A0;
const int Corriente_Motor2=A1;
const int Corriente_Motor3=A2;


// Valores a leer de interrupts deben ser volatiles
volatile int encoder1Pos=0;
volatile int encoder2Pos=0;
volatile int encoder3Pos=0;

//Uso de libreria del controlador de los motores Dagu
 

Dagu4Motor motor1(Pin_PWM_Motor1, Pin_Dir_Motor1, Corriente_Motor1,PinAEncoderMotor1,PinBEncoderMotor1); 
Dagu4Motor motor2(Pin_PWM_Motor2, Pin_Dir_Motor2, Corriente_Motor2,PinAEncoderMotor2,PinBEncoderMotor2); 
Dagu4Motor motor3(Pin_PWM_Motor3, Pin_Dir_Motor3, Corriente_Motor3,PinAEncoderMotor3,PinBEncoderMotor3); 

long timer=0; //timer de proposito general
long timer_old;


void setup() 
{
  
  pinMode(PinAEncoderMotor1, INPUT);
  digitalWrite(PinAEncoderMotor1, HIGH);
  
   pinMode(PinAEncoderMotor2, INPUT);
  digitalWrite(PinAEncoderMotor2, HIGH);
  
   pinMode(PinAEncoderMotor3, INPUT);
  digitalWrite(PinAEncoderMotor3, HIGH);
  
  pinMode(PinBEncoderMotor1, INPUT);
  digitalWrite(PinBEncoderMotor1, HIGH);
  
  pinMode(PinBEncoderMotor2, INPUT);
  digitalWrite(PinBEncoderMotor2, HIGH);
  
  pinMode(PinBEncoderMotor3, INPUT);
  digitalWrite(PinBEncoderMotor1, HIGH);
  
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
  

/* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
   Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(100);
    
  bno.setExtCrystalUse(true);
  attachInterrupt(0,doEncoder1,CHANGE); // Manejo Interrupcion encoder 1
  attachInterrupt(1,doEncoder2,CHANGE); //  Manejo Interrupcion encoder 2
  attachInterrupt(5,doEncoder3,CHANGE);// Manejo Interrupcion encoder 3
  timer=millis();
}
//Ganancias del controlador LQR #2
//float K_11=-210.0060,  K_12=-52.4385, K_13=0.0000, K_14=0.0000;
//float K_15= -0.2205,  K_16= -0.3190, K_17= 1.9596, K_18=  2.2787;
//float K_19= -0.0000,K_110=-0.0000;
//float K21=104.95,   K22=26.2193,K23=-181.18708,  K24=-45.4131;
//float K25=-0.2205,  K26=-0.3190,   K27=-0.9798,   K28=-1.1394 ;
//float K29=1.6971 , K210= 1.9734;
//float K31=104.95, K32=26.2193, K33=181.8708,  K34=45.4131 ;
//float K35= -0.2205,  K36= -0.3190 , K37=-0.9798 , K38=-1.1394;
//float K39=-1.6971 ,  K310=-1.9734;

////Ganancias del controlador LQR #1
float K_11=14.0572,  K_12=0*-2.78, K_13=0.0000, K_14=0.0000;
float K_15= 0*-0.3651,  K_16=0* -0.4565, K_17=0.1*0.0, K_18= 0*0.5589;
float K_19= -0.0000,K_110=-0.0000;

float K21=-7.5287,   K22=0*10.3902,K23=-9.7957,  K24=0*-17.9963;
float K25=0*-0.3651,  K26=0*-0.4565,   K27=-0.1826*0.0,   K28=0*-0.2794 ;
float K29=0.3162*0.0 , K210= 0*0.4840;

float K31=-7.5287,   K32=0*10.3902,K33=9.7957,  K34=0*17.9963;
float K35=0*-0.3651,  K36=0*-0.4565,   K37=-0.1826*0.0,   K38=0*-0.2794 ;
float K39=-0.3162*0.0 , K310= 0*-0.4840;



//Ecuaciones del controladorLQR

float GetLQRTorque1 (float theta_x,float dtheta_x,float theta_y,float dtheta_y,float theta_z,float dtheta_z, float phi_x,float dphi_x, float phi_y, float dphi_y ,int phi_xdeseado,int phi_ydeseado){

  float GetT1=  -1*(K_11*2.0*theta_x +K_12*dtheta_x+K_13*1.5*theta_y  +K_14*dtheta_y+K_15*theta_z +K_16*dtheta_z-K_17*(phi_xdeseado-phi_x)+K_18*dphi_x - K_19*(phi_ydeseado-phi_y)+K_110*dphi_y);
return GetT1;

}

float GetLQRTorque2 (float theta_x,float dtheta_x,float theta_y,float dtheta_y,float theta_z,float dtheta_z, float phi_x,float dphi_x, float phi_y, float dphi_y,int phi_xdeseado,int phi_ydeseado ){

  float GetT2 =-1*(K21*(1.5*theta_x)+K22*(dtheta_x)+K23*(1.5*theta_y)+ K24*(dtheta_y)+K25*(theta_z)+K26*(dtheta_z)-K27*(phi_xdeseado-phi_x)+K28*(dphi_x)-K29*(phi_ydeseado-phi_y)+K210*(dphi_y));
return GetT2;

}
float GetLQRTorque3(float theta_x,float dtheta_x,float theta_y,float dtheta_y,float theta_z,float dtheta_z, float phi_x,float dphi_x, float phi_y, float dphi_y ,int phi_xdeseado,int phi_ydeseado){

  float GetT3 =-1*( K31*2.0*theta_x+K32*dtheta_x +K33*1.5*theta_y +K34*dtheta_y+K35*theta_z+K36*dtheta_z-K37*(phi_xdeseado-phi_x)+K38*dphi_x-K39*(phi_ydeseado-phi_y)+K310*dphi_y);
return GetT3;

}
//Funcion para transformar torques deseados a voltajes y seguidamente a PWM

float TorqueAPWMDeseado1(float Torque1, float OmegaMotor1){

  float CorrienteDeseada1= Torque1/KMotor;
  float VoltDeseado1= CorrienteDeseada1*Rmotor + OmegaMotor1*KMotor;
  int pwmDeseado1= (VoltDeseado1*255)/11.1;
 if (pwmDeseado1>=255){
  pwmDeseado1=255;
  }
  if(pwmDeseado1<=-255){
     pwmDeseado1=-255;
  }
  return (abs(pwmDeseado1));


}
float TorqueAPWMDeseado2(float Torque2, float OmegaMotor2){

  float CorrienteDeseada2= Torque2/KMotor;
  float VoltDeseado2= CorrienteDeseada2*Rmotor + OmegaMotor2*KMotor;
  int pwmDeseado2= (VoltDeseado2*255)/11.1;
  if (pwmDeseado2>=255){
  pwmDeseado2=255;
  }
  if(pwmDeseado2<=-255){
     pwmDeseado2=-255;
  }
  return (abs(pwmDeseado2));

}

float TorqueAPWMDeseado3(float Torque3, float OmegaMotor3){

  float CorrienteDeseada3= Torque3/KMotor;
  float VoltDeseado3= CorrienteDeseada3*Rmotor + OmegaMotor3*KMotor;
  int pwmDeseado3= (VoltDeseado3*255)/11.1;
 if (pwmDeseado3>=255){
  pwmDeseado3=255;
  }
  if(pwmDeseado3<=-255){
     pwmDeseado3=-255;
  }
  return (abs(pwmDeseado3));

}

float G_Dt=0.02; // el dt segun la frec de muestreo que será 50Hz
float lastPsi1=0;
float lastPsi2=0;
float lastPsi3=0;
//Conversion ticks a radianes
float TicksToRad (int x) {
 
 float rad= x*0.001923522212515; 
return rad;
}

//Conversion grados a radianes


void loop() 
{
   if((millis()-timer)>=10)  // Main loop runs at 50Hz
  {
    
  
    
  timer_old = timer;
   timer=millis();
   motor1.setSpeed (PWM1);
motor2.setSpeed(PWM2);
motor3.setSpeed(PWM3);




dpsi_1= (TicksToRad(encoder1Pos)-lastPsi1)/G_Dt;
dpsi_2= (TicksToRad(encoder2Pos)-lastPsi2)/G_Dt;
dpsi_3= (TicksToRad(encoder3Pos)-lastPsi3)/G_Dt;



lastPsi1= TicksToRad(encoder1Pos);
lastPsi2= TicksToRad(encoder2Pos);
lastPsi3= TicksToRad(encoder3Pos);

   if (timer>timer_old){
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    }
    else{
      G_Dt = 0;
    }}
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);  // Interrupciones para giroscopio y acelerometro
 imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
   
   
  
   
   
  

//Definicion de estados
float Lectheta_x=DegToRad(event.orientation.z);
float theta_x;
float Lectheta_y=DegToRad(event.orientation.y);
float theta_y;


float Lectheta_z=event.orientation.x;
float theta_z;

for(int k=0;k<20;k++){
 theta_x=Lectheta_x+ theta_x;

}


for(int k=0;k<20;k++){
 theta_y=Lectheta_y+ theta_y;

}


/* if (abs(Lectheta_z)<180){
theta_z= DegToRad(Lectheta_z); } //Estan al reves en el sensor !
else {
theta_z= DegToRad(Lectheta_z-360);

}*///DegToRad(event.orientation.x); //Están al reves en el sensor!

theta_x=0;//theta_x/20.0;
theta_y=theta_y/20.0;
theta_z=0;



float Dtheta_x= gyro.x();
float Dtheta_y=gyro.y();
float Dtheta_z=gyro.z();
float dtheta_x;
float dtheta_y;
float dtheta_z;
for(int k=0;k<20;k++){
 dtheta_x=Dtheta_x+ dtheta_x;

}
 

 
 for(int i=0;i<20;i++){
 dtheta_y=Dtheta_y + dtheta_y;

}



for(int j=0;j<20;j++){
 dtheta_z=Dtheta_z+dtheta_z;

}


dtheta_x=dtheta_x/20.0 - 0.01;
dtheta_y=dtheta_y/20.0 -0.02;
dtheta_z=dtheta_z/20.0;
 //Obtener datos de encoders 
 


psi_1= TicksToRad(encoder1Pos);
psi_2=TicksToRad(encoder2Pos);
psi_3=TicksToRad(encoder3Pos);



phi_x= (1)/(3*Rb)*(raiz2*Rw*(cos(theta_y)*(-2*psi_1+psi_2+psi_3)-cos(theta_x)*sin(theta_y)*(psi_1+psi_2+psi_3)+raiz3*sin(theta_x)*sin(theta_y)*(-1*psi_2+psi_3))+3*Rb*cos(theta_y)*theta_x);
 phi_y= (1)/(3*Rb)*(raiz3*Rw*cos(theta_x)*(-1*psi_2+psi_3)-raiz2*Rw*sin(theta_x)*(psi_1+psi_2+psi_3)+3*Rb*theta_y);


dphi_x= cos(theta_y)*(dtheta_x - dtheta_z*sin(theta_y) + raiz2*(Rw*dpsi_2 - 2*Rw*dpsi_1 + Rw*dpsi_3))/(3*Rb) + sin(theta_x)*sin(theta_y)*(dtheta_y*cos(theta_x) + dtheta_z*cos(theta_y)*sin(theta_x) - (raiz2*raiz3*(Rw*dpsi_2 - Rw*dpsi_3))/(3*Rb)) + cos(theta_x)*sin(theta_y)*((raiz2*(Rw*dpsi_1 + Rw*dpsi_2 + Rw*dpsi_3))/(3*Rb) - dtheta_y*sin(theta_x) + dtheta_z*cos(theta_x)*cos(theta_y));
 dphi_y=  cos(theta_x)*(dtheta_y*cos(theta_x) + dtheta_z*cos(theta_y)*sin(theta_x) -raiz2*raiz3*(Rw*dpsi_2 - Rw*dpsi_3))/(3*Rb) - sin(theta_x)*((raiz2*(Rw*dpsi_1 + Rw*dpsi_2 + Rw*dpsi_3))/(3*Rb) - dtheta_y*sin(theta_x) + dtheta_z*cos(theta_x)*cos(theta_y));
//float dphi_z=  cos(theta_x)*cos(theta_y)*((raiz2*(Rw*dpsi_1 + Rw*dpsi_2 + Rw*dpsi_3))/(3*Rb) - dtheta_y*sin(theta_x) + dtheta_z*cos(theta_x)*cos(theta_y)) - sin(theta_y)*(dtheta_x - dtheta_z*sin(theta_y) + (raiz2*(Rw*dpsi_2 - 2*Rw*dpsi_1 + Rw*dpsi_3))/(3*Rb)) + cos(theta_y)*sin(theta_x)*(dtheta_y*cos(theta_x) + dtheta_z*cos(theta_y)*sin(theta_x) - (raiz2*raiz3*(Rw*dpsi_2 - Rw*dpsi_3))/(3*Rb))




 




 T1 =GetLQRTorque1(theta_x, dtheta_x, theta_y, dtheta_y, theta_z, dtheta_z,  phi_x, dphi_x,  phi_y,  dphi_y , phi_xdeseado, phi_ydeseado);

PWM1=TorqueAPWMDeseado1(T1,dpsi_1);

 T2=GetLQRTorque2(theta_x, dtheta_x, theta_y, dtheta_y, theta_z, dtheta_z,  phi_x, dphi_x,  phi_y,  dphi_y , phi_xdeseado, phi_ydeseado);
PWM2=TorqueAPWMDeseado2(T2,dpsi_2);

 T3=GetLQRTorque3(theta_x, dtheta_x, theta_y, dtheta_y, theta_z, dtheta_z,  phi_x, dphi_x,  phi_y,  dphi_y , phi_xdeseado, phi_ydeseado);
PWM3=TorqueAPWMDeseado3(T3,dpsi_3);

//sensado de corriente de los motores

if((motor1.getCurrent()*5.0)/ (1024.0)>CURRENT_LIMIT){
motor1.stopMotors();

}
if((motor2.getCurrent()*5.0)/ (1024.0)>CURRENT_LIMIT){
motor2.stopMotors();

}
if((motor3.getCurrent()*5.0)/ (1024.0)>CURRENT_LIMIT){
motor3.stopMotors();

}
// señal de control a motores (direccion y velocidad)
if (T1>=0){
motor1.setMotorDirection(false);
pin1=0;
}
else {
pin1=1;
motor1.setMotorDirection(true);

}
if (T2>=0){
  pin2=0;
motor2.setMotorDirection(false);
}
else {
  pin2=1;
motor2.setMotorDirection(true);

}
if (T3>=0){
  pin3=0;
motor3.setMotorDirection(true);
}
else {
  pin3=1;
motor3.setMotorDirection(false);

}


}


void doEncoder1() {
  if (digitalRead(PinAEncoderMotor1) == HIGH) {
    if (digitalRead(PinBEncoderMotor1) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      encoder1Pos = encoder1Pos - 1;         // CCW
    } 
    else {
      encoder1Pos = encoder1Pos + 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(PinBEncoderMotor1) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
      encoder1Pos = encoder1Pos + 1;          // CW
    } 
    else {
      encoder1Pos = encoder1Pos - 1;          // CCW
    }
  }}

void doEncoder2() {
  if (digitalRead(PinAEncoderMotor2) == HIGH) {
    if (digitalRead(PinBEncoderMotor2) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      encoder2Pos = encoder2Pos - 1;         // CCW
    } 
    else {
      encoder2Pos = encoder2Pos + 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(PinBEncoderMotor2) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
      encoder2Pos = encoder2Pos + 1;          // CW
    } 
    else {
      encoder2Pos = encoder2Pos - 1;          // CCW
    }
  }}
  
  void doEncoder3() {
  if (digitalRead(PinAEncoderMotor3) == HIGH) {
    if (digitalRead(PinBEncoderMotor3) == LOW) {  // check channel B to see which way
                                             // encoder is turning
      encoder3Pos = encoder3Pos - 1;         // CCW
    } 
    else {
      encoder3Pos = encoder3Pos + 1;         // CW
    }
  }
  else                                        // found a high-to-low on channel A
  { 
    if (digitalRead(PinBEncoderMotor3) == LOW) {   // check channel B to see which way
                                              // encoder is turning  
      encoder3Pos = encoder3Pos + 1;          // CW
    } 
    else {
      encoder3Pos = encoder3Pos - 1;          // CCW
    }
  }}


