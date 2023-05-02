#include <NewPing.h>
#include <PID_v1.h>
#define in1 13 
#define in2 12 
#define in3 9 
#define in4 8
#define ENA 10
#define ENB 11
#define LeftMotorENCA 2
#define RightMotorENCA 3
#define TRIGGER_PINF  A4  
#define ECHO_PINF     A5  
#define MAX_DISTANCE 293
volatile unsigned long count = 0;
unsigned long count_prev = 0;
float Theta, RPM, RPM_d;
float Theta_prev = 0;
int dt;
#define pi 3.1416
float Vmax = 6;
float Vmin = -6;
float V = 0.1;
float e, e_prev = 0, inte, inte_prev = 0;

int LeftMotorENCA_DATA,LeftMotorENCB_DATA;

float fSensor,oldFrontSensor;
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE); //setup front ultrasonic

int baseSpeed = 50 ;

int RMS ;  //right motor speeed
int LMS ;  //left motor speed

volatile long Pulse_Per_Rev_Left=0;
volatile long Pulse_Per_Rev_Right=0;
unsigned long currentTime;
unsigned long prevTime=0;
unsigned long RPM_Right=0;
unsigned long RPM_Left=0;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
pinMode(LeftMotorENCA, INPUT_PULLUP);
pinMode(RightMotorENCA, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(LeftMotorENCA), ISR_LeftMotor, FALLING);
attachInterrupt(digitalPinToInterrupt(RightMotorENCA), ISR_RightMotor, FALLING);

// unsigned int pingSpeed = 30; // frequincy of sending ping
// unsigned long pingTimer;     // wait time for next ping
for(int i =8;i<14;i++){
  pinMode(i,OUTPUT);
}



// cli();
// TCCR1A = 0;
// TCCR1B = 0;
// TCNT1 = 0;
// OCR1A = 12499; //Prescaler = 64
// TCCR1B |= (1 << WGM12);
// TCCR1B |= (1 << CS11 | 1 << CS10);
// TIMSK1 |= (1 << OCIE1A);
// sei();


}

void loop() {


 //if(prevTime + 2000 < currentTime){
    delay(2000);
//   //Serial.println("started");
   Forward();
   runMotors();
//  //}
 
// Serial.print("RPM_Right: ");
// Serial.print(Pulse_Per_Rev_Right);
// Serial.print(" ");
// Serial.print("RPM_Left: ");
// Serial.print(Pulse_Per_Rev_Left);
// Serial.println(" ");
//   Serial.print("Time: ");
//   Serial.println(currentTime);
//   // RPM_Right=0;
//   // RPM_Left =0;
// ReadFront();

delay(100000);

}
void Forward(){
  
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);  
}

void Left(){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);  
}
void Right(){
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);  
}
void Stop(){
  digitalWrite(in1,HIGH);
  digitalWrite(in2,HIGH);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,HIGH);   
}

void runMotors() {
  ReadFront();
  oldFrontSensor = fSensor;
   Serial.print("Front: ");
    Serial.println(fSensor);
  RMS = map(baseSpeed, 0, 1023, 0 , 1023);
  LMS = map(baseSpeed , 0, 1023, 0 , 1023);
  prevTime=millis();
  analogWrite(ENA, 600);
  analogWrite(ENB, 610);
  while(oldFrontSensor - 18.2 < fSensor){
    ReadFront();
    Serial.print("Front: ");
    Serial.println(fSensor);
  }
  
  Stop();
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  currentTime=millis() - prevTime;
}

void ISR_LeftMotor(){
  Pulse_Per_Rev_Left++;
  if(Pulse_Per_Rev_Left % 189 == 0 ){
    RPM_Left++;
   // Pulse_Per_Rev_Left=0;
  }
}

void ISR_RightMotor(){
  Pulse_Per_Rev_Right++;
   if(Pulse_Per_Rev_Right % 206 == 0 ){
    RPM_Right++;
    //Pulse_Per_Rev_Right=0;
  }
}

void ReadFront(){
  fSensor = sonarFront.ping()/100.0;
}