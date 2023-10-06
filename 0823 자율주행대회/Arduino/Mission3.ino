#include <Car_Library.h>

/////////////////////// DC Motor Control /////////////////////

int motorA1 = 2; // 모터 드라이버 IN1
int motorA2 = 3; // 모터 드라이버 IN2
int motorA3 = 5; // 모터 드라이버 IN1
int motorA4 = 6; // 모터 드라이버 IN2
int motorA5 = 8; // 모터 드라이버 IN1
int motorA6 = 9; // 모터 드라이버 IN2

/////////////////////// DC Motor Control /////////////////////



void setup() {
  // put your setup code here, to run once:
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorA3, OUTPUT);
  pinMode(motorA4, OUTPUT);
  pinMode(motorA5, OUTPUT);
  pinMode(motorA6, OUTPUT);
}
void STRAIGHT() //Straight
{
 motor_backward(motorA1, motorA2, 100);
 motor_forward(motorA3, motorA4, 100);
} 

void BACK() //Back
{ 
 motor_forward(motorA1, motorA2, 100);
 motor_backward(motorA3, motorA4, 100);
} 

void TURN() //Turn
{ 
 motor_forward(motorA1, motorA2, 240);
 motor_forward(motorA3, motorA4, 240);
}

void STOP() //STOP
{ 
 motor_hold(motorA1, motorA2);
 motor_hold(motorA3, motorA4);
} 
 
// 직진 - 홀드 - 턴 - 홀드 -후진 - 홀드(3초) - 직진 - 홀드 - 턴 - 홀드 -직진  
void loop() 
{
STRAIGHT ();
delay(11000);
STOP ();
delay(1000);
TURN ();
delay(1550);
STOP ();
delay(1000);
TURN ();
delay(1550);
STOP ();
delay(1000);
TURN ();
delay(1550);
STOP ();
delay(1000);
TURN ();
delay(1550);
STOP ();
delay(1000);
BACK ();
delay(3000);
STOP ();
delay(3000); // 주차
STRAIGHT ();
delay(4200);
STOP ();
delay(1000);
TURN ();
delay(1450);
STOP ();
delay(1000);
TURN ();
delay(1450);
STOP ();
delay(1000);
TURN ();
delay(1450);
STOP ();
delay(1000);
TURN ();
delay(1450);
STOP ();
delay(1000);
STRAIGHT ();
delay(7000);
}
