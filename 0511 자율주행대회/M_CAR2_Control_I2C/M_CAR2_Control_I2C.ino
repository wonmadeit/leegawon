#include <NewPing.h>
#include <Wire.h>

#define debug 1
#define SONAR_NUM 2      // Number of sensors.
#define MAX_DISTANCE 150 // Maximum distance (in cm) to ping.
#include <Servo.h>
Servo   Steeringservo;

union 
{   
   float data;
   unsigned char bytedata[4];
  
} sonar_float_data; 

float sonar_data[2] = {0.0,};
static int Steering_Angle;
static int Motor_Speed;

/////////////////////////// Sonnar Sensor  ///////////////////////

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
  NewPing(3, 2, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(12, 11, MAX_DISTANCE)  
};


/////////////////////////// Sonnar Sensor  ///////////////////////


///////////////////// Steering Servo Control /////////////////////
#define RC_SERVO_PIN 8
#define NEUTRAL_ANGLE 90
#define LEFT_STEER_ANGLE  -30
#define RIGHT_STEER_ANGLE  30



void steering_control()
{
  if(Steering_Angle<= LEFT_STEER_ANGLE + NEUTRAL_ANGLE)  Steering_Angle  = LEFT_STEER_ANGLE + NEUTRAL_ANGLE;
  if(Steering_Angle>= RIGHT_STEER_ANGLE + NEUTRAL_ANGLE)  Steering_Angle = RIGHT_STEER_ANGLE + NEUTRAL_ANGLE;
  Steeringservo.write(Steering_Angle);  
}

///////////////////// Steering Servo Control /////////////////////

/////////////////////// DC Motor Control /////////////////////

#define MOTOR_PWM 5 
#define MOTOR_DIR 4



void motor_control(int dir, int motor_pwm)
{
  
  if(dir == 1) // forward
  { 
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM,motor_pwm);    
  }
  else if(dir == -1) // backward
  {
    digitalWrite(MOTOR_DIR, LOW );
    analogWrite(MOTOR_PWM,motor_pwm);
  }
  else // stop
  {
    analogWrite(MOTOR_PWM,0);
  }
  
}

/////////////////////// DC Motor Control /////////////////////

/////////////////////////// I2C 통신 //////////////////////////


void receiveEvent(int howMany)
{
  unsigned char a[6];
  a[0] = Wire.read();
  a[1] = Wire.read();
  a[2] = Wire.read();
  a[3] = Wire.read();
  a[4] = Wire.read();
  a[5] = Wire.read();

  Steering_Angle =  a[1]*256 + a[2];
  Motor_Speed    =  a[4]*256 + a[5];
  
  /*
  Serial.print(a[0]);   Serial.print("  ");  Serial.print(a[1]);  Serial.print("  ");
  Serial.print(a[2]);   Serial.print("  ");  Serial.print(a[4]);   Serial.print("  ");
  Serial.print(a[5]);   Serial.print("  ");
  Serial.print(Steering_Angle);   Serial.print("  ");
  Serial.println(Motor_Speed);
  */
  steering_control();
  
  if(Motor_Speed> 0)  
  { 
    motor_control(1,Motor_Speed);
  }
  else if(Motor_Speed< 0) 
  {
    motor_control(-1,-Motor_Speed);
  }
  else motor_control(0,0);
   
}
/////////////////////////// I2C 통신 //////////////////////////

void requestEvent() 
{
  unsigned char s[8] = {0,};
  float temp1 ;
  float temp2 ;
  temp1 = sonar_data[0];
  sonar_float_data.data = temp1 ;

  s[0]= sonar_float_data.bytedata[0];
  s[1]= sonar_float_data.bytedata[1];
  s[2]= sonar_float_data.bytedata[2];
  s[3]= sonar_float_data.bytedata[3];

  temp2 = sonar_data[1];
  sonar_float_data.data = temp2;

  s[4]= sonar_float_data.bytedata[0];
  s[5]= sonar_float_data.bytedata[1];
  s[6]= sonar_float_data.bytedata[2];
  s[7]= sonar_float_data.bytedata[3];
  //sprintf("Data Send : %8s",String(sonar_data[0], 1).c_str());
  Wire.write(s,8); // respond 
}

void setup()
{
  Wire.begin(5);    // I2C bus  #5
  Wire.onRequest(requestEvent); // register events
  Wire.onReceive(receiveEvent);
  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(Steering_Angle);  

  Steering_Angle = NEUTRAL_ANGLE;
  Motor_Speed =0; 
  
  pinMode(MOTOR_PWM,OUTPUT); 
  pinMode(MOTOR_DIR,OUTPUT);
  
  Serial.begin(115200);
  delay(2000);
}

void loop()
{
 char buf[32];
 
 sonar_data[0] = sonar[0].ping_cm()*10.0;
 sonar_data[1] = sonar[1].ping_cm()*10.0;
 if(debug == 1)
  { 
   
    sprintf(buf,"Sonar : %8s %8s",String(sonar_data[0], 1).c_str(),String(sonar_data[1], 1).c_str()  );
    Serial.println(buf);

    Serial.print("Steering Angle : ");
    Serial.print(Steering_Angle);
  
    Serial.print("  Motor PWM : ");
    Serial.println(Motor_Speed); 

    delay(5);
     
  }
}
