//Arduiiiiiinomuhyun
#include <Car_Library.h>
#include <ros.h>
#include <geometry_msgs/Twist.h> //number
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

#define FORWARD_RIGHT 1
#define BACKWARD_LEFT 2
#define NO_TURN 3
#define IS_OBSTACLE 4
#define NO_OBSTACLE 5
#define WAITING_TURN_RIGHT 6
#define NO_WAITING_TURN_RIGHT 7
#define TURN_RIGHT_NOW 8
#define TURN_LEFT 9
#define TURN_RIGHT 10
#define HAVE_TO_NO_OB 11
#define NO_TO_HAVE_OB 12
#define DONE 13
#define LEFT 14
#define RIGHT 15
#define STATUS_NOT_CHANGED 16
#define OBSTACLE_FINISH_IDLE 21
#define TURN_LEFT_NOW 22
enum CameraOrLidar {FOLLOW_CAMERA_ANGLE = 17, FOLLOW_LIDAR = 18};
enum TrafficStatus {RED = 19, GREEN = 20};
int Motor_Speed;
int TurningSpeed = 250;
//나중에 모터 동작 할 때

// 전방(조향) 5,6 왼쪽뒤 1,2 오른쪽뒤 3,4
/////////////////////// 초음파 센서 //////////////////////////
int trig1 = 45; //좌측 전방
int echo1 = 44; //좌측 전방
int trig2 = 43; //우측 전방
int echo2 = 42; //우측 전방
int trig3 = 41; //우측 후방
int echo3 = 40; //우측 후방
/////////////////////// 초음파 센서 //////////////////////////

/////////////////////// DC Motor Control /////////////////////

int motorA1 = 2; // 모터 드라이버 IN1
int motorA2 = 3; // 모터 드라이버 IN2
int motorA3 = 5; // 모터 드라이버 IN1
int motorA4 = 6; // 모터 드라이버 IN2
int motorA5 = 8; // 모터 드라이버 IN1
int motorA6 = 9; // 모터 드라이버 IN2

/////////////////////// DC Motor Control /////////////////////

/////////////////////// Steering Control (가변 저항)  ////////////
int analogPin = A5;
/////////////////////// Steering Control (가변 저항)  ////////////
          //section                 1,             2,                3,                       4,                5,                        6                    7 
int angleRange[8][2] = {{0, 0}, {500-4*20, 500-3*20}, {500-3*20, 500-2*20},  {500-2*20, 500-20}, {500-20, 500}, {500, 500+20}, {500+20, 500+2*20},{500+2*20, 500+3*20}};
float speedScale[8] = { 0,             0.6,      0.7,                       0.8,                1,                0.8,                   0.7,                 0.6};
int sectionNum = 1;
float DrivScale = 1;
int RightAvoidAngle[2] = {angleRange[1][0],angleRange[1][1]}; //????????????????????????????????????????
int LeftAvoidAngle[2] ={angleRange[7][0],angleRange[7][1]}; //????????????????????????????????????????
bool motorRunning = false;
long distanceFrontLeft;
long distanceFrontRight;
long distanceBackRight;
////////////////////////////// 조향 //////////////////////////////////
int turningDir = 0;

int currentAngle;
// camera angle message
int targetAngle = 490;
bool targetAngleReceived = false;
//lidar message
int obstacleDistance = 100;
int isObstacle = NO_OBSTACLE;
int cameraOrLidar = CameraOrLidar::FOLLOW_CAMERA_ANGLE;
int obstacleFromSonicSensorState = NO_OBSTACLE;
int waitingForTurningRight = NO_WAITING_TURN_RIGHT;
int turnedDirection = LEFT;
int turningMaxValue;
int turningMinValue;
int obstacleStatusChanged = 100;
int obstacleFinished = OBSTACLE_FINISH_IDLE;
int countObstacle = 0;
//traffic light message
// 타겟과 현재비교하여 조향모터 동작함수
int sideDistance = 450;
//ROS Set up
ros::NodeHandle nh;

void CameraAngleCallback(const std_msgs::Int16& angle_msg)
{
  motorRunning = true;
  nh.loginfo("Lidar Obstacle Callback");
  targetAngle = angle_msg.data;
  //nh.loginfo("Camera Caallback");
  if (obstacleFinished == OBSTACLE_FINISH_IDLE && targetAngle >= angleRange[1][0] && targetAngle <= angleRange[7][1]) // have target angle
  {
    nh.loginfo("Camera true");
      targetAngleReceived = true;
  }
  else if (targetAngle == 1) // start drive motor
  {
      motorRunning = true;
}


}
ros::Subscriber<std_msgs::Int16> subCamAngle("cameraAngle", &CameraAngleCallback);
void CameraTrafficLightCallback(const std_msgs::String& color_msg)
{
  const char* str = (const char*)color_msg.data;
  String colorString(str);
  if (colorString.equals("red"))
  {
    motor_hold(motorA1, motorA2);
    motor_hold(motorA3, motorA4);
  }
  else
  {
    motorRunning = true;
  }
}
ros::Subscriber<geometry_msgs::Twist> subCamTraffic("color", &CameraTrafficLightCallback);
void LidarObstacle(const std_msgs::Int16& lidarObstacle_msg) //distance, obstacle status(safe or not): safe--> follow camera infor, not safe-->채헌 algorithm
{
  nh.loginfo("Lidar Obstacle Callback");
//  int distance = lidarObstacle_msg.linear.x;
  int isObstacleTemp = lidarObstacle_msg.data;
  motorRunning = true;
  if (isObstacleTemp == NO_OBSTACLE)// || obstacleFinished == HAVE_TO_NO_OB)
  {
      nh.loginfo("NO OBS");
      DrivScale = 1;
      if (isObstacle != isObstacleTemp)
      {
          obstacleStatusChanged = HAVE_TO_NO_OB;
          isObstacle = NO_OBSTACLE;
          nh.loginfo("NO OBS");
      }
      if (waitingForTurningRight == TURN_RIGHT_NOW)
      {
          isObstacle = IS_OBSTACLE;
          DrivScale = 0.6;
      }
  }
  else if (isObstacleTemp == IS_OBSTACLE)
  {
      nh.loginfo("OBS detected");
      if (isObstacle != isObstacleTemp)
      {
          DrivScale = 0.6;
          obstacleStatusChanged = NO_TO_HAVE_OB;
          isObstacle = IS_OBSTACLE;
          waitingForTurningRight = TURN_LEFT_NOW;
          obstacleFinished = NO_TO_HAVE_OB;
          nh.loginfo("OBS detected change value");
      }
      cameraOrLidar = CameraOrLidar::FOLLOW_LIDAR;
  }
}
ros::Subscriber<std_msgs::Int16> subLidar("/cmd_vel", &LidarObstacle);
void setup() {
  //drive 1 (left back)
  nh.initNode();
  nh.subscribe(subLidar);
  nh.subscribe(subCamAngle);
  
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  //drive 2 (right back)
  pinMode(motorA3, OUTPUT);
  pinMode(motorA4, OUTPUT);
  //sterring
  pinMode(motorA5, OUTPUT);
  pinMode(motorA6, OUTPUT);
  //sona
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT);

  Motor_Speed = 100; //모터 속도
  
  
}

void Drivspeed()
{
  
  motor_forward(motorA3, motorA4, Motor_Speed*DrivScale);
  motor_backward(motorA1, motorA2, Motor_Speed*DrivScale);
  //delay(1000);
}

void CheckDirectionAndSteering(int currentAngle, int targetAngle)
{
  if (currentAngle < targetAngle)
  {
    //Serial.println("Turn left");
    turningDir = BACKWARD_LEFT;
    motor_forward(motorA5, motorA6, TurningSpeed);
  }
  else if (currentAngle > targetAngle)
  {
    //Serial.println("Turn right");
    turningDir = FORWARD_RIGHT;
    motor_backward(motorA5, motorA6, TurningSpeed);
  }
  else
  {
    //Serial.println("No turn");
    turningDir = NO_TURN;
    motor_hold(motorA5, motorA6);
  }
}
void TurningDir(int turnedDirection)
{
  if (turnedDirection == TURN_LEFT)
  {
      turnedDirection = TURN_RIGHT;
      turningMinValue = RightAvoidAngle[0];
      turningMaxValue = RightAvoidAngle[1];
  }
  else if (turnedDirection == TURN_RIGHT)
  {
      turnedDirection = TURN_LEFT;
      turningMinValue = LeftAvoidAngle[0];
      turningMaxValue = LeftAvoidAngle[1];
  }
}
bool SteeringControl(int targetAngle)
 {
  //타겟의 해당구간찾는 함수
  //nh.loginfo("Steering");
  if (isObstacle == NO_OBSTACLE && targetAngleReceived==true) ///////////////////////////////////????????????????????????????????????????
  {
    //nh.loginfo("Check Section");
    targetAngleReceived = false;
    bool checkingSection = false;
    int minRange;
    int maxRange;
    while (!checkingSection)
    {
        minRange = angleRange[sectionNum][0];
        maxRange = angleRange[sectionNum][1];
        if (targetAngle >= minRange && targetAngle < maxRange)
            checkingSection = true;
        else
            sectionNum++;
        if (sectionNum >7) sectionNum = 1;
    }
    //현재조향을 해당구간으로 이동시키는 함수
    //Serial.print("Current angle value: "); //Serial.println(currentAngle);
    //Serial.print("You will move to section: "); //Serial.println(sectionNum);
    if (currentAngle >= minRange && currentAngle < maxRange) // moving
    {
        //Serial.println("Turned!");
    }
    else
    {
        CheckDirectionAndSteering(currentAngle, targetAngle);    
        //Serial.println("Turing");
        while (!(currentAngle >= minRange && currentAngle < maxRange))
        {
            currentAngle = potentiometer_Read(analogPin) * 4;
            //Serial.print("Current angle value: "); //Serial.println(currentAngle);
        }
        //Serial.println("Done!");
        turningDir = NO_TURN;
        motor_hold(motorA5,motorA6);
    }
  }
  else if (isObstacle == IS_OBSTACLE)///////////////////////// 초음파 센서 ////////////////////////// ///////////////////////////////////?
  {
    nh.loginfo("IS OBSTACLE");
    if (waitingForTurningRight == TURN_LEFT_NOW) //옆차선 차량x -> Lefttrun
    {
        //motor_forward(motorA3, motorA4, Motor_Speed*0.6);
        //motor_backward(motorA1, motorA2, Motor_Speed*0.6);
        sectionNum = 1;
        targetAngle = (LeftAvoidAngle[0] + LeftAvoidAngle[1])/2;
        if (currentAngle >= LeftAvoidAngle[0] && currentAngle < LeftAvoidAngle[1])
        {}
        else
        {
          //CheckDirectionAndSteering(currentAngle, targetAngle);
          while (!(currentAngle >= LeftAvoidAngle[0] && currentAngle < LeftAvoidAngle[1]))
          {
              if (currentAngle < targetAngle)
              {
                  //Serial.println("Turn left");
                  turningDir = BACKWARD_LEFT;
                  motor_forward(motorA5, motorA6, TurningSpeed);
                  
                 
              }
              else if (currentAngle > targetAngle)
              {
                  //Serial.println("Turn left");
                  turningDir = FORWARD_RIGHT;
                  motor_backward(motorA5, motorA6, TurningSpeed);
              }
              delayMicroseconds(1900);
              motor_hold(motorA5,motorA6);
              currentAngle = potentiometer_Read(analogPin) * 4;
          }
          motor_hold(motorA5,motorA6);
          //Serial.println("Done!");
          turningDir = NO_TURN;          
        }
    }
    else if (waitingForTurningRight == TURN_RIGHT_NOW) //옆차선 차량 -> Righttrun
    {
        sectionNum = 7;
        //motor_forward(motorA3, motorA4, Motor_Speed*0.6);
        //motor_backward(motorA1, motorA2, Motor_Speed*0.6);
        targetAngle = (RightAvoidAngle[0] + RightAvoidAngle[1])/2;
        if (currentAngle >= RightAvoidAngle[0] && currentAngle < RightAvoidAngle[1])
        {}
        else
        {
          //CheckDirectionAndSteering(currentAngle, targetAngle);
          //Serial.println("Turing");
          while (!(currentAngle >= RightAvoidAngle[0] && currentAngle < RightAvoidAngle[1]))
          {
              if (currentAngle < targetAngle)
              {
                  //Serial.println("Turn left");
                  turningDir = BACKWARD_LEFT;
                  motor_forward(motorA5, motorA6, TurningSpeed);
              }
              else if (currentAngle > targetAngle)
              {
                  //Serial.println("Righttrun");
                  turningDir = FORWARD_RIGHT;
                  motor_backward(motorA5, motorA6, TurningSpeed);
              }
              delayMicroseconds(1900);
              motor_hold(motorA5,motorA6);
              currentAngle = potentiometer_Read(analogPin) * 4;
          }
          //Serial.println("Done!");
          turningDir = NO_TURN;
          motor_hold(motorA5,motorA6);
        }
    }
   
  }
}
//main loop
void loop()
{
  distanceFrontLeft = ultrasonic_distance(trig1, echo1); //좌측 전방
  distanceFrontRight = ultrasonic_distance(trig2, echo2); //우측 전방
  distanceBackRight = ultrasonic_distance(trig3, echo3); //우측 후방
  if (obstacleStatusChanged == HAVE_TO_NO_OB) // check sonic distance
  {
    nh.loginfo("Checking WAITING_TURN_RIGHT");
    if (distanceFrontRight < sideDistance && distanceBackRight < sideDistance)
        countObstacle++;
    if (countObstacle >= 100)
    {
        waitingForTurningRight = WAITING_TURN_RIGHT;
        obstacleFinished = OBSTACLE_FINISH_IDLE; // allow receive data from camera
        countObstacle = 0;
        obstacleStatusChanged = DONE;
        nh.loginfo("WAITING_TURN_RIGHT");
    }    
  }
  if (waitingForTurningRight == WAITING_TURN_RIGHT)
  {
    nh.loginfo("Checking TURN_RIGHT_NOW");
  if (distanceFrontRight > sideDistance && distanceBackRight < sideDistance)
  {
    nh.loginfo("Checked: TURN_RIGHT_NOW");
    waitingForTurningRight = TURN_RIGHT_NOW;
    obstacleFinished = HAVE_TO_NO_OB; // not receive data from camera
  }
  }
  if (waitingForTurningRight == TURN_RIGHT_NOW)
  {
    nh.loginfo("Checking No_WAITING_TURN_RIGHT");
  if (distanceFrontRight > sideDistance && distanceBackRight > sideDistance)
  {
      waitingForTurningRight = NO_WAITING_TURN_RIGHT;
      obstacleFinished = OBSTACLE_FINISH_IDLE; //allow receive data from camera
      obstacleStatusChanged = 100;
      nh.loginfo("Done TURN_RIGHT_NOW!");
  }    
  }
  currentAngle = potentiometer_Read(analogPin) * 4;
  SteeringControl(targetAngle);
  if (motorRunning)
    Drivspeed();

  nh.spinOnce();
}
/////////////////////////시리얼 통신/////////////////////////
//by 채헌, 동욱
