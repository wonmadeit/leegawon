
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>

/*
explanation : example for data trasmit with I2C
*/
#include <string.h>  
#include <unistd.h>  
#include <errno.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <linux/i2c-dev.h>  
#include <sys/ioctl.h>  
#include <fcntl.h>  
#include <unistd.h>  

#include <sstream>

#define NEUTRAL_ANGLE 85  // gwlee original angle of wheels

//i2c address  
#define ADDRESS 0x05

//I2C bus  
static const char *deviceName = "/dev/i2c-0";
#define Base_Speed  100	// gwlee 0~255,speed control parameter 
float scalefactor = 1.5;


#define RAD2DEG(x) ((x)*180./M_PI)
// mm 
#define Sonar_Detect_Range 250 // gwlee 100mm stop threshold for object detection 100->150 modified

int steering_angle = NEUTRAL_ANGLE;
int motor_speed = 0;

int steering_angle_old = NEUTRAL_ANGLE;
int motor_speed_old = 0;

float sonar[2]={0.,0.};

unsigned char protocol_data[7] = {'S',0,0,'D',0,0,'#'};

union 
{
   
   float data;
   unsigned char bytedata[4];
  
} sonar_float_data; 


int file_I2C;

int open_I2C(void)
{
   int file;  
   
    if ((file = open( deviceName, O_RDWR ) ) < 0)   
    {  
        fprintf(stderr, "I2C: Failed to access %s\n", deviceName);  
        exit(1);  
    }  
    printf("I2C: Connected\n");  
  
   
    printf("I2C: acquiring buss to 0x%x\n", ADDRESS);  
    if (ioctl(file, I2C_SLAVE, ADDRESS) < 0)   
    {  
        fprintf(stderr, "I2C: Failed to acquire bus access/talk to slave 0x%x\n", ADDRESS);  
        exit(1);  
    } 
    
    return file; 
}

//Description : close the serial ports

void close_I2C(int fd)
{
   close(fd);
}

void CarControlCallback(const geometry_msgs::Twist& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  
   steering_angle = (int)(msg.angular.z+NEUTRAL_ANGLE) ; // gwlee original steering angle 85
   
   if(steering_angle >= 50+NEUTRAL_ANGLE)  steering_angle = NEUTRAL_ANGLE+50;
   if(steering_angle <=-50+NEUTRAL_ANGLE)  steering_angle = NEUTRAL_ANGLE-50;
   
   motor_speed = (int)msg.linear.x;
   if(motor_speed>=255)   motor_speed = 255;
   if(motor_speed<=-255)  motor_speed = -255;
   
}


void CarSteerControlCallback(const std_msgs::Int16& angle)
{
  steering_angle = (int)(angle.data+NEUTRAL_ANGLE) ;
  
  if(steering_angle >= 50+NEUTRAL_ANGLE)  steering_angle = NEUTRAL_ANGLE+50;
  if(steering_angle <=-50+NEUTRAL_ANGLE)  steering_angle = NEUTRAL_ANGLE-50;
  
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = (int)( 360. / RAD2DEG(scan->angle_increment));
    int sum=0; 
   // ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
   // ROS_INFO("%f %f",scan->scan_time , scan->time_increment);
   // ROS_INFO("angle_range, %f, %f %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max), RAD2DEG(scan->angle_increment));
  /*
    for(int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
    }
    */
    
    for(int i = 0; i < count; i++)
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        
        if( ((degree<=15) &&(degree>=0)) ||  ( (degree<=360)&&(degree>=360-15)))
        {
         // ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
          if(scan->ranges[i] <= 0.5) 
          {
            sum++;
            //ROS_INFO("sum 2= %d", sum);
          }
          
        }
      
    }
     //ROS_INFO("sum = %d", sum);
     if(sum >=10)   
     {
       motor_speed = 0;
       ROS_INFO("Obstacle Detected !!");
     }
     else           motor_speed = Base_Speed;     
}

int main(int argc, char **argv)
{
  char buf[10];
  ros::init(argc, argv, "Car_Control");
  
  ros::NodeHandle n;
  
  ros::Subscriber sub1 = n.subscribe("/cmd_vel", 10, &CarControlCallback);
  ros::Subscriber sub2 = n.subscribe("/Car_Control_cmd/SteerAngle_Int16",10, &CarSteerControlCallback);  
  ros::Subscriber sub3 = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &scanCallback);
  
  //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher car_control_pub1 = n.advertise<std_msgs::String>("Car_Control/SteerAngle_msgs", 10);
  ros::Publisher car_control_pub2 = n.advertise<std_msgs::String>("Car_Control/Speed_msgs", 10);
  ros::Publisher car_control_pub3 = n.advertise<std_msgs::Int16>("Car_Control/SteerAngle_Int16", 10);
  ros::Publisher car_control_pub4 = n.advertise<std_msgs::Int16>("Car_Control/Speed_Int16", 10);
 
  ros::Rate loop_rate(25);///10);  // 10
  file_I2C = open_I2C();
  if(file_I2C < 0)
  {
	  ROS_ERROR_STREAM("Unable to open I2C");
	  return -1;
  }
  else
  {
	  ROS_INFO_STREAM("I2C is Connected");
  }

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    
    std_msgs::String msg;
    std_msgs::Int16 steerangle;
    std_msgs::Int16 carspeed;
    std::stringstream ss;    
    std::string data;
    data = std::to_string(steering_angle);
    

    steerangle.data = steering_angle;
    printf("speed: %d \n",motor_speed);
    carspeed.data = motor_speed;
    ss<<data;
    msg.data = ss.str();
    ROS_INFO("Steer : %s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    car_control_pub1.publish(msg);
    printf("angle %d \n",steering_angle);
    //ros::Duration(0.5).sleep();
  
    data = std::to_string(motor_speed);
    msg.data = data;
    car_control_pub2.publish(msg);
    ROS_INFO("Speed : %s", msg.data.c_str());
    
    protocol_data[1] = (steering_angle&0xff00)>>8 ;
    protocol_data[2] = (steering_angle&0x00ff);
    protocol_data[4] = (motor_speed&0xff00)>>8 ;
    protocol_data[5] = (motor_speed&0x00ff);
    
    if(1)//steering_angle != steering_angle_old) 
    {
       //write_serial(protocol_data,5);
       write(file_I2C, protocol_data, 6);
       read(file_I2C,buf,8);
       
       sonar_float_data.bytedata[0] = buf[0];       
       sonar_float_data.bytedata[1] = buf[1];       
       sonar_float_data.bytedata[2] = buf[2];       
       sonar_float_data.bytedata[3] = buf[3];       
       
       sonar[0] = sonar_float_data.data;
       
       sonar_float_data.bytedata[0] = buf[4];       
       sonar_float_data.bytedata[1] = buf[5];       
       sonar_float_data.bytedata[2] = buf[6];       
       sonar_float_data.bytedata[3] = buf[7];       
       
       sonar[1] = sonar_float_data.data;
    }
     if(0)//motor_speed != motor_speed_old)
    {
    
      //write_serial(protocol_data,5);
       write(file_I2C, protocol_data, 6);
       read(file_I2C,buf,8);
       
       sonar_float_data.bytedata[0] = buf[0];       
       sonar_float_data.bytedata[1] = buf[1];       
       sonar_float_data.bytedata[2] = buf[2];       
       sonar_float_data.bytedata[3] = buf[3];       
       
       sonar[0] = sonar_float_data.data;
       
       sonar_float_data.bytedata[0] = buf[4];       
       sonar_float_data.bytedata[1] = buf[5];       
       sonar_float_data.bytedata[2] = buf[6];       
       sonar_float_data.bytedata[3] = buf[7];       
       
       sonar[1] = sonar_float_data.data;
    } 
   
    steering_angle_old = steering_angle;
    motor_speed_old = motor_speed ; 
    
    printf("sonar1 %6.3lf \n",sonar[0]);
    printf("sonar2 %6.3lf \n",sonar[1]);
    //gwlee 0430
    //////////////////// sonar obstacle detectioin //////////////////
    if( (( sonar[0] <= Sonar_Detect_Range) && ( sonar[0] > 0) )  ||( ( sonar[1] <= Sonar_Detect_Range) && ( sonar[1] > 0) ))
    {
       motor_speed  = 0;
       ROS_INFO("Sonar Obstacle detection : %5.1lf %5.1lf", sonar[0],sonar[1]);
       //ros::Duration(1).sleep();
       protocol_data[4] = (motor_speed&0xff00)>>8 ;
       protocol_data[5] = (motor_speed&0x00ff);
       write(file_I2C, protocol_data, 6);
       read(file_I2C,buf,8);
       
       sonar_float_data.bytedata[0] = buf[0];       
       sonar_float_data.bytedata[1] = buf[1];       
       sonar_float_data.bytedata[2] = buf[2];       
       sonar_float_data.bytedata[3] = buf[3];       
       
       sonar[0] = sonar_float_data.data;
       
       sonar_float_data.bytedata[0] = buf[4];       
       sonar_float_data.bytedata[1] = buf[5];       
       sonar_float_data.bytedata[2] = buf[6];       
       sonar_float_data.bytedata[3] = buf[7];       
       
       sonar[1] = sonar_float_data.data;
     }
    else
    {
        if (steering_angle >= 83 && steering_angle <= 87) //tung 0501 speed threshold added
			motor_speed = 1.5 * Base_Speed;
		else
		{
			float scale = 1.5 - std::abs(steering_angle - 85) / 30;
			if (scale < 1) scale = 1;
			//motor_speed = scale * Base_Speed;
      motor_speed = 80;      //gwlee 0501 
		}	
        
    } 
    car_control_pub3.publish(steerangle);
    car_control_pub4.publish(carspeed);
    ROS_INFO("Sonar : %4.1lf %4.1lf", sonar[0],sonar[1]);
    
    loop_rate.sleep();
    ros::spinOnce();
    
    ++count;
  }
  ROS_INFO("ROS Stopped");

  motor_speed = 0;
  protocol_data[4] = (motor_speed&0xff00)>>8 ;
  protocol_data[5] = (motor_speed&0x00ff);
  write(file_I2C, protocol_data, 6);
  read(file_I2C,buf,8);
  
  sonar_float_data.bytedata[0] = buf[0];       
  sonar_float_data.bytedata[1] = buf[1];       
  sonar_float_data.bytedata[2] = buf[2];       
  sonar_float_data.bytedata[3] = buf[3];       
       
  sonar[0] = sonar_float_data.data;
       
  sonar_float_data.bytedata[0] = buf[4];       
  sonar_float_data.bytedata[1] = buf[5];       
  sonar_float_data.bytedata[2] = buf[6];       
  sonar_float_data.bytedata[3] = buf[7];       
       
  sonar[1] = sonar_float_data.data;
       
  close_I2C(file_I2C);
  return 0;
}




     //ROS_INFO("%s", msg.data.c_str());
    //ROS_INFO("%c", protocol_data[0]);
    //ROS_INFO("%c", protocol_data[1]);
    //ROS_INFO("%c", protocol_data[2]);
    //ROS_INFO("%c", protocol_data[3]);
    //ROS_INFO("%x", protocol_data[4]);
