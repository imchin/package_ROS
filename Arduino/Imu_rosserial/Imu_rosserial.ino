#include <Arduino_LSM6DS3.h>


#include <ros.h>
#include <sensor_msgs/Imu.h>


#include <avr/interrupt.h>


ros::NodeHandle  nh;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data_raw", &imu_msg);


float ax, ay, az;  
float gx, gy, gz;  
void readandsendros();
float AA[9] ={-1,-1,-1,-1,-1,-1,-1,-1,-1};
float identity[9] ={1,0,0,0,1,0,0,0,1};

void setup() {
 
  
  IMU.begin(); 
  
  nh.initNode();
  nh.advertise(imu_pub);
  
  imu_msg.header.frame_id = 0;
//  imu_msg.orientation_covariance[0] = -1;
  memcpy(imu_msg.orientation_covariance, identity, sizeof(identity));
  memcpy(imu_msg.angular_velocity_covariance, identity, sizeof(identity));
  memcpy(imu_msg.linear_acceleration_covariance, identity, sizeof(identity));
  //Set timer
  TCB0.CTRLB = TCB_CNTMODE_INT_gc; 
  TCB0.CCMP = 1250; 
  TCB0.INTCTRL = TCB_CAPT_bm; 
  TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm;
  //end of set timer
  
}




void loop() {
   
}
void readandsendros(){


  if(IMU.readAcceleration(gx, gy, gz) && IMU.readGyroscope(ax, ay, az)){
      imu_msg.angular_velocity.x=ax*(3.141592/180);
      imu_msg.angular_velocity.y=ay*(3.141592/180);
      imu_msg.angular_velocity.z=az*(3.141592/180);
      imu_msg.linear_acceleration.x=gx*(9.80665);
      imu_msg.linear_acceleration.y=gy*(9.80665);
      imu_msg.linear_acceleration.z=gz*(-9.80665);
      
      imu_msg.header.stamp = nh.now();
      imu_pub.publish( &imu_msg );
    
  }
  
 
  
}

ISR(TCB0_INT_vect)
{

  
  readandsendros();
  nh.spinOnce();
  
  TCB0.INTFLAGS = TCB_CAPT_bm; 



}
