
#include <avr/interrupt.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>

Servo servo;  //init servo


ros::NodeHandle  nh;


void pubpoten();
std_msgs::UInt16 poten_msg;
ros::Publisher pubPoten("Arduino/poten", &poten_msg);



void ROS_CB( const std_msgs::UInt8 &msg){
  servo.write(msg.data);
}
ros::Subscriber<std_msgs::UInt8> subangletoa1("Arduino/cmd_sv", ROS_CB);





void B1clearmap();
std_msgs::UInt16 B1_msg;
ros::Publisher pubB1("Arduino/button1", &B1_msg);

void B2pause();
std_msgs::UInt16 B2_msg;
ros::Publisher pubB2("Arduino/button2", &B2_msg);



void B3rand();
std_msgs::UInt16 B3_msg;
ros::Publisher pubB3("Arduino/button3", &B3_msg);

void setup(){

//Set timer
TCB0.CTRLB = TCB_CNTMODE_INT_gc; // Use timer compare mode
TCB0.CCMP = 25000; // Value to compare with. This is 1/10th of the tick rate, so 10 Hz
TCB0.INTCTRL = TCB_CAPT_bm; // Enable the interrupt
TCB0.CTRLA = TCB_CLKSEL_CLKTCA_gc | TCB_ENABLE_bm; // Use Timer A as clock, enable timer
//end of set timer

 servo.attach(5);
 
 nh.initNode();
 nh.subscribe(subangletoa1);
 nh.advertise(pubPoten);
 nh.advertise(pubB1);
 nh.advertise(pubB2);
 nh.advertise(pubB3);
}

void loop(){

  
}





void pubpoten(){
  poten_msg.data=analogRead(A0);
  pubPoten.publish( &poten_msg );
}
void B1clearmap(){
  static uint8_t pre=0;
  static uint16_t count=0;
  if(digitalRead(A1)==0 && pre==1){
    count=count+1;
   
  }else{
     count=count;
  }
  B1_msg.data=count;
  pubB1.publish( &B1_msg );
  pre=digitalRead(A1);
 
}
void B2pause(){
  static uint8_t pre=0;
  static uint16_t count=0;
  if(digitalRead(A2)==0 && pre==1){
    count=count+1;
  }else{
     count=count;
  }
  B2_msg.data=count;
  pubB2.publish( &B2_msg );
  pre=digitalRead(A2);
 
}
void B3rand(){
  static uint8_t pre=0;
  static uint16_t count=0;
  if(digitalRead(A3)==0 && pre==1){
    count=count+1; 
  }else{
     count=count;
  }
  B3_msg.data=count;
  pubB3.publish( &B3_msg );
  pre=digitalRead(A3);
 
}





ISR(TCB0_INT_vect)
{
  pubpoten();
  B1clearmap();
  B2pause();
  B3rand();
  nh.spinOnce();

// Clear interrupt flag
TCB0.INTFLAGS = TCB_CAPT_bm;


}
