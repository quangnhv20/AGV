// MultiStepper.pde
// -*- mode: C++ -*-
// Use MultiStepper class to manage multiple steppers and make them all move to 
// the same position at the same time for linear 2d (or 3d) motion.

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>
//ROS header
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

// EG X-Y position bed driven by 2 steppers
// Alas its not possible to build an array of these with different pins for each :-(
AccelStepper stepper1(1,2,5);
AccelStepper stepper2(1,3,6);

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

#define LOOPTIME                      10              //Looptime in millisecond
unsigned long lastMilli = 0;
const byte noCommLoopMax = 10;                        //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0; 

long pos_req[2]; // Array of desired stepper positions
long pos_run[2];

long stepper_res = 3200;                      //Encoder ticks or counts per rotation

//--- Robot-specific constants ---
const double radius     = 0.003;                     //Wheel radius, in m - deprecated
const double wheelbase  = 0.3; 

double speed_req         = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                         //Desired angular speed for the robot, in rad/s

double speed_req_A = 0;                               //Desired speed for left wheel in m/s
double speed_req_B = 0;                               //Desired speed for right wheel in m/s

double act_pos_A = 0;
double act_pos_B = 0;

ros::NodeHandle nh;

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  speed_req   = cmd_vel.linear.x;                                   //Extract the commanded linear speed from the message
  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message
  speed_req_A = 100;//speed_req - angular_speed_req*(wheelbase/2);        //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_B = -100;//speed_req + angular_speed_req*(wheelbase/2);        //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg); 


void setup() {
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);
  pinMode(8,OUTPUT); // Enable
  digitalWrite(8,LOW); // Dặt Enable xuống low để khởi động động cơ

  // Configure each stepper
  stepper1.setMaxSpeed(1600);
  stepper2.setMaxSpeed(1600);
  pos_run[2];
  // Then give them to MultiStepper to manage
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
}


double step_handler_A(double speed){
    double rpm = 9.55*(speed/0.0725);
    /*
     * n rpm => 1 min rotate n round
     * => 60s rotate n round
     * => 1s rotate n/60 round
     * => 100ms rotate n/600 round
     * => 10ms rotate n/6000 round
     * => 10ms rotate 3200*n/6000
     */
    long step = (stepper_res*rpm)/6000 ;// steps per 10ms
    return step;
}

double step_handler_B(double speed){
    double rpm = 9.55*(speed/0.0725);
    /*
     * n rpm => 1 min rotate n round
     * => 60s rotate n round
     * => 1s rotate n/60 round
     * => 100ms rotate n/600 round
     * => 10ms rotate n/6000 round
     * => 10ms rotate 3200*n/6000
     */
    long step = (stepper_res*rpm)/6000 ;// steps per 10ms
    return step;
}

void move_motor(long pos_in[]){
  //pos_run[0] += pos_in[0] ;
  //pos_run[1] += pos_in[1];
  steppers.moveTo(pos_in);
  steppers.runSpeedToPosition(); // Blocks until all are in position
}

//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = act_pos_A;         //left wheel speed (in m/s)
  speed_msg.vector.y = act_pos_B;         //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}

void loop() {
  nh.spinOnce(); 
  if((millis()-lastMilli) >= LOOPTIME)   
  {                                                                           // enter timed loop
    lastMilli = millis();
  pos_req[0]=step_handler_A(100);
  pos_req[1]=step_handler_B(-100);
  move_motor(pos_req);
noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }
    publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
  }
}
