/*
 * Base Controller
 * Base Controller that connects the joint actuators to ROS master
 * Author :: Rae Jeong
 * Email  :: raychanjeongjeong@gmail.com
 */
#include <Wire.h>
#include <WireData.h>
#include <ros.h>
#include <std_msgs/Float32.h>

/*
 * I2C Commands available on each joint controller.
 *
 */
#define I2C_COMMAND_NULL                        0
#define I2C_COMMAND_JOINT_SET_SETPOINT          1
#define I2C_COMMAND_JOINT_GET_SETPOINT          2
#define I2C_COMMAND_JOINT_GET_POSITION          3
#define I2C_COMMAND_JOINT_SET_KP                4
#define I2C_COMMAND_JOINT_GET_KP                5
#define I2C_COMMAND_JOINT_GET_CALIBRATION_STATE 6
#define I2C_COMMAND_JOINT_GET_DIRECTION         7
#define I2C_COMMAND_JOINT_SET_DIRECTION         8
#define I2C_COMMAND_JOINT_GET_CAL_DIRECTION     9
#define I2C_COMMAND_JOINT_SET_CAL_DIRECTION     10
#define I2C_COMMAND_JOINT_CALIBRATE             11
#define I2C_COMMAND_JOINT_HALT                  12
#define I2C_COMMAND_JOINT_HOME                  13
#define I2C_COMMAND_JOINT_MOTOR_OFF             14

// 7 bit I2C/TWI addresses are in the range of 0x08 to 0x77

const int joint_1_address = 0x08;
const int joint_2_address = 0x09;
const int joint_3_address = 0x0a;

/*
 * Function prototypes
 */
void joint1SetSetpointCb(const std_msgs::Float32& joint_1_setpoint_msg);
void joint2SetSetpointCb(const std_msgs::Float32& joint_2_setpoint_msg);
void joint3SetSetpointCb(const std_msgs::Float32& joint_3_setpoint_msg);

double jointGet(int joint_address, int I2C_command);
void jointSet(int joint_address, int I2C_command, double set_value);

/*
 * Variable to store the current and new setpoint
 */
double joint_1_current_setpoint;
double joint_1_new_setpoint;

double joint_1_position;

double joint_2_current_setpoint;
double joint_2_new_setpoint;

double joint_2_position;

double joint_3_current_setpoint;
double joint_3_new_setpoint;

double joint_3_position;


/*
 * ROS node handle, encoder reading, subscriber and publisher
 */
std_msgs::Float32 joint_1_position_msg;

ros::Subscriber<std_msgs::Float32> joint_1_set_setpoint_listener(
    "joint_1_set_setpoint",
    &joint1SetSetpointCb);

ros::Publisher joint_1_position_publisher("joint_1_position", &joint_1_position_msg);


std_msgs::Float32 joint_2_position_msg;

ros::Subscriber<std_msgs::Float32> joint_2_set_setpoint_listener(
    "joint_2_set_setpoint",
    &joint2SetSetpointCb);

ros::Publisher joint_2_position_publisher("joint_2_position", &joint_2_position_msg);


std_msgs::Float32 joint_3_position_msg;

ros::Subscriber<std_msgs::Float32> joint_3_set_setpoint_listener(
    "joint_3_set_setpoint",
    &joint3SetSetpointCb);

ros::Publisher joint_3_position_publisher("joint_3_position", &joint_3_position_msg);


ros::NodeHandle nh;


void setup()
{
  Wire.begin();

  nh.initNode();
  nh.subscribe(joint_1_set_setpoint_listener);
  nh.advertise(joint_1_position_publisher);
  nh.subscribe(joint_2_set_setpoint_listener);
  nh.advertise(joint_2_position_publisher);
  nh.subscribe(joint_3_set_setpoint_listener);
  nh.advertise(joint_3_position_publisher);


  joint_1_current_setpoint  = 0.0; // Default setpoint
  joint_1_new_setpoint = 0.0;

  joint_2_current_setpoint  = 0.0; // Default setpoint
  joint_2_new_setpoint = 0.0;

  joint_3_current_setpoint  = 0.0; // Default setpoint
  joint_3_new_setpoint = 0.0;
}


void loop()
{
  joint_1_position = jointGet(joint_1_address, I2C_COMMAND_JOINT_GET_POSITION);

  // Report it to the publisher.
  joint_1_position_msg.data = joint_1_position;
  joint_1_position_publisher.publish(&joint_1_position_msg);

  // Now, check if we have a new setpoint from the listener.
  if (joint_1_new_setpoint != joint_1_current_setpoint)
  {
    jointSet(joint_1_address,
	     I2C_COMMAND_JOINT_SET_SETPOINT,
	     joint_1_new_setpoint);
    joint_1_current_setpoint = joint_1_new_setpoint;
  }



  joint_2_position = jointGet(joint_2_address, I2C_COMMAND_JOINT_GET_POSITION);

  // Report it to the publisher.
  joint_2_position_msg.data = joint_2_position;
  joint_2_position_publisher.publish(&joint_2_position_msg);

  // Now, check if we have a new setpoint from the listener.
  if (joint_2_new_setpoint != joint_2_current_setpoint)
  {
    jointSet(joint_2_address,
	     I2C_COMMAND_JOINT_SET_SETPOINT,
	     joint_2_new_setpoint);
    joint_2_current_setpoint = joint_2_new_setpoint;
  }


  joint_3_position = jointGet(joint_3_address, I2C_COMMAND_JOINT_GET_POSITION);

  // Report it to the publisher.
  joint_3_position_msg.data = joint_3_position;
  joint_3_position_publisher.publish(&joint_3_position_msg);

  // Now, check if we have a new setpoint from the listener.
  if (joint_3_new_setpoint != joint_3_current_setpoint)
  {
    jointSet(joint_3_address,
	     I2C_COMMAND_JOINT_SET_SETPOINT,
	     joint_3_new_setpoint);
    joint_3_current_setpoint = joint_3_new_setpoint;
  }



  nh.spinOnce();

  delay(100); 
}


double jointGet(int joint_address, int I2C_command)
{
  double get_value;
  Wire.beginTransmission(joint_address);
  Wire.write(I2C_command);
  Wire.endTransmission();

  // Now get the data
  Wire.requestFrom(joint_address, sizeof get_value);

  wireReadData(get_value);
  return get_value;
}

void jointSet(int joint_address, int I2C_command, double set_value)
{
  Wire.beginTransmission(joint_address);
  Wire.write(I2C_command);
  wireWriteData(set_value);
  Wire.endTransmission();
}

/*
 * Called when there is new msg on the setpoint topic. Updates the setpoint
 * value with the new setpoint msg value
 */
void joint1SetSetpointCb(const std_msgs::Float32& joint_1_setpoint_msg)
{
  // Copy payload from subscriber to our joint 
  joint_1_new_setpoint = joint_1_setpoint_msg.data;
}

/*
 * Called when there is new msg on the setpoint topic. Updates the setpoint
 * value with the new setpoint msg value
 */
void joint2SetSetpointCb(const std_msgs::Float32& joint_2_setpoint_msg)
{
  // Copy payload from subscriber to our joint 
  joint_2_new_setpoint = joint_2_setpoint_msg.data;
}

/*
 * Called when there is new msg on the setpoint topic. Updates the setpoint
 * value with the new setpoint msg value
 */
void joint3SetSetpointCb(const std_msgs::Float32& joint_3_setpoint_msg)
{
  // Copy payload from subscriber to our joint 
  joint_3_new_setpoint = joint_3_setpoint_msg.data;
}

