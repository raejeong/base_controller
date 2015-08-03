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

const int addressJoint1 = 0x08;

/*
 * Function prototypes
 */
void positionCb(const std_msgs::Float32& position_msg);


/*
 * Variable to store the current and new setpoint
 */
float current_setpoint;
float new_setpoint;


/*
 * ROS node handle, encoder reading, subscriber and publisher
 */
std_msgs::Float32 joint_position_msg;
ros::Subscriber<std_msgs::Float32> setpoint_listener("position", &setpointCb);
ros::Publisher pub_position("enc", &joint_position_msg);
ros::NodeHandle nh;


void setup()
{
  Wire.begin();

  nh.initNode();
  nh.subscribe(setpoint_listener);
  nh.advertise(pub_position);

  current_setpoint  = 0.0; // Default setpoint
  new_setpoint = 0.0;
}


void loop()
{
  double current_position;

  // Read from the joint's position register.
  Wire.beginTransmission(addressJoint1);
  Wire.write(I2C_COMMAND_JOINT_GET_POSITION);
  Wire.endTransmission();
  
  Wire.requestFrom(addressJoint1, sizeof current_position));

  wireReadData(current_position);

  // Report it to the publisher.
  joint_position_msg.data = current_position;
  pub_position.publish(&joint_position_msg);

  // Now, check if we have a new setpoint from the listener.
  if (new_setpoint != current_setpoint)
  {
    Wire.beginTransmission(addressJoint1);
    Wire.write(I2C_COMMAND_JOINT_SET_SETPOINT);
    wireWriteData(new_setpoint);
    Wire.endTransmission();
    current_setpoint = new_setpoint;
  }

  nh.spinOnce();

  delay(100); 
}


/*
 * Called when there is new msg on the setpoint topic. Updates the setpoint
 * value with the new setpoint msg value
 */
void setpointCb(const std_msgs::Float32& setpoint_msg)
{
  // Copy payload from subscriber to our joint 
  new_setpoint = setpoint_msg.data;
}

