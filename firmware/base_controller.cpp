/*
 * Base Controller
 * Base Controller that connects the joint actuators to ROS master
 * Author :: Rae Jeong
 * Email  :: raychanjeongjeong@gmail.com
 */
#include <Wire.h>
#include <I2C_Anything.h>
#include <ros.h>
#include <std_msgs/Float32.h>


/*
 * Function prototypes
 */
void positionCb(const std_msgs::Float32& position_msg);


/*
 * Variable to store the new position
 */
float new_position; 


/*
 * ROS node handle, encoder reading, subscriber and publisher
 */
std_msgs::Float32 enc_reading_msg;
ros::Subscriber<std_msgs::Float32> position_listener("position", &positionCb);
ros::Publisher pub_enc("enc", &enc_reading_msg);
ros::NodeHandle nh;


void setup()
{
  Wire.begin();

  nh.initNode();
  nh.subscribe(position_listener);
  nh.advertise(pub_enc);

  new_position  = 0.0; // Default position
}


void loop()
{
  Wire.requestFrom(1,3);
  int enc_reading = Wire.read();

  enc_reading_msg.data = (double)enc_reading;
  pub_enc.publish(&enc_reading_msg);
  
  double new_set_point = new_position;
  Wire.beginTransmission(1);
  I2C_writeAnything(new_set_point);
  Wire.endTransmission();

  nh.spinOnce();

  delay(100); 
}


/*
 * Called when there is new msg on the position topic. Updates the position
 * value with the new position msg value
 */
void positionCb(const std_msgs::Float32& position_msg)
{
  new_position = position_msg.data;
}
