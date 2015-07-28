#include <Wire.h>
#include <I2C_Anything.h>
#include <ros.h>
#include <std_msgs/Float32.h>


float new_position; 
std_msgs::Float32 enc_reading_msg;

/*
 * Called when there is new msg on the position topic. Updates the position value with the new position msg value
 */
void positionCb( const std_msgs::Float32& position_msg) {
  new_position = position_msg.data;
}

ros::Subscriber<std_msgs::Float32> position_listener("position", &positionCb ); // position subscriber with positionCb as the call back function
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
