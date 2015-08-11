#include <Wire.h>
#include <WireData.h>

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

void setup()
{
  Wire.begin();
  Serial.begin(9600);
}


void loop()
{
  float currentSetpoint;

  // Request position from Joint1
  Wire.beginTransmission(addressJoint1);
  Wire.write(I2C_COMMAND_JOINT_GET_SETPOINT);
  Wire.endTransmission();

  // Now get the data
  Wire.requestFrom(addressJoint1, sizeof currentSetpoint);

  wireReadData(currentSetpoint);

  Serial.print("Setpoint: ");
  Serial.println(currentSetpoint);

  // Send new position, if received over serial.
  if (Serial.available() > 0)
  {
    float new_setpoint = Serial.parseFloat();

    // Send command and data.
    Wire.beginTransmission(addressJoint1);
    Wire.write(I2C_COMMAND_JOINT_SET_SETPOINT);
    wireWriteData(new_setpoint);
    Wire.endTransmission();
  }
  delay(100);
}

