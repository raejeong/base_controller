#include <Wire.h>
#include <I2C_Anything.h>

void setup()
{
  Wire.begin();
  Serial.begin(9600);
}


void loop()
{
  Wire.requestFrom(1,3);
  int enc_reading = Wire.read();
  Serial.print("Position: ");
  Serial.print(enc_reading);
  Serial.println(" ");
  
  if(Serial.available() > 0)
  {
    double new_set_point = Serial.parseFloat();
    Wire.beginTransmission(1);
    I2C_writeAnything(new_set_point);
    Wire.endTransmission();
  }
  delay(100); 
}
