#include <Wire.h>

byte val = 0;

void setup()
{
  Wire.begin(0x48);
  Serial.begin(9600);  // start serial for output
}
byte i = 0;
void loop()
{

  Wire.beginTransmission(0x48);
  Wire.write(byte(i));            // sends value byte
  Wire.endTransmission();     // stop transmitting

  Wire.requestFrom(0x48, 1);
  while (Wire.available()) {
    byte c = Wire.read();
    if (i == c)
      Serial.println(c, HEX);
    else {
      Serial.print("ERROR: ");
      Serial.print(c, HEX);
      Serial.print(" != ");
      Serial.println(i, HEX);
      return;
    }
  }
  i++;
}
