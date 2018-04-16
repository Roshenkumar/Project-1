#include <seeed_pwm.h>
#include <motordriver_4wd.h>
#include <motordriver_4wd_dfs.h>

byte serIn;

void setup() 
{
  Serial.begin(9600);
  MOTOR.init();
  MOTOR.setSpeedDir1(0, DIRR);
  MOTOR.setSpeedDir2(0, DIRF);
}

void loop() 
{
  if (Serial.available())
  {
    while (Serial.available() > 0)
    {
      serIn = Serial.read();
      Serial.write(serIn);

      switch (serIn)
      {
        case 'c': //go straight
          MOTOR.setSpeedDir1(15, DIRR);   // Left Motor Inverted
          MOTOR.setSpeedDir2(15, DIRF);   // Right Motor Normal
          Serial.write("\n");
          break;
          
        case 'l': //go left
          MOTOR.setSpeedDir1(20, DIRF);
          MOTOR.setSpeedDir2(20, DIRF);
          Serial.write("\n");
          break;
          
        case 'r': //go right
          MOTOR.setSpeedDir1(20, DIRR);
          MOTOR.setSpeedDir2(20, DIRR);
          Serial.write("\n");
          break;
          
        case 'q': //look left
          MOTOR.setSpeedDir1(10, DIRF);
          MOTOR.setSpeedDir2(10, DIRF);
          Serial.write("\n");
          break;
          
        case 'e': //look right
          MOTOR.setSpeedDir1(10, DIRR);
          MOTOR.setSpeedDir2(10, DIRR);
          Serial.write("\n");
          break;
          
        default:
          MOTOR.setSpeedDir1(0, DIRR);
          MOTOR.setSpeedDir2(0, DIRF);
          Serial.write("\n");
          break;
      }
    }  
    Serial.println();
  }
}

