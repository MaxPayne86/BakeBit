// BakeBit Firmware

#include <Wire.h>

#define SLAVE_ADDRESS 0x04

void receiveData(int);
void sendData();

int cmd[5];
int index = 0;
int flag = 0;
byte val = 0, b[21];
int aRead = 0;
int run_once;

void setup()
{
  Serial.begin(115200);         // start serial for output
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.print("setup\n");
}

void loop()
{
  long dur, RangeCm;
  if (index == 4)
  {
    flag = 1;

    //Digital Read
    if (cmd[0] == 1)
      val = digitalRead(cmd[1]);

    //Digital Write
    else if (cmd[0] == 2)
      digitalWrite(cmd[1], cmd[2]);

    //Analog Read
    else if (cmd[0] == 3)
    {
      aRead = analogRead(cmd[1]);
      b[1] = aRead / 256;
      b[2] = aRead % 256;
    }

    //Set up Analog Write
    else if (cmd[0] == 4)
      analogWrite(cmd[1], cmd[2]);

    //Set up pinMode
    else if (cmd[0] == 5)
      pinMode(cmd[1], cmd[2]);

    //Firmware version
    else if (cmd[0] == 8)
    {
      b[1] = 1;
      b[2] = 2;
      b[3] = 6+1;
    }
  }
}

void receiveData(int byteCount)
{
  while (Wire.available())
  {
    if (Wire.available() == 4)
    {
      flag = 0;
      index = 0;
      run_once = 1;
    }
    cmd[index++] = Wire.read();
  }
}

// callback for sending data
void sendData()
{
  if (cmd[0] == 1)
    Wire.write(val);
  else if (cmd[0] == 3)
    Wire.write(b, 3);
  else if (cmd[0] == 8)
    Wire.write(b, 4);
}
