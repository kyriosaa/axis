#include <Arduino.h>
#include <IRremote.hpp>

const int RemotePin = 8;

int in1 = 3;
int in2 = 5;
int in3 = 6;
int in4 = 9;

void Forward()
{
  Serial.println("Moving Forward");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void Backward()
{
  Serial.println("Moving Backward");
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void Left()
{
  Serial.println("Turning Left");
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void Right()
{
  Serial.println("Turning Right");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void Stop()
{
  Serial.println("Stopping");
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void setup()
{
  Serial.begin(9600);

  // prints the program
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

  IrReceiver.begin(RemotePin, DISABLE_LED_FEEDBACK);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void loop()
{
  if (IrReceiver.available())
  {
    IrReceiver.initDecodedIRData();
    IrReceiver.decodeHash();
    IrReceiver.resume(); // early enable receiving of the next IR frame

    Serial.print("Received command: ");
    Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);

    unsigned long receivedCommand = IrReceiver.decodedIRData.decodedRawData;

    if (receivedCommand == 0x3D9AE3F7) // UP button
    {
      Forward();
    }
    else if (receivedCommand == 0x1BC0157B) // DOWN button
    {
      Backward();
    }
    else if (receivedCommand == 0x8C22657B) // LEFT button
    {
      Left();
    }
    else if (receivedCommand == 0x449E79F) // RIGHT button
    {
      Right();
    }
    else if (receivedCommand == 0x488F3CBB) // STOP button
    {
      Stop();
    }
    else
    {
      Serial.println("Unknown command");
    }
    delay(200);
    IrReceiver.resume();
  }
}
