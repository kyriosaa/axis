#include <Arduino.h>
#include <SoftwareSerial.h>

// Define SoftwareSerial pins for HM-10
SoftwareSerial HM10(10, 11); // RX, TX (connect TX of HM-10 to pin 10, RX to pin 11)

// Motor control pins
int in1 = 3;
int in2 = 5;
int in3 = 6;
int in4 = 9;

// Bluetooth data storage
float angleX = 0; // Received X value
float angleY = 0; // Received Y value

bool atMode = false; // Flag to toggle between AT command mode and normal mode

// Function to parse Bluetooth data in the format "X:<value>,Y:<value>"
bool parseBluetoothData(String data, float &x, float &y)
{
  int xIndex = data.indexOf("X:");
  int yIndex = data.indexOf(",Y:");

  if (xIndex == -1 || yIndex == -1)
    return false; // Invalid format

  x = data.substring(xIndex + 2, yIndex).toFloat();
  y = data.substring(yIndex + 3).toFloat();

  return true; // Successfully parsed
}

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
  Serial.begin(9600); // For communication with Serial Monitor
  HM10.begin(9600);   // For communication with HM-10

  // Motor pins setup
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  Serial.println("Enter 'AT' in Serial Monitor to enable AT command mode.");
}

void loop()
{
  // Check for input from Serial Monitor
  if (Serial.available())
  {
    String serialInput = Serial.readStringUntil('\n');
    serialInput.trim();

    if (serialInput == "AT")
    {
      atMode = !atMode; // Toggle AT command mode
      Serial.println(atMode ? "AT mode enabled. Enter AT commands:" : "AT mode disabled.");
    }
    else if (atMode)
    {
      // Send AT command to HM-10
      HM10.println(serialInput);
      delay(100);

      // Read and print the HM-10 response
      if (HM10.available())
      {
        String response = HM10.readStringUntil('\n');
        Serial.println("HM-10 Response: " + response);
      }
    }
  }

  // If not in AT mode, handle Bluetooth motor control
  if (!atMode && HM10.available())
  {
    String data = HM10.readStringUntil('\n');           // Read until newline
    Serial.println("Bluetooth Data Received: " + data); // Debug: print received data

    // Parse Bluetooth data
    if (parseBluetoothData(data, angleX, angleY))
    {
      Serial.print("Parsed angleX: ");
      Serial.print(angleX); // Debug: print parsed angleX
      Serial.print(" angleY: ");
      Serial.println(angleY); // Debug: print parsed angleY

      // Control car based on Bluetooth input
      if (angleY < 0)
      {
        Backward();
      }
      else if (angleY > 0)
      {
        Forward();
      }

      if (angleX < 0)
      {
        Left();
      }
      else if (angleX > 0)
      {
        Right();
      }

      if (angleX == 0 && angleY == 0)
      {
        Stop();
      }
    }
    else
    {
      Serial.println("Failed to parse Bluetooth data"); // Debug: print failure message
    }
  }
}
