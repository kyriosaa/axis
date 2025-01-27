#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int in1 = 25;
int in2 = 26;
int in3 = 27;
int in4 = 14;

float angleX = 0, angleY = 0;
bool connected = false;

// callback to handle notifs
class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *client)
  {
    connected = true;
    Serial.println("Connected to ESP32 Server");
  }

  void onDisconnect(BLEClient *client)
  {
    connected = false;
    Serial.println("Disconnected from ESP32 Server");
  }
};

bool parseBluetoothData(String data, float &x, float &y)
{
  int xIndex = data.indexOf("X:");
  int yIndex = data.indexOf(",Y:");
  if (xIndex == -1 || yIndex == -1)
    return false;
  x = data.substring(xIndex + 2, yIndex).toFloat();
  y = data.substring(yIndex + 3).toFloat();
  return true;
}

void Forward()
{
  ledcWrite(0, 255);
  ledcWrite(1, 0);
  ledcWrite(2, 255);
  ledcWrite(3, 0);
}

void Backward()
{
  ledcWrite(0, 0);
  ledcWrite(1, 255);
  ledcWrite(2, 0);
  ledcWrite(3, 255);
}

void Left()
{
  ledcWrite(0, 255);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

void Right()
{
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 255);
  ledcWrite(3, 0);
}

void Stop()
{
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  ledcWrite(2, 0);
  ledcWrite(3, 0);
}

void setup()
{
  Serial.begin(9600);

  // motor PWM setup
  // all 5kHz, 8-bit resolution
  ledcSetup(0, 5000, 8);
  ledcAttachPin(in1, 0);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(in2, 1);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(in3, 2);
  ledcSetup(3, 5000, 8);
  ledcAttachPin(in4, 3);

  // init BLE
  BLEDevice::init("ESP32_Client");
  BLEClient *client = BLEDevice::createClient();
  client->setClientCallbacks(new MyClientCallback());

  // connect to server
  BLEAddress serverAddress("20:43:A8:65:52:1E");
  if (client->connect(serverAddress))
  {
    Serial.println("Connected to server");
    BLERemoteService *service = client->getService("0000ffe0-0000-1000-8000-00805f9b34fb");
    if (service)
    {
      BLERemoteCharacteristic *characteristic = service->getCharacteristic("0000ffe1-0000-1000-8000-00805f9b34fb");
      if (characteristic)
      {
        characteristic->registerForNotify([](BLERemoteCharacteristic *characteristic, uint8_t *data, size_t length, bool isNotify)
                                          {
          String receivedData = String((char *)data).substring(0, length);
          if (parseBluetoothData(receivedData, angleX, angleY))
          {
            Serial.printf("Parsed: X=%.2f, Y=%.2f\n", angleX, angleY);
            if (angleY < 0)
            {
              Serial.println("Moving Backward");
              Backward();
            }
            else if (angleY > 0)
            {
              Serial.println("Moving Forward");
              Forward();
            }
            else if (angleX < 0)
            {
              Serial.println("Turning Left");
              Left();
            }
            else if (angleX > 0)
            {
              Serial.println("Turning Right");
              Right();
            }
            else
            {
              Serial.println("Stopping");
              Stop();
            }
          }
          else
          {
            Serial.println("Failed to connect to server");
          } });
      }
    }
  }
}

void loop()
{
  if (!connected)
  {
    Serial.println("Attempting to reconnect...");
    BLEDevice::getScan()->start(5, false);
  }
  delay(2000);
