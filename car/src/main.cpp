#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int IN1 = 25;
int IN2 = 26;
int IN3 = 27;
int IN4 = 14;

#define WHEEL_VELOCITY 255;

float angleX = 0, angleY = 0;
bool connected = false;

// callback to handle notifs
class MyClientCallback final : public BLEClientCallbacks // final derived class
{
public: // public indicate functions that can be called from outside of class, this should only apply to functions, variables should always be private/protected
  // call on creating variable with this class
  MyClientCallback() : m_isConnected(false) // m_isConnected = false on init
  {
    Serial.println("Not connected to ESP32 Server");
  }
  ~MyClientCallback()
  {
    m_isConnected = false;
  }

  void onConnect(BLEClient *client) override
  {
    m_isConnected = true;
    Serial.println("Connected to ESP32 Server");
  }

  void onDisconnect(BLEClient *client) override
  {
    m_isConnected = false;
    Serial.println("Disconnected from ESP32 Server");
  }

  // a read function for 'm_isConnected' as the variable is private and can't be accessed from outside of class
  const bool isConnected()
  {
    return m_isConnected;
  }

private:
  bool m_isConnected;
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

enum E_WHEEL : uint8_t
{
  WHEEL_RIGHT_FRONT = 0,
  WHEEL_RIGHT_BACK,
  WHEEL_LEFT_FRONT,
  WHEEL_LEFT_BACK,
  WHEEL_MAX
};

enum E_MOVE_DIR : uint8_t
{
  MOVE_FWRD = 0,
  MOVE_BWRD,
  MOVE_LEFT,
  MOVE_RGHT,
  MOVE_STOP,
};

void actMove(E_MOVE_DIR dir)
{
  switch (dir)
  {
  case MOVE_FWRD:
    ledcWrite(WHEEL_RIGHT_FRONT, 255);
    ledcWrite(WHEEL_RIGHT_BACK, 0);
    ledcWrite(WHEEL_LEFT_FRONT, 255);
    ledcWrite(WHEEL_LEFT_BACK, 0);
    break;
  case MOVE_BWRD:
    ledcWrite(WHEEL_RIGHT_FRONT, 0);
    ledcWrite(WHEEL_RIGHT_BACK, 255);
    ledcWrite(WHEEL_LEFT_FRONT, 0);
    ledcWrite(WHEEL_LEFT_BACK, 255);
    break;
  case MOVE_LEFT:
    ledcWrite(WHEEL_RIGHT_FRONT, 255);
    ledcWrite(WHEEL_RIGHT_BACK, 0);
    ledcWrite(WHEEL_LEFT_FRONT, 0);
    ledcWrite(WHEEL_LEFT_BACK, 0);
    break;
  case MOVE_RGHT:
    ledcWrite(WHEEL_RIGHT_FRONT, 0);
    ledcWrite(WHEEL_RIGHT_BACK, 0);
    ledcWrite(WHEEL_LEFT_FRONT, 255);
    ledcWrite(WHEEL_LEFT_BACK, 0);
    break;
  case MOVE_STOP:
  default:
    ledcWrite(WHEEL_RIGHT_FRONT, 0);
    ledcWrite(WHEEL_RIGHT_BACK, 0);
    ledcWrite(WHEEL_LEFT_FRONT, 0);
    ledcWrite(WHEEL_LEFT_BACK, 0);
    break;
  }
}

void judgeMove(int angleX, int angleY)
{
  E_MOVE_DIR dir;

  if (angleY > 100)
  {
    Serial.println("Moving Forward");
    dir = MOVE_FWRD;
  }
  else if (angleY < -100)
  {
    Serial.println("Moving Backward");
    dir = MOVE_BWRD;
  }
  else if (angleX < -100)
  {
    Serial.println("Turning Left");
    dir = MOVE_LEFT;
  }
  else if (angleX > 100)
  {
    Serial.println("Turning Right");
    dir = MOVE_RGHT;
  }
  else
  {
    Serial.println("Stopping");
    dir = MOVE_STOP;
  }

  actMove(dir);
}

void setup()
{
  Serial.begin(9600);

  // motor PWM setup
  // all 5kHz, 8-bit resolution
  ledcSetup(0, 5000, 8);
  ledcAttachPin(IN1, 0);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(IN2, 1);
  ledcSetup(2, 5000, 8);
  ledcAttachPin(IN3, 2);
  ledcSetup(3, 5000, 8);
  ledcAttachPin(IN4, 3);

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
            judgeMove(angleX, angleY);
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
}