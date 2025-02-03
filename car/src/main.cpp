#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int IN1 = 25;
int IN2 = 26;
int IN3 = 27;
int IN4 = 14;

#define WHEEL_ON 255
#define WHEEL_OFF 0

#define IS_MOVING_FORWARD(angleY) ((angleY) > 100)
#define IS_MOVING_BACKWARD(angleY) ((angleY) < -100)
#define IS_MOVING_LEFT(angleX) ((angleY) < -100)
#define IS_MOVING_RIGHT(angleX) ((angleY) > 100)

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
    Serial.println("Moving Forward");
    ledcWrite(WHEEL_RIGHT_FRONT, WHEEL_ON);
    ledcWrite(WHEEL_RIGHT_BACK, WHEEL_OFF);
    ledcWrite(WHEEL_LEFT_FRONT, WHEEL_ON);
    ledcWrite(WHEEL_LEFT_BACK, WHEEL_OFF);
    break;
  case MOVE_BWRD:
    Serial.println("Moving Backward");
    ledcWrite(WHEEL_RIGHT_FRONT, WHEEL_OFF);
    ledcWrite(WHEEL_RIGHT_BACK, WHEEL_ON);
    ledcWrite(WHEEL_LEFT_FRONT, WHEEL_OFF);
    ledcWrite(WHEEL_LEFT_BACK, WHEEL_ON);
    break;
  case MOVE_LEFT:
    Serial.println("Turning Left");
    ledcWrite(WHEEL_RIGHT_FRONT, WHEEL_ON);
    ledcWrite(WHEEL_RIGHT_BACK, WHEEL_OFF);
    ledcWrite(WHEEL_LEFT_FRONT, WHEEL_OFF);
    ledcWrite(WHEEL_LEFT_BACK, WHEEL_OFF);
    break;
  case MOVE_RGHT:
    Serial.println("Turning Right");
    ledcWrite(WHEEL_RIGHT_FRONT, WHEEL_OFF);
    ledcWrite(WHEEL_RIGHT_BACK, WHEEL_OFF);
    ledcWrite(WHEEL_LEFT_FRONT, WHEEL_ON);
    ledcWrite(WHEEL_LEFT_BACK, WHEEL_OFF);
    break;
  case MOVE_STOP:
  default:
    Serial.println("Stopping");
    ledcWrite(WHEEL_RIGHT_FRONT, WHEEL_OFF);
    ledcWrite(WHEEL_RIGHT_BACK, WHEEL_OFF);
    ledcWrite(WHEEL_LEFT_FRONT, WHEEL_OFF);
    ledcWrite(WHEEL_LEFT_BACK, WHEEL_OFF);
    break;
  }
}

void judgeMove(int angleX, int angleY)
{
  E_MOVE_DIR dir;

  if (IS_MOVING_FORWARD(angleY))
  {
    dir = MOVE_FWRD;
  }
  else if (IS_MOVING_BACKWARD(angleY))
  {
    dir = MOVE_BWRD;
  }
  else if (IS_MOVING_LEFT(angleX))
  {
    dir = MOVE_LEFT;
  }
  else if (IS_MOVING_RIGHT(angleX))
  {
    dir = MOVE_RGHT;
  }
  else
  {
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