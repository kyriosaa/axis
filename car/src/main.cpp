#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// bluetooth config
#define BLE_CLIENT_NAME     "ESP32_Client"
#define BLE_SERVER_ADDR     "20:43:A8:65:52:1E"
#define BLE_SERVICE_UUID    "0000ffe0-0000-1000-8000-00805f9b34fb"
#define BLE_CHAR_UUID       "0000ffe1-0000-1000-8000-00805f9b34fb"

// motor driver
#define IN1                 25
#define IN2                 26
#define IN3                 27
#define IN4                 14

// 255 is the max speed for the wheel spin (kinda like an LED has 0->255)
#define WHEEL_ON            255
#define WHEEL_OFF           0

// serial and timing
#define SERIAL_BAUD_RATE    9600  // i usually use 9600 or 115200
#define PWM_FREQ            5000
#define PWM_RESOLUTION      8
#define RECONNECT_DELAY_MS  2000
#define BLE_SCAN_SEC        5

// data parsing const
#define X_PREFIX_LEN        2
#define Y_PREFIX_LEN        3

// movement
#define MOVEMENT_THRESHOLD  100
#define NEGATIVE_THRESHOLD -100
#define IS_MOVING_FORWARD   (angleY) ((angleY) >  MOVEMENT_THRESHOLD)
#define IS_MOVING_BACKWARD  (angleY) ((angleY) <  NEGATIVE_THRESHOLD)
#define IS_TURNING_LEFT     (angleX) ((angleX) <  NEGATIVE_THRESHOLD)
#define IS_TURNING_RIGHT    (angleX) ((angleX) >  MOVEMENT_THRESHOLD)

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
  // -1 means BLE data not found
  if (xIndex == -1 || yIndex == -1)
    return false;
  x = data.substring(xIndex + X_PREFIX_LEN, yIndex).toFloat();
  y = data.substring(yIndex + Y_PREFIX_LEN).toFloat();
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
    ledcWrite(WHEEL_RIGHT_FRONT,  WHEEL_ON);
    ledcWrite(WHEEL_RIGHT_BACK,   WHEEL_OFF);
    ledcWrite(WHEEL_LEFT_FRONT,   WHEEL_ON);
    ledcWrite(WHEEL_LEFT_BACK,    WHEEL_OFF);
    break;
  case MOVE_BWRD:
    Serial.println("Moving Backward");
    ledcWrite(WHEEL_RIGHT_FRONT,  WHEEL_OFF);
    ledcWrite(WHEEL_RIGHT_BACK,   WHEEL_ON);
    ledcWrite(WHEEL_LEFT_FRONT,   WHEEL_OFF);
    ledcWrite(WHEEL_LEFT_BACK,    WHEEL_ON);
    break;
  case MOVE_LEFT:
    Serial.println("Turning Left");
    ledcWrite(WHEEL_RIGHT_FRONT,  WHEEL_ON);
    ledcWrite(WHEEL_RIGHT_BACK,   WHEEL_OFF);
    ledcWrite(WHEEL_LEFT_FRONT,   WHEEL_OFF);
    ledcWrite(WHEEL_LEFT_BACK,    WHEEL_OFF);
    break;
  case MOVE_RGHT:
    Serial.println("Turning Right");
    ledcWrite(WHEEL_RIGHT_FRONT,  WHEEL_OFF);
    ledcWrite(WHEEL_RIGHT_BACK,   WHEEL_OFF);
    ledcWrite(WHEEL_LEFT_FRONT,   WHEEL_ON);
    ledcWrite(WHEEL_LEFT_BACK,    WHEEL_OFF);
    break;
  case MOVE_STOP:
  default:
    Serial.println("Stopping");
    ledcWrite(WHEEL_RIGHT_FRONT,  WHEEL_OFF);
    ledcWrite(WHEEL_RIGHT_BACK,   WHEEL_OFF);
    ledcWrite(WHEEL_LEFT_FRONT,   WHEEL_OFF);
    ledcWrite(WHEEL_LEFT_BACK,    WHEEL_OFF);
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
  else if (IS_TURNING_LEFT(angleX))
  {
    dir = MOVE_LEFT;
  }
  else if (IS_TURNING_RIGHT(angleX))
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
  Serial.begin(SERIAL_BAUD_RATE); 

  // motor PWM setup
  // all 5kHz, 8-bit resolution
  // 0, 1, 2, 3 are the PWM channels respectively
  ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(IN1, 0);
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(IN2, 1);
  ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(IN3, 2);
  ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(IN4, 3);

  // init BLE
  BLEDevice::init(BLE_CLIENT_NAME);
  BLEClient *client = BLEDevice::createClient();
  client->setClientCallbacks(new MyClientCallback());

  // connect to server
  BLEAddress serverAddress(BLE_SERVER_ADDR);
  if (client->connect(serverAddress))
  {
    Serial.println("Connected to server");
    BLERemoteService *service = client->getService(BLE_SERVICE_UUID);
    if (service)
    {
      BLERemoteCharacteristic *characteristic = service->getCharacteristic(BLE_CHAR_UUID);
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
    BLEDevice::getScan()->start(BLE_SCAN_SEC, false);
  }
  delay(RECONNECT_DELAY_MS);
}