#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// serial config
#define SERIAL_BAUD_RATE 9600 // i usually do 9600 or 112500

// I2C config
#define I2C_SDA_PIN         2
#define I2C_SCL_PIN         15

// bluetooth config
#define BLE_DEVICE_NAME     "ESP32_Gyro_Sender"
#define BLE_SERVICE_UUID    "0000ffe0-0000-1000-8000-00805f9b34fb"
#define BLE_CHAR_UUID       "0000ffe1-0000-1000-8000-00805f9b34fb"

// gyro config
#define CALIBRATION_SAMPLES 100
#define CALIBRATION_DELAY   10     // ms

// timing
#define LOOP_DELAY          200    // ms
#define MS_CONVERT          1000.0f

// angle mapping
#define ANGLE_MIN_INPUT    -90.0f
#define ANGLE_MAX_INPUT     90.0f
#define ANGLE_MIN_OUTPUT   -10000.0f
#define ANGLE_MAX_OUTPUT    10000.0f

Adafruit_MPU6050 mpu;
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;

bool deviceConnected = false;

// angle variables
float angleX = 0, angleY = 0, angleZ = 0;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
float gyroXFiltered = 0, gyroYFiltered = 0, gyroZFiltered = 0;
float alpha = 0.5; // smoothing
unsigned long previousTime = 0;

// mapping values to a range
float mapToRange(float value, float minInput, float maxInput, float minOutput, float maxOutput)
{
    value = constrain(value, minInput, maxInput);
    return (value - minInput) * (maxOutput - minOutput) / (maxInput - minInput) + minOutput;
}

// calibrating the gyroscope
void calibrateGyro()
{
    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        gyroXOffset += g.gyro.x;
        gyroYOffset += g.gyro.y;
        gyroZOffset += g.gyro.z;
        delay(CALIBRATION_DELAY);
    }
    gyroXOffset /= CALIBRATION_SAMPLES;
    gyroYOffset /= CALIBRATION_SAMPLES;
    gyroZOffset /= CALIBRATION_SAMPLES;
    Serial.println("Gyro Calibration Complete!");
}

// callback for BLE server events
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        Serial.println("Device connected!");
    }

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        Serial.println("Device disconnected!");
        BLEDevice::startAdvertising();
    }
};

void setup()
{
    Serial.begin(SERIAL_BAUD_RATE);

    // init I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // init BLE
    BLEDevice::init(BLE_DEVICE_NAME);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(BLE_SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(BLE_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);

    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->start();
    Serial.println("BLE Advertising started!");

    // init MPU6050
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip!");
        while (1)
            ;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // calibrate gyro
    calibrateGyro();
    previousTime = millis();
}

void loop()
{
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / MS_CONVERT;
    previousTime = currentTime;

    // filter gyro data
    float gyroXCalibrated = g.gyro.x - gyroXOffset;
    float gyroYCalibrated = g.gyro.y - gyroYOffset;

    gyroXFiltered = alpha * gyroXCalibrated + (1 - alpha) * gyroXFiltered;
    gyroYFiltered = alpha * gyroYCalibrated + (1 - alpha) * gyroYFiltered;

    angleX += gyroXFiltered * deltaTime;
    angleY += gyroYFiltered * deltaTime;

    // map angles to a specific range
    float angleXMapped = mapToRange(angleX, ANGLE_MIN_INPUT, ANGLE_MAX_INPUT, ANGLE_MIN_OUTPUT, ANGLE_MAX_OUTPUT);
    float angleYMapped = mapToRange(angleY, ANGLE_MIN_INPUT, ANGLE_MAX_INPUT, ANGLE_MIN_OUTPUT, ANGLE_MAX_OUTPUT);

    // transmit data
    if (deviceConnected)
    {
        String data = "X:" + String(angleXMapped) + ",Y:" + String(angleYMapped) + "\n";
        pCharacteristic->setValue(data.c_str());
        pCharacteristic->notify();
    }

    delay(LOOP_DELAY);
}
