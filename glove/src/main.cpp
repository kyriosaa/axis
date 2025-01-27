#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

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
    int samples = 100;
    for (int i = 0; i < samples; i++)
    {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        gyroXOffset += g.gyro.x;
        gyroYOffset += g.gyro.y;
        gyroZOffset += g.gyro.z;
        delay(10);
    }
    gyroXOffset /= samples;
    gyroYOffset /= samples;
    gyroZOffset /= samples;
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
    Serial.begin(9600);

    // init BLE
    BLEDevice::init("ESP32_Gyro_Sender");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService("0000ffe0-0000-1000-8000-00805f9b34fb");

    pCharacteristic = pService->createCharacteristic(
        "0000ffe1-0000-1000-8000-00805f9b34fb",
        BLECharacteristic::PROPERTY_NOTIFY);

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
    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    // filter gyro data
    float gyroXCalibrated = g.gyro.x - gyroXOffset;
    float gyroYCalibrated = g.gyro.y - gyroYOffset;
    float gyroZCalibrated = g.gyro.z - gyroZOffset;

    gyroXFiltered = alpha * gyroXCalibrated + (1 - alpha) * gyroXFiltered;
    gyroYFiltered = alpha * gyroYCalibrated + (1 - alpha) * gyroYFiltered;
    gyroZFiltered = alpha * gyroZCalibrated + (1 - alpha) * gyroZFiltered;

    angleX += gyroXFiltered * deltaTime;
    angleY += gyroYFiltered * deltaTime;
    angleZ += gyroZFiltered * deltaTime;

    // map angles to a specific range
    float angleXMapped = mapToRange(angleX, -90, 90, -10000, 10000);
    float angleYMapped = mapToRange(angleY, -90, 90, -10000, 10000);

    // transmit data
    if (deviceConnected)
    {
        String data = "X:" + String(angleXMapped) + ",Y:" + String(angleYMapped) + "\n";
        pCharacteristic->setValue(data.c_str());
        pCharacteristic->notify();
    }

    delay(200);
}
