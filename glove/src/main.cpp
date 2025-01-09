#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// MPU6050 object
Adafruit_MPU6050 mpu;

// BLE Server and Characteristic
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;

// Connection status
bool deviceConnected = false;

// Variables for angles
float angleX = 0, angleY = 0, angleZ = 0;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;
float alpha = 0.5; // smoothing factor
float gyroXFiltered = 0, gyroYFiltered = 0, gyroZFiltered = 0;
unsigned long previousTime = 0;

// Function for mapping values to a range
float mapToRange(float value, float minInput, float maxInput, float minOutput, float maxOutput)
{
    value = constrain(value, minInput, maxInput);
    return (value - minInput) * (maxOutput - minOutput) / (maxInput - minInput) + minOutput;
}

// Function to calibrate the gyroscope
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

// Callback class for BLE server events
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
        BLEDevice::startAdvertising(); // Restart advertising
    }
};

void setup()
{
    Serial.begin(9600);

    // Initialize BLE
    BLEDevice::init("ESP32_Gyro_Sender");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create BLE Service
    BLEService *pService = pServer->createService("0000ffe0-0000-1000-8000-00805f9b34fb");

    // Create BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        "0000ffe1-0000-1000-8000-00805f9b34fb",
        BLECharacteristic::PROPERTY_NOTIFY);

    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->start();
    Serial.println("BLE Advertising started!");

    // Initialize MPU6050
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip!");
        while (1)
            ;
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Calibrate the gyroscope
    calibrateGyro();
    previousTime = millis();
}

void loop()
{
    // Get gyroscope data
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    // Calibrate and filter gyroscope data
    float gyroXCalibrated = g.gyro.x - gyroXOffset;
    float gyroYCalibrated = g.gyro.y - gyroYOffset;
    float gyroZCalibrated = g.gyro.z - gyroZOffset;

    gyroXFiltered = alpha * gyroXCalibrated + (1 - alpha) * gyroXFiltered;
    gyroYFiltered = alpha * gyroYCalibrated + (1 - alpha) * gyroYFiltered;
    gyroZFiltered = alpha * gyroZCalibrated + (1 - alpha) * gyroZFiltered;

    // Calculate angles by integrating angular velocity
    angleX += gyroXFiltered * deltaTime;
    angleY += gyroYFiltered * deltaTime;
    angleZ += gyroZFiltered * deltaTime;

    // Map angles to a suitable range
    float angleXMapped = mapToRange(angleX, -90, 90, -10000, 10000);
    float angleYMapped = mapToRange(angleY, -90, 90, -10000, 10000);

    // Transmit data via BLE
    if (deviceConnected)
    {
        String data = "X:" + String(angleXMapped) + ",Y:" + String(angleYMapped) + "\n";
        pCharacteristic->setValue(data.c_str());
        pCharacteristic->notify();
    }

    delay(200);
}
