#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float alpha = 0.5; // for smoothing (0 < alpha < 1)
float gyroXFiltered = 0, gyroYFiltered = 0, gyroZFiltered = 0;
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;

// angle estimation
float angleX = 0, angleY = 0, angleZ = 0;
unsigned long previousTime = 0;

float mapToRange(float value, float minInput, float maxInput, float minOutput, float maxOutput)
{
    value = constrain(value, minInput, maxInput);
    return (value - minInput) * (maxOutput - minOutput) / (maxInput - minInput) + minOutput;
}

void calibrateGyro()
{
    int samples = 100;
    gyroXOffset = 0;
    gyroYOffset = 0;
    gyroZOffset = 0;

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

    Serial.println("Gyro Calibration Complete:");
}

void setup()
{
    Serial.begin(9600);  // for Serial Monitor
    Serial1.begin(9600); // for Bluetooth HC-06

    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip!");
        while (1)
        {
            delay(10);
        }
    }

    Serial.println("MPU6050 Found!");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

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

    // calibration offsets
    float gyroXCalibrated = g.gyro.x - gyroXOffset;
    float gyroYCalibrated = g.gyro.y - gyroYOffset;
    float gyroZCalibrated = g.gyro.z - gyroZOffset;

    // smoothing filter
    gyroXFiltered = alpha * gyroXCalibrated + (1 - alpha) * gyroXFiltered;
    gyroYFiltered = alpha * gyroYCalibrated + (1 - alpha) * gyroYFiltered;
    gyroZFiltered = alpha * gyroZCalibrated + (1 - alpha) * gyroZFiltered;

    // calculate angles by integrating angular velocity
    angleX += gyroXFiltered * deltaTime;
    angleY += gyroYFiltered * deltaTime;
    angleZ += gyroZFiltered * deltaTime;

    // mapping angles
    float angleXMapped = mapToRange(angleX, -90, 90, -10000, 10000);
    float angleYMapped = mapToRange(angleY, -90, 90, -10000, 10000);
    float angleZMapped = mapToRange(angleZ, -90, 90, -10000, 10000);

    // transmit bluetooth
    Serial1.print("X:");
    Serial1.print(angleXMapped);
    Serial1.print(",Y:");
    Serial1.print(angleYMapped);
    Serial1.print(",Z:");
    Serial1.println(angleZMapped);

    delay(200);
}
