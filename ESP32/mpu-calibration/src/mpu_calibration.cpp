#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define ACCEL_DIVIDER 16384.0
#define GYRO_DIVIDER 131.0

/*#define AX_OFFSET (-2925)
#define AY_OFFSET 154
#define AZ_OFFSET 1335
#define GX_OFFSET 51
#define GY_OFFSET (-94)
#define GZ_OFFSET 9*/

MPU6050 mpu;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;


void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing I2C devices...");
    mpu.initialize();
    mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_2);
    mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_250);
    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    //calibrate device
    mpu.CalibrateAccel(10);
    mpu.CalibrateGyro(10);
    mpu.PrintActiveOffsets();

    Serial.println();
}

void loop() {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Serial.print("a/g:\t");
    Serial.print(ax/ACCEL_DIVIDER); Serial.print("\t");
    Serial.print(ay/ACCEL_DIVIDER); Serial.print("\t");
    Serial.print(az/ACCEL_DIVIDER); Serial.print("\t");
    Serial.print(gx/GYRO_DIVIDER); Serial.print("\t");
    Serial.print(gy/GYRO_DIVIDER); Serial.print("\t");
    Serial.println(gz/GYRO_DIVIDER);

    delay(1000);

}