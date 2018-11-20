#include "millis.h"
#include "quaternion.h"
#include "sensor_fusion.h"
#include "mbed.h"
#include <math.h>

const float pi = 3.14159265359;
const float GX_BIAS = -75;
const float GY_BIAS = 63;
const float GZ_BIAS = -9;
const float AX_BIAS = 1100;
const float AY_BIAS = 200;
const float AZ_BIAS = 16100;

int main()
{
    //Serial setup
    Serial pc(USBTX, USBRX);
    pc.baud(115200);

    //initialize and start the MPU
    MPU6050 mpu(SDA, SCL);
    mpu.start();
    
    //THIS WILL SAVE US FROM MILLIS!!!
    Timer timer;
    timer.start();

    //this unit vector is unchanging
    vector initVector = {0, 0, 1};
    
    while(1)
    {
        if (timer.read_ms() >= 100)
        {
            //get the raw data
            float data[6];
            mpu.read_raw(&data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);
    
            //account for bias and normalize the vector
            vector orientation = {data[3] - AX_BIAS, data[4] - AY_BIAS, data[5] - AZ_BIAS + 16384};
            vector norm_orientation;
            vector_normalize(&orientation, &norm_orientation);
            
            //get the three angles
            float xAngle = (data[0] - GX_BIAS);
            float yAngle = (data[1] - GY_BIAS);
            float zAngle = (data[2] - GZ_BIAS);
            vector rawGyro = {xAngle, yAngle, zAngle};
            vector placeholder;
            //returns magnitude of angular velocity
            float rawAngle = vector_normalize(&rawGyro, &placeholder);
            float initAngle = vector_normalize(&initVector, &placeholder);
            //angular velocity to angle
            rawAngle *= timer.read_ms();
            initAngle *= timer.read_ms();
            
            float dotProduct = (initVector.x * xAngle + initVector.y * yAngle + initVector.z * zAngle) * timer.read_ms();
            float magnitudes = rawAngle * initAngle;
            float cosAngle = dotProduct/magnitudes;
            float angle = acos(cosAngle) * 180 / pi;
            
            quaternion quatGyro;
            quaternion_create(&initVector, -angle, &quatGyro);
            vector gyro;
            quaternion_rotate(&initVector, &quatGyro, &gyro);
            
            initVector.x = gyro.x;
            initVector.y = gyro.y;
            initVector.z = gyro.z;
            
            pc.printf("<gyro>: %f, %f, %f\r\n", gyro.x, gyro.y, gyro.z);
    
            pc.printf("<accel>: %f, %f, %f\r\n", norm_orientation.x, norm_orientation.y, norm_orientation.z);
            
            //pc.printf("<gyro>: %f, %f, %f\r\n", data[0], data[1], data[2]);
            //pc.printf("<accel>: %f, %f, %f\r\n", data[3], data[4], data[5]);
    
            timer.reset();
        }
    }
}
