#include "millis.h"
#include "quaternion.h"
#include "sensor_fusion.h"
#include "mbed.h"

const float PI = 3.14159265359;
const float GX_BIAS = -75;
const float GY_BIAS = 63;
const float GZ_BIAS = -9;
const float AX_BIAS = 1100;
const float AY_BIAS = 200;
const float AZ_BIAS = 16100;

Timer timer;

int main()
{
    //Serial setup
    Serial pc(USBTX, USBRX);
    pc.baud(115200);

    //initialize and start the MPU
    MPU6050 mpu(SDA, SCL);
    mpu.start();
    
    //THIS WILL SAVE US FROM MILLIS!!!
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

            //find the magnitude of the angular velocities
            float mag = vector_normalize(&rawGyro, &rawGyro);
            
            //convert to angle value
            float angle = mag / 16.4 * timer.read() * 180 * PI;

            //create quaternion then rotate it around initial vector
            quaternion curr;
            quaternion_create(&rawGyro, -angle, &curr);
            vector tempVector;
            quaternion_rotate(&initVector, &curr, &tempVector);
            
            
            float alpha = 0.5;
            vector mult;
            vector rotate;
            quaternion quat;
            vector final;
            vector normFinal;
            
            //implement normalize(alpha * a + (1 - alpha) * rotate(n))
            quaternion_create(&tempVector, -angle, &quat);  //n
            vector_multiply(&orientation, alpha, &mult);
            quaternion_rotate(&initVector, &quat, &rotate);
            vector_add(&mult, &rotate, &final);
            vector_normalize(&final, &normFinal);
            
            pc.printf("%f, %f, %f\r\n", tempVector.x, tempVector.y, tempVector.z);
//            pc.printf("%f %f %f %f %f %f %f %f %f\r\n", 
//            norm_orientation.x, norm_orientation.y, norm_orientation.z,
//            tempVector.x. tempVector.y, tempVector.z,
//            normFinal.x, normFinal.y, normFinal.z);
            
            initVector = tempVector;
            timer.reset();
        }
    }
}
