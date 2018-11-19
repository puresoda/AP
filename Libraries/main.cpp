#include "millis.h"
#include "quaternion.h"
#include "sensor_fusion.h"
#include "mbed.h"

const float pi = 3.14159265359;
const float GX_BIAS = -77;
const float GY_BIAS = 64;
const float GZ_BIAS = -10;
const float AX_BIAS = -1060;
const float AY_BIAS = -24;
const float AZ_BIAS = 16000;

int main()
{
    //Serial setup
    Serial pc(USBTX, USBRX);
    pc.baud(115200);

    //initialize and start the MPU
    MPU6050 mpu(SDA, SCL);
    mpu.start();

    //array to store the biases
    float bias[6];

    while(1)
    {
        //get the raw data
        float data[6];
        mpu.read_raw(&data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);

        //account for bias and normalize the vector
        vector orientation = {data[3] - AX_BIAS, data[4] - AY_BIAS, data[5] - AZ_BIAS};
        vector norm_orientation;
        vector_normalize(&orientation, &norm_orientation);
        
        //vector rotation = {data[0] - GX_BIAS, data[1] - GY_BIAS, data[2] - GZ_BIAS};

        //for some reason, the x and z values are switched
        pc.printf("<accel>: %f, %f, %f\r\n", norm_orientation.x, norm_orientation.y, norm_orientation.z);

        wait(1);
    }
}
