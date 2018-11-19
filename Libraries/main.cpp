#include "millis.h"
#include "quaternion.h"
#include "sensor_fusion.h"
#include "mbed.h"

const float pi = 3.14159265359;

//TODO: check expected values and adjust bias_calc accordingly
void bias_calc(MPU6050 *mpu, float bias[6])
{
    float data[6];

    for(int i =0; i < 20; i++)
    {
        mpu->read_raw(&data[0],&data[1],&data[2],&data[3],&data[4],&data[5]);

        //sum errors and average
        for(int j = 0; j < 6; j++)
            bias[j] += (data[j]/20);
    }

    //expect ax to be 1 upwards
    bias[0] = bias[0] - 1;
}

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

    //get the biases
    //note that we pass mpu's address since it not copyable
    bias_calc(&mpu, bias);

    while(1)
    {
        //get the raw data
        float data[6];
        mpu.read_raw(&data[0], &data[1], &data[2], &data[3], &data[4], &data[5]);

        //account for bias and normalize the vector
        vector orientation = {data[3] - bias[3], data[4] - bias[4], data[5] - bias[5]};
        vector norm_orientation;
        vector_normalize(&orientation, &norm_orientation);

        pc.printf("<accel>: %f, %f, %f\r\n", norm_orientation.x, norm_orientation.y, norm_orientation.z);
        wait(0.1);
    }
}