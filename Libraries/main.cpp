#include "millis.h"
#include "quaternion.h"
#include "sensor_fusion.h"
#include "mbed.h"

using namespace std;

int main()
{
    //initialize and start the MPU
    //TODO: properly define the SDA and SLC pins based on our config
    MPU6050 yeet(SDA, SLC);
    yeet.start();

    //array to store the readings and Serial object to print
    float* data[6];

    //tbd how long the buffer should be?
    //check how to parse floats into printf?
    char* output[24]; 

    Serial terminal(USBTX,USBRX); 

    //take 20 readings and start clock
    int read_count = 0;
    millis_begin();

    while(read_count < 20)
    {
        if(millis() >= 100)
        {
            yeet.read_raw(data[0],data[1],data[2],data[3],data[4],data[5]);

            //TODO: print out the readings
            terminal.printf(data[0],data[1],data[2],data[3],data[4],data[5])
            millis_begin();
            read_count++;
        }
    }
}