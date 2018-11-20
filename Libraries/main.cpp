#include "mbed.h"
#include "sensor_fusion.h"
#include "quaternion.h"
#include "millis.h"

const float M_PI = 3.14159265358979323846;
const float alpha = .5;

const float AX_BIAS = -1300;
const float AY_BIAS = 8550;
const float AZ_BIAS = 2384;

const float GX_BIAS = 25;
const float GY_BIAS = -15;
const float GZ_BIAS = 18;

Serial pc(USBTX, USBRX); //create a Serial object    
Timer t; //create timer

int main() 
{
  t.start(); //start the timer
  pc.baud(115200); //set the baud rate
  float time_elapsed; //float to store time elapsed

  //create and start the MPU
  MPU6050 mpu(D4, D5);
  mpu.start();

  //inital orientation
  vector unit = {0, 0, 1};

  //data storage
  float gx,gy,gz,ax,ay,az;

  while(1) 
  {
    //get the gyroscope and acceleromter raw values
    mpu.read_raw(&gx,&gy,&gz,&ax,&ay,&az); 

    //get the elapsed time and reset the timer
    time_elapsed = t.read();
    t.reset();
        
    vector accel = {ax + AX_BIAS, ay + AY_BIAS, az + AZ_BIAS}; //calbrate accelerometer vector

    //normalize the calibrated acelerometer data
    vector norm_accel;
    vector_normalize(&accel, &norm_accel); 
        
    vector gyroadd = {gx + GX_BIAS, gy + GY_BIAS, gz + GZ_BIAS}; //calibrate gyroscope data

    //normalize the calirated gyroscope data
    vector gyrofinal;
    float gyro_mag = vector_normalize(&gyroadd, &gyrofinal);

    gyro_mag *= time_elapsed; //since gyro measures in deg/s, we multiply by the time passed

    //create gyroscope rotation quaternion
    quaternion quat;
    quaternion_create(&gyrofinal, -(gyro_mag/16.4/360*2*M_PI), &quat);

    //rotate the gyroscope vector
    vector gyroprint;
    quaternion_rotate(&unit, &quat, &gyroprint);

    //store and update unit's value
    vector n = unit;
    unit = gyroprint;

    vector mult; //vector to store scaled values 
    vector rotate; //vector to store rotated vector
    vector result; //resulting vector
    quaternion quat2; //rotation quaternion

    quaternion_create(&unit, -(gyro_mag/16.4/360*2*M_PI), &quat2);
    vector_multiply(&norm_accel, alpha, &mult);
    quaternion_rotate(&n, &quat2, &rotate);
    vector_add(&mult, &rotate, &result);
        
    printf("%f %f %f %f %f %f %f %f %f\r\n",
    norm_accel.x, norm_accel.y, norm_accel.z,
    gyroprint.x, gyroprint.y, gyroprint.z,
    result.x, result.y, result.z);

    wait(.1);
   }
}