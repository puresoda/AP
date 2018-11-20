#include "mbed.h"
#include "sensor_fusion.h"
#include "quaternion.h"
#include "millis.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
 Serial pc(USBTX, USBRX); //create a Serial object
       
    Timer t; 
float gx,gy,gz,ax,ay,az;
int main() 
{
    t.start();
   pc.baud(115200);
   float j;
    MPU6050 sense (D4, D5);
    sense.start();
  struct vector unit = {0, 0, 1};
  while(1) {
        sense.read_raw(&gx,&gy,&gz,&ax,&ay,&az);
        j = t.read();
        t.reset();
        //printf(" time = %f",j);
        //printf("gx,gy,gz,ax,ay,az %.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",gx,gy,gz,ax,ay,az);
       // wait(1);
        
         struct vector before = {ax, ay, az};
         float xvalue = -1300;
         float yvalue = 8550;
         float zvalue = 2384; //-13900
         struct vector add;
         struct vector bias = {xvalue, yvalue, zvalue};
         vector_add(&bias, &before, &add);
         struct vector after;
        vector_normalize(&add, &after); //unit vector that points in direction 
        //printf("%f %f %f\r\n", after.x, after.y, after.z);
        
        float alpha = .5;
        struct vector gyro1 = {gx, gy, gz};
        float xgyro = 25;
        float ygyro = -15;
        float zgyro = 18;
        struct vector gyroadd;
        struct vector equalize = {xgyro, ygyro, zgyro};
        vector_add(&equalize, &gyro1, &gyroadd);

        struct vector gyrofinal;
        float gyromag = vector_normalize(&gyroadd, &gyrofinal);

        gyromag *= j;
        struct quaternion quat;
        quaternion_create(&gyrofinal, -(gyromag/16.4/360*2*M_PI), &quat);

        vector gyroprint;
        quaternion_rotate(&unit, &quat, &gyroprint);

        struct vector n = unit;
        unit = gyroprint;

        struct vector mult; 
        struct vector rotate;
        struct vector final2;
        struct quaternion quat2;

        quaternion_create(&unit, -(gyromag/16.4/360*2*M_PI), &quat2);
        vector_multiply(&after, alpha, &mult);
        quaternion_rotate(&n, &quat2, &rotate);
        vector_add(&mult, &rotate, &final2);
        
        printf("%f %f %f %f %f %f %f %f %f\r\n", after.x, after.y, after.z, gyroprint.x, gyroprint.y, gyroprint.z, final2.x, final2.y, final2.z);
        wait(.1);
   }
}