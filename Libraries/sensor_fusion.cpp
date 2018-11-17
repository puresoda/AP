/* 
 * @author: Natasha Sarkar, 2018
 */

#include "mbed.h"
#include "sensor_fusion.h"
 
MPU6050::MPU6050(PinName sda, PinName scl): i2c_object(sda, scl) 
{
    i2c_object.frequency(400000);
}
 
void MPU6050::start(void) 
{
    //SLEEP to 0: Sleep mode off
    //CYCLE to 0: Cycle mode off
    //CLKSEL to 0 to read from internal 8MHz oscillator 
    //DEVICE_RESET to 0: device is not being reset
    //TEMP_DIS to 0: temperature sensor is on 
    write_reg(ADDRESS, PWR_MGMT_1, 0x00);
    
    //XA_ST to 0: no self test for x-axis acceleromter 
    //YA_ST to 0: no self test for y-axis acceleromter
    //ZA_ST to 0: no self test for z-axis acceleromter
    //AFS_SEL to 0: expect readings from -2g to 2g
    write_reg(ADDRESS, ACCEL_CONFIG, 0x00);

    //XG_ST to 0: no self test for x-axis gyroscope
    //YG_ST to 0: no self test for y-axis gyroscope
    //ZG_ST to 0: no self test for z-axis gyroscope
    //FS_SEL to 3: expect readings from -2000 deg/s to 2000 deg/s
    write_reg(ADDRESS, GYRO_CONFIG, 0x03 << 3);

    //EXT_SYNC_SET to 0: no FSYNC
    //DLPF_CFG to 0: 
        //acceleromter bandwith = 260 Hz
        //gyroscope bandwith = 256 Hz
    write_reg(ADDRESS, CONFIG, 0x00);
}
 
bool MPU6050::read_raw(float *gx, float *gy, float *gz, float *ax, float *ay, float *az) 
{
    if(!data_ready)
        return false;

    //initialize buffers to store output of gyroscope and acceleromter
    char accel[6];
    char gyro[6];
        
    read_reg(ADDRESS, ACCEL_X, accel, 6);
    read_reg(ADDRESS, GYRO_X, gyro, 6);
        
    //store data in pointers
    *ax = float(short(accel[0] | accel[1] << 8));
    *ay = float(short(accel[2] | accel[3] << 8));
    *az = float(short(accel[4] | accel[5] << 8));
    *gx = float(short(gyro[0] | gyro[1] << 8));
    *gy = float(short(gyro[2] | gyro[3] << 8));
    *gz = float(short(gyro[4] | gyro[5] << 8));
        
    return true;
}

bool MPU6050::data_ready(void) 
{
    //create buffer
    char status;

    //read status register
    read_reg(ADDRESS, INT_STATUS, &status, 1);
    return(status & 0x01);
}

bool MPU6050::write_reg(int addr, char reg, char buf)
{
    //create a single array to hold both the register and buffer per I2C standards
    char data[2] = {reg, buf};

    //return true if write was true
    return (i2c_object.write(addr, data, 2, false) == 0);
}
 
bool MPU6050::read_reg(int addr, char reg, char *buf, int length) 
{
    //return true if we can write to the register and we can read from it
    return (i2c_object.write(addr, reg, 1, false) == 0 && i2c_object.read(addr, buf, false) == 0);
}

