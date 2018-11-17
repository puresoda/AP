/* 
 * @author: Natasha Sarkar, 2018
 */

#include "mbed.h"
#include "sensor_fusion.h"
 
MPU6050::MPU6050(PinName sda, PinName scl): i2c_object(sda, scl) {
    i2c_object.frequency(400000);
}
 
void MPU6050::start(void) {

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
 
bool MPU6050::read_raw(float *gx, float *gy, float *gz, float *ax, float *ay, float *az) {
    if(data_ready == true){
        
        //initialize buffers to store output of gyroscope and acceleromter
        char accel[6];
        char gyro[6];
        
        //get data; how do I read the Y and Z values tho?
        read_reg(ADDRESS, ACCEL_X, accel, 6);
        read_reg(ADDRESS, GYRO_X, gyro, 6);
        
        //store data in pointers
        *ax = (accel[0] | accel[1]);
        *ay = (accel[2] | accel[3]);
        *az = (accel[4] | accel[5]);
        *gx = (gyro[0] | gyro[1]);
        *gy = (gyro[2] | gyro[3]);
        *gz = (gyro[4] | gyro[5]);
        
        return true;
    }
    
    return false;
}

bool MPU6050::data_ready(void) {
    /** TO DO
     * 
     * CHECK THE INT_STATUS REGISTER TO DETERMINE IF DATA IS READY 0x3A
     *
     * Return true if it is ready, false otherwise.
     */

    /** YOUR CODE GOES BELOW */
    char status;
    read_reg(ADDRESS, INT_STATUS, &status, 1);
    //then what?
}

bool MPU6050::write_reg(int addr, char reg, char buf) {
    /** TO DO
     * 
     * IMPELEMENT THIS FUNCTION
     *
     * See the documentation in sensor_fusion.h for detail.
     */

    /** YOUR CODE GOES BELOW */ 
    
    
    char data[2] = {reg, buf};
    return i2c_object.write(addr, data, 2, false) == 0;
}
 
bool MPU6050::read_reg(int addr, char reg, char *buf, int length) {
    /** TO DO
     * 
     * IMPLEMENT THIS FUNCTION
     * 
     * See the documentation in sensor_fusion.h for detail.
     */

    /** YOUR CODE GOES BELOW */ 
    
    return i2c_object.write(addr, reg, 1, false) == 0 && i2c_object.read(addr, buf, false) == 0;
}

