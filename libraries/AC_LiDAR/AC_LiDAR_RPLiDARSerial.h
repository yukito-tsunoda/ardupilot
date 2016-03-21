/*
 * AC_LiDAR_RPLiDARSerial.h
 *
 *  Created on: 10 Mar 2016
 *      Author: moon
 */

#ifndef LIBRARIES_AC_LIDAR_AC_LIDAR_RPLIDARSERIAL_H_
#define LIBRARIES_AC_LIDAR_AC_LIDAR_RPLIDARSERIAL_H_

#include "AC_LiDAR_Backend.h"
#include <AP_HAL/AP_HAL.h>

#define BUFF_SIZE 128
#define MSG_SIZE 6
 extern const AP_HAL::HAL& hal;

class AC_LiDAR_RPLiDARSerial: public AC_LiDAR_Backend {
public:
	AC_LiDAR_RPLiDARSerial(AC_LiDAR &_frontend,  uint8_t _instance, AC_LiDAR::Obstacle &_obstacle);
    // init - performs any required initialisation for this instance
    void init(const AP_SerialManager& serial_manager);

    // update mount position - should be called periodically
    void update();

    int read_serial();
    bool parse_serial();
    void calculate_rpm();

    // This function is not used in this class
    void update_sitl(double const scan[]){hal.console->printf_P(PSTR("Override Disabled"));}

protected:


private:
    // internal variables
    AP_HAL::UARTDriver *_port;
    bool _initialised;              // true once the driver has been initialised
    char buff[BUFF_SIZE];
};

#endif /* LIBRARIES_AC_LIDAR_AC_LIDAR_RPLIDARSERIAL_H_ */
