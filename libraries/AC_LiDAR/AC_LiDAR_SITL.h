/*
 * AC_LiDAR_SITL.h
 *
 *  Created on: 10 Mar 2016
 *      Author: moon
 */

#ifndef LIBRARIES_AC_LIDAR_AC_LIDAR_SITL_H_
#define LIBRARIES_AC_LIDAR_AC_LiDAR_SITL_H_

#include "AC_LiDAR_Backend.h"
#include <AP_HAL/AP_HAL.h>

#define SITL_SCAN_SIZE 180
#define DISTANCE_TO_AVOID 2.0

class AC_LiDAR_SITL: public AC_LiDAR_Backend {
public:
	AC_LiDAR_SITL(AC_LiDAR &_frontend,  uint8_t _instance, AC_LiDAR::Obstacle &_obstacle);
    // init - performs any required initialisation for this instance
    bool init(const AP_SerialManager& serial_manager);

    void update();

    void update_sitl(const double scan[]);
    void detect_obstacle();

private:
    // internal variables
    AP_HAL::UARTDriver *_port;
    bool _initialised;              // true once the driver has been initialised

    double scan[SITL_SCAN_SIZE];
    
};

#endif /* LIBRARIES_AC_LIDAR_AC_LiDAR_SITL_H_ */
