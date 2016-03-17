/*
 * AC_LiDAR_SITL.h
 *
 *  Created on: 10 Mar 2016
 *      Author: moon
 */

#ifndef LIBRARIES_AC_LIDAR_AC_LIDAR_SITL_H_
#define LIBRARIES_AC_LIDAR_AC_LiDAR_SITL_H_

#include <AP_HAL/AP_HAL.h>
#include "../AC_LiDAR/AC_LiDAR_Backend.h"

class AC_LiDAR_SITL: public AC_LiDAR_Backend {
public:
	AC_LiDAR_SITL(AC_LiDAR &frontend,  uint8_t instance);
    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager);

    // update mount position - should be called periodically
    virtual void update();

private:
    // internal variables
    AP_HAL::UARTDriver *_port;
    bool _initialised;              // true once the driver has been initialised


};

#endif /* LIBRARIES_AC_LIDAR_AC_LiDAR_SITL_H_ */
