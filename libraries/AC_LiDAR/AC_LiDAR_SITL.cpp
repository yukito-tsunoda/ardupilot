/*
 * AC_LiDAR_SITL.cpp
 *
 *  Created on: 10 Mar 2016
 *      Author: moon
 */

#include "AC_LiDAR_SITL.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;


AC_LiDAR_SITL::AC_LiDAR_SITL(AC_LiDAR &_frontend, uint8_t _instance, AC_LiDAR::Obstacle &_obstacle):
	AC_LiDAR_Backend(_frontend, _instance, _obstacle),
    _port(NULL),
    _initialised(false)
{
}

void AC_LiDAR_SITL::init(const AP_SerialManager& serial_manager)
{
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);

	if (_port) {
        _initialised = true;
    }
}

void AC_LiDAR_SITL::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

}
