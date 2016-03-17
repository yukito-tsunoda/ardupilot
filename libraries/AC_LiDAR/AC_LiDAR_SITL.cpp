
#ifndef __AP_LIDAR_SITL_H__
#define __AP_LIDAR_SITL_H__

#include "../AC_LiDAR/AC_LiDAR_SITL.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AC_LiDAR_SITL::AC_LiDAR_SITL(AC_LiDAR& frontend,uint8_t instance):
	AC_LiDAR_Backend(frontend, instance),
    _port(NULL),
    _initialised(false)
{}

void AC_LiDAR_SITL::init(const AP_SerialManager& serial_manager) {
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);
    if (_port) {
        _initialised = true;
    }
}

void AC_LiDAR_SITL::update() {
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    //Todo
}

#endif //  __AP_RANGEFINDER_SITL_H__
