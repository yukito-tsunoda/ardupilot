/*
 * AC_Lidar_RPLidarSerial.cpp
 *
 *  Created on: 10 Mar 2016
 *      Author: moon
 */

#include "AC_Lidar_RPLidarSerial.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

AC_Lidar_RPLidarSerial::AC_Lidar_RPLidarSerial(AC_Lidar& frontend,uint8_t instance):
	AC_Lidar_Backend(frontend, instance),
    _port(NULL),
    _initialised(false)
{}

void AC_Lidar_RPLidarSerial::init(const AP_SerialManager& serial_manager) {
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);
    if (_port) {
        _initialised = true;
    }
}

void AC_Lidar_RPLidarSerial::update() {
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    //Todo
}
