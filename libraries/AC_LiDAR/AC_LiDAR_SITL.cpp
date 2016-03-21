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
    for (int i=0; i<SITL_SCAN_SIZE; ++i)
        scan[i] = 10000.0;
}

void AC_LiDAR_SITL::update()
{
}

void AC_LiDAR_SITL::update_sitl(const double _scan[])
{

    for (int i=0; i<SITL_SCAN_SIZE; ++i)
    {
        scan[i] = _scan[i];
    }

    detect_obstacle();
}

void AC_LiDAR_SITL::detect_obstacle()
{
    int min_index = 0;

    for (int i=0; i<SITL_SCAN_SIZE; ++i)
    {
        if (scan[i] < scan[min_index])
            min_index = i;
    }

    if (scan[min_index] < DISTANCE_TO_AVOID)
    {
        int degree = -min_index + 90; 
        obstacle.direction = degree / 180.0 * M_PI;
        obstacle.distance = degree;//scan[min_index];
        obstacle.last_time_ms = hal.scheduler->millis();
        obstacle.avoid = true;     
    }

    else
        obstacle.avoid = false;
}
