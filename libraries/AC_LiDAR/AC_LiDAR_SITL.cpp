/*
 * AC_LiDAR_SITL.cpp
 *
 *  Created on: 10 Mar 2016
 *      Author: moon
 */

#include "AC_LiDAR_SITL.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#include <stdio.h>

#define MIN_SCAN_RANGE 0.5


AC_LiDAR_SITL::AC_LiDAR_SITL(AC_LiDAR &_frontend, uint8_t _instance, AC_LiDAR::Obstacle &_obstacle):
	AC_LiDAR_Backend(_frontend, _instance, _obstacle),
    _port(NULL),
    _initialised(false)
{
}

bool AC_LiDAR_SITL::init(const AP_SerialManager& serial_manager)
{
    for (int i=0; i<SITL_SCAN_SIZE; ++i)
        scan[i] = 10000.0;

    return false;
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
    double min_range = 10000;

    for (int i=2; i<SITL_SCAN_SIZE; ++i) // First index has an error, need to be fixed!
    {
        if (scan[i] < min_range && scan[i] > MIN_SCAN_RANGE)
        {
            min_range = scan[i];
            min_index = i;
        }
    }

    if (min_range < DISTANCE_TO_WITHDRAW)
    {
        int degree = -min_index + (SITL_SCAN_SIZE/2); 
        obstacle.direction = degree / 180.0 * M_PI;
        obstacle.distance = scan[min_index];
        obstacle.last_time_ms = hal.scheduler->millis();
        obstacle.withdraw = true;
        obstacle.disregard = false;
    }

    else if (min_range < DISTANCE_TO_DISREGARD)
    {
        int degree = -min_index + (SITL_SCAN_SIZE/2); 
        obstacle.direction = degree / 180.0 * M_PI;
        obstacle.distance = scan[min_index];
        //obstacle.last_time_ms = hal.scheduler->millis();
        obstacle.withdraw = false;
        obstacle.disregard = true;
    }

    else
    {
        obstacle.withdraw = false;
        obstacle.disregard = false;
    }
}
