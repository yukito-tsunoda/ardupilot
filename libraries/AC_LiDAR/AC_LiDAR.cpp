/*
 * AC_LiDAR.cpp
 *
 *  Created on: 9 Mar 2016
 *      Author: moon
 */

#include "AC_LiDAR.h"
#include "AC_LiDAR_RPLiDARSerial.h"
#include "AC_LiDAR_SITL.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

 // table of user settable parameters
const AP_Param::GroupInfo AC_LiDAR::var_info[] = {};

AC_LiDAR::AC_LiDAR()
{
    // initialise backend pointers and mode
    for (uint8_t i=0; i<AC_LIDAR_MAX_INSTANCES; i++)
    {
        _backends[i] = NULL;
    }
}

AC_LiDAR::AC_LiDAR(AP_SerialManager &_serial_manager) :
    _primary(0),
    _num_instances(0)
	{
	/*
		AP_Param::setup_object_defaults(this, var_info);

		// init state and drivers
		memset(state,0,sizeof(state));
		memset(drivers,0,sizeof(drivers));
	*/
	}

void AC_LiDAR::init(const AP_SerialManager& serial_manager)
{
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }

    // primary is reset to the first instantiated mount
    bool primary_set = false;

    // create each instance
    for (uint8_t instance=0; instance<AC_LIDAR_MAX_INSTANCES; instance++) {

    	// TODO: Pass the obstacle instance to only the primary LiDAR
		_backends[instance] = new AC_LiDAR_RPLiDARSerial(*this, instance, obstacle);
		_num_instances++;

		// init new instance
        if (_backends[instance] != NULL) {
            _backends[instance]->init(serial_manager);
			if (!primary_set) {
                _primary = instance;
                primary_set = true;
            }
        }
    }
}

void AC_LiDAR::update()
{
    // update each instance
    for (uint8_t instance=0; instance<AC_LIDAR_MAX_INSTANCES; instance++) {
        if (_backends[instance] != NULL) {
            _backends[instance]->update();
        }
    }
}


bool AC_LiDAR::obstacle_avoid()
{
	return obstacle.avoid;
}


float AC_LiDAR::obstacle_direction()
{
	return obstacle.direction;
}


float AC_LiDAR::obstacle_distance()
{
	return obstacle.distance;
}


uint32_t AC_LiDAR::obstacle_last_time_ms()
{
	return obstacle.last_time_ms;
}

uint32_t AC_LiDAR::obstacle_elapsed_time_ms()
{
	return abs(hal.scheduler->millis() - obstacle.last_time_ms);
}
