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
// table of user settable parameters
const AP_Param::GroupInfo AC_LiDAR::var_info[] PROGMEM = {
    // @Param: _TYPE
    // @DisplayName: LiDAR type
    // @Description: LiDAR type
    // @Values: 0:None,1:AUTO,2:RPLiDAR,3:HIL
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("_TYPE",    0, AC_LiDAR, _type[0], 0),
    
    AP_GROUPEND
};

AC_LiDAR::AC_LiDAR()
{
    // initialise backend pointers and mode
    for (uint8_t i=0; i<AC_LIDAR_MAX_INSTANCES; i++)
    {
        _backends[i] = NULL;
    }

    override_roll = RPM_NEUTRAL;
    override_pitch = RPM_NEUTRAL;
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
    if (_num_instances != 0) 
        return;

    // create each instance
    for (uint8_t instance=0; instance<AC_LIDAR_MAX_INSTANCES; instance++)
    {
    	switch(instance)
        {
            case 0 :
                _backends[instance] = new AC_LiDAR_RPLiDARSerial(*this, instance, obstacle);
                break;

            case 1 :
                _backends[instance] = new AC_LiDAR_SITL(*this, instance, obstacle);
                break;

            default :
                _backends[instance] = NULL;
        }

		_num_instances++;

	    // primary is reset to the first instantiated mount
	    bool primary_set = false;

        if (_backends[instance] != NULL)
        {
			if(!primary_set && _backends[instance]->init(serial_manager))
			{
				_primary = instance;
				primary_set = true;
			}
        }
    }
}


void AC_LiDAR::update()
{
    // update the primary instance only
	if (_backends[_primary] != NULL)
	{
		_backends[_primary]->update();
	}
}


bool AC_LiDAR::withdraw_from_obstacle()
{
	return obstacle.withdraw;
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

void AC_LiDAR::update_sitl(uint8_t instance, const double sitl_scan[])
{
    if (_backends[instance]) 
    {
    	// Set the primary instance to SITL
    	_primary = instance;
		_backends[instance]->update_sitl(sitl_scan);
    }
}

void AC_LiDAR::calculate_roll_n_pitch()
{
    override_roll = -sin(obstacle.direction) * RPM_OFFSET + RPM_NEUTRAL;
    override_pitch = cos(obstacle.direction) * RPM_OFFSET + RPM_NEUTRAL;
}

int AC_LiDAR::get_override_roll()
{
    return override_roll;
}

int AC_LiDAR::get_override_pitch()
{
    return override_pitch;
}

int AC_LiDAR::get_counter_roll()
{
    return RPM_NEUTRAL + RPM_COEFFICIENT * (RPM_NEUTRAL - override_roll);
}

int AC_LiDAR::get_counter_pitch()
{
    return RPM_NEUTRAL + RPM_COEFFICIENT * (RPM_NEUTRAL - override_pitch);
}

bool AC_LiDAR::disregard_pilot_input(int16_t in_roll, int16_t in_pitch)
{
	if (obstacle.disregard)
	{
		float x1 = sin(obstacle_direction() - M_PI/2);
		float y1 = cos(obstacle_direction() - M_PI/2);

		float x2 = sin(obstacle_direction() + M_PI/2);
		float y2 = cos(obstacle_direction() + M_PI/2);

		float rc_in_x = in_roll - RPM_NEUTRAL;
		float rc_in_y = -(in_pitch - RPM_NEUTRAL);

		float rc_direction = rc_in_x * (y1-y2) + x1 * (y2-rc_in_y) + x2 * (rc_in_y-y1);
		float obstacle_side = sin(obstacle_direction()) * (y1-y2) + x1 * (y2-cos(obstacle_direction())) + x2 * (cos(obstacle_direction())-y1);

		return (rc_direction > 0 && obstacle_side > 0) || (rc_direction < 0 && obstacle_side < 0);
	}

	else
		return false;
}
