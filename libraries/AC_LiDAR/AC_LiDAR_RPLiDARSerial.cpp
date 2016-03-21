/*
 * AC_LiDAR_RPLiDARSerial.cpp
 *
 *  Created on: 10 Mar 2016
 *      Author: moon
 */

#include "AC_LiDAR_RPLiDARSerial.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;


AC_LiDAR_RPLiDARSerial::AC_LiDAR_RPLiDARSerial(AC_LiDAR &_frontend, uint8_t _instance, AC_LiDAR::Obstacle &_obstacle):
	AC_LiDAR_Backend(_frontend, _instance, _obstacle),
    _port(NULL),
    _initialised(false)
{}

bool AC_LiDAR_RPLiDARSerial::init(const AP_SerialManager& serial_manager)
{
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);

	if (_port)
	{
		_initialised = true;
		return true;
	}
	else
	{
		return false;
	}
}


void AC_LiDAR_RPLiDARSerial::update()
{
    // exit immediately if not initialised
    if (!_initialised)
        return;


    int buff_cnt = read_serial();

    if (obstacle.avoid)
    {
    	// Clear buffer
    	memset(buff, 0, sizeof(buff));

    	// Check if avoidance is currently active
		if (abs(hal.scheduler->millis() - obstacle.last_time_ms) > 3000)
		{
			obstacle.avoid = false;
			//hal.console->printf_P(PSTR("Override Disabled"));
		}
    }

    else if (buff_cnt >= MSG_SIZE)
    {
    	parse_serial();
    }
}


int AC_LiDAR_RPLiDARSerial::read_serial()
{
	int buff_cnt = 0;

	while (_port->available() > 0 && buff_cnt < BUFF_SIZE)
	{
		buff[buff_cnt] = _port->read();
		++buff_cnt;
	}

	return buff_cnt;
}


bool AC_LiDAR_RPLiDARSerial::parse_serial()
{
	if(buff[0] == '*' && buff [5] == '%') // If obstacle detected
	{
		memcpy(&(obstacle.direction), &buff[1], sizeof(float));

		obstacle.last_time_ms = hal.scheduler->millis();
		obstacle.avoid = true;

		return true;
	}
	return false;
}
