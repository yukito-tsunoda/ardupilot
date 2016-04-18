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

    if (buff_cnt >= MSG_SIZE)
    {
        if (parse_serial())
        	obstacle.last_time_ms = hal.scheduler->millis();
    }

	// Disactivate if not receiving a packet for 300ms
	if (abs(hal.scheduler->millis() - obstacle.last_time_ms) > 300)
	{
		obstacle.withdraw = false;
		obstacle.disregard = false;
		//hal.console->printf_P(PSTR("Override Disabled"));
	}
}


int AC_LiDAR_RPLiDARSerial::read_serial()
{
	int buff_cnt = 0;

	// Clear buffer
	memset(buff, 0, sizeof(buff));

	while (_port->available() > 0 && buff_cnt < BUFF_SIZE)
	{
		buff[buff_cnt] = _port->read();
		++buff_cnt;
	}

	return buff_cnt;
}


bool AC_LiDAR_RPLiDARSerial::parse_serial()
{
	/*
	 *	protocol:
	 *		char flag			// 'c' is clear, 'd' is disregard, and 'w' is withdraw
	 *	 	float direction		// in radian
	 *	 	float distance		// in meter
	 *	 	char '%'
	 */

	if (buff [MSG_SIZE-1] == '%')
	{
		memcpy(&(obstacle.direction), &buff[1], sizeof(float));
		memcpy(&(obstacle.distance), &buff[5], sizeof(float));

		switch (buff[0])
		{
			case 'c':
				obstacle.withdraw = false;
				obstacle.disregard = false;
				return true;

			case 'd':
				obstacle.disregard = true;
				obstacle.withdraw = false;
				return true;

			case 'w':
				obstacle.disregard = false;
				obstacle.withdraw = true;
				return true;

			default:
				return false;
		}
	}
}
