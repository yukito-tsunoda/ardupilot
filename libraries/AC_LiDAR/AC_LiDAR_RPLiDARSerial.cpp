/*
 * AC_LiDAR_RPLiDARSerial.cpp
 *
 *  Created on: 10 Mar 2016
 *      Author: moon
 */

#include "../AC_LiDAR/AC_LiDAR_RPLiDARSerial.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;


AC_LiDAR_RPLiDARSerial::AC_LiDAR_RPLiDARSerial(AC_LiDAR& frontend,uint8_t instance):
	AC_LiDAR_Backend(frontend, instance),
    _port(NULL),
    _initialised(false)
{}

void AC_LiDAR_RPLiDARSerial::init(const AP_SerialManager& serial_manager)
{
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);

	if (_port) {
        _initialised = true;
    }
}

void AC_LiDAR_RPLiDARSerial::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    read_serial();
}

bool AC_LiDAR_RPLiDARSerial::read_serial()
{
	buff_cnt = 0;

	while (_port->available() > 0 && buff_cnt < BUFF_SIZE)
	{
		buff[buff_cnt] = _port->read();
		buff_cnt++;
	}

	if(buff_cnt >= 6)
	{
		if(buff[0] == '*' && buff[5] == '%') // If obstacle detected
		{

			float obstacle_direction;

			memcpy(&obstacle_direction, &buff[1], sizeof(float));

	//		//Calculate new roll & pitch
	//		const int neutral = 1500;
	//		const int avoid_speed = 80;
	//		const int avoid_speed_slow = 50;
	//
	//		new_rc_roll = -sin(obstacle_direction) * avoid_speed + neutral;
	//		new_rc_pitch = cos(obstacle_direction) * avoid_speed + neutral;

			//new_rc_roll_slow = -sin(obstacle_direction) * avoid_speed_slow + neutral;
			//new_rc_pitch_slow = cos(obstacle_direction) * avoid_speed_slow + neutral;

			hal.console->printf_P(PSTR("\n %f \n"), obstacle_direction);

	//		last_obstacle_time_ms = hal.scheduler->millis();
	//
	//		avoid_obstacle = true;
		}
	}
	return true;
}
