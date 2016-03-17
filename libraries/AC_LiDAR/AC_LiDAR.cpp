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
const AP_Param::GroupInfo AC_LiDAR::var_info[] = {
};

AC_LiDAR::AC_LiDAR()
{
    // initialise backend pointers and mode
    for (uint8_t i=0; i<AC_LIDAR_MAX_INSTANCES; i++) {
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

void AC_LiDAR::init(const AP_SerialManager& serial_manager) {
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }

    // primary is reset to the first instantiated mount
    bool primary_set = false;

    // create each instance
    for (uint8_t instance=0; instance<AC_LIDAR_MAX_INSTANCES; instance++) {
        // default instance's state
            _backends[instance] = new AC_LiDAR_RPLiDARSerial(*this,  instance);
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

void AC_LiDAR::update() {

    // update each instance
    for (uint8_t instance=0; instance<AC_LIDAR_MAX_INSTANCES; instance++) {
        if (_backends[instance] != NULL) {
            _backends[instance]->update();
        }
    }
}
