/*
 * AC_Lidar.cpp
 *
 *  Created on: 9 Mar 2016
 *      Author: moon
 */

#include "AC_Lidar.h"
#include "AC_Lidar_RPLidarSerial.h"
#include "AC_Lidar_SITL.h"

 // table of user settable parameters
const AP_Param::GroupInfo AC_Lidar::var_info[] = {
};

AC_Lidar::AC_Lidar():
	_num_instances(0),
	_primary(0)

{
    // initialise backend pointers and mode
    for (uint8_t i=0; i<AC_LIDAR_MAX_INSTANCES; i++) {
        _backends[i] = NULL;
    }
}

AC_Lidar::AC_Lidar(AP_SerialManager &_serial_manager) :
    _primary(0), //primary_instance(0),
    _num_instances(0){}
    
/*
    ,

    serial_manager(_serial_manager)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init state and drivers
    memset(state,0,sizeof(state));
    memset(drivers,0,sizeof(drivers));
}
*/
void AC_Lidar::init(const AP_SerialManager& serial_manager) {
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }

    // primary is reset to the first instantiated mount
    bool primary_set = false;

    // create each instance
    for (uint8_t instance=0; instance<AC_LIDAR_MAX_INSTANCES; instance++) {
        // default instance's state
            _backends[instance] = new AC_Lidar_RPLidarSerial(*this,  instance);
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

void AC_Lidar::update() {
    // update each instance
    for (uint8_t instance=0; instance<AC_LIDAR_MAX_INSTANCES; instance++) {
        if (_backends[instance] != NULL) {
            _backends[instance]->update();
        }
    }
}
