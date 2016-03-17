/*
 * AC_LiDAR_Backend.h
 *
 *  Created on: 9 Mar 2016
 *      Author: moon
 */

#ifndef LIBRARIES_AC_LIDAR_AC_LIDAR_BACKEND_H_
#define LIBRARIES_AC_LIDAR_AC_LIDAR_BACKEND_H_

#include "../AC_LiDAR/AC_LiDAR.h"

class AC_LiDAR_Backend {
public:
    // Constructor
	AC_LiDAR_Backend(AC_LiDAR &frontend,  uint8_t instance) :
        _frontend(frontend),
        _instance(instance)
	{}
    // Virtual destructor
	virtual ~AC_LiDAR_Backend(void) {}

    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager) = 0;

    // update mount position - should be called periodically
    virtual void update() = 0;

protected:
    AC_LiDAR    &_frontend; // reference to the front end which holds parameters
    uint8_t     _instance;  // this instance's number

};

#endif /* LIBRARIES_AC_LIDAR_AC_LIDAR_BACKEND_H_ */
