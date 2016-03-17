/*
 * AC_LiDAR.h
 *
 *  Created on: 9 Mar 2016
 *      Author: moon
 */

#ifndef LIBRARIES_AC_LIDAR_AC_LIDAR_H_
#define LIBRARIES_AC_LIDAR_AC_LIDAR_H_

#include <AP_SerialManager/AP_SerialManager.h>

// maximum number of LiDAR
#define AC_LIDAR_MAX_INSTANCES    1

class AC_LiDAR_Backend;
class AC_LiDAR_RPLiDARSerial;

class AC_LiDAR {
	friend class AC_LiDAR_Backend;
	friend class AC_LiDAR_RPLiDARSerial;

public:
    // Constructor
	AC_LiDAR();
	AC_LiDAR(AP_SerialManager &_serial_manager);

	// RangeFinder driver types
    enum RangeFinder_Type {
        RangeFinder_TYPE_NONE   = 0,
        RangeFinder_TYPE_RP_LIDAR = 1,
        RangeFinder_TYPE_SITL= 2
    };

	// init - detect and initialise all LiDARs
	void init(const AP_SerialManager& serial_manager);
	// update - give mount opportunity to update servos.  should be called at 10hz or higher
	void update();

	// support for SITL
    void setSITL(uint8_t instance, const struct LiDAR_State &state);

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // front end members
    uint8_t             _num_instances;     // number of LiDARs instantiated
    uint8_t             _primary;           // primary LiDAR
    AC_LiDAR_Backend    *_backends[AC_LIDAR_MAX_INSTANCES];         // pointers to instantiated LiDARs

};

#endif /* LIBRARIES_AC_LIDAR_AC_LIDAR_H_ */
