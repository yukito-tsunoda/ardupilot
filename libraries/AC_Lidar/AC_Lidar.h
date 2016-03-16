/*
 * AC_Lidar.h
 *
 *  Created on: 9 Mar 2016
 *      Author: moon
 */

#ifndef LIBRARIES_AC_LIDAR_AC_LIDAR_H_
#define LIBRARIES_AC_LIDAR_AC_LIDAR_H_

#include <AP_SerialManager/AP_SerialManager.h>

// maximum number of Lidar
#define AC_LIDAR_MAX_INSTANCES    1

class AC_Lidar_Backend;
class AC_Lidar_RPLidarSerial;

class AC_Lidar {
	friend class AC_Lidar_Backend;
	friend class AC_Lidar_RPLidarSerial;

public:
    // Constructor
	AC_Lidar();
	AC_Lidar(AP_SerialManager &_serial_manager);

	// RangeFinder driver types
    enum RangeFinder_Type {
        RangeFinder_TYPE_NONE   = 0,
        RangeFinder_TYPE_RP_LIDAR = 1,
        RangeFinder_TYPE_SITL= 2
    };


	// init - detect and initialise all Lidars
	void init(const AP_SerialManager& serial_manager);
	// update - give mount opportunity to update servos.  should be called at 10hz or higher
	void update();

	// support for SITL
    void setSITL(uint8_t instance, const struct Lidar_State &state);

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // front end members
    uint8_t             _num_instances;     // number of Lidars instantiated
    uint8_t             _primary;           // primary Lidar
    AC_Lidar_Backend    *_backends[AC_LIDAR_MAX_INSTANCES];         // pointers to instantiated Lidars


private:
	//AP_SerialManager &serial_manager;

};

#endif /* LIBRARIES_AC_LIDAR_AC_LIDAR_H_ */
