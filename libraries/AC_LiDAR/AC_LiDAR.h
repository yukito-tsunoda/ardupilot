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
#define AC_LIDAR_MAX_INSTANCES 2

#define RPM_NEUTRAL 1500
#define RPM_OFFSET 100
#define RPM_COEFFICIENT 1.0

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

    struct Obstacle {
    	bool avoid;
		float direction;
		float distance;
		uint32_t last_time_ms;

		Obstacle(): avoid (false), direction (0.0), distance (10000), last_time_ms (0){}
    };

	// init - detect and initialise all LiDARs
	void init(const AP_SerialManager& serial_manager);

	// update - give mount opportunity to update servos.  should be called at 10hz or higher
	void update();

	// support for SITL
    void update_sitl(uint8_t instance, const double sitl_scan[]);

    static const struct AP_Param::GroupInfo var_info[];

    AP_Int8  _type[AC_LIDAR_MAX_INSTANCES];

    bool obstacle_avoid();
	float obstacle_direction();
	float obstacle_distance();
	uint32_t obstacle_last_time_ms();
	uint32_t obstacle_elapsed_time_ms();

	void calculate_roll_n_pitch();
	int get_override_roll();
	int get_override_pitch();
	int get_counter_roll();
	int get_counter_pitch();

protected:
    // front end members
    uint8_t             _primary;           // primary LiDAR
    uint8_t             _num_instances;     // number of LiDARs instantiated
    AC_LiDAR_Backend    *_backends[AC_LIDAR_MAX_INSTANCES];         // pointers to instantiated LiDARs
    Obstacle obstacle;
private:
	int override_roll;
	int override_pitch;
};

#endif /* LIBRARIES_AC_LIDAR_AC_LIDAR_H_ */
