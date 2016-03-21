/*
  SITL handling

  This simulates a 2D LiDAR

 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_HAL_SITL.h"

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#define LIDAR_SITL_INST 1

/*
  setup the range_finder with new input
  range input is in meters
 */

void SITL_State::_update_lidar(double *lidar_scan)
{
    if (_lidar == NULL) {
        // no lidar in this sketch
        return;
    }

    _lidar->update_sitl(LIDAR_SITL_INST, lidar_scan);
}



#endif
