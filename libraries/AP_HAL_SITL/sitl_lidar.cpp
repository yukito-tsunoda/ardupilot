/*
  SITL handling

  This simulates a range finder

  Alex Buyval, August 2015
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

#define LIDAR_NUM_RAY 180

/*
  setup the range_finder with new input
  range input is in meters
 */

void SITL_State::_update_lidar(double *lidar_scan)
{
    if (_lidar == NULL) {
        // no range_finder in this sketch
        return;
    }

    //RangeFinder::RangeFinder_State state;
    //state.instance = RANGEFINDER_FRONT_INST;


    //state.distance_cm = range*100;
    for(int i=0; i<LIDAR_NUM_RAY; ++i)
      state.lidar_scan[i] = lidar_scan[i];

    // for(int i=0; i<LIDAR_NUM_RAY; ++i)
    //   fprintf(stdout, "%f ", state.lidar_scan[i]);
    //   fprintf(stderr, "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");

    //state.status = RangeFinder::RangeFinder_Good;

    // Use instance 1 for the front range finder
    //_range_finder->setHIL(RANGEFINDER_FRONT_INST, state);
}

/*
  This function is for a frontal range finder.
  For a downward facing range finder, use '_update_range_finder()'.
  setup the range_finder with new input
  range input is in meters
 */
void SITL_State::_update_lidar_state(bool is_present)
{
    
}


#endif
