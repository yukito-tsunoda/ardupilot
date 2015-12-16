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

#define RANGEFINDER_DOWN_INST     0
#define RANGEFINDER_FRONT_INST    1


/*
  setup the range_finder with new input
  range input is in meters
 */
void SITL_State::_update_range_finder(float range)
{
	if (_range_finder == NULL) {
        // no range_finder in this sketch
        return;
    }

    RangeFinder::RangeFinder_State state;
    state.instance = 0;
    state.distance_cm = range*100;
    state.status = RangeFinder::RangeFinder_Good;

    // Use instance 0 for the downward range finder
    _range_finder->setHIL(RANGEFINDER_DOWN_INST, state);
}

/*
  This function is for a frontal range finder.
  For a downward facing range finder, use '_update_range_finder()'.
  setup the range_finder with new input
  range input is in meters
 */
void SITL_State::_update_range_finder_front(float range)
{
    if (_range_finder == NULL) {
        // no range_finder in this sketch
        return;
    }

    RangeFinder::RangeFinder_State state;
    state.instance = RANGEFINDER_FRONT_INST;
    state.distance_cm = range*100;
    state.status = RangeFinder::RangeFinder_Good;

    // Use instance 1 for the front range finder
    _range_finder->setHIL(RANGEFINDER_FRONT_INST, state);

}

/*
  This function is for a frontal range finder.
  For a downward facing range finder, use '_update_range_finder()'.
  setup the range_finder with new input
  range input is in meters
 */
void SITL_State::_update_range_finder_front_state(bool is_present)
{
    if (_range_finder == NULL) {
        // no range_finder in this sketch
        return;
    }

    // Instances are incremental, so if there are fewer (or equal since they are 0 based)
    // sensors than the front instance, it means there is no front sensor declared
    if (_range_finder->num_sensors() <= RANGEFINDER_FRONT_INST)
        return;

    RangeFinder::RangeFinder_Status prev_status;
    prev_status = _range_finder->status(RANGEFINDER_FRONT_INST);

    // If the status is right (not present = not connected, and vice-versa), returns right away
    // for there is nothing to change
    if (   (!is_present && (prev_status == RangeFinder::RangeFinder_NotConnected))
        || ( is_present && (prev_status != RangeFinder::RangeFinder_NotConnected)))
        return;

    // Applies the new status
    RangeFinder::RangeFinder_State state;
    state.instance = RANGEFINDER_FRONT_INST;

    if (is_present) {
        state.status = RangeFinder::RangeFinder_Good;
        // Re-use the same distance as previous
        state.distance_cm = _range_finder->distance_cm(1);
    } else {
        state.status = RangeFinder::RangeFinder_NotConnected;
        state.distance_cm = 0;
    }

    // Use instance 1 for the front range finder
    _range_finder->setHIL(RANGEFINDER_FRONT_INST, state);

}


#endif
