/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include <stdio.h>

#include "AC_WallNav.h"


extern const AP_HAL::HAL& hal;



const AP_Param::GroupInfo AC_WallNav::var_info[] PROGMEM = {
    // index 0 was used for the old orientation matrix

    // @Param: SPEED
    // @DisplayName: Waypoint Horizontal Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
    // @Units: cm/s
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED",       0, AC_WallNav, _wp_speed_cms, WALLNAV_WP_SPEED),

    // @Param: RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: cm
    // @Range: 100 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RADIUS",      1, AC_WallNav, _wp_radius_cm, WALLNAV_WP_RADIUS),

    // @Param: SPEED_UP
    // @DisplayName: Waypoint Climb Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
    // @Units: cm/s
    // @Range: 0 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED_UP",    2, AC_WallNav, _wp_speed_up_cms, WALLNAV_WP_SPEED_UP),

    // @Param: SPEED_DN
    // @DisplayName: Waypoint Descent Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
    // @Units: cm/s
    // @Range: 0 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("SPEED_DN",    3, AC_WallNav, _wp_speed_down_cms, WALLNAV_WP_SPEED_DOWN),

    // @Param: LOIT_SPEED
    // @DisplayName: Loiter Horizontal Maximum Speed
    // @Description: Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode
    // @Units: cm/s
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("LOIT_SPEED",  4, AC_WallNav, _loiter_speed_cms, WALLNAV_LOITER_SPEED),

    // @Param: ACCEL
    // @DisplayName: Waypoint Acceleration
    // @Description: Defines the horizontal acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL",       5, AC_WallNav, _wp_accel_cms, WALLNAV_ACCELERATION),

    // @Param: ACCEL_Z
    // @DisplayName: Waypoint Vertical Acceleration
    // @Description: Defines the vertical acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL_Z",     6, AC_WallNav, _wp_accel_z_cms, WALLNAV_WP_ACCEL_Z_DEFAULT),

    // @Param: LOIT_JERK
    // @DisplayName: Loiter maximum jerk
    // @Description: Loiter maximum jerk in cm/s/s/s
    // @Units: cm/s/s/s
    // @Range: 500 2000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LOIT_JERK",   7, AC_WallNav, _loiter_jerk_max_cmsss, WALLNAV_LOITER_JERK_MAX_DEFAULT),

    // @Param: LOIT_MAXA
    // @DisplayName: Loiter maximum acceleration
    // @Description: Loiter maximum acceleration in cm/s/s.  Higher values cause the copter to accelerate and stop more quickly.
    // @Units: cm/s/s
    // @Range: 100 981
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LOIT_MAXA",   8, AC_WallNav, _loiter_accel_cmss, WALLNAV_LOITER_ACCEL),

    // @Param: LOIT_MINA
    // @DisplayName: Loiter minimum acceleration
    // @Description: Loiter minimum acceleration in cm/s/s. Higher values stop the copter more quickly when the stick is centered, but cause a larger jerk when the copter stops.
    // @Units: cm/s/s
    // @Range: 100 981
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LOIT_MINA",   9, AC_WallNav, _loiter_accel_min_cmss, WALLNAV_LOITER_ACCEL_MIN),

    // @Param: DIST_WALL
    // @DisplayName: Distance to Wall
    // @Description: Target distance (in cm) to keep from the wall
    // @Units: cm
    // @Range: 1 200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DIST_WALL", 10, AC_WallNav, _desired_dist_to_wall, WALLNAV_DIST_TO_WALL_DEFAULT),

    // @Param: X_SPEED_P
    // @DisplayName: Wall speed P
    // @Description: P factor to convert a distance error into a velocity
    // @Units: cm/s
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("X_SPEED_P", 11, AC_WallNav, _to_wall_speed_p, WALLNAV_TO_WALL_SPEED_P_DEFAULT),

    // @Param: RNG_FLT
    // @DisplayName: Range filter
    // @Description: Filtering on the input range finder (higher = more smoothed but more delayed)
    // @Units: 1
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("RNG_FLT",  12, AC_WallNav, _rng_flt_intensity, WALLNAV_RNG_FLT_DEFAULT),

    // @Param: RNG_MAX
    // @DisplayName: Maximum valid range
    // @Description: Wall ranges above this threshold are considered invalid
    // @Units: cm
    // @Range: 0 50000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("RNG_MAX", 13, AC_WallNav, _rng_max_valid, WALLNAV_RNG_MAX_DEFAULT),

    // @Param: RNG_MIN
    // @DisplayName: Minimum valid range
    // @Description: Wall ranges below this threshold are considered invalid
    // @Units: cm
    // @Range: 0 50000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("RNG_MIN", 14, AC_WallNav, _rng_min_valid, WALLNAV_RNG_MIN_DEFAULT),

    // @Param: RNG_GAP
    // @DisplayName: Maximum range gap
    // @Description: Measures with a gap range above this threshold are considered invalid
    // @Units: cm
    // @Range: 0 10000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("RNG_GAP", 15, AC_WallNav, _rng_max_gap_valid, WALLNAV_RNG_GAP_DEFAULT),

    // @Param: BLIND_TIME
    // @DisplayName: Delay before blind
    // @Description: If no valid range measures arrive for more than this threshold, switches to blind wall control
    // @Units: s
    // @Range: 1 20
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("BLIND_TIME", 16, AC_WallNav, _no_rng_blind_delay, WALLNAV_BLIND_TIME_DEFAULT),

    AP_GROUPEND
};


// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_WallNav::AC_WallNav(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control),
    _wall_yaw_normal_deg100(0),
    _to_wall_cos_yaw(0.0),
    _to_wall_sin_yaw(0.0),
    _pilot_accel_wf_x_cms(0),
    _pilot_accel_wf_y_cms(0),

    _wp_last_update(0),
    _track_length(0.0f),
    _track_desired(0.0f),
    _limited_speed_xy_cms(0.0f),
    _track_accel(0.0f),
    _track_speed(0.0f),
    _track_leash_length(0.0f),
    _slow_down_dist(0.0f),
   // _desired_dist_to_wall_with_safety(WALLNAV_SAFE_DIST_TO_WALL),
    //_est_real_wall_pos_x_wf(0.0f),
    _wall_rng_filt(0.0f),
    _last_valid_wall_rng(0),
    _is_ctrl_wall_blind(true),
    _isObservedSideLeft(false),
    _last_uav_pos_wf_x(0.0f),
    _target_pos_wf_x_on_wall_blind(0.0f),
    _pos_x_at_last_valid_wall_rng(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init flags
    _flags.reached_destination = false;
    //_flags.fast_waypoint = false;
    _flags.slowing_down = false;
    _flags.recalc_wp_leash = false;
    _flags.new_wp_destination = false;
    _flags.wall_x_rng_ctrl_active = false;
    _flags.reset_desired_vel_to_pos = true;
}



//------------------------------------------------
// Controller initialization - LOITER

///
/// loiter controller on the wall Y axis (parallel to the wall)
/// special controller on the wall X axis (toward the wall)
///

void AC_WallNav::init_wfloiter_controller_target(const float wall_yaw_normal_deg) {
    init_wfloiter_controller_target((int32_t)(wall_yaw_normal_deg * 100),
                             cosf(wall_yaw_normal_deg * DEG_TO_RAD),
                             sinf(wall_yaw_normal_deg * DEG_TO_RAD));
}

/// init_loiter_target - initialize's loiter position and feed-forward velocity from current pos and velocity
void AC_WallNav::init_wfloiter_controller_target(const int32_t wall_yaw_normal_deg100, const float wall_yaw_normal_cos, const float wall_yaw_normal_sin)
{
    const Vector3f& curr_pos = _inav.get_position();
    const Vector3f& curr_vel = _inav.get_velocity();

    // initialize position controller
    _pos_control.init_xy_controller();

    // initialize pos controller speed and acceleration
    _pos_control.set_speed_xy(_loiter_speed_cms);
    _pos_control.set_accel_xy(_loiter_accel_cmss);

    // set target position
    _pos_control.set_xy_target(curr_pos.x, curr_pos.y);

    // initialize desired accel and add fake wind
    _loiter_desired_accel.x = (_loiter_accel_cmss)*curr_vel.x/_loiter_speed_cms;
    _loiter_desired_accel.y = (_loiter_accel_cmss)*curr_vel.y/_loiter_speed_cms;

    // initialize pilot input
    _pilot_accel_wf_x_cms = 0;
    _pilot_accel_wf_y_cms = 0;

    _desired_speed_x_cms = 0;

    _desired_wf_vel_xy.x = 0;
    _desired_wf_vel_xy.y = 0;

    _wall_yaw_normal_deg100 = wall_yaw_normal_deg100;
    _to_wall_cos_yaw = wall_yaw_normal_cos;
    _to_wall_sin_yaw = wall_yaw_normal_sin;
    _target_dist_to_wall = (int32_t)_desired_dist_to_wall;

    // Resets the wall range filter
    _wall_rng_filt = 0.0f;
    _last_valid_wall_rng = 0;
    _is_ctrl_wall_blind = true;

    // Keeps track of the X (wall frame) position at the current valid range measure
    float curr_pos_wf_x;
    tranform_pt_to_wall_frame_x(_inav.get_position(), curr_pos_wf_x);
    _pos_x_at_last_valid_wall_rng = curr_pos_wf_x;

    // move current vehicle velocity into feed forward velocity - only the wall lateral part
    Vector3f curr_vel_wf;
    tranform_vec_to_wall_frame(curr_vel, curr_vel_wf);
    _desired_wf_vel_xy.x = 0.0f;                // toward known wall
    _desired_wf_vel_xy.y = curr_vel_wf.y;       // lateral to known wall
    send_desired_vel_xy_to_pos_controller();

    printf("target wall dist       = %d [cm]\n", _target_dist_to_wall);
    printf("target wall yaw normal = %d [deg]\n", wall_yaw_normal_deg100/100);
}



//------------------------------------------------
// Controller initialization - WAYPOINT

/// wp_and_spline_init - initialise straight line and spline waypoint controllers
///     updates target roll, pitch targets and I terms based on vehicle lean angles
///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
void AC_WallNav::init_wfwpt_controller(const Vector3f& start_loc, const Vector3f& origin, const Vector3f& destination,
                                       const float target_dist, const bool is_observed_side_left)
{
    // check _wp_accel_cms is reasonable
    if (_wp_accel_cms <= 0) {
        _wp_accel_cms.set_and_save(WALLNAV_ACCELERATION);
    }

    // initialize position controller
    _pos_control.init_xy_controller();

    // initialize position controller speed and acceleration
    _pos_control.set_speed_xy(_wp_speed_cms);
    _pos_control.set_accel_xy(_wp_accel_cms);
    _pos_control.set_speed_z(-_wp_speed_down_cms, _wp_speed_up_cms);
    _pos_control.set_accel_z(_wp_accel_z_cms);
    _pos_control.calc_leash_length_xy();
    _pos_control.calc_leash_length_z();

    // Resets the wall range filter
    _wall_rng_filt = 0.0f;
    _last_valid_wall_rng = 0;
    _is_ctrl_wall_blind = true;
    _last_uav_pos_wf_x = 0.0f;
    _target_pos_wf_x_on_wall_blind = 0.0f;

    set_wall_1st_and_end_points(origin, destination, target_dist, is_observed_side_left);
}


//------------------------------------------------
// Controller inputs - LOITER

/// set_pilot_desired_acceleration - sets pilot desired acceleration from roll and pitch stick input
void AC_WallNav::set_pilot_desired_acceleration(float control_roll, float control_pitch)
{
    // convert pilot input to desired acceleration in cm/s/s, in wall frame
    _pilot_accel_wf_x_cms = -control_pitch * _loiter_accel_cmss / 4500.0f;
    _pilot_accel_wf_y_cms =  control_roll * _loiter_accel_cmss / 4500.0f;
}


//------------------------------------------------
// Controller inputs - WALL DISTANCE

void AC_WallNav::update_dist_to_wall_sensor_measure(int16_t front_sonar_rng_cm)
{
    bool isMeasureValid = true;


    if ((front_sonar_rng_cm > _rng_max_valid) ||
        (front_sonar_rng_cm < _rng_min_valid) ||
        (!_is_ctrl_wall_blind && (abs((int32_t)front_sonar_rng_cm - (int32_t)_wall_rng_filt) > _rng_max_gap_valid))) {
        // Invalid measure
        isMeasureValid = false;
    } else {
        float pastRngsWeight = _rng_flt_intensity;

        // If the controller is currently wall blind, the first valid range overwrites past range history
        if (_is_ctrl_wall_blind)
            pastRngsWeight = 0.0;

        // Low pass filter
        _wall_rng_filt = (float)front_sonar_rng_cm * (1.0 - pastRngsWeight) + _wall_rng_filt * pastRngsWeight;
        isMeasureValid = true;
    }

    if (isMeasureValid) {
        _last_valid_wall_rng = hal.scheduler->millis();
        if (_is_ctrl_wall_blind) {
            // Restores the controller
            _is_ctrl_wall_blind = false;
            printf("Wall blind removed !\n");
        }

        // Keeps track of the X (wall frame) position at the current valid range measure
        float curr_pos_wf_x;
        tranform_pt_to_wall_frame_x(_inav.get_position(), curr_pos_wf_x);
        _pos_x_at_last_valid_wall_rng = curr_pos_wf_x;

    } else if (!_is_ctrl_wall_blind) {
        // Invalid measure, but the controller is still active
        uint32_t time_since_valid_rng = (hal.scheduler->millis() - _last_valid_wall_rng) / 1000;

        if (time_since_valid_rng > (uint32_t)_no_rng_blind_delay) {
            // Declares the controller wall blind
            _is_ctrl_wall_blind = true;
            printf("Wall blind !\n");
        }
    }
}

void AC_WallNav::update_sensor_lost()
{
    check_sensor_wall_blind();
}

// Sensor measures arrive at 10Hz, while the controller runs at 100Hz, so 9 times over 10,
// this function is called just to indicate that the next measure is not yet available.
void AC_WallNav::check_sensor_wall_blind()
{
    // If already blind, there is nothing to do (this function cannot remove it)
    if (_is_ctrl_wall_blind)
        return;

    uint32_t time_since_valid_rng = (hal.scheduler->millis() - _last_valid_wall_rng) / 1000;

    if (time_since_valid_rng > (uint32_t)_no_rng_blind_delay) {
        // Declares the controller wall blind
        _is_ctrl_wall_blind = true;
        printf("Wall blind !\n");
    }

    // Updates the estimated forward travel



}



//------------------------------------------------
// Controller update - LOITER

// update_controller - run the loiter Y controller and the special X  - gets called at 100hz (APM) or 400hz (PX4)
void AC_WallNav::update_wfloiter_nav(float ekfGndSpdLimit, float ekfNavVelGainScaler)
{
    // calculate dt
    float dt = _pos_control.time_since_last_xy_update();

    // run at poscontrol update rate.
    if (dt >= _pos_control.get_dt_xy()) {
        // sanity check dt
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // Loiter controller on Y (lateral) axis
        calc_wfloiter_desired_velocity_y(dt,ekfGndSpdLimit);

        // WallFollow controller on X (toward) axis
        calc_wfloiter_desired_velocity_x(dt,ekfGndSpdLimit);

        send_desired_vel_xy_to_pos_controller();
        //_pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_LIMITED_AND_VEL_FF, ekfNavVelGainScaler, true);
    }
}


//------------------------------------------------
// Controller update - WAYPOINT

/// update_wfwpt_nav - run the wp controller - should be called at 100hz or higher
void AC_WallNav::update_wfwpt_nav()
{
    // calculate dt
    float dt = _pos_control.time_since_last_xy_update();

    // update at poscontrol update rate
    if (dt >= _pos_control.get_dt_xy()) {
        // sanity check dt
        if (dt >= 0.2f) {
            dt = 0.0f;
        }

        // In case the parameter value is modified
        _target_dist_to_wall = (int32_t)_desired_dist_to_wall;

        // advance the target if necessary
        advance_wfwp_target_along_track(dt);

        // freeze feedforwards during known discontinuities
        if (_flags.new_wp_destination) {
            _flags.new_wp_destination = false;
            _pos_control.freeze_ff_xy();
        }
        _pos_control.freeze_ff_z();

        //_pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_ONLY, 1.0f, false);
        check_wp_leash_length();

        _wp_last_update = hal.scheduler->millis();
    }
}


//------------------------------------------------
// Controller core - LOITER

/// calc_loiter_desired_velocity - updates desired velocity (i.e. feed forward) with pilot requested acceleration and fake wind resistance
///     updated velocity sent directly to position controller
///     Should run at 100 Hz or more
void AC_WallNav::calc_wfloiter_desired_velocity_y(float nav_dt, float ekfGndSpdLimit)
{
    // calculate a loiter speed limit which is the minimum of the value set by the WALLNAV_LOITER_SPEED
    // parameter and the value set by the EKF to observe optical flow limits
    float gnd_speed_limit_cms = min(_loiter_speed_cms,ekfGndSpdLimit*100.0f);
    gnd_speed_limit_cms = max(gnd_speed_limit_cms, 10.0f);

    // range check nav_dt
    if( nav_dt < 0 ) {
        return;
    }

    // check loiter speed and avoid divide by zero
    if(gnd_speed_limit_cms < WALLNAV_LOITER_SPEED_MIN) {
        gnd_speed_limit_cms = WALLNAV_LOITER_SPEED_MIN;
    }

    _pos_control.set_speed_xy(gnd_speed_limit_cms);
    _pos_control.set_accel_xy(_loiter_accel_cmss);

    // rotate pilot input to lat/lon frame
    Vector2f desired_accel;
    //desired_accel.x = (_pilot_accel_fwd_cms*_ahrs.cos_yaw() - _pilot_accel_y_cms*_ahrs.sin_yaw());
    //desired_accel.x = 0;
    //desired_accel.y = (_pilot_accel_wf_x_cms*_ahrs.sin_yaw() + _pilot_accel_wf_y_cms*_ahrs.cos_yaw());
    desired_accel.x = 0;
    desired_accel.y = _pilot_accel_wf_y_cms;

    // calculate the difference
    Vector2f des_accel_diff = (desired_accel - _loiter_desired_accel);

    // constrain and scale the desired acceleration
    float des_accel_change_total = pythagorous2(des_accel_diff.x, des_accel_diff.y);
    float accel_change_max = _loiter_jerk_max_cmsss * nav_dt;

    if (_loiter_jerk_max_cmsss > 0.0f && des_accel_change_total > accel_change_max && des_accel_change_total > 0.0f) {
        //des_accel_diff.x = accel_change_max * des_accel_diff.x/des_accel_change_total;
        des_accel_diff.y = accel_change_max * des_accel_diff.y/des_accel_change_total;
    }

    // adjust the desired acceleration
    _loiter_desired_accel += des_accel_diff;

    // get pos_control's feed forward velocity
    const Vector3f &desired_vel_3d = _pos_control.get_desired_velocity();
    //Vector2f desired_vel(desired_vel_3d.x,desired_vel_3d.y);
    //Vector2f desired_wf_vel(0.0f, desired_vel_3d.y);
    Vector2f desired_wf_vel(0.0f, 0.0f);

    // Converts the previous desired velocity from earth frame to wall frame
    desired_wf_vel.y = -desired_vel_3d.x * _to_wall_sin_yaw + desired_vel_3d.y * _to_wall_cos_yaw;


    // add pilot commanded acceleration
    //desired_wf_vel.x += _loiter_desired_accel.x * nav_dt;
    desired_wf_vel.y += _loiter_desired_accel.y * nav_dt;

    float desired_speed = desired_wf_vel.length();

    if (!is_zero(desired_speed)) {
        Vector2f desired_vel_norm = desired_wf_vel/desired_speed;
        float drag_speed_delta = -_loiter_accel_cmss*nav_dt*desired_speed/gnd_speed_limit_cms;

        if (_pilot_accel_wf_x_cms == 0 && _pilot_accel_wf_y_cms == 0) {
            drag_speed_delta = min(drag_speed_delta,-_loiter_accel_min_cmss*nav_dt);
        }

        desired_speed = max(desired_speed+drag_speed_delta,0.0f);
        desired_wf_vel = desired_vel_norm*desired_speed;
    }

    // Apply EKF limit to desired velocity -  this limit is calculated by the EKF and adjusted as required to ensure certain sensor limits are respected (eg optical flow sensing)
    //float horizSpdDem = sqrtf(sq(desired_vel.x) + sq(desired_vel.y));
    float horizSpdDem = fabs(desired_wf_vel.y);
    if (horizSpdDem > gnd_speed_limit_cms) {
        //desired_vel.x = desired_vel.x * gnd_speed_limit_cms / horizSpdDem;
        desired_wf_vel.y = desired_wf_vel.y * gnd_speed_limit_cms / horizSpdDem;
    }

    // send adjusted feed forward velocity back to position controller
    // TODO replace
    //_pos_control.set_desired_velocity_xy(desired_wf_vel.x,desired_vel.y);
    _desired_wf_vel_xy.y = desired_wf_vel.y;
}

///     Should run at 100 Hz or more
void AC_WallNav::calc_wfloiter_desired_velocity_x(float nav_dt, float ekfGndSpdLimit)
{
    int32_t wall_dist_error;

    // range check nav_dt
    if( nav_dt < 0 )
        return;

    if (!_is_ctrl_wall_blind) {
        wall_dist_error = (int32_t)_target_dist_to_wall - (int32_t)_wall_rng_filt;

        // Removes the estimated traveled distance since last valid range measure
        float curr_pos_wf_x;
        tranform_pt_to_wall_frame_x(_inav.get_position(), curr_pos_wf_x);
        wall_dist_error -= (int32_t)(curr_pos_wf_x - _pos_x_at_last_valid_wall_rng);

        // speed shall be positive if the uav needs to move closer to the wall
        _desired_speed_x_cms = -wall_dist_error * _to_wall_speed_p;

        _desired_wf_vel_xy.x = _desired_speed_x_cms;
    } else {
        // On wall blind, no forward movement
        _desired_wf_vel_xy.x = 0.0f;
    }

    /*static uint16_t s_counter = 0;
    s_counter++;
    if (s_counter > 50) {
        printf("desired_wf_vel_xy.x = %.1f [cm/s]\n", _desired_wf_vel_xy.x);
        s_counter = 0;
    }*/
}


//------------------------------------------------
// Controller core - WAYPOINT

/// advance_wfwp_target_along_track - move target location along track from origin to destination
/*void AC_WallNav::advance_wfwp_target_along_track(float dt)
{
    float track_covered_horiz;  // distance (in cm) along the track that the vehicle has traveled.  Measured by drawing a perpendicular line from the track to the vehicle.
    Vector3f track_error;       // distance error (in cm) from the track_covered position (i.e. closest point on the line to the vehicle) and the vehicle
    float track_desired_max;    // the farthest distance (in cm) along the track that the leash will allow
    float track_leash_slack;    // additional distance (in cm) along the track from our track_covered position that our leash will allow
    bool reached_leash_limit = false;   // true when track has reached leash limit and we need to slow down the target point

    // get current location, expressed in wall frame
    Vector3f curr_pos_wf;
    tranform_pt_to_wall_frame(_inav.get_position(), curr_pos_wf);

    Vector3f curr_delta = curr_pos_wf - _wall_1st_pt_wf;

    // curr_delta.x is directly the cross-track
    // curr_delta.y is directly the track

    float track_error_horiz;
    float track_error_z;

    track_covered_horiz = curr_delta.y;
    track_error_horiz = curr_delta.x;       // TDODO
    track_error_z = fabsf(curr_delta.z);        // calculate the vertical error

    // If the X controller is active, then the X movement / speed do not influence the YZ motion
    // TODO: maybe this is not the best way ?
    if (_flags.wall_x_rng_ctrl_active) {
        track_error_horiz = 0;
    }

    _wall_rng_filt
    // calculate how far along the track we are
    //track_covered_horiz = curr_delta.x * _wall_horiz_unit_vec.x + curr_delta.y * _wall_horiz_unit_vec.y;

    //Vector3f track_covered_pos = _wall_unit_vec * track_covered_horiz;      // both horizontal and vertical
    //track_error = curr_delta - track_covered_pos;

    // calculate the horizontal error
    //float track_error_xy = pythagorous2(track_error.x, track_error.y);

    // calculate the vertical error
    //float track_error_z = fabsf(track_error.z);

    // get position control leash lengths
    float leash_xy = _pos_control.get_leash_xy();
    float leash_z;
    if (track_error.z >= 0) {
        leash_z = _pos_control.get_leash_up_z();
    }else{
        leash_z = _pos_control.get_leash_down_z();
    }

    // calculate how far along the track we could move the intermediate target before reaching the end of the leash
    track_leash_slack = min(_track_leash_length*(leash_z-track_error_z)/leash_z, _track_leash_length*(leash_xy-track_error_horiz)/leash_xy);
    if (track_leash_slack < 0) {
        track_desired_max = track_covered_horiz;
    }else{
        track_desired_max = track_covered_horiz + track_leash_slack;
    }

    // check if target is already beyond the leash
    if (_track_desired > track_desired_max) {
        reached_leash_limit = true;
    }

    // get current velocity in wall frame
    const Vector3f &curr_vel = _inav.get_velocity();
    Vector3f curr_vel_wf;
    tranform_vec_to_wall_frame(curr_vel, curr_vel_wf);


    // get speed along track
    float speed_along_track = (curr_vel.x * _wall_unit_vec_wf.x) + (curr_vel.y * _wall_unit_vec_wf.y) + (curr_vel.z * _wall_unit_vec_wf.z);

    // If the X controller is active, then the X movement / speed does not influence the YZ motion.
    // Only adds the X speed if the X controller is disabled
    // TODO: maybe this is not the best way ?

    //float speed_along_track = (curr_vel.y * _wall_unit_vec_wf.y) + (curr_vel.z * _wall_unit_vec_wf.z);
    //if (!_flags.wall_x_rng_ctrl_active) {
   //     speed_along_track += curr_vel.x * _wall_unit_vec_wf.x;
    //}


    // calculate point at which velocity switches from linear to sqrt
    float linear_velocity = _wp_speed_cms;
    float kP = _pos_control.get_pos_xy_kP();
    if (kP >= 0.0f) {   // avoid divide by zero
        linear_velocity = _track_accel/kP;
    }

    // let the limited_speed_y_cms be some range above or below current velocity along track
    if (speed_along_track < -linear_velocity) {
        // we are traveling fast in the opposite direction of travel to the waypoint so do not move the intermediate point
        _limited_speed_y_cms = 0;
    }else{
        // increase intermediate target point's velocity if not yet at the leash limit
        if(dt > 0 && !reached_leash_limit) {
            _limited_speed_y_cms += 2.0f * _track_accel * dt;
        }
        // do not allow speed to be below zero or over top speed
        _limited_speed_y_cms = constrain_float(_limited_speed_y_cms, 0.0f, _track_speed);

        // check if we should begin slowing down
        //if (!_flags.fast_waypoint) {
            float dist_to_dest = _track_length - _track_desired;
            if (!_flags.slowing_down && dist_to_dest <= _slow_down_dist) {
                _flags.slowing_down = true;
            }
            // if target is slowing down, limit the speed
            if (_flags.slowing_down) {
                _limited_speed_y_cms = min(_limited_speed_y_cms, get_slow_down_speed(dist_to_dest, _track_accel));
            }
        //}

        // if our current velocity is within the linear velocity range limit the intermediate point's velocity to
        // be no more than the linear_velocity above or below our current velocity
        if (fabsf(speed_along_track) < linear_velocity) {
            _limited_speed_y_cms = constrain_float(_limited_speed_y_cms,speed_along_track-linear_velocity,speed_along_track+linear_velocity);
        }
    }
    // advance the current target
    if (!reached_leash_limit) {
        _track_desired += _limited_speed_y_cms * dt;

        // reduce speed if we reach end of leash
        if (_track_desired > track_desired_max) {
            _track_desired = track_desired_max;
            _limited_speed_y_cms -= 2.0f * _track_accel * dt;
            if (_limited_speed_y_cms < 0.0f) {
                _limited_speed_y_cms = 0.0f;
            }
        }
    }

    // do not let desired point go past the end of the track unless it's a fast waypoint
    //if (!_flags.fast_waypoint) {
        _track_desired = constrain_float(_track_desired, 0, _track_length);
    //} else {
    //    _track_desired = constrain_float(_track_desired, 0, _track_length + WALLNAV_WP_FAST_OVERSHOOT_MAX);
    //}

    // recalculate the desired position
    //_pos_control.set_pos_target(_origin + _pos_delta_unit * _track_desired);

    //_desired_wf_pos_xyz = _wall_1st_pt_wf + _wall_normal_unit_vec_wf * WALLNAV_SAFE_DIST_TO_WALL;
    _desired_wf_pos_xyz.y = _wall_1st_pt_wf.y + _wall_unit_vec_wf.y * _track_desired;

    if (!_flags.wall_x_rng_ctrl_active) {
        _desired_wf_pos_xyz.x = _est_real_wall_pos_x_wf - _desired_dist_to_wall;
    } else {
        // Leaves the X axis position command to the wall-follow X controller
       // _desired_wf_pos_xyz.y = _wall_1st_pt_wf.y + _wall_unit_vec_wf.y * _track_desired;
       // _desired_wf_pos_xyz.z = _wall_1st_pt_wf.z + _wall_unit_vec_wf.z * _track_desired;
    }

    send_desired_pos_xyz_to_pos_controller();
    // TODO: review


    // check if we've reached the waypoint
    if( !_flags.reached_destination ) {
        if( _track_desired >= _track_length ) {
            // "fast" waypoints are complete once the intermediate point reaches the destination
            //if (_flags.fast_waypoint) {
            //    _flags.reached_destination = true;
            //}else{
                // regular waypoints also require the copter to be within the waypoint radius
                Vector3f dist_to_dest_wf = curr_pos_wf - _wall_end_pt_wf;
                dist_to_dest_wf.x = 0;     // the x axis (towards the wall) does not count
                if( dist_to_dest_wf.length() <= _wp_radius_cm ) {
                    _flags.reached_destination = true;
                }
            //}
        }
    }
}*/


/// advance_wp_target_along_track - move target location along track from origin to destination
void AC_WallNav::advance_wfwp_target_along_track(float dt)
{
    float track_covered;        // distance (in cm) along the track that the vehicle has traveled.  Measured by drawing a perpendicular line from the track to the vehicle.
    Vector3f track_error;       // distance error (in cm) from the track_covered position (i.e. closest point on the line to the vehicle) and the vehicle
    float track_desired_max;    // the farthest distance (in cm) along the track that the leash will allow
    float track_leash_slack;    // additional distance (in cm) along the track from our track_covered position that our leash will allow
    bool reached_leash_limit = false;   // true when track has reached leash limit and we need to slow down the target point

    // get current location, expressed in wall frame
    Vector3f curr_pos_wf;
    tranform_pt_to_wall_frame(_inav.get_position(), curr_pos_wf);
    Vector3f curr_delta = curr_pos_wf - _wall_1st_pt_wf;

    // calculate how far along the track we are (X_wf does not count)
    //track_covered = curr_delta.x * _pos_delta_unit.x + curr_delta.y * _pos_delta_unit.y + curr_delta.z * _pos_delta_unit.z;
    track_covered = curr_delta.y * _wall_unit_vec_wf.y + curr_delta.z * _wall_unit_vec_wf.z;

    Vector3f track_covered_pos = _wall_unit_vec_wf * track_covered;     //_pos_delta_unit * track_covered;
    track_error = curr_delta - track_covered_pos;

    //if (!_is_ctrl_wall_blind)
    // TODO: + or - depending on observed side
    if (!_is_ctrl_wall_blind) {
        if (_isObservedSideLeft)
            track_error.x = _wall_rng_filt - _desired_dist_to_wall;
        else
            track_error.x = _desired_dist_to_wall - _wall_rng_filt;

        _last_uav_pos_wf_x = curr_delta.x;
        _target_pos_wf_x_on_wall_blind = _last_uav_pos_wf_x;        // so it will be used in the next loop if wall blind
    } else {
        // TODO: review sign
        track_error.x = _target_pos_wf_x_on_wall_blind - curr_delta.x;
    }


    // calculate the horizontal error
    float track_error_xy = pythagorous2(track_error.x, track_error.y);

    // calculate the vertical error
    float track_error_z = fabsf(track_error.z);

    // get position control leash lengths
    float leash_xy = _pos_control.get_leash_xy();
    float leash_z;
    if (track_error.z >= 0) {
        leash_z = _pos_control.get_leash_up_z();
    }else{
        leash_z = _pos_control.get_leash_down_z();
    }

    // calculate how far along the track we could move the intermediate target before reaching the end of the leash
    track_leash_slack = min(_track_leash_length*(leash_z-track_error_z)/leash_z, _track_leash_length*(leash_xy-track_error_xy)/leash_xy);
    if (track_leash_slack < 0) {
        track_desired_max = track_covered;
    }else{
        track_desired_max = track_covered + track_leash_slack;
    }

    // check if target is already beyond the leash
    if (_track_desired > track_desired_max) {
        reached_leash_limit = true;
    }

    // get current velocity in wall frame
    Vector3f curr_vel_wf;
    const Vector3f &curr_vel = _inav.get_velocity();
    tranform_vec_to_wall_frame(curr_vel, curr_vel_wf);

    // get speed along track
    float speed_along_track = curr_vel_wf.x * _wall_unit_vec_wf.x + curr_vel_wf.y * _wall_unit_vec_wf.y + curr_vel_wf.z * _wall_unit_vec_wf.z;

    // calculate point at which velocity switches from linear to sqrt
    float linear_velocity = _wp_speed_cms;
    float kP = _pos_control.get_pos_xy_kP();
    if (kP >= 0.0f) {   // avoid divide by zero
        linear_velocity = _track_accel/kP;
    }

    // let the limited_speed_xy_cms be some range above or below current velocity along track
    if (speed_along_track < -linear_velocity) {
        // we are traveling fast in the opposite direction of travel to the waypoint so do not move the intermediate point
        _limited_speed_xy_cms = 0;
    }else{
        // increase intermediate target point's velocity if not yet at the leash limit
        if(dt > 0 && !reached_leash_limit) {
            _limited_speed_xy_cms += 2.0f * _track_accel * dt;
        }
        // do not allow speed to be below zero or over top speed
        _limited_speed_xy_cms = constrain_float(_limited_speed_xy_cms, 0.0f, _track_speed);

        // check if we should begin slowing down
        //if (!_flags.fast_waypoint) {
            float dist_to_dest = _track_length - _track_desired;
            if (!_flags.slowing_down && dist_to_dest <= _slow_down_dist) {
                _flags.slowing_down = true;
            }
            // if target is slowing down, limit the speed
            if (_flags.slowing_down) {
                _limited_speed_xy_cms = min(_limited_speed_xy_cms, get_slow_down_speed(dist_to_dest, _track_accel));
            }
        //}

        // if our current velocity is within the linear velocity range limit the intermediate point's velocity to be no more than the linear_velocity above or below our current velocity
        if (fabsf(speed_along_track) < linear_velocity) {
            _limited_speed_xy_cms = constrain_float(_limited_speed_xy_cms,speed_along_track-linear_velocity,speed_along_track+linear_velocity);
        }
    }
    // advance the current target
    if (!reached_leash_limit) {
        _track_desired += _limited_speed_xy_cms * dt;

        // reduce speed if we reach end of leash
        if (_track_desired > track_desired_max) {
            _track_desired = track_desired_max;
            _limited_speed_xy_cms -= 2.0f * _track_accel * dt;
            if (_limited_speed_xy_cms < 0.0f) {
                _limited_speed_xy_cms = 0.0f;
            }
        }
    }

    // do not let desired point go past the end of the track unless it's a fast waypoint
    //if (!_flags.fast_waypoint) {
        _track_desired = constrain_float(_track_desired, 0, _track_length);
    //} else {
    //    _track_desired = constrain_float(_track_desired, 0, _track_length + WALLNAV_WP_FAST_OVERSHOOT_MAX);
    //}

    // recalculate the desired position
    //_pos_control.set_pos_target(_origin + _pos_delta_unit * _track_desired);

    _desired_wf_pos_xyz = _wall_1st_pt_wf + _wall_unit_vec_wf * _track_desired;

    // Special handling of X
    _desired_wf_pos_xyz.x = curr_pos_wf.x;

    if (!_is_ctrl_wall_blind) {
        if (_isObservedSideLeft)
            _desired_wf_pos_xyz.x += _wall_rng_filt - _desired_dist_to_wall;
        else
            _desired_wf_pos_xyz.x += _desired_dist_to_wall - _wall_rng_filt;

    } else {
        // TODO: review sign
        _desired_wf_pos_xyz.x += _target_pos_wf_x_on_wall_blind - curr_delta.x;
    }

    send_desired_pos_xyz_to_pos_controller();

    // check if we've reached the waypoint
    if( !_flags.reached_destination ) {
        if( _track_desired >= _track_length ) {
            // "fast" waypoints are complete once the intermediate point reaches the destination
            //if (_flags.fast_waypoint) {
            //    _flags.reached_destination = true;
           // }else{
                // regular waypoints also require the copter to be within the waypoint radius
                Vector3f dist_to_dest = curr_pos_wf - _wall_end_pt_wf;      // curr_pos - _destination;
                dist_to_dest.x = 0;     // X axis does not count
                if( dist_to_dest.length() <= _wp_radius_cm ) {
                    _flags.reached_destination = true;
                }
            //}
        }
    }
}



//------------------------------------------------
// Controller parameters

/// set_speed_xy - allows main code to pass target horizontal velocity for wp navigation
void AC_WallNav::set_speed_xy(float speed_cms)
{
    // range check new target speed and update position controller
    if (speed_cms >= WALLNAV_WP_SPEED_MIN) {
        _wp_speed_cms = speed_cms;
        _pos_control.set_speed_xy(_wp_speed_cms);
        // flag that wp leash must be recalculated
        _flags.recalc_wp_leash = true;
    }
}

//------------------------------------------------
// Controller utilities - LOITER

/// get_loiter_stopping_point_xy - returns vector to stopping point based on a horizontal position and velocity
void AC_WallNav::get_loiter_stopping_point_xy(Vector3f& stopping_point) const
{
    _pos_control.get_stopping_point_xy(stopping_point);
}



//------------------------------------------------
// Controller utilities - WAYPOINT

/// set_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
void AC_WallNav::set_wall_1st_and_end_points(const Vector3f& wall_1st_pt, const Vector3f& wall_end_pt,
                                             const float target_dist, const bool is_observed_side_left)
{
    //TODO: Review, simplify

    // store origin and destination locations
    _wall_1st_pt = wall_1st_pt;
    _wall_end_pt = wall_end_pt;
    Vector3f pos_delta = _wall_end_pt - _wall_1st_pt;

    _track_length = pos_delta.length(); // get track length

    // calculate each axis' percentage of the total distance to the destination
    if (is_zero(_track_length)) {
        // avoid possible divide by zero
        _wall_unit_vec.x = 0;
        _wall_unit_vec.y = 0;
        _wall_unit_vec.z = 0;
    }else{
        _wall_unit_vec = pos_delta/_track_length;
    }

    // TODO:
    _isObservedSideLeft = is_observed_side_left;

    if (target_dist > 0)
        _target_dist_to_wall = (int32_t)target_dist;
    else
        _target_dist_to_wall = (int32_t)_desired_dist_to_wall;

    // Calculates the horizontal direction vector
    Vector2f pos_delta_horiz = Vector2f(_wall_end_pt.x - _wall_1st_pt.x, _wall_end_pt.y - _wall_1st_pt.y);
    _track_horiz_length = pos_delta_horiz.length();
    if (is_zero(_track_horiz_length)) {
        // avoid possible divide by zero
        _wall_horiz_unit_vec.x = 0;
        _wall_horiz_unit_vec.y = 0;
    }else{
        _wall_horiz_unit_vec = pos_delta_horiz/_track_horiz_length;
    }

    // Calculates the normal to the wall
    float wall_yaw_normal = atan2f(_wall_horiz_unit_vec.y, _wall_horiz_unit_vec.x);
    _wall_yaw_normal_deg100 = RadiansToCentiDegrees(wall_yaw_normal);


    // Calculates every variables in wall frame coordinates
    _wall_1st_pt_wf = Vector3f(0,0,_wall_1st_pt.z);
    _wall_end_pt_wf = Vector3f(0,-_track_horiz_length,_wall_end_pt_wf.z);

    _wall_horiz_unit_vec_wf = Vector2f(0, -1);
    _wall_unit_vec_wf = Vector3f(0, sqrt(_wall_unit_vec.x*_wall_unit_vec.x + _wall_unit_vec.y*_wall_unit_vec.y), _wall_unit_vec.z);
    _wall_normal_unit_vec_wf = Vector3f(-1, 0, 0);

    // Resets the estimated safe distance to the wall, for there
    // is not yet any valid sonar measurements
    //_desired_dist_to_wall_with_safety = max(WALLNAV_SAFE_DIST_TO_WALL, (float)_desired_dist_to_wall);
   // _est_real_wall_pos_x_wf = 0.0f;

    // calculate leash lengths
    calculate_wp_leash_length();

    // initialise intermediate point to the wall start (at a safe distance from it)
    //_pos_control.set_pos_target(origin);
    //_desired_wf_pos_xyz = _wall_1st_pt_wf + _wall_normal_unit_vec_wf * (_est_real_wall_pos_x_wf - _desired_dist_to_wall);
    _desired_wf_pos_xyz = _wall_1st_pt_wf;

    _track_desired = 0;                 // target is at beginning of track
    _flags.reached_destination = false;
    //_flags.fast_waypoint = false;       // default waypoint back to slow
    _flags.slowing_down = false;        // target is not slowing down yet
    _flags.new_wp_destination = true;   // flag new waypoint so we can freeze the pos controller's feed forward and smooth the transition
    _flags.wall_x_rng_ctrl_active = false;      // until valid sonar measurements arrive, the wall-follow X controller is unactive

    // initialise the limited speed to current speed along the track
    /*const Vector3f &curr_vel = _inav.get_velocity();
    // get speed along track (note: we convert vertical speed into horizontal speed equivalent)
    float speed_along_track = curr_vel.x * _pos_delta_unit.x + curr_vel.y * _pos_delta_unit.y + curr_vel.z * _pos_delta_unit.z;
    _limited_speed_y_cms = constrain_float(speed_along_track,0,_wp_speed_cms);
    */
    send_desired_pos_xyz_to_pos_controller();
}


// check_wp_leash_length - check if waypoint leash lengths need to be recalculated
//  should be called after _pos_control.update_xy_controller which may have changed the position controller leash lengths
void AC_WallNav::check_wp_leash_length()
{
    // exit immediately if recalc is not required
    if (_flags.recalc_wp_leash) {
        calculate_wp_leash_length();
    }
}

/// calculate_wp_leash_length - calculates horizontal and vertical leash lengths for waypoint controller
void AC_WallNav::calculate_wp_leash_length()
{
    // length of the unit direction vector in the horizontal
    //float pos_delta_unit_wf_y = pythagorous2(_pos_delta_unit.x, _pos_delta_unit.y);
    float pos_delta_unit_wf_y = fabsf(_wall_unit_vec_wf.y);
    float pos_delta_unit_wf_z = fabsf(_wall_unit_vec_wf.z);

    float speed_z;
    float leash_z;
    if (_wall_unit_vec_wf.z >= 0.0f) {
        speed_z = _wp_speed_up_cms;
        leash_z = _pos_control.get_leash_up_z();
    }else{
        speed_z = _wp_speed_down_cms;
        leash_z = _pos_control.get_leash_down_z();
    }

    // calculate the maximum acceleration, maximum velocity, and leash length in the direction of travel
    if(is_zero(pos_delta_unit_wf_z) && is_zero(pos_delta_unit_wf_y)){
        _track_accel = 0;
        _track_speed = 0;
        _track_leash_length = WALLNAV_LEASH_LENGTH_MIN;
    }else if(is_zero(_wall_unit_vec_wf.z)){
        _track_accel = _wp_accel_cms/pos_delta_unit_wf_y;
        _track_speed = _wp_speed_cms/pos_delta_unit_wf_y;
        _track_leash_length = _pos_control.get_leash_xy()/pos_delta_unit_wf_y;
    }else if(is_zero(pos_delta_unit_wf_y)){
        _track_accel = _wp_accel_z_cms/pos_delta_unit_wf_z;
        _track_speed = speed_z/pos_delta_unit_wf_z;
        _track_leash_length = leash_z/pos_delta_unit_wf_z;
    }else{
        _track_accel = min(_wp_accel_z_cms/pos_delta_unit_wf_z, _wp_accel_cms/pos_delta_unit_wf_y);
        _track_speed = min(speed_z/pos_delta_unit_wf_z, _wp_speed_cms/pos_delta_unit_wf_y);
        _track_leash_length = min(leash_z/pos_delta_unit_wf_z, _pos_control.get_leash_xy()/pos_delta_unit_wf_y);
    }

    // calculate slow down distance (the distance from the destination when the target point should begin to slow down)
    calc_slow_down_distance(_track_speed, _track_accel);

    // set recalc leash flag to false
    _flags.recalc_wp_leash = false;
}





// Called by the loiter controller
void AC_WallNav::send_desired_vel_xy_to_pos_controller()
{
    float ef_vel_x, ef_vel_y;

    // rotate from wall frame to earth frame
    ef_vel_x = _desired_wf_vel_xy.x * _to_wall_cos_yaw - _desired_wf_vel_xy.y * _to_wall_sin_yaw;
    ef_vel_y = _desired_wf_vel_xy.x * _to_wall_sin_yaw + _desired_wf_vel_xy.y * _to_wall_cos_yaw;
    // TODO: check signs, seems OK

    _pos_control.set_desired_velocity_xy(ef_vel_x, ef_vel_y);
}


// Called by the waypoint controller
void AC_WallNav::send_desired_pos_xyz_to_pos_controller()
{
    float ef_pos_x, ef_pos_y, ef_pos_z;

    // rotate from wall frame to earth frame
    ef_pos_x = _desired_wf_pos_xyz.x * _to_wall_cos_yaw - _desired_wf_pos_xyz.y * _to_wall_sin_yaw;
    ef_pos_y = _desired_wf_pos_xyz.x * _to_wall_sin_yaw + _desired_wf_pos_xyz.y * _to_wall_cos_yaw;
    ef_pos_z = _desired_wf_pos_xyz.z;
    // TODO: check signs, seems OK

    _pos_control.set_pos_target(Vector3f(ef_pos_x, ef_pos_y, ef_pos_z));
}





/// calc_slow_down_distance - calculates distance before waypoint that target point should begin to slow-down assuming it is travelling at full speed
void AC_WallNav::calc_slow_down_distance(float speed_cms, float accel_cmss)
{
    // protect against divide by zero
    if (accel_cmss <= 0.0f) {
        _slow_down_dist = 0.0f;
        return;
    }
    // To-Do: should we use a combination of horizontal and vertical speeds?
    // To-Do: update this automatically when speed or acceleration is changed
    _slow_down_dist = speed_cms * speed_cms / (4.0f*accel_cmss);
}

/// get_slow_down_speed - returns target speed of target point based on distance from the destination (in cm)
float AC_WallNav::get_slow_down_speed(float dist_from_dest_cm, float accel_cmss)
{
    // return immediately if distance is zero (or less)
    if (dist_from_dest_cm <= 0) {
        return WALLNAV_WP_TRACK_SPEED_MIN;
    }

    // calculate desired speed near destination
    float target_speed = safe_sqrt(dist_from_dest_cm * 4.0f * accel_cmss);

    // ensure desired speed never becomes too low
    if (target_speed < WALLNAV_WP_TRACK_SPEED_MIN) {
        return WALLNAV_WP_TRACK_SPEED_MIN;
    } else {
        return target_speed;
    }
}


///
/// Utility methods
///

// get_bearing_cd - return bearing in centi-degrees between two positions
// To-Do: move this to math library
float AC_WallNav::get_bearing_cd(const Vector3f &origin, const Vector3f &destination) const
{
    float bearing = 9000 + atan2f(-(destination.x-origin.x), destination.y-origin.y) * 5729.57795f;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}


/// get_bearing_to_target - get bearing to loiter target in centi-degrees
int32_t AC_WallNav::get_loiter_bearing_to_target() const
{
    return get_bearing_cd(_inav.get_position(), _pos_control.get_pos_target());
}



// X and Y are transformed so they express track and cross-track
// Z remains untouched
void AC_WallNav::tranform_pt_to_wall_frame(Vector3f point, Vector3f &result)
{
    float dx, dy;
    dx = point.x - _wall_1st_pt_wf.x;
    dy = point.y - _wall_1st_pt_wf.y;

    result.x = dx * _to_wall_cos_yaw - dy * _to_wall_sin_yaw;
    result.y = dx * _to_wall_sin_yaw + dy * _to_wall_cos_yaw;
    result.z = point.z;
}

// X and Y are transformed so they express track and cross-track
// Z remains untouched
void AC_WallNav::tranform_pt_to_wall_frame_x(Vector3f point, float &result_x)
{
    float dx, dy;
    dx = point.x - _wall_1st_pt_wf.x;
    dy = point.y - _wall_1st_pt_wf.y;

    result_x = dx * _to_wall_cos_yaw - dy * _to_wall_sin_yaw;
}

// X and Y are transformed so they express track and cross-track
// Z remains untouched
void AC_WallNav::tranform_vec_to_wall_frame(Vector3f vec, Vector3f &result)
{
    result.x = vec.x * _to_wall_cos_yaw - vec.y * _to_wall_sin_yaw;
    result.y = vec.x * _to_wall_sin_yaw + vec.y * _to_wall_cos_yaw;
    result.z = vec.z;
}
