/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AC_WALLNAV_H
#define AC_WALLNAV_H

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl/AC_PosControl.h>      // Position control library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude control library


// loiter maximum velocities and accelerations
#define WALLNAV_ACCELERATION              100.0f      // defines the default velocity vs distant curve.  maximum acceleration in cm/s/s that position controller asks for from acceleration controller
#define WALLNAV_ACCELERATION_MIN           50.0f      // minimum acceleration in cm/s/s - used for sanity checking _wp_accel parameter

#define WALLNAV_LOITER_SPEED              100.0f      // default loiter speed in cm/s
#define WALLNAV_LOITER_SPEED_MIN          25.0f      // minimum loiter speed in cm/s
#define WALLNAV_LOITER_ACCEL              250.0f      // default acceleration in loiter mode
#define WALLNAV_LOITER_ACCEL_MIN           25.0f      // minimum acceleration in loiter mode
#define WALLNAV_LOITER_JERK_MAX_DEFAULT  1000.0f      // maximum jerk in cm/s/s/s in loiter mode

#define WALLNAV_DIST_TO_WALL_DEFAULT         150.0f      // cm
//#define WALLNAV_MAX_OVERSHOOT_DIST_DEFAULT    15.0f      // cm
#define WALLNAV_TO_WALL_SPEED_P_DEFAULT   1.0f

#define WALLNAV_RNG_FLT_DEFAULT           0.2
#define WALLNAV_RNG_MAX_DEFAULT           5000      // [cm]
#define WALLNAV_RNG_MIN_DEFAULT           50        // [cm]
#define WALLNAV_RNG_GAP_DEFAULT           1000      // [cm]
#define WALLNAV_BLIND_TIME_DEFAULT        5         // [s]


#define WALLNAV_WP_SPEED                  500.0f      // default horizontal speed betwen waypoints in cm/s
#define WALLNAV_WP_SPEED_MIN              100.0f      // minimum horizontal speed between waypoints in cm/s
#define WALLNAV_WP_TRACK_SPEED_MIN         50.0f      // minimum speed along track of the target point the vehicle is chasing in cm/s (used as target slows down before reaching destination)
#define WALLNAV_WP_RADIUS                 200.0f      // default waypoint radius in cm

#define WALLNAV_WP_SPEED_UP               250.0f      // default maximum climb velocity
#define WALLNAV_WP_SPEED_DOWN             150.0f      // default maximum descent velocity

#define WALLNAV_WP_ACCEL_Z_DEFAULT        100.0f      // default vertical acceleration betwen waypoints in cm/s/s

#define WALLNAV_LEASH_LENGTH_MIN          100.0f      // minimum leash lengths in cm


#define WALLNAV_SAFE_DIST_TO_WALL         200.0f      // when no laser range is available, in cm

#define WALLNAV_WP_FAST_OVERSHOOT_MAX     200.0f      // 2m overshoot is allowed during fast waypoints to allow for smooth transitions to next waypoint


// A quick note about frames:
//
// Body frame:
//   Frame of the UAV, X is forward, Y is right.
//   Abbreviation: "bf"
//
// Wall frame:
//   Rotated frame according to the wall known normal (only Yaw rotation):
//     X - Toward axis, is parallel to the wall known normal
//     Y - Lateral axis, is perpendicular to the wall known normal
//   The wall known normal is defined as either drone's yaw at the mode entering,
//   or from the perpendicular to the current leg (waypoints segment).
//   Abbreviation: "wf"
//
// In the wall follow controller, internally everything is expressed in the wall frame.
//
// In waypoint mode, the wall frame X and Y positions are introduced.
//   X position is perpendicular to the leg (origin-destination waypoints segment)
//   Y position is parallel to the leg (origin-destination waypoints segment)
//
// In loiter mode, Pilot's inputs are assumed to be in wall frame (similar to the Simple
// mode, but relative to the wall).
//
// Outputs for the position controller are transformed from wall frame to body frame.
//

class AC_WallNav {
public:

    /// Constructor
    AC_WallNav(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control);

    static const struct AP_Param::GroupInfo var_info[];

    /// init_loiter_target in cm from home
    //void init_controller_target(const float wall_yaw_normal_deg, const Vector3f& position, bool reset_I);

    /// shift_loiter_target - shifts the loiter target by the given pos_adjustment
    ///     used by precision landing to adjust horizontal position target
    //void shift_loiter_target(const Vector3f &pos_adjustment);

    //------------------------------------------------
    // Controller initialization - LOITER

    /// init_loiter_target - initialize's loiter position and feed-forward velocity from current pos and velocity
    void init_wfloiter_controller_target(const float wall_yaw_normal_deg);

    void init_wfloiter_controller_target(const int32_t wall_yaw_normal_deg100, const float wall_yaw_normal_cos, const float wall_yaw_normal_sin);


    //------------------------------------------------
    // Controller initialization - WAYPOINT


    /// init_wfwpt_controller - set origin and destination waypoints using position vectors (distance from home in cm)
    void init_wfwpt_controller(const Vector3f& start_loc, const Vector3f& origin, const Vector3f& destination,
                               const float target_dist, const bool is_observed_side_left);


    //------------------------------------------------
    // Controller inputs - LOITER

    /// set_pilot_desired_acceleration - sets pilot desired acceleration from roll and pitch stick input
    void set_pilot_desired_acceleration(float control_roll, float control_pitch);

    /// clear_pilot_desired_acceleration - clear pilot desired acceleration
    void clear_pilot_desired_acceleration() { _pilot_accel_wf_x_cms = 0; _pilot_accel_wf_y_cms = 0; }



    //------------------------------------------------
    // Controller inputs - WALL DISTANCE

    // Front range finder measures / events
    void update_dist_to_wall_sensor_measure(int16_t front_sonar_rng_cm);
    void update_sensor_lost();
    void check_sensor_wall_blind();


    //------------------------------------------------
    // Controller update - LOITER

    // update_controller - run the loiter controller - gets called at 100hz (APM) or 400hz (PX4)
    void update_wfloiter_nav(float ekfGndSpdLimit, float ekfNavVelGainScaler);


    //------------------------------------------------
    // Controller update - WAYPOINT

    // update_controller - run the loiter controller - gets called at 100hz (APM) or 400hz (PX4)
    void update_wfwpt_nav();


    //------------------------------------------------
    // Controller parameters
    void set_speed_xy(float speed_cms);

    ///
    /// shared methods
    ///

    /// get desired roll, pitch which should be fed into stabilize controllers
    int32_t get_roll() const { return _pos_control.get_roll(); };
    int32_t get_pitch() const { return _pos_control.get_pitch(); };

    int32_t get_wall_yaw_normal() const {return _wall_yaw_normal_deg100; };
    int16_t get_target_dist_to_wall() const {return _target_dist_to_wall; };



    /// reached_destination - true when we have come within RADIUS cm of the waypoint, and if the scan is finished
    bool reached_wp_destination_scan_finished() const { return _flags.reached_destination; }

    bool is_controller_wall_blind() const { return _is_ctrl_wall_blind; }
    float get_wall_rng_filt() const { return _wall_rng_filt; }



protected:

    // flags structure
    struct wpnav_flags {
        uint8_t reached_destination     : 1;    // true if we have reached the destination
        //uint8_t fast_waypoint           : 1;    // true if we should ignore the waypoint radius and consider the waypoint complete once the intermediate target has reached the waypoint
        uint8_t slowing_down            : 1;    // true when target point is slowing down before reaching the destination
        uint8_t recalc_wp_leash         : 1;    // true if we need to recalculate the leash lengths because of changes in speed or acceleration
        uint8_t new_wp_destination      : 1;    // true if we have just received a new destination.  allows us to freeze the position controller's xy feed forward
        uint8_t wall_x_rng_ctrl_active  : 1;    // true if sonar measures towards the wall are valid, and thus the X controller is currently running
        uint8_t reset_desired_vel_to_pos : 1;
    } _flags;


    //------------------------------------------------
    // Controller core - LOITER
    /// calc_loiter_desired_velocity - updates desired velocity (i.e. feed forward) with pilot requested acceleration and fake wind resistance
    ///     updated velocity sent directly to position controller
    void calc_wfloiter_desired_velocity_y(float nav_dt, float ekfGndSpdLimit);
    void calc_wfloiter_desired_velocity_x(float nav_dt, float ekfGndSpdLimit);


    //------------------------------------------------
    // Controller core - WAYPOINT
    void advance_wfwp_target_along_track(float dt);


    //------------------------------------------------
    // Controller utilities - LOITER

    /// get_loiter_stopping_point_xy - returns vector to stopping point based on a horizontal position and velocity
    void get_loiter_stopping_point_xy(Vector3f& stopping_point) const;

    //------------------------------------------------
    // Controller utilities - WAYPOINT

    void set_wall_1st_and_end_points(const Vector3f& wall_1st_pt, const Vector3f& wall_end_pt,
                                     const float target_dist, const bool is_observed_side_left);
    void check_wp_leash_length();
    void calculate_wp_leash_length();


    //void calc_wall_target_pos_x(float nav_dt);
    //float expected_front_dist_to_wall(Vector3f cur_pos);

    void send_desired_vel_xy_to_pos_controller();
    void send_desired_pos_xyz_to_pos_controller();

    void calc_slow_down_distance(float speed_cms, float accel_cmss);
    float get_slow_down_speed(float dist_from_dest_cm, float accel_cmss);


    //------------------------------------------------
    // Utility methods

    /// get_bearing_to_target - get bearing to loiter target in centi-degrees
    int32_t get_loiter_bearing_to_target() const;

    // get_bearing_cd - return bearing in centi-degrees between two positions
    // To-Do: move this to math library
    float get_bearing_cd(const Vector3f &origin, const Vector3f &destination) const;

    void tranform_pt_to_wall_frame(Vector3f point, Vector3f &result);
    void tranform_pt_to_wall_frame_x(Vector3f point, float &result_x);
    void tranform_vec_to_wall_frame(Vector3f vec, Vector3f &result);



    // references to inertial nav and ahrs libraries
    const AP_InertialNav&   _inav;
    const AP_AHRS&          _ahrs;
    AC_PosControl&          _pos_control;
    const AC_AttitudeControl& _attitude_control;


    // parameters
    AP_Float    _loiter_speed_cms;      // maximum horizontal speed in cm/s while in loiter
    AP_Float    _loiter_jerk_max_cmsss; // maximum jerk in cm/s/s/s while in loiter
    AP_Float    _loiter_accel_cmss;     // loiter's max acceleration in cm/s/s
    AP_Float    _loiter_accel_min_cmss; // loiter's min acceleration in cm/s/s
    AP_Float    _wp_speed_cms;          // maximum horizontal speed in cm/s during missions
    AP_Float    _wp_speed_up_cms;       // climb speed target in cm/s
    AP_Float    _wp_speed_down_cms;     // descent speed target in cm/s
    AP_Float    _wp_radius_cm;          // distance from a waypoint in cm that, when crossed, indicates the wp has been reached
    AP_Float    _wp_accel_cms;          // horizontal acceleration in cm/s/s during missions
    AP_Float    _wp_accel_z_cms;        // vertical acceleration in cm/s/s during missions

    // parameters - wall follow controller
    AP_Int16    _desired_dist_to_wall;  // desired distance from the wall, in cm
    AP_Float    _to_wall_speed_p;       // kp to convert position error to speed
    AP_Float    _rng_flt_intensity;
    AP_Int16    _rng_max_valid;         // cm
    AP_Int16    _rng_min_valid;         // cm
    AP_Int16    _rng_max_gap_valid;     // cm
    AP_Int16    _no_rng_blind_delay;    // s

    // loiter controller internal variables
    //uint8_t     _loiter_step;           // used to decide which portion of loiter controller to run during this iteration
    int16_t     _pilot_accel_wf_x_cms;   // pilot's desired acceleration forward (body-frame)
    int16_t     _pilot_accel_wf_y_cms;   // pilot's desired acceleration lateral (body-frame)
    Vector2f    _loiter_desired_accel;  // slewed pilot's desired acceleration in lat/lon frame


    // waypoint controller internal variables
    uint32_t    _wp_last_update;        // time of last update_wpnav call
    uint8_t     _wp_step;               // used to decide which portion of wpnav controller to run during this iteration
    //Vector3f    _origin;                // starting point of trip to next waypoint in cm from home (equivalent to next_WP)
    //Vector3f    _destination;           // target destination in cm from home (equivalent to next_WP)
    //Vector3f    _pos_delta_unit;        // each axis's percentage of the total track from origin to destination
    float       _track_length;          // distance in cm between origin and destination
    float       _track_desired;         // our desired distance along the track in cm
    float       _limited_speed_xy_cms;  // horizontal speed in cm/s used to advance the intermediate target towards the destination.  used to limit extreme acceleration after passing a waypoint
    float       _track_accel;           // acceleration along track
    float       _track_speed;           // speed in cm/s along track
    float       _track_leash_length;    // leash length along track
    float       _slow_down_dist;        // vehicle should begin to slow down once it is within this distance from the destination


    // Wall frame reference
    int32_t     _wall_yaw_normal_deg100;     // degrees * 100
    float       _to_wall_cos_yaw;
    float       _to_wall_sin_yaw;

    // Range filtering
    //float       _wall_rng_raw;          // cm, raw measure from the range finder
    float       _wall_rng_filt;         // cm, filtered measure from the range finder
    uint32_t    _last_valid_wall_rng;
    bool        _is_ctrl_wall_blind;    // true if there are has been no valid wall range measures for more than X sec
    float       _pos_x_at_last_valid_wall_rng;      // [cm] when there is no valid range measure, but before assuming blindness, keeps track of forward movement


    // loiter controller internal variables
    float       _desired_speed_x_cms;       // desired speed towards the wall (positive to move towards it, negative to move away)
    Vector2f    _desired_wf_vel_xy;         // expressed in the wall frame, to send to the position controller
    int32_t     _target_dist_to_wall;



    // waypoint controller internal variables
    Vector3f    _wall_1st_pt;           // Coordinates of the wall start, in local NED [cm]
    Vector3f    _wall_end_pt;           // Coordinates of the wall end, in local NED [cm]
    Vector2f    _wall_horiz_unit_vec;   // Direction from wall's first to end points (2D, horizontal)
    Vector3f    _wall_unit_vec;         // Direction from wall's first to end points (3D)


    Vector3f    _wall_1st_pt_wf;          // Coordinates of the wall start, in local wall frame (cm)
    Vector3f    _wall_end_pt_wf;          // Coordinates of the wall end, in local wall frame (cm)
    Vector2f    _wall_horiz_unit_vec_wf;  // Direction from wall's first to end points (2D, horizontal), in local wall frame (cm)
    Vector3f    _wall_unit_vec_wf;        // Direction from wall's first to end points (3D), in local wall frame (cm)
    Vector3f    _wall_normal_unit_vec_wf;        // Direction from wall's first to end points (3D), in local wall frame (cm)

    bool        _isObservedSideLeft;      // true for left, false for right

    float       _track_horiz_length;    // distance in cm between origin and destination, on the horizontal plane

    float       _last_uav_pos_wf_x;
    float       _target_pos_wf_x_on_wall_blind;

    Vector3f    _desired_wf_pos_xyz;    // for the waypoint mode, expressed in the wall frame, to send to the position controller

    //float       _est_safe_dist_to_known_wall_pos;   // estimated safe distance to the wall position as it is known, considering previous sonar measures, (cm)
   // float       _desired_dist_to_wall_with_safety;  // equals to '_desired_dist_to_wall' is sonar range is valid, but may be increased to 2 meters (cm)
    //float       _est_wall_real_pos_x_wf;            // delta position X (in wall frame) between the known wall position, and the sensed one (cm)
    //float       _est_real_wall_pos_x_wf;            // estimate of the real wall position, relative to the leg (= wall frame)

    /*
    int16_t _sonar_front_rng;           // distance reported by the front sonar in cm - Values are 20 to 700 generally.
    uint8_t sonar_front_health;        // true if we can trust the range from the front sonar
    AC_PID  wall_dist_pid;             // controller on the distance to the wall
    bool new_measure_front_sonar;      // true when a new measure is available and has not yet been processed
    WALLFOLLOW_STATUS_FLAGS wf_mode_status;
     */

};


#endif // AC_WALLNAV_H
