/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"



#if NAV_WALL_FOLLOW == ENABLED


/*
 * control_wallfollow.cpp - init and run calls for Wall Follow flight mode
 */

// wallfollow_init - initialise Wall Follow controller
bool Copter::wallfollow_init(bool ignore_checks)
{
    if (position_ok() || optflow_position_ok() || ignore_checks) {

		/*if (sonar_front_health < SONAR_ALT_HEALTH_MAX) {
			printf("Front sonar not healthy, cannot enter WALL FOLLOW mode !");
			wf_mode_status = WALLFOLLOW_STATUS_FLAGS::WF_OFF;
			return false;
		}*/

		// TODO:
        // Choose wall_normal either from parameters, or from the current yaw
        // choice is dependent on a parameter
		// Computes the normal to the wall from the storage line

		//float wall_yaw_normal;      // [rad]
		//float wall_yaw_normal_cos, wall_yaw_normal_sin;
		//int32_t wall_yaw_normal_deg100;
		//wall_yaw_normal -= M_PI_2;
		//wall_yaw_normal_cos = cosf(wall_yaw_normal);
		//wall_yaw_normal_sin = sinf(wall_yaw_normal);
		//wall_yaw_normal_deg100 = (int32_t)(wall_yaw_normal * RAD_TO_DEG * 100.0f);

        // set target to current position
        wall_nav.init_wfloiter_controller_target(ahrs.yaw_sensor, ahrs.cos_yaw(), ahrs.sin_yaw());
        //wall_nav.init_wfloiter_controller_target(wall_yaw_normal_deg100, wall_yaw_normal_cos, wall_yaw_normal_sin);


        // initialize vertical speed and acceleration
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // initialise altitude target to stopping point
        pos_control.set_target_to_stopping_point_z();

        wf_mode_status = WALLFOLLOW_STATUS_FLAGS::WF_ON;

        return true;
    }else{
        wf_mode_status = WALLFOLLOW_STATUS_FLAGS::WF_OFF;
        return false;
    }
}





// wallfollow_run - runs the Wall Follow controller
// should be called at 100hz or more
void Copter::wallfollow_run()
{
    //float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if(!ap.auto_armed  || ap.land_complete || !motors.get_interlock()) {
        //wp_nav.init_loiter_target();
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->control_in)-throttle_average);
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {

        // process pilot's roll and pitch input
        wall_nav.set_pilot_desired_acceleration(channel_roll->control_in, channel_pitch->control_in);

        if (sonar_front_health >= SONAR_ALT_HEALTH_MAX) {
            if (new_measure_front_sonar) {
                // Consumes the new measure
                new_measure_front_sonar = false;
                wall_nav.update_dist_to_wall_sensor_measure(sonar_front_rng);

                if ((wf_mode_status == WALLFOLLOW_STATUS_FLAGS::WF_PAUSE_WALL_RNG_INVALID) && !wall_nav.is_controller_wall_blind())
                    wf_mode_status = WALLFOLLOW_STATUS_FLAGS::WF_ON;
            } else {
                wall_nav.check_sensor_wall_blind();
            }
        } else {
            wall_nav.update_sensor_lost();
        }

        if (wall_nav.is_controller_wall_blind())
            wf_mode_status = WALLFOLLOW_STATUS_FLAGS::WF_PAUSE_WALL_RNG_INVALID;

        // get pilot's desired yaw rate (resetted)
        //target_yaw_rate = get_pilot_desired_yaw_rate(0);            // channel_yaw->control_in

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->control_in);
        target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        // TODO
        wall_nav.clear_pilot_desired_acceleration();
        wf_mode_status = WALLFOLLOW_STATUS_FLAGS::WF_PAUSE_RC_FAILSAFE;
    }

    // run loiter controller
    wall_nav.update_wfloiter_nav(ekfGndSpdLimit, ekfNavVelGainScaler);

    // call attitude controller
    //attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wall_nav.get_roll(), wall_nav.get_pitch(), target_yaw_rate);
    attitude_control.angle_ef_roll_pitch_yaw(wall_nav.get_roll(), wall_nav.get_pitch(), (float)wall_nav.get_wall_yaw_normal(), true);

    // body-frame rate controller is run directly from 100hz loop

    // run altitude controller
    if (sonar_enabled && (sonar_alt_health >= SONAR_ALT_HEALTH_MAX)) {
        // if sonar is ok, use surface tracking
        //target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
    }

    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt, false);
    pos_control.update_z_controller();
}




//-------------------------------------------------
//  Automatic waypoint wall follow
//-------------------------------------------------


void Copter::do_wallfollow(const AP_Mission::Mission_Command& cmd)
{
    const Vector3f &curr_pos = inertial_nav.get_position();

    // Everything is expressed in local frame (whose origin is the EKF origin)
    const Vector3f cur_local_pos = curr_pos;
    const Vector3f origin_local_pos = curr_pos;       // trick, uses the current UAV location for the moment
    const Vector3f dest_local_pos = pv_location_to_vector_with_default(cmd.content.location, curr_pos);

    // TODO: get the previous wpt !

    // this will be used to remember the time in millis after we reach or pass the WP.
    loiter_time = 0;
    // this is the delay, stored in seconds
    //loiter_time_max = abs(cmd.p1);

    float target_dist = abs(cmd.p1);
    bool is_observed_side_left = (cmd.p1 > 0);

    // Set wp navigation target
    auto_wallfollow_start(cur_local_pos, origin_local_pos, dest_local_pos, target_dist, is_observed_side_left);
    // if no delay set the waypoint as "fast"
    //if (loiter_time_max == 0 ) {
   //     wall_nav.set_fast_waypoint(true);
    //}
}


bool Copter::verify_wallfollow(const AP_Mission::Mission_Command& cmd)
{
    // check if we have reached the waypoint
    if( !wall_nav.reached_wp_destination_scan_finished() ) {
        return false;
    }

    // play a tone
    AP_Notify::events.waypoint_complete = 1;

    // start timer if necessary
    if(loiter_time == 0) {
        loiter_time = millis();
    }

    // check if timer has run out
    if (((millis() - loiter_time) / 1000) >= loiter_time_max) {
        gcs_send_text_fmt(PSTR("Reached Command #%i"),cmd.index);
        return true;
    }else{
        return false;
    }
}


// auto_wallfollow_start - initialises controller
void Copter::auto_wallfollow_start(const Vector3f& local_pos, const Vector3f& origin, const Vector3f& destination,
                                   const float target_dist, const bool is_observed_side_left)
{
    auto_mode = Auto_WallFollow;

    // initialise wall_nav
    wall_nav.init_wfwpt_controller(local_pos, origin, destination, target_dist, is_observed_side_left);


    yaw_look_at_heading = wrap_360_cd(wall_nav.get_wall_yaw_normal());
    yaw_look_at_heading_slew = AUTO_YAW_SLEW_RATE;
    set_auto_yaw_mode(AUTO_YAW_LOOK_AT_HEADING);
}


// auto_wp_run - runs the auto waypoint controller
//      called by auto_run at 100hz or more
void Copter::auto_wallfollow_run()
{
    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if(!ap.auto_armed || !motors.get_interlock()) {
        // To-Do: reset waypoint origin to current location because copter is probably on the ground so we don't want it lurching left or right on take-off
        //    (of course it would be better if people just used take-off)

        // multicopters do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);

        // clear i term when we're taking off
        set_throttle_takeoff();
        return;
    }

    // process pilot's yaw input
    /*float target_yaw_rate = 0;
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }*/

    // run waypoint controller
    wall_nav.update_wfwpt_nav();

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    // roll, pitch from waypoint controller, yaw heading from auto_heading()
    attitude_control.angle_ef_roll_pitch_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), get_auto_heading(),true);
}




#endif  // NAV_WALL_FOLLOW


