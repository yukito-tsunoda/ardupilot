// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

void Copter::init_barometer(bool full_calibration)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));
    if (full_calibration) {
        barometer.calibrate();
    }else{
        barometer.update_calibration();
    }
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// return barometric altitude in centimeters
void Copter::read_barometer(void)
{
    barometer.update();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
    baro_alt = barometer.get_altitude() * 100.0f;
    baro_climbrate = barometer.get_climb_rate() * 100.0f;

    motors.set_air_density_ratio(barometer.get_air_density_ratio());
}

#if CONFIG_SONAR == ENABLED
void Copter::init_sonar(void)
{
   sonar.init();
}
#endif

// return sonar altitude in centimeters
int16_t Copter::read_sonar(void)
{
#if CONFIG_SONAR == ENABLED
    sonar.update();

    read_sonar_front();

    // exit immediately if sonar is disabled
    if (sonar.status() != RangeFinder::RangeFinder_Good) {
        sonar_alt_health = 0;
        return 0;
    }

    int16_t temp_alt = sonar.distance_cm();

    if (temp_alt >= sonar.min_distance_cm() && 
        temp_alt <= sonar.max_distance_cm() * SONAR_RELIABLE_DISTANCE_PCT) {
        if ( sonar_alt_health < SONAR_ALT_HEALTH_MAX ) {
            sonar_alt_health++;
        }
    }else{
        sonar_alt_health = 0;
    }

 #if SONAR_TILT_CORRECTION == 1
    // correct alt for angle of the sonar
    float temp = ahrs.cos_pitch() * ahrs.cos_roll();
    temp = max(temp, 0.707f);
    temp_alt = (float)temp_alt * temp;
 #endif

    return temp_alt;
#else
    return 0;
#endif
}

// Dirty trick, should be replaced by a new parameter for range finder
// that indicates their orientation (bottom/front/left/...). This way
// the primary range finder instance, used for terrain following, would
// not be confused with a frontal range finder.
#define RANGEFINDER_FRONT_INSTANCE    1

// return front sonar range in centimeters
int16_t Copter::read_sonar_front(void)
{
#if CONFIG_SONAR == ENABLED
    // We leave the update() call to the main loop
    //sonar.update();

    // exit immediately if sonar is disabled
    if (sonar.status(RANGEFINDER_FRONT_INSTANCE) != RangeFinder::RangeFinder_Good) {
        sonar_front_health = 0;
        return 0;
    }

    int16_t temp_rng = sonar.distance_cm(RANGEFINDER_FRONT_INSTANCE);

    if (temp_rng >= sonar.min_distance_cm(RANGEFINDER_FRONT_INSTANCE) &&
        temp_rng <= sonar.max_distance_cm(RANGEFINDER_FRONT_INSTANCE) * SONAR_RELIABLE_DISTANCE_PCT) {
        if ( sonar_front_health < SONAR_ALT_HEALTH_MAX ) {
            sonar_front_health++;
        }
    }else{
        sonar_front_health = 0;
    }

    sonar_front_rng = temp_rng;
    new_measure_front_sonar = true;

 #if SONAR_FRONT_TILT_CORRECTION == 1       // not for now, but can also be applied to horizontal sonar
    // correct alt for angle of the sonar
  /*  float temp = ahrs.cos_pitch() * ahrs.cos_roll();
    temp = max(temp, 0.707f);
    temp_alt = (float)temp_alt * temp;*/
 #endif

    return temp_rng;
#else
    return 0;
#endif      // CONFIG_SONAR
}

/*
  update RPM sensors
 */
void Copter::rpm_update(void)
{
    rpm_sensor.update();
    if (rpm_sensor.healthy(0) || rpm_sensor.healthy(1)) {
        if (should_log(MASK_LOG_RCIN)) {
            DataFlash.Log_Write_RPM(rpm_sensor);
        }
    }
}

// initialise compass
void Copter::init_compass()
{
    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        cliSerial->println_P(PSTR("COMPASS INIT ERROR"));
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
}

// initialise optical flow sensor
void Copter::init_optflow()
{
#if OPTFLOW == ENABLED
    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // initialise optical flow sensor
    optflow.init();
#endif      // OPTFLOW == ENABLED
}

// called at 200hz
#if OPTFLOW == ENABLED
void Copter::update_optical_flow(void)
{
    static uint32_t last_of_update = 0;

    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // read from sensor
    optflow.update();

    // write to log and send to EKF if new data has arrived
    if (optflow.last_update() != last_of_update) {
        last_of_update = optflow.last_update();
        uint8_t flowQuality = optflow.quality();
        Vector2f flowRate = optflow.flowRate();
        Vector2f bodyRate = optflow.bodyRate();
        ahrs.writeOptFlowMeas(flowQuality, flowRate, bodyRate, last_of_update);
        if (g.log_bitmask & MASK_LOG_OPTFLOW) {
            Log_Write_Optflow();
        }
    }
}
#endif  // OPTFLOW == ENABLED

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
void Copter::read_battery(void)
{
    battery.read();

    // update compass with current value
    if (battery.has_current()) {
        compass.set_current(battery.current_amps());
    }

    // update motors with voltage and current
    if (battery.get_type() != AP_BattMonitor::BattMonitor_TYPE_NONE) {
        motors.set_voltage(battery.voltage());
    }
    if (battery.has_current()) {
        motors.set_current(battery.current_amps());
    }

    // check for low voltage or current if the low voltage check hasn't already been triggered
    // we only check when we're not powered by USB to avoid false alarms during bench tests
    if (!ap.usb_connected && !failsafe.battery && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        failsafe_battery_event();
    }

    // log battery info to the dataflash
    if (should_log(MASK_LOG_CURRENT)) {
        Log_Write_Current();
    }
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void Copter::read_receiver_rssi(void)
{
    // avoid divide by zero
    if (g.rssi_range <= 0) {
        receiver_rssi = 0;
    }else{
        rssi_analog_source->set_pin(g.rssi_pin);
        float ret = rssi_analog_source->voltage_average() * 255 / g.rssi_range;
        receiver_rssi = constrain_int16(ret, 0, 255);
    }
}

#if EPM_ENABLED == ENABLED
// epm update - moves epm pwm output back to neutral after grab or release is completed
void Copter::epm_update()
{
    epm.update();
}
#endif
