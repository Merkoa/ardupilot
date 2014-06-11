/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_sport.pde - init and run calls for sport flight mode
 */

static int prev_target_pitch;
static int prev_target_roll;
// sport_init - initialise sport controller
static bool sport_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control.set_alt_target(0);
    
    // Init: prev target is 0
    prev_target_pitch = 0;
    prev_target_roll = 0;

    // stabilize should never be made to fail
    return true;
}

// sport_run - runs the sport controller
// should be called at 100hz or more
static void sport_run()
{
    int16_t target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || g.rc_3.control_in <= 0) {
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();
    
    // convert pilot input to lean angles WITHOUT RESCALING OR CONSTRAIN
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles_raw(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in);

    /////////////////////////////////////////////////
    // APPLY HERE TARGET SATURATION FOR ROLL-PITCH //
    /////////////////////////////////////////////////
    
    // Linear saturation
    target_roll = constrain_int16(target_roll,-g.sat_angles,g.sat_angles);
    target_pitch = constrain_int16(target_pitch,-g.sat_angles,g.sat_angles);
    
    // Target rate saturation
    // Read target rate
    target_roll = constrain_int16(target_roll,prev_target_roll - g.sat_angle_deriv,prev_target_roll + g.sat_angle_deriv);
    target_pitch = constrain_int16(target_pitch,prev_target_pitch - g.sat_angle_deriv,prev_target_pitch + g.sat_angle_deriv);
    
    // store targets for next iteration
    prev_target_roll = target_roll;
    prev_target_pitch = target_pitch;
    
    // reset target lean angles and heading while landed
    if (ap.land_complete) {
        attitude_control.init_targets();
    }else{
        // call attitude controller
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

        // body-frame rate controller is run directly from 100hz loop
    }

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true);
}
