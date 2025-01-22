#include "Sub.h"

/*****************************************************/
/*          Variables nuevas                         */
/*****************************************************/
//Altitud target
float target_profundidad=0;

//Indica si se hace uso del barometro en ves de la IMU
bool uso_barometro=false;


//Error del ciclo anterior
float error_anterior_profundidad=0;


//Acumulación de error
float acumulacion_I_profundidad=0;


/*
 * control_althold.pde - init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Sub::althold_init()
{
    if(!control_check_barometer()) {
        return false;
    }

    // initialize vertical maximum speeds and acceleration
    // sets the maximum speed up and down returned by position controller
    attitude_control.set_throttle_out(0.75, true, 100.0);
    pos_control.init_z_controller();
    pos_control.set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control.set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    attitude_control.relax_attitude_controllers();
    // initialise position and desired velocity
    float pos = stopping_distance();
    float zero = 0;
    pos_control.input_pos_vel_accel_z(pos, zero, zero);

    if(prev_control_mode != control_mode_t::STABILIZE) {
        last_roll = 0;
        last_pitch = 0;
    }
    last_pilot_heading = ahrs.yaw_sensor;
    last_input_ms = AP_HAL::millis();

    return true;
}


float Sub::stopping_distance() {
    const float curr_pos_z = inertial_nav.get_position().z;
    float curr_vel_z = inertial_nav.get_velocity().z;
    float distance = - (curr_vel_z * curr_vel_z) / (2 * g.pilot_accel_z);
    return curr_pos_z  + distance;
}

void Sub::handle_mavlink_attitude_target(){
    uint32_t tnow = AP_HAL::millis();
    float target_roll, target_pitch, target_yaw;
    // Check if set_attitude_target_no_gps is valid
    if (tnow - sub.set_attitude_target_no_gps.last_message_ms < 5000) {
        Quaternion(
            set_attitude_target_no_gps.packet.q
        ).to_euler(
            target_roll,
            target_pitch,
            target_yaw
        );
        target_roll = 100 * degrees(target_roll);
        target_pitch = 100 * degrees(target_pitch);
        target_yaw = 100 * degrees(target_yaw);
        last_roll = target_roll;
        last_pitch = target_pitch;
        last_pilot_heading = target_yaw;
        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw, true);
        return;
    }
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Sub::althold_run()
{
    // When unarmed, disable motors and stabilization
    if (!motors.armed()) {
        althold_init();
        profundidad_init();
        estabilidad_init();
        return;
    }

    if(control_nuevo){
        control_estabilidad();
        control_profundidad();
    }else{

        handle_attitude();

        control_depth();
    }
}

void Sub::control_depth() {
    // We rotate the RC inputs to the earth frame to check if the user is giving an input that would change the depth.
    // Output the Z controller + pilot input to all motors.
    Vector3f earth_frame_rc_inputs = ahrs.get_rotation_body_to_ned() * Vector3f(-channel_forward->norm_input(), -channel_lateral->norm_input(), (2.0f*(-0.5f+channel_throttle->norm_input())));
    float target_climb_rate_cm_s = get_pilot_desired_climb_rate(500 + g.pilot_speed_up * earth_frame_rc_inputs.z);

    bool surfacing = ap.at_surface || pos_control.get_pos_target_z_cm() > g.surface_depth;
    float upper_speed_limit = surfacing ? 0 : g.pilot_speed_up;
    float lower_speed_limit = ap.at_bottom ? 0 : -get_pilot_speed_dn();
    target_climb_rate_cm_s = constrain_float(target_climb_rate_cm_s, lower_speed_limit, upper_speed_limit);
    pos_control.set_pos_target_z_from_climb_rate_cm(target_climb_rate_cm_s);

    if (surfacing) {
        pos_control.set_alt_target_with_slew(MIN(pos_control.get_pos_target_z_cm(), g.surface_depth - 5.0f)); // set target to 5 cm below surface level
    } else if (ap.at_bottom) {
        pos_control.set_alt_target_with_slew(MAX(inertial_nav.get_altitude() + 10.0f, pos_control.get_pos_target_z_cm())); // set target to 10 cm above bottom
    }
    pos_control.update_z_controller();
    // Read the output of the z controller and rotate it so it always points up
    Vector3f throttle_vehicle_frame = ahrs.get_rotation_body_to_ned().transposed() * Vector3f(0, 0, motors.get_throttle_in_bidirectional());
    //TODO: scale throttle with the ammount of thrusters in the given direction
    float raw_throttle_factor = (ahrs.get_rotation_body_to_ned() * Vector3f(0, 0, 1.0)).xy().length();
    motors.set_throttle(throttle_vehicle_frame.z + raw_throttle_factor * channel_throttle->norm_input());
    motors.set_forward(-throttle_vehicle_frame.x + channel_forward->norm_input());
    motors.set_lateral(-throttle_vehicle_frame.y + channel_lateral->norm_input());
}



void Sub::profundidad_init(){
    target_profundidad =inertial_nav.get_altitude();
    acumulacion_I_profundidad=0;
    error_anterior_profundidad=0;
    uso_barometro=false;
}

void Sub::control_profundidad() {

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    float altura_actual= inertial_nav.get_altitude();
    float altura_barometro=barometer.get_altitude();

    float input_profundidad=0.0;
    float input_forward=0.0;
    float input_lateral=0.0;

    if ((channel_throttle->norm_input()>0.45)&&(channel_throttle->norm_input()<0.55)){
        input_profundidad=0;
    }else{
        input_profundidad = channel_throttle->norm_input()-0.5;
    }
    if ((channel_forward->norm_input()>-0.05)&&(channel_forward->norm_input()<0.05)){
        input_forward=0;
    }else{
        input_forward = channel_forward->norm_input();
    }
    if ((channel_lateral->norm_input()>-0.05)&&(channel_lateral->norm_input()<0.05)){
        input_lateral=0;
    }else{
        input_lateral = channel_lateral->norm_input();
    }


    target_profundidad=target_profundidad
                   +0.25*input_profundidad*cosf(ahrs.get_pitch())*cosf(ahrs.get_roll())
                   -0.25*sinf(ahrs.get_roll())*cosf(ahrs.get_pitch())*input_lateral
                   -0.25*sinf(ahrs.get_pitch())*cosf(ahrs.get_roll())*input_forward;

    if(ap.at_bottom&&(target_profundidad<altura_actual)){
        target_profundidad=altura_actual+10;
    }

    if((altura_barometro<-1000)&&!uso_barometro){
        target_profundidad=altura_barometro;
        uso_barometro=true;
    }else if((altura_barometro>-900)&&uso_barometro){
        target_profundidad=altura_actual;
        uso_barometro=false;
    }


    float error_profundidad=0.0;
    if(uso_barometro){
        error_profundidad=target_profundidad-altura_barometro;
    }else{
        error_profundidad=target_profundidad-altura_actual;
    }

    acumulacion_I_profundidad+=I_profundidad*0.001*error_profundidad;
    if(I_profundidad<0.01){
        acumulacion_I_profundidad=0;
    }else if(acumulacion_I_profundidad>1){
        acumulacion_I_profundidad=1;
    }else if (acumulacion_I_profundidad<-1){
        acumulacion_I_profundidad=-1;
    }

    float derivada_profundidad=error_profundidad-error_anterior_profundidad;
    error_anterior_profundidad=error_profundidad;

    float accionamiento_profundidad= P_profundidad*0.05*error_profundidad + 0.5*acumulacion_I_profundidad + D_profundidad*derivada_profundidad*10;


    Vector3f salida;
    salida.z=accionamiento_profundidad*cosf(ahrs.get_pitch())*cosf(ahrs.get_roll());
    salida.x=accionamiento_profundidad*sinf(ahrs.get_pitch())*abs(sinf(ahrs.get_pitch()))*cosf(ahrs.get_roll());
    salida.y=accionamiento_profundidad*sinf(ahrs.get_roll())*abs(sinf(ahrs.get_roll()))*cosf(ahrs.get_pitch());


    motors.set_throttle(salida.z*ganancia_throttle
                    +(channel_throttle->norm_input()-0.5)*abs(sinf(ahrs.get_pitch()))
                    +(channel_throttle->norm_input()-0.5)*abs(sinf(ahrs.get_roll()))
                    +0.5);
    motors.set_forward(-salida.x*ganancia_forward+channel_forward->norm_input()*abs(cosf(ahrs.get_pitch())));
    motors.set_lateral(-salida.y*ganancia_lateral+channel_lateral->norm_input()*abs(cosf(ahrs.get_roll())));

}
