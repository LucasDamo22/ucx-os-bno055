#include "PID.h"

#ifndef PID_C
#define PID_C

extern PIDdoubles roll;
extern PIDdoubles pitch;
extern PIDsingle yaw_head;
extern PIDsingle yaw_rate;
extern PIDsingle height;

// Following code assumes:
// -> All angles are euler angles, as in they've been converted from an EKF output quaternion to euler angles
// -> 

uint16_t constrain(uint16_t val, uint16_t min, uint16_t max) {
    if (val > max) return max;
    else if (val < min) return min;
    else return val;
}

void Roll_Pitch_PID_Calc(PIDdoubles* axis, uint16_t sp, uint16_t angle, uint16_t rate) {
    /* PID Outer (Roll and Pitch Angular Position Control)*/
    axis->out.target = sp; // sp angle of outer PID control
    axis->out.measured_val = angle; // angle from BNO

    axis->out.delta = axis->out.target - axis->out.measured_val;
    axis->out.p_res = axis->out.delta * axis->out.kp;

    axis->out.delta_sum = axis->out.delta_sum + axis->out.delta * TIMESTEP; // Get sum

    if (axis->out.delta_sum > DELTA_SUM_MAX) axis->out.delta_sum = DELTA_SUM_MAX;
    else if (axis->out.delta_sum < I_DELTA_MIN) axis->out.delta_sum = I_DELTA_MIN;

    axis->out.i_res = axis->out.delta_sum * axis->out.ki;
    axis->out.delta_deriv = -rate;

    #if !OUTER_DERIV_FILT_ENABLE
        axis->out.d_res = axis->out.delta_deriv * axis->out.kd;
    #else
        axis->out.delta_deriv_filt = axis->out.delta_deriv * 0.4f + axis->out.delta_deriv * 0.6f; // Deriv filter
        axis->out.d_result = axis->out.delta_deriv_filt * axis->out.kd;
    #endif

    axis->out.pid_res = constrain(axis->out.p_res + axis->out.i_res + axis->out.d_res, OUTPUT_MIN, OUTPUT_MAX);

    /* PID Inner (Roll and Pitch Angular Rate Control)*/

    axis->in.target = axis->out.pid_res;
    axis->in.measured_val = rate; // Gyroscope angular rate

    axis->in.delta = axis->in.target - axis->in.measured_val;
    axis->in.p_res = axis->in.delta * axis->in.kp;

    axis->in.delta_sum = axis->in.delta_sum + axis->in.delta * TIMESTEP;

    if (axis->in.delta_sum > DELTA_SUM_MAX) axis->in.delta_sum = DELTA_SUM_MAX;
    else if (axis->in.delta_sum < I_DELTA_MIN) axis->in.delta_sum = I_DELTA_MIN;

    axis->in.i_res = axis->in.delta_sum * axis->in.ki;
    axis->in.delta_deriv = -(axis->in.measured_val - axis->in.measured_val_prev) / TIMESTEP;
    axis->in.measured_val_prev = axis->in.measured_val;

    #if !INNER_DERIV_FILT_ENABLE
        axis->in.d_res = axis->in.delta_deriv * axis->in.kd;
    #else
        axis->in.delta_deriv_filt = axis->delta_deriv_filt * 0.5f + axis->in.delta_deriv * 0.5f;
        axis->in.d_res = axis->in.delta_deriv_filt * axis->in.kd;
    #endif

    axis->out.pid_res = constrain(axis->out.p_res + axis->out.i_res + axis->out.d_res, OUTPUT_MIN, OUTPUT_MAX);
}

void Yaw_Rate_PID_Calc(PIDsingle* axis, uint16_t sp, uint16_t val) {
    axis->target = sp;
    axis->measured_val = val; // Gyroscope Z

    axis->delta = axis->target - axis->measured_val;
    axis->p_res = axis->delta * axis->kp;

    axis->delta_sum = axis->delta_sum + axis->delta * TIMESTEP;
    axis->i_res = axis->delta_sum * axis->ki;

    axis->delta_deriv = -(axis->measured_val - axis->measured_val_prev) / TIMESTEP;
    axis->measured_val_prev = axis->measured_val;
    axis->d_res = axis->delta_deriv * axis->kd;

    axis->pid_res = constrain(axis->p_res + axis->i_res + axis->d_res, OUTPUT_MIN, OUTPUT_MAX);
}

void Yaw_Heading_PID_Calc(PIDsingle* axis, uint16_t sp, uint16_t angle, uint16_t rate) {
    axis->target = sp;
    axis->measured_val = angle; //BNO yaw angle

    axis->delta = axis->target - axis->measured_val;

    if (axis->delta > 180.f) axis->delta -= 360.f;
    else if (axis->delta < -180.f) axis->delta += 360.f;

    axis->delta_sum = axis->delta_sum + axis->delta * TIMESTEP;
    axis->i_res = axis->delta_sum * axis->ki;

    axis->delta_deriv = -rate;
    axis->d_res = axis->delta_deriv * axis->kd;

    axis->pid_res = axis->p_res + axis->i_res + axis->d_res;
}

void Height_PID_Calc(PIDsingle* axis, uint16_t sp, uint16_t val, uint16_t rate) {
    axis->target = sp;
    axis->measured_val = val; // BMP air pressure?

    axis->delta = axis->target - axis->measured_val;

    /*treat possible under/over flows here*/

    axis->delta_sum = axis->delta_sum + axis->delta * TIMESTEP;
    axis->i_res = axis->delta_sum * axis->ki;

    axis->delta_deriv = -rate; /*height derivative = d/dt * ((sp - val) + MINIMUM_THRUST); assuming it is calculated outside as argument rate*/
    axis->d_res = axis->delta_deriv * axis->kd;

    axis->pid_res = axis->p_res + axis->i_res + axis->d_res;
}

void Reset_PID_Integral(PIDsingle* axis) {
    axis->delta_sum = 0;
}

void Reset_All_PID_Integrals(void) {
    Reset_PID_Integral(&roll.in);
    Reset_PID_Integral(&roll.out);
    Reset_PID_Integral(&pitch.in);
    Reset_PID_Integral(&pitch.out);
    Reset_PID_Integral(&yaw_head);
    Reset_PID_Integral(&yaw_rate);
    Reset_PID_Integral(&height);
}

void init_PID_values(PIDsingle* axis, uint16_t gain_p, uint16_t gain_i, uint16_t gain_d) {
    axis->kp = gain_p;
    axis->ki = gain_i;
    axis->kd = gain_d;

    axis->target = 0;
    axis->measured_val = 0;
    axis->measured_val_prev = 0;
    axis->delta = 0;
    axis->delta_sum = 0;
    axis->delta_deriv = 0;
    axis->delta_deriv_filt = 0;

    axis->p_res = 0;
    axis->i_res = 0;
    axis->d_res = 0;

    axis->pid_res = 0;
}

void init_all_PID_values() {
    init_PID_values(&roll.in, ROLL_IN_KP, ROLL_IN_KI, ROLL_IN_KD);
    init_PID_values(&roll.out, ROLL_OUT_KP, ROLL_OUT_KI, ROLL_OUT_KD);
    init_PID_values(&pitch.in, PITCH_IN_KP, PITCH_IN_KI, PITCH_IN_KD);
    init_PID_values(&pitch.out, PITCH_OUT_KP, PITCH_OUT_KI, PITCH_OUT_KD);
    init_PID_values(&yaw_head, YAW_HEAD_KP, YAW_HEAD_KI, YAW_HEAD_KD);
    init_PID_values(&yaw_rate, YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD);
    init_PID_values(&height, HEIGHT_KP, HEIGHT_KI, HEIGHT_KD);
}

#endif