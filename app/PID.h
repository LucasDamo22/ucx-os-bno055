#ifndef PID
#define PID

#define TIMESTEP 0.001 // 1ms
#define OUTER_DERIV_FILT_EN 1
#define INNER_DERIV_FILT_EN 1

#define DELTA_SUM_MAX 500
#define I_DELTA_MIN -DELTA_SUM_MAX

#define ROLL_IN_KD      0.2
#define ROLL_IN_KP      1.5 
#define ROLL_IN_KI      0.01
#define ROLL_OUT_KD     0.01
#define ROLL_OUT_KP     0.8
#define ROLL_OUT_KI     0.05

#define PITCH_IN_KD     0.2
#define PITCH_IN_KP     1.5
#define PITCH_IN_KI     0.01
#define PITCH_OUT_KD    0.08
#define PITCH_OUT_KP    0.8
#define PITCH_OUT_KI    0.05

#define YAW_HEAD_KP     1.0
#define YAW_HEAD_KI     0.005
#define YAW_HEAD_KD     0.0
#define YAW_RATE_KP     0.5
#define YAW_RATE_KI     0.02
#define YAW_RATE_KD     0.05

#define HEIGHT_KP       2.0
#define HEIGHT_KI       0.1
#define HEIGHT_KD       0.5
#define MINIMUM_THRUST  9.8

#define OUTPUT_MAX 100.0
#define OUTPUT_MIN -100.0

typedef struct _PIDsingle {
    uint16_t kp;
    uint16_t ki;
    uint16_t kd;

    uint16_t target;
    uint16_t measured_val;
    uint16_t measured_val_prev;
    uint16_t delta;
    uint16_t delta_sum;
    uint16_t delta_deriv;
    uint16_t delta_deriv_filt;

    uint16_t p_res;
    uint16_t i_res;
    uint16_t d_res;

    uint16_t pid_res;
} PIDsingle;

typedef struct _PIDdoubles {
    PIDsingle in;
    PIDsingle out;
} PIDdoubles;

extern PIDdoubles roll;
extern PIDdoubles pitch;
extern PIDsingle yaw_heading;
extern PIDsingle yaw_rate;

void Roll_Pitch_PID_Calc(PIDdoubles* axis, uint16_t sp, uint16_t angle, uint16_t rate);
void Yaw_Rate_PID_Calc(PIDsingle* axis, uint16_t sp, uint16_t val);
void Yaw_Heading_PID_Calc(PIDsingle* axis, uint16_t sp, uint16_t angle, uint16_t rate);
void Reset_PID_Integral(PIDsingle* axis);
void Reset_All_PID_Integral(void);
void init_PID_values(PIDsingle* axis, uint16_t gain_p, uint16_t gain_i, uint16_t gain_d);
void init_all_PID_values();

#endif