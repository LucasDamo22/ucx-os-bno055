#include <ucx.h>
#include <device.h>
#include <pwm_ll.h>
#include <pwm.h>
#include <bno055.h>
#include "PID.h"

#ifndef PID_TEST
#define PID_TEST

#define FREQ 8500

/* ----------------- PWM configuration --------------------- */

/* motor 1*/
const struct pwm_config_s pwm_config1 = {
	.config_values.timer = PWM_TIMER2,
	.config_values.direction = PWM_OUTPUT,
	.config_values.mode = PWM_PHASE_CORRECT,
	.config_values.channels = PWM_CHANNEL1,
	.config_values.frequency = FREQ
};

const struct device_s pwm_device1 = {
	.name = "pwmdevice1",
	.config = &pwm_config1,
	.custom_api = &pwm_api
};

/* motor 2*/
const struct pwm_config_s pwm_config2 = {
	.config_values.timer = PWM_TIMER2,	//timer2, timer3, timer4, timer9 (apenas 2 canais)
	.config_values.direction = PWM_OUTPUT,
	.config_values.mode = PWM_PHASE_CORRECT, // motores
	.config_values.channels = PWM_CHANNEL2, // cada timer tem 4 canais
	.config_values.frequency = FREQ // resolução (recomendou 2k de inicio)
};

const struct device_s pwm_device2 = {
	.name = "pwmdevice2",
	.config = &pwm_config2,
	.custom_api = &pwm_api
};

/* motor 3*/
const struct pwm_config_s pwm_config3 = {
	.config_values.timer = PWM_TIMER2,
	.config_values.direction = PWM_OUTPUT,
	.config_values.mode = PWM_PHASE_CORRECT,
	.config_values.channels = PWM_CHANNEL3,
	.config_values.frequency = FREQ
};

const struct device_s pwm_device3 = {
	.name = "pwmdevice3",
	.config = &pwm_config3,
	.custom_api = &pwm_api
};

/*motor 4*/
const struct pwm_config_s pwm_config4 = {
	.config_values.timer = PWM_TIMER2,
	.config_values.direction = PWM_OUTPUT,
	.config_values.mode = PWM_PHASE_CORRECT,
	.config_values.channels = PWM_CHANNEL4,
	.config_values.frequency = FREQ
};

const struct device_s pwm_device4 = {
	.name = "pwmdevice4",
	.config = &pwm_config4,
	.custom_api = &pwm_api
};

const struct device_s *pwm1 = &pwm_device1;
const struct device_s *pwm2 = &pwm_device2;
const struct device_s *pwm3 = &pwm_device3;
const struct device_s *pwm4 = &pwm_device4;

/* ----------------- PID & BNO i2c configuration and tests --------------------- */

extern const struct device_s *i2c1;
uint16_t data[4];

/* ------------------ main program -------------------- */

void task0(void)
{
	while(1);
}

int32_t app_main(void)
{
	ucx_task_spawn(task0, DEFAULT_STACK_SIZE);

	long val1, val2, val3, val4, delay;
	 
	val1 = pwm_setup(pwm1);
	val2 = pwm_setup(pwm2);
	val3 = pwm_setup(pwm3);
	val4 = pwm_setup(pwm4);

    init_all_PID_values(); // Initialize all structs with gains and 0s for other values

    printf("Setting BNO comms\n");
    dev_init(i2c1);
    data[0] = read_reg(BNO055_SELFTEST_RESULT_ADDR);
    printf("TEST %d\n", data[0]);
    data[0] = read_reg(BNO055_OPR_MODE_ADDR);
    write_reg(BNO055_SELFTEST_RESULT_ADDR, OPERATION_MODE_AMG); // accel-mag-gyro mode
    data[0] = read_reg(BNO055_OPR_MODE_ADDR);
    if (OPERATION_MODE_AMG != data[0]) {
        printf("ERROR: read Operation Mode as %d, expected %d, aborting...", data[0], OPERATION_MODE_AMG);
        quit();
    } else {
        printf("OP %d\n", data[0]);
    }
    printf("BNO communication OK\n");
    printf("Testing PID calculation\n");

    readEuler(data);
    uint16_t h, r, p;
    h = data[0];
    r = data[1];
    p = data[2];

    readGyro(data);
    uint16_t x, y, z;
    x = data[0];
    y = data[1];
    z = data[2];

    printf("Read Euler Angles: Roll %f | Pitch %f | Yaw %f\n", r, p, y);
    printf("Read Gyro: Roll Rate %f | Pitch Rate %f | Yaw Rate %f\n", x, y, z);

    uint16_t sp = 0;
    Roll_Pitch_PID_Calc(&roll, sp, r, x);
    Roll_Pitch_PID_Calc(&pitch, sp, p, y);
    Yaw_Rate_PID_Calc(&yaw_rate, sp, h);
    Yaw_Heading_PID_Calc(&yaw_head, sp, z);
    //Height_PID_Calc(&height, sp, grav?);

    printf("Roll PID-in result:     %f\n", roll.in.pid_res);
    printf("Roll PID-out result:    %f\n", roll.out.pid_res);
    printf("Pitch PID result:       %f\n", pitch.pid_res);
    printf("Yaw Rate PID result:    %f\n", yaw_rate.pid_res);
    printf("Yaw Heading PID result: %f\n", yaw_head.pid_res);

    while (1) {
	    /*attempt flight*/
	    printf("Flight test\n");
	    pwm_set(pwm1, PWM_CHANNEL1, 3);
	    pwm_set(pwm2, PWM_CHANNEL2, 3);
	    pwm_set(pwm3, PWM_CHANNEL3, 3);
	    pwm_set(pwm4, PWM_CHANNEL4, 3);
	    long l = 2;
	    while (l < val1) {
		    l++;
		    pwm_set(pwm1, PWM_CHANNEL1, l);
		    pwm_set(pwm2, PWM_CHANNEL2, l);
		    pwm_set(pwm3, PWM_CHANNEL3, l);
		    pwm_set(pwm4, PWM_CHANNEL4, l);
	 	    _delay_us(delay);
	    }
	    _delay_us(delay*1000);
	    while (l > 0) {
		    l--;
		    pwm_set(pwm1, PWM_CHANNEL1, l);
		    pwm_set(pwm2, PWM_CHANNEL2, l);
		    pwm_set(pwm3, PWM_CHANNEL3, l);
		    pwm_set(pwm4, PWM_CHANNEL4, l);
		    _delay_us(delay);
	    }
    }
}

#endif