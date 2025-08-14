#include <ucx.h>
#include <device.h>
#include <pwm.h>
#include <pwm_ll.h>

#define FREQ 8500

/* PWM configuration */

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


/* pwm_setup() returns the PWM number of steps, so to configure
 * the duty cycle in pwm_set(), consider the following formula:
 * 
 * pulse = (val * 100) / duty
 * 
 * Where pulse is the PWM value to be passed to pwm_set(), val
 * is the number of steps returned by pwm_setup() and duty is
 * the duty cycle (0% to 100%).
 */

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

	printf("PWM1 value: %d\n", val1);
	printf("PWM2 value: %d\n", val2);
	printf("PWM3 value: %d\n", val3);
	printf("PWM4 value: %d\n", val4);

	delay = (65536 / val1) * 10;
	/*motor 1 test*/
	printf("PWM1: increasing duty cycle\n");
	for (int i = 0; i < val1; i++) {
		pwm_set(pwm1, PWM_CHANNEL1, i);
		_delay_us(delay);
	}
	printf("PWM1: decreasing duty cycle\n");
	for (int i = val1 - 1; i >= 0; i--) {
		pwm_set(pwm1, PWM_CHANNEL1, i);
		_delay_us(delay);
	}

	/*motor 2 test*/
	printf("PWM2: increasing duty cycle\n");
	for (int i = 0; i < val2; i++) {
		pwm_set(pwm2, PWM_CHANNEL2, i);
		_delay_us(delay);
	}
	printf("PWM2: decreasing duty cycle\n");
	for (int i = val2 - 1; i >= 0; i--) {
		pwm_set(pwm2, PWM_CHANNEL2, i);
		_delay_us(delay);
	}

	/*motor 3 test*/
	printf("PWM3: increasing duty cycle\n");
	for (int i = 0; i < val3; i++) {
		pwm_set(pwm3, PWM_CHANNEL3, i);
		_delay_us(delay);
	}
	printf("PWM3: decreasing duty cycle\n");
	for (int i = val3 - 1; i >= 0; i--) {
		pwm_set(pwm3, PWM_CHANNEL3, i);
		_delay_us(delay);
	}

	/*motor 4 test*/
	printf("PWM4: increasing duty cycle\n");
	for (int i = 0; i < val4; i++) {
		pwm_set(pwm4, PWM_CHANNEL4, i);
		_delay_us(delay);
	}
	printf("PWM4: decreasing duty cycle\n");
	for (int i = val4 - 1; i >= 0; i--) {
		pwm_set(pwm4, PWM_CHANNEL4, i);
		_delay_us(delay);
	}
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
	while (1);
	return 1;
}
