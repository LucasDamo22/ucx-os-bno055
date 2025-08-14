#include <ucx.h>
#include <device.h>
#include <pwm.h>
#include <pwm_ll.h>

/* PWM configuration */


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
	for (long freq = 1000; freq < 20000; freq += 1000) { 
		/* motor 1*/
		const struct pwm_config_s pwm_config1 = {
			.config_values.timer = PWM_TIMER2,
			.config_values.direction = PWM_OUTPUT,
			.config_values.mode = PWM_PHASE_CORRECT,
			.config_values.channels = PWM_CHANNEL1,
			.config_values.frequency = freq
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
			.config_values.frequency = freq // resolução (recomendou 2k de inicio)
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
			.config_values.frequency = freq
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
			.config_values.frequency = freq
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
		val1 = pwm_setup(pwm1);
		val2 = pwm_setup(pwm2);
		val3 = pwm_setup(pwm3);
		val4 = pwm_setup(pwm4);

		delay = (65536 / val1) * 100;
		long l = 2;
		while (l < val1) {
			l++;
			pwm_set(pwm1, PWM_CHANNEL1, l);
			pwm_set(pwm2, PWM_CHANNEL2, l);
			pwm_set(pwm3, PWM_CHANNEL3, l);
			pwm_set(pwm4, PWM_CHANNEL4, l);
			_delay_us(delay);
		}
		_delay_us(delay*10);
		while (l > 0) {
			l--;
			pwm_set(pwm1, PWM_CHANNEL1, l);
			pwm_set(pwm2, PWM_CHANNEL2, l);
			pwm_set(pwm3, PWM_CHANNEL3, l);
			pwm_set(pwm4, PWM_CHANNEL4, l);
			_delay_us(delay);
		}
		_delay_us(delay);
	}
	while (1);
	return 1;
}
