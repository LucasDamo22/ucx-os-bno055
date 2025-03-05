#include <ucx.h>
#include <device.h>
#include <gpio.h>

/* GPIO configuration */
const struct gpio_config_s gpio_config = {
	.config_values.port	= GPIO_PORTB,
	.config_values.pinsel	= GPIO_PIN5 | GPIO_PIN6 | GPIO_PIN7,
	.config_values.mode	= GPIO_INPUT << GPIO_PIN5_OPT |
				GPIO_INPUT << GPIO_PIN6_OPT |
				GPIO_INPUT << GPIO_PIN7_OPT,
	.config_values.pull	= GPIO_PULLUP << GPIO_PIN5_OPT |
				GPIO_PULLUP << GPIO_PIN6_OPT |
				GPIO_PULLUP << GPIO_PIN7_OPT
};

const struct gpio_config_s gpio_config2 = {
	.config_values.port	= GPIO_PORTD,
	.config_values.pinsel	= GPIO_PIN2,
	.config_values.mode	= GPIO_INPUT << GPIO_PIN2_OPT,
	.config_values.pull	= GPIO_PULLUP << GPIO_PIN2_OPT
};

/* device driver instantiation */
const struct device_s gpio_device = {
	.name = "gpio_device",
	.config = &gpio_config,
	.custom_api = &gpio_api
};

const struct device_s gpio_device2 = {
	.name = "gpio_device 2",
	.config = &gpio_config2,
	.custom_api = &gpio_api
};

const struct device_s *gpio = &gpio_device;
const struct gpio_api_s *gpio_dev_api = (const struct gpio_api_s *)(&gpio_device)->custom_api;
const struct device_s *gpio2 = &gpio_device2;
const struct gpio_api_s *gpio_dev_api2 = (const struct gpio_api_s *)(&gpio_device2)->custom_api;

/* application interrupt callbacks */
void pb5_int(void)
{
	printf("PB5 int! (rising)\n");
}

void pb6_int(void)
{
	printf("PB6 int! (falling)\n");
}

void pb7_int(void)
{
	printf("PB7 int! (change)\n");
}

void pd2_int(void)
{
	printf("PD2 int! (change)\n");
}

/* application tasks */
void task_int(void)
{
	gpio_dev_api->gpio_setup(gpio);
	gpio_dev_api->gpio_int_attach(gpio, GPIO_PIN5, pb5_int, GPIO_RISING);
	gpio_dev_api->gpio_int_attach(gpio, GPIO_PIN6, pb6_int, GPIO_FALLING);
	gpio_dev_api->gpio_int_attach(gpio, GPIO_PIN7, pb7_int, GPIO_CHANGE);

	gpio_dev_api2->gpio_setup(gpio2);
	gpio_dev_api2->gpio_int_attach(gpio2, GPIO_PIN2, pd2_int, GPIO_CHANGE);

	while (1) {
	}
}

int32_t app_main(void)
{
	ucx_task_spawn(task_int, DEFAULT_STACK_SIZE);

	// start UCX/OS, preemptive mode
	return 1;
}
