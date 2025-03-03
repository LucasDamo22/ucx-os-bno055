#define LED_PORT	GPIO_PORTB
#define LED_PIN		GPIO_PIN7
#define LED_PIN_OPT	GPIO_PIN7_OPT
	
int gpio_ll_setup(struct gpio_config_values_s *cfg);
int gpio_ll_get(struct gpio_config_values_s *cfg);
int gpio_ll_set(struct gpio_config_values_s *cfg, int val);
int gpio_ll_clear(struct gpio_config_values_s *cfg, int val);
int gpio_ll_toggle(struct gpio_config_values_s *cfg, int val);
