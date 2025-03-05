#include <hal.h>
#include <device.h>
#include <gpio.h>
#include <gpio_drv.h>


int gpio_ll_setup(struct gpio_config_values_s *cfg)
{
	uint32_t modesel, pullsel;
	
	for (int i = 0; i < 8; i++) {
		if (cfg->pinsel & (1 << i)) {
			modesel = (cfg->mode & (3 << (i << 1))) >> (i << 1);
			pullsel = (cfg->pull & (3 << (i << 1))) >> (i << 1);
			switch (modesel) {
			case GPIO_INPUT:
				switch (cfg->port) {
				case GPIO_PORTA: 
					if (pullsel == GPIO_PULLUP) PORTA |= (1 << i);
					DDRA &= ~(1 << i); break;
				case GPIO_PORTB: 
					if (pullsel == GPIO_PULLUP) PORTB |= (1 << i);
					DDRB &= ~(1 << i); break;
				case GPIO_PORTC:
					if (pullsel == GPIO_PULLUP) PORTC |= (1 << i);
					DDRC &= ~(1 << i); break;
				case GPIO_PORTD:
					if (pullsel == GPIO_PULLUP) PORTD |= (1 << i);
					DDRD &= ~(1 << i); break;
				default: break;
				}
				break;
			case GPIO_OUTPUT:
				switch (cfg->port) {
				case GPIO_PORTA: DDRA |= (1 << i); break;
				case GPIO_PORTB: DDRB |= (1 << i); break;
				case GPIO_PORTC: DDRC |= (1 << i); break;
				case GPIO_PORTD: DDRD |= (1 << i); break;
				default: break;
				}
				break;
			default: break;
			}
		}
	}

	return 0;
}

int gpio_ll_get(struct gpio_config_values_s *cfg)
{
	uint8_t val = 0;
	
	switch (cfg->port) {
	case GPIO_PORTA: val = PINA; break;
	case GPIO_PORTB: val = PINB; break;
	case GPIO_PORTC: val = PINC; break;
	case GPIO_PORTD: val = PIND; break;
	default: break;
	}
	
	return val & cfg->pinsel;
}

int gpio_ll_set(struct gpio_config_values_s *cfg, int val)
{
	uint8_t mask = val & cfg->pinsel;
	
	switch (cfg->port) {
	case GPIO_PORTA: PORTA |= mask; break;
	case GPIO_PORTB: PORTB |= mask; break;
	case GPIO_PORTC: PORTC |= mask; break;
	case GPIO_PORTD: PORTD |= mask; break;
	default: break;
	}
	
	return 0;
}

int gpio_ll_clear(struct gpio_config_values_s *cfg, int val)
{
	uint8_t mask = val & cfg->pinsel;
	
	switch (cfg->port) {
	case GPIO_PORTA: PORTA &= ~mask; break;
	case GPIO_PORTB: PORTB &= ~mask; break;
	case GPIO_PORTC: PORTC &= ~mask; break;
	case GPIO_PORTD: PORTD &= ~mask; break;
	default: break;
	}
	
	return 0;
}

int gpio_ll_toggle(struct gpio_config_values_s *cfg, int val)
{
	uint8_t mask = val & cfg->pinsel;
	
	switch (cfg->port) {
	case GPIO_PORTA: PORTA ^= mask; break;
	case GPIO_PORTB: PORTB ^= mask; break;
	case GPIO_PORTC: PORTC ^= mask; break;
	case GPIO_PORTD: PORTD ^= mask; break;
	default: break;
	}
	
	return 0;
}


static void (*int_cb[MAX_INT_SOURCES])(void) = { 0 };

ISR(INT0_vect)
{
	(int_cb[0])();
}

ISR(INT1_vect)
{
	(int_cb[1])();
}

ISR(INT2_vect)
{
	(int_cb[2])();
}

int gpio_ll_int_attach(struct gpio_config_values_s *cfg, int pin, void (*callback)(), int trigger)
{
	switch (cfg->port) {
	case GPIO_PORTB:
		switch (pin) {
		case GPIO_PIN2:
			switch (trigger) {
			case GPIO_RISING:	MCUCSR |= (1 << ISC2); break;
			case GPIO_FALLING:	break;
			default:
				return -1;
			}
			GICR |= (1 << INT2);
			int_cb[2] = callback;
			break;
		default:
			return -1;
		}
		break;
	case GPIO_PORTD:
		switch (pin) {
		case GPIO_PIN2:
			switch (trigger) {
			case GPIO_LOW:		break;
			case GPIO_RISING:	MCUCR |= (1 << ISC01) | (1 << ISC00); break;
			case GPIO_FALLING:	MCUCR |= (1 << ISC01); break;
			case GPIO_CHANGE:	MCUCR |= (1 << ISC00); break;
			default:
				return -1;
			}
			GICR |= (1 << INT0);
			int_cb[0] = callback;
			break;
		case GPIO_PIN3:
			switch (trigger) {
			case GPIO_LOW:		break;
			case GPIO_RISING:	MCUCR |= (1 << ISC11) | (1 << ISC10); break;
			case GPIO_FALLING:	MCUCR |= (1 << ISC11); break;
			case GPIO_CHANGE:	MCUCR |= (1 << ISC10); break;
			default:
				return -1;
			}
			GICR |= (1 << INT1);
			int_cb[1] = callback;
			break;
		default:
			return -1;
		}
		break;
	default:
		return -1;
	}
	
	return 0;
}
