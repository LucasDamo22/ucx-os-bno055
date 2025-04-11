#include <ucx.h>
#include <device.h>
#include <i2c_bitbang.h>
#include <bno055.h>
#include <stdint.h>

/* I2C configuration */

extern const struct device_s *i2c1;

int32_t app_main(void)
{
	dev_init(i2c1);
    while(1){
        uint16_t arr[3];
        readMagBuf((uint8_t)8, &arr);
        for (uint8_t i = 0; i < 3; i++) {
            printf("%d", arr[i]);
        }
        _delay_ms(500);
    }
	

	// start UCX/OS, preemptive mode
	return 1;
}