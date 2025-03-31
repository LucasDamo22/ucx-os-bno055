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
        uint8_t chip_id = read_reg(BNO055_CHIP_ID_ADDR);
        printf("Chip ID = %x\n", chip_id);
        if(chip_id == BNO055_ID){
            printf("correct read!\n");
        } else {
            printf("incorrect read :( \n");
        }
        _delay_ms(5000);
    }
	

	// start UCX/OS, preemptive mode
	return 1;
}