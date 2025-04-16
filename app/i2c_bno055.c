#include <ucx.h>
#include <device.h>
#include <i2c_bitbang.h>
#include <bno055.h>
#include <stdint.h>

/* I2C configuration */

extern const struct device_s *i2c1;

uint16_t data[4];

int32_t app_main(void)
{
	dev_init(i2c1);
  
  data[0] = read_reg(BNO055_SELFTEST_RESULT_ADDR);
  printf("TEST %d\n", data[0]);

  data[0] = read_reg(BNO055_OPR_MODE_ADDR);
  printf("OP %d\n", data[0]);

  write_reg(BNO055_OPR_MODE_ADDR, OPERATION_MODE_AMG);

  data[0] = read_reg(BNO055_OPR_MODE_ADDR);
  printf("OP %d\n", data[0]);

  

  while(1){

      _delay_ms(3000);
      
      data[0] = read_reg(BNO055_CHIP_ID_ADDR);
      printf("CPID %d\n", data[0]);     

      readQuat(data);
      printf("QUAT %d %d %d %d\n", data[0], data[1], data[2], data[3]);

      readEuler(data);
      printf("EULE %d %d %d\n", data[0], data[1], data[2]);

      readGrav(data);
      printf("GRAV %d %d %d\n", data[0], data[1], data[2]);

      readAcc(data);
      printf("ACEL %d %d %d\n", data[0], data[1], data[2]);

      // readLinear(data);
      //  
      // printf("LINE %d %d %d\n", data[0], data[1], data[2]);

      readGyro(data);
      printf("GYRO %d %d %d\n", data[0], data[1], data[2]);

      readMag(data);
      printf("MAG %d %d %d\n", data[0], data[1], data[2]);
  }

  // start UCX/OS, preemptive mode
	return 1;
}