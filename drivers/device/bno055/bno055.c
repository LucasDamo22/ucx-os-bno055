#include <ucx.h>
#include <device.h>
#include <gpio.h>
#include <i2c_bitbang.h>
// #include <i2c.h>
// #include <i2c_ll.h>
#include <bno055.h>


// bno adrs last bit -> 0: write 1: read

/* GPIO configuration: PB6 (scl) and PB7 (sda) - port it! */
const struct gpio_config_s gpio_config = {
	.config_values.port	= GPIO_PORTB,
	.config_values.pinsel	= GPIO_PIN6 | GPIO_PIN7,
	.config_values.mode	= GPIO_OUTPUT_OD << GPIO_PIN6_OPT |
				GPIO_OUTPUT_OD << GPIO_PIN7_OPT,
	.config_values.pull	= GPIO_PULLUP << GPIO_PIN6_OPT |
				GPIO_PULLUP << GPIO_PIN7_OPT
};

/* GPIO device driver instantiation */
const struct device_s gpio_device = {
	.name = "gpio_device",
	.config = &gpio_config,
	.custom_api = &gpio_api
};


const struct device_s *gpio = &gpio_device;
const struct gpio_api_s *gpio_dev_api = (const struct gpio_api_s *)(&gpio_device)->custom_api;


/* GPIO template callbacks - port them! */

/* configure SCL and SDA pins direction
 * 
 * SCL - open drain, output configured as logic low
 * SDA - open drain, output configured as logic low
 */
int gpio_configpins(void)
{
	printf("I2C: gpio_configpins()\n");
	gpio_dev_api->gpio_setup(gpio);

	return 0;
}

int gpio_scl(int val)
{
	switch (val) {
	case -1: return ((gpio_dev_api->gpio_get(gpio) & GPIO_PIN6) >> 6);
	case 0: gpio_dev_api->gpio_clear(gpio, GPIO_PIN6); return 0;
	case 1: gpio_dev_api->gpio_set(gpio, GPIO_PIN6); return 0;
	default: return -1;
	}
}

int gpio_sda(int val)
{
	switch (val) {
	case -1: return ((gpio_dev_api->gpio_get(gpio) & GPIO_PIN7) >> 7);
	case 0: gpio_dev_api->gpio_clear(gpio, GPIO_PIN7); return 0;
	case 1: gpio_dev_api->gpio_set(gpio, GPIO_PIN7); return 0;
	default: return -1;
	}
}

/* I2C (bitbang) configuration and driver instantiation */
const struct i2c_config_s i2c_config = {
	.sig_delay = 4,
	.gpio_configpins = gpio_configpins,
	.gpio_scl = gpio_scl,
	.gpio_sda = gpio_sda
};

struct i2c_data_s i2c_data;

const struct device_s i2c_device1 = {
	.name = "i2cdevice1",
	.config = &i2c_config,
	.data = &i2c_data,
	.api = &i2c_api
};
// /* I2C configuration */
// const struct i2c_hw_config_s i2c_config = {
// 	.config_values.port = I2C_PORT1,
// 	.config_values.speed = 100000,
// 	.config_values.mode = I2C_MASTER,
// 	.config_values.ack = I2C_NACK,
// 	.config_values.addr_mode = I2C_ADDR7BIT
// };

// struct i2c_hw_data_s i2c_data;

// const struct device_s i2c_device1 = {
// 	.name = "i2cdevice1",
// 	.config = &i2c_config,
// 	.data = &i2c_data,
// 	.api = &i2c_hw_api
// };

const struct device_s *i2c1 = &i2c_device1;
void write_reg(adafruit_bno055_reg_t reg, uint8_t value){
	char command[3] ={((BNO055_ADDRESS_A << 1) & 0xFE), reg, value};
	 // claiming the line
	dev_open(i2c1, 0);
	// select peripheral and write memory address
	dev_write(i2c1, command, 3);
	dev_close(i2c1);
}

uint8_t read_reg(adafruit_bno055_reg_t reg){

  // shfiting the address and setting to write
	// lsb = 0 means write
  uint8_t command[2] ={((BNO055_ADDRESS_A << 1) & 0xFE), reg};

	uint8_t return_value[2] = {0,0} ;

  // claiming the line
  dev_open(i2c1, 0);

  // select peripheral and write memory address
  dev_write(i2c1, command, 2);
	
  // changing the RW bit to read
	// lsb = 1
  command[0] = (BNO055_ADDRESS_A  << 1 ) | 1;

  // restart the transaction
  dev_write(i2c1, command, 0);

  // select read mode
  dev_write(i2c1, command, 1);

  // read reg
  dev_read(i2c1, return_value, 1);
	dev_close(i2c1);

	return return_value[0];
}

void read_reg_buf(adafruit_bno055_reg_t reg, uint8_t num, uint16_t* arr) {
	                        // shfiting the address and setting to write
						// lsb = 0 means write
    char command[2] ={((BNO055_ADDRESS_A << 1) & 0xFE), reg};

    // claiming the line
    dev_open(i2c1, 0);

    // select peripheral and write memory address
    dev_write(i2c1, command, 2);

	
    // changing the RW bit to read
	// lsb = 1
    command[0] = (BNO055_ADDRESS_A  << 1 ) | 1;

    // restart the transaction
    dev_write(i2c1, command, 0);

    // select read mode
    dev_write(i2c1, command, 1);

    // read reg
	uint8_t* aux = malloc(sizeof(arr)*sizeof(uint16_t)*num/2);
    dev_read(i2c1, aux, num);
	for (uint8_t i = 0; i < (num/2)-1; i++) {
		arr[i] = ((uint16_t)aux[i] << 8) | aux[i+1];
	}
    dev_close(i2c1);
    return;
}


void readMag(uint16_t* arr) {
	uint8_t x_lsb = read_reg(BNO055_MAG_DATA_X_LSB_ADDR);
	uint8_t x_msb = read_reg(BNO055_MAG_DATA_X_MSB_ADDR);
	uint8_t y_lsb = read_reg(BNO055_MAG_DATA_Y_LSB_ADDR);
	uint8_t y_msb = read_reg(BNO055_MAG_DATA_Y_MSB_ADDR);
	uint8_t z_lsb = read_reg(BNO055_MAG_DATA_Z_LSB_ADDR);
	uint8_t z_msb = read_reg(BNO055_MAG_DATA_Z_MSB_ADDR);
	arr[0] = ((uint16_t)x_msb << 8) + x_lsb; // shift msb (left side of 16 bit int) << 8
	arr[1] = ((uint16_t)y_msb << 8) + y_lsb;
	arr[2] = ((uint16_t)z_msb << 8) + z_lsb;
}

void readMagBuf(uint8_t num, uint16_t* arr) {
	read_reg_buf(BNO055_MAG_DATA_X_LSB_ADDR, num, arr);
	return;
}

void readAcc(uint16_t* arr) {
	uint8_t x_lsb = read_reg(BNO055_ACCEL_DATA_X_LSB_ADDR);
	uint8_t x_msb = read_reg(BNO055_ACCEL_DATA_X_MSB_ADDR);
	uint8_t y_lsb = read_reg(BNO055_ACCEL_DATA_Y_LSB_ADDR);
	uint8_t y_msb = read_reg(BNO055_ACCEL_DATA_Y_MSB_ADDR);
	uint8_t z_lsb = read_reg(BNO055_ACCEL_DATA_Z_LSB_ADDR);
	uint8_t z_msb = read_reg(BNO055_ACCEL_DATA_Z_MSB_ADDR);
  printf("x_lsb -> %d | x_msb -> %d\n", x_lsb, x_msb);
  printf("y_lsb -> %d | y_msb -> %d\n", y_lsb, y_msb);
  printf("z_lsb -> %d | z_msb -> %d\n", z_lsb, z_msb);
	arr[0] = ((uint16_t)x_msb << 8) + x_lsb; // shift msb (left side of 16 bit int) << 8
	arr[1] = ((uint16_t)y_msb << 8) + y_lsb;
	arr[2] = ((uint16_t)z_msb << 8) + z_lsb;
}

void readAccBuf(uint8_t num, uint16_t* arr) {
  read_reg_buf(BNO055_ACCEL_DATA_X_LSB_ADDR, num, arr);
  return;
}

void readGyro(uint16_t* arr) {
	uint8_t x_lsb = read_reg(BNO055_GYRO_DATA_X_LSB_ADDR);
	uint8_t x_msb = read_reg(BNO055_GYRO_DATA_X_MSB_ADDR);
	uint8_t y_lsb = read_reg(BNO055_GYRO_DATA_Y_LSB_ADDR);
	uint8_t y_msb = read_reg(BNO055_GYRO_DATA_Y_MSB_ADDR);
	uint8_t z_lsb = read_reg(BNO055_GYRO_DATA_Z_LSB_ADDR);
	uint8_t z_msb = read_reg(BNO055_GYRO_DATA_Z_MSB_ADDR);
	arr[0] = ((uint16_t)x_msb << 8) + x_lsb; // shift msb (left side of 16 bit int) << 8
	arr[1] = ((uint16_t)y_msb << 8) + y_lsb;
	arr[2] = ((uint16_t)z_msb << 8) + z_lsb;
}

void readEuler(uint16_t* arr) {
	uint8_t h_lsb = read_reg(BNO055_EULER_H_LSB_ADDR);
	uint8_t h_msb = read_reg(BNO055_EULER_H_MSB_ADDR);
	uint8_t r_lsb = read_reg(BNO055_EULER_R_LSB_ADDR);
	uint8_t r_msb = read_reg(BNO055_EULER_R_MSB_ADDR);
	uint8_t p_lsb = read_reg(BNO055_EULER_P_LSB_ADDR);
	uint8_t p_msb = read_reg(BNO055_EULER_P_MSB_ADDR);
	arr[0] = ((uint16_t)h_msb << 8) + h_lsb; // shift msb (left side of 16 bit int) << 8
	arr[1] = ((uint16_t)r_msb << 8) + r_lsb;
	arr[2] = ((uint16_t)p_msb << 8) + p_lsb;
}

void readQuat(uint16_t* arr) {
	uint8_t w_lsb = read_reg(BNO055_QUATERNION_DATA_W_LSB_ADDR);
	uint8_t w_msb = read_reg(BNO055_QUATERNION_DATA_W_MSB_ADDR);
	uint8_t x_lsb = read_reg(BNO055_QUATERNION_DATA_X_LSB_ADDR);
	uint8_t x_msb = read_reg(BNO055_QUATERNION_DATA_X_MSB_ADDR);
	uint8_t y_lsb = read_reg(BNO055_QUATERNION_DATA_Y_LSB_ADDR);
	uint8_t y_msb = read_reg(BNO055_QUATERNION_DATA_Y_MSB_ADDR);
	uint8_t z_lsb = read_reg(BNO055_QUATERNION_DATA_Z_MSB_ADDR);
	uint8_t z_msb = read_reg(BNO055_QUATERNION_DATA_Z_MSB_ADDR);
	arr[0] = ((uint16_t)w_msb << 8) + w_lsb; // shift msb (left side of 16 bit int) << 8
	arr[1] = ((uint16_t)x_msb << 8) + x_lsb;
	arr[2] = ((uint16_t)y_msb << 8) + y_lsb;
	arr[3] = ((uint16_t)z_msb << 8) + z_lsb;
}

void readGrav(uint16_t* arr) {
	uint8_t x_lsb = read_reg(BNO055_GRAVITY_DATA_X_LSB_ADDR);
	uint8_t x_msb = read_reg(BNO055_GRAVITY_DATA_X_MSB_ADDR);
	uint8_t y_lsb = read_reg(BNO055_GRAVITY_DATA_Y_LSB_ADDR);
	uint8_t y_msb = read_reg(BNO055_GRAVITY_DATA_Y_MSB_ADDR);
	uint8_t z_lsb = read_reg(BNO055_GRAVITY_DATA_Z_MSB_ADDR);
	uint8_t z_msb = read_reg(BNO055_GRAVITY_DATA_Z_MSB_ADDR);
	arr[0] = ((uint16_t)x_msb << 8) + x_lsb;
	arr[1] = ((uint16_t)y_msb << 8) + y_lsb;
	arr[2] = ((uint16_t)z_msb << 8) + z_lsb;
}
