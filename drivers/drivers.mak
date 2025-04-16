DRIVERS_DIR		= $(SRC_DIR)/drivers
BUS_DRIVERS_DIR		= $(DRIVERS_DIR)/bus
DEVICE_DRIVERS_DIR	= $(DRIVERS_DIR)/device

ddrivers:
	$(CC) $(CFLAGS) \
		$(DEVICE_DRIVERS_DIR)/device/device.c \
		$(BUS_DRIVERS_DIR)/gpio/gpio.c \
		$(BUS_DRIVERS_DIR)/i2c/i2c_bitbang.c \
		$(BUS_DRIVERS_DIR)/spi/spi_bitbang.c \
		$(BUS_DRIVERS_DIR)/i2c/i2c.c \
		$(BUS_DRIVERS_DIR)/uart/uart.c \
		$(BUS_DRIVERS_DIR)/pwm/pwm.c \
		$(DEVICE_DRIVERS_DIR)/device/device.c \
		$(DEVICE_DRIVERS_DIR)/24lcxxx/24lcxxx.c \
		$(DEVICE_DRIVERS_DIR)/25lcxxx/25lcxxx.c \
		$(DEVICE_DRIVERS_DIR)/vt100/vt100.c \
		$(DEVICE_DRIVERS_DIR)/bno055/bno055.c

