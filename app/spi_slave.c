#include <ucx.h>
#include <spi_bitbang.h>

/* GPIO template callbacks - port them! */

/* configure CS, SCK, MOSI and MISO pins direction
 * 
 * CS   - output (master mode), input (slave mode)
 * SCK  - output (master mode), input (slave mode)
 * MOSI - output (master mode), input (slave mode)
 * MISO - input (master mode), output (slave mode)
 */
int gpio_config(void)
{
	printf("SPI: gpio_config()\n");
	
	return 0;
}

int gpio_cs(int val)
{
	// output CS (master) or poll CS (slave)
	printf("SPI: gpio_cs() %d\n", val);

	return 0;
}

int gpio_sck(int val)
{
	// output SCK (master) or poll SCK (slave)
	printf("SPI: gpio_sck() %d\n", val);

	return 0;
}

int gpio_mosi(int val)
{
	// output MOSI (master) or poll MOSI (slave)
	printf("SPI: gpio_mosi() %d\n", val);
	
	return 0;
}

int gpio_miso(int val)
{
	// poll MISO (master) or output MISO (slave)
	printf("SPI: gpio_miso()\n");

	return 0;
}

/* SPI device driver configuration (slave) */
const struct spi_config_s spi_config = {
	.device_mode = SPI_SLAVE,
	.spi_mode = SPI_MODE0,
	.duplex_mode = SPI_HALF_DUPLEX,
	.bit_order = SPI_MSB,
	.cs_active = SPI_CS_LOW,
	.cs_delay = 1000,
	.sig_delay = 100,
	.gpio_config = gpio_config,
	.gpio_cs = gpio_cs,
	.gpio_sck = gpio_sck,
	.gpio_mosi = gpio_mosi,
	.gpio_miso = gpio_miso
};

struct spi_data_s spi_data;

const struct device_s spi_device1 = {
	.name = "spidevice1",
	.config = &spi_config,
	.data = &spi_data,
	.api = &spi_api
};

void task0(void)
{
	char buf[100];
	int bytes;
	
	while (1) {
		memset(buf, 0, sizeof(buf));
		spi_open(&spi_device1, 0);
		
		do {
			bytes = spi_read(&spi_device1, buf, sizeof(buf));
		} while (bytes == 0);
		printf("recv: %d %s\n", bytes, buf);
		
		strcat(buf, "hola");
		do {
			bytes = spi_write(&spi_device1, buf, strlen(buf) + 1);
		} while (bytes == 0);

		spi_close(&spi_device1);
		
		printf("%s\n", buf);
	}
}

int32_t app_main(void)
{
	ucx_task_spawn(task0, DEFAULT_STACK_SIZE);

	spi_init(&spi_device1);

	// start UCX/OS, preemptive mode
	return 1;
}
