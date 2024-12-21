#include <ucx.h>
#include <device.h>
#include "driver.h"

/* device driver function implementation */
static int my_init(const struct device_s *dev)
{
	struct my_data_s *pdata;
	
	printf("DEV: device init\n");
	
	pdata = (struct my_data_s *)dev->data;
	memset(pdata->buff, 0, BUF_SIZE);
	pdata->size = 0;
	pdata->in_use = -1;

	return 0;
}

static int my_open(const struct device_s *dev, int mode)
{
	struct my_data_s *pdata;
	
	pdata = (struct my_data_s *)dev->data;

	if (pdata->in_use == -1) {
		pdata->in_use = ucx_task_id();
		printf("DEV: device open (task %d)\n", ucx_task_id());
	} else {
		printf("DEV: device open failed\n");

		return -1;
	}
	CRITICAL_LEAVE();

	return 0;
}

static int my_close(const struct device_s *dev)
{
	struct my_data_s *pdata;
	
	pdata = (struct my_data_s *)dev->data;

	if (pdata->in_use > -1) {
		pdata->in_use = -1;
		printf("DEV: device close (task %d)\n", ucx_task_id());
	} else {
		printf("DEV: device close failed\n");
		CRITICAL_LEAVE();
		
		return -1;
	}

	return 0;
}

static size_t my_read(const struct device_s *dev, void *buf, size_t count)
{
	struct my_data_s *pdata;
	
	pdata = (struct my_data_s *)dev->data;
	if (pdata->in_use != ucx_task_id())
		return -1;

	if (count < pdata->size)
		pdata->size = count;
	else
		count = pdata->size;
		
	memcpy(buf, pdata->buff, pdata->size);
	pdata->size = 0;
	
	printf("DEV: device read %d bytes\n", count);
	
	return count;
}

static size_t my_write(const struct device_s *dev, void *buf, size_t count)
{
	struct my_data_s *pdata;
	
	pdata = (struct my_data_s *)dev->data;
	if (pdata->in_use != ucx_task_id())
		return -1;

	memcpy(pdata->buff, buf, count);
	pdata->size = count;
	
	printf("DEV: device write %d bytes\n", count);
	
	return pdata->size;
}

/* device driver function mapping for generic API */
struct device_api_s dev_api = {
	.dev_init = my_init,
	.dev_open = my_open,
	.dev_close = my_close,
	.dev_read = my_read,
	.dev_write = my_write
};
