// Copyright 2026
// 板级 I2C 总线管理实现

#include "bsp_i2c.h"
#include "bsp_board_cfg.h"

#include <driver/gpio.h>
#include <os/os.h>
#include <os/mem.h>
#include <components/log.h>

#define TAG "bsp_i2c"
#define BSP_LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define BSP_LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

/* gpio_driver 内部 API：把 GPIO 切到指定硬件功能。SDK 未在公开头文件导出，这里显式声明。 */
extern bk_err_t gpio_dev_map(gpio_id_t gpio_id, gpio_dev_t dev);

#define I2C_TIMEOUT_MS 50

typedef struct {
	i2c_id_t id;
	gpio_id_t scl;
	gpio_id_t sda;
	gpio_dev_t scl_func;
	gpio_dev_t sda_func;
	bool pin_mux_required;
	uint32_t ref_cnt;
	bool inited;
} bsp_i2c_ctx_t;

static bsp_i2c_ctx_t s_i2c[BSP_I2C_BUS_MAX] = {
	[BSP_I2C_BUS_CAM] = {
		.id = BOARD_CAM_I2C_ID,
		.scl = BOARD_CAM_I2C_SCL_PIN,
		.sda = BOARD_CAM_I2C_SDA_PIN,
		.scl_func = GPIO_DEV_I2C0_SCL,
		.sda_func = GPIO_DEV_I2C0_SDA,
		.pin_mux_required = false,   /* I2C0 引脚固定 GPIO20/21，驱动内部已处理 */
	},
	[BSP_I2C_BUS_SYS] = {
		.id = BOARD_TP_I2C_ID,
		.scl = GPIO_42,
		.sda = GPIO_43,
		.scl_func = GPIO_DEV_I2C1_SCL,
		.sda_func = GPIO_DEV_I2C1_SDA,
		.pin_mux_required = true,    /* I2C1 需显式选 GPIO42/43 复用组 */
	},
};

bk_err_t bsp_i2c_init(bsp_i2c_bus_t bus)
{
	bsp_i2c_ctx_t *c;
	i2c_config_t cfg;
	bk_err_t ret;

	if (bus >= BSP_I2C_BUS_MAX)
		return BK_ERR_PARAM;

	c = &s_i2c[bus];

	if (c->inited) {
		c->ref_cnt++;
		return BK_OK;
	}

	if (c->pin_mux_required) {
		gpio_dev_map(c->scl, c->scl_func);
		gpio_dev_map(c->sda, c->sda_func);
		bk_gpio_pull_up(c->scl);
		bk_gpio_pull_up(c->sda);
	}

	os_memset(&cfg, 0, sizeof(cfg));
	cfg.baud_rate = I2C_BAUD_RATE_100KHZ;
	cfg.addr_mode = I2C_ADDR_MODE_7BIT;

	ret = bk_i2c_init(c->id, &cfg);
	if (ret != BK_OK) {
		BSP_LOGE("bus %d (i2c%d) init failed: %d\r\n", bus, c->id, ret);
		return ret;
	}

	c->inited = true;
	c->ref_cnt = 1;
	BSP_LOGI("bus %d (i2c%d) ready, scl=GPIO%d sda=GPIO%d\r\n", bus, c->id, c->scl, c->sda);
	return BK_OK;
}

bk_err_t bsp_i2c_deinit(bsp_i2c_bus_t bus)
{
	bsp_i2c_ctx_t *c;

	if (bus >= BSP_I2C_BUS_MAX)
		return BK_ERR_PARAM;

	c = &s_i2c[bus];
	if (!c->inited)
		return BK_OK;

	if (c->ref_cnt > 1) {
		c->ref_cnt--;
		return BK_OK;
	}

	bk_i2c_deinit(c->id);
	c->inited = false;
	c->ref_cnt = 0;
	return BK_OK;
}

bool bsp_i2c_is_ready(bsp_i2c_bus_t bus)
{
	if (bus >= BSP_I2C_BUS_MAX)
		return false;

	return s_i2c[bus].inited;
}

static i2c_id_t bus_to_id(bsp_i2c_bus_t bus)
{
	return s_i2c[bus].id;
}

bk_err_t bsp_i2c_read_reg8(bsp_i2c_bus_t bus, uint8_t dev_addr, uint8_t reg, uint8_t *val)
{
	i2c_mem_param_t p;

	if (bus >= BSP_I2C_BUS_MAX || !val)
		return BK_ERR_NULL_PARAM;

	os_memset(&p, 0, sizeof(p));
	p.dev_addr = dev_addr;
	p.mem_addr = reg;
	p.mem_addr_size = I2C_MEM_ADDR_SIZE_8BIT;
	p.data = val;
	p.data_size = 1;
	p.timeout_ms = I2C_TIMEOUT_MS;

	return bk_i2c_memory_read(bus_to_id(bus), &p);
}

bk_err_t bsp_i2c_write_reg8(bsp_i2c_bus_t bus, uint8_t dev_addr, uint8_t reg, uint8_t val)
{
	i2c_mem_param_t p;

	if (bus >= BSP_I2C_BUS_MAX)
		return BK_ERR_PARAM;

	os_memset(&p, 0, sizeof(p));
	p.dev_addr = dev_addr;
	p.mem_addr = reg;
	p.mem_addr_size = I2C_MEM_ADDR_SIZE_8BIT;
	p.data = &val;
	p.data_size = 1;
	p.timeout_ms = I2C_TIMEOUT_MS;

	return bk_i2c_memory_write(bus_to_id(bus), &p);
}

bk_err_t bsp_i2c_read_buf(bsp_i2c_bus_t bus, uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint32_t len)
{
	i2c_mem_param_t p;

	if (bus >= BSP_I2C_BUS_MAX || !buf || !len)
		return BK_ERR_NULL_PARAM;

	os_memset(&p, 0, sizeof(p));
	p.dev_addr = dev_addr;
	p.mem_addr = reg;
	p.mem_addr_size = I2C_MEM_ADDR_SIZE_8BIT;
	p.data = buf;
	p.data_size = len;
	p.timeout_ms = I2C_TIMEOUT_MS;

	return bk_i2c_memory_read(bus_to_id(bus), &p);
}

bool bsp_i2c_probe(bsp_i2c_bus_t bus, uint8_t dev_addr)
{
	uint8_t dummy = 0;

	return bsp_i2c_read_reg8(bus, dev_addr, 0x00, &dummy) == BK_OK;
}
