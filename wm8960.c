#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "driver/i2c.h"

#include "i2c_bus.h"

#include "wm8960.h"

/* R25 - Power 1 */
#define WM8960_VMID_MASK 0x180
#define WM8960_VREF      0x40

/* R26 - Power 2 */
#define WM8960_PWR2_LOUT1	0x40
#define WM8960_PWR2_ROUT1	0x20
#define WM8960_PWR2_OUT3	0x02

/* R28 - Anti-pop 1 */
#define WM8960_POBCTRL   0x80
#define WM8960_BUFDCOPEN 0x10
#define WM8960_BUFIOEN   0x08
#define WM8960_SOFT_ST   0x04
#define WM8960_HPSTBY    0x01

/* R29 - Anti-pop 2 */
#define WM8960_DISOP     0x40
#define WM8960_DRES_MASK 0x30

/*
 * wm8960 register cache
 * We can't read the WM8960 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
static const struct reg_default wm8960_reg_defaults[] = {
	{ 0x00, 0x0097 },
	{ 0x01, 0x0097 },
	{ 0x02, 0x0000 },
	{ 0x03, 0x0000 },
	{ 0x04, 0x0000 },
	{ 0x05, 0x0008 },
	{ 0x06, 0x0000 },
	{ 0x07, 0x000a },
	{ 0x08, 0x01c0 },
	{ 0x09, 0x0000 },
	{ 0x0a, 0x00ff },
	{ 0x0b, 0x00ff },

	{ 0x10, 0x0000 },
	{ 0x11, 0x000b },
	{ 0x12, 0x0100 },
	{ 0x13, 0x0032 },
	{ 0x14, 0x0000 },
	{ 0x15, 0x00c3 },
	{ 0x16, 0x00c3 },
	{ 0x17, 0x01c0 },
	{ 0x18, 0x0000 },
	{ 0x19, 0x0000 },
	{ 0x1a, 0x0000 },
	{ 0x1b, 0x0000 },
	{ 0x1c, 0x0000 },
	{ 0x1d, 0x0000 },

	{ 0x20, 0x0100 },
	{ 0x21, 0x0100 },
	{ 0x22, 0x0050 },

	{ 0x25, 0x0050 },
	{ 0x26, 0x0000 },
	{ 0x27, 0x0000 },
	{ 0x28, 0x0000 },
	{ 0x29, 0x0000 },
	{ 0x2a, 0x0040 },
	{ 0x2b, 0x0000 },
	{ 0x2c, 0x0000 },
	{ 0x2d, 0x0050 },
	{ 0x2e, 0x0050 },
	{ 0x2f, 0x0000 },
	{ 0x30, 0x0002 },
	{ 0x31, 0x0037 },

	{ 0x33, 0x0080 },
	{ 0x34, 0x0008 },
	{ 0x35, 0x0031 },
	{ 0x36, 0x0026 },
	{ 0x37, 0x00e9 },
};

static struct register_map wm8960_reg_states = wm8960_reg_defaults;

/* Forward Declarations for static functions */
static esp_err_t wm8960_write_register(i2c_dev_t *dev, uint8_t reg_addr, uint32_t reg_val);





































static esp_err_t wm8960_write_register(i2c_dev_t *dev, uint8_t reg_addr, uint32_t reg_val){
    esp_err_t ret;
    uint8_t buff[3];
    buff[0] = (reg_addr << 1) | ((reg_val >> 8) & 0x0f);
    buff[1] = reg_val & 0xff;
    ret = i2c_bus_write_data((dev->port, dev->addr, buff, 2);
    return ret;
}