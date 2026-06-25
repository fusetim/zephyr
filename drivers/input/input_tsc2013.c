/*
 * Copyright (c) 2026 FuseTim
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tsc2013

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/input/input_touch.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/pm/pm.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tsc2013, CONFIG_INPUT_LOG_LEVEL);

/* TSC2013 used registers */
#define TSC2013_REG_X1 			0x0 		// X1 measurement result 		(R)
#define TSC2013_REG_X2 			0x1 		// X2 measurement result		(R)
#define TSC2013_REG_Y1 			0x2 		// Y1 measurement result		(R)
#define TSC2013_REG_Y2 			0x3 		// Y2 measurement result		(R)
#define TSC2013_REG_IX 			0x4 		// Ix measurement result		(R)
#define TSC2013_REG_IY 			0x5 		// Iy measurement result		(R)
#define TSC2013_REG_Z1 			0x6 		// Z1 measurement result		(R)
#define TSC2013_REG_Z2 			0x7 		// Z2 measurement result		(R)
#define TSC2013_REG_STATUS		0x8 		// Status						(R)
#define TSC2013_REG_AUX			0x9 		// AUX measurement result		(R)
// Reserved range: 0xA - 0xB
#define TSC2013_REG_CFR0		0xC			// Configuration Register 0 	(RW)
#define TSC2013_REG_CFR1		0xD			// Configuration Register 1 	(RW)
#define TSC2013_REG_CFR2		0xE			// Configuration Register 2 	(RW)
#define TSC2013_REG_CFN			0xF			// Converter Fn Select Status 	(RW)

/* TSC2013 Converter Function Select */
#define TSC2013_CFN_TRIPLET_TOUCH_SCANS		0b0000
#define TSC2013_CFN_IX_IY_TOUCH_SCANS		0b0001
// others...


/** TSC2013 configuration (DT). */
struct tsc2013_config {
	struct input_touchscreen_common_config common;
	/** I2C bus. */
	struct i2c_dt_spec bus;
	/** Reset GPIO information. */
	struct gpio_dt_spec rst_gpio;
	/** Interrupt GPIO information. */
	struct gpio_dt_spec int_gpio;
};

/** TSC2013 data. */
struct tsc2013_data {
	/** Device pointer. */
	const struct device *dev;
	/** Work queue (for deferred read). */
	struct k_work work;
	/** Actual device I2C address */
	uint8_t actual_address;
#ifdef CONFIG_INPUT_TSC2013_INTERRUPT
	/** Interrupt GPIO callback. */
	struct gpio_callback int_gpio_cb;
#else
	/** Timer (polling mode). */
	struct k_timer timer;
#endif
#ifdef CONFIG_PM
	struct pm_notifier pm_notifier_handle;
#endif
};

INPUT_TOUCH_STRUCT_CHECK(struct tsc2013_config);


static int tsc2013_i2c_read_register(const struct device *dev, uint8_t reg_addr, void *read_buf, size_t num_read) {
	// Control Byte Format:
	// MSB / D7		D6		D5		D4		D3		D2		D1		LSB / D0
	// CB0 (0)		A3		A2		A1		A0		RSV (0)	PND0	R/~W (1)
	// CB0 = Read/Write registers
	// A[3..0] = reg_addr
	// PND0 = Power-not-down control
	const struct tsc2013_config *config = dev->config;
	struct tsc2013_data *data = dev->data;

	uint8_t cmd_read = ((reg_addr & 0x0F) << 3) | 0b001;

	// TODO: Actual address does not take the R/W bit into account.
	return i2c_write_read(config->bus.bus, data->actual_address, &cmd_read, 1, read_buf, num_read);
}

// write_buf length must be > 0, and write_buf[0] will be overwritten with the command needed
// therefore, num_write must include one additional header packet as well
static int tsc2013_i2c_write_register(const struct device *dev, uint8_t reg_addr, void *write_buf, size_t num_write) {
	// Control Byte Format:
	// MSB / D7		D6		D5		D4		D3		D2		D1		LSB / D0
	// CB0 (0)		A3		A2		A1		A0		RSV (0)	PND0	R/~W (0)
	// CB0 = Read/Write registers
	// A[3..0] = reg_addr
	// PND0 = Power-not-down control
	const struct tsc2013_config *config = dev->config;
	struct tsc2013_data *data = dev->data;

	((uint8_t*) write_buf)[0] = (((reg_addr & 0x0F) << 3) | 0b000);

	// TODO: Actual address does not take the R/W bit into account.
	return i2c_write(config->bus.bus, write_buf, num_write, data->actual_address);
}

static int tsc2013_i2c_write_conversion(const struct device *dev, uint8_t converter_fn, bool sw_reset, bool stop_fns) {
	// Control Byte Format:
	// MSB / D7		D6		D5		D4		D3		D2		D1		LSB / D0
	// CB1 (1)		C3		C2		C1		C0		RM		SWRST	STS
	// CB1 = Start conversion, channel select and conversion-related config
	// C[3..0] = converter function select bits
	// RM = Resolution select, 1 for 12-bit, 0 for 10-bit
	// SWRST = Software reset (1 = Active)
	// STS = Stop bit for all converter functions (1 for stopping the current converter function).
	const struct tsc2013_config *config = dev->config;
	struct tsc2013_data *data = dev->data;
	uint8_t write_buf = 0b10000100;
	write_buf |= (converter_fn << 3);
	write_buf |= (sw_reset) ? 0b10 : 0b00;
	write_buf |= (stop_fns) ? 0b01 : 0b00;
	
	// TODO: Actual address does not take the R/W bit into account.
	return i2c_write(config->bus.bus, &write_buf, 1, data->actual_address);
}

static int tsc2013_process(const struct device *dev)
{
	int r;
	uint8_t i2c_buf[8];
	uint32_t row;
	uint32_t col;
	bool pressed;
	static uint32_t pointer_row;
	static uint32_t pointer_col;
	static uint32_t pointer_present = 0;

	// Check if a touch has been detected
	r = tsc2013_i2c_read_register(dev, TSC2013_REG_CFR0, i2c_buf, 2);
	if (r < 0) {
		return r;
	}
	pressed = (i2c_buf[0] & 0b10000000) != 0;

	// Handle release events
	if (!pressed) {
		if (pointer_present == 0) { return 0; }

		input_touchscreen_report_pos(dev, pointer_col, pointer_row, K_FOREVER);
		input_report_key(dev, INPUT_BTN_TOUCH, 0, true, K_FOREVER);
		pointer_present = 0;
		return 0;
	}
	if (pointer_present > 1) {
		// A bit of timeoff
		pointer_present--;
		return 0;
	}

	// Otherwise, we must read Ix & Iy
	// Iy follows Ix, therefore we can use a seq read cycle.
	r = tsc2013_i2c_read_register(dev, TSC2013_REG_X1, i2c_buf, 8);
	if (r < 0) {
		return r;
	}
	uint32_t x1 = (((uint32_t) i2c_buf[0]) << 8U) | ((uint32_t) i2c_buf[1]);
	uint32_t x2 = (((uint32_t) i2c_buf[2]) << 8U) | ((uint32_t) i2c_buf[3]);
	uint32_t y1 = (((uint32_t) i2c_buf[4]) << 8U) | ((uint32_t) i2c_buf[5]);
	uint32_t y2 = (((uint32_t) i2c_buf[6]) << 8U) | ((uint32_t) i2c_buf[7]);
	if (x1 >= x2)
	{
		col = x2 + ((x1 - x2) >> 1);
	}
	else
	{
		col = x1 + ((x2 - x1) >> 1);
	}

	if (y1 >= y2)
	{
		row = y2 + ((y1 - y2) >> 1);
	}
	else
	{
		row = y1 + ((y2 - y1) >> 1);
	}
	col = 4096 - col;
	row = 4096 - row;
	// Range 3200-4000
	col -= 3200;
	row -= 3200;
	// Map row to X (800->320)
	// Map col to Y (800->240)
	row = row * 32 / 80;
	col = col * 24 / 80;
	col = 240 - col;
	if (pointer_present > 0 && col == pointer_col && row == pointer_row) {
		// Same point, no need for a new event
		return 0;
	}
	pointer_col = col;
	pointer_row = row;
	pointer_present = 1;

	// Trigger touch events
	input_touchscreen_report_pos(dev, pointer_row, pointer_col, K_FOREVER);
	input_report_key(dev, INPUT_BTN_TOUCH, 1, true, K_FOREVER);
	LOG_INF("Touch row: %d, col: %d", pointer_row, pointer_col);

	// No need to clear any status register, the read was all it needs

	return 0;
}

static void tsc2013_work_handler(struct k_work *work)
{
	struct tsc2013_data *data = CONTAINER_OF(work, struct tsc2013_data, work);

	tsc2013_process(data->dev);
}

#ifdef CONFIG_INPUT_TSC2013_INTERRUPT
static void tsc2013_isr_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct tsc2013_data *data = CONTAINER_OF(cb, struct tsc2013_data, int_gpio_cb);

	k_work_submit(&data->work);
}
#else
static void tsc2013_timer_handler(struct k_timer *timer)
{
	struct tsc2013_data *data = CONTAINER_OF(timer, struct tsc2013_data, timer);

	k_work_submit(&data->work);
}
#endif

#if CONFIG_PM
static void tsc2013_pm_state_exit(const struct device *dev, enum pm_state state)
{
	switch (state) {
	case PM_STATE_STANDBY:
		/* Reconfigure the GPIO interrupt pin on exit from
		 * certain low power states as we might lose the GPIO state.
		 */
		const struct tsc2013_config *config = dev->config;
		int r;

		r = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
		if (r < 0) {
			LOG_ERR("Could not configure interrupt GPIO pin");
			return;
		}

#ifdef CONFIG_INPUT_TSC2013_INTERRUPT
		r = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		if (r < 0) {
			LOG_ERR("Could not configure interrupt GPIO interrupt.");
			return;
		}
#endif /* CONFIG_INPUT_TSC2013_INTERRUPT */
		break;
	default:
		break;
	}
}
#endif /* CONFIG_PM */

static int tsc2013_init(const struct device *dev)
{
	const struct tsc2013_config *config = dev->config;
	struct tsc2013_data *data = dev->data;

	if (!i2c_is_ready_dt(&config->bus)) {
		LOG_ERR_DEVICE_NOT_READY(config->bus.bus);
		return -ENODEV;
	}

	data->dev = dev;
	data->actual_address = config->bus.addr;

	k_work_init(&data->work, tsc2013_work_handler);

	int r;

	// Setup reset pin (active low)
	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR_DEVICE_NOT_READY(config->int_gpio.port);
		return -ENODEV;
	}

	if (config->rst_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->rst_gpio)) {
			LOG_ERR_DEVICE_NOT_READY(config->rst_gpio.port);
			return -ENODEV;
		}

		r = gpio_pin_configure_dt(&config->rst_gpio, GPIO_OUTPUT_HIGH);
		if (r < 0) {
			LOG_ERR("Could not configure reset GPIO pin");
			return r;
		}
	}

	// Setup Interrupt pin
#ifdef CONFIG_INPUT_TSC2013_INTERRUPT
	if (!gpio_is_ready_dt(&config->int_gpio)) {
		LOG_ERR_DEVICE_NOT_READY(config->int_gpio.port);
		return -ENODEV;
	}

	r = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
	if (r < 0) {
		LOG_ERR("Could not configure interrupt GPIO pin");
		return r;
	}

	r = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	if (r < 0) {
		LOG_ERR("Could not configure interrupt GPIO interrupt.");
		return r;
	}

	gpio_init_callback(&data->int_gpio_cb, tsc2013_isr_handler, BIT(config->int_gpio.pin));
#else
	k_timer_init(&data->timer, tsc2013_timer_handler, NULL);
#endif

	LOG_INF("i2c address: %p", data->actual_address);

	// Configure the device for continuous inputs
	// - Software reset, and setup for IX-IY scans
	r = tsc2013_i2c_write_conversion(dev, TSC2013_CFN_IX_IY_TOUCH_SCANS, true, true);
	if (r < 0) {
		LOG_ERR("Could not issue Software RESET of the device, cause: %d.", r);
		return r;
	}
	// - - Wait for status register to show reset
	uint8_t	i2c_buf[3] = {0, 0, 0};
	//while ((i2c_buf[1] & 0b1000000) == 0) {
	//	r = tsc2013_i2c_read_register(dev, TSC2013_REG_STATUS, i2c_buf, 2);
	//	if (r < 0) {
	//		LOG_ERR("Could not read STATUS register of the device.");
	//		return r;
	//	}
	//}
	r = tsc2013_i2c_write_conversion(dev, TSC2013_CFN_TRIPLET_TOUCH_SCANS, false, false);
	if (r < 0) {
		LOG_ERR("Could not write / select the conversion for IX/IY Touch scans.");
		return r;
	}
	i2c_buf[1] = 0b10000001; // TSC control, 10bit res, 100 µs voltage stabilization
	i2c_buf[2] = 0b00000000;
	r = tsc2013_i2c_write_register(dev, TSC2013_REG_CFR0, i2c_buf, 3);
	if (r < 0) {
		LOG_ERR("Could not configure device for TSC2013-controlled conversions.");
		return r;
	}

	// Read back all registers for debug
	uint8_t reg_buf[32] = {0};
	r = tsc2013_i2c_read_register(dev, TSC2013_REG_X1, reg_buf, 32);
	for (uint8_t i = 0; i < 16; i++) {
		uint16_t value = (((uint16_t) reg_buf[2*i]) << 8U) | ((uint16_t) reg_buf[2*i+1]);
		LOG_INF("reg %d: %p", i, value);
	}

#ifdef CONFIG_INPUT_TSC2013_INTERRUPT
	r = gpio_add_callback(config->int_gpio.port, &data->int_gpio_cb);
	if (r < 0) {
		LOG_ERR("Could not set gpio callback");
		return r;
	}
#else
	k_timer_start(&data->timer, K_MSEC(CONFIG_INPUT_TSC2013_PERIOD_MS),
		      K_MSEC(CONFIG_INPUT_TSC2013_PERIOD_MS));
#endif

#if CONFIG_PM
	/* We need to reconfigure the interrupt GPIO when waking up from
	 * certain low power modes.
	 */
	pm_notifier_register(&data->pm_notifier_handle);
#endif
	return 0;
}

#if CONFIG_PM
#define TSC2013_PM_NOTIFIER_FUNCS(n)                                                                 	\
static void TSC2013_##n##_pm_state_exit(enum pm_state state)                                         	\
{                                                                                                  		\
	tsc2013_pm_state_exit(DEVICE_DT_INST_GET(n), state);                                         		\
}

#define TSC2013_PM_NOTIFIER(n)                                                                       	\
	.pm_notifier_handle = {                                                                    			\
		.state_exit = tsc2013_##n##_pm_state_exit,                                           			\
	},
#else
#define TSC2013_PM_NOTIFIER_FUNCS(n)
#define TSC2013_PM_NOTIFIER(n)
#endif /* CONFIG_PM */

#define TSC2013_INIT(index)                                                                       		\
	static const struct tsc2013_config tsc2013_config_##index = {                              			\
		.common = INPUT_TOUCH_DT_INST_COMMON_CONFIG_INIT(index),		           						\
		.bus = I2C_DT_SPEC_INST_GET(index),                                                				\
		.rst_gpio = GPIO_DT_SPEC_INST_GET_OR(index, reset_gpios, {0}),                     				\
		.int_gpio = GPIO_DT_SPEC_INST_GET(index, irq_gpios),                               				\
	};                                                                                         			\
	TSC2013_PM_NOTIFIER_FUNCS(index)                                                             		\
	static struct tsc2013_data tsc2013_data_##index = {                                            		\
		TSC2013_PM_NOTIFIER(index)                                                           			\
	};                                                                                         			\
	DEVICE_DT_INST_DEFINE(index, tsc2013_init, NULL, &tsc2013_data_##index, &tsc2013_config_##index, 	\
			      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TSC2013_INIT)