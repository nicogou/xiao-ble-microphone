/*
 * Copyright (c) 2026 Nicolas Goualard
 * SPDX-License-Identifier: Apache-2.0
 *
 * IMU significant motion detection – LSM6DS3TR-C on XIAO BLE Sense.
 *
 * The Zephyr lsm6dsl driver is used in TRIGGER_NONE mode so that we can
 * own the INT1 GPIO line.  We configure the IMU's embedded-function engine
 * directly via I2C register writes (the driver does not expose significant-
 * motion configuration through the generic sensor API).
 *
 * Event flow:
 *   IMU asserts INT1 → GPIO edge interrupt fires imu_gpio_callback()
 *   → disables the interrupt and submits imu_work to the system work queue
 *   → imu_work_handler() reads FUNC_SRC1 to confirm the event
 *   → logs the event and re-arms the interrupt.
 */

#include "imu.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu, CONFIG_APP_LOG_LEVEL);

/* -------------------------------------------------------------------------
 * LSM6DS3TR-C register map (subset used by this module)
 * ------------------------------------------------------------------------- */
#define IMU_REG_INT1_CTRL  0x0D  /* INT1 pin function routing */
#define IMU_REG_CTRL10_C   0x19  /* Embedded-function enable  */
#define IMU_REG_FUNC_SRC1  0x53  /* Embedded-function status  */

/* CTRL10_C – embedded-function master enable + significant motion */
#define IMU_CTRL10_FUNC_EN      BIT(2)
#define IMU_CTRL10_SIGN_MOT_EN  BIT(0)

/* INT1_CTRL – route significant motion to INT1 */
#define IMU_INT1_SIGN_MOT  BIT(6)

/* FUNC_SRC1 status bits */
#define IMU_FUNC_SRC1_SIGN_MOT  BIT(6)

/* 7-bit I2C address of the LSM6DS3TR-C (matches reg = <0x6a> in DTS) */
#define IMU_I2C_ADDR  0x6AU

/* -------------------------------------------------------------------------
 * Static state
 * ------------------------------------------------------------------------- */
static const struct device *imu_i2c;

/* irq-gpios property from the lsm6ds3tr-c DTS node (gpio0 pin 11) */
static const struct gpio_dt_spec imu_int =
	GPIO_DT_SPEC_GET(DT_NODELABEL(lsm6ds3tr_c), irq_gpios);

static struct gpio_callback imu_gpio_cb;
static struct k_work imu_work;
static imu_motion_cb_t motion_cb;

/* -------------------------------------------------------------------------
 * Work-queue handler – runs in the system work queue thread
 * ------------------------------------------------------------------------- */
static void imu_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	uint8_t func_src1 = 0;

	i2c_reg_read_byte(imu_i2c, IMU_I2C_ADDR, IMU_REG_FUNC_SRC1, &func_src1);

	if (func_src1 & IMU_FUNC_SRC1_SIGN_MOT) {
		LOG_INF("Significant motion detected");
		if (motion_cb) {
			motion_cb();
		}
	}

	/* Re-arm the interrupt for the next event. */
	gpio_pin_interrupt_configure_dt(&imu_int, GPIO_INT_EDGE_TO_ACTIVE);
}

/* -------------------------------------------------------------------------
 * GPIO interrupt callback – runs in ISR context
 * ------------------------------------------------------------------------- */
static void imu_gpio_callback(const struct device *port,
			      struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	/* Disable the interrupt until the work handler has read the status. */
	gpio_pin_interrupt_configure_dt(&imu_int, GPIO_INT_DISABLE);
	k_work_submit(&imu_work);
}

/* -------------------------------------------------------------------------
 * Register configuration
 * ------------------------------------------------------------------------- */
static int imu_configure(void)
{
	int ret;

	/* Enable embedded functions and significant motion. */
	ret = i2c_reg_write_byte(imu_i2c, IMU_I2C_ADDR, IMU_REG_CTRL10_C,
				 IMU_CTRL10_FUNC_EN | IMU_CTRL10_SIGN_MOT_EN);
	if (ret) {
		return ret;
	}

	/* Route significant motion to INT1. */
	return i2c_reg_write_byte(imu_i2c, IMU_I2C_ADDR, IMU_REG_INT1_CTRL,
				  IMU_INT1_SIGN_MOT);
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */
int imu_init(void)
{
	int ret;

	/* Obtain the I2C controller that the sensor node sits on. */
	imu_i2c = DEVICE_DT_GET(DT_BUS(DT_NODELABEL(lsm6ds3tr_c)));
	if (!device_is_ready(imu_i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Enable accelerometer – required for the significant motion engine. */
	const struct device *imu_dev =
		DEVICE_DT_GET(DT_NODELABEL(lsm6ds3tr_c));
	if (!device_is_ready(imu_dev)) {
		LOG_ERR("LSM6DS3TR-C not ready");
		return -ENODEV;
	}

	struct sensor_value odr = { .val1 = 104, .val2 = 0 };
	ret = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	if (ret) {
		LOG_ERR("accel ODR set: %d", ret);
		return ret;
	}

	/* Set up the INT1 GPIO line. */
	if (!gpio_is_ready_dt(&imu_int)) {
		LOG_ERR("INT GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&imu_int, GPIO_INPUT);
	if (ret) {
		LOG_ERR("gpio_pin_configure_dt: %d", ret);
		return ret;
	}

	k_work_init(&imu_work, imu_work_handler);

	gpio_init_callback(&imu_gpio_cb, imu_gpio_callback, BIT(imu_int.pin));
	ret = gpio_add_callback(imu_int.port, &imu_gpio_cb);
	if (ret) {
		LOG_ERR("gpio_add_callback: %d", ret);
		return ret;
	}

	/* Write embedded-function and interrupt configuration registers. */
	ret = imu_configure();
	if (ret) {
		LOG_ERR("imu_configure: %d", ret);
		return ret;
	}

	/* Arm the interrupt – ready to receive events. */
	ret = gpio_pin_interrupt_configure_dt(&imu_int, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("gpio_pin_interrupt_configure_dt: %d", ret);
		return ret;
	}

	LOG_INF("IMU ready – significant motion detection active");
	return 0;
}

void imu_set_motion_cb(imu_motion_cb_t cb)
{
	motion_cb = cb;
}

SYS_INIT(imu_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
