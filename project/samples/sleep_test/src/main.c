/*
 * Sleep-test sample — STOP3 current measurement
 *
 * 1. Turns off 3V3 and 5V power domains
 * 2. Enters STOP3 (deepest STM32U5 low-power mode)
 * 3. Wakes on button2 (PC13) press, prints a message, then sleeps again
 *    on the next button2 press
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>

static const struct gpio_dt_spec button2 =
	GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

static const struct device *power_3v3 =
	DEVICE_DT_GET(DT_NODELABEL(power_3v3));
static const struct device *power_5v =
	DEVICE_DT_GET(DT_NODELABEL(power_5v));

static struct gpio_callback btn_cb_data;
static struct k_sem btn_sem;

static void button_pressed(const struct device *dev,
			    struct gpio_callback *cb, uint32_t pins)
{
	k_sem_give(&btn_sem);
}

int main(void)
{
	k_sem_init(&btn_sem, 0, 1);

	printk("=== Sleep Test ===\n");
	printk("5 second window to connect debugger...\n");
	k_msleep(5000);

	/* Turn off 3V3 rail (GPS, LoRa, sensors, SD) */
	if (device_is_ready(power_3v3)) {
		pm_device_action_run(power_3v3, PM_DEVICE_ACTION_SUSPEND);
		printk("3V3 rail: OFF\n");
	}

	/* Turn off 5V rail (display) */
	if (device_is_ready(power_5v)) {
		pm_device_action_run(power_5v, PM_DEVICE_ACTION_SUSPEND);
		printk("5V rail: OFF\n");
	}

	/* Configure button2 as wake source */
	if (!gpio_is_ready_dt(&button2)) {
		printk("Button2 GPIO not ready\n");
		return -ENODEV;
	}

	gpio_pin_configure_dt(&button2, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button2, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&btn_cb_data, button_pressed, BIT(button2.pin));
	gpio_add_callback(button2.port, &btn_cb_data);

	printk("Power domains off. Entering STOP3.\n");
	printk("Press button2 to wake.\n\n");

	while (1) {
		/* Block here — kernel will enter STOP3 while idle */
		k_sem_take(&btn_sem, K_FOREVER);

		/* Debounce */
		k_msleep(50);

		printk("AWAKE — press button2 to sleep again\n");

		/* Wait for next press to go back to sleep */
		k_sem_reset(&btn_sem);
		k_sem_take(&btn_sem, K_FOREVER);
		k_msleep(50);

		printk("Sleeping...\n");
	}

	return 0;
}
