/*
 * VBAT ADC sample — reads PA0 (ADC1 IN5) and prints raw + millivolt
 * readings over RTT.
 *
 * The STM32U5 ADC reference is 3.3 V (VDDA).
 * If VBAT is connected via a resistor divider, scale accordingly.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>

/* ADC1, channel 5 (PA0), 12-bit, single-ended */
static const struct adc_dt_spec adc_ch =
	ADC_DT_SPEC_GET(DT_PATH(zephyr_user));

int main(void)
{
	int ret;
	int32_t val_mv;
	uint16_t buf;

	struct adc_sequence seq = {
		.buffer      = &buf,
		.buffer_size = sizeof(buf),
	};

	if (!adc_is_ready_dt(&adc_ch)) {
		printk("ADC not ready\n");
		return -ENODEV;
	}

	ret = adc_channel_setup_dt(&adc_ch);
	if (ret < 0) {
		printk("ADC channel setup failed: %d\n", ret);
		return ret;
	}

	adc_sequence_init_dt(&adc_ch, &seq);

	printk("=== VBAT ADC sample ===\n");

	while (1) {
		ret = adc_read_dt(&adc_ch, &seq);
		if (ret < 0) {
			printk("ADC read error: %d\n", ret);
		} else {
			val_mv = buf;
			ret = adc_raw_to_millivolts_dt(&adc_ch, &val_mv);
			if (ret < 0) {
				printk("raw=%u  (no ref for mV conversion)\n", buf);
			} else {
				/* Resistor divider: 100k (top) + 200k (bottom)
				 * V_adc = VBAT * 200 / 300  =>  VBAT = V_adc * 3 / 2 */
				int32_t vbat_mv = val_mv * 3 / 2;

				printk("raw=%u  adc=%d mV  vbat=%d mV\n",
				       buf, val_mv, vbat_mv);
			}
		}

		k_msleep(1000);
	}

	return 0;
}
