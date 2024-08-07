/*
 * Copyright (c) 2024 Callender Consulting lLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include "kxtj3_dt_symbols.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);


void fetch_and_display(const struct device *sensor)
{
    static unsigned int count;
    struct sensor_value accel[3];
    const char *overrun = "";
    int rc = sensor_sample_fetch(sensor);

    ++count;
    if (rc == -EBADMSG) {
        /* Sample overrun.  Ignore in polled mode. */
        if (IS_ENABLED(CONFIG_KXTJ3_TRIGGER)) {
            overrun = "[OVERRUN] ";
        }
        rc = 0;
    }
    if (rc == 0) {
        rc = sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_XYZ, accel);
    }
    if (rc < 0) {
        LOG_ERR("ERROR: Update failed: %d", rc);
    }
    else {
#if 1
        LOG_INF("#%u @ %u ms: %sx %f , y %f , z %f",
               count, k_uptime_get_32(), overrun,
               sensor_value_to_double(&accel[0]),
               sensor_value_to_double(&accel[1]),
               sensor_value_to_double(&accel[2]));
#endif
    }
}

#ifdef CONFIG_KXTJ3_TRIGGER

#if USE_ANYMOTION

static void trigger_anymotion_callback(const struct device *dev,
                const struct sensor_trigger *trig)
{
    LOG_INF("%s", __func__);
}

#else
static void trigger_data_callback(const struct device *dev,
                const struct sensor_trigger *trig)
{
    LOG_DBG("%s", __func__);

    fetch_and_display(dev);
}
#endif // USE_ANYMOTION

#endif // CONFIG_KXTJ3_TRIGGER

int main(void)
{
    const struct device *const sensor = DEVICE_DT_GET_ANY(kionix_kxtj3);

    if (sensor == NULL) {
        LOG_INF("No device found");
        return 0;
    }
    if (!device_is_ready(sensor)) {
        LOG_INF("Device %s is not ready", sensor->name);
        return 0;
    }

#if CONFIG_KXTJ3_TRIGGER
    {
        struct sensor_trigger trig;
        int rc;
#if USE_ANYMOTION
        /* Any-Motion output */
        trig.type = SENSOR_TRIG_DELTA;
        trig.chan = 0;

        rc = sensor_trigger_set(sensor, &trig, trigger_anymotion_callback);
        if (rc != 0) {
            LOG_ERR("Failed to set trigger: %d", rc);
            return 0;
        }
  #else
        /* normal x,y,z output */
        trig.type = SENSOR_TRIG_DATA_READY;
        trig.chan = SENSOR_CHAN_ACCEL_XYZ;

        rc = sensor_trigger_set(sensor, &trig, trigger_data_callback);
        if (rc != 0) {
            LOG_ERR("Failed to set trigger: %d", rc);
            return 0;
        }
#endif // USE_ANYMOTION

        LOG_INF("Waiting for triggers");
        while (true) {
            k_sleep(K_MSEC(2000));
        }
    }
#else /* CONFIG_KXTJ3_TRIGGER */
    LOG_INF("Polling at 0.5 Hz");
    while (true) {
        fetch_and_display(sensor);
        k_sleep(K_MSEC(2000));
    }
#endif /* CONFIG_KXTJ3_TRIGGER */
}
