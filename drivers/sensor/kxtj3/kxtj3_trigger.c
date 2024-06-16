/*
 * Copyright (c) 2024  Callender Consulting LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT kionix_kxtj3

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(kxtj3, CONFIG_SENSOR_LOG_LEVEL);

#include "kxtj3.h"

#define START_TRIG_INT         0
#define TRIGGED_INT            4

static const gpio_flags_t gpio_int_cfg[5] = {
    GPIO_INT_EDGE,
    GPIO_INT_EDGE_RISING,
    GPIO_INT_EDGE_FALLING,
    GPIO_INT_LEVEL_HIGH,
    GPIO_INT_LEVEL_LOW,
};

static inline void setup_int(const struct device *dev, bool enable)
{
    const struct kxtj3_config *cfg = dev->config;

    gpio_pin_interrupt_configure_dt(&cfg->gpio_drdy,
                    enable ? gpio_int_cfg[cfg->int_mode] : GPIO_INT_DISABLE);
}

static int kxtj3_trigger_drdy_set(const struct device *dev,
                   enum sensor_channel chan,
                   sensor_trigger_handler_t handler,
                   const struct sensor_trigger *trig)
{
    const struct kxtj3_config *cfg = dev->config;
    struct kxtj3_data *kxtj3 = dev->data;
    int status;

    if (cfg->gpio_drdy.port == NULL) {
        LOG_ERR("trigger_set DRDY int not supported");
        return -ENOTSUP;
    }

    setup_int(dev, false);

    /* cancel potentially pending trigger */
    atomic_clear_bit(&kxtj3->trig_flags, TRIGGED_INT);

    status = kxtj3->hw_tf->update_reg(dev, KXTJ3_REG_CTRL3, KXTJ3_EN_DRDY1_INT, 0);

    kxtj3->handler_drdy = handler;
    kxtj3->trig_drdy = trig;
    if ((handler == NULL) || (status < 0)) {
        return status;
    }

    kxtj3->chan_drdy = chan;

    /* serialize start of int in thread to synchronize output sampling
     * and first interrupt. this avoids concurrent bus context access.
     */
    atomic_set_bit(&kxtj3->trig_flags, START_TRIG_INT);
#if defined(CONFIG_KXTJ3_TRIGGER_THREAD) 
    k_work_submit(&kxtj3->work);
#endif

    return 0;
}

static int kxtj3_start_trigger_int(const struct device *dev)
{
    int status;
    uint8_t raw[KXTJ3_BUF_SZ];
    uint8_t ctrl1 = 0U;
    struct kxtj3_data *kxtj3 = dev->data;

    /* power down temporarily to align interrupt & data output sampling */
    status = kxtj3->hw_tf->read_reg(dev, KXTJ3_REG_CTRL1, &ctrl1);
    if (unlikely(status < 0)) {
        return status;
    }
    status = kxtj3->hw_tf->write_reg(dev, KXTJ3_REG_CTRL1, ctrl1 & ~KXTJ3_ODR_MASK);

    if (unlikely(status < 0)) {
        return status;
    }

    LOG_DBG("ctrl1=0x%x @tick=%u", ctrl1, k_cycle_get_32());

    /* empty output data */
    status = kxtj3->hw_tf->read_data(dev, KXTJ3_REG_STATUS,
                      raw, sizeof(raw));
    if (unlikely(status < 0)) {
        return status;
    }

    setup_int(dev, true);

    /* re-enable output sampling */
    status = kxtj3->hw_tf->write_reg(dev, KXTJ3_REG_CTRL1, ctrl1);
    if (unlikely(status < 0)) {
        return status;
    }

    return kxtj3->hw_tf->update_reg(dev, KXTJ3_REG_CTRL3,
                                    KXTJ3_EN_DRDY1_INT,
                                    KXTJ3_EN_DRDY1_INT);
}

#define KXTJ3_ANYM_CFG (KXTJ3_INT_CFG_ZHIE_ZUPE | \
                        KXTJ3_INT_CFG_YHIE_YUPE | \
                        KXTJ3_INT_CFG_XHIE_XUPE)

static int kxtj3_trigger_anym_tap_set(const struct device *dev,
                                      sensor_trigger_handler_t handler,
                                      const struct sensor_trigger *trig)
{
    const struct kxtj3_config *cfg = dev->config;
    struct kxtj3_data *kxtj3 = dev->data;
    int status;
    uint8_t reg_val;

    if (cfg->gpio_int.port == NULL) {
        LOG_ERR("trigger_set AnyMotion int not supported");
        return -ENOTSUP;
    }

    if (cfg->hw.anym_on_int) {
        status = kxtj3->hw_tf->update_reg(dev, KXTJ3_REG_CTRL3, KXTJ3_EN_DRDY1_INT, 0);
    }

    /* disable any movement interrupt events */
    status = kxtj3->hw_tf->write_reg(dev, KXTJ3_REG_INT_CFG, 0);

    /* disable any click interrupt events */
    status = kxtj3->hw_tf->write_reg(dev, KXTJ3_REG_CFG_CLICK, 0);

    /* make sure any pending interrupt is cleared */
    status = kxtj3->hw_tf->read_reg(dev, KXTJ3_REG_INT_SRC, &reg_val);

    status = kxtj3->hw_tf->read_reg(dev, KXTJ3_REG_CLICK_SRC, &reg_val);

    if (trig->type == SENSOR_TRIG_DELTA) {
        kxtj3->handler_anymotion = handler;
        kxtj3->trig_anymotion = trig;
    } 
    else if (trig->type == SENSOR_TRIG_TAP) {
        kxtj3->handler_tap = handler;
        kxtj3->trig_tap = trig;
    }

    if ((handler == NULL) || (status < 0)) {
        return status;
    }

#if defined(CONFIG_KXTJ3_TRIGGER_THREAD)
    k_work_submit(&kxtj3->work);
#endif
    return 0;
}

static int kxtj3_trigger_anym_set(const struct device *dev,
                   sensor_trigger_handler_t handler,
                   const struct sensor_trigger *trig)
{
    return kxtj3_trigger_anym_tap_set(dev, handler, trig);
}

static int kxtj3_trigger_tap_set(const struct device *dev,
                  sensor_trigger_handler_t handler,
                  const struct sensor_trigger *trig)
{
    return kxtj3_trigger_anym_tap_set(dev, handler, trig);
}

int kxtj3_trigger_set(const struct device *dev,
               const struct sensor_trigger *trig,
               sensor_trigger_handler_t handler)
{
    if (trig->type == SENSOR_TRIG_DATA_READY &&
        trig->chan == SENSOR_CHAN_ACCEL_XYZ) {
        return kxtj3_trigger_drdy_set(dev, trig->chan, handler, trig);
    } 
    else if (trig->type == SENSOR_TRIG_DELTA) {
        return kxtj3_trigger_anym_set(dev, handler, trig);
    } 
    else if (trig->type == SENSOR_TRIG_TAP) {
        return kxtj3_trigger_tap_set(dev, handler, trig);
    }

    return -ENOTSUP;
}

#ifdef CONFIG_KXTJ3_ACCEL_HP_FILTERS
int kxtj3_acc_hp_filter_set(const struct device *dev, int32_t val)
{
    struct kxtj3_data *kxtj3 = dev->data;
    int status;

    status = kxtj3->hw_tf->update_reg(dev, KXTJ3_REG_CTRL2, KXTJ3_HPIS_EN_MASK, val);
    if (status < 0) {
        LOG_ERR("Failed to set high pass filters");
    }

    return status;
}
#endif

static void kxtj3_gpio_int_callback(const struct device *dev,
                                    struct gpio_callback *cb, uint32_t pins)
{
    struct kxtj3_data *kxtj3 = CONTAINER_OF(cb, struct kxtj3_data, gpio_int_cb);

    ARG_UNUSED(pins);

    atomic_set_bit(&kxtj3->trig_flags, TRIGGED_INT);

    /* int is level triggered so disable until processed */
    setup_int(kxtj3->dev, false);

#if defined(CONFIG_KXTJ3_TRIGGER_THREAD)
    k_work_submit(&kxtj3->work);
#endif
}

static void kxtj3_thread_cb(const struct device *dev)
{
    struct kxtj3_data *kxtj3 = dev->data;
    const struct kxtj3_config *cfg = dev->config;
    int status;

    if (cfg->gpio_drdy.port &&
        unlikely(atomic_test_and_clear_bit(&kxtj3->trig_flags, START_TRIG_INT))) {
        
        status = kxtj3_start_trigger_int(dev);

        if (unlikely(status < 0)) {
            LOG_ERR("kxtj3_start_trigger_int: %d", status);
        }
        return;
    }

    if (cfg->gpio_drdy.port &&
            atomic_test_and_clear_bit(&kxtj3->trig_flags, TRIGGED_INT)) {
        if (likely(kxtj3->handler_drdy != NULL)) {
            kxtj3->handler_drdy(dev, kxtj3->trig_drdy);
        }

        /* Reactivate level triggered interrupt if handler did not
         * disable itself
         */
        if (likely(kxtj3->handler_drdy != NULL)) {
            setup_int(dev, true);
        }

        return;
    }
}

#ifdef CONFIG_KXTJ3_TRIGGER_THREAD
static void kxtj3_work_cb(struct k_work *work)
{
    struct kxtj3_data *kxtj3 = CONTAINER_OF(work, struct kxtj3_data, work);

    kxtj3_thread_cb(kxtj3->dev);
}
#endif

int kxtj3_init_interrupt(const struct device *dev)
{
    struct kxtj3_data *kxtj3 = dev->data;
    const struct kxtj3_config *cfg = dev->config;
    int status;
    uint8_t raw[2];

    kxtj3->dev = dev;

#if defined(CONFIG_KXTJ3_TRIGGER_THREAD)
    kxtj3->work.handler = kxtj3_work_cb;
#endif

    /*
     * Setup INT (for DRDY) if defined in DT
     */

    /* setup data ready gpio interrupt */
    if (!gpio_is_ready_dt(&cfg->gpio_drdy)) {
        /* API may return false even when ptr is NULL */
        if (cfg->gpio_drdy.port != NULL) {
            LOG_ERR("device %s is not ready", cfg->gpio_drdy.port->name);
            return -ENODEV;
        }

        LOG_DBG("gpio_drdy not defined in DT");
        status = 0;
        goto check_gpio_int;
    }

    /* data ready int gpio configuration */
    status = gpio_pin_configure_dt(&cfg->gpio_drdy, GPIO_INPUT);
    if (status < 0) {
        LOG_ERR("Could not configure %s.%02u",
            cfg->gpio_drdy.port->name, cfg->gpio_drdy.pin);
        return status;
    }

    gpio_init_callback(&kxtj3->gpio_int_cb,
               kxtj3_gpio_int_callback,
               BIT(cfg->gpio_drdy.pin));

    status = gpio_add_callback(cfg->gpio_drdy.port, &kxtj3->gpio_int_cb);
    if (status < 0) {
        LOG_ERR("Could not add gpio int callback");
        return status;
    }

    LOG_INF("%s: int on %s.%02u", dev->name,
                       cfg->gpio_drdy.port->name,
                       cfg->gpio_drdy.pin);

check_gpio_int:
    /*
     * Setup Interrupt (for Any Motion) if defined in DT
     */

    /* setup any motion gpio interrupt */
    if (!gpio_is_ready_dt(&cfg->gpio_int)) {
        /* API may return false even when ptr is NULL */
        if (cfg->gpio_int.port != NULL) {
            LOG_ERR("device %s is not ready", cfg->gpio_int.port->name);
            return -ENODEV;
        }

        LOG_DBG("gpio_int not defined in DT");
        status = 0;
        goto end;
    }

    /* any motion int gpio configuration */
    status = gpio_pin_configure_dt(&cfg->gpio_int, GPIO_INPUT);
    if (status < 0) {
        LOG_ERR("Could not configure %s.%02u",
            cfg->gpio_int.port->name, cfg->gpio_int.pin);
        return status;
    }

    gpio_init_callback(&kxtj3->gpio_int_cb, kxtj3_gpio_int_callback,
                       BIT(cfg->gpio_int.pin));

    /* callback is going to be enabled by trigger setting function */
    status = gpio_add_callback(cfg->gpio_int.port, &kxtj3->gpio_int2_cb);
    if (status < 0) {
        LOG_ERR("Could not add gpio int2 callback (%d)", status);
        return status;
    }

    LOG_INF("%s: int2 on %s.%02u", dev->name,
       cfg->gpio_int.port->name,
       cfg->gpio_int.pin);

    /* disable interrupt in case of warm (re)boot */
    status = kxtj3->hw_tf->write_reg(dev, KXTJ3_REG_INT_CFG, 0);
    if (status < 0) {
        LOG_ERR("Interrupt disable reg write failed (%d)", status);
        return status;
    }
    status = kxtj3->hw_tf->write_reg(dev, KXTJ3_REG_CFG_CLICK, 0);
    if (status < 0) {
        LOG_ERR("Interrupt disable reg write failed (%d)", status);
        return status;
    }

    (void)memset(raw, 0, sizeof(raw));
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_REG_INT_THS, raw, sizeof(raw));
    if (status < 0) {
        LOG_ERR("Burst write to THS failed (%d)", status);
        return status;
    }

    if (cfg->hw.anym_latch) {
        /* latch line interrupt */
        status = kxtj3->hw_tf->write_reg(dev, KXTJ3_REG_CTRL5, KXTJ3_EN_LIR_INT);
    }

    if (status < 0) {
        LOG_ERR("enable reg write failed (%d)", status);
        return status;
    }

end:
    return status;
}
