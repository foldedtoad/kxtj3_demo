/*
 * Copyright (c) 2024  Callender Consulting LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT kionix_kxtj3

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(kxtj3_trigger);

#include "kxtj3.h"
#include "parameters.h"

#define START_TRIG_INT         0
#define TRIGGED_INT            4

static const gpio_flags_t gpio_int_cfg[5] = {
    GPIO_INT_EDGE,
    GPIO_INT_EDGE_RISING,
    GPIO_INT_EDGE_FALLING,
    GPIO_INT_LEVEL_HIGH,
    GPIO_INT_LEVEL_LOW,
};

static const char * gpio_int_cfg_tags[5] = {
    "EDGE",
    "EDGE_RISING",
    "EDGE_FALLING",
    "LEVEL_HIGH",
    "LEVEL_LOW",
};

static inline void setup_int(const struct device *dev, bool enable)
{
    const struct kxtj3_config *cfg = dev->config;

    if (cfg->gpio_drdy.port) {
        LOG_DBG("%s DRDY interrupt %s: %s",__func__, 
            enable ? "enable" : "disable",
            enable ? gpio_int_cfg_tags[cfg->int_mode] : "");

        gpio_pin_interrupt_configure_dt(&cfg->gpio_drdy,
                                        enable ? gpio_int_cfg[cfg->int_mode] : 
                                                 GPIO_INT_DISABLE);
        return;
    }

    if (cfg->gpio_int.port ) {

        LOG_DBG("%s INT interrupt %s: %s",__func__, 
            enable ? "enable" : "disable",
            enable ? gpio_int_cfg_tags[cfg->int_mode] : "");

        gpio_pin_interrupt_configure_dt(&cfg->gpio_int,
                                        enable ? gpio_int_cfg[cfg->int_mode] : 
                                                 GPIO_INT_DISABLE);
        return;
    }

    LOG_ERR("error ****");
}

static int kxtj3_trigger_drdy_set(const struct device *dev,
                   enum sensor_channel chan,
                   sensor_trigger_handler_t handler,
                   const struct sensor_trigger *trig)
{
    int status = 0;
    struct kxtj3_data *kxtj3 = dev->data;

    LOG_INF("%s", __func__);

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
    struct kxtj3_data *kxtj3 = dev->data;
    uint8_t reg[1];

    LOG_INF("%s", __func__);

    reg[0] = 0;
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_CTRL_REG1, reg, sizeof(reg));
    if (status < 0) {
        return status;
    }

    reg[0] = KXTJ3_CTRL_REG1_PC | KXTJ3_RESOL_BITS | KXTJ3_CTRL_REG1_DRDYE;
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_CTRL_REG1, reg, sizeof(reg));
    if (status < 0) {
        return status;
    }

    return status;
}

static int kxtj3_trigger_anymotion_tap_set(const struct device *dev,
                                           sensor_trigger_handler_t handler,
                                           const struct sensor_trigger *trig)
{
    const struct kxtj3_config *cfg = dev->config;
    struct kxtj3_data *kxtj3 = dev->data;
    int status;
    uint8_t reg[1];

    if (cfg->gpio_int.port == NULL) {
        LOG_ERR("%s: AnyMotion interrupt not supported", __func__);
        return -ENOTSUP;
    }

    LOG_INF("%s", __func__);

    reg[0] = 0;    
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_CTRL_REG1, reg, sizeof(reg));
    if (status < 0) {
        LOG_ERR("Failed to go to standby mode.");
        return status;
    }

    reg[0] = KXTJ3_DATA_CTRL_REG_200_HZ;    
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_DATA_CTRL_REG, reg, sizeof(reg));
    if (status < 0) {
        return status;
    }

    reg[0] = KXTJ3_INT_CTRL_REG1_IEN | KXTJ3_INT_CTRL_REG1_IEA; 
    reg[0] += (cfg->hw.anymotion_latch) ? KXTJ3_INT_CTRL_REG1_IEL : 0;

    status = kxtj3->hw_tf->write_data(dev, KXTJ3_INT_CTRL_REG1, reg, sizeof(reg));
    if (status < 0) {
        return status;
    }

    reg[0] = KXTJ3_INT_CTRL_REG2_XNWUAE | KXTJ3_INT_CTRL_REG2_XPWUAE |   
             KXTJ3_INT_CTRL_REG2_YNWUAE | KXTJ3_INT_CTRL_REG2_YPWUAE |   
             KXTJ3_INT_CTRL_REG2_ZNWUAE | KXTJ3_INT_CTRL_REG2_ZPWUAE ;  
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_INT_CTRL_REG2, reg, sizeof(reg));
    if (status < 0) {
        return status;
    }

    reg[0] = KXTJ3_CTRL_REG2_6p25_HZ;
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_CTRL_REG2, reg, sizeof(reg));
    if (status < 0) {
        return status;
    }

    reg[0] = 0x08;  // how to calculate this?
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_WAKEUP_THRD_H, reg, sizeof(reg));
    if (status < 0) {
        return status;
    }

    reg[0] = KXTJ3_CTRL_REG1_PC | KXTJ3_RESOL_BITS | KXTJ3_CTRL_REG1_WUFE;    
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_CTRL_REG1, reg, sizeof(reg));
    if (status < 0) {
        return status;
    }

    if (trig->type == SENSOR_TRIG_DELTA) {
        kxtj3->handler_anymotion = handler;
        kxtj3->trig_anymotion = trig;
    } 

#if defined(CONFIG_KXTJ3_TRIGGER_THREAD)
    k_work_submit(&kxtj3->work);
#endif
    return 0;
}

static int kxtj3_trigger_anymotion_set(const struct device *dev,
                   sensor_trigger_handler_t handler,
                   const struct sensor_trigger *trig)
{
    return kxtj3_trigger_anymotion_tap_set(dev, handler, trig);
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
        return kxtj3_trigger_anymotion_set(dev, handler, trig);
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
#endif // CONFIG_KXTJ3_ACCEL_HP_FILTERS

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

        LOG_DBG("%s: DRDY callback", __func__);

        if (likely(kxtj3->handler_drdy != NULL)) {
            kxtj3->handler_drdy(dev, kxtj3->trig_drdy);
        }

        /* 
         *  Reactivate level triggered interrupt if handler did not
         *  disable itself
         */
        if (likely(kxtj3->handler_drdy != NULL)) {
            setup_int(dev, true);
        }
        return;
    }

    if (cfg->gpio_int.port &&
        atomic_test_and_clear_bit(&kxtj3->trig_flags, TRIGGED_INT)) {

        LOG_DBG("%s: AnyMotion callback", __func__);

        if (likely(kxtj3->handler_anymotion != NULL)) {
            kxtj3->handler_anymotion(dev, kxtj3->trig_anymotion);
        }

        uint8_t reg;
        kxtj3->hw_tf->read_reg(dev, KXTJ3_INT_REL, &reg);

        /* 
         *  Reactivate level triggered interrupt if handler did not
         *  disable itself
         */
        if (likely(kxtj3->handler_anymotion != NULL)) {
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

    gpio_pin_interrupt_configure_dt(&cfg->gpio_drdy, GPIO_INT_EDGE_RISING);

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

    status = gpio_add_callback(cfg->gpio_drdy.port,
                               &kxtj3->gpio_int_cb);
    if (status < 0) {
        LOG_ERR("Could not add gpio int callback");
        return status;
    }

    LOG_INF("%s: DRDY int on %s.%02u", dev->name,
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

    gpio_init_callback(&kxtj3->gpio_int_cb,
                       kxtj3_gpio_int_callback,
                       BIT(cfg->gpio_int.pin));

    gpio_add_callback_dt(&cfg->gpio_int, &kxtj3->gpio_int_cb);

    gpio_pin_interrupt_configure_dt(&cfg->gpio_int, GPIO_INT_EDGE_RISING);

    LOG_INF("%s: int on %s.%02u", dev->name,
                       cfg->gpio_int.port->name,
                       cfg->gpio_int.pin);

end:
    return status;
}
