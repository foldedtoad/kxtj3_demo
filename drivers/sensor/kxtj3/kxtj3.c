/* 
 *  Kionix KXTJ3 3-axis accelerometer driver
 */

#define DT_DRV_COMPAT kionix_kxtj3

#include <zephyr/init.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(kxtj3);

#include "kxtj3.h"
#include "kxtj3_dt_symbols.h"

/* 
 *  Note: The Device Tree Compiler (DTC) enforces enums define in 
 *        "kioni,kxtj3-common.yaml" file, so direct indexing should 
 *        be safe.  FLW.
 */ 
uint8_t odr_table [] = {
    /* KXTJ3_DT_ODR_0p781_HZ 0  */  0x08,
    /* KXTJ3_DT_ODR_1p563_HZ 1  */  0x09,
    /* KXTJ3_DT_ODR_3p125_HZ 2  */  0x0A,
    /* KXTJ3_DT_ODR_6p25_HZ  3  */  0x0B,
    /* KXTJ3_DT_ODR_12p5_HZ  4  */  0x00,
    /* KXTJ3_DT_ODR_25_HZ    5  */  0x01,
    /* KXTJ3_DT_ODR_50_HZ    6  */  0x02,
    /* KXTJ3_DT_ODR_100_HZ   7  */  0x03,
    /* KXTJ3_DT_ODR_200_HZ   8  */  0x04,
    /* KXTJ3_DT_ODR_400_HZ   9  */  0x05,
    /* KXTJ3_DT_ODR_800_HZ   10 */  0x06,
    /* KXTJ3_DT_ODR_1600_HZ  11 */  0x07,
};

uint8_t mode_table [] = {
    /* KXTJ3_DT_ACCEL_MODE_2G_8BIT   0 */   0x00,
    /* KXTJ3_DT_ACCEL_MODE_2G_12BIT  1 */   0x40,
    /* KXTJ3_DT_ACCEL_MODE_4G_8BIT   2 */   0x10,
    /* KXTJ3_DT_ACCEL_MODE_4G_12BIT  3 */   0x50,
    /* KXTJ3_DT_ACCEL_MODE_8G_8BIT   4 */   0x08,
    /* KXTJ3_DT_ACCEL_MODE_8G_12BIT  5 */   0x48,
    /* KXTJ3_DT_ACCEL_MODE_16G_8BIT  6 */   0x04,
    /* KXTJ3_DT_ACCEL_MODE_16G_12BIT 7 */   0x54,
    /* KXTJ3_DT_ACCEL_MODE_16G_14BIT 8 */   0x5C,
};

static void set_mask_scale(const struct device * dev)
{
    struct kxtj3_data * kxtj3 = dev->data;
    const struct kxtj3_config * cfg = dev->config;

    switch (cfg->hw.accel_mode) {
        case KXTJ3_DT_ACCEL_MODE_2G_8BIT:
        case KXTJ3_DT_ACCEL_MODE_4G_8BIT:
        case KXTJ3_DT_ACCEL_MODE_8G_8BIT:
        case KXTJ3_DT_ACCEL_MODE_16G_8BIT:
            kxtj3->scale = 127;
            kxtj3->mask = 0b1111111100000000;
            break;

        case KXTJ3_DT_ACCEL_MODE_2G_12BIT:
        case KXTJ3_DT_ACCEL_MODE_4G_12BIT:
        case KXTJ3_DT_ACCEL_MODE_8G_12BIT:
        case KXTJ3_DT_ACCEL_MODE_16G_12BIT:
            kxtj3->scale = 2048;
            kxtj3->mask = 0b1111111111110000;
            break;

        case KXTJ3_DT_ACCEL_MODE_16G_14BIT:
            kxtj3->scale = 8192;
            kxtj3->mask = 0b1111111111111100;
            break;

        default:
            LOG_ERR("%s: undefine mode: %u", __func__, cfg->hw.accel_mode);
            break;
    }

}

static int soft_reset(const struct device * dev)
{
    struct kxtj3_data * kxtj3 = dev->data;
    int status = 0;
    uint8_t reg[1];

    reg[0] = KXTJ3_CTRL_REG2_SRST;
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_CTRL_REG2, reg, sizeof(reg));
    if (status < 0)
    {
        LOG_INF("%s: normal soft reset failed", __func__);

        const struct device * const dev_alt = DEVICE_DT_GET(DT_NODELABEL(kxtj3_alt));
        struct kxtj3_data * kxtj3_alt = dev_alt->data;
        const struct kxtj3_config * cfg_alt = dev_alt->config;

        if (dev_alt == NULL) {
            LOG_INF("No dev_alt found");
            return 0;
        }

        status = cfg_alt->bus_init(dev_alt);
        if (status < 0) {
            return status;
        }

        status = kxtj3_alt->hw_tf->write_data(dev_alt, KXTJ3_CTRL_REG2, reg, sizeof(reg));
        if (status < 0) {
            LOG_INF("%s: alternate soft reset failed", __func__);
            return 0;
        }

        reg[0] = 0;
        status = kxtj3->hw_tf->write_data(dev, KXTJ3_CTRL_REG2, reg, sizeof(reg));
    }

    k_sleep(K_MSEC(2));

    LOG_INF("soft reset OK");

    return status;
}


static void kxtj3_convert(int16_t raw_val, uint32_t scale,
                          struct sensor_value *val)
{
    int32_t converted_val;

    /*
     * maximum converted value we can get is: max(raw_val) * max(scale)
     *  max(raw_val >> 4) = +/- 2^11
     *  max(scale) = 114921
     *  max(converted_val) = 235358208 which is less than 2^31
     */
    converted_val = (raw_val >> 4) * scale;
    val->val1 = converted_val / 1000000;
    val->val2 = converted_val % 1000000;
}


static int kxtj3_channel_get(const struct device *dev,
                             enum sensor_channel chan,
                             struct sensor_value *val)
{
    struct kxtj3_data *kxtj3 = dev->data;
    int ofs_start;
    int ofs_end;
    int i;

    switch (chan) {
    case SENSOR_CHAN_ACCEL_X:
        ofs_start = ofs_end = 0;
        break;
    case SENSOR_CHAN_ACCEL_Y:
        ofs_start = ofs_end = 1;
        break;
    case SENSOR_CHAN_ACCEL_Z:
        ofs_start = ofs_end = 2;
        break;
    case SENSOR_CHAN_ACCEL_XYZ:
        ofs_start = 0;
        ofs_end = 2;
        break;
    default:
        return -ENOTSUP;
    }

    for (i = ofs_start; i <= ofs_end; i++, val++) {
        kxtj3_convert(kxtj3->sample.xyz[i], kxtj3->scale, val);
    }

    return 0;
}

static int kxtj3_fetch_xyz(const struct device *dev,
                           enum sensor_channel chan)
{
    struct kxtj3_data *kxtj3 = dev->data;
    int status = -ENODATA;
    size_t i;

    /*
     * since status and all accel data register addresses are consecutive,
     * a burst read can be used to read all the samples
     */
    status = kxtj3->hw_tf->read_data(dev, KXTJ3_OUT_X_L,
                                     kxtj3->sample.raw,
                                     sizeof(kxtj3->sample.raw));
    if (status < 0) {
        LOG_WRN("Could not read accel axis data");
        return status;
    }

    LOG_DBG("raw: x[%02x-%02x], y[%02x-%02x], z[%02x-%02x]", 
        kxtj3->sample.raw[0], kxtj3->sample.raw[1],
        kxtj3->sample.raw[2], kxtj3->sample.raw[3],
        kxtj3->sample.raw[4], kxtj3->sample.raw[5]);

    for (i = 0; i < (3 * sizeof(int16_t)); i += sizeof(int16_t)) {

        int16_t *sample = (int16_t *)&kxtj3->sample.raw[i];

        *sample = sys_le16_to_cpu(*sample);

        *sample &= kxtj3->mask;
    }

#if 0

    LOG_DBG("samplex: %04x, %04x, %04x", 
           kxtj3->sample.xyz[0],
           kxtj3->sample.xyz[1],
           kxtj3->sample.xyz[2]);

    LOG_DBG("sample: %d, %d, %d", 
           kxtj3->sample.xyz[0],
           kxtj3->sample.xyz[1],
           kxtj3->sample.xyz[2]);

    float results[3];

    results[0] = (float) kxtj3->sample.xyz[0] / 2047; //16384;
    results[1] = (float) kxtj3->sample.xyz[1] / 2047; //16384;
    results[2] = (float) kxtj3->sample.xyz[2] / 2047; //16384;

    LOG_INF("results: x: %.2g, \ty: %.2g, \tz: %.2g", 
                     (double)results[0], 
                     (double)results[1],
                     (double)results[2]);
#endif

    return status;
}

static int kxtj3_sample_fetch(const struct device *dev,
                              enum sensor_channel chan)
{
    int status = -ENODATA;

    if (chan == SENSOR_CHAN_ALL) {
        status = kxtj3_fetch_xyz(dev, chan);
    }
    else if (chan == SENSOR_CHAN_ACCEL_XYZ) {
        status = kxtj3_fetch_xyz(dev, chan);
    }
    else {
        __ASSERT(false, "Invalid sensor channel in fetch");
    }

    return status;
}

static int kxtj3_acc_config(const struct device *dev,
                            enum sensor_channel chan,
                            enum sensor_attribute attr,
                            const struct sensor_value *val)
{
    switch (attr) {
#ifdef CONFIG_KXTJ3_ACCEL_HP_FILTERS
    case SENSOR_ATTR_CONFIGURATION:
        return kxtj3_acc_hp_filter_set(dev, val->val1);
#endif
    default:
        LOG_DBG("Accel attribute not supported.");
        return -ENOTSUP;
    }

    return 0;
}

static int kxtj3_attr_set(const struct device *dev, 
                          enum sensor_channel chan,
                          enum sensor_attribute attr,
                          const struct sensor_value *val)
{
    switch (chan) {
    case SENSOR_CHAN_ACCEL_X:
    case SENSOR_CHAN_ACCEL_Y:
    case SENSOR_CHAN_ACCEL_Z:
    case SENSOR_CHAN_ACCEL_XYZ:
        return kxtj3_acc_config(dev, chan, attr, val);
    default:
        LOG_WRN("attr_set() not supported on this channel.");
        return -ENOTSUP;
    }

    return 0;
}

static const struct sensor_driver_api kxtj3_driver_api = {
    .attr_set = kxtj3_attr_set,
#if CONFIG_KXTJ3_TRIGGER
    .trigger_set = kxtj3_trigger_set,
#endif
    .sample_fetch = kxtj3_sample_fetch,
    .channel_get = kxtj3_channel_get,
};

int kxtj3_init(const struct device *dev)
{
    struct kxtj3_data *kxtj3 = dev->data;
    const struct kxtj3_config *cfg = dev->config;
    int status;
    uint8_t id;
    uint8_t reg[1];

    status = cfg->bus_init(dev);
    if (status < 0) {
        return status;
    }

    /* Quick return if this instance is marked as alt-reset-dev */
    if (cfg->hw.alt_reset_dev) {
        return 0;
    }

    soft_reset(dev);

    status = kxtj3->hw_tf->read_reg(dev, KXTJ3_WHO_AM_I, &id);
    if (status < 0) {
        LOG_ERR("Failed to read chip id.");
        return status;
    }

    if (id != KXTJ3_WHO_AM_I_VALUE) {
        LOG_ERR("Invalid chip ID: %02x\n", id);
        return -EINVAL;
    }

    set_mask_scale(dev);

    reg[0] = 0;    
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_CTRL_REG1, reg, sizeof(reg));
    if (status < 0) {
        LOG_ERR("Failed to go to standby mode.");
        return status;
    }

    LOG_INF("%s: DT accel_rate: %u", __func__, cfg->hw.accel_rate);
    LOG_INF("%s: odr_bits: 0x%02x", __func__, odr_table[cfg->hw.accel_rate]);

    reg[0] = odr_table[cfg->hw.accel_rate];
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_DATA_CTRL_REG, reg, sizeof(reg));

    if (status < 0) {
        LOG_ERR("Failed to set accel_rate.");
        return status;
    }

    LOG_INF("%s: DT accel_mode: %u", __func__, cfg->hw.accel_mode);
    LOG_INF("%s: mode_bits: 0x%02x", __func__, mode_table[cfg->hw.accel_mode]);

    reg[0] = KXTJ3_CTRL_REG1_PC + mode_table[cfg->hw.accel_mode];
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_CTRL_REG1, reg, sizeof(reg));
    if (status < 0) {
        LOG_ERR("Failed to set accel_mode.");
        return status;
    }

#ifdef CONFIG_KXTJ3_TRIGGER
    if (cfg->gpio_drdy.port != NULL || cfg->gpio_int.port != NULL) {
        status = kxtj3_init_interrupt(dev);
        if (status < 0) {
            LOG_ERR("Failed to initialize interrupts.");
            return status;
        }
    }
#endif

    return 0;

}

#ifdef CONFIG_PM_DEVICE
static int kxtj3_pm_action(const struct device *dev,
                enum pm_device_action action)
{
    int status;
    struct kxtj3_data *kxtj3 = dev->data;
    uint8_t regdata;

    switch (action) {
    case PM_DEVICE_ACTION_RESUME:
        status = kxtj3->hw_tf->read_reg(dev, KXTJ3_REG_REFERENCE, &regdata);
        if (status < 0) {
            LOG_ERR("failed to read reg_reference");
            return status;
        }

        /* Resume previous mode. */
        status = kxtj3->hw_tf->write_reg(dev, KXTJ3_REG_CTRL1,
                          kxtj3->reg_ctrl1_active_val);
        if (status < 0) {
            LOG_ERR("failed to write reg_crtl1");
            return status;
        }
        break;
    case PM_DEVICE_ACTION_SUSPEND:
        /* Store current mode, suspend. */
        status = kxtj3->hw_tf->read_reg(dev, KXTJ3_REG_CTRL1,
                         &kxtj3->reg_ctrl1_active_val);
        if (status < 0) {
            LOG_ERR("failed to read reg_crtl1");
            return status;
        }
        status = kxtj3->hw_tf->write_reg(dev, KXTJ3_REG_CTRL1, KXTJ3_SUSPEND);
        if (status < 0) {
            LOG_ERR("failed to write reg_crtl1");
            return status;
        }
        break;
    default:
        return -ENOTSUP;
    }

    return 0;
}
#endif /* CONFIG_PM_DEVICE */

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "KXTJ3 driver enabled without any devices"
#endif

/*
 * Device creation macro, KXTJ3_DEFINE_I2C().
 */

#define KXTJ3_DEVICE_INIT(inst)                        \
    PM_DEVICE_DT_INST_DEFINE(inst, kxtj3_pm_action);   \
    SENSOR_DEVICE_DT_INST_DEFINE(inst,                 \
                kxtj3_init,                            \
                PM_DEVICE_DT_INST_GET(inst),           \
                &kxtj3_data_##inst,                    \
                &kxtj3_config_##inst,                  \
                POST_KERNEL,                           \
                CONFIG_SENSOR_INIT_PRIORITY,           \
                &kxtj3_driver_api);

#define GET_DT_ODR(inst) \
    DT_INST_PROP(inst, accel_rate)

#define GET_DT_MODE(inst) \
    DT_INST_PROP(inst, accel_mode)

#define GET_DT_ALT_RESET_DEV(inst) \
    DT_INST_PROP(inst, alt_reset_dev)

#define ANYMOTION_ON_INT(inst) \
    DT_INST_PROP(inst, anymotion_on_int)

#define ANYMOTION_LATCH(inst) \
    !DT_INST_PROP(inst, anymotion_latch)

#define ANYMOTION_MODE(inst) \
    DT_INST_PROP(inst, anymotion_mode)

#ifdef CONFIG_KXTJ3_TRIGGER
#define GPIO_DT_SPEC_INST_GET_BY_IDX_COND(id, prop, idx)    \
    COND_CODE_1(DT_INST_PROP_HAS_IDX(id, prop, idx),        \
            (GPIO_DT_SPEC_INST_GET_BY_IDX(id, prop, idx)),  \
            ({.port = NULL, .pin = 0, .dt_flags = 0}))

#define KXTJ3_CFG_INT(inst)                                         \
    .gpio_drdy =                                                    \
        COND_CODE_1(ANYMOTION_ON_INT(inst),                         \
        ({.port = NULL, .pin = 0, .dt_flags = 0}),                  \
        (GPIO_DT_SPEC_INST_GET_BY_IDX_COND(inst, irq_gpios, 0))),   \
    .gpio_int =                                                     \
        COND_CODE_1(ANYMOTION_ON_INT(inst),                         \
        (GPIO_DT_SPEC_INST_GET_BY_IDX_COND(inst, irq_gpios, 0)),    \
        (GPIO_DT_SPEC_INST_GET_BY_IDX_COND(inst, irq_gpios, 1))),   \
    .int_mode = DT_INST_PROP(inst, int_gpio_config)
#else
#define KXTJ3_CFG_INT(inst)
#endif /* CONFIG_KXTJ3_TRIGGER */


/*
 * Instantiation macros used when a device is on an I2C bus.
 */
#define KXTJ3_CONFIG_I2C(inst)                              \
    {                                                       \
        .bus_init = kxtj3_i2c_init,                         \
        .bus_cfg = {                                        \
            .i2c = I2C_DT_SPEC_INST_GET(inst),              \
        },                                                  \
        .hw = {                                             \
            .alt_reset_dev = GET_DT_ALT_RESET_DEV(inst),    \
            .anymotion_on_int = ANYMOTION_ON_INT(inst),     \
            .anymotion_latch = ANYMOTION_LATCH(inst),       \
            .anymotion_mode = ANYMOTION_MODE(inst),         \
            .accel_rate = GET_DT_ODR(inst),                 \
            .accel_mode = GET_DT_MODE(inst),                \
        },                                                  \
        KXTJ3_CFG_INT(inst)                                 \
    }

#define KXTJ3_DEFINE_I2C(inst)                              \
    static struct kxtj3_data kxtj3_data_##inst;             \
    static const struct kxtj3_config kxtj3_config_##inst =  \
         KXTJ3_CONFIG_I2C(inst);                            \
    KXTJ3_DEVICE_INIT(inst)


#define KXTJ3_DEFINE(inst)          \
        KXTJ3_DEFINE_I2C(inst)

DT_INST_FOREACH_STATUS_OKAY(KXTJ3_DEFINE)
