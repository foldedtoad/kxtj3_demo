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


/*
 *  Convert Kconfig defines to kxtj3 ODR bits
 */
#if defined(CONFIG_KXTJ3_ODR_1)
                                    #define KXTJ3_ODR_BITS        0x08
#elif defined(CONFIG_KXTJ3_ODR_2)
                                    #define KXTJ3_ODR_BITS        0x09
#elif defined(CONFIG_KXTJ3_ODR_3)
                                    #define KXTJ3_ODR_BITS        0x0A
#elif defined(CONFIG_KXTJ3_ODR_4)
                                    #define KXTJ3_ODR_BITS        0x0B
#elif defined(CONFIG_KXTJ3_ODR_5)
                                    #define KXTJ3_ODR_BITS        0x00
#elif defined(CONFIG_KXTJ3_ODR_6)
                                    #define KXTJ3_ODR_BITS        0x01
#elif defined(CONFIG_KXTJ3_ODR_7)
                                    #define KXTJ3_ODR_BITS        0x02
#elif defined(CONFIG_KXTJ3_ODR_8)
                                    #define KXTJ3_ODR_BITS        0x03
#elif defined(CONFIG_KXTJ3_ODR_9)
                                    #define KXTJ3_ODR_BITS        0x04
#elif defined(CONFIG_KXTJ3_ODR_10)
                                    #define KXTJ3_ODR_BITS        0x05
#elif defined(CONFIG_KXTJ3_ODR_11)
                                    #define KXTJ3_ODR_BITS        0x06
#elif defined(CONFIG_KXTJ3_ODR_12)
                                    #define KXTJ3_ODR_BITS        0x07
#else
                                    #define KXTJ3_ODR_BITS        0xFF
#endif


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

    for (i = 0; i < (3 * sizeof(int16_t)); i += sizeof(int16_t)) {

        int16_t *sample = (int16_t *)&kxtj3->sample.raw[1 + i];

        *sample = sys_le16_to_cpu(*sample);
    }

#if 0
    if (kxtj3->sample.status & KXTJ3_STATUS_DRDY_MASK) {
        status = 0;
    }
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

#ifdef CONFIG_LIS2DH_ACCEL_RANGE_RUNTIME

#define KXTJ3_RANGE_IDX_TO_VALUE(idx)      (1 << ((idx) + 1))
#define KXTJ3_NUM_RANGES           4

static int kxtj3_range_to_reg_val(uint16_t range)
{
    int i;

    for (i = 0; i < KXTJ3_NUM_RANGES; i++) {
        if (range == KXTJ3_RANGE_IDX_TO_VALUE(i)) {
            return i;
        }
    }

    return -EINVAL;
}

static int kxtj3_acc_range_set(const struct device *dev, int32_t range)
{
    struct kxtj3_data *kxtj3 = dev->data;
    int fs;

    fs = kxtj3_range_to_reg_val(range);
    if (fs < 0) {
        return fs;
    }

    kxtj3->scale = kxtj3_reg_val_to_scale[fs];

    return kxtj3->hw_tf->update_reg(dev, KXTJ3_REG_CTRL4,
                     KXTJ3_FS_MASK,
                     (fs << KXTJ3_FS_SHIFT));
}
#endif

static int kxtj3_acc_config(const struct device *dev,
                            enum sensor_channel chan,
                            enum sensor_attribute attr,
                            const struct sensor_value *val)
{
    switch (attr) {
#ifdef CONFIG_KXTJ3_ACCEL_RANGE_RUNTIME
    case SENSOR_ATTR_FULL_SCALE:
        return kxtj3_acc_range_set(dev, sensor_ms2_to_g(val));
#endif
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
    uint8_t raw[6];

    status = cfg->bus_init(dev);
    if (status < 0) {
        return status;
    }
    status = kxtj3->hw_tf->read_reg(dev, KXTJ3_WHO_AM_I, &id);
    if (status < 0) {
        LOG_ERR("Failed to read chip id.");
        return status;
    }

    if (id != KXTJ3_WHO_AM_I_VALUE) {
        LOG_ERR("Invalid chip ID: %02x\n", id);
        return -EINVAL;
    }

    memset(raw, 0, sizeof(raw));
    
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_CTRL_REG1, raw, 1);
    if (status < 0) {
        LOG_ERR("Failed to go to standby mode.");
        return status;
    }

    raw[0] = KXTJ3_ODR_BITS;
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_DATA_CTRL_REG, raw, 1);

    if (status < 0) {
        LOG_ERR("Failed to set ODR.");
        return status;
    }

    raw[0] = KXTJ3_CTRL_REG1_PC + KXTJ3_CTRL_REG1_RES;
    status = kxtj3->hw_tf->write_data(dev, KXTJ3_CTRL_REG1, raw, 1);
    if (status < 0) {
        LOG_ERR("Failed to go to operating mode.");
        return status;
    }

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

#define IS_LSM303AGR_DEV(inst) \
    DT_NODE_HAS_COMPAT(DT_DRV_INST(inst), st_lsm303agr_accel)

#define ANYM_ON_INT(inst) \
    DT_INST_PROP(inst, anym_on_int)

#define ANYM_LATCH(inst) \
    !DT_INST_PROP(inst, anym_no_latch)

#define ANYM_MODE(inst) \
    DT_INST_PROP(inst, anym_mode)

#ifdef CONFIG_KXTJ3_TRIGGER
#define GPIO_DT_SPEC_INST_GET_BY_IDX_COND(id, prop, idx)    \
    COND_CODE_1(DT_INST_PROP_HAS_IDX(id, prop, idx),        \
            (GPIO_DT_SPEC_INST_GET_BY_IDX(id, prop, idx)),  \
            ({.port = NULL, .pin = 0, .dt_flags = 0}))

#define KXTJ3_CFG_INT(inst)                                         \
    .gpio_drdy =                                                    \
        COND_CODE_1(ANYM_ON_INT(inst),                             \
        ({.port = NULL, .pin = 0, .dt_flags = 0}),                  \
        (GPIO_DT_SPEC_INST_GET_BY_IDX_COND(inst, irq_gpios, 0))),   \
    .gpio_int =                                                     \
        COND_CODE_1(ANYM_ON_INT(inst),                             \
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
            .anym_on_int = ANYM_ON_INT(inst),               \
            .anym_latch = ANYM_LATCH(inst),                 \
            .anym_mode = ANYM_MODE(inst),                   \
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
