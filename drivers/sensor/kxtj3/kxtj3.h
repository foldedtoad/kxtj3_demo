/*
 *  Kionix KXTJ3 3-axis accelerometer driver header file
 */

#ifndef _SENSOR_KIONIX_KXTJ3_H_
#define _SENSOR_KIONIX_KXTJ3_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <string.h>

#define KXTJ3_REG_WAI           0x0f
#define KXTJ3_CHIP_ID           0x35

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#include <zephyr/drivers/spi.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

#include <zephyr/drivers/i2c.h>

#define KXTJ3_REG_CTRL0     0x1e
#define KXTJ3_SDO_PU_DISC_MASK      BIT(7)

#define KXTJ3_REG_CTRL1     0x20
#define KXTJ3_ACCEL_X_EN_BIT        BIT(0)
#define KXTJ3_ACCEL_Y_EN_BIT        BIT(1)
#define KXTJ3_ACCEL_Z_EN_BIT        BIT(2)
#define KXTJ3_ACCEL_EN_BITS     (KXTJ3_ACCEL_X_EN_BIT | \
                                 KXTJ3_ACCEL_Y_EN_BIT | \
                                 KXTJ3_ACCEL_Z_EN_BIT)
#define KXTJ3_ACCEL_XYZ_MASK        BIT_MASK(3)

#define KXTJ3_LP_EN_BIT_MASK        BIT(3)
#if defined(CONFIG_KXTJ3_OPER_MODE_LOW_POWER)
    #define KXTJ3_LP_EN_BIT BIT(3)
#else
    #define KXTJ3_LP_EN_BIT 0
#endif

#define KXTJ3_SUSPEND           0

#define KXTJ3_ODR_1         1
#define KXTJ3_ODR_2         2
#define KXTJ3_ODR_3         3
#define KXTJ3_ODR_4         4
#define KXTJ3_ODR_5         5
#define KXTJ3_ODR_6         6
#define KXTJ3_ODR_7         7
#define KXTJ3_ODR_8         8
#define KXTJ3_ODR_9         9

#if defined(CONFIG_KXTJ3_ODR_1)
    #define KXTJ3_ODR_IDX       KXTJ3_ODR_1
#elif defined(CONFIG_KXTJ3_ODR_2)
    #define KXTJ3_ODR_IDX       KXTJ3_ODR_2
#elif defined(CONFIG_KXTJ3_ODR_3)
    #define KXTJ3_ODR_IDX       KXTJ3_ODR_3
#elif defined(CONFIG_KXTJ3_ODR_4) || defined(CONFIG_KXTJ3_ODR_RUNTIME)
    #define KXTJ3_ODR_IDX       KXTJ3_ODR_4
#elif defined(CONFIG_KXTJ3_ODR_5)
    #define KXTJ3_ODR_IDX       KXTJ3_ODR_5
#elif defined(CONFIG_KXTJ3_ODR_6)
    #define KXTJ3_ODR_IDX       KXTJ3_ODR_6
#elif defined(CONFIG_KXTJ3_ODR_7)
    #define KXTJ3_ODR_IDX       KXTJ3_ODR_7
#elif defined(CONFIG_KXTJ3_ODR_8)
    #define KXTJ3_ODR_IDX       KXTJ3_ODR_8
#elif defined(CONFIG_KXTJ3_ODR_9_NORMAL) || defined(CONFIG_KXTJ3_ODR_9_LOW)
    #define KXTJ3_ODR_IDX       KXTJ3_ODR_9
#endif

#define KXTJ3_ODR_SHIFT     4
#define KXTJ3_ODR_RATE(r)       ((r) << KXTJ3_ODR_SHIFT)
#define KXTJ3_ODR_BITS          (KXTJ3_ODR_RATE(KXTJ3_ODR_IDX))
#define KXTJ3_ODR_MASK          (BIT_MASK(4) << KXTJ3_ODR_SHIFT)

#define KXTJ3_REG_CTRL2     0x21
#define KXTJ3_HPIS1_EN_BIT      BIT(0)
#define KXTJ3_HPIS2_EN_BIT      BIT(1)
#define KXTJ3_FDS_EN_BIT        BIT(3)

#define KXTJ3_HPIS_EN_MASK      BIT_MASK(2)

#define KXTJ3_REG_CTRL3     0x22
#define KXTJ3_EN_CLICK_INT1     BIT(7)
#define KXTJ3_EN_IA_INT1        BIT(6)
#define KXTJ3_EN_DRDY1_INT1     BIT(4)

#define KXTJ3_REG_CTRL4     0x23
#define KXTJ3_CTRL4_BDU_BIT     BIT(7)
#define KXTJ3_FS_SHIFT          4
#define KXTJ3_FS_MASK           (BIT_MASK(2) << KXTJ3_FS_SHIFT)

#if defined(CONFIG_KXTJ3_ACCEL_RANGE_2G) ||\
    defined(CONFIG_KXTJ3_ACCEL_RANGE_RUNTIME)
    #define KXTJ3_FS_IDX        0
#elif defined(CONFIG_KXTJ3_ACCEL_RANGE_4G)
    #define KXTJ3_FS_IDX        1
#elif defined(CONFIG_KXTJ3_ACCEL_RANGE_8G)
    #define KXTJ3_FS_IDX        2
#elif defined(CONFIG_KXTJ3_ACCEL_RANGE_16G)
    #define KXTJ3_FS_IDX        3
#endif

#define KXTJ3_FS_SELECT(fs)     ((fs) << KXTJ3_FS_SHIFT)
#define KXTJ3_FS_BITS           (KXTJ3_FS_SELECT(KXTJ3_FS_IDX))
#if defined(CONFIG_KXTJ3_OPER_MODE_HIGH_RES)
    #define KXTJ3_HR_BIT        BIT(3)
#else
    #define KXTJ3_HR_BIT        0
#endif

#define KXTJ3_REG_CTRL5     0x24
#define KXTJ3_EN_LIR_INT2       BIT(1)
#define KXTJ3_EN_LIR_INT1       BIT(3)

#define KXTJ3_REG_CTRL6     0x25
#define KXTJ3_EN_CLICK_INT2     BIT(7)
#define KXTJ3_EN_IA_INT2        BIT(5)

#define KXTJ3_REG_REFERENCE     0x26

#define KXTJ3_REG_STATUS        0x27
#define KXTJ3_STATUS_ZYZ_OVR        BIT(7)
#define KXTJ3_STATUS_Z_OVR      BIT(6)
#define KXTJ3_STATUS_Y_OVR      BIT(5)
#define KXTJ3_STATUS_X_OVR      BIT(4)
#define KXTJ3_STATUS_OVR_MASK       (BIT_MASK(4) << 4)
#define KXTJ3_STATUS_ZYX_DRDY       BIT(3)
#define KXTJ3_STATUS_Z_DRDY     BIT(2)
#define KXTJ3_STATUS_Y_DRDY     BIT(1)
#define KXTJ3_STATUS_X_DRDY     BIT(0)
#define KXTJ3_STATUS_DRDY_MASK      BIT_MASK(4)

#define KXTJ3_REG_ACCEL_X_LSB       0x28
#define KXTJ3_REG_ACCEL_Y_LSB       0x2A
#define KXTJ3_REG_ACCEL_Z_LSB       0x2C
#define KXTJ3_REG_ACCEL_X_MSB       0x29
#define KXTJ3_REG_ACCEL_Y_MSB       0x2B
#define KXTJ3_REG_ACCEL_Z_MSB       0x2D

#define KXTJ3_REG_INT1_CFG      0x30
#define KXTJ3_REG_INT1_SRC      0x31
#define KXTJ3_REG_INT1_THS      0x32
#define KXTJ3_REG_INT1_DUR      0x33
#define KXTJ3_REG_INT2_CFG      0x34
#define KXTJ3_REG_INT2_SRC      0x35
#define KXTJ3_REG_INT2_THS      0x36
#define KXTJ3_REG_INT2_DUR      0x37

#define KXTJ3_INT_CFG_MODE_SHIFT    6
#define KXTJ3_INT_CFG_AOI_CFG       BIT(7)
#define KXTJ3_INT_CFG_6D_CFG        BIT(6)
#define KXTJ3_INT_CFG_ZHIE_ZUPE BIT(5)
#define KXTJ3_INT_CFG_ZLIE_ZDOWNE   BIT(4)
#define KXTJ3_INT_CFG_YHIE_YUPE BIT(3)
#define KXTJ3_INT_CFG_YLIE_YDOWNE   BIT(2)
#define KXTJ3_INT_CFG_XHIE_XUPE BIT(1)
#define KXTJ3_INT_CFG_XLIE_XDOWNE   BIT(0)

#define KXTJ3_REG_CFG_CLICK     0x38
#define KXTJ3_EN_CLICK_ZD       BIT(5)
#define KXTJ3_EN_CLICK_ZS       BIT(4)
#define KXTJ3_EN_CLICK_YD       BIT(3)
#define KXTJ3_EN_CLICK_YS       BIT(2)
#define KXTJ3_EN_CLICK_XD       BIT(1)
#define KXTJ3_EN_CLICK_XS       BIT(0)

#define KXTJ3_REG_CLICK_SRC     0x39
#define KXTJ3_CLICK_SRC_DCLICK      BIT(5)
#define KXTJ3_CLICK_SRC_SCLICK      BIT(4)

#define KXTJ3_REG_CFG_CLICK_THS 0x3A
#define KXTJ3_CLICK_LIR     BIT(7)

#define KXTJ3_REG_TIME_LIMIT        0x3B

/* sample buffer size includes status register */
#define KXTJ3_BUF_SZ            7

union kxtj3_sample {
    uint8_t raw[KXTJ3_BUF_SZ];
    struct {
        uint8_t status;
        int16_t xyz[3];
    } __packed;
};

union kxtj3_bus_cfg {
    struct i2c_dt_spec i2c;
};

struct kxtj3_config {
    int (*bus_init)(const struct device *dev);
    const union kxtj3_bus_cfg bus_cfg;
#ifdef CONFIG_KXTJ3_TRIGGER
    const struct gpio_dt_spec gpio_drdy;
    const struct gpio_dt_spec gpio_int;
    const uint8_t int1_mode;
    const uint8_t int2_mode;
#endif /* CONFIG_KXTJ3_TRIGGER */
    struct {
        bool disc_pull_up : 1;
        bool anym_on_int1 : 1;
        bool anym_latch : 1;
        uint8_t anym_mode : 2;
    } hw;
};

struct kxtj3_transfer_function {
    int (*read_data)(const struct device *dev, uint8_t reg_addr,
                     uint8_t *value, uint8_t len);
    int (*write_data)(const struct device *dev, uint8_t reg_addr,
                      uint8_t *value, uint8_t len);
    int (*read_reg)(const struct device *dev, uint8_t reg_addr,
                    uint8_t *value);
    int (*write_reg)(const struct device *dev, uint8_t reg_addr,
                     uint8_t value);
    int (*update_reg)(const struct device *dev, uint8_t reg_addr,
                      uint8_t mask, uint8_t value);
};

struct kxtj3_data {
    const struct device *bus;
    const struct kxtj3_transfer_function *hw_tf;

    union kxtj3_sample sample;
    /* current scaling factor, in micro m/s^2 / lsb */
    uint32_t scale;

#ifdef CONFIG_PM_DEVICE
    uint8_t reg_ctrl1_active_val;
#endif

#ifdef CONFIG_KXTJ3_TRIGGER
    const struct device *dev;
    struct gpio_callback gpio_int1_cb;
    struct gpio_callback gpio_int2_cb;

    sensor_trigger_handler_t handler_drdy;
    const struct sensor_trigger *trig_drdy;
    sensor_trigger_handler_t handler_anymotion;
    const struct sensor_trigger *trig_anymotion;
    sensor_trigger_handler_t handler_tap;
    const struct sensor_trigger *trig_tap;
    atomic_t trig_flags;
    enum sensor_channel chan_drdy;

#if defined(CONFIG_KXTJ3_TRIGGER_OWN_THREAD)
    K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_KXTJ3_THREAD_STACK_SIZE);
    struct k_thread thread;
    struct k_sem gpio_sem;
#elif defined(CONFIG_KXTJ3_TRIGGER_GLOBAL_THREAD)
    struct k_work work;
#endif

#endif /* CONFIG_KXTJ3_TRIGGER */
};

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
int kxtj3_spi_access(struct kxtj3_data *ctx, uint8_t cmd,
              void *data, size_t length);
#endif

#ifdef CONFIG_KXTJ3_TRIGGER
int kxtj3_trigger_set(const struct device *dev,
               const struct sensor_trigger *trig,
               sensor_trigger_handler_t handler);

int kxtj3_init_interrupt(const struct device *dev);

int kxtj3_acc_slope_config(const struct device *dev,
                enum sensor_attribute attr,
                const struct sensor_value *val);
#endif

#ifdef CONFIG_KXTJ3_ACCEL_HP_FILTERS
int kxtj3_acc_hp_filter_set(const struct device *dev,
                 int32_t val);
#endif

int kxtj3_i2c_init(const struct device *dev);


#endif /* _SENSOR_KIONIX_KXTJ3_H_ */
