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


#include <zephyr/drivers/i2c.h>


#define KXTJ3_WHO_AM_I               0x0F
#define KXTJ3_WHO_AM_I_VALUE         0x35

#define KXTJ3_DCST_RESP              0x0C

#define KXTJ3_OUT_X_L                0x06
#define KXTJ3_OUT_X_H                0x07
#define KXTJ3_OUT_Y_L                0x08
#define KXTJ3_OUT_Y_H                0x09
#define KXTJ3_OUT_Z_L                0x0A
#define KXTJ3_OUT_Z_H                0x0B

#define KXTJ3_STATUS                 0x18
#define KXTJ3_INT_SOURCE1            0x16
#define KXTJ3_INT_SOURCE2            0x17
#define KXTJ3_INT_REL                0x1A

#define KXTJ3_CTRL_REG1              0x1B
#define KXTJ3_CTRL_REG1_PC           0x80
#define KXTJ3_CTRL_REG1_RES          0x40
#define KXTJ3_CTRL_REG1_DRDYE        0x20
#define KXTJ3_CTRL_REG1_GSEL1        0x10
#define KXTJ3_CTRL_REG1_GSEL2        0x08
#define KXTJ3_CTRL_REG1_EN16G        0x04
#define KXTJ3_CTRL_REG1_WUFE         0x02

#define KXTJ3_CTRL_REG2              0x1D
#define KXTJ3_CTRL_REG2_0p781_HZ     0x00
#define KXTJ3_CTRL_REG2_1p563_HZ     0x01
#define KXTJ3_CTRL_REG2_3p125_HZ     0x02
#define KXTJ3_CTRL_REG2_6p25_HZ      0x03
#define KXTJ3_CTRL_REG2_12p5_HZ      0x04
#define KXTJ3_CTRL_REG2_25_HZ        0x05
#define KXTJ3_CTRL_REG2_50_HZ        0x06
#define KXTJ3_CTRL_REG2_100_HZ       0x07

#define KXTJ3_INT_CTRL_REG1          0x1E
#define KXTJ3_INT_CTRL_REG1_IEN      0x20
#define KXTJ3_INT_CTRL_REG1_IEA      0x10
#define KXTJ3_INT_CTRL_REG1_IEL      0x08
#define KXTJ3_INT_CTRL_REG1_STPOL    0x02

#define KXTJ3_INT_CTRL_REG2          0x1F
#define KXTJ3_INT_CTRL_REG2_ULMODE   0x80
#define KXTJ3_INT_CTRL_REG2_XNWUAE   0x20
#define KXTJ3_INT_CTRL_REG2_XPWUAE   0x10
#define KXTJ3_INT_CTRL_REG2_YNWUAE   0x08
#define KXTJ3_INT_CTRL_REG2_YPWUAE   0x04
#define KXTJ3_INT_CTRL_REG2_ZNWUAE   0x02
#define KXTJ3_INT_CTRL_REG2_ZPWUAE   0x01

#define KXTJ3_DATA_CTRL_REG          0x21
#define KXTJ3_DATA_CTRL_REG_0p781_HZ 0x08
#define KXTJ3_DATA_CTRL_REG_1p563_HZ 0x09
#define KXTJ3_DATA_CTRL_REG_3p125_HZ 0x0A
#define KXTJ3_DATA_CTRL_REG_6p25_HZ  0x0B
#define KXTJ3_DATA_CTRL_REG_12p5_HZ  0x00
#define KXTJ3_DATA_CTRL_REG_25_HZ    0x01
#define KXTJ3_DATA_CTRL_REG_50_HZ    0x02
#define KXTJ3_DATA_CTRL_REG_100_HZ   0x03
#define KXTJ3_DATA_CTRL_REG_200_HZ   0x04
#define KXTJ3_DATA_CTRL_REG_400_HZ   0x05
#define KXTJ3_DATA_CTRL_REG_800_HZ   0x06
#define KXTJ3_DATA_CTRL_REG_1600_HZ  0x07

#define KXTJ3_WAKEUP_COUNTER         0x29
#define KXTJ3_NA_COUNTER             0x2A
#define KXTJ3_SELF_TEST              0x3A
#define KXTJ3_WAKEUP_THRD_H          0x6A
#define KXTJ3_WAKEUP_THRD_L          0x6B


extern uint8_t odr_table [];
extern uint8_t mode_table [];


/* sample buffer size includes status register */
#define KXTJ3_BUF_SZ            6

union kxtj3_sample {
    uint8_t raw[KXTJ3_BUF_SZ];
    struct {
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
    const uint8_t int_mode;
#endif /* CONFIG_KXTJ3_TRIGGER */
    struct {
        bool disc_pull_up      : 1;
        bool anymotion_on_int  : 1;
        bool anymotion_latch   : 1;
        uint8_t anymotion_mode : 2;
        uint8_t accel_rate;
        uint8_t accel_mode;
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
    uint16_t scale;
    uint16_t mask;

#ifdef CONFIG_PM_DEVICE
    uint8_t reg_ctrl1_active_val;
#endif

#ifdef CONFIG_KXTJ3_TRIGGER
    const struct device *dev;
    struct gpio_callback gpio_int_cb;

    sensor_trigger_handler_t handler_drdy;
    const struct sensor_trigger *trig_drdy;
    sensor_trigger_handler_t handler_anymotion;
    const struct sensor_trigger *trig_anymotion;
    atomic_t trig_flags;
    enum sensor_channel chan_drdy;

#if defined(CONFIG_KXTJ3_TRIGGER_THREAD)
    struct k_work work;
#endif

#endif /* CONFIG_KXTJ3_TRIGGER */
};

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
