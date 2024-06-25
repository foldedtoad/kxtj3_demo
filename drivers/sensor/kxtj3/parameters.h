/*
 *  Kionix KXTJ3 parameters file
 */

#ifndef _PARAMETERS_H_
#define _PARAMETERS_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <stdint.h>
#include <string.h>

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

/*
 *  Convert Kconfig defines to kxtj3 RANGE bits
 */
#if defined(CONFIG_KXTJ3_ACCEL_RANGE_2G)
                                    #define KXTJ3_RANGE_BITS      0x00
#elif defined(CONFIG_KXTJ3_ACCEL_RANGE_4G)
                                    #define KXTJ3_RANGE_BITS      0x08
#elif defined(CONFIG_KXTJ3_ACCEL_RANGE_8G)
                                    #define KXTJ3_RANGE_BITS      0x10
#elif defined(CONFIG_KXTJ3_ACCEL_RANGE_16G)
                                    #define KXTJ3_RANGE_BITS      0x01
#else
                                    #define KXTJ3_RANGE_BITS      0xFF
#endif


/*
 *  Convert Kconfig defines to kxtj3 RESOLUTION bits (Operating Mode)
 */
#if defined(CONFIG_KXTJ3_OPER_MODE_LOW_POWER)
                #define KXTJ3_RESOL_BITS      KXTJ3_CTRL_REG1_PC
                #define KXTJ3_SCALE           127
                #define KXTJ3_MASK            0b1111111100000000
#elif defined(CONFIG_KXTJ3_OPER_MODE_NORMAL)
                #define KXTJ3_RESOL_BITS      KXTJ3_CTRL_REG1_RES
                #define KXTJ3_SCALE           2047
                #define KXTJ3_MASK            0b1111111111110000
#elif defined(CONFIG_KXTJ3_OPER_MODE_HIGH_RES)
                #define KXTJ3_RESOL_BITS      (KXTJ3_CTRL_REG1_RES + KXTJ3_CTRL_REG1_EN16G)
                #define KXTJ3_SCALE           8191
                #define KXTJ3_MASK            0b1111111111111100
#else
                #define KXTJ3_RESOL_BITS      0xFF
                #define KXTJ3_SCALE           0
                #define KXTJ3_MASK            0b0000000000000000
#endif



#endif /* _PARAMETERS_H_ */
