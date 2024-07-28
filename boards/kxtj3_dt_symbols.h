/*
 * Copyright (c) 2024 Callender Consulting LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _KXTJ3_DT_SYMBOLS_H_
#define _KXTJ3_DT_SYMBOLS_H_

/* GPIO interrupt configuration */
#define KXTJ3_DT_GPIO_INT_EDGE             0
#define KXTJ3_DT_GPIO_INT_EDGE_RISING      1
#define KXTJ3_DT_GPIO_INT_EDGE_FALLING     2
#define KXTJ3_DT_GPIO_INT_LEVEL_HIGH       3
#define KXTJ3_DT_GPIO_INT_LEVEL_LOW        4

/* AnyMotion Rate */
#define KXTJ3_DT_ANYMOTION_RATE_0p781_HZ 0
#define KXTJ3_DT_ANYMOTION_RATE_1p563_HZ 1
#define KXTJ3_DT_ANYMOTION_RATE_3p125_HZ 2
#define KXTJ3_DT_ANYMOTION_RATE_6p25_HZ  3
#define KXTJ3_DT_ANYMOTION_RATE_12p5_HZ  4
#define KXTJ3_DT_ANYMOTION_RATE_25_HZ    5
#define KXTJ3_DT_ANYMOTION_RATE_50_HZ    6
#define KXTJ3_DT_ANYMOTION_RATE_100_HZ   7

/* Output Data Rate (ODR) in Hz */
#define KXTJ3_DT_ODR_0p781_HZ   0
#define KXTJ3_DT_ODR_1p563_HZ   1
#define KXTJ3_DT_ODR_3p125_HZ   2
#define KXTJ3_DT_ODR_6p25_HZ    3
#define KXTJ3_DT_ODR_12p5_HZ    4
#define KXTJ3_DT_ODR_25_HZ      5
#define KXTJ3_DT_ODR_50_HZ      6
#define KXTJ3_DT_ODR_100_HZ     7
#define KXTJ3_DT_ODR_200_HZ     8
#define KXTJ3_DT_ODR_400_HZ     9
#define KXTJ3_DT_ODR_800_HZ     10
#define KXTJ3_DT_ODR_1600_HZ    11

/* Perfromance mode */
#define KXTJ3_DT_ACCEL_MODE_2G_8BIT     0
#define KXTJ3_DT_ACCEL_MODE_2G_12BIT    1
#define KXTJ3_DT_ACCEL_MODE_4G_8BIT     2
#define KXTJ3_DT_ACCEL_MODE_4G_12BIT    3
#define KXTJ3_DT_ACCEL_MODE_8G_8BIT     4
#define KXTJ3_DT_ACCEL_MODE_8G_12BIT    5
#define KXTJ3_DT_ACCEL_MODE_16G_8BIT    6
#define KXTJ3_DT_ACCEL_MODE_16G_12BIT   7
#define KXTJ3_DT_ACCEL_MODE_16G_14BIT   8

#endif /* _KXTJ3_DT_SYMBOLS_H_ */