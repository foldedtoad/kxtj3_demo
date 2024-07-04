/*
 * Copyright (c) 2024 Callender Consulting LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _KXTJ3_DT_SYMBOLS_H_
#define _KXTJ3_DT_SYMBOLS_H_

#define USE_ANYMOTION 1

/* GPIO interrupt configuration */
#define KXTJ3_DT_GPIO_INT_EDGE             0
#define KXTJ3_DT_GPIO_INT_EDGE_RISING      1
#define KXTJ3_DT_GPIO_INT_EDGE_FALLING     2
#define KXTJ3_DT_GPIO_INT_LEVEL_HIGH       3
#define KXTJ3_DT_GPIO_INT_LEVEL_LOW        4

/* Any Motion mode */
#define KXTJ3_DT_ANYMOTION_OR_COMBINATION   0
#define KXTJ3_DT_ANYMOTION_6D_MOVEMENT      1
#define KXTJ3_DT_ANYMOTION_AND_COMBINATION  2
#define KXTJ3_DT_ANYMOTION_6D_POSITION      3

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

/* Output Range in G units */
#define KXTJ3_DT_ACCEL_RANGE_2G     0
#define KXTJ3_DT_ACCEL_RANGE_4G     1
#define KXTJ3_DT_ACCEL_RANGE_8G     2
#define KXTJ3_DT_ACCEL_RANGE_16G    3

/* Perfromance mode */
#define KXTJ3_DT_OPER_MODE_LOW_POWER     0
#define KXTJ3_DT_OPER_MODE_HIGH_RES      1
#define KXTJ3_DT_OPER_MODE_HIGH_RES_16G  2

#endif /* _KXTJ3_DT_SYMBOLS_H_ */