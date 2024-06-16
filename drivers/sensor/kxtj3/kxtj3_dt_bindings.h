/*
 * Copyright (c) 2024 Callender Consulting LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _DT_BINDINGS_KIONIX_KXTJ3_H_
#define _DT_BINDINGS_KIONIX_KXTJ3_H_

/* GPIO interrupt configuration */
#define KXTJ3_DT_GPIO_INT_EDGE             0
#define KXTJ3_DT_GPIO_INT_EDGE_RISING      1
#define KXTJ3_DT_GPIO_INT_EDGE_FALLING     2
#define KXTJ3_DT_GPIO_INT_LEVEL_HIGH       3
#define KXTJ3_DT_GPIO_INT_LEVEL_LOW        4

/* Any Motion mode */
#define KXTJ3_DT_ANYM_OR_COMBINATION       0
#define KXTJ3_DT_ANYM_6D_MOVEMENT          1
#define KXTJ3_DT_ANYM_AND_COMBINATION      2
#define KXTJ3_DT_ANYM_6D_POSITION          3

#endif /* _DT_BINDINGS_KIONIX_KXTJ3_H_ */