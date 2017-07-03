/* Cragganmore 6410 shared definitions
 *
 * Copyright 2011 Wolfson Microelectronics plc
 *	Mark Brown <broonie@opensource.wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef MACH_CRAG6410_H
#define MACH_CRAG6410_H

#include <linux/gpio.h>

#define GLENFARCLAS_PMIC_IRQ_BASE	IRQ_BOARD_START
<<<<<<< HEAD
<<<<<<< HEAD
=======
#define BANFF_PMIC_IRQ_BASE		(IRQ_BOARD_START + 64)
>>>>>>> f1f997bb2aa14231c38c2cd423ac6da380356b03
=======
>>>>>>> 2617302... source

#define PCA935X_GPIO_BASE		GPIO_BOARD_START
#define CODEC_GPIO_BASE			(GPIO_BOARD_START + 8)
#define GLENFARCLAS_PMIC_GPIO_BASE	(GPIO_BOARD_START + 32)
#define BANFF_PMIC_GPIO_BASE		(GPIO_BOARD_START + 64)
#define MMGPIO_GPIO_BASE		(GPIO_BOARD_START + 96)

#endif
