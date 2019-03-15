/*
 * gpio_util.c
 *
 *  Created on: 26 okt. 2018
 *      Author: Cas Doornkamp
 */

#include "gpio_util.h"
// List known GPIO pins

GPIO_Pin SX_IRQ 			= { SX_IRQ_GPIO_Port			, SX_IRQ_Pin			};
GPIO_Pin SX_RST				= { SX_RST_GPIO_Port			, SX_RST_Pin			};
GPIO_Pin SX_BUSY 			= { SX_BUSY_GPIO_Port			, SX_BUSY_Pin			};
GPIO_Pin LD_3 				= { LD3_GPIO_Port			, LD3_Pin			};
