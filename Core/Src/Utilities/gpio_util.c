/*
 * gpio_util.c
 *
 *  Created on: 26 okt. 2018
 *      Author: Cas Doornkamp
 */

#include "gpio_util.h"
// List known GPIO pins

GPIO_Pin SX_TX_IRQ 			= { SX_TX_IRQ_GPIO_Port			, SX_TX_IRQ_Pin			};
GPIO_Pin SX_TX_RST			= { SX_TX_RST_GPIO_Port			, SX_TX_RST_Pin			};
GPIO_Pin SX_TX_BUSY 		= { SX_TX_BUSY_GPIO_Port		, SX_TX_BUSY_Pin		};
GPIO_Pin SX_TX_CS 			= { SX_TX_CS_GPIO_Port			, SX_TX_CS_Pin			};

GPIO_Pin SX_RX_IRQ 			= { SX_RX_IRQ_GPIO_Port			, SX_RX_IRQ_Pin			};
GPIO_Pin SX_RX_RST			= { SX_RX_RST_GPIO_Port			, SX_RX_RST_Pin			};
GPIO_Pin SX_RX_BUSY 		= { SX_RX_BUSY_GPIO_Port		, SX_RX_BUSY_Pin		};
GPIO_Pin SX_RX_CS 			= { SX_RX_CS_GPIO_Port			, SX_RX_CS_Pin			};

GPIO_Pin LD_3 				= { LD3_GPIO_Port				, LD3_Pin				};
GPIO_Pin LD_2 				= { LD2_GPIO_Port				, LD2_Pin				};

GPIO_Pin LD_ACTIVE 			= { LD_ACTIVE_GPIO_Port			, LD_ACTIVE_Pin			};
GPIO_Pin LD_USB 			= { LD_USB_GPIO_Port			, LD_USB_Pin			};
GPIO_Pin LD_LED1 			= { LD_LED1_GPIO_Port			, LD_LED1_Pin			};
GPIO_Pin LD_LED2 			= { LD_LED2_GPIO_Port			, LD_LED2_Pin			};
GPIO_Pin LD_LED3 			= { LD_LED3_GPIO_Port			, LD_LED3_Pin			};
GPIO_Pin LD_TX 				= { LED_TX_GPIO_Port			, LED_TX_Pin			};
GPIO_Pin LD_RX 				= { LED_RX_GPIO_Port			, LED_RX_Pin			};
