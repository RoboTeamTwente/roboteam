/*
 * gpio_util.c
 *
 *  Created on: 8 April 2019
 *      Author: Cas Doornkamp
 */

#include "gpio_util.h"
#include "main.h"

#define DEFINE_PIN(label,pin_name) GPIO_Pin label = {pin_name##_GPIO_Port, pin_name##_Pin};
// List known GPIO pins

// Kick/Chip
GPIO_Pin Kick_pin 			= { Kick_GPIO_Port			, Kick_Pin			};
GPIO_Pin Chip_pin 			= { Chip_GPIO_Port			, Chip_Pin			};
GPIO_Pin Charge_pin 		= { Charge_GPIO_Port		, Charge_Pin		};
GPIO_Pin Charge_done_pin 	= { Charge_done_GPIO_Port	, Charge_done_Pin	};

// Wheels PWM
GPIO_Pin RB_PWM_pin			= { RB_PWM_GPIO_Port		, RB_PWM_Pin		};
GPIO_Pin RF_PWM_pin			= { RF_PWM_GPIO_Port		, RF_PWM_Pin		};
GPIO_Pin LB_PWM_pin			= { LB_PWM_GPIO_Port		, LB_PWM_Pin		};
GPIO_Pin LF_PWM_pin			= { LF_PWM_GPIO_Port		, LF_PWM_Pin		};

// Wheels DIR
GPIO_Pin RB_DIR_pin			= { RB_FR_GPIO_Port			, RB_FR_Pin			};
GPIO_Pin RF_DIR_pin			= { RF_FR_GPIO_Port			, RF_FR_Pin			};
GPIO_Pin LB_DIR_pin			= { LB_FR_GPIO_Port			, LB_FR_Pin			};
GPIO_Pin LF_DIR_pin			= { LF_FR_GPIO_Port			, LF_FR_Pin			};

// Wheels brake
GPIO_Pin RB_Brake_pin		= { RB_BRK_GPIO_Port		, RB_BRK_Pin		};
GPIO_Pin RF_Brake_pin		= { RF_BRK_GPIO_Port		, RF_BRK_Pin		};
GPIO_Pin LB_Brake_pin		= { LB_BRK_GPIO_Port		, LB_BRK_Pin		};
GPIO_Pin LF_Brake_pin		= { LF_BRK_GPIO_Port		, LF_BRK_Pin		};

// Wheels ENC
GPIO_Pin RB_ENC_A_pin		= { RB_CHA_GPIO_Port		, RB_CHA_Pin		};
GPIO_Pin RB_ENC_B_pin		= { RB_CHB_GPIO_Port		, RB_CHB_Pin		};
GPIO_Pin RF_ENC_A_pin		= { RF_CHA_GPIO_Port		, RF_CHA_Pin		};
GPIO_Pin RF_ENC_B_pin		= { RF_CHB_GPIO_Port		, RF_CHB_Pin		};
GPIO_Pin LB_ENC_A_pin		= { LB_CHA_GPIO_Port		, LB_CHA_Pin		};
GPIO_Pin LB_ENC_B_pin		= { LB_CHB_GPIO_Port		, LB_CHB_Pin		};
GPIO_Pin LF_ENC_A_pin		= { LF_CHA_GPIO_Port		, LF_CHA_Pin		};
GPIO_Pin LF_ENC_B_pin		= { LF_CHB_GPIO_Port		, LF_CHB_Pin		};

// Dribbler
GPIO_Pin Dribbler_PWM_pin	= { PWM_Dribbler_GPIO_Port	, PWM_Dribbler_Pin	};

// LEDs
GPIO_Pin LED0_pin			= { LD0_GPIO_Port			, LD0_Pin			};
GPIO_Pin LED1_pin			= { LD1_GPIO_Port			, LD1_Pin			};
GPIO_Pin LED2_pin			= { LD2_GPIO_Port			, LD2_Pin			};
GPIO_Pin LED3_pin			= { LD3_GPIO_Port			, LD3_Pin			};
GPIO_Pin LED4_pin			= { LD4_GPIO_Port			, LD4_Pin			};
GPIO_Pin LED5_pin			= { LD5_GPIO_Port			, LD5_Pin			};
GPIO_Pin LED6_pin			= { LD6_GPIO_Port			, LD6_Pin			};


// ID select
GPIO_Pin ID0_pin			= { ID0_GPIO_Port			, ID0_Pin			};
GPIO_Pin ID1_pin			= { ID1_GPIO_Port			, ID1_Pin			};
GPIO_Pin ID2_pin			= { ID2_GPIO_Port			, ID2_Pin			};
GPIO_Pin ID3_pin			= { ID3_GPIO_Port			, ID3_Pin			};

// Feature Select
DEFINE_PIN(FT0_pin, FT0)
DEFINE_PIN(FT1_pin, FT1)
DEFINE_PIN(FT2_pin, FT2)
DEFINE_PIN(FT3_pin, FT3)

// MTi
GPIO_Pin MTi_RST_pin 		= { XSENS_RST_GPIO_Port		, XSENS_RST_Pin		};
GPIO_Pin MTi_IRQ_pin 		= { XSENS_IRQ_GPIO_Port		, XSENS_IRQ_Pin		};
GPIO_Pin MTi_NSS_pin		= { SPI1_NSS_GPIO_Port		, SPI1_NSS_Pin		};

// Wireless
GPIO_Pin SX_IRQ_pin			= { SPI4_IRQ_GPIO_Port 		, SPI4_IRQ_Pin		};
GPIO_Pin SX_RST_pin			= { SPI4_RST_GPIO_Port 		, SPI4_RST_Pin		};
GPIO_Pin SX_NSS_pin			= { SPI4_NSS_GPIO_Port 		, SPI4_NSS_Pin		};
GPIO_Pin SX_BUSY_pin		= { SPI4_BUSY_GPIO_Port 	, SPI4_BUSY_Pin		};

// Ballsensor
GPIO_Pin BS_RST_pin			= { BS_RST_GPIO_Port 		, BS_RST_Pin		};
GPIO_Pin BS_IRQ_pin			= { BS_IRQ_GPIO_Port 		, BS_IRQ_Pin		};

// Battery
DEFINE_PIN(BAT_SDN_pin, Batt_SDN)
DEFINE_PIN(BAT_KILL_pin, Batt_kill)
DEFINE_PIN(BAT_IRQ_pin, Batt_IRQ)

// SD
DEFINE_PIN(SD_CD_pin, SD_MMC1_CD)

// Interface
DEFINE_PIN(INT_EN_pin, Interface_EN)
DEFINE_PIN(INT_ENneg_pin, Interface_ENnegative)