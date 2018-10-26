/*
 * gpio_util.h
 *
 *  Created on: 26 okt. 2018
 *      Author: Cas Doornkamp
 */

#ifndef UTILS_GPIO_UTIL_H_
#define UTILS_GPIO_UTIL_H_

#include "gpio.h"
#include "stdbool.h"

// abstract a GPIO pin combination to a struct
typedef struct GPIO_Pin{
	GPIO_TypeDef * PORT;
	uint16_t PIN;
} GPIO_Pin;

// List known GPIO pins

// Kick/Chip
GPIO_Pin Kick_pin 			= { Kick_GPIO_Port			, Kick_Pin			};
GPIO_Pin Chip_pin 			= { Chip_GPIO_Port			, Chip_Pin			};
GPIO_Pin Charge_pin 		= { Charge_GPIO_Port		, Charge_Pin		};

// Geneva
GPIO_Pin Geneva_PWM_pin		= { Geneva_PWM_GPIO_Port	, Geneva_PWM_Pin	};
GPIO_Pin Geneva_DIR_A_pin 	= { Geneva_dir_A_GPIO_Port	, Geneva_dir_A_Pin	};
GPIO_Pin Geneva_DIR_B_pin 	= { Geneva_dir_B_GPIO_Port	, Geneva_dir_B_Pin	};
GPIO_Pin Geneva_ENC_A_pin	= { Geneva_CHA_GPIO_Port	, Geneva_CHA_Pin	};
GPIO_Pin Geneva_ENC_B_pin	= { Geneva_CHB_GPIO_Port	, Geneva_CHB_Pin	};

// Wheels PWM
GPIO_Pin RB_PWM_pin			= { PWM_RB_GPIO_Port		, PWM_RB_Pin		};
GPIO_Pin RF_PWM_pin			= { PWM_RF_GPIO_Port		, PWM_RF_Pin		};
GPIO_Pin LB_PWM_pin			= { PWM_LB_GPIO_Port		, PWM_LB_Pin		};
GPIO_Pin LF_PWM_pin			= { PWM_LF_GPIO_Port		, PWM_LF_Pin		};

// Wheels DIR
GPIO_Pin RB_DIR_pin			= { FR_RB_GPIO_Port			, FR_RB_Pin			};
GPIO_Pin RF_DIR_pin			= { FR_RF_GPIO_Port			, FR_RF_Pin			};
GPIO_Pin LB_DIR_pin			= { FR_LB_GPIO_Port			, FR_LB_Pin			};
GPIO_Pin LF_DIR_pin			= { FR_LF_GPIO_Port			, FR_LF_Pin			};

// Wheels ENC
GPIO_Pin RB_ENC_A_pin		= { CHA_RB_GPIO_Port		, CHA_RB_Pin		};
GPIO_Pin RF_ENC_A_pin		= { CHA_RF_GPIO_Port		, CHA_RF_Pin		};
GPIO_Pin LB_ENC_A_pin		= { CHA_LB_GPIO_Port		, CHA_LB_Pin		};
GPIO_Pin LF_ENC_A_pin		= { CHA_LF_GPIO_Port		, CHA_LF_Pin		};
GPIO_Pin RB_ENC_B_pin		= { CHB_RB_GPIO_Port		, CHB_RB_Pin		};
GPIO_Pin RF_ENC_B_pin		= { CHB_RF_GPIO_Port		, CHB_RF_Pin		};
GPIO_Pin LB_ENC_B_pin		= { CHB_LB_GPIO_Port		, CHB_LB_Pin		};
GPIO_Pin LF_ENC_B_pin		= { CHB_LF_GPIO_Port		, CHB_LF_Pin		};

// Battery
GPIO_Pin Bat_pin			= { empty_battery_GPIO_Port	, empty_battery_Pin	};

// Dribbler
GPIO_Pin Dribbler_PWM_pin	= { PWM_Dribbler_GPIO_Port	, PWM_Dribbler_Pin	};

// LEDs
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

// Frequency select
GPIO_Pin FRQ_sel			= { SW_freq_GPIO_Port		, SW_freq_Pin		};


// Utility functions

// Set a GPIO Pin
inline void set_pin(GPIO_Pin p, bool value)
{
	HAL_GPIO_WritePin(p.PORT, p.PIN, value);
}

// Read a GPIO Pin
inline GPIO_PinState read_pin(GPIO_Pin p)
{
	return HAL_GPIO_ReadPin(p.PORT, p.PIN);
}

// Toggle a GPIO Pin
inline void toggle_pin(GPIO_Pin p)
{
	HAL_GPIO_TogglePin(p.PORT, p.PIN);
}


#endif /* UTILS_GPIO_UTIL_H_ */
