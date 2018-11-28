#include "gpio_util.h"
#include <stdbool.h>
#include "tim.h"
#include "KickChip.h"

#define Timestep 20

// Init variables and call kick_Callback(), which will keep calling itself.
void kick_Init(kickchip *kick){
    kick->Kstate = kick_Charging;

    set_pin(Kick_pin, GPIO_PIN_RESET); // Kick off
    set_pin(Chip_pin, GPIO_PIN_RESET); // Chip off
    set_pin(Charge_pin, GPIO_PIN_SET); // kick_Charging on
    kick->Charge = false;
    kick_Callback(kick);
}

// deactivates the kicker board to sleep mode
void kick_DeInit(kickchip *kick){
    HAL_TIM_Base_Stop(&htim13);
    kick->Kstate = kick_Idle;
    set_pin(Kick_pin, GPIO_PIN_RESET);   // Kick off
    set_pin(Chip_pin, GPIO_PIN_RESET);   // Chip off
    set_pin(Charge_pin, GPIO_PIN_RESET); // kick_Charging off
}

/*
 *  Initiates the kick_Shoot of the robot at a percentage of the max power.
 *  Will only do this if in state kick_Ready.
 *  Sets a timer for kick_Callback() to stop the kick. The power of the kick depends on the time interval between this function and the callback.
 */
kick_states kick_Shoot(kickchip *kick, int percentage, bool kick_chip){
    // check if output is in range
    if (percentage < 1) {
        return kick->Kstate = kick_Ready;   // kicking is not worth it
    } else if (percentage > 100){
        percentage = 100;
    }

    // check if can kick
    if (kick->Kstate == kick_Ready)
    {
        kick->Kstate = kick_Kicking;                            // Block kick_Charging
        set_pin(Charge_pin, GPIO_PIN_RESET);                    // Disable kick_Charging
        set_pin(kick_chip ? Kick_pin : Chip_pin, GPIO_PIN_SET); // Kick/Chip on
                                                                //		uprintf("kick_Kicking\n\r");

        HAL_TIM_Base_Stop(&htim13); // Stop timer
        __HAL_TIM_CLEAR_IT(&htim13, TIM_IT_UPDATE);
        __HAL_TIM_SET_COUNTER(&htim13, 0);                                   // Clear timer
        __HAL_TIM_SET_AUTORELOAD(&htim13, percentage * (kick_chip ? 4 : 6)); // Set kick time
        HAL_TIM_Base_Start_IT(&htim13);                                      // Start timer
    }
    return kick->Kstate;
}

/*
 * Handles all the callbacks for kick_Kicking, chipping and kick_Charging
 * This function keeps setting a new timer to call itself.
 */
void kick_Callback(kickchip *kick){
    HAL_TIM_Base_Stop(&htim13); // Stop the timer
    __HAL_TIM_SET_COUNTER(&htim13, 0);

    int Callback_time = 0; // local variable for setting timer duration
    if (kick->Kstate == kick_Kicking){
        set_pin(Kick_pin, GPIO_PIN_RESET); // Kick off
        set_pin(Chip_pin, GPIO_PIN_RESET); // Chip off
        kick->Kstate = kick_Charging;
        set_pin(Charge_pin, GPIO_PIN_SET); // Turn kick_Charging on
        Callback_time = 100 * Timestep;    // Set timer to 100ms
    }
    else if (kick->Kstate == kick_Charging){
        if (!read_pin(Charge_done_pin)){ // stay in charging if not done yet
            set_pin(Charge_pin, GPIO_PIN_RESET); // Turn kick_Charging off
            kick->Kstate = kick_Ready;           // Go to kick_Ready state
            kick->Charge = false;
        }
        Callback_time = 100 * Timestep; // Set timer to 100ms
    }
    else if (kick->Kstate == kick_Ready){
        set_pin(Charge_pin, kick->Charge); // Turn kick_Charging off
        Callback_time = 1000 * Timestep;   // Set timer to 1 second
        kick->Charge = !kick->Charge;
    }

    __HAL_TIM_CLEAR_IT(&htim13, TIM_IT_UPDATE);       // Clear timer
    __HAL_TIM_SET_AUTORELOAD(&htim13, Callback_time); // Set callback time to defined value
    HAL_TIM_Base_Start_IT(&htim13);
}

void kick_Stateprint()
{
    //	uprintf("Block = [%d]\n\r", kick_state);
}

void Kick_handle_command(kickchip *kick, char* input, int len){
    if(!memcmp(input, "kick" , strlen("kick"))){
        // kick
        int percentage = strtol(input + 1 + strlen("kick"), NULL, 10);
		kick_Shoot(kick, percentage ,KICK);
	}
    else if(!memcmp(input, "chip" , strlen("chip"))){
        // chip
        int percentage = strtol(input + 1 + strlen("chip"), NULL, 10);
		kick_Shoot(kick, percentage, CHIP);
    }
}