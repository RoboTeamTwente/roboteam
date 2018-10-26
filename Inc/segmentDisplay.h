#ifndef SEGDIS_H
#define SEGDIS_H
/*
Author: Antonio
Date: 14/10/2018

Description: Allows to use a segment display with only 3 pins, a timer and power.

Instructions:
 1) Initialize with SegmentDisplayInit(). This will already load 0.0.
 2) Load number you want to show with loadNumber(). This does not show antying.
 3) Run displayNextDigit() after the timer. Each time it's ran,
 a new display is updated (so don't do it too slowly)

Extra functions:
 - reset() is interesting if you want to map a reset button.
 - increaseResolution()/decreaseResolution() are almost necessary
 when working with doubles (since it lets you check how big the number is)

GPIO Pins:
 D1 (Data)
 CLK1 (Displays)
 CLK2 (Segments)

Notes:
 RCLK: Displays
 SCLK: Loads
 CLK2 enables: RCLKs, SCLKd, RCLKd
 CLK1 enables: SCLKs
 Display Numbering: 0 is the leftmost, n is the rightmost

 Due to the computer limitations, numbers such as 5.1 are represented as 5.099999999. This does not have an easy fix, so play around with the resolution. Note that this only happens with doubles.
 */

#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#define nsegments 8 // NEVER change!
#define ndisplays 4 // Change
#define actualSegments 8 // Change

////////////////////////////////////////////////// GLOBAL VARIABLES
// Variables
int resolution; // Where to put the dot
double lastInput = 0.0;
int cdisplay = 0; // Current display
int* numVector = NULL; // Vector holding all of the numbers to be displayed

// Booleans
bool theresDot = false; // If there is a dot
bool noDot = false;

////////////////////////////////////////////////// FUNCTION DECLARATIONS
void loadNumber (double number); // Stores the input number. This will be the number to be shown.
void SegmentDisplayInit (); // Initialization function
void reset_SegmentDisplay(); // Loads 0.0.
void loadSegments (int binary); // Loads the segments onto the first shift register (CLK1)
void showInDisplay (); // Loads appropiate value into second shift register to show onto display (CLK2)
void displayNextDigit (); // Refreshes the segment display, showing the next number
int binaryCode (int number, bool isDot); // Returns the binary equivalent of the input number
int * convertToArray (double number); // Converts a number to an array of digits
void increaseResolution (); // Moves the position of the dot left
void decreaseResolution (); // Moves the position of the dot right


/////////////////////////////////////////////////////////////////// move to .c file
////////////////////////////////////////////////// FUNCTIONS
// Load the number
void loadNumber (double number){
    lastInput = number; // Put the number as the last input
    numVector = convertToArray(lastInput); // Converts the number to a vector
    return;
}


// Takes care of CLK1 related tasks
void loadSegments (int binary) {
    int cycle = 0b00000001; // Used for XORing
    int output;
    // Load all segments
    for (int i =0; i<nsegments; i++){
        output = (cycle&binary)>>i;
           HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, output); // Set data line
           HAL_GPIO_WritePin(CLK1_GPIO_Port, CLK1_Pin, 1); // Load value
           HAL_GPIO_WritePin(CLK1_GPIO_Port, CLK1_Pin, 0); // Set clock low
        
        // printf("D1 Segment: %i \n",output);
        cycle=cycle<<1; // Move to the next segment

        // Note. Remember that you load the last one first.
    }
}


// Shows the corresponding number in the next display
// (Note. This is the function that should be called with a timer)
void showInDisplay (){
    // printf("Current/Load Onto Display: %i \n", cdisplay);

    if (cdisplay==0){ // Go back if we run out of displays
        HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, 1); // Set data line
        // printf("D1 Display: %i \n\n", 1);
    }
    else{
        HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, 0); // Set data line to 0
        // printf("D1 Display: %i \n\n", 0);
    }

    // Normal operation
       HAL_GPIO_WritePin(CLK2_GPIO_Port, CLK2_Pin, 1); // Load value
       HAL_GPIO_WritePin(CLK2_GPIO_Port, CLK2_Pin, 0); // Set clock low
}


// Initialization function
void SegmentDisplayInit (){
    resolution = floor(ndisplays / 2); // E.g. if there are 4, then resolution will be 2 (00.00). If there are 3, resolution will be 1 (00.0)

    // In case of a 7-segment
    if (actualSegments <= 7){
        resolution=0;
        noDot = true;
    }

    reset_SegmentDisplay(); // Resets the seven segment display

}


// Converts a double into an array of integers. The dot can be determined from the resolution.
int * convertToArray (double number) {
    int intNum = floor(number*pow(10,resolution)); // Obtain integer number based on the resolution
    static int outputArray [4]; // Vector to be output
    int digit = 0;
    int power; // Divide by a power of 10

    for (int i = (ndisplays-1); i>=0; i--){
        power = ceil(pow(10, i)); // Determine the power to be raised to
        digit = intNum/power; // Divide by 10^i

        // If it's the first division, obtain only the first number
        if (i == (ndisplays-1)){
            digit=digit%10;
        }

        intNum = intNum % power; // Find the rest of the number
        outputArray[(ndisplays-1)-i] = digit; // Store the digit
        // (Note. If the number is 3441, 3 is stored at i=0 and 1 is stored at i=3

    }

    return outputArray;
}

// Convert to binary code for segment display
int binaryCode (int number, bool isDot){
    // In case there is a dot
    int dotCode;
    if (isDot == true) {
        dotCode = 0b00000001;
    }
    else {
        dotCode = 0b00000000;
    }

    // Code for each number
    switch (number){
        case 0:
            return (dotCode | 0b11111100) ^ 0b11111111;
        case 1:
            return (dotCode | 0b01100000) ^ 0b11111111;
        case 2:
            return (dotCode | 0b11011010) ^ 0b11111111;
        case 3:
            return (dotCode | 0b11110010) ^ 0b11111111;
        case 4:
            return (dotCode | 0b01100110) ^ 0b11111111;
        case 5:
            return (dotCode | 0b10110110) ^ 0b11111111;
        case 6:
            return (dotCode | 0b10111110) ^ 0b11111111;
        case 7:
            return (dotCode | 0b11100000) ^ 0b11111111;
        case 8:
            return (dotCode | 0b11111110) ^ 0b11111111;
        case 9:
            return (dotCode | 0b11100110) ^ 0b11111111;
        default:
            return (dotCode | 0b10011110) ^ 0b11111111;
    }
}

void reset_SegmentDisplay () {
    cdisplay = 0; // Current display
    lastInput = 0; // Sets the last input to 0
    loadNumber(lastInput); // Loads 0
}


// Shows the next digit
// Note. Call at a certain frequency with a timer.
void displayNextDigit (){
    // Check if there is a dot based on the resolution
    if ((ndisplays-resolution-1)==(cdisplay)){
        theresDot=true;
    }
    else{
        theresDot=false;
    }

    showInDisplay(); // Shows and moves display.

    // Load segments and onto display
    loadSegments(binaryCode(*(numVector+cdisplay), theresDot)); // Converts the number to bits and loads it onto the shift register.
    

    // If the current display is the last one, change to the first one.
    if (cdisplay>= (ndisplays-1)){
        cdisplay=0;
    }
    else{
        cdisplay++;
    }
}


// Resolution for input button
void increaseResolution (){
    if (noDot==false && resolution<(ndisplays-1)){
        resolution++;
        loadNumber(lastInput);
    }
}

void decreaseResolution () {
    if (noDot==false && resolution>0) {
        resolution--;
        loadNumber(lastInput);
    }
}


#endif