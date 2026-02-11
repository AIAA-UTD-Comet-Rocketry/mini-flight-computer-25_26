#ifndef FLIGHT_STATE_H
#define FLIGHT_STATE_H

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

// Define the possible flight states.
typedef enum {
    STATE_IDLE,
    STATE_ARMED,
    STATE_DISARM,
    STATE_BURNING,
    STATE_RISING,
    STATE_APOGEE,
    STATE_DROGUE_DESCENT,
    STATE_MAIN_DESCENT,
    STATE_LANDED
} State;

// Define function pointer type for state callbacks.
typedef void (*StateCallback)(void);

// Define function pointer type for state transition logic.
typedef bool (*StateTransitionFunc)(void);

// FlightState structure holds the current state, state callbacks, and transition functions.
typedef struct {
    State currentState;

    // Callbacks for entering each state
    StateCallback enterIdle;
    StateCallback enterArmed;
    StateCallback enterDisarm;
    StateCallback enterBurning;
    StateCallback enterRising;
    StateCallback enterApogee;
    StateCallback enterDrogueDescent;
    StateCallback enterMainDescent;
    StateCallback enterLanded;

    // Transition functions for each state
    StateTransitionFunc idleTransition;
    StateTransitionFunc armedTransition;
    StateTransitionFunc disarmTransition;
    StateTransitionFunc burningTransition;
    StateTransitionFunc risingTransition;
    StateTransitionFunc apogeeTransition;
    StateTransitionFunc drogueDescentTransition;
    StateTransitionFunc mainDescentTransition;
    StateTransitionFunc landedTransition;
} FlightState;

// Initialization function
void initFlightState(FlightState *fs);

// Updates the flight state based on set transition conditions and
// calls the callback functions if a new state is entered.
void updateState(FlightState *fs);

#endif // FLIGHT_STATE_H
