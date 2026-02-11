/*
 * General flight logic that would apply to every flight computer firmware belongs
 * in the FlightState.c file.
 * All callbacks should be defined in implementation.
 *
 * STATE_IDLE <---------
 *      |               ^
 * STATE_ARMED --> STATE_DISARM
 *      |
 * STATE_BURNING
 *      |
 * STATE_RISING
 *      |
 * STATE_APOGEE
 *      |
 * STATE_DROGUE_DESCENT
 *      |
 * STATE_MAIN_DESCENT
 *      |
 * STATE_LANDED
 */

#include "FlightFSM.h"
#include <stdio.h>

/* Default Callback Implementations */
__attribute__((weak)) void enterIdle(void) { printf("Default: Entering Idle state.\n"); }
__attribute__((weak)) void enterArmed(void) { printf("Default: Entering Armed state.\n"); }
__attribute__((weak)) void enterDisarm(void) { printf("Default: Entering Disarm state.\n"); }
__attribute__((weak)) void enterBurning(void) { printf("Default: Entering Burning state.\n"); }
__attribute__((weak)) void enterRising(void) { printf("Default: Entering Rising state.\n"); }
__attribute__((weak)) void enterApogee(void) { printf("Default: Entering Apogee state.\n"); }
__attribute__((weak)) void enterDrogueDescent(void) { printf("Default: Entering Drogue Descent state.\n"); }
__attribute__((weak)) void enterMainDescent(void) { printf("Default: Entering Main Descent state.\n"); }
__attribute__((weak)) void enterLanded(void) { printf("Default: Entering Landed state.\n"); }

/* Default Transition Functions - Return true if the transition condition is met */
__attribute__((weak)) bool idleExitTransition(void) { return true; }
__attribute__((weak)) bool armedExitTransition(void) { return true; }
__attribute__((weak)) bool disarmExitTransition(void) { return true; }
__attribute__((weak)) bool burningExitTransition(void) { return true; }
__attribute__((weak)) bool risingExitTransition(void) { return true; }
__attribute__((weak)) bool apogeeExitTransition(void) { return true; }
__attribute__((weak)) bool drogueDescentExitTransition(void) { return true; }
__attribute__((weak)) bool mainDescentExitTransition(void) { return true; }
__attribute__((weak)) bool landedExitTransition(void) { return true; }

/*
 * Initialization function for FlightState.
 * Users can override weak functions or modify the structure after initialization.
 */
void initFlightState(FlightState *fs) {
    fs->currentState = STATE_IDLE;

    fs->enterIdle            = enterIdle;
    fs->enterArmed           = enterArmed;
    fs->enterDisarm          = enterDisarm;
    fs->enterBurning         = enterBurning;
    fs->enterRising          = enterRising;
    fs->enterApogee          = enterApogee;
    fs->enterDrogueDescent   = enterDrogueDescent;
    fs->enterMainDescent     = enterMainDescent;
    fs->enterLanded          = enterLanded;

    fs->idleTransition          = idleExitTransition;
    fs->armedTransition         = armedExitTransition;
    fs->disarmTransition        = disarmExitTransition;
    fs->burningTransition       = burningExitTransition;
    fs->risingTransition        = risingExitTransition;
    fs->apogeeTransition        = apogeeExitTransition;
    fs->drogueDescentTransition = drogueDescentExitTransition;
    fs->mainDescentTransition   = mainDescentExitTransition;
    fs->landedTransition        = landedExitTransition;

    // Note: The user can override these weak functions or modify fs after init.
}

// Helper function to get the state name as a string
static const char* stateToString(State state) {
    switch (state) {
        case STATE_IDLE:            return "Idle";
        case STATE_ARMED:           return "Armed";
        case STATE_DISARM:          return "Disarm";
        case STATE_BURNING:         return "Burning";
        case STATE_RISING:          return "Rising";
        case STATE_APOGEE:          return "Apogee";
        case STATE_DROGUE_DESCENT:   return "Drogue Descent";
        case STATE_MAIN_DESCENT:     return "Main Descent";
        case STATE_LANDED:          return "Landed";
        default:                    return "Unknown";
    }
}

/*
 * updateState() checks if the transition function returns true.
 * If so, it moves to the next state in sequence.
 */
void updateState(FlightState *fs) {
    bool shouldTransition = false;
    State newState = fs->currentState;

    switch (fs->currentState) {
        case STATE_IDLE:
            shouldTransition = fs->idleTransition();
            newState = STATE_ARMED;
            break;
        case STATE_ARMED:
            shouldTransition = fs->armedTransition();
            newState = STATE_BURNING;
            break;
        case STATE_DISARM:
            shouldTransition = fs->disarmTransition();
            newState = STATE_IDLE;
            break;
        case STATE_BURNING:
            shouldTransition = fs->burningTransition();
            newState = STATE_RISING;
            break;
        case STATE_RISING:
            shouldTransition = fs->risingTransition();
            newState = STATE_APOGEE;
            break;
        case STATE_APOGEE:
            shouldTransition = fs->apogeeTransition();
            newState = STATE_DROGUE_DESCENT;
            break;
        case STATE_DROGUE_DESCENT:
            shouldTransition = fs->drogueDescentTransition();
            newState = STATE_MAIN_DESCENT;
            break;
        case STATE_MAIN_DESCENT:
            shouldTransition = fs->mainDescentTransition();
            newState = STATE_LANDED;
            break;
        case STATE_LANDED:
            shouldTransition = fs->landedTransition();
            newState = STATE_DISARM;
            break;
    }

    if (shouldTransition && newState != fs->currentState) {
        printf("Transitioning from %s to %s\n", stateToString(fs->currentState), stateToString(newState));
        fs->currentState = newState;

        switch (newState) {
            case STATE_IDLE:
                fs->enterIdle();
                break;
            case STATE_ARMED:
                fs->enterArmed();
                break;
            case STATE_DISARM:
                fs->enterDisarm();
                break;
            case STATE_BURNING:
                fs->enterBurning();
                break;
            case STATE_RISING:
                fs->enterRising();
                break;
            case STATE_APOGEE:
                fs->enterApogee();
                break;
            case STATE_DROGUE_DESCENT:
                fs->enterDrogueDescent();
                break;
            case STATE_MAIN_DESCENT:
                fs->enterMainDescent();
                break;
            case STATE_LANDED:
                fs->enterLanded();
                break;
        }
    }
}
