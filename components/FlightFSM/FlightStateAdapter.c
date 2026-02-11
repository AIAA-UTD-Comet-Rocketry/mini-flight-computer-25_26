/*
 * General flight logic that would apply to every flight computer firmware belongs
 * in the FlightState.c file.
 * This file is for hardware and application dependent code, such as redefining
 * the weak transition functions so they interface with sensor data streams.
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


/* Extern data streams to include */
extern float gAltitude;
extern float gTotalAcc;
extern float gDegOffVert;
extern volatile uint32_t uwTick;


/* Extern Functions */
extern void M95p32_DebugPrint(const uint8_t*, uint32_t size);


/* Private Variables */
float prevAlt = 0;
uint32_t transDelay = UINT32_MAX;


/* Constant Defines */
#define LAUNCH_ACC_THRESH_G     1.6     // Threshold acceleration to indicate launch
#define LAUNCH_DEBOUNCE_MS      100     // Burn for 0.1 sec before trigger
#define BURNOUT_ACC_THRESH_G    1.3     // Lower acceleration bound to indicate burn end
#define MAX_BURN_TIME_MS        6000    // Burn state timeout to catch error
#define APOGEE_SAMPLE_PERIOD_MS 500     // Descent detection altitude sample compare period
#define MAIN_DEPLOY_ALTITUDE    1000.0  // End of drogue descent (ft)
#define LANDED_SAMPLE_PERIOD_MS 10000   // Landed detection altitude sample compare period


/* Redefined Callback Implementations, Called when new state is entered */
void enterIdle(void) { uint32_t nextprint[2] = {1, uwTick}; M95p32_DebugPrint((uint8_t*)nextprint, 8U); }
void enterArmed(void) { uint32_t nextprint[2] = {2, uwTick}; M95p32_DebugPrint((uint8_t*)nextprint, 8U); }
void enterDisarm(void) { uint32_t nextprint[2] = {3, uwTick}; M95p32_DebugPrint((uint8_t*)nextprint, 8U); }
void enterBurning(void) { uint32_t nextprint[2] = {4, uwTick}; M95p32_DebugPrint((uint8_t*)nextprint, 8U); }
void enterRising(void) { uint32_t nextprint[2] = {5, uwTick}; M95p32_DebugPrint((uint8_t*)nextprint, 8U); }
void enterApogee(void) { uint32_t nextprint[2] = {6, uwTick}; M95p32_DebugPrint((uint8_t*)nextprint, 8U); }
void enterDrogueDescent(void) { uint32_t nextprint[2] = {7, uwTick}; M95p32_DebugPrint((uint8_t*)nextprint, 8U); }
void enterMainDescent(void) { uint32_t nextprint[2] = {8, uwTick}; M95p32_DebugPrint((uint8_t*)nextprint, 8U); }
void enterLanded(void) { uint32_t nextprint[2] = {9, uwTick}; M95p32_DebugPrint((uint8_t*)nextprint, 8U); }

/*
 * Redefined Transition Functions, true moves to next.
 * Each state transition sets transDelay for next state, not a consistent value.
 * These are not the most efficient blocks, should only get checked
 * each 10ms and expect sensor misses during callback checks.
 */
bool idleExitTransition(void)
{
  transDelay = UINT32_MAX; //armed state requires large delay time
  return true;
}

bool armedExitTransition(void)
{
  // Look for launch
  if(gTotalAcc > LAUNCH_ACC_THRESH_G)
  {
    if(uwTick > transDelay)
    {
      transDelay = uwTick; // Burn state requires beginning of burn timestamp (i.e NOW!)
      return true;
    }
    else if (transDelay == UINT32_MAX) //first high g reading, start timer
    {
      transDelay = uwTick + LAUNCH_DEBOUNCE_MS; // wait debounce time before trigger
    }
  }
  else
  {
    transDelay = UINT32_MAX; // one bad reading resets debounce timer
  }

  return false;
}

bool disarmExitTransition(void) { return false; } // disarm does not support recovery yet

bool burningExitTransition(void)
{
  /*
   * Look for burnout or timeout.
   * Not robust, highly sensitive since as false negatives are high risk.
   * Better to move on too early than not at all.
   */
  if(gTotalAcc < BURNOUT_ACC_THRESH_G || (uwTick - transDelay) > MAX_BURN_TIME_MS)
  {
    transDelay = uwTick; // Rising state requires end of burn timestamp (i.e NOW!)
    return true;
  }
  return false;
}

bool risingExitTransition(void)
{
  // Look for apogee
  if(uwTick >= transDelay)
  {
    if(gAltitude < prevAlt)
    {
      transDelay = uwTick;
      return true; // no longer rising
    }

    // still rising, set-up next altitude comparison
    prevAlt = gAltitude;
    transDelay = uwTick + APOGEE_SAMPLE_PERIOD_MS; // Separate comparison samples
  }
  return false;
}

//todo ejection attempt should be attempted multiple times but briefly enough to avoid brownout.
bool apogeeExitTransition(void) { return true; }

bool drogueDescentExitTransition(void)
{
  if(gAltitude < MAIN_DEPLOY_ALTITUDE)
  {
    transDelay = uwTick;
    return true; // no longer rising
  }
  return false;
}

bool mainDescentExitTransition(void)
{
  // Look for apogee
  if(uwTick >= transDelay)
  {
    if(gAltitude >= prevAlt) //todo updrafts may trigger
    {
      transDelay = uwTick;
      return true; // no longer falling
    }

    // still falling, set-up next altitude comparison
    prevAlt = gAltitude;
    transDelay = uwTick + LANDED_SAMPLE_PERIOD_MS; // Separate comparison samples
  }
  return false;
}

bool landedExitTransition(void) { return false; } // not re-launch prior to reboot
