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

 /*
 IREC Rocket Sims Info:
  - max accel = 482 ft/s^2 
  - max velocity = 1167 ft/s 
  - time to apogee = 24 seconds
  - expected apogee: 10500ft
  - expected burn time = 3.33 seconds
  - drogue descent rate = 55 ft/s to 48 ft/s down
  - main descent rate = 15ft/s
 */

#include "FlightFSM.h"
#include "esp_log.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Extern pyro task handle (defined in main.c) */
extern TaskHandle_t xPyroTaskHandle;

/* Pyro channel bit masks (mirrors pyro_index_t in BSP.h) */
#define PYRO_APO1_BIT   (1 << 0)  // apo1_channel - 35g CO2
#define PYRO_APO2_BIT   (1 << 1)  // apo2_channel - 45g CO2
#define PYRO_MAIN1_BIT  (1 << 2)  // main1_channel - TD2 ejection
#define PYRO_MAIN2_BIT  (1 << 3)  // main2_channel - TD2 ejection

/* Extern data streams (defined in sensor_mgr.c) */
extern float gAltitude;
extern float gTotalAcc;
extern float gDegOffVert;

/* Extern tick function (defined in sensor_mgr.c) */
extern uint32_t sensor_get_tick_ms(void);

static const char *TAG = "FSM";

/* Private Variables */
float prevAlt = 0;
float prevVel = 0;
uint8_t apogeeConfirmed = 0;
uint8_t descendingSamples = 0;
uint8_t landedSamples = 0;
uint32_t transDelay = UINT32_MAX;


/* Constant Defines */
#define LAUNCH_ACC_THRESH_G     1.6     // Threshold acceleration to indicate launch
#define LAUNCH_DEBOUNCE_MS      100     // Burn for 0.1 sec before trigger
#define BURNOUT_ACC_THRESH_G    1.3     // Lower acceleration bound to indicate burn end
#define MAX_BURN_TIME_MS        4000    // Burn state timeout to catch error
#define APOGEE_SAMPLE_PERIOD_MS 500     // Descent detection altitude sample compare period
#define APOGEE_MIN_THRESHOLD    5000  // Min apogee altitude needed for drogue to be deployed (use 5000 for flight)
#define MAIN_DEPLOY_ALTITUDE    1000  // End of drogue descent (ft)
//#define MAIN_DEPLOY_ACC_THRESH_G   10   // Threshold acceleration for failsafe main deployment
#define LANDED_SAMPLE_PERIOD_MS 10000   // Landed detection altitude sample compare period
#define LANDED_ALT_THRESHOLD	  1.0		// 1ft change in altitude to be considered landed
#define LANDED_SAMPLES_REQ		  1		// 1 consecutive stable samples
#define APOGEE_COOLDOWN_MS      100     // 0.1s stable descent required
#define LANDED_COOLDOWN_MS		  10000	// 10 sec
#define MACH_LOCK_TIME_MS       4000 // ms - must cover transonic phase (~Mach 0.8-1.2)
#define APOGEE_CONSEC_SAMPLES   3    // consecutive descending samples before apogee confirm


/* Redefined Callback Implementations, Called when new state is entered */
void enterIdle(void)           { ESP_LOGW(TAG, "Entered IDLE at %lu ms", sensor_get_tick_ms()); }
void enterArmed(void)          { ESP_LOGW(TAG, "Entered ARMED at %lu ms", sensor_get_tick_ms()); }
void enterDisarm(void)         { ESP_LOGW(TAG, "Entered DISARM at %lu ms", sensor_get_tick_ms()); }
void enterBurning(void)        { ESP_LOGW(TAG, "Entered BURNING at %lu ms", sensor_get_tick_ms()); }
void enterRising(void)         { ESP_LOGW(TAG, "Entered RISING at %lu ms", sensor_get_tick_ms()); }
void enterApogee(void)         { ESP_LOGW(TAG, "Entered APOGEE at %lu ms", sensor_get_tick_ms()); }
void enterDrogueDescent(void)  { ESP_LOGW(TAG, "Entered DROGUE_DESCENT at %lu ms", sensor_get_tick_ms()); }
void enterMainDescent(void)    { ESP_LOGW(TAG, "Entered MAIN_DESCENT at %lu ms", sensor_get_tick_ms()); }
void enterLanded(void)         { ESP_LOGW(TAG, "Entered LANDED at %lu ms", sensor_get_tick_ms()); }

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
  uint32_t uwTick = sensor_get_tick_ms();
  // Look for launch
  if(gTotalAcc > LAUNCH_ACC_THRESH_G)
  {
    if(uwTick > transDelay)
    {
      transDelay = uwTick; // Burn state requires beginning of burn timestamp (i.e uwTick!)
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

bool disarmExitTransition(void) 
{ return false; } // disarm does not support recovery yet

bool burningExitTransition(void)
{
  uint32_t uwTick = sensor_get_tick_ms();
  /*
   * Look for burnout or timeout.
   * Not robust, highly sensitive since as false negatives are high risk.
   * Better to move on too early than not at all.
   */
  if(gTotalAcc < BURNOUT_ACC_THRESH_G || (uwTick - transDelay) > MAX_BURN_TIME_MS)
  {
    transDelay = uwTick + MACH_LOCK_TIME_MS; // Rising state requires end of burn timestamp (i.e uwTick!)
    return true;
  }
  return false;
}

bool risingExitTransition(void)
{
  uint32_t uwTick = sensor_get_tick_ms();
  // Look for apogee — require consecutive descending samples to filter transonic noise
  if(uwTick >= transDelay)
  {
    if(gAltitude < prevAlt) // Check for negative altitude -> rocket falling
    {
      descendingSamples++;
      if(descendingSamples >= APOGEE_CONSEC_SAMPLES) // Check multiple samples
      {
        descendingSamples = 0;
        transDelay = uwTick;
        return true;
      }
    }
    else
    {
      descendingSamples = 0; // reset sample back to 0 if positive altitude
    }

    prevAlt = gAltitude;
    transDelay = uwTick + APOGEE_SAMPLE_PERIOD_MS;
  }
  return false;
}

//In prog: ejection attempt should be attempted multiple times but briefly enough to avoid brownout.
bool apogeeExitTransition(void)
{
  uint32_t uwTick = sensor_get_tick_ms();
	float currentVel = (gAltitude - prevAlt) / (APOGEE_SAMPLE_PERIOD_MS * 0.001f); // ft / s

	// Apogee detection logic
	if (uwTick >= transDelay) {
		if (gAltitude > APOGEE_MIN_THRESHOLD && currentVel <= 0 && !apogeeConfirmed)
		{
			// Negative velocity confirmed — start cooldown before firing
			transDelay = uwTick + APOGEE_COOLDOWN_MS;
			apogeeConfirmed = 1;
		}
		else if (apogeeConfirmed && uwTick > transDelay)
		{
			// Cooldown elapsed — fire drogues
			if (xPyroTaskHandle != NULL)
			{
				xTaskNotify(xPyroTaskHandle, PYRO_APO1_BIT, eSetBits);
				xTaskNotify(xPyroTaskHandle, PYRO_APO2_BIT, eSetBits);
			}
			prevAlt = gAltitude; // reset for descent tracking
			return true;
		}
		else
		{
			// Not yet confirmed — keep sampling
			prevAlt = gAltitude;
			transDelay = uwTick + APOGEE_SAMPLE_PERIOD_MS;
		}
	}
	return false;
}

bool drogueDescentExitTransition(void)
{
  if(gAltitude < MAIN_DEPLOY_ALTITUDE)
  {
    // Fire main charge (TD2 ejection)
    if (xPyroTaskHandle != NULL)
    {
      xTaskNotify(xPyroTaskHandle, PYRO_MAIN1_BIT, eSetBits);
      //xTaskNotify(xPyroTaskHandle, PYRO_MAIN2_BIT, eSetBits);
    }
    transDelay = sensor_get_tick_ms();
    return true;
  }
  return false;
}

bool mainDescentExitTransition(void)
{
  uint32_t uwTick = sensor_get_tick_ms();
	// Calculate altitude stability
	float deltaAlt = fabs(gAltitude - prevAlt);

  // Look for apogee
  if(uwTick >= transDelay)
  {
    if (deltaAlt < LANDED_ALT_THRESHOLD)
	  {
    	landedSamples++;
    	if (landedSamples > LANDED_SAMPLES_REQ) return true;
	  }
    else
    {
    	landedSamples = 0; // Reset counter
    }

    // still falling, set-up next altitude comparison
    prevAlt = gAltitude;
    transDelay = uwTick + LANDED_SAMPLE_PERIOD_MS; // Separate comparison samples
  }
  return false;
}

bool landedExitTransition(void) // not re-launch prior to reboot
{
  uint32_t uwTick = sensor_get_tick_ms();
	static uint32_t landedTime = 0; // Additional landing verification

	// Check for low acceleration
	if (fabs(gTotalAcc - 1.0f) < 0.2f)
	{
		if (landedTime == 0)
		{
			landedTime = uwTick;
		}
		// Stay in landed state for min 10 sec
		if ((uwTick - landedTime) > LANDED_COOLDOWN_MS)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	landedTime = 0;
	return false;
}
