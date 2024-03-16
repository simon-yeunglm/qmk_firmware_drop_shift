/*
Copyright 2019 Massdrop Inc.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "arm_atsam_protocol.h"
#include "tmk_core/common/led.h"
#include "rgb_matrix.h"
#include <string.h>
#include <math.h>

typedef struct float3_s
{
	float x;
	float y;
	float z;
} float3;

float clamp(float x, float minVal, float maxVal)
{
    return (x > maxVal ? maxVal : (x < minVal ? minVal : x));
}

float square(float x)
{
    return x * x;
}

float smoothStep(float t)
{
    return t*t*(3.0f - 2.0f*t);
}

float lerp(float from, float to, float t)
{
	return (to - from) * t + from;
}

float3 lerp_f3(float3 from, float3 to, float t)
{
    float3 ret=
    {
        lerp(from.x, to.x, t),
        lerp(from.y, to.y, t),
        lerp(from.z, to.z, t)
    };
    return ret;
}

float3 add_f3(float3 a, float3 b)
{
    float3 ret=
    {
        a.x + b.x,
        a.y + b.y,
        a.z + b.z,
    };
    return ret;
}

float3 add_f3_s(float3 f3, float s)
{
    float3 ret=
    {
        f3.x + s,
        f3.y + s,
        f3.z + s,
    };
    return ret;
}

float3 mul_f3_s(float3 f3, float s)
{
    float3 ret=
    {
        f3.x * s,
        f3.y * s,
        f3.z * s,
    };
    return ret;
}

#ifdef USE_MASSDROP_CONFIGURATOR
__attribute__((weak))
led_instruction_t led_instructions[] = { { .end = 1 } };
static void led_matrix_massdrop_config_override(int i);
static void md_led_matrix_indicators(void);

static void led_matrix_simon_tick(void);
static void led_matrix_simon_config_override(int i);
static void change_idle_state(uint8_t newState);
uint32_t    timer_tick_last= 0;
float       render_pass_weight_ctrl = 0.0f;
float       render_pass_weight_shift= 0.0f;
float       render_pass_weight_alt  = 0.0f;

#define NUM_KEYS                        (99)
#define MAX_KEY_SCAN                    (102)
#define UPDATE_KEY_PRESS_NUM_PER_FRAME  (33)
#define PULSE_NUM_MAX                   (16)
#define PULSE_TRAVEL_TIME               (0.5f)

#define RIPPLE_NUM_MAX                   (16)
#define RIPPLE_TRAVEL_TIME               (0.35f)

uint8_t     is_key_pressed[NUM_KEYS];
uint8_t     scan_to_key_idx[MAX_KEY_SCAN];
uint8_t     update_key_press_idx= 0;
float       pulse_life[PULSE_NUM_MAX];  // ring buffer
uint8_t     pulse_next_free_idx = 0;
uint8_t     pulse_active_idx    = 0;
uint8_t     pulse_active_num    = 0;

typedef struct RippleData_s
{
	float life; // remain life in secodn
	float x;    // origin X
	float y;    // origin Y
} RippleData;

RippleData  ripple_data[RIPPLE_NUM_MAX];  // ring buffer
uint8_t     ripple_next_free_idx    = 0;
uint8_t     ripple_active_idx       = 0;
uint8_t     ripple_active_num       = 0;

#define IDLE_WAKE_UP_SLASH_TIME             (0.7f)
#define IDLE_WAKE_UP_SHOW_VERT_LINE_TIME    (0.2f)
#define IDLE_WAKE_UP_MOVE_VERT_LINE_TIME    (0.45f)
#define IDLE_ENTER_IDLE_TIME                (0.25f)
#define IDLE_TIME_THRESHOLD                 (14.0f * 60.0f)
enum IdleState
{
    IdleState_Idle,
    IdleState_WakingUpFromIdle,
    IdleState_WakingUpFromEnteringIdle,
    IdleState_Awake,
    IdleState_EnteringIdle,
    IdleState_Num
};
uint8_t     idle_state= IdleState_WakingUpFromIdle;
float       idle_state_timer= 0.0f;
float       idle_timer      = 0.0f;

#endif // USE_MASSDROP_CONFIGURATOR


void SERCOM1_0_Handler( void )
{
    if (SERCOM1->I2CM.INTFLAG.bit.ERROR)
    {
        SERCOM1->I2CM.INTFLAG.reg = SERCOM_I2CM_INTENCLR_ERROR;
    }
}

void DMAC_0_Handler( void )
{
    if (DMAC->Channel[0].CHINTFLAG.bit.TCMPL)
    {
        DMAC->Channel[0].CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL;

        i2c1_stop();

        i2c_led_q_running = 0;

        i2c_led_q_run();

        return;
    }

    if (DMAC->Channel[0].CHINTFLAG.bit.TERR)
    {
        DMAC->Channel[0].CHINTFLAG.reg = DMAC_CHINTENCLR_TERR;
    }
}

issi3733_driver_t issidrv[ISSI3733_DRIVER_COUNT];

issi3733_led_t led_map[ISSI3733_LED_COUNT] = ISSI3733_LED_MAP;
RGB led_buffer[ISSI3733_LED_COUNT];

uint8_t gcr_desired;                    //User's desired GCR value (brightness setting)
uint8_t gcr_actual;                     //The GCR value about to be written to LED drivers
uint8_t gcr_actual_last;                //The GCR value last written to LED drivers
#ifdef USE_MASSDROP_CONFIGURATOR
uint8_t gcr_breathe;                    //The maximum attained GCR value during a breathe cycle (for smooth light transitions)
float breathe_mult;
float pomod;
uint32_t power_sum = 0;                 //Sum of RGB values for current LED pattern (about to be visible)
uint32_t power_sum_last;                //Sum of RGB values for previous LED pattern (currently visible)
#endif // USE_MASSDROP_CONFIGURATOR

//Actions to take for GCR
#define ACT_GCR_NONE            0       //Take no action on GCR
#define ACT_GCR_INC             1       //Increase GCR
#define ACT_GCR_DEC             2       //Decrease GCR

//GCR step settings
#define LED_GCR_STEP_AUTO       1       //How many GCR steps to take per increase or decrease call
#define ACT_GCR_DEC_COUNT       -1      //Number at which a GCR decrease will actually happen (decrements should happen fast to react to sudden high load)
#define ACT_GCR_INC_COUNT       5       //Number at which a GCR increase will actually happen (increments should happen slow to gently approach power limits)

int8_t gcr_change_counter;              //GCR increase and decrease calls are counted here and acted upon when a count limit is hit
uint16_t v_5v_cat_hit;                  //Flag for when 5v catastrophic level has been reached, and timer for recovery period
uint64_t v_5v_low_timer;                //Timer for disabling USB extra device after causing a low voltage situation for an amount of time (-1 indicates timer not active)

uint8_t led_mfg_test_mode = LED_MFG_TEST_MODE_RAWP;


//WARNING: Automatic GCR is in place to prevent USB shutdown and LED driver overloading
//Note: GCR updates are currently synced to come before a PWM update, so GCR updates actually happen off the PWM update timer
void gcr_compute(void)
{
    uint8_t action = ACT_GCR_NONE;                                  //Default GCR action to be taken
    uint8_t gcr_use = gcr_desired;                                  //The GCR value to test against

    if (!I2C3733_Control_Get())                                     //If LED drivers are not on
    {
        gcr_actual = 0;                                             //Minimize GCR
        return;                                                     //Nothing else to do
    }

#ifdef USE_MASSDROP_CONFIGURATOR
    if (led_animation_breathing)                                    //If breathing effect is being used
    {
        gcr_use = gcr_breathe;                                      //Set max GCR possible to the highest GCR attained during breathe cycles
    }
#endif // USE_MASSDROP_CONFIGURATOR

    if (usb_gcr_auto)                                               //If automatic GCR is enabled
    {
        //If 5v bus is low or the desired GCR is less than actual, and we have not reached the decrease threshold
        if ((uint16_t)g_v_5v_avg < V5_LOW || gcr_desired < gcr_actual)
        {
            if (gcr_change_counter > ACT_GCR_DEC_COUNT)
            {
                gcr_change_counter--;                                   //Decrease GCR change count
            }
        }
        else if ((uint16_t)g_v_5v_avg > V5_HIGH)  //else if 5v bus is high and we have not reached the increase threshold
        {
            if (gcr_change_counter < ACT_GCR_INC_COUNT)
            {
                gcr_change_counter++;                                   //Increase GCR change count
            }
        }
        else
        {
            //If no action, seek zero
            if (gcr_change_counter > 0) { gcr_change_counter--; }
            else if (gcr_change_counter < 0) { gcr_change_counter++; }
        }

        if (gcr_change_counter == ACT_GCR_DEC_COUNT)                //If change counter has reached the decrease threshold
        {
            //Note: Not checking if there is room to decrease here as that is taken into account differently on the actual decrease action
            action = ACT_GCR_DEC;                                   //Set action to decrease
        }
        else if (gcr_change_counter == ACT_GCR_INC_COUNT)           //Else if change counter has reached the increase threshold
        {
            if (gcr_actual < gcr_use && I2C3733_Control_Get())      //If there is room to increase and LED drivers are on
            {
                action = ACT_GCR_INC;                               //Set action to increase
            }
        }
    }
    else                                                            //Else automatic GCR is disabled so immediately follow user's desired value
    {
        if (gcr_actual < gcr_use) action = ACT_GCR_INC;             //If actual has not met the use value, set action to increase
        else if (gcr_actual > gcr_use) action = ACT_GCR_DEC;        //Else if the actual is more than the use value, set action to decrease
    }

    if (action == ACT_GCR_INC)
    {
        gcr_change_counter = 0;
        if (LED_GCR_STEP_AUTO > LED_GCR_MAX - gcr_actual) gcr_actual = LED_GCR_MAX; //Obey max and prevent wrapping
        else gcr_actual += LED_GCR_STEP_AUTO;
    }
    else if (action == ACT_GCR_DEC)
    {
        gcr_change_counter = 0;
        if (LED_GCR_STEP_AUTO > gcr_actual) //Prevent wrapping
        {
            gcr_actual = 0;
        }
        else
        {
            //Power successfully cut back from LED drivers
            gcr_actual -= LED_GCR_STEP_AUTO;

#ifdef USE_MASSDROP_CONFIGURATOR
            //If breathe mode is active, the top end can fluctuate if the host can not supply enough current
            //So set the breathe GCR to where it can safely reach
            if (led_animation_breathing == 1)
            {
                gcr_breathe = gcr_actual;
                //PS: At this point, setting breathing to exhale makes a noticebly shorter cycle
                //    and the same would happen maybe one or two more times. Therefore I'm favoring
                //    powering through one full breathe and letting GCR settle completely
            }
#endif // USE_MASSDROP_CONFIGURATOR
        }
    }
}

void issi3733_prepare_arrays(void)
{
    memset(issidrv,0,sizeof(issi3733_driver_t) * ISSI3733_DRIVER_COUNT);

    int i;
    uint8_t addrs[ISSI3733_DRIVER_COUNT] = ISSI3773_DRIVER_ADDRESSES;

    for (i=0;i<ISSI3733_DRIVER_COUNT;i++)
    {
        issidrv[i].addr = addrs[i];
    }

    for (uint8_t i = 0; i < ISSI3733_LED_COUNT; i++)
    {
        //BYTE: 1 + (SW-1)*16 + (CS-1)
        led_map[i].rgb.g = issidrv[led_map[i].adr.drv-1].pwm + 1 + ((led_map[i].adr.swg-1)*16 + (led_map[i].adr.cs-1));
        led_map[i].rgb.r = issidrv[led_map[i].adr.drv-1].pwm + 1 + ((led_map[i].adr.swr-1)*16 + (led_map[i].adr.cs-1));
        led_map[i].rgb.b = issidrv[led_map[i].adr.drv-1].pwm + 1 + ((led_map[i].adr.swb-1)*16 + (led_map[i].adr.cs-1));

        //BYTE: 1 + (SW-1)*2 + (CS-1)/8
        //BIT: (CS-1)%8
        *(issidrv[led_map[i].adr.drv-1].onoff + 1 + (led_map[i].adr.swg-1)*2+(led_map[i].adr.cs-1)/8) |= (1<<((led_map[i].adr.cs-1)%8));
        *(issidrv[led_map[i].adr.drv-1].onoff + 1 + (led_map[i].adr.swr-1)*2+(led_map[i].adr.cs-1)/8) |= (1<<((led_map[i].adr.cs-1)%8));
        *(issidrv[led_map[i].adr.drv-1].onoff + 1 + (led_map[i].adr.swb-1)*2+(led_map[i].adr.cs-1)/8) |= (1<<((led_map[i].adr.cs-1)%8));
    }
}

void led_matrix_prepare(void)
{
    for (uint8_t i = 0; i < ISSI3733_LED_COUNT; i++)
    {
        *led_map[i].rgb.r = 0;
        *led_map[i].rgb.g = 0;
        *led_map[i].rgb.b = 0;
    }
    
    // init key press state
    for (uint8_t i = 0; i < NUM_KEYS; ++i)
        is_key_pressed[i]= 0;
    
    // initialize key map table
    for (uint8_t i = 0; i < MAX_KEY_SCAN; ++i)
        scan_to_key_idx[i]= NO_LED;
    for (uint8_t i = 0; i < NUM_KEYS; ++i)
    {
        uint8_t idx= led_map[i].id - 1;
        if (idx < MAX_KEY_SCAN)
            scan_to_key_idx[idx]= i;
    }
    
    // init pulse
    for(uint8_t i=0; i<PULSE_NUM_MAX; ++i)
        pulse_life[i]= 0.0f;

    // init ripple
    for(uint8_t i=0; i<RIPPLE_NUM_MAX; ++i)
        ripple_data[i].life= 0.0f;
}

__attribute__((weak))
void led_set_one(int i, uint8_t r, uint8_t g, uint8_t b)
{
	if(led_mfg_test_mode != LED_MFG_TEST_MODE_RAWP)
	{
		if (i < ISSI3733_LED_COUNT)
		{
#ifdef USE_MASSDROP_CONFIGURATOR
			led_matrix_massdrop_config_override(i);
#else
			led_buffer[i].r = r;
			led_buffer[i].g = g;
			led_buffer[i].b = b;
#endif // USE_MASSDROP_CONFIGURATOR
		}
	}
    else
    {
		if (i < ISSI3733_LED_COUNT)
		{
#ifdef USE_MASSDROP_CONFIGURATOR
			led_matrix_simon_config_override(i);
#endif
        }
    }
}

void led_set_all(uint8_t r, uint8_t g, uint8_t b)
{
	for (uint8_t i = 0; i < ISSI3733_LED_COUNT; i++)
	{
		led_set_one(i, r, g, b);
	}
}

void led_set_one_rawp(int i, uint8_t r, uint8_t g, uint8_t b)
{
	if(led_mfg_test_mode == LED_MFG_TEST_MODE_RAWP)
	{
		if (i < ISSI3733_LED_COUNT)
		{
			led_buffer[i].r = r;
			led_buffer[i].g = g;
			led_buffer[i].b = b;
		}
	}
}

void init(void)
{
    DBGC(DC_LED_MATRIX_INIT_BEGIN);

    issi3733_prepare_arrays();

    led_matrix_prepare();

    gcr_change_counter = 0;
    v_5v_cat_hit = 0;

    DBGC(DC_LED_MATRIX_INIT_COMPLETE);
}

void flush(void)
{
    uint8_t drvid;

#ifdef USE_MASSDROP_CONFIGURATOR
    md_led_matrix_indicators();

    //If there will be a sudden spike in required power, lower GCR prior to change according to some ratio
    if (power_sum > (uint32_t)((float)power_sum_last * 1.5))
    {
        //Lower GCR according to a percentage of the change in power ratio
        gcr_actual = (uint8_t)((((float)gcr_actual * ((float)power_sum_last / (float)power_sum)) * 0.8) + 0.5);
    }
    power_sum_last = power_sum;
    power_sum = 0;

    //NOTE: Allow GCR updates even if the drivers lighting is disabled
    if (gcr_actual != gcr_actual_last)
    {
        for (drvid=0;drvid<ISSI3733_DRIVER_COUNT;drvid++) { I2C_LED_Q_GCR(drvid); } //Queue data
        gcr_actual_last = gcr_actual;
        if (!I2C3733_Control_Get()) { i2c_led_q_run(); } //Run the queue if we know flush will not complete due to drivers being disabled
    }
#endif // USE_MASSDROP_CONFIGURATOR

    if (!I2C3733_Control_Get()) { return; } //Prevent calculations and I2C traffic if LED drivers are not enabled

    //Wait for previous transfer to complete
    //If timings are proper, we should never get into this busy wait state
    while (i2c_led_q_running) {}

    //Copy buffer to live DMA region
    for (uint8_t i = 0; i < ISSI3733_LED_COUNT; i++)
    {
        *led_map[i].rgb.r = led_buffer[i].r;
        *led_map[i].rgb.g = led_buffer[i].g;
        *led_map[i].rgb.b = led_buffer[i].b;
    }

    for (drvid = 0; drvid < ISSI3733_DRIVER_COUNT; drvid++) { I2C_LED_Q_PWM(drvid); } //Queue data

    i2c_led_q_run();

    //Perform any once-per-frame calculations or setup here

#ifdef USE_MASSDROP_CONFIGURATOR
    if (led_animation_breathing)
    {
        led_animation_breathe_cur += BREATHE_STEP * breathe_dir;

        if (led_animation_breathe_cur >= BREATHE_MAX_STEP) { breathe_dir = -1; }
        else if (led_animation_breathe_cur <= BREATHE_MIN_STEP) { breathe_dir = 1; }

        //Brightness curve created for 256 steps, 0 - ~98%
        breathe_mult = 0.000015 * led_animation_breathe_cur * led_animation_breathe_cur;
        if (breathe_mult > 1) { breathe_mult = 1; }
        else if (breathe_mult < 0) { breathe_mult = 0; }
    }

    pomod = 0;
    if (led_animation_speed != 0) //Avoid DIV0
    {
        pomod = (float)((g_rgb_counters.tick / 10) % (uint32_t)(1000.0f / led_animation_speed)) / 10.0f * led_animation_speed;
    }
    pomod *= 100.0f;
    pomod = (uint32_t)pomod % 10000;
    pomod /= 100.0f;


	if(led_mfg_test_mode == LED_MFG_TEST_MODE_RAWP)
        led_matrix_simon_tick();
    timer_tick_last= g_rgb_counters.tick;
#endif // USE_MASSDROP_CONFIGURATOR
}

void led_matrix_indicators(void)
{
	// Null - this is called too many times during the rendering process
}

#ifdef USE_MASSDROP_CONFIGURATOR
static void md_led_matrix_indicators(void)
{
    uint8_t kbled = keyboard_leds();
    if (/*kbled &&*/ rgb_matrix_config.enable)
    {
        for (uint8_t i = 0; i < ISSI3733_LED_COUNT; i++)
        {
#ifdef USB_LED_INDICATOR_ENABLE
            if ( (led_mfg_test_mode == LED_MFG_TEST_MODE_OFF) && (
            #ifdef USB_LED_NUM_LOCK_SCANCODE
                (led_map[i].scan == USB_LED_NUM_LOCK_SCANCODE && (kbled & (1<<USB_LED_NUM_LOCK))) ||
            #endif //NUM LOCK
            #ifdef USB_LED_CAPS_LOCK_SCANCODE
                (led_map[i].scan == USB_LED_CAPS_LOCK_SCANCODE && (kbled & (1<<USB_LED_CAPS_LOCK))) ||
            #endif //CAPS LOCK
            #ifdef USB_LED_SCROLL_LOCK_SCANCODE
                (led_map[i].scan == USB_LED_SCROLL_LOCK_SCANCODE && (kbled & (1<<USB_LED_SCROLL_LOCK))) ||
            #endif //SCROLL LOCK
            #ifdef USB_LED_COMPOSE_SCANCODE
                (led_map[i].scan == USB_LED_COMPOSE_SCANCODE && (kbled & (1<<USB_LED_COMPOSE))) ||
            #endif //COMPOSE
            #ifdef USB_LED_KANA_SCANCODE
                (led_map[i].scan == USB_LED_KANA_SCANCODE && (kbled & (1<<USB_LED_KANA))) ||
            #endif //KANA
			// Dedicated LEDs (Could be done more efficiently - meh)
            (0)))
            {
                led_buffer[i].r = 255 - led_buffer[i].r;
                led_buffer[i].g = 255 - led_buffer[i].g;
                led_buffer[i].b = 255 - led_buffer[i].b;
            }
#endif
#ifdef DEDICATED_LED_INDICATOR_ENABLE
            if ( (led_mfg_test_mode == LED_MFG_TEST_MODE_OFF) && (
			// Turn off unactive dedicated LED indicators
			#ifdef USB_LED_NUM_LOCK_LEDID
				(led_map[i].id == USB_LED_NUM_LOCK_LEDID && ((kbled & (1<<USB_LED_NUM_LOCK))==0) ) ||
			#endif
			#ifdef USB_LED_CAPS_LOCK_LEDID
				(led_map[i].id == USB_LED_CAPS_LOCK_LEDID && ((kbled & (1<<USB_LED_CAPS_LOCK))==0) ) ||
			#endif
			#ifdef USB_LED_SCROLL_LOCK_LEDID
				(led_map[i].id == USB_LED_SCROLL_LOCK_LEDID && ((kbled & (1<<USB_LED_SCROLL_LOCK))==0) ) ||
			#endif
			#ifdef USB_LED_COMPOSE_LEDID
				(led_map[i].id == USB_LED_COMPOSE_LEDID && !(kbled & (1<<USB_LED_COMPOSE))) ||
			#endif
			#ifdef USB_LED_KANA_LEDID
				(led_map[i].id == USB_LED_KANA_LEDID && !(kbled & (1<<USB_LED_KANA))) ||
			#endif
            (0)))
            {
                led_buffer[i].r = 0;
                led_buffer[i].g = 0;
                led_buffer[i].b = 0;
                //led_buffer[i].r = 255 - led_buffer[i].r;
                //led_buffer[i].g = 255 - led_buffer[i].g;
                //led_buffer[i].b = 255 - led_buffer[i].b;
            }
#endif
        }
    }

}
#endif

const rgb_matrix_driver_t rgb_matrix_driver = {
  .init = init,
  .flush = flush,
  .set_color = led_set_one,
  .set_color_all = led_set_all
};

/*==============================================================================
=                           Legacy Lighting Support                            =
==============================================================================*/

#ifdef USE_MASSDROP_CONFIGURATOR
// Ported from Massdrop QMK Github Repo

// TODO?: wire these up to keymap.c
uint8_t led_animation_orientation = 0;
uint8_t led_animation_direction = 0;
uint8_t led_animation_breathing = 0;
uint8_t led_animation_id = 0;
float led_animation_speed = 3.0f;//4.0f;
uint8_t led_lighting_mode = LED_MODE_NORMAL;
uint8_t led_animation_breathe_cur = BREATHE_MIN_STEP;
uint8_t breathe_dir = 1;
uint8_t led_animation_circular = 0;
float led_edge_brightness = 1.0f;
float led_ratio_brightness = 1.0f;
uint8_t led_edge_mode = LED_EDGE_MODE_ALL;

static void led_run_pattern(led_setup_t *f, float* ro, float* go, float* bo, float pos) {
    float po;

    while (f->end != 1)
    {
        po = pos; //Reset po for new frame

        //Add in any moving effects
        if ((!led_animation_direction && f->ef & EF_SCR_R) || (led_animation_direction && (f->ef & EF_SCR_L)))
        {
            po -= pomod;

            if (po > 100) po -= 100;
            else if (po < 0) po += 100;
        }
        else if ((!led_animation_direction && f->ef & EF_SCR_L) || (led_animation_direction && (f->ef & EF_SCR_R)))
        {
            po += pomod;

            if (po > 100) po -= 100;
            else if (po < 0) po += 100;
        }

        //Check if LED's po is in current frame
        if (po < f->hs) { f++; continue; }
        if (po > f->he) { f++; continue; }
        //note: < 0 or > 100 continue

        //Calculate the po within the start-stop percentage for color blending
        po = (po - f->hs) / (f->he - f->hs);

        //Add in any color effects
        if (f->ef & EF_OVER)
        {
            *ro = (po * (f->re - f->rs)) + f->rs;// + 0.5;
            *go = (po * (f->ge - f->gs)) + f->gs;// + 0.5;
            *bo = (po * (f->be - f->bs)) + f->bs;// + 0.5;
        }
        else if (f->ef & EF_SUBTRACT)
        {
            *ro -= (po * (f->re - f->rs)) + f->rs;// + 0.5;
            *go -= (po * (f->ge - f->gs)) + f->gs;// + 0.5;
            *bo -= (po * (f->be - f->bs)) + f->bs;// + 0.5;
        }
        else
        {
            *ro += (po * (f->re - f->rs)) + f->rs;// + 0.5;
            *go += (po * (f->ge - f->gs)) + f->gs;// + 0.5;
            *bo += (po * (f->be - f->bs)) + f->bs;// + 0.5;
        }

        f++;
    }
}

//TODO: For circular animation fix?// void disp_calc_extents(void)
//TODO: For circular animation fix?// {
//TODO: For circular animation fix?//     issi3733_led_t *cur = led_map;
//TODO: For circular animation fix?//
//TODO: For circular animation fix?//     disp.left = 1e10;
//TODO: For circular animation fix?//     disp.right = -1e10;
//TODO: For circular animation fix?//     disp.top = -1e10;
//TODO: For circular animation fix?//     disp.bottom = 1e10;
//TODO: For circular animation fix?//
//TODO: For circular animation fix?//     while (cur < lede)
//TODO: For circular animation fix?//     {
//TODO: For circular animation fix?//         if (cur->x < disp.left) disp.left = cur->x;
//TODO: For circular animation fix?//         if (cur->x > disp.right) disp.right = cur->x;
//TODO: For circular animation fix?//         if (cur->y < disp.bottom) disp.bottom = cur->y;
//TODO: For circular animation fix?//         if (cur->y > disp.top) disp.top = cur->y;
//TODO: For circular animation fix?//
//TODO: For circular animation fix?//         cur++;
//TODO: For circular animation fix?//     }
//TODO: For circular animation fix?//
//TODO: For circular animation fix?//     disp.width = disp.right - disp.left;
//TODO: For circular animation fix?//     disp.height = disp.top - disp.bottom;
//TODO: For circular animation fix?//     disp.max_distance = sqrtf(powf(disp.width, 2) + powf(disp.height, 2));
//TODO: For circular animation fix?// }

#define RGB_MAX_DISTANCE 232.9635f

static void led_matrix_massdrop_config_override(int i)
{
    float ro = 0;
    float go = 0;
    float bo = 0;
    float po;
    uint8_t highest_active_layer = biton32(layer_state);

    if (led_animation_circular) {
        // TODO: should use min/max values from LED configuration instead of
        // hard-coded 224, 64
        // po = sqrtf((powf(fabsf((disp.width / 2) - (led_cur->x - disp.left)), 2) + powf(fabsf((disp.height / 2) - (led_cur->y - disp.bottom)), 2))) / disp.max_distance * 100;
        po = sqrtf((powf(fabsf((224 / 2) - (float)g_led_config.point[i].x), 2) + powf(fabsf((64 / 2) - (float)g_led_config.point[i].y), 2))) / RGB_MAX_DISTANCE * 100;
    } else {
        if (led_animation_orientation) {
            po = (float)g_led_config.point[i].y / 64.f * 100;
        } else {
            po = (float)g_led_config.point[i].x / 224.f * 100;
        }
    }

    if (led_lighting_mode == LED_MODE_KEYS_ONLY && LED_IS_EDGE(led_map[i].scan)) {
        //Do not act on this LED
    } else if (led_lighting_mode == LED_MODE_NON_KEYS_ONLY && LED_IS_KEY(led_map[i].scan)) {
        //Do not act on this LED
    } else if (led_edge_mode == LED_EDGE_MODE_ALTERNATE && LED_IS_EDGE_ALT(led_map[i].scan)) {
        //Do not act on this LED (Edge alternate lighting mode)
    } else if (led_lighting_mode == LED_MODE_INDICATORS_ONLY && !LED_IS_INDICATOR(led_map[i].scan)) {
        //Do not act on this LED (Only show indicators)
    } else {
        led_instruction_t* led_cur_instruction = led_instructions;
        while (!led_cur_instruction->end) {
            // Check if this applies to current layer
            if ((led_cur_instruction->flags & LED_FLAG_MATCH_LAYER) &&
                (led_cur_instruction->layer != highest_active_layer)) {
                goto next_iter;
            }

            // Check if this applies to current index
            if (led_cur_instruction->flags & LED_FLAG_MATCH_ID) {
                uint8_t ledid = led_map[i].id - 1;
                uint8_t modid = ledid / 32; // Calculate which id# contains the led bit
                uint32_t modidbit = 1 << (ledid % 32); // Calculate the bit within the id#
                uint32_t *bitfield = &led_cur_instruction->id0 + modid;     //Add modid as offset to id0 address. *bitfield is now idX of the led id
                if (~(*bitfield) & modidbit) {                              //Check if led bit is not set in idX
                    goto next_iter;
                }
            }

            if (led_cur_instruction->flags & LED_FLAG_USE_RGB) {
                ro = led_cur_instruction->r;
                go = led_cur_instruction->g;
                bo = led_cur_instruction->b;
            } else if (led_cur_instruction->flags & LED_FLAG_USE_PATTERN) {
                led_run_pattern(led_setups[led_cur_instruction->pattern_id], &ro, &go, &bo, po);
            } else if (led_cur_instruction->flags & LED_FLAG_USE_ROTATE_PATTERN) {
                led_run_pattern(led_setups[led_animation_id], &ro, &go, &bo, po);
            }

            next_iter:
                led_cur_instruction++;
        }

        //Clamp values 0-255
        if (ro > 255) ro = 255; else if (ro < 0) ro = 0;
        if (go > 255) go = 255; else if (go < 0) go = 0;
        if (bo > 255) bo = 255; else if (bo < 0) bo = 0;

        if (led_animation_breathing)
        {
            ro *= breathe_mult;
            go *= breathe_mult;
            bo *= breathe_mult;
        }
    }

    //Adjust edge LED brightness
    if (led_edge_brightness != 1 && LED_IS_EDGE(led_map[i].scan))
    {
        ro *= led_edge_brightness;
        go *= led_edge_brightness;
        bo *= led_edge_brightness;
    }

    //Adjust ratio of key vs. underglow (edge) LED brightness
	if(LED_IS_EDGE(led_map[i].scan) && led_ratio_brightness > 1.0 )
	{
		// Decrease edge (underglow) LEDs
		ro *= (2.0 - led_ratio_brightness);
		go *= (2.0 - led_ratio_brightness);
		bo *= (2.0 - led_ratio_brightness);
	}
	else if(LED_IS_KEY(led_map[i].scan) && led_ratio_brightness < 1.0)
	{
		// Decrease KEY LEDs
		ro *= led_ratio_brightness;
		go *= led_ratio_brightness;
		bo *= led_ratio_brightness;
	}

    led_buffer[i].r = (uint8_t)ro;
    led_buffer[i].g = (uint8_t)go;
    led_buffer[i].b = (uint8_t)bo;

    power_sum += (uint8_t)ro + (uint8_t)go + (uint8_t)bo;
}

void add_reactive_ripple(float originX, float originY)
{
    ripple_data[ripple_next_free_idx].life  = RIPPLE_TRAVEL_TIME;
    ripple_data[ripple_next_free_idx].x     = originX;
    ripple_data[ripple_next_free_idx].y     = originY;
    ripple_next_free_idx                    = (ripple_next_free_idx + 1) % RIPPLE_NUM_MAX;
    ++ripple_active_num;
}

bool led_process_record(uint16_t keycode, keyrecord_t *record)
{
    uint8_t row     = record->event.key.row;
    uint8_t col     = record->event.key.col;
    uint8_t key_scan= g_led_config.matrix_co[row][col];

    // update idle state
	if(led_mfg_test_mode == LED_MFG_TEST_MODE_RAWP)
    {
    	if (idle_state == IdleState_Idle)
            change_idle_state(IdleState_WakingUpFromIdle);
        else if (idle_state == IdleState_EnteringIdle)
            change_idle_state(IdleState_WakingUpFromEnteringIdle);

        // reset idle state
        idle_timer= 0;
    }

    if (key_scan >= MAX_KEY_SCAN)
        return true;    // should not enter this case, need to update the scan_to_key_idx table

    uint8_t led_idx= scan_to_key_idx[key_scan];
    if (led_idx >= NUM_KEYS)
        return true;    // not a key

    if (record->event.pressed)
        is_key_pressed[led_idx]= 255;
    else
    {
        is_key_pressed[led_idx]= 254;

        // launch pulse when releasing back space key
        if (led_idx == 31) 
        {
            pulse_life[pulse_next_free_idx] = PULSE_TRAVEL_TIME;
            pulse_next_free_idx             = (pulse_next_free_idx + 1) % PULSE_NUM_MAX;
            ++pulse_active_num;
        }
        else if (led_idx == 5)  // F5
            add_reactive_ripple(22.0f, 2.0f);
        else if (led_idx == 7)  // F7
            add_reactive_ripple(30.0f, 2.0f);
        else if (led_idx == 66)  // enter
            add_reactive_ripple(54.0f,15.0f);
        else if (led_idx == 87)  // numpad enter
            add_reactive_ripple(72.0f,19.0f);
        else if (led_idx == 91)  // space bar
            add_reactive_ripple(27.0f,22.0f);
    }
    return true;
}

static bool cal_breath_col(float3 col0, float3 col1, float colT, float layerAlpha, float* accum_inv_alpha, float3* outCol)
{
    float invAlpha= *accum_inv_alpha;
    float3	col = lerp_f3(col0, col1, colT);
    *outCol     = add_f3(*outCol, mul_f3_s(col, layerAlpha * invAlpha));

    invAlpha            *= 1.0f - layerAlpha;
    *accum_inv_alpha    = invAlpha;
    return layerAlpha < 1.0f;
}

static void change_idle_state(uint8_t newState)
{
	if (!(idle_state == IdleState_EnteringIdle && idle_state == IdleState_WakingUpFromEnteringIdle))
		idle_state_timer= 0.0f;
	idle_state = newState;
}

static void led_matrix_simon_tick(void)
{
    // get time
    uint32_t    deltaTick   = min(g_rgb_counters.tick - timer_tick_last, 100);  // clamp it in case of lag or warp around, in ms
    float       deltaT      = deltaTick * 0.001f;                               // convert to seconds

    // update modifier weight
    {
        const float     blendTime   = 0.125f;
        const float     blendSpeed	= 1.0f/blendTime;
        float           blendDelta  = blendSpeed * deltaT;

        uint8_t modifers= get_mods();
        bool isHeldCtrl = modifers & MOD_MASK_CTRL  ;
        bool isHeldShift= modifers & MOD_MASK_SHIFT ;
        bool isHeldAlt  = modifers & MOD_MASK_ALT   ;
    
        render_pass_weight_ctrl = clamp(render_pass_weight_ctrl + (isHeldCtrl   ? blendDelta : -blendDelta) , 0.0f, 1.0f);
        render_pass_weight_shift= clamp(render_pass_weight_shift+ (isHeldShift  ? blendDelta : -blendDelta) , 0.0f, 1.0f);
        render_pass_weight_alt  = clamp(render_pass_weight_alt  + (isHeldAlt    ? blendDelta : -blendDelta) , 0.0f, 1.0f);
    }

    // update key press
    {
        uint8_t idx= update_key_press_idx;
        uint8_t updateVal= deltaTick * (NUM_KEYS / UPDATE_KEY_PRESS_NUM_PER_FRAME) / 2; // fade out in 0.5 sec
        for(uint8_t i=0; i<UPDATE_KEY_PRESS_NUM_PER_FRAME; ++i)
        {
            uint8_t key_val= is_key_pressed[idx];
            if (key_val > 0 && key_val < 255)
            {
                if (key_val < updateVal)
                    is_key_pressed[idx]= 0;
                else
                    is_key_pressed[idx]= key_val - updateVal;
            }
            idx= (idx + 1) % NUM_KEYS;
        }
        update_key_press_idx= idx;
    }

    // tick pulse
    {
        int idx= pulse_active_idx;
        for(int i=0; i<pulse_active_num; ++i)
        {
            float life= pulse_life[idx];
            if (life <= 0.0f)
                break;
            life -= deltaT;
            if (life <= 0.0f)
            {
                --pulse_active_num;
                pulse_active_idx= (pulse_active_idx + 1) % PULSE_NUM_MAX;
                life= 0.0f;
            }
            pulse_life[idx]= life;
            idx= (idx + 1) % PULSE_NUM_MAX;
        }
    }

    // tick ripple
    {
        int idx= ripple_active_idx;
        for(int i=0; i<ripple_active_num; ++i)
        {
            float life= ripple_data[idx].life;
            if (life <= 0.0f)
                break;
            life -= deltaT;
            if (life <= 0.0f)
            {
                --ripple_active_num;
                ripple_active_idx= (ripple_active_idx + 1) % RIPPLE_NUM_MAX;
                life= 0.0f;
            }
            ripple_data[idx].life= life;
            idx= (idx + 1) % RIPPLE_NUM_MAX;
        }
    }

    // tick idle
    {
        if (idle_state == IdleState_WakingUpFromIdle)
        {
            idle_state_timer+= deltaT;
            if (idle_state_timer >= (IDLE_WAKE_UP_SLASH_TIME + IDLE_WAKE_UP_SHOW_VERT_LINE_TIME + IDLE_WAKE_UP_MOVE_VERT_LINE_TIME))
                change_idle_state(IdleState_Awake);
        }
        else if (idle_state == IdleState_WakingUpFromEnteringIdle)
        {
            idle_state_timer-= deltaT;
            if (idle_state_timer <= 0.0f)
                change_idle_state(IdleState_Awake);
        }
        else if (idle_state == IdleState_Awake)
        {
            float prevTimer	= idle_timer;
            float currTimer	= prevTimer + deltaT;
            idle_timer		= currTimer;
            if (currTimer >= IDLE_TIME_THRESHOLD && prevTimer< IDLE_TIME_THRESHOLD)
                change_idle_state(IdleState_EnteringIdle);
        }
        else if (idle_state == IdleState_EnteringIdle)
        {
            idle_state_timer+= deltaT;
            if (idle_state_timer >= IDLE_ENTER_IDLE_TIME)
                change_idle_state(IdleState_Idle);
        }
    }
}

void led_sleep(void)
{
    if (idle_state == IdleState_Awake)
    {
        idle_timer= IDLE_TIME_THRESHOLD - 0.1f;
    }
}

static void led_matrix_simon_config_override(int i)
{
    // render target system values
    const float renderTargetWidth   = 76.0f;
    const float renderTargetHeight  = 24.0f;
    const float NUM_PIXEL_PER_KEY   = 4.0f;
    float       x                   = ((float)      g_led_config.point[i].x ) * (renderTargetWidth /225.0f);
    float       y                   = ((float)(64 - g_led_config.point[i].y)) * (renderTargetHeight/64.0f);
    float3      rgb                 = {0, 0, 0};
    bool        isContinueRenderPass= true;

    // get timer
    uint32_t    tick_warp   = g_rgb_counters.tick % (43200 * 1000);             // warp around 0.5 day, .tick unit is ms
    float       timer       = tick_warp * 0.001f;                               // convert to seconds
    float       accum_inv_alpha = 1.0f;

    // idle animation
    {
        if (idle_state == IdleState_Idle)
        {
            led_buffer[i].r = 0;
            led_buffer[i].g = 0;
            led_buffer[i].b = 0;
            return;
        }
        else if (idle_state == IdleState_WakingUpFromIdle)
        {
            if (idle_state_timer < IDLE_WAKE_UP_SLASH_TIME)
            {
                float		t				= idle_state_timer * (1.0f / IDLE_WAKE_UP_SLASH_TIME );
                float		m				= 4.0f;							// line - slope
                float		c				= lerp(70.0f, -350.0f, t);      // line - intercept
                float		a				= m;							// line equation general form
                float		b				=-1.0f;							// line equation general form
                float		disToLineFactor	= 1.0f/sqrtf(a*a+b*b);		    // denominator of distance to line equation
                const float	lineHalfWidth	= 10.5f;
                
				float dis   = fabsf(a*x + b*y + c) * disToLineFactor;
				float alpha	= min(dis/lineHalfWidth, 1.0f);
				alpha		*= alpha;
                rgb.x= 0.0f;
                rgb.y= 0.0f;
                rgb.z= 0.0f;
                accum_inv_alpha = 1.0f - alpha;
            }
            else if (idle_state_timer < (IDLE_WAKE_UP_SLASH_TIME + IDLE_WAKE_UP_SHOW_VERT_LINE_TIME))
            {
                led_buffer[i].r = 0;
                led_buffer[i].g = 0;
                led_buffer[i].b = 0;
                return;
            }
            else
            {
	            const float	keyWidth= 4.0f;
                float t	= (idle_state_timer - (IDLE_WAKE_UP_SLASH_TIME + IDLE_WAKE_UP_SHOW_VERT_LINE_TIME))/ IDLE_WAKE_UP_MOVE_VERT_LINE_TIME;
                float rtHalfWidth   = renderTargetWidth * 0.5f;
                float blackLineMaxX = (rtHalfWidth + keyWidth) * t;
                float blackLineMinX = blackLineMaxX - keyWidth;

                float x_center= fabsf(x - rtHalfWidth);
                float alpha   = clamp((x_center - blackLineMinX)/(blackLineMaxX - blackLineMinX), 0.0f, 1.0f);
                rgb.x= 0;
                rgb.y= 0;
                rgb.z= 0;
                accum_inv_alpha = 1.0f - alpha;
            }
        }
        else if (idle_state == IdleState_WakingUpFromEnteringIdle ||
                 idle_state == IdleState_EnteringIdle)
        {
            float	t			= idle_state_timer * (1.0f / IDLE_ENTER_IDLE_TIME);
            
            const float	keyHeight= 4.0f;
            float rtHalfHeight   = renderTargetHeight * 0.5f;
            float blackLineMaxY = (rtHalfHeight + keyHeight) * t;
            float blackLineMinY = blackLineMaxY - keyHeight;

            float y_center= fabsf(y - rtHalfHeight);
            float alpha   = 1.0f - clamp((y_center - blackLineMinY)/(blackLineMaxY - blackLineMinY), 0.0f, 1.0f);
            rgb.x= 0;
            rgb.y= 0;
            rgb.z= 0;
            accum_inv_alpha = 1.0f - alpha;
        }
    }

    // pulse launched from back space
    {
        const float pulseRadius     = 2.0f;
        const float pulseStartPos   = 50.0f;
        const float pulseEndPos     = -pulseRadius;
        int idx= pulse_active_idx;
        for(int i=0; i<pulse_active_num; ++i)
        {
            float life  = pulse_life[idx];
            float t     = life * (1.0f/PULSE_TRAVEL_TIME);
            float pulsePos= lerp(pulseEndPos, pulseStartPos, t);
            if (fabsf(x - pulsePos  ) <= pulseRadius &&
                fabsf(y - 6         ) <= pulseRadius)
            {
                rgb.x   = 1.0f;
                rgb.y   = 1.0f;
                rgb.z   = 1.0f;
                isContinueRenderPass= false;
                break;
            }
            idx= (idx + 1) % PULSE_NUM_MAX;
        }
    }

    // reactive ripple
    {
        const float rippleRadius    = 4.0f; // include fade out regin
        const float rippleTravelDis = 28.0f;
        int idx= ripple_active_idx;
        for(int i=0; i<ripple_active_num; ++i)
        {
            float life      = ripple_data[idx].life;
            float originX   = ripple_data[idx].x;
            float originY   = ripple_data[idx].y;
            float t         = life * (1.0f/RIPPLE_TRAVEL_TIME);
            float travelledDis  = rippleTravelDis * (1.0f - t);
            float disToOrigin   = sqrtf(square(x - originX) + square(y - originY));
            float disToRipple   = fabsf(travelledDis - disToOrigin);
            float alpha         = 1.0f - square(min(disToRipple * (1.0f / rippleRadius), 1.0f));
            alpha               *= sqrtf(t);
            if (alpha > 0.0f)
            {
                rgb             = add_f3_s(rgb, alpha * accum_inv_alpha);   // assume using white color
                accum_inv_alpha *= 1.0f - alpha;
            }
            idx= (idx + 1) % RIPPLE_NUM_MAX;
        }
    }

    // modifier blink
    {
        // blink settings
        const float     changeColorTime = 0.4f;
        float           speed           = 1.0f/changeColorTime;

        // calculate blink weight
        float blinkWeight   = fmodf(timer * speed, 2.0f);
        blinkWeight         = blinkWeight > 1.0f ? 2.0f - blinkWeight : blinkWeight;
        blinkWeight         = smoothStep(blinkWeight);

        // ctrl
        if (isContinueRenderPass && render_pass_weight_ctrl > 0.0f)
        {
            float3	col0= {1.0f, 0.075f, 0.075f};
            float3	col1= {0.35f, 0.025f, 0.025f};
            isContinueRenderPass = cal_breath_col(col0, col1, blinkWeight, render_pass_weight_ctrl, &accum_inv_alpha, &rgb);
        }
        
        // shift
        if (isContinueRenderPass && render_pass_weight_shift > 0.0f)
        {
            float3	col0= {0.075f, 0.075f, 1.0f};
            float3	col1= {0.02f, 0.02f, 0.35f};
            isContinueRenderPass = cal_breath_col(col0, col1, blinkWeight, render_pass_weight_shift, &accum_inv_alpha, &rgb);
        }
        
        // alt
        if (isContinueRenderPass && render_pass_weight_alt > 0.0f)
        {
            float3	col0= {1.0f, 0.9f, 0.1f};
            float3	col1= {0.5f, 0.4f, 0.04f};
            isContinueRenderPass = cal_breath_col(col0, col1, blinkWeight, render_pass_weight_alt, &accum_inv_alpha, &rgb);
        }
    }

    // color scroll
    if (isContinueRenderPass)
    {
		const float3    scrollColor[]= { 
			{1.0f , 0.0f , 0.0f }, // red
			{1.0f , 0.5f , 0.0f }, // orange
			{1.0f , 1.0f , 0.0f }, // yellow
			{0.5f , 0.75f, 1.0f }, // cyan 1
			{0.25f, 0.5f , 1.0f }, // cyan 2
			{0.0f , 0.0f , 1.0f }, // blue
			{0.5f , 0.0f , 0.75f}, // purple
			{1.0f , 0.0f , 0.75f}, // pink
		};
        const int       numColor    	    = sizeof(scrollColor)/sizeof(float3);
        const float     distanceBetweenColor= 10.0f;
        const float     changeColorTime     = 1.5f;
        float	idxOffset	= 1.0f/(distanceBetweenColor * NUM_PIXEL_PER_KEY);
        float   speed       = 1.0f/changeColorTime;
        float	idxStart	= numColor - fmodf(timer * speed, (float)numColor);
        {
            float	idxT= fmodf(idxStart + x * idxOffset, (float)numColor);
            int		idx0= (int)idxT;
            int		idx1= (idx0 + 1) % numColor;
            float3	col0= scrollColor[idx0];
            float3	col1= scrollColor[idx1];

            float	t	= idxT - (float)idx0;
            float3	col = lerp_f3(col0, col1, t);
            rgb         = add_f3(rgb, mul_f3_s(col, accum_inv_alpha));
        }
    }

    // reactive key
    if (i < NUM_KEYS)
    {
        if (is_key_pressed[i] > 0)
        {
            float t         = is_key_pressed[i] * (1.0f / 255.0f);
            float3 white    = { 1.0f, 1.0f, 1.0f };
            rgb             = lerp_f3(rgb, white, t);
        }
    }

    // blink lock keys
    uint8_t kbled = keyboard_leds();
    uint8_t keyID= led_map[i].id;
    bool isOffNumLock   = (kbled & (1<<USB_LED_NUM_LOCK   ))==0;
    bool isOffCapLock   = (kbled & (1<<USB_LED_CAPS_LOCK  ))==0;
    bool isOffScrollLock= (kbled & (1<<USB_LED_SCROLL_LOCK))==0;
    if (isOffNumLock || !isOffCapLock)
    {
        const float blinkSpeed  = 2.0f;
        float blinkTimer        = fmodf(timer * blinkSpeed, 1.0f);
        float blinkMultiplier   = blinkTimer > 0.5f ? 1.0f : 0.1f;
        if ((keyID == 36 && isOffNumLock    ) ||
            (keyID == 58 && (!isOffCapLock) ) )
            rgb= mul_f3_s(rgb, blinkMultiplier);
    }
  
    // dim dedicated indicator LED if not active
    if (((keyID == USB_LED_NUM_LOCK_LEDID   ) && isOffNumLock     ) ||
        ((keyID == USB_LED_CAPS_LOCK_LEDID  ) && isOffCapLock     ) ||
        ((keyID == USB_LED_SCROLL_LOCK_LEDID) && isOffScrollLock  ) )
    {
        float dimFactor= 0.1f;
        rgb= mul_f3_s(rgb, dimFactor);
    }

    // convert from float range [0, 1] to int range [0, 255] and compute power consumption
    {
        uint8_t ro= (uint8_t)(rgb.x * 255.0f);
        uint8_t go= (uint8_t)(rgb.y * 255.0f);
        uint8_t bo= (uint8_t)(rgb.z * 255.0f);

        led_buffer[i].r = ro;
        led_buffer[i].g = go;
        led_buffer[i].b = bo;

        power_sum += ro + go + bo;
    }
}

#endif // USE_MASSDROP_CONFIGURATOR
