#ifdef RGB_MATRIX_ENABLE
#include "shift.h"

#include "led_matrix.h"
#include "rgb_matrix.h"
#include "config_led.h"

// This table can be almost-automatically derived from ISSI3733_LED_MAP that is
// defined in config_led.h
//
// scan in the following equations refers to the scan variable of ISSI3733_LED_MAP
//   col = (uint8_t)(scan / 8)
//   row = (uint8_t)(scan % 8)
//
// You can calculate the (0-244, 0-64) x/y values from the x/y values defined in
// ISSI3733_LED_MAP with the following formula:
//   uint8_t rgb_x = ((ISSI3733_LED_MAP[i].x - MIN_X) / (MAX_X - MIN_X)) * 224;
//   uint8_t rgb_y = ((ISSI3733_LED_MAP[i].y - MIN_Y) / (MAX_Y - MIN_Y)) * 64; //TODO: 64 - this?
// Where the min/max vars are the minimum and maximum "bounds" of x/y values
// present in ISSI3733_LED_MAP
//
// The row/col values need to be manually tweaked though, compensating for the
// "empty" cells that are a product of larger keys
//

led_config_t g_led_config = {
  {

    {    0,		1,	2,	4,	5,	6,	7,	9,	11,	NO_LED,	14,	15,	NO_LED	},
    {    21,	22,	3,	26,	28,	29,	8,	10,	12,	17,	18,	19,	NO_LED	},
    {    39,	23,	24,	27,	45,	30,	31,	33,	34,	35,	36,	37,	38		},
    {    57,	40,	25,	43,	46,	47,	32,	50,	51,	53,	54,	55,	56		},
    {    74,	41,	42,	44,	63,	48,	49,	67,	52,	70,	71,	72,	73		},
    {    91,	58,	60,	61,	64,	65,	66,	83,	68,	86,	87,	88,	90		},
    {    92,	59,	76,	62,	79,	81,	82,	84,	69,	89,	101,97,	98		},
    {    93,	75,	77,	78,	94,	80,	95,	96,	85,	99,	100,NO_LED,	NO_LED  }
  },
  {                 // key map for default layout
	  {6, 61 },     // esc
	  {20, 61 },    // F1
	  {31, 61 },    // F2
	  {43, 61 },    // F3
	  {54, 61 },    // F4
	  {69, 61 },    // F5
	  {80, 61 },    // F6
	  {91, 61 },    // F7
	  {103, 61 },   // F8
	  {117, 61 },   // F9
	  {129, 61 },   // F10
	  {140, 61 },   // F11
	  {152, 61 },   // F12
	  {183, 61 },   // home
	  {206, 61 },   // page up
	  {166, 61 },   // del
	  {194, 61 },   // end
	  {217, 61 },   // page down
	  {6, 49 },     // `~
	  {17, 49 },    // 1
	  {29, 49 },    // 2
	  {40, 49 },    // 3
	  {51, 49 },    // 4
	  {63, 49 },    // 5
	  {74, 49 },    // 6
	  {86, 49 },    // 7
	  {97, 49 },    // 8
	  {109, 49 },   // 9
	  {120, 49 },   // 0
	  {132, 49 },   // -_
	  {143, 49 },   // =+
	  {160, 49 },   // backspace
	  {183, 49 },   // num lock
	  {194, 49 },   // numpad /
	  {206, 49 },   // numpad *
	  {217, 49 },   // pause
	  {9, 39 },     // tab
	  {23, 39 },    // Q
	  {34, 39 },    // W
	  {46, 39 },    // E
	  {57, 39 },    // R
	  {69, 39 },    // T
	  {80, 39 },    // Y
	  {91, 39 },    // U
	  {103, 39 },   // I
	  {114, 39 },   // O
	  {126, 39 },   // P
	  {137, 39 },   // [{
	  {149, 39 },   // ]}
	  {163, 39 },   // \|
	  {183, 39 },   // numpad 7
	  {194, 39 },   // numpad 8
	  {206, 39 },   // numpad 9
	  {217, 39 },   // numpad -
	  {10, 29 },    // cap lock
	  {26, 29 },    // A
	  {37, 29 },    // S
	  {49, 29 },    // D
	  {60, 29 },    // F
	  {71, 29 },    // G
	  {83, 29 },    // H
	  {94, 29 },    // J
	  {106, 29 },   // K
	  {117, 29 },   // L
	  {129, 29 },   // ;:
	  {140, 29 },   // '"
	  {159, 29 },   // enter
	  {183, 29 },   // numpad 4
	  {194, 29 },   // numpad 5
	  {206, 29 },   // numpad 6
	  {217, 29 },   // numpad +
	  {13, 20 },    // left shift
	  {31, 20 },    // Z
	  {43, 20 },    // X
	  {54, 20 },    // C
	  {66, 20 },    // V
	  {77, 20 },    // B
	  {89, 20 },    // N
	  {100, 20 },   // M
	  {112, 19 },   // ,<
	  {123, 20 },   // .>
	  {134, 20 },   // /?
	  {150, 20 },   // right shift
	  {183, 20 },   // numpad 1
	  {194, 20 },   // numpad 2
	  {206, 20 },   // numpad 3
	  {169, 17 },   // arrow up
	  {214, 12 },   // numpad enter
	  {7, 10 },     // left control
	  {21, 10 },    // left windows
	  {36, 10 },    // left alt
	  {79, 10 },    // space
	  {123, 10 },   // right alt
	  {140, 10 },   // Fn
	  {194, 10 },   // numpad 0
	  {206, 10 },   // numpad dot
	  {157, 7 },    // arrow left
	  {169, 7 },    // arrow down
	  {180, 7 },    // arrow right

	  // underglow
	  {1, 63 },     // top (left)
	  {6, 64 },     // top
	  {26, 64 },    // top
	  {35, 64 },    // top
	  {45, 64 },    // top
	  {54, 64 },    // top
	  {64, 64 },    // top
	  {74, 64 },    // top
	  {83, 64 },    // top
	  {93, 64 },    // top
	  {102, 64 },   // top
	  {112, 64 },   // top
	  {122, 64 },   // top
	  {131, 64 },   // top
	  {141, 64 },   // top
	  {150, 64 },   // top
	  {160, 64 },   // top
	  {170, 64 },   // top
	  {179, 64 },   // top
	  {189, 64 },   // top
	  {198, 64 },   // top
	  {218, 64 },   // top
	  {222, 63 },   // top (right)
	  {224, 58 },   // right (top)
	  {224, 51 },   // right
	  {224, 43 },   // right
	  {224, 35 },   // right
	  {224, 28 },   // right
	  {224, 20 },   // right
	  {224, 12 },   // right
	  {224, 5 },    // right (bottom)
	  {222, 0 },    // bottom (right)
	  {218, 0 },    // bottom
	  {208, 0 },    // bottom
	  {198, 0 },    // bottom
	  {189, 0 },    // bottom
	  {179, 0 },    // bottom
	  {170, 0 },    // bottom
	  {160, 0 },    // bottom
	  {150, 0 },    // bottom
	  {141, 0 },    // bottom
	  {131, 0 },    // bottom
	  {122, 0 },    // bottom
	  {112, 0 },    // bottom
	  {102, 0 },    // bottom
	  {93, 0 },     // bottom
	  {83, 0 },     // bottom
	  {74, 0 },     // bottom
	  {64, 0 },     // bottom
	  {54, 0 },     // bottom
	  {45, 0 },     // bottom
	  {35, 0 },     // bottom
	  {26, 0 },     // bottom
	  {16, 0 },     // bottom
	  {6, 0  },     // bottom
	  {1, 0  },     // bottom (left)
	  {0, 5  },     // left (bottom)
	  {0, 12 },     // left
	  {0, 20 },     // left
	  {0, 28 },     // left
	  {0, 35 },     // left
	  {0, 43 },     // left
	  {0, 51 },     // left
	  {0, 58 },     // left (top)

	  // indicator
	  {174, 61 },	// top      : num lock
	  {174, 58 },	// middle   : cap lock
	  {174, 55 }	// botom    : scroll lock
  },
  {
 // Key Lights (99) - LED_FLAG_KEYLIGHT, LED_FLAG_MODIFIER
		4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
		4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
		4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
		4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
		1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1, 4, 4, 4, 4, 4,
		1, 1, 1, 4, 1, 1, 4, 4, 4, 4, 4,

// Underglow/border LEDs (64) - LED_FLAG_UNDERGLOW
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2,

 // NCS Indicators (3)
		//0, 0, 0
		2, 2, 2
  }
};

#if defined(USB_LED_INDICATOR_ENABLE) || defined(DEDICATED_LED_INDICATOR_ENABLE)
void rgb_matrix_indicators_kb(void)
{
  led_matrix_indicators();
}
#endif // USB_LED_INDICATOR_ENABLE || DEDICATED_LED_INDICATOR_ENABLE

#endif
