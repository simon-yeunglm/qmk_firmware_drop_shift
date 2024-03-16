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

#ifdef RAW_ENABLE

#include "arm_atsam_protocol.h"
#include <string.h>
#include <raw_hid.h>
#include <version.h>


#define RECV_CUSTOM_RGB				(0)

#define COMPILE_VER QMK_KEYBOARD ":" QMK_KEYMAP

// MFT Test Protocol Definitions

#define RAWP_PROTOCOL_VERSION		0x00
#define RAWP_CAT_MFGTEST			0x01

#define RAWP_CMD_SET_RGB_MODE		0x01
#define RAWP_CMD_SET_ALL_RGBS		0x02
#define RAWP_CMD_SET_SPEC_RGBS		0x03
#define RAWP_CMD_READ_ALL_ADCS		0x04
#define RAWP_CMD_RET_GCR_INFO		0x05
#define RAWP_CMD_RET_FW_INFO		0x06
#define RAWP_CMD_RET_QMK_INFO		0x07

#if RECV_CUSTOM_RGB
// custom protocol for compact LED data
#define RAWP_CMD_SET_SPEC_RGBS_EX1	0x08	// the 1st packet which encoded cap/num/scroll lock state
#define RAWP_CMD_SET_SPEC_RGBS_EX2	0x09	// the 2nd to 5th packets
#endif

enum {
	RET_Success,
	RET_Fail,
	RET_Inactive,
	RET_UnknownCommand,
	RET_LEDNotFound
};

typedef struct {
	uint8_t		prot_ver;		// Protocol Version
	uint8_t		cat;			// Category
	uint8_t		cmd;			// Command
	uint8_t		data[61];		// Parameters
} __attribute__((packed)) sRawp_command_t;

typedef struct {
	uint8_t		prot_ver;		// Protocol Version
	uint8_t		cat;			// Category
	uint8_t		cmd;			// Command
	uint8_t		result;			// Result
	uint8_t		data[60];		// Parameters
} __attribute__((packed)) sRawp_response_t;

static uint8_t	raw_hid_todo = 0;
static uint8_t	*raw_hid_databuf = 0;

static void set_all_leds(uint8_t area, uint8_t r, uint8_t g, uint8_t b);
static bool set_one_led_id(uint8_t id, uint8_t r, uint8_t g, uint8_t b);

#if RECV_CUSTOM_RGB
void recv_custom_rgb(uint8_t packetIdx, const uint8_t* rgb_data)
{
	// set key color
	static const uint8_t numKeys[	6]= {18, 18, 18, 17, 17, 11};
	static const uint8_t numKeysAcc[6]= {	0,
											numKeys[0],
											numKeys[0] + numKeys[1],
											numKeys[0] + numKeys[1] + numKeys[2],
											numKeys[0] + numKeys[1] + numKeys[2] + numKeys[3],
											numKeys[0] + numKeys[1] + numKeys[2] + numKeys[3] + numKeys[4],	};
	int num		= numKeys[		packetIdx];
	int idxStart= numKeysAcc[	packetIdx];
	for(int i=0; i<num; ++i)
	{
		int idx= idxStart + i;
		int offset= i * 3;
		uint8_t r= rgb_data[offset  ];
		uint8_t g= rgb_data[offset+1];
		uint8_t b= rgb_data[offset+2];
		led_set_one_rawp(idx, r, g, b);
	}

	// set left/right edge LED color
	const uint8_t leftEdgeLEDIdxStart = 161;	// descending
	const uint8_t rightEdgeLEDIdxStart= 123;	// ascending
	int offsetLeftLED	= num * 3;
	int offsetRightLED	= offsetLeftLED + 3;
	led_set_one_rawp(leftEdgeLEDIdxStart - packetIdx, rgb_data[offsetLeftLED ], rgb_data[offsetLeftLED +1], rgb_data[offsetLeftLED +2]);
	led_set_one_rawp(rightEdgeLEDIdxStart+ packetIdx, rgb_data[offsetRightLED], rgb_data[offsetRightLED+1], rgb_data[offsetRightLED+2]);
}

void mix_rgb(uint8_t r0, uint8_t g0, uint8_t b0, uint8_t r1, uint8_t g1, uint8_t b1, uint8_t* r_mix, uint8_t* g_mix, uint8_t* b_mix)
{
	*r_mix= (r0 + r1) / 2;
	*g_mix= (g0 + g1) / 2;
	*b_mix= (b0 + b1) / 2;
}

void recv_custom_rgb_pkt_0(const uint8_t* rgb_data, uint8_t lockStates)
{
	// set other bottom row LED
	uint8_t r_lock= 0;
	uint8_t g_lock= 0;
	uint8_t b_lock= 0;
	{
		// Esc
		int 	offset0	= 0;
		uint8_t r0		= rgb_data[offset0  ];
		uint8_t g0		= rgb_data[offset0+1];
		uint8_t b0		= rgb_data[offset0+2];
		led_set_one_rawp(162, r0, g0, b0);
		led_set_one_rawp( 99, r0, g0, b0);
		led_set_one_rawp(100, r0, g0, b0);
		
		// F1
		int 	offset1	= 3;
		uint8_t r1		= rgb_data[offset1  ];
		uint8_t g1		= rgb_data[offset1+1];
		uint8_t b1		= rgb_data[offset1+2];

		// F2
		offset0	= 6;
		r0		= rgb_data[offset0  ];
		g0		= rgb_data[offset0+1];
		b0		= rgb_data[offset0+2];

		uint8_t r_mix;
		uint8_t g_mix;
		uint8_t b_mix;
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(101, r_mix, g_mix, b_mix);
		
		// F3
		offset1	= 9;
		r1		= rgb_data[offset1  ];
		g1		= rgb_data[offset1+1];
		b1		= rgb_data[offset1+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(102, r_mix, g_mix, b_mix);
		led_set_one_rawp(103, r1, g1, b1);

		// F4
		offset0	= 12;
		r0		= rgb_data[offset0  ];
		g0		= rgb_data[offset0+1];
		b0		= rgb_data[offset0+2];
		led_set_one_rawp(104, r0, g0, b0);

		// F5
		offset1	= 15;
		r1		= rgb_data[offset1  ];
		g1		= rgb_data[offset1+1];
		b1		= rgb_data[offset1+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(105, r_mix, g_mix, b_mix);

		// F6
		offset0	= 18;
		r0		= rgb_data[offset0  ];
		g0		= rgb_data[offset0+1];
		b0		= rgb_data[offset0+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(106, r_mix, g_mix, b_mix);

		// F7
		offset1	= 21;
		r1		= rgb_data[offset1  ];
		g1		= rgb_data[offset1+1];
		b1		= rgb_data[offset1+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(107, r_mix, g_mix, b_mix);
		led_set_one_rawp(108, r1, g1, b1);

		// F8
		offset0	= 24;
		r0		= rgb_data[offset0  ];
		g0		= rgb_data[offset0+1];
		b0		= rgb_data[offset0+2];
		led_set_one_rawp(109, r0, g0, b0);

		// F9
		offset1	= 27;
		r1		= rgb_data[offset1  ];
		g1		= rgb_data[offset1+1];
		b1		= rgb_data[offset1+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(110, r_mix, g_mix, b_mix);

		// F10
		offset0	= 30;
		r0		= rgb_data[offset0  ];
		g0		= rgb_data[offset0+1];
		b0		= rgb_data[offset0+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(111, r_mix, g_mix, b_mix);
		led_set_one_rawp(112, r0, g0, b0);

		// F11
		offset1	= 33;
		r1		= rgb_data[offset1  ];
		g1		= rgb_data[offset1+1];
		b1		= rgb_data[offset1+2];
		led_set_one_rawp(113, r1, g1, b1);

		// F12
		offset0	= 36;
		r0		= rgb_data[offset0  ];
		g0		= rgb_data[offset0+1];
		b0		= rgb_data[offset0+2];
		led_set_one_rawp(114, r0, g0, b0);

		// Del
		offset1	= 39;
		r1		= rgb_data[offset1  ];
		g1		= rgb_data[offset1+1];
		b1		= rgb_data[offset1+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(115, r_mix, g_mix, b_mix);

		// Home
		offset0	= 42;
		r0		= rgb_data[offset0  ];
		g0		= rgb_data[offset0+1];
		b0		= rgb_data[offset0+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(116, r_mix, g_mix, b_mix);
		led_set_one_rawp(117, r_mix, g_mix, b_mix);
		r_lock= r_mix;
		g_lock= g_mix;
		b_lock= b_mix;

		// End
		offset1	= 45;
		r1		= rgb_data[offset1  ];
		g1		= rgb_data[offset1+1];
		b1		= rgb_data[offset1+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(118, r_mix, g_mix, b_mix);

		// PgUp
		offset0	= 48;
		r0		= rgb_data[offset0  ];
		g0		= rgb_data[offset0+1];
		b0		= rgb_data[offset0+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(119, r_mix, g_mix, b_mix);

		// PgDn
		offset1	= 51;
		r1		= rgb_data[offset1  ];
		g1		= rgb_data[offset1+1];
		b1		= rgb_data[offset1+2];
		led_set_one_rawp(120, r1, g1, b1);
		led_set_one_rawp(121, r1, g1, b1);
		led_set_one_rawp(122, r1, g1, b1);
	}

	// update lock LED
	{
		bool numLock	= (lockStates & (1 << 0)) != 0;
		bool capLock	= (lockStates & (1 << 1)) != 0;
		bool scrollLock	= (lockStates & (1 << 2)) != 0;
		
		led_set_one_rawp(163, r_lock * numLock		, g_lock * numLock		, b_lock * numLock		);
		led_set_one_rawp(164, r_lock * capLock		, g_lock * capLock		, b_lock * capLock		);
		led_set_one_rawp(165, r_lock * scrollLock	, g_lock * scrollLock	, b_lock * scrollLock	);
	}
}

void recv_custom_rgb_pkt_3(const uint8_t* rgb_data)
{
	// botom edge space bar 1st LED
	int offset= 57;
	uint8_t r= rgb_data[offset  ];
	uint8_t g= rgb_data[offset+1];
	uint8_t b= rgb_data[offset+2];
	led_set_one_rawp(149, r, g, b);
}

void recv_custom_rgb_pkt_4(const uint8_t* rgb_data)
{
	// botom edge lower right corner
	int offset= 57;
	uint8_t r= rgb_data[offset  ];
	uint8_t g= rgb_data[offset+1];
	uint8_t b= rgb_data[offset+2];
	led_set_one_rawp(129, r, g, b);
	led_set_one_rawp(130, r, g, b);
	led_set_one_rawp(131, r, g, b);
}

void recv_custom_rgb_pkt_5(const uint8_t* rgb_data)
{
	// bottom edge space bar LED 2nd to 8th
	int numSpaceBarLED= 7;
	for(int i=0; i<numSpaceBarLED; ++i)
	{
		int 	offset	= (13 + i) * 3;
		uint8_t r		= rgb_data[offset  ];
		uint8_t g		= rgb_data[offset+1];
		uint8_t b		= rgb_data[offset+2];
		int 	LED_idx	= 148 - i;
		led_set_one_rawp(LED_idx, r, g, b);
	}

	// set other bottom row LED
	{
		// left control
		int 	offset0	= 0;
		uint8_t r0		= rgb_data[offset0  ];
		uint8_t g0		= rgb_data[offset0+1];
		uint8_t b0		= rgb_data[offset0+2];
		led_set_one_rawp(155, r0, g0, b0);
		led_set_one_rawp(154, r0, g0, b0);
		led_set_one_rawp(153, r0, g0, b0);
		
		// left win
		int 	offset1	= 3;
		uint8_t r1		= rgb_data[offset1  ];
		uint8_t g1		= rgb_data[offset1+1];
		uint8_t b1		= rgb_data[offset1+2];

		uint8_t r_mix;
		uint8_t g_mix;
		uint8_t b_mix;
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(152, r_mix, g_mix, b_mix);

		// left alt
		offset0			= 6;
		r0				= rgb_data[offset0  ];
		g0				= rgb_data[offset0+1];
		b0				= rgb_data[offset0+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(151, r_mix, g_mix, b_mix);
		led_set_one_rawp(150, r0, g0, b0);

		// right alt
		offset0			= 12;
		r0				= rgb_data[offset0  ];
		g0				= rgb_data[offset0+1];
		b0				= rgb_data[offset0+2];
		led_set_one_rawp(141, r0, g0, b0);

		// Fn
		offset1			= 15;
		r1				= rgb_data[offset1  ];
		g1				= rgb_data[offset1+1];
		b1				= rgb_data[offset1+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(140, r_mix, g_mix, b_mix);
		led_set_one_rawp(139, r1, g1, b1);
		
		// arrow left
		offset0			= 24;
		r0				= rgb_data[offset0  ];
		g0				= rgb_data[offset0+1];
		b0				= rgb_data[offset0+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(138, r_mix, g_mix, b_mix);
		
		// arrow down
		offset1			= 27;
		r1				= rgb_data[offset1  ];
		g1				= rgb_data[offset1+1];
		b1				= rgb_data[offset1+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(137, r_mix, g_mix, b_mix);
		led_set_one_rawp(136, r1, g1, b1);
		
		// arrow right
		offset0			= 27;
		r0				= rgb_data[offset0  ];
		g0				= rgb_data[offset0+1];
		b0				= rgb_data[offset0+2];
		led_set_one_rawp(135, r0, g0, b0);
		
		// numpad 0
		offset0			= 18;
		r1				= rgb_data[offset1  ];
		g1				= rgb_data[offset1+1];
		b1				= rgb_data[offset1+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(134, r_mix, g_mix, b_mix);
		
		// numpad del
		offset0			= 21;
		r0				= rgb_data[offset0  ];
		g0				= rgb_data[offset0+1];
		b0				= rgb_data[offset0+2];
		mix_rgb(r0, g0, b0, r1, g1, b1, &r_mix, &g_mix, &b_mix);
		led_set_one_rawp(133, r_mix, g_mix, b_mix);
		led_set_one_rawp(132, r0, g0, b0);
	}
}
#endif

void raw_hid_receive(uint8_t *data, uint8_t length)
{
	sRawp_command_t * pcmd = (sRawp_command_t *) data;
	sRawp_response_t  resp;
	uint8_t		result = RET_Fail;

	if( length != RAW_EPSIZE )
		return;		// Whaa?  Shouldn't happen

	if( (pcmd->prot_ver != RAWP_PROTOCOL_VERSION) || (pcmd->cat != RAWP_CAT_MFGTEST) )
		return;

	memset((void *)&resp, 0, sizeof(resp));
	resp.prot_ver = pcmd->prot_ver;
	resp.cat = pcmd->cat;
	resp.cmd = pcmd->cmd;

#if RECV_CUSTOM_RGB
	bool needResponse= true;
#endif
	switch(pcmd->cmd)
	{
		case RAWP_CMD_SET_RGB_MODE:
			if(pcmd->data[0] > 0)
			{
				led_mfg_test_mode = LED_MFG_TEST_MODE_RAWP;
			}
			else
			{
				led_mfg_test_mode = LED_MFG_TEST_MODE_OFF;
			}
			result = RET_Success;
			break;

		case RAWP_CMD_SET_ALL_RGBS:
			set_all_leds(pcmd->data[0], pcmd->data[1], pcmd->data[2], pcmd->data[3]);
			if(pcmd->data[4] != 0)
			{
				usb_gcr_auto = 0;	// Manual GCR setting
				gcr_desired = (pcmd->data[4] > LED_GCR_MAX ? LED_GCR_MAX : pcmd->data[4]);
			}
			else
			{
				usb_gcr_auto = 1;
			}
			result = RET_Success;
			break;

		case RAWP_CMD_SET_SPEC_RGBS:
		{
			uint8_t i, num = pcmd->data[0];
			uint8_t id, r, g, b;

			if(num>0 && num<=15)
			{
				result = RET_Success;
				for(i=0; i<num; i++)
				{
					id = pcmd->data[i*4+1];
					r = pcmd->data[i*4+2];
					g = pcmd->data[i*4+3];
					b = pcmd->data[i*4+4];
					if(id == 0)
					{
						set_all_leds(LED_MODE_NORMAL, r, g, b);
					}
					else
					{
						if( set_one_led_id(id, r, g, b) != true )
						{
							// Flag at least one LED ID not found
							result = RET_LEDNotFound;
						}
					}
				}
			}
			else
			{
				result = RET_Fail;
				break;
			}
			break;
		}
#if RECV_CUSTOM_RGB
		case RAWP_CMD_SET_SPEC_RGBS_EX1:
		{
			needResponse= false;
			recv_custom_rgb(0, pcmd->data + 1);
			recv_custom_rgb_pkt_0(pcmd->data + 1, pcmd->data[0]);
			break;
		}
		case RAWP_CMD_SET_SPEC_RGBS_EX2:
		{
			needResponse= false;
			int pktIdx= pcmd->data[0];
			if (pktIdx < 6)
			{
				recv_custom_rgb(pktIdx, pcmd->data + 1);
				if (pktIdx == 3)
					recv_custom_rgb_pkt_3(pcmd->data + 1);
				else if (pktIdx == 4)
					recv_custom_rgb_pkt_4(pcmd->data + 1);
				else if (pktIdx == 5)
					recv_custom_rgb_pkt_5(pcmd->data + 1);
			}
			break;
		}
#endif
		case RAWP_CMD_READ_ALL_ADCS:
			raw_hid_todo = RAWP_CMD_READ_ALL_ADCS;
			raw_hid_databuf = data;
			return;		// do not send any packet - defer to execute in foreground

		case RAWP_CMD_RET_GCR_INFO:
			resp.data[0] = ((uint16_t)g_v_5v_avg) & 0xff;
			resp.data[1] = ((uint16_t)g_v_5v_avg) >> 8;
			resp.data[2] = gcr_desired & 0xff;
			resp.data[3] = gcr_desired >> 8;
			resp.data[4] = gcr_actual & 0xff;
			resp.data[5] = gcr_actual >> 8;
			resp.data[6] = gcr_actual_last & 0xff;
			resp.data[7] = gcr_actual_last >> 8;
			result = RET_Success;
			break;

		case RAWP_CMD_RET_FW_INFO:
		{
			uint8_t length=0;
			const uint8_t * str = udc_get_string_serial_name(&length);

			resp.data[0] = PRODUCT_ID & 0xff;
			resp.data[1] = PRODUCT_ID >> 8;
			resp.data[2] = DEVICE_VER & 0xff;
			resp.data[3] = DEVICE_VER >> 8;
			if( str != NULL )
			{
				strncpy((char *)(&resp.data[4]), (char *)str, ( length<16 ? length : 16 ) );
			}
			else
			{
				strncpy((char *)(&resp.data[4]), SERIAL_NUM, 16);
			}
			strncpy((char *)(&resp.data[20]), COMPILE_VER, 40);
			result = RET_Success;
			break;
		}

		case RAWP_CMD_RET_QMK_INFO:
			strncpy((char *)(&resp.data[0]), QMK_VERSION, 40);
			strncpy((char *)(&resp.data[40]), QMK_BUILDDATE, 20);
			result = RET_Success;
			break;

		default:
			// Unhandled command
			result = RET_UnknownCommand;
	}

	resp.result = result;

#if RECV_CUSTOM_RGB
	if (needResponse)
#endif
		raw_hid_send((uint8_t *)&resp, sizeof(resp));
}


void raw_hid_task(void) {
	sRawp_command_t * pcmd = (sRawp_command_t *) raw_hid_databuf;
	sRawp_response_t  resp;
	uint8_t		result = RET_Fail;

	if( (raw_hid_todo == RAWP_CMD_READ_ALL_ADCS) && (raw_hid_databuf != NULL) )
	{
		uint8_t mask = pcmd->data[0];
		uint8_t  i, count=0;
		uint16_t val;

		memset((void *)&resp, 0, sizeof(resp));
		resp.prot_ver = pcmd->prot_ver;
		resp.cat = pcmd->cat;
		resp.cmd = pcmd->cmd;

		for(i=0; i<ADC_NUM_INDICES; i++)
		{
			if( mask & 0x01 )
			{
				val = adc_get(i);
				if( val != (uint16_t)ADC_NA )
				{
					resp.data[count*3 +1] = i;
					resp.data[count*3 +2] = val & 0xff;
					resp.data[count*3 +3] = val >> 8;
					count++;
				}
			}
			mask >>= 1;
		}
		resp.data[0] = count;
		result = RET_Success;

		resp.result = result;
		raw_hid_send((uint8_t *)&resp, sizeof(resp));

		raw_hid_todo = 0;
		raw_hid_databuf = 0;
	}
}


static void set_all_leds(uint8_t mode, uint8_t r, uint8_t g, uint8_t b)
{
	for (uint8_t i = 0; i < ISSI3733_LED_COUNT; i++)
	{
		if(  (mode == LED_MODE_NORMAL) ||
			((mode == LED_MODE_KEYS_ONLY) && !(LED_IS_EDGE(led_map[i].scan) || LED_IS_INDICATOR(led_map[i].scan)))  ||
			((mode == LED_MODE_NON_KEYS_ONLY) && (LED_IS_EDGE(led_map[i].scan) || LED_IS_INDICATOR(led_map[i].scan)))  ||
			((mode == LED_MODE_INDICATORS_ONLY) && LED_IS_INDICATOR(led_map[i].scan))
		  )
		{
			led_set_one_rawp(i, r, g, b);
		}
	}
}

static bool set_one_led_id(uint8_t id, uint8_t r, uint8_t g, uint8_t b)
{
	for (uint8_t i = 0; i < ISSI3733_LED_COUNT; i++)
	{
		if(id == led_map[i].id)
		{
			led_set_one_rawp(i, r, g, b);
			return(true);
		}
	}
	return(false);
}
#endif // RAW_ENABLE
