#include "user/vehicle_detect.h"
#include "stm32f3xx_hal.h"
#include <string.h>

/* State machine */
static vd_state_t vd_state = VD_IDLE;
static uint32_t vd_vehicle_count = 0;
static vd_event_t vd_last_event;

/* Detection state */
static uint16_t vd_no_signal_counter = 0;
static uint16_t vd_cooldown_counter = 0;
static uint16_t vd_cur_peak_speed_kmh = 0;
static uint16_t vd_cur_duration_frames = 0;
static uint32_t vd_cur_timestamp_ms = 0;


void vd_init(void)
{
	vd_state = VD_IDLE;
	vd_vehicle_count = 0;
	memset(&vd_last_event, 0, sizeof(vd_last_event));
}

BOOL vd_process_frame(uint16_t speed_kmh)
{
	BOOL signal_present = (speed_kmh >= VD_SPEED_THRESHOLD_KMH) ? TRUE : FALSE;

	switch (vd_state)
	{
		case VD_IDLE:
			if (signal_present != FALSE)
			{
				vd_state = VD_DETECTING;
				vd_cur_timestamp_ms = HAL_GetTick();
				vd_cur_peak_speed_kmh = speed_kmh;
				vd_cur_duration_frames = 1;
				vd_no_signal_counter = 0;
			}
			break;

		case VD_DETECTING:
			vd_cur_duration_frames++;

			if (speed_kmh > vd_cur_peak_speed_kmh)
			{
				vd_cur_peak_speed_kmh = speed_kmh;
			}

			if (signal_present == FALSE)
			{
				vd_no_signal_counter++;
			}
			else
			{
				vd_no_signal_counter = 0;
			}

			if (vd_no_signal_counter >= VD_END_COUNT)
			{
				vd_cur_duration_frames -= vd_no_signal_counter;

				if (vd_cur_duration_frames >= VD_MIN_DURATION_FRAMES)
				{
					vd_vehicle_count++;
					vd_last_event.event_number = vd_vehicle_count;
					vd_last_event.timestamp_ms = vd_cur_timestamp_ms;
					vd_last_event.speed_kmh = vd_cur_peak_speed_kmh;
					vd_last_event.duration_frames = vd_cur_duration_frames;

					float duration_s = (float)vd_cur_duration_frames * VD_FRAME_DURATION_S;
					float speed_ms = (float)vd_cur_peak_speed_kmh / 3.6f;
					vd_last_event.length_m = speed_ms * duration_s;

					if (vd_last_event.length_m < VD_LENGTH_PKW_MAX_M)
					{
						memcpy(vd_last_event.type, "PKW", 4);
					}
					else
					{
						memcpy(vd_last_event.type, "LKW", 4);
					}

					vd_state = VD_COOLDOWN;
					vd_cooldown_counter = 0;
					return TRUE;
				}

				vd_state = VD_COOLDOWN;
				vd_cooldown_counter = 0;
			}
			break;

		case VD_COOLDOWN:
			vd_cooldown_counter++;
			if (vd_cooldown_counter >= VD_COOLDOWN_FRAMES)
			{
				vd_state = VD_IDLE;
			}
			break;
	}

	return FALSE;
}

vd_event_t* vd_get_last_event(void)
{
	return &vd_last_event;
}

uint32_t vd_get_vehicle_count(void)
{
	return vd_vehicle_count;
}
