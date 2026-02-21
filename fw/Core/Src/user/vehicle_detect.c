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
static uint16_t vd_cur_peak_speed_raw = 0;
static uint16_t vd_cur_duration_frames = 0;
static uint32_t vd_cur_timestamp_ms = 0;


/*! \fn     vd_init(void)
*   \brief  Initialize vehicle detection
*/
void vd_init(void)
{
	vd_state = VD_IDLE;
	vd_vehicle_count = 0;
	memset(&vd_last_event, 0, sizeof(vd_last_event));
}

/*! \fn     vd_process_frame(uint16_t raw_peak_freq)
*   \brief  Process one FFT frame for vehicle detection
*   \param  raw_peak_freq   Raw (unsmoothed) peak frequency from FFT
*   \return TRUE if a vehicle event just completed
*/
BOOL vd_process_frame(uint16_t raw_peak_freq)
{
	BOOL signal_present = (raw_peak_freq >= VD_SPEED_THRESHOLD_RAW) ? TRUE : FALSE;

	switch (vd_state)
	{
		case VD_IDLE:
			if (signal_present != FALSE)
			{
				/* Vehicle detected, start tracking */
				vd_state = VD_DETECTING;
				vd_cur_timestamp_ms = HAL_GetTick();
				vd_cur_peak_speed_raw = raw_peak_freq;
				vd_cur_duration_frames = 1;
				vd_no_signal_counter = 0;
			}
			break;

		case VD_DETECTING:
			vd_cur_duration_frames++;

			/* Track peak speed */
			if (raw_peak_freq > vd_cur_peak_speed_raw)
			{
				vd_cur_peak_speed_raw = raw_peak_freq;
			}

			/* Check for signal loss */
			if (signal_present == FALSE)
			{
				vd_no_signal_counter++;
			}
			else
			{
				vd_no_signal_counter = 0;
			}

			/* Event ends after VD_END_COUNT consecutive frames without signal */
			if (vd_no_signal_counter >= VD_END_COUNT)
			{
				/* Subtract the no-signal frames from duration */
				vd_cur_duration_frames -= vd_no_signal_counter;

				/* Check minimum duration */
				if (vd_cur_duration_frames >= VD_MIN_DURATION_FRAMES)
				{
					/* Compute event data */
					vd_vehicle_count++;
					vd_last_event.event_number = vd_vehicle_count;
					vd_last_event.timestamp_ms = vd_cur_timestamp_ms;
					vd_last_event.peak_speed_raw = vd_cur_peak_speed_raw;
					vd_last_event.duration_frames = vd_cur_duration_frames;
					vd_last_event.speed_kmh = (float)vd_cur_peak_speed_raw * VD_RAW_TO_KMH;

					/* Length = speed_m_per_s * duration_s */
					float duration_s = (float)vd_cur_duration_frames * VD_FRAME_DURATION_S;
					float speed_ms = vd_last_event.speed_kmh / 3.6f;
					vd_last_event.length_m = speed_ms * duration_s;

					/* Classify */
					if (vd_last_event.length_m < VD_LENGTH_PKW_MAX_M)
					{
						memcpy(vd_last_event.type, "PKW", 4);
					}
					else
					{
						memcpy(vd_last_event.type, "LKW", 4);
					}

					/* Enter cooldown and signal event */
					vd_state = VD_COOLDOWN;
					vd_cooldown_counter = 0;
					return TRUE;
				}

				/* Too short, discard and enter cooldown */
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

/*! \fn     vd_get_last_event(void)
*   \brief  Get pointer to last completed vehicle event
*   \return Pointer to event data
*/
vd_event_t* vd_get_last_event(void)
{
	return &vd_last_event;
}

/*! \fn     vd_get_vehicle_count(void)
*   \brief  Get total number of detected vehicles
*   \return Vehicle count
*/
uint32_t vd_get_vehicle_count(void)
{
	return vd_vehicle_count;
}
