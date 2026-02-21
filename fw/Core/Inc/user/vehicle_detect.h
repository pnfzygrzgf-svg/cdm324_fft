#ifndef VEHICLE_DETECT_H_
#define VEHICLE_DETECT_H_

#include "user/defines.h"

/* State machine states */
typedef enum { VD_IDLE, VD_DETECTING, VD_COOLDOWN } vd_state_t;

/* Tuning parameters */
#define VD_SPEED_THRESHOLD_RAW  635     /* ~5 km/h as raw frequency */
#define VD_END_COUNT            5       /* Frames without signal until event ends (~143ms) */
#define VD_MIN_DURATION_FRAMES  3       /* Minimum duration for valid event (~86ms) */
#define VD_COOLDOWN_FRAMES      10      /* Cooldown after event (~286ms) */
#define VD_LENGTH_PKW_MAX_M     8.0f    /* Max length for PKW classification */
#define VD_FRAME_DURATION_S     0.0286f /* Duration of one frame in seconds (1024/35714) */
#define VD_RAW_TO_KMH           0.2262295f

/* Event data */
typedef struct {
	uint32_t event_number;
	uint32_t timestamp_ms;
	uint16_t peak_speed_raw;
	uint16_t duration_frames;
	float    speed_kmh;
	float    length_m;
	char     type[4];
} vd_event_t;

/* Prototypes */
void vd_init(void);
BOOL vd_process_frame(uint16_t raw_peak_freq);
vd_event_t* vd_get_last_event(void);
uint32_t vd_get_vehicle_count(void);

#endif
