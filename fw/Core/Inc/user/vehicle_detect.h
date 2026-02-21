#ifndef VEHICLE_DETECT_H_
#define VEHICLE_DETECT_H_

#include "user/defines.h"

/* State machine states */
typedef enum { VD_IDLE, VD_DETECTING, VD_COOLDOWN } vd_state_t;

/* Tuning parameters */
#define VD_SPEED_THRESHOLD_KMH  10      /* Minimum speed in km/h */
#define VD_END_COUNT            5       /* Frames without signal until event ends (~143ms) */
#define VD_MIN_DURATION_FRAMES  3       /* Minimum duration for valid event (~86ms) */
#define VD_COOLDOWN_FRAMES      10      /* Cooldown after event (~286ms) */
#define VD_LENGTH_PKW_MAX_M     8.0f    /* Max length for PKW classification */
#define VD_FRAME_DURATION_S     0.0286f /* Duration of one frame in seconds (1024/35714) */

/* Event data */
typedef struct {
	uint32_t event_number;
	uint32_t timestamp_ms;
	uint16_t speed_kmh;
	uint16_t duration_frames;
	float    length_m;
	char     type[4];
} vd_event_t;

/* Prototypes */
void vd_init(void);
BOOL vd_process_frame(uint16_t speed_kmh);
vd_event_t* vd_get_last_event(void);
uint32_t vd_get_vehicle_count(void);

#endif
