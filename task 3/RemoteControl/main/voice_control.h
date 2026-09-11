#pragma once
#include <stdbool.h>
#include <stdint.h>

/* PA4 -> GPIO21 (stop), PA0 -> GPIO1, PA1 -> GPIO2. */
#define VOICE_STOP_GPIO 21
#define VOICE_PA0_GPIO 1
#define VOICE_PA1_GPIO 2
#define VOICE_STABLE_US 30000
#define VOICE_SPEED_MPS 0.06f
#define VOICE_TURN_RAD_S 0.25f

typedef struct {
    bool initialized, armed, active;
    unsigned candidate, stable, command;
    int64_t since;
} voice_control_t;

/* A phone takeover/stop requires a fresh spoken stop before voice can resume. */
static inline void voice_cancel(voice_control_t *v) {
    v->active=false;
    v->armed=false;
}

/* Return true only on a newly observed stop edge. A held stop allows phone use.
 * Stop acts immediately; direction changes must settle for 30 ms. */
static inline bool voice_sample(voice_control_t *v, bool stop, bool pa0,
                                bool pa1, int64_t now, bool phone_owned) {
    unsigned raw=stop ? 4u : ((unsigned)pa0 << 1) | (unsigned)pa1;
    if (!v->initialized) {
        v->initialized=true;
        v->candidate=v->stable=raw;
        v->since=now;
    }
    bool stop_edge=stop && v->stable!=4u;
    if (stop) {
        v->active=false;
        v->stable=4u;
    }
    if (raw!=v->candidate) {
        v->candidate=raw;
        v->since=now;
    }
    if (phone_owned) {
        if (now-v->since >= VOICE_STABLE_US) v->stable=raw;
        voice_cancel(v);
        return stop_edge;
    }
    if (now-v->since < VOICE_STABLE_US) return stop_edge;
    v->stable=raw;
    if (stop) {
        v->armed=true;
    } else if (v->armed) {
        v->stable=raw;
        v->command=raw;
        v->active=true;
    }
    return stop_edge;
}

static inline void voice_velocity(const voice_control_t *v, float *y, float *w) {
    *y=*w=0;
    if (!v->active) return;
    switch (v->command) {
    case 3: *y=VOICE_SPEED_MPS; break;
    case 0: *y=-VOICE_SPEED_MPS; break;
    case 2: *w=VOICE_TURN_RAD_S; break;
    case 1: *w=-VOICE_TURN_RAD_S; break;
    }
}
