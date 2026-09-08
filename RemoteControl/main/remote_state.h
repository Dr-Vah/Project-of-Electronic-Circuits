#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define REMOTE_TIMEOUT_US 300000
typedef struct {
    uint32_t token, sequence;
    int64_t received;
    float x, y, turn;
} remote_state_t;

static inline void remote_stop(remote_state_t *s) { *s = (remote_state_t){0}; }
static inline void remote_expire(remote_state_t *s, int64_t now) {
    if (s->token && now - s->received >= REMOTE_TIMEOUT_US) remote_stop(s);
}
static inline bool remote_claim(remote_state_t *s, uint32_t token, int64_t now) {
    remote_expire(s, now);
    if (s->token || !token) return false;
    *s = (remote_state_t){.token=token, .received=now};
    return true;
}
static inline bool remote_command(remote_state_t *s, uint32_t token,
        uint32_t sequence, float x, float y, float turn, int64_t now) {
    remote_expire(s, now);
    if (!token || token != s->token || sequence <= s->sequence ||
        !isfinite(x) || !isfinite(y) || !isfinite(turn) ||
        fabsf(x)>1 || fabsf(y)>1 || fabsf(turn)>1) return false;
    float length = sqrtf(x*x+y*y);
    if (length > 1) { x/=length; y/=length; }
    s->sequence=sequence; s->received=now;
    s->x=x; s->y=y; s->turn=turn;
    return true;
}
