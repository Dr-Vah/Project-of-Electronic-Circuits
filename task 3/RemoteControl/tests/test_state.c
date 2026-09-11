#include <assert.h>
#include <stdio.h>
#include "../main/remote_state.h"
int main(void) {
 remote_state_t s={0};
 assert(!remote_command(&s,1,1,1,0,0,0));
 assert(remote_claim(&s,42,1000));
 assert(!remote_claim(&s,43,2000));
 assert(!remote_command(&s,43,1,1,0,0,2000));
 assert(remote_command(&s,42,1,1,1,.5f,2000));
 assert(fabsf(hypotf(s.x,s.y)-1)<.0001f);
 assert(!remote_command(&s,42,1,0,0,0,3000));
 assert(!remote_command(&s,42,2,NAN,0,0,3000));
 assert(!remote_command(&s,42,2,2,0,0,3000));
 assert(s.received==2000);
 remote_expire(&s,301999); assert(s.token==42);
 remote_expire(&s,302000); assert(!s.token && !s.x);
 assert(!remote_command(&s,42,3,1,0,0,303000));
 assert(remote_claim(&s,43,304000));
 assert(!remote_command(&s,42,4,1,0,0,305000));
 assert(remote_command(&s,43,1,0,0,1,305000));
 remote_stop(&s); assert(!s.token&&!s.turn);
 assert(!remote_command(&s,43,2,0,0,1,306000));
 puts("PASS: ownership, expiry, stale commands, stop, bounds, NaN, diagonal limit");
}
