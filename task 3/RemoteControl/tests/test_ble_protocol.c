#include <assert.h>
#include <stdio.h>
#include "../main/ble_protocol.h"
#include "../main/remote_state.h"
static ble_command_t parse(const char *s) { return ble_parse(s,strlen(s)); }
int main(void) {
    assert(parse("#ARM!").kind==BLE_ARM);
    assert(parse("#STOP!").kind==BLE_STOP);
    assert(parse("#FACE:-1!").kind==BLE_FACE&&parse("#FACE:-1!").face==-1);
    assert(parse("#FACE:11!").kind==BLE_FACE&&parse("#FACE:11!").face==11);
    const char *invalid[]={"", "#", "#1,2,3!", "#nan,0,0!", "#0,inf,0!",
        "#0,0,!", "#0,,0!", "#0,0,0!junk", "#0,0,0!!", "#0x1,0,0!",
        "# 0,0,0!", "#0,0,0", "#ARM!x", "#0.121,0,0!", "#0,0,0.61!",
        "#FACE:12!","#FACE:-2!","#FACE:1.0!","#FACE:!","#FACE:11!junk","#FACE:11111111!"};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);++i)assert(parse(invalid[i]).kind==BLE_INVALID);
    assert(ble_parse("#0,0,0!\0x",10).kind==BLE_INVALID);
    assert(ble_parse(NULL,1).kind==BLE_INVALID);
    ble_command_t c=parse("#-0.12,0.12,-0.60!");
    assert(c.kind==BLE_DRIVE&&c.x==-1&&c.y==1&&c.turn==-1);
    remote_state_t s={0};
    assert(remote_claim(&s,42,0));
    assert(remote_command(&s,42,1,c.x,c.y,c.turn,1));
    assert(fabsf(hypotf(s.x,s.y)-1)<0.00001f);
    assert(!remote_claim(&s,43,2));
    remote_expire(&s,REMOTE_TIMEOUT_US+1);
    assert(!remote_command(&s,42,2,c.x,c.y,c.turn,REMOTE_TIMEOUT_US+2));
    assert(!s.token&&s.x==0&&s.turn==0);
    puts("BLE protocol and shared watchdog tests passed");
}
