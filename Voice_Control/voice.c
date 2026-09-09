#include "asr.h"
extern "C"{ void * __dso_handle = 0 ;}
#include "setup.h"
#include "myLib/asr_event.h"

uint32_t snid;
void ASR_CODE();

//{speak:小蝶-清新女声,vol:10,speed:10,platform:haohaodada}
//{playid:10001,voice:}
//{playid:10002,voice:}

/*描述该功能...
*/
void ASR_CODE(){
  switch (snid) {
   case 1:
    digitalWrite(0,1);
    digitalWrite(1,1);
    digitalWrite(4,0);
    break;
   case 2:
    digitalWrite(0,0);
    digitalWrite(1,0);
    digitalWrite(4,0);
    break;
   case 3:
    digitalWrite(0,1);
    digitalWrite(1,0);
    digitalWrite(4,0);
    break;
   case 4:
    digitalWrite(0,0);
    digitalWrite(1,1);
    digitalWrite(4,0);
    break;
   case 5:
    digitalWrite(4,1);
    break;
  }

}

void setup()
{
  set_wakeup_forever();
  //{ID:1,keyword:"命令词",ASR:"前进",ASRTO:"前进"}
  //{ID:2,keyword:"命令词",ASR:"后退",ASRTO:"后退"}
  //{ID:3,keyword:"命令词",ASR:"左转",ASRTO:"左转"}
  //{ID:4,keyword:"命令词",ASR:"右转",ASRTO:"右转"}
  //{ID:5,keyword:"命令词",ASR:"停",ASRTO:"停"}
  setPinFun(4,FIRST_FUNCTION);
  pinMode(4,output);
  setPinFun(0,FIRST_FUNCTION);
  pinMode(0,output);
  dpmu_set_adio_reuse(PA0,DIGITAL_MODE);
  setPinFun(1,FIRST_FUNCTION);
  pinMode(1,output);
  dpmu_set_adio_reuse(PA1,DIGITAL_MODE);
}


/*语音模块
PA4   GPIO21
PA0   GPIO1
PA1   GPIO2
*/