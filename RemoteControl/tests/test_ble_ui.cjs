const assert=require('node:assert/strict');
const C=require('../miniprogram/utils/control');
const fs=require('node:fs'),vm=require('node:vm');
const zero={x:0,y:0,z:-1};
assert.equal(C.delta(359-1),-2);
assert.equal(C.delta(1-359),2);
assert(C.motion(zero,zero,359,1,1).every(n=>n===0));
const v=C.motion({x:1,y:-1},zero,90,0,1);
assert(v[0]>0&&v[1]>0&&v[2]<0);
assert(Math.hypot(v[0],v[1])<=0.12000001);
assert.equal(C.frame([-0.12,-0.12,-0.6]).length,19);
assert.equal(C.buffer(C.frame(v)).byteLength,C.frame(v).length);
assert.equal(C.normalize(C.SERVICE),'6E400001B5A3F393E0A9E50E24DCCA9E');
const firmware=fs.readFileSync(require.resolve('../main/ble_remote.c'),'utf8');
for(const [name,uuid] of [['SERVICE_UUID_128',C.SERVICE],['RX_CHAR_UUID_128',C.WRITE],['STATUS_UUID_128',C.STATUS]]) {
  const definition=firmware.match(new RegExp('#define '+name+'\\s+\\{([^}]+)\\}'))[1];
  const bytes=definition.match(/0x[0-9A-F]{2}/g).map(n=>parseInt(n,16));
  assert.equal(bytes.reverse().map(n=>n.toString(16).padStart(2,'0')).join('').toUpperCase(),C.normalize(uuid));
}
let calls=[],faults=0;
const driver=new C.Driver((s,done)=>calls.push({s,done}),()=>faults++);
assert(driver.arm());assert.equal(calls[0].s,'#ARM!');
driver.stop();calls[0].done();
assert.equal(calls[1].s,'#STOP!');assert(!driver.active);
calls[1].done();assert(driver.arm());calls[2].done();assert(driver.active);
driver.drive(v);driver.drive(v);assert.equal(calls.length,4);
driver.stop();calls[3].done();assert.equal(calls[4].s,'#STOP!');calls[4].done();
assert(!driver.active);driver.arm();calls[5].done(new Error('write failed'));
assert.equal(faults,1);assert(!driver.active);
const countAfterFailure=calls.length;
driver.stop();assert(!driver.arm());assert.equal(calls.length,countAfterFailure);

// Execute the actual page against a minimal wx adapter. Sensor loss and page
// backgrounding must release the drive latch; sensor recovery cannot re-arm it.
let page,accel,compass,tick,stops=0,closed=0;
const wx={
  onBLEConnectionStateChange(){},offBLEConnectionStateChange(){},
  onBLECharacteristicValueChange(){},offBLECharacteristicValueChange(){},
  onAccelerometerChange(fn){accel=fn;},onCompassChange(fn){compass=fn;},
  startAccelerometer(){},startCompass(){},stopAccelerometer(){},stopCompass(){},
  offAccelerometerChange(){},offCompassChange(){},closeBLEConnection(){closed++;},closeBluetoothAdapter(){}
};
vm.runInNewContext(fs.readFileSync(require.resolve('../miniprogram/pages/index/index.js'),'utf8'),{
  require:p=>p.endsWith('/video')?require('../miniprogram/utils/video'):C,Page:p=>page=p,wx,Date,Number,Math,Promise,Error,
  setInterval:fn=>{tick=fn;return 1;},clearInterval(){},setTimeout,clearTimeout
});
page.setData=function(d){Object.assign(this.data,d);};page.onLoad();page.onShow();
page.driver={active:false,arm(done){this.active=true;done();return true;},stop(){this.active=false;stops++;},drive(){}};
page.data.connected=true;page.deviceId='test';accel(zero);compass({direction:0});page.toggleTilt();
assert(page.data.driving);compass({direction:0,accuracy:-1});assert(!page.data.driving);
compass({direction:0});tick();assert(!page.data.driving);
page.toggleTilt();assert(page.data.driving);page.accelAt=Date.now()-301;tick();assert(!page.data.driving);
accel(zero);tick();assert(!page.data.driving);
// Compass is optional for translation, and cannot silently enable mid-press.
page.headingAt=0;page.toggleTilt();assert(page.data.driving&&page.data.tiltLatched&&!page.usingCompass);
compass({direction:90});tick();assert(page.data.driving&&!page.usingCompass);
compass({direction:90,accuracy:-1});assert(page.data.driving);
page.release();
// Opposite gravity-Z conventions are accepted at calibration, but flipping
// the phone during an active press still releases driving.
accel({x:0,y:0,z:1});page.toggleTilt();assert(page.data.driving);
accel(zero);tick();assert(!page.data.driving);
compass({direction:0});page.toggleTilt();assert(page.usingCompass);
page.headingAt=Date.now()-500;tick();assert(page.data.driving);
page.headingAt=Date.now()-1001;tick();assert(!page.data.driving);
accel(zero);
page.toggleTilt();assert(page.data.tiltLatched);
tick();assert(page.data.driving); // No touch needs to stay down.
page.turnStart({changedTouches:[{identifier:9}],currentTarget:{dataset:{sign:1}}});
page.turnEnd({changedTouches:[{identifier:9}]});assert(page.data.tiltLatched&&page.data.driving);
page.toggleTilt();assert(!page.data.tiltLatched&&!page.data.driving);
page.toggleTilt();assert(page.data.driving);page.onHide();assert(!page.data.driving);
assert(!page.data.tiltLatched);
assert.equal(closed,1);assert(stops>=3);
console.log('BLE mapping, serialized writes, release and sensor lifecycle tests passed');
let late,timeoutFaults=0;
const slow=new C.Driver((s,done)=>{late=done;},()=>timeoutFaults++);
slow.arm();
setTimeout(()=>{
  assert.equal(timeoutFaults,1);assert(!slow.active);
  late();assert(!slow.active);assert.equal(timeoutFaults,1);
  console.log('BLE write timeout and late callback test passed');
},250);

// Landscape physical poses: phone top left/right must produce the same screen-relative command.
for(const [side,right,forward] of [
  ['left',{x:0,y:-0.5,z:-0.86},{x:-0.5,y:0,z:-0.86}],
  ['right',{x:0,y:0.5,z:-0.86},{x:0.5,y:0,z:-0.86}]
]) {
  const r=C.motion(right,zero,0,0,1,side),f=C.motion(forward,zero,0,0,1,side);
  assert(r[0]>0);assert.equal(Math.abs(r[1]),0);
  assert(f[1]>0);assert.equal(Math.abs(f[0]),0);
  assert(C.motion(right,right,10,10,1,side).every(n=>n===0));
}
page.visible=true;page.data.connected=true;page.driver={active:false,arm(done){this.active=true;done();return true;},stop(){this.active=false;},drive(){}};accel(zero);page.toggleTilt();assert(page.held);
page.changeGrip({currentTarget:{dataset:{grip:'right'}}});assert(!page.held);assert.equal(page.data.grip,'right');
accel(zero);page.toggleTilt();assert(page.held);page.onResize();assert(!page.held);
console.log('Landscape axis direction, neutral calibration, grip change and resize stop tests passed');

