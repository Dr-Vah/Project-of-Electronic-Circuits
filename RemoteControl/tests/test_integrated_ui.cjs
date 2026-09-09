const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const C=require('../miniprogram/utils/control');
const videoSource=fs.readFileSync(require.resolve('../miniprogram/utils/video'),'utf8');
let now=0,nextTimer=0;const timers=new Map();
const timeout=(fn,ms)=>{const id=++nextTimer;timers.set(id,{fn,at:now+ms});return id;};
const advance=ms=>{
  const end=now+ms;
  while(true) {
    const next=[...timers].filter(([,v])=>v.at<=end).sort((a,b)=>a[1].at-b[1].at)[0];
    if(!next)break;now=next[1].at;timers.delete(next[0]);next[1].fn();
  }
  now=end;
};
const context={module:{exports:{}},Uint8Array,ArrayBuffer,DataView,Set,Date,Math,Error,
  setTimeout:timeout,clearTimeout:id=>timers.delete(id)};
vm.runInNewContext(videoSource,context);const {FrameParser,Video}=context.module.exports;
function packet({seq=1,age=0,status=0,jpeg=[255,216,255,217]}={}) {
  const b=new Uint8Array(20+(status?0:jpeg.length));b.set([79,86,70,49]);
  const d=new DataView(b.buffer);d.setUint32(4,b.length-20,true);d.setUint32(8,seq,true);
  d.setUint32(12,age,true);d.setUint32(16,status,true);if(!status)b.set(jpeg,20);return b;
}
const wire=packet();
for(let split=1;split<wire.length;split++) {
  const parser=new FrameParser();assert.equal(parser.push(wire.slice(0,split).buffer),null);
  assert.equal(parser.push(wire.slice(split).buffer).sequence,1);
}
assert.equal(new FrameParser().push(packet({status:1}).buffer).status,1);
const oversized=packet();new DataView(oversized.buffer).setUint32(4,128*1024+1,true);
assert.throws(()=>new FrameParser().push(oversized.buffer));
assert.throws(()=>new FrameParser().push(new Uint8Array([...wire,...wire]).buffer));
assert(new FrameParser().push(packet({jpeg:[1,2,3,4]}).buffer).invalid);
assert.equal(new FrameParser().push(packet({jpeg:[255,216,255,217,0,0,0]}).buffer).jpeg.byteLength,4);
// Embedded EOI in APP metadata must not mask a truncated main image.
assert(new FrameParser().push(packet({jpeg:[255,216,255,225,0,4,255,217]}).buffer).invalid);
const withScan=[255,216,255,225,0,4,255,217,255,218,0,2,1,255,0,2,255,208,3,255,217,0,0];
assert.equal(new FrameParser().push(packet({jpeg:withScan}).buffer).jpeg.byteLength,21);

let socket,files=new Map(),ui={},lost=0,writeLater=false,pendingWrite;
const fileApi={writeFile(o){if(writeLater)pendingWrite=o;else{files.set(o.filePath,o.data);o.success();}},
  unlink(o){files.delete(o.filePath);if(o.success)o.success();}};
const api={env:{USER_DATA_PATH:'/tmp'},getDeviceInfo:()=>({platform:'ios'}),getFileSystemManager:()=>fileApi,
  createTCPSocket(){socket={onError(fn){this.error=fn;},onClose(fn){this.closed=fn;},
    onConnect(fn){this.connected=fn;},onMessage(fn){this.message=fn;},
    connect(o){this.target=o;},writes:[],write(b){this.writes.push(b);},close(){if(this.closed)this.closed();}};return socket;}};
const video=new Video(api,d=>Object.assign(ui,d),()=>lost++,()=>now);
video.start();assert.equal(socket.target.port,8266);socket.connected();assert.equal(socket.writes.length,1);
socket.message({message:wire.slice(0,9).buffer});assert(!ui.frameSrc);
socket.message({message:wire.slice(9).buffer});assert(ui.frameSrc);assert(!ui.videoFresh);
video.rendered(ui.frameSrc);assert(ui.videoFresh);assert.equal(files.size,1);
advance(250);assert.equal(socket.writes.length,2);
socket.message({message:packet({seq:2}).buffer});video.rendered(ui.frameSrc);assert.equal(files.size,1);
advance(250);socket.message({message:packet({status:1}).buffer});assert(!ui.videoFresh);assert(!ui.frameSrc);assert(lost>0);
video.stop();assert.equal(files.size,0);
video.start();socket.connected();socket.message({message:packet({age:1000}).buffer});assert(!ui.videoFresh);
video.stop();video.start();socket.connected();advance(1001);assert(!video.running);assert(!ui.frameSrc);
video.start();socket.connected();writeLater=true;socket.message({message:wire.buffer});
const old=pendingWrite;video.stop();files.set(old.filePath,old.data);old.success();
assert(!ui.videoOn&&!ui.frameSrc);assert.equal(files.size,0);writeLater=false;
video.start();socket.connected();
socket.message({message:packet({jpeg:[255,216,1,2]}).buffer});
assert(video.running&&!ui.videoFresh);advance(250);
socket.message({message:packet({seq:2,jpeg:[255,216,255,217,0,0]}).buffer});
video.rendered(ui.frameSrc);assert(ui.videoFresh);assert.equal(video.invalidFrames,0);video.stop();
video.start();socket.connected();
for(let i=0;i<5;i++) {
  socket.message({message:packet({seq:i,jpeg:[255,216,1,2]}).buffer});
  if(i<4) { assert(video.running);advance(250); }
}
assert(!video.running&&ui.videoText.includes('连续收到异常画面'));
const android={...api,getDeviceInfo:()=>({platform:'android'}),
  startWifi(o){o.success();},getConnectedWifi(o){o.success({wifi:{BSSID:'aa:bb:cc:dd:ee:ff'}});},
  createTCPSocket(){const s=api.createTCPSocket();s.onBindWifi=fn=>{s.bound=fn;};s.bindWifi=o=>{s.binding=o;};return s;}};
const androidVideo=new Video(android,()=>{},()=>{},()=>now);
androidVideo.start();assert.equal(socket.binding.BSSID,'aa:bb:cc:dd:ee:ff');assert(!socket.target);
socket.bound();assert.equal(socket.target.address,'192.168.4.1');const cancelled=socket;
androidVideo.stop();cancelled.bound();assert(!androidVideo.running);

// Real page handlers: manual controls must work without sensors, and multiple
// fingers must preserve the other axis until the last input is released.
let page,queryCallback,statusCallback,commands=[],stops=0,faces=[];
const wx={...api,onBLEConnectionStateChange(){},offBLEConnectionStateChange(){},
  onBLECharacteristicValueChange(fn){statusCallback=fn;},offBLECharacteristicValueChange(){},
  onAccelerometerChange(){},onCompassChange(){},startAccelerometer(){},startCompass(){},
  offAccelerometerChange(){},offCompassChange(){},stopAccelerometer(){},stopCompass(){},
  closeBLEConnection(){},closeBluetoothAdapter(){},
  createSelectorQuery(){return {in(){return this;},select(){return this;},
    boundingClientRect(fn){queryCallback=fn;return this;},exec(){}};}};
vm.runInNewContext(fs.readFileSync(require.resolve('../miniprogram/pages/index/index.js'),'utf8'),{
  require:p=>p.endsWith('/video')?{Video}:C,Page:p=>page=p,wx,Date,Number,Math,Promise,Error,Object,
  setInterval(){return 1;},clearInterval(){},setTimeout:timeout,clearTimeout:id=>timers.delete(id)
});
page.setData=function(d){Object.assign(this.data,d);};page.onLoad();page.onShow();
page.data.connected=true;page.deviceId='car';
page.driver={active:false,arm(done){this.active=true;done();return true;},
  drive(v){commands.push(v);},stop(){this.active=false;stops++;},face(f){faces.push(f);}};
const touch=(id,x=150,y=100)=>({identifier:id,clientX:x,clientY:y});
const event=(id,sign=1)=>({changedTouches:[touch(id)],currentTarget:{dataset:{sign}}});
page.mode({currentTarget:{dataset:{mode:'joystick'}}});
page.joyStart(event(1));queryCallback({left:0,top:0,width:200,height:200});page.tick();
assert(page.data.driving);assert(commands.at(-1)[0]>0); // No accelerometer/compass needed.
page.turnStart(event(2));assert(commands.at(-1)[2]>0&&commands.at(-1)[0]>0);
page.joyEnd(event(1));assert(page.data.driving);assert.equal(commands.at(-1)[0],0);assert(commands.at(-1)[2]>0);
page.turnEnd(event(2));assert(!page.data.driving);
page.turnStart(event(3,-1));assert(page.data.driving);assert(commands.at(-1)[2]<0);
page.turnStart(event(4,1));assert.equal(commands.at(-1)[2],0);
page.turnEnd(event(3));assert(commands.at(-1)[2]>0);page.turnEnd(event(4));assert(!page.data.driving);
page.joyStart(event(1));const oldQuery=queryCallback;page.release();page.joyStart(event(1));
oldQuery({left:0,top:0,width:200,height:200});assert(!page.held);
queryCallback({left:0,top:0,width:200,height:200});assert(page.held);
page.chooseFace({detail:{value:4}});assert(!page.held);assert.equal(faces.at(-1),3);
const status={deviceId:'car',characteristicId:C.STATUS,value:new Uint8Array([1,66,25,0,4,0,0,0]).buffer};
statusCallback(status);assert.equal(page.data.faceName,'转晕了');assert.equal(page.data.dizzy,66);
statusCallback({...status,deviceId:'other',value:new Uint8Array(8).buffer});assert(page.data.statusFresh);
page.statusAt=Date.now()-1600;page.tick();assert(!page.data.statusFresh);
page.turnStart(event(6));statusCallback({...status,value:new Uint8Array([1,66,25,0,5,0,1,0]).buffer});
assert(page.held);page.release(); // Old fault telemetry must not cancel a newly accepted ARM.
page.turnStart(event(7));page.video.lost();assert(!page.held); // Image loss never auto-resumes.
page.turnStart(event(8));page.onHide();assert(!page.data.driving&&!page.data.videoOn);
assert(stops>5);
const wxml=fs.readFileSync(require.resolve('../miniprogram/pages/index/index.wxml'),'utf8');
for(const [,handler] of wxml.matchAll(/(?:bind|catch)[a-z]+="([A-Za-z]+)"/g))assert.equal(typeof page[handler],'function',handler);
assert.deepEqual(C.parseStatus(new Uint8Array([1,1,2,3,4,0,0,1]).buffer).manual,-1);
assert.throws(()=>C.parseStatus(new Uint8Array([1,101,2,3,4,0,0,1]).buffer));
console.log('PASS: integrated joystick/multitouch, expressions, live status, video fragmentation, stale frames, timeout and late-file cleanup');
