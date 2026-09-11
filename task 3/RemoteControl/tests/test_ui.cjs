const fs=require('node:fs');
const vm=require('node:vm');
const assert=require('node:assert/strict');
const source=fs.readFileSync(require('node:path').join(__dirname,'../main/index.html'),'utf8').split('<script>')[1].split('</script>')[0];
function setup(){
 const elements=new Map(), requests=[], events={};let timer, fail=false, pendingClaim=null;
 const element=id=>{if(!elements.has(id))elements.set(id,{value:'0.5',style:{},handlers:{},textContent:'',setPointerCapture(){},getBoundingClientRect(){return {left:0,top:0,width:290,height:290};},addEventListener(n,f){(this.handlers[n]??=[]).push(f);}});return elements.get(id);};
 let clock=100;
 const sandbox={Math,Map,Number,JSON,Error,AbortController,setTimeout,clearTimeout,performance:{now:()=>clock},
 document:{getElementById:element,addEventListener(n,f){events[n]=f;}},window:{isSecureContext:true,DeviceOrientationEvent:{},screen:{orientation:{angle:0}},addEventListener(n,f){events[n]=f;}},
 setInterval(f){timer=f;},async fetch(url,opt){const body=JSON.parse(opt.body);requests.push({url,body});if(fail)throw Error('network');if(url.endsWith('/claim')&&pendingClaim)await pendingClaim;return {ok:true,json:async()=>({ok:true,token:123})};}};
 vm.createContext(sandbox);vm.runInContext(source,sandbox);
 return {requests,element,events,sandbox,tick:()=>timer(),advance(ms){clock+=ms;},fail(){fail=true;},delayClaim(p){pendingClaim=p;},
 event(id,name,extra={}){const e={pointerId:1,clientX:145,clientY:40,preventDefault(){},...extra};for(const f of element(id).handlers[name]??[])f(e);}};
}
const flush=()=>new Promise(r=>setImmediate(r));
(async()=>{
 let s=setup();await s.tick();assert.equal(s.requests.length,0,'idle never moves');
 s.event('pad','pointerdown');await flush();await s.tick();
 let c=s.requests.at(-1).body;assert.equal(c.token,123);assert(c.y>0&&c.y<=.5);assert.equal(c.x,0);
 s.event('right','pointerdown',{pointerId:2});await s.tick();assert.equal(s.requests.at(-1).body.turn,-.5);
 s.event('pad','pointerup');await s.tick();c=s.requests.at(-1).body;assert.equal(c.y,0);assert.equal(c.turn,-.5,'rotation survives joystick release');
 s.event('right','pointerup',{pointerId:2});await flush();assert(s.requests.at(-1).url.endsWith('/stop'));
 let n=s.requests.length;await s.tick();assert.equal(s.requests.length,n,'release prevents future commands');
 s=setup();s.event('pad','pointerdown');await flush();s.fail();await s.tick();n=s.requests.length;await s.tick();assert.equal(s.requests.length,n,'network failure disarms without auto-resume');
 s=setup();s.sandbox.fetch=async()=>({ok:false,status:409});s.event('pad','pointerdown');await flush();assert(s.element('status').textContent.includes('驾驶权仍被占用'));
 s=setup();s.fail();s.event('pad','pointerdown');await flush();assert(s.element('status').textContent.includes('无法连接小车'));assert(!s.element('status').textContent.includes('占用'));
 s=setup();s.sandbox.fetch=async()=>{const e=Error('timeout');e.name='AbortError';throw e;};s.event('pad','pointerdown');await flush();assert(s.element('status').textContent.includes('请求超时'));
 s=setup();let resolve;s.delayClaim(new Promise(r=>resolve=r));s.event('pad','pointerdown');s.event('pad','pointerup');resolve();await flush();await s.tick();assert(!s.requests.some(r=>r.url.endsWith('/command')),'late claim cannot start motion');
 s=setup();s.event('pad','pointerdown');await flush();s.events.blur();await flush();await s.tick();assert(s.requests.at(-1).url.endsWith('/stop'),'background stops');
 s=setup();s.event('pad','pointerdown');await flush();s.event('pad','pointercancel');await flush();assert(s.requests.at(-1).url.endsWith('/stop'));
 s=setup();s.element('emoji').value='3';s.event('emoji','change');await flush();assert(s.requests.at(-1).url.endsWith('/emoji'));assert.equal(s.requests.at(-1).body.face,3);await s.tick();assert.equal(s.requests.length,1,'emoji cannot start driving');
 s=setup();s.sandbox.window.isSecureContext=false;s.event('tiltMode','click');await flush();s.event('tiltHold','pointerdown');assert(!s.requests.some(r=>r.url.endsWith('/claim')),'HTTP cannot arm tilt');
 s=setup();s.sandbox.window.DeviceOrientationEvent={requestPermission:async()=> 'denied'};s.event('tiltMode','click');await flush();assert(s.element('tiltHold').hidden,'denied permission stays joystick');
 s=setup();s.event('tiltMode','click');await flush();s.event('tiltHold','pointerdown');assert(!s.requests.some(r=>r.url.endsWith('/claim')),'must have sensor sample');
 s.events.deviceorientation({beta:30,gamma:0});s.event('tiltHold','pointerdown');await flush();await s.tick();assert.equal(s.requests.at(-1).body.x,0);assert.equal(s.requests.at(-1).body.y,0,'press calibrates current pose');
 s.advance(80);s.events.deviceorientation({beta:10,gamma:20});await s.tick();c=s.requests.at(-1).body;assert(c.x>0&&c.y>0&&Math.hypot(c.x,c.y)<=.5,'forward/right tilt with filtering and speed cap');
 s.event('right','pointerdown',{pointerId:2});await s.tick();assert.equal(s.requests.at(-1).body.turn,-.5);
 s.event('tiltHold','pointerup');await flush();n=s.requests.length;await s.tick();assert.equal(s.requests.length,n,'deadman release also clears rotation');
 s=setup();s.event('tiltMode','click');await flush();s.events.deviceorientation({beta:30,gamma:0});s.event('tiltHold','pointerdown');await flush();s.advance(301);await s.tick();assert(s.requests.at(-1).url.endsWith('/stop'),'sensor timeout stops');n=s.requests.length;s.events.deviceorientation({beta:5,gamma:20});await s.tick();assert.equal(s.requests.length,n,'sensor recovery cannot auto-arm');
 s=setup();s.event('tiltMode','click');await flush();s.events.deviceorientation({beta:30,gamma:0});s.event('tiltHold','pointerdown');await flush();s.events.deviceorientation({beta:null,gamma:null});assert(s.requests.at(-1).url.endsWith('/stop'),'invalid sensor stops');
 s=setup();s.event('tiltMode','click');await flush();s.events.deviceorientation({beta:30,gamma:0});s.event('tiltHold','pointerdown');await flush();s.sandbox.window.screen.orientation.angle=90;await s.tick();assert(s.requests.at(-1).url.endsWith('/stop'),'screen rotation stops');
 const axes=vm.runInContext('tiltAxes(0,4,{beta:0,gamma:0},0)',s.sandbox);assert.equal(axes.x,0,'dead zone');
 const landscape=vm.runInContext('tiltAxes(25,0,{beta:0,gamma:0},90)',s.sandbox);assert(landscape.x>.99&&Math.abs(landscape.y)<.001,'landscape mapping');
 console.log('PASS: joystick, multitouch, stale claims, network stop, emoji; tilt HTTPS/permission/sample gates, calibration, axes, dead zone, filtering, deadman, sensor timeout, invalid data, rotation');
})().catch(e=>{console.error(e);process.exitCode=1;});
