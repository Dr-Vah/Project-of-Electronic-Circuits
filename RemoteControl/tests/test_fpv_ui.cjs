const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const source=fs.readFileSync(require('node:path').join(__dirname,'../main/index.html'),'utf8').split('<script>')[2].split('</script>')[0];
function setup(){
 const elements=new Map(),events={},timers=[],requests=[],revoked=[];let fail=false,seq=1,age=0;
 const element=id=>{if(!elements.has(id))elements.set(id,{hidden:true,style:{},textContent:'',handlers:{},async decode(){},addEventListener(n,f){this.handlers[n]=f;}});return elements.get(id);};
 const sandbox={Math,Number,Error,AbortController,setTimeout,clearTimeout,performance,location:{hostname:'192.168.4.1',protocol:'http:'},
 URL:{createObjectURL(){return 'blob:'+seq;},revokeObjectURL(s){revoked.push(s);}},
 document:{hidden:false,getElementById:element,addEventListener(n,f){events[n]=f;}},window:{addEventListener(n,f){events[n]=f;}},
 setInterval(f){timers.push(f);},async fetch(url){requests.push(url);if(fail)throw Error('network');return{ok:true,headers:{get(n){return n==='X-Frame-Sequence'?String(seq):String(age);}},blob:async()=>({}),json:async()=>({face:4,dizzy:75,fatigue:20,manual:-1})};}};
 vm.createContext(sandbox);vm.runInContext(source,sandbox);
 return{element,events,requests,revoked,sandbox,timers,setFail(v){fail=v;},setFrame(s,a){seq=s;age=a;},click(id){element(id).handlers.click();}};
}
const flush=()=>new Promise(r=>setImmediate(r));
(async()=>{
 const s=setup();await s.timers[0]();assert.equal(s.requests.length,0);
 s.click('videoToggle');await flush();assert.equal(s.element('camera').hidden,false);assert(s.requests[0].includes(':81/frame.jpg'));
 s.setFrame(2,1200);await s.timers[0]();assert(s.element('camera').hidden,'stale frames hidden');
 s.setFrame(3,0);await s.timers[0]();assert(!s.element('camera').hidden);
 s.setFail(true);await s.timers[0]();assert(s.element('camera').hidden,'network failure hides old frame');
 s.setFail(false);s.setFrame(4,0);await s.timers[0]();assert(!s.element('camera').hidden);
 s.sandbox.document.hidden=true;s.events.visibilitychange();assert(s.element('camera').hidden);
 const n=s.requests.length;s.sandbox.document.hidden=false;await s.timers[0]();assert.equal(s.requests.length,n,'no background auto-resume');
 await s.timers[1]();assert(s.element('moodStatus').textContent.includes('75%'));
 assert(s.requests.every(u=>u.includes('frame.jpg')||u==='/api/status'),'no driving requests');
 assert(s.revoked.length>=3,'blob URLs released');
 const secure=setup();secure.sandbox.location.protocol='https:';secure.click('videoToggle');await flush();assert(secure.requests[0].startsWith('https://192.168.4.1:8443/'),'HTTPS page never fetches insecure video');
 console.log('PASS: explicit video start, stale/error hiding, recovery, background pause, mood, no motion API');
})().catch(e=>{console.error(e);process.exitCode=1;});
