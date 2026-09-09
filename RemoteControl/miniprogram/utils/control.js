// Gravity mapping and compass steering adapted from BLEControl (dev 2b072bb).
const SERVICE = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E';
const WRITE = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E';
const STATUS = '6E400003-B5A3-F393-E0A9-E50E24DCCA9E';
const FACES=['呼呼睡','开心','酷酷开车','抱心','转晕了','委屈掉线','左右张望','捧腹大笑','累到哈欠','吓一跳','气鼓鼓','得意点赞'];
function parseStatus(value) {
  const b=new Uint8Array(value);
  if(b.length!==8||b[0]!==1||b[1]>100||b[2]>100||b[4]>=FACES.length||
    b[5]>FACES.length||b[6]>1||b[7]>1)throw new Error('状态格式不支持，请更新固件');
  return {dizzy:b[1],fatigue:b[2],idle:b[3],face:b[4],faceName:FACES[b[4]],
    manual:b[5]-1,fault:!!b[6],owns:!!b[7]};
}
const normalize = s => (s || '').replace(/-/g, '').toUpperCase();
const delta = d => ((d + 180) % 360 + 360) % 360 - 180;
const axis = (v, dead, full) => Math.sign(v) * Math.min(1, Math.max(0, (Math.abs(v)-dead)/(full-dead)));
function motion(sample, zero, heading, reference, scale) {
  let x=axis(sample.x-zero.x, 0.15, 0.7);
  // Phone top tilted down: negative Y gravity -> chassis forward (+Y).
  let y=-axis(sample.y-zero.y, 0.15, 0.7);
  const length=Math.hypot(x,y); if(length>1) { x/=length;y/=length; }
  // Compass increases clockwise; chassis positive rotation is counterclockwise.
  const w=-axis(delta(heading-reference),8,45);
  return [x*0.12*scale,y*0.12*scale,w*0.60*scale];
}
function frame(v) {
  // Worst case 19 ASCII bytes: fits the default 20-byte ATT payload.
  return '#' + v.map(n => (Math.abs(n)<0.005?0:n).toFixed(2)).join(',') + '!';
}
function buffer(s) {
  const b=new ArrayBuffer(s.length), bytes=new Uint8Array(b);
  for(let i=0;i<s.length;i++) bytes[i]=s.charCodeAt(i);
  return b;
}

/* Exactly one write in flight. STOP follows any already-issued ARM/drive write.
   An epoch prevents an old completion from resuming driving after release. */
class Driver {
  constructor(write, fault) {
    this.write=write;this.fault=fault;
    this.active=false;this.busy=false;this.pendingStop=false;this.failed=false;this.epoch=0;
    this.pendingFace=null;
  }
  send(text, done) {
    if(this.failed)return;
    this.busy=true;
    let finished=false;
    const timer=setTimeout(() => finish(new Error('BLE 写入超时')),220);
    const finish=err => {
      if(finished)return;finished=true;clearTimeout(timer);this.busy=false;
      if(err) {
        this.active=false;this.failed=true;this.epoch++;this.pendingStop=false;this.pendingFace=null;
        this.fault(err);return;
      }
      if(this.pendingStop) {
        this.pendingStop=false;this.send('#STOP!');return;
      }
      if(done)done();
      if(!this.busy&&!this.active&&this.pendingFace!==null) {
        const face=this.pendingFace;this.pendingFace=null;this.send('#FACE:'+face+'!');
      }
    };
    try { this.write(text,finish); } catch(err) { finish(err); }
  }
  arm(done) {
    if(this.failed||this.busy||this.active||this.pendingStop||this.pendingFace!==null)return false;
    const epoch=++this.epoch;
    this.send('#ARM!',()=>{ if(epoch===this.epoch) { this.active=true;if(done)done(); } });
    return true;
  }
  drive(v) { if(this.active&&!this.busy)this.send(frame(v)); }
  face(face) {
    if(this.failed||!Number.isInteger(face)||face< -1||face>=FACES.length)return;
    this.pendingFace=face;
    if(!this.busy&&!this.active) { this.pendingFace=null;this.send('#FACE:'+face+'!'); }
  }
  stop() {
    this.active=false;this.epoch++;
    if(this.failed)return; // Fault handler closes BLE; watchdog is the fallback.
    if(this.busy)this.pendingStop=true;
    else this.send('#STOP!');
  }
}
module.exports={SERVICE,WRITE,STATUS,FACES,parseStatus,normalize,delta,motion,frame,buffer,Driver};
