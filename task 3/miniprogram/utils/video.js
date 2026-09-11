const CAP=128*1024,HEADER=20,PORT=8266;
// Walk JPEG markers, skipping length-delimited metadata. Do not mistake an
// embedded thumbnail's EOI for the end of the main image. Decode still happens
// in the native image component; this checks framing, not image correctness.
function jpegEnd(b) {
  if(b.length<4||b[0]!==255||b[1]!==216)return 0;
  let p=2,scan=false;
  while(p<b.length) {
    if(scan) { while(p<b.length&&b[p]!==255)p++; }
    if(p>=b.length||b[p++]!==255)return 0;
    while(p<b.length&&b[p]===255)p++;
    if(p>=b.length)return 0;
    const marker=b[p++];
    if(scan&&(marker===0||(marker>=208&&marker<=215)))continue;
    if(marker===217)return p;
    if(marker===216||marker===0)return 0;
    if(marker===1)continue;
    if(p+2>b.length)return 0;
    const length=b[p]*256+b[p+1];
    if(length<2||p+length>b.length)return 0;
    p+=length;
    scan=marker===218||(scan&&marker===220);
  }
  return 0;
}
/* TCP boundaries are arbitrary. Bounded storage, at most one response/request. */
class FrameParser {
  constructor() { this.bytes=new Uint8Array(CAP+HEADER);this.used=0; }
  push(message) {
    const b=new Uint8Array(message);
    if(this.used+b.length>this.bytes.length)throw new Error('视频数据过大');
    this.bytes.set(b,this.used);this.used+=b.length;
    if(this.used<HEADER)return null;
    if(this.bytes[0]!==79||this.bytes[1]!==86||this.bytes[2]!==70||this.bytes[3]!==49)
      throw new Error('视频协议不匹配，请更新固件');
    const d=new DataView(this.bytes.buffer),length=d.getUint32(4,true),status=d.getUint32(16,true);
    if(length>CAP||status>1||(status===1&&length!==0)||(status===0&&length<4))
      throw new Error('视频帧长度错误');
    if(this.used<HEADER+length)return null;
    if(this.used!==HEADER+length)throw new Error('收到未请求的视频数据');
    const result={status,sequence:d.getUint32(8,true),age:d.getUint32(12,true),
      jpeg:this.bytes.buffer.slice(HEADER,HEADER+length)};
    if(status===0) {
      const j=new Uint8Array(result.jpeg);
      const end=jpegEnd(j);
      if(!end) {
        const tail=Array.from(j.slice(-8),n=>n.toString(16).padStart(2,'0')).join(' ');
        result.invalid='JPEG 边界异常，'+j.length+' 字节，尾部 '+tail;
        result.jpeg=null;
      } else if(end<j.length)result.jpeg=result.jpeg.slice(0,end);
    }
    this.used=0;return result;
  }
}
let session=0;
class Video {
  constructor(api,update,lost,now=Date.now) {
    this.wx=api;this.update=update;this.lost=lost;this.now=now;
    this.files=new Set();this.attempts=[];this.epoch=0;this.running=false;
  }
  start() {
    this.stop();
    if(!this.wx.createTCPSocket) { this.update({videoText:'请更新微信以使用画面'});return; }
    // Respect WeChat's 20 sockets/5 minutes; no automatic reconnect storm.
    this.attempts=this.attempts.filter(t=>this.now()-t<300000);
    if(this.attempts.length>=18) { this.update({videoText:'连接尝试过多，请稍后重试'});return; }
    this.attempts.push(this.now());
    this.running=true;const epoch=++this.epoch;this.id=++session;
    this.count=0;this.invalidFrames=0;this.lastSequence=null;this.current='';this.pending=null;
    this.parser=new FrameParser();
    this.update({videoOn:true,videoFresh:false,frameSrc:'',videoText:'正在连接小车画面…'});
    const fail=e=>{
      if(epoch!==this.epoch)return;
      const d=e&&(e.errMsg||e.message||(e.errno!==undefined?'errno '+e.errno:''));
      this.fail('画面连接中断'+(d?('：'+d):''));
    };
    try {
      const socket=this.socket=this.wx.createTCPSocket({type:'ipv4'});
      socket.onError(fail);socket.onClose(fail);
      socket.onConnect(()=>{if(epoch===this.epoch) { clearTimeout(this.deadline);this.request(); }});
      socket.onMessage(r=>{
        if(epoch!==this.epoch)return;
        try {
          if(!this.waiting)throw new Error('意外的视频数据');
          const frame=this.parser.push(r.message);if(!frame)return;
          this.waiting=false;
          if(frame.invalid) {
            clearTimeout(this.deadline);
            if(++this.invalidFrames>=5) { this.fail('连续收到异常画面：'+frame.invalid);return; }
            this.invalidate('已跳过不完整画面，正在重试（'+this.invalidFrames+'/5）');
            this.next=setTimeout(()=>this.request(),250);return;
          }
          if(frame.status || frame.age+this.now()-this.sent>=1000 || frame.sequence===this.lastSequence) {
            clearTimeout(this.deadline);this.invalidate('摄像头未就绪或画面过期，正在等待…');
            this.next=setTimeout(()=>this.request(),400);return;
          }
          this.lastSequence=frame.sequence;
          this.invalidFrames=0;
          this.display(frame,epoch);
        } catch(e) { this.fail(e.message||'视频数据错误'); }
      });
      this.deadline=setTimeout(fail,2500);
      let connecting=false;
      const connect=()=>{
        if(epoch===this.epoch&&!connecting) { connecting=true;socket.connect({address:'192.168.4.1',port:PORT,timeout:2}); }
      };
      const platform=this.wx.getDeviceInfo?this.wx.getDeviceInfo().platform:this.wx.getSystemInfoSync().platform;
      if(platform==='android'&&socket.bindWifi&&socket.onBindWifi) {
        socket.onBindWifi(connect);
        // bindWifi requires the current BSSID, not the SSID. If Wi-Fi info is
        // unavailable (e.g. location permission), attempt normal LAN routing.
        if(this.wx.startWifi&&this.wx.getConnectedWifi) {
          this.wx.startWifi({success:()=>{
            if(epoch!==this.epoch)return;
            this.wx.getConnectedWifi({success:r=>{
              if(epoch!==this.epoch)return;
              this.update({videoText:r.wifi&&r.wifi.BSSID?('WiFi信息OK '+r.wifi.BSSID):'无WiFi信息，尝试直连'});
              if(r.wifi&&r.wifi.BSSID)socket.bindWifi({BSSID:r.wifi.BSSID});else connect();
            },fail:e=>{console.log('[video] getConnectedWifi fail',e);connect();}});
          },fail:connect});
        } else connect();
      } else connect();
    } catch(e) { fail(e); }
  }
  request() {
    if(!this.running)return;
    this.waiting=true;this.sent=this.now();
    clearTimeout(this.deadline);
    this.deadline=setTimeout(()=>this.fail('画面超时，已停止预览；请检查 Wi-Fi 后重新开启'),1000);
    try { this.socket.write(new Uint8Array([1]).buffer); }
    catch(e) { this.fail('无法请求画面，请检查小车 Wi-Fi'); }
  }
  display(frame,epoch) {
    const path=this.wx.env.USER_DATA_PATH+'/omni-fpv-'+this.id+'-'+(++this.count)+'.jpg';
    this.files.add(path);
    this.pending={path,started:this.sent,age:frame.age};
    this.wx.getFileSystemManager().writeFile({filePath:path,data:frame.jpeg,
      success:()=>{
        if(epoch!==this.epoch) { this.remove(path);return; }
        if(frame.age+this.now()-this.sent>=1000) { this.fail('画面处理超时，请重新开启');return; }
        this.update({frameSrc:path});
      },
      fail:()=>{this.remove(path);if(epoch===this.epoch)this.fail('画面缓存写入失败，请清理存储后重试');}
    });
  }
  rendered(path) {
    const p=this.pending;
    if(!this.running||!p||p.path!==path)return;
    const elapsed=this.now()-p.started;
    if(elapsed+p.age>=1000) { this.fail('画面已过期，请重新开启');return; }
    clearTimeout(this.deadline);clearTimeout(this.staleTimer);
    this.pending=null;
    if(this.current&&this.current!==path)this.remove(this.current);
    this.current=path;
    this.update({videoFresh:true,videoText:'实时画面 · 本帧接收 '+elapsed+' ms'});
    this.staleTimer=setTimeout(()=>this.invalidate('画面已过期，等待新画面…'),1000-elapsed-p.age);
    this.next=setTimeout(()=>this.request(),Math.max(0,250-elapsed));
  }
  renderError(path) { if(this.pending&&this.pending.path===path)this.fail('画面解码失败，请重新开启'); }
  invalidate(text) {
    this.update({...(this.pending?{}:{frameSrc:''}),videoFresh:false,videoText:text});
    if(this.current)this.remove(this.current);this.current='';
    this.lost();
  }
  remove(path) {
    this.files.delete(path);
    this.wx.getFileSystemManager().unlink({filePath:path,fail:()=>{}});
  }
  fail(text) { this.stop();this.update({videoText:text});this.lost(); }
  stop() {
    this.running=false;this.epoch++;
    clearTimeout(this.deadline);clearTimeout(this.next);clearTimeout(this.staleTimer);
    const socket=this.socket;this.socket=null;
    if(socket) { try { socket.close(); } catch(e) {} }
    for(const path of this.files)this.remove(path);
    this.current='';this.pending=null;this.waiting=false;
    this.update({videoOn:false,videoFresh:false,frameSrc:'',videoText:'画面已暂停'});
  }
}
module.exports={CAP,HEADER,PORT,FrameParser,Video,jpegEnd};
