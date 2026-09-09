const C=require('../../utils/control');
const {Video}=require('../../utils/video');
Page({
  data:{status:'未连接',connected:false,connecting:false,driving:false,percent:50,
    sensor:'等待传感器',sensorDetail:'',useCompass:true,tiltLatched:false,grip:'left',details:false,x:'0.00',y:'0.00',w:'0.00',mode:'joystick',joyX:0,joyY:0,
    videoOn:false,videoFresh:false,frameSrc:'',frames:[],flipped:true,videoText:'画面已暂停',
    wifiText:'看画面请连接 Omni-Remote Wi-Fi',statusFresh:false,faceName:'等待小车状态',
    dizzy:0,fatigue:0,idle:0,manual:-1,fault:false,faceOptions:['自动情绪',...C.FACES],faceIndex:0},
  onLoad() {
    this.scale=0.5;this.generation=0;this.turnTouches={};this.joy=[0,0];
    this.video=new Video(wx,d=>{
      if('frameSrc' in d)d.frames=d.frameSrc?[d.frameSrc]:[];
      this.setData(d);
    },()=>{if(this.held) { this.release();this.setData({status:'画面中断，已停车；确认环境后重新开始驾驶'}); }});
    this.accel=r=>{
      if(![r.x,r.y,r.z].every(Number.isFinite)) { this.sample=null;if(this.kind==='tilt'&&this.held)this.release();return; }
      this.sample={x:r.x,y:r.y,z:r.z};this.accelAt=Date.now();
      this.accelError='';
    };
    this.compass=r=>{
      if(!Number.isFinite(r.direction)||r.direction<0||r.direction>360||
        r.accuracy==='unreliable'||r.accuracy==='low'||
        (Number.isFinite(r.accuracy)&&(r.accuracy<0||r.accuracy>30))) {
        this.headingAt=0;this.compassError='罗盘数据或精度不可用';
        if(this.kind==='tilt'&&this.held&&this.usingCompass)this.release();return;
      }
      this.heading=r.direction;this.headingAt=Date.now();
      this.compassError='';
    };
    this.connection=r=>{if(r.deviceId===this.deviceId&&!r.connected)this.disconnect('蓝牙断开，请重新连接');};
    wx.onBLEConnectionStateChange(this.connection);
    this.statusChanged=r=>{
      if(r.deviceId!==this.deviceId||C.normalize(r.characteristicId)!==C.normalize(C.STATUS))return;
      try {
        const state=C.parseStatus(r.value);this.statusAt=Date.now();
        this.setData({...state,statusFresh:true,faceIndex:state.manual+1});
        // Telemetry may be queued before a new ARM. It is display-only;
        // command rejection and the firmware watchdog enforce driving ownership.
      } catch(e) { this.setData({statusFresh:false,faceName:e.message}); }
    };
    wx.onBLECharacteristicValueChange(this.statusChanged);
  },
  onShow() {
    this.visible=true;this.startSensors();
    this.timer=setInterval(()=>this.tick(),100);
  },
  startSensors() {
    this.sample=null;this.headingAt=0;this.accelError='';this.compassError='';
    const epoch=this.sensorEpoch=(this.sensorEpoch||0)+1;
    wx.offAccelerometerChange(this.accel);wx.offCompassChange(this.compass);
    wx.onAccelerometerChange(this.accel);wx.onCompassChange(this.compass);
    wx.startAccelerometer({interval:'game',fail:e=>{
      if(epoch!==this.sensorEpoch||!this.visible)return;
      this.accelError=(e&&e.errMsg)||'加速度计启动失败';this.sensorFail();
    }});
    wx.startCompass({fail:e=>{
      if(epoch!==this.sensorEpoch||!this.visible)return;
      this.compassError=(e&&e.errMsg)||'罗盘启动失败';this.headingAt=0;
      if(this.kind==='tilt'&&this.held&&this.usingCompass)this.release();
    }});
  },
  retrySensors() { this.release();this.startSensors(); },
  toggleDetails() { this.setData({details:!this.data.details}); },
  changeGrip(e) {
    const grip=e.currentTarget.dataset.grip;
    if(!['left','right'].includes(grip)||grip===this.data.grip)return;
    this.release();this.setData({grip,status:'握持方向已切换，请重新校准体感'});
  },
  onResize() { this.release();this.joyRect=null;this.setData({status:'显示尺寸已变化，请重新开始驾驶'}); },
  toggleCompass(e) { this.release();this.setData({useCompass:!!e.detail.value}); },
  sensorFail() { if(this.kind==='tilt')this.release();this.setData({sensor:'传感器不可用，可切换摇杆驾驶'}); },
  compassFresh() { return !!this.headingAt&&Date.now()-this.headingAt<1000; },
  tiltReason() {
    if(this.accelError)return '加速度计不可用：'+this.accelError;
    if(!this.sample)return '未收到加速度数据，请在手机微信中运行并重新启用传感器';
    if(Date.now()-this.accelAt>=300)return '加速度数据已超时，请重新启用传感器';
    const s=this.sample;
    if(Math.abs(s.z)<=0.2||Math.abs(s.x)>=0.95||Math.abs(s.y)>=0.95)
      return '手机接近竖直，请把屏幕朝上放平一些';
    if(this.held&&this.kind==='tilt'&&this.zero&&s.z*this.zero.z<=0)return '手机已翻面，请松手重新校准';
    if(this.held&&this.kind==='tilt'&&this.usingCompass&&!this.compassFresh())return '罗盘数据已超时，请停止后重新开始';
    return '';
  },
  fresh() {
    return !this.tiltReason();
  },
  tick() {
    const fresh=this.fresh();
    const compass=this.data.useCompass&&this.compassFresh();
    const turning=this.held&&this.kind==='tilt'?this.usingCompass:compass;
    const s=this.sample;
    this.setData({sensor:fresh?(turning?'体感就绪 · 倾斜平移＋转手机旋转':'倾斜就绪 · 旋转请用左右按钮'):this.tiltReason(),
      sensorDetail:(s?'加速度 '+s.x.toFixed(2)+', '+s.y.toFixed(2)+', '+s.z.toFixed(2):'加速度：无数据')+
        '；'+(this.compassFresh()?'罗盘：已收到':(this.compassError||'罗盘：未更新'))});
    if(this.statusAt&&Date.now()-this.statusAt>1500)this.setData({statusFresh:false});
    if(!fresh&&this.kind==='tilt') { if(this.held)this.release();return; }
    if(!this.held||!this.driver||!this.driver.active)return;
    const v=this.kind==='tilt'?C.motion(this.sample,this.zero,this.usingCompass?this.heading:0,this.usingCompass?this.reference:0,this.scale,this.data.grip):
      [this.joy[0]*0.12*this.scale,this.joy[1]*0.12*this.scale,0];
    const turns=Object.values(this.turnTouches);
    if(turns.length)v[2]=Math.max(-1,Math.min(1,turns.reduce((a,b)=>a+b,0)))*0.60*this.scale;
    this.setData({x:v[0].toFixed(2),y:v[1].toFixed(2),w:v[2].toFixed(2)});
    this.driver.drive(v);
  },
  begin(kind) {
    if(this.held)return this.kind===kind;
    if(!this.visible)return false;
    if(!this.checkConnection())return false;
    if(kind==='tilt'&&!this.fresh()) {
      this.controlHint(this.tiltReason());return false;
    }
    this.kind=kind;
    if(kind==='tilt') {
      this.zero={...this.sample};this.reference=this.heading;
      this.usingCompass=this.data.useCompass&&this.compassFresh();
    }
    this.held=true;
    if(!this.driver.arm(()=>{
      if(this.held&&(this.kind!=='tilt'||this.fresh())) { this.setData({driving:true});this.tick(); }
      else this.release();
    })) { this.held=false;this.controlHint('正在完成上一条指令，请停止后重新开始');return false; }
    return true;
  },
  controlHint(text) {
    this.setData({status:text});
    if(wx.showToast&&(!this.hintAt||Date.now()-this.hintAt>1500)) {
      this.hintAt=Date.now();wx.showToast({title:text,icon:'none',duration:2500});
    }
  },
  checkConnection() {
    if(this.data.connected&&this.driver)return true;
    this.controlHint('蓝牙未连接，请点顶部“连接小车”；连上 Wi-Fi 后也需要蓝牙');return false;
  },
  toggleTilt() {
    if(this.held) { this.release();return; }
    if(this.begin('tilt')&&this.held) {
      this.tiltLatched=true;
      this.setData({tiltLatched:true,status:'体感已开启，当前握姿为零点；点击停止结束'});
    }
  },
  mode(e) {
    this.release();this.setData({mode:e.currentTarget.dataset.mode});
  },
  joyStart(e) {
    if(this.joyTouch!==undefined||!this.checkConnection())return;
    const t=e.changedTouches[0];if(!t)return;
    this.joyTouch=t.identifier;
    const stamp=this.joyStamp=(this.joyStamp||0)+1;
    // Measure on each press so scroll/layout changes cannot shift the center.
    wx.createSelectorQuery().in(this).select('#joystick').boundingClientRect(rect=>{
      if(this.joyStamp!==stamp||this.joyTouch!==t.identifier||!rect)return;
      this.joyRect=rect;
      this.moveJoy(t);
      if(!this.begin('manual')) { this.joyTouch=undefined;this.joy=[0,0];this.setData({joyX:0,joyY:0}); }
    }).exec();
  },
  moveJoy(t) {
    const r=this.joyRect;if(!r||!Number.isFinite(t.clientX)||!Number.isFinite(t.clientY))return;
    let x=(t.clientX-r.left-r.width/2)/(r.width/2),y=-(t.clientY-r.top-r.height/2)/(r.height/2);
    const length=Math.hypot(x,y);if(length>1) { x/=length;y/=length; }
    this.joy=length<0.08?[0,0]:[x,y];
    const travel=Math.min(r.width,r.height)*0.32;
    this.setData({joyX:this.joy[0]*travel,joyY:-this.joy[1]*travel});
  },
  joyMove(e) { const t=e.touches.find(t=>t.identifier===this.joyTouch);if(t)this.moveJoy(t); },
  joyEnd(e) {
    if(!e.changedTouches.some(t=>t.identifier===this.joyTouch))return;
    this.joyTouch=undefined;this.joy=[0,0];this.setData({joyX:0,joyY:0});
    if(!Object.keys(this.turnTouches).length)this.release();else this.tick();
  },
  turnStart(e) {
    const t=e.changedTouches[0];if(!t||!this.checkConnection())return;
    const sign=Number(e.currentTarget.dataset.sign);if(sign!==1&&sign!==-1)return;
    this.turnTouches[t.identifier]=sign;
    if(!this.held&&!this.begin('manual'))this.turnTouches={};
    else this.tick();
  },
  turnEnd(e) {
    for(const t of e.changedTouches)delete this.turnTouches[t.identifier];
    if(!Object.keys(this.turnTouches).length&&this.joyTouch===undefined&&!this.tiltLatched)this.release();
    else this.tick();
  },
  release() {
    this.tiltLatched=false;
    this.usingCompass=false;
    this.joyStamp=(this.joyStamp||0)+1;
    this.held=false;this.joyTouch=undefined;this.turnTouches={};this.joy=[0,0];
    if(this.driver)this.driver.stop();
    this.setData({driving:false,tiltLatched:false,x:'0.00',y:'0.00',w:'0.00',joyX:0,joyY:0});
  },
  preventScroll() {},
  speed(e) {
    this.release();this.scale=Number(e.currentTarget.dataset.scale);
    this.setData({percent:Math.round(this.scale*100)});
  },
  chooseFace(e) {
    if(!this.driver||!this.data.connected)return;
    const face=Number(e.detail.value)-1;
    if(!Number.isInteger(face)||face< -1||face>=C.FACES.length)return;
    this.release();this.driver.face(face);
  },
  toggleVideo() {
    this.release();
    if(this.video.running)this.video.stop();else this.video.start();
  },
  flipVideo() { this.setData({flipped:!this.data.flipped}); },
  frameLoaded(e) { this.video.rendered(e.currentTarget.dataset.path); },
  frameError(e) { this.video.renderError(e.currentTarget.dataset.path); },
  connectWifi() {
    this.release();this.video.stop();
    if(!wx.startWifi||!wx.connectWifi) { this.setData({wifiText:'请在手机 Wi-Fi 设置连接 Omni-Remote，密码 omni2026'});return; }
    this.setData({wifiText:'正在连接小车 Wi-Fi…'});
    const generation=this.generation;
    const fail=()=>{if(this.visible)this.setData({wifiText:'请在 Wi-Fi 设置连接 Omni-Remote，密码 omni2026；允许本地网络访问'});};
    wx.startWifi({success:()=>{
      if(!this.visible||generation!==this.generation)return;
      wx.connectWifi({SSID:'Omni-Remote',password:'omni2026',
        success:()=>{if(this.visible)this.setData({wifiText:'已连接小车 Wi-Fi，可开启画面'});},fail});
    },fail});
  },
  // Generation checks discard callbacks from cancelled scans/connections.
  async connect() {
    if(this.data.connecting||this.data.connected)return;
    const generation=++this.generation;
    const call=(method,args={})=>new Promise((resolve,reject)=>wx[method]({...args,success:resolve,fail:reject}));
    const current=()=>{if(generation!==this.generation)throw new Error('连接已取消');};
    this.setData({connecting:true,status:'搜索中…'});
    try {
      await call('openBluetoothAdapter');current();
      const deviceId=await new Promise((resolve,reject)=>{
        let finished=false;
        const finish=(error,id)=>{
          if(finished)return;finished=true;
          clearTimeout(this.scanTimer);wx.offBluetoothDeviceFound(found);
          wx.stopBluetoothDevicesDiscovery({});this.cancelScan=null;
          error?reject(error):resolve(id);
        };
        const found=r=>{
          const d=(r.devices||[]).find(d=>(d.name||d.localName)==='Omni-Remote-BLE');
          if(d)finish(null,d.deviceId);
        };
        this.cancelScan=()=>finish(new Error('搜索已取消'));
        wx.onBluetoothDeviceFound(found);
        this.scanTimer=setTimeout(()=>finish(new Error('未找到小车，请确认已烧录 BLE 固件')),12000);
        wx.startBluetoothDevicesDiscovery({allowDuplicatesKey:false,
          success:()=>{if(finished||generation!==this.generation)wx.stopBluetoothDevicesDiscovery({});},
          fail:e=>finish(e)});
      });
      current();this.deviceId=deviceId;
      await call('createBLEConnection',{deviceId,timeout:10000});
      if(generation!==this.generation) { wx.closeBLEConnection({deviceId});return; }
      const {services}=await call('getBLEDeviceServices',{deviceId});current();
      const service=services.find(s=>C.normalize(s.uuid)===C.normalize(C.SERVICE));
      if(!service)throw new Error('未找到 Omni 服务，请更新固件');
      const serviceId=service.uuid;
      const {characteristics}=await call('getBLEDeviceCharacteristics',{deviceId,serviceId});current();
      const characteristic=characteristics.find(c=>C.normalize(c.uuid)===C.normalize(C.WRITE)&&c.properties.write);
      if(!characteristic)throw new Error('未找到带响应写特征值');
      const status=characteristics.find(c=>C.normalize(c.uuid)===C.normalize(C.STATUS)&&c.properties.notify);
      if(!status)throw new Error('请先烧录集成版固件：缺少小车状态服务');
      const characteristicId=characteristic.uuid;
      await call('notifyBLECharacteristicValueChange',{deviceId,serviceId,characteristicId:status.uuid,state:true});current();
      this.driver=new C.Driver((text,done)=>wx.writeBLECharacteristicValue({
        deviceId,serviceId,characteristicId,writeType:'write',value:C.buffer(text),
        success:()=>done(),fail:done
      }),()=>{if(generation===this.generation)this.disconnect('指令失败或超时，已停止，请重新连接');});
      this.setData({connecting:false,connected:true,status:'已连接，点击校准开始体感，或使用摇杆'});
    } catch(e) {if(generation===this.generation)this.disconnect(e.message||e.errMsg||'连接失败');}
  },
  disconnect(message) {
    this.generation++;this.release();this.driver=null;
    if(this.cancelScan)this.cancelScan();
    const deviceId=this.deviceId;this.deviceId='';
    if(deviceId)wx.closeBLEConnection({deviceId});
    this.statusAt=0;
    this.setData({connecting:false,connected:false,statusFresh:false,status:typeof message==='string'?message:'已断开'});
  },
  onHide() {
    this.sensorEpoch=(this.sensorEpoch||0)+1;
    this.visible=false;this.disconnect('离开页面已安全断开；返回后请点“连接小车”重新连接蓝牙');clearInterval(this.timer);
    this.video.stop();
    wx.offAccelerometerChange(this.accel);wx.offCompassChange(this.compass);
    wx.stopAccelerometer({});wx.stopCompass({});
  },
  onUnload() {
    this.onHide();wx.offBLEConnectionStateChange(this.connection);
    wx.offBLECharacteristicValueChange(this.statusChanged);wx.closeBluetoothAdapter({});
  }
});
