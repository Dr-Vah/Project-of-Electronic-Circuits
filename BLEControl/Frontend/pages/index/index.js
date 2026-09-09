// pages/index/index.js

// ============================================================================
// 通信协议约定（需与 ESP32 蓝牙代码保持一致）：
//   小车 BLE 广播名：ESP32_CAR
//   服务 UUID：      6E400001-B5A3-F393-E0A9-E50E24DCCA9E  (Nordic UART Service)
//   写特征值 UUID：  6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (可写 / Write)
//   数据帧格式：     "#vx,vy,w!"
//      vx=左右(X)速度、vy=前后(Y)速度（-0.2~0.2 m/s），
//      w=旋转速度（rad/s），由手机方向角（罗盘 0~360°）相对参考方向换算而来。
// ============================================================================

const DEVICE_NAME = 'ESP32_CAR';
const SERVICE_UUID = '6E400001-B5A3-F393-E0A9-E50E24DCCA9E';
const WRITE_CHAR_UUID = '6E400002-B5A3-F393-E0A9-E50E24DCCA9E';

// 死区 / 线性映射 / 发送节流
const DEADBAND = 0.15;                 // 死区
const MAX_SPEED = 0.2;                 // 满速 0.2 m/s
const FULL_TILT = 1.0;                 // 90° 时原始重力分量
const SPEED_PER_G = MAX_SPEED / FULL_TILT; // 0.2：90° 对应满速的线性增益
const HEADING_DEADBAND_DEG = 8;        // 旋转死区（度）
const MAX_OMEGA = 0.5;                 // 满旋转速度（rad/s）
const HEADING_GAIN = MAX_OMEGA / 45;   // 方向偏离 45° 即满速（每度对应的 rad/s）
const SEND_INTERVAL_MS = 100;

// 去掉 UUID 里的连字符并转大写，便于跨平台（iOS 返回完整 UUID，Android 可能返回短 UUID）比较。
function normalizeUuid(uuid) {
  return (uuid || '').replace(/-/g, '').toUpperCase();
}

function stringToArrayBuffer(str) {
  const buffer = new ArrayBuffer(str.length);
  const dataView = new DataView(buffer);
  for (let i = 0; i < str.length; i++) {
    dataView.setUint8(i, str.charCodeAt(i));
  }
  return buffer;
}

// 把任意角度差归一化到 -180 ~ 180 度（处理 0°/360° 跨越问题）
function normalizeAngleDelta(d) {
  return ((d + 180) % 360 + 360) % 360 - 180;
}

Page({
  data: {
    statusText: '未连接',
    connecting: false,
    deviceId: '',
    serviceId: '',
    characteristicId: '',
    x_val: '0.00',
    y_val: '0.00',
    heading_val: '0',
    w_val: '0.00'
  },

  onLoad() {
    this._lastSendTime = 0;
    this._w = 0;
    this._heading = 0;
    this._referenceHeading = null;

    // 体感采集独立于蓝牙连接启动：未连接小车时也能在屏幕上看到 X/Y 变化，
    // 连接成功后才把数值发送给小车。
    this.startSensor();

    // 监听蓝牙连接状态变化（如小车断电断开）
    wx.onBLEConnectionStateChange((res) => {
      if (!res.connected) {
        this.setData({ statusText: '已断开', deviceId: '', serviceId: '', characteristicId: '' });
      }
    });
  },

  onUnload() {
    this.cleanup();
  },

  // 1. 初始化蓝牙并开始搜索
  startConnect() {
    if (this.data.connecting) return;
    this.setData({ statusText: '搜索中...', connecting: true });

    wx.openBluetoothAdapter({
      success: () => {
        wx.startBluetoothDevicesDiscovery({
          allowDuplicatesKey: false,
          success: () => {
            this.onDeviceFound();
          },
          fail: () => {
            this.setData({ statusText: '开启搜索失败', connecting: false });
          }
        });
      },
      fail: () => {
        this.setData({ statusText: '请先开启手机蓝牙', connecting: false });
      }
    });
  },

  // 2. 发现设备并按名称过滤
  onDeviceFound() {
    if (wx.offBluetoothDeviceFound) wx.offBluetoothDeviceFound();
    wx.onBluetoothDeviceFound((res) => {
      const devices = res.devices || [];
      for (let i = 0; i < devices.length; i++) {
        const device = devices[i];
        const name = device.name || device.localName || '';
        if (name === DEVICE_NAME) {
          wx.stopBluetoothDevicesDiscovery();
          this.connectDevice(device.deviceId);
          return;
        }
      }
    });
  },

  // 3. 连接设备
  connectDevice(deviceId) {
    this.setData({ statusText: '连接中...', deviceId });

    wx.createBLEConnection({
      deviceId,
      success: () => {
        this.setData({ statusText: '已连接，正在读取服务...' });
        this.discoverServices(deviceId);
      },
      fail: (err) => {
        this.setData({ statusText: '连接失败', connecting: false, deviceId: '' });
        console.error('createBLEConnection fail', err);
      }
    });
  },

  // 4. 读取服务，定位写特征值
  discoverServices(deviceId) {
    wx.getBLEDeviceServices({
      deviceId,
      success: (res) => {
        const services = res.services || [];
        let service = services.find((s) => normalizeUuid(s.uuid) === normalizeUuid(SERVICE_UUID));
        if (!service && services.length > 0) service = services[0];

        if (!service) {
          this.setData({ statusText: '未找到服务', connecting: false });
          return;
        }

        this.setData({ serviceId: service.uuid });
        this.discoverCharacteristics(deviceId, service.uuid);
      },
      fail: () => {
        this.setData({ statusText: '读取服务失败', connecting: false });
      }
    });
  },

  discoverCharacteristics(deviceId, serviceId) {
    wx.getBLEDeviceCharacteristics({
      deviceId,
      serviceId,
      success: (res) => {
        const chars = res.characteristics || [];

        // 优先精确匹配约定的写特征值，其次取第一个支持写/写无响应的特征值
        let target = chars.find((c) => normalizeUuid(c.uuid) === normalizeUuid(WRITE_CHAR_UUID));
        if (!target) {
          target = chars.find((c) => {
            const p = c.properties || {};
            return p.write || p.writeNoResponse;
          });
        }
        if (!target && chars.length > 0) target = chars[0];

        if (!target) {
          this.setData({ statusText: '未找到可写特征值', connecting: false });
          return;
        }

        this.setData({
          characteristicId: target.uuid,
          statusText: '已连接，可以体感控制',
          connecting: false
        });
      },
      fail: () => {
        this.setData({ statusText: '读取特征值失败', connecting: false });
      }
    });
  },

  // 5. 启动加速度计并采集、节流发送
  startSensor() {
    if (wx.offAccelerometerChange) wx.offAccelerometerChange();
    wx.startAccelerometer({ interval: 'game' });

    wx.onAccelerometerChange((res) => {
      let vx = res.x;
      let vy = res.y;

      // 死区限制：绝对值小于阈值视为 0，防止手抖
      if (Math.abs(vx) < DEADBAND) vx = 0;
      if (Math.abs(vy) < DEADBAND) vy = 0;

      // 线性映射：原始重力分量(0~1，对应 0~90°) 映射到 0~满速，90° 才满速
      vx = vx * SPEED_PER_G;
      vy = vy * SPEED_PER_G;

      // 限制最大速度（安全兜底）
      vx = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, vx));
      vy = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, vy));

      const xStr = vx.toFixed(2);
      const yStr = vy.toFixed(2);
      this.setData({ x_val: xStr, y_val: yStr });

      // 节流发送：每 100ms 发一次
      const now = Date.now();
      if (now - this._lastSendTime > SEND_INTERVAL_MS) {
        this._lastSendTime = now;
        this.sendDataToCar(xStr, yStr, this._w.toFixed(2));
      }
    });

    // 启动罗盘采集方向角（0~360°），用于控制旋转
    if (wx.offCompassChange) wx.offCompassChange();
    wx.startCompass({
      fail: () => {
        console.warn('该设备不支持罗盘/方向角，旋转控制不可用');
      }
    });
    wx.onCompassChange((res) => {
      const heading = res.direction; // 0~360°，正北为 0，顺时针增大
      this._heading = heading;

      // 首次读到方向角时，锁定当前朝向为参考（归零）
      if (this._referenceHeading === null || this._referenceHeading === undefined) {
        this._referenceHeading = heading;
      }

      // 相对参考方向的偏差，归一化到 -180 ~ 180 度
      const delta = normalizeAngleDelta(heading - this._referenceHeading);

      // 死区 + 线性映射到旋转速度（偏离 45° 即满速）
      let w = 0;
      if (Math.abs(delta) > HEADING_DEADBAND_DEG) {
        w = delta * HEADING_GAIN;
      }
      w = Math.max(-MAX_OMEGA, Math.min(MAX_OMEGA, w));

      this._w = w;
      this.setData({
        heading_val: heading.toFixed(0),
        w_val: w.toFixed(2)
      });
    });
  },

  // 6. 格式化并发送
  sendDataToCar(vx, vy, w) {
    if (!this.data.deviceId || !this.data.serviceId || !this.data.characteristicId) {
      return;
    }

    const str = `#${vx},${vy},${w}!`;
    const buffer = stringToArrayBuffer(str);

    wx.writeBLECharacteristicValue({
      deviceId: this.data.deviceId,
      serviceId: this.data.serviceId,
      characteristicId: this.data.characteristicId,
      value: buffer,
      success: () => {
        console.log('发送成功:', str);
      },
      fail: (err) => {
        console.error('发送失败:', str, err);
      }
    });
  },

  stopSensor() {
    if (wx.stopAccelerometer) wx.stopAccelerometer();
    if (wx.offAccelerometerChange) wx.offAccelerometerChange();
    if (wx.stopCompass) wx.stopCompass();
    if (wx.offCompassChange) wx.offCompassChange();
  },

  // 归零：把当前手机朝向锁定为参考方向（旋转的“正中”）
  zeroHeading() {
    this._referenceHeading = this._heading;
    this._w = 0;
    this.setData({ w_val: '0.00' });
  },

  disconnect() {
    if (this.data.deviceId) {
      wx.closeBLEConnection({ deviceId: this.data.deviceId });
    }
    this.setData({ statusText: '已断开', deviceId: '', serviceId: '', characteristicId: '' });
  },

  cleanup() {
    this.stopSensor();
    if (this.data.deviceId) {
      wx.closeBLEConnection({ deviceId: this.data.deviceId });
    }
    wx.closeBluetoothAdapter();
  }
});
