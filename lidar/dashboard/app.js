"use strict";

const $ = (id) => document.getElementById(id);
const host = window.location.hostname || "localhost";
const query = new URLSearchParams(window.location.search);
const rosbridgePort = query.get("rosbridgePort") || "19092";
const imageBridgePort = query.get("imageBridgePort") || "19093";
const ROSBRIDGE_URL = `ws://${host}:${rosbridgePort}`;
const IMAGEBRIDGE_URL = `ws://${host}:${imageBridgePort}`;
const colors = ["#ff6b7d", "#5ee09a", "#5ca8ff"];

const state = {
  socket: null,
  imageSocket: null,
  connectTimer: null,
  imageConnectTimer: null,
  reconnectTimer: null,
  imageReconnectTimer: null,
  reconnectDelay: 800,
  imageReconnectDelay: 1000,
  subscriptionTimers: [],
  imageSubscriptionTimers: [],
  cameraTopic: "/image_left_raw",
  latestCamera: null,
  latestDepth: null,
  cameraRenderPending: false,
  depthRenderPending: false,
  map: null,
  mapLayer: null,
  scan: null,
  mapToOdom: { x: 0, y: 0, yaw: 0 },
  imu: { gyro: [], accel: [] },
  rates: new Map(),
  lastMessageAt: 0,
};

function setConnection(mode, title, detail) {
  $("connectionDot").className = `status-dot ${mode}`;
  $("connectionText").textContent = title;
  $("connectionDetail").textContent = detail;
}

function sendOn(socket, message) {
  if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
}

function send(message) { sendOn(state.socket, message); }
function sendImage(message) { sendOn(state.imageSocket, message); }

function subscribe(id, topic, throttleRate = 0) {
  send({ op: "subscribe", id, topic, throttle_rate: throttleRate, queue_length: 1, compression: "none" });
}

function unsubscribe(id, topic) {
  send({ op: "unsubscribe", id, topic });
}

function subscribeImage(id, topic, throttleRate = 0) {
  sendImage({ op: "subscribe", id, topic, throttle_rate: throttleRate, queue_length: 1, compression: "none" });
}

function unsubscribeImage(id, topic) {
  sendImage({ op: "unsubscribe", id, topic });
}

function subscriptionPlan() {
  return [
    [0, "imu", "/imu/left/data_calibrated", 50],
    [150, "scan", "/scan", 200],
    [300, "map", "/map", 500],
    [450, "tf", "/tf", 100],
    [600, "tf_static", "/tf_static", 0],
  ];
}

function imageSubscriptionPlan() {
  return [
    [0, "camera", state.cameraTopic, 500],
    [300, "depth", "/StereoNetNode/stereonet_depth", 750],
  ];
}

function clearSubscriptionTimers() {
  state.subscriptionTimers.forEach(clearTimeout);
  state.subscriptionTimers = [];
}

function clearImageSubscriptionTimers() {
  state.imageSubscriptionTimers.forEach(clearTimeout);
  state.imageSubscriptionTimers = [];
}

function subscribeAll() {
  clearSubscriptionTimers();
  for (const [delay, id, topic, throttleRate] of subscriptionPlan()) {
    const timer = setTimeout(() => subscribe(id, topic, throttleRate), delay);
    state.subscriptionTimers.push(timer);
  }
}

function subscribeImages() {
  clearImageSubscriptionTimers();
  for (const [delay, id, topic, throttleRate] of imageSubscriptionPlan()) {
    const timer = setTimeout(() => subscribeImage(id, topic, throttleRate), delay);
    state.imageSubscriptionTimers.push(timer);
  }
}

function unsubscribeAll() {
  clearSubscriptionTimers();
  for (const [_delay, id, topic] of subscriptionPlan()) unsubscribe(id, topic);
}

function unsubscribeImages() {
  clearImageSubscriptionTimers();
  for (const [_delay, id, topic] of imageSubscriptionPlan()) unsubscribeImage(id, topic);
}

function handleSocketMessage(event) {
  let envelope;
  try { envelope = JSON.parse(event.data); } catch (_error) { return; }
  if (envelope.op !== "publish" || !envelope.msg) return;
  state.lastMessageAt = performance.now();
  $("lastUpdate").textContent = `마지막 수신: ${new Date().toLocaleTimeString("ko-KR")}`;
  routeMessage(envelope.topic, envelope.msg);
}

function scheduleReconnect(socket) {
  if (state.socket !== socket) return;
  clearTimeout(state.connectTimer);
  clearSubscriptionTimers();
  clearTimeout(state.imageConnectTimer);
  clearTimeout(state.imageReconnectTimer);
  clearImageSubscriptionTimers();
  const imageSocket = state.imageSocket;
  state.imageSocket = null;
  if (imageSocket && imageSocket.readyState < WebSocket.CLOSING) imageSocket.close();
  state.socket = null;
  setConnection("offline", "연결 끊김 · 재시도 중", ROSBRIDGE_URL);
  clearTimeout(state.reconnectTimer);
  state.reconnectTimer = setTimeout(connect, state.reconnectDelay);
  state.reconnectDelay = Math.min(state.reconnectDelay * 1.7, 6000);
}

function connect() {
  if (state.socket && state.socket.readyState < WebSocket.CLOSING) return;
  clearTimeout(state.reconnectTimer);
  clearTimeout(state.connectTimer);
  setConnection("connecting", "ROSBridge 연결 중", ROSBRIDGE_URL);
  const socket = new WebSocket(ROSBRIDGE_URL);
  state.socket = socket;
  state.connectTimer = setTimeout(() => {
    if (state.socket !== socket || socket.readyState !== WebSocket.CONNECTING) return;
    setConnection("offline", "연결 시간 초과 · 재시도 중", ROSBRIDGE_URL);
    try { socket.close(); } catch (_error) { scheduleReconnect(socket); }
    setTimeout(() => scheduleReconnect(socket), 300);
  }, 8000);

  socket.addEventListener("open", () => {
    if (state.socket !== socket) return;
    clearTimeout(state.connectTimer);
    state.reconnectDelay = 800;
    setConnection("online", "로봇 연결됨", "텔레메트리 연결됨 · 영상 연결 중");
    subscribeAll();
    connectImages();
  });

  socket.addEventListener("message", handleSocketMessage);

  socket.addEventListener("close", () => {
    scheduleReconnect(socket);
  });

  socket.addEventListener("error", () => socket.close());
}

function scheduleImageReconnect(socket) {
  if (state.imageSocket !== socket) return;
  clearTimeout(state.imageConnectTimer);
  clearImageSubscriptionTimers();
  state.imageSocket = null;
  if (state.socket?.readyState !== WebSocket.OPEN) return;
  setConnection("online", "로봇 연결됨", "텔레메트리 연결됨 · 영상 재연결 중");
  clearTimeout(state.imageReconnectTimer);
  state.imageReconnectTimer = setTimeout(connectImages, state.imageReconnectDelay);
  state.imageReconnectDelay = Math.min(state.imageReconnectDelay * 1.7, 6000);
}

function connectImages() {
  if (state.socket?.readyState !== WebSocket.OPEN) return;
  if (state.imageSocket && state.imageSocket.readyState < WebSocket.CLOSING) return;
  clearTimeout(state.imageReconnectTimer);
  clearTimeout(state.imageConnectTimer);
  const socket = new WebSocket(IMAGEBRIDGE_URL);
  state.imageSocket = socket;
  state.imageConnectTimer = setTimeout(() => {
    if (state.imageSocket !== socket || socket.readyState !== WebSocket.CONNECTING) return;
    try { socket.close(); } catch (_error) { scheduleImageReconnect(socket); }
    setTimeout(() => scheduleImageReconnect(socket), 300);
  }, 8000);

  socket.addEventListener("open", () => {
    if (state.imageSocket !== socket) return;
    clearTimeout(state.imageConnectTimer);
    state.imageReconnectDelay = 1000;
    setConnection("online", "로봇 연결됨", "텔레메트리 · 영상 연결됨");
    subscribeImages();
  });
  socket.addEventListener("message", handleSocketMessage);
  socket.addEventListener("close", () => scheduleImageReconnect(socket));
  socket.addEventListener("error", () => socket.close());
}

function noteRate(key, elementId, label = "") {
  const now = performance.now();
  const samples = state.rates.get(key) || [];
  samples.push(now);
  while (samples.length > 2 && samples[0] < now - 3000) samples.shift();
  state.rates.set(key, samples);
  if (samples.length > 1) {
    const hz = ((samples.length - 1) * 1000) / (samples.at(-1) - samples[0]);
    $(elementId).textContent = `${label}${hz.toFixed(1)} Hz`;
  }
}

function routeMessage(topic, msg) {
  if (topic === state.cameraTopic) handleCamera(msg);
  else if (topic === "/StereoNetNode/stereonet_depth") handleDepth(msg);
  else if (topic === "/imu/left/data_calibrated") handleImu(msg);
  else if (topic === "/scan") handleScan(msg);
  else if (topic === "/map") handleMap(msg);
  else if (topic === "/tf" || topic === "/tf_static") handleTf(msg);
}

function byteArray(data) {
  if (data instanceof Uint8Array) return data;
  if (Array.isArray(data)) return Uint8Array.from(data, (value) => value & 255);
  if (typeof data === "string") {
    const raw = atob(data);
    const output = new Uint8Array(raw.length);
    for (let index = 0; index < raw.length; index += 1) output[index] = raw.charCodeAt(index);
    return output;
  }
  return new Uint8Array();
}

function clamp8(value) { return value < 0 ? 0 : value > 255 ? 255 : value; }

function handleCamera(msg) {
  state.latestCamera = msg;
  noteRate("camera", "cameraRate");
  $("cameraInfo").textContent = `${String(msg.encoding || "unknown").toUpperCase()} · ${msg.width}×${msg.height}`;
  $("cameraAge").textContent = `${state.cameraTopic.includes("left") ? "LEFT" : "RIGHT"} · 180°`;
  if (!state.cameraRenderPending) {
    state.cameraRenderPending = true;
    requestAnimationFrame(renderCamera);
  }
}

function renderCamera() {
  state.cameraRenderPending = false;
  const msg = state.latestCamera;
  if (!msg) return;
  const width = Number(msg.width);
  const height = Number(msg.height);
  const bytes = byteArray(msg.data);
  if (!width || !height || !bytes.length) return;
  const canvas = $("cameraCanvas");
  if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
  const context = canvas.getContext("2d", { alpha: false });
  const image = context.createImageData(width, height);
  const out = image.data;
  const encoding = String(msg.encoding || "").toLowerCase();

  if (encoding === "nv12") {
    const stride = Number(msg.step) || width;
    const uvStart = stride * height;
    for (let y = 0; y < height; y += 1) {
      const yRow = y * stride;
      const uvRow = uvStart + Math.floor(y / 2) * stride;
      for (let x = 0; x < width; x += 1) {
        const luminance = bytes[yRow + x] || 0;
        const uvIndex = uvRow + (x & ~1);
        const u = (bytes[uvIndex] ?? 128) - 128;
        const v = (bytes[uvIndex + 1] ?? 128) - 128;
        const c = Math.max(0, luminance - 16);
        const pixel = ((height - 1 - y) * width + (width - 1 - x)) * 4;
        out[pixel] = clamp8((298 * c + 409 * v + 128) >> 8);
        out[pixel + 1] = clamp8((298 * c - 100 * u - 208 * v + 128) >> 8);
        out[pixel + 2] = clamp8((298 * c + 516 * u + 128) >> 8);
        out[pixel + 3] = 255;
      }
    }
  } else {
    const channels = encoding === "rgba8" || encoding === "bgra8" ? 4 : 3;
    const bgr = encoding.startsWith("bgr");
    for (let index = 0; index < width * height; index += 1) {
      const source = index * channels;
      const target = (width * height - 1 - index) * 4;
      out[target] = bytes[source + (bgr ? 2 : 0)] || 0;
      out[target + 1] = bytes[source + 1] || 0;
      out[target + 2] = bytes[source + (bgr ? 0 : 2)] || 0;
      out[target + 3] = 255;
    }
  }
  context.putImageData(image, 0, 0);
  $("cameraEmpty").classList.add("hidden");
}

function handleDepth(msg) {
  state.latestDepth = msg;
  noteRate("depth", "depthRate");
  if (!state.depthRenderPending) {
    state.depthRenderPending = true;
    requestAnimationFrame(renderDepth);
  }
}

function depthRgb(t) {
  t = Math.max(0, Math.min(1, t));
  if (t < 0.25) return [70 - t * 120, 50 + t * 620, 210 + t * 130];
  if (t < 0.5) return [15 + (t - .25) * 250, 205 + (t - .25) * 160, 240 - (t - .25) * 400];
  if (t < 0.75) return [78 + (t - .5) * 680, 235 + (t - .5) * 45, 140 - (t - .5) * 400];
  return [248, 246 - (t - .75) * 650, 65 - (t - .75) * 220];
}

function renderDepth() {
  state.depthRenderPending = false;
  const msg = state.latestDepth;
  if (!msg) return;
  const width = Number(msg.width);
  const height = Number(msg.height);
  const bytes = byteArray(msg.data);
  if (!width || !height || bytes.length < width * height * 2) return;
  const minDisplay = Number($("depthMin").value) || 150;
  const maxDisplay = Math.max(minDisplay + 1, Number($("depthMax").value) || 4000);
  const canvas = $("depthCanvas");
  if (canvas.width !== width || canvas.height !== height) { canvas.width = width; canvas.height = height; }
  const context = canvas.getContext("2d", { alpha: false });
  const image = context.createImageData(width, height);
  const out = image.data;
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  let valid = 0;
  let minSeen = Infinity;
  let maxSeen = 0;

  for (let index = 0; index < width * height; index += 1) {
    const value = view.getUint16(index * 2, !msg.is_bigendian);
    const target = (width * height - 1 - index) * 4;
    if (value === 0 || value === 65535) {
      out[target] = 3; out[target + 1] = 8; out[target + 2] = 14; out[target + 3] = 255;
      continue;
    }
    valid += 1;
    minSeen = Math.min(minSeen, value);
    maxSeen = Math.max(maxSeen, value);
    const [red, green, blue] = depthRgb((value - minDisplay) / (maxDisplay - minDisplay));
    out[target] = clamp8(red); out[target + 1] = clamp8(green); out[target + 2] = clamp8(blue); out[target + 3] = 255;
  }

  context.putImageData(image, 0, 0);
  const centerSourceX = width - 1 - Math.floor(width / 2);
  const centerSourceY = height - 1 - Math.floor(height / 2);
  const center = view.getUint16((centerSourceY * width + centerSourceX) * 2, !msg.is_bigendian);
  $("centerDepth").textContent = center && center !== 65535 ? `${(center / 1000).toFixed(2)} m` : "—";
  $("depthInfo").textContent = `${String(msg.encoding).toUpperCase()} · ${width}×${height} · 180°`;
  $("depthStats").textContent = valid ? `valid ${((valid / (width * height)) * 100).toFixed(1)}% · ${minSeen}–${maxSeen} mm` : "유효 깊이 없음";
  $("depthEmpty").classList.add("hidden");
}

function quaternionToRpy(q = {}) {
  const x = Number(q.x) || 0, y = Number(q.y) || 0, z = Number(q.z) || 0, w = Number(q.w) || 1;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitchTerm = 2 * (w * y - z * x);
  const pitch = Math.abs(pitchTerm) >= 1 ? Math.sign(pitchTerm) * Math.PI / 2 : Math.asin(pitchTerm);
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [roll, pitch, yaw];
}

function vector(message = {}) { return [Number(message.x) || 0, Number(message.y) || 0, Number(message.z) || 0]; }

function handleImu(msg) {
  noteRate("imu", "imuRate");
  const now = performance.now() / 1000;
  const gyro = vector(msg.angular_velocity);
  const accel = vector(msg.linear_acceleration);
  state.imu.gyro.push({ t: now, values: gyro });
  state.imu.accel.push({ t: now, values: accel });
  while (state.imu.gyro.length > 240) state.imu.gyro.shift();
  while (state.imu.accel.length > 240) state.imu.accel.shift();
  const rpy = quaternionToRpy(msg.orientation).map((value) => value * 180 / Math.PI);
  $("rollValue").textContent = `${rpy[0].toFixed(1)}°`;
  $("pitchValue").textContent = `${rpy[1].toFixed(1)}°`;
  $("yawValue").textContent = `${rpy[2].toFixed(1)}°`;
  $("accelNorm").textContent = `${Math.hypot(...accel).toFixed(2)} m/s²`;
  drawChart($("gyroChart"), state.imu.gyro, 0.1);
  drawChart($("accelChart"), state.imu.accel, 10);
}

function drawChart(canvas, samples, minimumRange) {
  const context = canvas.getContext("2d");
  const width = canvas.width, height = canvas.height, padding = 18;
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#07111b"; context.fillRect(0, 0, width, height);
  context.strokeStyle = "rgba(137,160,179,.14)"; context.lineWidth = 1;
  for (let index = 0; index <= 4; index += 1) {
    const y = padding + (height - padding * 2) * index / 4;
    context.beginPath(); context.moveTo(padding, y); context.lineTo(width - padding, y); context.stroke();
  }
  if (samples.length < 2) return;
  const maxAbs = Math.max(minimumRange, ...samples.flatMap((sample) => sample.values.map(Math.abs))) * 1.15;
  const oldest = samples[0].t, newest = samples.at(-1).t, span = Math.max(1, newest - oldest);
  for (let axis = 0; axis < 3; axis += 1) {
    context.strokeStyle = colors[axis]; context.lineWidth = 2; context.beginPath();
    samples.forEach((sample, index) => {
      const x = padding + ((sample.t - oldest) / span) * (width - padding * 2);
      const y = height / 2 - (sample.values[axis] / maxAbs) * (height / 2 - padding);
      if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
    });
    context.stroke();
  }
}

function handleMap(msg) {
  noteRate("map", "mapRate");
  const width = Number(msg.info?.width), height = Number(msg.info?.height);
  if (!width || !height) return;
  let data;
  if (Array.isArray(msg.data)) data = Int8Array.from(msg.data);
  else { const raw = byteArray(msg.data); data = new Int8Array(raw.buffer, raw.byteOffset, raw.byteLength); }
  state.map = { width, height, resolution: Number(msg.info.resolution), origin: msg.info.origin?.position || { x: 0, y: 0 }, data };
  state.mapLayer = buildMapLayer(state.map);
  $("mapInfo").textContent = `${width}×${height} cells · ${state.map.resolution.toFixed(2)} m/cell`;
  $("mapEmpty").classList.add("hidden");
  drawMap();
}

function buildMapLayer(map) {
  const layer = document.createElement("canvas");
  layer.width = map.width; layer.height = map.height;
  const context = layer.getContext("2d");
  const image = context.createImageData(map.width, map.height);
  for (let index = 0; index < map.width * map.height; index += 1) {
    const value = map.data[index] ?? -1;
    const target = index * 4;
    let shade;
    if (value < 0) { image.data[target] = 18; image.data[target + 1] = 30; image.data[target + 2] = 42; }
    else { shade = 242 - Math.round(Math.max(0, Math.min(100, value)) * 2.2); image.data[target] = shade; image.data[target + 1] = shade + 3; image.data[target + 2] = shade + 5; }
    image.data[target + 3] = 255;
  }
  context.putImageData(image, 0, 0);
  return layer;
}

function handleScan(msg) {
  noteRate("scan", "scanRate");
  state.scan = msg;
  const finite = (msg.ranges || []).filter(Number.isFinite);
  $("scanInfo").textContent = finite.length ? `${finite.length}/${msg.ranges.length} points · ${Math.min(...finite).toFixed(2)}–${Math.max(...finite).toFixed(2)} m` : "유효 스캔 없음";
  drawMap();
}

function handleTf(msg) {
  for (const transform of msg.transforms || []) {
    const parent = String(transform.header?.frame_id || "").replace(/^\//, "");
    const child = String(transform.child_frame_id || "").replace(/^\//, "");
    if (parent === "map" && (child === "odom" || child === "laser_frame")) {
      const translation = transform.transform?.translation || {};
      const rotation = transform.transform?.rotation || {};
      state.mapToOdom = { x: Number(translation.x) || 0, y: Number(translation.y) || 0, yaw: quaternionToRpy(rotation)[2] };
    }
  }
}

function drawMap() {
  const map = state.map, layer = state.mapLayer;
  if (!map || !layer) return;
  const canvas = $("mapCanvas"), context = canvas.getContext("2d");
  const padding = 30;
  const scale = Math.min((canvas.width - padding * 2) / map.width, (canvas.height - padding * 2) / map.height);
  const left = (canvas.width - map.width * scale) / 2;
  const top = (canvas.height - map.height * scale) / 2;
  context.fillStyle = "#050b12"; context.fillRect(0, 0, canvas.width, canvas.height);
  context.save(); context.imageSmoothingEnabled = false;
  context.translate(left, top + map.height * scale); context.scale(scale, -scale); context.drawImage(layer, 0, 0); context.restore();

  const worldToCanvas = (x, y) => [left + ((x - Number(map.origin.x || 0)) / map.resolution) * scale, top + map.height * scale - ((y - Number(map.origin.y || 0)) / map.resolution) * scale];
  const pose = state.mapToOdom;
  if (state.scan?.ranges) {
    context.fillStyle = "rgba(54,211,229,.86)";
    const cosine = Math.cos(pose.yaw), sine = Math.sin(pose.yaw);
    state.scan.ranges.forEach((range, index) => {
      if (!Number.isFinite(range) || range < state.scan.range_min || range > state.scan.range_max) return;
      const angle = Number(state.scan.angle_min) + index * Number(state.scan.angle_increment);
      const localX = range * Math.cos(angle), localY = range * Math.sin(angle);
      const worldX = pose.x + cosine * localX - sine * localY;
      const worldY = pose.y + sine * localX + cosine * localY;
      const [x, y] = worldToCanvas(worldX, worldY);
      context.fillRect(x - 1.2, y - 1.2, 2.4, 2.4);
    });
  }
  const [robotX, robotY] = worldToCanvas(pose.x, pose.y);
  context.fillStyle = "#ff6b7d"; context.beginPath(); context.arc(robotX, robotY, 5, 0, Math.PI * 2); context.fill();
  context.strokeStyle = "#ff6b7d"; context.lineWidth = 2.5; context.beginPath(); context.moveTo(robotX, robotY); context.lineTo(robotX + Math.cos(pose.yaw) * 16, robotY - Math.sin(pose.yaw) * 16); context.stroke();
}

$("cameraTopic").addEventListener("change", (event) => {
  const previous = state.cameraTopic;
  state.cameraTopic = event.target.value;
  state.latestCamera = null;
  $("cameraEmpty").classList.remove("hidden");
  unsubscribeImage("camera", previous);
  subscribeImage("camera", state.cameraTopic, 500);
});
$("depthMin").addEventListener("change", renderDepth);
$("depthMax").addEventListener("change", renderDepth);

setInterval(() => {
  if (state.socket?.readyState === WebSocket.OPEN && state.lastMessageAt && performance.now() - state.lastMessageAt > 5000) {
    setConnection("connecting", "연결됨 · 센서 데이터 대기", ROSBRIDGE_URL);
  }
}, 1000);

window.addEventListener("pagehide", () => {
  clearTimeout(state.connectTimer);
  clearTimeout(state.imageConnectTimer);
  clearTimeout(state.reconnectTimer);
  clearTimeout(state.imageReconnectTimer);
  unsubscribeAll();
  unsubscribeImages();
  const socket = state.socket;
  const imageSocket = state.imageSocket;
  state.socket = null;
  state.imageSocket = null;
  if (socket && socket.readyState < WebSocket.CLOSING) socket.close(1000, "page hidden");
  if (imageSocket && imageSocket.readyState < WebSocket.CLOSING) imageSocket.close(1000, "page hidden");
});

connect();
