#include <Arduino.h>
#include <math.h>
#include "esp_http_server.h"
#include "esp_camera.h"
#include "img_converters.h"

// ====================== PIN SETUP ======================
#define L_IN1   2
#define L_IN2   3
#define R_IN1   43
#define R_IN2   44
#define RELAY_PIN 7

// Safety: jump duration (ms)
static const uint32_t JUMP_PULSE_MS = 4650; // 4.65 seconds

// PWM config
static const int PWM_FREQ = 20000;
static const int PWM_RES  = 8;

// Camera uses LEDC channel 0 for XCLK -> keep motors off channel 0
static const int CH_L1 = 1;
static const int CH_L2 = 2;
static const int CH_R1 = 3;
static const int CH_R2 = 4;

// ====================== Globals expected by Main.ino ======================
volatile unsigned int motor_speed = 180;          // 0..255
volatile unsigned long previous_time = 0;
volatile unsigned long move_interval = 700;       // ms auto-stop interval (joystick refreshes it)
uint8_t robo = 0;

static int noStop = 0;

// ===== Timed relay pulse safety =====
static volatile bool     jump_active = false;
static volatile uint32_t jump_end_ms = 0;

static void jump_watchdog_task(void *param)
{
  (void)param;
  while (true) {
    if (jump_active) {
      int32_t remaining = (int32_t)(jump_end_ms - millis());
      if (remaining <= 0) {
        digitalWrite(RELAY_PIN, LOW);
        jump_active = false;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void jump_watchdog_start()
{
  static bool started = false;
  if (started) return;
  started = true;

  xTaskCreatePinnedToCore(
    jump_watchdog_task,
    "jump_watchdog",
    2048,
    nullptr,
    1,
    nullptr,
    1
  );
}

static void start_jump_pulse(uint32_t duration_ms)
{
  // hard safety clamps
  if (duration_ms < 50) duration_ms = 50;
  if (duration_ms > 10000) duration_ms = 10000;

  digitalWrite(RELAY_PIN, HIGH);
  jump_active = true;
  jump_end_ms = millis() + duration_ms;
}

// ====================== Motor helpers (DRV8833) ======================
static inline void pwmWriteCh(int ch, int duty)
{
  duty = constrain(duty, 0, 255);
  ledcWriteChannel(ch, duty);
}

static void motor_stop_all()
{
  pwmWriteCh(CH_L1, 0);
  pwmWriteCh(CH_L2, 0);
  pwmWriteCh(CH_R1, 0);
  pwmWriteCh(CH_R2, 0);
}

// leftSpeed/rightSpeed: -255..255
static void motor_drive(int leftSpeed, int rightSpeed)
{
  leftSpeed  = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // Left motor
  if (leftSpeed > 0) {
    pwmWriteCh(CH_L1, leftSpeed);
    pwmWriteCh(CH_L2, 0);
  } else if (leftSpeed < 0) {
    pwmWriteCh(CH_L1, 0);
    pwmWriteCh(CH_L2, -leftSpeed);
  } else {
    pwmWriteCh(CH_L1, 0);
    pwmWriteCh(CH_L2, 0);
  }

  // Right motor
  if (rightSpeed > 0) {
    pwmWriteCh(CH_R1, rightSpeed);
    pwmWriteCh(CH_R2, 0);
  } else if (rightSpeed < 0) {
    pwmWriteCh(CH_R1, 0);
    pwmWriteCh(CH_R2, -rightSpeed);
  } else {
    pwmWriteCh(CH_R1, 0);
    pwmWriteCh(CH_R2, 0);
  }
}

void robot_stop()  { motor_stop_all(); }

void robot_setup()
{
  ledcAttachChannel(L_IN1, PWM_FREQ, PWM_RES, CH_L1);
  ledcAttachChannel(L_IN2, PWM_FREQ, PWM_RES, CH_L2);
  ledcAttachChannel(R_IN1, PWM_FREQ, PWM_RES, CH_R1);
  ledcAttachChannel(R_IN2, PWM_FREQ, PWM_RES, CH_R2);

  motor_stop_all();

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  jump_watchdog_start();
}

// ====================== Joystick control (GAME FEEL) ======================
static float expo(float v, float e)
{
  float a = fabsf(v);
  return (1.0f - e) * v + e * (v * a);
}

// x,y expected in range [-100, 100]
// x: turn, y: forward
static void apply_joystick(int x, int y)
{
  x = constrain(x, -100, 100);
  y = constrain(y, -100, 100);

  // FIX: invert left/right turning
  x = -x;

  const int DZ = 10;
  if (abs(x) < DZ) x = 0;
  if (abs(y) < DZ) y = 0;

  if (x == 0 && y == 0) {
    robot_stop();
    robo = 0;
    return;
  }

  float xf = x / 100.0f;
  float yf = y / 100.0f;

  xf = expo(xf, 0.70f);
  yf = expo(yf, 0.70f);

  float left  = yf + xf;
  float right = yf - xf;

  left  = constrain(left,  -1.0f, 1.0f);
  right = constrain(right, -1.0f, 1.0f);

  int l = (int)lroundf(left  * (float)motor_speed);
  int r = (int)lroundf(right * (float)motor_speed);

  motor_drive(l, r);

  previous_time = millis();
  robo = 1;
}

// ====================== Stream server (port 81) ======================
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY     = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART         = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  delay(120);

  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      delay(20);
      continue;
    }

    uint8_t *jpg_buf = fb->buf;
    size_t   jpg_len = fb->len;

    if (fb->format != PIXFORMAT_JPEG) {
      bool ok = frame2jpg(fb, 80, &jpg_buf, &jpg_len);
      esp_camera_fb_return(fb);
      fb = nullptr;
      if (!ok) {
        if (jpg_buf) free(jpg_buf);
        delay(20);
        continue;
      }
    }

    char part[64];
    size_t hlen = snprintf(part, sizeof(part), STREAM_PART, (unsigned int)jpg_len);

    if (httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY)) != ESP_OK ||
        httpd_resp_send_chunk(req, part, hlen) != ESP_OK ||
        httpd_resp_send_chunk(req, (const char*)jpg_buf, jpg_len) != ESP_OK) {

      if (!fb && jpg_buf) free(jpg_buf);
      break;
    }

    if (fb) esp_camera_fb_return(fb);
    else if (jpg_buf) free(jpg_buf);

    vTaskDelay(1);
  }

  return ESP_OK;
}

// ====================== UI HTML (FPV HUD overlays + quality slider) ======================
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1, maximum-scale=1, user-scalable=no">
<title>XIAO S3 Robot</title>
<style>
  :root{
    --bg:#0f1115;
    --panel:#151a22;
    --text:#e7e7e7;
    --muted:#a7b0c0;
    --ok:#22c55e;
    --stop:#ef4444;
    --btn:#262c38;
    --btn2:#2f3748;
    --radius:18px;
    --pad:12px;
  }
  html,body{height:100%;}
  body{
    margin:0;
    font-family:-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,Arial,sans-serif;
    background:var(--bg);
    color:var(--text);
    -webkit-user-select:none; user-select:none;
    overscroll-behavior:none;
  }
  .wrap{max-width:1200px;margin:0 auto;padding:var(--pad);}
  .topbar{
    display:flex;align-items:center;justify-content:space-between;
    margin:4px 2px 10px;
  }
  .title{font-weight:900;font-size:18px;margin:0;}
  .pill{
    font-size:12px;color:var(--muted);
    background:rgba(21,26,34,.85);
    padding:6px 10px;border-radius:999px;
    border:1px solid rgba(255,255,255,.08);
  }

  .panel{
    background:var(--panel);
    border:1px solid rgba(255,255,255,.06);
    border-radius:var(--radius);
    padding:12px;
  }

  /* Video stage */
  .videoStage{ position:relative; }
  .videoBox{
    width:100%;
    aspect-ratio: 16 / 9;
    background:#000;
    border-radius:var(--radius);
    overflow:hidden;
    border:1px solid rgba(255,255,255,.10);
  }
  img{
    width:100%;
    height:100%;
    object-fit: cover;   /* no black bars */
    display:block;
  }

  /* HUD overlays */
  .overlay{
    position:absolute;
    z-index:5;
    display:flex;
    align-items:center;
    gap:10px;

    padding:10px;
    border-radius:16px;
    background:rgba(15,17,21,0.35);
    border:1px solid rgba(255,255,255,0.10);
    backdrop-filter: blur(8px);
    -webkit-backdrop-filter: blur(8px);
  }
  .overlayLeft{
    left:12px;
    bottom:12px;
    flex-direction:column;
    gap:6px;
  }
  .overlayRight{
    right:12px;
    bottom:12px;
    flex-direction:column;
    gap:8px;
  }
  .overlayBottom{
    left:50%;
    transform:translateX(-50%);
    bottom:12px;
    gap:10px;
  }

  /* Buttons (small) */
  .btn{
    padding:10px 12px;
    border-radius:12px;
    border:1px solid rgba(255,255,255,.10);
    background:var(--btn);
    color:var(--text);
    font-size:14px;
    font-weight:900;
    letter-spacing:.2px;
    touch-action:manipulation;
  }
  .btn:active{transform:scale(.99); background:var(--btn2);}
  .btn.ok{background:rgba(34,197,94,.18); border-color:rgba(34,197,94,.30);}
  .btn.stop{background:rgba(239,68,68,.18); border-color:rgba(239,68,68,.30);}

  .btnSquare{
    width:96px;
    height:40px;
    font-size:13px;
    display:flex;
    align-items:center;
    justify-content:center;
  }

  /* Joystick */
  .joyBase{
    width:120px;height:120px;
    border-radius:999px;
    background:rgba(255,255,255,.06);
    border:1px solid rgba(255,255,255,.12);
    position:relative;
    touch-action:none;
  }
  .joyStick{
    width:46px;height:46px;
    border-radius:999px;
    background:rgba(255,255,255,.18);
    border:1px solid rgba(255,255,255,.20);
    position:absolute;
    left:50%; top:50%;
    transform:translate(-50%,-50%);
    box-shadow:0 10px 30px rgba(0,0,0,.35);
  }
  .muted{color:var(--muted);font-size:12px;}
  .joyLabel{color:var(--muted); font-size:12px;}

  /* Sliders under video */
  .sliderPanel{margin-top:12px;}
  .sliderRow{display:flex;align-items:center;gap:10px;margin-top:8px;}
  input[type=range]{width:100%;}
  .kv{display:flex;justify-content:space-between;margin-top:8px;}

  @media (orientation: portrait){
    .videoBox{aspect-ratio: 4 / 3;}
    .overlayLeft{left:10px; bottom:10px;}
    .overlayRight{right:10px; bottom:10px;}
    .overlayBottom{bottom:10px;}
  }
</style>
</head>
<body>
<div class="wrap">
  <div class="topbar">
    <div class="title">XIAO ESP32S3 Robot</div>
    <div class="pill" id="status">Stream: off</div>
  </div>

  <div class="panel videoStage">
    <div class="videoBox">
      <img id="stream" src="" />
    </div>

    <!-- LEFT overlay: joystick -->
    <div class="overlay overlayLeft">
      <div class="joyBase" id="joyBase">
        <div class="joyStick" id="joyStick"></div>
      </div>
      <div class="joyLabel" id="joyReadout">x:0 y:0</div>
    </div>

    <!-- RIGHT overlay: relay buttons -->
    <div class="overlay overlayRight">
      <button class="btn btnSquare ok" onclick="relay(true)">JUMP</button>
      <button class="btn btnSquare stop" onclick="relay(false)">STOP</button>
    </div>

    <!-- BOTTOM overlay: stream + stop -->
    <div class="overlay overlayBottom">
      <button id="toggle-stream" class="btn">Start Video</button>
      <button class="btn stop" onclick="sendStop()">STOP</button>
    </div>
  </div>

  <div class="panel sliderPanel">
    <!-- Speed -->
    <div class="sliderRow">
      <div class="muted" style="min-width:64px;font-weight:800;">Speed</div>
      <input type="range" min="60" max="255" value="180" id="spd" />
      <div style="min-width:44px;text-align:right;font-weight:900;" id="spdval">180</div>
    </div>

    <!-- Quality (lower = better) -->
    <div class="sliderRow">
      <div class="muted" style="min-width:64px;font-weight:800;">Quality</div>
      <input type="range" min="6" max="20" value="8" id="qual" />
      <div style="min-width:44px;text-align:right;font-weight:900;" id="qualval">8</div>
    </div>
    <div class="muted" style="margin-top:6px;">
      Lower number = better image, slightly more bandwidth.
    </div>

    <div class="kv">
      <div class="muted">Stream: <span id="streamUrl"></span></div>
      <div class="muted" id="ip"></div>
    </div>
  </div>
</div>

<script>
const host = window.location.hostname;
document.getElementById('ip').textContent = host;
document.getElementById('streamUrl').textContent = host + ':81/stream';

function fetchNoCache(url){
  return fetch(url, { cache: "no-store" }).catch(()=>{});
}

function relay(on){
  if(on){
    // Safety pulse: 4.65 seconds (firmware also enforces watchdog)
    fetchNoCache('/control?var=jump&val=4650');
  } else {
    fetchNoCache('/control?var=flashoff&val=0');
  }
}

function sendStop(){
  fetchNoCache('/control?var=car&val=3');
}

// ===== Stream toggle =====
const img = document.getElementById('stream');
const btn = document.getElementById('toggle-stream');
const status = document.getElementById('status');
let streaming = false;

btn.onclick = () => {
  streaming = !streaming;
  if(streaming){
    img.src = 'http://' + host + ':81/stream?rand=' + Date.now();
    btn.textContent = 'Stop Video';
    status.textContent = 'Stream: on';
  }else{
    img.src = '';
    btn.textContent = 'Start Video';
    status.textContent = 'Stream: off';
  }
};

// ===== Speed slider =====
const spd = document.getElementById('spd');
const spdval = document.getElementById('spdval');
spd.oninput = () => {
  spdval.textContent = spd.value;
  fetchNoCache('/control?var=speed&val=' + spd.value);
};

// ===== Quality slider =====
const qual = document.getElementById('qual');
const qualval = document.getElementById('qualval');

// simple debounce so you don't spam the ESP32 while dragging
let qualTimer = null;
qual.oninput = () => {
  qualval.textContent = qual.value;
  if (qualTimer) clearTimeout(qualTimer);
  qualTimer = setTimeout(() => {
    fetchNoCache('/control?var=quality&val=' + qual.value);
  }, 120);
};

// ===== Joystick -> send var=joy&val=x,y =====
const joyBase = document.getElementById('joyBase');
const joyStick = document.getElementById('joyStick');
const joyReadout = document.getElementById('joyReadout');

let active = false;
let center = {x:0,y:0};
let radius = 1;
let lastX = 0, lastY = 0;
let sendTimer = null;

function setStick(px, py){
  joyStick.style.left = px + 'px';
  joyStick.style.top  = py + 'px';
  joyStick.style.transform = 'translate(-50%,-50%)';
}

function updateGeometry(){
  const r = joyBase.getBoundingClientRect();
  center.x = r.width/2;
  center.y = r.height/2;
  radius = Math.min(r.width, r.height) * 0.40;
  setStick(center.x, center.y);
}

function startSending(){
  if (sendTimer) return;
  sendTimer = setInterval(() => {
    fetchNoCache('/control?var=joy&val=' + lastX + ',' + lastY);
  }, 60);
}

function stopSending(){
  if (sendTimer){
    clearInterval(sendTimer);
    sendTimer = null;
  }
}

function pointerPos(e){
  const r = joyBase.getBoundingClientRect();
  const x = (e.clientX - r.left);
  const y = (e.clientY - r.top);
  return {x,y};
}

function onDown(e){
  e.preventDefault();
  active = true;
  joyBase.setPointerCapture(e.pointerId);
  updateGeometry();
  startSending();
  onMove(e);
}

function onMove(e){
  if(!active) return;
  e.preventDefault();
  const p = pointerPos(e);
  let dx = p.x - center.x;
  let dy = p.y - center.y;

  const dist = Math.hypot(dx, dy);
  if (dist > radius){
    const s = radius / dist;
    dx *= s; dy *= s;
  }

  setStick(center.x + dx, center.y + dy);

  const x = Math.round((dx / radius) * 100);
  const y = Math.round((-dy / radius) * 100);

  lastX = x;
  lastY = y;
  joyReadout.textContent = `x:${x} y:${y}`;
}

function onUp(e){
  if(!active) return;
  e.preventDefault();
  active = false;

  setStick(center.x, center.y);

  lastX = 0; lastY = 0;
  joyReadout.textContent = 'x:0 y:0';
  fetchNoCache('/control?var=joy&val=0,0');
  setTimeout(()=>fetchNoCache('/control?var=joy&val=0,0'), 80);

  stopSending();
}

window.addEventListener('resize', updateGeometry);
updateGeometry();

joyBase.addEventListener('pointerdown', onDown);
joyBase.addEventListener('pointermove', onMove);
joyBase.addEventListener('pointerup', onUp);
joyBase.addEventListener('pointercancel', onUp);
joyBase.addEventListener('pointerleave', (e)=>{ if(active) onUp(e); });
</script>
</body>
</html>
)rawliteral";

static esp_err_t index_handler(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

// ====================== Control endpoint (port 80) ======================
static esp_err_t cmd_handler(httpd_req_t *req)
{
  char *buf;
  size_t buf_len;
  char variable[32] = {0};
  char value[64] = {0};

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len <= 1) { httpd_resp_send_500(req); return ESP_FAIL; }

  buf = (char *)malloc(buf_len);
  if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }

  if (httpd_req_get_url_query_str(req, buf, buf_len) != ESP_OK) {
    free(buf); httpd_resp_send_500(req); return ESP_FAIL;
  }

  if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) != ESP_OK ||
      httpd_query_key_value(buf, "val", value, sizeof(value)) != ESP_OK) {
    free(buf); httpd_resp_send_500(req); return ESP_FAIL;
  }
  free(buf);

  int val = atoi(value);

  if (!strcmp(variable, "speed")) {
    motor_speed = (unsigned int)constrain(val, 0, 255);
  }
  else if (!strcmp(variable, "quality")) {
    // Lower = better quality. Typical useful range: 6..20
    int q = constrain(val, 6, 20);
    sensor_t *s = esp_camera_sensor_get();
    if (s) s->set_quality(s, q);
  }
  else if (!strcmp(variable, "jump")) {
    // Safety pulse (use val if provided, otherwise default)
    uint32_t dur = (val > 0) ? (uint32_t)val : JUMP_PULSE_MS;
    start_jump_pulse(dur);
  }
  else if (!strcmp(variable, "flashoff")) {
    digitalWrite(RELAY_PIN, LOW);
    jump_active = false;
  }
  else if (!strcmp(variable, "joy")) {
    int x = 0, y = 0;
    char tmp[64];
    strncpy(tmp, value, sizeof(tmp) - 1);
    tmp[sizeof(tmp) - 1] = '\0';

    char *comma = strchr(tmp, ',');
    if (comma) {
      *comma = '\0';
      x = atoi(tmp);
      y = atoi(comma + 1);
    } else {
      x = 0; y = 0;
    }
    apply_joystick(x, y);
  }
  else if (!strcmp(variable, "car")) {
    if (val == 3) {
      robot_stop();
      robo = 0;
    }
  }
  else if (!strcmp(variable, "nostop")) {
    noStop = val;
    (void)noStop;
  }
  else {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

// ====================== Server bootstrap ======================
void startCameraServer()
{
  // ---- Control/UI server on port 80 ----
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.ctrl_port   = 32768;
  config.stack_size  = 8192;

  httpd_handle_t server = NULL;

  httpd_uri_t index_uri = { .uri="/",        .method=HTTP_GET, .handler=index_handler, .user_ctx=NULL };
  httpd_uri_t cmd_uri   = { .uri="/control", .method=HTTP_GET, .handler=cmd_handler,   .user_ctx=NULL };

  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &cmd_uri);
  }

  // ---- Stream server on port 81 ----
  httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
  stream_config.server_port = 81;
  stream_config.ctrl_port   = 32769;
  stream_config.stack_size  = 8192;

  httpd_handle_t stream_httpd = NULL;

  httpd_uri_t stream_uri = { .uri="/stream", .method=HTTP_GET, .handler=stream_handler, .user_ctx=NULL };

  if (httpd_start(&stream_httpd, &stream_config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}
