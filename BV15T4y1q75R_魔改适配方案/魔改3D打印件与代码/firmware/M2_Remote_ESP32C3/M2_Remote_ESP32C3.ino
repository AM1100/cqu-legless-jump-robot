/*
 * ============================================================================
 *  M2_Remote_ESP32C3.ino  —  「不用腿的跳跃机器人」ESP-NOW 物理手柄端固件
 * ----------------------------------------------------------------------------
 *  配套：M2_JumpRobot_ESP32C3（机载端）
 *  硬件：ESP32-C3 SuperMini + 9 个轻触按键 + 板载 LED（可选振动马达/蜂鸣器）
 *
 *  为什么要有物理手柄（网页不是够了吗）：
 *    附录 3 §1 第 5 条"可遥控性"要求"完全可遥控、不需人工辅助"。手机网页在
 *    强光下看不清、戴手套点不准、还可能被电话打断；打比赛时手里有个实体键
 *    更稳。两路互为主备，任何一路在线机器人都能跑。
 *
 *  Deadman 心跳：
 *    本机每 200ms 发一帧 PING。机载端以 300ms 为超时，一旦收不到（手柄没电/
 *    超出范围/掉地上）就立即切断电机 PWM 并进安全状态。**这是保护，不是装饰**：
 *    没有它，机器人会在失联后继续把弹簧压到底然后自己跳出去。
 *
 *  帧格式（8 字节，与机载端、03 文档 §6.2 完全一致）：
 *     [0] cmd  [1] arg  [2] seqLo  [3] seqHi  [4] crc8  [5] 0xA5  [6][7] 保留
 *     CRC8: poly 0x07, 初值 0xFF, 覆盖 [0..3] 与 [5]
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ★ V2.0：TlmFrame 多了一个 wheel 字节 ⇒ 必须与机载端同版本一起烧
// ★ V2.1：机载端去掉偏航舵机 ⇒ 本端把 K7 标注改"掉头步(不起跳)"、K9 改"停轮锁死"，
//          0x04/0x09 两个字节停用但不复用 ⇒ 版本号一起升到 1.2.0
#define FW_VERSION "1.2.0"

// ============================================================================
// 1. 按键（全部 INPUT_PULLUP，按下 = LOW）
//    刻意避开 GPIO2 / GPIO8 / GPIO9 三个 strapping 脚：
//    上电瞬间它们被按下会改变启动模式或进下载模式。
// ============================================================================
#define PIN_K_ARM      0    // K1 预蓄能
#define PIN_K_JUMP     1    // K2 起跳（长按 250ms 防误触）
#define PIN_K_CONFIRM  3    // K3 ★确认落稳 / 推进下一步
#define PIN_K_STOP     4    // K4 急停
#define PIN_K_VENT     5    // K5 反转泄能
#define PIN_K_G1       6    // K6 小跳1 越单板
#define PIN_K_G2       7    // K7 掉头步（进段2，★V2.1 该段不起跳，只在地面差速掉头）
#define PIN_K_G3       10   // K8 大跳3 越双板
#define PIN_K_MORE     20   // K9 停轮锁死（★V2.1 复用：原"加掉头脉冲"已随偏航舵机停用）
#define PIN_LED        8    // 板载 LED（低有效）

// ---------------------------------------------------------------------------
// ★ V2.0 区域2 调向按键
// ---------------------------------------------------------------------------
#define PIN_K_PIVOT    21   // K10 原地掉头：短按=右转180° / 长按=左转180°
#define PIN_K_NUDGE     2   // K11 前后微调：短按=前进30mm / 长按=后退30mm
//  ★ GPIO2 是 ESP32-C3 strapping 脚：**上电/复位瞬间不要按 K11**，否则进下载模式。
//    务必外接 10kΩ 上拉到 3V3（板载弱上拉不够稳）。
//    不想承担这个启动风险就置 0 —— 微调改用手机网页，功能完全一样。
#define USE_K11_GPIO2  1

#define PIN_VIB        21   // 可选：振动马达（低电平导通），不用就悬空
#define USE_VIB        0

#if (USE_VIB != 0)
  #error "USE_VIB 与 K10(PIN_K_PIVOT=GPIO21) 抢同一根脚：请把 USE_VIB 置 0"
#endif

// ============================================================================
// 2. 协议
// ============================================================================
#define EN_MAGIC 0xA5
#define WIFI_CHANNEL 1              // ★ 必须与机载端 softAP 信道一致
#define HB_INTERVAL_MS 200          // Deadman 心跳周期
#define DEBOUNCE_MS 60
#define JUMP_HOLD_MS 250            // 起跳要长按，防误触
#define CMD_LOCK_MS 500             // 同一命令最小间隔

enum Cmd : uint8_t {
  CMD_NONE      = 0x00,
  CMD_ARM       = 0x01,
  CMD_JUMP      = 0x02,
  CMD_PITCH     = 0x03,
  CMD_YAW       = 0x04,   // ⚠ V2.1 停用（机载端无偏航舵机）；字节保留不复用
  CMD_CONFIRM   = 0x05,
  CMD_DISARM    = 0x06,   // 反转泄能
  CMD_GOTO      = 0x08,
  CMD_TURN_MORE = 0x09,   // ⚠ V2.1 停用（脉冲掉头取消）；字节保留不复用
  CMD_MACRO     = 0x10,
  CMD_STOP      = 0x20,
  // ★ V2.0 区域2 双螺旋轮差速调向（机载端只放行 S1_IDLE / S10_WAIT_CONFIRM）
  CMD_WHEEL_PIVOT_L = 0x30,
  CMD_WHEEL_PIVOT_R = 0x31,
  CMD_WHEEL_FWD     = 0x32,
  CMD_WHEEL_BACK    = 0x33,
  CMD_WHEEL_STOP    = 0x34,
  CMD_PING      = 0x7F,
};

struct __attribute__((packed)) EnFrame {
  uint8_t  cmd;
  uint8_t  arg;
  uint16_t seq;
  uint8_t  crc;
  uint8_t  magic;
  uint8_t  r0, r1;
};

struct __attribute__((packed)) TlmFrame {
  uint8_t  magic;
  uint8_t  state;
  uint8_t  seg;
  uint8_t  pulse;
  uint8_t  fault;
  uint16_t vbat100;
  int16_t  mA;
  uint16_t seqAck;
  uint8_t  wheel;    // ★ V2.0：0=轮锁死 1=就绪 2=调向中（两端固件必须同时升级）
  uint8_t  crc;
};

static uint8_t crc8(const uint8_t* d, size_t n) {
  uint8_t c = 0xFF;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (uint8_t b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// ============================================================================
// 3. 按键表
// ============================================================================
struct Btn {
  uint8_t  pin;
  uint8_t  cmd;
  uint8_t  arg;
  const char* name;
  bool     holdRequired;
  bool     tapOrLong;      // ★V2.0 true = 短按发 cmd / 长按发 cmd2
  uint8_t  cmd2;
  uint8_t  arg2;
  bool     lastLevel;      // true = 已按下
  bool     longSent;       // 本家长按已经发过了（短按就不再发）
  uint32_t tDown;
  uint32_t tLastSend;
};

// 顺序与接线表一致，改线只改这里
static Btn BTNS[] = {
  { PIN_K_ARM,     CMD_ARM,       0, "预蓄能",         false, false, CMD_NONE,          0,  false, false, 0, 0 },
  { PIN_K_JUMP,    CMD_JUMP,      0, "起跳",           true,  false, CMD_NONE,          0,  false, false, 0, 0 },
  { PIN_K_CONFIRM, CMD_CONFIRM,   0, "★确认落稳",      false, false, CMD_NONE,          0,  false, false, 0, 0 },
  { PIN_K_STOP,    CMD_STOP,      0, "急停",           false, false, CMD_NONE,          0,  false, false, 0, 0 },
  { PIN_K_VENT,    CMD_DISARM,    0, "反转泄能",       false, false, CMD_NONE,          0,  false, false, 0, 0 },
  { PIN_K_G1,      CMD_GOTO,      0, "小跳1 越单板",   false, false, CMD_NONE,          0,  false, false, 0, 0 },
  { PIN_K_G2,      CMD_GOTO,      1, "掉头步 不起跳",   false, false, CMD_NONE,          0,  false, false, 0, 0 },
  { PIN_K_G3,      CMD_GOTO,      2, "大跳3 越双板",   false, false, CMD_NONE,          0,  false, false, 0, 0 },
  { PIN_K_MORE,    CMD_WHEEL_STOP,0, "停轮锁死",       false, false, CMD_NONE,          0,  false, false, 0, 0 },
  // ★ V2.0：短按/长按两档，一个键干两件事（省引脚）
  { PIN_K_PIVOT,   CMD_WHEEL_PIVOT_R, 0, "原地右转180°", false, true,  CMD_WHEEL_PIVOT_L, 0, false, false, 0, 0 },
#if USE_K11_GPIO2
  { PIN_K_NUDGE,   CMD_WHEEL_FWD,     0, "前进微调30mm", false, true,  CMD_WHEEL_BACK,    0, false, false, 0, 0 },
#endif
};
static const uint8_t NBTN = sizeof(BTNS) / sizeof(BTNS[0]);

// ============================================================================
// 4. 状态
// ============================================================================
static uint16_t g_seq = 0;
static uint32_t g_tHb = 0;
static bool     g_nowReady = false;
static uint32_t g_tLastRx = 0;
static bool     g_robotSeen = false;
static TlmFrame g_tlm;
// （V2.0 清掉两个历史遗留的 LED 状态变量：loop() 里已经直接用表达式算闪烁）

static void vib(uint16_t ms) {
#if USE_VIB
  digitalWrite(PIN_VIB, LOW);
  delay(ms);
  digitalWrite(PIN_VIB, HIGH);
#else
  (void)ms;
#endif
}

static void blink(uint8_t n, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < n; i++) {
    digitalWrite(PIN_LED, LOW);
    delay(onMs);
    digitalWrite(PIN_LED, HIGH);
    delay(offMs);
  }
}

// ============================================================================
// 5. 发送
// ============================================================================
static bool ensureBcastPeer() {
  uint8_t b[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  if (esp_now_is_peer_exist(b)) return true;
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, b, 6);
  p.channel = 0;
  p.encrypt = false;
  return esp_now_add_peer(&p) == ESP_OK;
}

static void sendCmd(uint8_t cmd, uint8_t arg) {
  if (!g_nowReady) {
    Serial.printf("[TX] ESP-NOW 未就绪，丢弃 %02X\n", cmd);
    return;
  }
  EnFrame f{};
  f.cmd   = cmd;
  f.arg   = arg;
  f.seq   = ++g_seq;
  f.magic = EN_MAGIC;
  uint8_t buf[5] = { f.cmd, f.arg, (uint8_t)(f.seq & 0xFF), (uint8_t)(f.seq >> 8), f.magic };
  f.crc = crc8(buf, 5);

  uint8_t raw[sizeof(EnFrame)];
  memcpy(raw, &f, sizeof(f));
  uint8_t b[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  esp_err_t e = esp_now_send(b, raw, sizeof(raw));

  Serial.printf("[TX] %02X arg=%u seq=%u %s\n", cmd, arg, f.seq,
                e == ESP_OK ? "ok" : "FAIL");
  blink(1, 12, 0);
}

// ============================================================================
// 6. 接收遥测
// ============================================================================
static const char* stName(uint8_t s) {
  switch (s) {
    case 0:  return "S0_BOOT";
    case 1:  return "S1_IDLE";
    case 2:  return "S2_WIND";
    case 3:  return "S3_ARMED";
    case 4:  return "S4_RELEASE";
    case 5:  return "S5_SETTLE";
    case 6:  return "S6_REENGAGE";
    case 8:  return "S8a_PITCH";
    case 9:  return "S9_FAULT";
    case 10: return "S10_CONFIRM";
    case 18: return "S8b_YAW(停用)";
    default: return "?";
  }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
#else
static void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  if (len < (int)sizeof(TlmFrame)) return;
  TlmFrame t;
  memcpy(&t, data, sizeof(t));
  if (t.magic != 'J') return;
  if (crc8((const uint8_t*)&t, sizeof(t) - 1) != t.crc) return;

  uint8_t prevWheel = g_tlm.wheel;
  g_tlm = t;
  g_robotSeen = true;
  g_tLastRx = millis();

  Serial.printf("[RX] %s seg=%u pulse=%u %.2fV 轮=%s %s\n",
                stName(t.state), t.seg, t.pulse,
                t.vbat100 / 100.0f,
                t.wheel == 2 ? "调向中" : (t.wheel == 1 ? "就绪" : "锁死"),
                t.fault ? "★FAULT" : "");
  // 轮从"就绪"变"锁死"通常就是开始蓄能了 —— 提醒操作者此刻调向键无效
  if (t.wheel == 0 && prevWheel != 0) vib(40);
  if (t.fault) vib(120);
}

// ============================================================================
// 7. 按键扫描
// ============================================================================
static void scanButtons() {
  uint32_t now = millis();
  for (uint8_t i = 0; i < NBTN; i++) {
    Btn& b = BTNS[i];
    bool pressed = (digitalRead(b.pin) == LOW);

    if (pressed && !b.lastLevel) {                 // 按下沿
      b.tDown = now;
      b.lastLevel = true;
      b.longSent = false;
      // 需要长按的（起跳）或双档键（短按/长按）都不在按下沿立即发
      if (b.holdRequired || b.tapOrLong) continue;
      if (now - b.tLastSend < CMD_LOCK_MS) continue;
      b.tLastSend = now;
      sendCmd(b.cmd, b.arg);
    } else if (pressed && b.lastLevel) {           // 持续按住
      if (b.tapOrLong) {
        // ★ 双档键：按住超过 JUMP_HOLD_MS 就发"长按档"，且只发一次
        if (!b.longSent && now - b.tDown >= JUMP_HOLD_MS) {
          b.longSent = true;
          b.tLastSend = now;
          sendCmd(b.cmd2, b.arg2);
        }
        continue;
      }
      if (b.holdRequired) {
        if (now - b.tDown >= JUMP_HOLD_MS) {
          if (now - b.tLastSend >= CMD_LOCK_MS) {
            b.tLastSend = now;
            sendCmd(b.cmd, b.arg);
          }
        }
        continue;
      }
      // 长按连发（急停除外，急停按住也每 400ms 重发一次更安全）
      if (now - b.tDown > 700 && now - b.tLastSend >= 400) {
        b.tLastSend = now;
        sendCmd(b.cmd, b.arg);
      }
    } else if (!pressed && b.lastLevel) {          // 抬起
      b.lastLevel = false;
      // ★ 双档键的"短按档"在抬起时发；已经发过长按档就不再发
      if (b.tapOrLong && !b.longSent) {
        b.tLastSend = now;
        sendCmd(b.cmd, b.arg);
      }
    }
  }
}

// ============================================================================
// 8. setup / loop
// ============================================================================
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
#if USE_VIB
  pinMode(PIN_VIB, OUTPUT);
  digitalWrite(PIN_VIB, HIGH);
#endif

  for (uint8_t i = 0; i < NBTN; i++) pinMode(BTNS[i].pin, INPUT_PULLUP);

  Serial.begin(115200);
  delay(150);
  Serial.println();
  Serial.println("=====================================================");
  Serial.printf(" 跳跃机器人 ESP-NOW 手柄端 v%s\n", FW_VERSION);
  Serial.println(" 按键：K1预蓄能 K2起跳(长按) K3确认 K4急停");
  Serial.println("       K5泄能 K6小跳1 K7掉头步(不起跳) K8大跳3 K9停轮锁死");
  Serial.println("       ★K10 短按=原地右转180° / 长按=原地左转180°");
#if USE_K11_GPIO2
  Serial.println("       ★K11 短按=前进微调 / 长按=后退微调（上电时别按）");
#else
  Serial.println("       K11 未启用：前后微调请用手机网页");
#endif
  Serial.println(" 调向只在 S1_IDLE / S10_WAIT_CONFIRM 生效（弹簧未蓄能）");
  Serial.println(" ★V2.1 无偏航舵机：第2段=地面双轮差速掉头，不蓄能不起跳");
  Serial.println(" 蓄能/待发/起跳期间轮电机被机载端强制制动锁死");
  Serial.println("=====================================================");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("[WiFi] 手柄 MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  esp_err_t chErr = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.printf("[WiFi] 固定信道 %d %s\n", WIFI_CHANNEL, chErr == ESP_OK ? "ok" : "FAIL");

  if (esp_now_init() != ESP_OK) {
    Serial.println("[NOW] 初始化失败！检查是否重复 init");
  } else {
    esp_now_register_recv_cb(onRecv);
    g_nowReady = ensureBcastPeer();
    Serial.printf("[NOW] %s（广播对端，机载端无需配对）\n", g_nowReady ? "就绪" : "对端添加失败");
  }

  blink(3, 60, 60);
  Serial.println("[BOOT] 开始发 Deadman 心跳（200ms）");
}

void loop() {
  uint32_t now = millis();

  // Deadman 心跳：这条不断，机载端才认为链路活着
  if (now - g_tHb >= HB_INTERVAL_MS) {
    g_tHb = now;
    sendCmd(CMD_PING, 0);
  }

  scanButtons();

  // 链路指示：收到遥测 → 常亮 80ms；3s 无遥测 → 慢闪
  if (g_robotSeen && now - g_tLastRx < 600) {
    digitalWrite(PIN_LED, LOW);
  } else {
    digitalWrite(PIN_LED, ((now / 600) % 2) ? LOW : HIGH);
  }

  // 每 3 秒打印一次机器人状态（无遥测时说明链路断了）
  static uint32_t tLog = 0;
  if (now - tLog > 3000) {
    tLog = now;
    if (g_robotSeen && now - g_tLastRx < 1500) {
      Serial.printf("[TLM] %s seg=%u pulse=%u %.2fV mA=%d fault=%u\n",
                    stName(g_tlm.state), g_tlm.seg, g_tlm.pulse,
                    g_tlm.vbat100 / 100.0f, g_tlm.mA, g_tlm.fault);
    } else {
      Serial.println("[TLM] 未收到机载端遥测（检查：同一信道?机载端上电?距离?）");
    }
  }

  delay(5);
}
