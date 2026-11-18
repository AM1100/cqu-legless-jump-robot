/*
 * ============================================================================
 *  M2_JumpRobot_ESP32C3.ino   —  「不用腿的跳跃机器人」机载端主固件
 * ----------------------------------------------------------------------------
 *  课程：2026 秋《自然与设计》V3.0   小组：02-5
 *  硬件：ESP32-C3 SuperMini + TB6612FNG + 1×N20(5V,缺齿齿轮) + 2×5V 小舵机(选位)
 *        + 霍尔相位检测 + 母线电压 ADC
 *  电源：课程 HCC1325 模组 5V 直供（唯一电池，禁拆壳/禁改板/禁并联其它储能件）
 *
 *  ★★ 重要声明（与 03 电控文档口径一致）★★
 *  1. 本文件里所有"角度 / 占空比 / 延时"都是【工程占位值】，**不是标定值**。
 *     机械线的 φ_hold / φ_stop / 缺齿安全窗 / t_return 全部仍是 TBD，
 *     因此 CFG.holdStopMs、CFG.tReturnMs 等必须上台架标定后才能用于正式测试。
 *     未标定前严禁直接拿去跑考核（固件会在串口反复提示 UNVERIFIED）。
 *  2. 电流值一律是估算。未装 INA219 时 g_mA = -1（"未知"），界面显示"未装传感器"，
 *     不会假装测到了。
 *  3. 本固件不含摄像头、不含行走轮、不含 2S 供电 —— 那三项在附录 1 里分别是
 *     违规/取消资格/违规。
 *
 *  状态机（与 03 文档 §5 完全一致）：
 *      S0_BOOT → S1_IDLE → S8a_PITCH → S2_WIND → S3_ARMED → S4_RELEASE
 *              → S5_SETTLE → S6_REENGAGE → S10_WAIT_CONFIRM → 回到 S8a / S1
 *      任意状态异常 → S9_FAULT
 *  S10 是【结构级人工闸门】：无超时、唯一出口是 CONFIRM，且每次只放行一步。
 *  "开环连跳三圈"在本固件里**没有代码路径**。
 *
 *  双遥控：
 *    (A) Wi-Fi 热点 + 内置中文网页（手机浏览器，无需装 App、无需联网）
 *    (B) ESP-NOW 物理手柄（M2_Remote_ESP32C3 固件）
 *  两路互为主备，任一路在线即可；两路都掉线 >300ms 且正在出力 → 立即安全停机。
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ★ 所有枚举/结构体放在这个头里，原因见文件头注释
//   （Arduino 自动插入函数原型的位置在第一个 #include 之后，
//     类型若不在这之前可见，会报 "MotorDir was not declared"）
#include "robot_defs.h"

// ============================================================================
// 0. 版本
// ============================================================================
// ★ V2.0：TlmFrame 多了一个 wheel 字节 ⇒ 机载端/手柄端**必须同时烧**，版本号一起升。
// ★ V2.1：偏航舵机不装机（HAVE_YAW_SERVO=0）。帧格式不变，但 0x04/0x09 停用、
//          GOTO 1 改为"不蓄能、停在 S10 等轮掉头" ⇒ 语义变了，版本号仍要一起升。
#define FW_VERSION      "1.2.0"
#define FW_TAG          "M2-C3-5V-COMPLIANT"

// ============================================================================
// 1. 引脚定义（与 03 文档 §2.5 一致；ESP32-C3 SuperMini）
// ----------------------------------------------------------------------------
//  ★ 硬件注意 1：GPIO2 是 C3 的 strapping 脚，**启动时必须为高**。
//    舵机信号线请用 10kΩ **上拉**到 3V3（不要用下拉！下拉会让芯片进不了正常启动）。
//    03 文档 §2.5 早期写的"10kΩ 下拉"是错的，本固件配套接线表已更正。
//  ★ 硬件注意 2：TB6612 的 STBY 直接接 3V3（省一个 IO），不要接 GPIO。
//  ★ 硬件注意 3：GPIO9 是 BOOT 键，不占用；GPIO8 板载 LED，低电平点亮。
//  ★ V2.1 构型变更：机械正式构型**不含偏航舵机**（Yaw 鞋不装机）。
//    下方 PIN_SERVO_YAW 仅作退回路径保留，HAVE_YAW_SERVO=0 时不会被初始化也不会被驱动。
//    连带结果：LEDC **ch1 空出**（见 §2 注释），可作为将来"左右轮各一路 PWM"的后备通道。
// ============================================================================
#define PIN_HALL          0    // 霍尔相位检测输入（鼓轮磁钢，一圈一个沿）
#define PIN_SERVO_PITCH   1    // 俯仰（起跳角）舵机 —— 只做选位
#define PIN_SERVO_YAW     2    // ⚠ V2.1 不装机：偏航舵机，已由 HAVE_YAW_SERVO=0 停用
#define PIN_VBAT_ADC      3    // 母线电压采样（分压 220k/100k = 1:3.2）
#define PIN_AIN1          4    // TB6612 AIN1
#define PIN_AIN2          5    // TB6612 AIN2
#define PIN_PWMA          6    // TB6612 PWMA（20kHz）
#define PIN_LED           8    // 板载 LED（低有效）

// ----------------------------------------------------------------------------
// ★ V2.0 双螺旋轮（左右差速转向轮）—— 3 根线：1 路共享 PWM + 2 路方向
// ----------------------------------------------------------------------------
//  为什么"共享 PWM"而不是左右各一路：
//    ESP32-C3 只有 6 路 LEDC 通道 / 4 个定时器，且通道与定时器是按 N%4 绑定的。
//    V2.0 定稿时占了 3 路（俯仰舵机 50Hz/16bit、偏航舵机 50Hz/16bit、蓄能电机 20kHz/10bit），
//    左右轮再各占一路就变成 5 路，第 5 路会落到已配成 50Hz 的定时器上，
//    **把舵机频率改掉**（舵机乱跳）——这是不易察觉的坑。
//    只用 4 路（每路一个独立定时器）就彻底没有冲突：ch0..ch3 → T0..T3。
//  ★ V2.1 现状（偏航舵机不装机）：ch1 已经空出，理论上够给左右轮各一路 PWM。
//    但**本轮不改架构**（属于功能扩展，不是清单统一），保持共享 PWM：
//    当前"D=50% 对转掉头 + 时间标定 180°"已满足区域2 需求，且 wheelAsymPct 可补左右打滑差。
//    今后若需要单轮调速/软起对转，把 ch1 给右轮即可 —— 只改 §7b 的 wheelLevels()，
//    其余状态机、互锁、协议都不用动。
//  代价：两轮**占空比必须相同**，所以
//    · 同向直行：D = 设定值（前进）/ 1−设定值（后退）
//    · 原地对转：D 必须 = 50%，否则两轮力矩不对称、旋转中心会漂 ⇒ 掉头固定 50%，用时间标定角度
//    · **不支持单轮转向/曲率转弯**（本赛题也不需要，见 03 文档 §4.4b）
//  接线（DRV8833 两路 H 桥；TB6612 的 BIN1/BIN2/PWMB 同构，二者皆可）：
//    GPIO7  → AIN1 + BIN1（两路 PWM 输入并联，共用一个 GPIO）
//    GPIO10 → AIN2（左轮方向，静态电平）
//    GPIO20 → BIN2（右轮方向，静态电平）
//    nSLEEP/SLP 直接接 3V3（省一个 IO，与 TB6612 的 STBY 同理）
//    ★ 方向真值表：IN2=LOW ⇒ 平均 +D·Vm（正转）；IN2=HIGH ⇒ 平均 −(1−D)·Vm（反转，占空比取补）
//  ★ 若 Serial 走硬件 UART0（GPIO20=RXD/21=TXD）而不是 USB-CDC，把 WHEEL_PINMAP 改 1。
#define WHEEL_PINMAP 0
#if WHEEL_PINMAP == 0
  #define PIN_W_PWM   7     // 共享 PWM（20kHz）
  #define PIN_W_DIR_L 10    // 左轮方向（原"备用/IMU INT"，本版占用）
  #define PIN_W_DIR_R 20    // 右轮方向
#else
  #define PIN_W_PWM   7
  #define PIN_W_DIR_L 10
  #define PIN_W_DIR_R 21    // 换到 GPIO21（同样是 UART0 TXD，无按键冲突）
#endif

// GPIO21 在 WHEEL_PINMAP=0 时仍空闲；GPIO9 是 BOOT 键，**不要**拿来做输出
// （按下 BOOT 会把推挽输出直接短路到地）。

// 电池分压比：R1=220k 上臂, R2=100k 下臂 ⇒ (220+100)/100 = 3.2
#define VBAT_DIVIDER      3.2f
#define VBAT_LOW_V        4.40f   // 低于此值判供电异常（故障码 1）
#define VBAT_HIGH_V       5.60f   // 高于此值判供电异常（故障码 2）

// ============================================================================
// 2. LEDC / PWM 兼容层
// ----------------------------------------------------------------------------
//  arduino-esp32 3.x： ledcAttach(pin,freq,res) / ledcWrite(pin,duty)
//  arduino-esp32 2.x： ledcSetup(ch,freq,res) + ledcAttachPin(pin,ch) / ledcWrite(ch,duty)
//  2.x 分支不能用"引脚号当通道号"：V2.0 引入 GPIO20（> 15，2.x 只有 16 个通道），
//  会直接越界。所以 2.x 改用**显式通道映射**，并把通道选在 0..3（每个一个定时器）。
// ============================================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  static inline bool pwmSetup(int pin, uint32_t freq, uint8_t res) { return ledcAttach(pin, freq, res); }
  static inline void pwmWrite(int pin, uint32_t duty)              { ledcWrite(pin, duty); }
  static inline void pwmDetach(int pin)                            { ledcDetach(pin); }
#else
  static inline int8_t pwmCh(int pin) {
    switch (pin) {
      case PIN_SERVO_PITCH: return 0;
      case PIN_SERVO_YAW:   return 1;   // ⚠ V2.1 不装机 ⇒ ch1 实际空置，留作后备
      case PIN_PWMA:        return 2;
      case PIN_W_PWM:       return 3;
      default:              return -1;
    }
  }
  static inline bool pwmSetup(int pin, uint32_t freq, uint8_t res) {
    int8_t ch = pwmCh(pin);
    if (ch < 0) return false;
    if (ledcSetup((uint8_t)ch, freq, res) == 0) return false;
    ledcAttachPin(pin, (uint8_t)ch);
    return true;
  }
  static inline void pwmWrite(int pin, uint32_t duty) {
    int8_t ch = pwmCh(pin);
    if (ch >= 0) ledcWrite((uint8_t)ch, duty);
  }
  static inline void pwmDetach(int pin) { ledcDetachPin(pin); }
#endif

#define MOTOR_PWM_FREQ   20000
#define MOTOR_PWM_RES    10
#define MOTOR_PWM_MAX    1023

#define SERVO_FREQ       50
#define SERVO_RES        16
#define SERVO_MIN_US     500
#define SERVO_MAX_US     2500

// ============================================================================
// 3. 可标定配置（结构体定义见 robot_defs.h；存 NVS，掉电不丢；串口可改）
// ----------------------------------------------------------------------------
//  ★ 带 [TBD] 的项 = 机械线尚未给出 / 从未实测，当前值只为"让代码能跑起来"，
//    正式测试前必须台架标定并在日志里写清标定条件。
// ============================================================================
static Config  CFG;
static Preferences prefs;

// ============================================================================
// 4. 状态机（枚举定义见 robot_defs.h）
// ============================================================================
static State     g_state       = S0_BOOT;
static uint32_t  g_tState      = 0;
static uint32_t  g_tRelease    = 0;
static uint8_t   g_fault       = F_NONE;
static uint32_t  g_phaseEdges  = 0;      // 霍尔沿累计（调试用）
static uint32_t  g_tLastEdge   = 0;
static uint8_t   g_retryJog    = 0;
static bool      g_everArmed   = false;
static int16_t   g_mA          = -1;     // -1 = 未装电流传感器（不假装测到）
static float     g_vbat        = 0.0f;
static uint32_t  g_bootMs      = 0;

// ---- 分段（"三次跳跃"工程路线，见 03 文档 §0.2；类型见 robot_defs.h）----
//  ★ V2.1：第 2 段 = 地面双轮差速原地掉头（SEG_PIVOT），**不蓄能、不起跳**。
//    所以第 2 段的 pitchDeg 无意义，填 0（留着只为结构体完整）。
//    真正的"跨板"动作仍然只有第 1 跳和第 3 跳，两次都是真弹射 ⇒ 正文"必须跳跃"成立。
static Seg SEGS[3] = {
  { "第1跳 越单板(小跳)",     SEG_JUMP,  72 },
  { "第2步 原地掉头(双轮差速)", SEG_PIVOT, 0 },
  { "第3跳 越双板(大跳)",     SEG_JUMP,  56 },
};
static uint8_t g_seg    = 0;   // 当前段
static uint8_t g_pulse  = 0;   // ⚠ V2.1 仅 SEG_TURN（已停用）使用，保留以兼容旧日志
static int8_t  g_turnDir = +1; // 最近一次掉头方向：+1 右 / -1 左（V2.1 由 cmdWheelJob 写入并打印）

// ============================================================================
// 5. 链路（Wi-Fi 网页 与 ESP-NOW 手柄，双通道）
// ============================================================================
static uint32_t g_lastNowMs  = 0;   // 最后一次 ESP-NOW 包
static uint32_t g_lastHttpMs = 0;   // 最后一次网页心跳
static bool     g_nowSeen    = false;
static bool     g_httpSeen   = false;
static uint8_t  g_remoteMac[6] = {0};
static bool     g_remoteKnown  = false;

#define LINK_TIMEOUT_NOW_MS   300    // 任务要求：>300ms 断联安全停机
#define LINK_TIMEOUT_HTTP_MS  1000   // 网页心跳放宽（手机后台节流）

static bool linkAlive() {
  uint32_t now = millis();
  if (g_nowSeen  && (now - g_lastNowMs)  < LINK_TIMEOUT_NOW_MS)  return true;
  if (g_httpSeen && (now - g_lastHttpMs) < LINK_TIMEOUT_HTTP_MS) return true;
  return false;
}

// ============================================================================
// 6. 帧格式与 CRC（帧结构 EnFrame / TlmFrame、指令枚举 Cmd 见 robot_defs.h）
// ----------------------------------------------------------------------------
//   [0] cmd  [1] arg  [2] seqLo  [3] seqHi  [4] crc8  [5] 0xA5  [6][7] 保留
//   CRC8：poly 0x07，初值 0xFF，覆盖 [0..3] 与 [5]
// ============================================================================
static uint8_t crc8(const uint8_t* d, size_t n) {
  uint8_t c = 0xFF;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (uint8_t b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

// ============================================================================
// 7. 电机 / 舵机 底层
// ============================================================================
static uint16_t g_motorDuty = 0;   // 当前占空比（0..MOTOR_PWM_MAX）

static void motorApply(MotorDir d, uint16_t duty) {
  switch (d) {
    case DIR_FWD:   digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW);  break;
    case DIR_REV:   digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); break;
    case DIR_BRAKE: digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, HIGH); break;
    case DIR_COAST:
    default:        digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, LOW);  break;
  }
  g_motorDuty = (d == DIR_COAST || d == DIR_BRAKE) ? 0 : duty;
  pwmWrite(PIN_PWMA, g_motorDuty);
}

/*
 * 30ms 软启动斜坡：把占空比从当前值线性推到目标值。
 * 目的：N20 堵转瞬间的电流尖峰是稳态的 5~10 倍，直接给 88% 占空比会把
 *       HCC1325 的 5V/500mA 母线拉塌，ESP32-C3 直接欠压复位。
 * 斜坡是"限制 di/dt"，不是"限制能量"——它不能突破 1A 硬件限流，
 * 所以占空比上限仍由 CFG.windPct 等控制。
 */
static void motorRamp(MotorDir d, uint16_t targetDuty, uint16_t rampMs) {
  if (rampMs < 6) rampMs = 6;
  uint16_t from = (d == DIR_FWD || d == DIR_REV) ? g_motorDuty : 0;
  uint16_t to   = targetDuty;
  const uint8_t steps = 10;
  uint16_t stepMs = rampMs / steps;
  if (stepMs < 3) stepMs = 3;

  // 先定方向（占空比此时仍为 0 或旧值）
  if (d == DIR_FWD)      { digitalWrite(PIN_AIN1, HIGH); digitalWrite(PIN_AIN2, LOW); }
  else if (d == DIR_REV) { digitalWrite(PIN_AIN1, LOW);  digitalWrite(PIN_AIN2, HIGH); }

  for (uint8_t i = 1; i <= steps; i++) {
    int32_t v = (int32_t)from + ((int32_t)to - (int32_t)from) * i / steps;
    if (v < 0) v = 0;
    g_motorDuty = (uint16_t)v;
    pwmWrite(PIN_PWMA, g_motorDuty);
    delay(stepMs);
  }
  g_motorDuty = to;
  pwmWrite(PIN_PWMA, g_motorDuty);
}

static uint16_t pctToDuty(uint8_t pct) {
  if (pct > 100) pct = 100;
  return (uint16_t)((uint32_t)pct * MOTOR_PWM_MAX / 100);
}

// ---- 舵机 ----
static bool g_servoReady = false;

static inline uint16_t degToUs(uint16_t deg) {
  if (deg > 180) deg = 180;
  return (uint16_t)(SERVO_MIN_US + (uint32_t)deg * (SERVO_MAX_US - SERVO_MIN_US) / 180);
}

static void servosInit() {
  pwmSetup(PIN_SERVO_PITCH, SERVO_FREQ, SERVO_RES);
#if HAVE_YAW_SERVO
  pwmSetup(PIN_SERVO_YAW,   SERVO_FREQ, SERVO_RES);   // ⚠ V2.1 不装机 ⇒ 本行不编译，ch1 空出
#endif
  pwmWrite(PIN_SERVO_PITCH, 0);   // 0 = 无脉冲（不驱动，避免上电抖动）
#if HAVE_YAW_SERVO
  pwmWrite(PIN_SERVO_YAW,   0);
#endif
  g_servoReady = true;
}

/*
 * 选位：给脉冲 → 等 settleMs → 归零。
 * 注意：舵机"不承担主弹性载荷"是架构要求，**但选位瞬间扭矩不是 0**
 * （机体重力矩 + 铰链摩擦 + 连杆机械优势损失）。
 * 所以这里必须留 settleMs（默认 300ms）让它真的到位，不能"发完就切"。
 */
static void servoSelect(int pin, uint16_t deg, uint16_t settleMs = 300) {
  if (!g_servoReady) return;
  uint32_t duty = (uint32_t)degToUs(deg) * 65536UL / 20000UL;  // 50Hz ⇒ 周期 20000us
  pwmWrite(pin, duty);
  delay(settleMs);
  pwmWrite(pin, 0);              // 断电保持（靠连杆自锁/挡块）
}

// ============================================================================
// 7b. 双螺旋轮（区域2 差速调向）★★ 合规硬互锁全部在这里 ★★
// ----------------------------------------------------------------------------
//  【规则依据】正文要求"跨越挡板必须靠跳"，PDF 第 14 页第 9 条把"轮驱跨越"列为
//  取消资格。本机确实有轮（用于区域2 地面调向），所以必须**从软件底层证明**
//  "跳的时候轮子不驱动"。做法是三重闸门：
//
//    ① 状态闸门：轮电机唯一允许出力的状态 = S7_ALIGN（原地调向）。
//       进 S7 的唯一入口 = S1_IDLE / S10_WAIT_CONFIRM —— 这两个状态弹簧**没有蓄能**，
//       机器人也在板上/地上没在跳。蓄能(S2)/待发(S3)/释放(S4)/落稳(S5)/再啮合(S6)
//       全都不在允许集里。
//    ② 迁移闸门：enter() 每次状态迁移，只要目标不是 S7_ALIGN 就无条件 wheelLock()。
//    ③ 巡检闸门：wheelGuard() 在每个 loop 开头复核，发现"非 S7 却有力"立刻锁死，
//       并在串口打 [INTERLOCK] 留痕（合规自查可拿串口日志当证据）。
//
//  再加一道"分时供电"闸门：S7 里若蓄能电机占空比 ≠ 0，轮电机立即锁死。
//  这样 5V 母线**永远只有一个执行器组在吃电流**，峰值不会叠加。
//
//  ★ 锁死是"零驱动"：制动模式 = 两输入同高（绕组短路，电磁制动，抗落地冲击）；
//    高阻模式 = 两输入同低（输出 Hi-Z）。两种都不产生任何转矩。
// ============================================================================
static const char* stateName(State s);          // 前置声明（定义在 §10）

static bool     g_wheelReady = false;
static int8_t   g_wDirL = 0;                    // 上次下发的方向（+1 前 / -1 后 / 0 停）
static int8_t   g_wDirR = 0;
static uint16_t g_wDuty = 0;                    // 上次下发的占空比（共享 PWM，两轮同值）
static WheelJob g_wjob   = WJOB_NONE;
static uint32_t g_tWjob  = 0;
static uint16_t g_wjobMs = 0;
static State    g_wReturn = S1_IDLE;            // 调向结束后回到哪个状态

static const char* wheelJobName(WheelJob j) {
  switch (j) {
    case WJOB_PIVOT_L: return "原地左转180°";
    case WJOB_PIVOT_R: return "原地右转180°";
    case WJOB_FWD:     return "前进微调";
    case WJOB_BACK:    return "后退微调";
    default:           return "无";
  }
}

/*
 * 写一轮的控制电平。dir 是**机器坐标系**的方向（+1 前 / -1 后），
 * invert 处理镜向安装（左右电机镜像装，同一个"前"在电气上是反的）。
 * 共享 PWM ⇒ 两轮 D 必须同值，这里只负责"方向脚 + 占空比"的落笔。
 */
static void wheelLevels(int8_t dirL, int8_t dirR, uint16_t duty) {
  int8_t l = CFG.wheelInvL ? (int8_t)-dirL : dirL;
  int8_t r = CFG.wheelInvR ? (int8_t)-dirR : dirR;
  // IN2 = LOW  ⇒ 平均 +D·Vm（正转）
  // IN2 = HIGH ⇒ 平均 −(1−D)·Vm（反转；D 已由调用方取补，见 wheelSet）
  digitalWrite(PIN_W_DIR_L, (l < 0) ? HIGH : LOW);
  digitalWrite(PIN_W_DIR_R, (r < 0) ? HIGH : LOW);
  pwmWrite(PIN_W_PWM, duty);
  g_wDirL = dirL; g_wDirR = dirR; g_wDuty = duty;
}

/*
 * 下发一个"目标动作"。因为两轮共用一路 PWM，占空比只有一个自由度，所以：
 *   · 同向前进：D = pct
 *   · 同向后退：D = 100 − pct   （反转取补）
 *   · 原地对转：D 必须 = 50%，否则两轮有效力矩不等，旋转中心漂（用时间标角度）
 * 单轮动作不可实现 —— 本赛题也不需要（03 文档 §4.4b 已写明）。
 */
static void wheelSet(int8_t dirL, int8_t dirR, uint8_t pct) {
  uint16_t duty;
  if (dirL == 0 || dirR == 0) {                  // 单轮 → 不支持，直接按锁死处理
    pwmWrite(PIN_W_PWM, 0);
    digitalWrite(PIN_W_DIR_L, LOW);
    digitalWrite(PIN_W_DIR_R, LOW);
    g_wDirL = 0; g_wDirR = 0; g_wDuty = 0;
    return;
  }
  if ((dirL > 0) != (dirR > 0)) {               // 对转 ⇒ 强制对称
    if (pct != 50) {
      static bool warned = false;
      if (!warned) { warned = true;
        Serial.println("[WHEEL] 对转占空比被强制为 50%（共享 PWM 下不对称会漂中心）");
      }
    }
    duty = pctToDuty(50);
  } else if (dirL > 0) {                        // 同向前进
    duty = pctToDuty(pct);
  } else {                                      // 同向后退：占空比取补
    duty = pctToDuty((uint8_t)(100 - pct));
  }
  wheelLevels(dirL, dirR, duty);
}

// ★ 零驱动锁死（默认电磁制动；可配成高阻）
static void wheelLock() {
  g_wDirL = 0; g_wDirR = 0; g_wDuty = 0; g_wjob = WJOB_NONE;
  if (!g_wheelReady) return;
  if (CFG.wheelLockMode) {                      // 制动：两输入同高 ⇒ 下管双开，绕组短路
    digitalWrite(PIN_W_DIR_L, HIGH);
    digitalWrite(PIN_W_DIR_R, HIGH);
    pwmWrite(PIN_W_PWM, MOTOR_PWM_MAX);
  } else {                                      // 高阻：两输入同低 ⇒ 输出 Hi-Z
    digitalWrite(PIN_W_DIR_L, LOW);
    digitalWrite(PIN_W_DIR_R, LOW);
    pwmWrite(PIN_W_PWM, 0);
  }
}

// 闸门查询：给任何调用方一个可打印的理由
static bool wheelAllowed() {
  if (!g_wheelReady) return false;
  if (g_state != S7_ALIGN) return false;        // ① 只有调向态
  if (g_motorDuty != 0) return false;           // ② 分时供电：蓄能电机必须停
  return true;
}

// 不合规时**不只是返回 false**，还要顺手锁死 + 打日志（互锁要留痕，方便合规自查）
static bool wheelReject(int8_t dirL, int8_t dirR) {
  Serial.printf("[INTERLOCK] REJ 轮驱动 state=%s winding=%u/1023 dirs=%+d/%+d\n",
                stateName(g_state), (unsigned)g_motorDuty, (int)dirL, (int)dirR);
  wheelLock();
  return false;
}

/*
 * ★★ 唯一对外驱动入口：任何轮出力都必须走这里，绕过它会被 wheelGuard() 抓。
 * 软启动：从小占空比爬到目标，避免 N20 启动电流尖峰（分时供电下更容易超流）。
 * ★ 只对**同向**（直行）有效。原地对转做不到软启动：共享 PWM 下"两轮等幅反向"
 *   只在 D=50% 成立，任何中间占空比都会变成一轮猛转、一轮慢转 ⇒ 对转只能一刀切到 50%。
 *   对转的启动电流靠"只在弹簧未蓄能状态才允许对转"来保证不叠加，不靠斜坡。
 */
static bool wheelRamp(int8_t dirL, int8_t dirR, uint8_t pct) {
  if (!wheelAllowed()) return wheelReject(dirL, dirR);
  if (dirL != dirR) {                           // 对转：不能斜坡，直接给
    wheelSet(dirL, dirR, pct);
    return true;
  }
  uint16_t rampMs = CFG.wheelRampMs < 10 ? 10 : CFG.wheelRampMs;
  const uint8_t steps = 10;
  uint16_t stepMs = (uint16_t)(rampMs / steps);
  if (stepMs < 2) stepMs = 2;
  for (uint8_t i = 1; i <= steps; i++) {
    if (!wheelAllowed()) return wheelReject(dirL, dirR);
    wheelSet(dirL, dirR, (uint8_t)((uint32_t)pct * i / steps));
    delay(stepMs);
  }
  wheelSet(dirL, dirR, pct);
  return true;
}

/*
 * ★ 巡检闸门（第 ③ 重）：每个 loop 开头跑一次，成本 3 次比较。
 * 它的作用是兜住"以后有人加了新状态/新指令、忘了走 enter()"的情况。
 */
static void wheelGuard() {
  if (!g_wheelReady) return;
  bool force = (g_wDirL != 0 || g_wDirR != 0 || g_wDuty != 0);
  if (g_state == S7_ALIGN) {
    // 在调向态里也要守"分时供电"：蓄能电机一有占空比，轮立刻让路
    if (force && g_motorDuty != 0) {
      Serial.printf("[INTERLOCK] 分时供电冲突：蓄能电机=%u/1023 → 轮电机立即锁死\n",
                    (unsigned)g_motorDuty);
      wheelLock();
    }
    return;
  }
  if (force) {
    Serial.printf("[INTERLOCK] 状态 %s 下检测到轮电机出力（D=%u dirs=%+d/%+d）"
                  "→ 强制锁死（跨隔板禁止轮驱动）\n",
                  stateName(g_state), (unsigned)g_wDuty, (int)g_wDirL, (int)g_wDirR);
    wheelLock();
  }
}

// 0 = 锁死（禁止） 1 = 就绪（可发起调向） 2 = 调向中
static uint8_t wheelStateCode() {
  if (g_state == S7_ALIGN) return 2;
  if (g_state == S1_IDLE || g_state == S10_WAIT_CONFIRM) return 1;
  return 0;
}

static void wheelInit() {
  pinMode(PIN_W_DIR_L, OUTPUT);
  pinMode(PIN_W_DIR_R, OUTPUT);
  digitalWrite(PIN_W_DIR_L, LOW);
  digitalWrite(PIN_W_DIR_R, LOW);
  bool ok = pwmSetup(PIN_W_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  pwmWrite(PIN_W_PWM, 0);
  g_wheelReady = true;      // 即便 PWM 申请失败也置位，让 wheelLock() 能兜住 IO
  wheelLock();
  Serial.printf("[WHEEL] 转向轮 PWM=GPIO%d(共享) DIR_L=GPIO%d DIR_R=GPIO%d  锁死方式=%s  %s\n",
                PIN_W_PWM, PIN_W_DIR_L, PIN_W_DIR_R,
                CFG.wheelLockMode ? "制动" : "高阻",
                ok ? "PWM:OK" : "PWM:申请失败!");
  if (!ok) {
    Serial.println("[WHEEL] ⚠ LEDC 通道被拒（ESP32-C3 只有 6 路）；"
                   "轮功能不可用，但互锁仍生效（轮始终保持锁死）。");
  }
}

// ============================================================================
// 8. 传感
// ============================================================================
static volatile uint32_t g_edgeISR = 0;

static void IRAM_ATTR onHallEdge() { g_edgeISR = g_edgeISR + 1; }  // volatile 别用 ++

static void sensorsInit() {
  pinMode(PIN_HALL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), onHallEdge, FALLING);

  pinMode(PIN_VBAT_ADC, INPUT);
  analogSetPinAttenuation(PIN_VBAT_ADC, ADC_11db);
}

static uint32_t g_edgeSeen     = 0;
static uint32_t g_edgeInterval = 0;   // 相邻两个有效沿的时间间隔（主轮一周，若磁钢装主轮）

// ms → 主轮角度（度）。分母必须是"主轮一周时间"，由 g_edgeInterval 实测（03 §3.3c-3②）
static float msToMainDeg(uint16_t ms) {
  if (g_edgeInterval == 0) return -1.0f;
  return (float)ms * 360.0f / (float)g_edgeInterval;
}

/*
 * 轮询霍尔沿（不在 ISR 里做复杂逻辑）
 * V1.4：加入抖动过滤 —— 齿条啮合的机械振动会在同一个沿附近多次翻转，
 * 若不加 minEdgeMs，一次啮合会被记成好几次，相位全乱。
 */
static void pollHall() {
  uint32_t e = g_edgeISR;
  if (e == g_edgeSeen) return;

  uint32_t now = millis();
  uint32_t newEdges = e - g_edgeSeen;
  g_edgeSeen = e;

  // 抖动过滤：距上一个**有效**沿太近的，直接丢（不计入相位）
  if (g_tLastEdge != 0 && (now - g_tLastEdge) < CFG.minEdgeMs) {
    return;
  }
  if (newEdges > 1) {
    Serial.printf("[PHASE] 警告：一次轮询收到 %lu 个沿（ISR 积压/抖动）\n",
                  (unsigned long)newEdges);
  }

  if (g_tLastEdge != 0) g_edgeInterval = now - g_tLastEdge;
  g_phaseEdges = g_phaseEdges + 1;   // volatile：别用 ++（C++20 起已弃用）
  // 保留 ISR 计数的低位一致性：把被丢弃的沿也"消费"掉，避免下次重复告警
  g_tLastEdge = now;

  // ★ 每次都打印换算，标定时照着记（03 §3.3c-3②）
  if (g_edgeInterval > 0) {
    float rpm  = 60000.0f / (float)g_edgeInterval;             // 主轮 rpm
    float degPerMs = 360.0f / (float)g_edgeInterval;
    Serial.printf("[PHASE] 沿间隔=%lums ⇒ 主轮 %.2f rpm；hold=%ums ≡ %.2f°；"
                  "1°=%.1fms；名义加载行程≈%.2fmm(S060)/%.2fmm(原尺寸)\n",
                  (unsigned long)g_edgeInterval, rpm,
                  CFG.holdStopMs, msToMainDeg(CFG.holdStopMs),
                  (degPerMs > 0.0001f) ? (1.0f / degPerMs) : 0.0f,
                  LOAD_STROKE_MM, LOAD_STROKE_MM_FULL);
  } else {
    Serial.printf("[PHASE] 首个沿（间隔未知）。齿比 %.3f:1，主轮一周=电机 %.3f 圈\n",
                  GEAR_TOTAL_RATIO, GEAR_TOTAL_RATIO);
  }
}

static float readVbat() {
  uint32_t acc = 0;
  for (uint8_t i = 0; i < 8; i++) acc += analogReadMilliVolts(PIN_VBAT_ADC);
  float mv = (float)acc / 8.0f;
  return mv * VBAT_DIVIDER / 1000.0f;
}

// ============================================================================
// 9. 相位判据（★ 全部 TBD，见 03 文档 §3.3b）
// ----------------------------------------------------------------------------
//  两个相位不是一回事，绝不能合成一个"δ_arm"：
//   φ_hold：末齿前、载荷仍挂着的承载待发相位 —— 决定 S2 何时停
//   φ_stop：释放后已卸载、靠惯性/制动停下 —— 决定能否安全再啮合
//  本固件用"霍尔沿 + 可标定延时"近似，延时量必须台架标定。
// ============================================================================
static uint32_t g_tWindStart = 0;

// S2 阶段：霍尔出现沿后，再过 holdStopMs 就停 —— 这个 ms 就是 φ_hold 的全部误差余量
static bool holdPhaseReached() {
  if (g_tLastEdge < g_tWindStart) return false;
  return (millis() - g_tLastEdge) >= CFG.holdStopMs;
}

// S6 阶段：是否允许再啮合（等够齿条复位 t_return，且不在脱啮后立即抢齿）
static bool mayReengage() {
  if (millis() - g_tRelease < CFG.tReturnMs) return false;
  return true;
}

// ============================================================================
// 10. 状态迁移
// ============================================================================
static const char* stateName(State s) {
  switch (s) {
    case S0_BOOT:           return "S0_BOOT";
    case S1_IDLE:           return "S1_IDLE";
    case S8a_PITCH:         return "S8a_PITCH";
    case S8b_YAW:           return "S8b_YAW(停用·无偏航舵机)";
    case S2_WIND:           return "S2_WIND";
    case S3_ARMED:          return "S3_ARMED";
    case S4_RELEASE:        return "S4_RELEASE";
    case S5_SETTLE:         return "S5_SETTLE";
    case S6_REENGAGE:       return "S6_REENGAGE";
    case S7_ALIGN:          return "S7_ALIGN";
    case S10_WAIT_CONFIRM:  return "S10_WAIT_CONFIRM";
    case S9_FAULT:          return "S9_FAULT";
    default:                return "S?";
  }
}

static const char* faultName(uint8_t f) {
  switch (f) {
    case F_NONE:          return "无";
    case F_VBAT_LOW:      return "电压过低";
    case F_VBAT_HIGH:     return "电压过高";
    case F_WIND_TIMEOUT:  return "蓄能超时";
    case F_NO_RELEASE:    return "释放未脱啮";
    case F_PHASE_TIMEOUT: return "到位超时/打滑";
    case F_OVERCURRENT:   return "过流";
    case F_REENGAGE_FAIL: return "未重新啮合(需干预)";
    case F_LINK_LOST:     return "链路丢失安全停机";
    case F_POSTURE:       return "落稳超时/姿态异常";
    default:              return "未知";
  }
}

static void motorAllStop() {
  motorApply(DIR_COAST, 0);
  g_motorDuty = 0;
  pwmWrite(PIN_PWMA, 0);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
}

static void enter(State s) {
  g_state  = s;
  g_tState = millis();
  // ★★ 合规硬互锁（第 ② 重）：只要目标状态不是 S7_ALIGN，就无条件锁死轮电机。
  //    这样"进入蓄能/待发/释放/落稳/再啮合"的同一瞬间，轮就已经没有转矩了，
  //    不存在"状态对了但轮还在转"的时间窗。
  if (s != S7_ALIGN) {
    wheelLock();
  }
}

static void fault(uint8_t code) {
  motorAllStop();
  wheelLock();                 // 故障态同样必须有轮锁（enter 也会再兜一次）
  pwmWrite(PIN_SERVO_PITCH, 0);
#if HAVE_YAW_SERVO
  pwmWrite(PIN_SERVO_YAW, 0);
#endif
  g_fault = code;
  enter(S9_FAULT);
  Serial.printf("[FAULT] %u %s\n", code, faultName(code));
}

// ============================================================================
// 11. 动作
// ============================================================================
static void startWind();          // 前向声明（doPitchAdjust 里要用）

static uint16_t segPitchDeg() {
  const Seg& s = SEGS[g_seg];
  if (s.kind == SEG_TURN)  return CFG.pitchTurnDeg;   // ⚠ 停用分支，仅退回路径可达
  if (s.kind == SEG_PIVOT) return CFG.pitchJ1Deg;     // 不会用到：SEG_PIVOT 不蓄能不起跳
  if (g_seg == 0)          return CFG.pitchJ1Deg;
  return CFG.pitchJ3Deg;
}

// S8a：调俯仰 +（仅退回路径）调偏航，然后进 S2 蓄能
static void startPitchAdjust() {
  enter(S8a_PITCH);
}

static void doPitchAdjust() {
  uint16_t deg = segPitchDeg();
  servoSelect(PIN_SERVO_PITCH, deg, 300);

#if HAVE_YAW_SERVO
  if (SEGS[g_seg].kind == SEG_TURN) {
    enter(S8b_YAW);
    uint16_t yawDeg = (g_turnDir > 0) ? CFG.yawRightDeg : CFG.yawLeftDeg;
    servoSelect(PIN_SERVO_YAW, yawDeg, 300);   // 只选向，不产生偏航力矩
  } else {
    servoSelect(PIN_SERVO_YAW, CFG.yawCenterDeg, 200);   // 直行：回中
  }
#else
  // ★ V2.1：偏航舵机不装机 ⇒ 没有"选向"，掉头完全由 §7b 双轮差速完成。
  //   这里不再写任何偏航动作，也不会进 S8b_YAW（该状态在本构型下不可达）。
#endif

  startWind();        // ★ 真正开始卷簧（带 30ms 斜坡）
  enter(S2_WIND);
}

static void startWind() {
  // V2.1：只有 SEG_TURN（已停用）才用 turnWindPct，其余（含 SEG_PIVOT，虽然它不蓄能）用 windPct
  uint8_t pct = (SEGS[g_seg].kind == SEG_TURN) ? CFG.turnWindPct : CFG.windPct;
  g_tWindStart = millis();                 // 先记时，避免斜坡期间漏判
  motorRamp(DIR_FWD, pctToDuty(pct), CFG.rampMs);
  Serial.printf("[S2] 蓄能开始 seg=%u pct=%u%%\n", g_seg, pct);
}

// 预蓄能（ARM）：S1/S10 → S8a → S2 → S3
static void cmdArm(const char* why) {
  if (g_state != S1_IDLE && g_state != S10_WAIT_CONFIRM) {
    Serial.printf("[REJ] ARM 只能从 S1/S10 起，当前 %s\n", stateName(g_state));
    return;
  }
  g_fault = F_NONE;
  g_everArmed = true;
  Serial.printf("[CMD] ARM(%s) seg=%u\n", why, g_seg);
  startPitchAdjust();
}

static void cmdJump() {
  if (g_state != S3_ARMED) {
    Serial.printf("[REJ] JUMP 只能从 S3_ARMED 起，当前 %s\n", stateName(g_state));
    return;
  }
  Serial.printf("[CMD] JUMP seg=%u pulse=%u\n", g_seg, g_pulse);
  enter(S4_RELEASE);
  motorRamp(DIR_FWD, pctToDuty(CFG.releasePct), CFG.rampMs);
}

static void cmdStop(const char* why) {
  Serial.printf("[CMD] STOP(%s)\n", why);
  motorAllStop();
  pwmWrite(PIN_SERVO_PITCH, 0);
#if HAVE_YAW_SERVO
  pwmWrite(PIN_SERVO_YAW, 0);
#endif
  g_fault = F_NONE;
  g_retryJog = 0;
  enter(S1_IDLE);
}

// ----------------------------------------------------------------------------
// 11b. 区域2 调向（V2.0）
//   入口只允许 S1_IDLE / S10_WAIT_CONFIRM —— 这两个状态机械能已经卸掉，
//   转轮子不可能变成"用轮驱动跨越隔板"。S7 内部也只有一个动作在跑。
// ----------------------------------------------------------------------------
static void cmdWheelJob(WheelJob j, uint16_t argMm) {
  if (g_state != S1_IDLE && g_state != S10_WAIT_CONFIRM && g_state != S7_ALIGN) {
    Serial.printf("[REJ] 调向只能在 S1_IDLE / S10_WAIT_CONFIRM 发起，当前 %s"
                  "（蓄能/待发/释放/落稳/再啮合一律禁止轮驱动）\n", stateName(g_state));
    return;
  }
  if (!g_wheelReady) { Serial.println("[REJ] 轮 PWM 不可用"); return; }

  // ★ 分时供电：动轮之前先把蓄能电机彻底断开（占空比 0 + 两输入低 = coast）
  motorAllStop();
  pwmWrite(PIN_SERVO_PITCH, 0);
#if HAVE_YAW_SERVO
  pwmWrite(PIN_SERVO_YAW, 0);
#endif

  int8_t dL = 0, dR = 0;
  uint16_t ms = 0;
  uint8_t pct = CFG.wheelPct;
  switch (j) {
    case WJOB_PIVOT_L:                       // 俯视逆时针：左轮后退 + 右轮前进
      dL = -1; dR = +1; ms = CFG.turn180Ms; g_turnDir = -1;
      if (CFG.wheelAsymPct != 100) ms = (uint16_t)((uint32_t)ms * CFG.wheelAsymPct / 100);
      pct = 50; break;
    case WJOB_PIVOT_R:                       // 俯视顺时针
      dL = +1; dR = -1; ms = CFG.turn180Ms; g_turnDir = +1; pct = 50; break;
    case WJOB_FWD: {
      uint16_t mm = argMm ? argMm : CFG.nudgeMm;
      ms = (uint16_t)((uint32_t)mm * 1000UL / (CFG.wheelMmPerSec ? CFG.wheelMmPerSec : 1));
      dL = +1; dR = +1; break;
    }
    case WJOB_BACK: {
      uint16_t mm = argMm ? argMm : CFG.nudgeMm;
      ms = (uint16_t)((uint32_t)mm * 1000UL / (CFG.wheelMmPerSec ? CFG.wheelMmPerSec : 1));
      dL = -1; dR = -1; break;
    }
    default:
      return;
  }
  if (ms < 60) ms = 60;                      // 太短没有意义，且会让 N20 频繁启停

  if (g_state != S7_ALIGN) g_wReturn = g_state;
  // ★ 顺序很重要：先进 S7（唯一放行态），再踩轮子
  enter(S7_ALIGN);
  g_wjob = j; g_wjobMs = ms; g_tWjob = millis();
  Serial.printf("[S7] 调向开始 %s  进给=%ums  占空比=%u%%  dir=%+d  蓄能电机=0/1023 ★轮独占供电\n",
                wheelJobName(j), ms, (unsigned)pct, (int)g_turnDir);
  if (!wheelRamp(dL, dR, pct)) {
    Serial.println("[S7] 轮驱动被互锁拒绝 → 回到原状态");
    g_wjob = WJOB_NONE;
    enter(g_wReturn);
  }
}

static void cmdWheelStop(const char* why) {
  if (g_state != S7_ALIGN) {
    Serial.printf("[REJ] 当前不在调向态（%s）\n", stateName(g_state));
    return;
  }
  Serial.printf("[S7] 调向中止(%s) 已走 %lums\n", why, (unsigned long)(millis() - g_tWjob));
  wheelLock();                                // wheelLock 会把 g_wjob 清零
  enter(g_wReturn);
}

// S7 的每轮推进：非阻塞，随时可被 STOP / 掉线 / 互锁打断
static void stepWheelJob() {
  if (g_wjob == WJOB_NONE) { enter(g_wReturn); return; }
  uint32_t dt = millis() - g_tWjob;
  if (dt >= (uint32_t)g_wjobMs) {
    Serial.printf("[S7] 调向完成 %s 实走 %lums / 目标 %ums → 回 %s\n",
                  wheelJobName(g_wjob), (unsigned long)dt, (unsigned)g_wjobMs,
                  stateName(g_wReturn));
    wheelLock();
    enter(g_wReturn);
    return;
  }
  // 看门狗：再慢也不允许无限转（例如时间参数被写成 9999）
  if (dt > (uint32_t)g_wjobMs + 1500U) {
    Serial.printf("[S7] 调向看门狗超时 %lums → 强制锁死并回 %s\n",
                  (unsigned long)dt, stateName(g_wReturn));
    wheelLock();
    enter(g_wReturn);
  }
}


// 反转泄能 / 解除待发（S3 安全释放已储存的机械能；也可用于 S9 脱困）
static void cmdVent(const char* why) {
  if (g_state == S4_RELEASE || g_state == S2_WIND) {
    // 运行中泄能：先停再反转
    motorAllStop();
    delay(20);
  }
  Serial.printf("[CMD] VENT(%s)\n", why);
  motorRamp(DIR_REV, pctToDuty(CFG.ventPct), CFG.rampMs);
  delay(400);
  motorAllStop();
  g_fault = F_NONE;
  enter(S1_IDLE);
}

// 直接跳到第 N 段并预蓄能（网页上的三个大按钮）
static void cmdGoto(uint8_t seg) {
  if (seg >= 3) { Serial.println("[REJ] GOTO 段号超范围"); return; }
  if (g_state != S1_IDLE && g_state != S10_WAIT_CONFIRM && g_state != S3_ARMED) {
    Serial.printf("[REJ] GOTO 仅在空闲/待确认/待发时可用，当前 %s\n", stateName(g_state));
    return;
  }
  motorAllStop();
  g_seg = seg;
  g_pulse = (SEGS[seg].kind == SEG_TURN) ? 1 : 0;
  if (g_state == S3_ARMED) enter(S1_IDLE);   // 从待发跳段：先解除待发，再重新蓄能
  Serial.printf("[CMD] GOTO %u (%s)\n", seg, SEGS[seg].name);

  // ★ V2.1：掉头段不蓄能、不起跳 —— 直接停在 S10（轮驱动唯一放行态 + CONFIRM 可用的交集）
  if (SEGS[seg].kind == SEG_PIVOT) {
    enter(S10_WAIT_CONFIRM);
    Serial.println("[CMD] 该段为地面差速掉头：请用 wl/wr（或网页 ↺/↻）原地转 180°，");
    Serial.println("[CMD] 转到位后再按一次确认 → 进入第3跳；未转好可用 ↑/↓ 微调。");
    return;
  }

  cmdArm("goto");
}

// ⚠ V2.1 停用：脉冲掉头随偏航舵机一起取消。本命令保留只为兼容旧手柄/旧网页按钮，
//   收到后不动作，回一条指引。0x09 字节不复用，避免旧包被误解释成别的动作。
static void cmdTurnMore() {
  Serial.println("[REJ] TURN_MORE 已停用（偏航舵机不装机，脉冲掉头取消）");
  Serial.println("[REJ] 掉头请用 wl / wr（网页 ↺ 原地左转180° / ↻ 原地右转180°）");
}

/*
 * ★ CONFIRM：S10 的唯一出口。
 *   每次只放行一步：
 *     - 掉头段且脉冲未满 → 再打一发脉冲（⚠ 仅 HAVE_YAW_SERVO=1 的退回路径可达）
 *     - 下一段是掉头段 → 停在 S10，等操作员用轮把车转 180°（V2.1 主路径）
 *     - 否则 → 进入下一段；最后一段则结束回 S1
 *   没有"超时自动继续"，也没有"一次确认走多步"。
 */
static void cmdConfirm(const char* why) {
  if (g_state != S10_WAIT_CONFIRM) {
    Serial.printf("[REJ] CONFIRM 只能从 S10 起，当前 %s\n", stateName(g_state));
    return;
  }
  g_fault = F_NONE;
  const Seg& s = SEGS[g_seg];

  if (s.kind == SEG_TURN && g_pulse < CFG.turnPulses) {
    g_pulse++;
    Serial.printf("[CMD] CONFIRM(%s) → 掉头脉冲 %u/%u\n", why, g_pulse, CFG.turnPulses);
    startPitchAdjust();
    return;
  }

  if (g_seg + 1 < 3) {
    g_seg++;
    // 进入掉头段时，紧接着的这一跳就是第 1 发脉冲，所以从 1 开始计数
    g_pulse = (SEGS[g_seg].kind == SEG_TURN) ? 1 : 0;
    Serial.printf("[CMD] CONFIRM(%s) → 进入 %s\n", why, SEGS[g_seg].name);
    // ★ V2.1：掉头段就地等，不蓄能不起跳（此时仍停在 S10，轮驱动可用）
    if (SEGS[g_seg].kind == SEG_PIVOT) {
      Serial.println("[CMD] 该段不起跳：用 wl/wr（或网页 ↺/↻）原地转 180° 后再次确认");
      return;
    }
    startPitchAdjust();
    return;
  }

  Serial.printf("[CMD] CONFIRM(%s) → 全部完成，回 S1\n", why);
  enter(S1_IDLE);
}

// ============================================================================
// 12. 状态机主循环
// ============================================================================
static void stepStateMachine() {
  uint32_t dt = millis() - g_tState;
  pollHall();

  // ---- 全局安全检查 ----
  if (g_state != S9_FAULT && g_state != S0_BOOT) {
    // 电压要连续 3 次（≈0.75s）都越限才判故障：
    // 起跳瞬间母线会被拉低，单次采样低不代表电池不行，误跳故障比漏判更危险。
    static uint8_t lowCnt = 0, highCnt = 0;
    if (g_vbat > 0.1f) {
      lowCnt  = (g_vbat < VBAT_LOW_V)  ? (uint8_t)(lowCnt  + 1) : 0;
      highCnt = (g_vbat > VBAT_HIGH_V) ? (uint8_t)(highCnt + 1) : 0;
      if (lowCnt >= 3)  { lowCnt = 0;  fault(F_VBAT_LOW);  return; }
      if (highCnt >= 3) { highCnt = 0; fault(F_VBAT_HIGH); return; }
    }
    // 出力状态下掉线 → 立即停机（待发/待确认不出力，允许短暂失联）
    bool driving = (g_state == S2_WIND || g_state == S4_RELEASE ||
                    g_state == S6_REENGAGE || g_state == S7_ALIGN);
    // 调向是"在地面上真的动轮子"，不管有没有蓄过能都必须守链路
    bool needLink = (g_state == S7_ALIGN) || (driving && g_everArmed);
    if (needLink && !linkAlive()) {
      if (g_state == S7_ALIGN) { wheelLock(); }
      fault(F_LINK_LOST);
      return;
    }
  }

  switch (g_state) {

    case S0_BOOT:
      if (dt > 400) enter(S1_IDLE);
      break;

    case S1_IDLE:
      motorAllStop();
      break;

    case S7_ALIGN:
      // ★ 轮驱动唯一放行态：非阻塞推进，随时可被 STOP / 掉线 / 互锁打断
      stepWheelJob();
      break;

    case S8a_PITCH:
      doPitchAdjust();          // 内部会进入 S2_WIND
      break;

    case S8b_YAW:
      enter(S2_WIND);           // 兜底。⚠ V2.1：偏航舵机不装机，本状态不可达
      break;

    case S2_WIND: {
      // 找到承载待发相位 φ_hold → 停机
      if (holdPhaseReached()) {
        motorAllStop();
        Serial.printf("[S2] φ_hold 到达 @%lums, edges=%lu\n",
                      (unsigned long)(millis() - g_tWindStart), (unsigned long)g_phaseEdges);
        enter(S3_ARMED);        // 待发：零电流、可无限等待（赛前预蓄能就停在这）
      } else if (dt > CFG.windTimeoutMs) {
        fault(F_PHASE_TIMEOUT); // 12s 没到相 ⇒ 打滑/卡死/相位标定错
      }
      break;
    }

    case S3_ARMED:
      // 零电流保持；只接受 JUMP / STOP / VENT / 网页心跳
      if (dt > 600000UL) { /* 待发超过 10 分钟不动作，仅提示 */ }
      break;

    case S4_RELEASE: {
      // 脱啮判据：未装电流传感器时用"推进角上限 + 时间"兜底
      bool engaged = false;
      if (g_mA >= 0) {
        engaged = (g_mA >= 80);
      } else {
        engaged = (dt < CFG.releaseMaxMs);
      }
      bool edgeMoved = (g_tLastEdge > g_tState) && ((millis() - g_tLastEdge) < 60);
      if (!engaged || edgeMoved || dt > CFG.releaseMaxMs + 200) {
        motorAllStop();
        g_tRelease = millis();
        Serial.printf("[S4] 释放完成 dt=%lums\n", (unsigned long)dt);
        enter(S5_SETTLE);
      }
      break;
    }

    case S5_SETTLE:
      // 落稳确认：无 IMU 时只能计时（必须延长到 2s 上限，超时即故障）
      if (dt >= CFG.settleMs) {
        enter(S6_REENGAGE);
        g_retryJog = 0;
        motorRamp(DIR_FWD, pctToDuty(CFG.creepPct), CFG.rampMs);
      } else if (dt > CFG.settleMs + 1200) {
        fault(F_POSTURE);
      }
      break;

    case S6_REENGAGE: {
      if (!mayReengage()) {
        motorApply(DIR_COAST, 0);
        if (dt > 3000) fault(F_REENGAGE_FAIL);
        break;
      }
      if (dt > 60 && (g_tLastEdge > g_tState) && (millis() - g_tLastEdge) < 80) {
        // 霍尔翻过 = 齿已重新啮合
        motorApply(DIR_COAST, 0);
        Serial.printf("[S6] 已重新啮合 dt=%lums\n", (unsigned long)dt);
        g_retryJog = 0;
        enter(S10_WAIT_CONFIRM);
      } else if (dt > 1500) {
        // 退一点再进齿（最多 3 次）—— 纯遥控脱困，不需要用手
        if (g_retryJog < 3) {
          g_retryJog++;
          Serial.printf("[S6] 未啮合，退齿重试 %u/3\n", g_retryJog);
          motorRamp(DIR_REV, pctToDuty(CFG.creepPct), CFG.rampMs);
          delay(120);
          motorAllStop();
          motorRamp(DIR_FWD, pctToDuty(CFG.creepPct), CFG.rampMs);
          g_tState = millis();
        } else {
          fault(F_REENGAGE_FAIL);
        }
      }
      break;
    }

    case S10_WAIT_CONFIRM:
      // ★ 结构级人工闸门：无超时、不清状态、只有 CONFIRM 能出去
      // ★ V2.1：第2段(SEG_PIVOT)停留的就是本状态 —— 弹簧没有蓄能，
      //   所以这里放开双轮掉头（走 S7）在物理上是安全的：卷簧里没有能量。
      motorAllStop();
      pwmWrite(PIN_SERVO_PITCH, 0);
#if HAVE_YAW_SERVO
      pwmWrite(PIN_SERVO_YAW, 0);
#endif
      break;

    case S9_FAULT:
      motorAllStop();
      break;

    default:
      enter(S1_IDLE);
      break;
  }
}

// ============================================================================
// 13. ESP-NOW
// ----------------------------------------------------------------------------
//  ★ 重要：ESP-NOW 的接收回调跑在 Wi-Fi 任务上下文里，**绝对不能在里面做
//    delay() / 舵机选位 / 电机斜坡**（会把 Wi-Fi 任务卡住、丢包、甚至触发看门狗）。
//    所以回调只做三件事：校验 → 记链路时间 → 把指令塞进单槽队列；
//    真正的动作由 loop() 里的 pumpRemoteQueue() 在**主上下文**执行。
// ============================================================================
static void sendTelemetry(const uint8_t* mac, uint16_t seqAck);

// 单槽"最新指令"队列：手柄一次只按一个键，够用；
// 连续按多个键时保留最后一个（宁丢中间，不排队造成"迟到动作"）。
static volatile uint8_t g_qCmd = CMD_NONE;
static volatile uint8_t g_qArg = 0;
static volatile uint16_t g_qSeq = 0;

static void pumpRemoteQueue() {
  uint8_t c = g_qCmd;
  if (c == CMD_NONE) return;
  g_qCmd = CMD_NONE;
  uint8_t arg = g_qArg;

  switch (c) {
    case CMD_ARM:       cmdArm("now");         break;
    case CMD_JUMP:      cmdJump();             break;
    case CMD_CONFIRM:   cmdConfirm("now");     break;
    case CMD_DISARM:    cmdVent("now");        break;
    case CMD_STOP:      cmdStop("now");        break;
    case CMD_MACRO:     cmdArm("now-macro");   break;   // 只准备，不连跳
    case CMD_GOTO:      cmdGoto(arg);          break;
    case CMD_TURN_MORE: cmdTurnMore();         break;
    case CMD_PITCH:
      if (g_state == S1_IDLE || g_state == S10_WAIT_CONFIRM)
        servoSelect(PIN_SERVO_PITCH, arg > 180 ? 180 : arg, 300);
      break;
    case CMD_YAW:
#if HAVE_YAW_SERVO
      if (g_state == S1_IDLE || g_state == S10_WAIT_CONFIRM) {
        uint16_t d = (arg == 1) ? CFG.yawLeftDeg : (arg == 2 ? CFG.yawRightDeg : CFG.yawCenterDeg);
        servoSelect(PIN_SERVO_YAW, d, 300);
      }
#else
      // ★ V2.1：没有偏航舵机 ⇒ 明说，不静默吞掉，也不假装成功
      Serial.println("[REJ] CMD_YAW 已停用：正式构型无偏航舵机（Yaw 鞋不装机）");
#endif
      break;
    // ★ V2.0 区域2 调向（cmdWheelJob 内部还会再查一次状态闸门）
    case CMD_WHEEL_PIVOT_L: cmdWheelJob(WJOB_PIVOT_L, 0);  break;
    case CMD_WHEEL_PIVOT_R: cmdWheelJob(WJOB_PIVOT_R, 0);  break;
    case CMD_WHEEL_FWD:     cmdWheelJob(WJOB_FWD,  arg);   break;
    case CMD_WHEEL_BACK:    cmdWheelJob(WJOB_BACK, arg);   break;
    case CMD_WHEEL_STOP:    cmdWheelStop("now");           break;
    default: break;
  }
}

static void handlePacket(const uint8_t* mac, const uint8_t* data, int len) {
  if (len < (int)sizeof(EnFrame)) return;
  EnFrame f;
  memcpy(&f, data, sizeof(f));
  if (f.magic != EN_MAGIC) return;
  uint8_t buf[5] = { f.cmd, f.arg, (uint8_t)(f.seq & 0xFF), (uint8_t)(f.seq >> 8), f.magic };
  if (crc8(buf, 5) != f.crc) return;      // 不打印，避免坏帧刷屏阻塞 Wi-Fi 任务

  g_lastNowMs = millis();
  g_nowSeen   = true;
  if (!g_remoteKnown || memcmp(g_remoteMac, mac, 6) != 0) {
    memcpy(g_remoteMac, mac, 6);
    g_remoteKnown = true;
    Serial.printf("[NOW] 手柄 MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }

  // 只入队，不在 Wi-Fi 任务里动作（见本节开头的说明）
  if (f.cmd != CMD_PING) {
    g_qArg = f.arg;
    g_qSeq = f.seq;
    g_qCmd = f.cmd;
  }
  sendTelemetry(mac, f.seq);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
static void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  handlePacket(info->src_addr, data, len);
}
#else
static void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  handlePacket(mac, data, len);
}
#endif

static bool ensurePeer(const uint8_t* mac) {
  if (esp_now_is_peer_exist(mac)) return true;
  esp_now_peer_info_t p{};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.encrypt = false;
  return esp_now_add_peer(&p) == ESP_OK;
}

static void sendTelemetry(const uint8_t* mac, uint16_t seqAck) {
  if (!g_remoteKnown) return;
  TlmFrame t{};
  t.magic   = 'J';
  t.state   = (uint8_t)g_state;
  t.seg     = g_seg;
  t.pulse   = g_pulse;
  t.fault   = g_fault;
  t.vbat100 = (uint16_t)(g_vbat * 100.0f);
  t.mA      = g_mA;
  t.seqAck  = seqAck;
  t.wheel   = wheelStateCode();          // V2.0：0=轮锁死 1=就绪 2=调向中
  uint8_t raw[sizeof(TlmFrame)];
  memcpy(raw, &t, sizeof(t));
  raw[sizeof(t) - 1] = crc8(raw, sizeof(t) - 1);
  if (ensurePeer(mac)) esp_now_send(mac, raw, sizeof(raw));
}

static void espNowInit() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("[NOW] 初始化失败（网页遥控仍可用）");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);

  // 广播对端（手柄端也用广播，简化配对）
  uint8_t bcast[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  ensurePeer(bcast);
  Serial.println("[NOW] 就绪（信道须与手柄一致，本机固定 1）");
}

// ============================================================================
// 14. 网页遥控（改造自 Ayson-A/Jumping-Robot 的 Web UI）
// ----------------------------------------------------------------------------
//  与原复刻版的差异：
//   ✗ 去掉 MJPEG 推流、去掉 esp_camera、去掉 app_httpd.cpp 双端口（80/81）
//   ✗ 去掉摇杆 / 差速 / 行走轮 / 速度滑杆
//   ✓ 换成三跳状态机面板：预蓄能 / 小跳1 / 掉头段(双轮差速) / 大跳3 / 推进下一步
//     / 起跳 / 反转泄能 / 急停，加一块状态卡（状态机、段、脉冲、电压、故障）
//   ✓ 加 /hb 心跳（链路看门狗用）
//  理由：复刻版用 2×N20 差速 + 摄像头，是"轮式 FPV 小车"；我们只能跳不能走，
//        摄像头与摇杆在我们的规则下既无用也增加违规风险（附录 1 §9 雷同）。
// ============================================================================
static WebServer server(80);
static uint32_t  g_httpCmdCount = 0;

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="zh-CN"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>跳跃机器人 · 02-5</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;background:#0d1014;color:#e8ecf2;font-family:-apple-system,"PingFang SC","Microsoft YaHei",sans-serif;padding:10px 10px 96px}
h1{font-size:16px;margin:2px 0 10px;font-weight:800;letter-spacing:.5px}
h1 span{font-size:11px;color:#7f8b9c;font-weight:500;margin-left:6px}
.card{background:#151b23;border:1px solid #222b36;border-radius:14px;padding:10px 12px;margin-bottom:10px}
.row{display:flex;justify-content:space-between;font-size:13px;padding:3px 0;color:#c6d0dc}
.row b{color:#fff;font-weight:700}
#st{font-weight:800}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
button{border:1px solid #2b3846;background:#1d2733;color:#e8ecf2;border-radius:12px;padding:14px 8px;font-size:14px;font-weight:700;line-height:1.25}
button:active{transform:scale(.985);background:#26333f}
button:disabled{opacity:.32}
.b1{background:#14243a;border-color:#2a4a72}
.b2{background:#3a2a12;border-color:#6b4a17}
.b3{background:#123027;border-color:#1d5a45}
.confirm{grid-column:1/3;background:#0f3a22;border-color:#1f8a52;padding:18px;font-size:17px;color:#7ef0ad}
.stop{background:#3d1212;border-color:#8a1f1f;color:#ffb4b4}
.hint{font-size:12px;color:#8f9bab;margin-top:8px;line-height:1.5}
#bar{position:fixed;left:0;right:0;bottom:0;padding:10px;background:linear-gradient(180deg,rgba(13,16,20,.6),#0d1014 40%)}
#bar button{width:100%;padding:18px;font-size:17px}
.warn{color:#ffcd6b}
</style></head><body>
<h1>跳跃机器人 02-5<span id="ver"></span></h1>

<div class="card">
  <div class="row"><span>状态机</span><b id="st">--</b></div>
  <div class="row"><span>当前段</span><b id="sg">--</b></div>
  <div class="row"><span>掉头脉冲</span><b id="pl">--</b><span style="font-size:12px;color:#8f9bab">(已停用)</span></div>
  <div class="row"><span>母线电压</span><b id="vb">--</b></div>
  <div class="row"><span>母线电流</span><b id="ma">--</b></div>
  <div class="row"><span>故障</span><b id="fl">--</b></div>
  <div class="row"><span>链路</span><b id="lk">--</b></div>
  <div class="hint" id="hint"></div>
</div>

<div class="card">
  <div class="grid3">
    <button class="b1" onclick="go(0)">小跳1<br>越单板</button>
    <button class="b1" onclick="go(1)">掉头段<br>(不起跳)</button>
    <button class="b1" onclick="go(2)">大跳3<br>越双板</button>
  </div>
  <div class="grid" style="margin-top:8px">
    <button class="b3" onclick="cmd('arm')">预蓄能</button>
    <button class="b3" onclick="cmd('jump')" id="bjump">起跳</button>
  </div>
  <div class="grid" style="margin-top:8px">
    <button class="b2" onclick="cmd('vent')">反转泄能</button>
    <button class="b2" onclick="cmd('wstop')">⏹ 停轮锁死</button>
  </div>
</div>

<div class="card">
  <div class="grid">
    <button class="confirm" onclick="cmd('confirm')" id="bconf">✓ 确认落稳 · 推进下一步</button>
  </div>
  <div class="hint">确认键是<b>唯一</b>能离开"待确认"的操作。机器人停住不动 ≠ 卡死，
  它在等你确认落稳。不会自己继续跳。</div>
</div>

<div class="card">
  <div class="row"><span>调向轮</span><b id="wh">--</b></div>
  <div class="row"><span>调向动作</span><b id="wj">--</b></div>
  <div class="grid">
    <button class="b2" onclick="cmd('pivotl')" id="bpl">↺ 原地左转<br>180°</button>
    <button class="b2" onclick="cmd('pivotr')" id="bpr">↻ 原地右转<br>180°</button>
  </div>
  <div class="grid" style="margin-top:8px">
    <button class="b3" onclick="cmd('wfwd')" id="bwf">↑ 前进微调<br>对准起跳位</button>
    <button class="b3" onclick="cmd('wback')" id="bwb">↓ 后退微调</button>
  </div>
  <div class="grid" style="margin-top:8px">
    <button class="b1" onclick="cmd('wstop')" id="bws">⏹ 停止调向</button>
    <button class="b1" onclick="cmd('wfwd',5)">前进 5mm</button>
  </div>
  <div class="hint">调向只在 <b>S1_IDLE</b> / <b>S10_WAIT_CONFIRM</b> 可用（此时弹簧未蓄能）。
  一旦蓄能/待发/起跳，轮电机由<b>软件强制制动锁死</b>，串口会打 [INTERLOCK]，
  从底层杜绝"用轮驱动跨越隔板"。掉头角度靠 <b>t180</b> 时间标定，不是开环角度。</div>
</div>

<div id="bar"><button class="stop" onclick="cmd('stop')">■ 急停</button></div>

<script>
const $=id=>document.getElementById(id);
let cmdAt=0;
function cmd(k,v){
  const t=Date.now(); if(t-cmdAt<300) return; cmdAt=t;
  fetch('/cmd?k='+k+(v!==undefined?('&v='+v):''),{cache:'no-store'})
   .then(r=>r.json()).then(j=>{ if(!j.ok) flash(j.msg); }).catch(()=>flash('指令发送失败'));
  setTimeout(poll,120);
}
function go(n){cmd('goto',n);}
function flash(m){$('hint').innerHTML='<span class="warn">'+m+'</span>';}
function poll(){
  fetch('/status',{cache:'no-store'}).then(r=>r.json()).then(s=>{
    $('ver').textContent='FW '+s.ver;
    $('st').textContent=s.stn;
    $('sg').textContent=s.segm;
    $('pl').textContent=s.k+' / '+s.n;
    $('vb').textContent=s.vbat.toFixed(2)+' V';
    $('ma').textContent=(s.mA<0?'未装传感器':s.mA+' mA');
    $('fl').textContent=(s.fault==0?'无':('#'+s.fault+' '+s.faultn));
    $('lk').textContent=s.link?'在线':'离线';
    $('lk').style.color=s.link?'#7ef0ad':'#ffb4b4';
    $('bjump').disabled=(s.st!==3);
    $('bconf').disabled=(s.st!==10);
    // 调向轮：0=锁死(禁止) 1=就绪 2=调向中
    const wOK=(s.wheel===1);
    const wRunning=(s.wheel===2);
    $('wh').textContent = wRunning?('调向中 '+s.wjob+' '+s.wms+'/'+s.wt180+'ms')
                          : (s.wheel===1?'就绪（弹簧未蓄能）':'锁死 · 禁止轮驱动');
    $('wh').style.color = wRunning?'#ffcd6b':(s.wheel===1?'#7ef0ad':'#ffb4b4');
    $('wj').textContent = wRunning?s.wjob:'—';
    $('bpl').disabled = !(wOK||wRunning);
    $('bpr').disabled = !(wOK||wRunning);
    $('bwf').disabled = !wOK;
    $('bwb').disabled = !wOK;
    $('bws').disabled = !wRunning;
    if(s.cal===0) $('hint').innerHTML='<span class="warn">固件未标定（相位/角度均为占位值），仅供台架调试</span>';
  }).catch(()=>{ $('lk').textContent='离线'; });
}
setInterval(poll,300);
setInterval(()=>fetch('/hb',{cache:'no-store'}).catch(()=>{}),200);
poll();
</script></body></html>
)HTML";

static String jsonStatus() {
  char b[560];
  const Seg& s = SEGS[g_seg];
  snprintf(b, sizeof(b),
    "{\"ok\":1,\"ver\":\"%s\",\"st\":%u,\"stn\":\"%s\",\"seg\":%u,\"segm\":\"%s\","
    "\"k\":%u,\"n\":%u,\"vbat\":%.2f,\"mA\":%d,\"fault\":%u,\"faultn\":\"%s\","
    "\"link\":%d,\"cal\":%u,\"phase\":%lu,"
    "\"wheel\":%u,\"wjob\":\"%s\",\"wms\":%u,\"wt180\":%u,\"wpct\":%u}",
    FW_VERSION, (unsigned)g_state, stateName(g_state), g_seg, s.name,
    g_pulse, CFG.turnPulses, g_vbat, (int)g_mA, g_fault, faultName(g_fault),
    linkAlive() ? 1 : 0, CFG.calibrateOk, (unsigned long)g_phaseEdges,
    (unsigned)wheelStateCode(), wheelJobName(g_wjob),
    (unsigned)(g_wjob == WJOB_NONE ? 0 : (uint32_t)(millis() - g_tWjob)),
    (unsigned)CFG.turn180Ms, (unsigned)CFG.wheelPct);
  return String(b);
}

static void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void handleStatus() {
  g_lastHttpMs = millis(); g_httpSeen = true;
  server.send(200, "application/json", jsonStatus());
}

static void handleHb() {
  g_lastHttpMs = millis(); g_httpSeen = true;
  server.send(204, "text/plain", "");
}

static void handleCmd() {
  g_lastHttpMs = millis(); g_httpSeen = true;
  String k = server.arg("k");
  String v = server.arg("v");
  bool ok = true;
  String msg = "ok";
  g_httpCmdCount++;

  if (k == "arm")            cmdArm("http");
  else if (k == "jump")      cmdJump();
  else if (k == "confirm")   cmdConfirm("http");
  else if (k == "stop")      cmdStop("http");
  else if (k == "vent")      cmdVent("http");
  else if (k == "turnmore")  cmdTurnMore();
  else if (k == "goto")      cmdGoto((uint8_t)v.toInt());
  else if (k == "pitch")     servoSelect(PIN_SERVO_PITCH, (uint16_t)v.toInt(), 300);
  // ★ V2.1：偏航舵机不装机 ⇒ 网页 yaw? 明确回错，不静默成功
  else if (k == "yaw") { ok = false; msg = "偏航舵机不装机(已停用)"; }
  // ★ V2.0 区域2 调向
  else if (k == "pivotl")    cmdWheelJob(WJOB_PIVOT_L, 0);
  else if (k == "pivotr")    cmdWheelJob(WJOB_PIVOT_R, 0);
  else if (k == "wfwd")      cmdWheelJob(WJOB_FWD,  (uint16_t)v.toInt());
  else if (k == "wback")     cmdWheelJob(WJOB_BACK, (uint16_t)v.toInt());
  else if (k == "wstop")     cmdWheelStop("http");
  else { ok = false; msg = "未知指令"; }

  char b[128];
  snprintf(b, sizeof(b), "{\"ok\":%d,\"msg\":\"%s\",\"st\":%u}", ok ? 1 : 0, msg.c_str(), (unsigned)g_state);
  server.send(200, "application/json", b);
}

static void webInit() {
  server.on("/",        handleRoot);
  server.on("/status",  handleStatus);
  server.on("/hb",      handleHb);
  server.on("/cmd",     handleCmd);
  server.onNotFound([]() { server.send(404, "text/plain", "404"); });
  server.begin();
}

// ============================================================================
// 15. 配置持久化 + 串口命令
// ============================================================================
static void cfgLoad() {
  prefs.begin("jumpcfg", true);
  CFG.rampMs        = prefs.getUShort("ramp",   CFG.rampMs);
  CFG.windPct       = prefs.getUChar ("wind",   CFG.windPct);
  CFG.turnWindPct   = prefs.getUChar ("twind",  CFG.turnWindPct);
  CFG.releasePct    = prefs.getUChar ("rel",    CFG.releasePct);
  CFG.creepPct      = prefs.getUChar ("creep",  CFG.creepPct);
  CFG.ventPct       = prefs.getUChar ("vent",   CFG.ventPct);
  CFG.holdStopMs    = prefs.getUShort("hold",   CFG.holdStopMs);
  CFG.tReturnMs     = prefs.getUShort("tret",   CFG.tReturnMs);
  CFG.minEdgeMs     = prefs.getUShort("minedge",CFG.minEdgeMs);
  CFG.settleMs      = prefs.getUShort("settle", CFG.settleMs);
  CFG.turnPulses    = prefs.getUChar ("npulse", CFG.turnPulses);
  CFG.pitchJ1Deg    = prefs.getUShort("pj1",    CFG.pitchJ1Deg);
  CFG.pitchTurnDeg  = prefs.getUShort("pturn",  CFG.pitchTurnDeg);
  CFG.pitchJ3Deg    = prefs.getUShort("pj3",    CFG.pitchJ3Deg);
  // ⚠ V2.1 停用：偏航舵机不装机（HAVE_YAW_SERVO=0）。yl/yc/yr 仍读写，
  //   只是没有任何执行路径会用到它们 —— 保留是为了把开关改回 1 时档位还在。
  CFG.yawLeftDeg    = prefs.getUShort("yl",     CFG.yawLeftDeg);
  CFG.yawCenterDeg  = prefs.getUShort("yc",     CFG.yawCenterDeg);
  CFG.yawRightDeg   = prefs.getUShort("yr",     CFG.yawRightDeg);
  // ---- V2.0 转向轮 ----
  CFG.wheelPct      = prefs.getUChar ("wpct",   CFG.wheelPct);
  CFG.wheelRampMs   = prefs.getUShort("wramp",  CFG.wheelRampMs);
  CFG.turn180Ms     = prefs.getUShort("t180",   CFG.turn180Ms);
  CFG.wheelAsymPct  = prefs.getUChar ("wasym",  CFG.wheelAsymPct);
  CFG.wheelMmPerSec = prefs.getUShort("wmms",   CFG.wheelMmPerSec);
  CFG.nudgeMm       = prefs.getUChar ("nudge",  CFG.nudgeMm);
  CFG.wheelInvL     = prefs.getUChar ("winvl",  CFG.wheelInvL);
  CFG.wheelInvR     = prefs.getUChar ("winvr",  CFG.wheelInvR);
  CFG.wheelLockMode = prefs.getUChar ("wlock",  CFG.wheelLockMode);
  CFG.calibrateOk   = prefs.getUChar ("cal",    CFG.calibrateOk);
  prefs.end();
}

static void cfgDump() {
  Serial.println("---- 当前配置（★=未标定项）----");
  Serial.printf(" ramp=%ums  wind=%u%%  twind=%u%%  rel=%u%%  creep=%u%%  vent=%u%%\n",
                CFG.rampMs, CFG.windPct, CFG.turnWindPct, CFG.releasePct, CFG.creepPct, CFG.ventPct);
  Serial.printf(" ★hold=%ums(φ_hold 停机延时, TBD)  ★tret=%ums(t_return, TBD)  minedge=%ums\n",
                CFG.holdStopMs, CFG.tReturnMs, CFG.minEdgeMs);
  Serial.printf(" settle=%ums  npulse=%u\n", CFG.settleMs, CFG.turnPulses);
  Serial.printf(" ★pitch: J1=%u° turn=%u°(停用) J3=%u°\n",
                CFG.pitchJ1Deg, CFG.pitchTurnDeg, CFG.pitchJ3Deg);
#if !HAVE_YAW_SERVO
  Serial.println(" 偏航舵机: 不装机(HAVE_YAW_SERVO=0) ⇒ 无 yaw 档位/S8b_YAW/脉冲掉头，掉头靠双轮差速");
#endif
  Serial.printf(" ★轮: t180=%ums(turn180Ms, TBD)  asym=%u%%  wpct=%u%%  wramp=%ums  wmms=%umm/s  nudge=%umm\n",
                CFG.turn180Ms, CFG.wheelAsymPct, CFG.wheelPct, CFG.wheelRampMs,
                CFG.wheelMmPerSec, CFG.nudgeMm);
  Serial.printf("   轮: invL=%u invR=%u  锁死方式=%s  PWM=GPIO%d(共享) DIR_L=GPIO%d DIR_R=GPIO%d\n",
                CFG.wheelInvL, CFG.wheelInvR, CFG.wheelLockMode ? "制动" : "高阻",
                PIN_W_PWM, PIN_W_DIR_L, PIN_W_DIR_R);
  Serial.printf(" calibrateOk=%u %s\n", CFG.calibrateOk,
                CFG.calibrateOk ? "" : "← 未标定，禁止正式测试");
}

static void cfgSave() {
  prefs.begin("jumpcfg", false);
  prefs.putUShort("ramp",   CFG.rampMs);    prefs.putUChar("wind",   CFG.windPct);
  prefs.putUChar ("twind",  CFG.turnWindPct); prefs.putUChar("rel",  CFG.releasePct);
  prefs.putUChar ("creep",  CFG.creepPct);  prefs.putUChar("vent",   CFG.ventPct);
  prefs.putUShort("hold",   CFG.holdStopMs); prefs.putUShort("tret", CFG.tReturnMs);
  prefs.putUShort("minedge",CFG.minEdgeMs);
  prefs.putUShort("settle", CFG.settleMs);  prefs.putUChar("npulse", CFG.turnPulses);
  prefs.putUShort("pj1",    CFG.pitchJ1Deg); prefs.putUShort("pturn", CFG.pitchTurnDeg);
  prefs.putUShort("pj3",    CFG.pitchJ3Deg);
  // ⚠ V2.1 yl/yc/yr 保留写入（见 cfgLoad 说明），不影响任何执行路径
  prefs.putUShort("yl",     CFG.yawLeftDeg);
  prefs.putUShort("yc",     CFG.yawCenterDeg); prefs.putUShort("yr",  CFG.yawRightDeg);
  // ---- V2.0 转向轮 ----
  prefs.putUChar ("wpct",   CFG.wheelPct);   prefs.putUShort("wramp", CFG.wheelRampMs);
  prefs.putUShort("t180",   CFG.turn180Ms);  prefs.putUChar ("wasym", CFG.wheelAsymPct);
  prefs.putUShort("wmms",   CFG.wheelMmPerSec); prefs.putUChar("nudge", CFG.nudgeMm);
  prefs.putUChar ("winvl",  CFG.wheelInvL);  prefs.putUChar ("winvr", CFG.wheelInvR);
  prefs.putUChar ("wlock",  CFG.wheelLockMode);
  prefs.putUChar ("cal",    CFG.calibrateOk);
  prefs.end();
  Serial.println("[CFG] 已保存到 NVS");
}

static void printHelp() {
  Serial.println();
  Serial.println("================ 串口命令 ================");
  Serial.println(" a            预蓄能 (ARM)");
  Serial.println(" j            起跳 (仅 S3)");
  Serial.println(" c            确认落稳 (仅 S10，唯一出口)");
  Serial.println(" s            急停");
  Serial.println(" v            反转泄能");
  Serial.println(" g0 / g1 / g2 直接跳到 第1跳 / 掉头段 / 第3跳(掉头段不蓄能)");
  Serial.println(" p <deg>      单独调俯仰舵机");
#if HAVE_YAW_SERVO
  Serial.println(" y <0|1|2>    单独调偏航舵机 (中/左/右)");
#else
  Serial.println(" y            已停用（正式构型无偏航舵机，掉头用 wl/wr）");
#endif
  Serial.println(" ---- 区域2 调向（V2.1，仅 S1/S10 可用）----");
  Serial.println(" wl           原地左转 180°   (逆时针)");
  Serial.println(" wr           原地右转 180°   (顺时针)");
  Serial.println(" wf [mm]      前进微调 (省略 mm = CFG.nudgeMm)");
  Serial.println(" wb [mm]      后退微调");
  Serial.println(" ws           中止调向并锁死轮电机");
  Serial.println(" wi           查询轮锁/互锁状态");
  Serial.println(" d            打印状态");
  Serial.println(" cfg          打印配置");
  Serial.println(" set <k> <v>  改配置, 例: set t180 1500");
  Serial.println("               k = ramp wind twind rel creep vent");
  Serial.println("                   hold tret settle npulse");
  Serial.println("                   pj1 pturn pj3 cal");
  Serial.println("                   wpct wramp t180 wasym wmms nudge");
  Serial.println("                   winvl winvr wlock");
#if HAVE_YAW_SERVO
  Serial.println("                 + yl yc yr (仅偏航舵机装机时)");
#endif
  Serial.println(" save         写入 NVS");
  Serial.println("==========================================");
  Serial.println();
}

static void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.isEmpty()) return;
  char c = line.charAt(0);

  if (line == "cfg")  { cfgDump(); return; }
  if (line == "save") { cfgSave(); return; }
  if (line == "help" || c == '?') { printHelp(); return; }

  if (line.startsWith("set ")) {
    int sp = line.indexOf(' ', 4);
    if (sp < 0) { Serial.println("[CFG] 用法: set <k> <v>"); return; }
    String k = line.substring(4, sp); k.trim();
    long v = line.substring(sp + 1).toInt();
    if      (k == "ramp")   CFG.rampMs       = v;
    else if (k == "wind")   CFG.windPct      = v;
    else if (k == "twind")  CFG.turnWindPct  = v;
    else if (k == "rel")    CFG.releasePct   = v;
    else if (k == "creep")  CFG.creepPct     = v;
    else if (k == "vent")   CFG.ventPct      = v;
    else if (k == "hold")   CFG.holdStopMs   = v;
    else if (k == "tret")   CFG.tReturnMs    = v;
    else if (k == "minedge")CFG.minEdgeMs    = v;
    else if (k == "settle") CFG.settleMs     = v;
    else if (k == "npulse") CFG.turnPulses   = v;
    else if (k == "pj1")    CFG.pitchJ1Deg   = v;
    else if (k == "pturn")  CFG.pitchTurnDeg = v;
    else if (k == "pj3")    CFG.pitchJ3Deg   = v;
#if HAVE_YAW_SERVO
    else if (k == "yl")     CFG.yawLeftDeg   = v;
    else if (k == "yc")     CFG.yawCenterDeg = v;
    else if (k == "yr")     CFG.yawRightDeg  = v;
#endif
    // ---- V2.0 转向轮 ----
    else if (k == "wpct")   CFG.wheelPct      = v;
    else if (k == "wramp")  CFG.wheelRampMs   = v;
    else if (k == "t180")   CFG.turn180Ms     = v;
    else if (k == "wasym")  CFG.wheelAsymPct  = v;
    else if (k == "wmms")   CFG.wheelMmPerSec = v;
    else if (k == "nudge")  CFG.nudgeMm       = v;
    else if (k == "winvl")  CFG.wheelInvL     = v;
    else if (k == "winvr")  CFG.wheelInvR     = v;
    else if (k == "wlock")  CFG.wheelLockMode = v;
    else if (k == "cal")    CFG.calibrateOk  = v;
    else { Serial.println("[CFG] 未知键"); return; }
    Serial.printf("[CFG] %s = %ld （记得 save）\n", k.c_str(), v);
    return;
  }

  switch (c) {
    case 'a': cmdArm("serial");   break;
    case 'j': cmdJump();          break;
    case 'c': cmdConfirm("serial"); break;
    case 's': cmdStop("serial");  break;
    case 'v': cmdVent("serial");  break;
    case 'd': Serial.println(jsonStatus()); break;
    case 'g': {
      long n = line.substring(1).toInt();
      cmdGoto((uint8_t)n);
      break;
    }
    case 'p': servoSelect(PIN_SERVO_PITCH, (uint16_t)line.substring(1).toInt(), 300); break;
    case 'y': {
#if HAVE_YAW_SERVO
      long n = line.substring(1).toInt();
      servoSelect(PIN_SERVO_YAW,
                  n == 1 ? CFG.yawLeftDeg : (n == 2 ? CFG.yawRightDeg : CFG.yawCenterDeg), 300);
#else
      Serial.println("[REJ] y 已停用：正式构型无偏航舵机（掉头用 wl / wr）");
#endif
      break;
    }
    case 'w': {
      String sub = line.substring(0, 2);
      if (sub == "wl")      cmdWheelJob(WJOB_PIVOT_L, 0);
      else if (sub == "wr") cmdWheelJob(WJOB_PIVOT_R, 0);
      else if (sub == "wf") cmdWheelJob(WJOB_FWD,  (uint16_t)line.substring(2).toInt());
      else if (sub == "wb") cmdWheelJob(WJOB_BACK, (uint16_t)line.substring(2).toInt());
      else if (sub == "ws") cmdWheelStop("serial");
      else if (sub == "wi")
        Serial.printf("[WHEEL] state=%s 闸门=%s dirs=%+d/%+d D=%u/1023 锁死方式=%s\n",
                      stateName(g_state), wheelAllowed() ? "放行(仅S7)" : "锁死",
                      (int)g_wDirL, (int)g_wDirR, (unsigned)g_wDuty,
                      CFG.wheelLockMode ? "制动" : "高阻");
      else Serial.println("[?] w 子命令: wl wr wf wb ws wi");
      break;
    }
    default: Serial.println("[?] 未知命令，输入 help"); break;
  }
}

// ============================================================================
// 16. setup / loop
// ============================================================================
static const char* AP_SSID = "JumpRobot-02-5";
static const char* AP_PASS = "jump2026";

static void ledPattern() {
  uint32_t t = millis();
  bool on = false;
  switch (g_state) {
    case S1_IDLE:          on = (t / 900) % 2 == 0; break;              // 慢闪
    case S3_ARMED:         on = true; break;                             // 常亮（已蓄能，危险）
    case S10_WAIT_CONFIRM: on = ((t / 200) % 4) < 2; break;              // 双闪
    case S7_ALIGN:         on = ((t / 120) % 6) < 3; break;              // 三连闪（调向中）
    case S9_FAULT:         on = (t / 100) % 2 == 0; break;               // 快闪
    case S2_WIND:
    case S4_RELEASE:       on = (t / 60)  % 2 == 0; break;               // 极快闪
    default:               on = false; break;
  }
  digitalWrite(PIN_LED, on ? LOW : HIGH);   // 低有效
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  pwmSetup(PIN_PWMA, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  pwmWrite(PIN_PWMA, 0);

  Serial.begin(115200);
  delay(120);
  Serial.println();
  Serial.println("=====================================================");
  Serial.printf(" %s  %s  v%s\n", FW_TAG, "跳跃机器人机载端", FW_VERSION);
  Serial.println(" 2026秋《自然与设计》02-5 · 5V 单母线 · 单电机缺齿加载");
  Serial.println("=====================================================");

  cfgLoad();
  cfgDump();

  // 齿轮链实测参数（STEP/STL 解析，见 03 文档 §3.3c）
  Serial.println("---- 齿轮链（实测几何，非假设）----");
  Serial.printf(" %dT(输入) → %dT/%dT(双联) → 末级名义 %d 齿位、中央层 %d 齿 → Rack %d 周期齿\n",
                GEAR_INPUT_TEETH, GEAR_COMPOUND_BIG, GEAR_COMPOUND_SMALL,
                GEAR_MAIN_NOMINAL, GEAR_MAIN_CENTER, 12);
  Serial.printf(" 级间 %.4f × %.4f = 总减速比 %.4f:1（主轮 1 圈 = 电机轴 %.3f 圈）\n",
                GEAR_STAGE1, GEAR_STAGE2, GEAR_TOTAL_RATIO, GEAR_TOTAL_RATIO);
  Serial.printf(" 齿距 %.3fmm（S060 档，机械线已确认）；原尺寸 %.6fmm\n",
                RACK_PITCH_MM, RACK_PITCH_MM_FULL);
  Serial.printf(" 名义加载长度 = %d 节距 × 齿距 ⇒ %.2f mm（S060）/ %.2f mm（原尺寸）\n",
                GEAR_MAIN_CENTER, LOAD_STROKE_MM, LOAD_STROKE_MM_FULL);
  Serial.println("   ↑ 只是单向加载段长度，**总周期仍须走完 mod360° 全周相位**（含回位空程）");
  Serial.printf(" 中央层缺 %d 齿位 = %.3f°（节距跨度，不是空白弧）\n",
                GEAR_MAIN_MISSING, MISSING_SLOTS_DEG);
  Serial.printf(" 齿顶中心实测 %d 个，与 360/%d 栅格吻合（r>21.788mm 连续齿顶区间中点）\n",
                GEAR_TIP_CENTER_COUNT, GEAR_MAIN_NOMINAL);
  Serial.println(" ★ z0 可作 CAD 参考截面，但不是判据；各 Z 相位差是**确定性几何偏置**，");
  Serial.println("   全齿宽安全窗 = 各 Z 窗口在共同轮角 φ 中的交集（须在整个接触域上求，中部也可能是极值）。");
  Serial.println(" ★ 检测精度由 磁滞/安装/采样/转速/停止延迟/滑行/载荷 决定，与齿宽相位差没有必然下限，");
  Serial.println("   具体能做到多少**必须实测**，本固件不预设任何角度下限。");
  Serial.println(" ★ 128.571° / 108.15° 都不是安全窗，且不在固件里（只在几何报告）。");
  Serial.println(" ★ 全周 mod360° 相位必须保留：回位空程不能省，不能用'距离停车'替代相位判据。");
  Serial.println(" ★磁钢装主轮还是装电机轴，会使沿间隔差 2.857 倍 —— 必须先确认！");
  Serial.println(" ★★ 以上全是【名义/几何】值：不是已装配可用行程，也不是停车窗。");
  Serial.println("    实际可用行程 / φ_hold / φ_stop / t_return 一律台架实测后才填。");
  Serial.println("--------------------------------");
  Serial.println("---- ★★ 跨隔板轮驱动互锁（PDF 第14页第9条 / 正文'必须跳'）----");
  Serial.println(" 轮电机唯一允许出力的状态：S7_ALIGN（区域2 原地调向）");
  Serial.println(" 进 S7 的唯一入口：S1_IDLE / S10_WAIT_CONFIRM（此时弹簧均未蓄能）");
  Serial.println(" 蓄能(S2)/待发(S3)/释放(S4)/落稳(S5)/再啮合(S6)：");
  Serial.println("   ① enter() 在迁移瞬间无条件锁死轮电机；");
  Serial.println("   ② wheelGuard() 每轮 loop 复核，发现非 S7 有力即锁死并打 [INTERLOCK]；");
  Serial.println("   ③ wheelRamp()（唯一对外驱动入口）内部再查一次闸门，不合规直接拒绝。");
  Serial.println(" 另有'分时供电'闸门：S7 内若蓄能电机占空比 ≠ 0，轮电机立即让路。");
  Serial.println(" ⇒ 5V 母线任何时刻只有一个执行器组吃电流，峰值不叠加。");
  Serial.println("--------------------------------");
  if (!CFG.calibrateOk) {
    Serial.println();
    Serial.println(" ######  !!! 警告：本机尚未标定  !!!  ######");
    Serial.println(" # 相位延时(hold/tret)、起跳角均为占位值（偏航档位已随舵机停用）。");
    Serial.println(" # 机械线的 φ_hold / φ_stop / t_return 仍是 TBD。");
    Serial.println(" # 请先做台架标定，再执行 `set cal 1` + `save` 解除本警告。");
    Serial.println(" ##########################################");
    Serial.println();
  }

  sensorsInit();
  servosInit();
  wheelInit();            // ★ 必须在 enter(S1_IDLE) 之前：enter 会调 wheelLock()

  // Wi-Fi：AP 给网页，STA 给 ESP-NOW（AP_STA 并存最稳）
  WiFi.mode(WIFI_AP_STA);
  uint8_t mac[6]; WiFi.macAddress(mac);
  Serial.printf("[WiFi] 本机 STA MAC %02X:%02X:%02X:%02X:%02X:%02X（手柄端填这个）\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);   // 信道固定 1，ESP-NOW 必须同信道
  Serial.printf("[WiFi] 热点: %s  密码: %s  IP: %s\n",
                AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());

  espNowInit();
  webInit();

  g_vbat   = readVbat();
  g_bootMs = millis();
  Serial.printf("[PWR] 母线 %.2f V（估算值，分压 220k/100k）\n", g_vbat);

  printHelp();
  g_fault = F_NONE;
  enter(S1_IDLE);
  Serial.println("[BOOT] 就绪。S1_IDLE。");
}

static uint32_t g_tVolt = 0;

void loop() {
  server.handleClient();
  handleSerial();

  uint32_t now = millis();
  if (now - g_tVolt > 250) {          // 电压 4Hz 采样，别把 loop 拖慢
    g_tVolt = now;
    g_vbat = readVbat();
  }

  pumpRemoteQueue();      // 执行 ESP-NOW 收到的手柄指令（主上下文）
  wheelGuard();           // ★ 合规闸门：非 S7_ALIGN 一律锁死轮电机（留痕）
  stepStateMachine();
  ledPattern();

  // 待发/待确认状态下周期性提示"没卡死，在等人确认"
  static uint32_t tNote = 0;
  if ((g_state == S3_ARMED || g_state == S10_WAIT_CONFIRM) && now - tNote > 3000) {
    tNote = now;
    Serial.printf("[WAIT] %s · 段=%u(%s) · 等待遥控（不会自行继续）\n",
                  stateName(g_state), g_seg, SEGS[g_seg].name);
    if (g_state == S10_WAIT_CONFIRM && SEGS[g_seg].kind == SEG_PIVOT) {
      Serial.println("[WAIT] 本段为地面掉头：wl=左转180° wr=右转180° wi=前进5mm 后按确认进第3跳");
    }
  }

  delay(2);
}
