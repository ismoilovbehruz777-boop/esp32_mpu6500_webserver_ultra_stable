include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>

// ===========================================================
// ESP32 + MPU-6500 ULTRA STABLE WEB VIEW
//c:\Users\user\Documents\Arduino\esp32_mpu6500_webserver_ultra_stable-main\esp32_mpu6500_webserver_ultra_stable-main.ino
// Упор этой версии:
// - максимальная живучесть веб-сервера
// - устойчивое отображение ориентации
// - защита от I2C-сбоев и выбросов
// - защита от NaN / бесконечностей
// - автоматическое восстановление MPU после серии ошибок
// - уменьшенная нагрузка на Wi-Fi и браузер
//
// ВАЖНО:
// MPU-6500 = 6 осей (гироскоп + акселерометр), БЕЗ магнитометра.
// Поэтому абсолютный yaw он не знает. Здесь yaw сделан максимально
// стабильным визуально, но не является "настоящим компасом".
// ===========================================================

// -------------------- Настройки подключения --------------------
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_ADDR 0x68   // если AD0 на 3.3V, поставь 0x69

const char* AP_SSID = "ESP32-MPU6500-ULTRA";
const char* AP_PASS = "12345678";

WebServer server(80);

// -------------------- Тайминги --------------------
static const uint32_t SENSOR_PERIOD_US = 4000; // 250 Гц: легче для ESP32 и достаточно для веб-визуализации
static const uint32_t WEB_PERIOD_MS    = 50;   // обновление данных на сервере 20 Гц

// -------------------- Масштабы MPU --------------------
static const float ACC_SCALE  = 16384.0f; // ±2g
static const float GYRO_SCALE = 65.5f;    // ±500 dps

// -------------------- Состояние ориентации --------------------
float rollDeg  = 0.0f;
float pitchDeg = 0.0f;
float yawDeg   = 0.0f;

// -------------------- Bias гироскопа (raw) --------------------
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

// -------------------- Статус / телеметрия --------------------
float accMagG = 1.0f;
float gyroAbsDps = 0.0f;
float accelTrust = 1.0f;

bool stationary = false;
bool freefall = false;
bool shock = false;
bool holdMode = false;
bool mpuOk = false;

uint32_t stationaryCount = 0;
uint32_t goodAccelCount = 0;
uint32_t badReadCount = 0;
uint32_t recoverCount = 0;

// -------------------- Время --------------------
uint32_t lastSensorMicros = 0;
uint32_t lastWebBuildMs = 0;

// -------------------- Кэш веб-данных --------------------
char dataCache[256] = "{\"roll\":0,\"pitch\":0,\"yaw\":0,\"accMag\":1,\"gyroAbs\":0,\"trust\":1,"
                      "\"stationary\":false,\"freefall\":false,\"shock\":false,\"hold\":false,\"ok\":true}";

// -------------------- Фильтры --------------------
bool lpInit = false;
float axLP = 0.0f, ayLP = 0.0f, azLP = 1.0f;

// median-of-3 буферы для гашения одиночных выбросов
int16_t axHist[3] = {0,0,0};
int16_t ayHist[3] = {0,0,0};
int16_t azHist[3] = {0,0,0};
int16_t gxHist[3] = {0,0,0};
int16_t gyHist[3] = {0,0,0};
int16_t gzHist[3] = {0,0,0};
uint8_t histFill = 0;

// -------------------- HTML --------------------
const char MAIN_page[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ru">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>MPU-6500 Ultra Stable</title>
  <style>
    body{
      margin:0;
      font-family:Arial,sans-serif;
      background:#101418;
      color:#e8eef5;
      text-align:center;
    }
    h1{
      margin:14px 0 6px;
      font-size:24px;
    }
    .sub{
      opacity:.8;
      margin-bottom:10px;
      font-size:13px;
    }
    .wrap{
      display:flex;
      flex-direction:column;
      align-items:center;
      gap:14px;
      padding:10px 12px 20px;
    }
    .scene{
      width:230px;
      height:230px;
      perspective:900px;
    }
    .cube{
      width:110px;
      height:110px;
      position:relative;
      transform-style:preserve-3d;
      transform:rotateX(0deg) rotateY(0deg) rotateZ(0deg);
      transition:transform .08s linear;
      margin:60px auto;
    }
    .face{
      position:absolute;
      width:110px;
      height:110px;
      border:2px solid rgba(255,255,255,.75);
      box-sizing:border-box;
      display:flex;
      align-items:center;
      justify-content:center;
      font-weight:bold;
      font-size:18px;
      background:rgba(90,170,255,.18);
    }
    .front { transform:translateZ(55px); }
    .back  { transform:rotateY(180deg) translateZ(55px); }
    .right { transform:rotateY(90deg) translateZ(55px); }
    .left  { transform:rotateY(-90deg) translateZ(55px); }
    .top   { transform:rotateX(90deg) translateZ(55px); background:rgba(80,255,160,.18); }
    .bottom{ transform:rotateX(-90deg) translateZ(55px); }

    .panel{
      width:min(94vw,470px);
      background:#171d24;
      border-radius:16px;
      padding:14px;
      box-sizing:border-box;
      text-align:left;
      box-shadow:0 8px 24px rgba(0,0,0,.25);
    }
    .row{
      display:flex;
      justify-content:space-between;
      margin:8px 0;
      font-size:18px;
    }
    .value{ font-weight:bold; }
    .small{ font-size:13px; opacity:.8; }
    .btns{
      display:flex;
      gap:10px;
      flex-wrap:wrap;
      justify-content:center;
    }
    button{
      border:none;
      border-radius:10px;
      padding:12px 16px;
      font-size:15px;
      cursor:pointer;
      background:#3b82f6;
      color:white;
    }
    .barWrap{
      height:12px;
      background:#26303a;
      border-radius:999px;
      overflow:hidden;
      margin:10px 0 4px;
    }
    .bar{
      height:100%;
      width:100%;
      background:linear-gradient(90deg,#ff6b6b,#f7b731,#26de81);
      transition:width .08s linear;
    }
    .warn{
      width:min(94vw,470px);
      font-size:13px;
      opacity:.82;
      line-height:1.45;
      text-align:left;
      background:#141a20;
      border-radius:14px;
      padding:12px 14px;
      box-sizing:border-box;
    }
  </style>
</head>
<body>
  <h1>MPU-6500 Ultra Stable</h1>
  <div class="sub">повышенная устойчивость к рывкам и зависаниям</div>

  <div class="wrap">
    <div class="scene">
      <div class="cube" id="cube">
        <div class="face front">F</div>
        <div class="face back">B</div>
        <div class="face right">R</div>
        <div class="face left">L</div>
        <div class="face top">TOP</div>
        <div class="face bottom">BOT</div>
      </div>
    </div>

    <div class="panel">
      <div class="row"><span>Roll</span><span class="value" id="roll">0°</span></div>
      <div class="row"><span>Pitch</span><span class="value" id="pitch">0°</span></div>
      <div class="row"><span>Yaw <span class="small">(относительный)</span></span><span class="value" id="yaw">0°</span></div>
      <div class="row"><span>|A|</span><span class="value" id="accMag">1.00 g</span></div>
      <div class="row"><span>|G|</span><span class="value" id="gyroAbs">0.0 dps</span></div>
      <div class="row"><span>Доверие к акселю</span><span class="value" id="trustText">100%</span></div>
      <div class="barWrap"><div class="bar" id="trustBar"></div></div>
      <div class="row"><span>Статус</span><span class="value" id="statusText">OK</span></div>
    </div>

    <div class="btns">
      <button onclick="zeroYaw()">Сброс yaw</button>
      <button onclick="rebootBoard()">Перезагрузка</button>
    </div>

    <div class="warn">
      Эта версия сделана так, чтобы страница не "умирала" от плохих значений.
      Даже если MPU временно даст сбой, сервер должен продолжить работу и
      попытаться восстановить датчик автоматически.
    </div>
  </div>

  <script>
    let busy = false;

    function setStatus(d){
      let status = 'OK';
      if (!d.ok) status = 'MPU recover';
      else if (d.hold) status = 'HOLD / защита';
      else if (d.freefall) status = 'Свободное падение';
      else if (d.shock) status = 'Сильное ускорение';
      else if (d.stationary) status = 'Неподвижно';
      document.getElementById('statusText').textContent = status;
    }

    async function updateData(){
      if (busy) return;
      busy = true;
      try{
        const ctrl = new AbortController();
        const t = setTimeout(() => ctrl.abort(), 1200);

        const res = await fetch('/data', { cache:'no-store', signal: ctrl.signal });
        clearTimeout(t);

        const d = await res.json();

        document.getElementById('roll').textContent = d.roll.toFixed(1) + '°';
        document.getElementById('pitch').textContent = d.pitch.toFixed(1) + '°';
        document.getElementById('yaw').textContent = d.yaw.toFixed(1) + '°';
        document.getElementById('accMag').textContent = d.accMag.toFixed(2) + ' g';
        document.getElementById('gyroAbs').textContent = d.gyroAbs.toFixed(1) + ' dps';

        const trustPct = Math.max(0, Math.min(100, d.trust * 100));
        document.getElementById('trustText').textContent = trustPct.toFixed(0) + '%';
        document.getElementById('trustBar').style.width = trustPct + '%';

        setStatus(d);

        document.getElementById('cube').style.transform =
          `rotateX(${-d.pitch}deg) rotateY(${d.yaw}deg) rotateZ(${d.roll}deg)`;
      }catch(e){
        document.getElementById('statusText').textContent = 'Связь...';
      }
      busy = false;
    }

    async function zeroYaw(){
      try{ await fetch('/zero', { cache:'no-store' }); }catch(e){}
    }

    async function rebootBoard(){
      try{ await fetch('/reboot', { cache:'no-store' }); }catch(e){}
    }

    setInterval(updateData, 100); // всего 10 Гц запросов — легче для Wi-Fi и браузера
    updateData();
  </script>
</body>
</html>
)rawliteral";

// -------------------- Вспомогательные --------------------
static inline float clampf(float x, float lo, float hi){
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline bool finite3(float a, float b, float c){
  return isfinite(a) && isfinite(b) && isfinite(c);
}

float wrap180(float a){
  while(a > 180.0f) a -= 360.0f;
  while(a < -180.0f) a += 360.0f;
  return a;
}

float median3f(float a, float b, float c){
  if (a > b){ float t = a; a = b; b = t; }
  if (b > c){ float t = b; b = c; c = t; }
  if (a > b){ float t = a; a = b; b = t; }
  return b;
}

int16_t median3i(int16_t a, int16_t b, int16_t c){
  if (a > b){ int16_t t = a; a = b; b = t; }
  if (b > c){ int16_t t = b; b = c; c = t; }
  if (a > b){ int16_t t = a; a = b; b = t; }
  return b;
}

// -------------------- I2C / MPU --------------------
bool writeReg(uint8_t reg, uint8_t value){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return (Wire.endTransmission() == 0);
}

bool readBytes(uint8_t reg, uint8_t count, uint8_t* buf){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;

  int got = Wire.requestFrom((int)MPU_ADDR, (int)count);
  if (got != count) return false;

  for (uint8_t i = 0; i < count; i++) buf[i] = Wire.read();
  return true;
}

bool readMPURawOnce(int16_t& ax, int16_t& ay, int16_t& az,
                    int16_t& gx, int16_t& gy, int16_t& gz){
  uint8_t raw[14];
  if (!readBytes(0x3B, 14, raw)) return false;

  ax = (int16_t)((raw[0] << 8) | raw[1]);
  ay = (int16_t)((raw[2] << 8) | raw[3]);
  az = (int16_t)((raw[4] << 8) | raw[5]);
  gx = (int16_t)((raw[8] << 8) | raw[9]);
  gy = (int16_t)((raw[10] << 8) | raw[11]);
  gz = (int16_t)((raw[12] << 8) | raw[13]);

  return true;
}

bool readMPUFiltered(int16_t& ax, int16_t& ay, int16_t& az,
                     int16_t& gx, int16_t& gy, int16_t& gz){
  // До 3 попыток чтения
  int16_t ax0, ay0, az0, gx0, gy0, gz0;
  for (int tries = 0; tries < 3; tries++){
    if (readMPURawOnce(ax0, ay0, az0, gx0, gy0, gz0)){
      // простой контроль на странные значения
      // если совсем всё нули — считаем чтение подозрительным
      if (!(ax0 == 0 && ay0 == 0 && az0 == 0 && gx0 == 0 && gy0 == 0 && gz0 == 0)){
        axHist[0] = axHist[1]; axHist[1] = axHist[2]; axHist[2] = ax0;
        ayHist[0] = ayHist[1]; ayHist[1] = ayHist[2]; ayHist[2] = ay0;
        azHist[0] = azHist[1]; azHist[1] = azHist[2]; azHist[2] = az0;
        gxHist[0] = gxHist[1]; gxHist[1] = gxHist[2]; gxHist[2] = gx0;
        gyHist[0] = gyHist[1]; gyHist[1] = gyHist[2]; gyHist[2] = gy0;
        gzHist[0] = gzHist[1]; gzHist[1] = gzHist[2]; gzHist[2] = gz0;

        if (histFill < 3) histFill++;

        if (histFill >= 3){
          ax = median3i(axHist[0], axHist[1], axHist[2]);
          ay = median3i(ayHist[0], ayHist[1], ayHist[2]);
          az = median3i(azHist[0], azHist[1], azHist[2]);
          gx = median3i(gxHist[0], gxHist[1], gxHist[2]);
          gy = median3i(gyHist[0], gyHist[1], gyHist[2]);
          gz = median3i(gzHist[0], gzHist[1], gzHist[2]);
        } else {
          ax = ax0; ay = ay0; az = az0;
          gx = gx0; gy = gy0; gz = gz0;
        }

        badReadCount = 0;
        return true;
      }
    }
    delay(0);
  }

  badReadCount++;
  return false;
}

bool initMPU6500(){
  uint8_t who = 0;
  if (!readBytes(0x75, 1, &who)) return false;

  Serial.print("WHO_AM_I = 0x");
  Serial.println(who, HEX);

  if (who != 0x70){
    Serial.println("Это не MPU-6500 или неверный адрес.");
    return false;
  }

  // Сброс сна + PLL по X gyro
  if (!writeReg(0x6B, 0x01)) return false;
  delay(100);

  // Sample rate divider = 3 -> 250 Гц, легче и стабильнее
  if (!writeReg(0x19, 0x03)) return false;

  // DLPF gyro
  if (!writeReg(0x1A, 0x03)) return false;

  // Gyro ±500 dps
  if (!writeReg(0x1B, 0x08)) return false;

  // Accel ±2g
  if (!writeReg(0x1C, 0x00)) return false;

  // Accel DLPF
  if (!writeReg(0x1D, 0x03)) return false;

  return true;
}

bool calibrateAtStart(){
  const int samples = 1800;
  long sumGX = 0, sumGY = 0, sumGZ = 0;
  long sumAX = 0, sumAY = 0, sumAZ = 0;
  int valid = 0;

  Serial.println("Калибровка... не двигай модуль");

  for (int i = 0; i < samples; i++){
    int16_t ax, ay, az, gx, gy, gz;
    if (readMPUFiltered(ax, ay, az, gx, gy, gz)){
      sumGX += gx; sumGY += gy; sumGZ += gz;
      sumAX += ax; sumAY += ay; sumAZ += az;
      valid++;
    }
    delay(2);
  }

  if (valid < 500) return false;

  gyroBiasX = (float)sumGX / valid;
  gyroBiasY = (float)sumGY / valid;
  gyroBiasZ = (float)sumGZ / valid;

  float ax = ((float)sumAX / valid) / ACC_SCALE;
  float ay = ((float)sumAY / valid) / ACC_SCALE;
  float az = ((float)sumAZ / valid) / ACC_SCALE;

  if (!finite3(ax, ay, az)) return false;

  // Первичная ориентация только из гравитации
  rollDeg  = atan2f(ay, az) * 57.2957795131f;
  pitchDeg = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2957795131f;
  yawDeg   = 0.0f;

  axLP = ax; ayLP = ay; azLP = az;
  lpInit = true;

  return true;
}

bool recoverMPU(){
  Serial.println("Пытаюсь восстановить MPU...");
  recoverCount++;

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(12);

  if (!initMPU6500()) return false;
  if (!calibrateAtStart()) return false;

  badReadCount = 0;
  mpuOk = true;
  Serial.println("MPU восстановлен");
  return true;
}

// -------------------- Устойчивое обновление ориентации --------------------
void updateOrientation(){
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  if (!readMPUFiltered(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw)){
    mpuOk = false;
    if (badReadCount > 12){
      recoverMPU();
    }
    return;
  }

  mpuOk = true;

  uint32_t now = micros();
  float dt = (now - lastSensorMicros) * 1e-6f;
  lastSensorMicros = now;

  if (!isfinite(dt) || dt <= 0.0f || dt > 0.05f) dt = SENSOR_PERIOD_US * 1e-6f;

  // Перевод в физические величины
  float ax = axRaw / ACC_SCALE;
  float ay = ayRaw / ACC_SCALE;
  float az = azRaw / ACC_SCALE;

  if (!finite3(ax, ay, az)) return;

  // лёгкий LPF поверх median
  if (!lpInit){
    axLP = ax; ayLP = ay; azLP = az;
    lpInit = true;
  } else {
    const float alpha = 0.10f;
    axLP += alpha * (ax - axLP);
    ayLP += alpha * (ay - ayLP);
    azLP += alpha * (az - azLP);
  }

  if (!finite3(axLP, ayLP, azLP)) {
    axLP = 0.0f; ayLP = 0.0f; azLP = 1.0f;
  }

  float gxDps = (gxRaw - gyroBiasX) / GYRO_SCALE;
  float gyDps = (gyRaw - gyroBiasY) / GYRO_SCALE;
  float gzDps = (gzRaw - gyroBiasZ) / GYRO_SCALE;

  if (!finite3(gxDps, gyDps, gzDps)) return;

  // Ограничение совсем безумных выбросов
  gxDps = clampf(gxDps, -800.0f, 800.0f);
  gyDps = clampf(gyDps, -800.0f, 800.0f);
  gzDps = clampf(gzDps, -800.0f, 800.0f);

  gyroAbsDps = sqrtf(gxDps*gxDps + gyDps*gyDps + gzDps*gzDps);
  if (!isfinite(gyroAbsDps)) gyroAbsDps = 0.0f;

  accMagG = sqrtf(axLP*axLP + ayLP*ayLP + azLP*azLP);
  if (!isfinite(accMagG)) accMagG = 1.0f;

  float accErr = fabsf(accMagG - 1.0f);

  freefall = (accMagG < 0.22f);
  shock    = (accMagG > 1.85f);
  stationary = (accErr < 0.025f && gyroAbsDps < 1.2f);

  if (stationary) stationaryCount++;
  else stationaryCount = 0;

  if (accErr < 0.05f) goodAccelCount++;
  else goodAccelCount = 0;

  // Подстройка bias при неподвижности
  if (stationaryCount > 60){
    const float b = 0.003f;
    gyroBiasX += b * ((float)gxRaw - gyroBiasX);
    gyroBiasY += b * ((float)gyRaw - gyroBiasY);
    gyroBiasZ += b * ((float)gzRaw - gyroBiasZ);
  }

  // Deadband от медленного самоповорота
  if (fabsf(gxDps) < 0.12f) gxDps = 0.0f;
  if (fabsf(gyDps) < 0.12f) gyDps = 0.0f;
  if (fabsf(gzDps) < 0.20f) gzDps = 0.0f;

  // Если модуль долго неподвижен, вообще не даём yaw ползти
  if (stationaryCount > 90){
    gxDps = 0.0f;
    gyDps = 0.0f;
    gzDps = 0.0f;
  }

  // Доверие к акселю
  if (accErr < 0.02f) accelTrust = 1.0f;
  else if (accErr < 0.05f) accelTrust = 0.8f;
  else if (accErr < 0.10f) accelTrust = 0.35f;
  else accelTrust = 0.0f;

  // HOLD режим: при плохом акселе и не слишком большой угловой скорости
  holdMode = false;
  if ((freefall || shock || accErr > 0.18f) && gyroAbsDps < 50.0f){
    holdMode = true;
  }

  // Гироскопическая интеграция
  if (!holdMode){
    rollDeg  += gxDps * dt;
    pitchDeg += gyDps * dt;
    yawDeg   += gzDps * dt;
  }

  // Акселерометрические углы
  float rollAcc  = atan2f(ayLP, azLP) * 57.2957795131f;
  float pitchAcc = atan2f(-axLP, sqrtf(ayLP*ayLP + azLP*azLP)) * 57.2957795131f;

  if (!isfinite(rollAcc) || !isfinite(pitchAcc)){
    rollAcc = rollDeg;
    pitchAcc = pitchDeg;
  }

  // Коррекция roll/pitch только когда аксель нормальный
  if (!holdMode && accelTrust > 0.01f){
    float corr = 0.015f + 0.045f * accelTrust; // мягкая подстройка
    rollDeg  += corr * (rollAcc  - rollDeg);
    pitchDeg += corr * (pitchAcc - pitchDeg);
  }

  // Если снова всё спокойно — мягко и сильнее возвращаем roll/pitch к гравитации
  if (stationaryCount > 120 && goodAccelCount > 120){
    rollDeg  += 0.06f * (rollAcc  - rollDeg);
    pitchDeg += 0.06f * (pitchAcc - pitchDeg);
  }

  // Жёсткие пределы от развала
  if (!isfinite(rollDeg))  rollDeg = 0.0f;
  if (!isfinite(pitchDeg)) pitchDeg = 0.0f;
  if (!isfinite(yawDeg))   yawDeg = 0.0f;

  rollDeg  = wrap180(rollDeg);
  pitchDeg = clampf(pitchDeg, -89.9f, 89.9f);
  yawDeg   = wrap180(yawDeg);
}

void rebuildDataCache(){
  // При необходимости можно поменять знаки здесь:
  float showRoll  = rollDeg;
  float showPitch = pitchDeg;
  float showYaw   = yawDeg;

  if (!isfinite(showRoll))  showRoll = 0.0f;
  if (!isfinite(showPitch)) showPitch = 0.0f;
  if (!isfinite(showYaw))   showYaw = 0.0f;

  if (!isfinite(accMagG)) accMagG = 1.0f;
  if (!isfinite(gyroAbsDps)) gyroAbsDps = 0.0f;
  if (!isfinite(accelTrust)) accelTrust = 0.0f;

  snprintf(dataCache, sizeof(dataCache),
           "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
           "\"accMag\":%.3f,\"gyroAbs\":%.2f,\"trust\":%.3f,"
           "\"stationary\":%s,\"freefall\":%s,\"shock\":%s,\"hold\":%s,\"ok\":%s}",
           showRoll, showPitch, showYaw,
           accMagG, gyroAbsDps, accelTrust,
           stationary ? "true" : "false",
           freefall ? "true" : "false",
           shock ? "true" : "false",
           holdMode ? "true" : "false",
           mpuOk ? "true" : "false");
}

// -------------------- HTTP --------------------
void handleRoot(){
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.send_P(200, "text/html; charset=utf-8", MAIN_page);
}

void handleData(){
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.send(200, "application/json", dataCache);
}

void handleZero(){
  yawDeg = 0.0f;
  server.send(200, "text/plain", "OK");
}

void handleReboot(){
  server.send(200, "text/plain", "Rebooting");
  delay(100);
  ESP.restart();
}

// -------------------- setup / loop --------------------
void setup(){
  Serial.begin(115200);
  delay(1200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(12);

  if (!initMPU6500()){
    Serial.println("Ошибка инициализации MPU-6500");
    while(true){
      delay(100);
      yield();
    }
  }

  if (!calibrateAtStart()){
    Serial.println("Ошибка калибровки MPU-6500");
    while(true){
      delay(100);
      yield();
    }
  }

  lastSensorMicros = micros();
  lastWebBuildMs = millis();
  rebuildDataCache();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println();
  Serial.println("Wi-Fi точка создана");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("PASS: ");
  Serial.println(AP_PASS);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/zero", handleZero);
  server.on("/reboot", handleReboot);
  server.begin();

  Serial.println("Web server started");
}

void loop(){
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastSensorMicros) >= SENSOR_PERIOD_US){
    updateOrientation();
  }

  uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - lastWebBuildMs) >= WEB_PERIOD_MS){
    rebuildDataCache();
    lastWebBuildMs = nowMs;
  }

  server.handleClient();
  yield();
  delay(1);
}
