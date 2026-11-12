#include <Arduino.h>
#include <Adafruit_TinyUSB.h>  
#include <Wire.h>
#include <LSM6DS3.h>
#include <math.h>
#include <bluefruit.h>

LSM6DS3 imu(I2C_MODE, 0x6A);
BLEUart bleuart;   // Nordic UART Service: 휴대폰으로 상태 전송, 휴대폰에서 'r' 수신

// BLE 패킷: 정확히 20바이트(기본 BLE 페이로드 크기), little-endian
// uint32 time_ms; int16 ax,ay,az (0.001 g); int16 gx,gy,gz (0.1 deg/s);
// uint16 pressure_raw; uint8 status; uint8 tilt_deg
// status 비트: 0~3 = 상태(0 NORMAL, 1 WAIT_TILT, 2 VERIFY, 3 ALARM),
//              4 = 최근 3초 내 유효 파지(grip), 5 = 임계값 설정 완료
struct __attribute__((packed)) BlePacket {
  uint32_t time_ms;
  int16_t ax_mg, ay_mg, az_mg;
  int16_t gx_dps10, gy_dps10, gz_dps10;
  uint16_t pressure;
  uint8_t status;
  uint8_t tilt_deg;
};
static_assert(sizeof(BlePacket) == 20, "BlePacket must be 20 bytes");

int16_t scaledToI16(float value, float factor) {
  float s = value * factor;
  if (s > 32767.0f) s = 32767.0f;
  if (s < -32768.0f) s = -32768.0f;
  return (int16_t)roundf(s);
}

// ================== IMU 판정 임계값(1차 권장값) ==================
const float IMPACT_THRESHOLD_G        = 3.7f;  // 충격 감지: 합성가속도 크기 기준(g)
const float TILT_THRESHOLD_DEG        = 40.0f; // 충격 후 최소 자세 변화 각도(도)
const float STRONG_TILT_THRESHOLD_DEG = 60.0f; // 경로 B에서 요구하는 더 큰 자세 변화 각도(도)
const float STILL_GYRO_THRESHOLD_DPS  = 20.0f; // 정지 판단: 합성각속도 상한(도/초)
const float STILL_ACCEL_TOL_G         = 0.20f; // 정지 판단: 합성가속도와 1g의 차이 허용값(g)

// ================== 압력센서 변화 패턴 설정 ==================
const int PRESSURE_THRESHOLD = 300;               // 압력센서 값 자체가 300 이상인지 판단(증가량 300이 아님)
const unsigned long PRESSURE_CONFIRM_MS = 100;    // 300 이상이 0.1초 이상 이어져야 유효한 신호로 인정
const unsigned long PRESSURE_LOOKBACK_MS = 3000;  // 최근 3초 이내에 유효한 300 이상 신호가 있었는지 확인
const unsigned long PRESSURE_B_RECENCY_MS = 1500; // 경로 B: 충격 후 최근 1.5초 안에도 유효한 파지 신호 필요
const bool PRESSED_WHEN_HIGH = true;              // 잡았을 때 값이 낮아지는 역방향 센서일 때만 false로 변경
// ==================================================================

const int PRESSURE_PIN = D0;
const int LED_PIN = D6;
const int BUZZER_PIN = D7;

const unsigned long SAMPLE_INTERVAL_MS = 20;  // 센서 측정 주기 20ms(약 50Hz)
const unsigned long CSV_INTERVAL_MS = 100;    // CSV 시리얼 출력 주기 100ms(초당 10줄)
const unsigned long TILT_WINDOW_MS = 1500;
const unsigned long VERIFY_WINDOW_MS = 8000;
const unsigned long NO_GRIP_HOLD_MS = 3000;   // 경로 A: 유효한 300 이상 압력 신호가 3초간 사라짐
const unsigned long GRIP_HOLD_MS = 3000;      // 경로 B: 충격 후 3초 이상에 걸쳐 파지 신호 확인
const unsigned long STILL_HOLD_MS = 3000;     // 충격 후 정지 상태가 3초 이상 지속되어야 함

const float GRAVITY_ALPHA = 0.16f;
const float REFERENCE_ALPHA = 0.025f;

enum DetectState { NORMAL, WAIT_TILT, VERIFY, ALARM };
DetectState state = NORMAL;

unsigned long lastSampleMs = 0, lastCsvMs = 0, impactMs = 0;

// 압력 감지: 순간적인 튐을 걸러낸 300 이상 신호와 최근 3초간의 이력 저장
bool rawHighTiming = false;
unsigned long rawHighSince = 0;
bool pressureHighStable = false;
bool everValidPeak = false;
unsigned long lastValidPeakMs = 0;

// 경로 B: 충격 후 처음 확인된 300 이상 압력 신호의 시점을 저장하고
// 3초 이후에도 최근 파지 신호(또는 연속된 높은 값)가 있는지 검사
bool postImpactGripStarted = false;
unsigned long postImpactGripSince = 0;

bool stillTiming = false;
unsigned long stillSince = 0;

bool gravityReady = false, referenceReady = false;
float gravX = 0, gravY = 0, gravZ = 1;
float refX = 0, refY = 0, refZ = 1;

// 모든 임계값이 유효하게 설정됐는지 확인 (유효하지 않으면 낙상 판정을 실행하지 않음)
bool thresholdsReady() {
  return IMPACT_THRESHOLD_G > 0.0f &&
         TILT_THRESHOLD_DEG > 0.0f &&
         STRONG_TILT_THRESHOLD_DEG >= TILT_THRESHOLD_DEG &&
         STILL_GYRO_THRESHOLD_DPS > 0.0f &&
         STILL_ACCEL_TOL_G > 0.0f &&
         PRESSURE_THRESHOLD >= 0 && PRESSURE_THRESHOLD <= 1023;
}

bool pressureRawOverThreshold(int raw) {
  return PRESSED_WHEN_HIGH ? (raw >= PRESSURE_THRESHOLD)
                           : (raw <= PRESSURE_THRESHOLD);
}

// 압력센서의 현재 값으로 0.1초 연속 파지 신호와 마지막 유효 시점을 갱신
void updatePressurePattern(int pressureRaw, unsigned long now) {
  bool rawHigh = pressureRawOverThreshold(pressureRaw);

  if (rawHigh) {
    if (!rawHighTiming) {
      rawHighTiming = true;
      rawHighSince = now;
    }
    // 20ms 동안 순간적으로 튀는 값 하나만으로는 파지 신호로 인정하지 않음
    pressureHighStable = (now - rawHighSince >= PRESSURE_CONFIRM_MS);
    if (pressureHighStable) {
      everValidPeak = true;
      lastValidPeakMs = now;  // 300 이상 신호가 유효하게 계속 유지되면 마지막 파지 시각을 갱신
    }
  } else {
    rawHighTiming = false;
    pressureHighStable = false;
  }
}

// 최근 3초 안에 유효한 파지 신호가 한 번이라도 있었는지 판단
bool gripEvidenceWithin3s(unsigned long now) {
  return everValidPeak && (now - lastValidPeakMs <= PRESSURE_LOOKBACK_MS);
}

// 충격 전 기준 자세와 현재 중력 방향 벡터의 사이각(도)을 계산
float gravityVectorAngleDeg() {
  float dot = gravX * refX + gravY * refY + gravZ * refZ;
  float a = sqrtf(gravX * gravX + gravY * gravY + gravZ * gravZ);
  float b = sqrtf(refX * refX + refY * refY + refZ * refZ);
  if (a < 0.01f || b < 0.01f) return 0.0f;
  float cosine = dot / (a * b);
  if (cosine > 1.0f) cosine = 1.0f;
  if (cosine < -1.0f) cosine = -1.0f;
  return acosf(fmaxf(-1.0f, fminf(1.0f, cosine))) * 180.0f / PI;
}

// 정상 상태로 복귀하면서 정지 확인과 경보(LED·부저)를 초기화
void resetToNormal() {
  state = NORMAL;
  stillTiming = false;
  postImpactGripStarted = false;
  referenceReady = false; // 초기화 또는 판정 실패 후 오래된 기준 자세를 사용하지 않도록 해제
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
}

// 낙상 의심 판정 완료: LED와 능동형 부저를 동시에 켬
void enterAlarm() {
  state = ALARM;
  digitalWrite(LED_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
}

// CSV 출력에 사용할 상태 이름을 반환
const char *stateName() {
  switch (state) {
    case NORMAL: return "NORMAL";
    case WAIT_TILT: return "WAIT_TILT";
    case VERIFY: return "VERIFY";
    case ALARM: return "ALARM";
    default: return "UNKNOWN";
  }
}

// BLE 광고 시작: 휴대폰 웹페이지에서 "FallCane_IMU"로 검색된다.
void startAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

// 전원 인가 후 최초 1회: 입출력 핀과 IMU, 시리얼 통신 설정
void setup() {
  Serial.begin(115200);
  pinMode(PRESSURE_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  analogReadResolution(10);

  if (imu.begin() != 0) {
    Serial.println("IMU ERROR: check board and LSM6DS3 library");
    while (true) delay(100);
  }

  Bluefruit.begin();
  Bluefruit.setName("FallCane_IMU");
  Bluefruit.setTxPower(4);
  bleuart.begin();
  startAdvertising();

  if (!thresholdsReady()) {
    Serial.println("# WARNING: thresholds not configured; alarm disabled");
  }
  Serial.println("time_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,accel_mag_g,gyro_mag_dps,pressure_raw,grip,tilt_deg,state,pressure_stable_300,peak_age_ms");
}

// 20ms마다 센서 데이터를 읽고 상태머신을 순서대로 실행
void loop() {
  // 시리얼 모니터에서 r을 보내면 경보를 초기화하고 LED와 부저를 끔
  if (Serial.available() && Serial.read() == 'r') resetToNormal();
  // 휴대폰 경보 화면의 "확인" 버튼도 'r'을 보내 같은 방식으로 해제한다.
  while (bleuart.available()) {
    if (bleuart.read() == 'r') resetToNormal();
  }

  unsigned long now = millis();
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();
  float gx = imu.readFloatGyroX();
  float gy = imu.readFloatGyroY();
  float gz = imu.readFloatGyroZ();
  float accelMag = sqrtf(ax * ax + ay * ay + az * az);
  float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);
  int pressureRaw = analogRead(PRESSURE_PIN);

  updatePressurePattern(pressureRaw, now);
  bool recentGripEvidence = gripEvidenceWithin3s(now);

  // 가속도 저역통과 필터로 중력 방향을 근사함(자이로 센서 융합 각도는 아님)
  if (!gravityReady) {
    gravX = ax; gravY = ay; gravZ = az;
    gravityReady = true;
  } else {
    gravX += GRAVITY_ALPHA * (ax - gravX);
    gravY += GRAVITY_ALPHA * (ay - gravY);
    gravZ += GRAVITY_ALPHA * (az - gravZ);
  }

  if (thresholdsReady()) {
    if (state == NORMAL) {
      bool impact = (accelMag >= IMPACT_THRESHOLD_G);

      // 정상 상태에서는 충격 전 파지 이력이 있어야만 기준 자세를 사용하고
      // 충격 후 기울기 확인 상태로 진입함. 현재 약한 파지는 확정할 수 없음
      if (impact && recentGripEvidence && referenceReady) {
        state = WAIT_TILT;
        impactMs = now;
        stillTiming = false;
        postImpactGripStarted = false;
        // 충격 순간에도 300 이상 신호가 안정적으로 유지됐다면 충격 후 파지 시작으로 기록
        if (pressureHighStable) {
          postImpactGripStarted = true;
          postImpactGripSince = now;
        }
      } else if (recentGripEvidence) {
        if (!referenceReady) {
          refX = gravX; refY = gravY; refZ = gravZ;
          referenceReady = true;
        } else {
          refX += REFERENCE_ALPHA * (gravX - refX);
          refY += REFERENCE_ALPHA * (gravY - refY);
          refZ += REFERENCE_ALPHA * (gravZ - refZ);
        }
      } else {
        // 유효한 300 이상 신호가 3초 넘게 없으면 오래된 기준 자세를 삭제
        referenceReady = false;
      }
    } else if (state == WAIT_TILT || state == VERIFY) {
      // 300 이상 파지 신호가 3초 넘게 없으면 경로 B의 파지 지속 확인을 다시 시작
      if (postImpactGripStarted &&
          (!everValidPeak || now - lastValidPeakMs > PRESSURE_LOOKBACK_MS)) {
        postImpactGripStarted = false;
      }
      if (pressureHighStable && !postImpactGripStarted) {
        postImpactGripStarted = true;
        postImpactGripSince = now;
      }

      float tilt = gravityVectorAngleDeg();
      if (state == WAIT_TILT) {
        if (tilt >= TILT_THRESHOLD_DEG) {
          state = VERIFY;
          stillTiming = false;
        } else if (now - impactMs > TILT_WINDOW_MS) {
          resetToNormal();
        }
      }

      if (state == VERIFY) {
        // 정지 판정: 합성각속도가 낮고 합성가속도가 약 1g인지 확인
        bool stationary = (gyroMag <= STILL_GYRO_THRESHOLD_DPS) &&
                          (fabsf(accelMag - 1.0f) <= STILL_ACCEL_TOL_G);
        bool tiltedAndStill = stationary && (tilt >= TILT_THRESHOLD_DEG);
        if (tiltedAndStill) {
          if (!stillTiming) {
            stillTiming = true;
            stillSince = now;
          }
        } else {
          stillTiming = false;
        }
        bool still3s = stillTiming && (now - stillSince >= STILL_HOLD_MS);

        // 경로 A: 충격 후 3초 동안 유효한 300 이상 압력 신호가 없는 경우
        // 단, 약하게 잡았을 수도 있으므로 실제 비파지를 확정하는 조건은 아님
        bool noRecent300AfterImpact = (now - impactMs >= NO_GRIP_HOLD_MS) &&
          (!everValidPeak || lastValidPeakMs <= impactMs ||
            (now - lastValidPeakMs >= NO_GRIP_HOLD_MS));
        bool pathA = still3s && noRecent300AfterImpact;

        // 경로 B: 충격 후 3초 이상에 걸쳐 유효한 300 이상 신호가 확인되고
        // 최근 1.5초 내에도 유효한 신호가 있으며 자세 변화가 60도 이상일 때
        bool pathB = still3s && postImpactGripStarted &&
                     (now - postImpactGripSince >= GRIP_HOLD_MS) &&
                     everValidPeak && (lastValidPeakMs >= impactMs) &&
                     (now - lastValidPeakMs <= PRESSURE_B_RECENCY_MS) &&
                     (tilt >= STRONG_TILT_THRESHOLD_DEG);

        // 경로 A 또는 경로 B의 모든 조건이 충족되면 경보 상태로 전환
        if (pathA || pathB) {
          enterAlarm();
        } else if (now - impactMs > VERIFY_WINDOW_MS) {
          resetToNormal();
        }
      }
    }
  }

  // 휴대폰이 알림을 구독 중이면 매 샘플(약 50Hz)마다 상태를 전송한다.
  if (Bluefruit.connected() && bleuart.notifyEnabled()) {
    float tiltNow = referenceReady ? gravityVectorAngleDeg() : 0.0f;
    BlePacket p;
    p.time_ms  = now;
    p.ax_mg    = scaledToI16(ax, 1000.0f);
    p.ay_mg    = scaledToI16(ay, 1000.0f);
    p.az_mg    = scaledToI16(az, 1000.0f);
    p.gx_dps10 = scaledToI16(gx, 10.0f);
    p.gy_dps10 = scaledToI16(gy, 10.0f);
    p.gz_dps10 = scaledToI16(gz, 10.0f);
    p.pressure = (uint16_t)pressureRaw;
    p.status   = (uint8_t)state | (recentGripEvidence ? 0x10 : 0) | (thresholdsReady() ? 0x20 : 0);
    p.tilt_deg = (uint8_t)constrain((int)roundf(tiltNow), 0, 180);
    bleuart.write((uint8_t *)&p, sizeof(p));
  }

  // 기존 CSV 수신 프로그램과 호환되도록 앞의 13개 출력 열 순서를 유지
  // grip=1은 최근 3초 안에 유효한 300 이상 신호가 있었음을 뜻함
  if (now - lastCsvMs >= CSV_INTERVAL_MS) {
    lastCsvMs = now;
    Serial.print(now); Serial.print(',');
    Serial.print(ax, 3); Serial.print(',');
    Serial.print(ay, 3); Serial.print(',');
    Serial.print(az, 3); Serial.print(',');
    Serial.print(gx, 2); Serial.print(',');
    Serial.print(gy, 2); Serial.print(',');
    Serial.print(gz, 2); Serial.print(',');
    Serial.print(accelMag, 3); Serial.print(',');
    Serial.print(gyroMag, 2); Serial.print(',');
    Serial.print(pressureRaw); Serial.print(',');
    Serial.print(recentGripEvidence ? 1 : 0); Serial.print(',');
    Serial.print(referenceReady ? gravityVectorAngleDeg() : 0.0f, 1);
    Serial.print(','); Serial.print(stateName()); Serial.print(',');
    Serial.print(pressureHighStable ? 1 : 0); Serial.print(',');
    if (everValidPeak) Serial.println(now - lastValidPeakMs);
    else Serial.println(-1);
  }
}
