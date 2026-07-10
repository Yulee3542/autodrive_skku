// autodrive_skku 차량 펌웨어 (실차 검증본 run_test_fixed.ino 기반)
//
// 시리얼 프로토콜 (9600bps) — README '시리얼 프로토콜' 절과 동일하게 유지할 것
//   PC -> Arduino:
//     G / 1        주행 허용 (V 미수신 상태면 기본 속도로 전진)
//     2            후진 (레거시 — 수동 테스트용)
//     S / 3        정지 (주행 게이트 닫힘)
//     V<int>\n     부호 있는 속도 -255..255 (음수 = 후진). 최초 수신 시
//                  시리얼 속도 제어로 전환되고 워치독이 활성화된다
//     L / R        스티어링 모터 한 펄스 (120ms) — 바퀴 각도가 그만큼 이동
//     F            스티어링 모터 즉시 정지
//   Arduino -> PC:
//     0 / 1 / 2    상태 (정지/전진/후진)

const int LEFT_PWM = 4;
const int LEFT_IN1 = 26;
const int LEFT_IN2 = 27;

const int RIGHT_PWM = 3;
const int RIGHT_IN1 = 24;
const int RIGHT_IN2 = 25;

const int STEER_PWM = 2;
const int STEER_IN1 = 22;
const int STEER_IN2 = 23;

const int DEFAULT_SPEED = 100;
const int STEER_SPEED = 160;
const unsigned long STEER_PULSE_MS = 120;
const unsigned long WATCHDOG_MS = 500;   // V 수신 후 시리얼 두절 시 정지
const unsigned long STATUS_PRINT_MS = 100;

bool canGo = false;
int commandedSpeed = 0;      // 부호 있는 속도. V 명령 또는 레거시 G/2로 설정
bool useSerialSpeed = false; // 첫 V 수신 후 true → 워치독 활성화

unsigned long steerStartTime = 0;
unsigned long lastSerialTime = 0;
int lastState = -1;
unsigned long lastStatusPrintTime = 0;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(20);  // parseInt가 오래 블록되지 않게

  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(STEER_PWM, OUTPUT);
  pinMode(STEER_IN1, OUTPUT);
  pinMode(STEER_IN2, OUTPUT);

  stopCar();
  steerStop();
  printState(0);
}

void loop() {
  readSerialCommand();

  // 조향 펄스 자동 종료
  if (steerStartTime > 0 && millis() - steerStartTime >= STEER_PULSE_MS) {
    steerStop();
  }

  bool watchdogTripped = useSerialSpeed && (millis() - lastSerialTime > WATCHDOG_MS);

  int speed = (!canGo || watchdogTripped) ? 0 : commandedSpeed;

  setDrive(speed);
  printState(speed > 0 ? 1 : (speed < 0 ? 2 : 0));
}

void readSerialCommand() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    lastSerialTime = millis();

    if (cmd == 'G' || cmd == '1') {
      canGo = true;
      if (!useSerialSpeed) {
        commandedSpeed = DEFAULT_SPEED;
      }
    }
    else if (cmd == '2') {
      canGo = true;
      if (!useSerialSpeed) {
        commandedSpeed = -DEFAULT_SPEED;
      }
    }
    else if (cmd == 'S' || cmd == '3') {
      canGo = false;
    }
    else if (cmd == 'V') {
      long v = Serial.parseInt();
      commandedSpeed = constrain(v, -255, 255);
      useSerialSpeed = true;
    }
    else if (cmd == 'F') {
      steerStop();
    }
    else if (cmd == 'L') {
      steerLeft();
    }
    else if (cmd == 'R') {
      steerRight();
    }
  }
}

void setDrive(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(LEFT_IN1, HIGH);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(RIGHT_IN1, HIGH);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(LEFT_PWM, speed);
    analogWrite(RIGHT_PWM, speed);
  }
  else if (speed < 0) {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, HIGH);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, HIGH);
    analogWrite(LEFT_PWM, -speed);
    analogWrite(RIGHT_PWM, -speed);
  }
  else {
    digitalWrite(LEFT_IN1, LOW);
    digitalWrite(LEFT_IN2, LOW);
    digitalWrite(RIGHT_IN1, LOW);
    digitalWrite(RIGHT_IN2, LOW);
    analogWrite(LEFT_PWM, 0);
    analogWrite(RIGHT_PWM, 0);
  }
}

void stopCar() {
  setDrive(0);
}

void steerLeft() {
  digitalWrite(STEER_IN1, LOW);
  digitalWrite(STEER_IN2, HIGH);
  analogWrite(STEER_PWM, STEER_SPEED);
  steerStartTime = millis();
}

void steerRight() {
  digitalWrite(STEER_IN1, HIGH);
  digitalWrite(STEER_IN2, LOW);
  analogWrite(STEER_PWM, STEER_SPEED);
  steerStartTime = millis();
}

void steerStop() {
  digitalWrite(STEER_IN1, LOW);
  digitalWrite(STEER_IN2, LOW);
  analogWrite(STEER_PWM, 0);
  steerStartTime = 0;
}

void printState(int state) {
  unsigned long now = millis();

  if (state != lastState || now - lastStatusPrintTime >= STATUS_PRINT_MS) {
    Serial.println(state);
    lastState = state;
    lastStatusPrintTime = now;
  }
}
