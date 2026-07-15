// 순수 GPIO 출력 진단 스케치 — 시리얼 프로토콜/워치독/canGo 게이트 등
// car_controller.ino의 로직을 전부 건너뛰고, 각 모터 채널의 핀을 직접
// digitalWrite/analogWrite로 순서대로 돌려본다.
//
// car_controller.ino를 업로드했는데도 모터가 안 움직일 때, "펌웨어 로직/시리얼
// 통신 문제"와 "핀/배선/전원 문제"를 구분하기 위한 임시 진단용이다.
// 실차 운영에는 쓰지 않음 — 확인 후 반드시 car_controller.ino로 재업로드할 것.
//
// 업로드: ./tools/upload_firmware.sh <포트> arduino/pin_test
// 확인: 시리얼 모니터(9600bps)로 어느 채널을 돌리는 중인지 출력되고,
//       LEFT -> RIGHT -> STEER 순서로 1초씩 해당 모터가 도는지 눈으로 확인.

const int LEFT_PWM = 4;
const int LEFT_IN1 = 26;
const int LEFT_IN2 = 27;

const int RIGHT_PWM = 3;
// 우측 모터가 좌측과 마주보게 장착돼 있어 실제 하드웨어 보정이 필요함(2026-07
// 확인, car_controller.ino와 동일) — 이건 명령 해석이 아니라 진짜 배선/마운트
// 사실이라 진단 스케치인 이 파일에도 반영함. LEFT/STEER는 원래 값 그대로 유지.
const int RIGHT_IN1 = 25;
const int RIGHT_IN2 = 24;

const int STEER_PWM = 2;
const int STEER_IN1 = 22;
const int STEER_IN2 = 23;

const int SPIN_MS = 1000;
const int PAUSE_MS = 500;
const int TEST_PWM = 200;

void setup() {
  Serial.begin(9600);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(STEER_PWM, OUTPUT);
  pinMode(STEER_IN1, OUTPUT);
  pinMode(STEER_IN2, OUTPUT);
  Serial.println("pin_test 시작 — LEFT -> RIGHT -> STEER 순서로 1초씩 반복");
}

void spin(const char *name, int pwmPin, int in1, int in2) {
  Serial.println(name);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(pwmPin, TEST_PWM);
  delay(SPIN_MS);
  analogWrite(pwmPin, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(PAUSE_MS);
}

void loop() {
  spin("LEFT", LEFT_PWM, LEFT_IN1, LEFT_IN2);
  spin("RIGHT", RIGHT_PWM, RIGHT_IN1, RIGHT_IN2);
  spin("STEER", STEER_PWM, STEER_IN1, STEER_IN2);
}
