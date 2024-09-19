#include <Arduino.h>
#include <ESP32Servo.h>
#include <Motor.h>
#include <PS4Controller.h>

/* JOYSTICK */
const int DEAD_ZONE = 30;

/*pwm dir channel*/
Motor rightMotor(26, 21, 5);
Motor leftMotor(27, 22, 6);
Motor windingMotor(13, 23, 7);

/* SERVO */
Servo launchingServo;
int launchingDegree;
const int LAUNCHING_SERVO_PIN = 4;

uint32_t previous_millis = 0;

const uint8_t LAUNCH_LIMIT_PIN = 32;

const int DEBOUNCE_DELAY = 50;

bool is_auto_mode = false; // auto mode

const uint16_t delay_time = 100; // 

bool share_pressed = false;
uint32_t share_debounce_time = 0;

void setup() {
  Serial.begin(115200);

  uint8_t bt_mac[6];
  esp_read_mac(bt_mac, ESP_MAC_BT);
  Serial.printf("Bluetooth Mac Address => %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                bt_mac[0], bt_mac[1], bt_mac[2], bt_mac[3], bt_mac[4],
                bt_mac[5]);

  PS4.begin("48:E7:29:A3:C5:0E"); 
  Serial.printf("ready.\r\n");

  launchingServo.attach(LAUNCHING_SERVO_PIN, 500, 2500);
  launchingDegree = 116;
  launchingServo.write(launchingDegree);
  pinMode(LAUNCH_LIMIT_PIN, INPUT_PULLDOWN);
}

void loop() {
  if (!PS4.isConnected()) {
    Serial.printf("PS4 controller disconnected.\r\n");
    rightMotor.run(0,0);
    leftMotor.run(0,0);
    windingMotor.run(0,0);
    return;
  }

  /* MANUALLY MOVE FORWARD, BACKWARD AND ROTATE */

  if (DEAD_ZONE <= abs(PS4.RStickY())) {
    rightMotor.run(abs(PS4.RStickY()),
                   (PS4.RStickY() > 0 ? 1 : 0)); // 右モーター
  }else{
    rightMotor.run(0,0);
  }
  if (DEAD_ZONE <= abs(PS4.LStickY())) {
    leftMotor.run(abs(PS4.LStickY()),
                  (PS4.LStickY() > 0 ? 1 : 0)); // 左モーター
  }else{
    leftMotor.run(0,0);
  }

  if (PS4.Share()) { // shareボタンを押したとき
    if (!share_pressed &&
        (millis() - share_debounce_time >
         DEBOUNCE_DELAY)) { // share_pressがfalseかつ前回ボタンを押してから50ms以上経過
      is_auto_mode = !is_auto_mode;
      share_debounce_time = millis();
    }
    share_pressed = true;
  } else {
    share_pressed = false;
  }

  if (is_auto_mode) {
    uint32_t currentMillis = millis();

    launchingDegree = 60;
    launchingServo.write(launchingDegree);

    // 100ms後にモーターを回す
    if (currentMillis >= delay_time) {
      windingMotor.run(127, 0); // モーターを回し続ける
    }

    // リミットスイッチが押されたら停止
    bool is_on_limit_switch = digitalRead(LAUNCH_LIMIT_PIN);
    if (is_on_limit_switch) {
      is_auto_mode = false;
      windingMotor.run(0, 0); // モーター停止
    }
  }

  if (PS4.R2Value() > 15) {
    windingMotor.run(PS4.R2Value() / 2, 0); // 正転
  } else if (PS4.L2Value() > 15) {
    windingMotor.run(PS4.L2Value() / 2, 1); // 逆転
  } else if (!is_auto_mode) {
    windingMotor.run(0, 0);
  }

  if (PS4.Right() && launchingDegree < 115) { // セット方向
    uint32_t current_millis = millis();
    if (current_millis - previous_millis >= delay_time) {
        previous_millis = current_millis; 
        launchingDegree += 5;
        launchingServo.write(launchingDegree);
    }
  }
  if (PS4.Left() && launchingDegree > 5) { // 発射方向
    uint32_t current_millis = millis();
    if (current_millis - previous_millis >= delay_time) {
        previous_millis = current_millis; 
        launchingDegree -= 5;
        launchingServo.write(launchingDegree);
    }
  }

  if (PS4.PSButton()) {
    ESP.restart();
  }
}
