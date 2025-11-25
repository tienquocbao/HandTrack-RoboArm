#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> // Cần cài thư viện này

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Theo file header của bạn: Kênh 2, 3, 4, 5
int servo_channels[] = {2, 3, 4, 5}; 

void setup() {
  Serial.begin(115200);
  Serial.println("--- TEST DRIVER PCA9685 ---");

  pwm.begin();
  pwm.setPWMFreq(50);
}

void loop() {
  Serial.println("Dang quay tat ca Servo...");
  
  // Duyệt qua các kênh 2, 3, 4, 5
  for (int i = 0; i < 4; i++) {
    int channel = servo_channels[i];
    Serial.print("Kenh so: "); Serial.println(channel);

    // Quay 0 độ
    pwm.setPWM(channel, 0, 180); // 180 là xung MIN trong file bạn gửi
    delay(500);
    
    // Quay 180 độ
    pwm.setPWM(channel, 0, 375); // 375 là xung MAX trong file bạn gửi
    delay(500);
  }
}