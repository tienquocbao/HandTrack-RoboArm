#include <ESP32Servo.h>

Servo myServo;

// Danh sách các chân có khả năng được dùng trên mạch ESP32 (Đã loại bỏ 6-11)
int possiblePins[] = {2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
int numPins = 19; 

void setup() {
  Serial.begin(115200);
  Serial.println("--- BAT DAU QUET CHAN TREN MACH CONG SUAT ---");
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
}

void loop() {
  for (int i = 0; i < numPins; i++) {
    int pin = possiblePins[i];
    
    Serial.print("Dang thu kich hoat GPIO so: ");
    Serial.println(pin);
    
    // Thử kết nối servo vào chân này
    myServo.attach(pin);
    
    // Lệnh cho servo lắc
    myServo.write(45);
    delay(200);
    myServo.write(90);
    delay(200);
    
    myServo.detach(); // Gỡ ra để thử chân kế tiếp
    delay(100);
  }
  Serial.println("\n--- HET 1 VONG - LAP LAI ---\n");
  delay(1000);
}