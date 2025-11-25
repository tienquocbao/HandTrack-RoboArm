// --- FILE: wifi_servo_test.ino ---
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// 1. CẤU HÌNH WIFI (Sửa lại cho đúng)
const char* ssid = "ChoCong";
const char* password = "chothanhduy";

// 2. CẤU HÌNH SERVO
#define CH_GRIPPER 6      // Chân cắm Servo Kẹp (Kiểm tra kỹ số 6 hay 5)
#define PULSE_MIN 150     // Xung min (Mở rộng một chút để test)
#define PULSE_MAX 500     // Xung max

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
WiFiUDP udp;
unsigned int localPort = 1234;
char packetBuffer[255];

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);

  // Khởi động ở góc an toàn (90 độ)
  setServoAngle(90);

  // Kết nối Wifi
  Serial.print("Dang ket noi Wifi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nDa ket noi Wifi!");
  Serial.print("IP Cua Robot: ");
  Serial.println(WiFi.localIP()); // <--- GHI LẠI SỐ NÀY
  
  udp.begin(localPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Đọc dữ liệu gửi đến
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    String data = String(packetBuffer);
    int angle = data.toInt(); // Chuyển chuỗi thành số (VD: "90" -> 90)

    if (angle >= 0 && angle <= 180) {
       Serial.print("Nhan lenh Wifi: ");
       Serial.println(angle);
       setServoAngle(angle);
    }
  }
}

void setServoAngle(int degree) {
  int pulse = map(degree, 0, 180, PULSE_MIN, PULSE_MAX);
  pwm.setPWM(CH_GRIPPER, 0, pulse);
}