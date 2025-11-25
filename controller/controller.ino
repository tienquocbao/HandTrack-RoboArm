#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// --- WIFI ---
const char* ssid = "ChoCong"; 
const char* password = "chothanhduy";

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define CH_S1 2  
#define CH_S2 3  
#define CH_S3 4  
#define CH_S4 5  
#define CH_GRIPPER 6 

#define PULSE_MIN 180 
#define PULSE_MAX 375 

// --- SỬA 1: TĂNG TỐC ĐỘ PHẢN HỒI ---
// Tăng từ 0.15 lên 0.6 để hết bị chậm/lag
float K_SMOOTH = 0.6; 

float currentAngle[7];
float targetAngle[7];

WiFiUDP udp;
unsigned int localPort = 1234;
char packetBuffer[255];

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  
  for(int i=0; i<7; i++) {
    currentAngle[i] = 90.0;
    targetAngle[i] = 90.0;
  }

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  
  udp.begin(localPort);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    String data = String(packetBuffer);
    
    int ang1 = parseValue(data, "1:");
    int ang2 = parseValue(data, "2:");
    int ang3 = parseValue(data, "3:");
    int ang4 = parseValue(data, "4:");
    int angG = parseValue(data, "G:");

    if(ang1 != -1) targetAngle[CH_S1] = ang1;
    if(ang2 != -1) targetAngle[CH_S2] = ang2;
    if(ang3 != -1) targetAngle[CH_S3] = ang3;
    if(ang4 != -1) targetAngle[CH_S4] = ang4;
    if(angG != -1) targetAngle[CH_GRIPPER] = angG;
  }

  updateServoSmooth(CH_S1);
  updateServoSmooth(CH_S2);
  updateServoSmooth(CH_S3);
  // updateServoSmooth(CH_S4); // Đã tắt servo 360
  updateServoSmooth(CH_GRIPPER);
  
  delay(10); 
}

void updateServoSmooth(int channel) {
  float error = targetAngle[channel] - currentAngle[channel];
  // Giảm ngưỡng sai số để robot nhạy hơn
  if (abs(error) < 0.8) {
    currentAngle[channel] = targetAngle[channel];
  } else {
    currentAngle[channel] += error * K_SMOOTH;
  }
  setServoAngle(channel, (int)currentAngle[channel]);
}

void setServoAngle(int channel, int degree) {
  // --- SỬA 2: BỎ ĐẢO CHIỀU Ở ĐÂY (CHỈNH BÊN PYTHON CHO DỄ) ---
  // Code cũ có đoạn 180 - degree, mình đã xóa để tín hiệu đi thẳng trực tiếp.
  
  if (degree < 0) degree = 0;
  if (degree > 180) degree = 180;
  int pulse = map(degree, 0, 180, PULSE_MIN, PULSE_MAX);
  pwm.setPWM(channel, 0, pulse);
}

int parseValue(String data, String key) {
  int index = data.indexOf(key);
  if (index == -1) return -1;
  int start = index + key.length();
  int comma = data.indexOf(",", start);
  if (comma == -1) comma = data.length();
  return data.substring(start, comma).toInt();
}