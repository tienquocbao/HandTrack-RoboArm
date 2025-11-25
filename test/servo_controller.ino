#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// --- CẤU HÌNH WIFI ---
const char* ssid = "TEN_WIFI_CUA_BAN";
const char* password = "MAT_KHAU_WIFI";

// --- CẤU HÌNH UDP ---
WiFiUDP udp;
unsigned int localPort = 1234;
char packetBuffer[255];

// --- CẤU HÌNH SERVO ---
// Kiểm tra lại chân GPIO trên mạch VIA/Công suất của bạn
#define PIN_S1 13  // Base
#define PIN_S2 12  // Shoulder
#define PIN_S3 14  // Elbow
#define PIN_S4 27  // Wrist
#define PIN_SG 26  // Gripper

Servo s1, s2, s3, s4, sg;

void setup() {
  Serial.begin(115200);

  // Kết nối WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); // <--- Lấy IP này điền vào file Python

  // Khởi động UDP
  udp.begin(localPort);

  // Gắn Servo
  s1.attach(PIN_S1);
  s2.attach(PIN_S2);
  s3.attach(PIN_S3);
  s4.attach(PIN_S4);
  sg.attach(PIN_SG);
  
  // Về vị trí Home an toàn
  moveServos(90, 90, 90, 90, 0);
}

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    
    String data = String(packetBuffer);
    // Dữ liệu nhận: "1:90,2:45,3:120,4:90,G:0"
    
    int val1 = parseValue(data, "1:");
    int val2 = parseValue(data, "2:");
    int val3 = parseValue(data, "3:");
    int val4 = parseValue(data, "4:");
    int valG = parseValue(data, "G:");
    
    moveServos(val1, val2, val3, val4, valG);
  }
}

// Hàm tách giá trị từ chuỗi
int parseValue(String data, String key) {
  int index = data.indexOf(key);
  if (index == -1) return -1; // Không tìm thấy
  
  int start = index + key.length();
  int comma = data.indexOf(",", start);
  if (comma == -1) comma = data.length();
  
  return data.substring(start, comma).toInt();
}

void moveServos(int v1, int v2, int v3, int v4, int vg) {
  if(v1 != -1) s1.write(v1);
  if(v2 != -1) s2.write(v2);
  if(v3 != -1) s3.write(v3);
  if(v4 != -1) s4.write(v4);
  if(vg != -1) sg.write(vg);
}