#include <WiFi.h>

// --- SỬA TÊN WIFI VÀ MẬT KHẨU CỦA BẠN Ở ĐÂY ---
const char* ssid = "Quoc Bao 1";      // Ví dụ: "iPhone cua Tien"
const char* password = "quocbao2015";     // Ví dụ: "12345678"

void setup() {
  Serial.begin(115200);

  Serial.println();
  Serial.println("Dang ket noi WiFi...");

  WiFi.begin(ssid, password);

  // Đợi đến khi kết nối được
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Da ket noi thanh cong!");
  Serial.print("DIA CHI IP CUA MACH LA: ");
  Serial.println(WiFi.localIP()); // <--- Đây là cái bạn cần
}

void loop() {
  // Nếu đã kết nối thì in IP ra liên tục
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Mat ket noi WiFi!");
  }
  delay(2000); // Đợi 2 giây in lại lần nữa
}