#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(115200);
  while (!Serial); // Đợi mở Serial Monitor
  
  Serial.println("\n\n--- BAT DAU KIEM TRA CHIP PCA9685 ---");

  // 1. Kiểm tra kết nối I2C cơ bản
  // Mặc định ESP32 dùng SDA=21, SCL=22. 
  // Nếu mạch bạn khác, sửa dòng dưới thành: Wire.begin(SDA_PIN, SCL_PIN);
  Wire.begin(); 

  Serial.print("Dang tim chip tai dia chi 0x40... ");
  
  Wire.beginTransmission(0x40);
  byte error = Wire.endTransmission();

  if (error == 0) {
    Serial.println("OK! (Tim thay chip)");
    Serial.println(">> Trang thai: KET NOI TOT.");
    
    // 2. Thử khởi động Driver
    Serial.print("Dang khoi dong Driver... ");
    pwm.begin();
    pwm.setPWMFreq(50);
    Serial.println("OK!");
    Serial.println(">> Ket luan: Chip song, co the nap code Servo.");
    
  } else if (error == 2) {
    Serial.println("LOI! (Khong thay chip)");
    Serial.println(">> NGUYEN NHAN CO THE:");
    Serial.println("   1. CHUA BAT CONG TAC PIN (Quan trong nhat!).");
    Serial.println("   2. Pin yeu/het dien.");
    Serial.println("   3. Chip bi hong hoac long day.");
    
  } else {
    Serial.print("LOI KHAC (Ma loi: ");
    Serial.print(error);
    Serial.println(")");
    Serial.println(">> Kiem tra lai day ket noi SDA/SCL.");
  }
  
  Serial.println("---------------------------------------\n");
}

void loop() {
  // Quét lại liên tục mỗi 3 giây để bạn kịp bật công tắc
  delay(3000);
  setup(); 
}