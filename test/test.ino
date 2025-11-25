#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
}

void loop() {
  byte error, address;
  int nDevices = 0;
  Serial.println("Scanning...");
  for(address = 1; address < 127; address++ ) {
    // Thu ket noi voi dia chi nay
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Tim thay thiet bi o dia chi: 0x");
      if (address<16) Serial.print("0");
      Serial.println(address,HEX);
      nDevices++;
    }
  }
  if (nDevices == 0) Serial.println("Khong tim thay thiet bi nao (Kiem tra Pin/Day noi!)\n");
  else Serial.println("done\n");
  
  delay(2000); // Quet lai sau moi 2 giay
}