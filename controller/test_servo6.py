# --- FILE: wifi_control.py ---
import socket

# ================= CẤU HÌNH =================
ROBOT_IP = "172.20.10.3"  # <--- THAY IP CỦA ESP32 VÀO ĐÂY (Xem trong Serial Monitor)
ROBOT_PORT = 1234
# ============================================

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"--- KET NOI WIFI TOI ROBOT {ROBOT_IP} ---")
print("Nhap goc (0-180) roi Enter. Nhap 'q' de thoat.")

try:
    while True:
        angle_input = input("Nhap goc servo: ")
        
        if angle_input.lower() == 'q':
            break
            
        # Kiểm tra xem có phải số không
        if angle_input.isdigit():
            angle = int(angle_input)
            if 0 <= angle <= 180:
                # Gửi số góc sang Robot
                sock.sendto(str(angle).encode(), (ROBOT_IP, ROBOT_PORT))
                print(f"--> Da gui: {angle}")
            else:
                print("Loi: Goc phai tu 0 den 180")
        else:
            print("Loi: Vui long nhap so!")

except KeyboardInterrupt:
    print("\nNgat ket noi.")
finally:
    sock.close()