import cv2
import mediapipe as mp
import pybullet as p
import pybullet_data
import time
import math
import socket

# ================= 1. CẤU HÌNH KẾT NỐI ESP32 =================
ESP_IP = "172.20.10.3"  # <--- ĐỔI IP CỦA BẠN VÀO ĐÂY
ESP_PORT = 1234
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Cấu hình Servo (Góc lệch để khớp với lắp ráp thực tế)
OFFSET_1 = 90
OFFSET_2 = 90
OFFSET_3 = 90
OFFSET_4 = 90

# Chiều quay (1 hoặc -1)
DIR_1 = 1
DIR_2 = -1
DIR_3 = -1
DIR_4 = 1

# ================= 2. SETUP MÔ PHỎNG PYBULLET =================
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

# Load robot
try:
    robotId = p.loadURDF("my_robot.urdf", [0, 0, 0], useFixedBase=True)
except:
    print("LOI: Khong tim thay file my_robot.urdf!")
    # Tạo robot giả để không crash nếu thiếu file
    robotId = p.loadURDF("r2d2.urdf", [0, 0, 1]) 

# Tìm khớp End Effector
end_effector_index = 3
num_joints = p.getNumJoints(robotId)
joint_ids = {}
for i in range(num_joints):
    info = p.getJointInfo(robotId, i)
    joint_name = info[1].decode('utf-8')
    joint_ids[joint_name] = i
    # Tự động tìm link cuối cùng
    if 'palm' in info[12].decode('utf-8') or 'link_4' in info[12].decode('utf-8'):
        end_effector_index = i

# ================= 3. SETUP MEDIAPIPE & BIẾN =================
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)

# Vị trí nghỉ (Home)
HOME_POS = [0.2, 0, 0.3] 
current_x, current_y, current_z = HOME_POS
gripper_state = 0 # 0: Mở, 90: Đóng (Gửi xuống ESP32)
gripper_sim_val = 0.0 # Giá trị cho PyBullet

smooth_factor = 0.15 # Độ mượt (0.1 -> 1.0)
last_send_time = 0
SEND_INTERVAL = 0.05 # Gửi dữ liệu mỗi 50ms

print("--- SYSTEM READY: Press 'q' to exit ---")

try:
    while True:
        success, img = cap.read()
        if not success: break

        img = cv2.flip(img, 1)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        results = hands.process(img_rgb)
        
        target_x, target_y, target_z = current_x, current_y, current_z
        is_reset = False
        is_tracking = False

        if results.multi_hand_landmarks:
            is_tracking = True
            for hand_lms in results.multi_hand_landmarks:
                mp_draw.draw_landmarks(img, hand_lms, mp_hands.HAND_CONNECTIONS)
                
                # --- SỬA LỖI 1: NHẬN DIỆN RESET (DÙNG KHOẢNG CÁCH) ---
                thumb_tip = hand_lms.landmark[4]
                pinky_tip = hand_lms.landmark[20]
                wrist = hand_lms.landmark[0]
                middle_mcp = hand_lms.landmark[9]

                # Đo độ rộng xòe tay (Cái -> Út)
                spread_dist = math.hypot(thumb_tip.x - pinky_tip.x, thumb_tip.y - pinky_tip.y)
                # Đo kích thước tay chuẩn (Cổ tay -> Đốt giữa)
                ref_size = math.hypot(wrist.x - middle_mcp.x, wrist.y - middle_mcp.y)

                # Nếu xòe rộng gấp 2.3 lần kích thước tay -> RESET
                if spread_dist > ref_size * 2.3:
                    is_reset = True
                    cv2.putText(img, "!!! RESET MODE !!!", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                
                if is_reset:
                    # Về vị trí Home
                    target_x, target_y, target_z = HOME_POS
                    gripper_state = 0 # Mở kẹp
                    gripper_sim_val = 0.0
                else:
                    # --- TÍNH TOẠ ĐỘ ĐIỀU KHIỂN ---
                    lm = hand_lms.landmark[9]
                    raw_y = (lm.x - 0.5) * 2.0      # Trái/Phải
                    raw_z = 0.65 - (lm.y * 0.7)     # Lên/Xuống
                    raw_x = 0.2 + (0.5 - lm.y) * 0.2 # Xa/Gần

                    # Làm mượt toạ độ
                    current_x += (raw_x - current_x) * smooth_factor
                    current_y += (raw_y - current_y) * smooth_factor
                    current_z += (raw_z - current_z) * smooth_factor
                    
                    target_x, target_y, target_z = current_x, current_y, current_z

                    # --- SỬA LỖI 2: KẸP (Tăng ngưỡng nhận diện) ---
                    thumb = hand_lms.landmark[4]
                    index = hand_lms.landmark[8]
                    dist_fingers = math.hypot(index.x - thumb.x, index.y - thumb.y)
                    
                    # Ngưỡng 0.08 dễ kẹp hơn 0.05
                    if dist_fingers < 0.08: 
                        gripper_state = 90    # Đóng (Gửi ESP)
                        gripper_sim_val = 0.8 # Đóng (PyBullet)
                        cv2.putText(img, "GRIP: CLOSE", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    else:
                        gripper_state = 0     # Mở
                        gripper_sim_val = 0.0

        # --- TÍNH TOÁN IK (PyBullet) ---
        joint_poses = p.calculateInverseKinematics(
            robotId, end_effector_index, [target_x, target_y, target_z],
            lowerLimits=[-3.14]*4, upperLimits=[3.14]*4,
            jointRanges=[6.28]*4, restPoses=[0, -0.5, 2.0, 0],
            maxNumIterations=50
        )

        # Cập nhật hình ảnh Robot ảo
        for i in range(4):
            p.setJointMotorControl2(robotId, joint_ids[f'joint_{i+1}'], p.POSITION_CONTROL, joint_poses[i])
        
        # Cập nhật kẹp ảo (nếu có khớp kẹp trong URDF)
        if 'finger_left_joint' in joint_ids:
             p.setJointMotorControl2(robotId, joint_ids['finger_left_joint'], p.POSITION_CONTROL, gripper_sim_val)
             p.setJointMotorControl2(robotId, joint_ids['finger_right_joint'], p.POSITION_CONTROL, -gripper_sim_val)

        p.stepSimulation()

        # --- GỬI DỮ LIỆU XUỐNG MẠCH (ESP32) ---
        if time.time() - last_send_time > SEND_INTERVAL:
            # 1. Chuyển từ Radian (PyBullet) sang Độ
            d1 = math.degrees(joint_poses[0] * DIR_1)
            d2 = math.degrees(joint_poses[1] * DIR_2)
            d3 = math.degrees(joint_poses[2] * DIR_3)
            d4 = math.degrees(joint_poses[3] * DIR_4)

            # 2. Cộng Offset và giới hạn 0-180
            s1 = max(0, min(180, int(d1 + OFFSET_1)))
            s2 = max(0, min(180, int(d2 + OFFSET_2)))
            s3 = max(0, min(180, int(d3 + OFFSET_3)))
            s4 = max(0, min(180, int(d4 + OFFSET_4)))

            # 3. Tạo gói tin
            # G: Gripper (0 hoặc 90)
            msg = f"1:{s1},2:{s2},3:{s3},4:{s4},G:{gripper_state}"
            
            # 4. Gửi UDP
            sock.sendto(msg.encode(), (ESP_IP, ESP_PORT))
            
            # Debug lên màn hình
            status_text = f"S1:{s1} S2:{s2} S3:{s3} G:{gripper_state}"
            cv2.putText(img, status_text, (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            last_send_time = time.time()

        # Hiển thị
        cv2.imshow("Robot Controller (Final)", img)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

except KeyboardInterrupt:
    print("Stopping...")
finally:
    sock.close()
    cap.release()
    cv2.destroyAllWindows()
    p.disconnect()