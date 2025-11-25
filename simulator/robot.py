import cv2
import mediapipe as mp
import pybullet as p
import pybullet_data
import time
import math

# --- 1. SETUP PYBULLET ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

# Load robot
robotId = p.loadURDF("my_robot.urdf", [0, 0, 0], useFixedBase=True)

# --- 2. TÌM KHỚP END EFFECTOR ---
end_effector_index = 3 
num_joints = p.getNumJoints(robotId)
joint_ids = {}
for i in range(num_joints):
    info = p.getJointInfo(robotId, i)
    joint_name = info[1].decode('utf-8')
    link_name = info[12].decode('utf-8')
    joint_ids[joint_name] = i
    if 'palm' in link_name or 'link_4' in link_name:
        end_effector_index = i

# --- 3. HÀM TÍNH GÓC GIỮA CÁC NGÓN TAY ---
def calculate_angle(p1, p2, ref_p):
    """
    Tính góc (độ) giữa vector (ref_p -> p1) và (ref_p -> p2)
    """
    # Vector 1
    v1_x = p1.x - ref_p.x
    v1_y = p1.y - ref_p.y
    # Vector 2
    v2_x = p2.x - ref_p.x
    v2_y = p2.y - ref_p.y
    
    # Tích vô hướng (Dot product)
    dot_product = v1_x * v2_x + v1_y * v2_y
    
    # Độ dài vector (Magnitude)
    mag1 = math.sqrt(v1_x**2 + v1_y**2)
    mag2 = math.sqrt(v2_x**2 + v2_y**2)
    
    if mag1 * mag2 == 0: return 0
    
    # Tính cos và kẹp giá trị trong khoảng [-1, 1] để tránh lỗi acos
    cos_angle = dot_product / (mag1 * mag2)
    cos_angle = max(-1.0, min(1.0, cos_angle))
    
    angle_rad = math.acos(cos_angle)
    return math.degrees(angle_rad)

# --- 4. SETUP MEDIAPIPE & VARS ---
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0) 

smooth_factor = 0.08
HOME_POS = [0.2, 0, 0.3] # Vị trí ban đầu (nghỉ)

# Khởi tạo vị trí hiện tại bằng vị trí Home
current_x, current_y, current_z = HOME_POS
gripper_val = 0.0

while True:
    success, img = cap.read()
    if not success: break

    img = cv2.flip(img, 1)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)
    
    # Mặc định là giữ nguyên vị trí cũ
    raw_x, raw_y, raw_z = current_x, current_y, current_z
    
    is_reset_gesture = False # Cờ đánh dấu có đang xòe tay không

    if results.multi_hand_landmarks:
        for hand_lms in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(img, hand_lms, mp_hands.HAND_CONNECTIONS)
            
            # --- KIỂM TRA ĐIỀU KIỆN XÒE TAY (GÓC > 10 ĐỘ) ---
            # Các điểm đầu ngón tay: 4(Cái), 8(Trỏ), 12(Giữa), 16(Nhẫn), 20(Út)
            # Điểm gốc: 0 (Cổ tay)
            wrist = hand_lms.landmark[0]
            tips = [4, 8, 12, 16, 20]
            
            angles = []
            # Tính góc giữa các cặp ngón liền kề: (Cái-Trỏ), (Trỏ-Giữa)...
            for i in range(4):
                p1 = hand_lms.landmark[tips[i]]
                p2 = hand_lms.landmark[tips[i+1]]
                angle = calculate_angle(p1, p2, wrist)
                angles.append(angle)

            # Kiểm tra: Nếu TẤT CẢ các khe ngón tay đều > 10 độ
            if all(a > 10 for a in angles):
                is_reset_gesture = True
                cv2.putText(img, "RESET DETECTED (SPREAD)", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # --- XỬ LÝ LOGIC ---
            if is_reset_gesture:
                # Nếu xòe tay -> Về vị trí ban đầu
                raw_x, raw_y, raw_z = HOME_POS
                gripper_val = 0.0 # Mở kẹp luôn cho an toàn
            else:
                # Nếu không xòe -> Điều khiển bình thường
                lm = hand_lms.landmark[9] 
                raw_y = (lm.x - 0.5) * 2.0  
                raw_z = 0.65 - (lm.y * 0.7)
                raw_x = 0.2 + (0.5 - lm.y) * 0.2

                # Xử lý kẹp (Chỉ khi không reset)
                thumb = hand_lms.landmark[4]
                index = hand_lms.landmark[8]
                dist = math.hypot(index.x - thumb.x, index.y - thumb.y)
                if dist < 0.05: gripper_val = 0.6 
                else: gripper_val = 0.0 

            # Hiển thị thông số góc để debug
            debug_str = f"Angles: {[int(a) for a in angles]}"
            cv2.putText(img, debug_str, (10, 450), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    # --- LÀM MƯỢT CHUYỂN ĐỘNG (Áp dụng cho cả lúc Reset) ---
    # Nhờ có smoothing, robot sẽ từ từ trôi về vị trí ban đầu chứ không giật cục
    current_x += (raw_x - current_x) * smooth_factor
    current_y += (raw_y - current_y) * smooth_factor
    current_z += (raw_z - current_z) * smooth_factor
    
    final_target_pos = [current_x, current_y, current_z]
    
    # --- HIỂN THỊ TRẠNG THÁI ---
    cv2.putText(img, f"Target: {current_x:.2f}, {current_y:.2f}, {current_z:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # --- TÍNH TOÁN IK ---
    joint_poses = p.calculateInverseKinematics(
        robotId, 
        end_effector_index, 
        final_target_pos,
        lowerLimits=[-3.14, -3.0, -2.8, -3.14], 
        upperLimits=[3.14, 3.0, 2.8, 3.14],
        jointRanges=[6.28, 6.0, 5.6, 6.28],
        restPoses=[0, -0.5, 2.0, 0], 
        maxNumIterations=100
    )

    # --- GỬI LỆNH ---
    for i in range(4):
        p.setJointMotorControl2(
            robotId, 
            joint_ids[f'joint_{i+1}'], 
            p.POSITION_CONTROL, 
            joint_poses[i], 
            force=200,
            maxVelocity=2.5 
        )
        
    p.setJointMotorControl2(robotId, joint_ids['joint_4'], p.POSITION_CONTROL, 0, force=100)
    p.setJointMotorControl2(robotId, joint_ids['finger_left_joint'], p.POSITION_CONTROL, gripper_val, force=100)
    p.setJointMotorControl2(robotId, joint_ids['finger_right_joint'], p.POSITION_CONTROL, -gripper_val, force=100)

    p.stepSimulation()
    cv2.imshow("Robot Control - Gesture Reset", img)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()
p.disconnect()