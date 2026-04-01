from controller import Supervisor
import math

# --- 1. CẤU HÌNH BẢN ĐỒ & THUẬT TOÁN BFS ---
GRID_SIZE = 8
TILE_SIZE = 0.25  # Giả sử mỗi ô vuông trên bàn cờ là 0.2m (thay đổi cho khớp với World của bạn)
OFFSET = 0.1     # Đưa robot vào giữa ô 
ORIGIN_X = -0.864852
ORIGIN_Y = -0.876037

grid = [
    [0, 0, 0, 0, 1, 0, 0, 0],
    [0, 1, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 1, 0, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1, 0],
    [0, 0, 0, 1, 0 , 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0]
]

def dfs(start, goal):
    if start == goal: return [start]
    
    stack = [start]
    came_from = {start: None}
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    
    found = False
    while stack:
        curr = stack.pop() 
        if curr == goal:
            found = True
            break
        for dx, dy in directions:
            nx, ny = curr[0] + dx, curr[1] + dy
            if 0 <= nx < 8 and 0 <= ny < 8 and grid[nx][ny] == 0:
                if (nx, ny) not in came_from:
                    stack.append((nx, ny))
                    came_from[(nx, ny)] = curr
    
    if not found: return None
    
    path = []
    curr = goal
    while curr:
        path.append(curr)
        curr = came_from.get(curr)
    return path[::-1]
    
def world_to_grid(x, z):
    r = round((x - OFFSET) / TILE_SIZE)
    c = round((z - OFFSET) / TILE_SIZE)
    return (max(0, min(7, r)), max(0, min(7, c)))

def grid_to_world(r, c):
    return ORIGIN_X + c * TILE_SIZE, ORIGIN_Y + r * TILE_SIZE

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
MAX_SPEED = 6.28

# Lấy node của chính robot để truy cập translation/rotation
robot_node = robot.getSelf()
trans_field = robot_node.getField("translation")
rot_field = robot_node.getField("rotation")

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# --- 4. KHỞI TẠO ĐƯỜNG ĐI ---
# Đọc vị trí thực tế ban đầu để xác định ô Start
init_pos = trans_field.getSFVec3f()
print(init_pos)
init_x = init_pos[0]
init_y = init_pos[1] 

# Quy đổi X, Y thực tế sang Cột (c) và Hàng (r)
# X map với Cột (c), Y map với Hàng (r)
start_c = round((init_x - ORIGIN_X) / TILE_SIZE)
start_r = round((init_y - ORIGIN_Y) / TILE_SIZE)

# Ép giới hạn 0-7 để không bị văng ra ngoài mảng
start_r = max(0, min(7, start_r))
start_c = max(0, min(7, start_c))
start_node = (start_r, start_c)
print(start_node)
goal_node = (7, 7)

full_path = dfs(start_node, goal_node)
print(full_path)
path_index = 1 # Bắt đầu từ ô tiếp theo trong danh sách

# --- 3. VÒNG LẶP ĐIỀU KHIỂN DỨT KHOÁT ---
state = "TURN" # Bắt đầu bằng việc xoay người đúng hướng

# --- 4. VÒNG LẶP ĐIỀU KHIỂN THEO TRẠNG THÁI ---
while robot.step(timestep) != -1:
    if path_index >= len(full_path):
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        print("HOÀN THÀNH LỘ TRÌNH!")
        break

    # 1. Lấy thông tin hiện tại
    curr_pos = trans_field.getSFVec3f()
    cx = curr_pos[0]
    cy = curr_pos[1]
    
    rot = rot_field.getSFRotation()
    current_angle = rot[3] if rot[2] > 0 else -rot[3]

    # 2. Phân tích chặng đường (Ô trước -> Ô mục tiêu)
    target_r, target_c = full_path[path_index]
    prev_r, prev_c = full_path[path_index - 1]
    tx, ty = grid_to_world(target_r, target_c)

    # 3. CHỐT GÓC QUAY CỐ ĐỊNH (Chỉ phụ thuộc vào lưới bản đồ)
    dx = target_c - prev_c
    dy = target_r - prev_r
    target_angle = math.atan2(dy, dx)

    # 4. Tính sai số
    dist = math.sqrt((tx - cx)**2 + (ty - cy)**2)
    angle_error = target_angle - current_angle
    
    while angle_error > math.pi: angle_error -= 2 * math.pi
    while angle_error < -math.pi: angle_error += 2 * math.pi

    # --- STATE MACHINE (LOGIC DỨT KHOÁT) ---
    
    if state == "TURN":
        # Nếu chưa nhìn đúng hướng mục tiêu -> Xoay tại chỗ
        if abs(angle_error) > 0.02: 
            turn_speed = MAX_SPEED * 0.2 
            if angle_error > 0:
                left_motor.setVelocity(-turn_speed)
                right_motor.setVelocity(turn_speed)
            else:
                left_motor.setVelocity(turn_speed)
                right_motor.setVelocity(-turn_speed)
        else:
            # Xoay xong, phanh lại 1 nhịp và chuyển trạng thái
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            state = "MOVE"
            
    elif state == "MOVE":
        # Nếu đã tiến vào tâm ô (sai số 4cm cho chắc chắn)
        if dist < 0.04: 
            print(f"Đã cập bến ô: {target_r, target_c}")
            path_index += 1
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            state = "TURN" # Chuyển lại trạng thái quay cho ô tiếp theo
        else:
            # Nếu trên đường đi bị trượt nhẹ, thêm chút bù trừ góc (P-control) để đi cho thẳng tắp
            correction = angle_error * 2.0
            base_speed = MAX_SPEED * 0.6
            left_motor.setVelocity(base_speed - correction)
            right_motor.setVelocity(base_speed + correction)