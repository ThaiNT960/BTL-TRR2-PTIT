import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# Tạo danh sách tọa độ cho vật cản
ox, oy = [], []
for i in range(60):
    ox.append(float(i))
    oy.append(0.0)
for i in range(60):
    ox.append(60.0)
    oy.append(float(i))
for i in range(61):
    ox.append(float(i))
    oy.append(60.0)
for i in range(61):
    ox.append(0.0)
    oy.append(float(i))
for i in range(40):
    ox.append(20.0)
    oy.append(float(i))
for i in range(40):
    ox.append(40.0)
    oy.append(60.0 - i)

# Thông số điểm bắt đầu, kết thúc và bán kính robot
sx, sy = 10.0, 10.0
gx, gy = 50.0, 50.0
robot_radius = 5.0

# Tạo cây KDTree cho vật cản
obstacle_kd_tree = KDTree(np.vstack((ox, oy)).T)

# Hàm sinh điểm mẫu
def sample_points(sx, sy, gx, gy, rr, ox, oy, obstacle_kd_tree, N_SAMPLE=500):
    sample_x, sample_y = [], []
    rng = np.random.default_rng()
    max_x, max_y = max(ox), max(oy)
    min_x, min_y = min(ox), min(oy)
    while len(sample_x) <= N_SAMPLE:
        tx = rng.random() * (max_x - min_x) + min_x
        ty = rng.random() * (max_y - min_y) + min_y
        dist, _ = obstacle_kd_tree.query([tx, ty])
        if dist >= rr:
            sample_x.append(tx)
            sample_y.append(ty)
    # Thêm điểm bắt đầu và kết thúc
    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)
    return sample_x, sample_y

# Sinh các điểm mẫu
sample_x, sample_y = sample_points(sx, sy, gx, gy, robot_radius, ox, oy, obstacle_kd_tree)

# Vẽ
plt.figure()
plt.plot(ox, oy, ".k", label="Vật cản")               # Vật cản
plt.plot(sample_x, sample_y, ".b", label="Điểm mẫu")  # Các điểm mẫu
plt.plot(sx, sy, "^r", label="Điểm bắt đầu")          # Bắt đầu: tam giác đỏ
plt.plot(gx, gy, "^c", label="Điểm đích")             # Đích: tam giác xanh cyan
plt.xlabel("Trục X [m]")
plt.ylabel("Trục Y [m]")
plt.title("Giai đoạn 2: Vật cản, Điểm mẫu và Vị trí bắt đầu/đích")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.show()
