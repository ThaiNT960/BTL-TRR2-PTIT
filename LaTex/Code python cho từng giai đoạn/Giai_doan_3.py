import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# Danh sách vật cản
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

# Vị trí bắt đầu và kết thúc, bán kính robot
sx, sy = 10.0, 10.0
gx, gy = 50.0, 50.0
robot_radius = 5.0

# KDTree cho vật cản
obstacle_kd_tree = KDTree(np.vstack((ox, oy)).T)

# Sinh điểm mẫu
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

# Lấy danh sách điểm mẫu
sample_x, sample_y = sample_points(sx, sy, gx, gy, robot_radius, ox, oy, obstacle_kd_tree)

# Vẽ vật cản và điểm mẫu
plt.figure()
plt.plot(ox, oy, ".k", label="Vật cản")
plt.plot(sample_x, sample_y, ".b", label="Điểm mẫu")

# Vẽ dấu X màu xanh lá tại các chỉ số chẵn
for i in range(len(sample_x)):
    if i % 2 == 0:
        plt.plot(sample_x[i], sample_y[i], "xg")  # Duyệt - X màu xanh lá

# Vẽ điểm bắt đầu và điểm đích
plt.plot(sx, sy, "^r", label="Điểm bắt đầu")  # Tam giác đỏ
plt.plot(gx, gy, "^c", label="Điểm đích")     # Tam giác cyan

plt.xlabel("Trục X [m]")
plt.ylabel("Trục Y [m]")
plt.title("Giai đoạn 3: Duyệt nút và đánh dấu các điểm")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.show()
