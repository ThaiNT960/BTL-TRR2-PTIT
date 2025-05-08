import matplotlib.pyplot as plt

# Tạo danh sách tọa độ cho vật cản
ox = []  # tọa độ x của vật cản
oy = []  # tọa độ y của vật cản

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

# Tọa độ điểm bắt đầu và điểm đích
sx, sy = 10.0, 10.0  # Điểm bắt đầu
gx, gy = 50.0, 50.0  # Điểm đích

# Vẽ vật cản và các điểm đặc biệt
plt.figure()
plt.plot(ox, oy, ".k", label="Vật cản")           # Dấu chấm màu đen là vật cản
plt.plot(sx, sy, "^r", label="Điểm bắt đầu")      # Tam giác đỏ là start
plt.plot(gx, gy, "^c", label="Điểm đích")         # Tam giác xanh là goal
plt.xlabel("Trục X [m]")
plt.ylabel("Trục Y [m]")
plt.title("Giai đoạn 1: Ghi nhớ bản đồ ")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.show()
