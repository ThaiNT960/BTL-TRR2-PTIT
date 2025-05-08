import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# Cau hinh co ban
N_SAMPLE = 500  # So diem lay mau
N_KNN = 10      # So hang xom gan nhat de ket noi
MAX_EDGE_LEN = 30.0  # Do dai cung toi da

show_animation = True  # Hien thi hoat hinh

# Lop Nut luu thong tin diem
class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x  # Toa do x
        self.y = y  # Toa do y
        self.cost = cost  # Chi phi den diem nay
        self.parent_index = parent_index  # Chi so diem cha

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)

# Ham chinh lap ke hoach duong di PRM
def prm_planning(start_x, start_y, goal_x, goal_y,
                 obstacle_x_list, obstacle_y_list, robot_radius):
    
    obstacle_kd_tree = KDTree(np.vstack((obstacle_x_list, obstacle_y_list)).T)  # Tao KDTree cho vat can

    sample_x, sample_y = sample_points(
        start_x, start_y, goal_x, goal_y,
        robot_radius,
        obstacle_x_list, obstacle_y_list,
        obstacle_kd_tree
    )  # Lay mau diem

    if show_animation:
        plt.plot(sample_x, sample_y, ".b")  # Ve diem lay mau

    road_map = generate_road_map(sample_x, sample_y, robot_radius, obstacle_kd_tree)  # Tao ban do duong di

    rx, ry = dijkstra_planning(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y)  # Tim duong di ngan nhat

    return rx, ry

# Lay mau diem trong khong gian
def sample_points(sx, sy, gx, gy, rr, ox, oy, obstacle_kd_tree):
    max_x = max(ox)  # Gioi han tren x
    max_y = max(oy)  # Gioi han tren y
    min_x = min(ox)  # Gioi han duoi x
    min_y = min(oy)  # Gioi han duoi y

    sample_x, sample_y = [], []
    rng = np.random.default_rng()  # Tao so ngau nhien

    while len(sample_x) <= N_SAMPLE:
        tx = (rng.random() * (max_x - min_x)) + min_x  # Toa do x ngau nhien
        ty = (rng.random() * (max_y - min_y)) + min_y  # Toa do y ngau nhien

        dist, index = obstacle_kd_tree.query([tx, ty])  # Khoang cach den vat can

        if dist >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)  # Them diem bat dau x
    sample_y.append(sy)  # Them diem bat dau y
    sample_x.append(gx)  # Them diem ket thuc x
    sample_y.append(gy)  # Them diem ket thuc y

    return sample_x, sample_y

# Tao ban do duong di
def generate_road_map(sample_x, sample_y, rr, obstacle_kd_tree):
    road_map = []
    n_sample = len(sample_x)
    sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)  # Tao KDTree cho diem mau

    for (ix, iy) in zip(sample_x, sample_y):
        dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)  # Tim hang xom gan nhat

        edge_id = []

        for ii in range(1, len(indexes)):
            nx = sample_x[indexes[ii]]
            ny = sample_y[indexes[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obstacle_kd_tree):  # Kiem tra va cham
                edge_id.append(indexes[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    return road_map

# Kiem tra va cham giua hai diem
def is_collision(sx, sy, gx, gy, rr, obstacle_kd_tree):
    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)  # Goc huong
    d = math.hypot(dx, dy)  # Khoang cach

    if d >= MAX_EDGE_LEN:
        return True

    D = rr
    n_step = round(d / D)  # So buoc kiem tra

    for i in range(n_step):
        dist, index = obstacle_kd_tree.query([x, y])  # Khoang cach den vat can

        if dist <= rr:
            return True

        x += D * math.cos(yaw)  # Di chuyen doc doan thang
        y += D * math.sin(yaw)

    dist, index = obstacle_kd_tree.query([gx, gy])  # Kiem tra diem cuoi

    if dist <= rr:
        return True

    return False

# Tim duong di ngan nhat bang Dijkstra
def dijkstra_planning(sx, sy, gx, gy, road_map, sample_x, sample_y):
    start_node = Node(sx, sy, 0.0, -1)  # Nut bat dau
    goal_node = Node(gx, gy, 0.0, -1)  # Nut dich

    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node  # Them nut bat dau vao tap mo

    path_found = True

    while True:
        if not open_set:
            print("Cannot find path")  # Khong tim thay duong
            path_found = False
            break

        c_id = min(open_set, key=lambda o: open_set[o].cost)  # Chon nut chi phi nho nhat
        current = open_set[c_id]

        if show_animation and len(closed_set.keys()) % 2 == 0:
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None]
            )  # Thoat khi nhan phim escape
            plt.plot(current.x, current.y, "xg")  # Ve diem hien tai
            plt.pause(0.001)

        if c_id == (len(road_map) - 1):
            print("goal is found!")  # Tim thay dich
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        del open_set[c_id]
        closed_set[c_id] = current

        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.hypot(dx, dy)  # Khoang cach

            node = Node(sample_x[n_id], sample_y[n_id], current.cost + d, c_id)  # Tao nut moi
            if n_id in closed_set:
                continue
            if n_id in open_set:
                if open_set[n_id].cost > node.cost:
                    open_set[n_id].cost = node.cost
                    open_set[n_id].parent_index = c_id
            else:
                open_set[n_id] = node

    if path_found is False:
        return [], []

    rx, ry = [goal_node.x], [goal_node.y]  # Truy vet duong di
    parent_index = goal_node.parent_index

    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    final_path_x = rx[::-1]  # Dao nguoc duong di x
    final_path_y = ry[::-1]  # Dao nguoc duong di y

    path_str = " -> ".join(f"[{x:.2f}, {y:.2f}]" for x, y in zip(final_path_x, final_path_y))
    print("The path found is: " + path_str)  # In duong di

    return rx, ry

# Ham chinh chay chuong trinh
def main():
    print("start!!")  # Bat dau
    
# NOI NHAP TESTCASE
    sx = 10.0  # Toa do x bat dau
    sy = 10.0  # Toa do y bat dau
    gx = 50.0  # Toa do x ket thuc
    gy = 50.0  # Toa do y ket thuc
    robot_size = 5.0  # Ban kinh robot

    ox = []  # Danh sach toa do x vat can
    oy = []  # Danh sach toa do y vat can

    for i in range(60):
        ox.append(float(i))
        oy.append(0.0)  # Bien duoi

    for i in range(60):
        ox.append(60.0)
        oy.append(float(i))  # Bien phai

    for i in range(61):
        ox.append(float(i))
        oy.append(60.0)  # Bien tren

    for i in range(61):
        ox.append(0.0)
        oy.append(float(i))  # Bien trai

    for i in range(40):
        ox.append(20.0)
        oy.append(float(i))  # Tuong doc 1

    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)  # Tuong doc 2
# NOI KET THUC NHAP TESTCASE

    if show_animation:
        plt.plot(ox, oy, ".k")  # Ve vat can
        plt.plot(sx, sy, "^r")  # Ve diem bat dau
        plt.plot(gx, gy, "^c")  # Ve diem ket thuc
        plt.grid(True)
        plt.axis("equal")

    rx, ry = prm_planning(sx, sy, gx, gy, ox, oy, robot_size)  # Lap ke hoach duong di

    assert rx, 'Cannot found path'  # Kiem tra duong di

    if show_animation:
        for i in range(len(rx) - 1, 0, -1):
            plt.plot(rx[i - 1:i + 1], ry[i - 1:i + 1], "-r")  # Ve duong di
            plt.pause(0.1)
        plt.show()

if __name__ == '__main__':
    main()
