import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# parameter
N_SAMPLE = 500  # Số lượng điểm mẫu ngẫu nhiên được sinh ra
N_KNN = 10   # Số lượng cạnh tối đa kết nối từ một điểm mẫu 
MAX_EDGE_LEN = 30.0 # [m] Độ dài cạnh tối đa cho phép giữa hai điểm 

show_animation = True

class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x) + "," + str(self.y) + "," +\
               str(self.cost) + "," + str(self.parent_index)

def prm_planning(start_x, start_y, goal_x, goal_y,obstacle_x_list, obstacle_y_list, robot_radius):
    """
    PRM (Probabilistic Road Map).

    Thuật toán PRM được sử dụng để tìm đường đi trong không gian có vật cản, thông qua việc xây dựng một bản đồ xác suất. 
    Thuật toán giúp xác định lộ trình từ điểm bắt đầu đến điểm đích, tránh các vật cản trong môi trường.

    :param start_x: Tọa độ x điểm bắt đầu.
    :param start_y: Tọa độ y điểm bắt đầu.
    :param goal_x: Tọa độ x điểm đích.
    :param goal_y: Tọa độ y điểm đích.
    :param obstacle_x_list: Danh sách tọa độ x của các vật cản.
    :param obstacle_y_list: Danh sách tọa độ y của các vật cản.
    :param robot_radius: Bán kính robot, dùng để tính toán vùng không gian mà robot cần tránh.
    :return: Danh sách các điểm đường đi (rx, ry) từ điểm bắt đầu đến điểm đích.
    """


    obstacle_kd_tree = KDTree(np.vstack((obstacle_x_list, obstacle_y_list)).T)

    sample_x, sample_y = sample_points(start_x, start_y, goal_x, goal_y,
                                       robot_radius,
                                       obstacle_x_list, obstacle_y_list,
                                       obstacle_kd_tree)
    if show_animation:
        plt.plot(sample_x, sample_y, ".b") # thêm dấu chấm blue
   
    road_map = generate_road_map(sample_x, sample_y,
                                 robot_radius, obstacle_kd_tree)

    rx, ry = dijkstra_planning(
        start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y)

    return rx, ry
def sample_points(sx, sy, gx, gy, rr, ox, oy, obstacle_kd_tree):
    max_x = max(ox)
    max_y = max(oy)
    min_x = min(ox)
    min_y = min(oy)

    sample_x, sample_y = [], []

    rng = np.random.default_rng()

    while len(sample_x) <= N_SAMPLE:
        tx = (rng.random() * (max_x - min_x)) + min_x
        ty = (rng.random() * (max_y - min_y)) + min_y

        # Kiểm tra xem điểm (tx, ty) có nằm trong vùng vật cản hay không
        dist, index = obstacle_kd_tree.query([tx, ty])

        if dist >= rr:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return sample_x, sample_y

def generate_road_map(sample_x, sample_y, rr, obstacle_kd_tree):
    """
    Tạo bản đồ đường đi (Road Map)

    sample_x: [m] Danh sách các tọa độ x của các điểm mẫu đã chọn
    sample_y: [m] Danh sách các tọa độ y của các điểm mẫu đã chọn
    rr: Bán kính của robot (Robot Radius) [m]
    obstacle_kd_tree: Đối tượng KDTree chứa các vật cản (obstacles)

    """

    road_map = []
    n_sample = len(sample_x)
    sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):

        dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
        edge_id = [] #tạo danh sách để lưu các điểm kết nối

        for ii in range(1, len(indexes)):
            nx = sample_x[indexes[ii]]
            ny = sample_y[indexes[ii]]

            if not is_collision(ix, iy, nx, ny, rr, obstacle_kd_tree):
                edge_id.append(indexes[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    return road_map

def is_collision(sx, sy, gx, gy, rr, obstacle_kd_tree):
    """
    Kiểm tra xem đoạn đường từ điểm bắt đầu (sx, sy) đến điểm kết thúc (gx, gy)
    có va chạm với chướng ngại vật hay không.

    :param sx: tọa độ x điểm bắt đầu
    :param sy: tọa độ y điểm bắt đầu
    :param gx: tọa độ x điểm kết thúc
    :param gy: tọa độ y điểm kết thúc
    :param rr: bán kính robot
    :param obstacle_kd_tree: cây KDTree chứa tọa độ chướng ngại vật
    :return: True nếu va chạm, False nếu không va chạm
    """

    x = sx
    y = sy
    dx = gx - sx
    dy = gy - sy
    yaw = math.atan2(gy - sy, gx - sx)
    d = math.hypot(dx, dy)

    if d >= MAX_EDGE_LEN:
        return True # va chạm

    D = rr
    n_step = round(d / D)

    for i in range(n_step):
        dist, _ = obstacle_kd_tree.query([x, y])
        if dist <= rr:
            return True  
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    # check điểm cuối
    dist, index = obstacle_kd_tree.query([gx, gy])
    if dist <= rr:
        return True  

    return False  

def dijkstra_planning(sx, sy, gx, gy, road_map, sample_x, sample_y):
    """
    :param sx: Tọa độ x điểm bắt đầu [m].
    :param sy: Tọa độ y điểm bắt đầu [m].
    :param gx: Tọa độ x điểm đích [m].
    :param gy: Tọa độ y điểm đích [m].
    :param obstacle_x_list: Danh sách tọa độ x của vật cản [m].
    :param obstacle_y_list: Danh sách tọa độ y của vật cản [m].
    :param robot_radius: Bán kính robot [m].
    :param road_map: Bản đồ các kết nối giữa các điểm mẫu.
    :param sample_x: Danh sách tọa độ x của các điểm mẫu [m].
    :param sample_y: Danh sách tọa độ y của các điểm mẫu [m].

    :return: Hai danh sách tọa độ của đường đi ([x1, x2, ...], [y1, y2, ...]), trả về rỗng nếu không tìm thấy đường đi.
    """

    start_node = Node(sx, sy, 0.0, -1)
    goal_node = Node(gx, gy, 0.0, -1)

    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    path_found = True

    while True:
        if not open_set:
            print("Cannot find path")
            path_found = False
            break

        c_id = min(open_set, key=lambda o: open_set[o].cost)
        current = open_set[c_id]

       
        if show_animation and len(closed_set.keys()) % 2 == 0:
           
            plt.gcf().canvas.mpl_connect(
               'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        if c_id == (len(road_map) - 1):
            print("goal is found!")
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

       
        del open_set[c_id]
        
        closed_set[c_id] = current

        
        for i in range(len(road_map[c_id])):
            n_id = road_map[c_id][i]
            dx = sample_x[n_id] - current.x
            dy = sample_y[n_id] - current.y
            d = math.hypot(dx, dy)
            node = Node(sample_x[n_id], sample_y[n_id],
                        current.cost + d, c_id)

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

    
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index
    
    final_path_x = rx[::-1]
    final_path_y = ry[::-1]
    path_str = " -> ".join(f"[{x:.2f}, {y:.2f}]" for x, y in zip(final_path_x, final_path_y))
    print("Đường đi tìm được: " + path_str)

    return rx, ry

def main():
    print("start!!")

    # start và goal 
    sx = 10.0  # [m] vi tri truc x robot 
    sy = 10.0  # [m]  vi tri truc y robot
    gx = 50.0  # [m] đích
    gy = 50.0  # [m]  đích
    robot_size = 5.0  # [m]

    ox = [] # Danh sách hoành độ (tọa độ x) của vật cản
    oy = [] # Danh sách tung độ (tọa độ y) của vật cản

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

    if show_animation:
        plt.plot(ox, oy, ".k")  # k màu đen
        plt.plot(sx, sy, "^r")  # '^' = tam giác hướng lên, 'r' = red (đỏ)
        plt.plot(gx, gy, "^c")   # '^' = tam giác hướng lên, 'c' = cyan
        plt.grid(True)     # Hiển thị lưới (grid) cho dễ quan sát
        plt.axis("equal")  # Đặt tỉ lệ trục X và Y bằng nhau để bản đồ không bị méo
     

    rx, ry = prm_planning(sx, sy, gx, gy, ox, oy, robot_size)

    assert rx, 'Cannot found path'

    """  if show_animation:
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()
    """
    if show_animation:
        for i in range(len(rx)-1, 0, -1):
            plt.plot(rx[i-1:i+1], ry[i-1:i+1], "-r")
            plt.pause(0.1)
        plt.show()


if __name__ == '__main__':
    main()