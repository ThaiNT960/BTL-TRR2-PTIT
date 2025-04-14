# BTL-TRR2-PTIT

## Mô tả dự án
Đây là dự án Tự động hoá Robot di chuyển trong môi trường có vật cản . Dự án xây dựng một hệ thống lập kế hoạch đường đi cho robot, sử dụng các thuật toán PRM (Probabilistic Roadmap) xác định các điểm mẫu và tìm đường đi hiệu quả từ điểm xuất phát đến điểm đích.

## Các chức năng chính
- Tạo các điểm mẫu (Sampling) trong không gian 2D, tránh các vật cản.
- Sử dụng thuật toán PRM để lập kế hoạch đường đi cho robot.
- Tìm đường đi ngắn nhất giữa hai điểm thông qua thuật toán tìm kiếm Dijkstra.
- Vẽ các kết quả, bao gồm bản đồ vật cản, các điểm mẫu và đường đi của robot.

## Cài đặt

### 1. Cài đặt Visual Studio Code (VSCode) và Python
Để bắt đầu, bạn cần cài đặt Visual Studio Code (VSCode) và Python trên máy tính của mình.

#### Cài đặt VSCode
1. Tải và cài đặt Visual Studio Code từ trang chính thức: [Visual Studio Code](https://code.visualstudio.com/).
2. Cài đặt các extension sau trong VSCode:
   - **Python**: Extension giúp VSCode nhận diện và hỗ trợ phát triển Python.
   - **Code Runner**: Chạy mã Python trực tiếp từ VSCode.

#### Cài đặt Python
1. Tải và cài đặt Python từ trang chính thức: [Python](https://www.python.org/downloads/).
2. Đảm bảo rằng Python đã được thêm vào PATH khi cài đặt.

### 2. Cài đặt các thư viện `numpy`, `matplotlib`, và `scipy`
