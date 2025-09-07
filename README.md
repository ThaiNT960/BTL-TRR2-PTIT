#  BTL-TRR2-PTIT

##  Mô tả dự án

Đây là dự án **Tự động hoá Robot di chuyển trong môi trường có vật cản**.
Dự án xây dựng một hệ thống lập kế hoạch đường đi cho robot, sử dụng thuật toán **PRM (Probabilistic Roadmap)** để xác định các điểm mẫu và tìm đường đi hiệu quả từ điểm xuất phát đến điểm đích.

---

##  Các chức năng chính

*  Tạo các điểm mẫu (Sampling) trong không gian 2D, tránh các vật cản.
  * Sử dụng thuật toán **PRM** để lập kế hoạch đường đi cho robot.
*  Tìm đường đi ngắn nhất giữa hai điểm thông qua **thuật toán Dijkstra**.
*  Vẽ kết quả: bản đồ vật cản, các điểm mẫu và đường đi của robot.

---

##  Cài đặt

###  Cài đặt Visual Studio Code (VSCode) và Python

Bạn cần cài đặt **Visual Studio Code (VSCode)** và **Python** trên máy tính.

####  Cài đặt VSCode

1. Tải và cài đặt từ trang chính thức: [Visual Studio Code](https://code.visualstudio.com/).
2. Cài các extension cần thiết:

   *  **Python**: hỗ trợ nhận diện và phát triển Python.
   *  **Code Runner**: chạy code Python trực tiếp từ VSCode.

####  Cài đặt Python

1. Tải và cài đặt từ: [Python.org](https://www.python.org/downloads/).
2. Khi cài đặt nhớ tick chọn **Add Python to PATH**.

---

###  Cài đặt thư viện cần thiết

Mở terminal và chạy:

```bash
pip install numpy matplotlib scipy
```

---

##  Hình ảnh minh họa

<p align="center">
  <img width="562" height="455" alt="giai_doan_1" src="https://github.com/user-attachments/assets/0519fc81-ca50-47da-9a48-7e3055a28813" />
</p>

<p align="center">
  <img width="562" height="456" alt="giai_doan_2" src="https://github.com/user-attachments/assets/040228ed-65b0-45ee-b24e-8fdaffa23949" />
</p>

<p align="center">
  <img width="562" height="457" alt="giai_doan_3" src="https://github.com/user-attachments/assets/5639c9db-2567-4687-9618-52dc3b76333b" />
</p>

<p align="center">
  <img width="640" height="480" alt="giai_doan_4" src="https://github.com/user-attachments/assets/5ddd8bf3-863f-48b4-9684-23b5fa14eb60" />
</p>

---

##  Một số lưu ý

Bản Latex hiện tại **mới chỉ là demo**, cần chỉnh sửa lại nội dung,bố cục cho gọn gàng và chuẩn hơn,code được tham khảo và sửa lại dựa trên phiên bản mã nguồn mở.<br>
Tham khảo mã nguồn gốc tại đây: [PythonRobotics - Probabilistic Road Map](https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/ProbabilisticRoadMap)

