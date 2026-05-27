# Hệ thống nhận diện màu STM32 + GY-33

##  Giới thiệu
Dự án sử dụng **STM32F103** để đọc cảm biến màu **GY-33**, xử lý dữ liệu RGB và gửi kết quả về máy tính qua UART theo giao thức tùy chỉnh.

---

##  Chức năng chính
- Đọc dữ liệu RGB từ cảm biến GY-33 (UART)
- Hiệu chỉnh nền (baseline calibration)
- Lọc nhiễu bằng EMA
- Nhận diện màu: Đỏ / Xanh lá / Xanh dương / Vàng / Không xác định
- Giao tiếp với PC qua UART mềm (PA0/PA1)
- Hiển thị trạng thái bằng LED

---

##  Phần cứng
- STM32F103C8T6
- Cảm biến màu GY-33
- LED báo trạng thái
- UART USB debug

##  Kết nối

### UART mềm (PC)
- TX: PA1  
- RX: PA0  

### GY-33
- TX: PA2  
- RX: PA3  

### LED
- PB12: Đỏ  
- PB13: Xanh lá  
- PB14: Xanh dương  
- PA4: Vàng  

---

## 📡 Giao thức truyền dữ liệu
