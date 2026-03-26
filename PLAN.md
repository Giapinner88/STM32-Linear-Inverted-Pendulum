# Kế Hoạch Triển Khai Swing-up & LQR: Kiến Trúc Điều Khiển Phân Tán (PC - STM32)

Bản kế hoạch này kết hợp tư duy hệ thống thời gian thực (real-time) nguyên thủy của vi điều khiển với sức mạnh tối ưu hóa quỹ đạo từ máy tính, được thiết kế đặc thù cho cơ cấu chấp hành là động cơ DC có chổi than.

## 1. Kiến Trúc Phần Mềm (Software Architecture)
Hệ thống được chia làm hai mảng hoàn toàn độc lập, giao tiếp với nhau qua giao thức nối tiếp (Type-C UART).

### 1.1. Mảng Vi Điều Khiển (STM32 Firmware)
Cấu trúc thư mục được thiết kế để tách biệt lớp vật lý và lớp giao thức.

```text
IP570_Firmware/
├── lib/
│   ├── Hardware/          
│   │   ├── motor.c/.h     # Băm xung PWM, đảo chiều, CÙ BÙ VÙNG CHẾT (Dead-zone compensation)
│   │   ├── encoder.c/.h   # Đọc Timer, lượng tử hóa vận tốc tịnh tiến
│   │   └── mpu6050.c/.h   # Giao tiếp I2C, bộ lọc Complementary Filter (tính góc theta)
│   └── Protocol/          
│       └── comms.c/.h     # Đóng gói/Giải mã struct nhị phân (Telemetry & Command)
├── src/
│   └── main.c             # State Machine thời gian thực (chạy trong ngắt 10ms)
└── test/                  
    ├── test_motor.c       # Đo đạc dead-zone thực tế của động cơ MG513
    └── test_comms.c       # Kiểm thử tốc độ truyền nhận UART 115200 bps
```

### 1.2. Mảng Máy Tính (Python Control PC)
Toàn bộ các phép toán ma trận nặng nề được đưa lên máy tính.

```text
IP570_PC_Control/
├── models/
│   └── dc_cartpole.py     # Phương trình Euler-Lagrange cho xe thăng bằng 2 bánh
├── controllers/
│   ├── lqr_solver.py      # Giải phương trình Riccati liên tục (CARE) tìm ma trận K
│   └── swingup_pydrake.py # Tối ưu hóa quỹ đạo Direct Collocation sinh bảng tra cứu (Lookup table)
├── comms/
│   └── serial_link.py     # Đọc/Ghi dữ liệu nhị phân với cổng Type-C
└── main_dashboard.py      # Giao diện vẽ đồ thị và gửi lệnh (Start/Stop/Mode)
```

## 2. Giao Thức Truyền Thông (PC - MCU Protocol)
Việc sử dụng trực tiếp các cấu trúc (`struct`) byte nhị phân thay vì chuỗi ASCII (`printf`) giúp STM32 phản hồi cực nhanh mà không bị nghẽn cổ chai truyền thông.

**Trên STM32 (C/C++):**
```c
// Gói dữ liệu gửi lên PC (Telemetry - 100Hz)
#pragma pack(push, 1)
typedef struct {
    uint8_t header[2]; // 0xAA, 0xBB
    float x_pos;       
    float x_vel;
    float theta;       
    float theta_vel;
} RobotStatePacket;

// Gói dữ liệu nhận từ PC (Command - 100Hz)
typedef struct {
    uint8_t header[2]; // 0xCC, 0xDD
    float control_effort; // Lực u (Torque/Voltage)
    uint8_t mode;         // 0: Idle, 1: Swing-up, 2: LQR
} ControlCommandPacket;
#pragma pack(pop)
```

## 3. Lộ Trình Phát Triển 4 Giai Đoạn

### Giai Đoạn 1: Làm sạch lớp Phần cứng & Xác minh (STM32)
Trước khi truyền thông, phần cứng phải được lượng hóa thành các hằng số.
1.  **Đo Vùng Chết (Dead-zone):** Động cơ DC cần một lượng điện áp tối thiểu để thắng ma sát tĩnh. Viết hàm `Motor_SetTorque(float u)` có khả năng tự động nội suy (mapping) tín hiệu $u$ nhỏ thành mức PWM đủ để bánh xe nhúc nhích. Nếu bỏ qua bước này, LQR sẽ gây ra hiện tượng rung lắc (chattering) quanh điểm cân bằng.
2.  **Đồng bộ Hệ Quy Chiếu:** * Kéo xe tới trước $\rightarrow$ Encoder $\dot{x} > 0$.
    * Nghiêng xe tới trước $\rightarrow$ MPU6050 $\theta > 0$.
    * Truyền lệnh tiến tới $\rightarrow$ Motor quay tới trước ($u > 0$).

### Giai Đoạn 2: Thiết lập Cầu nối Truyền thông (PC <-> STM32)
1.  Viết bộ xử lý ngắt UART (sử dụng DMA để không chặn CPU) trên STM32 để nhận `ControlCommandPacket` và đẩy `RobotStatePacket` lên.
2.  Viết script `serial_link.py` trên Python để đọc dữ liệu và vẽ đồ thị góc $\theta$ theo thời gian thực.
3.  **Bài test:** Dùng máy tính gửi lệnh thay đổi lực `control_effort` dưới dạng sóng hình sin để kiểm tra phản hồi của bánh xe.

### Giai Đoạn 3: Mô hình hóa & LQR (Python)
1.  Thay vì dùng mô hình của động cơ bước (hệ điều khiển vận tốc), xây dựng ma trận trạng thái $A$ và $B$ dựa trên đặc tính động cơ DC (hệ điều khiển lực đẩy/điện áp). Các biến cần đo đạc: Khối lượng xe ($M$), khối lượng con lắc ($m$), chiều dài con lắc ($l$).
2.  Chạy script tính toán ma trận độ lợi $K$.
3.  Truyền cứng (hardcode) ma trận $K$ này xuống firmware STM32 hoặc truyền qua cổng Type-C lúc khởi động. STM32 chỉ làm duy nhất một phép tính mỗi 10ms: `u = - (k1*x + k2*x_dot + k3*theta + k4*theta_dot)`.

### Giai Đoạn 4: Swing-up bằng Tối ưu hóa Quỹ đạo (Pydrake)
Việc hất một khung xe nặng như IP570 cần một quỹ đạo phi tuyến cực kỳ chính xác.
1.  Đưa mô hình động lực học đã xây dựng ở Giai đoạn 3 vào `pydrake`.
2.  Chạy Direct Collocation để tìm ra một chuỗi các giá trị tín hiệu lực $u(t)$ lý tưởng theo thời gian.
3.  Lưu chuỗi $u(t)$ này thành một mảng (Lookup table).
4.  Khi người dùng nhấn lệnh "Swing-up" từ PC, hệ thống sẽ tuần tự bơm các giá trị $u$ này xuống STM32. Khi góc $\theta$ lọt vào vùng $\pm 10^\circ$, State Machine trên STM32 tự động chuyển sang trạng thái `LQR_BALANCE` để bắt giữ con lắc.