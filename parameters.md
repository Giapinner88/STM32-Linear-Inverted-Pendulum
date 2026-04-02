# Thông số vật lý — Wheeltec IP570 Linear Inverted Pendulum

## Con lắc (Pendulum Rod)

| Ký hiệu | Mô tả | Giá trị | Đơn vị | Nguồn |
|:-------:|-------|--------:|:------:|-------|
| $L$ | Chiều dài thanh con lắc | 200.0 | mm | Đo thực tế |
| $d$ | Đường kính thanh | 6.0 | mm | Đo thực tế |
| $\rho$ | Khối lượng riêng (thép không gỉ) | 7900 | kg/m³ | Tra cứu vật liệu |
| $m$ | Khối lượng con lắc | 44.67 | g | Tính từ $\rho \cdot A \cdot L$ |
| $l$ | Khoảng cách từ khớp đến CoM | 100.0 | mm | $l = L/2$ (thanh đồng nhất) |
| $J$ | Momen quán tính quanh khớp | $5.96 \times 10^{-4}$ | kg·m² | $J = \frac{1}{3}mL^2$ |
| $mgl$ | Tích momen trọng lực | 0.04378 | N·m | Tính từ $m, g, l$ |

> **Công thức:** Thanh đồng nhất tiết diện tròn, khớp tại một đầu:
> $$m = \rho \cdot \frac{\pi d^2}{4} \cdot L, \qquad l = \frac{L}{2}, \qquad J = \frac{1}{3}mL^2$$

---

## Xe (Cart)

| Ký hiệu | Mô tả | Giá trị | Đơn vị | Nguồn |
|:-------:|-------|--------:|:------:|-------|
| $M_c$ | Khối lượng xe (cart + motor + board) | 0.10 | kg | Tạm đặt để tune swing-up |

---

## Hệ thống điều khiển

| Ký hiệu | Mô tả | Giá trị | Đơn vị | Nguồn |
|:-------:|-------|--------:|:------:|-------|
| $g$ | Gia tốc trọng trường | 9.81 | m/s² | Hằng số |
| $T_s$ | Chu kỳ lấy mẫu | 5 | ms | `dt = 0.005` trong `filter.c` |
| $V$ | Điện áp nguồn | 12 | V | `u/12.0f` trong `control.c` |

---

## Bộ tham số swing-up preset (đã nạp vào firmware)

Mục tiêu của preset này là **đánh đu có kiểm soát**, ưu tiên không đập tường trong giai đoạn đầu trước khi có mô hình đầy đủ.

### 1) Mô hình năng lượng dùng cho swing-up

Với $\theta=0$ tại vị trí thả xuống, năng lượng được dùng trong code:

$$E = \frac{1}{2}l^2\dot{\theta}^2 + gl(1-\cos\theta), \qquad E_{target}=2gl$$

và luật điều khiển chính:

$$u = -k_E(E-E_{target})\dot{\theta}\cos\theta - k_x x_{err} - k_v\dot{x}$$

Trong đó đang dùng:
- $l = 0.200\ \text{m}$
- $M_c = 0.100\ \text{kg}$ (đang để placeholder cho bước model-based kế tiếp)
- $k_E = 0.030$
- $k_x = 1.2\times10^{-5}$
- $k_v = 9.5\times10^{-4}$
- Giới hạn biên điều khiển: $|u| \le 0.42$

### 2) Chống đập tường (wall-aware shaping)

Ngoài soft-limit cứng, firmware có thêm lớp giảm lực theo khoảng cách tới biên encoder:
- Soft limit cứng: $x \in [-18000,\ 18000]$ ticks
- Bắt đầu giảm lực khi còn $4500$ ticks tới tường
- Vùng nguy hiểm gần tường: $1700$ ticks
- Tại sát tường, hệ số lực tối thiểu còn $20\%$
- Nếu xe vẫn đang lao ra ngoài gần tường, hệ sẽ tự thêm lực hãm ngược chiều vận tốc

### 3) Trình tự tune khuyến nghị (an toàn)

1. Tăng/giảm $k_E$ trước để chỉnh mức bơm năng lượng.
2. Nếu xe chạy xa tâm, tăng nhẹ $k_x$ và $k_v$.
3. Nếu còn chạm tường, giảm tiếp biên $|u|$ hoặc tăng vùng bắt đầu giảm lực (4500 -> 5500 ticks).

---

## LQR Gains (từ source code gốc, $L = 390$ mm)

> ⚠️ Các gain dưới đây được tính cho cấu hình **L = 390 mm**. Khi thay đổi chiều dài con lắc xuống **200 mm**, cần tính lại ma trận A, B và thiết kế lại LQR.

| Ký hiệu | Mô tả | Giá trị |
|:-------:|-------|--------:|
| $k_1$ | Gain vị trí xe $x$ | −36.4232 |
| $k_2$ | Gain vận tốc xe $\dot{x}$ | −40.0996 |
| $k_3$ | Gain góc con lắc $\theta$ | −101.4914 |
| $k_4$ | Gain vận tốc góc $\dot{\theta}$ | −15.8376 |

---

## Ma trận hệ thống tuyến tính hoá (cần $M_c$)

Vector trạng thái: $\mathbf{x} = [x,\ \dot{x},\ \theta,\ \dot{\theta}]^\top$

$$\Delta = (M_c + m)J - (ml)^2$$

$$A = \begin{bmatrix} 0 & 1 & 0 & 0 \\ 0 & 0 & \dfrac{(ml)^2 g}{\Delta} & 0 \\ 0 & 0 & 0 & 1 \\ 0 & 0 & \dfrac{(M_c+m)mgl}{\Delta} & 0 \end{bmatrix}, \qquad B = \begin{bmatrix} 0 \\ \dfrac{J + ml^2}{\Delta} \\ 0 \\ \dfrac{-ml}{\Delta} \end{bmatrix}$$

---

*Tài liệu tổng hợp từ reverse-engineering source code STM32 Wheeltec IP570.*