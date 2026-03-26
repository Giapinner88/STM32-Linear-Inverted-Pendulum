# LED Blink Test - PA4

Thư viện test nhấp nháy LED trên chân PA4 cho dự án STM32F103C8TX.

## Tệp tin

- `led_blink_test.h` - Header file với các khai báo hàm
- `led_blink_test.c` - Implement các hàm nhấp nháy LED
- `led_blink_test_example.c` - Các ví dụ sử dụng

## Các hàm có sẵn

### 1. `LED_Blink_Init()`
Khởi tạo LED (PA4 đã được cấu hình sẵn trong `MX_GPIO_Init()`).

```c
LED_Blink_Init();
```

### 2. `LED_Blink(uint16_t interval_ms)`
Nhấp nháy LED một lần với khoảng thời gian được chỉ định.

**Tham số:**
- `interval_ms`: Khoảng thời gian (ms) giữa mỗi lần toggle

**Ví dụ:**
```c
while(1)
{
    LED_Blink(500);  // Toggle LED mỗi 500ms (tần số 1Hz)
}
```

### 3. `LED_Blink_N_Times(uint16_t times, uint16_t interval_ms)`
Nhấp nháy LED N lần rồi tắt.

**Tham số:**
- `times`: Số lần nhấp nháy
- `interval_ms`: Khoảng thời gian (ms) cho mỗi chu kỳ on/off

**Ví dụ:**
```c
LED_Blink_N_Times(10, 300);  // Nhấp nháy 10 lần với khoảng 300ms
```

## Cách sử dụng

### Bước 1: Thêm vào platformio.ini (nếu cần)

Nếu sử dụng PlatformIO, thêm dòng này vào `platformio.ini`:

```ini
lib_extra_dirs = lib/led_blink_test
```

### Bước 2: Include header trong main.c

```c
#include "led_blink_test.h"
```

### Bước 3: Sử dụng trong main()

#### Ví dụ 1: Nhấp nháy liên tục
```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM3_Init();
    // ... các init khác ...
    
    LED_Blink_Init();
    
    while(1)
    {
        LED_Blink(500);  // Nhấp nháy mỗi 500ms
    }
    
    return 0;
}
```

#### Ví dụ 2: Nhấp nháy N lần
```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    // ... các init khác ...
    
    LED_Blink_Init();
    LED_Blink_N_Times(10, 300);  // Nhấp nháy 10 lần
    
    // Tiếp tục chương trình chính
    while(1)
    {
        // ...
    }
    
    return 0;
}
```

#### Ví dụ 3: Pattern nhấp nháy tùy chỉnh
```c
while(1)
{
    LED_Blink_N_Times(5, 100);   // Blink nhanh 5 lần
    delay_ms(500);
    
    LED_Blink_N_Times(3, 500);   // Blink chậm 3 lần
    delay_ms(1000);
}
```

## Thông tin kỹ thuật

- **Chân GPIO:** PA4
- **Mode:** GPIO_MODE_OUTPUT_PP (Push-Pull Output)
- **Tốc độ:** GPIO_SPEED_FREQ_HIGH
- **Phụ thuộc:** delay.h từ thư viện delay

## Lưu ý

1. PA4 đã được cấu hình sẵn trong `MX_GPIO_Init()` của dự án
2. Hàm `LED_Blink()` là hàm blocking (chặn thực thi)
3. Sử dụng `delay_ms()` từ driver delay để timing chính xác
4. LED sẽ tắt sau khi gọi `LED_Blink_N_Times()`

## Troubleshooting

**LED không nhấp nháy:**
- Kiểm tra kết nối phần cứng trên PA4
- Kiểm tra LED có đúng hướng không (anode vào PA4, cathode vào GND)
- Kiểm tra trở hạn dòng (thường 220Ω - 1kΩ)
- Kiểm tra xem PA4 có bị sử dụng bởi module khác không (ví dụ: ADC)

**Blink không đều:**
- Đảm bảo `delay_ms()` được khởi tạo đúng
- Kiểm tra System Clock configuration

