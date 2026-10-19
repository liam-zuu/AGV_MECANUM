# AGV_MECANUM

Firmware ESP32-S3 cho AGV Mecanum, đồng thời là **controller under test** trong khung HIL (repo `liam-zuu/hil-framework`).

PlatformIO + ESP-IDF, board `esp32-s3-devkitc-1`, flash 16 MB.

## Cấu trúc `src/`

| Module | Vai trò |
|---|---|
| `main.c` | Task chính, máy trạng thái `MODE_WAITING` / `MODE_MANUAL` / `MODE_AUTO` |
| `controller/` | `pid.c`, `adrc.c`: vòng trong điều khiển tốc độ bánh |
| `kinematics/` | Động học nghịch / thuận Mecanum |
| `motor/` | PWM MCPWM cho 4 driver (RPWM/LPWM) |
| `encoder/` | Đọc encoder 4 bánh (PCNT) |
| `imu/` | BNO085 qua thư viện `components/esp32_BNO08x` |
| `traj_receiver/` | Nhận setpoint `vx, vy, wz`: gói `0xBB` 14 byte, từ WiFi UDP (port 5010) hoặc UART từ H7 (chế độ HIL) |
| `logger/` | Gửi log qua WiFi UDP (port 5005), đọc bằng `tools/agv_logger.py` của repo hil-framework |
| `wifi/`, `ibus/`, `manual_control/` | WiFi, tay điều khiển RC iBUS, lái tay |
| `ultrasonic/`, `led_buzzer/` | Cảm biến siêu âm dừng khẩn, đèn / còi |
| `tests/` | Test từng module, bật bằng cờ `-DTEST_MODE` + `-DTEST_xxx` trong `platformio.ini` |

## Build và nạp

```bash
pio run
pio run -t upload
pio device monitor
```

Muốn chạy test một module: bỏ comment `-DTEST_MODE` và đúng một dòng `-DTEST_xxx` trong `platformio.ini`, rồi build lại.

## Lưu ý

- `main.c` đang bật `HIL_BYPASS_MOTOR_ENABLE`: bỏ qua kiểm tra công tắc SWA của RC để test trên bàn HIL không cần bộ thu RC. **Tắt dòng này trước khi chạy trên AGV thật.**
- `platformio.ini` còn dòng `-I src/rasp_uart`, nhưng thư mục này không còn (đã thay bằng `traj_receiver/`). Dòng này vô hại, có thể xoá khi dọn lần sau.
- Mapping chân PWM ESP32 → H7 nằm trong session summary 18/06 của luận văn. Trong `motor.c`, hai field `rpwm_gpio` / `lpwm_gpio` của `motor_cfg` đang bị đổi chỗ có chủ đích.
