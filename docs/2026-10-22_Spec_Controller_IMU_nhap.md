# Spec controller–IMU (nháp v0, chờ Liam duyệt; THESIS kiểm theo O2)

*Viết 22/10/2026, phiên CODE. Outline V7.4 quyết định 3, 4, 12, 23. Các số ghi **[ĐỀ XUẤT]** chưa khóa: khóa sau khi có số đo nhiễu IMU và thí nghiệm tần số trên fwsim (quyết định 21: khóa trước đo chính). Căn cứ code: `AGV_MECANUM` `a6644a5`.*

## 1. Đã chốt (Liam 22/10)

1. **IMU vào vòng kín**, không chỉ ghi log. Mọi firmware đều khởi tạo và đọc IMU (xe thật, controller-only trên HIL, env đo). Không có cấu hình "không IMU".
2. **wz:** lấy hẳn từ gyro IMU khi dữ liệu hợp lệ; không hợp lệ thì lấy wz từ encoder (động học thuận). Không trộn hai nguồn.
3. **vx, vy:** từ encoder như hiện tại.
4. **Có vòng giữ hướng** dùng yaw IMU.
5. **Cờ nghi trượt / nghi bất nhất:** chỉ báo (log, panel), không tự đổi cách điều khiển, không kết luận nguyên nhân.
6. **IMU lỗi lúc khởi tạo thì dừng và báo**, không âm thầm chạy tiếp.
7. Tần số IMU quyết bằng thí nghiệm fwsim (20 Hz so với 100 Hz); trong lúc chờ, mặc định 20 Hz như hiện tại (`imu.cpp:47-49`).

## 2. Hiện trạng code (cần sửa)

| Chỗ | Vấn đề |
|---|---|
| `controller/vel_ctrl.c:41-58` | wz đo lấy từ encoder; IMU không tham gia |
| `imu/imu.cpp:77` | `valid = true` bật một lần, không bao giờ tắt |
| `imu/imu.h:7-18` | Không có thời điểm, số thứ tự mẫu; yaw tính bằng độ |
| `imu/imu.cpp:54`, `78` | `xSemaphoreTakeFromISR`/`GiveFromISR` gọi trong task callback (sai API) |
| `imu/imu.cpp:40-43` | Khởi tạo lỗi chỉ in log rồi `return`; `main.c:483` chạy tiếp |
| Thư viện `sh2.c` | `getProdIds` không có timeout (có thể treo); `sh2_open` báo thành công kể cả khi không nhận reset complete (chi tiết: `hil-framework/docs/2026-10-22_Spec_G0_Gia_lap_BNO085.md`, R1–R2) |

## 3. Dữ liệu IMU đưa vào controller

`imu.h` mở rộng (giữ API C):
- `gz` (rad/s), `yaw` (**rad**, quấn về [−π, π]), `ax`, `ay`, `az` (m/s²).
- `t_us` (thời điểm nhận mẫu gyro, `esp_timer`), `seq` (sequence của report gyro, byte 1 của report), `n_resets` (số lần chip reset từ lúc khởi động).
- Bỏ cờ `valid` cũ; tính hợp lệ ở lớp hợp nhất (mục 4), không ở driver.

Vòng điều khiển 100 Hz đọc mẫu mới nhất (giữ nguyên giữa hai mẫu) và tuổi mẫu `age = now − t_us`.

## 4. Quy tắc hợp lệ

IMU hợp lệ ở tick hiện tại khi **tất cả** đúng:

| Điều kiện | Ngưỡng |
|---|---|
| Mẫu còn mới: `age ≤ T_age` | `T_age` = 2 chu kỳ report [ĐỀ XUẤT: 100 ms ở 20 Hz] |
| Số thứ tự tăng (không đứng im, không nhảy lùi) | — |
| Không bão hòa: `|gz| < 0,9 × dải đo` | dải đo theo cấu hình chip [ĐỀ XUẤT: dùng `imu_gyro_range` trong `params.py`, kiểm SH-2 manual] |
| Không vừa reset: đã qua `T_settle` kể từ reset complete gần nhất | [ĐỀ XUẤT: 0,5 s] |

Chuyển nguồn có trễ (hysteresis): mất hợp lệ thì **chuyển ngay** sang encoder; có lại hợp lệ thì chỉ quay về IMU sau `N_back` tick liên tục hợp lệ [ĐỀ XUẤT: 10 tick = 100 ms]. Mỗi lần chuyển nguồn, bộ điều khiển wz giữ nguyên trạng thái (không reset tích phân / ESO), để đáp ứng không giật.

**Bias gyro:** chip đã tự bù bias (report calibrated gyro). Firmware ước lượng thêm bias dư chỉ khi xe thật sự đứng yên: setpoint = 0 **và** mọi bánh 0 count trong `T_still` [ĐỀ XUẤT: 1 s]; bias = trung bình gz trong cửa sổ đó, trừ khỏi gz. Không ước lượng khi chỉ có setpoint = 0 mà bánh còn quay.

## 5. Vòng giữ hướng

- Hướng tham chiếu: `ψ_ref` tích phân `wz_ref` mỗi tick (trajectory gửi vx, vy, wz trong hệ thân xe).
- Lệnh wz sau vòng hướng: `wz_cmd = wz_ref + k_ψ · wrap(ψ_ref − ψ_imu)`, giới hạn `|k_ψ · e_ψ| ≤ wz_corr_max`.
- `ψ_imu = yaw − ψ0`, với `ψ0` = yaw lúc bắt đầu chạy (motor enable / bắt đầu quỹ đạo); khi đó `ψ_ref = 0`.
- IMU không hợp lệ: **tắt** vòng hướng (`wz_cmd = wz_ref`), giữ `ψ_ref` tiếp tục tích phân. Có lại hợp lệ: bật lại; nếu `|e_ψ|` khi đó lớn hơn `e_ψ_reset` thì đặt `ψ_ref = ψ_imu` (không quay giật để "đòi" lại hướng) và ghi cờ.
- `k_ψ`, `wz_corr_max`, `e_ψ_reset`: [ĐỀ XUẤT: `k_ψ` vào miền GA (quyết định 12); `wz_corr_max` = 0,5 rad/s; `e_ψ_reset` = 0,35 rad (≈ 20°)].
- Vòng hướng chạy **ngoài** cả PID lẫn LADRC (giữ `vel_ctrl` chung cho hai bộ điều khiển).

## 6. Cờ nghi trượt / nghi bất nhất

- `r = wz_enc − gz` (chỉ khi IMU hợp lệ).
- Bật cờ khi `|r| > r_th` liên tục `T_slip`; tắt khi `|r| < r_th/2` liên tục `T_slip`.
- `r_th` tính từ nhiễu đo: `r_th = k_r × √(σ_gz² + q_wz²)`, với `q_wz` = RMS lượng tử của wz encoder (0,0196 rad/s, `03_CODE/CLAUDE.md`) và `σ_gz` từ phép đo nhiễu IMU (chưa có). [ĐỀ XUẤT: `k_r` = 4, `T_slip` = 200 ms.]
- Cờ chỉ ghi log và báo panel/LED; **không** đổi nguồn wz, không đổi gain.

## 7. Khởi tạo và báo lỗi

- `imu_init()` trả mã lỗi; khởi tạo coi là **đạt** khi trong `T_init` [ĐỀ XUẤT: 2 s] nhận được ít nhất một report gyro hợp lệ (không chỉ dựa vào `sh2_open`, vì R1–R2 ở trên).
- Watchdog khởi tạo: chạy `imu_init` trong task riêng có hạn `T_init`; quá hạn coi là lỗi (tránh treo vì `getProdIds` không timeout).
- Lỗi khởi tạo → không bật motor, ở trạng thái lỗi:
  - xe thật: LED đỏ + còi theo mẫu riêng cho lỗi IMU [ĐỀ XUẤT: 3 tiếng ngắn lặp mỗi 2 s];
  - HIL: log `IMU_INIT_FAIL` qua UART log; RPi5 thấy qua log/status;
  - fwsim: hàm khởi tạo trả lỗi → test fail với thông báo rõ.
- Mất IMU **trong lúc chạy** không phải lỗi dừng: chuyển sang encoder (mục 4), ghi cờ, báo panel.

## 8. Log (thêm cột)

`gz`, `yaw`, `imu_age_ms`, `imu_seq`, `imu_valid`, `wz_src` (0 = encoder, 1 = IMU), `psi_ref`, `e_psi`, `head_on`, `slip_flag`, `imu_resets`. Cột hiện có `gx, gy, gz, yaw, pitch, roll, ax, ay, az` (`main.c:442-450`) giữ nguyên tên nếu được, đổi yaw sang rad thì ghi rõ trong header log.

## 9. Cấu trúc code

- `controller/imu_fusion.c/.h` (**C thuần, không gọi ESP-IDF**, chạy được trong fwsim): kiểm hợp lệ, hysteresis, bias khi đứng yên, cờ trượt.
- `controller/heading.c/.h` (C thuần): vòng giữ hướng.
- `vel_ctrl_step` nhận thêm mẫu IMU (gz, yaw, age, seq) và trả các cột log ở mục 8.
- `imu/imu.cpp`: chỉ đọc chip, gắn `t_us`/`seq`, sửa API semaphore, khởi tạo có hạn.
- `s01_openloop`: thêm `imu_init` và ghi IMU vào log (Liam 22/10). S01 run01 (20/10) không có log IMU; báo cáo ghi rõ.

## 10. Kiểm trên PC (fwsim) trước khi nạp board

fwsim cần mô hình IMU: plant tính gz, yaw, ax, ay từ chuyển động thân xe + nhiễu + bias trôi (seed tái lập), lấy mẫu theo tần số report, cùng mô hình với plant HIL (RPi5).

| Ca | Đạt khi |
|---|---|
| IMU hợp lệ suốt | `wz_src = 1` sau `T_settle`; không cờ trượt (trượt tắt) |
| Mất IMU giữa chừng (bơm lỗi) | Chuyển encoder trong ≤ `T_age` + 1 tick; vòng hướng tắt; xe không giật |
| IMU có lại | Quay về IMU sau `N_back` tick; reset `ψ_ref` nếu lệch lớn |
| Mẫu đứng im (seq không tăng) | Bị loại như mẫu cũ |
| Trượt mô phỏng (khi bật mô hình trượt, S03) | Cờ bật trong `T_slip`; không đổi nguồn wz |
| Khởi tạo IMU lỗi | Không bật motor; báo lỗi |
| 20 Hz so với 100 Hz | Thí nghiệm chọn tần số: so e_wz, e_ψ, độ rung PWM trên nhóm A; chọn 100 Hz chỉ khi cải thiện vượt mức sàn lượng tử |

## 11. Còn mở

- Tần số IMU (mục 1.7).
- Mọi ngưỡng [ĐỀ XUẤT], chờ số đo nhiễu IMU (checklist Ưu tiên 1c, đã hoãn tới khi chốt tần số).
- Định dạng truyền IMU RPi5 → H7 (frame 40 byte hay gửi luân phiên): xem tài liệu G0, mục 5.
