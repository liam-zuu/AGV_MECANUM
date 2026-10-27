# Spec controller–IMU (nháp v1, chờ Liam duyệt; THESIS kiểm theo O2)

*v0 viết 22/10/2026, phiên CODE. **v1 cùng ngày** sau bảng kiểm K1–K9 của THESIS (`01_THESIS/review/2026-10-22_Kiem_spec_controller_IMU_v0.md`) và review Codex vòng 1 (`01_THESIS/review/2026-10-22_Review_spec_controller_IMU_vong1.md`, 18 mục). Outline V7.4 quyết định 3, 4, 12, 16, 21, 23, 25. Số ghi **[ĐỀ XUẤT]** chưa khóa: khóa sau khi có số đo nhiễu IMU 20/100 Hz và thí nghiệm tần số trên fwsim, trước pha đo chính (QĐ 21). Căn cứ code: `AGV_MECANUM` `a6644a5` + sửa đo nhiễu 22/10.*

**Thay đổi v0 → v1 (mục review):** kiểm hợp lệ theo **mẫu mới** thay vì mỗi tick, bộ đếm 32 bit (Codex 2); yaw có kiểm hợp lệ riêng, khởi tạo đòi cả gyro lẫn yaw (Codex 4); chuyển nguồn không giật và tiêu chí đo bằng bước PWM (Codex 10, K5); vòng hướng khi STOP/mất setpoint/đổi mode, đếm sự kiện đặt lại ψ_ref (Codex 10, 11); điều kiện đứng yên chặt hơn (Codex 12); log thêm `wz_enc`, tỷ lệ tick dùng IMU (K2, K4, N3); mất log UDP (Codex 9); đo nhiễu 2 tần số trước thí nghiệm tần số và protocol thí nghiệm (K1, Codex 13); phép thử trượt trên xe (K6) và trượt trong plant (QĐ 25).

## 1. Đã chốt (Liam 22/10)

1. **IMU vào vòng kín**, không chỉ ghi log. Mọi firmware đều khởi tạo và đọc IMU. Không có cấu hình "không IMU" (QĐ 23).
2. **wz:** lấy hẳn từ gyro IMU khi dữ liệu hợp lệ; không hợp lệ thì lấy wz từ encoder. Không trộn hai nguồn ở trạng thái xác lập (xem 4.3 về đoạn chuyển tiếp).
3. **vx, vy:** từ encoder như hiện tại.
4. **Có vòng giữ hướng** dùng yaw IMU.
5. **Cờ nghi trượt / nghi bất nhất:** chỉ báo (log, panel), không tự đổi cách điều khiển, không kết luận nguyên nhân.
6. **IMU lỗi lúc khởi tạo thì dừng và báo**, không âm thầm chạy tiếp.
7. **Tần số IMU** chốt bằng thí nghiệm fwsim (20 Hz so với 100 Hz), **sau khi đo nhiễu IMU ở cả 2 tần số** (K1, Liam 22/10 ở phiên THESIS; thay quyết định "hoãn đo nhiễu" cùng ngày). Trong lúc chờ: 20 Hz (`IMU_REPORT_PERIOD_US`, `imu.h`).
8. **Trượt:** thêm động lực học trượt bánh–thân vào plant (QĐ 25) để thử cờ trượt trên SIM/HIL; thêm phép thử trượt có kiểm soát trên xe thật, không làm được thì ghi giới hạn (K6).

### 1b. Liam quyết 27/10 (phiên CODE `local_f031991b…`, đưa vào v2)

1. **Dừng độc lập cho phép thử trượt ca A–B (§6.1):** dùng **công tắc nguồn động cơ** trên xe (ngắt phần cứng, không phụ thuộc firmware đang thử). Liam mở cho Claude gợi ý thêm cách tắt bằng tay điều khiển; nếu không được thì công tắc nguồn là đủ.
2. **Báo lỗi IMU khởi tạo trên xe (§7):** **LED trắng** (LED đỏ đang dùng cho ultrasonic/emergency stop) + còi **1 tiếng dài rồi 5 tiếng ngắn**, lặp lại (độ dài tiếng, khoảng lặp: CODE đề xuất khi viết code). Thay đề xuất cũ "LED đỏ + 3 tiếng ngắn mỗi 2 s".
3. **Ngưỡng [ĐỀ XUẤT] trong spec dùng tạm** (`wz_corr_max`, `e_ψ_reset`, `k_r`, `T_slip`, `T_init`, `N_init`…); số chính thức chốt sau khi đo nhiễu IMU 20/100 Hz trên xe.
4. Mô hình trượt bánh–thân trong plant (QĐ 25, §6.1): Liam hỏi giải thích, **chưa chốt**.

## 2. Hiện trạng code

| Chỗ | Vấn đề | Trạng thái |
|---|---|---|
| `controller/vel_ctrl.c:41-58` | wz đo lấy từ encoder; IMU không tham gia | Chờ làm |
| `imu/imu.cpp` | `valid` bật một lần; không có thời điểm/bộ đếm mỗi report; `xSemaphoreTakeFromISR` trong task | **Đã sửa 22/10** (thời điểm `esp_timer` + bộ đếm 32 bit cho từng report, API semaphore đúng, chu kỳ qua `IMU_REPORT_PERIOD_US`); `valid` cũ giữ cho log, không dùng làm kiểm hợp lệ |
| `imu/imu.h` | yaw tính bằng độ | Đổi sang rad ở lớp hợp nhất (mục 3) |
| `imu/imu.cpp:40-43`, `main.c:483` | Khởi tạo lỗi chỉ in log rồi chạy tiếp | Chờ làm (mục 7) |
| Thư viện (`sh2.c`, `shtp.c`, `BNO08xSH2HAL.cpp`) | `getProdIds` không timeout; `sh2_open` báo thành công dù không có reset complete; `txProcess` có thể lặp mãi; timestamp host = 0; lỗi biên bộ đệm và CS | Ghi bug log M7; xử lý ở mục 7 (watchdog) và G0 v1 R1–R2, R9–R11 |
| `logger/logger.c:41,48,90,131` | Log qua WiFi/UDP: hàng đợi không chờ (`xQueueSend(…, 0)`), `sendto` có thể mất gói, không đếm | Mục 8 |

## 3. Dữ liệu IMU đưa vào controller

Từ `imu_get()` (`imu.h`): mỗi report có giá trị, **thời điểm nhận** `*_t_us` (esp_timer) và **bộ đếm** `*_n` (uint32, không quay vòng trong thời gian chạy thực tế).
- Gyro: `gz` (rad/s), `gyro_t_us`, `gyro_n`.
- Hướng: `yaw` (độ trong `imu.h`; lớp hợp nhất đổi sang **rad**, quấn về [−π, π]), `rv_t_us`, `rv_n`.
- Gia tốc: `ax`, `ay`, `az` (chỉ dùng cho điều kiện đứng yên, mục 4.4).

Vòng điều khiển 100 Hz giữ mẫu gần nhất của từng report (zero-order hold) và tính tuổi riêng cho từng report: `age_g = now − gyro_t_us`, `age_r = now − rv_t_us`. Thư viện không cho timestamp chip dùng được (`shtp.c:804`), nên tuổi tính từ lúc nhận ở host; dữ liệu plant cũ trên HIL được H7 xử lý bằng cách ngừng phát (G0 v1 §3.2).

## 4. Quy tắc hợp lệ

### 4.1. Kiểm khi có mẫu mới, không kiểm mỗi tick

`imu_fusion` nhớ `gyro_n` và `rv_n` của lần trước. Ở mỗi tick:
- Nếu `gyro_n` tăng (có mẫu gyro mới): kiểm mẫu đó (bão hòa, số nhảy lùi). Nếu `gyro_n` không đổi: mẫu cũ vẫn dùng được **chừng nào tuổi còn trong hạn**. Như vậy giữ mẫu 20 Hz giữa các tick 100 Hz không bị coi là lỗi (v0 sai ở chỗ này).
- `gyro_n` giảm (chip/driver khởi động lại) → coi như reset (4.2).

### 4.2. Điều kiện hợp lệ của từng nguồn

| Điều kiện | Gyro (wz) | Hướng (yaw) | Ngưỡng |
|---|---|---|---|
| Đã nhận ít nhất một mẫu sau lần reset gần nhất | có | có | — |
| Tuổi mẫu | `age_g ≤ T_age` | `age_r ≤ T_age` | `T_age` = 2,5 chu kỳ report [ĐỀ XUẤT: 125 ms ở 20 Hz, 25 ms ở 100 Hz] |
| Không bão hòa | `|gz| < 0,9 × dải đo` | — | dải đo theo cấu hình chip, kiểm SH-2 manual [ĐỀ XUẤT] |
| Không vừa reset | đã qua `T_settle` kể từ khi bộ đếm reset hoặc driver báo reset | như gyro; **gốc yaw đổi sau reset** nên vòng hướng đặt lại `ψ0` (mục 5) | `T_settle` [ĐỀ XUẤT: 0,5 s] |

**wz dùng IMU khi gyro hợp lệ. Vòng hướng chạy khi yaw hợp lệ.** Hai điều kiện độc lập: gyro mất thì wz về encoder, vòng hướng vẫn có thể chạy nếu yaw còn hợp lệ, và ngược lại.

### 4.3. Chuyển nguồn wz không giật (Codex 10, K5)

- Mất hợp lệ → chuyển **ngay** sang encoder. Có lại → chỉ quay về IMU sau khi gyro hợp lệ liên tục `T_back` [ĐỀ XUẤT: 3 mẫu gyro mới].
- Lúc chuyển, phép đo nhảy một bước `Δ = wz_mới − wz_cũ`, làm khâu P (và sai số ESO của LADRC) nhảy theo.
- **Liam quyết 22/10 (nói ở phiên THESIS): KHÔNG bù, chỉ đo.** Mỗi lần chuyển nguồn ghi log (thời điểm, chiều chuyển, `Δ`) và đo bước PWM theo tiêu chí §10. Chỉ thêm bù khi đo trên fwsim/HIL thấy bước PWM vượt ngưỡng đã khóa. Lý do: lúc chuyển là lúc gyro vừa bị loại vì có thể sai, mà offset lại giữ giá trị của nó; bù còn thêm trường hợp phải xử lý và trái "không trộn" (1.2). *(Đề xuất v1 "offset tắt dần `Δ·e^{−t/τ_b}`" [ĐÃ THAY 22/10 bởi quyết định này].)*
- Tích phân PID và trạng thái ESO **giữ nguyên** khi chuyển.

### 4.4. Bias gyro khi đứng yên (Codex 12)

Chip tự bù bias (calibrated gyro). Firmware ước lượng thêm bias dư **chỉ khi thỏa tất cả** trong `T_still` [ĐỀ XUẤT: 2 s]:
- setpoint = 0 và motor disable (hoặc PWM = 0);
- mọi bánh 0 count;
- `|gz| < g_still` [ĐỀ XUẤT: 3σ của nhiễu gyro đo được] và độ lệch chuẩn của gz trong cửa sổ < 2σ;
- `| |a| − g | < a_still` [ĐỀ XUẤT: 3σ của nhiễu gia tốc];
- **không** ở chế độ kê bánh (đặt qua cấu hình bài chạy). Ở chế độ kê bánh, logic bias không kiểm được: ghi giới hạn.

Bias = trung bình gz trong cửa sổ; cập nhật bằng lọc chậm, giới hạn biên độ. Bị đẩy, trượt, mất encoder thì ít nhất một điều kiện trên hỏng nên không cập nhật.

## 5. Vòng giữ hướng

- `ψ_imu = wrap(yaw − ψ0)`; `ψ0` = yaw lúc **bắt đầu chạy** (motor enable + quỹ đạo VALID), khi đó `ψ_ref = 0`.
- `ψ_ref` tích phân `wz_ref` mỗi tick (trajectory gửi vx, vy, wz trong hệ thân xe).
- `wz_cmd = wz_ref + k_ψ · wrap(ψ_ref − ψ_imu)`, giới hạn `|k_ψ · e_ψ| ≤ wz_corr_max`. Vòng hướng chạy **ngoài** PID và LADRC (dùng chung cho hai bộ điều khiển).
- Trạng thái khác:

| Tình huống | Hành vi |
|---|---|
| Yaw không hợp lệ | Tắt vòng hướng (`wz_cmd = wz_ref`), `ψ_ref` vẫn tích phân |
| Yaw có lại, `|e_ψ| ≤ e_ψ_reset` | Bật lại |
| Yaw có lại, `|e_ψ| > e_ψ_reset` | Đặt `ψ_ref = ψ_imu`, **đếm sự kiện `psi_reset`** |
| STOP / motor disable / quỹ đạo hết VALID (mất setpoint) | Tắt vòng hướng; `ψ_ref` bám `ψ_imu` |
| Đổi mode (PID ↔ LADRC) hoặc chip reset (đổi gốc yaw) | Đặt lại `ψ0`, `ψ_ref = 0`, đếm `psi_reset` |

- **`psi_reset` có thể che lệch hướng nhiệm vụ (Codex 11):** mọi lần đặt lại được ghi log và báo cùng metric của lượt; lệch hướng so với nhiệm vụ đánh giá bằng **phép đo ngoài** (camera, QĐ 8), không bằng `e_ψ` nội bộ.
- `k_ψ`, `wz_corr_max`, `e_ψ_reset`: [ĐỀ XUẤT: `k_ψ` vào miền GA (QĐ 12); `wz_corr_max` = 0,5 rad/s; `e_ψ_reset` = 0,35 rad ≈ 20°]. Miền GA và số tham số mỗi controller ghi ở S04 (đọc `tune_ga.py`).

## 6. Cờ nghi trượt / nghi bất nhất

- `r = wz_enc − gz` (chỉ khi gyro hợp lệ và có mẫu mới).
- Bật cờ khi `|r| > r_th` liên tục `T_slip`; tắt khi `|r| < r_th/2` liên tục `T_slip`.
- `r_th = k_r × √(σ_gz² + q_wz²)`: `q_wz` = RMS lượng tử của wz encoder (0,0196 rad/s, `03_CODE/CLAUDE.md`), `σ_gz` từ phép đo nhiễu IMU ở tần số đã chọn. [ĐỀ XUẤT: `k_r` = 4, `T_slip` = 200 ms.]
- Cờ chỉ ghi log và báo panel/LED; **không** đổi nguồn wz, không đổi gain.

### 6.1. Phép thử cờ trượt

**Trên SIM/HIL:** cần plant có trượt bánh–thân (QĐ 25). Plant hiện chỉ đổi mô-men khi trượt, vận tốc thân vẫn tính từ tốc độ bánh (`plant_step.py:70`, `:85-86`), nên gz luôn khớp encoder và cờ không thể bật. Kế hoạch mô hình trượt trình riêng; Python plant và fwsim dùng cùng mô hình; mô hình mới phải kiểm lại fidelity bằng dữ liệu (QĐ 2).

**Trên xe thật (K6), đề xuất, chờ THESIS kiểm và Liam chốt:**

| Ca | Cách làm | Kỳ vọng | N |
|---|---|---|---|
| A. Kê bánh hoàn toàn | Xe trên giá, lệnh quay tại chỗ: bánh quay, thân đứng yên | Cờ bật trong `T_slip` + 1 mẫu gyro | 5 |
| B. Giữ thân xe | Trên sàn, tốc độ thấp, chặn thân xe vào vật cố định rồi lệnh quay | Cờ bật | 5 |
| C. Mặt trơn dưới 1–2 bánh | Tấm mica/giấy nến dưới bánh, đi thẳng hoặc quay | Mô tả: ghi `r` và trạng thái cờ, không đặt đạt/không đạt | 5 |
| D. Chạy bình thường | Nhóm bài A trên sàn | Không có lần bật cờ nào (báo nhầm) | theo S02 |

Ca A–B dùng gyro thật và encoder thật nên kiểm được đường phát hiện; không chứng minh độ nhạy với trượt một phần (ca C chỉ mô tả). Không làm được ca nào thì ghi giới hạn.

## 7. Khởi tạo và báo lỗi

- `imu_init()` trả mã lỗi. Khởi tạo **đạt** khi trong `T_init` [ĐỀ XUẤT: 2 s] nhận được ít nhất `N_init` report **gyro và game RV** [ĐỀ XUẤT: 3 mỗi loại], không chỉ dựa vào `sh2_open` (thư viện có thể báo thành công giả hoặc treo: G0 v1 R1, R2, R10).
- Watchdog: chạy `imu_init` trong task riêng có hạn `T_init`; quá hạn là lỗi.
- Lỗi khởi tạo → không bật motor, ở trạng thái lỗi:
  - xe thật: LED đỏ + còi theo mẫu riêng [ĐỀ XUẤT: 3 tiếng ngắn lặp mỗi 2 s];
  - HIL: log `IMU_INIT_FAIL`; RPi5 thấy qua log/status;
  - fwsim: hàm khởi tạo trả lỗi → test fail với thông báo rõ.
- Mất IMU **trong lúc chạy** không phải lỗi dừng: chuyển encoder (4.3), tắt vòng hướng (5), ghi cờ, báo panel.

## 8. Log và metric

**Cột thêm (100 Hz):** `wz_enc` (K2), `gz`, `yaw` (rad), `age_g_ms`, `age_r_ms`, `gyro_n`, `rv_n`, `g_valid`, `r_valid`, `wz_src` (0 = encoder, 1 = IMU), `psi_ref`, `e_psi`, `head_on`, `slip_flag`, `r`, `psi_reset_count`, `imu_resets`, `log_qdrop`. Giữ tên cột cũ `gx, gy, gz, yaw, pitch, roll, ax, ay, az` (`main.c:442-450`); đơn vị yaw ghi trong header log.

**Metric (K2, K4, N3, Codex 11):**
- Khi điều khiển theo gyro, e_wz tính từ chính gz không còn khách quan. Metric quay/hướng: **SIM/HIL so với giá trị thật của plant; xe thật so với camera** (QĐ 7, 8). Log luôn có cả `wz_enc` lẫn `gz` để tính e_wz theo cả hai nguồn.
- Mỗi lượt báo **% tick `wz_src = 1`**, **% tick vòng hướng chạy**, số `psi_reset`. Lượt mất IMU là **kết quả** (báo tỷ lệ thất bại cùng metric), không phải dữ liệu để loại; đặc biệt ở bài bơm lỗi.
- Mức sàn lượng tử của e_wz đổi theo nguồn: encoder 0,024 rad/s mỗi count; gyro Q9 1/512 ≈ 0,002 rad/s (K3). `qrms` là giá trị kỳ vọng có điều kiện, không phải sàn bắt buộc (N2).
- `fwsim.metrics` hiện tính sai số theo vận tốc thật (`fwsim.py:192-195`), còn S02 theo vận tốc firmware: định nghĩa thống nhất ghi ở S02 v1.2 / S04 (THESIS).

**Mất log (Codex 9, K8):** log đi qua WiFi/UDP (`logger.c`), hàng đợi không chờ và `sendto` có thể mất gói. Firmware đếm số dòng bỏ ở hàng đợi (`log_qdrop`) và số lần `sendto` lỗi; phía PC kiểm tick liên tục. Nghiệm thu HIL (R10) và xe: báo số dòng mất mỗi lượt; ngưỡng loại lượt khóa trước (QĐ 21). Băng thông ước lượng khoảng 300 byte × 100 dòng/s ≈ 30 kB/s (ước lượng).

## 9. Cấu trúc code

- `controller/imu_fusion.c/.h` (**C thuần, không gọi ESP-IDF**, chạy được trong fwsim): kiểm hợp lệ theo mẫu mới, chuyển nguồn có bù bước, bias khi đứng yên, cờ trượt.
- `controller/heading.c/.h` (C thuần): vòng giữ hướng và các trạng thái ở mục 5.
- `vel_ctrl_step` nhận thêm mẫu IMU (giá trị, `*_t_us`, `*_n`) và trả các cột log ở mục 8.
- `imu/imu.cpp`: chỉ đọc chip, gắn thời điểm/bộ đếm (đã làm), khởi tạo có hạn (mục 7).
- `s01_openloop`: thêm `imu_init` và ghi IMU vào log (Liam 22/10). S01 run01 (20/10) không có log IMU; báo cáo ghi rõ.

## 10. Kiểm trên PC (fwsim) trước khi nạp board

fwsim cần mô hình IMU dùng chung với plant HIL: gz, yaw, ax, ay, az từ chuyển động thân xe + nhiễu + bias trôi (seed tái lập), lấy mẫu theo tần số report, số nhiễu từ phép đo ở đúng tần số.

| Ca | Đạt khi [ĐỀ XUẤT] |
|---|---|
| IMU hợp lệ suốt | `wz_src = 1` ở ≥ 99 % tick sau `T_settle`; không cờ trượt (trượt tắt) |
| Giữ mẫu 20 Hz giữa tick 100 Hz | Không có tick nào mất hợp lệ chỉ vì chưa có mẫu mới |
| Mất gyro giữa chừng (bơm lỗi) | Chuyển encoder trong ≤ `T_age` + 1 tick |
| Chuyển nguồn (cả hai chiều) | max \|ΔPWM\| một tick trong ±0,2 s quanh lúc chuyển ≤ k_sw × max \|ΔPWM\| cùng cửa sổ ở lượt không chuyển nguồn (cùng bài, cùng seed) [ĐỀ XUẤT: k_sw = 1,5]; ghi cả max \|Δwz_cmd\| |
| Mất yaw, gyro còn | wz vẫn từ IMU; vòng hướng tắt rồi bật lại đúng bảng mục 5 |
| Chip reset giữa chừng | Đặt lại `ψ0`, đếm `psi_reset`, không quay giật |
| Trượt (khi plant có trượt, QĐ 25) | Cờ bật trong `T_slip`; không đổi nguồn wz |
| Khởi tạo IMU lỗi | Không bật motor; báo lỗi |

**Thí nghiệm chọn tần số (Codex 13), protocol khóa trước khi chạy:** đầu vào nhiễu đo ở từng tần số; cùng gain (gain khởi đầu, chưa tune), cùng seed {0..4}; tập bài nhóm A + bài quay (A-F2) + bài có thay đổi wz nhanh; metric: e_wz và e_ψ so với giá trị thật của plant, độ rung PWM, % tick dùng IMU; trễ/lọc của chip ở 2 tần số lấy từ phép đo (độ trễ không đo được thì ghi giả định). Luật quyết: chọn 100 Hz chỉ khi e_wz hoặc e_ψ giảm vượt mức sàn lượng tử của gyro và độ rung PWM không tăng quá [ĐỀ XUẤT: 10 %]; ngược lại giữ 20 Hz.

## 11. Còn mở

- ~~Cách chuyển nguồn có bù bước tắt dần (4.3)~~: Liam quyết 22/10 không bù, chỉ đo (xem 4.3).
- Các mục review Codex vòng 2 (`01_THESIS/review/2026-10-22_Review_spec_controller_IMU_vong2.md`) đưa vào v2: `v_true` fwsim, luật chọn tần số (thứ nguyên), dừng độc lập cho ca A–B, sự kiện reset chip, dải gia tốc ±8 g, cận số G3/G4, đếm mất log, `imu_seq`.
- Mọi ngưỡng [ĐỀ XUẤT]: chờ số đo nhiễu IMU 20/100 Hz (env `imu_log_20`, `imu_log_100`; checklist Ưu tiên 1c).
- Tần số IMU: theo thí nghiệm mục 10.
- Mô hình trượt bánh–thân trong plant (QĐ 25): kế hoạch riêng.
- Phép thử trượt trên xe (6.1): chờ THESIS kiểm, Liam chốt.
- Định dạng IMU RPi5 → H7: frame 40 byte (Liam chốt 22/10), bố cục ở G0 v1 §5.
