# Spec controller–IMU v2 (CODE soạn 27/10; mới là tài liệu, chưa có code)

*v2 viết 27/10/2026, phiên CODE `local_f031991b…`. Thay v1 (`docs/2026-10-22_Spec_Controller_IMU_nhap.md`, giữ để tra). Đưa vào: 4 quyết định Liam 27/10 (§1b, chốt ở phiên CODE (fork) `local_5d146186…`); các mục review Codex vòng 2 (`01_THESIS/review/2026-10-22_Review_spec_controller_IMU_vong2.md`, ký hiệu C2-#); đề xuất SWA/mất sóng (§7b, **chờ Liam duyệt**). Số ghi **[ĐỀ XUẤT]** dùng tạm (Liam 27/10), chốt sau khi đo nhiễu IMU 20/100 Hz trên xe. Căn cứ code: `AGV_MECANUM` `e3c28fe`, `hil-framework` `g2-bno-h7` `f458ebd`.*

*v2.1 (27/10 khuya): sửa theo bản kiểm O2 lượt 2 của THESIS (`01_THESIS/review/2026-10-27_Kiem_spec_controller_IMU_v2.md`, ký hiệu V#): V1, V3–V6, V9, V11–V14 CODE tự sửa; ~~V2, V7, V8, V10 chờ Liam~~ **v2.2: V2, V7, V8, V10 Liam chốt 27/10 ở phiên THESIS (Liam: "tôi nghiêng theo bạn đấy, chốt 4 V đó cho tôi đi"; `01_THESIS/review/2026-10-27_Kiem_spec_controller_IMU_v2.md` mục "Kết quả", repo gốc `d580a05`)**, ghi tại chỗ. Tên ca thử trên xe đổi thành **X1–X4** (V12) để khỏi trùng "cách A/C" (plant) và "nhóm bài A".*

## 0. Thay đổi v1 → v2

| Mục | Thay đổi | Nguồn |
|---|---|---|
| §1b | 4 quyết định Liam 27/10: công tắc nguồn động cơ, LED trắng + còi 1 dài 5 ngắn, ngưỡng dùng tạm, trượt **A + C** (ban đầu chỉ C, đổi 27/10 khuya); đo μ hiệu dụng | Liam 27/10 |
| §4.1, §3 | Nhận biết chip reset bằng **sự kiện reset** (bộ đếm riêng), bỏ cách "`gyro_n` giảm" (không bao giờ xảy ra); loại mẫu NaN/vô cực | C2-#5 |
| §4.2 | Dải đo: gyro ±2000 °/s; gia tốc chip ±8 g, frame RPi5 → H7 ±4 g (giữ) | C2-#9 |
| §4.4 | Điều kiện đứng yên dùng **gia số count mỗi tick**; tuổi mẫu gia tốc; giới hạn bias; ghi giới hạn còn lại | C2-#7 |
| §5 | Thứ tự ưu tiên khi nhiều sự kiện cùng tick; bật/tắt vòng hướng đo bước PWM như chuyển nguồn | C2-#8 |
| §6, §6.1 | Cờ chỉ xét khi bánh đang quay; ca X2 phải xác nhận bánh quay (tách trượt với kẹt); tách 2 claim về độ trễ; báo nhầm theo thời lượng; trượt **A + C**; `r_true`, ca theo r mục tiêu, cửa sổ đạt có hạn tắt, giới hạn PWM/thời gian ca X1–X2 | C2-#14, Liam 27/10, V1, V3–V6, V12, V14 |
| §7 | Báo lỗi: LED trắng + còi; `IMU READY` chỉ in khi khởi tạo đạt | Liam 27/10, C2-N1 |
| §7b (mới) | Dừng motor: công tắc nguồn (chính) + SWA/mất sóng chỉ bỏ qua ở env bàn HIL | C2-#4, Liam 27/10 (đề xuất, **chờ duyệt**) |
| §8 | Đếm mất log khép kín: số thứ tự dòng, đếm lỗi `sendto`, không reset khi đổi controller, có dòng khi motor tắt; mất report trong thư viện | C2-#13, #15 |
| §10 | fwsim: `v_true` lấy từ trạng thái thân của plant; luật chọn tần số đúng thứ nguyên | C2-#2, #3 |
| §10 (v2.1) | Ca "IMU hợp lệ suốt" ở μ danh định + điều kiện `r_true`; sàn tuyệt đối `ΔPWM_min`; thí nghiệm tần số là dữ liệu phát triển | V1, V9, V11 |
| §11 (mới) | Mục thuộc giả lập H7 (G0): `imu_seq`, dải gia tốc, cận số G3/G4: trạng thái và chỗ xử lý | C2-#9, #10, #12 |

## 1. Đã chốt (Liam 22/10)

1. **IMU vào vòng kín**, không chỉ ghi log. Mọi firmware đều khởi tạo và đọc IMU. Không có cấu hình "không IMU" (QĐ 23).
2. **wz:** lấy hẳn từ gyro IMU khi dữ liệu hợp lệ; không hợp lệ thì lấy wz từ encoder. Không trộn hai nguồn (4.3).
3. **vx, vy:** từ encoder như hiện tại.
4. **Có vòng giữ hướng** dùng yaw IMU.
5. **Cờ nghi trượt / nghi bất nhất:** chỉ báo (log, panel), không tự đổi cách điều khiển, không kết luận nguyên nhân.
6. **IMU lỗi lúc khởi tạo thì dừng và báo**, không âm thầm chạy tiếp.
7. **Tần số IMU** chốt bằng thí nghiệm fwsim (20 Hz so với 100 Hz), sau khi đo nhiễu IMU ở cả 2 tần số (K1). Trong lúc chờ: 20 Hz (`IMU_REPORT_PERIOD_US`, `imu.h`).
8. ~~Trượt: thêm động lực học trượt bánh–thân vào plant (QĐ 25)~~ → **[ĐÃ THAY 27/10, §1b.4]:** ~~trượt cách C (bơm trượt có kiểm soát) + phép thử trên xe~~ **[ĐÃ THAY 27/10 khuya]:** trượt **cách A (mô hình vật lý tối thiểu) + cách C (bơm có kiểm soát)** + phép thử trên xe (§6.1).

### 1b. Liam quyết 27/10

1. **Dừng độc lập cho phép thử trượt ca X1–X2** (v2 gọi là ca A–B): **công tắc nguồn động cơ** trên xe (ngắt phần cứng, không phụ thuộc firmware đang thử). Cách tắt bằng tay điều khiển: Claude gợi ý ở §7b.
2. **Báo lỗi IMU khởi tạo trên xe:** **LED trắng** (LED đỏ đang dùng cho ultrasonic/emergency stop) + còi **1 tiếng dài rồi 5 tiếng ngắn**, lặp lại.
3. **Ngưỡng [ĐỀ XUẤT] dùng tạm**; số chính thức chốt sau khi đo nhiễu IMU 20/100 Hz trên xe.
4. ~~**Trượt cách C:** plant bơm trượt có kiểm soát (tín hiệu thử, không phải mô hình vật lý, không cần kiểm fidelity) + ca A, B trên xe. **Cách A** (mô hình vật lý tối thiểu) xét lại sau G4, trước S03, chỉ làm khi (a) xong trước khi chạy GA / khóa cấu hình và (b) đã có số liệu trượt xe thật (ca C) để kiểm theo QĐ 2; thiếu một điều kiện thì vào "hướng phát triển".~~ **[ĐÃ THAY 27/10 khuya, Liam chốt ở phiên CODE `local_f031991b…`]: làm cả A và C ngay từ đầu.** Lời Liam: "làm cả A và C … tôi sẽ chỉ thay đổi quyết định khi có đủ căn cứ cho thấy tôi không kịp thời gian để làm cả 2". **A** (mô hình vật lý tối thiểu: thân có động lực học riêng M, Iz; mỗi bánh lực kéo bão hòa ở μ·N; tham số μ quét được trong DOE) cho DOE/GA và giữ câu Chương 3 về trượt/ma sát là yếu tố nhiễu; **C** (bơm trượt có kiểm soát, tín hiệu thử) để thử cờ nghi trượt lặp lại được, cộng ca X1, X2 trên xe (v2 gọi là ca A, B). A phải xong **trước** thí nghiệm chọn tần số, DOE/GA và khóa cấu hình (15/11, Outline §4). Kiểm A khi chưa có số liệu trượt xe thật: chỉ tới mức μ lớn cho lại plant hiện tại (phát lại S01, S02 P1), kiểm vật lý, tất định; phần trượt chưa kiểm → luận văn ghi là độ nhạy với mô hình chưa kiểm bằng xe thật. ~~Đo μ tĩnh bánh–sàn (kéo/nghiêng) để có mốc: Claude đề xuất 27/10, chưa chốt.~~ **[ĐÃ THAY 27/10 khuya, Liam duyệt]: đo μ hiệu dụng** bằng cách cho xe tự đẩy vào lực kế: dây nối đuôi xe với điểm cố định phía sau qua lực kế; xe chạy tới, tăng PWM thật chậm tới khi bánh quay trơn; đọc lực lớn nhất ngay trước khi trượt (quay video); 5 lần đi thẳng, 5 lần đi ngang; cân xe; μ_hd = F_max / (m·g). Không dùng phép kéo xe khóa bánh hay mặt nghiêng: con lăn Mecanum tự lăn nên đo ra lực cản lăn. Dừng độc lập: công tắc nguồn động cơ. Kết quả cho A một mốc kiểm (mức lực bắt đầu trượt); động lực sau trượt vẫn chưa kiểm. Liam tự tìm lực kế. Cách lái: **env riêng do CODE viết** (tăng PWM chậm theo nhịp cố định, log PWM + tốc độ bánh); Liam 27/10: **chưa viết bây giờ, viết khi chuẩn bị đo**, Claude nhắc lúc đó. Dữ liệu: `05_DATA/results/MU_FLOOR/`. Dữ kiện khả thi CODE gửi THESIS 27/10: A khoảng 11–19 h [UL]; tải 1 kHz chờ số đo Pi (`rpi5/plant_timing.py`, Liam đo 28/10).

## 2. Hiện trạng code (kiểm 27/10)

| Chỗ | Vấn đề | Trạng thái |
|---|---|---|
| `controller/vel_ctrl.c` | wz đo lấy từ encoder; IMU không tham gia | Chờ làm |
| `imu/imu.cpp:69-96` | Thời điểm `esp_timer` + bộ đếm 32 bit mỗi report, hook cho log | Đã có (22/10) |
| `imu/imu.cpp`, `main.c` | Khởi tạo lỗi chỉ in log rồi chạy tiếp | Chờ làm (§7) |
| `BNO08xSH2HAL.cpp:143` | Sự kiện `SH2_RESET` của chip chỉ đặt bit nội bộ cho thư viện bật lại report; ứng dụng không biết chip đã reset | Chờ làm (§4.1) |
| `src/main.c:255` | `#define HIL_BYPASS_MOTOR_ENABLE` đặt cứng cho **mọi** env, kể cả firmware xe: gạt SWA không tắt motor | Đề xuất §7b, chờ Liam duyệt |
| `ibus/ibus.c:86-97` | Mất byte 20 ms thì `valid = false`; dòng dừng motor khi failsafe còn TODO | Đi cùng §7b |
| `logger/logger.c:41,90,131` | Hàng đợi không chờ, `sendto` không kiểm lỗi, không đếm | §8 |
| `test_imu_log.c:67` | In `IMU READY` cả khi khởi tạo lỗi (C2-N1) | §7 |
| H7 giả lập BNO085 | `hil-framework` `g2-bno-h7`: lõi + SPI4/INT/RST, RPi5 gửi IMU không nhiễu (`rpi5/imu_model.py`) | Đã biên dịch, **chưa nạp, chưa kiểm G2** |

## 3. Dữ liệu IMU đưa vào controller

Từ `imu_get()` (`imu.h`): mỗi report có giá trị, **thời điểm nhận** `*_t_us` (esp_timer) và **bộ đếm** `*_n` (uint32, chỉ tăng ở host).
- Gyro: `gz` (rad/s), `gyro_t_us`, `gyro_n`.
- Hướng: `yaw` (độ trong `imu.h`; lớp hợp nhất đổi sang **rad**, quấn về [−π, π]), `rv_t_us`, `rv_n`.
- Gia tốc: `ax`, `ay`, `az`, `accel_t_us` (chỉ dùng cho điều kiện đứng yên, 4.4).
- **Mới:** `reset_n` (uint32): số lần chip báo `SH2_RESET` kể từ khi khởi động, kèm `reset_t_us`. Tăng ở `BNO08xSH2HAL.cpp:143` qua một hàm của `imu.cpp` (sửa thư viện 1 dòng, ghi trong comment và bug log thư viện).

Vòng điều khiển 100 Hz giữ mẫu gần nhất của từng report (zero-order hold) và tính tuổi riêng: `age_g = now − gyro_t_us`, `age_r = now − rv_t_us`, `age_a = now − accel_t_us`. Thư viện không cho timestamp chip dùng được (`shtp.c:804`), nên tuổi tính từ lúc nhận ở host.

## 4. Quy tắc hợp lệ

### 4.1. Kiểm khi có mẫu mới; nhận biết reset bằng sự kiện

`imu_fusion` nhớ `gyro_n`, `rv_n`, `reset_n` của tick trước. Ở mỗi tick:
- `gyro_n` tăng: kiểm mẫu mới (không NaN/vô cực, không bão hòa). Mẫu hỏng thì gyro **không hợp lệ** tới mẫu tốt kế tiếp. `gyro_n` không đổi: mẫu cũ vẫn dùng **chừng nào tuổi còn trong hạn** (giữ mẫu 20 Hz giữa các tick 100 Hz không phải lỗi).
- `reset_n` tăng: coi như chip reset: cả gyro và yaw không hợp lệ cho tới khi qua `T_settle` và có mẫu mới **sau** `reset_t_us`; vòng hướng xử lý đổi gốc yaw (§5). *(v1 dùng "`gyro_n` giảm": bộ đếm chỉ tăng ở host nên điều kiện đó không bao giờ xảy ra, C2-#5.)*
- Trường trạng thái độ chính xác (accuracy) của report: **ghi log, không dùng để xét hợp lệ** trong v2 (giá trị của chip giả lập H7 là hằng, chưa đối chiếu chip thật; xét lại sau G4).

### 4.2. Điều kiện hợp lệ của từng nguồn

| Điều kiện | Gyro (wz) | Hướng (yaw) | Ngưỡng |
|---|---|---|---|
| Đã nhận ít nhất một mẫu sau lần reset gần nhất | có | có | — |
| Tuổi mẫu | `age_g ≤ T_age` | `age_r ≤ T_age` | `T_age` = 2,5 chu kỳ report [ĐỀ XUẤT: 125 ms ở 20 Hz, 25 ms ở 100 Hz] |
| Giá trị hữu hạn | có | có | — |
| Không bão hòa | `|gz| < 0,9 × 2000 °/s` (34,9 rad/s) | — | dải gyro ±2000 °/s (cấu hình hiện tại; kiểm SH-2 manual khi chốt) |
| Không vừa reset | đã qua `T_settle` sau `reset_t_us` | như gyro; gốc yaw đổi nên vòng hướng đặt lại `ψ0` (§5) | `T_settle` [ĐỀ XUẤT: 0,5 s] |

**wz dùng IMU khi gyro hợp lệ. Vòng hướng chạy khi yaw hợp lệ.** Hai điều kiện độc lập.

**Dải gia tốc (C2-#9):** chip để ±8 g (datasheet BNO08X, theo trích của Codex; Claude chưa mở trang); giả lập H7 bão hòa ở ±8 g như chip. Frame RPi5 → H7 giữ ±4 g (độ phân giải 1,2 mm/s² mỗi LSB): xe Mecanum chạy sàn không tới 4 g, và gia tốc chỉ dùng cho điều kiện đứng yên. Ghi rõ trong luận văn là giới hạn của giả lập.

### 4.3. Chuyển nguồn wz không giật (giữ v1)

- Mất hợp lệ → chuyển **ngay** sang encoder. Có lại → chỉ quay về IMU sau khi gyro hợp lệ liên tục `T_back` [ĐỀ XUẤT: 3 mẫu gyro mới].
- **Liam quyết 22/10: KHÔNG bù, chỉ đo.** Mỗi lần chuyển ghi log (thời điểm, chiều, `Δ = wz_mới − wz_cũ`) và đo bước PWM theo tiêu chí §10. Chỉ thêm bù khi đo trên fwsim/HIL thấy vượt ngưỡng đã khóa.
- Tích phân PID và trạng thái ESO **giữ nguyên** khi chuyển.

### 4.4. Bias gyro khi đứng yên (C2-#7)

Chip tự bù bias (calibrated gyro). Firmware ước lượng thêm bias dư **chỉ khi thỏa tất cả** suốt cửa sổ `T_still` [ĐỀ XUẤT: 2 s]:
- setpoint = 0 và motor disable (hoặc PWM = 0 ở cả 4 bánh);
- **gia số count của mỗi bánh trong mỗi tick 10 ms = 0** suốt cửa sổ (không dùng count tích lũy);
- gyro và gia tốc hợp lệ, có mẫu mới trong cửa sổ, `age_a ≤ T_age`;
- `|gz| < g_still` [ĐỀ XUẤT: 3σ nhiễu gyro đo được] và độ lệch chuẩn của gz trong cửa sổ < 2σ;
- `| |a| − g | < a_still` [ĐỀ XUẤT: 3σ nhiễu gia tốc] ở mọi mẫu trong cửa sổ;
- **không** ở chế độ kê bánh (đặt qua cấu hình bài chạy).

Bias = trung bình gz trong cửa sổ; cập nhật bằng lọc chậm; mỗi lần cập nhật đổi không quá `b_step`, tổng `|bias| ≤ b_max` [ĐỀ XUẤT: `b_step` = 0,1σ, `b_max` = 0,02 rad/s; chốt sau đo nhiễu]. Ghi log mỗi lần cập nhật.

**Giới hạn còn lại:** thân xe trôi đều mà bánh không quay và gyro gần 0 (ví dụ bị đẩy thẳng thật chậm, đều) thì các điều kiện trên vẫn đạt. Vì `b_step` nhỏ và chỉ ước lượng bias **quay** (gz), sai lệch do trường hợp này bị chặn bởi `b_max`. Chế độ kê bánh: logic bias không kiểm được.

## 5. Vòng giữ hướng

- `ψ_imu = wrap(yaw − ψ0)`; `ψ0` = yaw lúc **bắt đầu chạy** (motor enable + quỹ đạo VALID), khi đó `ψ_ref = 0`.
- `ψ_ref` tích phân `wz_ref` mỗi tick (trajectory gửi vx, vy, wz trong hệ thân xe).
- `wz_cmd = wz_ref + k_ψ · wrap(ψ_ref − ψ_imu)`, giới hạn `|k_ψ · e_ψ| ≤ wz_corr_max`. Vòng hướng chạy **ngoài** PID và LADRC.

**Thứ tự ưu tiên khi nhiều sự kiện trong cùng một tick (C2-#8):** xét từ trên xuống, áp **sự kiện đầu tiên khớp**, rồi mới tính `wz_cmd`:

| Ưu tiên | Sự kiện | Hành vi |
|---|---|---|
| 1 | Chip reset (`reset_n` tăng) hoặc đổi mode (PID ↔ LADRC) | Tắt vòng hướng; khi yaw hợp lệ lại thì đặt `ψ0` mới, `ψ_ref = 0`; đếm `psi_reset` |
| 2 | STOP / motor disable / quỹ đạo hết VALID | Tắt vòng hướng; `ψ_ref` bám `ψ_imu` |
| 3 | Yaw không hợp lệ | Tắt vòng hướng (`wz_cmd = wz_ref`); `ψ_ref` vẫn tích phân |
| 4 | Yaw có lại, `|e_ψ| > e_ψ_reset` | Đặt `ψ_ref = ψ_imu`, bật lại, đếm `psi_reset` |
| 5 | Yaw có lại, `|e_ψ| ≤ e_ψ_reset` | Bật lại |

- **Bước `wz_cmd` khi bật/tắt vòng hướng:** cùng cách với chuyển nguồn (Liam 22/10): không làm mềm, **ghi log và đo bước PWM** theo tiêu chí §10; chỉ thêm làm mềm khi đo thấy vượt ngưỡng đã khóa. Bước lớn nhất bị chặn bởi `wz_corr_max`.
- `psi_reset` có thể che lệch hướng nhiệm vụ: mọi lần đặt lại được ghi log và báo cùng metric; lệch hướng so với nhiệm vụ đánh giá bằng **phép đo ngoài** (camera, QĐ 8).
- `k_ψ`, `wz_corr_max`, `e_ψ_reset`: [ĐỀ XUẤT: `k_ψ` vào miền GA (QĐ 12); `wz_corr_max` = 0,5 rad/s; `e_ψ_reset` = 0,35 rad ≈ 20°].

## 6. Cờ nghi trượt / nghi bất nhất

- `r = wz_enc − gz`, chỉ xét khi gyro hợp lệ, có mẫu mới **và bánh đang quay**: `max_i |ω_i| ≥ ω_min` [ĐỀ XUẤT: `ω_min` = 2 rad/s, khoảng 4 count mỗi tick] (C2-#14). Bánh đứng yên thì không xét (kẹt cứng: encoder và gyro đều gần 0, cờ không bật là đúng).
- Bật cờ khi `|r| > r_th` liên tục `T_slip`; tắt khi `|r| < r_th/2` liên tục `T_slip`.
- `r_th = k_r × √(σ_gz² + q_wz²)`: `q_wz` = 0,0196 rad/s (RMS lượng tử wz encoder, `03_CODE/CLAUDE.md`), `σ_gz` từ phép đo nhiễu. [ĐỀ XUẤT: `k_r` = 4, `T_slip` = 200 ms.]
- Cờ chỉ ghi log và báo panel/LED; **không** đổi nguồn wz, không đổi gain.

### 6.1. Phép thử cờ trượt

**Trên SIM/HIL: cách C (bơm có kiểm soát) trên nền plant cách A (Liam 27/10 khuya, §1b.4).**

- **Bơm vào đâu (V2, Liam chốt (a)):** **ở đầu vào mô hình gyro**: gyro giả thấy `(1 − s)·wz_thân` trong khoảng bơm, trạng thái plant không đổi, nên `r` biết chính xác và không phụ thuộc A. Luận văn gọi là *controlled IMU–encoder inconsistency injection*; trượt vật lý để cho A. *(Phương án (b) bơm vào trạng thái thân: không chọn.)* Vì bơm nằm ở mô hình IMU, chỗ code là `rpi5/imu_model.py` (dùng chung RPi5 và fwsim), không phải `plant_step.py`. Đây là năng lực của bộ HIL, thuộc claim C7 Ch. 3 (V8, Liam chốt (a); THESIS sửa `.tex` sau khi Liam duyệt câu chữ). Khoảng bơm `[t0, t0 + T_inj]` khóa trước. Python plant (RPi5) và fwsim dùng chung code. C là **tín hiệu thử**, không kiểm fidelity.
- **Trượt thật của plant A (V1):** plant A ghi thêm `r_true = wz_suy_từ_bánh − wz_thân` mỗi bước (từ trạng thái plant). Mỗi lần cờ bật được phân loại **đúng** (`|r_true| ≥ r_th/2`, hoặc đang trong cửa sổ bơm) hay **nhầm**.
- **Đặt ca theo biên độ r mục tiêu (V3):** ca dưới ngưỡng `r ≈ 0,5·r_th`, ca trên ngưỡng `r ≥ 2·r_th`; `s` của từng bài tính sau khi đo nhiễu (K1, ra `σ_gz`), khóa cùng các ngưỡng khác (QĐ 21).

| Ca SIM/HIL | Đạt khi [ĐỀ XUẤT] |
|---|---|
| Bơm trên ngưỡng (`r ≥ 2·r_th`), lệnh quay tại chỗ | Cờ bật trong `T_slip` + 1 chu kỳ report + 1 tick (10 ms) sau `t0` (V4); **tắt trong** `T_slip` + 1 chu kỳ report + 1 tick sau `t0 + T_inj`; nguồn wz không đổi |
| Bơm dưới ngưỡng (`r ≈ 0,5·r_th`) | Cờ không bật (kiểm ngưỡng, không phải độ nhạy với trượt thật) |
| Không bơm, nhóm bài A, **μ danh định** | Lượt chỉ tính khi `|r_true| < r_th/2` suốt lượt (V1); khi đó 0 lần bật cờ. Lượt có `|r_true|` vượt ngưỡng là ca trượt thật của plant: báo riêng, không tính là báo nhầm |

**Độ trễ, hai claim tách riêng (V5):** (1) đo **từ `t0` bơm tới lúc cờ bật trên fwsim** (một đồng hồ, biết chính xác `t0`). Trên HIL, `t0` ở đồng hồ RPi5 còn cờ ở log ESP32 (WiFi/UDP): chỉ báo "cờ bật trong cửa sổ", trừ khi có cách ghép thời gian (ví dụ mang số frame SPI vào log ESP32), chưa làm. (2) Trên xe chỉ báo "cờ bật trong lượt" và thời điểm bật, **không** claim độ trễ từ sự kiện vật lý.

**Trên xe thật** (dừng độc lập: công tắc nguồn động cơ, §1b.1; thêm SWA khi §7b được duyệt). Tên ca đổi thành X1–X4 (V12).

| Ca | Cách làm | Kỳ vọng | N |
|---|---|---|---|
| X1. Kê bánh hoàn toàn | Xe trên giá, lệnh quay tại chỗ: bánh quay, thân đứng yên | Cờ bật trong lượt | 5 |
| X2. Giữ thân xe | Trên sàn, tốc độ thấp, chặn thân vào vật cố định rồi lệnh quay. **Hợp lệ chỉ khi log cho thấy bánh quay** (`max |ω| ≥ ω_min`); bánh không quay là kẹt, ghi riêng | Cờ bật ở các lượt hợp lệ | 5 |
| X3. Mặt trơn dưới 1–2 bánh | Tấm mica/giấy nến dưới bánh, đi thẳng hoặc quay | Mô tả: ghi `r` và trạng thái cờ, không đặt đạt/không đạt. **Dùng X3 để đối chiếu hành vi trượt của plant A (V7, Liam chốt có):** log đủ lệnh để phát lại trong fwsim với A; **đo thêm μ_hd trên mặt trơn** bằng cùng cách lực kế, cùng buổi đo μ sàn (§1b.4); so `r` đo với `r` mô phỏng, chỉ mô tả; xếp lịch **trước GA/khóa cấu hình 15/11** | 5 |
| X4. Chạy bình thường | Nhóm bài A trên sàn | Báo **số lần bật cờ trên tổng thời gian chạy**; 0 lần không có nghĩa xác suất bằng 0 | theo S02 |

- **N = 5 (V14):** chỉ báo "k/5 lượt", không suy ra xác suất phát hiện.
- **X1, X2: tích phân dồn (V6).** Gyro vẫn hợp lệ nên wz đo ≈ 0 trong khi `wz_ref ≠ 0`: vòng wz và vòng hướng tích phân dồn, PWM lên tới bão hòa (bánh kê quay hết tốc, hoặc motor đẩy mạnh vào vật chặn). Đây là hành vi thật của controller, được ghi nhận, nhưng có rủi ro an toàn và nóng motor. Giới hạn trong bài chạy: |PWM| ≤ `PWM_X12` và thời gian lệnh quay ≤ `T_X12` [ĐỀ XUẤT: 0,5 và 3 s; Liam duyệt khi viết env đo]; ghi PWM lớn nhất mỗi lượt. Công tắc nguồn vẫn là dừng chính.

## 7. Khởi tạo và báo lỗi

- `imu_init()` trả mã lỗi. Khởi tạo **đạt** khi trong `T_init` [ĐỀ XUẤT: 2 s] nhận được ít nhất `N_init` report **gyro và game RV** [ĐỀ XUẤT: 3 mỗi loại], không chỉ dựa vào `sh2_open` (thư viện có thể báo thành công giả hoặc treo: G0 v1 R1, R2, R10).
- Watchdog: `imu_init` chạy trong task riêng có hạn `T_init`; quá hạn là lỗi.
- Lỗi khởi tạo → không bật motor, ở trạng thái lỗi:
  - **xe thật (Liam 27/10):** LED RGB (GPIO48) **trắng** + còi (GPIO17) **1 tiếng dài rồi 5 tiếng ngắn**, lặp [ĐỀ XUẤT: dài 600 ms, nghỉ 200 ms, ngắn 120 ms bật / 120 ms tắt, lặp mỗi 3 s];
  - HIL: log `IMU_INIT_FAIL`; RPi5 thấy qua log/status;
  - fwsim: hàm khởi tạo trả lỗi → test fail với thông báo rõ;
  - env đo `imu_log_*`: chỉ in `IMU READY` khi đạt, lỗi thì in `IMU INIT FAIL <mã>` (C2-N1).
- Mất IMU **trong lúc chạy** không phải lỗi dừng: chuyển encoder (4.3), tắt vòng hướng (§5), ghi cờ, báo panel.

### 7b. Dừng motor bằng tay điều khiển (**Liam duyệt 28/10**, đã làm: AGV commit kế tiếp, env `hil_bench`; chưa nạp)

Liam mở cho gợi ý (§1b.1). Hiện trạng: `main.c:255` đặt cứng `HIL_BYPASS_MOTOR_ENABLE` cho mọi env, nên `swa_on = true` luôn; khi bỏ cờ này, code sẵn có đã làm đúng: `swa_on = has_rc && SWA > 1500`, và `ibus.c` đặt `valid = false` khi 20 ms không có byte, nên mất sóng cũng tắt motor ở tick điều khiển kế tiếp.

Đề xuất:
1. Bỏ `#define` trong `main.c`; cờ `HIL_BYPASS_MOTOR_ENABLE` chỉ có ở **env bàn HIL** mới (`hil_bench`, kế thừa env chính; bàn HIL không có bộ thu RC). Firmware xe (env chính) dùng SWA.
2. Mất sóng: dựa vào `valid = false` như trên; dòng TODO trong `ibus.c` thay bằng ghi log `RC_LOST` (dừng đã do vòng SWA làm).
3. **Liam cần kiểm (chưa có trong file, Claude không đoán):** khi tắt tay điều khiển, bộ thu còn phát gói iBUS (giá trị failsafe) không. Nếu còn, cần đặt failsafe của kênh 7 (SWA) trên tay điều khiển về vị trí tắt, nếu không mất sóng sẽ không tắt motor.
4. Công tắc nguồn động cơ vẫn là **dừng độc lập chính** cho ca X1–X2, vì SWA vẫn đi qua firmware đang thử.

Hệ quả: nạp firmware xe thì phải bật SWA mới chạy motor; nạp bàn HIL dùng env `hil_bench`. Checklist và hướng dẫn nạp phải ghi env nào cho board nào.

## 8. Log và metric

**Cột thêm (100 Hz):** `wz_enc`, `gz`, `yaw` (rad), `age_g_ms`, `age_r_ms`, `gyro_n`, `rv_n`, `reset_n`, `g_valid`, `r_valid`, `wz_src` (0 = encoder, 1 = IMU), `psi_ref`, `e_psi`, `head_on`, `slip_flag`, `r`, `psi_reset_count`, `bias_gz`, `log_seq`, `log_qdrop`, `log_send_err`. Giữ tên cột cũ; đơn vị yaw ghi trong header log.

**Metric (giữ v1):**
- Khi điều khiển theo gyro, e_wz tính từ chính gz không còn khách quan: SIM/HIL so với giá trị thật của plant; xe thật so với camera (QĐ 7, 8). Log luôn có cả `wz_enc` lẫn `gz`.
- Mỗi lượt báo % tick `wz_src = 1`, % tick vòng hướng chạy, số `psi_reset`, số lần chuyển nguồn và bước PWM lớn nhất. Lượt mất IMU là **kết quả**, không phải dữ liệu để loại.
- Mức sàn lượng tử của e_wz: encoder 0,024 rad/s mỗi count; gyro Q9 1/512 ≈ 0,002 rad/s.

**Mất log, khép kín (C2-#13):**
- `log_seq`: số thứ tự dòng tăng từ lúc khởi động, **không reset** khi đổi controller hay bật/tắt motor; PC thấy được dòng mất ở giữa.
- Khi motor tắt vẫn gửi dòng nhịp thưa [ĐỀ XUẤT: 10 Hz] mang `log_seq`, để thấy mất gói đầu và cuối lượt.
- `log_qdrop` (hàng đợi đầy) và `log_send_err` (`sendto` trả lỗi) là bộ đếm tích lũy, có trong mọi dòng.
- Kết thúc lượt: dòng tổng kết lặp 3 lần (tổng `log_seq`, `log_qdrop`, `log_send_err`).
- Ngưỡng loại lượt do mất log khóa trước pha đo chính (QĐ 21).

**Mất report trong thư viện (C2-#15):** hàng đợi event của thư viện gửi không chờ; report mất trước callback thì `*_n` không thấy. Cách phát hiện: so số report nhận với số kỳ vọng theo chu kỳ (tỷ lệ report, khoảng trống lớn nhất), như `tools/g2_bno_check.py` và `imu_capture.py` đang làm. Ghi là giới hạn: không đếm trực tiếp được.

## 9. Cấu trúc code

- `controller/imu_fusion.c/.h` (**C thuần**, chạy được trong fwsim): kiểm hợp lệ theo mẫu mới và sự kiện reset, chuyển nguồn, bias khi đứng yên, cờ trượt.
- `controller/heading.c/.h` (C thuần): vòng giữ hướng, bảng ưu tiên §5.
- `vel_ctrl_step` nhận thêm mẫu IMU (giá trị, `*_t_us`, `*_n`, `reset_n`) và trả các cột log §8.
- `imu/imu.cpp`: đọc chip, gắn thời điểm/bộ đếm (đã có), khởi tạo có hạn (§7), bộ đếm reset (§3).
- Thư viện: thêm 1 lời gọi tăng `reset_n` ở `BNO08xSH2HAL.cpp:143`. Lõi giả lập `bnoemu_pc` (G1) không biên dịch file này nên không ảnh hưởng.
- `led_buzzer`: mẫu báo lỗi IMU (§7).
- `platformio.ini`: env `hil_bench` (§7b, nếu Liam duyệt).
- `s01_openloop`: thêm `imu_init` và ghi IMU vào log (Liam 22/10).

## 10. Kiểm trên PC (fwsim) trước khi nạp board

**fwsim (C2-#2):** `v_true` hiện tính từ tốc độ bánh (`fwsim.py:179`, `H_fwd @ x[6:10]`). Đổi thành vận tốc thân của plant: `[vx, vy]` thân = quay `[x[3], x[4]]` (hệ thế giới) theo −θ, `wz` = `x[5]`. Ở chế độ kê bánh (`plant_step.py` đặt thân = 0) và khi bơm trượt, giá trị thật mới đúng. IMU trong fwsim dùng chung `rpi5/imu_model.py` (G2: không nhiễu; nhiễu và bias có seed thêm ở G3 sau khi đo).

| Ca | Đạt khi [ĐỀ XUẤT] |
|---|---|
| IMU hợp lệ suốt (μ danh định) | `wz_src = 1` ở ≥ 99 % tick sau `T_settle`; không cờ trượt, với điều kiện `|r_true| < r_th/2` suốt lượt (V1) |
| Giữ mẫu 20 Hz giữa tick 100 Hz | Không tick nào mất hợp lệ chỉ vì chưa có mẫu mới |
| Mất gyro giữa chừng (bơm lỗi) | Chuyển encoder trong ≤ `T_age` + 1 tick |
| Chuyển nguồn, bật/tắt vòng hướng | max \|ΔPWM\| một tick trong ±0,2 s quanh sự kiện ≤ max(k_sw × max \|ΔPWM\| cùng cửa sổ ở lượt không có sự kiện (cùng bài, cùng seed), `ΔPWM_min`) [ĐỀ XUẤT: k_sw = 1,5; `ΔPWM_min` = vài bước PWM, chốt khi viết test, khóa theo QĐ 21] (V11); ghi cả max \|Δwz_cmd\| |
| Mất yaw, gyro còn | wz vẫn từ IMU; vòng hướng tắt rồi bật lại đúng bảng §5 |
| Chip reset giữa chừng (sự kiện reset) | Đặt lại `ψ0`, đếm `psi_reset`, không quay giật |
| Mẫu NaN | Gyro không hợp lệ tới mẫu tốt kế tiếp |
| Bơm trượt (§6.1) | Như bảng §6.1 |
| Khởi tạo IMU lỗi | Không bật motor; báo lỗi |

**Thí nghiệm chọn tần số (C2-#3: sửa thứ nguyên và luật "hoặc").** Protocol khóa trước khi chạy: đầu vào nhiễu đo ở từng tần số; cùng gain khởi đầu, cùng seed {0..4}; tập bài nhóm A + bài quay (A-F2) + bài đổi wz nhanh. Metric so với giá trị thật của plant: `E_wz` = RMS e_wz (rad/s), `E_ψ` = RMS e_ψ (rad), `R_pwm` = độ rung PWM. Hiệu từng cặp seed: `d_X = X(20 Hz) − X(100 Hz)`. **Dữ liệu phát triển (V9):** các lượt của thí nghiệm này là dữ liệu phát triển; tập kiểm cuối khóa riêng, không trùng lượt (Ch. 3). Tần số chọn ở **gain khởi đầu**, trước GA: ghi là giới hạn; có thể chạy lại so sánh ở gain đã chọn nhưng chỉ để mô tả, không đổi tần số đã khóa. **δ (V10, Liam chốt (b)):** `δ = max(độ phân giải, một phần nhỏ của mục tiêu nhiệm vụ)`, mục tiêu lấy từ đặc tính H ở Ch. 4.2; tỉ lệ "một phần nhỏ" chốt cùng mục tiêu ở Ch. 4.2 (chưa chốt). **Lối thoát:** tới lúc chạy thí nghiệm mà Ch. 4 chưa chốt mục tiêu thì dùng δ theo độ phân giải ở trên và ghi thành giới hạn.

Chọn **100 Hz** chỉ khi đủ cả ba:
1. ít nhất một trong hai **cải thiện thật**: `mean(d_wz) > max(δ_wz, 2·SE(d_wz))` hoặc `mean(d_ψ) > max(δ_ψ, 2·SE(d_ψ))`;
2. cái còn lại **không xấu đi** quá ngưỡng của chính nó: `mean(d) ≥ −max(δ, 2·SE(d))`;
3. `R_pwm` không tăng quá [ĐỀ XUẤT: 10 %].

Mỗi metric so với ngưỡng **cùng thứ nguyên**: `δ_wz` = 1/512 rad/s (bước gyro Q9), `δ_ψ` = bước yaw của game RV [ĐỀ XUẤT: tính từ Q14, khoảng 1,2·10⁻⁴ rad; chốt khi đo]. Ngược lại giữ 20 Hz.

## 11. Mục thuộc giả lập H7 (G0), trạng thái 27/10

| Mục | Trạng thái | Chỗ xử lý |
|---|---|---|
| `imu_seq` 8 bit (C2-#10) | Đã làm ở `g2-bno-h7`: RPi5 tăng 1..255 mỗi frame (bỏ 0); H7 kiểm thay đổi mỗi tick 100 µs, nên quay vòng 256 ms không ảnh hưởng phát hiện đóng băng; plant ngừng thì H7 ngừng report sau `stale_periods` = 3 chu kỳ report | Kiểm ở G2 |
| Dải gia tốc ±8 g (C2-#9) | Lõi giả lập bão hòa ±8 g; frame ±4 g (§4.2) | Ghi giới hạn |
| Cận số cho G3, G4 (C2-#12) | Chưa có | Sửa G0 v2 (`hil-framework/docs`) sau G2, cùng số liệu G2 |
| Bản ghi UART S1–S5 (C2-#11) | Env ghi UART trên xe chưa viết | Việc CODE sau G2 (cho G4, QĐ 26) |

## 12. Còn mở

- ~~§7b chờ Liam duyệt~~ **Liam duyệt 28/10, đã làm (env `hil_bench`)**; còn Liam kiểm failsafe bộ thu (§7b.3).
- **LED trắng cho lỗi IMU trùng với trạng thái WAITING** (`main.c:150-156`: trắng nháy + bíp ngắn mỗi giây): chờ Liam chọn lại màu/kiểu (phát hiện 28/10).
- Mọi ngưỡng [ĐỀ XUẤT]: dùng tạm, chốt sau đo nhiễu IMU 20/100 Hz (env `imu_log_20`, `imu_log_100`; checklist).
- Tần số IMU: theo thí nghiệm §10.
- Trượt cách A: **làm từ đầu** cùng C (Liam 27/10 khuya, §1b.4); cách viết A (số bước con, vòng trong không numpy) chờ số đo thời gian plant trên Pi (28/10). Đo μ hiệu dụng: Liam duyệt 27/10 (§1b.4); env lái do CODE viết khi chuẩn bị đo.
- ~~THESIS kiểm lại §6.1 và §10~~ **[XONG 27/10]:** O2 lượt 2 (`01_THESIS/review/2026-10-27_Kiem_spec_controller_IMU_v2.md`); v2.1 đã sửa V1, V3–V6, V9, V11–V14.
- ~~Chờ Liam: V2, V7, V8, V10~~ **[XONG 27/10, v2.2]:** Liam chốt 27/10 ở phiên THESIS (Liam: "tôi nghiêng theo bạn đấy, chốt 4 V đó cho tôi đi"; `01_THESIS/review/2026-10-27_Kiem_spec_controller_IMU_v2.md` mục "Kết quả", repo gốc `d580a05`): V2 (a) bơm ở đầu vào gyro; V7 có (X3 đối chiếu A, đo μ_hd mặt trơn); V8 (a) bơm C vào claim C7; V10 (b) δ theo mục tiêu nhiệm vụ, lối thoát độ phân giải.
- `PWM_X12`, `T_X12` (V6): Liam duyệt khi viết env đo X1–X2.
