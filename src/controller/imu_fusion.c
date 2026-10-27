#include "imu_fusion.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265f

enum { CLS_NONE = 0, CLS_ABOVE, CLS_BELOW, CLS_BAND };

static imu_fusion_params_t P = IMU_FUSION_PARAMS_DEFAULT;

static struct {
    bool     have_base;          // đã lấy mốc bộ đếm ở lần gọi đầu có imu
    uint32_t gyro_n, rv_n, accel_n, reset_n;
    bool     reset_seen;         // đã có ít nhất một sự kiện reset (kể cả lúc khởi động)
    int64_t  reset_t_us;
    bool     gyro_bad, rv_bad;   // mẫu gần nhất không dùng được (NaN/vô cực/bão hòa)
    float    gz_raw, yaw;        // mẫu tốt gần nhất
    int      good_run;           // số mẫu gyro tốt liên tiếp khi đang hợp lệ
    int      wz_src;
    uint32_t switch_count;
    float    switch_dwz;

    // Cờ trượt
    int      cls;
    float    r;
    bool     slip;
    int64_t  above_since, below_since;   // −1: chưa chạy
    uint32_t slip_on_count;

    // Bias khi đứng yên
    float    bias;
    uint32_t bias_updates;
    bool     still_open;
    int64_t  still_t0;
    int      still_n;
    double   still_sum, still_sumsq;
    bool     still_accel_seen;
    bool     count_prev_ok;
    int32_t  count_prev[4];
} S;

static float wrap_pi(float a) {
    while (a >  PI_F) a -= 2.0f * PI_F;
    while (a < -PI_F) a += 2.0f * PI_F;
    return a;
}

void imu_fusion_init(const imu_fusion_params_t *p) {
    if (p) P = *p;
    else   P = (imu_fusion_params_t)IMU_FUSION_PARAMS_DEFAULT;
    memset(&S, 0, sizeof(S));
    S.wz_src      = WZ_SRC_ENC;
    S.above_since = -1;
    S.below_since = -1;
}

imu_fusion_params_t *imu_fusion_params(void) { return &P; }

static void still_close(void) {
    S.still_open = false;
}

// §4.4: cửa sổ đứng yên; chỉ cập nhật bias khi đạt mọi điều kiện suốt T_still
static void still_step(const imu_fusion_in_t *in, bool g_valid, bool gyro_new, bool accel_new) {
    const imu_data_t *d = in->imu;

    bool counts_still = S.count_prev_ok;
    for (int i = 0; i < 4; i++) {
        if (in->enc_count[i] != S.count_prev[i]) counts_still = false;
        S.count_prev[i] = in->enc_count[i];
    }
    S.count_prev_ok = true;

    bool accel_ok = d && d->accel_n > 0 && (in->now_us - d->accel_t_us) <= P.t_age_us;
    bool cond = P.sigma_gz > 0.0f && P.sigma_a > 0.0f && !in->wheels_lifted &&
                in->sp_zero && in->drive_off && counts_still && g_valid && accel_ok;

    if (cond && gyro_new && !(fabsf(S.gz_raw) < 3.0f * P.sigma_gz)) cond = false;
    if (cond && accel_new) {
        float an = sqrtf(d->accel_x * d->accel_x + d->accel_y * d->accel_y + d->accel_z * d->accel_z);
        if (!(fabsf(an - P.g_ref) < 3.0f * P.sigma_a)) cond = false;
    }
    if (!cond) { still_close(); return; }

    if (!S.still_open) {
        S.still_open = true;
        S.still_t0 = in->now_us;
        S.still_n = 0;
        S.still_sum = S.still_sumsq = 0.0;
        S.still_accel_seen = false;
    }
    if (gyro_new) {
        S.still_n++;
        S.still_sum   += S.gz_raw;
        S.still_sumsq += (double)S.gz_raw * S.gz_raw;
    }
    if (accel_new) S.still_accel_seen = true;

    if (in->now_us - S.still_t0 < P.t_still_us) return;

    if (S.still_n >= 2 && S.still_accel_seen) {
        double mean = S.still_sum / S.still_n;
        double var  = S.still_sumsq / S.still_n - mean * mean;
        double sd   = var > 0.0 ? sqrt(var) : 0.0;
        if (sd < 2.0 * P.sigma_gz) {
            // Lọc chậm: mỗi lần đổi không quá b_step = 0,1σ; tổng |bias| ≤ b_max
            float b_step = 0.1f * P.sigma_gz;
            float step = (float)mean - S.bias;
            if (step >  b_step) step =  b_step;
            if (step < -b_step) step = -b_step;
            S.bias += step;
            if (S.bias >  P.b_max) S.bias =  P.b_max;
            if (S.bias < -P.b_max) S.bias = -P.b_max;
            S.bias_updates++;
        }
    }
    still_close();   // cửa sổ kế tiếp bắt đầu ở tick sau
}

// §6: cờ nghi trượt / nghi bất nhất, timer theo thời điểm tick
static void slip_step(const imu_fusion_in_t *in, bool g_valid, bool gyro_new, float gz) {
    float wmax = 0.0f;
    for (int i = 0; i < 4; i++) {
        float w = fabsf(in->omega[i]);
        if (w > wmax) wmax = w;
    }
    bool evaluable = g_valid && wmax >= P.omega_min;
    float r_th = P.k_r * sqrtf(P.sigma_gz * P.sigma_gz + P.q_wz * P.q_wz);

    if (!evaluable) {
        S.cls = CLS_NONE;
        S.above_since = -1;
        if (S.slip) {
            if (S.below_since < 0) S.below_since = in->now_us;
            if (in->now_us - S.below_since >= P.t_slip_us) { S.slip = false; S.below_since = -1; }
        } else {
            S.below_since = -1;
        }
        return;
    }

    if (gyro_new) {
        S.r = in->wz_enc - gz;
        float ar = fabsf(S.r);
        S.cls = ar > r_th ? CLS_ABOVE : ar < 0.5f * r_th ? CLS_BELOW : CLS_BAND;
    }

    switch (S.cls) {
    case CLS_ABOVE:
        S.below_since = -1;
        if (S.above_since < 0) S.above_since = in->now_us;
        if (!S.slip && in->now_us - S.above_since >= P.t_slip_us) { S.slip = true; S.slip_on_count++; }
        break;
    case CLS_BELOW:
        S.above_since = -1;
        if (S.below_since < 0) S.below_since = in->now_us;
        if (S.slip && in->now_us - S.below_since >= P.t_slip_us) S.slip = false;
        break;
    case CLS_BAND:
        S.above_since = S.below_since = -1;
        break;
    default:
        break;
    }
}

void imu_fusion_step(const imu_fusion_in_t *in, imu_fusion_out_t *out) {
    const imu_data_t *d = in->imu;
    bool reset_event = false, gyro_new = false, rv_new = false, accel_new = false;

    if (d && !S.have_base) {
        // Mốc: reset lúc khởi động không phải sự kiện, nhưng vẫn tính T_settle từ đó
        S.have_base  = true;
        S.reset_n    = d->reset_n;
        S.reset_seen = d->reset_n > 0;
        S.reset_t_us = d->reset_t_us;
    }

    if (d) {
        if (d->reset_n != S.reset_n) {
            reset_event  = true;
            S.reset_n    = d->reset_n;
            S.reset_seen = true;
            S.reset_t_us = d->reset_t_us;
            S.good_run   = 0;
        }
        if (d->gyro_n != S.gyro_n) {
            S.gyro_n = d->gyro_n;
            gyro_new = true;
            float gz = d->gyro_z;
            S.gyro_bad = !(isfinite(gz) && fabsf(gz) < P.gyro_sat);
            if (!S.gyro_bad) S.gz_raw = gz;
        }
        if (d->rv_n != S.rv_n) {
            S.rv_n = d->rv_n;
            rv_new = true;
            float y = d->yaw * (PI_F / 180.0f);
            S.rv_bad = !isfinite(y);
            if (!S.rv_bad) S.yaw = wrap_pi(y);
        }
        if (d->accel_n != S.accel_n) {
            S.accel_n = d->accel_n;
            accel_new = isfinite(d->accel_x) && isfinite(d->accel_y) && isfinite(d->accel_z);
        }
    }

    bool settled = !S.reset_seen || (in->now_us - S.reset_t_us) >= P.t_settle_us;
    bool g_valid = false, r_valid = false;
    float age_g = -1.0f, age_r = -1.0f;
    if (d && d->gyro_n > 0) {
        int64_t a = in->now_us - d->gyro_t_us;
        age_g = a * 1e-3f;
        g_valid = !S.gyro_bad && settled && a <= P.t_age_us &&
                  (!S.reset_seen || d->gyro_t_us > S.reset_t_us);
    }
    if (d && d->rv_n > 0) {
        int64_t a = in->now_us - d->rv_t_us;
        age_r = a * 1e-3f;
        r_valid = !S.rv_bad && settled && a <= P.t_age_us &&
                  (!S.reset_seen || d->rv_t_us > S.reset_t_us);
    }

    // §4.3: mất hợp lệ → encoder ngay; có lại → IMU sau n_back mẫu gyro tốt liên tiếp
    if (!g_valid)             S.good_run = 0;
    else if (gyro_new && S.good_run < 1000000) S.good_run++;

    float gz = S.gz_raw - S.bias;
    int src_new = S.wz_src;
    if (S.wz_src == WZ_SRC_IMU && !g_valid)                           src_new = WZ_SRC_ENC;
    else if (S.wz_src == WZ_SRC_ENC && g_valid && S.good_run >= P.n_back) src_new = WZ_SRC_IMU;
    if (src_new != S.wz_src) {
        S.switch_count++;
        S.switch_dwz = (src_new == WZ_SRC_IMU) ? gz - in->wz_enc : in->wz_enc - gz;
        S.wz_src = src_new;
    }

    if (reset_event) {
        S.cls = CLS_NONE;
        S.above_since = S.below_since = -1;
    }
    slip_step(in, g_valid, gyro_new, gz);
    still_step(in, g_valid, gyro_new, accel_new);
    gz = S.gz_raw - S.bias;   // bias có thể vừa đổi

    out->g_valid          = g_valid;
    out->r_valid          = r_valid;
    out->gyro_new         = gyro_new;
    out->wz_src           = S.wz_src;
    out->wz_meas          = S.wz_src == WZ_SRC_IMU ? gz : in->wz_enc;
    out->gz               = (d && d->gyro_n > 0) ? gz : 0.0f;
    out->yaw              = S.yaw;
    out->age_g_ms         = age_g;
    out->age_r_ms         = age_r;
    out->reset_event      = reset_event;
    out->bias_gz          = S.bias;
    out->bias_updates     = S.bias_updates;
    out->r                = S.r;
    out->r_th             = P.k_r * sqrtf(P.sigma_gz * P.sigma_gz + P.q_wz * P.q_wz);
    out->slip_flag        = S.slip;
    out->slip_on_count    = S.slip_on_count;
    out->src_switch_count = S.switch_count;
    out->src_switch_dwz   = S.switch_dwz;
    (void)rv_new;
}
