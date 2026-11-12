#include "gain_set.h"
#include "vel_ctrl.h"
#include "pid.h"
#include "adrc.h"
#include "heading.h"
#include "pos_ctrl.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Thiết kế: 03_CODE/docs/2026-11-12_Thiet_ke_doi_gain_qua_WiFi.md (L90). Không gọi ESP-IDF.

static const char *const NAMES_PID[GAIN_N]  = { "kp_xy", "ki_xy", "kp_wz", "ki_wz",
                                                "pos_kp", "pos_ki", "k_psi" };
static const char *const NAMES_ADRC[GAIN_N] = { "omega_c_xy", "omega_o_xy", "omega_c_wz", "omega_o_wz",
                                                "pos_kp", "pos_ki", "k_psi" };

// Giới hạn an toàn [ĐỀ XUẤT 12/11, chặn số vô lý; miền tune là việc của script chiến dịch].
// LADRC omega_o < 50: beta1*dt = 2*omega_o*0.01 < 1 (adrc.c).
static const float LO_PID[GAIN_N]  = { 0.0f,  0.0f, 0.0f,  0.0f, 0.0f,  0.0f, 0.0f };
static const float HI_PID[GAIN_N]  = { 20.0f, 50.0f, 20.0f, 50.0f, 30.0f, 30.0f, 10.0f };
static const float LO_ADRC[GAIN_N] = { 0.5f,  1.0f, 0.5f,  1.0f, 0.0f,  0.0f, 0.0f };
static const float HI_ADRC[GAIN_N] = { 50.0f, 49.0f, 50.0f, 49.0f, 30.0f, 30.0f, 10.0f };

static uint32_t s_id;          // id của bộ đang chạy; 0 = gain build sẵn
static uint32_t s_applied_n;   // số lần áp thành công từ lúc khởi động

const char *gain_err_str(int err) {
    switch (err) {
    case GAIN_OK:           return "";
    case GAIN_ERR_FORMAT:   return "format";
    case GAIN_ERR_CTRL:     return "ctrl";
    case GAIN_ERR_RANGE:    return "range";
    case GAIN_ERR_MOTOR_ON: return "motor_on";
    case GAIN_ERR_BUSY:     return "busy";
    case GAIN_ERR_TIMEOUT:  return "timeout";
    default:                return "unknown";
    }
}

const char *const *gain_set_names(int ctrl) {
    return ctrl == CTRL_PID ? NAMES_PID : ctrl == CTRL_ADRC ? NAMES_ADRC : NULL;
}

const char *gain_ctrl_str(int ctrl) {
    return ctrl == CTRL_PID ? "PID" : ctrl == CTRL_ADRC ? "ADRC" : "?";
}

// Số nguyên không dấu thập phân, toàn bộ chuỗi, lo..hi (không dấu, không số 0 đứng đầu thừa)
static int parse_uint(const char *s, uint64_t lo, uint64_t hi, uint32_t *out) {
    size_t n = strlen(s);
    if (n == 0 || n > 10 || (s[0] == '0' && n > 1)) return 0;
    uint64_t v = 0;
    for (const char *p = s; *p; p++) {
        if (*p < '0' || *p > '9') return 0;
        v = v * 10u + (uint64_t)(*p - '0');
    }
    if (v < lo || v > hi) return 0;
    *out = (uint32_t)v;
    return 1;
}

static int is_digit(char c) { return c >= '0' && c <= '9'; }

// Mẫu số thập phân -?d+(.d+)?([eE][+-]?d+)? trên toàn chuỗi, kiểm trước strtof (Codex L90-10:
// strtof tự nhận khoảng trắng đầu, dấu +, hex, inf/nan).
static int number_grammar(const char *s) {
    const char *p = s;
    if (*p == '-') p++;
    if (!is_digit(*p)) return 0;
    while (is_digit(*p)) p++;
    if (*p == '.') {
        p++;
        if (!is_digit(*p)) return 0;
        while (is_digit(*p)) p++;
    }
    if (*p == 'e' || *p == 'E') {
        p++;
        if (*p == '+' || *p == '-') p++;
        if (!is_digit(*p)) return 0;
        while (is_digit(*p)) p++;
    }
    return *p == '\0';
}

static int parse_float(const char *s, float *out) {
    if (!number_grammar(s) || strlen(s) > 24) return 0;
    char *end = NULL;
    float v = strtof(s, &end);
    if (end == s || *end != '\0') return 0;
    *out = v;            // hữu hạn và miền kiểm ở gain_set_check_range
    return 1;
}

int gain_msg_parse(const char *line, size_t len, int *kind, gain_set_t *set, uint32_t *tx) {
    char buf[GAIN_MSG_MAX + 1];
    *tx = 0;
    // Đúng một kết thúc dòng "\n" hoặc "\r\n" được phép ở cuối (Codex L90-7)
    if (len > 0 && line[len - 1] == '\n') {
        len--;
        if (len > 0 && line[len - 1] == '\r') len--;
    }
    if (len == 0 || len > GAIN_MSG_MAX) return GAIN_ERR_FORMAT;
    for (size_t i = 0; i < len; i++) {
        unsigned char c = (unsigned char)line[i];
        if (c < 0x21 || c > 0x7E) return GAIN_ERR_FORMAT;        // ký tự điều khiển, cách, NUL, không ASCII
    }
    memcpy(buf, line, len);
    buf[len] = '\0';

    gain_set_t g = { .id = 0, .ctrl = -1 };
    int have_v = 0, have_id = 0, have_ctrl = 0, have_tx = 0;
    uint32_t tx_v = 0;
    char  gname[GAIN_N][16];
    float gval[GAIN_N];
    int   ng = 0;

    // Tách theo ',' (không dùng strtok_r: fwsim/test PC biên dịch bằng MinGW). Ô rỗng là lỗi khuôn.
    char *tok = buf;
    char *next = strchr(tok, ',');
    if (next) *next++ = '\0';
    int k_msg;
    if      (strcmp(tok, "#gain_set") == 0) k_msg = GAIN_MSG_SET;
    else if (strcmp(tok, "#gain_get") == 0) k_msg = GAIN_MSG_GET;
    else return GAIN_ERR_FORMAT;
    while ((tok = next) != NULL) {
        next = strchr(tok, ',');
        if (next) *next++ = '\0';
        char *eq = strchr(tok, '=');
        if (eq == NULL || eq == tok) return GAIN_ERR_FORMAT;
        *eq = '\0';
        const char *k = tok, *val = eq + 1;
        if (strcmp(k, "v") == 0) {
            if (have_v++ || strcmp(val, "1") != 0) return GAIN_ERR_FORMAT;
        } else if (strcmp(k, "tx") == 0) {
            if (have_tx++ || !parse_uint(val, 1u, 4294967295u, &tx_v)) return GAIN_ERR_FORMAT;
            *tx = tx_v;                                             // trả lời được cả gói hỏng ở trường sau
        } else if (k_msg == GAIN_MSG_GET) {
            return GAIN_ERR_FORMAT;                                 // #gain_get chỉ có v, tx
        } else if (strcmp(k, "id") == 0) {
            if (have_id++ || !parse_uint(val, 1u, 2147483647u, &g.id)) return GAIN_ERR_FORMAT;
        } else if (strcmp(k, "ctrl") == 0) {
            if (have_ctrl++) return GAIN_ERR_FORMAT;
            if      (strcmp(val, "PID") == 0)  g.ctrl = CTRL_PID;
            else if (strcmp(val, "ADRC") == 0) g.ctrl = CTRL_ADRC;
            else return GAIN_ERR_FORMAT;
        } else {
            if (ng >= GAIN_N || strlen(k) >= sizeof(gname[0])) return GAIN_ERR_FORMAT;
            strcpy(gname[ng], k);
            if (!parse_float(val, &gval[ng])) return GAIN_ERR_FORMAT;
            ng++;
        }
    }
    if (!have_v || !have_tx) return GAIN_ERR_FORMAT;
    if (k_msg == GAIN_MSG_GET) {
        *kind = GAIN_MSG_GET;
        return GAIN_OK;
    }
    if (!have_id || !have_ctrl || ng != GAIN_N) return GAIN_ERR_FORMAT;

    // Mỗi tên của controller đúng một lần
    const char *const *names = gain_set_names(g.ctrl);
    for (int i = 0; i < GAIN_N; i++) {
        int found = -1;
        for (int j = 0; j < GAIN_N; j++) {
            if (strcmp(gname[j], names[i]) == 0) {
                if (found >= 0) return GAIN_ERR_FORMAT;
                found = j;
            }
        }
        if (found < 0) return GAIN_ERR_FORMAT;
        g.v[i] = gval[found];
    }
    *kind = GAIN_MSG_SET;
    *set = g;
    return GAIN_OK;
}

int gain_set_check_range(const gain_set_t *g) {
    const float *lo = g->ctrl == CTRL_PID ? LO_PID : g->ctrl == CTRL_ADRC ? LO_ADRC : NULL;
    const float *hi = g->ctrl == CTRL_PID ? HI_PID : g->ctrl == CTRL_ADRC ? HI_ADRC : NULL;
    if (lo == NULL) return GAIN_ERR_FORMAT;
    for (int i = 0; i < GAIN_N; i++) {
        float v = g->v[i];
        if (!isfinite(v) || v < lo[i] || v > hi[i]) return GAIN_ERR_RANGE;
    }
    return GAIN_OK;
}

int gain_set_apply(const gain_set_t *g) {
    if (g->ctrl != vel_ctrl_mode()) return GAIN_ERR_CTRL;
    int r = gain_set_check_range(g);
    if (r != GAIN_OK) return r;

    // Cùng phép gán như fwsim (fw_sim.c fwsim_set_pid / fwsim_set_adrc, tune_ga.gains_for)
    if (g->ctrl == CTRL_PID) {
        pid_controller_t *c = vel_ctrl_pid();
        c->vx.kp = g->v[0]; c->vx.ki = g->v[1];
        c->vy.kp = g->v[0]; c->vy.ki = g->v[1];
        c->wz.kp = g->v[2]; c->wz.ki = g->v[3];
    } else {
        adrc_controller_t *c = vel_ctrl_adrc();
        adrc_set_params(&c->vx, g->v[0], g->v[1], c->vx.eso.b0, c->vx.output_limit);
        adrc_set_params(&c->vy, g->v[0], g->v[1], c->vy.eso.b0, c->vy.output_limit);
        adrc_set_params(&c->wz, g->v[2], g->v[3], c->wz.eso.b0, c->wz.output_limit);
    }
    pos_params()->kp = g->v[4];
    pos_params()->ki = g->v[5];
    heading_params()->k_psi = g->v[6];

    // Trạng thái cũ (tích phân, ESO) tính với gain cũ: bỏ. Motor tắt nên vel_ctrl_step cũng sẽ
    // đặt lại ở cạnh bật; gọi ở đây để không phụ thuộc thứ tự đó.
    pid_reset(vel_ctrl_pid());
    adrc_reset(vel_ctrl_adrc());

    s_id = g->id;
    s_applied_n++;
    return GAIN_OK;
}

void gain_set_current(gain_set_t *out) {
    gain_set_t g = { .id = s_id, .ctrl = vel_ctrl_mode() };
    if (g.ctrl == CTRL_ADRC) {
        const adrc_controller_t *c = vel_ctrl_adrc();
        g.v[0] = c->vx.kp; g.v[1] = c->vx.eso.beta1 / 2.0f;
        g.v[2] = c->wz.kp; g.v[3] = c->wz.eso.beta1 / 2.0f;
    } else {
        const pid_controller_t *c = vel_ctrl_pid();
        g.v[0] = c->vx.kp; g.v[1] = c->vx.ki;
        g.v[2] = c->wz.kp; g.v[3] = c->wz.ki;
    }
    g.v[4] = pos_params()->kp;
    g.v[5] = pos_params()->ki;
    g.v[6] = heading_params()->k_psi;
    *out = g;
}

uint32_t gain_set_applied_n(void) { return s_applied_n; }

int gain_set_format(char *buf, size_t n, const gain_set_t *g, uint32_t applied_n) {
    if (n == 0) return -1;
    const char *const *names = gain_set_names(g->ctrl);
    int k = snprintf(buf, n, ",id=%lu,applied_n=%lu,ctrl=%s",
                     (unsigned long)g->id, (unsigned long)applied_n, gain_ctrl_str(g->ctrl));
    for (int i = 0; names != NULL && i < GAIN_N; i++) {
        if (k < 0 || (size_t)k >= n) break;
        k += snprintf(buf + k, n - (size_t)k, ",%s=%.9g", names[i], (double)g->v[i]);
    }
    if (k < 0 || (size_t)k >= n) { buf[0] = '\0'; return -1; }   // không trả dòng cụt
    return k;
}
