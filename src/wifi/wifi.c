#include "wifi.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "WIFI";

// SSID and password live in wifi_secrets.h, which is not in git (.gitignore).
// Copy wifi_secrets.h.example to wifi_secrets.h next to this file and fill it in.
// Without the file the firmware still builds and runs (control, motors, IMU, iBUS
// do not need WiFi); it prints an error at boot and sends no UDP log.
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#define WIFI_HAVE_SECRETS 1
#else
#warning "src/wifi/wifi_secrets.h missing: WiFi disabled, no UDP log (copy wifi_secrets.h.example)"
#define WIFI_HAVE_SECRETS 0
#define WIFI_SSID     ""
#define WIFI_PASSWORD ""
#endif

// Reconnect: after a disconnect wait WIFI_RETRY_DELAY_US, then try again, forever.
// Logged on the first failure and then every WIFI_LOG_EVERY attempts (no log spam).
#define WIFI_RETRY_DELAY_US  (2 * 1000 * 1000)
#define WIFI_LOG_EVERY       15

static volatile bool connected = false;
static uint32_t retry_count = 0;
static esp_timer_handle_t s_retry_timer = NULL;

static void retry_timer_cb(void *arg) {
    (void)arg;
    esp_wifi_connect();
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        connected = false;
        retry_count++;
        if (retry_count == 1 || retry_count % WIFI_LOG_EVERY == 0) {
            const wifi_event_sta_disconnected_t *d = (const wifi_event_sta_disconnected_t *)event_data;
            ESP_LOGW(TAG, "Not connected to \"%s\" (reason %d), retry %lu, every %d s",
                     WIFI_SSID, d ? (int)d->reason : -1, (unsigned long)retry_count,
                     (int)(WIFI_RETRY_DELAY_US / 1000000));
        }
        esp_timer_stop(s_retry_timer);  // ESP_ERR_INVALID_STATE if not running: harmless
        esp_timer_start_once(s_retry_timer, WIFI_RETRY_DELAY_US);
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR " (after %lu failed attempts)",
                 IP2STR(&event->ip_info.ip), (unsigned long)retry_count);
        retry_count = 0;
        connected = true;
    }
}

// Non-blocking: starts the WiFi driver and returns at once; the connection comes up
// in the background. Users (logger, traj_receiver, s06) wait on wifi_is_connected().
void wifi_init(void) {
    // NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    if (!WIFI_HAVE_SECRETS) {
        ESP_LOGE(TAG, "wifi_secrets.h missing at build time: WiFi off, no UDP log");
        return;
    }

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    const esp_timer_create_args_t targs = {
        .callback = retry_timer_cb,
        .name     = "wifi_retry",
    };
    esp_timer_create(&targs, &s_retry_timer);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_cfg = { 0 };
    strncpy((char *)wifi_cfg.sta.ssid, WIFI_SSID, sizeof(wifi_cfg.sta.ssid));
    strncpy((char *)wifi_cfg.sta.password, WIFI_PASSWORD, sizeof(wifi_cfg.sta.password));
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg);
    esp_wifi_start();
    ESP_LOGI(TAG, "WiFi started, connecting to \"%s\" in the background", WIFI_SSID);
}

bool wifi_is_connected(void) {
    return connected;
}
