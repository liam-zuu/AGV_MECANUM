#include "test_runner.h"

#ifdef TEST_MOTOR
#include "test_motor.h"
#endif
#ifdef TEST_ENCODER
#include "test_encoder.h"
#endif
#ifdef TEST_IBUS
#include "test_ibus.h"
#endif
#ifdef TEST_IMU
#include "test_imu.h"
#endif
#ifdef TEST_ULTRASONIC
#include "test_ultrasonic.h"
#endif
#ifdef TEST_LOGGER
#include "test_logger.h"
#endif
#ifdef TEST_RASP_UART
#include "test_rasp_uart.h"
#endif
#ifdef TEST_WIFI
#include "test_wifi.h"
#endif
#ifdef TEST_LED_BUZZER
#include "test_led_buzzer.h"
#endif
#ifdef TEST_KINEMATICS
#include "test_kinematics.h"
#endif
#ifdef TEST_MANUAL
#include "test_manual.h"
#endif

void run_tests(void) {
#ifdef TEST_MOTOR
    test_motor();
#elif defined(TEST_ENCODER)
    test_encoder();
#elif defined(TEST_IBUS)
    test_ibus();
#elif defined(TEST_IMU)
    test_imu();
#elif defined(TEST_ULTRASONIC)
    test_ultrasonic();
#elif defined(TEST_LOGGER)
    test_logger();
#elif defined(TEST_RASP_UART)
    test_rasp_uart();
#elif defined(TEST_WIFI)
    test_wifi();
#elif defined(TEST_LED_BUZZER)
    test_led_buzzer();
#elif defined(TEST_KINEMATICS)
    test_kinematics();
#elif defined(TEST_MANUAL)
    test_manual();
#endif
}