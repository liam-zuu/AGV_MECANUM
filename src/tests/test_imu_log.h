#ifndef TEST_IMU_LOG_H
#define TEST_IMU_LOG_H

// Raw BNO085 logging for the IMU noise measurement. The ESP32 streams every
// update of the IMU data as a CSV line over UART0; the PC script
// hil-framework tools/imu_capture.py records it. Motors stay disabled.
void test_imu_log(void);

#endif
