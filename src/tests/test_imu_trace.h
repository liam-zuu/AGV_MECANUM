#ifndef TEST_IMU_TRACE_H
#define TEST_IMU_TRACE_H

// SPI trace of the BNO085 link (QD 26, G0 v1 section 7): runs the scenarios
// S1/S2, S3, S4, S5 one after another and prints the trace of each over UART0.
// Same env on the car (real chip) and on the HIL bench (H7 emulator).
// PC side: tools/bno_trace_capture.py. Motors stay disabled.
void test_imu_trace(void);

#endif
