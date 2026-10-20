#ifndef TEST_OPENLOOP_H
#define TEST_OPENLOOP_H

// Spec S01: open-loop wheel test with the wheels lifted off the ground.
// Controller is not used. A PC script (hil-framework tools/s01_openloop_runner.py)
// sends a PWM schedule over UART0, the ESP32 runs it on one wheel and samples the
// encoder at 1 kHz, then sends the samples back. See test_openloop.c for the protocol.
void test_openloop(void);

#endif
