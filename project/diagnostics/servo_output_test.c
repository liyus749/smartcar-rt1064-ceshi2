/* Standalone H5/C30 test. Motor outputs remain zero throughout both stages.
 * A uses the production Servo.c / FlexPWM driver.
 * B drives the same C30 pad with GPIO pulses timed by GPT2.
 * No camera, radio, encoder, control PIT or buttons are initialized.
 */
#include "zf_common_headfile.h"
#include "fsl_gpt.h"
#include "Motor.h"
#include "Servo.h"
#include "Screen.h"

#if SERVO_SETUP_MODE != SERVO_SETUP_RUN
#error This diagnostic uses the normal RUN servo configuration.
#endif

#define TEST_CENTER SERVO_DUTY_MIDDLE
#define TEST_LEFT   (SERVO_DUTY_MIDDLE + 40)
#define TEST_RIGHT  (SERVO_DUTY_MIDDLE - 40)
#define TEST_ROUNDS (3U)
static uint8 test_round;

#if (TEST_LEFT > SERVO_DUTY_LEFT) || (TEST_RIGHT < SERVO_DUTY_RIGHT)
#error Diagnostic endpoints must remain inside the measured steering limits.
#endif

/* system_delay_us() waits once per microsecond and accumulates loop overhead.
 * Use one GPT compare for the whole interval so the pulse width stays valid.
 * clock_init() configures GPT2 for the 24 MHz crystal, divide by one.
 */
static void Test_WaitUs(uint32 us)
{
    GPT_StopTimer(GPT2);
    GPT_ClearStatusFlags(GPT2, kGPT_OutputCompare1Flag);
    GPT_SetOutputCompareValue(GPT2, kGPT_OutputCompare_Channel1,
                             us * (BOARD_XTAL_FREQ / 1000000U));
    GPT_StartTimer(GPT2);
    while(!GPT_GetStatusFlags(GPT2, kGPT_OutputCompare1Flag)) { }
    GPT_StopTimer(GPT2);
    GPT_ClearStatusFlags(GPT2, kGPT_OutputCompare1Flag);
}

static void Test_GpioFrame(uint16 duty)
{
    uint32 high_us = (uint32)duty * 2U;
    gpio_set_level(C30, GPIO_HIGH);
    Test_WaitUs(high_us);
    gpio_set_level(C30, GPIO_LOW);
    Test_WaitUs(20000U - high_us);
}

static void Test_ShowStage(uint8 direct_gpio)
{
    static const char *round_names[] = {"ROUND 1/3", "ROUND 2/3", "ROUND 3/3"};
    ips200_clear();
    ips200_show_string(0, 8, "SERVO TEST T2 / C30");
    ips200_show_string(0, 40, "MOTORS: ALWAYS OFF");
    ips200_show_string(0, 72, direct_gpio ? "B: GPIO / GPT2" : "A: HARDWARE PWM");
    ips200_show_string(0, 104, "DUTY:");
    ips200_show_string(0, 128, "PULSE/us:");
    ips200_show_string(0, 152, "790 -> 830 -> 790");
    ips200_show_string(0, 176, "    -> 750 -> 790");
    ips200_show_string(0, 216, "50Hz / 1500-1660us");
    ips200_show_string(0, 248, "AUTO TEST / NO S1");
    ips200_show_string(0, 296, round_names[test_round]);
}

static void Test_Sequence(uint8 direct_gpio)
{
    static const uint16 duties[] = {TEST_CENTER, TEST_LEFT, TEST_CENTER,
                                    TEST_RIGHT, TEST_CENTER};
    uint16 tick;
    uint8 pos;
    Test_ShowStage(direct_gpio);
    for(pos = 0; pos < sizeof(duties)/sizeof(duties[0]); pos++)
    {
        Motor_Stop();
        ips200_show_uint(64, 104, duties[pos], 3);
        ips200_show_uint(80, 128, (uint32)duties[pos] * 2U, 4);
        if(!direct_gpio) { Servo_SetDuty(duties[pos]); }
        /* Center first for three seconds, then hold each position two seconds. */
        for(tick = 0; tick < (pos == 0 ? 150U : 100U); tick++)
        {
            Motor_Stop();
            if(direct_gpio) { Test_GpioFrame(duties[pos]); }
            else { system_delay_ms(20); }
        }
    }
}

/* Required by the shared interrupt source; no control timer is enabled here. */
void pit_handler0(void) { Motor_Stop(); }
void pit_handler1(void) { Motor_Stop(); }

int main(void)
{
    clock_init(SYSTEM_CLOCK_600M);
    interrupt_global_disable();
    Motor_Init();
    Motor_Stop();
    Servo_Init();
    Screen_Init();

    for(test_round = 0; test_round < TEST_ROUNDS; test_round++)
    {
        /* Restore the C30 PWM mux when coming back from the GPIO stage. */
        if(test_round > 0) { Servo_Init(); }
        Test_Sequence(0);
        /* Switch only C30 from FlexPWM to GPIO for the independent path. */
        gpio_init(C30, GPO, GPIO_LOW, GPO_PUSH_PULL);
        Test_Sequence(1);
    }
    ips200_show_string(0, 248, "DONE: HOLD CENTER 790");
    while(1)
    {
        Motor_Stop();
        Test_GpioFrame(TEST_CENTER);
    }
}
