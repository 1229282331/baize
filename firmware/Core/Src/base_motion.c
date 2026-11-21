#include "base_motion.h"
#include "baize_drive.h"
#include "main.h"

#include <string.h>

extern TIM_HandleTypeDef htim8;

/* Initial bench limits. PWM units are timer counts out of 2000. */
enum { SPEED_LIMIT_MRAD_S = 2000, PWM_LIMIT = 300, COMMAND_TIMEOUT_MS = 150 };
static const uint32_t channels[] = {TIM_CHANNEL_4, TIM_CHANNEL_3, TIM_CHANNEL_2, TIM_CHANNEL_1};
static const uint16_t dir_pins[] = {FL_DIR_Pin, FR_DIR_Pin, RL_DIR_Pin, RR_DIR_Pin};
static const bool forward_high[] = {true, false, true, false};
static BaizeDrive drive;
static BaizeWheelPi wheels[BAIZE_WHEELS];
static int16_t last_effort[BAIZE_WHEELS];

void base_motion_start(void)
{
    baize_drive_init(&drive, SPEED_LIMIT_MRAD_S, COMMAND_TIMEOUT_MS);
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        /* Vendor nominal P/I converted from counts/5 ms to protocol velocity units. */
        wheels[i] = (BaizeWheelPi){.kp = 0.067265f, .ki = 3.2392f};
        __HAL_TIM_SET_COMPARE(&htim8, channels[i], htim8.Init.Period + 1u);
    }
    /* Latch neutral and start all four channels before assigning the pins to TIM8. */
    htim8.Instance->EGR = TIM_EGR_UG;
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        if (HAL_TIM_PWM_Start(&htim8, channels[i]) != HAL_OK) Error_Handler();
    }
}

void base_motion_command(const BaizeFrame *command, uint32_t received_ms)
{
    baize_drive_command(&drive, command, received_ms);
}

static void write_effort(size_t wheel, int16_t effort)
{
    if ((effort > 0 && last_effort[wheel] < 0) || (effort < 0 && last_effort[wheel] > 0)) {
        effort = 0; /* Hold neutral for one control update before reversing DIR. */
    }
    if (effort != 0 && last_effort[wheel] == 0) {
        bool high = (effort > 0) == forward_high[wheel];
        HAL_GPIO_WritePin(GPIOC, dir_pins[wheel], high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    uint32_t magnitude = (uint32_t)(effort < 0 ? -effort : effort);
    __HAL_TIM_SET_COMPARE(&htim8, channels[wheel], htim8.Init.Period + 1u - magnitude);
    last_effort[wheel] = effort;
}

void base_motion_update(const int32_t velocity_mrad_s[BAIZE_WHEELS], uint32_t elapsed_ms,
                        uint8_t *drive_state, uint8_t *stop_reason)
{
    int32_t targets[BAIZE_WHEELS];
    /* A complete command arrives in the UART IRQ. Copy one coherent control input. */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    baize_drive_tick(&drive, HAL_GetTick());
    *drive_state = drive.state;
    *stop_reason = drive.stop_reason;
    bool reset = drive.reset_output;
    drive.reset_output = false;
    memcpy(targets, drive.target_mrad_s, sizeof targets);
    __set_PRIMASK(mask);

    if (reset) {
        for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
            wheels[i].integral = 0;
            write_effort(i, 0);
        }
        htim8.Instance->EGR = TIM_EGR_UG;
    } else if (*drive_state == BAIZE_ENABLED) {
        for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
            int16_t effort = baize_wheel_effort(&wheels[i], targets[i], velocity_mrad_s[i], elapsed_ms, PWM_LIMIT);
            write_effort(i, effort);
        }
    }
}
