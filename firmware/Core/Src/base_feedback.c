#include "base_feedback.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;

static BaizeReceiver receiver;
static uint8_t rx_byte;
static uint8_t tx_wire[BAIZE_WIRE_MAX];
static volatile uint8_t stop_reason = BAIZE_RESET;
static bool adc_calibrated;

void base_feedback_start(void)
{
    adc_calibrated = HAL_ADCEx_Calibration_Start(&hadc1) == HAL_OK;
    if (adc_calibrated) HAL_ADC_Start(&hadc1);
    if (HAL_UART_Receive_IT(&huart2, &rx_byte, 1) != HAL_OK) Error_Handler();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart->ErrorCode != HAL_UART_ERROR_NONE) return;
    uint8_t byte = rx_byte;
    HAL_UART_Receive_IT(uart, &rx_byte, 1);
    BaizeFrame command;
    if (baize_receive(&receiver, BAIZE_TO_BASE, byte, &command) && command.type == BAIZE_STOP) {
        /* This acquisition build keeps the drive disabled and acknowledges STOP. */
        stop_reason = BAIZE_REQUESTED_STOP;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    /* Resume at the next frame boundary after a corrupt or lost byte. */
    receiver.length = 0;
    receiver.discarding = true;
    __HAL_UART_CLEAR_OREFLAG(uart);
    if (uart->RxState == HAL_UART_STATE_READY) HAL_UART_Receive_IT(uart, &rx_byte, 1);
}

static uint16_t sample_supply_mv(void)
{
    uint16_t mv = UINT16_MAX;
    if (!adc_calibrated) return mv;
    /* VREFINT is converted automatically after the PA4 conversion. */
    if (__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOC)) {
        uint32_t divider = HAL_ADC_GetValue(&hadc1);
        uint32_t reference = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        if (reference != 0) {
            /* Nominal 1200 mV reference and (68k + 10k) / 10k divider. */
            uint32_t value = (divider * 9360u + reference / 2u) / reference;
            if (value < UINT16_MAX) mv = (uint16_t)value;
        }
    }
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    HAL_ADC_Start(&hadc1);
    return mv;
}

void base_feedback_send(const BaizeEncoders *encoders)
{
    uint16_t supply_mv = sample_supply_mv();
    if (huart2.gState != HAL_UART_STATE_READY) return;

    BaizeFrame frame = {
        .type = BAIZE_STATUS,
        .status = {
            .sample_ms = encoders->sample_ms,
            .drive_state = BAIZE_DISABLED,
            .stop_reason = stop_reason,
            .supply_mv = supply_mv
        }
    };
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        frame.status.angle_mrad[i] = encoders->angle_mrad[i];
        frame.status.velocity_mrad_s[i] = encoders->velocity_mrad_s[i];
    }
    uint16_t size = (uint16_t)baize_encode(&frame, tx_wire);
    HAL_UART_Transmit_IT(&huart2, tx_wire, size);
}
