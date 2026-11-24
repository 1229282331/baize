#include "base_feedback.h"
#include "base_motion.h"
#include "main.h"
#include "mpu6050.h"
#include "maintenance.h"

extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;

static BaizeReceiver receiver;
static uint8_t rx_byte;
static uint8_t tx_wire[BAIZE_WIRE_MAX];
/* One latest sample per stream. A nonzero type means it is waiting for TX. */
static BaizeFrame status_frame, imu_frame;
static bool adc_calibrated;
static volatile bool request_ready;
static MaintenanceRequest request;
static bool reset_after_reply;

void base_feedback_start(void)
{
    maintenance_start();
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
    if (baize_receive(&receiver, BAIZE_TO_BASE, byte, &command)) {
        if (command.type == BAIZE_MAINTENANCE) {
            if (!request_ready) {
                request = command.maintenance;
                request_ready = true;
            }
        } else if (command.type == BAIZE_STOP || !maintenance_inhibits_motion()) {
            base_motion_command(&command, HAL_GetTick());
        }
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

void base_feedback_queue_status(const Encoders *encoders, uint8_t state, uint8_t reason)
{
    status_frame.type = BAIZE_STATUS;
    BaizeStatus *status = &status_frame.status;
    status->sample_ms = encoders->sample_ms;
    status->drive_state = state;
    status->stop_reason = reason;
    status->supply_mv = sample_supply_mv();
    for (size_t i = 0; i < BAIZE_WHEELS; ++i) {
        status->angle_mrad[i] = encoders->angle_mrad[i];
        status->velocity_mrad_s[i] = encoders->velocity_mrad_s[i];
    }
}

void base_feedback_poll(uint32_t now_ms)
{
    maintenance_check_deadline(now_ms);
    if (mpu6050_poll(now_ms, &imu_frame.imu)) imu_frame.type = BAIZE_IMU;
    if (huart2.gState != HAL_UART_STATE_READY) return;
    if (reset_after_reply) NVIC_SystemReset();

    if (request_ready) {
        BaizeFrame response = {.type = BAIZE_MAINTENANCE_REPLY};
        maintenance_handle(&request, &response.reply);
        uint16_t size = (uint16_t)baize_encode(&response, tx_wire);
        if (HAL_UART_Transmit_IT(&huart2, tx_wire, size) != HAL_OK) Error_Handler();
        reset_after_reply = request.operation == BAIZE_ENTER_UPDATE && response.reply.result == BAIZE_OK;
        request_ready = false;
        return;
    }

    BaizeFrame *next = status_frame.type ? &status_frame : &imu_frame;
    if (next->type == 0) return;
    uint16_t size = (uint16_t)baize_encode(next, tx_wire);
    if (HAL_UART_Transmit_IT(&huart2, tx_wire, size) == HAL_OK) next->type = 0;
}
