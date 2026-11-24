#include "maintenance.h"
#include "base_motion.h"
#include "main.h"

#include <string.h>

/* MCUboot v2 image format, BOOT_MAX_ALIGN=8, swap-using-offset. */
enum { SLOT_START = 0x08010000, SLOT_END = 0x08027800, HEADER_SIZE = 512,
       IMAGE_OK = SLOT_END - 24, COPY_DONE = SLOT_END - 32 };
static volatile bool trial, entering;
static const uint8_t *image_hash;
static const uint8_t *const header = (const uint8_t *)SLOT_START;

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static const uint8_t *read_digest(void)
{
    uint32_t length;
    memcpy(&length, header + 12, sizeof length);
    if (le16(header + 8) != HEADER_SIZE || length > SLOT_END - SLOT_START - HEADER_SIZE)
        return NULL;
    uint32_t off = HEADER_SIZE + length + le16(header + 10);
    if (off > SLOT_END - SLOT_START - 4) return NULL;
    const uint8_t *p = header + off;
    uint16_t total = le16(p + 2);
    if (le16(p) != 0x6907 || total < 4 || total > SLOT_END - SLOT_START - off)
        return NULL;
    const uint8_t *end = p + total;
    for (p += 4; end - p >= 4;) {
        uint16_t size = le16(p + 2);
        if (size > end - p - 4) return NULL;
        if (le16(p) == 0x10 && size == 32) return p + 4;
        p += 4 + size;
    }
    return NULL;
}

void maintenance_start(void)
{
    image_hash = read_digest();
    trial = *(const volatile uint8_t *)COPY_DONE == 1 &&
            *(const volatile uint8_t *)IMAGE_OK != 1;
}

bool maintenance_inhibits_motion(void)
{
    return trial || entering || !image_hash;
}

void maintenance_check_deadline(uint32_t now_ms)
{
    if (trial && now_ms >= 10000u) NVIC_SystemReset();
}

void maintenance_handle(const MaintenanceRequest *request, MaintenanceReply *reply)
{
    *reply = (MaintenanceReply){.operation = request->operation};
    if (!image_hash) {
        reply->result = BAIZE_BAD_IMAGE;
        return;
    }
    memcpy(reply->version, header + 20, sizeof reply->version);
    memcpy(reply->hash, image_hash, sizeof reply->hash);
    if (request->operation != BAIZE_INFO) {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (!base_motion_parked() || entering) reply->result = BAIZE_BUSY;
        else if (request->operation == BAIZE_ENTER_UPDATE && !trial) entering = true;
        else if (request->operation != BAIZE_CONFIRM || !trial ||
                 memcmp(request->hash, image_hash, sizeof reply->hash) != 0)
            reply->result = BAIZE_BAD_IMAGE;
        __set_PRIMASK(mask);
        if (reply->result == BAIZE_OK && request->operation == BAIZE_CONFIRM) {
            /* Only the image_ok halfword in the primary trailer is changed. */
            if (HAL_FLASH_Unlock() == HAL_OK &&
                HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, IMAGE_OK, 0xff01) == HAL_OK &&
                *(const volatile uint8_t *)IMAGE_OK == 1) trial = false;
            else reply->result = BAIZE_WRITE_FAILED;
            HAL_FLASH_Lock();
        }
    }
    reply->trial = trial;
}
