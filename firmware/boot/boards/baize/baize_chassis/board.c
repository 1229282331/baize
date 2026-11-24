#include <zephyr/init.h>
#include <stm32f1xx.h>
#include <bootutil/mcuboot_uuid.h>
#include <string.h>

/* PC6..PC9 are the four active-low motor inputs. Set the output latch first. */
static int park_wheels(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    (void)RCC->APB2ENR;
    GPIOC->BSRR = 0x3c0;
    GPIOC->CRL = (GPIOC->CRL & 0x00ffffffu) | 0x22000000u;
    GPIOC->CRH = (GPIOC->CRH & 0xffffff00u) | 0x22u;
    return 0;
}
SYS_INIT(park_wheels, PRE_KERNEL_1, 0);

fih_ret boot_uuid_init(void)
{
    FIH_RET(FIH_SUCCESS);
}

fih_ret boot_uuid_cid_match(uint32_t image_id, const struct image_uuid *cid)
{
    static const uint8_t chassis_id[16] = {
        0xfe, 0x43, 0x1d, 0xd9, 0xeb, 0xe8, 0x5f, 0xa2,
        0x93, 0xea, 0x07, 0x90, 0xb4, 0xa0, 0x47, 0x2d
    };
    if (image_id != 0 || memcmp(cid->raw, chassis_id, sizeof chassis_id) != 0)
        FIH_RET(FIH_FAILURE);
    FIH_RET(FIH_SUCCESS);
}
