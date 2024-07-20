#ifndef CUSTOM_GPIO_H
#define CUSTOM_GPIO_H


#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "qemu/main-loop.h" /* iothread mutex */

#define GET_BIT(offset,reg) (bool)((reg >> offset) & 0x1)


#define info(fmt, ...) printf("cgpio: " fmt, ##__VA_ARGS__)
#define warning(fmt, ...) printf("cgpio: [WARNING] " fmt, ##__VA_ARGS__)
#define error(fmt, ...) printf("cgpio: [ERROR] " fmt, ##__VA_ARGS__)


#define TYPE_CUSTOM_GPIO "custom_gpio"
OBJECT_DECLARE_SIMPLE_TYPE(CUSTOM_GPIOState, CUSTOM_GPIO)


typedef struct custom_gpio_regs{
    uint32_t gpio_cfg; // 0x0
    uint32_t gpio_state; //0x4 and so on...
    uint32_t irq_en;
    uint32_t irq_sta;
    uint32_t irq_clr;
}c_gpio_regs;

typedef struct sh_mem_struct{
    uint32_t update_indicator;
    c_gpio_regs regs;
}sh_mem_struct;

// This is the runtime state, describes current machine state.
struct CUSTOM_GPIOState
{

    SysBusDevice parent_obj;

    MemoryRegion iomem;

    c_gpio_regs regs;
    uint32_t prev_state;
    uint32_t outputs;

    qemu_irq irq;
    qemu_irq out[32];

    QemuThread thread;
    int shm_fd;
    char* sh_mem_base;
};




DeviceState *custom_gpio_create(hwaddr addr);

#endif /*CUSTOM_GPIO_H*/