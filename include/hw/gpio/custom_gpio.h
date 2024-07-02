#ifndef CUSTOM_GPIO_H
#define CUSTOM_GPIO_H


#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qom/object.h"

#define TYPE_CUSTOM_GPIO "custom_gpio"
OBJECT_DECLARE_SIMPLE_TYPE(CUSTOM_GPIOState, CUSTOM_GPIO)
// This is the runtime state, describes current machine state.
struct CUSTOM_GPIOState
{

    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint32_t reg;

    qemu_irq irq;

    qemu_irq out[32];

};


DeviceState *custom_gpio_create(hwaddr addr);













#endif /*CUSTOM_GPIO_H*/