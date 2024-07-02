#ifndef SWIS_GPIO_H
#define SWIS_GPIO_H

#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qapi/error.h" /* provides error_fatal() handler */
#include "qom/object.h"
#include "stdio.h"


#define TYPE_SWIS_GPIO "swis_gpio"
OBJECT_DECLARE_SIMPLE_TYPE(SWIS_GPIOState, SWIS_GPIO)

struct SWIS_GPIOState
{
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint32_t id;
    uint32_t test_reg;
};

#endif