#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qom/object.h"

#include "include/hw/gpio/custom_gpio.h"

static const VMStateDescription vmstate_custom_gpio = {
    .name = "custom_gpio",

    .fields = (const VMStateField[]){
        VMSTATE_UINT32(reg, CUSTOM_GPIOState),
        VMSTATE_END_OF_LIST()}};

static void irq_handler(void *opaque, int n, int level)
{
    // CUSTOM_GPIOState *s = (CUSTOM_GPIOState *)opaque;
    //  Perform actions needed when interrupt is triggered
    printf("-----> Interrupt triggered\n");
}

static uint64_t custom_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{

    CUSTOM_GPIOState *s = (CUSTOM_GPIOState *)opaque;
    printf("Read called! (Addr: 0x%x)\n", (unsigned int)addr);
    if (addr == 0x4)
    {   
        qemu_irq_lower(s->irq);
        return 0xc0ffe;
    }
    if (addr == 0x8)
    {
        printf("Trying to trigger interrupt...\n");
        qemu_irq_raise(s->irq);
        

        
    }

        if (addr == 0x10)
    {   
        qemu_irq_lower(s->irq);
        return 0xbaba;
    }
    return 0x1234;
}

static void custom_gpio_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    // CHECK: Would this work?
    // CUSTOM_GPIOState *s = CUSTOM_GPIO(opaque);

    CUSTOM_GPIOState *s = (CUSTOM_GPIOState *)opaque;

    s->reg = (uint32_t)value;
    printf("Write called!\n");
}

static void custom_gpio_reset(DeviceState *dev)
{

    CUSTOM_GPIOState *s = CUSTOM_GPIO(dev);

    s->reg = 0x54321;
}

static const MemoryRegionOps custom_gpio_ops = {
    .read = custom_gpio_read,
    .write = custom_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

// MARK: INITIALIZATION
static void custom_gpio_class_init(ObjectClass *class, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);

    // virtual machine state description
    dc->vmsd = &vmstate_custom_gpio;
    dc->reset = custom_gpio_reset;
}

// Initialize device
static void custom_gpio_init(Object *obj)
{

    printf("-----> Init start\n");
    DeviceState *dev = DEVICE(obj);
    CUSTOM_GPIOState *s = CUSTOM_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &custom_gpio_ops, s, "custom_gpio_iomem", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    // printf("--------> Allocatin`g irqs\n");
    // CHECK: What does it even do? Handler is not called...
    // s->irq =qemu_allocate_irq(irq_handler,s,1);

    // printf("--------> Sysbus init\n");
    // sysbus_init_irq(sbd, &s->irq);

    sysbus_init_mmio(sbd, &s->iomem);

    sysbus_init_irq(sbd, &s->irq);
    qdev_init_gpio_in(dev, irq_handler, 32);
    qdev_init_gpio_out(dev, s->out, 32);

    (void)dev;

    printf("-----> Init end\n");
}

static const TypeInfo custom_gpio_info = {

    .name = TYPE_CUSTOM_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CUSTOM_GPIOState),
    .instance_init = custom_gpio_init,
    .class_init = custom_gpio_class_init,
};

static void custom_gpio_register_types(void)
{
    type_register_static(&custom_gpio_info);
}

type_init(custom_gpio_register_types)

    DeviceState *custom_gpio_create(hwaddr addr)
{
    DeviceState *dev = qdev_new(TYPE_CUSTOM_GPIO);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, addr);

    return dev;
}