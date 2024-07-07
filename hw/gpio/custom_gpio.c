#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qom/object.h"

#include <mqueue.h>

#include <semaphore.h>

#include "include/hw/gpio/custom_gpio.h"

#define SEM_NAME "/gpio_semaphore"
static sem_t *sem;
#define SHM_NAME "/gpio_shm"
#define SHM_SIZE sizeof(c_gpio_regs)
static char sh_data[SHM_SIZE];

static void * remote_gpio_thread(void * arg);

static const VMStateDescription vmstate_custom_gpio = {
    .name = "custom_gpio",
    .fields = (const VMStateField[]){
        VMSTATE_STRUCT(regs, CUSTOM_GPIOState, 0, vmstate_custom_gpio, c_gpio_regs),
        VMSTATE_END_OF_LIST()}

};

static uint64_t custom_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    (void)size; // Don't use it, always 32 bit access.

    CUSTOM_GPIOState *s = (CUSTOM_GPIOState *)opaque;
    uint32_t *regs = (uint32_t *)&s->regs;
    info("Read called! (Addr: 0x%x)\n", (unsigned int)addr);

    // Some checks...
    if (addr > sizeof(c_gpio_regs))
    {
        error("Requested read from too high address, returning 0x0!\n");
        return 0x0;
    }

    if (addr % sizeof(uint32_t) != 0)
    {
        warning("Requested address is not aligned! (will read from 0x%x)\n",
                (unsigned)(addr / sizeof(uint32_t) * sizeof(uint32_t)));
    }

    // Treat struct as an array, compute index and return
    // requested field...
    uint32_t index = addr / sizeof(uint32_t);

    uint32_t retval = regs[index];
    return (uint64_t)retval;
}

static void custom_gpio_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)
{
    // CHECK: Would this work?
    // CUSTOM_GPIOState *s = CUSTOM_GPIO(opaque);

    CUSTOM_GPIOState *s = (CUSTOM_GPIOState *)opaque;
    (void)s;

    printf("Write called!\n");
}

static void custom_gpio_reset(DeviceState *dev)
{
    info("Reset called!\n");
    CUSTOM_GPIOState *s = CUSTOM_GPIO(dev);
    uint32_t *regs = (uint32_t *)&s->regs;

    for (uint32_t i = 0; i < sizeof(c_gpio_regs) / sizeof(uint32_t); i++)
    {

        regs[i] = i;
    }

    qemu_irq_lower(s->irq);
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

    info("Init start\n");
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

    custom_gpio_reset(dev);
    (void)custom_gpio_read(dev, 0x100, 0x0);
    (void)custom_gpio_read(dev, 0x3, 0x0);
    (void)custom_gpio_read(dev, 0x7, 0x0);
    info("Read: %x\n", (unsigned)custom_gpio_read(dev, 0x8, 0x0));
    // qdev_init_gpio_in(dev, irq_handler, 32);
    (void)dev;


        // This is only for the case we kill QEMU and not unlink semaphore correctly.
    if (!sem_unlink(SEM_NAME))
    {
        info("Semaphore unlinked successfully. (Probably left from killing QEMU)\n");
    }


    sem = sem_open(SEM_NAME, O_CREAT | O_EXCL, 0666, 1);  // Start with 1 to allow first access
    sem_trywait(sem);



    // Create shared memory object
    (void)sh_data;
    s->shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (s->shm_fd == -1)
    {
        perror("shm_open failed");
        while (1)
            ;
    }

    // Size the shared memory
    int res = ftruncate(s->shm_fd, sizeof(sh_data));
    (void)res;
    // Map the shared memory
    s->sh_mem_base = mmap(0, sizeof(sh_data), PROT_READ | PROT_WRITE, MAP_SHARED, s->shm_fd, 0);
    if (s->sh_mem_base == MAP_FAILED)
    {
        perror("mmap failed");
        while (1)
            ;
    }

    memcpy(s->sh_mem_base,&s->regs,sizeof(sh_data));
    sem_post(sem);

    qemu_thread_create(&s->thread, "remote_gpio", remote_gpio_thread, s,
                       QEMU_THREAD_JOINABLE);   



    info("Init end\n");
}

static const TypeInfo custom_gpio_info = {

    .name = TYPE_CUSTOM_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CUSTOM_GPIOState),
    .instance_init = custom_gpio_init,
    .class_init = custom_gpio_class_init,
};


//#MARK: THREAD
static void * remote_gpio_thread(void * arg)
{
    info("Starting thread\n");
    CUSTOM_GPIOState *s = (CUSTOM_GPIOState *) arg;
    uint32_t j=0;
    while(1){

        sem_wait(sem);
        s->regs.gpio_cfg = j;
        j++;
        memcpy(s->sh_mem_base,&s->regs,sizeof(sh_data));
        sem_post(sem);

    }

    return NULL;

}



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