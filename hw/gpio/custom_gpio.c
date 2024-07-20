#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qom/object.h"
#include <mqueue.h>
#include <semaphore.h>
#include <unistd.h>
#include "qemu/main-loop.h" /* iothread mutex */
#include <time.h>

#include "qemu/thread.h"
#include "include/hw/gpio/custom_gpio.h"

unsigned int irq_count = 0;

#define PINS_COUNT 8 * sizeof(uint32_t)

#define SEM_NAME "/gpio_semaphore"
static sem_t *sem;
static sem_t *rw_sem;
#define SHM_NAME "/gpio_shm"
#define SHM_SIZE sizeof(sh_mem_struct)
static char sh_data[SHM_SIZE];
static uint32_t qemu_indicator = 0;

static void *remote_gpio_thread(void *arg);
static void custom_gpio_update_state(CUSTOM_GPIOState *s, uint32_t *update_indicator);
static void analyze_input_pin(uint32_t pin, CUSTOM_GPIOState *s);
static void analyze_output_pin(bool state, uint32_t pin, CUSTOM_GPIOState *s);

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
    info("Read called! (Addr: 0x%x, size: %u)\n", (unsigned int)addr, (unsigned int)size);

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

    printf("retval: %u, address:  %u, index: %u\n", retval, (uint32_t)addr, index);

    sem_post(sem);
    sem_post(rw_sem);
    return (uint64_t)retval;
}

static void custom_gpio_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned size)

{

    sem_wait(rw_sem);
    sem_wait(sem);

    
    printf("Write called! address: %u, size: %u, value: %u \n",
           (unsigned int)offset, (unsigned int)size, (unsigned int)value);
    (void)size;
    CUSTOM_GPIOState *s = (CUSTOM_GPIOState *)opaque;
    uint32_t *regs = (uint32_t *)&s->regs;
    (void)s;

    // Some checks...
    if (offset > sizeof(c_gpio_regs))
    {
        error("Requested write to too high address, returning 0x0!\n");
        return;
    }

    if (offset % sizeof(uint32_t) != 0)
    {
        warning("Requested offset is not aligned! (will not write!)\n");
        return;
    }

    uint32_t index = offset / sizeof(uint32_t);

    regs[index] = (uint32_t)value;

    sem_post(sem);
    sem_post(rw_sem);
}

static void custom_gpio_reset(DeviceState *dev)
{
    info("Reset called!\n");
    CUSTOM_GPIOState *s = CUSTOM_GPIO(dev);
    uint32_t *regs = (uint32_t *)&s->regs;

    for (uint32_t i = 0; i < sizeof(c_gpio_regs) / sizeof(uint32_t); i++)
    {

        regs[i] = 0;
    }

    s->prev_state = 0;
    qemu_irq_lower(s->irq);
    // qemu_indicator = (uint32_t) get_clock_realtime();
    qemu_indicator = 0;
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

static void placeholder_handler(void *opaque, int irq, int level)
{
    printf("IRQ HANDLER QEMU\n");
}

static void custom_gpio_init(Object *obj)
{

    info("Init start\n");
    DeviceState *dev = DEVICE(obj);
    CUSTOM_GPIOState *s = CUSTOM_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &custom_gpio_ops, s, "custom_gpio_iomem", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    sysbus_init_mmio(sbd, &s->iomem);

    // This attaches irq to sysbus, now this is what the kernel will give
    // us when we request an IRQ.
    sysbus_init_irq(sbd, &s->irq);

    // This is GPIO management inside QEMU. The device has 32 (anonymous) output pins
    // and 32 input pins.
    qdev_init_gpio_in(dev, placeholder_handler, 32);
    qdev_init_gpio_out(dev, s->out, 32);

    // Reset device before threading stuff
    custom_gpio_reset(dev);

    // This is only for the case we kill QEMU and not unlink semaphore correctly.
    if (!sem_unlink(SEM_NAME))
    {
        info("Semaphore unlinked successfully. (Probably left from killing QEMU)\n");
    }

    if (!sem_unlink("RW_SEM"))
    {
        info("Semaphore unlinked successfully. (Probably left from killing QEMU)\n");
    }

    sem = sem_open(SEM_NAME, O_CREAT | O_EXCL, 0666, 1);
    rw_sem = sem_open("RW_SEM", O_CREAT | O_EXCL, 0666, 1);
    // sem_trywait(sem);
    //  Create shared memory object
    s->shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);

    if (s->shm_fd == -1)
    {
        perror("shm_open failed");
        while (1)
            ;
    }

    // TODO: Error handling
    //  Size the shared memory
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
    s->regs.gpio_state = 0x0;
    memcpy(s->sh_mem_base, &s->regs, sizeof(sh_data));
    sem_post(sem);
    sem_post(rw_sem);

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

static void custom_gpio_update_state(CUSTOM_GPIOState *s, uint32_t *update_indicator)
{

    uint32_t shared_indicator;
    memcpy(&shared_indicator, s->sh_mem_base, sizeof(uint32_t));

    // Check if state was changed by remote
    if (shared_indicator != *update_indicator)
    {
        printf("Updating state from remote! (%lu)\n", (unsigned long)qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
        memcpy(&s->regs, s->sh_mem_base + 4, sizeof(c_gpio_regs));
    }

    memcpy(s->sh_mem_base, update_indicator, sizeof(uint32_t));

    c_gpio_regs *regs = &s->regs;

    while (bql_locked())
        ;
    if (!bql_locked())
    {
        bql_lock();

        for (uint32_t pin = 0; pin < PINS_COUNT; pin++)
        {
            if (GET_BIT(pin, regs->gpio_cfg))
            {
                analyze_input_pin(pin, s);
            }
            else
            {
                analyze_output_pin(GET_BIT(pin, s->outputs), pin, s);
            }
        }
        // TODO: those are char pointers so they dont add 0x4
        memcpy(s->sh_mem_base + 4, regs, sizeof(c_gpio_regs));
        s->prev_state = s->regs.gpio_state;

        bql_unlock();
    }
}

// #MARK: THREAD
static void *remote_gpio_thread(void *arg)
{
    info("Starting thread\n");
    CUSTOM_GPIOState *s = (CUSTOM_GPIOState *)arg;

    while (1)
    {
        // printf("thread try lock\n");
        if (sem_trywait(rw_sem) == 0)
        {
            // printf("thread locking\n");
            sem_wait(sem);
            custom_gpio_update_state(s, &qemu_indicator);
            // printf("posting sem\n");
            sem_post(sem);
            // printf("posting rw");
            sem_post(rw_sem);
        }
        g_usleep(10);
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

// #MARK: State update
// TODO: IRQ on rising edge.
static void analyze_input_pin(uint32_t pin, CUSTOM_GPIOState *s)
{
    c_gpio_regs *regs = &s->regs;
    bool state = GET_BIT(pin, regs->gpio_state);
    bool prev_state = GET_BIT(pin, s->prev_state);
    bool irq_en = GET_BIT(pin, regs->irq_en);
    bool clr = GET_BIT(pin, regs->irq_clr);
    // Check state

    // Check irq_clr
    if (clr)
    {
        regs->irq_sta &= ~(0x1 << pin);
        regs->irq_clr &= ~(0x1 << pin);
        qemu_irq_lower(s->irq);
    }
    bool sta = GET_BIT(pin, regs->irq_sta);

    // Check IRQ EN?
    // If IRQ enabled raise interrupt and set sta flag

    bool rising_edge = (prev_state == false && state == true) ? true : false;
    if ((rising_edge && irq_en) || sta)
    {
        regs->irq_sta |= (0x1 << pin);
        qemu_irq_raise(s->irq);
        // printf("Raising IRQ! (pin:%u,state: %u, en: %u, sta: %u) \n", pin, state, irq_en, regs->irq_sta);
    }
};
static void analyze_output_pin(bool state, uint32_t pin, CUSTOM_GPIOState *s)
{

    s->regs.gpio_state &= ~((0x1 & state) << pin);
};