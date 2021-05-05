#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/interrupt.h>
#include <linux/spinlock_types.h>

#define EDA_IRQ "eda-irq"
#define UINPUT_BASE 0xff200000
#define UINPUT_SIZE PAGE_SIZE

int UINPUT_INT_NUM = 72;
module_param(UINPUT_INT_NUM, int, 0);

void *eda_irq_mem;

static DEFINE_SEMAPHORE(interrupt_mutex);
static DECLARE_WAIT_QUEUE_HEAD(interrupt_wq);

static int interrupt_flag = 0;
static DEFINE_SPINLOCK(interrupt_flag_lock);
static uint8_t input_state;

static int eda_irq_flags = IRQF_SHARED;

static irqreturn_t eda_irq_interrupt(int irq, void *dev_id)
{
	if (irq != UINPUT_INT_NUM)
		return IRQ_NONE;

	spin_lock(&interrupt_flag_lock);
	interrupt_flag = 1;
	input_state = ioread8(eda_irq_mem);
	spin_unlock(&interrupt_flag_lock);

	wake_up_interruptible(&interrupt_wq);

	return IRQ_HANDLED;
}

static struct device_driver eda_irq_driver = {
	.name = EDA_IRQ,
	.bus = &platform_bus_type,
};

static ssize_t eda_irq_show(struct device_driver *drv, char *buf)
{
	int ret;

	if (down_trylock(&interrupt_mutex))
		return -EAGAIN;

	if (wait_event_interruptible(interrupt_wq, interrupt_flag != 0)) {
		ret = -ERESTART;
		goto release_and_exit;
	}

	spin_lock(&interrupt_flag_lock);
	interrupt_flag = 0;
	spin_unlock(&interrupt_flag_lock);

	buf[0] = input_state;
	ret = 1;

release_and_exit:
	up(&interrupt_mutex);
	return ret;
}

static ssize_t eda_irq_store(struct device_driver *drv,
		const char *buf, size_t count)
{
	return -EROFS;
}

static DRIVER_ATTR(eda_irq, S_IRUSR, eda_irq_show, eda_irq_store);

static int __init eda_irq_init(void)
{
	int ret;
	struct resource *res;

    printk(KERN_DEBUG "eda-irq: enabling IRQ UINPUT=%d...\n", UINPUT_INT_NUM);

	ret = driver_register(&eda_irq_driver);
	if (ret < 0) {
		goto fail_driver_register;
	}

	ret = driver_create_file(&eda_irq_driver, &driver_attr_eda_irq);
	if (ret < 0)
		goto fail_create_file;

	res = request_mem_region(UINPUT_BASE, UINPUT_SIZE, EDA_IRQ);
	if (res == NULL) {
		ret = -EBUSY;
		goto fail_request_mem;
	}

	eda_irq_mem = ioremap(UINPUT_BASE, UINPUT_SIZE);
	if (eda_irq_mem == NULL) {
		ret = -EFAULT;
		goto fail_ioremap;
	}

//	ret = can_request_irq(UINPUT_INT_NUM, eda_irq_flags);
//	if (ret < 0) {
//        printk(KERN_DEBUG "eda-irq: could not can_request_irq: %d\n", ret);
//		goto fail_request_irq;
//	}

	ret = request_irq(UINPUT_INT_NUM, eda_irq_interrupt,
			eda_irq_flags, EDA_IRQ, (void*)eda_irq_interrupt);
	if (ret < 0) {
        printk(KERN_DEBUG "eda-irq: could not request_irq: %d\n", ret);
		goto fail_request_irq;
	}

    printk(KERN_DEBUG "eda-irq: enabling IRQ UINPUT=%d... [ok]\n", UINPUT_INT_NUM);
	return 0;

fail_request_irq:
	iounmap(eda_irq_mem);
fail_ioremap:
	release_mem_region(UINPUT_BASE, UINPUT_SIZE);
fail_request_mem:
	driver_remove_file(&eda_irq_driver, &driver_attr_eda_irq);
fail_create_file:
	driver_unregister(&eda_irq_driver);
fail_driver_register:
	return ret;
}

static void __exit eda_irq_exit(void)
{
	free_irq(UINPUT_INT_NUM, (void*)eda_irq_interrupt);
	iounmap(eda_irq_mem);
	release_mem_region(UINPUT_BASE, UINPUT_SIZE);
	driver_remove_file(&eda_irq_driver, &driver_attr_eda_irq);
	driver_unregister(&eda_irq_driver);
    printk(KERN_DEBUG "eda-irq: disabled IRQ UINPUT=%d\n", UINPUT_INT_NUM);
}

MODULE_LICENSE("Dual BSD/GPL");

module_init(eda_irq_init);
module_exit(eda_irq_exit);
