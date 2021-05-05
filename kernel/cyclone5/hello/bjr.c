#include <linux/kernel.h>  /* for KERN_DEBUG */
#include <linux/module.h>  /* for all kernel modules */

int init_module(void)
{
        printk(KERN_DEBUG "bjr: Hello World.\n");
        return 0; /* init_module loaded successfully */
}

void cleanup_module(void)
{
        printk(KERN_DEBUG "bjr: oh, the rest is silence.\n");
}

MODULE_LICENSE("Dual BSD/GPL");
