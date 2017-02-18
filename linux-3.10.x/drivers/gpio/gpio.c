#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/fs.h>           /* everything... */
#include <linux/interrupt.h>    /* request_irq() */
#include <linux/delay.h>        /* mdelay() */
#include <linux/device.h>       /* class_create() */
#include <linux/slab.h>

#include <asm/uaccess.h>	/* copy_*_user */

#include <mach/map.h>
#include <mach/regs-serial.h>
#include <mach/regs-gcr.h>
#include <mach/regs-gpio.h>
#include <mach/mfp.h>

#include <linux/wait.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/clk.h>
#include <linux/serial_reg.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/nmi.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/mm.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/serial.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>


struct fasync_struct *detect_async;

MODULE_AUTHOR("CDJZ-TECH, ALAN");
MODULE_LICENSE("GPL");

#include "gpio.h"
#include "../scale-A8-drivers.h"

#ifndef GPIO_MAJOR
#define GPIO_MAJOR 0   /* dynamic major by default */
#endif

/*
 * Split minors in two parts
 */
#define TYPE(minor)	(((minor) >> 4) & 0xf)	/* high nibble */
#define NUM(minor)	((minor) & 0xf)		/* low  nibble */

extern int gpio_major;     /* gpio.c */

struct gpio_dev
{
    unsigned long size;       /* amount of data stored here */
    struct semaphore sem;     /* mutual exclusion semaphore     */
    struct cdev cdev;	  /* Char device structure		*/
    struct timer_list trace_timer;
};

static struct class *gpio_class;

//#define GPIO_DEBUG
#ifdef GPIO_DEBUG
#define __D(level, fmt, args...)	printk(level "gpio: Debug " fmt, ##args)
#else
#define __D(level, fmt, args...)
#endif

#define __E(level, fmt, args...)	printk(level "gpio: Error " fmt, ##args)


int gpio_major =   GPIO_MAJOR;
int gpio_minor =   0;
static unsigned char gpio_inc = 0;
static unsigned int gpio_delay_time = 0;
static unsigned int gpio_trace_flag = 0;
static unsigned int gpio_valid_input_flag = 0;
static unsigned int gpio_has_irq_flag = 0;

wait_queue_head_t   trace_queue;
atomic_t            trace_condition;


struct gpio_dev *gpio_dev;

#define DEVICENAME "scale-gpio"
#define TRACE_TIMER_PERIOD      50

static ssize_t detect_fasync(int fd, struct file *filp, int on)
{
    return fasync_helper(fd, filp, on, &detect_async);
}

static ssize_t gpio_open(struct inode *inode, struct file *filp)
{
    struct gpio_dev *dev;

    if (gpio_inc > 0)
    {
        __D(KERN_NOTICE, " device has been opened \n");
        return -ERESTARTSYS;
    }
    gpio_inc ++;

    dev = container_of(inode->i_cdev, struct gpio_dev, cdev);

    filp->private_data = dev;

    __D(KERN_NOTICE, " gpio open:sucess! \n");

    return 0;
}

static ssize_t gpio_read(struct file *filp, char *buf, size_t count, loff_t *f_ops)
{
    return 0;
}

static ssize_t gpio_write(struct file *filp, const char *buf, size_t count, loff_t *f_ops)
{

    return 0;
}

static int gpio_get_pin_number(unsigned long arg)
{
    int pin_number = 0;

    switch (arg)
    {
    case GPIO_PIN_SPEAKER:
        pin_number = SCALE_PIN_SPEAKER;
        break;
    case GPIO_PIN_CALIBRATION:
        pin_number = SCALE_PIN_CALIBRATION;
        break;
    case GPIO_PIN_PRINTER_STATUS:
        pin_number = SCALE_PIN_PRINTER_STATUS;
        break;
    case GPIO_PIN_PRINTER_POWER:
        pin_number = SCALE_PIN_PRINTER_POWER;
        break;
    case GPIO_PIN_RUN_LED:
        pin_number = SCALE_PIN_RUN_LED;
        break;
    case GPIO_PIN_RFID_RESET:
        pin_number = SCALE_PIN_RFID_RESET;
        break;
    case GPIO_PIN_RFID_POWER:
        pin_number = SCALE_PIN_RFID_POWER;
        break;
    case GPIO_PIN_ZIGBEE_RESET:
        pin_number = SCALE_PIN_ZIGBEE_RESET;
        break;
    case GPIO_PIN_ZIGBEE_CONFIG:
        pin_number = SCALE_PIN_ZIGBEE_CONFIG;
        break;
    case GPIO_PIN_ZIGBEE_SLEEP:
        pin_number = SCALE_PIN_ZIGBEE_SLEEP;
        break;
    case GPIO_PIN_BLE_POWER:
        pin_number = SCALE_PIN_BLE_POWER;
        break;
    case GPIO_PIN_BLE_STATE:
        pin_number = SCALE_PIN_BLE_STATE;
        break;
    default:
        break;
    }
    __D(KERN_NOTICE, "the siglegpio pin_number = %d\n", pin_number);

    return pin_number;
}

int check_pin(unsigned long arg)
{
    if((arg > GPIO_TO_PIN(9, 4)) || (arg < GPIO_TO_PIN(0, 0)))
        return -1;
    else
        return 0;
}

static irqreturn_t gpio_trace_interrupt(int irq, void *dev_id)
{
    volatile unsigned int *reg;
    
    //ISR,'1' to clear
    reg = REG_GPIOH_ISR;
    __raw_writel(__raw_readl(reg) | (0x01<<(1*4)), reg);

    //wake up trace pin
    if(gpio_trace_flag > 0)
    {
        atomic_set(&trace_condition, 1);
        wake_up_interruptible(&trace_queue);
    }

    gpio_has_irq_flag = 1;

    return IRQ_HANDLED;
}

static irqreturn_t gpio_detect_interrupt(int irq, void *dev_id)
{
    volatile unsigned int *reg;

    //disable this irq
    //IREN,'1' rissing edge enable
    reg = REG_GPIOH_IREN;
    __raw_writel(__raw_readl(reg) & (~(0x01<<(1*5))), reg);

    //IFEN,'1' falling edge enable
    reg = REG_GPIOH_IFEN;
    __raw_writel(__raw_readl(reg) & (~(0x01<<(1*5))), reg);

    //send sig
    kill_fasync(&detect_async, SIGIO, POLL_IN);

    //ISR,'1' to clear
    reg = REG_GPIOH_ISR;
    __raw_writel(__raw_readl(reg) | (0x01<<(1*5)), reg);
    
    return IRQ_HANDLED;
}

void trace_control(void)
{
    int ret;
    
    if(gpio_trace_flag > 0)
    {
        atomic_set(&trace_condition, 0);
        
        if(atomic_read(&trace_condition) == 0)
        {
            ret = wait_event_interruptible_timeout(trace_queue, atomic_read(&trace_condition), 1*HZ/20);
            if(ret != 0)
            {
                if(gpio_delay_time > 0)
                {
                    udelay(gpio_delay_time); 
                }
            }
        }

        gpio_delay_time = 0;
        gpio_trace_flag = 0;
    }
}

static void trace_do_timer(unsigned long arg)
{
    struct gpio_dev *dev = (struct gpio_dev *)(arg);

    dev->trace_timer.expires = jiffies + HZ * TRACE_TIMER_PERIOD / 1000;
    add_timer(&dev->trace_timer);

    if(gpio_has_irq_flag)
    {
        gpio_valid_input_flag = 1;
        gpio_has_irq_flag = 0;
    }
    else
    {
        gpio_valid_input_flag = 0;
    }
}

/*
 * The ioctl() implementation
 */
static long gpio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

    int err = 0;
    int retval = 0;
    int pin_number = 0;
    int irq;
    volatile unsigned int *reg;
    struct gpio_dev *dev;

    __D(KERN_NOTICE, " gpio_ioctl \n");

    __D(KERN_NOTICE, "cmd = %d \n", cmd);

    /*
     * extract the type and number bitfields, and don't decode
     * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
     */
    if (_IOC_TYPE(cmd) != SCALE_MAGIC) return -ENOTTY;
    if (_IOC_NR(cmd) > SCALE_IOC_MAXNR) return -ENOTTY;

    __D(KERN_NOTICE, " cmd was accepted \n");

    /*
     * the direction is a bitmask, and VERIFY_WRITE catches R/W
     * transfers. `Type' is user-oriented, while
     * access_ok is kernel-oriented, so the concept of "read" and
     * "write" is reversed
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE, arg, _IOC_SIZE(cmd));
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        err =  !access_ok(VERIFY_READ, arg, _IOC_SIZE(cmd));
    if (err)
    {
        __E(KERN_ALERT, "access error %d \n", err);
        return -EFAULT;
    }
    __D(KERN_NOTICE, " cmd can be accessed \n");

    switch (cmd)
    {
    case SCALE_GPIO_IOC_TRACE_DELAY:
        //trace delay
        if(gpio_valid_input_flag > 0)
        {
            gpio_delay_time = (arg > 0)?arg:0;
            gpio_trace_flag = 1;
        }
        return retval;
    case SCALE_GPIO_IOC_DETECT_REINIT:
        //IREN,'1' rissing edge enable
        reg = REG_GPIOH_IREN;
        __raw_writel(__raw_readl(reg) | (0x01<<(1*5)), reg);

        //IFEN,'1' falling edge enable
        reg = REG_GPIOH_IFEN;
        __raw_writel(__raw_readl(reg) | (0x01<<(1*5)), reg);

        //ISR,'1' to clear
        reg = REG_GPIOH_ISR;
        __raw_writel(__raw_readl(reg) | (0x01<<(1*5)), reg);
        return retval;
    default:
        break;
    }

    if (check_pin(arg) != 0)
    {
        __E(KERN_ALERT, "pin number error\n");
        return -EFAULT;
    }
    pin_number = arg;

    switch (cmd)
    {

    case SCALE_GPIO_IOC_RESET:
        break;
    case SCALE_GPIO_IOC_INPUT:
        gpio_request(pin_number, "gpioin");
        gpio_direction_input(pin_number);
        retval = gpio_get_value(pin_number);
        __D(KERN_NOTICE, "the siglegpio pin_state = %d\n", retval);

        break;
    case SCALE_GPIO_IOC_OUTPUT_HIGHT:
        trace_control();
        gpio_request(pin_number, "gpioout");
        gpio_direction_output(pin_number, 1);
        gpio_set_value(pin_number, 1);
        break;
    case SCALE_GPIO_IOC_OUTPUT_LOW:
        trace_control();
        gpio_request(pin_number, "gpioout");
        gpio_direction_output(pin_number, 0);
        gpio_set_value(pin_number, 0);
        break;
    case SCALE_GPIO_IOC_TRACE_INIT:
        //trace pin init
        pin_number = GPIO_TO_PIN(7, 4);
        gpio_request(pin_number, "gpioin");
        gpio_direction_input(pin_number);
        irq = gpio_to_irq(pin_number);
        retval = request_irq(irq, gpio_trace_interrupt, 0, "trace pin irq", NULL);
    	if (retval) {
    		printk("request trace pin irq failed...\n");
    		return retval;
    	}

        //control pin is PH4,init register
        //IMD,'0' edge
        reg = REG_GPIOH_IMD;
        __raw_writel(__raw_readl(reg) & (~(0x01<<(1*4))), reg);

        //IREN,'1' rissing edge enable
        reg = REG_GPIOH_IREN;
        __raw_writel(__raw_readl(reg) | (0x01<<(1*4)), reg);

        //IFEN,'1' falling edge enable
        reg = REG_GPIOH_IFEN;
        __raw_writel(__raw_readl(reg) | (0x01<<(1*4)), reg);

        //ISR,'1' to clear
        reg = REG_GPIOH_ISR;
        __raw_writel(__raw_readl(reg) | (0x01<<(1*4)), reg);

        //init wait q
        init_waitqueue_head(&trace_queue);
        atomic_set(&trace_condition, 0);

        //create cycle check timer
        dev = (struct gpio_dev *)filp->private_data;
        init_timer(&dev->trace_timer);
        dev->trace_timer.function = &trace_do_timer;
        dev->trace_timer.data = (unsigned long)dev;
        dev->trace_timer.expires = jiffies + HZ * TRACE_TIMER_PERIOD / 1000;
        add_timer(&dev->trace_timer);
        break;
    case SCALE_GPIO_IOC_DETECT_INIT:
        //detect init
        pin_number = GPIO_TO_PIN(7, 5);
        gpio_request(pin_number, "gpioin");
        gpio_direction_input(pin_number);
        irq = gpio_to_irq(pin_number);
        retval = request_irq(irq, gpio_detect_interrupt, 0, "detect pin irq", NULL);
    	if (retval) {
    		printk("request detect pin irq failed...\n");
    		return retval;
    	}

        //control pin is PH5,init register
        //IMD,'0' edge
        reg = REG_GPIOH_IMD;
        __raw_writel(__raw_readl(reg) & (~(0x01<<(1*5))), reg);

        //IREN,'1' rissing edge enable
        reg = REG_GPIOH_IREN;
        __raw_writel(__raw_readl(reg) | (0x01<<(1*5)), reg);

        //IFEN,'1' falling edge enable
        reg = REG_GPIOH_IFEN;
        __raw_writel(__raw_readl(reg) | (0x01<<(1*5)), reg);

        //ISR,'1' to clear
        reg = REG_GPIOH_ISR;
        __raw_writel(__raw_readl(reg) | (0x01<<(1*5)), reg);
        break;

    default:
        __D(KERN_NOTICE, " invallid cmd \n");
        return -ENOTTY;
    }
    return retval;
}

static ssize_t gpio_release(struct inode *inode, struct file *filp)
{
    gpio_inc --;

    __D(KERN_NOTICE, " release \n");

    detect_fasync(-1, filp, 0);
    
    return 0;
}


struct file_operations gpio_fops =
{
    .owner   = THIS_MODULE,
    .open    = gpio_open,
    .read    = gpio_read,
    .write   = gpio_write,
    .release = gpio_release,
    .unlocked_ioctl   = gpio_ioctl,
    .fasync = detect_fasync,
};

static void gpio_cleanup_module(void)
{
    dev_t devno = MKDEV(gpio_major, gpio_minor);

    /* Get rid of our char dev entries */
    if (gpio_dev)
    {
        cdev_del(&gpio_dev->cdev);
        kfree(gpio_dev);
    }

    /* cleanup_module is never called if registering failed */
    unregister_chrdev_region(devno, 1);
    device_destroy(gpio_class, MKDEV(gpio_major, gpio_minor));
    class_destroy(gpio_class);
    __D(KERN_ALERT, "clean \n");
}


/*
 * Set up the char_dev structure for this device.
 */
static void gpio_setup_cdev(struct gpio_dev *dev)
{
    int err, devno = MKDEV(gpio_major, gpio_minor);

    cdev_init(&dev->cdev, &gpio_fops);
    dev->cdev.owner = THIS_MODULE;
    dev->cdev.ops = &gpio_fops;
    err = cdev_add (&dev->cdev, devno, 1);
    /* Fail gracefully if need be */
    if (err)
        __E(KERN_NOTICE, "Error %d adding gpio", err);
}

static int  __init gpio_create_module( void )
{
    int result;
    dev_t dev = 0;
    /*
     * Get a range of minor numbers to work with, asking for a dynamic
     * major unless directed otherwise at load time.
     */
    if (gpio_major)
    {
        dev = MKDEV(gpio_major, gpio_minor);//将主设备号和次设备号转换成dev_t类型zzz
        result = register_chrdev_region(dev, 1, DEVICENAME);//表示静态的申请和注册设备号zzz
    }
    else
    {
        result = alloc_chrdev_region(&dev, gpio_minor, 1,
                                     DEVICENAME);//表示动态的申请和注册设备号zzz
        gpio_major = MAJOR(dev);
    }
    if (result < 0)
    {
        __E(KERN_WARNING, "gpio: can't get major %d\n", gpio_major);
        return result;
    }

    /*
    * allocate the devices -- we can't have them static, as the number
     * can be specified at load time
     */
    gpio_dev = kmalloc(1 * sizeof(struct gpio_dev), GFP_KERNEL);
    if (!gpio_dev)
    {
        result = -ENOMEM;
        goto fail;  /* Make this more graceful */
    }
    memset(gpio_dev, 0, 1 * sizeof(struct gpio_dev));

    gpio_setup_cdev(gpio_dev);

    gpio_class = class_create(THIS_MODULE, "gpio_class");//为该设备创建一个class zzz

    device_create(gpio_class, NULL, MKDEV(gpio_major, 0), NULL, DEVICENAME); //创建设备文件zzz

    /* At this point call the init function for any friend device */
    dev = MKDEV(gpio_major, gpio_minor + 1);

    __D(KERN_ALERT, "create \n");
    return 0;

fail:
    gpio_cleanup_module();

    return result;
}

module_init(gpio_create_module);//驱动的入口函数zzz
module_exit(gpio_cleanup_module);
