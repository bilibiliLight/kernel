#include <linux/init.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include "dmx512.h"

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

#include <mach/map.h>
#include <mach/regs-serial.h>
#include <mach/regs-gcr.h>
#include <mach/regs-gpio.h>
#include <mach/mfp.h>


#define UART_NR     11

static int dmx512_major;
static dev_t devno;
static char *DevName = {"dmx512"};
static struct cdev dmx512_cdev;
struct class *pClass;

struct dmx512_data {
#define DMX512_DATA_LEN 512
    unsigned char buf[DMX512_DATA_LEN];
};

struct dmx512_tx {
    struct dmx512_data *p_data;
    struct mutex mutex_lock;
    struct spinlock spin_lock;
    unsigned char *p_start;
    unsigned char *p_end;
};

struct dmx512_rx {
    struct dmx512_data *p_data;
    struct mutex mutex_lock;
    struct spinlock spin_lock;
    int wr_pos;
    int cur_idx;
    int skip_cnt;
    int total_num;
};

struct dmx512_timer {
    struct timeval last_tv;
    struct timeval total_diff;
    unsigned long diff_time;
    unsigned long max_diff;
    unsigned long min_diff;
};

struct dmx512_rx_status {
    unsigned int cur_frame;
    unsigned long frame_freq;
    struct dmx512_timer rx_timer;

#define DMX512_RX_STATUS_DONE     (1<<0)
#define DMX512_RX_STATUS_BREAK    (1<<1)
#define DMX512_RX_STATUS_TMSHORT  (1<<2)
#define DMX512_RX_STATUS_CHLOST   (1<<3)
#define DMX512_RX_STATUS_CACHE    (1<<4)
    unsigned int flags;
};

struct dmx512_tx_status {
    struct dmx512_time_seq cur_time_seq;
    dmx512_time_seq_mode cur_mode;
    unsigned long cur_frame_freq;
    struct dmx512_timer tx_timer;
};

struct uart_nuc970_port {
	struct uart_port	port;

	unsigned short		capabilities;	/* port capabilities */
	unsigned char		ier;
	unsigned char		lcr;
	unsigned char		mcr;
	unsigned char		mcr_mask;	/* mask of user bits */
	unsigned char		mcr_force;	/* mask of forced bits */

	struct serial_rs485     rs485;          /* rs485 settings */
    struct dmx512_tx    dmx512tx;     /* dmx512 send data */
    struct dmx512_rx    dmx512rx;     /* dmx512 receive data */
    struct dmx512_tx_status dmx512tx_status; /* status of dmx512 tx */
    struct dmx512_rx_status dmx512rx_status; /* status of dmx512 rx */
    char                *pmmapbuf;    /* mmap to user buf */
    char                *palloctxbuf;       /* alloc for dmx512 send */
    char                *pallocrxbuf1;      /* alloc for dmx512 receive */
    char                *pallocrxbuf2;      /* alloc for dmx512 receive */
    unsigned long       malloc_size;    /* mmap to user buf size */
    atomic_t            tx_condition;   /* tx wait queue condition */
    wait_queue_head_t   tx_queue;       /* tx wait queue */
    atomic_t            rx_condition;   /* rx wait queue condition */
    wait_queue_head_t   rx_queue;       /* rx wait queue */
	/*
	 * We provide a per-port pm hook.
	 */
	void			(*pm)(struct uart_port *port,
				      unsigned int state, unsigned int old);
};

extern struct uart_nuc970_port nuc970serial_ports[UART_NR];

long dmx512_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    unsigned int para;

    switch(cmd)
    {
        case DMX512_WAKEUPQ:
            if (copy_from_user(&para, (unsigned int *) arg, sizeof(unsigned int)))
			    return -EFAULT;

            atomic_set(&nuc970serial_ports[para].rx_condition, 1);
            wake_up_interruptible(&nuc970serial_ports[para].rx_queue);
            break;

        default:
            return -EFAULT;
    }

    return ret;
}

int dmx512_open(struct inode *inode, struct file *filp)
{
    filp->private_data = &dmx512_cdev;
    
    return 0;
}

int dmx512_release(struct inode *inode, struct file *filp)
{
    return 0;
}



static const struct file_operations fifo_operations = {
    .open = dmx512_open,
    .release = dmx512_release,
    .unlocked_ioctl = dmx512_unlocked_ioctl,
};

int __init dmx512_init(void)
{
	int ret;
    struct device *device;

    //alloc devno
    ret = alloc_chrdev_region(&devno, 0, 1, DevName);
    if(ret < 0)
    {
        printk("Fail to register_chrdev_region\n");
        goto err_register_chrdev_region;
    }
    
    dmx512_major = MAJOR(devno);  

    pClass = class_create(THIS_MODULE, DevName);
    if(IS_ERR(pClass)){
        ret = PTR_ERR(pClass);
        goto err_class_create;
    }
    
    cdev_init(&dmx512_cdev, &fifo_operations);

    ret = cdev_add(&dmx512_cdev, devno,1);
    if (ret < 0)
    {
        goto err_cdev_add;
    }
    
    device = device_create(pClass, NULL, devno, NULL, DevName);
    if(IS_ERR(device)){
        ret = PTR_ERR(device);
        printk("Fail to device_create\n");
        goto err_device_create;    
    }

    printk("Register  to system,ok!\n");

    
    return 0;

err_device_create:
    device_destroy(pClass, devno);    

err_cdev_add:
    cdev_del(&dmx512_cdev);

err_class_create:
    unregister_chrdev_region(devno, 1);

err_register_chrdev_region:
    return ret;
}

EXPORT_SYMBOL(dmx512_init);

static void __exit dmx512_exit(void)
{
    device_destroy(pClass, devno);    
    class_destroy(pClass);
    cdev_del(&dmx512_cdev);
    unregister_chrdev_region(devno, 1);
    return;
}

module_init(dmx512_init);
module_exit(dmx512_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DMX512 driver");

