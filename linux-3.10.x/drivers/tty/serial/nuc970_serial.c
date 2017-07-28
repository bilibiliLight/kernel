/*
 *  linux/drivers/serial/nuc970_serial.c
 *
 *  NUC970 serial driver
 *
 *
 *  Copyright (C) 2014 Nuvoton Technology Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#if defined(CONFIG_SERIAL_NUC970_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

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

#include <asm/div64.h>

#include "nuc970_serial.h"

#define UART_NR                 11

#define TIOCDMX512SET	        0x5461
#define TIOCDMX512GET	        0x5462
#define TIOCDMX512TX            0x5463
#define TIOCDMX512RX            0x5464
#define TIOCDMX512RXSTOP        0x5465
#define TIOCDMX512RXPAUSE       0x5466
#define TIOCDMX512RXCONTINUE    0x5467
#define TIOCDMX512RXSTATUS      0x5468
#define TIOCDMX512TXSTATUS      0x5469
#define TIOCBIT9ADDR            0x546A
#define TIOCBIT9DATA            0x546B

static struct uart_driver nuc970serial_reg;

struct clk		*clk;

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

//uart0-10:PE0,PE2,PF11,PE12,PH8,PB0,PG11,PI1,PH12,PH2,PB12
volatile unsigned int * tx_reg_func[11] = {    REG_MFP_GPE_L,     REG_MFP_GPE_L,     REG_MFP_GPF_H,     REG_MFP_GPE_H,     REG_MFP_GPH_H,     REG_MFP_GPB_L,     REG_MFP_GPG_H,     REG_MFP_GPI_L,     REG_MFP_GPH_H,     REG_MFP_GPH_L,     REG_MFP_GPB_H};
unsigned int            tx_pos_func[11] = {      0x0F<<(4*0),       0x0F<<(4*2),  0x0F<<(4*(11-8)), 0x0F<<(4*(12-8)),   0x0F<<(4*(8-8)),       0x0F<<(4*0),  0x0F<<(4*(11-8)),       0x0F<<(4*1), 0x0F<<(4*(12-8)),       0x0F<<(4*2),  0x0F<<(4*(12-8))};
volatile unsigned int *  tx_reg_dir[11] = {    REG_GPIOE_DIR,     REG_GPIOE_DIR,     REG_GPIOF_DIR,     REG_GPIOE_DIR,     REG_GPIOH_DIR,     REG_GPIOB_DIR,     REG_GPIOG_DIR,     REG_GPIOI_DIR,     REG_GPIOH_DIR,     REG_GPIOH_DIR,     REG_GPIOB_DIR};
unsigned int             tx_pos_dir[11] = {      0x01<<(1*0),       0x01<<(1*2),      0x01<<(1*11),      0x01<<(1*12),       0x01<<(1*8),       0x01<<(1*0),      0x01<<(1*11),       0x01<<(1*1),      0x01<<(1*12),       0x01<<(1*2),      0x01<<(1*12)};
volatile unsigned int *  tx_reg_out[11] = {REG_GPIOE_DATAOUT, REG_GPIOE_DATAOUT, REG_GPIOF_DATAOUT, REG_GPIOE_DATAOUT, REG_GPIOH_DATAOUT, REG_GPIOB_DATAOUT, REG_GPIOG_DATAOUT, REG_GPIOI_DATAOUT, REG_GPIOH_DATAOUT, REG_GPIOH_DATAOUT, REG_GPIOB_DATAOUT};
unsigned int             tx_pos_out[11] = {      0x01<<(1*0),       0x01<<(1*2),      0x01<<(1*11),      0x01<<(1*12),       0x01<<(1*8),       0x01<<(1*0),      0x01<<(1*11),       0x01<<(1*1),      0x01<<(1*12),       0x01<<(1*2),      0x01<<(1*12)};

struct uart_nuc970_port nuc970serial_ports[UART_NR];
EXPORT_SYMBOL(nuc970serial_ports);

static inline struct uart_nuc970_port *
to_nuc970_uart_port(struct uart_port *uart)
{
	return container_of(uart, struct uart_nuc970_port, port);
}

static inline unsigned int serial_in(struct uart_nuc970_port *p, int offset)
{
	return(__raw_readl(p->port.membase + offset));
}

static inline void serial_out(struct uart_nuc970_port *p, int offset, int value)
{
	__raw_writel(value, p->port.membase + offset);
}

static void rs485_start_rx(struct uart_nuc970_port *port)
{
	#if 0
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;

	if(port->rs485.flags & SER_RS485_RTS_AFTER_SEND)
	{
		// Set logical level for RTS pin equal to high 
		serial_out(port, UART_REG_MCR, (serial_in(port, UART_REG_MCR) & ~0x200) );
    }
    	else
    {
		// Set logical level for RTS pin equal to low 
		serial_out(port, UART_REG_MCR, (serial_in(port, UART_REG_MCR) | 0x200) );
	}
	#endif
}

static void rs485_stop_rx(struct uart_nuc970_port *port)
{
	#if 0
	if(port->rs485.flags & SER_RS485_RTS_ON_SEND)
	{
		// Set logical level for RTS pin equal to high 
		serial_out(port, UART_REG_MCR, (serial_in(port, UART_REG_MCR) & ~0x200) );
	}
	else
	{
		// Set logical level for RTS pin equal to low 
		serial_out(port, UART_REG_MCR, (serial_in(port, UART_REG_MCR) | 0x200) );
	}
	#endif

}

static inline void __stop_tx(struct uart_nuc970_port *p)
{
	unsigned int ier;
	struct tty_struct *tty = p->port.state->port.tty;

	if ((ier = serial_in(p, UART_REG_IER)) & THRE_IEN) {
		serial_out(p, UART_REG_IER, ier & ~THRE_IEN);
	}
	if (p->rs485.flags & SER_RS485_ENABLED)
		rs485_start_rx(p);

	if (tty->termios.c_line == N_IRDA)
	{
		while(!(serial_in(p, UART_REG_FSR) & TX_EMPTY));
		while(!(serial_in(p, UART_REG_FSR) & TE_FLAG));
	
		serial_out(p, UART_REG_IRCR, (serial_in(p, UART_REG_IRCR) & ~0x2) ); // Tx disable (select Rx)
	}
}

static void nuc970serial_stop_tx(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;

	__stop_tx(up);

}

static void transmit_chars(struct uart_nuc970_port *up);

static void nuc970serial_start_tx(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	unsigned int ier;	
	struct tty_struct *tty = up->port.state->port.tty;

	if (tty->termios.c_line == N_IRDA)
	{
		serial_out(up, UART_REG_IRCR, (serial_in(up, UART_REG_IRCR) | 0x2) ); // Tx enable
	}

	if (up->rs485.flags & SER_RS485_ENABLED)
		rs485_stop_rx(up);

	if (!((ier = serial_in(up, UART_REG_IER)) & THRE_IEN)) {
		ier |= THRE_IEN;
		serial_out(up, UART_REG_IER, ier);
	}

}

static void nuc970serial_stop_rx(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;

	serial_out(up, UART_REG_IER, serial_in(up, UART_REG_IER) & ~RDA_IEN);
}

static void nuc970serial_enable_ms(struct uart_port *port)
{

}

static void
receive_chars(struct uart_nuc970_port *up)
{
	unsigned char ch;
	unsigned int fsr;
	int max_count = 256;
	char flag;

	do {
		ch = (unsigned char)serial_in(up, UART_REG_RBR);
		fsr = serial_in(up, UART_REG_FSR);
		flag = TTY_NORMAL;
		up->port.icount.rx++;

		if (unlikely(fsr & (BIF | FEF | PEF | RX_OVER_IF))) {
			if (fsr & BIF) {
				serial_out(up, UART_REG_FSR, BIF);
				up->port.icount.brk++;
				if (uart_handle_break(&up->port))
					continue;
			}

			if (fsr & FEF) {
				serial_out(up, UART_REG_FSR, FEF);
				up->port.icount.parity++;
			}

			if (fsr & PEF) {
				serial_out(up, UART_REG_FSR, PEF);
				up->port.icount.frame++;
			}

			if (fsr & RX_OVER_IF) {
				serial_out(up, UART_REG_FSR, RX_OVER_IF);
				up->port.icount.overrun++;
			}
			// FIXME: check port->read_status_mask to determin report flags
			if (fsr & BIF)
				flag = TTY_BREAK;
			if (fsr & PEF)
				flag = TTY_PARITY;
			if (fsr & FEF)
				flag = TTY_FRAME;
		}
		
		if (uart_handle_sysrq_char(&up->port, ch))
			continue;

		uart_insert_char(&up->port, fsr, RX_OVER_IF, ch, flag);

	} while (!(fsr & RX_EMPTY) && (max_count-- > 0));

	spin_lock(&up->port.lock);
	tty_flip_buffer_push(&up->port.state->port);	
	spin_unlock(&up->port.lock);
}

static void transmit_chars(struct uart_nuc970_port *up)
{
	struct circ_buf *xmit = &up->port.state->xmit;
	int count = 12;

	if (up->port.x_char) {
		while(serial_in(up, UART_REG_FSR) & TX_FULL);
		serial_out(up, UART_REG_THR, up->port.x_char);
		up->port.icount.tx++;
		up->port.x_char = 0;
		return;
	}
	if (uart_tx_stopped(&up->port)) {
		nuc970serial_stop_tx(&up->port);
		return;
	}

	if (uart_circ_empty(xmit)) {
		__stop_tx(up);
		return;
	}

	do {
		//while(serial_in(up, UART_REG_FSR) & TX_FULL);
		serial_out(up, UART_REG_THR, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		up->port.icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&up->port);

	if (uart_circ_empty(xmit))
		__stop_tx(up);
}

static int get_diff_time(struct timeval *pafter, struct timeval *pbefore, unsigned long *pdiff)
{
    int valid_flag;

    if((pafter == NULL) || (pbefore == NULL) || (pdiff == NULL))
        return -1;

    //check valid
    if(pafter->tv_usec >= pbefore->tv_usec)
    {
        if((pafter->tv_sec >= pbefore->tv_sec) && (pafter->tv_sec - pbefore->tv_sec <= 0xFFFFFFFF / 1000000))
        {
            //valid
            valid_flag = 0;
            *pdiff = (pafter->tv_sec - pbefore->tv_sec) * 1000000 + (pafter->tv_usec - pbefore->tv_usec);
        }
        else
        {
            //invalid
            valid_flag = -1;
        }
    }
    else
    {
        if((pafter->tv_sec > pbefore->tv_sec) && (pafter->tv_sec - pbefore->tv_sec <= 0xFFFFFFFF / 1000000))
        {
            //valid
            valid_flag = 0;
            *pdiff = (pafter->tv_sec - 1 - pbefore->tv_sec) * 1000000 + (pafter->tv_usec + 1000000 - pbefore->tv_usec);
        }
        else
        {
            //invalid
            valid_flag = -1;
        }
    }

    return valid_flag;
}

static void dmx512_rx_timer_handle(struct uart_nuc970_port *up)
{
    struct timeval cur_tv;
    unsigned long t_usec;

    do_gettimeofday(&cur_tv);
    
    if(up->dmx512rx_status.cur_frame == 0)
    {
        //first frame only record
        up->dmx512rx_status.rx_timer.max_diff = 0;
        up->dmx512rx_status.rx_timer.min_diff = 0xFFFFFFFF;
        up->dmx512rx_status.rx_timer.diff_time = 0;
        up->dmx512rx_status.rx_timer.total_diff.tv_sec = 0;
        up->dmx512rx_status.rx_timer.total_diff.tv_usec = 0;
        memcpy(&up->dmx512rx_status.rx_timer.last_tv, &cur_tv, sizeof(struct timeval));
    }
    else if(up->rs485.flags & SER_RS485_DMX512_PAUSE)
    {
        //diff,max,min time maintain the last
        t_usec = up->dmx512rx_status.rx_timer.total_diff.tv_usec;
        t_usec += up->dmx512rx_status.rx_timer.diff_time;
        up->dmx512rx_status.rx_timer.total_diff.tv_sec += t_usec / 1000000;
        up->dmx512rx_status.rx_timer.total_diff.tv_usec = t_usec % 1000000;
        
        memcpy(&up->dmx512rx_status.rx_timer.last_tv, &cur_tv, sizeof(struct timeval));
    }
    else
    {
        if(get_diff_time(&cur_tv, &up->dmx512rx_status.rx_timer.last_tv, &up->dmx512rx_status.rx_timer.diff_time) == 0)
        {            
            //find max diff time
            if(up->dmx512rx_status.rx_timer.max_diff < up->dmx512rx_status.rx_timer.diff_time)
            {
                up->dmx512rx_status.rx_timer.max_diff = up->dmx512rx_status.rx_timer.diff_time;
            }

            //find min diff time
            if(up->dmx512rx_status.rx_timer.min_diff > up->dmx512rx_status.rx_timer.diff_time)
            {
                up->dmx512rx_status.rx_timer.min_diff = up->dmx512rx_status.rx_timer.diff_time;
            }

            //total diff time
            t_usec = up->dmx512rx_status.rx_timer.total_diff.tv_usec;
            t_usec += up->dmx512rx_status.rx_timer.diff_time;
            up->dmx512rx_status.rx_timer.total_diff.tv_sec += t_usec / 1000000;
            up->dmx512rx_status.rx_timer.total_diff.tv_usec = t_usec % 1000000;
        }
        else
        {
            up->dmx512rx_status.rx_timer.max_diff = 0;
            up->dmx512rx_status.rx_timer.min_diff = 0xFFFFFFFF;
            up->dmx512rx_status.rx_timer.diff_time = 0;
            up->dmx512rx_status.rx_timer.total_diff.tv_sec = 0;
            up->dmx512rx_status.rx_timer.total_diff.tv_usec = 0;
        }
        
        //record last timeval
        memcpy(&up->dmx512rx_status.rx_timer.last_tv, &cur_tv, sizeof(struct timeval));
    }
}

static void dmx512_tx_timer_handle(struct uart_nuc970_port *up)
{
    struct timeval cur_tv;
    unsigned long t_usec;

    //fix mtbp_time manual
    switch(up->dmx512tx_status.cur_mode)
    {
        case SET_WITH_SEQ:
            udelay(up->dmx512tx_status.cur_time_seq.mtbp_time);
            break;
        default:
            break;
    }

    do_gettimeofday(&cur_tv);

    if((up->dmx512tx_status.tx_timer.last_tv.tv_sec == 0) && (up->dmx512tx_status.tx_timer.last_tv.tv_usec == 0))
    {
        //first frame only record
        up->dmx512tx_status.tx_timer.max_diff = 0;
        up->dmx512tx_status.tx_timer.min_diff = 0xFFFFFFFF;
        up->dmx512tx_status.tx_timer.diff_time = 0;
        up->dmx512tx_status.tx_timer.total_diff.tv_sec = 0;
        up->dmx512tx_status.tx_timer.total_diff.tv_usec = 0;
        memcpy(&up->dmx512tx_status.tx_timer.last_tv, &cur_tv, sizeof(struct timeval));
    }
    else
    {
        if(get_diff_time(&cur_tv, &up->dmx512tx_status.tx_timer.last_tv, &up->dmx512tx_status.tx_timer.diff_time) == 0)
        {
            //fix mtbp_time auto
            switch(up->dmx512tx_status.cur_mode)
            {
                case SET_WITH_SEQ:
                    up->dmx512tx_status.cur_time_seq.period = up->dmx512tx_status.tx_timer.diff_time;
                    up->dmx512tx_status.cur_frame_freq = 1000 * 1000 * 1000 / up->dmx512tx_status.tx_timer.diff_time;
                    up->dmx512tx_status.cur_time_seq.freq = up->dmx512tx_status.cur_frame_freq;
                    break;
                case SET_WITH_SEQ_AND_PERIOD:
                case SET_WITH_FREQ:
                default:
                    if(up->dmx512tx_status.tx_timer.diff_time < up->dmx512tx_status.cur_time_seq.period)
                    {
                        up->dmx512tx_status.cur_time_seq.mtbp_time = up->dmx512tx_status.cur_time_seq.period - up->dmx512tx_status.tx_timer.diff_time;
                        udelay(up->dmx512tx_status.cur_time_seq.mtbp_time);

                        do_gettimeofday(&cur_tv);

                        if(get_diff_time(&cur_tv, &up->dmx512tx_status.tx_timer.last_tv, &up->dmx512tx_status.tx_timer.diff_time) == 0)
                        {
                            //find max diff time
                            if(up->dmx512tx_status.tx_timer.max_diff < up->dmx512tx_status.tx_timer.diff_time)
                            {
                                up->dmx512tx_status.tx_timer.max_diff = up->dmx512tx_status.tx_timer.diff_time;
                            }

                            //find min diff time
                            if(up->dmx512tx_status.tx_timer.min_diff > up->dmx512tx_status.tx_timer.diff_time)
                            {
                                up->dmx512tx_status.tx_timer.min_diff = up->dmx512tx_status.tx_timer.diff_time;
                            }

                            //total diff time
                            t_usec = up->dmx512tx_status.tx_timer.total_diff.tv_usec;
                            t_usec += up->dmx512tx_status.tx_timer.diff_time;
                            up->dmx512tx_status.tx_timer.total_diff.tv_sec += t_usec / 1000000;
                            up->dmx512tx_status.tx_timer.total_diff.tv_usec = t_usec % 1000000;
                            
                            up->dmx512tx_status.cur_frame_freq = 1000 * 1000 * 1000 / up->dmx512tx_status.tx_timer.diff_time;
                        }            
                    }
                    else
                    {
                        up->dmx512tx_status.cur_frame_freq = 1000 * 1000 * 1000 / up->dmx512tx_status.tx_timer.diff_time;
                    }
                    break;
            }
        }
        else
        {
            up->dmx512tx_status.tx_timer.max_diff = 0;
            up->dmx512tx_status.tx_timer.min_diff = 0xFFFFFFFF;
            up->dmx512tx_status.tx_timer.diff_time = 0;
            up->dmx512tx_status.tx_timer.total_diff.tv_sec = 0;
            up->dmx512tx_status.tx_timer.total_diff.tv_usec = 0;
        }

        //record last timeval
        memcpy(&up->dmx512tx_status.tx_timer.last_tv, &cur_tv, sizeof(struct timeval));
    }
}

static void dmx512_rx_handle_done(struct uart_nuc970_port *up, unsigned int flags)
{
    unsigned long long cal_m, cal_d;
    unsigned long cal_mod;
    unsigned int result;
    
    //add flags
    up->dmx512rx_status.flags = flags;

    if(up->dmx512rx.cur_idx <= 255)
    {
        up->dmx512rx_status.flags |= DMX512_RX_STATUS_CACHE;
    }

    if(!((flags & DMX512_RX_STATUS_BREAK) && ((up->dmx512rx.cur_idx + 1) % 256 == 0)))
    {
        if(up->dmx512rx.p_data != (struct dmx512_data *)up->pmmapbuf)
        {
            up->pmmapbuf = (char *)up->dmx512rx.p_data;
            up->dmx512rx_status.flags |= DMX512_RX_STATUS_CACHE;
        }
    }

    //disable interupt
    serial_out(up, UART_REG_IER, serial_in(up, UART_REG_IER) & (~(RTO_IEN | RDA_IEN | TIME_OUT_EN)));

    //flush fifo
    serial_out(up, UART_REG_FCR, RFR);

    //Clear pending interrupts (not every bit are write 1 clear though...)
    serial_out(up, UART_REG_ISR, 0xFFFFFFFF);

    //calculate frame freq
    if(up->dmx512rx_status.rx_timer.diff_time != 0)
    {
        //fix last frame
        if(flags & DMX512_RX_STATUS_BREAK)
        {
            if((up->dmx512rx.wr_pos < DMX512_DATA_LEN) && (up->dmx512rx.wr_pos > 0))
            {
                if(up->dmx512rx.cur_idx > 0)
                {
                    if(up->dmx512rx_status.rx_timer.total_diff.tv_usec < up->dmx512rx_status.rx_timer.diff_time)
                    {
                        if(up->dmx512rx_status.rx_timer.total_diff.tv_sec > 0)
                        {
                            up->dmx512rx_status.rx_timer.total_diff.tv_sec--;
                            up->dmx512rx_status.rx_timer.total_diff.tv_usec += 1000000;
                            up->dmx512rx_status.rx_timer.total_diff.tv_usec -= up->dmx512rx_status.rx_timer.diff_time;
                        }
                    }
                    else
                    {
                        up->dmx512rx_status.rx_timer.total_diff.tv_usec -= up->dmx512rx_status.rx_timer.diff_time;
                    }
                }
            }
        }
        
        cal_m = up->dmx512rx_status.cur_frame;
        cal_m = cal_m * 1000 * 1000 * 10000;
        cal_d = up->dmx512rx_status.rx_timer.total_diff.tv_sec;
        cal_d = cal_d * 1000 * 1000 + up->dmx512rx_status.rx_timer.total_diff.tv_usec;
        cal_mod = do_div(cal_m, cal_d);
        result = cal_m;
        up->dmx512rx_status.frame_freq = (result / 10) + ((result % 10) + 5) / 10;
    }
    else
    {
        up->dmx512rx_status.frame_freq = 0;
    }

    //send wakeup event
    atomic_set(&up->rx_condition, 1);
    wake_up_interruptible(&up->rx_queue);
}

static void
dmx512_rx_chars(struct uart_nuc970_port *up)
{
	volatile unsigned char ch;
    volatile unsigned int isr;
	volatile unsigned int fsr;
	int max_count = 7;
    unsigned int flags = 0;

    isr = serial_in(up, UART_REG_ISR);
    fsr = serial_in(up, UART_REG_FSR);

    if(isr & RDA_IF)
    {
        if((up->rs485.rx_config.skip_frames > 0) && (up->dmx512rx.skip_cnt > 0))
        {
            while (max_count--)
            {
                //read head
                if(up->dmx512rx.wr_pos == 0)
                {
                    ch = (unsigned char)serial_in(up, UART_REG_RBR);
                    ch = (unsigned char)serial_in(up, UART_REG_RBR);
                    max_count -= 2;

                    //rx timer handle
                    dmx512_rx_timer_handle(up);
                }
                
        		ch = (unsigned char)serial_in(up, UART_REG_RBR);
        		
                if(up->dmx512rx.wr_pos < DMX512_DATA_LEN)
                {
        		    up->dmx512rx.wr_pos++;
                }
                else
                {
                    //receive is error
                    if((up->dmx512rx.cur_idx != 0) && ((up->rs485.flags & SER_RS485_DMX512_PAUSE) == 0))
                    {
                        flags = DMX512_RX_STATUS_BREAK | DMX512_RX_STATUS_TMSHORT;
                        dmx512_rx_handle_done(up, flags);
                        isr = 0;
                        fsr = 0;

                        up->rs485.flags &= (~SER_RS485_DMX512_PAUSE);
                        break;
                    }
                }

                fsr = serial_in(up, UART_REG_FSR);
            }
        }
        else
        {
            while (max_count--)
            {
                //read head
                if(up->dmx512rx.wr_pos == 0)
                {
                    ch = (unsigned char)serial_in(up, UART_REG_RBR);
                    ch = (unsigned char)serial_in(up, UART_REG_RBR);
                    max_count -= 2;

                    //rx timer handle
                    dmx512_rx_timer_handle(up);
                }
                
        		ch = (unsigned char)serial_in(up, UART_REG_RBR);
        		
                if(up->dmx512rx.wr_pos < DMX512_DATA_LEN)
                {
        		    up->dmx512rx.p_data[up->dmx512rx.cur_idx % 256].buf[up->dmx512rx.wr_pos++] = ch;
                }
                else
                {
                    //receive is error
                    if((up->dmx512rx.cur_idx != 0) && ((up->rs485.flags & SER_RS485_DMX512_PAUSE) == 0))
                    {
                        //skip last frame
                        if(up->dmx512rx.cur_idx > 0)
                            up->dmx512rx.cur_idx--;
                        
                        flags = DMX512_RX_STATUS_BREAK | DMX512_RX_STATUS_TMSHORT;
                        dmx512_rx_handle_done(up, flags);
                        isr = 0;
                        fsr = 0;

                        up->rs485.flags &= (~SER_RS485_DMX512_PAUSE);
                        break;
                    }
                }

                fsr = serial_in(up, UART_REG_FSR);
    	    }
        }
    }
    
    if(isr & TOUT_IF)
    {
        if((up->rs485.rx_config.skip_frames > 0) && (up->dmx512rx.skip_cnt > 0))
        {
            while (!(fsr & RX_EMPTY))
            {
        		ch = (unsigned char)serial_in(up, UART_REG_RBR);
        		
                if(up->dmx512rx.wr_pos < DMX512_DATA_LEN)
                {
        		    up->dmx512rx.wr_pos++;
                }

                fsr = serial_in(up, UART_REG_FSR);
    	    }

            //receive is error
            if(up->dmx512rx.wr_pos != DMX512_DATA_LEN)
            {
                //reset wr_pos
                up->dmx512rx.wr_pos = 0;
                
                if((up->dmx512rx.cur_idx != 0) && ((up->rs485.flags & SER_RS485_DMX512_PAUSE) == 0))
                {   
                    flags = DMX512_RX_STATUS_BREAK | DMX512_RX_STATUS_CHLOST;
                    dmx512_rx_handle_done(up, flags);
                    isr = 0;
                    fsr = 0;

                    up->rs485.flags &= (~SER_RS485_DMX512_PAUSE);
                    return;
                }
                else if(up->rs485.flags & SER_RS485_DMX512_PAUSE)
                {
                    //fix first frame
                    if(up->dmx512rx.cur_idx > 0)
                    {
                        if(up->dmx512rx_status.rx_timer.total_diff.tv_usec < up->dmx512rx_status.rx_timer.diff_time)
                        {
                            if(up->dmx512rx_status.rx_timer.total_diff.tv_sec > 0)
                            {
                                up->dmx512rx_status.rx_timer.total_diff.tv_sec--;
                                up->dmx512rx_status.rx_timer.total_diff.tv_usec += 1000000;
                                up->dmx512rx_status.rx_timer.total_diff.tv_usec -= up->dmx512rx_status.rx_timer.diff_time;
                            }
                        }
                        else
                        {
                            up->dmx512rx_status.rx_timer.total_diff.tv_usec -= up->dmx512rx_status.rx_timer.diff_time;
                        }
                    }
                    return;
                }
                else
                {
                    return;
                }
            }

            //reset wr_pos
            up->dmx512rx.wr_pos = 0;
        }
        else
        {
            while (!(fsr & RX_EMPTY))
            {
        		ch = (unsigned char)serial_in(up, UART_REG_RBR);
        		
                if(up->dmx512rx.wr_pos < DMX512_DATA_LEN)
                {
        		    up->dmx512rx.p_data[up->dmx512rx.cur_idx % 256].buf[up->dmx512rx.wr_pos++] = ch;
                }

                fsr = serial_in(up, UART_REG_FSR);
    	    }

            //receive is error
            if(up->dmx512rx.wr_pos != DMX512_DATA_LEN)
            {
                //reset wr_pos
                up->dmx512rx.wr_pos = 0;
                
                if((up->dmx512rx.cur_idx != 0) && ((up->rs485.flags & SER_RS485_DMX512_PAUSE) == 0))
                {
                    //skip last frame
                    if(up->dmx512rx.cur_idx > 0)
                        up->dmx512rx.cur_idx--;
                        
                    flags = DMX512_RX_STATUS_BREAK | DMX512_RX_STATUS_CHLOST;
                    dmx512_rx_handle_done(up, flags);
                    isr = 0;
                    fsr = 0;

                    up->rs485.flags &= (~SER_RS485_DMX512_PAUSE);
                    
                    return;
                }
                else if(up->rs485.flags & SER_RS485_DMX512_PAUSE)
                {
                    //fix first frame
                    if(up->dmx512rx.cur_idx > 0)
                    {
                        if(up->dmx512rx_status.rx_timer.total_diff.tv_usec < up->dmx512rx_status.rx_timer.diff_time)
                        {
                            if(up->dmx512rx_status.rx_timer.total_diff.tv_sec > 0)
                            {
                                up->dmx512rx_status.rx_timer.total_diff.tv_sec--;
                                up->dmx512rx_status.rx_timer.total_diff.tv_usec += 1000000;
                                up->dmx512rx_status.rx_timer.total_diff.tv_usec -= up->dmx512rx_status.rx_timer.diff_time;
                            }
                        }
                        else
                        {
                            up->dmx512rx_status.rx_timer.total_diff.tv_usec -= up->dmx512rx_status.rx_timer.diff_time;
                        }
                    }
                    return;
                }
                else
                {
                    return;
                }
            }

            //reset wr_pos
            up->dmx512rx.wr_pos = 0;

            //increase cur index
            if(up->dmx512rx.cur_idx < (up->dmx512rx.total_num - 1))
            {
                //change cache
                if((up->dmx512rx.cur_idx % 256) == 255)
                {
                    if(up->dmx512rx.p_data == (struct dmx512_data *)up->pallocrxbuf1)
                    {
                        up->dmx512rx.p_data = (struct dmx512_data *)up->pallocrxbuf2;
                        up->pmmapbuf = up->pallocrxbuf1;
                    }
                    else
                    {
                        up->dmx512rx.p_data = (struct dmx512_data *)up->pallocrxbuf1;
                        up->pmmapbuf = up->pallocrxbuf2;
                    }

                    //add flags
                    up->dmx512rx_status.flags = DMX512_RX_STATUS_CACHE;

                    //send wakeup event
                    atomic_set(&up->rx_condition, 1);
                    wake_up_interruptible(&up->rx_queue);
                }

                up->dmx512rx.cur_idx++;
            }
            else
            {
                //receive done
                flags = DMX512_RX_STATUS_DONE;

                dmx512_rx_handle_done(up, flags);
            }
        } 

        if(up->rs485.rx_config.skip_frames > 0)
        {
            if(up->dmx512rx.skip_cnt >= up->rs485.rx_config.skip_frames)
            {
                up->dmx512rx.skip_cnt = 0;
            }
            else
            {
                up->dmx512rx.skip_cnt++;
            }
        }

        if(up->rs485.flags & SER_RS485_DMX512_PAUSE)
        {
            up->rs485.flags &= (~SER_RS485_DMX512_PAUSE);
        }
    }
}

static void dmx512_tx_chars(struct uart_nuc970_port *up)
{
    int count = 16;

    while(count--)
    {
        if(up->dmx512tx.p_start == up->dmx512tx.p_end)
        {
            __stop_tx(up);
            atomic_set(&up->tx_condition, 1);
            wake_up_interruptible(&up->tx_queue); 
            return;
        }

        if(serial_in(up, UART_REG_FSR) & TX_FULL)
            break;
        serial_out(up, UART_REG_THR, *up->dmx512tx.p_start++);
    }
}

static unsigned int check_modem_status(struct uart_nuc970_port *up)
{
	unsigned int status = 0;

	if (0) {

		wake_up_interruptible(&up->port.state->port.delta_msr_wait);
	}

	return status;
}

static irqreturn_t nuc970serial_interrupt(int irq, void *dev_id)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)dev_id;
	volatile unsigned int isr;
    volatile unsigned int ier;
    unsigned long irq_flags;

    if(up->rs485.flags & (SER_RS485_DMX512_TX | SER_RS485_DMX512_RX))
    {
        isr = serial_in(up, UART_REG_ISR);
        ier = serial_in(up, UART_REG_IER);

        if(ier & (RTO_IEN | RDA_IEN | TIME_OUT_EN))
        {
            if (isr & (RDA_IF | TOUT_IF))
            {
                //disable all irq avoid break record
                local_irq_save(irq_flags);

        		dmx512_rx_chars(up);

                //enable irq
                local_irq_restore(irq_flags);
            }
            else if ((isr & THRE_IF) == 0)
            {
                serial_out(up, UART_REG_FSR, RX_OVER_IF | PEF | FEF | BIF | TX_OVER_IF);
                printk("error:uart%d irq trigger error!\n", up->port.line);
                printk("isr is 0x%08X\n", isr);
            }
        }

    	check_modem_status(up);

        if(ier & THRE_IEN)
        {
            if (isr & THRE_IF)
            {   
        		dmx512_tx_chars(up);            
            }
        }
    }
    else
    {
        isr = serial_in(up, UART_REG_ISR);

    	if (isr & (RDA_IF | TOUT_IF))
    		receive_chars(up);
    	
    	check_modem_status(up);
    	
    	if (isr & THRE_IF)
    		transmit_chars(up);
    }

	return IRQ_HANDLED;
}

static unsigned int nuc970serial_tx_empty(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	//unsigned long flags;
	unsigned int fsr;

	//spin_lock_irqsave(&up->port.lock, flags);
	fsr = serial_in(up, UART_REG_FSR);
	//spin_unlock_irqrestore(&up->port.lock, flags);

	return (fsr & (TE_FLAG | TX_EMPTY)) == (TE_FLAG | TX_EMPTY) ? TIOCSER_TEMT : 0;
}

static unsigned int nuc970serial_get_mctrl(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	unsigned int status;
	unsigned int ret = 0;

	//status = check_modem_status(up);

	status = serial_in(up, UART_REG_MSR);;
	
	if(status & 0x10)
		ret |= TIOCM_CTS;

	return ret;
}

static void nuc970serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS)
	{
		// set RTS high level trigger
		mcr = serial_in(up, UART_REG_MCR);
		mcr |= 0x200;
		mcr &= ~(0x2);
	}

	if (up->mcr & UART_MCR_AFE)
	{
		// set RTS high level trigger
		mcr = serial_in(up, UART_REG_MCR);
		mcr |= 0x200;
		mcr &= ~(0x2);

		// enable CTS/RTS auto-flow control
		serial_out(up, UART_REG_IER, (serial_in(up, UART_REG_IER) | (0x3000)));
	}

	// set CTS high level trigger
	serial_out(up, UART_REG_MSR, (serial_in(up, UART_REG_MSR) | (0x100)));
	serial_out(up, UART_REG_MCR, mcr);
}

static void nuc970serial_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	unsigned long flags;
	unsigned int lcr;

	spin_lock_irqsave(&up->port.lock, flags);
	lcr = serial_in(up, UART_REG_LCR);
	if (break_state != 0)
		lcr |= BCB;	// set break
	else
		lcr &= ~BCB;	// clr break
	serial_out(up, UART_REG_LCR, lcr);
	spin_unlock_irqrestore(&up->port.lock, flags);
}

static int nuc970serial_startup(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	struct tty_struct *tty = port->state->port.tty;
	int retval;
	
	//  TODO: configure pin function and enable engine clock
	//nuc970serial_pinctrl();

	/* Reset FIFO */
	serial_out(up, UART_REG_FCR, TFR | RFR /* | RX_DIS */);

	/* Clear pending interrupts (not every bit are write 1 clear though...) */
	serial_out(up, UART_REG_ISR, 0xFFFFFFFF);

	retval = request_irq(port->irq, nuc970serial_interrupt, 0,
			tty ? tty->name : "nuc970_serial", port);

	if (retval) {
		printk("request irq failed...\n");
		return retval;
	}

	/*
	 * Now, initialize the UART
	 */
	serial_out(up, UART_REG_FCR, serial_in(up, UART_REG_FCR) | 0x10);	// Trigger level 4 byte
	serial_out(up, UART_REG_LCR, 0x7);						// 8 bit
	serial_out(up, UART_REG_TOR, 0x40);
	serial_out(up, UART_REG_IER, RTO_IEN | RDA_IEN | TIME_OUT_EN);
	
	/* 12MHz reference clock input, 115200 */
	serial_out(up, UART_REG_BAUD, 0x30000066);
	
	return 0;
}

static void nuc970serial_shutdown(struct uart_port *port)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	//unsigned long flags;
	free_irq(port->irq, port);

	/*
	 * Disable interrupts from this port
	 */
	serial_out(up, UART_REG_IER, 0);
	serial_out(up, UART_REG_FCR, TFR | RFR /* | RX_DIS */);

}

static unsigned int nuc970serial_get_divisor(struct uart_port *port, unsigned int baud)
{
	unsigned int quot;

	quot = (port->uartclk / baud) - 2;

	return quot;
}

static void
nuc970serial_set_termios(struct uart_port *port, struct ktermios *termios,
		       struct ktermios *old)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;
	unsigned int lcr = 0;
	unsigned long flags;
	unsigned int baud, quot;

	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = 0;
		break;
	case CS6:
		lcr |= 1;
		break;
	case CS7:
		lcr |= 2;
		break;
	default:
	case CS8:
		lcr |= 3;
		break;
	}

	if (termios->c_cflag & CSTOPB)
		lcr |= NSB;
	if (termios->c_cflag & PARENB)
		lcr |= PBE;
	if (!(termios->c_cflag & PARODD))
		lcr |= EPE;
	if (termios->c_cflag & CMSPAR)
		lcr |= SPE;

	baud = uart_get_baud_rate(port, termios, old,
				  port->uartclk / 16 / 0xffff,
				  port->uartclk / 16);

    /*
	 * baud 230400 force to 250000,edit by lzy 2016.10.24
	 */
	if(baud == 230400)
	{
		baud = 250000;
        quot = nuc970serial_get_divisor(port, baud);
        quot = 47;
        printk("uart set baud is %d\n", baud);
	    printk("uart set quot is %d\n", quot);
	}
    else
    {
        quot = nuc970serial_get_divisor(port, baud);
    }

	/*
	 * Ok, we're now changing the port state.  Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&up->port.lock, flags);

	up->port.read_status_mask = RX_OVER_IF /*| UART_LSR_THRE | UART_LSR_DR*/;
	if (termios->c_iflag & INPCK)
		up->port.read_status_mask |= FEF | PEF;
	if (termios->c_iflag & (BRKINT | PARMRK))
		up->port.read_status_mask |= BIF;

	/*
	 * Characteres to ignore
	 */
	up->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		up->port.ignore_status_mask |= FEF | PEF;
	if (termios->c_iflag & IGNBRK) {
		up->port.ignore_status_mask |= BIF;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			up->port.ignore_status_mask |= RX_OVER_IF;
	}

	if (termios->c_cflag & CRTSCTS)
		up->mcr |= UART_MCR_AFE;
	else
		up->mcr &= ~UART_MCR_AFE;

	nuc970serial_set_mctrl(&up->port, up->port.mctrl);

	serial_out(up, UART_REG_BAUD, quot | 0x30000000);
	serial_out(up, UART_REG_LCR, lcr);		

	spin_unlock_irqrestore(&up->port.lock, flags);

}

static void
nuc970serial_set_ldisc(struct uart_port *port, int ld)
{
	struct uart_nuc970_port *uart = (struct uart_nuc970_port *)port;
	unsigned int baud;

	switch (ld) {
	case N_IRDA:

		baud = serial_in(uart, UART_REG_BAUD);
		baud = baud & (0x0000ffff);
		baud = baud + 2;
		baud = baud / 16;
		baud = baud - 2;

		serial_out(uart, UART_REG_BAUD, baud);
		serial_out(uart, UART_REG_IRCR, (serial_in(uart, UART_REG_IRCR) & ~0x40) );  // Rx inverse
			
		serial_out(uart, UART_FUN_SEL, (serial_in(uart, UART_FUN_SEL) & ~FUN_SEL_Msk) );
		serial_out(uart, UART_FUN_SEL, (serial_in(uart, UART_FUN_SEL) | FUN_SEL_IrDA) );

		break;
	default:
		serial_out(uart, UART_FUN_SEL, (serial_in(uart, UART_FUN_SEL) & ~FUN_SEL_Msk) );
	}

}

static void
nuc970serial_pm(struct uart_port *port, unsigned int state,
	      unsigned int oldstate)
{
	struct uart_nuc970_port *p = (struct uart_nuc970_port *)port;


	if (p->pm)
		p->pm(port, state, oldstate);
}

static void nuc970serial_release_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	int size = pdev->resource[0].end - pdev->resource[0].start + 1;
    struct uart_nuc970_port *p = to_nuc970_uart_port(port);
    struct page *page;

    //if haved alloc mem,free it
    if(p->palloctxbuf > 0)
    {
        //free page
        for (page = virt_to_page(p->palloctxbuf); page < virt_to_page(p->palloctxbuf + PAGE_ALIGN(DMX512_DATA_LEN)); page++) {  
            ClearPageReserved(page);  
        }  
        kfree(p->palloctxbuf); 

        mutex_destroy(&p->dmx512tx.mutex_lock);
    }
    
    //if haved alloc mem,free it
    if(p->pallocrxbuf1 > 0)
    {
        //free page
        for (page = virt_to_page(p->pallocrxbuf1); page < virt_to_page(p->pallocrxbuf1 + PAGE_ALIGN(256 * DMX512_DATA_LEN)); page++) {  
            ClearPageReserved(page);  
        }  
        kfree(p->pallocrxbuf1); 
    }

    //if haved alloc mem,free it
    if(p->pallocrxbuf2 > 0)
    {
        //free page
        for (page = virt_to_page(p->pallocrxbuf2); page < virt_to_page(p->pallocrxbuf2 + PAGE_ALIGN(256 * DMX512_DATA_LEN)); page++) {  
            ClearPageReserved(page);  
        }  
        kfree(p->pallocrxbuf2); 

        mutex_destroy(&p->dmx512rx.mutex_lock);
    }

    p->pmmapbuf = 0;
    p->palloctxbuf = 0;
    p->pallocrxbuf1 = 0;
    p->pallocrxbuf2 = 0;
    p->malloc_size = 0;
 
	release_mem_region(port->mapbase, size);

	iounmap(port->membase);
	port->membase = NULL;


}

static int nuc970serial_request_port(struct uart_port *port)
{
	return 0;
}

static void nuc970serial_config_port(struct uart_port *port, int flags)
{
	int ret;

	/*
	 * Find the region that we can probe for.  This in turn
	 * tells us whether we can probe for the type of port.
	 */
	ret = nuc970serial_request_port(port);
	if (ret < 0)
		return;
	port->type = PORT_NUC970;


}

static int
nuc970serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_NUC970)
		return -EINVAL;
	return 0;
}

static const char *
nuc970serial_type(struct uart_port *port)
{

	return (port->type == PORT_NUC970) ? "NUC970" : NULL;
}

/* Enable or disable the rs485 support */
void nuc970serial_config_rs485(struct uart_port *port, struct serial_rs485 *rs485conf)
{
	struct uart_nuc970_port *p = to_nuc970_uart_port(port);

    spin_lock(&port->lock);

    p->rs485 = *rs485conf;

    if (p->rs485.delay_rts_before_send >= 1000)
    	p->rs485.delay_rts_before_send = 1000;

    //init 485 trans config, default bit9 '1'
    serial_out(p, UART_REG_LCR, serial_in(p, UART_REG_LCR) &~ (EPE | NSB | BCB));
    serial_out(p, UART_REG_LCR, serial_in(p, UART_REG_LCR) | PBE | SPE);

    serial_out(p, UART_FUN_SEL, (serial_in(p, UART_FUN_SEL) & ~FUN_SEL_Msk) );
    	
	serial_out(p, UART_FUN_SEL, (serial_in(p, UART_FUN_SEL) | FUN_SEL_RS485) );

	//rs485_start_rx(p);	// stay in Rx mode 	

	if(rs485conf->flags & SER_RS485_RTS_ON_SEND)
	{
		serial_out(p, UART_REG_MCR, (serial_in(p, UART_REG_MCR) & ~0x200) );
	}
	else
	{
		serial_out(p, UART_REG_MCR, (serial_in(p, UART_REG_MCR) | 0x200) );
	}
	
	// set auto direction mode
	serial_out(p,UART_REG_ALT_CSR,(serial_in(p, UART_REG_ALT_CSR) | (1 << 10)) );

    //set NMM, ADDEN
    serial_out(p, UART_REG_ALT_CSR, serial_in(p, UART_REG_ALT_CSR) | (1 << 8) | (1<<15)); 

    spin_unlock(&port->lock);
}

/* Enable or disable the rs485 support */
void nuc970serial_config_rs485_dmx512tx(struct uart_port *port, struct serial_rs485 *rs485conf)
{
	struct uart_nuc970_port *p = to_nuc970_uart_port(port);
    unsigned int reg_val;

	spin_lock(&port->lock);

    //clear bit9 control and interupt enable flags
	serial_out(p, UART_REG_LCR, 0x7);// 8 bit
    serial_out(p, UART_REG_IER, serial_in(p, UART_REG_IER) & (~(THRE_IEN | RTO_IEN | RDA_IEN | TIME_OUT_EN)));
    
	p->rs485 = *rs485conf;

	if (p->rs485.delay_rts_before_send >= 1000)
		p->rs485.delay_rts_before_send = 1000;

	serial_out(p, UART_FUN_SEL, (serial_in(p, UART_FUN_SEL) & ~FUN_SEL_Msk) );
		
	serial_out(p, UART_FUN_SEL, (serial_in(p, UART_FUN_SEL) | FUN_SEL_RS485) );

	//rs485_start_rx(p);	// stay in Rx mode 	

	if(rs485conf->flags & SER_RS485_RTS_ON_SEND)
	{
		serial_out(p, UART_REG_MCR, (serial_in(p, UART_REG_MCR) & ~0x200) );
	}
	else
	{
		serial_out(p, UART_REG_MCR, (serial_in(p, UART_REG_MCR) | 0x200) );
	}
	
	// set auto direction mode
	serial_out(p,UART_REG_ALT_CSR,(serial_in(p, UART_REG_ALT_CSR) | (1 << 10)) );

    //set force parity check to "1",for nine bit detect
    serial_out(p, UART_REG_LCR, serial_in(p, UART_REG_LCR) &~ (EPE | NSB | BCB));
    serial_out(p, UART_REG_LCR, serial_in(p, UART_REG_LCR) | PBE | SPE);

    //check between_time, set tor
    if((p->rs485.tx_config.mode == SET_WITH_SEQ) || (p->rs485.tx_config.mode == SET_WITH_SEQ_AND_PERIOD))
    {
        reg_val = serial_in(p, UART_REG_TOR) &~ (0xFF << 8);

        //between_time = x(us)/(1/250000) = x/4
        reg_val |= ((p->rs485.tx_config.time_seq.between_time / 4) << 8);
        
        serial_out(p, UART_REG_TOR, reg_val);
    }

	spin_unlock(&port->lock);
}

/* Enable or disable the rs485 support */
void nuc970serial_config_rs485_dmx512rx(struct uart_port *port, struct serial_rs485 *rs485conf)
{
	struct uart_nuc970_port *p = to_nuc970_uart_port(port);
    unsigned int timeout = 0;

	spin_lock(&port->lock);

    __stop_tx(p);

    //clear bit9 control and interupt enable flags
    serial_out(p, UART_REG_LCR, 0x7);// 8 bit
    serial_out(p, UART_REG_IER, serial_in(p, UART_REG_IER) & (~(THRE_IEN | RTO_IEN | RDA_IEN | TIME_OUT_EN)));

	p->rs485 = *rs485conf;

	if (p->rs485.delay_rts_before_send >= 1000)
		p->rs485.delay_rts_before_send = 1000;

	serial_out(p, UART_FUN_SEL, (serial_in(p, UART_FUN_SEL) & ~FUN_SEL_Msk) );
		
	serial_out(p, UART_FUN_SEL, (serial_in(p, UART_FUN_SEL) | FUN_SEL_RS485) );

	if(rs485conf->flags & SER_RS485_RTS_ON_SEND)
	{
		serial_out(p, UART_REG_MCR, (serial_in(p, UART_REG_MCR) & ~0x200) );
	}
	else
	{
		serial_out(p, UART_REG_MCR, (serial_in(p, UART_REG_MCR) | 0x200) );
	}
	
	// set auto direction mode
	serial_out(p,UART_REG_ALT_CSR,(serial_in(p, UART_REG_ALT_CSR) | (1 << 10)) );

    //set NMM, ADDEN
    serial_out(p, UART_REG_ALT_CSR, serial_in(p, UART_REG_ALT_CSR) | (1 << 8) | (1<<15));  
    
    //set trigger level 8 byte
    serial_out(p, UART_REG_FCR, serial_in(p, UART_REG_FCR) & (~0xF0));
    serial_out(p, UART_REG_FCR, serial_in(p, UART_REG_FCR) | 0x30);

    //disable bit9 is "0" data
    serial_out(p, UART_REG_FCR, serial_in(p, UART_REG_FCR) | (1 << 8));

    //set tor is max, buad 250000 is 1020us
    timeout = p->rs485.rx_config.frame_timeout / 4;
    if((timeout >= 20) && (timeout <= 255))
    {
        serial_out(p, UART_REG_TOR, timeout);
    }
    else if(timeout == 0)
    {
        serial_out(p, UART_REG_TOR, 128);
    }
    else if(timeout < 20)
    {
        serial_out(p, UART_REG_TOR, 20);
    }
    else if(timeout > 255)
    {
        serial_out(p, UART_REG_TOR, 255);
    }
    
    //enable RTO RDA interupt
    serial_out(p, UART_REG_IER, serial_in(p, UART_REG_IER) | (RTO_IEN | RDA_IEN | TIME_OUT_EN));

	spin_unlock(&port->lock);
}

static int
nuc970serial_rs485_mmap(struct uart_port *port, struct vm_area_struct * vma)
{
    struct uart_nuc970_port *p = to_nuc970_uart_port(port);
    unsigned long start = vma->vm_start;  
    unsigned long size = PAGE_ALIGN(vma->vm_end - vma->vm_start);  

    if (!(p->pmmapbuf)) {  
        return -EINVAL;  
    } 
    
    if(size > p->malloc_size)
    {
        size = p->malloc_size;
    }

    return remap_pfn_range(vma, start, (virt_to_phys(p->pmmapbuf) >> PAGE_SHIFT), size, PAGE_SHARED);
}

static void generate_mark_for_dmx512tx(struct uart_nuc970_port *p)
{
    unsigned int port_line = p->port.line;
    unsigned int regRec;
    volatile unsigned int *reg;
    
    //release uart,init for gpio,gpio mode,out,high
    reg = tx_reg_func[port_line];
    regRec = __raw_readl(reg);
    __raw_writel(regRec & (~tx_pos_func[port_line]), reg);
    reg = tx_reg_dir[port_line];
    __raw_writel(__raw_readl(reg) | tx_pos_dir[port_line], reg);
    reg = tx_reg_out[port_line];
    __raw_writel(__raw_readl(reg) | tx_pos_out[port_line], reg);
    
    //generate mark time between packets
    dmx512_tx_timer_handle(p);
    
    //generate break info
    reg = tx_reg_out[port_line];
    __raw_writel(__raw_readl(reg) & (~tx_pos_out[port_line]), reg);
    udelay(p->dmx512tx_status.cur_time_seq.break_time);
    __raw_writel(__raw_readl(reg) | tx_pos_out[port_line], reg);
    
    //generate mark after break info
    udelay(p->dmx512tx_status.cur_time_seq.mab_time);
    
    //release gpio,restore for uart
    reg = tx_reg_func[port_line];
    __raw_writel(regRec, reg);
}

static int
nuc970serial_rs485_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
    struct uart_nuc970_port *p = to_nuc970_uart_port(port);
	struct serial_rs485 rs485conf;
    struct dmx512_rx_status rx_status;
    struct dmx512_tx_status tx_status;
    struct page *page;
    unsigned long isr_flags;
    unsigned long total_time_seq;
    int ret = 0;

	switch (cmd) {
	case TIOCSRS485:
		if (copy_from_user(&rs485conf, (struct serial_rs485 *) arg,
					sizeof(rs485conf)))
			return -EFAULT;

		nuc970serial_config_rs485(port, &rs485conf);
		break;

	case TIOCGRS485:
		if (copy_to_user((struct serial_rs485 *) arg,
					&(to_nuc970_uart_port(port)->rs485),
					sizeof(rs485conf)))
			return -EFAULT;
		break;
        
    case TIOCDMX512SET:
        if (copy_from_user(&rs485conf, (struct serial_rs485 *) arg,
					sizeof(rs485conf)))
			return -EFAULT;

        p->rs485 = rs485conf;

        //disable rx
        serial_out(p, UART_REG_IER, 0);

        //malloc space for send or receive
        if(p->rs485.flags & SER_RS485_DMX512_TX)
        {
            //check mode set corrent
            if(p->rs485.tx_config.mode == SET_WITH_SEQ)
            {
                if(p->rs485.tx_config.time_seq.between_time / 4 > 255)
                {
                    printk("dmx512 tx error: between_time set too long!\n");
                    return -EFAULT;
                }
            }
            
            if(p->rs485.tx_config.mode == SET_WITH_SEQ_AND_PERIOD)
            {
                if(p->rs485.tx_config.time_seq.between_time / 4 > 255)
                {
                    printk("dmx512 tx error: between_time set too long!\n");
                    return -EFAULT;
                }

                total_time_seq = p->rs485.tx_config.time_seq.break_time;
                total_time_seq += p->rs485.tx_config.time_seq.mab_time;
                total_time_seq += (11 * 4 + p->rs485.tx_config.time_seq.between_time) * 512 + 11 * 4;//data had 11bits
                if(total_time_seq > p->rs485.tx_config.time_seq.period)
                {
                    printk("dmx512 tx error: time seq is more than period!\n");
                    return -EFAULT;
                }
            }
            
            //align size
            p->malloc_size = PAGE_ALIGN(DMX512_DATA_LEN);
            
            //init once
            if(p->palloctxbuf == 0)
            {
                //alloc mem for dmx512_data
                p->palloctxbuf = kmalloc(p->malloc_size, GFP_ATOMIC);  
                if (!p->palloctxbuf) {
                    return -EFAULT;
                }
                
                for (page = virt_to_page(p->palloctxbuf); page < virt_to_page(p->palloctxbuf + p->malloc_size); page++) {  
                    SetPageReserved(page);  
                }

                //init lock
                spin_lock_init(&p->dmx512tx.spin_lock);
                mutex_init(&p->dmx512tx.mutex_lock);
            }

            //clear buf
            memset(p->palloctxbuf, 0, p->malloc_size);

            //init dmx512_tx            
            p->dmx512tx.p_start = 0;
            p->dmx512tx.p_end = 0;
            p->dmx512tx.p_data = (struct dmx512_data *)p->palloctxbuf;
            p->pmmapbuf = p->palloctxbuf;

            //init status
            memset((char *)&p->dmx512tx_status, 0, sizeof(struct dmx512_tx_status));
            p->dmx512tx_status.tx_timer.min_diff = 0xFFFFFFFF;

            //copy time sequence and mode
            p->dmx512tx_status.cur_mode = p->rs485.tx_config.mode;
            switch(p->rs485.tx_config.mode)
            {
                case SET_WITH_SEQ:
                case SET_WITH_SEQ_AND_PERIOD:
                    p->dmx512tx_status.cur_time_seq = p->rs485.tx_config.time_seq;
                    break;
                case SET_WITH_FREQ:
                default:
                    p->dmx512tx_status.cur_time_seq.mtbp_time = DEFAULT_MTBP_TIME;
                    p->dmx512tx_status.cur_time_seq.mab_time = DEFAULT_MAB_TIME;
                    p->dmx512tx_status.cur_time_seq.break_time = DEFAULT_BREAK_TIME;
                    p->dmx512tx_status.cur_time_seq.between_time = DEFAULT_BETWEEN_TIME;
                    p->dmx512tx_status.cur_time_seq.freq = p->rs485.tx_config.time_seq.freq;
                    if(p->dmx512tx_status.cur_time_seq.freq > 0)
                    {
                        p->dmx512tx_status.cur_time_seq.period = 1000 * 1000 * 1000 / p->dmx512tx_status.cur_time_seq.freq;
                    }
                    else
                    {
                        p->dmx512tx_status.cur_time_seq.period = 0;
                    }
                    break;
                    
            }

            //config registers
            nuc970serial_config_rs485_dmx512tx(port, &rs485conf);
        }
        else if(p->rs485.flags & SER_RS485_DMX512_RX)
        {
            //if total frames isn't positive
            if(p->rs485.rx_config.record_frames <= 0)
            {
                return -EFAULT;
            }

            //align size
            p->malloc_size = PAGE_ALIGN(256 * DMX512_DATA_LEN);

            //init once
            if(p->pallocrxbuf1 == 0)
            {
                //alloc mem for dmx512_data
                p->pallocrxbuf1 = kmalloc(p->malloc_size, GFP_ATOMIC);  
                if (!p->pallocrxbuf1) {
                    return -EFAULT;
                }
                
                for (page = virt_to_page(p->pallocrxbuf1); page < virt_to_page(p->pallocrxbuf1 + p->malloc_size); page++) {  
                    SetPageReserved(page);  
                }
            }

            //init once
            if(p->pallocrxbuf2 == 0)
            {
                //alloc mem for dmx512_data
                p->pallocrxbuf2 = kmalloc(p->malloc_size, GFP_ATOMIC);  
                if (!p->pallocrxbuf2) {
                    return -EFAULT;
                }
                
                for (page = virt_to_page(p->pallocrxbuf2); page < virt_to_page(p->pallocrxbuf2 + p->malloc_size); page++) {  
                    SetPageReserved(page);  
                }

                //init dmx512_rx
                spin_lock_init(&p->dmx512rx.spin_lock);
                mutex_init(&p->dmx512rx.mutex_lock);
            }

            //clear buf
            memset(p->pallocrxbuf1, 0, p->malloc_size);
            memset(p->pallocrxbuf2, 0, p->malloc_size);

            p->dmx512rx.cur_idx = 0;
            p->dmx512rx.wr_pos = 0;
            p->dmx512rx.skip_cnt = 0;
            p->dmx512rx.total_num = p->rs485.rx_config.record_frames;
            p->dmx512rx.p_data = (struct dmx512_data *)p->pallocrxbuf1;
            p->pmmapbuf = p->pallocrxbuf1;
            atomic_set(&p->rx_condition, 0);
            p->rs485.flags &= (~SER_RS485_DMX512_PAUSE);

            //init status
            memset((char *)&p->dmx512rx_status, 0, sizeof(struct dmx512_rx_status));
            p->dmx512rx_status.rx_timer.min_diff = 0xFFFFFFFF;

            //config registers
            nuc970serial_config_rs485_dmx512rx(port, &rs485conf);
        }
		break;
        
    case TIOCDMX512GET:
        if (copy_to_user((struct serial_rs485 *) arg,
					&(to_nuc970_uart_port(port)->rs485),
					sizeof(rs485conf)))
			return -EFAULT;
		break;
        
    case TIOCDMX512TX:
        //mutex lock
        mutex_lock(&p->dmx512tx.mutex_lock);
        
        //stop uart tx
        nuc970serial_stop_tx(port);
       
        //init pointer
        p->dmx512tx.p_start = p->dmx512tx.p_data->buf;
        p->dmx512tx.p_end = p->dmx512tx.p_start + DMX512_DATA_LEN;

        //flush tx fifo
        serial_out(p, UART_REG_FCR, serial_in(p, UART_REG_FCR) | TFR);

        //gen mark
        generate_mark_for_dmx512tx(p);

        //send empty
        serial_out(p, UART_REG_THR, 0);
        
        //uart send
        nuc970serial_start_tx(port);

        if(atomic_read(&p->tx_condition) == 0)
        {
            wait_event_interruptible_timeout(p->tx_queue, atomic_read(&p->tx_condition), 1*HZ);
        }

        atomic_set(&p->tx_condition, 0);

        ret = p->dmx512tx_status.cur_frame_freq;

        //mutex unlock
        mutex_unlock(&p->dmx512tx.mutex_lock);
        break;
    case TIOCDMX512RX:
        //mutex lock
        mutex_lock(&p->dmx512rx.mutex_lock);

        if(atomic_read(&p->rx_condition) == 0)
        {
            ret = wait_event_interruptible_timeout(p->rx_queue, atomic_read(&p->rx_condition), 1*HZ / 100);
            if(ret == 0)
            {
                ret = -1;
            }
            else
            {
                ret = 0;
            }
        }

        atomic_set(&p->rx_condition, 0);

        //mutex unlock
        mutex_unlock(&p->dmx512rx.mutex_lock);

        //copy status
        spin_lock_irqsave(&p->dmx512rx.spin_lock, isr_flags);
        p->dmx512rx_status.cur_frame = p->dmx512rx.cur_idx;
        memcpy((char *)&rx_status, (char *)&p->dmx512rx_status, sizeof(struct dmx512_rx_status));
        p->dmx512rx_status.flags &= DMX512_RX_STATUS_CACHE;
        spin_unlock_irqrestore(&p->dmx512rx.spin_lock, isr_flags);

        if (copy_to_user((struct dmx512_rx_status*) arg,
					&rx_status,
					sizeof(struct dmx512_rx_status)))
			return -EFAULT;
        break;

    case TIOCDMX512RXSTOP:
        spin_lock_irqsave(&p->dmx512rx.spin_lock, isr_flags);
        rx_status.flags = p->dmx512rx_status.flags | DMX512_RX_STATUS_BREAK;
        spin_unlock_irqrestore(&p->dmx512rx.spin_lock, isr_flags);
        dmx512_rx_handle_done(p, rx_status.flags);

        //if last frame not enough, skip it
        if((p->dmx512rx.wr_pos < DMX512_DATA_LEN) && (p->dmx512rx.wr_pos > 0))
        {
            p->dmx512rx.wr_pos = 0;
            if(p->dmx512rx.cur_idx > 0)
                p->dmx512rx.cur_idx--;
        }

        //copy status
        p->dmx512rx_status.cur_frame = p->dmx512rx.cur_idx;
        memcpy((char *)&rx_status, (char *)&p->dmx512rx_status, sizeof(struct dmx512_rx_status));
        
        if (copy_to_user((struct dmx512_rx_status*) arg,
					&rx_status,
					sizeof(struct dmx512_rx_status)))
			return -EFAULT;
        break;

    case TIOCDMX512RXPAUSE:
        spin_lock_irqsave(&p->dmx512rx.spin_lock, isr_flags);
        serial_out(p, UART_REG_IER, serial_in(p, UART_REG_IER) & (~(RTO_IEN | RDA_IEN | TIME_OUT_EN)));
        p->rs485.flags |= SER_RS485_DMX512_PAUSE;
        spin_unlock_irqrestore(&p->dmx512rx.spin_lock, isr_flags);

        //clear last
        if(p->dmx512rx.wr_pos > 0)
        {
            p->dmx512rx.wr_pos = 0;

            //fix last frame
            if(p->dmx512rx.cur_idx > 0)
            {
                if(p->dmx512rx_status.rx_timer.total_diff.tv_usec < p->dmx512rx_status.rx_timer.diff_time)
                {
                    if(p->dmx512rx_status.rx_timer.total_diff.tv_sec > 0)
                    {
                        p->dmx512rx_status.rx_timer.total_diff.tv_sec--;
                        p->dmx512rx_status.rx_timer.total_diff.tv_usec += 1000000;
                        p->dmx512rx_status.rx_timer.total_diff.tv_usec -= p->dmx512rx_status.rx_timer.diff_time;
                    }
                }
                else
                {
                    p->dmx512rx_status.rx_timer.total_diff.tv_usec -= p->dmx512rx_status.rx_timer.diff_time;
                }
            }
        }
        printk("paused!\n");
        break;

    case TIOCDMX512RXCONTINUE:
        spin_lock_irqsave(&p->dmx512rx.spin_lock, isr_flags);
        serial_out(p, UART_REG_IER, serial_in(p, UART_REG_IER) | (RTO_IEN | RDA_IEN | TIME_OUT_EN));
        spin_unlock_irqrestore(&p->dmx512rx.spin_lock, isr_flags);
        break;

    case TIOCDMX512RXSTATUS:
        spin_lock_irqsave(&p->dmx512rx.spin_lock, isr_flags);
        p->dmx512rx_status.cur_frame = p->dmx512rx.cur_idx;
        memcpy((char *)&rx_status, (char *)&p->dmx512rx_status, sizeof(struct dmx512_rx_status));
        spin_unlock_irqrestore(&p->dmx512rx.spin_lock, isr_flags);
        
        if (copy_to_user((struct dmx512_rx_status*) arg,
					&rx_status,
					sizeof(struct dmx512_rx_status)))
			return -EFAULT;
        break;
        
    case TIOCDMX512TXSTATUS:
        spin_lock_irqsave(&p->dmx512tx.spin_lock, isr_flags);
        memcpy((char *)&tx_status, (char *)&p->dmx512tx_status, sizeof(struct dmx512_tx_status));
        spin_unlock_irqrestore(&p->dmx512tx.spin_lock, isr_flags);
        
        if (copy_to_user((struct dmx512_tx_status*) arg,
					&tx_status,
					sizeof(struct dmx512_tx_status)))
			return -EFAULT;
        break;
        
    case TIOCBIT9ADDR:
        //trans bit9 force set '1'
        serial_out(p, UART_REG_LCR, serial_in(p, UART_REG_LCR) &~ (EPE | NSB | BCB));
        serial_out(p, UART_REG_LCR, serial_in(p, UART_REG_LCR) | PBE | SPE);
        break;
        
    case TIOCBIT9DATA:
        //trans bit9 force set '0'
        serial_out(p, UART_REG_LCR, serial_in(p, UART_REG_LCR) &~ (NSB | BCB));
        serial_out(p, UART_REG_LCR, serial_in(p, UART_REG_LCR) | EPE | PBE | SPE);
        break;
        
	default:
		return -ENOIOCTLCMD;
	}
	return ret;
}

static struct uart_ops nuc970serial_ops = {
	.tx_empty	= nuc970serial_tx_empty,
	.set_mctrl	= nuc970serial_set_mctrl,
	.get_mctrl	= nuc970serial_get_mctrl,
	.stop_tx	= nuc970serial_stop_tx,
	.start_tx	= nuc970serial_start_tx,
	.stop_rx	= nuc970serial_stop_rx,
	.enable_ms	= nuc970serial_enable_ms,
	.break_ctl	= nuc970serial_break_ctl,
	.startup	= nuc970serial_startup,
	.shutdown	= nuc970serial_shutdown,
	.set_termios	= nuc970serial_set_termios,
	.set_ldisc	= nuc970serial_set_ldisc,
	.pm		= nuc970serial_pm,
	.type		= nuc970serial_type,
	.release_port	= nuc970serial_release_port,
	.request_port	= nuc970serial_request_port,
	.config_port	= nuc970serial_config_port,
	.verify_port	= nuc970serial_verify_port,
	.ioctl		= nuc970serial_rs485_ioctl,
    .mmap       = nuc970serial_rs485_mmap,
};

static void __init nuc970serial_init_ports(void)
{
	static int first = 1;
	int i;
	
	// enable clock
	clk = clk_get(NULL, "uart0");	
	clk_prepare(clk);
	clk_enable(clk);

	
	if (!first)
		return;
	first = 0;

	for (i = 0; i < UART_NR; i++) {
		struct uart_nuc970_port *up = &nuc970serial_ports[i];

		up->port.line = i;
		spin_lock_init(&up->port.lock);

		up->port.ops = &nuc970serial_ops;
		up->port.iobase = (long)(NUC970_VA_UART + (i*0x100));
		up->port.membase = NUC970_VA_UART + (i*0x100);
		up->port.uartclk = 12000000;
	
	}
}

#ifdef CONFIG_SERIAL_NUC970_CONSOLE
static void nuc970serial_console_putchar(struct uart_port *port, int ch)
{
	struct uart_nuc970_port *up = (struct uart_nuc970_port *)port;

	while(!(serial_in(up, UART_REG_FSR) & TX_EMPTY));
	serial_out(up, UART_REG_THR, ch);
}

/*
 *	Print a string to the serial port trying not to disturb
 *	any possible real use of the port...
 *
 *	The console_lock must be held when we get here.
 */
static void
nuc970serial_console_write(struct console *co, const char *s, unsigned int count)
{
	struct uart_nuc970_port *up = &nuc970serial_ports[co->index];
	unsigned long flags;
	unsigned int ier;

	local_irq_save(flags);

	/*
	 *	First save the IER then disable the interrupts
	 */
	ier = serial_in(up, UART_REG_IER);
	serial_out(up, UART_REG_IER, 0);

	uart_console_write(&up->port, s, count, nuc970serial_console_putchar);

	/*
	 *	Finally, wait for transmitter to become empty
	 *	and restore the IER
	 */
	while(!(serial_in(up, UART_REG_FSR) & TX_EMPTY));
	serial_out(up, UART_REG_IER, ier);


	local_irq_restore(flags);
}

static int __init nuc970serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index >= UART_NR)
		co->index = 0;
	port = &nuc970serial_ports[co->index].port;

	if (!port->iobase && !port->membase)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}


static struct console nuc970serial_console = {
	.name		= "ttyS",
	.write		= nuc970serial_console_write,
	.device		= uart_console_device,
	.setup		= nuc970serial_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &nuc970serial_reg,
};

static int __init nuc970serial_console_init(void)
{
	nuc970serial_init_ports();		
	register_console(&nuc970serial_console);
	
	return 0;
}
console_initcall(nuc970serial_console_init);

#define NUC970SERIAL_CONSOLE	&nuc970serial_console
#else
#define NUC970SERIAL_CONSOLE	NULL
#endif

static struct uart_driver nuc970serial_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "serial",
	.dev_name		= "ttyS",
	.major			= TTY_MAJOR,
	.minor			= 64,
	.cons			= NUC970SERIAL_CONSOLE,
	.nr				= UART_NR,
};


/**
 *
 *	Suspend one serial port.
 */
void nuc970serial_suspend_port(int line)
{
	uart_suspend_port(&nuc970serial_reg, &nuc970serial_ports[line].port);
}

/**
 *
 *	Resume one serial port.
 */
void nuc970serial_resume_port(int line)
{
	struct uart_nuc970_port *up = &nuc970serial_ports[line];

	uart_resume_port(&nuc970serial_reg, &up->port);
}

static int nuc970serial_pinctrl(struct platform_device *pdev)
{
	struct pinctrl *p = NULL;
	int retval = 0;  

if(pdev->id == 1)
{
#if defined (CONFIG_NUC970_UART1_PE)
    p = devm_pinctrl_get_select(&pdev->dev, "uart1-PE");
#elif defined (CONFIG_NUC970_UART1_FC_PE)
    p = devm_pinctrl_get_select(&pdev->dev, "uart1-fc-PE");
#elif defined (CONFIG_NUC970_UART1_FF_PE)
	p = devm_pinctrl_get_select(&pdev->dev, "uart1-ff-PE");
#elif defined (CONFIG_NUC970_UART1_PH)
    p = devm_pinctrl_get_select(&pdev->dev, "uart1-PH");
#elif defined (CONFIG_NUC970_UART1_FC_PH)
    p = devm_pinctrl_get_select(&pdev->dev, "uart1-fc-PH");
#elif defined (CONFIG_NUC970_UART1_PI)
    p = devm_pinctrl_get_select(&pdev->dev, "uart1-PI");
#elif defined (CONFIG_NUC970_UART1_FC_PI)
    p = devm_pinctrl_get_select(&pdev->dev, "uart1-fc-PI");
#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
}
else if(pdev->id == 2)
{
#if defined (CONFIG_NUC970_UART2)
	p = devm_pinctrl_get_select(&pdev->dev, "uart2");
#elif defined (CONFIG_NUC970_UART2_FC)
	p = devm_pinctrl_get_select(&pdev->dev, "uart2_fc");
#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
}
else if(pdev->id == 4)
{
#if defined (CONFIG_NUC970_UART4_PC)
	p = devm_pinctrl_get_select(&pdev->dev, "uart4-PC");
#elif defined (CONFIG_NUC970_UART4_FC_PC)
	p = devm_pinctrl_get_select(&pdev->dev, "uart4-fc-PC");
#elif defined (CONFIG_NUC970_UART4_PH)
	p = devm_pinctrl_get_select(&pdev->dev, "uart4-PH");
#elif defined (CONFIG_NUC970_UART4_FC_PH)
	p = devm_pinctrl_get_select(&pdev->dev, "uart4-fc-PH");
#elif defined (CONFIG_NUC970_UART4_PI)
	p = devm_pinctrl_get_select(&pdev->dev, "uart4-PI");
#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
}
else if(pdev->id == 6)
{
#if defined (CONFIG_NUC970_UART6_PB)
	p = devm_pinctrl_get_select(&pdev->dev, "uart6-PB");
#elif defined (CONFIG_NUC970_UART6_FC_PB)
	p = devm_pinctrl_get_select(&pdev->dev, "uart6-fc-PB");
#elif defined (CONFIG_NUC970_UART6_PG)
	p = devm_pinctrl_get_select(&pdev->dev, "uart6-PG");
#elif defined (CONFIG_NUC970_UART6_FC_PG)
	p = devm_pinctrl_get_select(&pdev->dev, "uart6-fc-PG");
#elif defined (CONFIG_NUC970_UART6_FC_PB)
	p = devm_pinctrl_get_select(&pdev->dev, "uart6-fc-PB");
#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
}
else if(pdev->id == 7)
{
#if defined (CONFIG_NUC970_UART7_PG)
	p = devm_pinctrl_get_select(&pdev->dev, "uart7-PG");
#elif defined (CONFIG_NUC970_UART7_PI)
	p = devm_pinctrl_get_select(&pdev->dev, "uart7-PI");
#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
}
else if(pdev->id == 8)
{
#if defined (CONFIG_NUC970_UART8_PE)
	p = devm_pinctrl_get_select(&pdev->dev, "uart8-PE");
#elif defined (CONFIG_NUC970_UART8_FC_PE)
	p = devm_pinctrl_get_select(&pdev->dev, "uart8-fc-PE");
#elif defined (CONFIG_NUC970_UART8_PH)
	p = devm_pinctrl_get_select(&pdev->dev, "uart8-PH");
#elif defined (CONFIG_NUC970_UART8_FC_PH)
	p = devm_pinctrl_get_select(&pdev->dev, "uart8-fc-PH");
#elif defined (CONFIG_NUC970_UART8_PI)
	p = devm_pinctrl_get_select(&pdev->dev, "uart8-PI");
#elif defined (CONFIG_NUC970_UART8_FC_PI)
	p = devm_pinctrl_get_select(&pdev->dev, "uart8-fc-PI");
#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
}
else if(pdev->id == 9)
{
#if defined (CONFIG_NUC970_UART9_PD0)
	p = devm_pinctrl_get_select(&pdev->dev, "uart9-PD0");
#elif defined (CONFIG_NUC970_UART9_PD1)
	p = devm_pinctrl_get_select(&pdev->dev, "uart9-PD1");
#elif defined (CONFIG_NUC970_UART9_PH)
	p = devm_pinctrl_get_select(&pdev->dev, "uart9-PH");
#endif

	if (IS_ERR(p))
	{
		dev_err(&pdev->dev, "unable to reserve pin\n");
		retval = PTR_ERR(p);
	}
}
else if(pdev->id == 10)
{
#if defined (CONFIG_NUC970_UART10_PB0)
	p = devm_pinctrl_get_select(&pdev->dev, "uart10-PB0");
#elif defined (CONFIG_NUC970_UART10_PB1)
	p = devm_pinctrl_get_select(&pdev->dev, "uart10-PB1");
#elif defined (CONFIG_NUC970_UART10_FC_PB1)
	p = devm_pinctrl_get_select(&pdev->dev, "uart10-fc-PB1");
#elif defined (CONFIG_NUC970_UART10_PC)
	p = devm_pinctrl_get_select(&pdev->dev, "uart10-PC");
#elif defined (CONFIG_NUC970_UART10_FC_PC)
	p = devm_pinctrl_get_select(&pdev->dev, "uart10-fc-PC");
#endif

    if (IS_ERR(p))
    {
        dev_err(&pdev->dev, "unable to reserve pin\n");
        retval = PTR_ERR(p);
    }
}
	return retval;
}

void nuc970serial_set_clock(void)
{
	clk = clk_get(NULL, "uart0");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart0_eclk");	
	clk_prepare(clk);
	clk_enable(clk);


	#ifdef CONFIG_NUC970_UART1
	clk = clk_get(NULL, "uart1");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart1_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART2
	clk = clk_get(NULL, "uart2");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart2_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART3
	clk = clk_get(NULL, "uart3");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart3_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART4
	clk = clk_get(NULL, "uart4");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart4_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART5
	clk = clk_get(NULL, "uart5");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart5_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART6
	clk = clk_get(NULL, "uart6");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart6_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART7
	clk = clk_get(NULL, "uart7");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart7_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART8
	clk = clk_get(NULL, "uart8");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart8_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART9
	clk = clk_get(NULL, "uart9");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart9_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif

	#ifdef CONFIG_NUC970_UART10
	clk = clk_get(NULL, "uart10");	
	clk_prepare(clk);
	clk_enable(clk);

	clk = clk_get(NULL, "uart10_eclk");	
	clk_prepare(clk);
	clk_enable(clk);
	#endif
	
}

/*
 * Register a set of serial devices attached to a platform device.  The
 * list is terminated with a zero flags entry, which means we expect
 * all entries to have at least UPF_BOOT_AUTOCONF set.
 */
static int nuc970serial_probe(struct platform_device *pdev)
{
	struct plat_nuc970serial_port *p = pdev->dev.platform_data;
	struct uart_nuc970_port *up;
	int ret, i, retval;

//	memset(&port, 0, sizeof(struct uart_port));

	retval = nuc970serial_pinctrl(pdev);
	if(retval != 0)
		return retval;

	nuc970serial_set_clock();

	i = pdev->id;

	//for (i = 0; i < UART_NR && p; p++, i++) {
		up = &nuc970serial_ports[i];
		
		up->port.line 			= i;
		up->port.iobase       	= p->iobase;
		up->port.membase      	= p->membase;
		up->port.irq          	= p->irq;
		up->port.uartclk      	= p->uartclk;
		up->port.mapbase     	= p->mapbase;
		up->port.private_data 	= p->private_data;
		up->port.dev 			= &pdev->dev;
		up->port.flags 			= ASYNC_BOOT_AUTOCONF;
		
		/* Possibly override default I/O functions.  */
		if (p->serial_in)
			up->port.serial_in = p->serial_in;
		if (p->serial_out)
			up->port.serial_out = p->serial_out;

        //lzy add 2016.11.1
        up->malloc_size = 0;
        up->pmmapbuf = 0;
        up->palloctxbuf = 0;
        up->pallocrxbuf1 = 0;
        up->pallocrxbuf2 = 0;
        memset((char *)&up->dmx512tx, 0, sizeof(struct dmx512_tx));
        memset((char *)&up->dmx512rx, 0, sizeof(struct dmx512_rx));
        memset((char *)&up->dmx512tx_status, 0, sizeof(struct dmx512_tx_status));
        up->dmx512tx_status.tx_timer.min_diff = 0xFFFFFFFF;
        memset((char *)&up->dmx512rx_status, 0, sizeof(struct dmx512_rx_status));
        up->dmx512rx_status.rx_timer.min_diff = 0xFFFFFFFF;

        //init tx wait q
        init_waitqueue_head(&up->tx_queue);
        atomic_set(&up->tx_condition, 0);
                
        //init rx wait q
        init_waitqueue_head(&up->rx_queue);
        atomic_set(&up->rx_condition, 0);

		ret = uart_add_one_port(&nuc970serial_reg, &up->port);


	//}

	return 0;
}

/*
 * Remove serial ports registered against a platform device.
 */
static int nuc970serial_remove(struct platform_device *dev)
{
	int i;
	struct uart_port *port = platform_get_drvdata(dev);

	free_irq(port->irq, port);

	for (i = 0; i < UART_NR; i++) {
		struct uart_nuc970_port *up = &nuc970serial_ports[i];

		if (up->port.dev == &dev->dev)
			uart_remove_one_port(&nuc970serial_reg, &up->port);
	}
	return 0;
}

static int nuc970serial_suspend(struct platform_device *dev, pm_message_t state)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_nuc970_port *up = &nuc970serial_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			uart_suspend_port(&nuc970serial_reg, &up->port);
	}

	return 0;
}

static int nuc970serial_resume(struct platform_device *dev)
{
	int i;

	for (i = 0; i < UART_NR; i++) {
		struct uart_nuc970_port *up = &nuc970serial_ports[i];

		if (up->port.type != PORT_UNKNOWN && up->port.dev == &dev->dev)
			nuc970serial_resume_port(i);
	}

	return 0;
}

static struct platform_driver nuc970serial_driver = {
	.probe		= nuc970serial_probe,
	.remove		= nuc970serial_remove,
	.suspend	= nuc970serial_suspend,
	.resume		= nuc970serial_resume,
	.driver		= {
		.name	= "nuc970-uart",
		.owner	= THIS_MODULE,
	},
};

static int __init nuc970serial_init(void)
{
	int ret;

	ret = uart_register_driver(&nuc970serial_reg);
	if (ret)
		return ret;

	ret = platform_driver_register(&nuc970serial_driver);
	if (ret)
		uart_unregister_driver(&nuc970serial_reg);

	return ret;
}

static void __exit nuc970serial_exit(void)
{
	platform_driver_unregister(&nuc970serial_driver);
	uart_unregister_driver(&nuc970serial_reg);
}

module_init(nuc970serial_init);
module_exit(nuc970serial_exit);

EXPORT_SYMBOL(nuc970serial_suspend_port);
EXPORT_SYMBOL(nuc970serial_resume_port);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("NUC970 serial driver");

MODULE_ALIAS_CHARDEV_MAJOR(TTY_MAJOR);
