/*
 * include/linux/serial.h
 *
 * Copyright (C) 1992 by Theodore Ts'o.
 * 
 * Redistribution of this file is permitted under the terms of the GNU 
 * Public License (GPL)
 */

#ifndef _UAPI_LINUX_SERIAL_H
#define _UAPI_LINUX_SERIAL_H

#include <linux/types.h>

#include <linux/tty_flags.h>


struct serial_struct {
	int	type;
	int	line;
	unsigned int	port;
	int	irq;
	int	flags;
	int	xmit_fifo_size;
	int	custom_divisor;
	int	baud_base;
	unsigned short	close_delay;
	char	io_type;
	char	reserved_char[1];
	int	hub6;
	unsigned short	closing_wait; /* time to wait before closing */
	unsigned short	closing_wait2; /* no longer used... */
	unsigned char	*iomem_base;
	unsigned short	iomem_reg_shift;
	unsigned int	port_high;
	unsigned long	iomap_base;	/* cookie passed into ioremap */
};

/*
 * For the close wait times, 0 means wait forever for serial port to
 * flush its output.  65535 means don't wait at all.
 */
#define ASYNC_CLOSING_WAIT_INF	0
#define ASYNC_CLOSING_WAIT_NONE	65535

/*
 * These are the supported serial types.
 */
#define PORT_UNKNOWN	0
#define PORT_8250	1
#define PORT_16450	2
#define PORT_16550	3
#define PORT_16550A	4
#define PORT_CIRRUS     5	/* usurped by cyclades.c */
#define PORT_16650	6
#define PORT_16650V2	7
#define PORT_16750	8
#define PORT_STARTECH	9	/* usurped by cyclades.c */
#define PORT_16C950	10	/* Oxford Semiconductor */
#define PORT_16654	11
#define PORT_16850	12
#define PORT_RSA	13	/* RSA-DV II/S card */
#define PORT_MAX	13

#define SERIAL_IO_PORT	0
#define SERIAL_IO_HUB6	1
#define SERIAL_IO_MEM	2

#define UART_CLEAR_FIFO		0x01
#define UART_USE_FIFO		0x02
#define UART_STARTECH		0x04
#define UART_NATSEMI		0x08


/*
 * Multiport serial configuration structure --- external structure
 */
struct serial_multiport_struct {
	int		irq;
	int		port1;
	unsigned char	mask1, match1;
	int		port2;
	unsigned char	mask2, match2;
	int		port3;
	unsigned char	mask3, match3;
	int		port4;
	unsigned char	mask4, match4;
	int		port_monitor;
	int	reserved[32];
};

/*
 * Serial input interrupt line counters -- external structure
 * Four lines can interrupt: CTS, DSR, RI, DCD
 */
struct serial_icounter_struct {
	int cts, dsr, rng, dcd;
	int rx, tx;
	int frame, overrun, parity, brk;
	int buf_overrun;
	int reserved[9];
};

/*
 * use method:
 * set frame freq, auto fix it
 * set seq(mtbp_time, mab_time, break_time, between_time), manual fix it
 * set seq and period, auto fix mtbp_time, this mode use to fix dmx512 device which is not support too short seq
 */
typedef enum _dmx512_time_seq_mode
{
    SET_WITH_FREQ = 0,
    SET_WITH_SEQ,
    SET_WITH_SEQ_AND_PERIOD
}dmx512_time_seq_mode;

/*
 * mtbp_time is frame gap, mab_time is first low V width, break_time is first High V to num. 0 data,
 * between_time is data gap, period = mtbp_time + break_time + mab_time + (1 / data_rate * 11 + between_time) * 513
 * freq is 1/period, freq is multiply by 1000
 */
struct dmx512_time_seq {
#define DEFAULT_MTBP_TIME       0
#define DEFAULT_BREAK_TIME      180
#define DEFAULT_MAB_TIME        30
#define DEFAULT_BETWEEN_TIME    0
    unsigned int mtbp_time;
    unsigned int break_time;
    unsigned int mab_time;
    unsigned int between_time;
    unsigned long period;
    unsigned long freq;
};

struct dmx512_config_tx {
    struct dmx512_time_seq time_seq;
    dmx512_time_seq_mode mode;
};

struct dmx512_config_rx {
    unsigned int record_frames;
    unsigned int skip_frames;
    unsigned int frame_timeout;
};

/*
 * Serial interface for controlling RS485 settings on chips with suitable
 * support. Set with TIOCSRS485 and get with TIOCGRS485 if supported by your
 * platform. The set function returns the new state, with any unsupported bits
 * reverted appropriately.
 */

struct serial_rs485 {
#define SER_RS485_ENABLED		    (1 << 0)	/* If enabled */
#define SER_RS485_RTS_ON_SEND		(1 << 1)	/* Logical level for RTS pin when sending */
#define SER_RS485_RTS_AFTER_SEND	(1 << 2)	/* Logical level for RTS pin after sent*/
#define SER_RS485_RX_DURING_TX		(1 << 3)
#define SER_RS485_DMX512_TX		    (1 << 4)
#define SER_RS485_DMX512_RX		    (1 << 5)
#define SER_RS485_DMX512_PAUSE      (1 << 6)

	__u32	                flags;			    /* RS485 feature flags */
	__u32	                delay_rts_before_send;	/* Delay before send (milliseconds) */
	__u32	                delay_rts_after_send;	/* Delay after send (milliseconds) */
	__u32	                padding[5];		    /* Memory is cheap, new structs are a royal PITA .. */
    struct dmx512_config_tx tx_config;
    struct dmx512_config_rx rx_config;
};

#endif /* _UAPI_LINUX_SERIAL_H */
