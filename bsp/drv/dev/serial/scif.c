/*-
 * Copyright (c) 2008-2009, Kohsuke Ohtani
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * pl011.c - ARM PrimeCell PL011 UART
 */

#include <driver.h>
#include <tty.h>
#include <serial.h>

/* #define DEBUG_PL011 1 */

#ifdef DEBUG_SCIF
#define DPRINTF(a) printf a
#else
#define DPRINTF(a)
#endif

#define SCIF_BASE	CONFIG_SCIF_BASE
#define SCIF_IRQ	CONFIG_SCIF_IRQ
#define UART_CLK	64000000
#define BAUD_RATE	38400

/* SCIF Register Configuration
 *    (R01UH0543EJ0100 40. Serial Communication Interface with FIFO (SCIF))
 *     Table 40.2
 */

#define SCIF_SCSMR	(SCIF_BASE + 0x00)
#define SCIF_SCBRR	(SCIF_BASE + 0x04)
#define SCIF_SCSCR	(SCIF_BASE + 0x08)
#define SCIF_SCFTDR	(SCIF_BASE + 0x0C)
#define SCIF_SCFSR  (SCIF_BASE + 0x10)
#define SCIF_SCFRDR (SCIF_BASE + 0x14)
#define SCIF_SCFCR  (SCIF_BASE + 0x18)
#define SCIF_SCFDR  (SCIF_BASE + 0x1C)
#define SCIF_SCSPTR (SCIF_BASE + 0x20)
#define SCIF_SCLSR  (SCIF_BASE + 0x24)
#define SCIF_DL     (SCIF_BASE + 0x30)
#define SCIF_CKS    (SCIF_BASE + 0x34)

/* SCIF_SCSMR FLAGS */
#define SCSMR_CM   0x80
#define SCSMR_CHR  0x40
#define SCSMR_PE   0x20
#define SCSMR_PM   0x10
#define SCSMR_STOP 0x08
#define SCSMR_CKS  0x03

/* SCIF_SCSCR FLAGS */
#define SCSCR_TEIE 0x800
#define SCSCR_TIE  0x080
#define SCSCR_RIE  0x040
#define SCSCR_TE   0x020
#define SCSCR_RE   0x010
#define SCSCR_REIE 0x008
#define SCSCR_TOIE 0x004
#define SCSCR_CKE  0x003

/* SCIF_SCFSR FLAGS */
#define SCFSR_PER	   0xF000
#define SCFSR_FER      0x0F00
#define SCFSR_ER	   0x0080
#define SCFSR_TEND	   0x0040
#define SCFSR_TDFE	   0x0020
#define SCFSR_BRK      0x0010
#define SCFSR_FER      0x0008
#define SCFSR_PER      0x0004
#define SCFSR_RDF      0x0002
#define SCFSR_DR       0x0001

/* SCIF_SCFCR FLAGS */
#define SCFCR_RSTRG	   0x0700
#define SCFCR_RTRG     0x00C0
#define SCFCR_TTRG	   0x0030
#define SCFCR_MCE	   0x0008
#define SCFCR_TFRST	   0x0004
#define SCFCR_RFRST    0x0002
#define SCFCR_LOOP     0x0001

/* SCIF_SCFDR FLAGS */
#define SCFDR_T		   0x1F00
#define SCFDR_T_FULL   0x1000
#define SCFDR_R		   0x001F
#define SCFDR_R_FULL   0x10

/* SCIF_SCSPTR FLAGS */
#define SCSPTR_RTSIO	0x0080
#define SCSPTR_RTSDT	0x0040
#define SCSPTR_CTSIO	0x0020
#define SCSPTR_CTSDT	0x0010
#define SCSPTR_SCKIO	0x0008
#define SCSPTR_SCKDT    0x0004
#define SCSPTR_SPB2IO	0x0002
#define SCSPTR_SPB2DT	0x0001


/* SCIF_SCLSR FLAGS */
#define SCLSR_TO	0x0004
#define SCLSR_ORER	0x0001

/* Forward functions */
static int	scif_init(struct driver *);
static void	scif_xmt_char(struct serial_port *, char);
static char	scif_rcv_char(struct serial_port *);
static void	scif_set_poll(struct serial_port *, int);
static void	scif_start(struct serial_port *);
static void	scif_stop(struct serial_port *);


struct driver scif_driver = {
	/* name */	"scif",
	/* devops */	NULL,
	/* devsz */	0,
	/* flags */	0,
	/* probe */	NULL,
	/* init */	scif_init,
	/* detach */	NULL,
};

static struct serial_ops scif_ops = {
	/* xmt_char */	scif_xmt_char,
	/* rcv_char */	scif_rcv_char,
	/* set_poll */	scif_set_poll,
	/* start */	scif_start,
	/* stop */	scif_stop,
};

static struct serial_port scif_port;


static void
scif_xmt_char(struct serial_port *sp, char c)
{

	while ( (bus_read_16(SCIF_SCFDR) & SCFDR_T) == SCFDR_T_FULL)
		;
	bus_write_8(SCIF_SCFTDR, (uint32_t)c);
}

static char
scif_rcv_char(struct serial_port *sp)
{
	char c;

	while ((bus_read_16(SCIF_SCFDR) & SCFDR_R) == 0)
		;
	c = bus_read_8(SCIF_SCFRDR) & 0xff;
	return c;
}

static void
scif_set_poll(struct serial_port *sp, int on)
{
	register unsigned int reg;
	reg = bus_read_32(SCIF_SCSCR);

	if (on) {
		/*
		 * Disable interrupt for polling mode.
		 */

		reg &=~(SCSCR_TIE|SCSCR_RIE);
		bus_write_32(SCIF_SCSCR, 0);
	} else {
		reg |= SCSCR_TIE|SCSCR_RIE;
		bus_write_32(SCIF_SCSCR, reg);
	}
}

static int
scif_isr(void *arg)
{
	struct serial_port *sp = arg;
	char c;
	uint16_t scfsr;

	scfsr = bus_read_16(SCIF_SCFSR);

	if (scfsr & SCFSR_RDF) {
		/*
		 * Receive interrupt
		 */
		while ( (bus_read_16(SCIF_SCFDR) & SCFDR_R) == 0);
		do {
			c = bus_read_8(SCIF_SCFRDR);
			serial_rcv_char(sp, c);
		} while ((bus_read_16(SCIF_SCFDR) & SCFDR_R) > 0);

		/* Clear interrupt status */
		scfsr &=~(SCFSR_RDF);
		bus_write_16(SCIF_SCFSR, scfsr);
	}
	if (scfsr & SCFSR_TEND) {
		/*
		 * Transmit interrupt
		 */
		serial_xmt_done(sp);

		/* Clear interrupt status */
		scfsr &=~(SCFSR_TEND);
		bus_write_16(SCIF_SCFSR, scfsr);
	}
	return 0;
}

static void
scif_start(struct serial_port *sp)
{
	uint32_t shift,scbrr,delay;
	uint16_t temp;

	bus_write_16(SCIF_SCSCR, 0);	/* Disable all interrupts */
	bus_write_16(SCIF_SCFCR, SCFCR_TFRST | SCFCR_RFRST);	/* Disable all interrupts */

	temp = bus_read_16(SCIF_SCFSR);
	temp &=~(SCFSR_ER|SCFSR_DR|SCFSR_BRK|SCFSR_RDF);
	bus_write_16(SCIF_SCFSR, temp);

	temp = bus_read_16(SCIF_SCLSR);
	temp &=~(SCLSR_TO|SCLSR_ORER);
	bus_write_16(SCIF_SCLSR, temp);

	bus_write_16(SCIF_SCSCR, 0); /* Set only CKE bits, 0 = internal clock*/

	bus_write_16(SCIF_SCSMR, 0); /* 8N1 CLK WITH NO DIVIDER */

	/*
	 * Set baud rate:
	 *
	 * SCIF_SCBRR = (CLK / (64 * BAUD_RATE * ( 2^( 2 * SCSMR_CKS - 1) ) ) ) - 1
	 * SCIF_SCBRR = (CLK / (BAUD_RATE<<(2*SCSMR_CKS+5)))-1
	 */
	shift = 2*(bus_read_16(SCIF_SCSMR) & SCSMR_CKS)+5;
	scbrr = (UART_CLK/BAUD_RATE) >> (shift);
	bus_write_8(SCIF_SCBRR, scbrr);

	/* Delay for 1 bit interval */
		delay=32768;
		while(delay--);

	//SCIF_SCFCR = SCFCR_TTRG = 3 | SCFCR_RTRG = 0;
	bus_write_16(SCIF_SCFCR, SCFCR_TTRG);

	/* Install interrupt handler */
	sp->irq = irq_attach(SCIF_IRQ, IPL_COMM, 0, scif_isr, IST_NONE, sp);

	/* Enable TX/RX interrupt */
	temp = bus_read_16(SCIF_SCSCR);
	bus_write_16(SCIF_SCSCR, temp|SCSCR_TE|SCSCR_RE);//|SCSCR_TIE|SCSCR_RIE|SCSCR_REIE|SCSCR_TOIE);
}

static void
scif_stop(struct serial_port *sp)
{
	uint16_t temp = bus_read_16(SCIF_SCSCR);
	temp &= ~(SCSCR_TIE|SCSCR_RIE|SCSCR_REIE|SCSCR_TOIE);
	bus_write_16(SCIF_SCSCR, temp);	/* Disable all interrupts */
}

static int
scif_init(struct driver *self)
{

	serial_attach(&scif_ops, &scif_port);
	return 0;
}
