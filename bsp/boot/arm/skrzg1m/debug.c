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

#include <sys/param.h>
#include <boot.h>

#define SCIF_BASE	0xE6E60000
#define UART_CLK	64000000
#define BAUD_RATE	38400

/* SCIF Register Configuration
 *    (R01UH0543EJ0100 40. Serial Communication Interface with FIFO (SCIF))
 *     Table 40.2
 */

#define SCIF_SCSMR	(*(volatile uint16_t *)(SCIF_BASE + 0x00))
#define SCIF_SCBRR	(*(volatile uint8_t *)(SCIF_BASE + 0x04))
#define SCIF_SCSCR	(*(volatile uint16_t *)(SCIF_BASE + 0x08))
#define SCIF_SCFTDR	(*(volatile uint8_t *)(SCIF_BASE + 0x0C))
#define SCIF_SCFSR  (*(volatile uint16_t *)(SCIF_BASE + 0x10))
#define SCIF_SCFRDR (*(volatile uint8_t *)(SCIF_BASE + 0x14))
#define SCIF_SCFCR  (*(volatile uint16_t *)(SCIF_BASE + 0x18))
#define SCIF_SCFDR  (*(volatile uint16_t *)(SCIF_BASE + 0x1C))
#define SCIF_SCSPTR (*(volatile uint16_t *)(SCIF_BASE + 0x20))
#define SCIF_SCLSR  (*(volatile uint16_t *)(SCIF_BASE + 0x24))
#define SCIF_DL     (*(volatile uint16_t *)(SCIF_BASE + 0x30))
#define SCIF_CKS    (*(volatile uint16_t *)(SCIF_BASE + 0x34))

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
#define SCFSR_PERC	   0xF000
#define SCFSR_FERC     0x0F00
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


/* Flag register */
#define FR_RXFE		0x10	/* Receive FIFO empty */
#define FR_TXFF		0x20	/* Transmit FIFO full */

/* Masked interrupt status register */
#define MIS_RX		0x10	/* Receive interrupt */
#define MIS_TX		0x20	/* Transmit interrupt */

/* Interrupt clear register */
#define ICR_RX		0x10	/* Clear receive interrupt */
#define ICR_TX		0x20	/* Clear transmit interrupt */

/* Line control register (High) */
#define LCRH_WLEN8	0x60	/* 8 bits */
#define LCRH_FEN	0x10	/* Enable FIFO */

/* Control register */
#define CR_UARTEN	0x0001	/* UART enable */
#define CR_TXE		0x0100	/* Transmit enable */
#define CR_RXE		0x0200	/* Receive enable */

/* Interrupt mask set/clear register */
#define IMSC_RX		0x10	/* Receive interrupt mask */
#define IMSC_TX		0x20	/* Transmit interrupt mask */

/*
 * Print one chracter
 */
void
debug_putc(int c)
{

#if defined(DEBUG) && defined(CONFIG_DIAG_SERIAL)
	while ((SCIF_SCFDR & SCFDR_T) == SCFDR_T_FULL)
		;
	SCIF_SCFTDR = c;
#endif
}

/*
 * Initialize debug port.
 */
void
debug_init(void)
{

#if defined(DEBUG) && defined(CONFIG_DIAG_SERIAL)
	unsigned int shift;
	unsigned int delay;


	SCIF_SCSCR = 0;	/* Disable all interrupts */
	SCIF_SCFCR = SCFCR_TFRST | SCFCR_RFRST;

	SCIF_SCFSR &= ~(SCFSR_ER|SCFSR_DR|SCFSR_BRK|SCFSR_RDF);
	SCIF_SCLSR &= ~(SCLSR_TO|SCLSR_ORER);
	SCIF_SCSCR = 0; /* Set only CKE bits, 0 = internal clock*/

	SCIF_SCSMR = 0;

	/*
	 * Set baud rate:
	 *
	 * SCIF_SCBRR = (CLK / (64 * BAUD_RATE * ( 2^( 2 * SCSMR_CKS - 1) ) ) ) - 1
	 * SCIF_SCBRR = (CLK / (BAUD_RATE<<(2*SCSMR_CKS+5)))-1
	 */
	shift = 2*(SCIF_SCSMR & SCSMR_CKS)+5;
	SCIF_SCBRR = (UART_CLK/BAUD_RATE) >> (shift);

	/* Delay for 1 bit interval */
	delay=32768;
	while(delay--);

	/*SCIF_SCFCR = SCFCR_TTRG = 3 | SCFCR_RTRG = 0;*/
	SCIF_SCFCR = SCFCR_TTRG;

	SCIF_SCSCR |= SCSCR_TE|SCSCR_RE;
#endif
}
