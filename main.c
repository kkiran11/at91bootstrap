/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2006, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "common.h"
#include "board.h"
#include "usart.h"
#include "slowclk.h"
#include "board_hw_info.h"
#include "tz_utils.h"
#include "pm.h"
#include "act8865.h"
#include "secure.h"
#include "sfr_aicredir.h"

#include "timer.h"
#include "hardware.h"
#include "debug.h"

#include "sama5d3_xplained.h"
#include "watchdog.h"

#ifdef CONFIG_HW_DISPLAY_BANNER
static void display_banner (void)
{
	usart_puts(BANNER);
}
#endif

static void dram_test(void)
{
	unsigned int read_data = 0;
	volatile unsigned int *p = (volatile unsigned int *)0x20000000;

	dbg_info("Start DRAM test\n");

	while(p < (volatile unsigned int *)0x40000000)
	{
		*p = p;/*test_pat;*/
		p++;
	}

	/*wait for  seconds*/
	mdelay(200);

	p = (volatile unsigned int *)0x20000000;
	while (p < (volatile unsigned int *)0x40000000)
	{
		read_data = *p;
		if (read_data != (unsigned int)p)
		{
			dbg_info("Failed: read %d from addr %d\n", read_data, p);
		}

		if (p == (volatile unsigned int *)0x26f00000)
		{
			dbg_info("read %d from addr %d\n", read_data, p);
		}
		p++;
	}

	dbg_info("End of DRAM test\n");
}

int main(void)
{
	struct image_info image;
	int ret;

	at91_wdt_reload_counter();
		
#ifdef CONFIG_HW_INIT
	hw_init();
#endif

#if defined(CONFIG_SCLK)
#if !defined(CONFIG_SAMA5D4)
	slowclk_enable_osc32();
#endif
#endif

#ifdef CONFIG_HW_DISPLAY_BANNER
	display_banner();
#endif

	at91_wdt_reload_counter();
	
#ifdef CONFIG_REDIRECT_ALL_INTS_AIC
	redirect_interrupts_to_nsaic();
#endif

#ifdef CONFIG_LOAD_HW_INFO
	load_board_hw_info();
#endif

	at91_wdt_reload_counter();
	
#ifdef CONFIG_PM
	at91_board_pm();
#endif

	at91_wdt_reload_counter();

#ifdef CONFIG_ACT8865
	act8865_workaround();
#endif

	at91_wdt_reload_counter();
	
	setLEDColor();

/*	dram_test();*/

	at91_wdt_reload_counter();
	
	init_load_image(&image);

	at91_wdt_reload_counter();
		
#if defined(CONFIG_SECURE)
	image.dest -= sizeof(at91_secure_header_t);
#endif

	ret = (*load_image)(&image);

#if defined(CONFIG_SECURE)
	if (!ret)
		ret = secure_check(image.dest);
	image.dest += sizeof(at91_secure_header_t);
#endif

	load_image_done(ret);

#ifdef CONFIG_SCLK
//	slowclk_switch_osc32();
#endif

#if defined(CONFIG_ENTER_NWD)
	switch_normal_world();

	/* point never reached with TZ support */
#endif

	at91_wdt_reload_counter();
	
	return JUMP_ADDR;
}
