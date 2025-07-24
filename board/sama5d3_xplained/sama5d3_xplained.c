/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2014, Atmel Corporation
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
#include "hardware.h"
#include "pmc.h"
#include "usart.h"
#include "debug.h"
#include "ddramc.h"
#include "spi.h"
#include "gpio.h"
#include "timer.h"
#include "watchdog.h"
#include "string.h"

#include "arch/at91_pmc.h"
#include "arch/at91_rstc.h"
#include "arch/sama5_smc.h"
#include "arch/at91_pio.h"
#include "arch/at91_ddrsdrc.h"
#include "sama5d3_xplained.h"
#include "act8865.h"
#include "twi.h"

#ifdef CONFIG_QNX_USE_BOOT_DATA
#include "startup.h"
#include "boot_data_api.h"
#include "crc32.h"

#define NUM_BOOT_DATA           2
#define PREBOOT_SIZE            8
#define SW_UPDATE_RESTART_MAX   3
#define BOOT_DATA_MAGIC         (((uint32_t)'Q' << 24) | ((uint32_t)'B' << 16) | ((uint32_t)'D' << 8) | (uint32_t)'T')


#ifdef CONFIG_NANDFLASH

#ifdef CONFIG_USE_PMECC
#include "pmecc.h"
#endif

#include "nand.h"
extern int nandflash_get_type(struct nand_info *nand);
extern int nand_loadimage(struct nand_info *nand,
                unsigned int offset,
                unsigned int length,
                unsigned char *dest);
extern int nand_saveimage(struct nand_info *nand,
                unsigned int offset,
                unsigned int length,
                unsigned char *src);

#define BOOT_DATA0_OFFSET       0x0100000
#define BOOT_DATA1_OFFSET       0x0180000
#define IFS0_OFFSET             0x0200000
#define IFS1_OFFSET             0x4200000
#define PAGE_SIZE               2048

/* Simple static assert to make sure boot data will fit in a page */
char const check_boot_data_size[sizeof(boot_data_t) <= PAGE_SIZE ? 1 : -1];

//TODO Needed to make this larger since nand_loadimage doesn't seem to read just a page
static unsigned char boot_data_nand_buffer[PAGE_SIZE * 3];

#endif /* #ifdef CONFIG_NANDFLASH */
#endif /* #ifdef CONFIG_QNX_USE_BOOT_DATA */
static void at91_dbgu_hw_init(void)
{
    /* Configure DBGU pin */
    const struct pio_desc dbgu_pins[] = {
        {"RXD", AT91C_PIN_PB(30), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {"TXD", AT91C_PIN_PB(31), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
    };

    /*  Configure the dbgu pins */
    pmc_enable_periph_clock(AT91C_ID_PIOB);
    pio_configure(dbgu_pins);

    /* Enable clock */
    pmc_enable_periph_clock(AT91C_ID_DBGU);
}

static void initialize_dbgu(void)
{
    at91_dbgu_hw_init();
    usart_init(BAUDRATE(MASTER_CLOCK, 115200));
}


#if defined(CONFIG_LPDDR2)

static void lpddr2_init(void)
{
    unsigned int reg;

    /* enable ddr clock */
    pmc_enable_periph_clock(AT91C_ID_MPDDRC);
    pmc_enable_system_clock(AT91C_PMC_DDR);

    /* Init the special register for sama5d3x */
    /* MPDDRC DLL Slave Offset Register: LPDDR2 configuration */
    reg = AT91C_MPDDRC_S0OFF(0x04)
        | AT91C_MPDDRC_S1OFF(0x03)
        | AT91C_MPDDRC_S2OFF(0x04)
        | AT91C_MPDDRC_S3OFF(0x04);
    writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_DLL_SOR));

    /* MPDDRC DLL Master Offset Register */
    /* write master + clk90 offset */
    reg = AT91C_MPDDRC_MOFF(7)
        | AT91C_MPDDRC_CLK90OFF(0x1F)
        | AT91C_MPDDRC_SELOFF_ENABLED;
    writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_DLL_MOR));

    /* MPDDRC_LPDDR2_CAL_MR4 */
    /*With the LPDDR2-SDRAM device, RDIV field must be equal to DS (Drive strength) field ”DS: Drive Strength” on page 46*/
    reg = readl(AT91C_BASE_MPDDRC + MPDDRC_LPDDR2_CAL_MR4);
    reg &= ~(0xFFFF);
    reg |= 3;
    writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_LPDDR2_CAL_MR4));

    /* MPDDRC I/O Calibration Register */
    /* DDR2 RZQ = 50 Ohm */
    /* TZQIO = 4 */
    reg = readl(AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR);
    reg &= ~AT91C_MPDDRC_RDIV;
    reg &= ~AT91C_MPDDRC_TZQIO;
    /*With the LPDDR2-SDRAM device, RDIV field must be equal to MPDDRC_LPDDR2_LPR DS
        (Drive strength) field ”DS: Drive Strength” on page 46*/
    reg |= AT91C_MPDDRC_RDIV_LPDDR2_RZQ_48;
    reg |= AT91C_MPDDRC_TZQIO_3;
    writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR));

    /* DDRSDRC High Speed Register (MPDDRC_HS)  : hidden option -> calibration during autorefresh */
    *(unsigned volatile int *)0xFFFFEA24 |= (1 << 5);


/*  Initialization sequence STEP 1
    Program the memory device type into the Memory Device Register */
    /*   Memory device = LPDDR2 => MPDDRC_MD_MD_LPDDR2_SDRAM
     Data bus width = 32 bits => 0x0 (The system is in 64 bits, thus memory data bus width should be 32 bits) */
    reg = AT91C_DDRC2_MD_LPDDR2_SDRAM;
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_MDR));


/* Initialization sequence STEP 2
               Program the features of Low-power DDR2-SDRAM device into the Timing Register
              (asynchronous timing, trc, tras, etc.) and into the Configuration Register (number of
              columns, rows, banks, CAS latency and output drive strength) (see Section 8.3 on
              page 35, Section 8.4 on page 39 and Section 80.5 on page 41). */
    reg = (AT91C_DDRC2_NC_DDR10_SDR9
          | AT91C_DDRC2_NR_14
          | AT91C_DDRC2_CAS_3
          | AT91C_DDRC2_ZQ_SHORT
          | AT91C_DDRC2_NB_BANKS_8
          | AT91C_DDRC2_UNAL_SUPPORTED);
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_CR));

    /*With the LPDDR2-SDRAM device, IO_CALIBR RDIV field must be equal to LPR DS (Drive strength) field ”DS:
        Drive Strength” on page 46*/
    reg = readl(AT91C_BASE_MPDDRC + MPDDRC_LPDDR2_LPR);
    reg |= AT91C_LPDDRC2_DS(0x03);
    writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_LPDDR2_LPR));

    reg = (AT91C_DDRC2_TRAS_(6)
            | AT91C_DDRC2_TRCD_(2)
            | AT91C_DDRC2_TWR_(3)
            | AT91C_DDRC2_TRC_(8)
            | AT91C_DDRC2_TRP_(2)
            | AT91C_DDRC2_TRRD_(2)
            | AT91C_DDRC2_TWTR_(2)
            | AT91C_DDRC2_TMRD_(3));
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_T0PR));

    reg = (AT91C_DDRC2_TXP_(2)
            | AT91C_DDRC2_TXSNR_(18)
            | AT91C_DDRC2_TXSRD_(14)
            | AT91C_DDRC2_TRFC_(17));
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_T1PR));

    reg = (AT91C_DDRC2_TFAW_(8)
            | AT91C_DDRC2_TRTP_(2)
            | AT91C_DDRC2_TRPA_(3)
            | AT91C_DDRC2_TXARDS_(1)
            | AT91C_DDRC2_TXARD_(1));
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_T2PR));

/*  Initialization sequence STEP 3
    An NOP command is issued to the Low-power DDR2-SDRAM. Program the NOP
    command into the Mode Register, the application must set the MODE (MDDRC Command
    Mode) field to 1 in the Mode Register (see Section 8.1 on page 32). Perform a
    write access to any Low-power DDR2-SDRAM address to acknowledge this command.
    Now, clocks which drive Low-power DDR2-SDRAM devices are enabled.
    A minimum pause of 100 ns must be observed to precede any signal toggle. */

    reg = AT91C_DDRC2_MODE_NOP_CMD;
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_MR)); // NOP to ENABLE CLOCK output
    *(unsigned volatile int *)AT91C_BASE_DDRCS  = 0x00000000;               // Access to memory

/*  Initialization sequence STEP 4:
    wait at least 100 ns */

    udelay(2);

/*  Initialization sequence STEP 5
    An NOP command is issued to the Low-power DDR2-SDRAM. Program the NOP
    command into the Mode Register, the application must set MODE to 1 in the Mode
    Register (see Section 8.1 on page 32). Perform a write access to any Low-power
    DDR2-SDRAM address to acknowledge this command. Now, CKE is driven high.
    A minimum pause of 200 us must be satisfied before Reset Command.
*/

    reg = AT91C_DDRC2_MODE_NOP_CMD;
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_MR)); // NOP to ENABLE CLOCK output
    *(unsigned volatile int *)AT91C_BASE_DDRCS  = 0x00000000;               // Access to memory

/*  Initialization sequence STEP 6
    wait at least 200 ns */

    udelay(500);

/*  Initialization sequence STEP 7
    A reset command is issued to the Low-power DDR2-SDRAM. Program
    LPDDR2_CMD in the MODE (MDDRC Command Mode) and MRS (Mode Register
    Select LPDDR2) field of the Mode Register, the application must set MODE to 7 and
    MRS to 63. (see Section 8.1 on page 32). Perform a write access to any Low-power
    DDR2-SDRAM address to acknowledge this command. Now, the reset command is issued.
    A minimum pause of 1us must be satisfied before any commands. */

    reg = AT91C_DDRC2_MRS(0x3F) | AT91C_DDRC2_MODE_LPDDR2_CMD;
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_MR));
    *(unsigned volatile int *)AT91C_BASE_DDRCS  = 0x00000000;

/*  Initialization sequence STEP 8
    wait at least 1us */

    udelay(2);

/*  Initialization sequence STEP 9
    A calibration command is issued to the Low-power DDR2-SDRAM. Program the type
    of calibration into the Configuration Register, ZQ field, RESET value (see Section 8.3
    ”MPDDRC Configuration Register?on page 37). In the Mode Register, program the
    MODE field to LPDDR2_CMD value, and the MRS field; the application must set
    MODE to 7 and MRS to 10 (see Section 8.1 LPDDRC Mode Register?on page 34).
    Perform a write access to any Low-power DDR2-SDRAM address to acknowledge
    this command. Now, the ZQ Calibration command is issued. Program the type of calibration
    into the Configuration Register, ZQ field */

    reg = readl(AT91C_BASE_MPDDRC + HDDRSDRC2_CR);
    reg &= ~AT91C_DDRC2_ZQ;
    reg |= AT91C_DDRC2_ZQ_RESET;
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_CR));

    reg =  AT91C_DDRC2_MODE_LPDDR2_CMD | AT91C_DDRC2_MRS(0x0A); // Mode Register Read  command. MODE = 0x7 and MRS = 0x0A
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_MR));

    *(unsigned volatile int *)AT91C_BASE_DDRCS  = 0x00000000; // Access to memory
    udelay(2); // Delay loop (at least 500 ns)

    reg = readl(AT91C_BASE_MPDDRC + HDDRSDRC2_CR);
    reg &= ~AT91C_DDRC2_ZQ;
    reg |= AT91C_DDRC2_ZQ_SHORT;
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_CR));

/*  Initialization sequence STEP 10
    A Mode Register Write command is issued to the Low-power DDR2-SDRAM. Program
    LPPDR2_CMD in the MODE and MRS field in the Mode Register, the
    application must set MODE to 7 and must set MRS field to 1 (see Section 8.1 on
    page 32). The Mode Register Write command cycle is issued to program the parameters
    of the Low-power DDR2-SDRAM devices, in particular burst length. Perform a
    write access to any Low-power DDR2-SDRAM address to acknowledge this command.
    Now, the Mode Register Write command is issued. */

    reg =  AT91C_DDRC2_MODE_LPDDR2_CMD | AT91C_DDRC2_MRS(0x01);
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_MR));
    *(unsigned volatile int *)AT91C_BASE_DDRCS  = 0x00000000;
    udelay(1); // Delay loop (at least 500 ns)

/*  Initialization sequence STEP 11
    Mode Register Write Command is issued to the Low-power DDR2-SDRAM. Program
    LPPDR2_CMD in the MODE and MRS field in the Mode Register, the
    application must set MODE to 7 and must set MRS field to 2. (see Section 8.1 on
    page 32). The Mode Register Write command cycle is issued to program the parameters
    of the Low-power DDR2-SDRAM devices, in particular CAS latency. Perform a
    write access to any Low-power DDR2-SDRAM address to acknowledge this command.
    Now, the Mode Register Write command is issued. */

    reg =  AT91C_DDRC2_MODE_LPDDR2_CMD | AT91C_DDRC2_MRS(0x02);
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_MR));
    *(unsigned volatile int *)AT91C_BASE_DDRCS  = 0x00000000;
    udelay(1); // Delay loop (at least 500 ns)

/*  Initialization sequence STEP 12
    A Mode Register Write Command is issued to the Low-power DDR2-SDRAM. Program
    LPPDR2_CMD in the MODE and MRS field of the Mode Register, the
    application must set MODE to 7 and must set MRS field to 3. (see Section 8.1 on
    page 32). The Mode Register Write command cycle is issued to program the parameters
    of the Low-power DDR2-SDRAM devices, in particular Drive Strength and Slew
    Rate. Perform a write access to any Low-power DDR2-SDRAM address to acknowledge
    this command. Now, the Mode Register Write command is issued. */

    reg =  AT91C_DDRC2_MODE_LPDDR2_CMD | AT91C_DDRC2_MRS(0x03);
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_MR));
    *(unsigned volatile int *)AT91C_BASE_DDRCS  = 0x00000000;
    udelay(1); // Delay loop (at least 500 ns)

/*  Initialization sequence STEP 13
    A Mode Register Write Command is issued to the Low-power DDR2-SDRAM. Program
    LPPDR2_CMD in the MODE and MRS field of the Mode Register, the
    application must set MODE to 7 and must set MRS field to 16. (see Section 8.1 on
    page 32). Mode Register Write command cycle is issued to program the parameters
    of the Low-power DDR2-SDRAM devices, in particular Partial Array Self Refresh
   (PASR). Perform a write access to any Low-power DDR2-SDRAM address to
    acknowledge this command. Now, the Mode Register Write command is issued.*/

    reg =  AT91C_DDRC2_MODE_LPDDR2_CMD | AT91C_DDRC2_MRS(0x10);
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_MR));
    *(unsigned volatile int *)AT91C_BASE_DDRCS  = 0x00000000;
    udelay(1); // Delay loop (at least 500 ns)


/*  Initialization sequence STEP 14
    A Normal Mode command is provided. Program the Normal mode in the MPDDRC_MR. Read the
    MPDDRC_MR and add a memory barrier assembler instruction just after the read. Perform a write access to
    any low-power DDR2-SDRAM address to acknowledge this command. */

    writel(AT91C_DDRC2_MODE_NORMAL_CMD, (AT91C_BASE_MPDDRC + HDDRSDRC2_MR));
    *(unsigned volatile int *)AT91C_BASE_DDRCS  = 0x00000000;

/*  Initialization sequence STEP 15
    In the DDR configuration Register (SFR_DDRCCFG), the application must write a 0 to fields 17 and 16 to
    close the input buffers. The buffers are then driven by the HMPDDRC controller. */
    /* SFR_DDRCFG  DDR Configuration  Force DDR_DQ and DDR_DQS input buffer always on */

    *(unsigned volatile int *)0xF0038004 |= (0x3 << 16);


/*  Initialization sequence STEP 16
    Write the refresh rate into the COUNT field in the Refresh Timer register (see page
    33). (Refresh rate = delay between refresh cycles). The Low-power DDR2-SDRAM
    device requires a refresh every 7.81 ìs. With a 100 MHz frequency, the refresh timer
    count register must to be set with (7.81/100 MHz) = 781 i.e. 0x030d. */

    reg = readl(AT91C_BASE_MPDDRC + HDDRSDRC2_RTR);
    reg &= ~AT91C_DDRC2_COUNT;
    reg |= 999; /*1030;*/  /*7.81 * MCK: for 132MHz: 1030; 128MHz: 999 */
    writel(reg, (AT91C_BASE_MPDDRC + HDDRSDRC2_RTR));

    writel(AT91C_DDRC2_MODE_NORMAL_CMD, (AT91C_BASE_MPDDRC + HDDRSDRC2_MR));

    *(unsigned volatile int *)AT91C_BASE_DDRCS  = 0x00000000; // Access to memory
    udelay(1); // Delay loop (at least 500 ns)

    /* DDRAM2 Controller initialize */
//  lpddr2_sdram_initialize(AT91C_BASE_MPDDRC, AT91C_BASE_DDRCS, &ddramc_reg);
}

#elif defined(CONFIG_DDR2)

static void ddramc_reg_config(struct ddramc_register *ddramc_config)
{
    ddramc_config->mdr = (AT91C_DDRC2_DBW_32_BITS
                | AT91C_DDRC2_MD_DDR2_SDRAM);

    ddramc_config->cr = (AT91C_DDRC2_NC_DDR10_SDR9
                | AT91C_DDRC2_NR_13
                | AT91C_DDRC2_CAS_3
                | AT91C_DDRC2_DLL_RESET_DISABLED
                | AT91C_DDRC2_DIS_DLL_DISABLED
                | AT91C_DDRC2_ENRDM_ENABLE
                | AT91C_DDRC2_NB_BANKS_8
                | AT91C_DDRC2_NDQS_DISABLED
                | AT91C_DDRC2_DECOD_INTERLEAVED
                | AT91C_DDRC2_UNAL_SUPPORTED);

#if defined(CONFIG_BUS_SPEED_133MHZ)
    /*
     * The DDR2-SDRAM device requires a refresh every 15.625 us or 7.81 us.
     * With a 133 MHz frequency, the refresh timer count register must to be
     * set with (15.625 x 133 MHz) ~ 2084 i.e. 0x824
     * or (7.81 x 133 MHz) ~ 1039 i.e. 0x40F.
     */
    ddramc_config->rtr = 0x40F;     /* Refresh timer: 7.812us */

    /* One clock cycle @ 133 MHz = 7.5 ns */
    ddramc_config->t0pr = (AT91C_DDRC2_TRAS_(6) /* 6 * 7.5 = 45 ns */
            | AT91C_DDRC2_TRCD_(2)      /* 2 * 7.5 = 22.5 ns */
            | AT91C_DDRC2_TWR_(2)       /* 2 * 7.5 = 15   ns */
            | AT91C_DDRC2_TRC_(8)       /* 8 * 7.5 = 75   ns */
            | AT91C_DDRC2_TRP_(2)       /* 2 * 7.5 = 15   ns */
            | AT91C_DDRC2_TRRD_(2)      /* 2 * 7.5 = 15   ns */
            | AT91C_DDRC2_TWTR_(2)      /* 2 clock cycles min */
            | AT91C_DDRC2_TMRD_(2));    /* 2 clock cycles */

    ddramc_config->t1pr = (AT91C_DDRC2_TXP_(2)  /* 2 clock cycles */
            | AT91C_DDRC2_TXSRD_(200)   /* 200 clock cycles */
            | AT91C_DDRC2_TXSNR_(19)    /* 19 * 7.5 = 142.5 ns */
            | AT91C_DDRC2_TRFC_(17));   /* 17 * 7.5 = 127.5 ns */

    ddramc_config->t2pr = (AT91C_DDRC2_TFAW_(6) /* 6 * 7.5 = 45 ns */
            | AT91C_DDRC2_TRTP_(2)      /* 2 clock cycles min */
            | AT91C_DDRC2_TRPA_(2)      /* 2 * 7.5 = 15 ns */
            | AT91C_DDRC2_TXARDS_(8)    /* = TXARD */
            | AT91C_DDRC2_TXARD_(8));   /* MR12 = 1 */

#elif defined(CONFIG_BUS_SPEED_166MHZ)
    /*
     * The DDR2-SDRAM device requires a refresh of all rows every 64ms.
     * ((64ms) / 8192) * 166MHz = 1296 i.e. 0x510
     */
    ddramc_config->rtr = 0x510;

    /* One clock cycle @ 166 MHz = 6.0 ns */
    ddramc_config->t0pr = (AT91C_DDRC2_TRAS_(8) /* 8 * 6 = 48 ns */
            | AT91C_DDRC2_TRCD_(3)      /* 3 * 6 = 18 ns */
            | AT91C_DDRC2_TWR_(3)       /* 3 * 6 = 18 ns */
            | AT91C_DDRC2_TRC_(10)      /* 10 * 6 = 60 ns */
            | AT91C_DDRC2_TRP_(3)       /* 3 * 6 = 18 ns */
            | AT91C_DDRC2_TRRD_(2)      /* 2 * 6 = 12 ns */
            | AT91C_DDRC2_TWTR_(2)      /* 2 clock cycles */
            | AT91C_DDRC2_TMRD_(2));    /* 2 clock cycles */

    ddramc_config->t1pr = (AT91C_DDRC2_TXP_(2)  /* 2 * 6 = 12ns */
            | AT91C_DDRC2_TXSRD_(200)   /* 200 clock cycles */
            | AT91C_DDRC2_TXSNR_(23)    /* 23 * 6 = 138 ns */
            | AT91C_DDRC2_TRFC_(22));   /* 22 * 6 = 132 ns */

    ddramc_config->t2pr = (AT91C_DDRC2_TFAW_(8) /* 45 ns */
            | AT91C_DDRC2_TRTP_(2)      /* 2 * 6 = 15ns */
            | AT91C_DDRC2_TRPA_(3)      /* 15 ns */
            | AT91C_DDRC2_TXARDS_(8)    /* = TXARD */
            | AT91C_DDRC2_TXARD_(8));   /* 8 clock cycles */

#else
#error "No bus clock provided!"
#endif
}

static void ddramc_init(void)
{
    struct ddramc_register ddramc_reg;
    unsigned int reg;

    ddramc_reg_config(&ddramc_reg);

    /* enable ddr2 clock */
    pmc_enable_periph_clock(AT91C_ID_MPDDRC);
    pmc_enable_system_clock(AT91C_PMC_DDR);

    /* Init the special register for sama5d3x */
    /* MPDDRC DLL Slave Offset Register: DDR2 configuration */
    reg = AT91C_MPDDRC_S0OFF_1
        | AT91C_MPDDRC_S2OFF_1
        | AT91C_MPDDRC_S3OFF_1;
    writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_DLL_SOR));

    /* MPDDRC DLL Master Offset Register */
    /* write master + clk90 offset */
    reg = AT91C_MPDDRC_MOFF_7
        | AT91C_MPDDRC_CLK90OFF_31
        | AT91C_MPDDRC_SELOFF_ENABLED | AT91C_MPDDRC_KEY;
    writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_DLL_MOR));

    /* MPDDRC I/O Calibration Register */
    /* DDR2 RZQ = 50 Ohm */
    /* TZQIO = 4 */
    reg = AT91C_MPDDRC_RDIV_DDR2_RZQ_50
        | AT91C_MPDDRC_TZQIO_4;
    writel(reg, (AT91C_BASE_MPDDRC + MPDDRC_IO_CALIBR));

    /* DDRAM2 Controller initialize */
    ddram_initialize(AT91C_BASE_MPDDRC, AT91C_BASE_DDRCS, &ddramc_reg);
}
#else
#error "No right DDR-SDRAM device type provided"
#endif /* #ifdef CONFIG_LPDDR2 */



#ifdef CONFIG_USER_HW_INIT
/*
 * Special setting for PM.
 * Since for the chips with no EMAC or GMAC, No actions is done to make
 * its phy to enter the power save mode when linux system enter suspend
 * to memory or standby.
 * And it causes the VDDCORE current is higher than our expection.
 * So set GMAC clock related pins GTXCK(PB8), GRXCK(PB11), GMDCK(PB16),
 * G125CK(PB18) and EMAC clock related pins EREFCK(PC7), EMDC(PC8)
 * to Pullup and Pulldown disabled, and output low.
 */

#define GMAC_PINS   ((0x01 << 8) | (0x01 << 11) \
                | (0x01 << 16) | (0x01 << 18))

#define EMAC_PINS   ((0x01 << 7) | (0x01 << 8))

static void at91_special_pio_output_low(void)
{
    unsigned int base;
    unsigned int value;

    base = AT91C_BASE_PIOB;
    value = GMAC_PINS;

    writel((1 << AT91C_ID_PIOB), (PMC_PCER + AT91C_BASE_PMC));

    writel(value, base + PIO_REG_PPUDR);    /* PIO_PPUDR */
    writel(value, base + PIO_REG_PPDDR);    /* PIO_PPDDR */
    writel(value, base + PIO_REG_PER);  /* PIO_PER */
    writel(value, base + PIO_REG_OER);  /* PIO_OER */
    writel(value, base + PIO_REG_CODR); /* PIO_CODR */

    base = AT91C_BASE_PIOC;
    value = EMAC_PINS;

    writel((1 << AT91C_ID_PIOC), (PMC_PCER + AT91C_BASE_PMC));

    writel(value, base + PIO_REG_PPUDR);    /* PIO_PPUDR */
    writel(value, base + PIO_REG_PPDDR);    /* PIO_PPDDR */
    writel(value, base + PIO_REG_PER);  /* PIO_PER */
    writel(value, base + PIO_REG_OER);  /* PIO_OER */
    writel(value, base + PIO_REG_CODR); /* PIO_CODR */
}
#endif

#if defined(CONFIG_MAC0_PHY)
unsigned int at91_eth0_hw_init(void)
{
    unsigned int base_addr = AT91C_BASE_GMAC;

    const struct pio_desc macb_pins[] = {
        {"GMDC",    AT91C_PIN_PB(16), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {"GMDIO",   AT91C_PIN_PB(17), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
    };

    pio_configure(macb_pins);
    pmc_enable_periph_clock(AT91C_ID_PIOB);

    pmc_enable_periph_clock(AT91C_ID_GMAC);

    return base_addr;
}
#endif

#if defined(CONFIG_MAC1_PHY)
unsigned int at91_eth1_hw_init(void)
{
    unsigned int base_addr = AT91C_BASE_EMAC;

    const struct pio_desc macb_pins[] = {
        {"EMDC",    AT91C_PIN_PC(8), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {"EMDIO",   AT91C_PIN_PC(9), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
    };

    pio_configure(macb_pins);
    pmc_enable_periph_clock(AT91C_ID_PIOC);

    pmc_enable_periph_clock(AT91C_ID_EMAC);

    return base_addr;
}
#endif

#if defined(CONFIG_MACB)
void at91_disable_mac_clock(void)
{
#if defined(CONFIG_MAC0_PHY)
    pmc_disable_periph_clock(AT91C_ID_GMAC);
#endif
#if defined(CONFIG_MAC1_PHY)
    pmc_disable_periph_clock(AT91C_ID_EMAC);
#endif
}
#endif

#if defined(CONFIG_TWI0)
unsigned int at91_twi0_hw_init(void)
{
    return 0;
}
#endif

#if defined(CONFIG_TWI1)
unsigned int at91_twi1_hw_init(void)
{
    unsigned int base_addr = AT91C_BASE_TWI1;

    const struct pio_desc twi_pins[] = {
        {"TWD", AT91C_PIN_PC(26), 0, PIO_DEFAULT, PIO_PERIPH_B},
        {"TWCK", AT91C_PIN_PC(27), 0, PIO_DEFAULT, PIO_PERIPH_B},
        {(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
    };

    pio_configure(twi_pins);
    pmc_enable_periph_clock(AT91C_ID_PIOC);

    pmc_enable_periph_clock(AT91C_ID_TWI1);

    return base_addr;
}
#endif

#if defined(CONFIG_TWI2)
unsigned int at91_twi2_hw_init(void)
{
    return 0;
}
#endif

#if defined(CONFIG_AUTOCONFIG_TWI_BUS)
void at91_board_config_twi_bus(void)
{
    act8865_twi_bus = 1;
}
#endif

#if defined(CONFIG_ACT8865_SET_VOLTAGE)
int at91_board_act8865_set_reg_voltage(void)
{
    unsigned char reg, value;
    int ret;

    /* Check ACT8865 I2C interface */
    if (act8865_check_i2c_disabled()) {
//      dbg_info("ACT8865: I2C is disabled, exiting set_reg_voltage()\n");
        return 0;
    }

    /* Enable REG2 output 1.25V */
    reg = REG2_0;
    value = ACT8865_1V25;
    ret = act8865_set_reg_voltage(reg, value);
    if (ret) {
        dbg_info("ACT8865: Failed to make REG2 output 1250mV\n");
        return -1;
    }

    dbg_info("ACT8865: The REG2 output 1250mV\n");

    /* Enable REG5 output 3.3V */
    reg = REG5_0;
    value = ACT8865_3V3;
    ret = act8865_set_reg_voltage(reg, value);
    if (ret) {
        dbg_info("ACT8865: Failed to make REG5 output 3300mV\n");
        return -1;
    }

    dbg_info("ACT8865: The REG5 output 3300mV\n");

    return 0;
}
#endif

#if defined(CONFIG_PM)
void at91_disable_smd_clock(void)
{
    /*
     * set pin DIBP to pull-up and DIBN to pull-down
     * to save power on VDDIOP0
     */
    pmc_enable_system_clock(AT91C_PMC_SMDCK);
    pmc_set_smd_clock_divider(AT91C_PMC_SMDDIV);
    pmc_enable_periph_clock(AT91C_ID_SMD);
    writel(0xF, (0x0C + AT91C_BASE_SMD));
    pmc_disable_periph_clock(AT91C_ID_SMD);
    pmc_disable_system_clock(AT91C_PMC_SMDCK);
}
#endif

#ifdef CONFIG_HW_INIT
void hw_init(void)
{
	/*LPDDR enable*/    
	const struct pio_desc pe1_pins[] = {
        {"PE1", AT91C_PIN_PE(1), 0, PIO_DEFAULT, PIO_OUTPUT},
        {(char *)0, 0, 0, PIO_DEFAULT, PIO_OUTPUT},
    };

	int i, j;

    /* Disable watchdog */
    at91_disable_wdt();

    /*
     * At this stage the main oscillator
     * is supposed to be enabled PCK = MCK = MOSC
     */
#if defined(CONFIG_CPU_CLK_512MHZ)
    sci_clock_init();
#else
    /* Configure PLLA = MOSC * (PLL_MULA + 1) / PLL_DIVA */
    pmc_cfg_plla(PLLA_SETTINGS);

    /* Initialize PLLA charge pump */
    pmc_init_pll(AT91C_PMC_IPLLA_3);

    /* Switch PCK/MCK on Main clock output */
    pmc_cfg_mck(BOARD_PRESCALER_MAIN_CLOCK);

    /* Switch PCK/MCK on PLLA output */
    pmc_cfg_mck(BOARD_PRESCALER_PLLA);
#endif

#ifdef CONFIG_USER_HW_INIT
    /* Set GMAC & EMAC pins to output low */
    at91_special_pio_output_low();
#endif

    pio_configure(pe1_pins);
	/* wait the pin stable before LPDDR init */
	for (i = 0; i < 0x1ffff; i++)
	{
		j = 1;
	}

    /* Init timer */
    timer_init();

    /* initialize the dbgu */
    initialize_dbgu();

#ifdef CONFIG_LPDDR2
    /* Initialize MPDDR Controller */
    lpddr2_init();
#elif CONFIG_DDR2
    /* Initialize MPDDR Controller */
    ddramc_init();
#endif
}
#endif /* #ifdef CONFIG_HW_INIT */

#ifdef CONFIG_DATAFLASH
void at91_spi0_hw_init(void)
{
    /* Configure PIN for SPI0 */
    const struct pio_desc spi0_pins[] = {
        {"MISO",    AT91C_PIN_PD(10), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {"MOSI",    AT91C_PIN_PD(11), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {"SPCK",    AT91C_PIN_PD(12), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {"NPCS",    CONFIG_SYS_SPI_PCS, 1, PIO_DEFAULT, PIO_OUTPUT},
        {(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
    };

    /* Configure the PIO controller */
    pmc_enable_periph_clock(AT91C_ID_PIOD);
    pio_configure(spi0_pins);

    /* Enable the clock */
    pmc_enable_periph_clock(AT91C_ID_SPI0);
}
#endif /* #ifdef CONFIG_DATAFLASH */

#ifdef CONFIG_SDCARD
#ifdef CONFIG_OF_LIBFDT
void at91_board_set_dtb_name(char *of_name)
{
    strcat(of_name, "at91-sama5d3_xplained.dtb");
}
#endif

void at91_mci0_hw_init(void)
{
    const struct pio_desc mci_pins[] = {
        {"MCCK", AT91C_PIN_PD(9), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {"MCCDA", AT91C_PIN_PD(0), 0, PIO_DEFAULT, PIO_PERIPH_A},

        {"MCDA0", AT91C_PIN_PD(1), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {"MCDA1", AT91C_PIN_PD(2), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {"MCDA2", AT91C_PIN_PD(3), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {"MCDA3", AT91C_PIN_PD(4), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {"MCDA4", AT91C_PIN_PD(5), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {"MCDA5", AT91C_PIN_PD(6), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {"MCDA6", AT91C_PIN_PD(7), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {"MCDA7", AT91C_PIN_PD(8), 0, PIO_DEFAULT, PIO_PERIPH_A},
        {(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
    };

    /* Configure the PIO controller */
    pmc_enable_periph_clock(AT91C_ID_HSMCI0);
    pio_configure(mci_pins);

    /* Enable the clock */
    pmc_enable_periph_clock(AT91C_ID_HSMCI0);
}
#endif /* #ifdef CONFIG_SDCARD */

#ifdef CONFIG_NANDFLASH
void nandflash_hw_init(void)
{
    /* Configure nand pins */
    const struct pio_desc nand_pins[] = {
        {"NANDALE", AT91C_PIN_PE(21), 0, PIO_PULLUP, PIO_PERIPH_A},
        {"NANDCLE", AT91C_PIN_PE(22), 0, PIO_PULLUP, PIO_PERIPH_A},
        {(char *)0, 0, 0, PIO_DEFAULT, PIO_PERIPH_A},
    };

    /* Configure the nand controller pins*/
    pmc_enable_periph_clock(AT91C_ID_PIOE);
    pio_configure(nand_pins);

    /* Enable the clock */
    pmc_enable_periph_clock(AT91C_ID_SMC);

    /* Configure SMC CS3 for NAND/SmartMedia */
    writel(AT91C_SMC_SETUP_NWE(1)
        | AT91C_SMC_SETUP_NCS_WR(1)
        | AT91C_SMC_SETUP_NRD(2)
        | AT91C_SMC_SETUP_NCS_RD(1),
        (ATMEL_BASE_SMC + SMC_SETUP3));

    writel(AT91C_SMC_PULSE_NWE(5)
        | AT91C_SMC_PULSE_NCS_WR(7)
        | AT91C_SMC_PULSE_NRD(5)
        | AT91C_SMC_PULSE_NCS_RD(7),
        (ATMEL_BASE_SMC + SMC_PULSE3));

    writel(AT91C_SMC_CYCLE_NWE(8)
        | AT91C_SMC_CYCLE_NRD(9),
        (ATMEL_BASE_SMC + SMC_CYCLE3));

    writel(AT91C_SMC_TIMINGS_TCLR(3)
        | AT91C_SMC_TIMINGS_TADL(10)
        | AT91C_SMC_TIMINGS_TAR(3)
        | AT91C_SMC_TIMINGS_TRR(4)
        | AT91C_SMC_TIMINGS_TWB(5)
        | AT91C_SMC_TIMINGS_RBNSEL(3)
        | AT91C_SMC_TIMINGS_NFSEL,
        (ATMEL_BASE_SMC + SMC_TIMINGS3));

    writel(AT91C_SMC_MODE_READMODE_NRD_CTRL
        | AT91C_SMC_MODE_WRITEMODE_NWE_CTRL
        | AT91C_SMC_MODE_EXNWMODE_DISABLED
        | AT91C_SMC_MODE_DBW_8
        | AT91C_SMC_MODE_TDF_CYCLES(1),
        (ATMEL_BASE_SMC + SMC_MODE3));
}
#endif /* #ifdef CONFIG_NANDFLASH */
#ifdef CONFIG_QNX_USE_BOOT_DATA
uint32_t crc32_calculate(uint32_t crc32, const void *buffer, uint32_t nbytes)
{
    const uint8_t *buffer_cur = buffer;
    const uint8_t *buffer_end = buffer_cur + nbytes;
    while (buffer_cur < buffer_end)
    {
        crc32 = (crc32 << 8) ^ crc32_table[(crc32 >> 24) ^ ((uint32_t)(*buffer_cur++))];
    }

    return crc32;
}
static bool is_boot_data_valid(const boot_data_t *boot_data)
{
    uint32_t crc32;

    /* nothing we can do if it's the wrong structure type */
    if (boot_data->magic != BOOT_DATA_MAGIC)
    {
        dbg_very_loud("%s: Expected a magic value of %x but found %x\n", __FUNCTION__, BOOT_DATA_MAGIC, boot_data->magic);
        return false;
    }

    crc32 = crc32_calculate(0, (const uint8_t *)&boot_data->size, boot_data->size - (sizeof(uint32_t) * 2));

    return crc32 == boot_data->crc32;
}

static void process_boot_data(struct image_info *image, boot_data_t *boot_data[NUM_BOOT_DATA], uint8_t *boot_data_index)
{
    bool boot_data_valid[NUM_BOOT_DATA];

    boot_data_valid[0] = is_boot_data_valid(boot_data[0]);
    boot_data_valid[1] = is_boot_data_valid(boot_data[1]);

    if (boot_data_valid[0] && boot_data_valid[1])
    {
        if (boot_data[0]->sequence >= boot_data[1]->sequence)
        {
            *boot_data_index = 0;
        }
        else
        {
            *boot_data_index = 1;
        }

        dbg_info("%s: Using boot_data %u (both valid)\n", __FUNCTION__, *boot_data_index);
    }
    else if (boot_data_valid[0])
    {
        dbg_info("%s: Using boot_data 0 (1 is invalid)\n", __FUNCTION__);
        *boot_data_index = 0;
    }
    else if (boot_data_valid[1])
    {
        dbg_info("%s: Using boot_data 1 (0 is invalid)\n", __FUNCTION__);
        *boot_data_index = 1;
    }
    else
    {
        dbg_very_loud("%s: Both boot_data structures are invalid", __FUNCTION__);
        *boot_data_index = 0xff;
        return;
    }

    if (boot_data[*boot_data_index]->updating)
    {
        /* We're rebooting as part of a system update process */
        if (boot_data[*boot_data_index]->configuration[0].failure_boot_count > SW_UPDATE_RESTART_MAX)
        {
            configuration_t configuration;

            /* Failed update detected. Loading previous configuration */
            dbg_very_loud("%s: Failed update detected - retry %u (max %u)\n", __FUNCTION__, boot_data[*boot_data_index]->configuration[0].failure_boot_count, SW_UPDATE_RESTART_MAX);

            /* Set error flag, reset update flag and swap configurations */
            boot_data[*boot_data_index]->error_flags |= BOOT_ERROR_IFS_UPDATE_BIT;
            boot_data[*boot_data_index]->updating = 0;
            memcpy(&configuration, &boot_data[*boot_data_index]->configuration[1], sizeof(configuration_t));
            memcpy(&boot_data[*boot_data_index]->configuration[1], &boot_data[*boot_data_index]->configuration[0], sizeof(configuration_t));
            memcpy(&boot_data[*boot_data_index]->configuration[0], &configuration, sizeof(configuration_t));
        }
        else
        {
            dbg_info("%s: Update detected (attempt %u of %u)\n", __FUNCTION__, boot_data[*boot_data_index]->configuration[0].failure_boot_count + 1, SW_UPDATE_RESTART_MAX + 1);
            boot_data[*boot_data_index]->configuration[0].failure_boot_count++;
        }
    }

    if (boot_data[*boot_data_index]->configuration[0].ifs_index != 1)
    {
        image->offset = IFS0_OFFSET;
    }
    else
    {
        image->offset = IFS1_OFFSET;
    }

    dbg_info("%s: Loading IFS %d from %x...\n", __FUNCTION__, boot_data[*boot_data_index]->configuration[0].ifs_index, image->offset);

    boot_data[*boot_data_index]->sequence++;
    boot_data[*boot_data_index]->hard_boot_count++;
    boot_data[*boot_data_index]->configuration[0].hard_boot_count++;

	/* Update bootdata version info */
    boot_data[*boot_data_index]->min_version = BOOT_DATA_MIN_VERSION;
    boot_data[*boot_data_index]->version = BOOT_DATA_VERSION;
}

#ifdef CONFIG_NANDFLASH

/* Based off of load_nandflash() in nandflash.c */
int load_nandflash_boot_data(struct image_info *image)
{
    struct nand_info nand;
    uint8_t boot_data_index;
    int ret;
    struct startup_header *shdr;
    boot_data_t *boot_data[NUM_BOOT_DATA];

    nandflash_hw_init();

    if (nandflash_get_type(&nand))
    {
        return -1;
    }

#ifdef CONFIG_USE_PMECC
    if (init_pmecc(&nand))
    {
        return -1;
    }
#endif

    /* Load both boot data structures from NAND */
    ret = nand_loadimage(&nand, BOOT_DATA0_OFFSET, PAGE_SIZE, &boot_data_nand_buffer[0]);
    if (ret)
    {
        dbg_very_loud("%s: Error reading BOOT_DATA_0 from NAND\n", __FUNCTION__);
        return ret;
    }
    else
    {
        boot_data[0] = (boot_data_t *)&boot_data_nand_buffer[0];
    }

    ret = nand_loadimage(&nand, BOOT_DATA1_OFFSET, PAGE_SIZE, &boot_data_nand_buffer[PAGE_SIZE]);
    if (ret)
    {
        dbg_very_loud("%s: Error reading BOOT_DATA_1 from NAND\n", __FUNCTION__);
        return ret;
    }
    else
    {
        boot_data[1] = (boot_data_t *)&boot_data_nand_buffer[PAGE_SIZE];
    }

    /* Read information from the boot data to fill out the image structure */
    process_boot_data(image, boot_data, &boot_data_index);

    if (boot_data_index >= NUM_BOOT_DATA)
    {
        dbg_very_loud("%s: Invalid boot_data_index: %d\n", __FUNCTION__, boot_data_index);
        return -1;
    }

    /* Read in the IFS header */
	ret = nand_loadimage(&nand, image->offset, PAGE_SIZE * 2, image->dest);
    shdr = (struct startup_header *)(image->dest + PREBOOT_SIZE);

    if (shdr->signature != STARTUP_HDR_SIGNATURE)
    {
        dbg_very_loud("%s: Invalid IFS signature\n", __FUNCTION__, boot_data_index);
        boot_data[boot_data_index]->error_flags |= BOOT_ERROR_IFS_SCAN_BIT;
        ret = -1;
        goto exit_load_nandflash;
    }

    image->length = shdr->ram_size + PREBOOT_SIZE;

    dbg_info("%s: Image: Copy %d bytes from %d to %d\n", __FUNCTION__,
            image->length, image->offset, image->dest);

    /* Load specified image from NAND*/
    ret = nand_loadimage(&nand, image->offset, image->length, image->dest);
    if (ret)
    {
        dbg_very_loud("%s: Error loading IFS from %d: %d\n", __FUNCTION__, image->offset, ret);
        boot_data[boot_data_index]->error_flags |= BOOT_ERROR_IFS_LOADING_BIT;
        goto exit_load_nandflash;
    }

    exit_load_nandflash:

    /* Save boot data out regardles of previous errors */
    boot_data[boot_data_index]->crc32 =
        crc32_calculate(0, (const uint8_t *)&boot_data[boot_data_index]->size, boot_data[boot_data_index]->size - (sizeof(uint32_t) * 2));
    if (boot_data_index == 0) {
        nand_saveimage(&nand, BOOT_DATA0_OFFSET, PAGE_SIZE, (unsigned char *)&boot_data_nand_buffer[0]);
    } else {
        nand_saveimage(&nand, BOOT_DATA1_OFFSET, PAGE_SIZE, (unsigned char *)&boot_data_nand_buffer[PAGE_SIZE]);
    }

    dbg_info("%s: Exiting with %d\n", __FUNCTION__, ret);

    return ret;
 }
#endif /* #ifdef CONFIG_NANDFLASH */

#endif /* #ifdef CONFIG_QNX_USE_BOOT_DATA */

void setLEDColor(void)
{
    unsigned int base;
    unsigned int value;

	/* send pulse to PA2 to control LED color */ 
    base = AT91C_BASE_PIOA;
    value = (0x1 << 2);
    writel(value, base + PIO_REG_PER);  /* PIO_PER */
    writel(value, base + PIO_REG_OER);  /* PIO_OER */

	mdelay(1);
	writel(value, base + PIO_REG_CODR);
	mdelay(1);
	writel(value, base + PIO_REG_SODR);
}
