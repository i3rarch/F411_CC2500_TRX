/**
 * @file    cli_handler.c
 * @brief   Command Line Interface handler implementation
 * @author  i3rarch
 * @date    2025
 */

#include "cli_handler.h"
#include "usbd_cdc_if.h"
#include "radio_handler.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ============================================================================
 * Private Defines
 * ========================================================================= */
#define CLI_RX_BUFFER_SIZE  64U
#define CLI_TX_BUFFER_SIZE  128U

/* ============================================================================
 * Private Types
 * ========================================================================= */

/**
 * @brief Baud rate configuration entry
 */
typedef struct {
    uint32_t baudrate;
    uint8_t mdmcfg4;
    uint8_t mdmcfg3;
} BaudRateConfig_t;

/**
 * @brief Power level configuration entry
 */
typedef struct {
    int8_t power_dbm;
    uint8_t pa_table_val;
} PowerConfig_t;

/* ============================================================================
 * Private Variables
 * ========================================================================= */
static char s_rx_buffer[CLI_RX_BUFFER_SIZE];
static uint32_t s_rx_index = 0;
static char s_tx_buffer[CLI_TX_BUFFER_SIZE];

static CC2500CTX *s_cc2500_ctx = NULL;
static uint8_t s_stored_deviatn = 0x44;     /* Default deviation ~47kHz */
static uint8_t s_current_pa_table_val = 0xFE; /* Default Power 0dBm */

/* Supported baud rates lookup table */
static const BaudRateConfig_t s_baud_rates[] = {
    {1200,   0xF5, 0x83},
    {2400,   0xF6, 0x83},
    {4800,   0xF7, 0x83},
    {9600,   0xE7, 0x83},
    {19200,  0xD7, 0x83},
    {38400,  0xDA, 0x83},
    {57600,  0xB9, 0x83},
    {125000, 0x2D, 0x55},
    {250000, 0x2D, 0x3B},
    {500000, 0x3E, 0x93}
};

/* Supported power levels lookup table */
static const PowerConfig_t s_power_levels[] = {
    {1,   0xFF},
    {0,   0xFE},
    {-2,  0xBB},
    {-4,  0xA9},
    {-6,  0x7F},
    {-8,  0x6E},
    {-10, 0x97},
    {-16, 0x55},
    {-20, 0x46},
    {-30, 0x50}
};

/* ============================================================================
 * Private Function Prototypes
 * ========================================================================= */
static void cli_process_command(const char *cmd);
static void cli_transmit(const char *str);
static void cli_handle_set_baud(uint32_t baudrate);
static void cli_handle_get_status(void);
static void cli_handle_set_freq(uint32_t freq_khz);
static void cli_handle_set_mod(const char *mod_str);
static void cli_handle_set_dev(uint32_t dev_khz);
static void cli_handle_set_power(int8_t power_dbm);
static void cli_handle_help(void);
static void cli_handle_dump_regs(void);
static void cli_handle_debug(const char *arg);

/* ============================================================================
 * Public Functions
 * ========================================================================= */

void cli_init(CC2500CTX *cc2500_context)
{
    s_cc2500_ctx = cc2500_context;
    s_rx_index = 0;
    memset(s_rx_buffer, 0, sizeof(s_rx_buffer));
}

void cli_process_input(uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        char ch = (char)buf[i];
        
        if (ch == '\r' || ch == '\n') {
            if (s_rx_index > 0) {
                s_rx_buffer[s_rx_index] = '\0';
                cli_process_command(s_rx_buffer);
                s_rx_index = 0;
            }
        } else if (s_rx_index < (CLI_RX_BUFFER_SIZE - 1)) {
            s_rx_buffer[s_rx_index++] = ch;
        }
        /* Characters beyond buffer size are silently dropped */
    }
}

/* ============================================================================
 * Private Functions
 * ========================================================================= */

static void cli_transmit(const char *str)
{
    if (str != NULL) {
        CDC_Transmit_FS((uint8_t *)str, (uint16_t)strlen(str));
    }
}

static void cli_process_command(const char *cmd)
{
    uint32_t value = 0;
    int32_t s_value = 0;
    char str_value[16] = {0};

    if (sscanf(cmd, "set_baud %lu", &value) == 1) {
        cli_handle_set_baud(value);
    } else if (sscanf(cmd, "set_freq %lu", &value) == 1) {
        cli_handle_set_freq(value);
    } else if (sscanf(cmd, "set_mod %15s", str_value) == 1) {
        cli_handle_set_mod(str_value);
    } else if (sscanf(cmd, "set_dev %lu", &value) == 1) {
        cli_handle_set_dev(value);
    } else if (sscanf(cmd, "set_power %ld", &s_value) == 1) {
        cli_handle_set_power((int8_t)s_value);
    } else if (strcmp(cmd, "get_status") == 0) {
        cli_handle_get_status();
    } else if (strcmp(cmd, "help") == 0) {
        cli_handle_help();
    } else if (strcmp(cmd, "dump_regs") == 0) {
        cli_handle_dump_regs();
    } else if (sscanf(cmd, "debug %15s", str_value) == 1) {
        cli_handle_debug(str_value);
    } else if (strcmp(cmd, "reboot") == 0) {
        cli_transmit("Rebooting system...\r\n");
        HAL_Delay(100);
        NVIC_SystemReset();
    } else {
        cli_transmit("Unknown command. Type 'help' for a list of commands.\r\n");
    }
}

/* ============================================================================
 * Command Handlers
 * ========================================================================= */

static void cli_handle_set_baud(uint32_t baudrate)
{
    const BaudRateConfig_t *cfg = NULL;
    
    /* Search for matching baud rate in lookup table */
    for (size_t i = 0; i < ARRAY_SIZE(s_baud_rates); i++) {
        if (s_baud_rates[i].baudrate == baudrate) {
            cfg = &s_baud_rates[i];
            break;
        }
    }
    
    if (cfg != NULL) {
        cc2500_writeRegister(s_cc2500_ctx, CC2500_10_MDMCFG4, cfg->mdmcfg4);
        cc2500_writeRegister(s_cc2500_ctx, CC2500_11_MDMCFG3, cfg->mdmcfg3);
        snprintf(s_tx_buffer, CLI_TX_BUFFER_SIZE, "Baud rate set to %lu\r\n", baudrate);
        cli_transmit(s_tx_buffer);
    } else {
        cli_transmit("Baud rate not supported. Use one of the predefined values.\r\n");
    }
}

static void cli_handle_set_freq(uint32_t freq_khz)
{
    /* CC2500 frequency range: 2400-2483.5 MHz */
    if (freq_khz < 2400000UL || freq_khz > 2483500UL) {
        cli_transmit("Frequency out of range (2400000 - 2483500 kHz).\r\n");
        return;
    }

    /* Calculate frequency register value
     * FREQ = (f_carrier * 2^16) / f_xosc
     * f_xosc = 26 MHz for CC2500 */
    uint64_t f_vco = (uint64_t)freq_khz * 1000ULL;
    uint32_t freq_reg = (uint32_t)((f_vco * 65536ULL) / 26000000ULL);

    uint8_t freq2 = (uint8_t)((freq_reg >> 16) & 0xFFU);
    uint8_t freq1 = (uint8_t)((freq_reg >> 8) & 0xFFU);
    uint8_t freq0 = (uint8_t)(freq_reg & 0xFFU);

    cc2500_writeRegister(s_cc2500_ctx, CC2500_0D_FREQ2, freq2);
    cc2500_writeRegister(s_cc2500_ctx, CC2500_0E_FREQ1, freq1);
    cc2500_writeRegister(s_cc2500_ctx, CC2500_0F_FREQ0, freq0);

    snprintf(s_tx_buffer, CLI_TX_BUFFER_SIZE, "Frequency set to %lu kHz\r\n", freq_khz);
    cli_transmit(s_tx_buffer);
}

static void cli_handle_set_mod(const char *mod_str)
{
    uint8_t mdmcfg2_val;
    uint8_t mod_format = 0;
    uint8_t frend0_val = 0x10;  /* Default FREND0 for FSK/GFSK/MSK */
    bool is_msk = false;
    bool is_ook = false;
    bool supported = true;

    if (strcmp(mod_str, "2fsk") == 0) {
        mod_format = 0x00;
    } else if (strcmp(mod_str, "gfsk") == 0) {
        mod_format = 0x10;
    } else if ((strcmp(mod_str, "ask") == 0) || (strcmp(mod_str, "ook") == 0)) {
        mod_format = 0x30;
        frend0_val = 0x11;  /* OOK uses PATABLE index 1 for TX '1' */
        is_ook = true;
    } else if (strcmp(mod_str, "msk") == 0) {
        mod_format = 0x70;
        is_msk = true;
    } else {
        supported = false;
    }

    if (!supported) {
        cli_transmit("Unsupported modulation. Use 2fsk, gfsk, ask/ook, msk.\r\n");
        return;
    }

    /* 1. Update MDMCFG2 (Modulation format) */
    cc2500_readRegister(s_cc2500_ctx, CC2500_12_MDMCFG2, &mdmcfg2_val);
    mdmcfg2_val &= 0x8FU;  /* Clear MOD_FORMAT bits [6:4] */
    mdmcfg2_val |= mod_format;
    cc2500_writeRegister(s_cc2500_ctx, CC2500_12_MDMCFG2, mdmcfg2_val);

    /* 2. Update FREND0 (PA power index selection) */
    cc2500_writeRegister(s_cc2500_ctx, CC2500_22_FREND0, frend0_val);

    /* 3. Update DEVIATN (Deviation) */
    if (is_msk) {
        /* For MSK, deviation must be 0 */
        cc2500_writeRegister(s_cc2500_ctx, CC2500_15_DEVIATN, 0x00);
    } else {
        /* Restore saved deviation value */
        cc2500_writeRegister(s_cc2500_ctx, CC2500_15_DEVIATN, s_stored_deviatn);
    }

    /* 4. Update PATABLE for OOK modulation */
    if (is_ook) {
        /* OOK: Index 0 = 0x00 (Off), Index 1 = Power (On) */
        uint8_t pa_values[2] = {0x00, s_current_pa_table_val};
        cc2500_writeRegisterBurst(s_cc2500_ctx, CC2500_3E_PATABLE, pa_values, 2);
    } else {
        /* FSK/GFSK/MSK use only index 0 */
        cc2500_writeRegister(s_cc2500_ctx, CC2500_3E_PATABLE, s_current_pa_table_val);
    }
    
    snprintf(s_tx_buffer, CLI_TX_BUFFER_SIZE, "Modulation set to %s\r\n", mod_str);
    cli_transmit(s_tx_buffer);
}

static void cli_handle_set_dev(uint32_t dev_khz)
{
    /* Maximum deviation check */
    if (dev_khz > 500UL) {
        cli_transmit("Deviation is too high (max 500 kHz).\r\n");
        return;
    }
    
    /* Calculate DEVIATN register value
     * Deviation = (f_xosc / 2^17) * (8 + DEVIATN_M) * 2^DEVIATN_E
     * This is a simplified calculation */
    uint32_t reg_val = (dev_khz * 1000UL * 131072UL) / 26000000UL;
    uint8_t deviatn = (reg_val > 255U) ? 255U : (uint8_t)reg_val;

    /* Store value for restoration after MSK mode */
    s_stored_deviatn = deviatn;

    cc2500_writeRegister(s_cc2500_ctx, CC2500_15_DEVIATN, deviatn);
    snprintf(s_tx_buffer, CLI_TX_BUFFER_SIZE, 
             "Deviation set to approx %lu kHz (reg: 0x%02X)\r\n", dev_khz, deviatn);
    cli_transmit(s_tx_buffer);
}

static void cli_handle_set_power(int8_t power_dbm)
{
    const PowerConfig_t *cfg = NULL;
    
    /* Search for matching power level in lookup table */
    for (size_t i = 0; i < ARRAY_SIZE(s_power_levels); i++) {
        if (s_power_levels[i].power_dbm == power_dbm) {
            cfg = &s_power_levels[i];
            break;
        }
    }
    
    if (cfg == NULL) {
        cli_transmit("Unsupported power level. See datasheet for PATABLE values.\r\n");
        return;
    }

    s_current_pa_table_val = cfg->pa_table_val;

    /* Check current modulation to update PATABLE correctly */
    uint8_t mdmcfg2;
    cc2500_readRegister(s_cc2500_ctx, CC2500_12_MDMCFG2, &mdmcfg2);
    
    if ((mdmcfg2 & 0x70U) == 0x30U) {
        /* OOK modulation active */
        uint8_t pa_values[2] = {0x00, s_current_pa_table_val};
        cc2500_writeRegisterBurst(s_cc2500_ctx, CC2500_3E_PATABLE, pa_values, 2);
    } else {
        cc2500_writeRegister(s_cc2500_ctx, CC2500_3E_PATABLE, s_current_pa_table_val);
    }

    snprintf(s_tx_buffer, CLI_TX_BUFFER_SIZE, "Output power set to %d dBm\r\n", power_dbm);
    cli_transmit(s_tx_buffer);
}

static void cli_handle_get_status(void)
{
    uint8_t partnum, version, marcstate;
    
    cc2500_readStatusRegister(s_cc2500_ctx, CC2500_30_PARTNUM, &partnum);
    cc2500_readStatusRegister(s_cc2500_ctx, CC2500_31_VERSION, &version);
    cc2500_readStatusRegister(s_cc2500_ctx, CC2500_35_MARCSTATE, &marcstate);

    snprintf(s_tx_buffer, CLI_TX_BUFFER_SIZE,
             "CC2500 Status:\r\n"
             "  Part Number: 0x%02X\r\n"
             "  Version:     0x%02X\r\n"
             "  MARC State:  0x%02X\r\n",
             partnum, version, marcstate & 0x1FU);
    cli_transmit(s_tx_buffer);
}

static void cli_handle_help(void)
{
    static const char help_msg[] =
        "Available commands:\r\n"
        "  help                  - Show this message\r\n"
        "  reboot                - Reboot the device\r\n"
        "  get_status            - Get CC2500 status registers\r\n"
        "  dump_regs             - Dump all CC2500 registers\r\n"
        "  debug <on|off>        - Enable/disable debug output\r\n"
        "  set_baud <rate>       - Set baud rate (1200, 2400, 4800, 9600, 19200,\r\n"
        "                                   38400, 57600, 125000, 250000, 500000)\r\n"
        "  set_freq <kHz>        - Set frequency in kHz (2'400'000-2'483'500)\r\n"
        "  set_mod <type>        - Set modulation (2FSK, GFSK, OOK/ASK, MSK)\r\n"
        "  set_dev <kHz>         - Set frequency deviation in kHz (up to 500)\r\n"
        "  set_power <dBm>       - Set output power (1, 0, -2, -4, -6, -8, -10, -16, -20, -30)\r\n";
    
    cli_transmit(help_msg);
}

static void cli_handle_dump_regs(void)
{
    cc2500_dumpRegisters(s_cc2500_ctx);
    cli_transmit("Registers dumped to g_cc2500_dump - check with debugger\r\n");
}

static void cli_handle_debug(const char *arg)
{
    if (strcmp(arg, "on") == 0) {
        radio_set_debug(1);
        cli_transmit("Debug mode enabled\r\n");
    } else if (strcmp(arg, "off") == 0) {
        radio_set_debug(0);
        cli_transmit("Debug mode disabled\r\n");
    } else {
        cli_transmit("Usage: debug <on|off>\r\n");
    }
}