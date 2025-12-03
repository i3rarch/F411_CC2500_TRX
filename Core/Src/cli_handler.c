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
static void cli_handle_set_sync(uint32_t sync_word);
static void cli_handle_set_syncmode(uint8_t mode);
static void cli_handle_set_bw(uint32_t bw_khz);
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

    /* set_baud / sb */
    if (sscanf(cmd, "set_baud %lu", &value) == 1 || sscanf(cmd, "sb %lu", &value) == 1) {
        cli_handle_set_baud(value);
    /* set_freq / sf */
    } else if (sscanf(cmd, "set_freq %lu", &value) == 1 || sscanf(cmd, "sf %lu", &value) == 1) {
        cli_handle_set_freq(value);
    /* set_mod / sm */
    } else if (sscanf(cmd, "set_mod %15s", str_value) == 1 || sscanf(cmd, "sm %15s", str_value) == 1) {
        cli_handle_set_mod(str_value);
    /* set_dev / sd */
    } else if (sscanf(cmd, "set_dev %lu", &value) == 1 || sscanf(cmd, "sd %lu", &value) == 1) {
        cli_handle_set_dev(value);
    /* set_power / sp */
    } else if (sscanf(cmd, "set_power %ld", &s_value) == 1 || sscanf(cmd, "sp %ld", &s_value) == 1) {
        cli_handle_set_power((int8_t)s_value);
    /* set_sync / ss - sync word in hex (e.g. ss D391 or ss D391D391) */
    } else if (sscanf(cmd, "set_sync %lx", &value) == 1 || sscanf(cmd, "ss %lx", &value) == 1) {
        cli_handle_set_sync(value);
    /* set_syncmode / ssm - sync mode (0-7) */
    } else if (sscanf(cmd, "set_syncmode %lu", &value) == 1 || sscanf(cmd, "ssm %lu", &value) == 1) {
        cli_handle_set_syncmode((uint8_t)value);
    /* set_bw / sbw - RX filter bandwidth in kHz */
    } else if (sscanf(cmd, "set_bw %lu", &value) == 1 || sscanf(cmd, "sbw %lu", &value) == 1) {
        cli_handle_set_bw(value);
    /* get_status / gs */
    } else if (strcmp(cmd, "get_status") == 0 || strcmp(cmd, "gs") == 0) {
        cli_handle_get_status();
    /* help / h */
    } else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0) {
        cli_handle_help();
    /* dump_regs / dr */
    } else if (strcmp(cmd, "dump_regs") == 0 || strcmp(cmd, "dr") == 0) {
        cli_handle_dump_regs();
    /* debug */
    } else if (sscanf(cmd, "debug %15s", str_value) == 1 || sscanf(cmd, "dbg %15s", str_value) == 1) {
        cli_handle_debug(str_value);
    /* reboot / rb */
    } else if (strcmp(cmd, "reboot") == 0 || strcmp(cmd, "rb") == 0) {
        cli_transmit("Rebooting system...\r\n");
        HAL_Delay(100);
        NVIC_SystemReset();
    } else {
        cli_transmit("Unknown command. Type 'help' or 'h' for a list.\r\n");
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
        "Commands (short | full):\r\n"
        "  h   | help            - Show this message\r\n"
        "  rb  | reboot          - Reboot the device\r\n"
        "  gs  | get_status      - Get CC2500 status registers\r\n"
        "  dr  | dump_regs       - Dump all CC2500 registers\r\n"
        "  dbg | debug <on|off>  - Enable/disable debug output\r\n"
        "  sb  | set_baud <rate> - Set baud rate (1200-500000)\r\n"
        "  sf  | set_freq <kHz>  - Set frequency (2400000-2483500)\r\n"
        "  sm  | set_mod <type>  - Set modulation (2fsk,gfsk,ook,msk)\r\n"
        "  sd  | set_dev <kHz>   - Set deviation (up to 500)\r\n"
        "  sp  | set_power <dBm> - Set power (1,0,-2,-4,-6,-8,-10,-16,-20,-30)\r\n"
        "  ss  | set_sync <hex>  - Set sync word (e.g. ss D391)\r\n"
        "  ssm | set_syncmode <0-7> - Set sync mode:\r\n"
        "        0=No sync, 1=15/16, 2=16/16, 3=30/32\r\n"
        "        4-7=same + carrier sense\r\n"
        "  sbw | set_bw <kHz>    - Set RX filter BW (58-812)\r\n";
    
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

static void cli_handle_set_sync(uint32_t sync_word)
{
    /* Sync word can be 16-bit (0xXXXX) or 32-bit (0xXXXXXXXX) */
    uint8_t sync1, sync0;
    
    if (sync_word > 0xFFFFU) {
        /* 32-bit sync word - use upper 16 bits for SYNC1/SYNC0 */
        /* Note: For 30/32 bit mode, CC2500 uses SYNC1+SYNC0 twice */
        sync1 = (uint8_t)((sync_word >> 24) & 0xFFU);
        sync0 = (uint8_t)((sync_word >> 16) & 0xFFU);
        /* Store lower 16 bits as well if needed - CC2500 sends SYNC1+SYNC0 twice for 32-bit */
        snprintf(s_tx_buffer, CLI_TX_BUFFER_SIZE, 
                 "Sync word set to 0x%08lX (32-bit mode)\r\n", sync_word);
    } else {
        /* 16-bit sync word */
        sync1 = (uint8_t)((sync_word >> 8) & 0xFFU);
        sync0 = (uint8_t)(sync_word & 0xFFU);
        snprintf(s_tx_buffer, CLI_TX_BUFFER_SIZE, 
                 "Sync word set to 0x%04lX (16-bit mode)\r\n", sync_word);
    }
    
    cc2500_writeRegister(s_cc2500_ctx, CC2500_04_SYNC1, sync1);
    cc2500_writeRegister(s_cc2500_ctx, CC2500_05_SYNC0, sync0);
    
    cli_transmit(s_tx_buffer);
}

static void cli_handle_set_syncmode(uint8_t mode)
{
    if (mode > 7U) {
        cli_transmit("Sync mode must be 0-7\r\n");
        return;
    }
    
    uint8_t mdmcfg2;
    cc2500_readRegister(s_cc2500_ctx, CC2500_12_MDMCFG2, &mdmcfg2);
    
    /* Clear SYNC_MODE bits [2:0] and set new value */
    mdmcfg2 = (mdmcfg2 & 0xF8U) | mode;
    cc2500_writeRegister(s_cc2500_ctx, CC2500_12_MDMCFG2, mdmcfg2);
    
    static const char *mode_names[] = {
        "No preamble/sync",
        "15/16 sync bits",
        "16/16 sync bits",
        "30/32 sync bits",
        "No sync + carrier sense",
        "15/16 + carrier sense",
        "16/16 + carrier sense",
        "30/32 + carrier sense"
    };
    
    snprintf(s_tx_buffer, CLI_TX_BUFFER_SIZE, 
             "Sync mode set to %d: %s\r\n", mode, mode_names[mode]);
    cli_transmit(s_tx_buffer);
}

static void cli_handle_set_bw(uint32_t bw_khz)
{
    /* CC2500 RX filter bandwidth is set via MDMCFG4[7:4]
     * BW = f_xosc / (8 * (4 + CHANBW_M) * 2^CHANBW_E)
     * f_xosc = 26 MHz
     * 
     * Available bandwidths (kHz):
     * CHANBW_E | CHANBW_M=0 | M=1  | M=2  | M=3
     *    0     |    812     | 650  | 541  | 464
     *    1     |    406     | 325  | 270  | 232
     *    2     |    203     | 162  | 135  | 116
     *    3     |    101     |  81  |  67  |  58
     */
    
    /* Lookup table: {bandwidth_kHz, CHANBW_E, CHANBW_M} */
    static const struct {
        uint16_t bw;
        uint8_t chanbw_e;
        uint8_t chanbw_m;
    } bw_table[] = {
        {812, 0, 0}, {650, 0, 1}, {541, 0, 2}, {464, 0, 3},
        {406, 1, 0}, {325, 1, 1}, {270, 1, 2}, {232, 1, 3},
        {203, 2, 0}, {162, 2, 1}, {135, 2, 2}, {116, 2, 3},
        {101, 3, 0}, {81,  3, 1}, {67,  3, 2}, {58,  3, 3}
    };
    
    /* Find closest bandwidth */
    uint8_t best_idx = 0;
    uint32_t min_diff = 0xFFFFFFFFU;
    
    for (uint8_t i = 0; i < 16U; i++) {
        uint32_t diff = (bw_khz > bw_table[i].bw) 
                        ? (bw_khz - bw_table[i].bw) 
                        : (bw_table[i].bw - bw_khz);
        if (diff < min_diff) {
            min_diff = diff;
            best_idx = i;
        }
    }
    
    /* Read current MDMCFG4 to preserve data rate exponent */
    uint8_t mdmcfg4;
    cc2500_readRegister(s_cc2500_ctx, CC2500_10_MDMCFG4, &mdmcfg4);
    
    /* Update CHANBW bits [7:4] */
    mdmcfg4 = (mdmcfg4 & 0x0FU) | 
              ((bw_table[best_idx].chanbw_e << 6) | (bw_table[best_idx].chanbw_m << 4));
    
    cc2500_writeRegister(s_cc2500_ctx, CC2500_10_MDMCFG4, mdmcfg4);
    
    snprintf(s_tx_buffer, CLI_TX_BUFFER_SIZE, 
             "RX filter BW set to %u kHz (requested %lu)\r\n", 
             bw_table[best_idx].bw, bw_khz);
    cli_transmit(s_tx_buffer);
}