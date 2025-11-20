#include "cli_handler.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 128

// Локальные переменные модуля
static char rx_buffer[RX_BUFFER_SIZE];
static uint32_t rx_index = 0;
static char tx_buffer[TX_BUFFER_SIZE];

static CC2500CTX* p_cc2500_ctx; // Указатель на контекст CC2500

// Прототипы локальных функций
static void process_command(char* cmd);
static void cli_transmit(const char* str);
static void handle_set_baud(uint32_t baudrate);
static void handle_get_status(void);
static void handle_set_freq(uint32_t freq_khz);
static void handle_set_mod(char* mod_str);
static void handle_set_dev(uint32_t dev_khz);
static void handle_set_power(int8_t power_dbm);
static void handle_help(void);

// Инициализация
void cli_init(CC2500CTX* cc2500_context) {
    p_cc2500_ctx = cc2500_context;
    rx_index = 0;
}

// Обработка входящих данных
void cli_process_input(uint8_t* buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) {
        if (buf[i] == '\r' || buf[i] == '\n') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0';
                process_command(rx_buffer);
                rx_index = 0;
            }
        } else {
            if (rx_index < RX_BUFFER_SIZE - 1) {
                rx_buffer[rx_index++] = buf[i];
            }
        }
    }
}

// Отправка ответа через USB
static void cli_transmit(const char* str) {
    CDC_Transmit_FS((uint8_t*)str, strlen(str));
}

// Парсинг и вызов обработчиков команд
static void process_command(char* cmd) {
    uint32_t value = 0;
    int32_t s_value = 0;
    char str_value[16] = {0};

    if (sscanf(cmd, "set_baud %lu", &value) == 1) {
        handle_set_baud(value);
    } else if (sscanf(cmd, "set_freq %lu", &value) == 1) {
        handle_set_freq(value);
    } else if (sscanf(cmd, "set_mod %15s", str_value) == 1) {
        handle_set_mod(str_value);
    } else if (sscanf(cmd, "set_dev %lu", &value) == 1) {
        handle_set_dev(value);
    } else if (sscanf(cmd, "set_power %ld", &s_value) == 1) {
        handle_set_power(s_value);
    } else if (strcmp(cmd, "get_status") == 0) {
        handle_get_status();
    } else if (strcmp(cmd, "help") == 0) {
        handle_help();
    } else if (strcmp(cmd, "reboot") == 0) {
        cli_transmit("Rebooting system...\r\n");
        HAL_Delay(100);
        NVIC_SystemReset();
    } else {
        cli_transmit("Unknown command. Type 'help' for a list of commands.\r\n");
    }
}

// --- Обработчики команд ---

static void handle_set_baud(uint32_t baudrate) {
    uint8_t mdmcfg4, mdmcfg3;
    int supported = 1;

    // Список скоростей
    switch (baudrate) {
        case 1200:   mdmcfg4 = 0xF5; mdmcfg3 = 0x83; break;
        case 2400:   mdmcfg4 = 0xF6; mdmcfg3 = 0x83; break;
        case 4800:   mdmcfg4 = 0xF7; mdmcfg3 = 0x83; break;
        case 9600:   mdmcfg4 = 0xE7; mdmcfg3 = 0x83; break;
        case 19200:  mdmcfg4 = 0xD7; mdmcfg3 = 0x83; break;
        case 38400:  mdmcfg4 = 0xDA; mdmcfg3 = 0x83; break;
        case 57600:  mdmcfg4 = 0xB9; mdmcfg3 = 0x83; break;
        case 125000: mdmcfg4 = 0x2D; mdmcfg3 = 0x55; break;
        case 250000: mdmcfg4 = 0x2D; mdmcfg3 = 0x3B; break;
        case 500000: mdmcfg4 = 0x3E; mdmcfg3 = 0x93; break;
        default:
            supported = 0;
            break;
    }

    if (supported) {
        cc2500_writeRegister(p_cc2500_ctx, CC2500_10_MDMCFG4, mdmcfg4);
        cc2500_writeRegister(p_cc2500_ctx, CC2500_11_MDMCFG3, mdmcfg3);
        snprintf(tx_buffer, TX_BUFFER_SIZE, "Baud rate set to %lu\r\n", baudrate);
        cli_transmit(tx_buffer);
    } else {
        cli_transmit("Baud rate not supported. Use one of the predefined values.\r\n");
    }
}

static void handle_set_freq(uint32_t freq_khz) {
    if (freq_khz < 2400000 || freq_khz > 2483500) {
        cli_transmit("Frequency out of range (2400000 - 2483500 kHz).\r\n");
        return;
    }

    uint64_t f_vco = (uint64_t)freq_khz * 1000;
    uint32_t freq_reg = (f_vco * 65536) / 26000000; // FXTAL = 26 MHz

    uint8_t freq2 = (freq_reg >> 16) & 0xFF;
    uint8_t freq1 = (freq_reg >> 8) & 0xFF;
    uint8_t freq0 = freq_reg & 0xFF;

    cc2500_writeRegister(p_cc2500_ctx, CC2500_0D_FREQ2, freq2);
    cc2500_writeRegister(p_cc2500_ctx, CC2500_0E_FREQ1, freq1);
    cc2500_writeRegister(p_cc2500_ctx, CC2500_0F_FREQ0, freq0);

    snprintf(tx_buffer, TX_BUFFER_SIZE, "Frequency set to %lu kHz\r\n", freq_khz);
    cli_transmit(tx_buffer);
}

static void handle_set_mod(char* mod_str) {
    uint8_t mdmcfg2_val;
    uint8_t mod_format = 0;
    int supported = 1;

    if (strcmp(mod_str, "2fsk") == 0) {
        mod_format = 0x00;
    } else if (strcmp(mod_str, "gfsk") == 0) {
        mod_format = 0x10;
    } else if (strcmp(mod_str, "ask") == 0 || strcmp(mod_str, "ook") == 0) {
        mod_format = 0x30;
    } else if (strcmp(mod_str, "msk") == 0) {
        mod_format = 0x70;
    } else {
        supported = 0;
    }

    if (supported) {
        cc2500_readRegister(p_cc2500_ctx, CC2500_12_MDMCFG2, &mdmcfg2_val);
        mdmcfg2_val &= 0x8F; // Очистить биты MOD_FORMAT [6:4]
        mdmcfg2_val |= mod_format;
        cc2500_writeRegister(p_cc2500_ctx, CC2500_12_MDMCFG2, mdmcfg2_val);
        snprintf(tx_buffer, TX_BUFFER_SIZE, "Modulation set to %s\r\n", mod_str);
        cli_transmit(tx_buffer);
    } else {
        cli_transmit("Unsupported modulation. Use 2fsk, gfsk, ask/ook, msk.\r\n");
    }
}

static void handle_set_dev(uint32_t dev_khz) {
    // Формула: Deviation = (FXTAL / 2^17) * (8 + DEVIATN_M) * 2^DEVIATN_E
    // FXTAL = 26 MHz. Для простоты используем предрассчитанные значения.
    // Это очень грубый расчет, для точных значений см. даташит или SmartRF Studio.
    if (dev_khz > 500) { // Ограничение для предотвращения некорректных значений
        cli_transmit("Deviation is too high (max 500 kHz).\r\n");
        return;
    }
    uint32_t reg_val = (dev_khz * 1000 * 131072) / 26000000;
    uint8_t deviatn = reg_val > 255 ? 255 : (uint8_t)reg_val;

    cc2500_writeRegister(p_cc2500_ctx, CC2500_15_DEVIATN, deviatn);
    snprintf(tx_buffer, TX_BUFFER_SIZE, "Deviation set to approx %lu kHz (reg: 0x%02X)\r\n", dev_khz, deviatn);
    cli_transmit(tx_buffer);
}

static void handle_set_power(int8_t power_dbm) {
    uint8_t pa_table_val;
    int supported = 1;

    switch (power_dbm) {
        case 1: pa_table_val = 0xFF; break;
        case 0: pa_table_val = 0xFE; break;
        case -2: pa_table_val = 0xC6; break;
        case -4: pa_table_val = 0x85; break;
        case -6: pa_table_val = 0x66; break;
        case -8: pa_table_val = 0x55; break;
        case -10: pa_table_val = 0x27; break;
        case -15: pa_table_val = 0x1D; break;
        case -20: pa_table_val = 0x0E; break;
        case -30: pa_table_val = 0x03; break;
        default:
            supported = 0;
            break;
    }

    if (supported) {
        cc2500_writeRegister(p_cc2500_ctx, CC2500_3E_PATABLE, pa_table_val);
        snprintf(tx_buffer, TX_BUFFER_SIZE, "Output power set to %d dBm\r\n", power_dbm);
        cli_transmit(tx_buffer);
    } else {
        cli_transmit("Unsupported power level. See datasheet for PATABLE values.\r\n");
    }
}

static void handle_get_status(void) {
    uint8_t partnum, version, marcstate;
    cc2500_readRegister(p_cc2500_ctx, CC2500_30_PARTNUM, &partnum);
    cc2500_readRegister(p_cc2500_ctx, CC2500_31_VERSION, &version);
    cc2500_readStatusRegister(p_cc2500_ctx, CC2500_35_MARCSTATE, &marcstate);

    snprintf(tx_buffer, TX_BUFFER_SIZE,
             "CC2500 Status:\r\n"
             "  Part Number: 0x%02X\r\n"
             "  Version:     0x%02X\r\n"
             "  MARC State:  0x%02X\r\n",
             partnum, version, marcstate & 0x1F);
    cli_transmit(tx_buffer);
}

static void handle_help(void) {
    const char* help_msg =
        "Available commands:\r\n"
        "  help                  - Show this message\r\n"
        "  reboot                - Reboot the device\r\n"
        "  get_status            - Get CC2500 status registers\r\n"
        "  set_baud <rate>       - Set baud rate (1200, 2400, 4800, 9600, 19200, \r\n"
        "                                   38400, 57600, 125000, 250000, 500000)\r\n"
        "  set_freq <kHz>        - Set frequency in kHz (2'400'000-2'483'500)\r\n"
        "  set_mod <type>        - Set modulation (2FSK, GFSK, OOK/ASK, MSK)\r\n"
        "  set_dev <kHz>         - Set frequency deviation in kHz (up to 500)\r\n"
        "  set_power <dBm>       - Set output power (1, 0, -2, -4, -6, -10, ... -30)\r\n";
    cli_transmit(help_msg);
}