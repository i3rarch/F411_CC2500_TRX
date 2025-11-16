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

    if (sscanf(cmd, "set_baud %lu", &value) == 1) {
        handle_set_baud(value);
    } else if (sscanf(cmd, "set_freq %lu", &value) == 1) {
        handle_set_freq(value);
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
    uint8_t mdmcfg4, mdmcfg3, deviatn;
    int supported = 1;

    switch (baudrate) {
        case 9600:
            mdmcfg4 = 0xE7; mdmcfg3 = 0x83; deviatn = 0x24; break;
        case 38400:
            mdmcfg4 = 0xDA; mdmcfg3 = 0x83; deviatn = 0x35; break;
        case 250000:
            mdmcfg4 = 0x2D; mdmcfg3 = 0x3B; deviatn = 0x62; break;
        default:
            supported = 0;
            break;
    }

    if (supported) {
        cc2500_writeRegister(p_cc2500_ctx, CC2500_10_MDMCFG4, mdmcfg4);
        cc2500_writeRegister(p_cc2500_ctx, CC2500_11_MDMCFG3, mdmcfg3);
        cc2500_writeRegister(p_cc2500_ctx, CC2500_15_DEVIATN, deviatn);
        snprintf(tx_buffer, TX_BUFFER_SIZE, "Baud rate set to %lu\r\n", baudrate);
        cli_transmit(tx_buffer);
    } else {
        cli_transmit("Baud rate not supported. Use 9600, 38400, or 250000.\r\n");
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
        "  set_baud <rate>       - Set radio baud rate (9600, 38400, 250000)\r\n"
        "  set_freq <kHz>        - Set radio frequency in kHz (e.g., 2405000)\r\n";
    cli_transmit(help_msg);
}