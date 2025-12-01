#include "radio_handler.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

// Приватные переменные модуля
static CC2500CTX* p_ctx = NULL;
static RadioMode_t current_mode = RADIO_MODE_RX;
static RadioStats_t stats = {0};
static volatile uint8_t rx_packet_pending = 0;
static uint8_t debug_enabled = 1;
static uint8_t tx_counter = 0;

// Буферы
static uint8_t rx_buffer[64];
static char usb_buffer[400];

// Приватные функции
static void process_rx_packet(void);
static void process_tx_cycle(void);
static void send_debug_msg(const char* fmt, ...);

// ============================================================================
// Публичные функции
// ============================================================================

void radio_init(CC2500CTX* ctx, RadioMode_t mode)
{
    p_ctx = ctx;
    current_mode = mode;
    
    memset(&stats, 0, sizeof(stats));
    rx_packet_pending = 0;
    tx_counter = 0;
    
    if (mode == RADIO_MODE_RX) {
        // Включение прерываний для GDO пинов
        HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);
        HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(EXTI1_IRQn);
        
        cc2500_setRxMode(p_ctx);
        
        const char* msg = "Radio: RX Mode initialized\r\n";
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    } else {
        const char* msg = "Radio: TX Mode initialized\r\n";
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    }
}

void radio_process(void)
{
    if (p_ctx == NULL) return;
    
    if (current_mode == RADIO_MODE_TX) {
        process_tx_cycle();
    } else {
        if (rx_packet_pending) {
            rx_packet_pending = 0;
            process_rx_packet();
        }
    }
}

void radio_gdo_irq_handler(uint16_t gpio_pin)
{
    if (current_mode == RADIO_MODE_RX) {
        if (gpio_pin == GD00_Pin || gpio_pin == GD02_Pin) {
            rx_packet_pending = 1;
        }
    }
}

RadioStats_t* radio_get_stats(void)
{
    return &stats;
}

RadioMode_t radio_get_mode(void)
{
    return current_mode;
}

void radio_set_debug(uint8_t enable)
{
    debug_enabled = enable;
}

// ============================================================================
// Приватные функции
// ============================================================================

static void send_debug_msg(const char* fmt, ...)
{
    if (!debug_enabled) return;
    
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(usb_buffer, sizeof(usb_buffer), fmt, args);
    va_end(args);
    
    if (len > 0) {
        CDC_Transmit_FS((uint8_t*)usb_buffer, len);
    }
}

static void process_rx_packet(void)
{
    uint8_t rxbytes;
    uint8_t marcstate;
    uint8_t pktstatus;
    
    marcstate = cc2500_getState(p_ctx);
    cc2500_readStatusRegister(p_ctx, CC2500_3B_RXBYTES, &rxbytes);
    cc2500_readStatusRegister(p_ctx, CC2500_38_PKTSTATUS, &pktstatus);
    
    uint8_t fifo_bytes = rxbytes & 0x7F;
    
    if (fifo_bytes < 1) {
        send_debug_msg("DEBUG: Spurious IRQ, RXBYTES=0x%02X, STATE=0x%02X\r\n", 
                       rxbytes, marcstate);
        stats.rx_errors++;
        cc2500_strobe(p_ctx, CC2500_SFRX);
        cc2500_setRxMode(p_ctx);
        return;
    }
    
    // Читаем байт длины
    uint8_t pkt_length;
    cc2500_readRegister(p_ctx, CC2500_3F_RXFIFO, &pkt_length);
    
    // Проверка валидности
    if (pkt_length > 61 || fifo_bytes < (pkt_length + 2)) {
        send_debug_msg("DEBUG: Invalid packet, LEN=%u, RXBYTES=0x%02X\r\n",
                       pkt_length, rxbytes);
        stats.rx_errors++;
        cc2500_strobe(p_ctx, CC2500_SFRX);
        cc2500_setRxMode(p_ctx);
        return;
    }
    
    // Читаем данные пакета
    uint8_t rx_length = pkt_length;
    cc2500_readRegisterBurst(p_ctx, CC2500_3F_RXFIFO, rx_buffer, rx_length);
    
    // Читаем статусные байты (RSSI + LQI|CRC_OK)
    uint8_t status_bytes[2];
    cc2500_readRegisterBurst(p_ctx, CC2500_3F_RXFIFO, status_bytes, 2);
    
    // Парсинг статуса
    int8_t rssi_raw = (int8_t)status_bytes[0];
    int8_t rssi_dbm = (rssi_raw / 2) - 74;
    uint8_t lqi = status_bytes[1] & 0x7F;
    uint8_t crc_ok = (status_bytes[1] & 0x80) ? 1 : 0;
    
    // Мигаем LED
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    stats.rx_count++;
    
    // Формируем вывод
    int pos = 0;
    pos += sprintf(usb_buffer + pos, "[%lu] RX: ", HAL_GetTick());
    
    // HEX данные
    for (int i = 0; i < rx_length && pos < 300; i++) {
        pos += sprintf(usb_buffer + pos, "%02X ", rx_buffer[i]);
    }
    
    // ASCII
    pos += sprintf(usb_buffer + pos, "| ");
    for (int i = 0; i < rx_length && pos < 350; i++) {
        char c = (rx_buffer[i] >= 32 && rx_buffer[i] <= 126) ? rx_buffer[i] : '.';
        usb_buffer[pos++] = c;
    }
    
    // Статус
    pos += sprintf(usb_buffer + pos, " | RSSI:%d LQI:%u CRC:%s #%lu",
                   rssi_dbm, lqi, crc_ok ? "OK" : "FAIL", stats.rx_count);
    
    if (debug_enabled) {
        pos += sprintf(usb_buffer + pos, "\r\n  [DBG] LEN=%u RXBYTES=0x%02X STATE=0x%02X",
                       pkt_length, rxbytes, marcstate);
    }
    
    pos += sprintf(usb_buffer + pos, "\r\n");
    CDC_Transmit_FS((uint8_t*)usb_buffer, pos);
    
    // Очистка и возврат в RX
    cc2500_strobe(p_ctx, CC2500_SFRX);
    cc2500_setRxMode(p_ctx);
}

static void process_tx_cycle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    
    // Формируем пакет
    uint8_t tx_data[8];
    int len = sprintf((char*)tx_data, "PING%d", tx_counter++);
    
    // Заполняем остаток нулями
    for (int i = len; i < 8; i++) {
        tx_data[i] = 0;
    }
    
    cc2500_transmit(p_ctx, tx_data, 8);
    stats.tx_count++;
    
    // Периодический вывод статистики
    if (stats.tx_count % 10 == 0) {
        int msg_len = sprintf(usb_buffer, "TX: %lu packets sent\r\n", stats.tx_count);
        CDC_Transmit_FS((uint8_t*)usb_buffer, msg_len);
    }
    
    HAL_Delay(500);
}