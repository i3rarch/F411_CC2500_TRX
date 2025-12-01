/**
 * @file    radio_handler.c
 * @brief   Radio communication handler implementation
 * @author  i3rarch
 * @date    2025
 */

#include "radio_handler.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ============================================================================
 * Private Defines
 * ========================================================================= */
#define RADIO_RX_BUFFER_SIZE    64U
#define RADIO_USB_BUFFER_SIZE   400U
#define RADIO_MAX_PACKET_LEN    61U
#define RADIO_TX_INTERVAL_MS    500U

/* ============================================================================
 * Private Variables
 * ========================================================================= */
static CC2500CTX *s_ctx = NULL;
static RadioMode_t s_current_mode = RADIO_MODE_RX;
static RadioStats_t s_stats = {0};
static volatile uint8_t s_rx_packet_pending = 0;
static uint8_t s_debug_enabled = 1;
static uint8_t s_tx_counter = 0;

static uint8_t s_rx_buffer[RADIO_RX_BUFFER_SIZE];
static char s_usb_buffer[RADIO_USB_BUFFER_SIZE];

/* ============================================================================
 * Private Function Prototypes
 * ========================================================================= */
static void radio_process_rx_packet(void);
static void radio_process_tx_cycle(void);
static void radio_send_debug(const char *fmt, ...);

/* ============================================================================
 * Public Functions
 * ========================================================================= */

void radio_init(CC2500CTX *ctx, RadioMode_t mode)
{
    if (ctx == NULL) {
        return;
    }
    
    s_ctx = ctx;
    s_current_mode = mode;
    
    memset(&s_stats, 0, sizeof(s_stats));
    s_rx_packet_pending = 0;
    s_tx_counter = 0;
    
    if (mode == RADIO_MODE_RX) {
        /* Enable GDO pin interrupts for RX mode */
        HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);
        HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(EXTI1_IRQn);
        
        cc2500_setRxMode(s_ctx);
        
        const char *msg = "Radio: RX Mode initialized\r\n";
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
    } else {
        const char *msg = "Radio: TX Mode initialized\r\n";
        CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
    }
}

void radio_process(void)
{
    if (s_ctx == NULL) {
        return;
    }
    
    if (s_current_mode == RADIO_MODE_TX) {
        radio_process_tx_cycle();
    } else {
        if (s_rx_packet_pending != 0U) {
            s_rx_packet_pending = 0;
            radio_process_rx_packet();
        }
    }
}

void radio_gdo_irq_handler(uint16_t gpio_pin)
{
    if (s_current_mode == RADIO_MODE_RX) {
        if ((gpio_pin == GD00_Pin) || (gpio_pin == GD02_Pin)) {
            s_rx_packet_pending = 1;
        }
    }
}

RadioStats_t *radio_get_stats(void)
{
    return &s_stats;
}

RadioMode_t radio_get_mode(void)
{
    return s_current_mode;
}

void radio_set_debug(uint8_t enable)
{
    s_debug_enabled = enable;
}

/* ============================================================================
 * Private Functions
 * ========================================================================= */

static void radio_send_debug(const char *fmt, ...)
{
    if (s_debug_enabled == 0U) {
        return;
    }
    
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(s_usb_buffer, sizeof(s_usb_buffer), fmt, args);
    va_end(args);
    
    if (len > 0) {
        CDC_Transmit_FS((uint8_t *)s_usb_buffer, (uint16_t)len);
    }
}

static void radio_process_rx_packet(void)
{
    uint8_t rxbytes;
    uint8_t marcstate;
    uint8_t pktstatus;
    
    marcstate = cc2500_getState(s_ctx);
    cc2500_readStatusRegister(s_ctx, CC2500_3B_RXBYTES, &rxbytes);
    cc2500_readStatusRegister(s_ctx, CC2500_38_PKTSTATUS, &pktstatus);
    UNUSED(pktstatus);
    
    uint8_t fifo_bytes = rxbytes & 0x7FU;
    
    if (fifo_bytes < 1U) {
        radio_send_debug("DEBUG: Spurious IRQ, RXBYTES=0x%02X, STATE=0x%02X\r\n", 
                         rxbytes, marcstate);
        s_stats.rx_errors++;
        cc2500_strobe(s_ctx, CC2500_SFRX);
        cc2500_setRxMode(s_ctx);
        return;
    }
    
    /* Read length byte */
    uint8_t pkt_length;
    cc2500_readRegister(s_ctx, CC2500_3F_RXFIFO, &pkt_length);
    
    /* Validate packet */
    if ((pkt_length > RADIO_MAX_PACKET_LEN) || (fifo_bytes < (pkt_length + 2U))) {
        radio_send_debug("DEBUG: Invalid packet, LEN=%u, RXBYTES=0x%02X\r\n",
                         pkt_length, rxbytes);
        s_stats.rx_errors++;
        cc2500_strobe(s_ctx, CC2500_SFRX);
        cc2500_setRxMode(s_ctx);
        return;
    }
    
    /* Read packet data */
    uint8_t rx_length = pkt_length;
    cc2500_readRegisterBurst(s_ctx, CC2500_3F_RXFIFO, s_rx_buffer, rx_length);
    
    /* Read status bytes (RSSI + LQI|CRC_OK) */
    uint8_t status_bytes[2];
    cc2500_readRegisterBurst(s_ctx, CC2500_3F_RXFIFO, status_bytes, 2);
    
    /* Parse status */
    int8_t rssi_raw = (int8_t)status_bytes[0];
    int8_t rssi_dbm = (int8_t)((rssi_raw / 2) - 74);
    uint8_t lqi = status_bytes[1] & 0x7FU;
    uint8_t crc_ok = ((status_bytes[1] & 0x80U) != 0U) ? 1U : 0U;
    
    /* Toggle LED */
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    s_stats.rx_count++;
    
    /* Format output */
    int pos = 0;
    pos += snprintf(s_usb_buffer + pos, sizeof(s_usb_buffer) - (size_t)pos,
                    "[%lu] RX: ", HAL_GetTick());
    
    /* HEX data */
    for (int i = 0; (i < rx_length) && (pos < 300); i++) {
        pos += snprintf(s_usb_buffer + pos, sizeof(s_usb_buffer) - (size_t)pos,
                        "%02X ", s_rx_buffer[i]);
    }
    
    /* ASCII representation */
    pos += snprintf(s_usb_buffer + pos, sizeof(s_usb_buffer) - (size_t)pos, "| ");
    for (int i = 0; (i < rx_length) && (pos < 350); i++) {
        char c = ((s_rx_buffer[i] >= 32U) && (s_rx_buffer[i] <= 126U)) 
                 ? (char)s_rx_buffer[i] : '.';
        s_usb_buffer[pos++] = c;
    }
    
    /* Status info */
    pos += snprintf(s_usb_buffer + pos, sizeof(s_usb_buffer) - (size_t)pos,
                    " | RSSI:%d LQI:%u CRC:%s #%lu",
                    rssi_dbm, lqi, (crc_ok != 0U) ? "OK" : "FAIL", s_stats.rx_count);
    
    if (s_debug_enabled != 0U) {
        pos += snprintf(s_usb_buffer + pos, sizeof(s_usb_buffer) - (size_t)pos,
                        "\r\n  [DBG] LEN=%u RXBYTES=0x%02X STATE=0x%02X",
                        pkt_length, rxbytes, marcstate);
    }
    
    pos += snprintf(s_usb_buffer + pos, sizeof(s_usb_buffer) - (size_t)pos, "\r\n");
    CDC_Transmit_FS((uint8_t *)s_usb_buffer, (uint16_t)pos);
    
    /* Flush and return to RX mode */
    cc2500_strobe(s_ctx, CC2500_SFRX);
    cc2500_setRxMode(s_ctx);
}

static void radio_process_tx_cycle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    
    /* Build packet */
    uint8_t tx_data[8];
    int len = snprintf((char *)tx_data, sizeof(tx_data), "PING%d", s_tx_counter++);
    
    /* Fill remainder with zeros */
    for (int i = len; i < 8; i++) {
        tx_data[i] = 0;
    }
    
    cc2500_transmit(s_ctx, tx_data, 8);
    s_stats.tx_count++;
    
    /* Periodic status output */
    if ((s_stats.tx_count % 10U) == 0U) {
        int msg_len = snprintf(s_usb_buffer, sizeof(s_usb_buffer),
                               "TX: %lu packets sent\r\n", s_stats.tx_count);
        CDC_Transmit_FS((uint8_t *)s_usb_buffer, (uint16_t)msg_len);
    }
    
    HAL_Delay(RADIO_TX_INTERVAL_MS);
}