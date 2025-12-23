/**
 * @file    radio_handler.h
 * @brief   Radio communication handler for CC2500 TX/RX operations
 * @author  i3rarch
 * @date    2025
 */

#ifndef RADIO_HANDLER_H_
#define RADIO_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cc2500.h"

/**
 * @brief Radio operation modes
 */
typedef enum {
    RADIO_MODE_RX = 0,  /**< Receive mode */
    RADIO_MODE_TX = 1   /**< Transmit mode */
} RadioMode_t;

/**
 * @brief Radio statistics structure
 */
typedef struct {
    uint32_t tx_count;    /**< Number of transmitted packets */
    uint32_t rx_count;    /**< Number of received packets */
    uint32_t rx_errors;   /**< Number of receive errors */
} RadioStats_t;

/**
 * @brief Initialize radio handler module
 * @param ctx Pointer to CC2500 context
 * @param mode Initial operating mode (TX or RX)
 */
void radio_init(CC2500CTX *ctx, RadioMode_t mode);

/**
 * @brief Main radio processing function (call from main loop)
 */
void radio_process(void);

/**
 * @brief GDO interrupt handler (call from HAL_GPIO_EXTI_Callback)
 * @param gpio_pin GPIO pin that triggered the interrupt
 */
void radio_gdo_irq_handler(uint16_t gpio_pin);

/**
 * @brief Get pointer to radio statistics
 * @return Pointer to RadioStats_t structure
 */
RadioStats_t *radio_get_stats(void);

/**
 * @brief Get current radio mode
 * @return Current RadioMode_t value
 */
RadioMode_t radio_get_mode(void);

/**
 * @brief Enable or disable debug output
 * @param enable 1 to enable, 0 to disable
 */
void radio_set_debug(uint8_t enable);

/**
 * @brief Transmit a packet manually
 * @param data Pointer to data buffer
 * @param length Length of data (max 61 bytes)
 * @return 0 on success, negative on error
 */
int radio_transmit_packet(const uint8_t *data, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* RADIO_HANDLER_H_ */