/**
 * @file    cli_handler.h
 * @brief   Command Line Interface handler for USB CDC communication
 * @author  i3rarch
 * @date    2025
 */

#ifndef CLI_HANDLER_H_
#define CLI_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cc2500.h"

/**
 * @brief Initialize CLI handler module
 * @param cc2500_context Pointer to CC2500 context structure
 */
void cli_init(CC2500CTX *cc2500_context);

/**
 * @brief Process incoming data from USB CDC
 * @param buf Pointer to received data buffer
 * @param len Length of received data
 */
void cli_process_input(uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* CLI_HANDLER_H_ */