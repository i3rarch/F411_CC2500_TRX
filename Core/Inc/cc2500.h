/**
 * @file    cc2500.h
 * @brief   CC2500 2.4GHz RF Transceiver driver
 * @author  opiopan (original), i3rarch (modified)
 * @date    2019/01/17, modified 2025/01/10
 */

#ifndef CC2500_H_
#define CC2500_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "project.h"
#include "iface_cc2500.h"

/**
 * @brief CC2500 context structure containing all GPIO and SPI configuration
 */
typedef struct {
    struct {
        GPIO_TypeDef *port;     /**< Chip Select GPIO port */
        uint16_t pin;           /**< Chip Select GPIO pin */
    } selector;
    
    struct {
        GPIO_TypeDef *gd0_port; /**< GDO0 GPIO port */
        uint16_t gd0_pin;       /**< GDO0 GPIO pin */
        GPIO_TypeDef *gd2_port; /**< GDO2 GPIO port */
        uint16_t gd2_pin;       /**< GDO2 GPIO pin */
    } gpio;
    
    struct {
        GPIO_TypeDef *pa_en_port;   /**< PA Enable GPIO port */
        uint16_t pa_en_pin;         /**< PA Enable GPIO pin */
        GPIO_TypeDef *rx_en_port;   /**< RX Enable GPIO port */
        uint16_t rx_en_pin;         /**< RX Enable GPIO pin */
    } rf_ctrl;
    
    SPI_HandleTypeDef *spi;     /**< SPI handle pointer */
} CC2500CTX;

/**
 * @brief CC2500 register dump structure for debugging
 */
typedef struct {
    uint8_t iocfg2, iocfg0;
    uint8_t pktlen, pktctrl1, pktctrl0;
    uint8_t sync1, sync0;
    uint8_t freq2, freq1, freq0;
    uint8_t mdmcfg4, mdmcfg3, mdmcfg2;
    uint8_t mdmcfg1, mdmcfg0, deviatn;
    uint8_t marcstate, pktstatus;
    uint8_t rxbytes, txbytes;
    uint8_t rssi, lqi;
} CC2500_RegDump;

/** Global register dump structure (use debugger to view) */
extern CC2500_RegDump g_cc2500_dump;

/* ============================================================================
 * Initialization Functions
 * ========================================================================= */

/**
 * @brief Basic CC2500 initialization
 * @param ctx     Pointer to CC2500 context
 * @param cs_port Chip Select GPIO port
 * @param sel_pin Chip Select GPIO pin
 * @param spi     SPI handle pointer
 */
void cc2500_init(CC2500CTX *ctx, GPIO_TypeDef *cs_port, uint16_t sel_pin, 
                 SPI_HandleTypeDef *spi);

/**
 * @brief Full CC2500 initialization with all GPIO pins
 * @param ctx        Pointer to CC2500 context
 * @param cs_port    Chip Select GPIO port
 * @param sel_pin    Chip Select GPIO pin
 * @param gd0_port   GDO0 GPIO port
 * @param gd0_pin    GDO0 GPIO pin
 * @param gd2_port   GDO2 GPIO port
 * @param gd2_pin    GDO2 GPIO pin
 * @param pa_en_port PA Enable GPIO port
 * @param pa_en_pin  PA Enable GPIO pin
 * @param rx_en_port RX Enable GPIO port
 * @param rx_en_pin  RX Enable GPIO pin
 * @param spi        SPI handle pointer
 */
void cc2500_init_full(CC2500CTX *ctx, 
                      GPIO_TypeDef *cs_port, uint16_t sel_pin,
                      GPIO_TypeDef *gd0_port, uint16_t gd0_pin,
                      GPIO_TypeDef *gd2_port, uint16_t gd2_pin,
                      GPIO_TypeDef *pa_en_port, uint16_t pa_en_pin,
                      GPIO_TypeDef *rx_en_port, uint16_t rx_en_pin,
                      SPI_HandleTypeDef *spi);

/* ============================================================================
 * Register Operations
 * ========================================================================= */

/**
 * @brief Write single register
 * @param ctx   Pointer to CC2500 context
 * @param addr  Register address
 * @param value Value to write
 * @return Status byte
 */
int cc2500_writeRegister(CC2500CTX *ctx, uint8_t addr, uint8_t value);

/**
 * @brief Read single register
 * @param ctx   Pointer to CC2500 context
 * @param addr  Register address
 * @param value Pointer to store read value
 * @return Status byte
 */
int cc2500_readRegister(CC2500CTX *ctx, uint8_t addr, uint8_t *value);

/**
 * @brief Write multiple registers (burst mode)
 * @param ctx    Pointer to CC2500 context
 * @param addr   Starting register address
 * @param values Pointer to values array
 * @param len    Number of bytes to write
 * @return Status byte
 */
int cc2500_writeRegisterBurst(CC2500CTX *ctx, uint8_t addr, 
                               const uint8_t *values, int len);

/**
 * @brief Read multiple registers (burst mode)
 * @param ctx    Pointer to CC2500 context
 * @param addr   Starting register address
 * @param values Pointer to buffer for read values
 * @param len    Number of bytes to read
 * @return Status byte
 */
int cc2500_readRegisterBurst(CC2500CTX *ctx, uint8_t addr, 
                              uint8_t *values, int len);

/* ============================================================================
 * Command and State Functions
 * ========================================================================= */

/**
 * @brief Reset CC2500 chip
 * @param ctx Pointer to CC2500 context
 * @return Status byte
 */
int cc2500_reset(CC2500CTX *ctx);

/**
 * @brief Send strobe command
 * @param ctx   Pointer to CC2500 context
 * @param state Strobe command
 * @return Status byte
 */
int cc2500_strobe(CC2500CTX *ctx, uint8_t state);

/**
 * @brief Send strobe command with read
 * @param ctx   Pointer to CC2500 context
 * @param state Strobe command
 * @return Status byte
 */
int cc2500_strobeR(CC2500CTX *ctx, uint8_t state);

/**
 * @brief Read from RX FIFO
 * @param ctx    Pointer to CC2500 context
 * @param buf    Buffer to store data
 * @param length Number of bytes to read
 * @return Status byte
 */
int cc2500_readFIFO(CC2500CTX *ctx, uint8_t *buf, int length);

/**
 * @brief Wait for specific state
 * @param ctx   Pointer to CC2500 context
 * @param state Target state to wait for
 * @return Status byte
 */
int cc2500_waitForState(CC2500CTX *ctx, uint8_t state);

/**
 * @brief Apply default configuration
 * @param ctx Pointer to CC2500 context
 * @return 0 on success, -1 on error
 */
int cc2500_configure(CC2500CTX *ctx);

/* ============================================================================
 * Mode Control Functions
 * ========================================================================= */

/**
 * @brief Set transmit mode
 * @param ctx Pointer to CC2500 context
 */
void cc2500_setTxMode(CC2500CTX *ctx);

/**
 * @brief Set receive mode
 * @param ctx Pointer to CC2500 context
 */
void cc2500_setRxMode(CC2500CTX *ctx);

/**
 * @brief Set idle mode
 * @param ctx Pointer to CC2500 context
 */
void cc2500_setIdleMode(CC2500CTX *ctx);

/* ============================================================================
 * GPIO Functions
 * ========================================================================= */

/**
 * @brief Read GDO0 pin state
 * @param ctx Pointer to CC2500 context
 * @return Pin state (0 or 1)
 */
uint8_t cc2500_readGD0(CC2500CTX *ctx);

/**
 * @brief Read GDO2 pin state
 * @param ctx Pointer to CC2500 context
 * @return Pin state (0 or 1)
 */
uint8_t cc2500_readGD2(CC2500CTX *ctx);

/**
 * @brief Enable/disable Power Amplifier
 * @param ctx    Pointer to CC2500 context
 * @param enable 1 to enable, 0 to disable
 */
void cc2500_setPAEnabled(CC2500CTX *ctx, uint8_t enable);

/**
 * @brief Enable/disable RX amplifier
 * @param ctx    Pointer to CC2500 context
 * @param enable 1 to enable, 0 to disable
 */
void cc2500_setRxEnabled(CC2500CTX *ctx, uint8_t enable);

/* ============================================================================
 * Data Transfer Functions
 * ========================================================================= */

/**
 * @brief Transmit packet
 * @param ctx    Pointer to CC2500 context
 * @param data   Pointer to data buffer
 * @param length Data length (max 61 bytes for variable length mode)
 * @return 0 on success
 */
int cc2500_transmit(CC2500CTX *ctx, const uint8_t *data, uint8_t length);

/**
 * @brief Receive packet
 * @param ctx    Pointer to CC2500 context
 * @param data   Pointer to buffer for received data
 * @param length Pointer to store received data length
 * @return 0 on success, -1 if no data
 */
int cc2500_receive(CC2500CTX *ctx, uint8_t *data, uint8_t *length);

/* ============================================================================
 * Status and Debug Functions
 * ========================================================================= */

/**
 * @brief Get RSSI value in dBm
 * @param ctx Pointer to CC2500 context
 * @return RSSI in dBm
 */
int8_t cc2500_getRSSI(CC2500CTX *ctx);

/**
 * @brief Get LQI value
 * @param ctx    Pointer to CC2500 context
 * @param crc_ok Pointer to store CRC status (optional, can be NULL)
 * @return LQI value (0-127)
 */
uint8_t cc2500_getLQI(CC2500CTX *ctx, uint8_t *crc_ok);

/**
 * @brief Get current MARC state
 * @param ctx Pointer to CC2500 context
 * @return MARCSTATE register value
 */
uint8_t cc2500_getState(CC2500CTX *ctx);

/**
 * @brief Dump all registers to g_cc2500_dump structure
 * @param ctx Pointer to CC2500 context
 */
void cc2500_dumpRegisters(CC2500CTX *ctx);

/**
 * @brief Read status register macro
 * @note Uses burst read for correct status register access
 */
#define cc2500_readStatusRegister(ctx, addr, value) \
    cc2500_readRegisterBurst(ctx, addr, value, 1)

#ifdef __cplusplus
}
#endif

#endif /* CC2500_H_ */
