/**
 * @file    cc2500.c
 * @brief   CC2500 2.4GHz RF Transceiver driver implementation
 * @author  opiopan (original), i3rarch (modified)
 * @date    2019/01/17, modified 2025/01/10
 */

#include "cc2500.h"
#include <string.h>

/* ============================================================================
 * Private Defines
 * ========================================================================= */
#define CC2500_CS_HIGH(ctx)     HAL_GPIO_WritePin((ctx)->selector.port, \
                                                   (ctx)->selector.pin, GPIO_PIN_SET)
#define CC2500_CS_LOW(ctx)      HAL_GPIO_WritePin((ctx)->selector.port, \
                                                   (ctx)->selector.pin, GPIO_PIN_RESET)
#define CC2500_SPI_TIMEOUT      500U
#define CC2500_STATUS_ERROR     0x80U
#define CC2500_MAX_PKT_LEN      61U

/* ============================================================================
 * Private Types
 * ========================================================================= */

/**
 * @brief Register configuration entry
 */
typedef struct {
    uint8_t addr;
    uint8_t value;
} CC2500_RegConfig_t;

/* ============================================================================
 * Private Constants
 * ========================================================================= */

/**
 * @brief Default CC2500 configuration (Packet TX/RX mode)
 * @note Variable length packets, CRC enabled, status append
 */
static const CC2500_RegConfig_t s_cc2500_default_config[] = {
    {CC2500_02_IOCFG0,    0x06},   /* GDO0 - Asserts when sync word sent/received */
    {CC2500_00_IOCFG2,    0x06},   /* GDO2 - Asserts when sync word sent/received */
    {CC2500_07_PKTCTRL1,  0x04},   /* Append status (RSSI, LQI, CRC_OK), no address check */
    {CC2500_08_PKTCTRL0,  0x05},   /* Variable length, CRC enabled, whitening OFF */
    {CC2500_06_PKTLEN,    0x3D},   /* Max packet length: 61 bytes */
    {CC2500_04_SYNC1,     0xD3},   /* Sync word high byte */
    {CC2500_05_SYNC0,     0x91},   /* Sync word low byte */
    {CC2500_09_ADDR,      0x00},   /* Device address */
    {CC2500_0A_CHANNR,    0x00},   /* Channel number */
    {CC2500_0B_FSCTRL1,   0x06},   /* IF Frequency: 152.34 kHz */
    {CC2500_0C_FSCTRL0,   0x00},   /* Frequency offset */
    {CC2500_0D_FREQ2,     0x5C},   /* Frequency: 2405 MHz high byte */
    {CC2500_0E_FREQ1,     0x80},   /* Frequency: 2405 MHz middle byte */
    {CC2500_0F_FREQ0,     0x00},   /* Frequency: 2405 MHz low byte */
    {CC2500_10_MDMCFG4,   0x78},   /* Modem config: data rate exponent, channel BW */
    {CC2500_11_MDMCFG3,   0x93},   /* Modem config: data rate mantissa */
    {CC2500_12_MDMCFG2,   0x02},   /* 2-FSK, 16/16 sync word bits */
    {CC2500_13_MDMCFG1,   0x20},   /* 4 preamble bytes */
    {CC2500_14_MDMCFG0,   0xF8},   /* Channel spacing mantissa */
    {CC2500_15_DEVIATN,   0x44},   /* Modem deviation setting */
    {CC2500_18_MCSM0,     0x18},   /* Main Radio Control State Machine config */
    {CC2500_19_FOCCFG,    0x16},   /* Frequency Offset Compensation config */
    {CC2500_1A_BSCFG,     0x6C},   /* Bit Synchronization configuration */
    {CC2500_1B_AGCCTRL2,  0x43},   /* AGC control */
    {CC2500_1C_AGCCTRL1,  0x40},   /* AGC control */
    {CC2500_1D_AGCCTRL0,  0x91},   /* AGC control */
    {CC2500_21_FREND1,    0x56},   /* Front end RX configuration */
    {CC2500_22_FREND0,    0x10},   /* Front end TX configuration */
    {CC2500_23_FSCAL3,    0xA9},   /* Frequency synthesizer calibration */
    {CC2500_24_FSCAL2,    0x0A},   /* Frequency synthesizer calibration */
    {CC2500_25_FSCAL1,    0x00},   /* Frequency synthesizer calibration */
    {CC2500_26_FSCAL0,    0x11},   /* Frequency synthesizer calibration */
    {CC2500_29_FSTEST,    0x59},   /* Frequency synthesizer test */
    {CC2500_2C_TEST2,     0x88},   /* Various test settings */
    {CC2500_2D_TEST1,     0x31},   /* Various test settings */
    {CC2500_2E_TEST0,     0x0B},   /* Various test settings */
};

/** Global register dump structure */
CC2500_RegDump g_cc2500_dump = {0};

/* ============================================================================
 * Initialization Functions
 * ========================================================================= */

void cc2500_init(CC2500CTX *ctx, GPIO_TypeDef *sel_ch, uint16_t sel_pin, 
                 SPI_HandleTypeDef *spi)
{
    if ((ctx == NULL) || (spi == NULL)) {
        return;
    }
    
    memset(ctx, 0, sizeof(CC2500CTX));
    ctx->selector.port = sel_ch;
    ctx->selector.pin = sel_pin;
    ctx->spi = spi;

    CC2500_CS_HIGH(ctx);
    HAL_Delay(1);
    cc2500_reset(ctx);
    cc2500_strobe(ctx, CC2500_SIDLE);
}

void cc2500_init_full(CC2500CTX *ctx, 
                      GPIO_TypeDef *sel_ch, uint16_t sel_pin,
                      GPIO_TypeDef *gd0_port, uint16_t gd0_pin,
                      GPIO_TypeDef *gd2_port, uint16_t gd2_pin,
                      GPIO_TypeDef *pa_en_port, uint16_t pa_en_pin,
                      GPIO_TypeDef *rx_en_port, uint16_t rx_en_pin,
                      SPI_HandleTypeDef *spi)
{
    if ((ctx == NULL) || (spi == NULL)) {
        return;
    }
    
    memset(ctx, 0, sizeof(CC2500CTX));
    
    ctx->selector.port = sel_ch;
    ctx->selector.pin = sel_pin;
    ctx->gpio.gd0_port = gd0_port;
    ctx->gpio.gd0_pin = gd0_pin;
    ctx->gpio.gd2_port = gd2_port;
    ctx->gpio.gd2_pin = gd2_pin;
    ctx->rf_ctrl.pa_en_port = pa_en_port;
    ctx->rf_ctrl.pa_en_pin = pa_en_pin;
    ctx->rf_ctrl.rx_en_port = rx_en_port;
    ctx->rf_ctrl.rx_en_pin = rx_en_pin;
    ctx->spi = spi;

    /* Initialize control pins */
    cc2500_setPAEnabled(ctx, 0);
    cc2500_setRxEnabled(ctx, 0);
    
    CC2500_CS_HIGH(ctx);
    HAL_Delay(1);
    cc2500_reset(ctx);
    cc2500_strobe(ctx, CC2500_SIDLE);
    
    /* Apply default configuration */
    cc2500_configure(ctx);
}

/* ============================================================================
 * Register Operations
 * ========================================================================= */

int cc2500_writeRegister(CC2500CTX *ctx, uint8_t addr, uint8_t value)
{
    uint8_t rc = CC2500_STATUS_ERROR;
    uint8_t cmd = addr | CC2500_WRITE_SINGLE;
    
    CC2500_CS_LOW(ctx);
    
    if (HAL_SPI_TransmitReceive(ctx->spi, &cmd, &rc, 1, CC2500_SPI_TIMEOUT) != HAL_OK) {
        CC2500_CS_HIGH(ctx);
        return CC2500_STATUS_ERROR;
    }
    
    if ((rc & CC2500_STATUS_CHIP_RDYn_BM) != 0U) {
        CC2500_CS_HIGH(ctx);
        return rc;
    }
    
    if (HAL_SPI_Transmit(ctx->spi, &value, 1, CC2500_SPI_TIMEOUT) != HAL_OK) {
        CC2500_CS_HIGH(ctx);
        return CC2500_STATUS_ERROR;
    }
    
    CC2500_CS_HIGH(ctx);
    return rc;
}

int cc2500_readRegister(CC2500CTX *ctx, uint8_t addr, uint8_t *value)
{
    uint8_t rc = CC2500_STATUS_ERROR;
    uint8_t cmd = CC2500_READ_SINGLE | addr;
    uint8_t dummy = 0;
    
    CC2500_CS_LOW(ctx);
    
    if (HAL_SPI_TransmitReceive(ctx->spi, &cmd, &rc, 1, CC2500_SPI_TIMEOUT) != HAL_OK) {
        CC2500_CS_HIGH(ctx);
        return CC2500_STATUS_ERROR;
    }
    
    if ((rc & CC2500_STATUS_CHIP_RDYn_BM) != 0U) {
        CC2500_CS_HIGH(ctx);
        return rc;
    }
    
    if (HAL_SPI_TransmitReceive(ctx->spi, &dummy, value, 1, CC2500_SPI_TIMEOUT) != HAL_OK) {
        CC2500_CS_HIGH(ctx);
        return CC2500_STATUS_ERROR;
    }
    
    CC2500_CS_HIGH(ctx);
    return rc;
}

int cc2500_writeRegisterBurst(CC2500CTX *ctx, uint8_t addr, const uint8_t *values, int len)
{
    uint8_t rc = CC2500_STATUS_ERROR;
    uint8_t cmd = addr | CC2500_WRITE_BURST;
    
    CC2500_CS_LOW(ctx);
    
    if (HAL_SPI_TransmitReceive(ctx->spi, &cmd, &rc, 1, CC2500_SPI_TIMEOUT) != HAL_OK) {
        CC2500_CS_HIGH(ctx);
        return CC2500_STATUS_ERROR;
    }
    
    if ((rc & CC2500_STATUS_CHIP_RDYn_BM) != 0U) {
        CC2500_CS_HIGH(ctx);
        return rc;
    }
    
    for (int i = 0; i < len; i++) {
        if (HAL_SPI_Transmit(ctx->spi, (uint8_t *)&values[i], 1, CC2500_SPI_TIMEOUT) != HAL_OK) {
            CC2500_CS_HIGH(ctx);
            return CC2500_STATUS_ERROR;
        }
    }
    
    CC2500_CS_HIGH(ctx);
    return rc;
}

int cc2500_readRegisterBurst(CC2500CTX *ctx, uint8_t addr, uint8_t *values, int len)
{
    uint8_t rc = CC2500_STATUS_ERROR;
    uint8_t cmd = addr | CC2500_READ_BURST;
    uint8_t dummy = 0;
    
    CC2500_CS_LOW(ctx);
    
    if (HAL_SPI_TransmitReceive(ctx->spi, &cmd, &rc, 1, CC2500_SPI_TIMEOUT) != HAL_OK) {
        CC2500_CS_HIGH(ctx);
        return CC2500_STATUS_ERROR;
    }
    
    if ((rc & CC2500_STATUS_CHIP_RDYn_BM) != 0U) {
        CC2500_CS_HIGH(ctx);
        return rc;
    }
    
    for (int i = 0; i < len; i++) {
        if (HAL_SPI_TransmitReceive(ctx->spi, &dummy, &values[i], 1, CC2500_SPI_TIMEOUT) != HAL_OK) {
            CC2500_CS_HIGH(ctx);
            return CC2500_STATUS_ERROR;
        }
    }
    
    CC2500_CS_HIGH(ctx);
    return rc;
}

/* ============================================================================
 * Command and State Functions
 * ========================================================================= */

int cc2500_reset(CC2500CTX *ctx)
{
    uint8_t rc = CC2500_STATUS_ERROR;
    uint8_t cmd, status;
    
    CC2500_CS_LOW(ctx);
    
    cmd = CC2500_SRES;
    if (HAL_SPI_TransmitReceive(ctx->spi, &cmd, &rc, 1, CC2500_SPI_TIMEOUT) != HAL_OK) {
        CC2500_CS_HIGH(ctx);
        return CC2500_STATUS_ERROR;
    }
    
    /* Wait for chip to be ready */
    cmd = CC2500_SNOP;
    do {
        if (HAL_SPI_TransmitReceive(ctx->spi, &cmd, &status, 1, CC2500_SPI_TIMEOUT) != HAL_OK) {
            CC2500_CS_HIGH(ctx);
            return CC2500_STATUS_ERROR;
        }
    } while (status == 0xFFU);
    
    CC2500_CS_HIGH(ctx);
    return rc;
}

int cc2500_strobe(CC2500CTX *ctx, uint8_t state)
{
    uint8_t rc = CC2500_STATUS_ERROR;
    uint8_t cmd = state | CC2500_READ_SINGLE;
    
    CC2500_CS_LOW(ctx);
    
    if (HAL_SPI_TransmitReceive(ctx->spi, &cmd, &rc, 1, CC2500_SPI_TIMEOUT) != HAL_OK) {
        CC2500_CS_HIGH(ctx);
        return CC2500_STATUS_ERROR;
    }
    
    CC2500_CS_HIGH(ctx);
    return rc;
}

int cc2500_strobeR(CC2500CTX *ctx, uint8_t state)
{
    /* Same as cc2500_strobe - kept for API compatibility */
    return cc2500_strobe(ctx, state);
}

int cc2500_readFIFO(CC2500CTX *ctx, uint8_t *buf, int length)
{
    uint8_t rc = CC2500_STATUS_ERROR;
    uint8_t cmd = CC2500_3F_RXFIFO | CC2500_READ_BURST;
    uint8_t dummy = 0;
    
    CC2500_CS_LOW(ctx);
    
    if (HAL_SPI_TransmitReceive(ctx->spi, &cmd, &rc, 1, CC2500_SPI_TIMEOUT) != HAL_OK) {
        CC2500_CS_HIGH(ctx);
        return CC2500_STATUS_ERROR;
    }
    
    if (((rc & CC2500_STATUS_CHIP_RDYn_BM) != 0U) || 
        ((rc & CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM) < (uint8_t)length)) {
        CC2500_CS_HIGH(ctx);
        return rc | CC2500_STATUS_CHIP_RDYn_BM;
    }

    for (int i = 0; i < length; i++) {
        if (HAL_SPI_TransmitReceive(ctx->spi, &dummy, &buf[i], 1, CC2500_SPI_TIMEOUT) != HAL_OK) {
            CC2500_CS_HIGH(ctx);
            return CC2500_STATUS_ERROR;
        }
    }

    CC2500_CS_HIGH(ctx);
    return rc;
}

int cc2500_waitForState(CC2500CTX *ctx, uint8_t state)
{
    uint8_t cmd = CC2500_SNOP;
    uint8_t status;
    
    CC2500_CS_LOW(ctx);
    
    do {
        if (HAL_SPI_TransmitReceive(ctx->spi, &cmd, &status, 1, CC2500_SPI_TIMEOUT) != HAL_OK) {
            CC2500_CS_HIGH(ctx);
            return CC2500_STATUS_ERROR;
        }
    } while ((status & CC2500_STATUS_STATE_BM) != state);
    
    CC2500_CS_HIGH(ctx);
    return status;
}

int cc2500_configure(CC2500CTX *ctx)
{
    size_t config_count = sizeof(s_cc2500_default_config) / sizeof(s_cc2500_default_config[0]);
    
    for (size_t i = 0; i < config_count; i++) {
        int result = cc2500_writeRegister(ctx, s_cc2500_default_config[i].addr,
                                          s_cc2500_default_config[i].value);
        if ((result & CC2500_STATUS_ERROR) != 0) {
            return -1;
        }
    }
    return 0;
}

/* ============================================================================
 * Mode Control Functions
 * ========================================================================= */

void cc2500_setTxMode(CC2500CTX *ctx)
{
    cc2500_setPAEnabled(ctx, 1);
    cc2500_setRxEnabled(ctx, 0);
    cc2500_strobe(ctx, CC2500_STX);
}

void cc2500_setRxMode(CC2500CTX *ctx)
{
    cc2500_setPAEnabled(ctx, 0);
    cc2500_setRxEnabled(ctx, 1);
    cc2500_strobe(ctx, CC2500_SRX);
}

void cc2500_setIdleMode(CC2500CTX *ctx)
{
    cc2500_setPAEnabled(ctx, 0);
    cc2500_setRxEnabled(ctx, 0);
    cc2500_strobe(ctx, CC2500_SIDLE);
}

/* ============================================================================
 * GPIO Functions
 * ========================================================================= */

uint8_t cc2500_readGD0(CC2500CTX *ctx)
{
    if (ctx->gpio.gd0_port != NULL) {
        return (uint8_t)HAL_GPIO_ReadPin(ctx->gpio.gd0_port, ctx->gpio.gd0_pin);
    }
    return 0;
}

uint8_t cc2500_readGD2(CC2500CTX *ctx)
{
    if (ctx->gpio.gd2_port != NULL) {
        return (uint8_t)HAL_GPIO_ReadPin(ctx->gpio.gd2_port, ctx->gpio.gd2_pin);
    }
    return 0;
}

void cc2500_setPAEnabled(CC2500CTX *ctx, uint8_t enable)
{
    if (ctx->rf_ctrl.pa_en_port != NULL) {
        HAL_GPIO_WritePin(ctx->rf_ctrl.pa_en_port, ctx->rf_ctrl.pa_en_pin, 
                          (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void cc2500_setRxEnabled(CC2500CTX *ctx, uint8_t enable)
{
    if (ctx->rf_ctrl.rx_en_port != NULL) {
        HAL_GPIO_WritePin(ctx->rf_ctrl.rx_en_port, ctx->rf_ctrl.rx_en_pin, 
                          (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/* ============================================================================
 * Data Transfer Functions
 * ========================================================================= */

int cc2500_transmit(CC2500CTX *ctx, const uint8_t *data, uint8_t length)
{
    if ((data == NULL) || (length > CC2500_MAX_PKT_LEN)) {
        return -1;
    }
    
    /* Flush TX FIFO */
    cc2500_strobe(ctx, CC2500_SFTX);

    /* In variable length mode (PKTCTRL0=0x05), first byte is length */
    uint8_t tx_buffer[65];
    tx_buffer[0] = length;
    memcpy(&tx_buffer[1], data, length);
    
    /* Write to FIFO: length byte + data */
    cc2500_writeRegisterBurst(ctx, CC2500_3F_TXFIFO, tx_buffer, length + 1);

    /* Enter transmit mode */
    cc2500_setTxMode(ctx);

    /* Wait for transmission to complete */
    cc2500_waitForState(ctx, CC2500_STATE_IDLE);

    return 0;
}

int cc2500_receive(CC2500CTX *ctx, uint8_t *data, uint8_t *length)
{
    if ((data == NULL) || (length == NULL)) {
        return -1;
    }
    
    uint8_t rxbytes;

    /* Enter receive mode */
    cc2500_setRxMode(ctx);

    /* Check for data in FIFO */
    cc2500_readRegister(ctx, CC2500_3B_RXBYTES, &rxbytes);

    if ((rxbytes & 0x7FU) != 0U) {
        /* Read packet length */
        cc2500_readRegister(ctx, CC2500_3F_RXFIFO, length);

        if (*length <= 64U) {
            /* Read data */
            cc2500_readRegisterBurst(ctx, CC2500_3F_RXFIFO, data, *length);

            /* Flush RX FIFO */
            cc2500_strobe(ctx, CC2500_SFRX);

            return 0;
        }
    }

    return -1;
}

/* ============================================================================
 * Status and Debug Functions
 * ========================================================================= */

int8_t cc2500_getRSSI(CC2500CTX *ctx)
{
    uint8_t rssi_raw;
    cc2500_readStatusRegister(ctx, CC2500_34_RSSI, &rssi_raw);

    /* Convert to dBm according to CC2500 datasheet
     * RSSI_dBm = (RSSI_dec / 2) - 74 */
    int16_t rssi_dbm = ((int16_t)rssi_raw / 2) - 74;

    return (int8_t)rssi_dbm;
}

uint8_t cc2500_getLQI(CC2500CTX *ctx, uint8_t *crc_ok)
{
    uint8_t lqi_raw;
    cc2500_readStatusRegister(ctx, CC2500_33_LQI, &lqi_raw);

    /* Bit 7 is CRC_OK */
    if (crc_ok != NULL) {
        *crc_ok = ((lqi_raw & CC2500_LQI_CRC_OK_BM) != 0U) ? 1U : 0U;
    }

    /* Bits [6:0] contain LQI estimate */
    return lqi_raw & CC2500_LQI_EST_BM;
}

uint8_t cc2500_getState(CC2500CTX *ctx)
{
    uint8_t marcstate;
    cc2500_readStatusRegister(ctx, CC2500_35_MARCSTATE, &marcstate);
    return marcstate;
}

void cc2500_dumpRegisters(CC2500CTX *ctx)
{
    /* Configuration registers */
    cc2500_readRegister(ctx, CC2500_00_IOCFG2, &g_cc2500_dump.iocfg2);
    cc2500_readRegister(ctx, CC2500_02_IOCFG0, &g_cc2500_dump.iocfg0);
    cc2500_readRegister(ctx, CC2500_06_PKTLEN, &g_cc2500_dump.pktlen);
    cc2500_readRegister(ctx, CC2500_07_PKTCTRL1, &g_cc2500_dump.pktctrl1);
    cc2500_readRegister(ctx, CC2500_08_PKTCTRL0, &g_cc2500_dump.pktctrl0);
    cc2500_readRegister(ctx, CC2500_04_SYNC1, &g_cc2500_dump.sync1);
    cc2500_readRegister(ctx, CC2500_05_SYNC0, &g_cc2500_dump.sync0);
    cc2500_readRegister(ctx, CC2500_0D_FREQ2, &g_cc2500_dump.freq2);
    cc2500_readRegister(ctx, CC2500_0E_FREQ1, &g_cc2500_dump.freq1);
    cc2500_readRegister(ctx, CC2500_0F_FREQ0, &g_cc2500_dump.freq0);
    cc2500_readRegister(ctx, CC2500_10_MDMCFG4, &g_cc2500_dump.mdmcfg4);
    cc2500_readRegister(ctx, CC2500_11_MDMCFG3, &g_cc2500_dump.mdmcfg3);
    cc2500_readRegister(ctx, CC2500_12_MDMCFG2, &g_cc2500_dump.mdmcfg2);
    cc2500_readRegister(ctx, CC2500_13_MDMCFG1, &g_cc2500_dump.mdmcfg1);
    cc2500_readRegister(ctx, CC2500_14_MDMCFG0, &g_cc2500_dump.mdmcfg0);
    cc2500_readRegister(ctx, CC2500_15_DEVIATN, &g_cc2500_dump.deviatn);

    /* Status registers (use burst read for correct access) */
    cc2500_readStatusRegister(ctx, CC2500_35_MARCSTATE, &g_cc2500_dump.marcstate);
    cc2500_readStatusRegister(ctx, CC2500_38_PKTSTATUS, &g_cc2500_dump.pktstatus);
    cc2500_readStatusRegister(ctx, CC2500_3B_RXBYTES, &g_cc2500_dump.rxbytes);
    cc2500_readStatusRegister(ctx, CC2500_3A_TXBYTES, &g_cc2500_dump.txbytes);
    cc2500_readStatusRegister(ctx, CC2500_34_RSSI, &g_cc2500_dump.rssi);
    cc2500_readStatusRegister(ctx, CC2500_33_LQI, &g_cc2500_dump.lqi);
}
