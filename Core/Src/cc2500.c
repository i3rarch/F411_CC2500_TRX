/*
 * cc2500.c
 *
 *  Created on: 2019/01/17
 *      Author: opiopan
 *  Modified on: 2025/01/10
 *      Author: i3rarch
 */

#include "cc2500.h"

#define CS_UP(ctx) HAL_GPIO_WritePin((ctx)->selector.port, (ctx)->selector.pin, GPIO_PIN_SET)
#define CS_DOWN(ctx) HAL_GPIO_WritePin((ctx)->selector.port, (ctx)->selector.pin, GPIO_PIN_RESET)
#define SPITIMEOUT 500

// Упрощенная конфигурация для CC2500
// 2-FSK модуляция, без CRC, минимальное синхрослово
static const uint8_t cc2500_config[][2] = {
    {CC2500_02_IOCFG0,    0x06},   // GDO0 - Packet received/transmitted
    {CC2500_00_IOCFG2,    0x06},   // GDO2 - Packet received/transmitted
    {CC2500_07_PKTCTRL1,  0x00},   // Packet control: CRC autoflush OFF, append status OFF
    {CC2500_08_PKTCTRL0,  0x00},   // Packet control: Fixed length, CRC OFF, whitening OFF
    {CC2500_06_PKTLEN,    0x08},   // Packet length: 8 bytes
    {CC2500_04_SYNC1,     0xD3},   // Sync word high byte
    {CC2500_05_SYNC0,     0x91},   // Sync word low byte
    {CC2500_09_ADDR,      0x00},   // Device address
    {CC2500_0A_CHANNR,    0x00},   // Channel number
    {CC2500_0B_FSCTRL1,   0x0A},   // Frequency synthesizer control
    {CC2500_0C_FSCTRL0,   0x00},   // Frequency synthesizer control
    {CC2500_0D_FREQ2,     0x5C},   // Frequency: 2405 MHz high byte
    {CC2500_0E_FREQ1,     0x7F},   // Frequency: 2405 MHz middle byte
    {CC2500_0F_FREQ0,     0xFA},   // Frequency: 2405 MHz low byte
    {CC2500_10_MDMCFG4,   0xE7},   // Modem configuration: data rate ~9.6 kBaud
    {CC2500_11_MDMCFG3,   0x83},   // Modem configuration
    {CC2500_12_MDMCFG2,   0x03},   // 2-FSK, 16/16 sync word bits
    {CC2500_13_MDMCFG1,   0x22},   // Modem configuration
    {CC2500_14_MDMCFG0,   0xF8},   // Modem configuration
    {CC2500_15_DEVIATN,   0x24},   // Modem deviation setting
    {CC2500_17_MCSM1,     0x0C},   // Main Radio Cntrl State Machine config
    {CC2500_18_MCSM0,     0x18},   // Main Radio Cntrl State Machine config
    {CC2500_19_FOCCFG,    0x1D},   // Frequency Offset Compensation config
    {CC2500_1A_BSCFG,     0x1C},   // Bit Synchronization configuration
    {CC2500_1B_AGCCTRL2,  0xC7},   // AGC control
    {CC2500_1C_AGCCTRL1,  0x00},   // AGC control
    {CC2500_1D_AGCCTRL0,  0xB2},   // AGC control
    {CC2500_21_FREND1,    0xB6},   // Front end RX configuration
    {CC2500_22_FREND0,    0x10},   // Front end TX configuration
    {CC2500_23_FSCAL3,    0xEA},   // Frequency synthesizer calibration
    {CC2500_24_FSCAL2,    0x0A},   // Frequency synthesizer calibration
    {CC2500_25_FSCAL1,    0x00},   // Frequency synthesizer calibration
    {CC2500_26_FSCAL0,    0x11},   // Frequency synthesizer calibration
    {CC2500_2C_TEST2,     0x88},   // Various test settings
    {CC2500_2D_TEST1,     0x31},   // Various test settings
    {CC2500_2E_TEST0,     0x0B},   // Various test settings
    {0xFF, 0xFF}  // End marker
};

/*---------------------------------------------------------------
 * initialize context - базовая инициализация
 *-------------------------------------------------------------*/
void cc2500_init(CC2500CTX* ctx, GPIO_TypeDef* sel_ch, uint16_t sel_pin, SPI_HandleTypeDef* spi)
{
    *ctx = (CC2500CTX){
        .selector = {
            .port = sel_ch,
            .pin = sel_pin
        },
        .spi = spi
    };

    CS_UP(ctx);
    HAL_Delay(1);
    cc2500_reset(ctx);
    cc2500_strobe(ctx, CC2500_SIDLE);
}

/*---------------------------------------------------------------
 * initialize context - полная инициализация со всеми GPIO
 *-------------------------------------------------------------*/
void cc2500_init_full(CC2500CTX* ctx, 
                     GPIO_TypeDef* sel_ch, uint16_t sel_pin,
                     GPIO_TypeDef* gd0_port, uint16_t gd0_pin,
                     GPIO_TypeDef* gd2_port, uint16_t gd2_pin,
                     GPIO_TypeDef* pa_en_port, uint16_t pa_en_pin,
                     GPIO_TypeDef* rx_en_port, uint16_t rx_en_pin,
                     SPI_HandleTypeDef* spi)
{
    *ctx = (CC2500CTX){
        .selector = {
            .port = sel_ch,
            .pin = sel_pin
        },
        .gpio = {
            .gd0_port = gd0_port,
            .gd0_pin = gd0_pin,
            .gd2_port = gd2_port,
            .gd2_pin = gd2_pin
        },
        .rf_ctrl = {
            .pa_en_port = pa_en_port,
            .pa_en_pin = pa_en_pin,
            .rx_en_port = rx_en_port,
            .rx_en_pin = rx_en_pin
        },
        .spi = spi
    };

    // Инициализация контрольных пинов
    cc2500_setPAEnabled(ctx, 0);
    cc2500_setRxEnabled(ctx, 0);
    
    CS_UP(ctx);
    HAL_Delay(1);
    cc2500_reset(ctx);
    cc2500_strobe(ctx, CC2500_SIDLE);
    
    // Конфигурация регистров
    cc2500_configure(ctx);
}

/*---------------------------------------------------------------
 * basic communication - существующие функции остаются без изменений
 *-------------------------------------------------------------*/
int cc2500_writeRegister(CC2500CTX* ctx, uint8_t addr, uint8_t value)
{
    uint8_t rc = 0x80;
    addr |= CC2500_WRITE_SINGLE;
    CS_DOWN(ctx);
    if (HAL_SPI_TransmitReceive(ctx->spi, &addr, &rc, 1, SPITIMEOUT) != HAL_OK){
        CS_UP(ctx);
        return 0x80;
    }
    if (rc & CC2500_STATUS_CHIP_RDYn_BM){
        CS_UP(ctx);
        return rc;
    }
    if (HAL_SPI_Transmit(ctx->spi, &value, 1, SPITIMEOUT) != HAL_OK){
        CS_UP(ctx);
        return 0x80;
    }
    CS_UP(ctx);
    return rc;
}

int cc2500_readRegister(CC2500CTX* ctx, uint8_t addr, uint8_t* value)
{
    uint8_t rc = 0x80;
    uint8_t maddr = CC2500_READ_SINGLE | addr;
    CS_DOWN(ctx);
    if (HAL_SPI_TransmitReceive(ctx->spi, &maddr, &rc, 1, SPITIMEOUT) != HAL_OK){
        CS_UP(ctx);
        return 0x80;
    }
    if (rc & CC2500_STATUS_CHIP_RDYn_BM){
        CS_UP(ctx);
        return rc;
    }
    uint8_t dummy = 0;
    if (HAL_SPI_TransmitReceive(ctx->spi, &dummy, value, 1, SPITIMEOUT) != HAL_OK){
        CS_UP(ctx);
        return 0x80;
    }
    CS_UP(ctx);
    return rc;
}

int cc2500_writeRegisterBurst(CC2500CTX* ctx, uint8_t addr, const uint8_t* values, int len)
{
    uint8_t rc = 0x80;
    addr |= CC2500_WRITE_BURST;
    CS_DOWN(ctx);
    if (HAL_SPI_TransmitReceive(ctx->spi, &addr, &rc, 1, SPITIMEOUT) != HAL_OK){
        CS_UP(ctx);
        return 0x80;
    }
    if (rc & CC2500_STATUS_CHIP_RDYn_BM){
        CS_UP(ctx);
        return rc;
    }
    for (int i = 0; i < len; i++){
        if (HAL_SPI_Transmit(ctx->spi, (uint8_t*)values + i, 1, SPITIMEOUT) != HAL_OK){
            CS_UP(ctx);
            return 0x80;
        }
    }
    CS_UP(ctx);
    return rc;
}

int cc2500_readRegisterBurst(CC2500CTX* ctx, uint8_t addr, uint8_t* values, int len)
{
    uint8_t rc = 0x80;
    addr |= CC2500_READ_BURST;
    CS_DOWN(ctx);
    if (HAL_SPI_TransmitReceive(ctx->spi, &addr, &rc, 1, SPITIMEOUT) != HAL_OK){
        CS_UP(ctx);
        return 0x80;
    }
    if (rc & CC2500_STATUS_CHIP_RDYn_BM){
        CS_UP(ctx);
        return rc;
    }
    uint8_t dummy = 0;
    for (int i = 0; i < len; i++){
        if (HAL_SPI_TransmitReceive(ctx->spi, &dummy, values + i, 1, SPITIMEOUT) != HAL_OK){
            CS_UP(ctx);
            return 0x80;
        }
    }
    CS_UP(ctx);
    return rc;
}

/*---------------------------------------------------------------
 * high level communication
 *-------------------------------------------------------------*/
int cc2500_reset(CC2500CTX* ctx)
{
    uint8_t rc = 0x80;
    uint8_t sdata, rdata;
    CS_DOWN(ctx);
    sdata = CC2500_SRES;
    if (HAL_SPI_TransmitReceive(ctx->spi, &sdata, &rc, 1, SPITIMEOUT) != HAL_OK){
        CS_UP(ctx);
        return 0x80;
    }
    sdata = CC2500_SNOP;
    do {
        if (HAL_SPI_TransmitReceive(ctx->spi, &sdata, &rdata, 1, SPITIMEOUT) != HAL_OK){
            CS_UP(ctx);
            return 0x80;
        }
    }while (rdata == 0xff);
    CS_UP(ctx);
    return rc;
}

int cc2500_strobe(CC2500CTX* ctx, uint8_t state)
{
    uint8_t rc = 0x80;
    state |= CC2500_READ_SINGLE;
    CS_DOWN(ctx);
    if (HAL_SPI_TransmitReceive(ctx->spi, &state, &rc, 1, SPITIMEOUT) != HAL_OK){
        CS_UP(ctx);
        return 0x80;
    }
    CS_UP(ctx);
    return rc;
}

int cc2500_strobeR(CC2500CTX* ctx, uint8_t state)
{
    uint8_t rc = 0x80;
    state |= CC2500_READ_SINGLE;
    CS_DOWN(ctx);
    if (HAL_SPI_TransmitReceive(ctx->spi, &state, &rc, 1, SPITIMEOUT) != HAL_OK){
        CS_UP(ctx);
        return 0x80;
    }
    CS_UP(ctx);
    return rc;
}

int cc2500_readFIFO(CC2500CTX* ctx, uint8_t* buf, int length)
{
    uint8_t rc = 0x80;
    uint8_t cmd = CC2500_3F_RXFIFO | CC2500_READ_BURST;
    CS_DOWN(ctx);
    if (HAL_SPI_TransmitReceive(ctx->spi, &cmd, &rc, 1, SPITIMEOUT) != HAL_OK){
        CS_UP(ctx);
        return 0x80;
    }
    if ((rc & CC2500_STATUS_CHIP_RDYn_BM) || CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM < length){
        CS_UP(ctx);
        return rc | CC2500_STATUS_CHIP_RDYn_BM;
    }

    cmd = 0;
    for (int i = 0; i < length; i++){
        if (HAL_SPI_TransmitReceive(ctx->spi, &cmd, buf + i, 1, SPITIMEOUT) != HAL_OK){
            CS_UP(ctx);
            return 0x80;
        }
    }

    CS_UP(ctx);
    return rc;
}

int cc2500_waitForState(CC2500CTX* ctx, uint8_t state)
{
    CS_DOWN(ctx);
    uint8_t sdata, rdata;
    sdata = CC2500_SNOP;
    do {
        if (HAL_SPI_TransmitReceive(ctx->spi, &sdata, &rdata, 1, SPITIMEOUT) != HAL_OK){
            CS_UP(ctx);
            return 0x80;
        }
    }while ((rdata & CC2500_STATUS_STATE_BM) != state);
    CS_UP(ctx);
    return rdata;
}

int cc2500_configure(CC2500CTX* ctx)
{
    int i = 0;
    while (cc2500_config[i][0] != 0xFF) {
        if (cc2500_writeRegister(ctx, cc2500_config[i][0], cc2500_config[i][1]) & 0x80) {
            return -1;
        }
        i++;
    }
    return 0;
}

// Установка режима передачи
void cc2500_setTxMode(CC2500CTX* ctx)
{
    cc2500_setPAEnabled(ctx, 1);
    cc2500_setRxEnabled(ctx, 0);
    cc2500_strobe(ctx, CC2500_STX);
}

// Установка режима приема
void cc2500_setRxMode(CC2500CTX* ctx)
{
    cc2500_setPAEnabled(ctx, 0);
    cc2500_setRxEnabled(ctx, 1);
    cc2500_strobe(ctx, CC2500_SRX);
}

// Установка режима ожидания
void cc2500_setIdleMode(CC2500CTX* ctx)
{
    cc2500_setPAEnabled(ctx, 0);
    cc2500_setRxEnabled(ctx, 0);
    cc2500_strobe(ctx, CC2500_SIDLE);
}

// GPIO функции
uint8_t cc2500_readGD0(CC2500CTX* ctx)
{
    if (ctx->gpio.gd0_port) {
        return HAL_GPIO_ReadPin(ctx->gpio.gd0_port, ctx->gpio.gd0_pin);
    }
    return 0;
}

uint8_t cc2500_readGD2(CC2500CTX* ctx)
{
    if (ctx->gpio.gd2_port) {
        return HAL_GPIO_ReadPin(ctx->gpio.gd2_port, ctx->gpio.gd2_pin);
    }
    return 0;
}

void cc2500_setPAEnabled(CC2500CTX* ctx, uint8_t enable)
{
    if (ctx->rf_ctrl.pa_en_port) {
        HAL_GPIO_WritePin(ctx->rf_ctrl.pa_en_port, ctx->rf_ctrl.pa_en_pin, 
                         enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void cc2500_setRxEnabled(CC2500CTX* ctx, uint8_t enable)
{
    if (ctx->rf_ctrl.rx_en_port) {
        HAL_GPIO_WritePin(ctx->rf_ctrl.rx_en_port, ctx->rf_ctrl.rx_en_pin, 
                         enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

// Передача данных
int cc2500_transmit(CC2500CTX* ctx, const uint8_t* data, uint8_t length)
{
    // Очистка TX FIFO
    cc2500_strobe(ctx, CC2500_SFTX);
    
    // Запись длины пакета
    cc2500_writeRegister(ctx, CC2500_3F_TXFIFO, length);
    
    // Запись данных в FIFO
    cc2500_writeRegisterBurst(ctx, CC2500_3F_TXFIFO, data, length);
    
    // Переход в режим передачи
    cc2500_setTxMode(ctx);
    
    // Ожидание завершения передачи
    cc2500_waitForState(ctx, CC2500_STATE_IDLE);
    
    return 0;
}

// Прием данных
int cc2500_receive(CC2500CTX* ctx, uint8_t* data, uint8_t* length)
{
    uint8_t rxbytes;

    // Переход в режим приема
    cc2500_setRxMode(ctx);

    // Проверка наличия данных в FIFO
    cc2500_readRegister(ctx, CC2500_3B_RXBYTES, &rxbytes);

    if (rxbytes & 0x7F) {  // Есть данные в FIFO
        // Чтение длины пакета
        cc2500_readRegister(ctx, CC2500_3F_RXFIFO, length);

        if (*length <= 64) {  // Проверка корректности длины
            // Чтение данных
            cc2500_readRegisterBurst(ctx, CC2500_3F_RXFIFO, data, *length);

            // Очистка RX FIFO
            cc2500_strobe(ctx, CC2500_SFRX);

            return 0;
        }
    }

    return -1;
}

// Получение RSSI (Received Signal Strength Indicator)
int8_t cc2500_getRSSI(CC2500CTX* ctx)
{
    uint8_t rssi_raw;
    cc2500_readRegister(ctx, CC2500_34_RSSI, &rssi_raw);

    // Конвертация в dBm согласно datasheet CC2500
    // RSSI_dBm = (RSSI_dec / 2) - 74
    int16_t rssi_dbm = ((int16_t)rssi_raw / 2) - 74;

    return (int8_t)rssi_dbm;
}

// Получение LQI (Link Quality Indicator)
uint8_t cc2500_getLQI(CC2500CTX* ctx, uint8_t* crc_ok)
{
    uint8_t lqi_raw;
    cc2500_readRegister(ctx, CC2500_33_LQI, &lqi_raw);

    // Бит 7 - CRC_OK
    if (crc_ok) {
        *crc_ok = (lqi_raw & CC2500_LQI_CRC_OK_BM) ? 1 : 0;
    }

    // Биты [6:0] - LQI estimate
    return lqi_raw & CC2500_LQI_EST_BM;
}

// Получение текущего состояния CC2500
uint8_t cc2500_getState(CC2500CTX* ctx)
{
    uint8_t marcstate;
    cc2500_readRegister(ctx, CC2500_35_MARCSTATE, &marcstate);
    return marcstate;
}

// Дамп всех регистров CC2500 через USB CDC
void cc2500_dumpRegisters(CC2500CTX* ctx)
{
    extern int CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
    char buffer[128];
    uint8_t value;

    // Конфигурационные регистры
    const uint8_t config_regs[] = {
        CC2500_00_IOCFG2, CC2500_02_IOCFG0,
        CC2500_06_PKTLEN, CC2500_07_PKTCTRL1, CC2500_08_PKTCTRL0,
        CC2500_04_SYNC1, CC2500_05_SYNC0,
        CC2500_0D_FREQ2, CC2500_0E_FREQ1, CC2500_0F_FREQ0,
        CC2500_10_MDMCFG4, CC2500_11_MDMCFG3, CC2500_12_MDMCFG2,
        CC2500_13_MDMCFG1, CC2500_14_MDMCFG0, CC2500_15_DEVIATN
    };

    const char* config_names[] = {
        "IOCFG2", "IOCFG0",
        "PKTLEN", "PKTCTRL1", "PKTCTRL0",
        "SYNC1", "SYNC0",
        "FREQ2", "FREQ1", "FREQ0",
        "MDMCFG4", "MDMCFG3", "MDMCFG2",
        "MDMCFG1", "MDMCFG0", "DEVIATN"
    };

    CDC_Transmit_FS((uint8_t*)"\r\n=== CC2500 Configuration Registers ===\r\n", 42);

    for (int i = 0; i < sizeof(config_regs); i++) {
        cc2500_readRegister(ctx, config_regs[i], &value);
        int len = sprintf(buffer, "%s (0x%02X): 0x%02X\r\n", config_names[i], config_regs[i], value);
        CDC_Transmit_FS((uint8_t*)buffer, len);
    }

    // Статусные регистры
    CDC_Transmit_FS((uint8_t*)"\r\n=== Status Registers ===\r\n", 29);

    cc2500_readRegister(ctx, CC2500_35_MARCSTATE, &value);
    sprintf(buffer, "MARCSTATE: 0x%02X\r\n", value);
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));

    cc2500_readRegister(ctx, CC2500_38_PKTSTATUS, &value);
    sprintf(buffer, "PKTSTATUS: 0x%02X\r\n", value);
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));

    cc2500_readRegister(ctx, CC2500_3B_RXBYTES, &value);
    sprintf(buffer, "RXBYTES: 0x%02X (%d bytes in FIFO)\r\n", value, value & 0x7F);
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));

    cc2500_readRegister(ctx, CC2500_3A_TXBYTES, &value);
    sprintf(buffer, "TXBYTES: 0x%02X\r\n", value);
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));

    cc2500_readRegister(ctx, CC2500_34_RSSI, &value);
    sprintf(buffer, "RSSI: 0x%02X (%d dBm)\r\n", value, ((int16_t)value / 2) - 74);
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));

    cc2500_readRegister(ctx, CC2500_33_LQI, &value);
    sprintf(buffer, "LQI: 0x%02X\r\n\r\n", value);
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
}
