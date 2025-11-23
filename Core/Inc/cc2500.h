/*
 * cc2500.h
 *
 *  Created on: 2019/01/17
 *      Author: opiopan
 *  Modified on: 2025/01/10
 *      Author: i3rarch
 */

#ifndef CC2500_H_
#define CC2500_H_

#include "project.h"
#include "iface_cc2500.h"

typedef struct {
    struct {
        GPIO_TypeDef* port;
        uint16_t pin;
    } selector;
    struct {
        GPIO_TypeDef* gd0_port;
        uint16_t gd0_pin;
        GPIO_TypeDef* gd2_port;
        uint16_t gd2_pin;
    } gpio;
    struct {
        GPIO_TypeDef* pa_en_port;
        uint16_t pa_en_pin;
        GPIO_TypeDef* rx_en_port;
        uint16_t rx_en_pin;
    } rf_ctrl;
    SPI_HandleTypeDef* spi;
} CC2500CTX;

// Основные функции
void cc2500_init(CC2500CTX* ctx, GPIO_TypeDef* cs_port, uint16_t sel_pin, SPI_HandleTypeDef* spi);
void cc2500_init_full(CC2500CTX* ctx, 
                     GPIO_TypeDef* cs_port, uint16_t sel_pin,
                     GPIO_TypeDef* gd0_port, uint16_t gd0_pin,
                     GPIO_TypeDef* gd2_port, uint16_t gd2_pin,
                     GPIO_TypeDef* pa_en_port, uint16_t pa_en_pin,
                     GPIO_TypeDef* rx_en_port, uint16_t rx_en_pin,
                     SPI_HandleTypeDef* spi);

// Регистровые операции
int cc2500_writeRegister(CC2500CTX* ctx, uint8_t addr, uint8_t value);
int cc2500_readRegister(CC2500CTX* ctx, uint8_t addr, uint8_t* value);
int cc2500_writeRegisterBurst(CC2500CTX* ctx, uint8_t addr, const uint8_t* values, int len);
int cc2500_readRegisterBurst(CC2500CTX* ctx, uint8_t addr, uint8_t* values, int len);

// Команды и состояния
int cc2500_reset(CC2500CTX* ctx);
int cc2500_strobe(CC2500CTX* ctx, uint8_t state);
int cc2500_strobeR(CC2500CTX* ctx, uint8_t state);
int cc2500_readFIFO(CC2500CTX* ctx, uint8_t* buf, int length);
int cc2500_waitForState(CC2500CTX *ctx, uint8_t state);

// Конфигурация и контроль
int cc2500_configure(CC2500CTX* ctx);
void cc2500_setTxMode(CC2500CTX* ctx);
void cc2500_setRxMode(CC2500CTX* ctx);
void cc2500_setIdleMode(CC2500CTX* ctx);

// GPIO функции
uint8_t cc2500_readGD0(CC2500CTX* ctx);
uint8_t cc2500_readGD2(CC2500CTX* ctx);
void cc2500_setPAEnabled(CC2500CTX* ctx, uint8_t enable);
void cc2500_setRxEnabled(CC2500CTX* ctx, uint8_t enable);

// Передача данных
int cc2500_transmit(CC2500CTX* ctx, const uint8_t* data, uint8_t length);
int cc2500_receive(CC2500CTX* ctx, uint8_t* data, uint8_t* length);

// Функции для работы с RSSI и LQI
int8_t cc2500_getRSSI(CC2500CTX* ctx);
uint8_t cc2500_getLQI(CC2500CTX* ctx, uint8_t* crc_ok);

// Отладочные функции
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

extern CC2500_RegDump g_cc2500_dump;

void cc2500_dumpRegisters(CC2500CTX* ctx);
uint8_t cc2500_getState(CC2500CTX* ctx);

#define cc2500_readStatusRegister(ctx, addr, value) cc2500_readRegisterBurst(ctx, addr, value, 1)

#endif /* CC2500_H_ */
