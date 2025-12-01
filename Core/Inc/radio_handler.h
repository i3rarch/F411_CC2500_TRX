#ifndef __RADIO_HANDLER_H
#define __RADIO_HANDLER_H

#include "main.h"
#include "cc2500.h"

// Режимы работы радио
typedef enum {
    RADIO_MODE_RX = 0,
    RADIO_MODE_TX = 1
} RadioMode_t;

// Статистика радио
typedef struct {
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t rx_errors;
} RadioStats_t;

// Инициализация модуля радио
void radio_init(CC2500CTX* ctx, RadioMode_t mode);

// Основной цикл обработки (вызывается из main loop)
void radio_process(void);

// Обработчик прерывания GDO (вызывается из HAL_GPIO_EXTI_Callback)
void radio_gdo_irq_handler(uint16_t gpio_pin);

// Получение статистики
RadioStats_t* radio_get_stats(void);

// Получение текущего режима
RadioMode_t radio_get_mode(void);

// Установка флага debug
void radio_set_debug(uint8_t enable);

#endif /* __RADIO_HANDLER_H */