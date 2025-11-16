#ifndef __CLI_HANDLER_H
#define __CLI_HANDLER_H

#include "main.h"
#include "cc2500.h"

// Инициализация обработчика команд
void cli_init(CC2500CTX* cc2500_context);

// Функция для обработки входящих данных из USB VCP
void cli_process_input(uint8_t* buf, uint32_t len);

#endif /* __CLI_HANDLER_H */