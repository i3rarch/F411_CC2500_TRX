# CC2500 STM32F411 Black Pill Project

Проект для работы с радиомодулем **CC2500** на базе микроконтроллера **STM32F411CEU6** (Black Pill).

### Подключение CC2500 к STM32F411

| CC2500 Pin | STM32F411 Pin | Функция    |
|------------|---------------|------------|
| VCC        | 3.3V          | Питание    |
| GND        | GND           | Земля      |
| CSN        | PA4           | Chip Select (активный низкий) |
| SCK        | PA5           | SPI Clock  |
| MISO       | PA6           | Master In Slave Out |
| MOSI       | PA7           | Master Out Slave In |
| GDO0       | PB0           | General Digital Output 0 |
| GDO2       | PB1           | General Digital Output 2 |

## Сборка и прошивка

### Требования
- **ARM GCC Toolchain** (arm-none-eabi-gcc)
- **Make**

### Сборка
```bash
make clean
make all
```

## Лицензия

Код проекта лицензирован в соответствии с условиями STMicroelectronics.

## Автор

[tg: @i3rarch]

---

**Примечание**: Данный проект является базовой конфигурацией. Для полноценной работы с CC2500 требуется реализация дополнительных функций управления чипом.