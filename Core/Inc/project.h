/**
 * @file    project.h
 * @brief   Common project definitions and includes
 * @author  opiopan (original), i3rarch (modified)
 * @date    2019/01/17, modified 2025
 */

#ifndef PROJECT_H_
#define PROJECT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Standard includes */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* STM32 HAL includes */
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

/* Utility macros */
#ifndef MAX
#define MAX(a, b)   (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b)   (((a) < (b)) ? (a) : (b))
#endif

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr)  (sizeof(arr) / sizeof((arr)[0]))
#endif

/* Unused parameter macro to suppress warnings */
#ifndef UNUSED
#define UNUSED(x)   ((void)(x))
#endif

#ifdef __cplusplus
}
#endif

#endif /* PROJECT_H_ */
