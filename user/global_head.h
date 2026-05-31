#ifndef __GLOBAL_HEAD_H__
#define __GLOBAL_HEAD_H__

#include "common/common.h"
#include "serial/serial.h"

#include "main.h"
#include "serial_config.h"

/* Fast GPIO macros using BSRR register (library-independent) */
#define GPIO_FAST_SETBIT(port, bit)   GPIO##port##->BSRR |= 0x00000001 << bit
#define GPIO_FAST_RESETBIT(port, bit) GPIO##port##->BSRR |= 0x00000001 << (bit + 16)

/* SPI1 handle for DAC8563 driver (defined in Core/Src/spi.c) */
extern SPI_HandleTypeDef hspi1;

#endif /* __GLOBAL_HEAD_H__ */
