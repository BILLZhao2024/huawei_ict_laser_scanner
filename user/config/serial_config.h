/**
 * @file    serial_config.h
 * @brief   STM32F401 应用配置层头文件
 *
 * 声明全局串口实例和初始化函数。
 * 应用通过此头文件访问 serial0。
 */

#ifndef __SERIAL_CONFIG_H__
#define __SERIAL_CONFIG_H__

#include "common/common.h"
#include "serial/serial.h"
#include "serial/ports/stm32f4/serial_port_stm32f4.h"

/** @brief USART1 实例 (PA9/PA10) */
extern serial_t serial0;

/** @brief USART6 实例 (PA11/PA12) — 激光指令接收 */
extern serial_t serial6;

/** @brief 初始化串口实例 */
void serial_config_init(void);

#endif
