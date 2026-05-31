/**
 * @file    serial_config.c
 * @brief   应用配置层 — STM32F401 USART1 实例定义与 ISR 入口
 *
 * 本文件定义：
 *   - USART1 外设的硬件配置（引脚 PA9/PA10、波特率 115200、中断号）
 *   - TX 环形缓冲区和 RX 双缓冲池
 *   - serial_t 实例（serial0）
 *   - 初始化函数 serial_config_init()
 *   - ISR 入口（USART1_IRQHandler）
 *
 * 引脚映射：
 *   USART1: PA9(TX) / PA10(RX) — 通常连接板载 USB-UART 调试串口
 */

#include "serial_config.h"

/* ============================== 缓冲区 ============================== */

static uint8_t serial0_tx_buf[512];
static uint8_t serial0_rx_pool[2][256];

/* ============================== 硬件引脚配置 ============================== */

/** @brief USART1: PA9(TX) / PA10(RX) @ 115200 8N1 */
static const serial_stm32f4_config_t serial0_hw = {
    .periph          = USART1,
    .baudrate        = 115200,
    .irqn            = USART1_IRQn,
    .tx_gpio         = GPIOA,  .tx_pin = GPIO_PIN_9,   .tx_af = GPIO_AF7_USART1,
    .rx_gpio         = GPIOA,  .rx_pin = GPIO_PIN_10,  .rx_af = GPIO_AF7_USART1,
    .usart_clk_reg   = &RCC->APB2ENR,
    .usart_clk_bit   = RCC_APB2ENR_USART1EN,
    .tx_gpio_clk_reg = &RCC->AHB1ENR,
    .tx_gpio_clk_bit = RCC_AHB1ENR_GPIOAEN,
    .rx_gpio_clk_reg = &RCC->AHB1ENR,
    .rx_gpio_clk_bit = RCC_AHB1ENR_GPIOAEN,
    .get_pclk        = HAL_RCC_GetPCLK2Freq,  /* USART1 在 APB2 总线上 */
};

/* ============================== 串口实例 ============================== */

/** @brief USART1 实例 */
serial_t serial0 = {
    .ops              = &stm32f4_serial_ops,
    .port_data        = (void *)&serial0_hw,
    .tx_buf           = serial0_tx_buf,
    .tx_buf_size      = 512,
    .tx_mask          = 511,
    .rx_pool          = &serial0_rx_pool[0][0],
    .rx_pool_count    = 2,
    .rx_pool_buf_size = 256,
};

/* ============================== USART6 缓冲区 ============================== */

static uint8_t serial6_tx_buf[512];
static uint8_t serial6_rx_pool[2][256];

/* ============================== USART6 硬件引脚配置 ============================== */

/** @brief USART6: PA11(TX) / PA12(RX) @ 115200 8N1 — 激光指令接收 */
static const serial_stm32f4_config_t serial6_hw = {
    .periph          = USART6,
    .baudrate        = 115200,
    .irqn            = USART6_IRQn,
    .tx_gpio         = GPIOA,  .tx_pin = GPIO_PIN_11,  .tx_af = GPIO_AF8_USART6,
    .rx_gpio         = GPIOA,  .rx_pin = GPIO_PIN_12,  .rx_af = GPIO_AF8_USART6,
    .usart_clk_reg   = &RCC->APB2ENR,
    .usart_clk_bit   = RCC_APB2ENR_USART6EN,
    .tx_gpio_clk_reg = &RCC->AHB1ENR,
    .tx_gpio_clk_bit = RCC_AHB1ENR_GPIOAEN,
    .rx_gpio_clk_reg = &RCC->AHB1ENR,
    .rx_gpio_clk_bit = RCC_AHB1ENR_GPIOAEN,
    .get_pclk        = HAL_RCC_GetPCLK2Freq,  /* USART6 在 APB2 总线上 */
};

/* ============================== USART6 串口实例 ============================== */

/** @brief USART6 实例 — 接收激光控制指令 */
serial_t serial6 = {
    .ops              = &stm32f4_serial_ops,
    .port_data        = (void *)&serial6_hw,
    .tx_buf           = serial6_tx_buf,
    .tx_buf_size      = 512,
    .tx_mask          = 511,
    .rx_pool          = &serial6_rx_pool[0][0],
    .rx_pool_count    = 2,
    .rx_pool_buf_size = 256,
};

/* ============================== 初始化 ============================== */

void serial_config_init(void)
{
    serial_init(&serial0);
    serial_init(&serial6);
}

/* ============================== 中断服务函数 ============================== */

void USART1_IRQHandler(void) { stm32f4_isr(&serial0, USART1); }
void USART6_IRQHandler(void) { stm32f4_isr(&serial6, USART6); }
