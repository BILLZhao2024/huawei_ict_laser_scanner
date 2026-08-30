#include "dac8563.h"

#if DAC8563_USE_SPI_HW
void dac8563_write(uint8_t cmd, uint16_t data)
{
  SYNC_L;

  DAC8563_SPI.Instance->DR = cmd;
  while(__HAL_SPI_GET_FLAG(&DAC8563_SPI, SPI_FLAG_BSY) == 1);

  DAC8563_SPI.Instance->DR = (data >> 8) & 0xFF;
  while(__HAL_SPI_GET_FLAG(&DAC8563_SPI, SPI_FLAG_BSY) == 1);

  DAC8563_SPI.Instance->DR = data & 0xFF;
  while(__HAL_SPI_GET_FLAG(&DAC8563_SPI, SPI_FLAG_BSY) == 1);

  SYNC_H;
}
#else
void dac8563_write(uint8_t cmd,uint16_t data)
{
  uint8_t s=0;
  SYNC_H;
  __nop();
  SYNC_L;
  SCLK_L;
  for( s=0;s<8;s++)
  {
    if((cmd&0x80)==0x80){DIN_H;}
    else{DIN_L;}
    __nop();
    SCLK_H;
    __nop();
    cmd<<=1;
    SCLK_L;
    __nop();
  }
  for( s=0;s<16;s++)
  {
    if((data&0x8000)==0x8000){DIN_H;}
    else{DIN_L;}
    __nop();
    SCLK_H;
    __nop();
    data<<=1;
    SCLK_L;
    __nop();
  }
}
#endif

void dac8563_init(void)
{
#if DAC8563_USE_SPI_HW
  __HAL_SPI_ENABLE(&DAC8563_SPI);
#endif

  CLR_L;
  LDAC_H;
  dac8563_write(CMD_RESET_ALL_REG, DATA_RESET_ALL_REG);
  dac8563_write(CMD_PWR_UP_A_B, DATA_PWR_UP_A_B);
  dac8563_write(CMD_INTERNAL_REF_EN, DATA_INTERNAL_REF_EN);
  dac8563_write(CMD_GAIN, DATA_GAIN_B2_A2);
}

void dac8563_output(uint16_t data_a, uint16_t data_b)
{
  dac8563_write(CMD_SETA_UPDATEA, data_a);
  dac8563_write(CMD_SETB_UPDATEB, data_b);
  LDAC_L;
  __nop();
  LDAC_H;
}

void dac8563_output_int16(int16_t data_a, int16_t data_b)
{
  dac8563_output((uint16_t)(data_a + 32768), (uint16_t)(data_b + 32768));
}

void dac8563_output_float(float data_a, float data_b)
{
  uint16_t a = (uint16_t)((data_a / 20.0f) * 65535.0f);
  uint16_t b = (uint16_t)((data_b / 20.0f) * 65535.0f);
  dac8563_output(a, b);
}
