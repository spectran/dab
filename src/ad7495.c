/*
 * ad7495.c
 *
 *  Created on: 24 сент. 2016 г.
 *      Author: Spectran
 */

#include "stm32f1xx_hal.h"
#include "spi.h"
#include "usbd_cdc_if.h"

uint8_t txd[2] = {0x11, 0x11};

uint8_t adc1[2];
uint8_t adc2[2];

/**
  * @brief  Read_Adc
  *         This function performs ADC conversion using selected
  *         AD7495 and sends conversion results via CDC interface.
  *
  * @param  mode: configuration byte
  * bit 0: ADC0 is on
  * bit 1: ADC1 is on
  * @retval none
  */
void Read_Adc(uint8_t mode)
{
	if((mode & 0x03) == 0x03) {
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)txd, (uint8_t*)adc1, 1, 0x1000);
		HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)txd, (uint8_t*)adc2, 1, 0x1000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
		while(CDC_Transmit_FS(adc1, 2) != USBD_OK);
		while(CDC_Transmit_FS(adc2, 2) != USBD_OK);
	}

	if((mode & 0x03) == 0x01) {
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)txd, (uint8_t*)adc1, 1, 0x1000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
		while(CDC_Transmit_FS(adc1, 2) != USBD_OK);
	}

	if((mode & 0x03) == 0x02) {
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)txd, (uint8_t*)adc2, 1, 0x1000);
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
		while(CDC_Transmit_FS(adc2, 2) != USBD_OK);
	}
}

/**
  * @brief  Adc_Probe
  *         Test ADC conversion using AD7495 (not valid)
  *
  * @param  none
  * @retval none
  */
void Adc_Probe()
{
	HAL_Delay(100);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)txd, (uint8_t*)adc1, 1, 0x1000);
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)txd, (uint8_t*)adc2, 1, 0x1000);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}
