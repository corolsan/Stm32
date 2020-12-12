#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_spi.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


xQueueHandle g_led;



void gpio()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

	GPIO_InitTypeDef g;

	// Leds
	GPIO_StructInit(&g);
	g.GPIO_Pin = 0xFF00;
	g.GPIO_Mode = GPIO_Mode_OUT;
	g.GPIO_Speed = GPIO_Speed_Level_1;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &g);

	// SS for gyro
	GPIO_StructInit(&g);
	g.GPIO_Pin = GPIO_Pin_3;
	g.GPIO_Mode = GPIO_Mode_OUT;
	g.GPIO_Speed = GPIO_Speed_Level_1;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &g);

	// SPI1
	GPIO_StructInit(&g);
	g.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	g.GPIO_Mode = GPIO_Mode_AF;
	g.GPIO_Speed = GPIO_Speed_Level_1;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &g);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_5);
}

void spi()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	SPI_InitTypeDef s;

	SPI_StructInit(&s);
	s.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	s.SPI_DataSize = SPI_DataSize_8b;
	s.SPI_CPOL = SPI_CPOL_High;
	s.SPI_CPHA = SPI_CPHA_2Edge;
	s.SPI_NSS = SPI_NSS_Soft;
	s.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	s.SPI_FirstBit = SPI_FirstBit_MSB;
	s.SPI_CRCPolynomial = 7;
	s.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPI1, &s);
	SPI_Cmd(SPI1, ENABLE);

	SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
}

uint8_t sendByte(uint8_t data)
{
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){};
	SPI_SendData8(SPI1, data);
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){};
	return (uint8_t)SPI_ReceiveData8(SPI1);
}

void writeData(uint8_t address, uint8_t data)
{
	GPIO_ResetBits(GPIOE, GPIO_Pin_3);
	sendByte(address);
	sendByte(data);
	GPIO_SetBits(GPIOE, GPIO_Pin_3);
}

void GyroScan()
{
	uint8_t receiveData[2];
	uint16_t xResult;
	uint8_t xSign;
	double xPosition;

	while (1)
	{
		GPIO_ResetBits(GPIOE, GPIO_Pin_3); // Start talk
		sendByte(0xE8); // Start gyro conversion
		receiveData[0] = sendByte(0x00); // Get first byte of conversion
		receiveData[1] = sendByte(0x00); // Get second byte of conversion
		GPIO_SetBits(GPIOE, GPIO_Pin_3); // End talk

		xResult = (receiveData[0] | (receiveData[1] << 8)) - 10; // -10 is offset of gyro, get it calibrated!
		if ((xResult & 0x8000) == 0) // find out sign
			xSign = 0; // "+"
		else
		{
			xSign = 1; // "-"
			// Flip according to datasheet, this is reverse-code according to our gyro
			xResult &= 0x7FFF;
			xResult = 0x8000 - xResult;
		}
		if (xResult < 0x20) // Threshold to remove electrical/thermal bouncing of gyro, get it calibrated!
			xResult = 0;
		// 0.07 is degrees per second; 0.025 is sampling rate in seconds.
		// Normally it should be 0.02, but we are too slow in this interrupt handler, so we are using 0.025
		if (xSign == 0)
			xPosition += 0.07 * xResult * 0.025;
		else
			xPosition -= 0.07 * xResult * 0.025;

		xQueueSend(g_led, &xPosition, 0);
		vTaskDelay(20);
	}
}

void taskLED()
{
	double pos;
	while (1)
	{
		xQueueReceive(g_led, &pos, 0);

		GPIO_Write(GPIOE, 0x0000); // Switch off all LEDs

		// Map angle to LEDs
		if ((pos > -105) && (pos < -75))
			GPIO_SetBits(GPIOE, GPIO_Pin_8);
		if ((pos > -75) && (pos < -45))
			GPIO_SetBits(GPIOE, GPIO_Pin_9);
		if ((pos > -45) && (pos < -15))
			GPIO_SetBits(GPIOE, GPIO_Pin_10);
		if ((pos > -15) && (pos < 15))
			GPIO_SetBits(GPIOE, GPIO_Pin_11);
		if ((pos > 15) && (pos < 45))
			GPIO_SetBits(GPIOE, GPIO_Pin_12);
		if ((pos > 45) && (pos < 75))
			GPIO_SetBits(GPIOE, GPIO_Pin_13);
		if ((pos > 75) && (pos < 105))
			GPIO_SetBits(GPIOE, GPIO_Pin_14);
		if ((pos > 105) || (pos < -105))
					GPIO_SetBits(GPIOE, GPIO_Pin_15);
	}
}


int main()
{
	gpio();
	spi();

	writeData(0x20, 0x0A);
	writeData(0x23, 0x30);

	g_led = xQueueCreate(1, sizeof(double));

	xTaskCreate(GyroScan, (char *)"GYRO", configMINIMAL_STACK_SIZE, NULL, 2, (xTaskHandle *)NULL);
	xTaskCreate(taskLED, (char *)"LED", configMINIMAL_STACK_SIZE, NULL, 2, (xTaskHandle *)NULL);
	vTaskStartScheduler();

	while (1)
	{
		__NOP();
	}

	return 0;
}
void vApplicationIdleHook ( void ){}
void vApplicationMallocFailedHook ( void ){for ( ;; );}
void vApplicationStackOverflowHook ( xTaskHandle pxTask, char *pcTaskName ){
( void ) pcTaskName;
( void ) pxTask;
for ( ;; );}
void vApplicationTickHook ( void ){}
