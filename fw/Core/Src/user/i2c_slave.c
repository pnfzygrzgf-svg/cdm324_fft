#include "user/i2c_slave.h"
#include "stm32f3xx_hal.h"

/* TX buffer: [speed_lo, speed_hi] */
static volatile uint8_t i2c_tx_buf[2] = {0, 0};
static volatile uint8_t i2c_tx_idx = 0;

void i2c_slave_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Enable clocks */
	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();

	/* Configure PF0 (SDA) and PF1 (SCL) for I2C2 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/* Disable I2C2 for configuration */
	I2C2->CR1 = 0;

	/* Timing (not critical for slave, but required) */
	I2C2->TIMINGR = 0x0000020B;

	/* Own address 1: enable, 7-bit mode */
	I2C2->OAR1 = I2C_OAR1_OA1EN | I2C_SLAVE_ADDR;

	/* Disable own address 2 */
	I2C2->OAR2 = 0;

	/* CR1: Enable I2C, enable ADDR and TXIS and STOP interrupts, NoStretch */
	I2C2->CR1 = I2C_CR1_PE | I2C_CR1_ADDRIE | I2C_CR1_TXIE | I2C_CR1_STOPIE
	           | I2C_CR1_ERRIE | I2C_CR1_NOSTRETCH;

	/* Preload first byte into TXDR so it's ready immediately */
	I2C2->TXDR = i2c_tx_buf[0];

	/* Enable I2C2 event and error interrupts */
	HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
	HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
}

void i2c_slave_update_speed(uint16_t speed_kmh)
{
	i2c_tx_buf[0] = (uint8_t)(speed_kmh & 0xFF);
	i2c_tx_buf[1] = (uint8_t)(speed_kmh >> 8);
}

/* I2C2 event IRQ: handle ADDR, TXIS, STOP */
void I2C2_EV_IRQHandler(void)
{
	uint32_t isr = I2C2->ISR;

	/* Address matched */
	if (isr & I2C_ISR_ADDR)
	{
		i2c_tx_idx = 0;
		/* Preload first byte */
		I2C2->TXDR = i2c_tx_buf[0];
		/* Clear ADDR flag */
		I2C2->ICR = I2C_ICR_ADDRCF;
	}

	/* Transmit register empty - load next byte */
	if (isr & I2C_ISR_TXIS)
	{
		i2c_tx_idx++;
		if (i2c_tx_idx < 2)
			I2C2->TXDR = i2c_tx_buf[i2c_tx_idx];
		else
			I2C2->TXDR = 0x00;  /* Padding if master reads more */
	}

	/* STOP detected - reset for next transaction */
	if (isr & I2C_ISR_STOPF)
	{
		I2C2->ICR = I2C_ICR_STOPCF;
		/* Flush TX and preload for next read */
		I2C2->ISR |= I2C_ISR_TXE;
		I2C2->TXDR = i2c_tx_buf[0];
	}
}

/* I2C2 error IRQ: clear errors and recover */
void I2C2_ER_IRQHandler(void)
{
	/* Clear all error flags */
	I2C2->ICR = I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
	/* Flush TX and preload */
	I2C2->ISR |= I2C_ISR_TXE;
	I2C2->TXDR = i2c_tx_buf[0];
}
