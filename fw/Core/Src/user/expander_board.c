#include "user/expander_board.h"
#include "fatfs.h"
#include <string.h>
/* SPI to the sd card */
SPI_HandleTypeDef hspi3;
/* Variables to write to the SD card */
FATFS expander_board_fs;
FIL expander_board_file;


/*! \fn     expander_init(void)
*   \brief  Expander initialization code
*   \return TRUE if a SD card is detected
*/
BOOL expander_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* Configure GPIO pins: SD card detect */
	GPIO_InitStruct.Pin = SD_CD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SD_CD_Pin_Port, &GPIO_InitStruct);

	/* Configure GPIO pins: user switch */
	GPIO_InitStruct.Pin = SWITCH_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SWITCH_GPIO_Port, &GPIO_InitStruct);

	/* Configure GPIO pins: SPI slave select */
	GPIO_InitStruct.Pin = SPI_SS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(SPI_SS_GPIO_Port, &GPIO_InitStruct);

	/* Configure SPI pins */
	GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* SPI3 parameter configuration */
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 7;
	hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	HAL_SPI_Init(&hspi3);

	/* FatFS library init */
	MX_FATFS_Init();

	/* uSD card inserted? */
	if (HAL_GPIO_ReadPin(SD_CD_Pin_Port, SD_CD_Pin) == GPIO_PIN_RESET)
	{
		/* Mount filesystem */
		f_mount(&expander_board_fs, "", 0);

		/* Test code */
		/*f_open(&expander_board_file, "test.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
		f_lseek(&expander_board_file, expander_board_file.fsize);
		f_puts("This is an example text to check SD Card Module with STM32 Blue Pill\n", &expander_board_file);
		f_close(&expander_board_file);*/

		/* Return success */
		return TRUE;
	}

	/* No uSD card */
	return FALSE;
}

/*! \fn     expander_is_kph_selected(void)
*   \brief  Check if KPH is selected
*   \return TRUE if so
*/
BOOL expander_is_kph_selected(void)
{
	if (HAL_GPIO_ReadPin(SWITCH_GPIO_Port, SWITCH_Pin) == GPIO_PIN_RESET)
	{
		return TRUE;
	}
	return FALSE;
}

/*! \fn     expander_sd_open_csv(void)
*   \brief  Open or create CSV log file on SD card
*   \return TRUE if file opened successfully
*/
BOOL expander_sd_open_csv(void)
{
	FRESULT res;

	res = f_open(&expander_board_file, "log.csv", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
	if (res != FR_OK)
	{
		return FALSE;
	}

	/* Check if file is new (empty) and write CSV header */
	if (f_size(&expander_board_file) == 0)
	{
		f_puts("nr,timestamp_ms,speed_kmh,duration_ms,length_m,type\n", &expander_board_file);
	}

	/* Seek to end for appending */
	f_lseek(&expander_board_file, f_size(&expander_board_file));
	f_sync(&expander_board_file);
	return TRUE;
}

/*! \fn     expander_sd_log_vehicle(vd_event_t* evt)
*   \brief  Write one vehicle event as CSV line to SD card
*   \param  evt     Pointer to completed vehicle event
*/
/* Simple uint32 to string, returns pointer to end of written chars */
static char* u32_to_str(char* buf, uint32_t val)
{
	char tmp[10];
	int i = 0;

	if (val == 0)
	{
		*buf++ = '0';
		return buf;
	}
	while (val > 0)
	{
		tmp[i++] = '0' + (val % 10);
		val /= 10;
	}
	while (i > 0)
	{
		*buf++ = tmp[--i];
	}
	return buf;
}

void expander_sd_log_vehicle(vd_event_t* evt)
{
	char line_buf[80];
	char* p = line_buf;
	uint16_t speed_int = (uint16_t)evt->speed_kmh;
	uint16_t speed_dec = (uint16_t)((evt->speed_kmh - speed_int) * 10);
	uint16_t len_int = (uint16_t)evt->length_m;
	uint16_t len_dec = (uint16_t)((evt->length_m - len_int) * 10);
	uint32_t duration_ms = (uint32_t)(evt->duration_frames * 28.6f);

	/* Build CSV line: nr,timestamp_ms,speed_kmh,duration_ms,length_m,type */
	p = u32_to_str(p, evt->event_number); *p++ = ',';
	p = u32_to_str(p, evt->timestamp_ms); *p++ = ',';
	p = u32_to_str(p, speed_int); *p++ = '.';
	p = u32_to_str(p, speed_dec); *p++ = ',';
	p = u32_to_str(p, duration_ms); *p++ = ',';
	p = u32_to_str(p, len_int); *p++ = '.';
	p = u32_to_str(p, len_dec); *p++ = ',';
	memcpy(p, evt->type, strlen(evt->type)); p += strlen(evt->type);
	*p++ = '\n';
	*p = '\0';

	f_puts(line_buf, &expander_board_file);
	f_sync(&expander_board_file);
}
