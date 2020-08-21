#include "XPT2046_touch.h"

extern SPI_HandleTypeDef XPT2046_SPI_PORT;

#define READ_X 0x90
#define READ_Y 0xD0

static void XPT2046_doTouchSelect()
{
	// chip select signal is active
	HAL_GPIO_WritePin(XPT2046_CS_GPIO_Port, XPT2046_CS_Pin, GPIO_PIN_RESET);
}

static void XPT2046_doTouchUnselect()
{
	// chip select signal is deactive
	HAL_GPIO_WritePin(XPT2046_CS_GPIO_Port, XPT2046_CS_Pin, GPIO_PIN_SET);
}

uint8_t XPT2046_isTouchPressed(void)
{
    // if IRQ signal on input pin has low level -> return TRUE
	return HAL_GPIO_ReadPin(XPT2046_IRQ_GPIO_Port, XPT2046_IRQ_Pin) == GPIO_PIN_RESET;
}

uint8_t XPT2046_doGetTouchGetCoordinates(uint16_t* x, uint16_t* y)
{
    // prepare command data for sending to touch screen via SPI
	static const uint8_t cmd_read_x[] = { READ_X };
    static const uint8_t cmd_read_y[] = { READ_Y };
    static const uint8_t zeroes_tx[] = { 0x00, 0x00 };

    // activate chip select signal
    XPT2046_doTouchSelect();

    uint32_t avg_x = 0;
    uint32_t avg_y = 0;

    // get 16 samples max and calculate average value
    for(uint8_t i = 0; i < 16; i++)
    {
        // finger is out from screen -> stop getting data -> short touching is ignored
    	if (!XPT2046_isTouchPressed())
    	{
    		// deactivate chip select signal
    		XPT2046_doTouchUnselect();
    		// return error
    		return 0;
    	}

        // get y
        HAL_SPI_Transmit(&XPT2046_SPI_PORT, (uint8_t*)cmd_read_y, sizeof(cmd_read_y), HAL_MAX_DELAY);
        uint8_t y_raw[2];
        HAL_SPI_TransmitReceive(&XPT2046_SPI_PORT, (uint8_t*)zeroes_tx, y_raw, sizeof(y_raw), HAL_MAX_DELAY);

        // get x
        HAL_SPI_Transmit(&XPT2046_SPI_PORT, (uint8_t*)cmd_read_x, sizeof(cmd_read_x), HAL_MAX_DELAY);
        uint8_t x_raw[2];
        HAL_SPI_TransmitReceive(&XPT2046_SPI_PORT, (uint8_t*)zeroes_tx, x_raw, sizeof(x_raw), HAL_MAX_DELAY);

        // accumulate data
        avg_x += (((uint16_t)x_raw[0]) << 8) | ((uint16_t)x_raw[1]);
        avg_y += (((uint16_t)y_raw[0]) << 8) | ((uint16_t)y_raw[1]);
    }
    // deactivate chip select signal
    XPT2046_doTouchUnselect();

    // calculate values according scale

    // divide 16
    uint32_t raw_x = (avg_x >> 4);
    // overflow & underflow checking
    if(raw_x < XPT2046_MIN_RAW_X) raw_x = XPT2046_MIN_RAW_X;
    if(raw_x > XPT2046_MAX_RAW_X) raw_x = XPT2046_MAX_RAW_X;

    // divide 16
    uint32_t raw_y = (avg_y >> 4);
    // overflow & underflow checking
    if(raw_y < XPT2046_MIN_RAW_Y) raw_y = XPT2046_MIN_RAW_Y;
    if(raw_y > XPT2046_MAX_RAW_Y) raw_y = XPT2046_MAX_RAW_Y;

    // make scaling
    *x = (raw_x - XPT2046_MIN_RAW_X) * XPT2046_SCALE_X / (XPT2046_MAX_RAW_X - XPT2046_MIN_RAW_X);
    *y = (raw_y - XPT2046_MIN_RAW_Y) * XPT2046_SCALE_Y / (XPT2046_MAX_RAW_Y - XPT2046_MIN_RAW_Y);

    // return -> have new coordinates
    return 1;
}


