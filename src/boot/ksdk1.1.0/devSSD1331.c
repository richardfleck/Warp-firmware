#include <stdint.h>
#include <stdbool.h>
#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[1];
volatile uint8_t	payloadBytes[1];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
};

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}

void drawRectLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t C, uint8_t B, uint8_t A, bool rect){
	// Draw a rectangle or line from (x0, y0) to (x1, y1) of colour (C, B, A)

        if (rect == 1){
		writeCommand(0x22); // Enter 'draw rectangle' mode
	} else {
		writeCommand(0x21); // Enter 'draw line' mode
	}
        writeCommand(x0);      // Start column address
        writeCommand(y0);      // Start row address
        writeCommand(x1);      // End column address
        writeCommand(y1);      // End row address
	writeCommand(C);       // Set outline colour C
        writeCommand(B);       // Set outline colour B
        writeCommand(A);       // Set outline colour A
        writeCommand(C);       // Set fill colour C
        writeCommand(B);       // Set fill colour B
        writeCommand(A);       // Set fill colour A
}

void drawDigit(uint8_t x0, uint8_t y0, uint8_t number, uint8_t C, uint8_t B, uint8_t A){
	// Draw digit from starting upper left corner (x0, y0) of colour (C, B, A)

	// First draw block of colour (C, B, A)
	drawRectLine(x0, y0, (x0+9), (y0+21), C, B, A, 1);

	// Depending on the number, different parts of the block are cut out
	switch(number)
	{
		case 0:
			drawRectLine(x0+2, y0+2, x0+7, y0+19, 0x00, 0x00, 0x00, 1);
			break;
		case 1:
			drawRectLine(x0, y0, x0+7, y0+21, 0x00, 0x00, 0x00, 1);
			break;
		case 2: 
			drawRectLine(x0, y0+2, x0+7, y0+9, 0x00, 0x00, 0x00, 1);
			drawRectLine(x0+2, y0+12, x0+9, y0+19, 0x00, 0x00, 0x00, 1);
			break;
		case 3:
                        drawRectLine(x0, y0+2, x0+7, y0+9, 0x00, 0x00, 0x00, 1);
			drawRectLine(x0, y0+12, x0+7, y0+19, 0x00, 0x00, 0x00, 1);
			break;
		case 4:
			drawRectLine(x0+2, y0, x0+7, y0+9, 0x00, 0x00, 0x00, 1);
			drawRectLine(x0, y0+12, x0+7, y0+21, 0x00, 0x00, 0x00, 1);
			break;
		case 5:
			drawRectLine(x0+2, y0+2, x0+9, y0+9, 0x00, 0x00, 0x00, 1);
			drawRectLine(x0, y0+12, x0+7, y0+19, 0x00, 0x00, 0x00, 1);
			break;
		case 6:
			drawRectLine(x0+2, y0+2, x0+9, y0+9, 0x00, 0x00, 0x00, 1);
			drawRectLine(x0+2, y0+12, x0+7, y0+19, 0x00, 0x00, 0x00, 1);
			break;
		case 7: 
			drawRectLine(x0, y0+2, x0+7, y0+21, 0x00, 0x00, 0x00, 1);
			break;
		case 8:
			drawRectLine(x0+2, y0+2, x0+7, y0+9, 0x00, 0x00, 0x00, 1);
			drawRectLine(x0+2, y0+12, x0+7, y0+19, 0x00, 0x00, 0x00, 1);
			break;
		case 9:
			drawRectLine(x0+2, y0+2, x0+7, y0+9, 0x00, 0x00, 0x00, 1);
			drawRectLine(x0, y0+12, x0+7, y0+19, 0x00, 0x00, 0x00, 1);
			break;	
		default:
			break;
	}
}


int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	enableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x06);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0x50);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);

	/*
	 *	Draw fixed lines and icons
	 */	

	drawRectLine(0x1C, 0x02, 0x21, 0x07, 0x3F, 0x3F, 0x3F, 1); // Temp degree block
	drawRectLine(0x1E, 0x04, 0x1F, 0x05, 0x00, 0x00, 0x00, 1); // Temp degree cutout
	drawRectLine(0x22, 0x0B, 0x29, 0x17, 0x3F, 0x3F, 0x3F, 1); // Temp celsius block
	drawRectLine(0x24, 0x0D, 0x29, 0x15, 0x00, 0x00, 0x00, 1); // Temp celsius cutout
	drawRectLine(0x53, 0x16, 0x59, 0x08, 0x3F, 0x3F, 0x3F, 0); // Hum % slash
	drawRectLine(0x50, 0x07, 0x55, 0x0C, 0x3F, 0x3F, 0x3F, 1); // Hum % upper block
	drawRectLine(0x52, 0x09, 0x53, 0x0A, 0x00, 0x00, 0x00, 1); // Hum % upper cutout
	drawRectLine(0x58, 0x12, 0x5D, 0x17, 0x3F, 0x3F, 0x3F, 1); // Hum % lower block
	drawRectLine(0x5A, 0x14, 0x5B, 0x15, 0x00, 0x00, 0x00, 1); // Hum % lower cutout
	drawRectLine(0x06, 0x1B, 0x59, 0x26, 0x3F, 0x3F, 0x3F, 1); // IAQ block
	drawRectLine(0x08, 0x1D, 0x57, 0x24, 0x00, 0x00, 0x00, 1); // IAQ cutout		
	return 0;
}


void    devSSD1331DrawTemp(uint8_t temp){
	// Draw two-digit temperature reading (blue if <20, red if >25, green if acceptable)

	uint8_t digit1 = (temp / 10) % 10;
	uint8_t digit0 = temp % 10;
	if(temp<20){	
        	drawDigit(0x02, 0x02, digit1, 0x00, 0x00, 0x3F);
        	drawDigit(0x10, 0x02, digit0, 0x00, 0x00, 0x3F);
	}
        else if(temp>25){    
                drawDigit(0x02, 0x02, digit1, 0x3F, 0x00, 0x00);
                drawDigit(0x10, 0x02, digit0, 0x3F, 0x00, 0x00);
        }
        else{
                drawDigit(0x02, 0x02, digit1, 0x00, 0x3F, 0x00);
                drawDigit(0x10, 0x02, digit0, 0x00, 0x3F, 0x00);
        }
}

void    devSSD1331DrawHum(uint8_t hum){
	// Draw two-digit %rH reading (blue if <40, red if >60, green if acceptable)

        uint8_t digit1 = (hum / 10) % 10;
        uint8_t digit0 = hum % 10;
	if (hum < 40) {
        	drawDigit(0x36, 0x02, digit1, 0x00, 0x00, 0x3F);
        	drawDigit(0x44, 0x02, digit0, 0x00, 0x00, 0x3F);
	}
	else if (hum > 60) {
        	drawDigit(0x36, 0x02, digit1, 0x3F, 0x00, 0x00);
        	drawDigit(0x44, 0x02, digit0, 0x3F, 0x00, 0x00);
	}
	else { 
		drawDigit(0x36, 0x02, digit1, 0x00, 0x3F, 0x00);
		drawDigit(0x44, 0x02, digit0, 0x00, 0x3F, 0x00);
	}
}

void	devSSD1331DrawIAQ(uint16_t gas_res){
	// Draw gas resistance/indoor air quality slider (red if below 20kOhm, green if above)

	uint8_t sliderLength = (gas_res * 80) / 500;
	drawRectLine(0x08, 0x1D, 0x57, 0x24, 0x00, 0x00, 0x00, 1);
	if (gas_res > 200){
		drawRectLine(0x08, 0x1D, sliderLength+8, 0x24, 0x00, 0x3F, 0x00, 1);
	}
	else {
	        drawRectLine(0x08, 0x1D, sliderLength+8, 0x24, 0x3F, 0x00, 0x00, 1);
	}
}

void	devSSD1331DrawWindowIcon(){
	drawRectLine(0x02, 0x2A, 0x1B, 0x3D, 0x3F, 0x3F, 0x3F, 1);
	drawRectLine(0x05, 0x2D, 0x0D, 0x32, 0x00, 0x00, 0x00, 1);
	drawRectLine(0x10, 0x2D, 0x18, 0x32, 0x00, 0x00, 0x00, 1);
	drawRectLine(0x05, 0x35, 0x0D, 0x3A, 0x00, 0x00, 0x00, 1);
	drawRectLine(0x10, 0x35, 0x18, 0x3A, 0x00, 0x00, 0x00, 1);	
}

void	devSSD1331ClearWindowIcon(){
	drawRectLine(0x02, 0x2A, 0x1B, 0x3D, 0x00, 0x00, 0x00, 1);
}
           
void	devSSD1331DrawRadiatorIcon(){
	drawRectLine(0x46, 0x2A, 0x48, 0x3B, 0x3F, 0x3F, 0x3F, 1);
	drawRectLine(0x4A, 0x2C, 0x4C, 0x3B, 0x3F, 0x3F, 0x3F, 1);
	drawRectLine(0x4E, 0x2C, 0x50, 0x3B, 0x3F, 0x3F, 0x3F, 1);
        drawRectLine(0x52, 0x2C, 0x54, 0x3B, 0x3F, 0x3F, 0x3F, 1);
        drawRectLine(0x56, 0x2C, 0x58, 0x3B, 0x3F, 0x3F, 0x3F, 1);
        drawRectLine(0x5A, 0x2C, 0x5C, 0x3D, 0x3F, 0x3F, 0x3F, 1);
}

void	devSSD1331ClearRadiatorIcon(){
	drawRectLine(0x46, 0x2A, 0x5C, 0x3D, 0x00, 0x00, 0x00, 1);
}

void    devSSD1331DrawLightIcon(uint8_t C, uint8_t B, uint8_t A){
        drawRectLine(0x2F, 0x2A, 0x30, 0x3D, C, B, A, 1);
        drawRectLine(0x26, 0x33, 0x39, 0x34, C, B, A, 1);
        drawRectLine(0x28, 0x2C, 0x37, 0x3B, C, B, A, 0);
	drawRectLine(0x37, 0x2C, 0x28, 0x3B, C, B, A, 0);
}

void	devSSD1331DrawSmiley(){
	drawRectLine(0x2B, 0x2D, 0x2C, 0x33, 0x00, 0x3F, 0x00, 1);
	drawRectLine(0x33, 0x2D, 0x34, 0x33, 0x00, 0x3F, 0x00, 1);
	drawRectLine(0x2E, 0x3A, 0x31, 0x3B, 0x00, 0x3F, 0x00, 1);
        drawRectLine(0x2C, 0x39, 0x2D, 0x3A, 0x00, 0x3F, 0x00, 1);
        drawRectLine(0x32, 0x39, 0x33, 0x3a, 0x00, 0x3F, 0x00, 1);
        drawRectLine(0x2B, 0x38, 0x2C, 0x39, 0x00, 0x3F, 0x00, 1);
        drawRectLine(0x33, 0x38, 0x34, 0x39, 0x00, 0x3F, 0x00, 1);
        drawRectLine(0x2A, 0x37, 0x2B, 0x38, 0x00, 0x3F, 0x00, 1);
        drawRectLine(0x34, 0x37, 0x35, 0x38, 0x00, 0x3F, 0x00, 1);
        drawRectLine(0x29, 0x36, 0x2A, 0x36, 0x00, 0x3F, 0x00, 1);
        drawRectLine(0x35, 0x36, 0x36, 0x36, 0x00, 0x3F, 0x00, 1);
}

void	devSSD1331ClearMiddle(){
	// Clears the middle portion of the bottom row, where both the light icon and smiley icon can appear
	drawRectLine(0x23, 0x2A, 0x3C, 0x3D, 0x00, 0x00, 0x00, 1);
}	
