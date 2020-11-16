#ifdef Adfr_SSD1306
#include "DispAdfSSD1306.h"
#include <SPI.h>
#include <Wire.h>
//#include <Adafruit_GFX.h>
//#include <Fonts\GregFF.h>
#include <Adafruit_SSD1306.h>
#include <U8g2_for_Adafruit_GFX.h>

// OLED display TWI address
#define OLED_ADDR   0x3C
#define SCL_PIN 22 // SCL CLK SCK
#define SDA_PIN 21 // SDA MOSI

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//Adafruit_SSD1306 display(
//	SCREEN_WIDTH,
//	SCREEN_HEIGHT,
//	SDA_PIN,
//	SCL_PIN,
//	OLED_RESET,
//	OLED_RESET,
//	OLED_RESET);

U8G2_FOR_ADAFRUIT_GFX u8g2_for_adafruit_gfx;

#define U8G_DRAW_UPPER_RIGHT 0x01
#define U8G_DRAW_UPPER_LEFT  0x02
#define U8G_DRAW_LOWER_LEFT 0x04
#define U8G_DRAW_LOWER_RIGHT  0x08
#define U8G_DRAW_ALL (U8G_DRAW_UPPER_RIGHT|U8G_DRAW_UPPER_LEFT|U8G_DRAW_LOWER_RIGHT|U8G_DRAW_LOWER_LEFT)



#define RowStep 18
#define Row0 10
#define Row1 Row0 + RowStep
#define Row2 Row1 + RowStep
#define Row3 Row2 + RowStep

void DispAdfSSD1306::Init()
{
	// initialize and clear display
	Wire.begin(SDA_PIN, SCL_PIN);
	display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
	u8g2_for_adafruit_gfx.begin(display);

	u8g2_for_adafruit_gfx.setFontMode(1);                     // use u8g2 none transparent mode
	u8g2_for_adafruit_gfx.setFontDirection(0);                // left to right (this is default)
	u8g2_for_adafruit_gfx.setFont(u8g2_font_unifont_t_latin); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
	//u8g2_for_adafruit_gfx.setFont(u8g2_font_helvR12_te);

	ShowInfo(3, 30, "Initialization");
}

void DispAdfSSD1306::ShowInfo(
	uint16_t x,
	uint16_t y,
	const char* str)
{
	display.clearDisplay();

	u8g2_for_adafruit_gfx.setForegroundColor(WHITE);
	u8g2_for_adafruit_gfx.setCursor(x, y);
	u8g2_for_adafruit_gfx.print(str);

	// display a pixel in each corner of the screen
	display.drawPixel(0, 0, WHITE);
	display.drawPixel(127, 0, WHITE);
	display.drawPixel(0, 63, WHITE);
	display.drawPixel(127, 63, WHITE);

	// update display with all of the above graphics
	display.display();
}

void DispAdfSSD1306::ShowInfo(
	uint16_t x1,
	uint16_t y1,
	const char* str1,
	uint16_t x2,
	uint16_t y2,
	const char* str2)
{
	display.clearDisplay();

	u8g2_for_adafruit_gfx.setForegroundColor(WHITE);
	u8g2_for_adafruit_gfx.setCursor(x1, y1);
	u8g2_for_adafruit_gfx.print(str1);

	u8g2_for_adafruit_gfx.setCursor(x2, y2);
	u8g2_for_adafruit_gfx.print(str2);

	// display a pixel in each corner of the screen
	display.drawPixel(0, 0, WHITE);
	display.drawPixel(127, 0, WHITE);
	display.drawPixel(0, 63, WHITE);
	display.drawPixel(127, 63, WHITE);

	// update display with all of the above graphics
	display.display();
}

void DispAdfSSD1306::Show(
	uint32_t temperature,
	const char* time,
	bool appSent,
	uint8_t sttFlags,
	uint16_t pauseHeatingLeft, // in min, 0 - is Off
	uint8_t heatDestTemp,
	uint8_t pauseSchedule) // How many schedule points will be paused
{
#define roundR 5
#define boxH 11
#define sensors 4

	auto inTimeRange = IS_FLG_IN_TIME_RANGE_IN(sttFlags);
	auto heatingOnProcess = IS_HEATING(sttFlags);
	auto automaticMode = IS_FLG_AUTOMATIC_MODE_ON(sttFlags);
	auto manualHeating = IS_FLG_MANUAL_HEATING_ON(sttFlags);
	auto schedulerPaused = IS_FLG_SCHEDULER_PAUSED_YES(sttFlags);

	//static char tmpBuff[128];
	auto tp = (uint8_t*)& temperature;

	display.clearDisplay();
	u8g2_for_adafruit_gfx.setForegroundColor(WHITE);


	//==DrawAntenna(116, 10);
	//==DrawAndroid(115, 27);
	//==DrawTerminal(116, 27);

	//if (appSent)
	//{
	//	DrawAndroid(112, Row0);
	//}
	//else
	//{
	//	DrawAntenna(116, Row0);
	//}

	// Draw temperatures
	for (uint8_t i = 0; i < sensors; ++i)
	{
		u8g2_for_adafruit_gfx.setCursor(i * 22, Row0);
		u8g2_for_adafruit_gfx.print(tp[i]);
	}

	// Draw Mode status: A:ON, A:ON O (disk is auto mode in range), A:OFF
	u8g2_for_adafruit_gfx.setCursor(0, Row1);
	u8g2_for_adafruit_gfx.print(S_A_COLON);
	if (automaticMode)
	{
		display.fillCircle(24, Row1 - roundR, roundR, WHITE);
		u8g2_for_adafruit_gfx.setCursor(40, Row1);
		u8g2_for_adafruit_gfx.print(S_T_COLON);

		if (inTimeRange)
		{
			if (schedulerPaused)
			{
				display.writeFillRect(64, Row1 - boxH, 4, boxH, WHITE);
				display.writeFillRect(71, Row1 - boxH, 4, boxH, WHITE);
				u8g2_for_adafruit_gfx.setCursor(84, Row1);
				u8g2_for_adafruit_gfx.print(pauseSchedule);
			}
			else
			{
				display.fillCircle(64, Row1 - roundR, roundR, WHITE);
			}
		}
		else
		{
			display.drawCircle(64, Row1 - roundR, roundR, WHITE);
		}
	}
	else
	{
		display.drawCircle(24, Row1 - roundR, roundR, WHITE);
	}

	// Draw Heating status: H:Auto, H:Manual, [Destination temperature if manual/auto, or time left (min) if paused]
	u8g2_for_adafruit_gfx.setCursor(0, Row2);
	u8g2_for_adafruit_gfx.print(S_H_COLON);
	if (heatingOnProcess)
	{
		display.fillCircle(24, Row2 - roundR, roundR, WHITE);
		u8g2_for_adafruit_gfx.setCursor(40, Row2);
		u8g2_for_adafruit_gfx.print(manualHeating ? S_M_COLON : S_A_COLON);
		u8g2_for_adafruit_gfx.setCursor(58, Row2);
		u8g2_for_adafruit_gfx.print(heatDestTemp);
	}
	else if (pauseHeatingLeft > 0)
	{
		display.writeFillRect(20, Row2 - boxH, 4, boxH, WHITE);
		display.writeFillRect(27, Row2 - boxH, 4, boxH, WHITE);
		u8g2_for_adafruit_gfx.setCursor(40, Row2);
		u8g2_for_adafruit_gfx.print(pauseHeatingLeft);
	}
	else
	{
		display.drawCircle(24, Row2 - roundR, roundR, WHITE);
	}

	// Show current time
	u8g2_for_adafruit_gfx.setCursor(0, Row3);
	u8g2_for_adafruit_gfx.print((nullptr == time) ? "" : time);

	display.display();

	//#endif // OLED
}

//void DispAdfSSD1306::DrawAndroid(uint16_t x, uint16_t y)
//{
//#define andrBodyW 9
//#define andrBodyH 9
//#define andrBodyY y
//#define andrBodyX x + 2
//#define andrLeftHandX x
//#define andrHandY y + 1
//#define andrRightHandX andrBodyX + andrBodyW + 1
//#define andrHandH 6
//	// Head
//#define andrHeadRX 4
//#define andrHeadRY 3
//#define andrHeadX andrBodyX + andrHeadRX
//#define andrHeadY andrBodyY - 2
//
//#define andrLeftLegX andrBodyX + 1
//#define andrLegY andrBodyY + andrBodyH
//#define andrRightLegX andrBodyX + andrBodyW - 2
//#define andrLegH 3
//
//// Draw Head
//	DrawFilledEllipse(
//		andrHeadX,
//		andrHeadY,
//		andrHeadRX,
//		andrHeadRY,
//		U8G_DRAW_UPPER_RIGHT | U8G_DRAW_UPPER_LEFT);
//	// Draw Body
//	display.writeFillRect(andrBodyX, andrBodyY, andrBodyW, andrBodyH, WHITE);
//	//u8g.drawCircle(andrBodyX + 3, andrBodyY + 3, 3);
//	//u8g.drawCircle(andrBodyX + 3, andrBodyY - 3, 3);
//	// Draw Hands
//	display.drawFastVLine(andrLeftHandX, andrHandY, andrHandH, WHITE);
//	display.drawFastVLine(andrRightHandX, andrHandY, andrHandH, WHITE);
//	// Draw Legs
//	display.drawFastVLine(andrLeftLegX, andrLegY, andrLegH, WHITE);
//	display.drawFastVLine(andrRightLegX, andrLegY, andrLegH, WHITE);
//	// Draw Horns
//	display.drawPixel(andrBodyX, andrHeadY - 3, WHITE);
//	display.drawPixel(andrBodyX - 1, andrHeadY - 4, WHITE);
//	display.drawPixel(andrBodyX + andrBodyW - 1, andrHeadY - 3, WHITE);
//	display.drawPixel(andrBodyX + andrBodyW, andrHeadY - 4, WHITE);
//}

//void DispAdfSSD1306::DrawTerminal(uint16_t x, uint16_t y)
//{
//#define tmlW 8
//#define tmlH 8
//	display.drawLine(x, y, x + tmlW, y + tmlH / 2, WHITE);
//	display.drawLine(x, y - 1, x + tmlW + 1, y + tmlH / 2, WHITE);
//
//	display.drawLine(x, y + tmlH, x + tmlW, y + tmlH / 2, WHITE);
//	display.drawLine(x, y + tmlH + 1, x + tmlW + 1, y + tmlH / 2, WHITE);
//}
//
//void DispAdfSSD1306::DrawAntenna(uint16_t x, uint16_t y)
//{
//#define antW 9
//#define antCenter 5
//#define antH 10
//#define antHSplit 5
//
//	display.drawLine(x - antCenter, y - antH, x, y - antHSplit, WHITE);
//	display.drawLine(x, y, x, y - antH, WHITE);
//	display.drawLine(x + antCenter, y - antH, x, y - antHSplit, WHITE);
//}


//ELLIPSE

void DispAdfSSD1306::draw_ellipse_section(uint8_t x, uint8_t y, uint8_t x0, uint8_t y0, uint8_t option)
{
	/* upper right */
	if (option & U8G_DRAW_UPPER_RIGHT)
	{
		display.drawPixel(x0 + x, y0 - y, WHITE);
	}

	/* upper left */
	if (option & U8G_DRAW_UPPER_LEFT)
	{
		display.drawPixel(x0 - x, y0 - y, WHITE);
	}

	/* lower right */
	if (option & U8G_DRAW_LOWER_RIGHT)
	{
		display.drawPixel(x0 + x, y0 + y, WHITE);
	}

	/* lower left */
	if (option & U8G_DRAW_LOWER_LEFT)
	{
		display.drawPixel(x0 - x, y0 + y, WHITE);
	}
}

void DispAdfSSD1306::draw_ellipse(uint8_t x0, uint8_t y0, uint8_t rx, uint8_t ry, uint8_t option)
{
	uint8_t x, y;
	int16_t xchg, ychg;
	int16_t err;
	int16_t rxrx2;
	int16_t ryry2;
	int16_t stopx, stopy;

	rxrx2 = rx;
	rxrx2 *= rx;
	rxrx2 *= 2;

	ryry2 = ry;
	ryry2 *= ry;
	ryry2 *= 2;

	x = rx;
	y = 0;

	xchg = 1;
	xchg -= rx;
	xchg -= rx;
	xchg *= ry;
	xchg *= ry;

	ychg = rx;
	ychg *= rx;

	err = 0;

	stopx = ryry2;
	stopx *= rx;
	stopy = 0;

	while (stopx >= stopy)
	{
		draw_ellipse_section(x, y, x0, y0, option);
		y++;
		stopy += rxrx2;
		err += ychg;
		ychg += rxrx2;
		if (2 * err + xchg > 0)
		{
			x--;
			stopx -= ryry2;
			err += xchg;
			xchg += ryry2;
		}
	}

	x = 0;
	y = ry;

	xchg = ry;
	xchg *= ry;

	ychg = 1;
	ychg -= ry;
	ychg -= ry;
	ychg *= rx;
	ychg *= rx;

	err = 0;

	stopx = 0;

	stopy = rxrx2;
	stopy *= ry;


	while (stopx <= stopy)
	{
		draw_ellipse_section(x, y, x0, y0, option);
		x++;
		stopx += ryry2;
		err += xchg;
		xchg += ryry2;
		if (2 * err + ychg > 0)
		{
			y--;
			stopy -= rxrx2;
			err += ychg;
			ychg += rxrx2;
		}
	}

}

void DispAdfSSD1306::DrawEllipse(uint8_t x0, uint8_t y0, uint8_t rx, uint8_t ry, uint8_t option)
{
	/* check for bounding box */
	{
		uint8_t rxp, rxp2;
		uint8_t ryp, ryp2;

		rxp = rx;
		rxp++;
		rxp2 = rxp;
		rxp2 *= 2;

		ryp = ry;
		ryp++;
		ryp2 = ryp;
		ryp2 *= 2;
	}

	draw_ellipse(x0, y0, rx, ry, option);
}

void DispAdfSSD1306::draw_filled_ellipse_section(uint8_t x, uint8_t y, uint8_t x0, uint8_t y0, uint8_t option)
{
	/* upper right */
	if (option & U8G_DRAW_UPPER_RIGHT)
	{
		display.drawFastVLine(x0 + x, y0 - y, y + 1, WHITE);
	}

	/* upper left */
	if (option & U8G_DRAW_UPPER_LEFT)
	{
		display.drawFastVLine(x0 - x, y0 - y, y + 1, WHITE);
	}

	/* lower right */
	if (option & U8G_DRAW_LOWER_RIGHT)
	{
		display.drawFastVLine(x0 + x, y0, y + 1, WHITE);
	}

	/* lower left */
	if (option & U8G_DRAW_LOWER_LEFT)
	{
		display.drawFastVLine(x0 - x, y0, y + 1, WHITE);
	}
}

void DispAdfSSD1306::draw_filled_ellipse(uint8_t x0, uint8_t y0, uint8_t rx, uint8_t ry, uint8_t option)
{
	uint8_t x, y;
	int16_t xchg, ychg;
	int16_t err;
	int16_t rxrx2;
	int16_t ryry2;
	int16_t stopx, stopy;

	rxrx2 = rx;
	rxrx2 *= rx;
	rxrx2 *= 2;

	ryry2 = ry;
	ryry2 *= ry;
	ryry2 *= 2;

	x = rx;
	y = 0;

	xchg = 1;
	xchg -= rx;
	xchg -= rx;
	xchg *= ry;
	xchg *= ry;

	ychg = rx;
	ychg *= rx;

	err = 0;

	stopx = ryry2;
	stopx *= rx;
	stopy = 0;

	while (stopx >= stopy)
	{
		draw_filled_ellipse_section(x, y, x0, y0, option);
		y++;
		stopy += rxrx2;
		err += ychg;
		ychg += rxrx2;
		if (2 * err + xchg > 0)
		{
			x--;
			stopx -= ryry2;
			err += xchg;
			xchg += ryry2;
		}
	}

	x = 0;
	y = ry;

	xchg = ry;
	xchg *= ry;

	ychg = 1;
	ychg -= ry;
	ychg -= ry;
	ychg *= rx;
	ychg *= rx;

	err = 0;

	stopx = 0;

	stopy = rxrx2;
	stopy *= ry;


	while (stopx <= stopy)
	{
		draw_filled_ellipse_section(x, y, x0, y0, option);
		x++;
		stopx += ryry2;
		err += xchg;
		xchg += ryry2;
		if (2 * err + ychg > 0)
		{
			y--;
			stopy -= rxrx2;
			err += ychg;
			ychg += rxrx2;
		}
	}

}

void DispAdfSSD1306::DrawFilledEllipse(uint8_t x0, uint8_t y0, uint8_t rx, uint8_t ry, uint8_t option)
{
	/* check for bounding box */
	{
		uint8_t rxp, rxp2;
		uint8_t ryp, ryp2;

		rxp = rx;
		rxp++;
		rxp2 = rxp;
		rxp2 *= 2;

		ryp = ry;
		ryp++;
		ryp2 = ryp;
		ryp2 *= 2;
	}

	draw_filled_ellipse(x0, y0, rx, ry, option);
}
#endif // Adfr_SSD1306
