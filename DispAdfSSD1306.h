#ifndef DISP_ADF_SSD1306_H
#define DISP_ADF_SSD1306_H
#include "DataTypes.h"

#ifdef Adfr_SSD1306
#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#endif

class DispAdfSSD1306
{
public:
	void Init();
	void ShowInfo(
		uint16_t x,
		uint16_t y,
		const char* str);

	void ShowInfo(
		uint16_t x1,
		uint16_t y1,
		const char* str1,
		uint16_t x2,
		uint16_t y2,
		const char* str2);

	void ShowRssi(int8_t rssi);

	void Show(
		uint32_t temperature,
		const char* time,
		bool appSent,
		uint8_t sttFlags,
		uint16_t pauseHeatingLeft, // in min, 0 - is Off
		uint8_t heatDestTemp,
		uint8_t pauseSchedule);

private:
	//void DrawTerminal(uint16_t x, uint16_t y);
	//void DrawAntenna(uint16_t x, uint16_t y);
	//void DrawAndroid(uint16_t x, uint16_t y);

	void DrawEllipse(uint8_t x0, uint8_t y0, uint8_t rx, uint8_t ry, uint8_t option);
	void DrawFilledEllipse(uint8_t x0, uint8_t y0, uint8_t rx, uint8_t ry, uint8_t option);

	void draw_ellipse_section(uint8_t x, uint8_t y, uint8_t x0, uint8_t y0, uint8_t option);
	void draw_ellipse(uint8_t x0, uint8_t y0, uint8_t rx, uint8_t ry, uint8_t option);
	void draw_filled_ellipse_section(uint8_t x, uint8_t y, uint8_t x0, uint8_t y0, uint8_t option);
	void draw_filled_ellipse(uint8_t x0, uint8_t y0, uint8_t rx, uint8_t ry, uint8_t option);
};
#endif // Adfr_SSD1306
#endif