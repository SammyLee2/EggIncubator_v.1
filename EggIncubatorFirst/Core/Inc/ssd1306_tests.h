#ifndef __SSD1306_TEST_H__
#define __SSD1306_TEST_H__

#include <_ansi.h>

_BEGIN_STD_C


typedef struct time_egg_incubator {
	uint32_t elapsed_days;
	uint32_t elapsed_hours;
	uint32_t remaining_days;
	uint32_t remaining_hours;
	uint32_t progress;
} time_info_egg_incubator;



void ssd1306_TestBorder(void);
void ssd1306_TestFonts1(void);
void ssd1306_TestFonts2(void);
void ssd1306_TestFPS(void);
void ssd1306_TestAll(void);
void ssd1306_TestLine(void);
void ssd1306_TestRectangle(void);
void ssd1306_TestRectangleFill(void);
void ssd1306_TestRectangleInvert(void);
void ssd1306_TestCircle(void);
void ssd1306_TestArc(void);
void ssd1306_TestPolyline(void);
void ssd1306_TestDrawBitmap(void);

void ssd1306_egg_incubator_booting(void);
void ssd1306_draw_egg_shaking(void);
void ssd1306_draw_egg_breaking(void);
void ssd1306_draw_egg_hatching(void);

void ssd1306_show_egg_incubator_state_first_page(char* status, float temperature, int heater_state);
void ssd1306_show_egg_incubator_state_second_page(void);

void ssd1306_show_egg_incubator_state_test_1(void);
void ssd1306_show_egg_incubator_state_test_2(void);
void calculate_time(uint32_t *startTime);
void display_ssd1306(uint32_t *startTime, char *status, float temperature, int heater_state);

void ssd1306_Preparig_transition(void);
void ssd1306_Incubating_transition(void);
void ssd1306_Completing_transition(void);



_END_STD_C

#endif // __SSD1306_TEST_H__
