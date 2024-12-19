#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/TS_DISCO_F429ZI.h"
const char *info;
const char *preinfo;
uint32_t currentScreenColor = LCD_COLOR_WHITE;

// Function to display text on the LCD
void Displaytext(char *text, LCD_DISCO_F429ZI lcd, uint16_t text_x, uint16_t text_y, uint16_t font_size)
{
    for (int row = 0; row * 16 < strlen(text); row++)
    {
        char display_buffer[32];
        strncpy(display_buffer, text + row * 16, 16);
        display_buffer[16] = '\0';
        lcd.SetTextColor(LCD_COLOR_BLACK);                                    // Set the color to the background color
        lcd.FillRect(0, text_y + row * font_size, lcd.GetXSize(), font_size); // Clear a specific line
        lcd.SetTextColor(LCD_COLOR_BLUE);                                     // Reset the text color
        lcd.DisplayStringAt(text_x, text_y + row * font_size, (uint8_t *)display_buffer, CENTER_MODE);
    }
}
void UpdateInfo(LCD_DISCO_F429ZI lcd, char *info)
{
    info = info;
    if (preinfo != info)
    {
        Displaytext(const_cast<char *>(info), lcd, 0, 150, 18);
        preinfo = info;
    }
}
void ClearScreen(LCD_DISCO_F429ZI lcd, uint32_t color)
{
    if (currentScreenColor != color)
    {
        currentScreenColor = color;
        lcd.Clear(color);
    }
}
