
#include <stdarg.h>
#include <stdio.h>
#include <lcd.h>

/*!
    \brief      output a message on the LCD screen
    \param      fmt: format string just like printf
*/
int dbgprint(const char *fmt, ...)
{
    char buffer[64];
    va_list args;
    static int x = 0;
    static int y = 0;

    va_start(args, fmt);
    int ret = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    for (int i = 0; i < ret; i++) {
        if (y > 75) {
            LCD_Clear();
            x = y = 0;
        }

        if (buffer[i] == '\n') {
            x = 0;
            y += 16;
        }
        else {
            LCD_ShowChar(x, y, buffer[i], 0, WHITE);
            x += 8;
        }
    }

    return ret;
}

