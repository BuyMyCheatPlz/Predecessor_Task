#include "../../Utils/Inc/justfloat.h"
#include <math.h>

void jf_ftoa(float value, char *buf, int precision)
{
    char *p = buf;
    if (isnan(value)) {
        *p++ = 'N'; *p++ = 'a'; *p++ = 'N'; *p = '\0';
        return;
    }
    if (isinf(value)) {
        if (value < 0) { *p++ = '-'; }
        *p++ = 'I'; *p++ = 'n'; *p++ = 'f'; *p = '\0';
        return;
    }
    if (value < 0.0f) {
        *p++ = '-';
        value = -value;
    }

    /* Handle rounding that may carry into integer part */
    float rounding = 0.5f;
    for (int i = 0; i < precision; ++i) rounding /= 10.0f;
    value += rounding;

    long long int_part = (long long)value;
    float frac = value - (float)int_part;

    /* write integer part */
    char intbuf[32];
    char *ip = intbuf + sizeof(intbuf) - 1;
    *ip = '\0';
    if (int_part == 0) {
        *--ip = '0';
    } else {
        while (int_part > 0 && ip > intbuf) {
            int digit = int_part % 10;
            *--ip = '0' + digit;
            int_part /= 10;
        }
    }
    /* copy integer */
    while (*ip) *p++ = *ip++;

    if (precision > 0) {
        *p++ = '.';
        /* scale fractional */
        long long mult = 1;
        for (int i = 0; i < precision; ++i) mult *= 10;
        long long frac_part = (long long)(frac * (float)mult);

        /* write fractional with leading zeros */
        char fbuf[32];
        fbuf[precision] = '\0';
        for (int i = precision - 1; i >= 0; --i) {
            fbuf[i] = '0' + (frac_part % 10);
            frac_part /= 10;
        }
        char *fp = fbuf;
        while (*fp) *p++ = *fp++;
    }
    *p = '\0';
}
